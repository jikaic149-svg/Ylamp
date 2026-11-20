#include "motion_mode.h"

#include <assert.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

FDCAN_HandleTypeDef hfdcan1, hfdcan2;

static void step(DM_Motor_Feedback_t fb[2], MotionMode_State_t *state,
                 MotionMode_Status_t *status, DM_Motor_MIT_Command_t cmd[2])
{
    for (unsigned i = 0; i < 2; ++i) {
        fb[i].received_at_ms += 5U;
        ++fb[i].received_count;
    }
    assert(MotionMode_Update(&g_motor_config, fb, .005f, state, status, cmd));
    assert(status->valid == (state->phase != MOTION_MODE_OFF));
    if (!status->valid) { return; }
    float velocity[2] = {fb[0].velocity_rad_s, fb[1].velocity_rad_s};
    RobotModel_Dynamics_t model;
    assert(RobotModel_Dynamics(&g_robot_model,
        fb[1].position_rad - g_motor_config.drag_j2_zero_rad,
        velocity, status->target_acceleration_rad_s2, &model));
    for (unsigned i = 0; i < 2; ++i) {
        float friction;
        assert(RobotModel_Friction(&g_robot_friction[i], status->friction_velocity_rad_s[i],
            g_motion_config.friction_smooth_rad_s, &friction));
        assert(fabsf(status->friction_nm[i] - friction) < 1e-7f);
        assert(fabsf(cmd[i].torque_ff_nm - state->entry_blend *
            (model.inertial_nm[i] + model.coupling_nm[i] + model.gravity_nm[i] +
             friction)) < 1e-6f);
        assert(cmd[i].kp == state->entry_blend * g_motion_config.kp[i]);
        assert(cmd[i].kd == state->entry_blend * g_motion_config.kd[i]);
        assert(cmd[i].position_rad == (float)state->reference.position[i]);
        assert(cmd[i].velocity_rad_s == (float)state->reference.velocity[i]);
    }
}

static void check_encoder_friction(void)
{
    DM_Motor_Feedback_t fb[2] = {
        {.position_rad = -1.0f, .received_at_ms = UINT32_MAX - 200U,
         .received_count = UINT32_MAX - 100U},
        {.position_rad = g_motor_config.drag_j2_zero_rad + .04f, .received_at_ms = UINT32_MAX - 200U,
         .received_count = UINT32_MAX - 100U}
    };
    MotionMode_State_t state = {0};
    MotionMode_Status_t status;
    DM_Motor_MIT_Command_t cmd[2];
    assert(MotionMode_Enter(&g_motor_config, fb, &state));
    for (unsigned n = 0; n < 220; ++n) { step(fb, &state, &status, cmd); }
    assert(state.phase == MOTION_MODE_HOLDING);
    assert(state.filtered_velocity_rad_s[0] == 0.0f);
    const JointTrajectory_Sample_t center = state.reference;
    const float tau = g_motion_config.friction_velocity_filter_ms * .001f;
    const float quantum = 2.0f * g_motor_config.joint[0].device.pmax / 65535.0f;

    /* Differentiation precedes filtering. Reported motor speed is deliberately
     * opposite; changing it alone must never become the friction input. */
    fb[0].velocity_rad_s = -.7f;
    fb[1].velocity_rad_s = .7f;
    for (unsigned n = 0; n < 200; ++n) {
        fb[0].position_rad += .001f;
        fb[1].position_rad -= .001f;
        step(fb, &state, &status, cmd);
        if (n == 0) {
            assert(fabsf(state.filtered_velocity_rad_s[0] - .001f/(tau+.005f)) < 1e-6f);
        }
    }
    assert(fabsf(state.filtered_velocity_rad_s[0] - .2f) < 2e-5f);
    assert(fabsf(status.friction_velocity_rad_s[0] - (.2f-quantum/tau)) < 2e-5f);
    assert(status.friction_nm[0] > .6f && status.friction_nm[1] < -.48f);
    assert(memcmp(&state.reference, &center, sizeof(center)) == 0);

    /* A cached frame must not be differentiated again, including at dt=0. */
    const float filtered = state.filtered_velocity_rad_s[0];
    for (unsigned n = 0; n < 20; ++n) {
        assert(MotionMode_Update(&g_motor_config, fb, .005f, &state, &status, cmd));
        assert(state.filtered_velocity_rad_s[0] == filtered);
    }
    fb[0].position_rad += .001f;
    ++fb[0].received_count; /* New count, but timestamp still in the same ms. */
    assert(MotionMode_Update(&g_motor_config, fb, 0, &state, &status, cmd));
    assert(state.filtered_velocity_rad_s[0] == filtered);
    const float old_position = state.encoder_position_rad[0];
    fb[0].received_at_ms += 15U;
    ++fb[0].received_count;
    const float derivative = (fb[0].position_rad-old_position)/.015f;
    const float expected = filtered + .015f/(tau+.015f)*(derivative-filtered);
    assert(MotionMode_Update(&g_motor_config, fb, .005f, &state, &status, cmd));
    assert(fabsf(state.filtered_velocity_rad_s[0]-expected) < 1e-7f);

    /* Reversal is filtered instead of switching a whole Coulomb term at once. */
    const float before_reverse = status.friction_velocity_rad_s[0];
    fb[0].position_rad -= .001f;
    step(fb, &state, &status, cmd);
    assert(status.friction_velocity_rad_s[0] < before_reverse);
    assert(status.friction_velocity_rad_s[0] > 0.0f);
    for (unsigned n = 0; n < 100; ++n) {
        fb[0].position_rad -= .001f;
        step(fb, &state, &status, cmd);
    }
    assert(status.friction_nm[0] < -.6f);
    for (unsigned n = 0; n < 220; ++n) { step(fb, &state, &status, cmd); }
    assert(status.friction_nm[0] == 0 && status.friction_nm[1] == 0);

    /* One encoder count of stationary jitter must not create Coulomb chatter.
     * This includes both one-count blips and long alternating plateaus. */
    const float parked = fb[0].position_rad;
    for (unsigned n = 0; n < 800; ++n) {
        fb[0].position_rad = parked + ((n/25U) % 2U ? quantum : 0);
        step(fb, &state, &status, cmd);
        assert(fabsf(status.friction_nm[0]) < 2e-5f);
    }
    for (unsigned n = 0; n < 200; ++n) {
        fb[0].position_rad = parked + (n % 2U ? quantum : 0);
        step(fb, &state, &status, cmd);
        assert(fabsf(status.friction_nm[0]) < 2e-5f);
    }
    assert(memcmp(&state.reference, &center, sizeof(center)) == 0);

    /* An acquisition gap/backwards timestamp reseeds only the differentiator,
     * without moving the fixed target or adding another fault threshold. */
    for (unsigned gap = 0; gap < 2; ++gap) {
        fb[0].position_rad += .1f;
        ++fb[0].received_count;
        fb[0].received_at_ms += gap ? UINT32_MAX : g_motor_config.feedback_max_age_ms+1U;
        assert(MotionMode_Update(&g_motor_config, fb, .005f, &state, &status, cmd));
        assert(state.filtered_velocity_rad_s[0] == 0 && status.friction_nm[0] == 0);
        assert(memcmp(&state.reference, &center, sizeof(center)) == 0);
    }
    MotionMode_Reset(&state);
    assert(state.filtered_velocity_rad_s[0] == 0 && state.encoder_sample_count[0] == 0);
    assert(MotionMode_Enter(&g_motor_config, fb, &state));
    step(fb, &state, &status, cmd);
    assert(status.friction_nm[0] == 0 && status.friction_nm[1] == 0);
}

/* Host-only replay of production Motion code. Each line contains the two
 * joints' receive count, timestamp, continuous encoder position, motor speed.
 * Input cadence is not claimed to be the original firmware control cadence. */
static int replay(void)
{
    DM_Motor_Feedback_t fb[2];
    MotionMode_State_t state = {0};
    MotionMode_Status_t status;
    DM_Motor_MIT_Command_t cmd[2];
    JointTrajectory_Sample_t center = {0};
    char line[512];
    unsigned samples = 0;
    while (fgets(line, sizeof(line), stdin) != NULL) {
        assert(sscanf(line, "%"SCNu32",%"SCNu32",%f,%f,%"SCNu32",%"SCNu32",%f,%f",
            &fb[0].received_count, &fb[0].received_at_ms, &fb[0].position_rad,
            &fb[0].velocity_rad_s, &fb[1].received_count, &fb[1].received_at_ms,
            &fb[1].position_rad, &fb[1].velocity_rad_s) == 8);
        if (samples++ == 0) {
            assert(MotionMode_Enter(&g_motor_config, fb, &state));
            center = state.reference;
        }
        assert(MotionMode_Update(&g_motor_config, fb, .02f, &state, &status, cmd));
        assert(memcmp(&center, &state.reference, sizeof(center)) == 0);
        printf("%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
            state.filtered_velocity_rad_s[0], state.filtered_velocity_rad_s[1],
            status.friction_velocity_rad_s[0], status.friction_velocity_rad_s[1],
            status.friction_nm[0], status.friction_nm[1],
            cmd[0].position_rad, cmd[1].position_rad);
    }
    assert(samples > 0);
    return 0;
}

static void check_stop_margin(void)
{
    /* Stops at many acceleration/deceleration phases near both travel ends.
     * Re-enter/recalculate each time: the adapter deliberately has one owner. */
    for (unsigned side = 0; side < 2; ++side) {
        for (unsigned cut = 0; cut <= 40; ++cut) {
            DM_Motor_Feedback_t fb[2] = {
                {.position_rad = -1.8f}, {.position_rad = g_motor_config.drag_j2_zero_rad + .04f}
            };
            MotionMode_State_t state = {0};
            MotionMode_Status_t status;
            DM_Motor_MIT_Command_t commands[2];
            assert(MotionMode_Enter(&g_motor_config, fb, &state));
            for (unsigned n = 0; n < 220; ++n) { step(fb, &state, &status, commands); }
            MotionMode_Request_t request = {.velocity_rad_s = {.2f, .35f},
                .acceleration_rad_s2 = {.4f, .7f}, .jerk_rad_s3 = {1.2f, 1.8f}};
            for (unsigned i = 0; i < 2; ++i) {
                const MotorTask_JointConfig_t *joint = &g_motor_config.joint[i];
                const float margin = 4 * joint->device.pmax / 65535.0f;
                request.target_rad[i] = side ? joint->upper_rad - margin :
                                               joint->lower_rad + margin;
            }
            assert(MotionMode_Move(&g_motor_config, &request, &state));
            unsigned steps = (unsigned)(state.duration_s * 200 * cut / 40);
            for (unsigned n = 0; n < steps; ++n) { step(fb, &state, &status, commands); }
            JointTrajectory_Sample_t last = state.reference;
            assert(MotionMode_Stop(&g_motor_config, &state));
            for (unsigned n = 0; n < 2000 && state.phase != MOTION_MODE_HOLDING; ++n) {
                step(fb, &state, &status, commands);
                for (unsigned i = 0; i < 2; ++i) {
                    assert(fabs(state.reference.acceleration[i] - last.acceleration[i]) <=
                           request.jerk_rad_s3[i] * .005 + 1e-6);
                    assert(fabs(state.reference.velocity[i] - last.velocity[i]) <=
                           request.acceleration_rad_s2[i] * .005 + 1e-6);
                }
                last = state.reference;
            }
            assert(state.phase == MOTION_MODE_HOLDING);
        }
    }
}

int main(int argc, char **argv)
{
    if (argc == 2 && strcmp(argv[1], "--replay") == 0) { return replay(); }
    assert(argc == 1);
    DM_Motor_Feedback_t fb[2] = {
        {.position_rad = -1.0f}, {.position_rad = g_motor_config.drag_j2_zero_rad + .04f}
    };
    MotionMode_State_t state = {0};
    MotionMode_Status_t status;
    DM_Motor_MIT_Command_t cmd[2];
    assert(MotionMode_ValidConfig(&g_motor_config));
    assert(!MotionMode_ValidConfig(NULL));
    step(fb, &state, &status, cmd);
    assert(cmd[0].kp == 0 && cmd[1].torque_ff_nm == 0);
    assert(!MotionMode_Stop(&g_motor_config, &state));
    assert(MotionMode_Enter(&g_motor_config, fb, &state));
    assert(!MotionMode_Enter(&g_motor_config, fb, &state));
    for (unsigned n = 0; n < 220; ++n) { step(fb, &state, &status, cmd); }
    assert(state.phase == MOTION_MODE_HOLDING);
    assert(cmd[0].position_rad == fb[0].position_rad);
    assert(cmd[1].position_rad == fb[1].position_rad);
    MotionMode_Request_t request = {
        .target_rad = {-1.2f, g_motor_config.drag_j2_zero_rad + .34f},
        .velocity_rad_s = {.2f, .3f},
        .acceleration_rad_s2 = {.4f, .6f},
        .jerk_rad_s3 = {1.0f, 1.5f}
    };
    MotionMode_Request_t bad = request;
    bad.target_rad[1] = 0.0f;
    assert(!MotionMode_Move(&g_motor_config, &bad, &state));
    bad = request; bad.jerk_rad_s3[0] = NAN;
    assert(!MotionMode_Move(&g_motor_config, &bad, &state));
    bad = request; bad.acceleration_rad_s2[0] = 2.01f;
    assert(!MotionMode_Move(&g_motor_config, &bad, &state));
    bad = request; bad.velocity_rad_s[0] = 17.0f;
    assert(!MotionMode_Move(&g_motor_config, &bad, &state));
    assert(state.phase == MOTION_MODE_HOLDING && cmd[0].position_rad == -1.0f);
    assert(MotionMode_Move(&g_motor_config, &request, &state));
    assert(!MotionMode_Move(&g_motor_config, &request, &state));
    double last_v[2] = {0}, last_a[2] = {0};
    unsigned count = 0;
    while (state.phase != MOTION_MODE_HOLDING && ++count < 3000) {
        step(fb, &state, &status, cmd);
        for (unsigned i = 0; i < 2; ++i) {
            const double v = state.reference.velocity[i], a = state.reference.acceleration[i];
            assert(fabs(v) <= request.velocity_rad_s[i] + 1e-7);
            assert(fabs(a) <= request.acceleration_rad_s2[i] + 1e-7);
            assert(fabs(a - last_a[i]) <= request.jerk_rad_s3[i] * .005 + 1e-7);
            assert(fabs(v - last_v[i]) <= request.acceleration_rad_s2[i] * .005 + 1e-7);
            last_v[i] = v; last_a[i] = a;
            /* Perfect-follow fixture; real settling is not claimed. */
            fb[i].position_rad = cmd[i].position_rad;
            fb[i].velocity_rad_s = cmd[i].velocity_rad_s;
        }
    }
    assert(count < 3000 && state.phase == MOTION_MODE_HOLDING);
    assert(fabsf(cmd[0].position_rad - request.target_rad[0]) < 1e-6f);
    assert(fabsf(cmd[1].position_rad - request.target_rad[1]) < 1e-6f);
    assert(cmd[0].velocity_rad_s == 0 && cmd[1].velocity_rad_s == 0);
    for (unsigned n = 0; n < 220; ++n) { step(fb, &state, &status, cmd); }
    assert(state.phase == MOTION_MODE_HOLDING);
    /* Motor-reported speed alone cannot create encoder friction assist. */
    fb[0].velocity_rad_s = .2f;
    step(fb, &state, &status, cmd);
    assert(status.friction_nm[0] == 0.0f && status.friction_nm[1] == 0.0f);
    fb[0].velocity_rad_s = 0.0f;

    /* Brake mid-move, including positive acceleration, without jumping p/v/a. */
    request.target_rad[0] = -1.6f;
    request.target_rad[1] = g_motor_config.drag_j2_zero_rad - .16f;
    assert(MotionMode_Move(&g_motor_config, &request, &state));
    for (unsigned n = 0; n < 65; ++n) { step(fb, &state, &status, cmd); }
    JointTrajectory_Sample_t before = state.reference;
    assert(MotionMode_Stop(&g_motor_config, &state));
    assert(state.phase == MOTION_MODE_STOPPING);
    assert(memcmp(&before, &state.reference, sizeof(before)) == 0);
    assert(MotionMode_Stop(&g_motor_config, &state)); /* Idempotent. */
    for (unsigned n = 0; n < 3000 && state.phase != MOTION_MODE_HOLDING; ++n) {
        step(fb, &state, &status, cmd);
    }
    assert(state.phase == MOTION_MODE_HOLDING);
    assert(cmd[0].velocity_rad_s == 0 && cmd[1].velocity_rad_s == 0);
    assert(fabsf(cmd[0].position_rad - request.target_rad[0]) > .05f);
    const MotionMode_State_t saved = state;
    const MotionMode_Status_t saved_status = status;
    DM_Motor_MIT_Command_t saved_cmd[2];
    memcpy(saved_cmd, cmd, sizeof(cmd));
    fb[0].position_rad = NAN;
    assert(!MotionMode_Update(&g_motor_config, fb, .005f, &state, &status, cmd));
    assert(memcmp(&state, &saved, sizeof(state)) == 0);
    assert(memcmp(&status, &saved_status, sizeof(status)) == 0);
    assert(memcmp(cmd, saved_cmd, sizeof(cmd)) == 0);
    MotionMode_Reset(&state);
    assert(state.phase == MOTION_MODE_OFF);
    check_encoder_friction();
    check_stop_margin();
    return 0;
}
