/* Included by test_bsp.c to reuse its real Device/HAL transport fixture. */
#include "joint_coordinates.h"
#include "drag_mode.h"

static uint16_t signed_code(float value, float maximum, uint16_t scale)
{
    return (uint16_t)((value / maximum + 1.0f) * 0.5f * (float)scale);
}

static MotorTask_Status_t task_status(void)
{
    MotorTask_Status_t status;
    assert(MotorTask_GetStatus(&status) == BSP_CAN_OK);
    return status;
}

static void task_feedback(unsigned i, float position, float velocity, uint8_t state)
{
    const DM_Motor_Config_t *cfg = &g_motor_config.joint[i].device;
    const uint16_t p = signed_code(position, cfg->pmax, UINT16_MAX);
    const uint16_t v = signed_code(velocity, cfg->vmax, 0xFFFU);
    const uint8_t data[8] = {(uint8_t)((state << 4) | cfg->can_id),
        (uint8_t)(p >> 8), (uint8_t)p, (uint8_t)(v >> 4),
        (uint8_t)((v << 4) | 7U), 0xFFU, 25U, 26U};
    inject_motor_feedback(cfg->bus, cfg->master_id, data, 8U);
}

static void feedback_pair(uint32_t now, uint8_t state)
{
    test_tick = now;
    for (unsigned i = 0U; i < 2U; ++i) { task_feedback(i, -2.6f, 0.0f, state); }
}

static void task_run(uint32_t now)
{
    test_tick = now;
    (void)MotorTask_Run(now);
}

static void task_steps(uint32_t *now, unsigned count, uint8_t state)
{
    for (unsigned i = 0U; i < count; ++i) {
        *now += 5U;
        feedback_pair(*now, state);
        task_run(*now);
    }
}

static void setup_task(bool approved, uint32_t now)
{
    MotorTask_Config_t cfg = g_motor_config;
    DM_Motor_t *motors[2];
    MotorTask_Status_t unused;
    assert(cfg.drag_reference_confirmed == 1U);
    if (!approved) { cfg.drag_reference_confirmed = 0U; }
    assert(MotorTask_GetStatus(&unused) == BSP_CAN_NOT_READY);
    assert(MotorTask_Run(now) == BSP_CAN_NOT_READY);
    assert(BSP_CAN_Init(&hfdcan2) == BSP_CAN_OK);
    assert(BSP_KEY_Init(cfg.key_debounce_ms) == HAL_OK);
    for (unsigned i = 0U; i < 2U; ++i) {
        assert(DM_Motor_Init(&cfg.joint[i].device, &motors[i]) == BSP_CAN_OK);
    }
    MotorTask_Config_t invalid = cfg;
    invalid.drag_gravity_scale = NAN;
    assert(MotorTask_Init(&invalid, motors, now) == BSP_CAN_INVALID_ARG);
    invalid = cfg;
    invalid.joint[0].hold_kp = 0.0f;
    assert(MotorTask_Init(&invalid, motors, now) == BSP_CAN_INVALID_ARG);
    invalid = cfg;
    invalid.joint[1].drag_kd_high = 6.0f;
    assert(MotorTask_Init(&invalid, motors, now) == BSP_CAN_INVALID_ARG);
    invalid = cfg;
    invalid.joint[1].lower_rad = -6.0f;
    invalid.joint[1].upper_rad = 1.0f; /* More than one turn: startup ambiguous. */
    assert(MotorTask_Init(&invalid, motors, now) == BSP_CAN_INVALID_ARG);
    invalid = cfg;
    invalid.joint[0].protection_flags = 1U; /* Retired SPEED bit must not be accepted. */
    assert(MotorTask_Init(&invalid, motors, now) == BSP_CAN_INVALID_ARG);
    invalid = cfg;
    invalid.joint[1].start_speed_rad_s = invalid.joint[1].device.vmax + 0.1f;
    assert(MotorTask_Init(&invalid, motors, now) == BSP_CAN_INVALID_ARG);
    assert(MotorTask_Init(&cfg, NULL, now) == BSP_CAN_INVALID_ARG);
    test_ipsr = 1U;
    assert(MotorTask_Init(&cfg, motors, now) == BSP_CAN_INVALID_ARG);
    test_ipsr = 0U;
    assert(MotorTask_Init(&cfg, motors, now) == BSP_CAN_OK);
    assert(MotorTask_SelectMode(MOTOR_MODE_MOTION) == BSP_CAN_OK);
    assert(MotorTask_SelectMode((MotorMode_Id_t)2) == BSP_CAN_INVALID_ARG);
    assert(MotorTask_SelectMode(MOTOR_MODE_DRAG) == BSP_CAN_OK);
    assert(task_status().mode == MOTOR_MODE_DRAG);
    assert(task_status().mode_phase == MOTOR_MODE_OFF);
    assert(MotorTask_Init(&cfg, motors, now) == BSP_CAN_BUSY);
    assert(task_status().policy_ready == approved);
    assert(can_tx_calls == 0U && gpio_high_writes == 0U);
    feedback_pair(now, 0U);
}

static void start_task(uint32_t *now, bool key)
{
    if (key) {
        test_gpioa.IDR = 0U;
        task_steps(now, 6U, 0U);
        test_gpioa.IDR = KEY__INPUT_Pin;
    } else {
        assert(MotorTask_Command(MOTOR_COMMAND_START, *now) == BSP_CAN_OK);
    }
    assert(task_status().state == MOTOR_TASK_ARMING);
    assert(task_status().target_kp[0] == 0.0f);
    assert(task_status().target_torque_ff_nm[1] == 0.0f);
    task_steps(now, 1U, 0U);
    assert(task_status().state == MOTOR_TASK_ARMING); /* Must see new enabled feedback. */
    task_steps(now, 1U, 1U);
    assert(task_status().state == MOTOR_TASK_RUNNING);
    assert(motor_enable_calls[1] == 1U && motor_enable_calls[2] == 1U);
}

static void test_task_clamp(void)
{
    float out = 123.0f;
    for (unsigned i = 0U; i < 2U; ++i) {
        const MotorTask_JointConfig_t *cfg = &g_motor_config.joint[i];
        for (unsigned j = 0U; j <= 10000U; ++j) {
            assert(MotorTask_ClampTarget(cfg, -12.5f + j * .0025f, &out) == BSP_CAN_OK);
            const uint16_t code = signed_code(out, cfg->device.pmax, UINT16_MAX);
            const float actual = (2.0f * code / 65535.0f - 1.0f) * cfg->device.pmax;
            assert(actual >= cfg->lower_rad && actual <= cfg->upper_rad);
        }
        out = 123.0f;
        assert(MotorTask_ClampTarget(cfg, NAN, &out) == BSP_CAN_INVALID_ARG && out == 123.0f);
        assert(MotorTask_ClampTarget(NULL, 0.0f, &out) == BSP_CAN_INVALID_ARG);
    }
}

static void test_model(void)
{
    MotorTask_Config_t cfg = g_motor_config;
    DragMode_State_t mode = {0};
    DragMode_Status_t status;
    DM_Motor_MIT_Command_t cmd[2];
    DM_Motor_Feedback_t fb[2] = {{.position_rad = -2.0f}, {.position_rad = cfg.drag_j2_zero_rad}};
    assert(DragMode_Update(&cfg, fb, .005f, &mode, &status, cmd));
    assert(status.valid && status.gravity_nm[0] == 0.0f && fabsf(status.gravity_nm[1]) < 1e-7f);
    assert(cmd[1].kp == 0.0f && cmd[1].torque_ff_nm == 0.0f);
    /* Uniform sweep checks the CAD mass matrix and gravity convention. */
    for (int n = -180; n <= 180; ++n) {
        fb[1].position_rad = cfg.drag_j2_zero_rad + n * .01745329252f;
        ++fb[0].received_count; ++fb[1].received_count;
        fb[0].received_at_ms += 5U; fb[1].received_at_ms += 5U;
        assert(DragMode_Update(&cfg, fb, .005f, &mode, &status, cmd));
        const float a = status.inertia[0], b = status.inertia[1], d = status.inertia[2];
        assert(a > 0.0f && d > 0.0f && a*d > b*b);
        assert(fabsf(status.gravity_nm[1] -
            g_drag_model.gravity_sin_nm * sinf(n*.01745329252f)) < 1e-6f);
    }
    fb[1].position_rad = cfg.drag_j2_zero_rad + .5f;
    assert(DragMode_Enter(&cfg, fb, &mode));
    assert(!DragMode_Enter(&cfg, fb, &mode));
    float previous_ff = 0.0f;
    for (unsigned n = 0U; n < 220U; ++n) {
        assert(DragMode_Update(&cfg, fb, .005f, &mode, &status, cmd));
        assert(fabsf(cmd[1].torque_ff_nm - previous_ff) <= .005001f);
        assert(cmd[0].torque_ff_nm == 0.0f && cmd[0].kp == 0.0f);
        previous_ff = cmd[1].torque_ff_nm;
    }
    assert(mode.phase == DRAG_MODE_DRAGGING);
    assert(fabsf(previous_ff - status.gravity_nm[1]) < 1e-6f);
    assert(DragMode_Stop(&cfg, fb, &mode));
    const float captured = mode.hold_target_rad[1];
    fb[1].position_rad += .01f;
    for (unsigned n = 0U; n < 220U; ++n) {
        assert(DragMode_Update(&cfg, fb, .005f, &mode, &status, cmd));
        assert(cmd[1].position_rad == captured);
    }
    assert(mode.phase == DRAG_MODE_HOLDING && cmd[1].kp == cfg.joint[1].hold_kp);
    const float ff = cmd[1].torque_ff_nm;
    assert(DragMode_Resume(&mode));
    assert(DragMode_Update(&cfg, fb, .005f, &mode, &status, cmd));
    assert(cmd[1].position_rad == captured && cmd[1].kp > 0.0f);
    assert(fabsf(cmd[1].torque_ff_nm - ff) < .00001f);
    for (unsigned n = 0U; n < 220U; ++n) {
        assert(DragMode_Update(&cfg, fb, .005f, &mode, &status, cmd));
    }
    assert(mode.phase == DRAG_MODE_DRAGGING && cmd[1].kp == 0.0f);
    assert(cmd[1].position_rad == fb[1].position_rad);
    /* A/B derivatives and velocity products, not merely zero-speed diagnostics. */
    DragMode_Reset(&mode);
    cfg.drag_velocity_filter_ms = 0.0f;
    fb[0].velocity_rad_s = .1f; fb[1].velocity_rad_s = .2f;
    fb[0].received_at_ms = fb[1].received_at_ms = UINT32_MAX - 2U;
    assert(DragMode_Enter(&cfg, fb, &mode));
    assert(DragMode_Update(&cfg, fb, .005f, &mode, &status, cmd));
    const float q = status.q2_rad;
    const float ap = -2*g_drag_model.a_cos2*sinf(2*q) + 2*g_drag_model.a_sin2*cosf(2*q);
    const float bp = -g_drag_model.b_cos*sinf(q) + g_drag_model.b_sin*cosf(q);
    assert(fabsf(status.velocity_coupling_nm[0] - (ap*.1f*.2f + bp*.2f*.2f)) < 1e-8f);
    assert(fabsf(status.velocity_coupling_nm[1] + .5f*ap*.1f*.1f) < 1e-8f);
    fb[0].received_at_ms = fb[1].received_at_ms = 2U;
    ++fb[0].received_count; ++fb[1].received_count;
    fb[1].velocity_rad_s = .25f;
    const float old_kd = cmd[1].kd;
    assert(DragMode_Update(&cfg, fb, .005f, &mode, &status, cmd));
    assert(status.acceleration_rad_s2[1] > 0.0f);
    assert(fabsf(cmd[1].kd - old_kd) <= cfg.joint[1].drag_kd_slew_per_s*.005001f);
    assert(fabsf(status.inertial_nm[0] -
        status.inertia[1]*status.acceleration_rad_s2[1]) < 1e-8f);
    /* No new feedback: acceleration must not be differentiated at control rate. */
    const float acceleration = status.acceleration_rad_s2[1];
    assert(DragMode_Update(&cfg, fb, .005f, &mode, &status, cmd));
    assert(status.acceleration_rad_s2[1] == acceleration);
    DragMode_State_t before = mode;
    DragMode_Status_t before_status = status;
    DM_Motor_MIT_Command_t before_cmd[2];
    memcpy(before_cmd, cmd, sizeof(cmd));
    fb[1].velocity_rad_s = NAN;
    assert(!DragMode_Update(&cfg, fb, .005f, &mode, &status, cmd));
    assert(memcmp(&mode, &before, sizeof(mode)) == 0);
    assert(memcmp(&status, &before_status, sizeof(status)) == 0);
    assert(memcmp(cmd, before_cmd, sizeof(cmd)) == 0);
    DragMode_Reset(&mode);
    assert(mode.phase == DRAG_MODE_OFF);
}

static void test_drag_polarity(void)
{
    MotorTask_Config_t cfg = g_motor_config;
    cfg.drag_velocity_filter_ms = 0.0f;
    const float speeds[] = {0.0f, 5.0f, 10.0f, 20.0f, 30.0f, 40.0f,
                            50.0f, 55.0f, 60.0f, 90.0f, 120.0f};
    /* Pin the steeper 20..60 deg/s curve, independently of config bounds. */
    const float expected_kd[][2] = {
        {.005f, .01f}, {.005f, .01f}, {.005f, .01f}, {.005f, .01f},
        {.19171875f, .2896875f}, {.6025f, .905f},
        {1.01328125f, 1.5203125f}, {1.14865234f, 1.72308594f},
        {1.2f, 1.8f}, {1.2f, 1.8f}, {1.2f, 1.8f}
    };
    for (int direction = -1; direction <= 1; direction += 2) {
        float previous_kd[2] = {0};
        for (unsigned n = 0U; n < sizeof(speeds) / sizeof(speeds[0]); ++n) {
            DragMode_State_t mode = {0};
            DragMode_Status_t status;
            DM_Motor_MIT_Command_t cmd[2];
            const float velocity = direction * speeds[n] * .01745329252f;
            DM_Motor_Feedback_t fb[2] = {
                {.position_rad = -2.0f, .velocity_rad_s = velocity},
                {.position_rad = cfg.drag_j2_zero_rad + 1.5707963268f,
                 .velocity_rad_s = -velocity}
            };
            assert(DragMode_Enter(&cfg, fb, &mode));
            for (unsigned step = 0U; step < 225U; ++step) {
                for (unsigned i = 0U; i < 2U; ++i) {
                    fb[i].received_at_ms += 5U;
                    ++fb[i].received_count;
                }
                assert(DragMode_Update(&cfg, fb, .005f, &mode, &status, cmd));
            }
            assert(mode.phase == DRAG_MODE_DRAGGING);
            assert(fabsf(cmd[0].torque_ff_nm - status.dynamic_ff_nm[0]) < 1e-6f);
            assert(cmd[1].torque_ff_nm * status.q2_rad < 0.0f);
            for (unsigned i = 0U; i < 2U; ++i) {
                assert(cmd[i].kp == 0.0f && cmd[i].velocity_rad_s == 0.0f);
                assert(cmd[i].position_rad == fb[i].position_rad);
                const float damping = cmd[i].kd *
                    (cmd[i].velocity_rad_s - fb[i].velocity_rad_s);
                assert(damping * fb[i].velocity_rad_s <= 0.0f);
                assert(cmd[i].kd >= previous_kd[i]);
                assert(fabsf(cmd[i].kd - expected_kd[n][i]) < 1e-5f);
                if (speeds[n] <= 20.0f) {
                    assert(fabsf(cmd[i].kd - cfg.joint[i].drag_kd_low) < 1e-6f);
                } else if (speeds[n] == 30.0f) {
                    /* Hand-speed regression budget, not merely checking config bounds. */
                    assert(fabsf(damping) < (i == 0U ? .102f : .153f));
                } else if (speeds[n] >= 60.0f) {
                    assert(fabsf(cmd[i].kd - cfg.joint[i].drag_kd_high) < 1e-6f);
                }
                previous_kd[i] = cmd[i].kd;
            }
        }
    }
    /* Actual default filter + slew: a fresh 0 -> +/-90 deg/s feedback step
     * must reach >=90% of the Kd rise within 75ms, without bypassing slew.
     * This is a 5ms-sampled host response budget, not a motor measurement. */
    cfg = g_motor_config;
    for (int direction = -1; direction <= 1; direction += 2) {
        DragMode_State_t mode = {0};
        DragMode_Status_t status;
        DM_Motor_MIT_Command_t cmd[2];
        DM_Motor_Feedback_t fb[2] = {
            {.position_rad = -2.0f},
            {.position_rad = cfg.drag_j2_zero_rad + 1.5707963268f}
        };
        assert(DragMode_Enter(&cfg, fb, &mode));
        for (unsigned step = 0U; step < 225U; ++step) {
            for (unsigned i = 0U; i < 2U; ++i) {
                fb[i].received_at_ms += 5U;
                ++fb[i].received_count;
            }
            assert(DragMode_Update(&cfg, fb, .005f, &mode, &status, cmd));
        }
        fb[0].velocity_rad_s = direction * 90.0f * .01745329252f;
        fb[1].velocity_rad_s = -fb[0].velocity_rad_s;
        for (unsigned step = 1U; step <= 20U; ++step) {
            const float previous_kd[2] = {cmd[0].kd, cmd[1].kd};
            for (unsigned i = 0U; i < 2U; ++i) {
                fb[i].received_at_ms += 5U;
                ++fb[i].received_count;
            }
            assert(DragMode_Update(&cfg, fb, .005f, &mode, &status, cmd));
            for (unsigned i = 0U; i < 2U; ++i) {
                const MotorTask_JointConfig_t *joint = &cfg.joint[i];
                assert(cmd[i].kp == 0.0f && cmd[i].velocity_rad_s == 0.0f);
                assert(fabsf(cmd[i].torque_ff_nm -
                    mode.gravity_ff[i] - status.dynamic_ff_nm[i]) < 1e-6f);
                assert(cmd[i].kd >= previous_kd[i]);
                assert(cmd[i].kd - previous_kd[i] <=
                    joint->drag_kd_slew_per_s * .005001f);
                if (step == 15U) {
                    assert(cmd[i].kd >= joint->drag_kd_low +
                        .9f * (joint->drag_kd_high - joint->drag_kd_low));
                }
                if (step == 20U) {
                    assert(fabsf(cmd[i].kd - joint->drag_kd_high) < 1e-6f);
                }
            }
        }
    }
    /* Exercise the real Task -> Device -> CAN packing path in both directions,
     * including equivalent encoder turns. No extra sign is introduced on the wire. */
    uint32_t now = 1000U;
    setup_task(true, now);
    start_task(&now, false);
    float position[2] = {-2.6f, -2.6f};
    for (unsigned step = 0U; step < 450U; ++step) {
        now += 5U;
        test_tick = now;
        const float v = (step < 225U ? 30.0f : -30.0f) * .01745329252f;
        const float turn = step < 225U ? JOINT_TURN_RAD : -JOINT_TURN_RAD;
        position[0] += v * .005f;
        position[1] -= v * .005f;
        task_feedback(0, position[0] + turn, v, 1U);
        task_feedback(1, position[1] + turn, -v, 1U);
        task_run(now);
        const MotorTask_Status_t s = task_status();
        assert(s.state == MOTOR_TASK_RUNNING && s.reason == MOTOR_REASON_NONE);
        for (unsigned i = 0U; i < 2U; ++i) {
            const uint8_t *data = motor_last_mit[i + 1U];
            const float wire_velocity = (((data[2] << 4) | (data[3] >> 4)) /
                4095.0f * 2.0f - 1.0f) * cfg.joint[i].device.vmax;
            const unsigned wire_kp = ((data[3] & 15U) << 8) | data[4];
            const float wire_kd = ((data[5] << 4) | (data[6] >> 4)) / 4095.0f * 5.0f;
            const float wire_ff = ((((data[6] & 15U) << 8) | data[7]) /
                4095.0f * 2.0f - 1.0f) * cfg.joint[i].device.tmax;
            const float velocity = s.feedback[i].velocity_rad_s;
            assert(wire_kp == 0U && wire_kd >= 0.0f);
            assert(wire_kd * (wire_velocity - velocity) * velocity <= 0.0f);
            assert(fabsf(wire_ff - s.target_torque_ff_nm[i]) < .01368f);
            assert(fabsf(s.reference_rad[i] - position[i]) < .0008f);
            if (step == 224U || step == 449U) {
                DragGeometry_t geometry;
                float expected[3];
                assert(DragGeometry_Evaluate(&g_drag_grip, s.reference_rad[0],
                    s.reference_rad[1] - cfg.drag_j2_zero_rad, &geometry));
                assert(DragGeometry_Damping(&cfg, &geometry,
                    s.drag.filtered_velocity_rad_s, expected));
                assert(fabsf(s.target_kd[i] - expected[i == 0U ? 0U : 2U]) < 1e-5f);
            }
        }
    }
}

static void drag_step(const MotorTask_Config_t *cfg, DM_Motor_Feedback_t fb[2],
    DragMode_State_t *mode, DragMode_Status_t *status, DM_Motor_MIT_Command_t cmd[2])
{
    for (unsigned i = 0U; i < 2U; ++i) {
        fb[i].received_at_ms += 5U;
        ++fb[i].received_count;
    }
    assert(DragMode_Update(cfg, fb, .005f, mode, status, cmd));
}

static void test_grip_geometry(void)
{
    const float pi = 3.14159265358979323846f;
    DragGeometry_t g;
    assert(DragGeometry_Evaluate(&g_drag_grip, 0.0f, 0.0f, &g));
    assert(fabsf(g.radius_m[0] - .023625f) < 1e-7f);
    assert(fabsf(g.radius_m[1] - .1435f) < 1e-7f);
    assert(fabsf(g.position_m[2] - .097f) < 1e-7f);
    MotorTask_GripPoint_t shifted = g_drag_grip;
    shifted.point_from_j2_m[0] += .01f;
    shifted.point_from_j2_m[1] -= .02f;
    shifted.point_from_j2_m[2] += .03f;
    /* Full Jacobian: both columns vs independent position finite differences,
     * nominal and offset points, all poses, including equivalent encoder turns. */
    for (unsigned point = 0U; point < 2U; ++point) {
        const MotorTask_GripPoint_t *p = point == 0U ? &g_drag_grip : &shifted;
        for (int pose = -18; pose <= 18; ++pose) {
            float q[2] = {.37f * pose, pi * pose / 18.0f};
            assert(DragGeometry_Evaluate(p, q[0], q[1], &g));
            DragGeometry_t equivalent;
            assert(DragGeometry_Evaluate(p, q[0] + JOINT_TURN_RAD,
                q[1] - JOINT_TURN_RAD, &equivalent));
            for (unsigned k = 0U; k < 3U; ++k) {
                assert(fabsf(g.position_m[k] - equivalent.position_m[k]) < 3e-7f);
            }
            for (unsigned col = 0U; col < 2U; ++col) {
                DragGeometry_t plus, minus;
                q[col] += .001f;
                assert(DragGeometry_Evaluate(p, q[0], q[1], &plus));
                q[col] -= .002f;
                assert(DragGeometry_Evaluate(p, q[0], q[1], &minus));
                q[col] += .001f;
                for (unsigned k = 0U; k < 3U; ++k) {
                    const float derivative =
                        (plus.position_m[k] - minus.position_m[k]) / .002f;
                    assert(fabsf(derivative - g.jacobian[k][col]) < 4e-5f);
                }
            }
        }
    }
    assert(!DragGeometry_Evaluate(&g_drag_grip, NAN, 0.0f, &g));
    assert(!DragGeometry_Evaluate(&(MotorTask_GripPoint_t){0}, 0.0f, 0.0f, &g));
    /* A rank-deficient upright point needs neither an inverse nor a radius floor. */
    shifted = g_drag_grip;
    shifted.j2_origin_m[1] = -shifted.point_from_j2_m[1];
    assert(DragGeometry_Evaluate(&shifted, 0.0f, 0.0f, &g));
    float d[3];
    assert(DragGeometry_Damping(&g_motor_config, &g, (float[2]){1.0f, 0.0f}, d));
    assert(d[0] == 0.0f && d[1] == 0.0f && isfinite(d[2]));
    /* Same tangential hand speed, J2 fixed: equal modeled damping force at the
     * SAME nominal point across J2 poses. Not a friction/inertia measurement. */
    for (unsigned speed = 1U; speed <= 20U; ++speed) {
        const float hand_speed = speed * .01f;
        assert(DragGeometry_Evaluate(&g_drag_grip, 0.0f, pi*.5f, &g));
        const float rref = g.radius_m[0];
        assert(DragGeometry_Damping(&g_motor_config, &g,
            (float[2]){hand_speed/rref, 0.0f}, d));
        const float force = d[0] * hand_speed / (rref*rref);
        for (int pose = -18; pose <= 18; ++pose) {
            assert(DragGeometry_Evaluate(&g_drag_grip, .7f, pi*pose/18.0f, &g));
            const float r = g.radius_m[0];
            assert(DragGeometry_Damping(&g_motor_config, &g,
                (float[2]){hand_speed/r, 0.0f}, d));
            assert(fabsf(d[0]*hand_speed/(r*r) - force) < 1e-4f);
            assert(d[0]*d[2] - d[1]*d[1] >= -1e-6f);
        }
    }
    /* Full Mode: preserve original base FF; coupled damping remains dissipative
     * after Kd slew/quantization and ONE shared cross-term budget factor. */
    unsigned nonzero_cross = 0U, limited_cross = 0U;
    for (unsigned budget = 0U; budget < 3U; ++budget) {
        MotorTask_Config_t cfg = g_motor_config;
        if (budget != 0U) {
            cfg.joint[0].drag_ff_limit_nm = .02f;
            cfg.joint[1].drag_ff_limit_nm = budget == 1U ? .02f : 0.0f;
        }
        for (int direction = -1; direction <= 1; direction += 2) {
            for (unsigned pose = 0U; pose < 7U; ++pose) {
                DragMode_State_t mode = {0};
                DragMode_Status_t s;
                DM_Motor_MIT_Command_t cmd[2];
                DM_Motor_Feedback_t fb[2] = {
                    {.position_rad = -2.0f, .velocity_rad_s = direction*2.0f},
                    {.position_rad = cfg.drag_j2_zero_rad + pose*.25f,
                     .velocity_rad_s = (pose & 1U) ? 1.2f : -1.2f}
                };
                assert(DragMode_Enter(&cfg, fb, &mode));
                for (unsigned n = 0U; n < 230U; ++n) {
                    const float previous_kd[2] = {mode.kd[0], mode.kd[1]};
                    const bool initialized = mode.initialized;
                    drag_step(&cfg, fb, &mode, &s, cmd);
                    float power = 0.0f;
                    for (unsigned i = 0U; i < 2U; ++i) {
                        const float v = fb[i].velocity_rad_s;
                        const float wire_kd = s.damping[i == 0U ? 0U : 2U];
                        power += (-wire_kd*v + s.grip_ff_nm[i])*v;
                        assert(cmd[i].kp == 0.0f && cmd[i].velocity_rad_s == 0.0f);
                        assert(fabsf(cmd[i].torque_ff_nm) <= cfg.joint[i].drag_ff_limit_nm);
                        assert(fabsf(cmd[i].torque_ff_nm - s.grip_ff_nm[i] -
                            mode.gravity_ff[i] - mode.dynamic_ff[i]) < 1e-7f);
                        if (initialized) {
                            assert(fabsf(mode.kd[i] - previous_kd[i]) <=
                                cfg.joint[i].drag_kd_slew_per_s*.005001f);
                        }
                    }
                    assert(power <= 2e-6f);
                    assert(s.damping[0]*s.damping[2] -
                        s.damping[1]*s.damping[1] >= -1e-6f);
                }
                nonzero_cross += fabsf(s.grip_ff_nm[0]) > 1e-5f;
                limited_cross += budget != 0U && fabsf(s.damping[1]) < 1e-7f;
            }
        }
    }
    assert(nonzero_cross > 0U && limited_cross > 0U);
}

static void test_drag_dynamics(void)
{
    MotorTask_Config_t cfg = g_motor_config;
    assert(g_drag_dynamics.scale == .25f);
    /* Both directions: nonzero M*ddq AND c reach FF at the common fraction.
     * Position targets remain current position, not an invented trajectory. */
    for (int direction = -1; direction <= 1; direction += 2) {
        DragMode_State_t mode = {0};
        DragMode_Status_t s;
        DM_Motor_MIT_Command_t cmd[2];
        DM_Motor_Feedback_t fb[2] = {
            {.position_rad = -2.0f},
            {.position_rad = cfg.drag_j2_zero_rad + direction * .7f}
        };
        assert(DragMode_Enter(&cfg, fb, &mode));
        for (unsigned n = 0U; n < 225U; ++n) {
            drag_step(&cfg, fb, &mode, &s, cmd);
            assert(s.dynamic_ff_nm[0] == 0.0f && s.dynamic_ff_nm[1] == 0.0f);
        }
        for (unsigned n = 0U; n < 200U; ++n) {
            fb[0].velocity_rad_s += direction * .2f * .005f;
            fb[1].velocity_rad_s -= direction * .3f * .005f;
            drag_step(&cfg, fb, &mode, &s, cmd);
            for (unsigned i = 0U; i < 2U; ++i) {
                const float expected = .25f * (s.inertial_nm[i] +
                                                    s.velocity_coupling_nm[i]);
                assert(fabsf(s.dynamic_ff_nm[i] - expected) < 1e-7f);
                assert(fabsf(cmd[i].torque_ff_nm -
                    (s.gravity_nm[i] + expected + s.grip_ff_nm[i])) < 1e-7f);
                assert(cmd[i].kp == 0.0f && cmd[i].velocity_rad_s == 0.0f);
                assert(cmd[i].position_rad == fb[i].position_rad);
            }
        }
        assert(fabsf(s.dynamic_ff_nm[0]) > .00003f);
        assert(fabsf(s.inertial_nm[0]) > .0001f);
        assert(fabsf(s.velocity_coupling_nm[0]) > .0001f);
        /* Replaying the same feedback does not recompute acceleration.
         * >20ms without a new pair clears assistance, NOT gravity support. */
        const float acceleration = s.acceleration_rad_s2[0];
        assert(DragMode_Update(&cfg, fb, .005f, &mode, &s, cmd));
        assert(s.acceleration_rad_s2[0] == acceleration);
        for (unsigned n = 0U; n < 5U; ++n) {
            assert(DragMode_Update(&cfg, fb, .005f, &mode, &s, cmd));
        }
        assert(s.acceleration_rad_s2[0] == 0.0f);
        assert(s.dynamic_ff_nm[0] == 0.0f && s.dynamic_ff_nm[1] == 0.0f);
        assert(cmd[0].torque_ff_nm == 0.0f);
        assert(fabsf(cmd[1].torque_ff_nm - s.gravity_nm[1]) < 1e-7f);
        for (unsigned i = 0U; i < 2U; ++i) { fb[i].received_at_ms += 30U; }
        drag_step(&cfg, fb, &mode, &s, cmd);
        assert(!mode.derivative_ready[0] && !mode.derivative_ready[1]);
        assert(s.dynamic_ff_nm[0] == 0.0f && s.dynamic_ff_nm[1] == 0.0f);
        /* If only one axis recovers, the coupled model remains inactive. */
        for (unsigned n = 0U; n < 6U; ++n) {
            fb[0].received_at_ms += 5U; ++fb[0].received_count;
            fb[0].velocity_rad_s += direction * .02f;
            assert(DragMode_Update(&cfg, fb, .005f, &mode, &s, cmd));
            assert(s.dynamic_ff_nm[0] == 0.0f && s.dynamic_ff_nm[1] == 0.0f);
        }
        for (unsigned i = 0U; i < 2U; ++i) { fb[i].received_at_ms += 35U; }
        for (unsigned n = 0U; n < 40U; ++n) {
            fb[0].velocity_rad_s += direction * .01f;
            drag_step(&cfg, fb, &mode, &s, cmd);
        }
        assert(fabsf(s.dynamic_ff_nm[0]) > .0001f);
        assert(DragMode_Stop(&cfg, fb, &mode));
        for (unsigned n = 0U; n < 225U; ++n) {
            const float previous[2] = {s.dynamic_ff_nm[0], s.dynamic_ff_nm[1]};
            /* Moving feedback must not re-activate acceleration assistance. */
            fb[0].velocity_rad_s = (n % 2U ? .9f : -.9f);
            fb[1].velocity_rad_s = -fb[0].velocity_rad_s;
            drag_step(&cfg, fb, &mode, &s, cmd);
            for (unsigned i = 0U; i < 2U; ++i) {
                assert(fabsf(s.dynamic_ff_nm[i]) <= fabsf(previous[i]));
            }
        }
        assert(s.dynamic_ff_nm[0] == 0.0f && s.dynamic_ff_nm[1] == 0.0f);
        DragMode_Reset(&mode);
        drag_step(&cfg, fb, &mode, &s, cmd);
        assert(cmd[0].torque_ff_nm == 0.0f && cmd[1].torque_ff_nm == 0.0f);
    }
    /* Extreme feedback steps: per-axis cap, slew and smaller total budget. */
    for (unsigned small_budget = 0U; small_budget < 2U; ++small_budget) {
        cfg = g_motor_config;
        if (small_budget) {
            cfg.joint[0].drag_ff_limit_nm = .02f;
            cfg.joint[1].drag_ff_limit_nm = .02f;
        }
        DragMode_State_t mode = {0};
        DragMode_Status_t s;
        DM_Motor_MIT_Command_t cmd[2];
        DM_Motor_Feedback_t fb[2] = {
            {.position_rad = -2.0f},
            {.position_rad = cfg.drag_j2_zero_rad + .7f}
        };
        assert(DragMode_Enter(&cfg, fb, &mode));
        for (unsigned n = 0U; n < 225U; ++n) { drag_step(&cfg, fb, &mode, &s, cmd); }
        for (unsigned n = 0U; n < 80U; ++n) {
            const float previous[2] = {s.dynamic_ff_nm[0], s.dynamic_ff_nm[1]};
            fb[0].velocity_rad_s = 9.0f; fb[1].velocity_rad_s = 9.0f;
            drag_step(&cfg, fb, &mode, &s, cmd);
            for (unsigned i = 0U; i < 2U; ++i) {
                assert(fabsf(s.dynamic_ff_nm[i]) <= g_drag_dynamics.torque_limit_nm[i]);
                assert(fabsf(s.dynamic_ff_nm[i] - previous[i]) <= .002501f);
                assert(fabsf(cmd[i].torque_ff_nm) <= cfg.joint[i].drag_ff_limit_nm);
                const float gravity = fmaxf(-cfg.joint[i].drag_ff_limit_nm,
                    fminf(cfg.joint[i].drag_ff_limit_nm, s.gravity_nm[i]));
                assert(fabsf(cmd[i].torque_ff_nm - s.dynamic_ff_nm[i] -
                    s.grip_ff_nm[i] - gravity) < 1e-7f);
            }
        }
        assert(fabsf(s.dynamic_ff_nm[0]) > (small_budget ? .019f : .099f));
    }
    /* Real Task -> Device -> CAN: dynamics must survive packing, and remain
     * identical through an encoder +/-one-turn change (no sign workaround). */
    uint32_t now = 1000U;
    setup_task(true, now);
    start_task(&now, false);
    task_steps(&now, 225U, 1U);
    for (unsigned n = 0U; n < 25U; ++n) {
        now += 5U; test_tick = now;
        task_feedback(0, -2.6f, .12f * (n + 1U), 1U);
        task_feedback(1, -2.2f, -.08f * (n + 1U), 1U);
        task_run(now);
        const MotorTask_Status_t s = task_status();
        assert(s.state == MOTOR_TASK_RUNNING && s.drag.valid);
        assert(fabsf(s.drag.dynamic_ff_nm[0]) > 0.0f);
        for (unsigned i = 0U; i < 2U; ++i) {
            const uint8_t *data = motor_last_mit[i + 1U];
            const unsigned code = ((data[6] & 15U) << 8) | data[7];
            const float wire_ff = (code / 4095.0f * 2.0f - 1.0f) * 28.0f;
            assert(fabsf(wire_ff - s.target_torque_ff_nm[i]) < .01368f);
            assert(s.target_kp[i] == 0.0f && s.target_velocity_rad_s[i] == 0.0f);
        }
    }
    const MotorTask_Status_t before = task_status();
    now += 5U; test_tick = now;
    task_feedback(0, -2.6f + JOINT_TURN_RAD,
                  before.feedback[0].velocity_rad_s, 1U);
    task_feedback(1, -2.2f + JOINT_TURN_RAD,
                  before.feedback[1].velocity_rad_s, 1U);
    task_run(now);
    const MotorTask_Status_t after = task_status();
    assert(after.state == MOTOR_TASK_RUNNING);
    assert(fabsf(before.drag.inertia[0] - after.drag.inertia[0]) < .00001f);
    assert(fabsf(after.reference_rad[1] - before.reference_rad[1]) < .0008f);
    assert(after.target_torque_ff_nm[0] ==
        after.drag.dynamic_ff_nm[0] + after.drag.grip_ff_nm[0]);
    can_fail_mit_id = 2U;
    task_steps(&now, 1U, 1U);
    assert(task_status().state == MOTOR_TASK_FAULT && !task_status().drag.valid);
    const unsigned mit = motor_mit_calls[1] + motor_mit_calls[2];
    task_steps(&now, 10U, 1U);
    assert(motor_mit_calls[1] + motor_mit_calls[2] == mit);
}

static uint32_t float_bits(float value)
{
    uint32_t result;
    memcpy(&result, &value, sizeof(result));
    return result;
}

static uint32_t tuning_request(uint8_t op, uint8_t scope, uint8_t field,
                               uint32_t value, BSP_CAN_Status_t expected)
{
    static uint8_t seq;
    const uint8_t data[8] = {++seq, op, scope, field,
        (uint8_t)value, (uint8_t)(value>>8), (uint8_t)(value>>16), (uint8_t)(value>>24)};
    inject_motor_feedback(&hfdcan2, MOTOR_TUNING_REQUEST_ID, data, 8U);
    assert(MotorTuning_Run(test_tick) == BSP_CAN_OK);
    assert(can_tx_header.Identifier == MOTOR_TUNING_RESPONSE_ID);
    assert(can_tx_data[0] == seq && can_tx_data[1] == expected);
    return (uint32_t)can_tx_data[4] | ((uint32_t)can_tx_data[5]<<8) |
        ((uint32_t)can_tx_data[6]<<16) | ((uint32_t)can_tx_data[7]<<24);
}

static void test_tuning(uint32_t now)
{
    task_steps(&now, 1U, 0U);
    assert(MotorTuning_Init(&hfdcan2) == BSP_CAN_OK);
    assert(JointCalibration_Init() == BSP_CAN_OK);
    assert(tuning_request(0, 0, 0, 0, BSP_CAN_OK) == 0x594C0003U);
    for (uint8_t joint = 1U; joint <= 2U; ++joint) {
        (void)tuning_request(1, joint, 6, 0, BSP_CAN_INVALID_ARG);
        const DM_Motor_Config_t *device = &g_motor_config.joint[joint - 1U].device;
        assert(tuning_request(1, joint, 240, 0, BSP_CAN_OK) == float_bits(device->pmax));
        assert(tuning_request(1, joint, 241, 0, BSP_CAN_OK) == float_bits(device->vmax));
        assert(tuning_request(1, joint, 242, 0, BSP_CAN_OK) == float_bits(device->tmax));
    }
    (void)tuning_request(1, 0, 240, 0, BSP_CAN_INVALID_ARG);
    (void)tuning_request(1, 3, 240, 0, BSP_CAN_INVALID_ARG);
    (void)tuning_request(1, 1, 243, 0, BSP_CAN_INVALID_ARG);
    assert(tuning_request(11, 0, 255, 0, BSP_CAN_OK) == 0x594C4402U);
    (void)tuning_request(11, 1, 4, 0, BSP_CAN_NOT_READY);
    assert((tuning_request(11, 0, 0, 0, BSP_CAN_OK) & 1U) != 0U);
    const uint32_t gravity = tuning_request(11, 1, 4, 0, BSP_CAN_OK);
    assert(gravity == float_bits(0.0f));
    (void)tuning_request(2, 0, 0, 1, BSP_CAN_OK);
    (void)tuning_request(3, 1, 240, float_bits(1.0f), BSP_CAN_INVALID_ARG);
    (void)tuning_request(3, 1, 6, float_bits(10.0f), BSP_CAN_INVALID_ARG);
    (void)tuning_request(3, 1, 3, float_bits(NAN), BSP_CAN_INVALID_ARG);
    (void)tuning_request(3, 1, 4, float_bits(6.0f), BSP_CAN_INVALID_ARG);
    (void)tuning_request(3, 1, 3, float_bits(6.0f), BSP_CAN_OK);
    (void)tuning_request(4, 0, 0, 0, BSP_CAN_INVALID_ARG); /* partial */
    (void)tuning_request(2, 0, 0, 1, BSP_CAN_OK);
    for (unsigned attempt = 0U; attempt < 2U; ++attempt) {
#define STAGE(scope, id) do { \
    uint32_t bits = tuning_request(1, scope, id, 0, BSP_CAN_OK); \
    (void)tuning_request(3, scope, id, bits, BSP_CAN_OK); \
} while (0)
#define TUNE_JOINT(id, member, type, unit, lo, hi, label, group) STAGE(1, id); STAGE(2, id);
#define TUNE_GLOBAL(id, member, type, unit, lo, hi, label, group) STAGE(0, id);
#include "motor_tuning_fields.def"
#undef TUNE_JOINT
#undef TUNE_GLOBAL
#undef STAGE
        if (attempt == 0U) {
            (void)tuning_request(3, 1, 9, float_bits(1.0f), BSP_CAN_OK);
            (void)tuning_request(4, 0, 0, 0, BSP_CAN_INVALID_ARG); /* Removed SPEED bit. */
            /* Failed commit discards staging: retry needs a new full transaction. */
            (void)tuning_request(2, 0, 0, 1, BSP_CAN_OK);
        }
    }
    (void)tuning_request(3, 1, 3, float_bits(6.0f), BSP_CAN_OK);
    assert(tuning_request(4, 0, 0, 0, BSP_CAN_OK) == 2U);
    MotorTask_Config_t cfg;
    assert(MotorTask_GetConfig(&cfg) == BSP_CAN_OK && cfg.joint[0].hold_kp == 6.0f);
    assert(motor_enable_calls[1] == 0U && motor_mit_calls[1] == 0U);
    (void)tuning_request(7, 0, MOTOR_COMMAND_START, 1, BSP_CAN_BUSY);
    (void)tuning_request(11, 0, 0, 0, BSP_CAN_OK);
    test_tick += 1001U;
    (void)tuning_request(11, 1, 4, 0, BSP_CAN_NOT_READY);
    assert((tuning_request(11, 0, 0, 0, BSP_CAN_OK) & 1U) == 0U);
    uint32_t unknown = tuning_request(11, 1, 4, 0, BSP_CAN_OK);
    float value;
    memcpy(&value, &unknown, sizeof(value));
    assert(isnan(value));
}

static void stage_motion(const MotionMode_Request_t *request)
{
    (void)tuning_request(12, 0, 2, 1, BSP_CAN_OK);
    for (uint8_t j = 1; j <= 2; ++j) {
        const float values[] = {request->target_rad[j-1], request->velocity_rad_s[j-1],
            request->acceleration_rad_s2[j-1], request->jerk_rad_s3[j-1]};
        for (uint8_t f = 1; f <= 4; ++f) {
            (void)tuning_request(12, j, f, float_bits(values[f-1] * 57.295779513f),
                                 BSP_CAN_OK);
        }
    }
}

static void test_motion(uint32_t now)
{
    assert(MotorTuning_Init(&hfdcan2) == BSP_CAN_OK);
    assert(tuning_request(12, 0, 255, 0, BSP_CAN_OK) == MOTOR_MOTION_VERSION);
    (void)tuning_request(12, 0, 1, MOTOR_MODE_MOTION, BSP_CAN_OK);
    assert(tuning_request(12, 0, 0, 0, BSP_CAN_OK) == MOTOR_MODE_MOTION);
    (void)tuning_request(12, 0, 2, 1, BSP_CAN_BUSY); /* No implicit enable. */
    assert(motor_enable_calls[1] == 0U);
    start_task(&now, true); /* Physical KEY also starts Motion, not just RPC. */
    task_steps(&now, 220U, 1U);
    assert(task_status().state == MOTOR_TASK_HOLDING && !task_status().drag.valid);
    assert(task_status().motion.valid && task_status().target_kp[0] == g_motion_config.kp[0]);
    (void)tuning_request(12, 0, 1, MOTOR_MODE_DRAG, BSP_CAN_BUSY);
    MotionMode_Request_t request = {.target_rad = {-2.3f, -2.35f},
        .velocity_rad_s = {.2f, .25f}, .acceleration_rad_s2 = {.4f, .5f},
        .jerk_rad_s3 = {1, 1.2f}};
    (void)tuning_request(12, 0, 2, 1, BSP_CAN_OK);
    (void)tuning_request(12, 1, 1, float_bits(-140), BSP_CAN_OK);
    (void)tuning_request(12, 0, 3, 1, BSP_CAN_INVALID_ARG); /* Partial job. */
    assert(task_status().state == MOTOR_TASK_HOLDING);
    stage_motion(&request);
    (void)tuning_request(12, 0, 3, 1, BSP_CAN_OK);
    (void)tuning_request(12, 0, 3, 1, BSP_CAN_INVALID_ARG); /* Never replay. */
    assert(task_status().state == MOTOR_TASK_RUNNING);
    for (unsigned n = 0; n < 1500 && task_status().state != MOTOR_TASK_HOLDING; ++n) {
        task_steps(&now, 1U, 1U);
        const MotorTask_Status_t status = task_status();
        assert(status.state != MOTOR_TASK_FAULT);
        for (unsigned i = 0; i < 2; ++i) {
            const RobotModel_Dynamics_t *model = &status.motion.model;
            assert(fabsf(status.target_torque_ff_nm[i] - model->inertial_nm[i] -
                         model->coupling_nm[i] - model->gravity_nm[i] -
                         status.motion.friction_nm[i]) < 1e-6f);
        }
    }
    MotorTask_Status_t status = task_status();
    assert(status.state == MOTOR_TASK_HOLDING);
    assert(fabsf(status.target_reference_rad[0] - request.target_rad[0]) < 1e-6f);
    assert(fabsf(status.motion.position_error_rad[0]) > .1f); /* Compliant lag is not Drag trip. */
    (void)tuning_request(12, 0, 5, 0, BSP_CAN_OK);
    assert(tuning_request(12, 2, 11, 0, BSP_CAN_OK) ==
           float_bits(status.motion.model.gravity_nm[1]));
    task_steps(&now, 30, 1);
    assert(task_status().state == MOTOR_TASK_HOLDING && motor_enable_calls[1] == 1);
    request.target_rad[0] = -2.8f; request.target_rad[1] = -2.9f;
    assert(MotorTask_Move(&request, now) == BSP_CAN_OK);
    task_steps(&now, 65, 1);
    const float v_before = task_status().target_velocity_rad_s[0];
    assert(fabsf(v_before) > .01f);
    assert(MotorTask_Command(MOTOR_COMMAND_STOP, now) == BSP_CAN_OK);
    task_steps(&now, 1, 1);
    assert(task_status().state == MOTOR_TASK_STOPPING);
    assert(fabsf(task_status().target_velocity_rad_s[0] - v_before) < .01f);
    task_steps(&now, 400, 1);
    status = task_status();
    assert(status.state == MOTOR_TASK_HOLDING);
    assert(fabsf(status.target_reference_rad[0] - request.target_rad[0]) > .05f);
    /* Same inverse encoder branch in Motion; never a multi-turn PD jump. */
    test_tick = now += 5;
    for (unsigned i = 0; i < 2; ++i) { task_feedback(i, -2.6f + JOINT_TURN_RAD, 0, 1); }
    task_run(now);
    assert(task_status().state == MOTOR_TASK_HOLDING);
    assert(fabsf(task_status().target_rad[0] - status.target_reference_rad[0] -
                 JOINT_TURN_RAD) < .001f);
    request.target_rad[0] = NAN;
    assert(MotorTask_Move(&request, now) == BSP_CAN_INVALID_ARG);
    assert(task_status().state == MOTOR_TASK_HOLDING);
    request.target_rad[0] = -2.7f;
    stage_motion(&request);
    (void)tuning_request(7, 0, MOTOR_COMMAND_STOP, 0, BSP_CAN_OK);
    (void)tuning_request(12, 0, 3, 1, BSP_CAN_INVALID_ARG);
    stage_motion(&request);
    task_steps(&now, 602, 1);
    (void)tuning_request(12, 0, 3, 1, BSP_CAN_INVALID_ARG);
    stage_motion(&request);
    assert(MotorTask_Command(MOTOR_COMMAND_DISABLE, now) == BSP_CAN_OK);
    task_steps(&now, 1, 0);
    assert(MotorTask_Command(MOTOR_COMMAND_CLEAR, now) == BSP_CAN_OK);
    assert(MotorTask_Command(MOTOR_COMMAND_START, now) == BSP_CAN_OK);
    task_steps(&now, 222, 1);
    assert(task_status().state == MOTOR_TASK_HOLDING);
    (void)tuning_request(12, 0, 3, 1, BSP_CAN_INVALID_ARG); /* Old generation discarded. */
}

static void test_impedance(uint32_t now)
{
    assert(MotorTuning_Init(&hfdcan2) == BSP_CAN_OK);
    assert(tuning_request(12, 0, 255, 0, BSP_CAN_OK) == 0x594C5002U);
    (void)tuning_request(12, 0, 8, 0, BSP_CAN_BUSY); /* Wrong config revision. */
    assert(task_status().mode == MOTOR_MODE_DRAG && motor_enable_calls[1] == 0);
    (void)tuning_request(12, 0, 8, 1, BSP_CAN_OK); /* No KEY/Mode/8-field draft. */
    const float center[2] = {task_status().reference_rad[0], task_status().reference_rad[1]};
    assert(task_status().state == MOTOR_TASK_ARMING);
    assert(task_status().mode == MOTOR_MODE_MOTION);
    assert(motor_enable_calls[1] == 1 && motor_enable_calls[2] == 1);
    task_steps(&now, 3, 0); /* Do not ramp gains before both enable ACKs. */
    assert(task_status().state == MOTOR_TASK_ARMING && task_status().target_kp[0] == 0);
    test_tick = now += 5;
    for (unsigned i = 0; i < 2; ++i) { task_feedback(i, -2.55f, 0, 1); }
    task_run(now);
    task_steps(&now, 220, 1);
    assert(task_status().state == MOTOR_TASK_HOLDING);
    for (unsigned i = 0; i < 2; ++i) {
        assert(task_status().target_reference_rad[i] == center[i]); /* Not ACK-time pose. */
    }
    /* Push either side, well beyond Drag's tracking threshold. The center
     * stays fixed and the PD request always pulls back toward it. This checks
     * the command law, not real motor settling/friction or mechanical motion. */
    for (unsigned side = 0; side < 2; ++side) {
        const float displacement = side ? .3f : -.3f;
        for (unsigned n = 0; n < 30; ++n) {
            test_tick = now += 5;
            for (unsigned i = 0; i < 2; ++i) {
                task_feedback(i, center[i] + displacement, 0, 1);
            }
            task_run(now);
            const MotorTask_Status_t status = task_status();
            assert(status.state == MOTOR_TASK_HOLDING);
            for (unsigned i = 0; i < 2; ++i) {
                assert(status.target_reference_rad[i] == center[i]);
                assert(status.target_velocity_rad_s[i] == 0);
                assert(status.motion.target_acceleration_rad_s2[i] == 0);
                assert(status.target_kp[i] == g_motion_config.kp[i]);
                assert(status.target_kp[i] * status.motion.position_error_rad[i] *
                       displacement < 0);
            }
        }
    }
    /* Exercise the real Device -> continuous coordinates -> Motion -> MIT
     * path: moving encoders create friction even with fixed target v=0 and
     * deliberately opposite motor-reported velocity. No recapture occurs. */
    for (unsigned side = 0; side < 2; ++side) {
        const float direction = side ? -1.0f : 1.0f;
        for (unsigned n = 0; n < 120; ++n) {
            test_tick = now += 5;
            for (unsigned i = 0; i < 2; ++i) {
                task_feedback(i, center[i]+direction*.001f*n, -direction*.2f, 1);
            }
            task_run(now);
            assert(task_status().state == MOTOR_TASK_HOLDING);
        }
        const MotorTask_Status_t moving = task_status();
        for (unsigned i = 0; i < 2; ++i) {
            assert(moving.motion.friction_velocity_rad_s[i]*direction > .17f);
            assert(moving.motion.friction_nm[i]*direction > .48f);
            assert(moving.target_reference_rad[i] == center[i]);
            assert(moving.target_velocity_rad_s[i] == 0);
            assert(fabsf(moving.target_torque_ff_nm[i] -
                         moving.motion.model.inertial_nm[i] -
                         moving.motion.model.coupling_nm[i] -
                         moving.motion.model.gravity_nm[i] -
                         moving.motion.friction_nm[i]) < 1e-6f);
        }
    }
    /* Explicitly clicking again recaptures once without cycling the enables. */
    (void)tuning_request(12, 0, 8, 1, BSP_CAN_OK);
    const float recaptured = task_status().reference_rad[0];
    task_steps(&now, 220, 1);
    assert(task_status().target_reference_rad[0] == recaptured);
    assert(motor_enable_calls[1] == 1 && motor_disable_calls[1] == 0);
    /* A stale feedback failure cannot be silently cleared by the shortcut. */
    test_tick = now += g_motor_config.feedback_max_age_ms + 1;
    (void)tuning_request(12, 0, 8, 1, BSP_CAN_ERROR);
    assert(task_status().state == MOTOR_TASK_FAULT);
    (void)tuning_request(12, 0, 8, 1, BSP_CAN_BUSY);
    assert(motor_enable_calls[1] == 1);
    task_steps(&now, 1, 0);
    assert(MotorTask_Command(MOTOR_COMMAND_CLEAR, now) == BSP_CAN_OK);
    assert(MotorTask_SelectMode(MOTOR_MODE_DRAG) == BSP_CAN_OK);
    assert(MotorTask_Command(MOTOR_COMMAND_START, now) == BSP_CAN_OK);
    task_steps(&now, 1, 1);
    task_steps(&now, 220, 1);
    const MotorTask_Status_t drag = task_status();
    assert(drag.mode == MOTOR_MODE_DRAG && drag.state == MOTOR_TASK_RUNNING);
    const unsigned enables = motor_enable_calls[1], disables = motor_disable_calls[1];
    (void)tuning_request(12, 0, 8, 1, BSP_CAN_OK);
    task_steps(&now, 1, 1);
    const MotorTask_Status_t entering = task_status();
    assert(entering.mode == MOTOR_MODE_MOTION && !entering.drag.valid);
    assert(entering.target_kp[0] > 0 && entering.target_kp[0] < g_motion_config.kp[0]);
    assert(fabsf(entering.target_kd[1] - drag.target_kd[1]) < .01f);
    assert(fabsf(entering.target_torque_ff_nm[1] - drag.target_torque_ff_nm[1]) < .01f);
    task_steps(&now, 220, 1);
    assert(task_status().state == MOTOR_TASK_HOLDING);
    assert(motor_enable_calls[1] == enables && motor_disable_calls[1] == disables);
    /* F6 remains effective; a new explicit impedance click may re-enable
     * after fresh disabled feedback, without clearing unrelated faults. */
    assert(MotorTask_Command(MOTOR_COMMAND_DISABLE, now) == BSP_CAN_OK);
    task_steps(&now, 1, 0);
    (void)tuning_request(12, 0, 8, 1, BSP_CAN_OK);
    task_steps(&now, 222, 1);
    assert(task_status().state == MOTOR_TASK_HOLDING);
    assert(motor_enable_calls[1] == enables + 1);
}

static void assert_task_fault(MotorTask_Reason_t reason, uint32_t now, bool submitted)
{
    assert(task_status().state == MOTOR_TASK_FAULT && task_status().reason == reason);
    assert(task_status().disable_submitted_mask == (submitted ? 3U : 0U));
    const unsigned commands = motor_mit_calls[1] + motor_mit_calls[2];
    const unsigned disables = motor_disable_calls[1] + motor_disable_calls[2];
    task_steps(&now, 20U, 1U);
    assert(motor_mit_calls[1] + motor_mit_calls[2] == commands);
    assert(motor_disable_calls[1] + motor_disable_calls[2] == disables);
    assert(MotorTask_Command(MOTOR_COMMAND_START, now) == BSP_CAN_BUSY);
}

static void coordinate_feedback(uint32_t now, uint8_t state, float turns)
{
    test_tick = now;
    task_feedback(0, -.66129f + turns * 6.28318530718f, 0, state);
    task_feedback(1, -3.17320568f + turns * 6.28318530718f, 0, state);
}

static void test_coordinates(bool hold)
{
    uint32_t now = 1000U;
    setup_task(true, now);
    /* FIRST Task snapshot has positive raw positions in the equivalent +1 turn.
     * The configured reference intervals, not the feedback sign, choose branch. */
    coordinate_feedback(now += 5U, 0U, 1.0f);
    task_run(now);
    MotorTask_Status_t s = task_status();
    assert(s.feedback[0].position_rad > 0 && s.feedback[1].position_rad > 0);
    assert(fabsf(s.reference_rad[0] + .66129f) < .0004f);
    assert(fabsf(s.reference_rad[1] + 3.17320568f) < .0004f);
    assert(fabsf(s.drag.q2_rad - (-3.17320568f - g_motor_config.drag_j2_zero_rad)) < .0004f);
    assert(s.drag.gravity_nm[0] == 0 && s.drag.gravity_nm[1] > .24f);
    assert(motor_enable_calls[1] == 0U && motor_mit_calls[1] == 0U);
    const float reference = s.reference_rad[1], gravity = s.drag.gravity_nm[1];
    const float a = s.drag.inertia[0], b = s.drag.inertia[1];
    assert(MotorTuning_Init(&hfdcan2) == BSP_CAN_OK);
    assert(JointCalibration_Init() == BSP_CAN_OK);
    assert(tuning_request(8, 0, 255, 0, BSP_CAN_OK) == 0x594C4302U);
    (void)tuning_request(8, 0, 0, 0, BSP_CAN_OK);
    assert(tuning_request(8, 2, 2, 0, BSP_CAN_OK) ==
           float_bits(s.feedback[1].position_rad * 57.295779513f));
    assert(tuning_request(8, 2, 3, 0, BSP_CAN_OK) ==
           float_bits((reference - g_motor_config.drag_j2_zero_rad) * 57.295779513f));
    assert(tuning_request(8, 2, 4, 0, BSP_CAN_OK) ==
           float_bits(reference * 57.295779513f));
    (void)tuning_request(11, 0, 0, 0, BSP_CAN_OK);
    assert(tuning_request(11, 2, 13, 0, BSP_CAN_OK) ==
           float_bits(reference * 57.295779513f));
    assert(tuning_request(11, 2, 14, 0, BSP_CAN_OK) ==
           float_bits(s.reference_offset_rad[1] * 57.295779513f));
    assert(MotorTask_Command(MOTOR_COMMAND_START, now) == BSP_CAN_OK);
    for (unsigned n = 0; n < 225U; ++n) {
        coordinate_feedback(now += 5U, 1U, 1.0f);
        task_run(now);
    }
    s = task_status();
    assert(s.state == MOTOR_TASK_RUNNING);
    assert(fabsf(s.target_rad[1] - s.feedback[1].position_rad) < .0004f);
    assert(fabsf(s.target_torque_ff_nm[1] - gravity) < .0001f);
    if (hold) {
        assert(MotorTask_Command(MOTOR_COMMAND_STOP, now) == BSP_CAN_OK);
        for (unsigned n = 0; n < 225U; ++n) {
            coordinate_feedback(now += 5U, 1U, 1.0f);
            task_run(now);
        }
        assert(task_status().state == MOTOR_TASK_HOLDING);
    }
    /* Same physical pose with -1, 0, +1 wire turns. Both pure drag and nonzero
     * Kp holding must retain reference/gravity/inertia and invert each target. */
    for (int turns = -1; turns <= 1; ++turns) {
        coordinate_feedback(now += 5U, 1U, (float)turns);
        task_run(now);
        s = task_status();
        assert(s.state == (hold ? MOTOR_TASK_HOLDING : MOTOR_TASK_RUNNING));
        assert(fabsf(s.reference_rad[1] - reference) < .0008f);
        assert(fabsf(s.drag.gravity_nm[1] - gravity) < .0004f);
        assert(fabsf(s.drag.inertia[0] - a) < .00001f);
        assert(fabsf(s.drag.inertia[1] - b) < .000001f);
        for (unsigned i = 0; i < 2U; ++i) {
            assert(fabsf(s.target_rad[i] - s.feedback[i].position_rad) < .0008f);
            /* Check the real submitted MIT position, not only status fields. */
            const uint16_t code = ((uint16_t)motor_last_mit[i+1][0] << 8) |
                                  motor_last_mit[i+1][1];
            const float wire_target = (2.0f * code / 65535.0f - 1.0f) * 12.5f;
            assert(fabsf(wire_target - s.feedback[i].position_rad) < .0012f);
        }
        assert(s.target_kp[1] == (hold ? 5.0f : 0.0f));
    }
    assert(motor_enable_calls[1] == 1U && motor_disable_calls[1] == 0U);
    /* Physical small motion through the usual +pi -> -pi raw boundary. */
    for (unsigned n = 0; n < 2U; ++n) {
        coordinate_feedback(now += 5U, 1U, 0);
        task_feedback(1, n == 0U ? 3.1414f : -3.1414f, 0, 1);
        task_run(now);
        s = task_status();
        assert(s.state == (hold ? MOTOR_TASK_HOLDING : MOTOR_TASK_RUNNING));
        assert(fabsf(s.reference_rad[1] + 3.141592654f) < .0007f);
        assert(fabsf(s.target_rad[1] - s.feedback[1].position_rad) < .04f);
    }
    /* A genuinely out-of-range posture is NOT pulled onto a legal limit. */
    coordinate_feedback(now += 5U, 1U, 0);
    task_feedback(1, -.5f, 0, 1);
    task_run(now);
    assert(task_status().state == MOTOR_TASK_FAULT &&
           task_status().reason == MOTOR_REASON_ANGLE);
}

static void trigger_bus_off(uint32_t *now)
{
    can_protocol[1].BusOff = 1U;
    can_protocol[1].LastErrorCode = 5U;
    hfdcan2.Instance->CCCR = FDCAN_CCCR_INIT;
    hfdcan2.Instance->TXBRP = 0xF0U; /* Include pre-fault MIT/query requests. */
    hfdcan2.Instance->IR = FDCAN_IT_BUS_OFF;
    task_steps(now, 1U, 1U);
    for (unsigned n = 0U; n < 20U && task_status().state == MOTOR_TASK_IDLE; ++n) {
        task_steps(now, 1U, 0U); /* Inactive bus check is intentionally 100ms. */
    }
    assert(task_status().state == MOTOR_TASK_RECOVERING);
}

static void test_recovery(const char *name)
{
    uint32_t now = UINT32_MAX - 200U;
    const bool idle = strcmp(name, "task-recovery-idle") == 0;
    const bool motion = strcmp(name, "task-recovery-motion") == 0;
    const bool hold = strcmp(name, "task-recovery-hold") == 0;
    const bool blocked = strcmp(name, "task-recovery-flush") == 0;
    const bool silent = strcmp(name, "task-recovery-stale") == 0;
    const bool limit = strcmp(name, "task-recovery-limit") == 0;
    const bool key = strcmp(name, "task-recovery-key") == 0;
    const bool stop = strcmp(name, "task-recovery-stop") == 0;
    const bool disable = strcmp(name, "task-recovery-disable") == 0;
    const bool hot = strcmp(name, "task-recovery-hot") == 0;
    setup_task(true, now);
    task_steps(&now, 1U, 0U);
    if (motion) { assert(MotorTask_SelectMode(MOTOR_MODE_MOTION) == BSP_CAN_OK); }
    if (!idle) {
        start_task(&now, false);
        task_steps(&now, 220U, 1U);
        if (motion) {
            const MotionMode_Request_t job = {.target_rad = {-2, -2.2f},
                .velocity_rad_s = {.2f, .2f}, .acceleration_rad_s2 = {.4f, .4f},
                .jerk_rad_s3 = {1, 1}};
            assert(MotorTask_Move(&job, now) == BSP_CAN_OK);
            task_steps(&now, 40U, 1U);
            assert(task_status().state == MOTOR_TASK_RUNNING);
        }
        if (hold) {
            assert(MotorTask_Command(MOTOR_COMMAND_STOP, now) == BSP_CAN_OK);
            task_steps(&now, 220U, 1U);
            assert(task_status().state == MOTOR_TASK_HOLDING);
        }
    }
    if (blocked) { can_stop_flush = false; }
    trigger_bus_off(&now);
    /* Recovery is not a healthy result merely because the next 5ms control
     * step is not due yet. The fast and full Run paths must agree. */
    test_tick = now + 1U;
    assert(MotorTask_Run(test_tick) == BSP_CAN_ERROR);
    assert(task_status().state == MOTOR_TASK_RECOVERING);
    assert(can_stop_calls == 1U && can_start_calls == 1U);
    assert(motor_enable_calls[1] == (idle ? 0U : 1U));
    const unsigned mit = motor_mit_calls[1];
    assert(task_status().drag.phase == DRAG_MODE_OFF);
    if (!blocked) { assert(hfdcan2.Instance->TXBRP == 0U); }
    MotorTask_Config_t cfg;
    assert(MotorTask_GetConfig(&cfg) == BSP_CAN_OK);
    assert(MotorTask_ApplyConfig(&cfg, now) == BSP_CAN_BUSY);
    assert(MotorTask_Command(MOTOR_COMMAND_START, now) == BSP_CAN_BUSY);
    assert(MotorTask_Command(MOTOR_COMMAND_CLEAR, now) == BSP_CAN_BUSY);
    if (stop || disable) {
        (void)MotorTask_Command(stop ? MOTOR_COMMAND_STOP : MOTOR_COMMAND_DISABLE, now);
    }
    if (key) { test_gpioa.IDR = 0U; }
    task_steps(&now, 19U, 1U); /* Bus must not restart before the retry spacing. */
    test_gpioa.IDR = KEY__INPUT_Pin;
    assert(can_start_calls == 1U && motor_mit_calls[1] == mit);
    task_steps(&now, 2U, 1U);
    if (blocked) {
        task_steps(&now, 950U, 1U);
        assert(task_status().state == MOTOR_TASK_FAULT);
        assert(can_start_calls == 1U && hfdcan2.Instance->TXBRP == 0xF0U);
        assert(motor_mit_calls[1] == mit && motor_enable_calls[1] == 1U);
        assert(can_stop_calls == 1U); /* Never release INIT with old requests. */
        return;
    }
    assert(can_start_calls == 2U && motor_mit_calls[1] == mit);
    task_steps(&now, 2U, 1U);
    assert(motor_disable_calls[1] == 0U); /* BO still true: Start is not completion. */
    can_protocol[1].BusOff = 0U; /* Simulate hardware finishing bus-idle recovery. */
    task_steps(&now, 1U, 1U);
    assert(motor_disable_calls[1] == 1U && motor_disable_calls[2] == 1U);
    assert(motor_mit_calls[1] == mit);
    if (silent) {
        for (unsigned n = 0; n < 950U; ++n) { task_run(now += 5U); }
        assert(task_status().state == MOTOR_TASK_FAULT);
        assert(motor_mit_calls[1] == mit && motor_enable_calls[1] == 1U);
        return;
    }
    if (hot) {
        /* Current fixture rotor is 26C, below the real default 60C. Inject a
         * real hot feedback frame instead of changing policy during recovery. */
        const uint8_t data[8] = {2U, 0x65U, 0x60U, 0x7FU, 0xF7U, 0xFFU, 60U, 60U};
        for (unsigned n = 0; n < 25U; ++n) {
            feedback_pair(now += 5U, 0U);
            inject_motor_feedback(&hfdcan2, 0x12U, data, 8U);
            task_run(now);
        }
        task_steps(&now, 30U, 0U); /* Cooling later must not restore the vetoed output. */
        assert(task_status().state == MOTOR_TASK_FAULT);
        assert(task_status().reason == MOTOR_REASON_TEMPERATURE);
        assert(motor_enable_calls[1] == 1U && motor_mit_calls[1] == mit);
        return;
    }
    /* Recovery uses a different actual pose, never the old hold/MIT target. */
    for (unsigned n = 0U; n < 22U; ++n) {
        test_tick = now += 5U;
        task_feedback(0, -2.4f, 0, 0U);
        task_feedback(1, -2.4f, 0, 0U);
        task_run(now);
    }
    if (idle || key || stop || disable) {
        assert(task_status().state == (idle ? MOTOR_TASK_IDLE : MOTOR_TASK_FAULT));
        assert(motor_enable_calls[1] == (idle ? 0U : 1U));
        assert(motor_mit_calls[1] == mit);
        task_steps(&now, 50U, 0U);
        assert(motor_enable_calls[1] == (idle ? 0U : 1U));
        return;
    }
    assert(task_status().state == MOTOR_TASK_ARMING);
    assert(motor_enable_calls[1] == 2U && motor_enable_calls[2] == 2U);
    assert(task_status().target_kp[1] == 0.0f && task_status().target_torque_ff_nm[1] == 0.0f);
    assert(fabsf(task_status().target_rad[1] + 2.4f) < .001f);
    BSP_CAN_Diagnostics_t diag;
    assert(BSP_CAN_GetDiagnostics(&hfdcan2, &diag) == BSP_CAN_OK);
    assert(diag.first_valid && diag.bus_off_valid && diag.bus_off.tx_pending_mask == 0xF0U);
    task_steps(&now, 1U, 1U); /* Fresh enable ACKs still required after automatic start. */
    assert(task_status().state == (hold ? MOTOR_TASK_STOPPING : MOTOR_TASK_RUNNING));
    if (motion) {
        task_steps(&now, 500U, 1U);
        assert(task_status().state == MOTOR_TASK_HOLDING);
        assert(task_status().mode == MOTOR_MODE_MOTION && task_status().motion.valid);
        assert(fabsf(task_status().target_reference_rad[0] + 2.6f) < .001f);
        assert(task_status().target_velocity_rad_s[0] == 0.0f);
        assert(task_status().motion.duration_s == 0.0f); /* No old-job replay. */
    }
    if (hold) {
        task_steps(&now, 220U, 1U);
        assert(task_status().state == MOTOR_TASK_HOLDING);
        assert(fabsf(task_status().target_reference_rad[1] + 2.6f) < .001f);
    }
    if (limit) {
        for (unsigned n = 0U; n < 2U; ++n) {
            trigger_bus_off(&now);
            can_protocol[1].BusOff = 0U;
            task_steps(&now, 21U, 0U);
            task_steps(&now, 22U, 0U);
            assert(task_status().state == MOTOR_TASK_ARMING);
            task_steps(&now, 1U, 1U);
        }
        const unsigned starts = can_start_calls, enables = motor_enable_calls[1];
        can_protocol[1].BusOff = 1U;
        hfdcan2.Instance->CCCR = FDCAN_CCCR_INIT;
        task_steps(&now, 1U, 1U);
        assert(task_status().state == MOTOR_TASK_FAULT);
        task_steps(&now, 100U, 0U);
        assert(can_start_calls == starts && motor_enable_calls[1] == enables);
    }
}

static void test_task(const char *name)
{
    if (strcmp(name, "task-grip") == 0) { test_grip_geometry(); return; }
    if (strncmp(name, "task-recovery", 13U) == 0) { test_recovery(name); return; }
    if (strcmp(name, "task-clamp") == 0) { test_task_clamp(); return; }
    if (strcmp(name, "task-model") == 0) { test_model(); return; }
    if (strcmp(name, "task-drag-polarity") == 0) { test_drag_polarity(); return; }
    if (strcmp(name, "task-drag-dynamics") == 0) { test_drag_dynamics(); return; }
    if (strcmp(name, "task-coordinates") == 0) { test_coordinates(false); return; }
    if (strcmp(name, "task-coordinate-hold") == 0) { test_coordinates(true); return; }
    uint32_t now = UINT32_MAX - 100U; /* Exercise unsigned tick wrap. */
    const bool locked = strcmp(name, "task-locked") == 0;
    setup_task(!locked, now);
    task_steps(&now, 1U, 0U);
    assert(task_status().drag.valid);
    assert(motor_mit_calls[1] == 0U && motor_enable_calls[1] == 0U);
    if (strcmp(name, "task-motion") == 0) { test_motion(now); return; }
    if (strcmp(name, "task-impedance") == 0) { test_impedance(now); return; }
    if (strcmp(name, "task-can-schedule") == 0) {
        task_steps(&now, 19U, 0U);
        assert(motor_query_calls[1] == 10U && motor_query_calls[2] == 10U);
        assert(can_protocol_reads == 0U && motor_mit_calls[1] == 0U);
        /* Include the enable handshake: no separate query is needed there. */
        start_task(&now, false);
        task_steps(&now, 220U, 1U);
        assert(task_status().state == MOTOR_TASK_RUNNING);
        assert(MotorTask_Command(MOTOR_COMMAND_STOP, now) == BSP_CAN_OK);
        task_steps(&now, 220U, 1U);
        assert(task_status().state == MOTOR_TASK_HOLDING);
        assert(MotorTask_Command(MOTOR_COMMAND_START, now) == BSP_CAN_OK);
        task_steps(&now, 220U, 1U);
        assert(task_status().state == MOTOR_TASK_RUNNING);
        assert(motor_query_calls[1] == 10U && motor_query_calls[2] == 10U);
        assert(motor_mit_calls[1] == motor_mit_calls[2] && motor_mit_calls[1] > 660U);
        assert(can_protocol_reads == 0U && motor_enable_calls[1] == 1U);
        /* Missing feedback still faults; no fallback poll or automatic enable. */
        for (unsigned n = 0U; n < 22U; ++n) { now += 5U; task_run(now); }
        assert_task_fault(MOTOR_REASON_STALE, now, true);
        const unsigned queries = motor_query_calls[1];
        task_steps(&now, 2U, 0U);
        assert(motor_query_calls[1] > queries && motor_query_calls[2] > queries);
        assert(motor_enable_calls[1] == 1U && motor_enable_calls[2] == 1U);
        assert(MotorTask_Command(MOTOR_COMMAND_CLEAR, now) == BSP_CAN_OK);
        task_steps(&now, 2U, 0U);
        assert(task_status().state == MOTOR_TASK_IDLE);
        assert(motor_enable_calls[1] == 1U && motor_enable_calls[2] == 1U);
        return;
    }
    if (locked) {
        assert(MotorTask_Command(MOTOR_COMMAND_START, now) == BSP_CAN_ERROR);
        assert(!task_status().policy_ready && task_status().reason == MOTOR_REASON_POLICY);
        assert(motor_mit_calls[1] == 0U && motor_enable_calls[1] == 0U);
        return;
    }
    if (strcmp(name, "task-tuning") == 0) { test_tuning(now); return; }
    if (strcmp(name, "task-start-still") == 0) {
        for (unsigned joint = 0U; joint < 2U; ++joint) {
            feedback_pair(now, 0U);
            task_feedback(joint, -2.6f, joint == 0U ? .3f : -.3f, 0U);
            assert(MotorTask_Command(MOTOR_COMMAND_START, now) == BSP_CAN_ERROR);
            assert(task_status().state == MOTOR_TASK_IDLE);
            assert(task_status().reason == MOTOR_REASON_NOT_STILL);
            assert(motor_enable_calls[1] == 0U && motor_enable_calls[2] == 0U);
            assert(motor_mit_calls[1] == 0U && motor_mit_calls[2] == 0U);
        }
        feedback_pair(now, 0U);
        start_task(&now, false); /* Explicit new start works after becoming still. */
        return;
    }
    if (strcmp(name, "task-bounds") == 0) {
        task_feedback(0, .1f, 0, 0);
        assert(MotorTask_Command(MOTOR_COMMAND_START, now) == BSP_CAN_ERROR);
        assert(task_status().reason == MOTOR_REASON_ANGLE && motor_enable_calls[1] == 0U);
        return;
    }
    if (strcmp(name, "task-arming") == 0) {
        assert(MotorTask_Command(MOTOR_COMMAND_START, now) == BSP_CAN_OK);
        task_steps(&now, 101U, 0U);
        assert_task_fault(MOTOR_REASON_ENABLE_TIMEOUT, now, true);
        return;
    }
    if (strcmp(name, "task-stop-arming") == 0) {
        assert(MotorTask_Command(MOTOR_COMMAND_START, now) == BSP_CAN_OK);
        assert(MotorTask_Command(MOTOR_COMMAND_STOP, now) == BSP_CAN_OK);
        task_steps(&now, 220U, 1U);
        assert(task_status().state == MOTOR_TASK_HOLDING && task_status().target_kp[1] == 5.0f);
        return;
    }
    start_task(&now, strcmp(name, "task-drag") == 0);
    if (strcmp(name, "task-sample-clock") == 0) {
        /* A CAN IRQ can run after the caller took now_ms, before snapshot.
         * Fresh feedback must not turn into an unsigned ~49-day old sample. */
        for (unsigned n = 0U; n < 250U; ++n) {
            now += 5U;
            feedback_pair(now + 1U, 1U);
            assert(MotorTask_Run(now) == BSP_CAN_OK);
            assert(task_status().state == MOTOR_TASK_RUNNING);
            assert(task_status().reason == MOTOR_REASON_NONE);
        }
        /* Genuine silence must still fail at the original age threshold. */
        now += g_motor_config.feedback_max_age_ms + 6U;
        test_tick = now;
        (void)MotorTask_Run(now);
        assert(task_status().state == MOTOR_TASK_FAULT);
        return;
    } else if (strcmp(name, "task-stale") == 0) {
        for (unsigned i = 0U; i < 22U; ++i) { now += 5U; task_run(now); }
        assert_task_fault(MOTOR_REASON_STALE, now, true);
    } else if (strcmp(name, "task-transport") == 0) {
        can_io[1].free_slots = 0U;
        task_steps(&now, 1U, 1U);
        assert_task_fault(MOTOR_REASON_TRANSPORT, now, false);
        assert(task_status().disable_status[0] == BSP_CAN_BUSY);
    } else if (strcmp(name, "task-timing") == 0) {
        now += 21U;
        feedback_pair(now, 1);
        task_run(now);
        assert_task_fault(MOTOR_REASON_TIMING, now, true);
    } else if (strcmp(name, "task-partial") == 0) {
        can_fail_mit_id = 2U;
        const unsigned j1 = motor_mit_calls[1], j2 = motor_mit_calls[2];
        task_steps(&now, 1U, 1U);
        assert_task_fault(MOTOR_REASON_TRANSPORT, now, true);
        assert(motor_mit_calls[1] == j1+1U && motor_mit_calls[2] == j2);
    } else if (strcmp(name, "task-temperature") == 0) {
        MotorTask_Config_t cfg;
        assert(MotorTask_GetConfig(&cfg) == BSP_CAN_OK);
        /* Fixture reports 25/26 C: modify the callback's actual feedback bytes. */
        now += 5U;
        feedback_pair(now, 1);
        uint8_t data[8] = {0x11U, 0x65U, 0x60U, 0x7FU, 0xF7U, 0xFFU,
                           cfg.joint[0].temperature_c, 26U};
        inject_motor_feedback(&hfdcan2, 0x11U, data, 8U);
        task_run(now);
        assert_task_fault(MOTOR_REASON_TEMPERATURE, now, true);
    } else if (strcmp(name, "task-state") == 0) {
        now += 5U;
        feedback_pair(now, 1);
        task_feedback(1, -2.6f, 0, 8);
        task_run(now);
        assert_task_fault(MOTOR_REASON_STATE, now, true);
    } else if (strcmp(name, "task-drag-speed") == 0) {
        float position[2] = {-2.6f, -2.6f};
        const unsigned initial_mit[2] = {motor_mit_calls[1], motor_mit_calls[2]};
        /* Both directions, both axes, >20 deg/s throughout the support transition
         * and free drag. Speeds still raise Kd instead of submitting disable. */
        for (unsigned n = 0U; n < 300U; ++n) {
            now += 5U;
            test_tick = now;
            const float velocity = n < 150U ? .6f : -.6f;
            position[0] += velocity * .005f;
            position[1] -= velocity * .005f;
            task_feedback(0, position[0], velocity, 1U);
            task_feedback(1, position[1], -velocity, 1U);
            task_run(now);
            const MotorTask_Status_t s = task_status();
            assert(s.state == MOTOR_TASK_RUNNING && s.reason == MOTOR_REASON_NONE);
            assert(s.target_kp[0] == 0.0f && s.target_kp[1] == 0.0f);
            assert(motor_disable_calls[1] == 0U && motor_disable_calls[2] == 0U);
            if (n == 149U || n == 299U) {
                for (unsigned i = 0U; i < 2U; ++i) {
                    assert(s.target_kd[i] >= 0.0f);
                    assert(s.target_kd[i] < g_motor_config.joint[i].drag_kd_high);
                }
            }
        }
        assert(motor_mit_calls[1] == initial_mit[0] + 300U);
        assert(motor_mit_calls[2] == initial_mit[1] + 300U);
        const MotorTask_Status_t moving = task_status();
        assert(fabsf(moving.drag.gravity_nm[1]) > .01f);
        assert(fabsf(moving.target_torque_ff_nm[1] - moving.drag.dynamic_ff_nm[1] -
            moving.drag.grip_ff_nm[1] - moving.drag.gravity_nm[1]) < 1e-5f);
        /* Normal stop/hold must not reintroduce a runtime speed trip either. */
        assert(MotorTask_Command(MOTOR_COMMAND_STOP, now) == BSP_CAN_OK);
        for (unsigned n = 0U; n < 5U; ++n) {
            now += 5U;
            feedback_pair(now, 1U);
            task_feedback(0, position[0], -.3f, 1U);
            task_feedback(1, position[1], .3f, 1U);
            task_run(now);
            assert(task_status().state == MOTOR_TASK_STOPPING);
            assert(task_status().reason == MOTOR_REASON_NONE);
        }
        task_steps(&now, 220U, 1U);
        assert(task_status().state == MOTOR_TASK_HOLDING);
        now += 5U;
        feedback_pair(now, 1U);
        task_feedback(0, position[0], .3f, 1U);
        task_feedback(1, position[1], -.3f, 1U);
        task_run(now);
        assert(task_status().state == MOTOR_TASK_HOLDING);
        assert(motor_disable_calls[1] == 0U && motor_disable_calls[2] == 0U);
        assert(MotorTask_Command(MOTOR_COMMAND_DISABLE, now) == BSP_CAN_OK);
        assert_task_fault(MOTOR_REASON_MANUAL_DISABLE, now, true);
    } else if (strcmp(name, "task-tracking") == 0) {
        assert(MotorTask_Command(MOTOR_COMMAND_STOP, now) == BSP_CAN_OK);
        now += 5U;
        feedback_pair(now, 1);
        task_feedback(0, -2.4f, 0, 1);
        task_run(now);
        assert_task_fault(MOTOR_REASON_TRACKING, now, true);
    } else {
        assert(strcmp(name, "task-drag") == 0);
        task_steps(&now, 220U, 1);
        const float ff = task_status().target_torque_ff_nm[1];
        assert(fabsf(ff) > .01f && task_status().target_kp[1] == 0.0f);
        now += 5U;
        feedback_pair(now, 1);
        task_feedback(0, -2.3f, .05f, 1); /* Free drag ignores old tracking target. */
        task_feedback(1, -2.0f, .05f, 1);
        task_run(now);
        assert(task_status().state == MOTOR_TASK_RUNNING && task_status().target_kp[0] == 0.0f);
        assert(task_status().target_torque_ff_nm[0] == task_status().drag.dynamic_ff_nm[0] +
            task_status().drag.grip_ff_nm[0]);
        task_steps(&now, 220U, 1);
        /* KEY pressed/released uses the same Mode stop/continue lifecycle. */
        test_gpioa.IDR = 0U;
        task_steps(&now, 6U, 1);
        test_gpioa.IDR = KEY__INPUT_Pin;
        task_steps(&now, 220U, 1);
        assert(task_status().state == MOTOR_TASK_HOLDING);
        assert(task_status().target_kp[0] == 5.0f);
        assert(fabsf(task_status().target_torque_ff_nm[1] - ff) < 1e-5f);
        test_gpioa.IDR = 0U;
        task_steps(&now, 6U, 1);
        test_gpioa.IDR = KEY__INPUT_Pin;
        task_steps(&now, 220U, 1);
        assert(task_status().state == MOTOR_TASK_RUNNING && task_status().target_kp[0] == 0.0f);
        assert(motor_enable_calls[1] == 1U && motor_disable_calls[1] == 0U);
        assert(MotorTask_Command(MOTOR_COMMAND_DISABLE, now) == BSP_CAN_OK);
        assert_task_fault(MOTOR_REASON_MANUAL_DISABLE, now, true);
        now += 5U;
        feedback_pair(now, 0);
        const unsigned enables = motor_enable_calls[1];
        assert(MotorTask_Command(MOTOR_COMMAND_CLEAR, now) == BSP_CAN_OK);
        assert(task_status().state == MOTOR_TASK_IDLE && motor_enable_calls[1] == enables);
    }
    test_ipsr = 1U;
    assert(MotorTask_Run(now) == BSP_CAN_INVALID_ARG);
    test_ipsr = 0U;
    assert(gpio_high_writes == 0U);
}
