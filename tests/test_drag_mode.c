#include "drag_mode.h"
#include "motor_mode.h"

#include <assert.h>
#include <math.h>
#include <string.h>

/* Only the config's bus identity is needed; no HAL/Device/Task functions link. */
FDCAN_HandleTypeDef hfdcan1, hfdcan2;

static void step(const MotorTask_Config_t *config, DM_Motor_Feedback_t fb[2],
    DragMode_State_t *mode, DragMode_Status_t *status, DM_Motor_MIT_Command_t cmd[2])
{
    for (unsigned i = 0U; i < 2U; ++i) {
        fb[i].received_at_ms += 5U;
        ++fb[i].received_count;
    }
    assert(DragMode_Update(config, fb, .005f, mode, status, cmd));
}

int main(void)
{
    const MotorTask_Config_t config = g_motor_config;
    DragMode_State_t mode = {0}, other = {0};
    DragMode_Status_t status;
    DM_Motor_MIT_Command_t cmd[2];
    DM_Motor_Feedback_t fb[2] = {
        {.position_rad = -2.0f},
        {.position_rad = config.drag_j2_zero_rad + .4f}
    };

    /* OFF previews the model but never produces position stiffness or FF. */
    step(&config, fb, &mode, &status, cmd);
    assert(mode.phase == DRAG_MODE_OFF && status.valid);
    for (unsigned i = 0U; i < 2U; ++i) {
        assert(cmd[i].kp == 0.0f && cmd[i].torque_ff_nm == 0.0f);
    }
    assert(DragMode_Enter(&config, fb, &mode));
    assert(!DragMode_Enter(&config, fb, &mode));
    for (unsigned n = 0U; n < 220U; ++n) {
        step(&config, fb, &mode, &status, cmd);
    }
    assert(mode.phase == DRAG_MODE_DRAGGING);
    assert(cmd[0].kp == 0.0f && cmd[1].kp == 0.0f);
    assert(cmd[0].velocity_rad_s == 0.0f && cmd[1].velocity_rad_s == 0.0f);
    assert(cmd[0].torque_ff_nm == 0.0f);
    assert(fabsf(cmd[1].torque_ff_nm -
        config.drag_gravity_scale * g_drag_model.gravity_sin_nm *
        sinf(fb[1].position_rad - config.drag_j2_zero_rad)) < 1e-6f);

    /* Stop captures once; repeated Stop must not walk the hold target. */
    assert(DragMode_Stop(&config, fb, &mode));
    float target[2];
    memcpy(target, mode.hold_target_rad, sizeof(target));
    fb[0].position_rad += .01f;
    fb[1].position_rad += .01f;
    assert(DragMode_Stop(&config, fb, &mode));
    assert(memcmp(target, mode.hold_target_rad, sizeof(target)) == 0);
    for (unsigned n = 0U; n < 220U; ++n) {
        step(&config, fb, &mode, &status, cmd);
        for (unsigned i = 0U; i < 2U; ++i) {
            assert(cmd[i].position_rad == target[i]);
        }
    }
    assert(mode.phase == DRAG_MODE_HOLDING);
    for (unsigned i = 0U; i < 2U; ++i) {
        assert(cmd[i].kp == config.joint[i].hold_kp);
        assert(cmd[i].kd == config.joint[i].hold_kd);
    }

    /* Resume releases Kp at the captured target, then follows the actual pose. */
    assert(DragMode_Resume(&mode));
    assert(!DragMode_Resume(&mode));
    step(&config, fb, &mode, &status, cmd);
    assert(cmd[1].kp > 0.0f && cmd[1].position_rad == target[1]);
    for (unsigned n = 0U; n < 220U; ++n) {
        step(&config, fb, &mode, &status, cmd);
    }
    assert(mode.phase == DRAG_MODE_DRAGGING);
    assert(cmd[1].kp == 0.0f && cmd[1].position_rad == fb[1].position_rad);

    /* Rejected updates are atomic: no partial state or command commits. */
    const DragMode_State_t saved_mode = mode;
    const DragMode_Status_t saved_status = status;
    DM_Motor_MIT_Command_t saved_cmd[2];
    memcpy(saved_cmd, cmd, sizeof(saved_cmd));
    fb[1].velocity_rad_s = NAN;
    assert(!DragMode_Update(&config, fb, .005f, &mode, &status, cmd));
    assert(memcmp(&saved_mode, &mode, sizeof(mode)) == 0);
    assert(memcmp(&saved_status, &status, sizeof(status)) == 0);
    assert(memcmp(saved_cmd, cmd, sizeof(cmd)) == 0);
    fb[1].velocity_rad_s = 0.0f;

    /* An independent OFF instance cannot affect the active one. */
    step(&config, fb, &other, &status, cmd);
    assert(other.phase == DRAG_MODE_OFF && cmd[1].torque_ff_nm == 0.0f);
    assert(memcmp(&saved_mode, &mode, sizeof(mode)) == 0);
    DragMode_Reset(&mode);
    assert(mode.phase == DRAG_MODE_OFF && !mode.initialized);
    assert(!DragMode_Stop(&config, fb, &mode) && !DragMode_Resume(&mode));
    MotorMode_State_t selected = {.id = MOTOR_MODE_DRAG};
    float captured[2];
    assert(MotorMode_ValidConfig(MOTOR_MODE_DRAG, &config));
    assert(MotorMode_Available(MOTOR_MODE_MOTION));
    assert(MotorMode_Phase(&selected) == MOTOR_MODE_OFF);
    assert(!MotorMode_TracksPosition(&selected));
    assert(MotorMode_Enter(&config, fb, &selected));
    assert(MotorMode_Phase(&selected) == MOTOR_MODE_ENTERING);
    assert(MotorMode_Stop(&config, fb, &selected, captured));
    assert(MotorMode_TracksPosition(&selected));
    assert(captured[0] == fb[0].position_rad && captured[1] == fb[1].position_rad);
    MotorMode_Reset(&selected);
    selected.id = MOTOR_MODE_MOTION;
    MotorMode_Diagnostics_t diagnostics;
    assert(MotorMode_Enter(&config, fb, &selected));
    assert(MotorMode_Update(&config, fb, .005f, &selected, &diagnostics, cmd));
    assert(MotorMode_Phase(&selected) == MOTOR_MODE_ENTERING);
    assert(!MotorMode_TracksPosition(&selected));
    assert(!diagnostics.drag.valid && diagnostics.motion.valid);
    return 0;
}
