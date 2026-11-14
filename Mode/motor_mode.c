#include "motor_mode.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

bool MotorMode_Available(MotorMode_Id_t id)
{
    return id == MOTOR_MODE_DRAG || id == MOTOR_MODE_MOTION;
}

bool MotorMode_ValidConfig(MotorMode_Id_t id, const MotorTask_Config_t *config)
{
    return id == MOTOR_MODE_MOTION ? MotionMode_ValidConfig(config) :
        id == MOTOR_MODE_DRAG && DragMode_ValidConfig(config);
}

void MotorMode_Reset(MotorMode_State_t *state)
{
    if (state == NULL) { return; }
    if (state->id == MOTOR_MODE_MOTION) { MotionMode_Reset(&state->motion); }
    else { DragMode_Reset(&state->drag); }
}

MotorMode_Phase_t MotorMode_Phase(const MotorMode_State_t *state)
{
    if (state == NULL || !MotorMode_Available(state->id)) { return MOTOR_MODE_OFF; }
    if (state->id == MOTOR_MODE_MOTION) {
        switch (state->motion.phase) {
        case MOTION_MODE_ENTERING: return MOTOR_MODE_ENTERING;
        case MOTION_MODE_MOVING: return MOTOR_MODE_RUNNING;
        case MOTION_MODE_STOPPING: return MOTOR_MODE_STOPPING;
        case MOTION_MODE_HOLDING: return MOTOR_MODE_HOLDING;
        default: return MOTOR_MODE_OFF;
        }
    }
    switch (state->drag.phase) {
    case DRAG_MODE_ENTERING: return MOTOR_MODE_ENTERING;
    case DRAG_MODE_DRAGGING: return MOTOR_MODE_RUNNING;
    case DRAG_MODE_STOPPING: return MOTOR_MODE_STOPPING;
    case DRAG_MODE_HOLDING: return MOTOR_MODE_HOLDING;
    case DRAG_MODE_RESUMING: return MOTOR_MODE_RESUMING;
    default: return MOTOR_MODE_OFF;
    }
}

bool MotorMode_TracksPosition(const MotorMode_State_t *state)
{
    const MotorMode_Phase_t phase = MotorMode_Phase(state);
    /* Compliance intentionally permits position error, unlike Drag hold.
     * Existing range/device/temperature/communication checks remain in Task. */
    return state != NULL && state->id == MOTOR_MODE_DRAG &&
        (phase == MOTOR_MODE_STOPPING || phase == MOTOR_MODE_HOLDING);
}

bool MotorMode_Enter(const MotorTask_Config_t *config,
                     const DM_Motor_Feedback_t feedback[2], MotorMode_State_t *state)
{
    if (state == NULL) { return false; }
    return state->id == MOTOR_MODE_MOTION ?
        MotionMode_Enter(config, feedback, &state->motion) :
        state->id == MOTOR_MODE_DRAG && DragMode_Enter(config, feedback, &state->drag);
}

bool MotorMode_CaptureHold(const MotorTask_Config_t *config,
                           const DM_Motor_Feedback_t feedback[2],
                           const DM_Motor_MIT_Command_t previous[2],
                           MotorMode_State_t *state)
{
    if (state == NULL || !MotionMode_ValidConfig(config)) { return false; }
    MotionMode_State_t next = {0};
    if (previous != NULL) {
        for (unsigned i = 0U; i < 2U; ++i) {
            if (!isfinite(previous[i].kd) || previous[i].kd < 0.0f ||
                previous[i].kd > 5.0f || !isfinite(previous[i].torque_ff_nm) ||
                fabsf(previous[i].torque_ff_nm) > config->joint[i].device.tmax) {
                return false;
            }
        }
    }
    if (!MotionMode_Enter(config, feedback, &next)) { return false; }
    if (previous != NULL) {
        for (unsigned i = 0U; i < 2U; ++i) {
            next.entry_kd[i] = previous[i].kd;
            next.entry_ff_nm[i] = previous[i].torque_ff_nm;
        }
    }
    /* Capturing a new center intentionally replaces any old reference.
     * Ramp its spring from zero; do not inherit Drag's hold stiffness. */
    state->id = MOTOR_MODE_MOTION;
    state->motion = next;
    return true;
}

bool MotorMode_Stop(const MotorTask_Config_t *config,
                    const DM_Motor_Feedback_t feedback[2], MotorMode_State_t *state,
                    float captured_target[2])
{
    if (state == NULL || captured_target == NULL) { return false; }
    if (state->id == MOTOR_MODE_MOTION) {
        if (!MotionMode_Stop(config, &state->motion)) { return false; }
        for (unsigned i = 0U; i < 2U; ++i) {
            captured_target[i] = (float)state->motion.reference.position[i];
        }
        return true;
    }
    if (state->id != MOTOR_MODE_DRAG ||
        !DragMode_Stop(config, feedback, &state->drag)) { return false; }
    memcpy(captured_target, state->drag.hold_target_rad,
           sizeof(state->drag.hold_target_rad));
    return true;
}

bool MotorMode_Resume(MotorMode_State_t *state)
{
    if (state == NULL) { return false; }
    /* Starting a held Motion never replays the previous job. */
    return state->id == MOTOR_MODE_MOTION ?
        state->motion.phase == MOTION_MODE_HOLDING :
        state->id == MOTOR_MODE_DRAG && DragMode_Resume(&state->drag);
}

bool MotorMode_Move(const MotorTask_Config_t *config,
                    const MotionMode_Request_t *request, MotorMode_State_t *state)
{
    return state != NULL && state->id == MOTOR_MODE_MOTION &&
        MotionMode_Move(config, request, &state->motion);
}

bool MotorMode_Update(const MotorTask_Config_t *config,
                      const DM_Motor_Feedback_t feedback[2], float dt_s,
                      MotorMode_State_t *state, MotorMode_Diagnostics_t *diagnostics,
                      DM_Motor_MIT_Command_t commands[2])
{
    if (state == NULL || diagnostics == NULL || !MotorMode_Available(state->id)) {
        return false;
    }
    memset(diagnostics, 0, sizeof(*diagnostics));
    return state->id == MOTOR_MODE_MOTION ?
        MotionMode_Update(config, feedback, dt_s, &state->motion,
                          &diagnostics->motion, commands) :
        DragMode_Update(config, feedback, dt_s, &state->drag, &diagnostics->drag, commands);
}
