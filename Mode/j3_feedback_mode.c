#include "j3_feedback_mode.h"

#include <math.h>

BSP_CAN_Status_t J3FeedbackMode_Start(J3FeedbackMode_t *mode)
{
    if (mode->phase != J3_FEEDBACK_OFF) { return BSP_CAN_BUSY; }
    *mode = (J3FeedbackMode_t){.phase = J3_FEEDBACK_PREPARING};
    return BSP_CAN_OK;
}

void J3FeedbackMode_Stop(J3FeedbackMode_t *mode)
{
    if (mode->phase == J3_FEEDBACK_STOPPING) { return; }
    mode->phase = J3_FEEDBACK_STOPPING;
    mode->submitted = false;
}

static J3Feedback_Action_t stop(J3FeedbackMode_t *mode, J3Feedback_Reason_t reason)
{
    mode->reason = reason;
    J3FeedbackMode_Stop(mode);
    return J3_ACTION_DISABLE;
}

J3Feedback_Action_t J3FeedbackMode_Update(J3FeedbackMode_t *mode,
    const MotorTask_Config_t *config, const DM_Motor_Feedback_t *feedback,
    uint32_t now_ms)
{
    if (mode->phase == J3_FEEDBACK_OFF ||
        mode->phase == J3_FEEDBACK_UNCONFIRMED) { return J3_ACTION_NONE; }
    const bool fresh = feedback != NULL && isfinite(feedback->position_rad) &&
        now_ms - feedback->received_at_ms <= config->feedback_max_age_ms;
    const bool changed = fresh && feedback->received_count != mode->baseline;
    const bool hot = fresh && (feedback->mos_temperature_c >= config->joint[0].temperature_c ||
        feedback->rotor_temperature_c >= config->joint[0].temperature_c);
    if (mode->phase == J3_FEEDBACK_PREPARING || mode->phase == J3_FEEDBACK_STOPPING) {
        if (!mode->submitted) { return J3_ACTION_DISABLE; }
        if (changed && feedback->state == 0U) {
            if (mode->phase == J3_FEEDBACK_STOPPING || hot) {
                mode->phase = J3_FEEDBACK_OFF;
                if (hot) { mode->reason = J3_FEEDBACK_TEMPERATURE; }
                return J3_ACTION_NONE;
            }
            mode->phase = J3_FEEDBACK_PRELOADING;
            mode->submitted = false;
            return J3_ACTION_MIT;
        }
        if (now_ms - mode->phase_ms >= config->enable_timeout_ms) {
            mode->phase = J3_FEEDBACK_UNCONFIRMED;
            mode->reason = J3_FEEDBACK_DISABLE_TIMEOUT;
        }
        return J3_ACTION_NONE;
    }
    if (hot) { return stop(mode, J3_FEEDBACK_TEMPERATURE); }
    if (mode->phase == J3_FEEDBACK_ENABLING) {
        if (!fresh) { return stop(mode, J3_FEEDBACK_STALE); }
        if (feedback->state > 1U) { return stop(mode, J3_FEEDBACK_STATE); }
        if (!mode->submitted) { return J3_ACTION_ENABLE; }
        if (changed && feedback->state == 1U) {
            mode->phase = J3_FEEDBACK_STREAMING;
        } else if (changed && feedback->state > 1U) {
            return stop(mode, J3_FEEDBACK_STATE);
        } else if (now_ms - mode->phase_ms >= config->enable_timeout_ms) {
            return stop(mode, J3_FEEDBACK_ENABLE_TIMEOUT);
        }
    }
    if (mode->phase == J3_FEEDBACK_STREAMING) {
        if (!fresh) { return stop(mode, J3_FEEDBACK_STALE); }
        if (feedback->state != 1U) { return stop(mode, J3_FEEDBACK_STATE); }
    }
    if (now_ms - mode->last_mit_ms >= config->query_period_ms) {
        return J3_ACTION_MIT;
    }
    return J3_ACTION_NONE;
}

void J3FeedbackMode_Submitted(J3FeedbackMode_t *mode, J3Feedback_Action_t action,
    BSP_CAN_Status_t result, uint32_t feedback_count, uint32_t now_ms)
{
    if (result != BSP_CAN_OK) {
        mode->reason = J3_FEEDBACK_TRANSPORT;
        if (action == J3_ACTION_DISABLE) {
            mode->phase = J3_FEEDBACK_UNCONFIRMED;
        } else if (mode->phase == J3_FEEDBACK_PRELOADING) {
            mode->phase = J3_FEEDBACK_OFF; /* Already confirmed disabled. */
        } else { J3FeedbackMode_Stop(mode); }
        return;
    }
    if (action == J3_ACTION_MIT) {
        mode->last_mit_ms = now_ms;
        if (mode->phase == J3_FEEDBACK_PRELOADING) {
            mode->phase = J3_FEEDBACK_ENABLING;
            mode->submitted = false;
        }
    } else {
        mode->submitted = true;
        mode->baseline = feedback_count;
        mode->phase_ms = now_ms;
        if (action == J3_ACTION_ENABLE) { mode->last_mit_ms = now_ms; }
    }
}
