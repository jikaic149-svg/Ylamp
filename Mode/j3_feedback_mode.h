#ifndef YLAMP_J3_FEEDBACK_MODE_H
#define YLAMP_J3_FEEDBACK_MODE_H

#include "motor_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    J3_FEEDBACK_OFF, J3_FEEDBACK_PREPARING, J3_FEEDBACK_PRELOADING,
    J3_FEEDBACK_ENABLING, J3_FEEDBACK_STREAMING, J3_FEEDBACK_STOPPING,
    J3_FEEDBACK_UNCONFIRMED
} J3Feedback_Phase_t;
typedef enum {
    J3_FEEDBACK_OK, J3_FEEDBACK_STALE, J3_FEEDBACK_STATE,
    J3_FEEDBACK_TEMPERATURE, J3_FEEDBACK_TRANSPORT,
    J3_FEEDBACK_ENABLE_TIMEOUT, J3_FEEDBACK_DISABLE_TIMEOUT
} J3Feedback_Reason_t;
typedef enum {
    J3_ACTION_NONE, J3_ACTION_DISABLE, J3_ACTION_MIT, J3_ACTION_ENABLE
} J3Feedback_Action_t;

typedef struct {
    J3Feedback_Phase_t phase;
    J3Feedback_Reason_t reason;
    uint32_t phase_ms, last_mit_ms, baseline;
    bool submitted;
} J3FeedbackMode_t;

/* Pure lifecycle, no hardware calls. One USB task owns state and submission.
 * Boot OFF. Explicit start: fresh disable -> neutral preload -> enable ->
 * neutral MIT at config.query_period_ms. Stop: one disable + NEW state=0.
 * Uses the existing bench age/ack/temperature limits (joint[0] temperature).
 * No holding, recovery/re-enable, catch-up or implicit retry. */
BSP_CAN_Status_t J3FeedbackMode_Start(J3FeedbackMode_t *mode);
void J3FeedbackMode_Stop(J3FeedbackMode_t *mode);
/* feedback=NULL when never received; now_ms sampled AFTER acquiring feedback. */
J3Feedback_Action_t J3FeedbackMode_Update(J3FeedbackMode_t *mode,
    const MotorTask_Config_t *config, const DM_Motor_Feedback_t *feedback,
    uint32_t now_ms);
/* Call exactly once for the returned action, with the same input snapshot.
 * Accepted CAN submission is NOT an enable/disable confirmation. */
void J3FeedbackMode_Submitted(J3FeedbackMode_t *mode, J3Feedback_Action_t action,
    BSP_CAN_Status_t result, uint32_t feedback_count, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
#endif
