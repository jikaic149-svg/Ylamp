#ifndef YLAMP_CAN_CHECK_H
#define YLAMP_CAN_CHECK_H

#include "motor_config.h"

typedef enum {
    CAN_CHECK_IDLE, CAN_CHECK_QUERY, CAN_CHECK_ENABLING,
    CAN_CHECK_DISABLING, CAN_CHECK_DONE, CAN_CHECK_STOPPED, CAN_CHECK_SCANNING,
    CAN_CHECK_PREPARING, CAN_CHECK_STREAMING
} CanCheck_Phase_t;

typedef enum {
    CAN_CHECK_OK, CAN_CHECK_NO_REPLY, CAN_CHECK_STATE,
    CAN_CHECK_TEMPERATURE, CAN_CHECK_TRANSPORT,
    CAN_CHECK_ENABLE_TIMEOUT, CAN_CHECK_DISABLE_TIMEOUT,
    CAN_CHECK_QUERY_UNSUPPORTED
} CanCheck_Reason_t;

typedef struct {
    uint32_t query_submitted, query_failed, feedback_start_count, replies;
    uint32_t enable_submitted, enable_confirmed;
    uint32_t disable_submitted, disable_confirmed;
    uint32_t reason, last_transport, feedback_valid;
    DM_Motor_Feedback_t feedback;
    float minimum_rad, maximum_rad;
    BSP_CAN_Diagnostics_t can;
    uint32_t mit_submitted, mit_failed, stream_replies, stream_elapsed_ms;
} CanCheck_Result_t;

/* SWD-only single-writer mailbox: 1=enable check from IDLE only;
 * 2=read-only queries from IDLE or DONE (no MIT/enable/disable).
 * 3=read-only address discovery, CAN1 then CAN2, IDs1..15, CC + Master ID read.
 * 4=read-only J3 query from IDLE or DONE; no other motor queried, completed=1.
 * 5=one legacy J3 disable from IDLE or DONE, then bounded feedback wait;
 *   no query, MIT, enable, other-motor command or automatic retry.
 * 6=explicit J3 legacy zero-gain/zero-FF feedback test: one initial disable
 *   confirmed by NEW feedback, neutral preload, one enable, periodic neutral
 *   MIT at query_period_ms for MOTOR_CAN_CHECK_STREAM_MS, then one disable
 *   confirmed by NEW state=0. No CC/33 or other-motor TX. No auto-rerun.
 *   No holding/trajectory; quantized zero FF is not physically exact zero.
 *   Stream timeout/state/temperature/TX failure ends with the disable attempt.
 * Requests 1/2/4 skip LEGACY_MIT query stages with QUERY_UNSUPPORTED, not
 * NO_REPLY; no implicit disable/neutral command to obtain legacy feedback.
 * Request 3 cannot probe registered legacy addresses; these are not offline.
 * Never boot-enabled. Per motor: bounded query window, zero gains/FF preload,
 * one enable, immediate disable after fresh state=1, confirm fresh state=0.
 * No trajectory/holding, fault clearing, retry, bus reset or automatic rerun.
 * Unconfirmed disable stops the sequence; completion does not imply all passed.
 * Read snapshot only if sequence==sequence_end and even; DONE is frozen.
 * Ordinary application task only, not reentrant; borrows handles for lifetime. */
typedef struct {
    uint32_t sequence, tick_ms, phase, motor_index, completed;
    CanCheck_Result_t motor[MOTOR_CAN_JOINT_COUNT];
    uint32_t sequence_end;
} CanCheck_Status_t;

void CanCheck_Init(DM_Motor_t *const motors[MOTOR_CAN_JOINT_COUNT], uint32_t now_ms);
void CanCheck_Run(uint32_t now_ms);

#endif
