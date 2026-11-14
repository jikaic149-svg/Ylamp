#ifndef YLAMP_J5_TEST_H
#define YLAMP_J5_TEST_H

#include "j5_test_mode.h"

typedef enum {
    J5_TEST_IDLE = 0, J5_TEST_ARMING, J5_TEST_MOVING, J5_TEST_HOLDING,
    J5_TEST_FAULT, J5_TEST_RELEASED
} J5Test_Phase_t;

typedef enum {
    J5_REASON_NONE = 0, J5_REASON_FEEDBACK, J5_REASON_STATE, J5_REASON_TEMP,
    J5_REASON_TRAVEL, J5_REASON_TRANSPORT, J5_REASON_TIMING, J5_REASON_ENABLE,
    J5_REASON_REQUEST
} J5Test_Reason_t;

/* Debugger mailbox, not a wire protocol. One SWD writer, one aligned word,
 * only when request==0: 1=single round trip, 2=release (loses support).
 * Snapshot is stable iff sequence==sequence_end and even. DTCM is coherent
 * with SWD without halting or cache maintenance. Never boot-enabled. */
typedef struct {
    uint32_t sequence, tick_ms, phase, reason, last_request, last_transport;
    uint32_t query_submitted, query_failed, enable_submitted;
    uint32_t mit_submitted, disable_submitted, feedback_valid;
    DM_Motor_Feedback_t feedback;
    float origin_rad, target_rad, minimum_rad, maximum_rad, maximum_error_rad;
    uint32_t feedback_start_count, mode_phase;
    BSP_CAN_Diagnostics_t can;
    uint32_t sequence_end;
} J5Test_Status_t;

void J5Test_Init(DM_Motor_t *motor, uint32_t now_ms);
void J5Test_Run(uint32_t now_ms);

#endif
