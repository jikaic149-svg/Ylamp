#include "j5_test.h"

#include <math.h>

/* Debug-only globals, not shared mutable application interfaces. */
volatile uint32_t g_j5_test_request;
volatile J5Test_Status_t g_j5_test_status;

static DM_Motor_t *test_motor;
static J5Test_Status_t status;
static J5TestMode_State_t mode;
static uint32_t last_run_ms, last_query_ms, phase_start_ms;

static bool accepted(BSP_CAN_Status_t result)
{
    status.last_transport = (uint32_t)result;
    return result == BSP_CAN_OK;
}

static void release_motor(J5Test_Phase_t phase, J5Test_Reason_t reason)
{
    /* Best effort, no hidden retry/recovery. Check fresh state for confirmation. */
    if (accepted(DM_Motor_Disable(test_motor))) { ++status.disable_submitted; }
    status.phase = phase;
    status.reason = reason;
}

static void publish(uint32_t now_ms)
{
    const uint32_t even = status.sequence + 2U;
    g_j5_test_status.sequence = even - 1U;
    status.tick_ms = now_ms;
    (void)BSP_CAN_GetDiagnostics(g_motor_can_config[4].bus, &status.can);
    status.sequence = even - 1U;
    status.sequence_end = even;
    g_j5_test_status = status;
    g_j5_test_status.sequence = even;
    status.sequence = even;
}

void J5Test_Init(DM_Motor_t *motor, uint32_t now_ms)
{
    test_motor = motor;
    status = (J5Test_Status_t){.phase = J5_TEST_IDLE};
    g_j5_test_request = 0U;
    last_run_ms = last_query_ms = phase_start_ms = now_ms;
    publish(now_ms);
}

void J5Test_Run(uint32_t now_ms)
{
    /* The RTOS wakes every 1ms; match the existing 5ms control contract. */
    if (now_ms - last_run_ms < g_motor_config.control_period_ms) { return; }
    DM_Motor_Feedback_t feedback;
    const bool valid = DM_Motor_GetFeedback(test_motor, &feedback) == BSP_CAN_OK;
    now_ms = HAL_GetTick(); /* Feedback IRQ can be newer than the caller tick. */
    const uint32_t dt_ms = now_ms - last_run_ms;
    last_run_ms = now_ms;
    status.feedback_valid = (uint32_t)valid;
    if (valid) { status.feedback = feedback; }
    const bool fresh = valid &&
        now_ms - feedback.received_at_ms <= g_motor_config.feedback_max_age_ms;
    if (now_ms - last_query_ms >= g_motor_config.query_period_ms) {
        last_query_ms = now_ms;
        if (DM_Motor_RequestFeedback(test_motor) == BSP_CAN_OK) {
            ++status.query_submitted;
        } else {
            ++status.query_failed;
        }
    }
    const uint32_t request = g_j5_test_request;
    if (request != 0U) {
        g_j5_test_request = 0U;
        status.last_request = request;
        if (request == 2U) {
            release_motor(J5_TEST_RELEASED, J5_REASON_NONE);
        } else if (request != 1U ||
                   (status.phase != J5_TEST_IDLE && status.phase != J5_TEST_RELEASED)) {
            status.reason = J5_REASON_REQUEST; /* Do not disrupt an active hold. */
        } else if (!fresh) {
            status.reason = J5_REASON_FEEDBACK;
        } else if (feedback.state != 0U ||
                   fabsf(feedback.velocity_rad_s) > g_motor_config.joint[0].start_speed_rad_s) {
            status.reason = J5_REASON_STATE;
        } else if (feedback.mos_temperature_c >= g_motor_config.joint[0].temperature_c ||
                   feedback.rotor_temperature_c >= g_motor_config.joint[0].temperature_c) {
            status.reason = J5_REASON_TEMP;
        } else if (!isfinite(feedback.position_rad) ||
                   fabsf(feedback.position_rad) + g_j5_test_config.travel_limit_rad >=
                       g_motor_can_config[4].pmax) {
            status.reason = J5_REASON_TRAVEL;
        } else {
            status.origin_rad = status.target_rad = feedback.position_rad;
            status.minimum_rad = status.maximum_rad = feedback.position_rad;
            status.maximum_error_rad = 0.0f;
            status.feedback_start_count = feedback.received_count;
            status.reason = J5_REASON_NONE;
            /* Clear the old target before enabling, zero gains/FF while disabled. */
            const DM_Motor_MIT_Command_t idle = {.position_rad = feedback.position_rad};
            if (!accepted(DM_Motor_SendMIT(test_motor, &idle))) {
                release_motor(J5_TEST_FAULT, J5_REASON_TRANSPORT);
            } else {
                ++status.mit_submitted;
                if (!accepted(DM_Motor_Enable(test_motor))) {
                    release_motor(J5_TEST_FAULT, J5_REASON_TRANSPORT);
                } else {
                    ++status.enable_submitted;
                    status.phase = J5_TEST_ARMING;
                    phase_start_ms = now_ms;
                }
            }
        }
    }
    const bool active = status.phase == J5_TEST_ARMING ||
        status.phase == J5_TEST_MOVING || status.phase == J5_TEST_HOLDING;
    if (active) {
        J5Test_Reason_t fault = J5_REASON_NONE;
        if (!fresh) { fault = J5_REASON_FEEDBACK; }
        else if (dt_ms > g_motor_config.max_step_ms) { fault = J5_REASON_TIMING; }
        else if (feedback.mos_temperature_c >= g_motor_config.joint[0].temperature_c ||
                 feedback.rotor_temperature_c >= g_motor_config.joint[0].temperature_c) {
            fault = J5_REASON_TEMP;
        } else if (!isfinite(feedback.position_rad) ||
                   fabsf(feedback.position_rad - status.origin_rad) >
                       g_j5_test_config.travel_limit_rad) {
            fault = J5_REASON_TRAVEL;
        } else if (feedback.state > 1U ||
                   (status.phase != J5_TEST_ARMING && feedback.state != 1U)) {
            fault = J5_REASON_STATE;
        }
        if (fault != J5_REASON_NONE) {
            release_motor(J5_TEST_FAULT, fault);
        } else if (status.phase == J5_TEST_ARMING) {
            if (feedback.state == 1U &&
                feedback.received_count != status.feedback_start_count) {
                J5TestMode_Begin(status.origin_rad, &mode);
                phase_start_ms = now_ms;
                status.phase = J5_TEST_MOVING;
            } else if (now_ms - phase_start_ms >= g_motor_config.enable_timeout_ms) {
                release_motor(J5_TEST_FAULT, J5_REASON_ENABLE);
            }
        } else {
            const uint32_t total_ms = g_j5_test_config.entry_ms +
                2U * g_j5_test_config.leg_ms + g_j5_test_config.settle_ms;
            /* Holding must never replay the move when the HAL tick wraps. */
            const uint32_t elapsed_ms = status.phase == J5_TEST_HOLDING
                ? total_ms : now_ms - phase_start_ms;
            DM_Motor_MIT_Command_t command;
            J5TestMode_Update(&g_j5_test_config, elapsed_ms, &mode, &command);
            status.target_rad = command.position_rad;
            status.mode_phase = (uint32_t)mode.phase;
            if (feedback.position_rad < status.minimum_rad) { status.minimum_rad = feedback.position_rad; }
            if (feedback.position_rad > status.maximum_rad) { status.maximum_rad = feedback.position_rad; }
            const float error = fabsf(command.position_rad - feedback.position_rad);
            if (error > status.maximum_error_rad) { status.maximum_error_rad = error; }
            if (elapsed_ms >= total_ms) {
                status.phase = J5_TEST_HOLDING;
            }
            if (accepted(DM_Motor_SendMIT(test_motor, &command))) {
                ++status.mit_submitted;
            } else {
                release_motor(J5_TEST_FAULT, J5_REASON_TRANSPORT);
            }
        }
    }
    publish(now_ms);
}
