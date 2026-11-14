#include "j5_test.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

struct DM_Motor { unsigned marker; };
static DM_Motor_t motor;
FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
extern volatile uint32_t g_j5_test_request;
extern volatile J5Test_Status_t g_j5_test_status;

static uint32_t tick;
static bool valid = true;
static DM_Motor_Feedback_t feedback;
static DM_Motor_MIT_Command_t last_command;
static BSP_CAN_Status_t mit_result = BSP_CAN_OK;
static unsigned enables, disables, mits, queries;

uint32_t HAL_GetTick(void) { return tick; }
BSP_CAN_Status_t DM_Motor_GetFeedback(const DM_Motor_t *m, DM_Motor_Feedback_t *f)
{
    assert(m == &motor);
    if (!valid) { return BSP_CAN_NOT_READY; }
    *f = feedback;
    return BSP_CAN_OK;
}
BSP_CAN_Status_t DM_Motor_RequestFeedback(DM_Motor_t *m)
{
    assert(m == &motor);
    ++queries;
    return BSP_CAN_OK;
}
BSP_CAN_Status_t DM_Motor_Enable(DM_Motor_t *m)
{
    assert(m == &motor);
    ++enables;
    return BSP_CAN_OK;
}
BSP_CAN_Status_t DM_Motor_Disable(DM_Motor_t *m)
{
    assert(m == &motor);
    ++disables;
    return BSP_CAN_OK;
}
BSP_CAN_Status_t DM_Motor_SendMIT(DM_Motor_t *m, const DM_Motor_MIT_Command_t *c)
{
    assert(m == &motor);
    last_command = *c;
    ++mits;
    return mit_result;
}
BSP_CAN_Status_t BSP_CAN_GetDiagnostics(FDCAN_HandleTypeDef *bus, BSP_CAN_Diagnostics_t *d)
{
    assert(bus == &hfdcan1);
    *d = (BSP_CAN_Diagnostics_t){0};
    return BSP_CAN_OK;
}

static void step(uint32_t advance, bool update_feedback)
{
    tick += advance;
    if (update_feedback) {
        feedback.received_at_ms = tick;
        ++feedback.received_count;
    }
    J5Test_Run(tick);
    assert(g_j5_test_status.sequence == g_j5_test_status.sequence_end);
    assert((g_j5_test_status.sequence & 1U) == 0U);
}

static void reset(void)
{
    tick = 1000U;
    valid = true;
    enables = disables = mits = queries = 0U;
    mit_result = BSP_CAN_OK;
    feedback = (DM_Motor_Feedback_t){
        .position_rad = -2.0f, .received_at_ms = tick, .received_count = 1U,
        .mos_temperature_c = 25U, .rotor_temperature_c = 25U
    };
    J5Test_Init(&motor, tick);
}

static void start(void)
{
    g_j5_test_request = 1U;
    step(5U, true);
    assert(enables == 1U && g_j5_test_status.phase == J5_TEST_ARMING);
    assert(last_command.kp == 0.0f && last_command.kd == 0.0f);
    feedback.state = 1U;
    step(5U, true);
    assert(g_j5_test_status.phase == J5_TEST_MOVING);
}

int main(void)
{
    reset();
    const uint32_t initial_sequence = g_j5_test_status.sequence;
    for (unsigned i = 0U; i < 4U; ++i) { step(1U, true); }
    assert(g_j5_test_status.sequence == initial_sequence);
    step(1U, true);
    assert(g_j5_test_status.sequence == initial_sequence + 2U);
    reset();
    for (unsigned i = 0U; i < 20U; ++i) { step(5U, true); }
    assert(enables == 0U && mits == 0U && disables == 0U && queries == 10U);
    valid = false;
    g_j5_test_request = 1U;
    step(5U, false);
    assert(enables == 0U && g_j5_test_status.reason == J5_REASON_FEEDBACK);
    valid = true;
    feedback.state = 3U;
    g_j5_test_request = 1U;
    step(5U, true);
    assert(enables == 0U && g_j5_test_status.reason == J5_REASON_STATE);

    reset();
    start();
    const float origin = feedback.position_rad;
    const float peak_speed = 1.875f * g_j5_test_config.amplitude_rad *
        1000.0f / (float)g_j5_test_config.leg_ms;
    float maximum_target = origin;
    for (unsigned i = 0U; i < 1200U; ++i) {
        /* Ideal follower tests the real controller, not a physical motor model. */
        feedback.position_rad = last_command.position_rad;
        step(5U, true);
        assert(last_command.position_rad >= origin - 0.000001f);
        assert(last_command.position_rad <= origin + g_j5_test_config.amplitude_rad + 0.000001f);
        assert(fabsf(last_command.velocity_rad_s) <= peak_speed + 0.000001f);
        assert(last_command.kp >= 0.0f && last_command.kp <= g_j5_test_config.kp);
        assert(last_command.torque_ff_nm == 0.0f);
        if (last_command.position_rad > maximum_target) { maximum_target = last_command.position_rad; }
    }
    assert(g_j5_test_status.phase == J5_TEST_HOLDING && disables == 0U);
    assert(fabsf(maximum_target - origin - g_j5_test_config.amplitude_rad) < 0.000001f);
    assert(last_command.position_rad == origin && last_command.velocity_rad_s == 0.0f);
    g_j5_test_request = 2U;
    step(5U, true);
    assert(disables == 1U && g_j5_test_status.phase == J5_TEST_RELEASED);
    const unsigned stopped_mits = mits;
    step(5U, true);
    assert(mits == stopped_mits);

    reset();
    start();
    step(101U, false);
    assert(g_j5_test_status.phase == J5_TEST_FAULT && disables == 1U);
    assert(g_j5_test_status.reason == J5_REASON_FEEDBACK);
    const unsigned stale_mits = mits;
    step(5U, true);
    assert(enables == 1U && mits == stale_mits);

    reset();
    start();
    feedback.position_rad += g_j5_test_config.travel_limit_rad + 0.001f;
    step(5U, true);
    assert(g_j5_test_status.reason == J5_REASON_TRAVEL && disables == 1U);
    reset();
    start();
    mit_result = BSP_CAN_BUSY;
    step(5U, true);
    assert(g_j5_test_status.reason == J5_REASON_TRANSPORT && disables == 1U);
    reset();
    tick = UINT32_MAX - 15U;
    feedback.received_at_ms = tick;
    J5Test_Init(&motor, tick);
    start();
    step(10U, true);
    assert(g_j5_test_status.phase == J5_TEST_MOVING);
    puts("J5 isolated trajectory/handshake/feedback/fault checks passed");
    return 0;
}
