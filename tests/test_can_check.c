#include "can_check.h"

#include <assert.h>
#include <stdio.h>

struct DM_Motor { unsigned index; };
static DM_Motor_t devices[5];
static DM_Motor_t *handles[5];
FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
extern volatile uint32_t g_can_check_request;
extern volatile CanCheck_Status_t g_can_check_status;
static DM_Motor_Feedback_t feedback[5];
static unsigned enables[5], disables[5], preloads[5], queries[5];
static uint32_t tick;
static int silent = -1, disable_silent = -1;
static unsigned probes;
static BSP_CAN_Status_t disable_transport;
static BSP_CAN_Status_t mit_transport;
static bool mit_silent;

uint32_t HAL_GetTick(void) { return tick; }
BSP_CAN_Status_t DM_Motor_GetFeedback(const DM_Motor_t *m, DM_Motor_Feedback_t *f)
{
    if ((int)m->index == silent) { return BSP_CAN_NOT_READY; }
    *f = feedback[m->index];
    return BSP_CAN_OK;
}
BSP_CAN_Status_t DM_Motor_RequestFeedback(DM_Motor_t *m)
{
    assert(m->index != 2U); /* Configured legacy J3 must never be CC-queried. */
    ++queries[m->index];
    if ((int)m->index != silent) {
        feedback[m->index].received_at_ms = tick;
        ++feedback[m->index].received_count;
    }
    return BSP_CAN_OK;
}
BSP_CAN_Status_t DM_Motor_ProbeReadOnly(FDCAN_HandleTypeDef *bus,
                                      uint16_t id, bool read_master)
{
    assert(bus == (probes < 30U ? &hfdcan1 : &hfdcan2));
    assert(id == (probes % 30U) / 2U + 1U);
    assert(read_master == ((probes % 2U) != 0U));
    ++probes;
    return BSP_CAN_OK;
}
BSP_CAN_Status_t DM_Motor_SendMIT(DM_Motor_t *m, const DM_Motor_MIT_Command_t *c)
{
    assert(c->kp == 0.0f && c->kd == 0.0f && c->torque_ff_nm == 0.0f);
    assert(c->velocity_rad_s == 0.0f);
    ++preloads[m->index];
    if (m->index == 2U && mit_transport == BSP_CAN_OK && !mit_silent) {
        feedback[m->index].received_at_ms = tick;
        ++feedback[m->index].received_count;
    }
    return mit_transport;
}
BSP_CAN_Status_t DM_Motor_Enable(DM_Motor_t *m)
{
    assert(preloads[m->index] == 1U && feedback[m->index].state == 0U);
    ++enables[m->index];
    feedback[m->index].state = 1U;
    if (m->index == 2U) {
        feedback[m->index].received_at_ms = tick;
        ++feedback[m->index].received_count;
    }
    return BSP_CAN_OK;
}
BSP_CAN_Status_t DM_Motor_Disable(DM_Motor_t *m)
{
    ++disables[m->index];
    if (disable_transport != BSP_CAN_OK) { return disable_transport; }
    if ((int)m->index != disable_silent) {
        feedback[m->index].state = 0U;
        feedback[m->index].received_at_ms = tick;
        ++feedback[m->index].received_count;
    }
    return BSP_CAN_OK;
}
BSP_CAN_Status_t BSP_CAN_GetDiagnostics(FDCAN_HandleTypeDef *bus, BSP_CAN_Diagnostics_t *d)
{
    assert(bus == &hfdcan1 || bus == &hfdcan2);
    *d = (BSP_CAN_Diagnostics_t){0};
    return BSP_CAN_OK;
}
static void run(unsigned steps)
{
    while (steps-- != 0U) {
        tick += 5U;
        CanCheck_Run(tick);
        assert(g_can_check_status.sequence == g_can_check_status.sequence_end);
        assert((g_can_check_status.sequence & 1U) == 0U);
    }
}
static void reset(void)
{
    tick = 1000U;
    disable_transport = BSP_CAN_OK;
    mit_transport = BSP_CAN_OK;
    mit_silent = false;
    for (unsigned i = 0U; i < 5U; ++i) {
        devices[i].index = i;
        handles[i] = &devices[i];
        feedback[i] = (DM_Motor_Feedback_t){.position_rad = -1.0f};
        enables[i] = disables[i] = preloads[i] = queries[i] = 0U;
    }
    CanCheck_Init(handles, tick);
}
int main(void)
{
    reset();
    run(200U);
    for (unsigned i = 0U; i < 5U; ++i) {
        assert(enables[i] == 0U && queries[i] == 0U);
    }
    g_can_check_request = 1U;
    run(700U);
    assert(g_can_check_status.phase == CAN_CHECK_DONE);
    for (unsigned i = 0U; i < 5U; ++i) {
        if (i == 2U) {
            assert(enables[i] == 0U && disables[i] == 0U && queries[i] == 0U);
            assert(g_can_check_status.motor[i].reason == CAN_CHECK_QUERY_UNSUPPORTED);
            continue;
        }
        assert(enables[i] == 1U && disables[i] == 1U);
        assert(g_can_check_status.motor[i].enable_confirmed == 1U);
        assert(g_can_check_status.motor[i].disable_confirmed == 1U);
    }
    g_can_check_request = 1U;
    run(200U);
    assert(enables[0] == 1U); /* Never automatically rerun. */
    reset();
    silent = 1;
    g_can_check_request = 1U;
    run(700U);
    assert(g_can_check_status.phase == CAN_CHECK_DONE && enables[1] == 0U);
    assert(g_can_check_status.motor[1].reason == CAN_CHECK_NO_REPLY);
    assert(enables[4] == 1U);
    silent = -1;
    reset();
    disable_silent = 0;
    g_can_check_request = 1U;
    run(700U);
    assert(g_can_check_status.phase == CAN_CHECK_STOPPED && enables[1] == 0U);
    assert(g_can_check_status.motor[0].reason == CAN_CHECK_DISABLE_TIMEOUT);
    disable_silent = -1;
    reset();
    g_can_check_request = 2U;
    run(700U);
    assert(g_can_check_status.phase == CAN_CHECK_DONE);
    for (unsigned i = 0U; i < 5U; ++i) {
        assert(enables[i] == 0U && disables[i] == 0U && preloads[i] == 0U);
        assert((g_can_check_status.motor[i].replies != 0U) == (i != 2U));
    }
    g_can_check_request = 3U;
    run(200U);
    assert(g_can_check_status.phase == CAN_CHECK_DONE && probes == 60U);
    for (unsigned i = 0U; i < 5U; ++i) {
        assert(enables[i] == 0U && disables[i] == 0U && preloads[i] == 0U);
    }
    reset();
    g_can_check_request = 4U;
    run(200U);
    assert(g_can_check_status.phase == CAN_CHECK_DONE);
    assert(g_can_check_status.motor_index == 2U && g_can_check_status.completed == 1U);
    assert(g_can_check_status.motor[2].reason == CAN_CHECK_QUERY_UNSUPPORTED);
    assert(g_can_check_status.motor[2].replies == 0U);
    for (unsigned i = 0U; i < 5U; ++i) {
        assert(enables[i] == 0U && disables[i] == 0U && preloads[i] == 0U);
        assert(queries[i] == 0U);
    }
    silent = 2;
    g_can_check_request = 4U; /* Explicit rerun from DONE, still J3 only. */
    run(200U);
    assert(g_can_check_status.phase == CAN_CHECK_DONE);
    assert(g_can_check_status.completed == 1U);
    assert(g_can_check_status.motor[2].reason == CAN_CHECK_QUERY_UNSUPPORTED);
    assert(g_can_check_status.motor[2].replies == 0U);
    for (unsigned i = 0U; i < 5U; ++i) {
        assert(enables[i] == 0U && disables[i] == 0U && preloads[i] == 0U);
        if (i != 2U) { assert(queries[i] == 0U); }
    }
    silent = -1;
    g_can_check_request = 2U; /* Single-axis selection must not leak to all-axis queries. */
    run(700U);
    assert(g_can_check_status.phase == CAN_CHECK_DONE && g_can_check_status.completed == 5U);
    for (unsigned i = 0U; i < 5U; ++i) {
        assert((g_can_check_status.motor[i].replies != 0U) == (i != 2U));
        assert(enables[i] == 0U && disables[i] == 0U && preloads[i] == 0U);
    }
    reset();
    feedback[2].received_count = 9U; /* Cached feedback must not confirm disable. */
    g_can_check_request = 5U;
    run(200U);
    assert(g_can_check_status.phase == CAN_CHECK_DONE);
    assert(g_can_check_status.motor_index == 2U && g_can_check_status.completed == 1U);
    assert(g_can_check_status.motor[2].feedback_start_count == 9U);
    assert(g_can_check_status.motor[2].replies == 1U);
    assert(g_can_check_status.motor[2].disable_confirmed == 1U);
    const uint32_t frozen_sequence = g_can_check_status.sequence;
    run(200U);
    assert(g_can_check_status.sequence == frozen_sequence);
    for (unsigned i = 0U; i < 5U; ++i) {
        assert(queries[i] == 0U && enables[i] == 0U && preloads[i] == 0U);
        assert(disables[i] == (i == 2U ? 1U : 0U));
    }
    g_can_check_request = 2U; /* Legacy mode must not suppress later CC queries. */
    run(700U);
    assert(g_can_check_status.phase == CAN_CHECK_DONE && g_can_check_status.completed == 5U);
    for (unsigned i = 0U; i < 5U; ++i) {
        assert((queries[i] != 0U) == (i != 2U));
        assert(enables[i] == 0U && preloads[i] == 0U);
        assert(disables[i] == (i == 2U ? 1U : 0U));
    }
    reset();
    disable_silent = 2;
    feedback[2].received_count = 9U;
    g_can_check_request = 5U;
    run(200U);
    assert(g_can_check_status.phase == CAN_CHECK_STOPPED);
    assert(g_can_check_status.motor[2].reason == CAN_CHECK_DISABLE_TIMEOUT);
    assert(g_can_check_status.motor[2].replies == 0U);
    assert(g_can_check_status.motor[2].disable_submitted == 1U);
    assert(g_can_check_status.motor[2].disable_confirmed == 0U);
    for (unsigned i = 0U; i < 5U; ++i) {
        assert(queries[i] == 0U && enables[i] == 0U && preloads[i] == 0U);
        assert(disables[i] == (i == 2U ? 1U : 0U));
    }
    disable_silent = -1;
    reset();
    disable_transport = BSP_CAN_BUSY;
    g_can_check_request = 5U;
    run(200U);
    assert(g_can_check_status.phase == CAN_CHECK_STOPPED);
    assert(g_can_check_status.motor[2].reason == CAN_CHECK_TRANSPORT);
    assert(g_can_check_status.motor[2].last_transport == BSP_CAN_BUSY);
    assert(g_can_check_status.motor[2].disable_submitted == 0U);
    for (unsigned i = 0U; i < 5U; ++i) {
        assert(queries[i] == 0U && enables[i] == 0U && preloads[i] == 0U);
        assert(disables[i] == (i == 2U ? 1U : 0U));
    }
    reset();
    g_can_check_request = 6U;
    run(20U);
    assert(g_can_check_status.phase == CAN_CHECK_STREAMING && enables[2] == 1U);
    g_can_check_request = 6U; /* A second trigger cannot restart an active test. */
    run(500U);
    assert(g_can_check_status.phase == CAN_CHECK_DONE && g_can_check_status.completed == 1U);
    assert(g_can_check_status.motor[2].reason == CAN_CHECK_OK);
    assert(g_can_check_status.motor[2].enable_confirmed == 1U);
    assert(g_can_check_status.motor[2].disable_confirmed == 2U);
    assert(g_can_check_status.motor[2].mit_submitted == preloads[2]);
    assert(g_can_check_status.motor[2].mit_failed == 0U);
    assert(g_can_check_status.motor[2].stream_replies >= 199U);
    assert(g_can_check_status.motor[2].stream_elapsed_ms == MOTOR_CAN_CHECK_STREAM_MS);
    assert(enables[2] == 1U && disables[2] == 2U && feedback[2].state == 0U);
    for (unsigned i = 0U; i < 5U; ++i) {
        assert(queries[i] == 0U);
        if (i != 2U) { assert(enables[i] == 0U && disables[i] == 0U && preloads[i] == 0U); }
    }
    const unsigned frozen_mit = preloads[2];
    run(100U);
    assert(preloads[2] == frozen_mit); /* No background neutral traffic. */
    g_can_check_request = 2U;
    run(700U);
    assert(g_can_check_status.phase == CAN_CHECK_DONE && queries[0] != 0U && queries[2] == 0U);
    reset();
    disable_silent = 2;
    g_can_check_request = 6U;
    run(200U);
    assert(g_can_check_status.phase == CAN_CHECK_STOPPED);
    assert(g_can_check_status.motor[2].reason == CAN_CHECK_DISABLE_TIMEOUT);
    assert(enables[2] == 0U && preloads[2] == 0U);
    disable_silent = -1;
    reset();
    g_can_check_request = 6U;
    run(20U);
    mit_silent = true;
    run(100U);
    assert(g_can_check_status.phase == CAN_CHECK_DONE);
    assert(g_can_check_status.motor[2].reason == CAN_CHECK_NO_REPLY);
    assert(g_can_check_status.motor[2].disable_confirmed == 2U && feedback[2].state == 0U);
    reset();
    g_can_check_request = 6U;
    run(20U);
    mit_transport = BSP_CAN_BUSY;
    run(100U);
    assert(g_can_check_status.phase == CAN_CHECK_DONE);
    assert(g_can_check_status.motor[2].reason == CAN_CHECK_TRANSPORT);
    assert(g_can_check_status.motor[2].mit_failed == 1U);
    assert(g_can_check_status.motor[2].disable_confirmed == 2U);
    reset();
    g_can_check_request = 6U;
    run(20U);
    disable_silent = 2; /* Final disable must not reuse the initial state=0 ACK. */
    run(600U);
    assert(g_can_check_status.phase == CAN_CHECK_STOPPED);
    assert(g_can_check_status.motor[2].reason == CAN_CHECK_DISABLE_TIMEOUT);
    assert(g_can_check_status.motor[2].disable_confirmed == 1U && disables[2] == 2U);
    disable_silent = -1;
    reset();
    feedback[2].mos_temperature_c = 100U;
    g_can_check_request = 6U;
    run(20U);
    assert(g_can_check_status.phase == CAN_CHECK_DONE);
    assert(g_can_check_status.motor[2].reason == CAN_CHECK_TEMPERATURE && enables[2] == 0U);
    puts("CAN query/enable/disable and bounded legacy neutral stream checks passed");
    return 0;
}
