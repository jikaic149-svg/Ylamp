#include "can3_telemetry.h"
#include "can3_config.h"

#include <stddef.h>
#include <string.h>

_Static_assert(MOTOR_CAN_JOINT_COUNT <= 6U, "CAN3 receiver supports at most six joints");
_Static_assert(CAN3_TELEMETRY_MAX_AGE_MS <= UINT16_MAX, "age field is u16");
_Static_assert(CAN3_TELEMETRY_PERIOD_MS >= MOTOR_CAN_JOINT_COUNT,
               "one frame per ms must fit the snapshot period");

static DM_Motor_t *sources[MOTOR_CAN_JOINT_COUNT];
static uint8_t packets[MOTOR_CAN_JOINT_COUNT][CAN3_TELEMETRY_LENGTH];
static CAN3Telemetry_Status_t status;
static uint32_t last_snapshot_ms, last_run_ms;
static unsigned next_record = MOTOR_CAN_JOINT_COUNT;
static bool initialized;

BSP_CAN_Status_t CAN3Telemetry_Init(DM_Motor_t *const motors[MOTOR_CAN_JOINT_COUNT],
    uint32_t now_ms, BSP_CAN_Status_t transport_status)
{
    if (motors == NULL || __get_IPSR() != 0U) { return BSP_CAN_INVALID_ARG; }
    if (initialized) { return BSP_CAN_BUSY; }
    for (unsigned i = 0U; i < MOTOR_CAN_JOINT_COUNT; ++i) {
        if (motors[i] == NULL) { return BSP_CAN_INVALID_ARG; }
    }
    memcpy(sources, motors, sizeof(sources));
    status.init_status = transport_status;
    status.last_transport = transport_status;
    last_snapshot_ms = now_ms - CAN3_TELEMETRY_PERIOD_MS;
    last_run_ms = now_ms - 1U;
    initialized = true;
    return transport_status;
}

static void snapshot(uint32_t now_ms)
{
    DM_Motor_Feedback_t feedback[MOTOR_CAN_JOINT_COUNT];
    bool valid[MOTOR_CAN_JOINT_COUNT];
    for (unsigned i = 0U; i < MOTOR_CAN_JOINT_COUNT; ++i) {
        valid[i] = DM_Motor_GetFeedback(sources[i], &feedback[i]) == BSP_CAN_OK;
    }
    /* An RX IRQ can run while acquiring the per-motor snapshots. Timestamp
     * AFTER copying them so fresh IRQ ticks never become huge unsigned ages. */
    const uint32_t sampled_ms = HAL_GetTick();
    ++status.snapshots; /* sequence wraps naturally; includes dropped attempts */
    last_snapshot_ms = now_ms;
    for (unsigned i = 0U; i < MOTOR_CAN_JOINT_COUNT; ++i) {
        const DM_Motor_Config_t *cfg = &g_motor_can_config[i];
        const uint8_t bus = cfg->bus == &hfdcan1 ? 1U : cfg->bus == &hfdcan2 ? 2U : 3U;
        MotorSnapshot_Encode(packets[i], MOTOR_CAN_JOINT_COUNT, (uint8_t)i, bus,
            status.snapshots, sampled_ms, CAN3_TELEMETRY_MAX_AGE_MS,
            cfg, valid[i] ? &feedback[i] : NULL);
    }
    next_record = 0U;
}

void CAN3Telemetry_Run(uint32_t now_ms)
{
    if (!initialized || status.init_status != BSP_CAN_OK ||
        __get_IPSR() != 0U || now_ms == last_run_ms) { return; }
    last_run_ms = now_ms;
    if (now_ms - last_snapshot_ms >= CAN3_TELEMETRY_PERIOD_MS) {
        if (next_record < MOTOR_CAN_JOINT_COUNT) { ++status.snapshots_dropped; }
        snapshot(now_ms);
    }
    if (next_record == MOTOR_CAN_JOINT_COUNT) { return; }
    BSP_CAN_FDFrame_t frame = {.id = CAN3_TELEMETRY_ID, .len = CAN3_TELEMETRY_LENGTH};
    memcpy(frame.data, packets[next_record], CAN3_TELEMETRY_LENGTH);
    status.last_transport = BSP_CAN_SendFD(&hfdcan3, &frame);
    if (status.last_transport == BSP_CAN_OK) {
        ++status.frames_submitted;
        ++next_record;
    } else {
        ++status.snapshots_dropped;
        next_record = MOTOR_CAN_JOINT_COUNT;
    }
}

BSP_CAN_Status_t CAN3Telemetry_GetStatus(CAN3Telemetry_Status_t *out)
{
    if (out == NULL || __get_IPSR() != 0U) { return BSP_CAN_INVALID_ARG; }
    if (!initialized) { return BSP_CAN_NOT_READY; }
    *out = status;
    return BSP_CAN_OK;
}
