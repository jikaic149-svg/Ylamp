#include "usb_telemetry.h"

#include <string.h>

_Static_assert(MOTOR_SNAPSHOT_MAX_JOINTS * MOTOR_SNAPSHOT_LENGTH <= BSP_USB_TX_CAPACITY,
               "whole snapshot must fit one owned USB transfer");
static USBTelemetry_Source_t sources[MOTOR_SNAPSHOT_MAX_JOINTS];
static uint8_t packet[MOTOR_SNAPSHOT_MAX_JOINTS * MOTOR_SNAPSHOT_LENGTH];
static uint8_t source_count;
static uint16_t period, max_age;
static uint32_t last_run_ms;
static USBTelemetry_Status_t status;

BSP_USB_Status_t USBTelemetry_Init(const USBTelemetry_Source_t *input,
    uint8_t count, uint16_t period_ms, uint16_t max_age_ms, uint32_t now_ms)
{
    if (input == NULL || count == 0U || count > MOTOR_SNAPSHOT_MAX_JOINTS ||
        period_ms == 0U || max_age_ms == 0U || __get_IPSR() != 0U) {
        return BSP_USB_INVALID_ARG;
    }
    if (source_count != 0U) { return BSP_USB_BUSY; }
    for (unsigned i = 0U; i < count; ++i) {
        if (input[i].motor == NULL || (input[i].bus_number != 1U && input[i].bus_number != 2U) ||
            input[i].config.can_id == 0U || input[i].config.can_id > 15U ||
            input[i].config.master_id == 0U || input[i].config.master_id > 0x7FEU ||
            (input[i].config.protocol != DM_MOTOR_PROTOCOL_V13 &&
             input[i].config.protocol != DM_MOTOR_PROTOCOL_LEGACY_MIT)) {
            return BSP_USB_INVALID_ARG;
        }
    }
    memcpy(sources, input, count * sizeof(*input));
    source_count = count; period = period_ms; max_age = max_age_ms;
    last_run_ms = now_ms - period;
    status.last_transport = BSP_USB_NOT_READY;
    return BSP_USB_OK;
}

void USBTelemetry_Run(uint32_t now_ms)
{
    if (source_count == 0U || __get_IPSR() != 0U || now_ms - last_run_ms < period) { return; }
    last_run_ms = now_ms;
    DM_Motor_Feedback_t feedback[MOTOR_SNAPSHOT_MAX_JOINTS];
    bool valid[MOTOR_SNAPSHOT_MAX_JOINTS];
    for (unsigned i = 0U; i < source_count; ++i) {
        valid[i] = DM_Motor_GetFeedback(sources[i].motor, &feedback[i]) == BSP_CAN_OK;
    }
    const uint32_t sampled_ms = HAL_GetTick();
    ++status.snapshots;
    for (unsigned i = 0U; i < source_count; ++i) {
        MotorSnapshot_Encode(&packet[i * MOTOR_SNAPSHOT_LENGTH], source_count,
            (uint8_t)i, sources[i].bus_number, status.snapshots, sampled_ms,
            max_age, &sources[i].config, valid[i] ? &feedback[i] : NULL);
    }
    status.last_transport = BSP_USB_Send(packet, source_count * MOTOR_SNAPSHOT_LENGTH);
    if (status.last_transport == BSP_USB_OK) { ++status.submitted; } else { ++status.dropped; }
}

BSP_USB_Status_t USBTelemetry_GetStatus(USBTelemetry_Status_t *out)
{
    if (out == NULL || __get_IPSR() != 0U) { return BSP_USB_INVALID_ARG; }
    if (source_count == 0U) { return BSP_USB_NOT_READY; }
    *out = status;
    return BSP_USB_OK;
}
