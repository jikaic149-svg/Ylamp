#ifndef YLAMP_USB_TELEMETRY_H
#define YLAMP_USB_TELEMETRY_H

#include "bsp_usb.h"
#include "motor_snapshot.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    DM_Motor_t *motor;
    DM_Motor_Config_t config;
    uint8_t bus_number; /* Logical source identity 1/2; never MCU pin mapping. */
} USBTelemetry_Source_t;

typedef struct {
    uint32_t snapshots, submitted, dropped;
    BSP_USB_Status_t last_transport;
} USBTelemetry_Status_t;

/* One ordinary task owns all calls. Init copies 1..6 source descriptions;
 * handles remain valid for device lifetime. period_ms 1..65535, max_age_ms>0.
 * No allocation/USB init/motor command/query. Run copies latest feedback caches,
 * timestamps AFTER acquisition, encodes N adjacent 48-byte records, submits once.
 * First Run is due immediately; later calls are period-limited, wrap-safe
 * for gaps<2^31ms. No catch-up/replay; BUSY/closed host drops the whole attempt.
 * Counter sequence includes drops. Submission is not a delivery guarantee.
 * Configured reporting cadence is NOT the source encoder sampling frequency. */
BSP_USB_Status_t USBTelemetry_Init(const USBTelemetry_Source_t *sources,
    uint8_t count, uint16_t period_ms, uint16_t max_age_ms, uint32_t now_ms);
void USBTelemetry_Run(uint32_t now_ms);
BSP_USB_Status_t USBTelemetry_GetStatus(USBTelemetry_Status_t *out);

#ifdef __cplusplus
}
#endif
#endif
