#ifndef YLAMP_MOTOR_TELEMETRY_H
#define YLAMP_MOTOR_TELEMETRY_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CRC-16/CCITT-FALSE shared by USB requests, replies and observations. */
uint16_t MotorTelemetry_CRC16(const uint8_t *bytes, size_t length);

#ifdef __cplusplus
}
#endif
#endif
