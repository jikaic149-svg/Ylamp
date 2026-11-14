#ifndef YLAMP_MOTOR_TELEMETRY_H
#define YLAMP_MOTOR_TELEMETRY_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* v2: A5 5A, version, type, payload_length(u16 LE), payload, CRC16-CCITT.
 * Payload = 64-byte common header + count * 64-byte joint records.
 * Binary32 floats and all integers are little endian, never native structs.
 * This transport capacity does NOT extend the current 2-DOF control/model. */
#define MOTOR_TELEMETRY_VERSION 2U
#define MOTOR_TELEMETRY_MAX_JOINTS 6U
#define MOTOR_TELEMETRY_HEADER_SIZE 64U
#define MOTOR_TELEMETRY_RECORD_SIZE 64U
#define MOTOR_TELEMETRY_FRAME_SIZE(count) (8U + 64U + 64U * (count))

typedef struct {
    uint32_t tick_ms, sequence;
    uint8_t state, reason, policy, mode, phase;
    uint32_t can_observations;
    uint8_t can_flags, first_lec, first_tec, first_rec;
    uint32_t first_ms, bo_ms;
    uint8_t bo_lec, bo_tec, bo_rec, last_transport;
    uint32_t pending, rx_overflows, crc_errors, tx_drops, uart_error;
} MotorTelemetry_Header_t;

typedef struct {
    uint8_t id, flags; /* logical ID 1..6; flags bit0 valid, bit1 disable submitted */
    uint16_t can_id, feedback_id;
    uint8_t disable_status;
    uint32_t received_ms, received_count;
    float position, velocity, torque;
    uint8_t state, mos_c, rotor_c;
    float reference, target, target_velocity, kp, kd, ff;
    float target_reference, tracking_error;
} MotorTelemetry_Joint_t;

/* Pure bounded codec. No HAL, allocation or I/O. Unique IDs, count/capacity
 * checked before touching output. Returns complete frame length, 0 on error.
 * Input/output may not overlap. Output is unchanged on rejected input. */
size_t MotorTelemetry_Encode(uint8_t *out, size_t capacity,
    const MotorTelemetry_Header_t *header,
    const MotorTelemetry_Joint_t *joints, unsigned count);
uint16_t MotorTelemetry_CRC16(const uint8_t *bytes, size_t length);

#ifdef __cplusplus
}
#endif
#endif
