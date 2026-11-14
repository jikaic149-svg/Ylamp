#include "motor_telemetry.h"

#include <math.h>
#include <string.h>

static void put_u16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)value; out[1] = (uint8_t)(value >> 8);
}

static void put_u32(uint8_t *out, uint32_t value)
{
    for (unsigned i = 0U; i < 4U; ++i) { out[i] = (uint8_t)(value >> (8U*i)); }
}

static void put_float(uint8_t *out, float value)
{
    uint32_t bits;
    _Static_assert(sizeof(float) == 4U, "Telemetry requires binary32");
    memcpy(&bits, &value, sizeof(bits));
    put_u32(out, bits);
}

uint16_t MotorTelemetry_CRC16(const uint8_t *bytes, size_t length)
{
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0U; i < length; ++i) {
        crc ^= (uint16_t)((uint16_t)bytes[i] << 8);
        for (unsigned bit = 0U; bit < 8U; ++bit) {
            crc = (uint16_t)((crc << 1) ^ ((crc & 0x8000U) ? 0x1021U : 0U));
        }
    }
    return crc;
}

size_t MotorTelemetry_Encode(uint8_t *out, size_t capacity,
    const MotorTelemetry_Header_t *header,
    const MotorTelemetry_Joint_t *joints, unsigned count)
{
    if (out == NULL || header == NULL || joints == NULL || count == 0U ||
        count > MOTOR_TELEMETRY_MAX_JOINTS ||
        capacity < MOTOR_TELEMETRY_FRAME_SIZE(count)) { return 0U; }
    uint8_t seen = 0U;
    for (unsigned i = 0U; i < count; ++i) {
        const MotorTelemetry_Joint_t *j = &joints[i];
        if (j->id == 0U || j->id > MOTOR_TELEMETRY_MAX_JOINTS ||
            (j->flags & ~3U) != 0U || j->can_id > 0x7FFU ||
            j->feedback_id > 0x7FFU || (seen & (1U << (j->id - 1U))) != 0U ||
            ((j->flags & 1U) != 0U &&
             (!isfinite(j->position) || !isfinite(j->velocity) || !isfinite(j->torque)))) {
            return 0U;
        }
        seen |= (uint8_t)(1U << (j->id - 1U));
    }
    const size_t total = MOTOR_TELEMETRY_FRAME_SIZE(count);
    const uint16_t payload = (uint16_t)(total - 8U);
    memset(out, 0, total);
    out[0] = 0xA5U; out[1] = 0x5AU;
    out[2] = MOTOR_TELEMETRY_VERSION; out[3] = 1U;
    put_u16(&out[4], payload);
    uint8_t *p = &out[6];
    put_u32(&p[0], header->tick_ms); put_u32(&p[4], header->sequence);
    p[8] = header->state; p[9] = header->reason; p[10] = header->policy;
    p[11] = (uint8_t)count; p[12] = header->mode; p[13] = header->phase;
    p[14] = MOTOR_TELEMETRY_RECORD_SIZE;
    put_u32(&p[16], header->can_observations);
    p[20] = header->can_flags; p[21] = header->first_lec;
    p[22] = header->first_tec; p[23] = header->first_rec;
    put_u32(&p[24], header->first_ms); put_u32(&p[28], header->bo_ms);
    p[32] = header->bo_lec; p[33] = header->bo_tec;
    p[34] = header->bo_rec; p[35] = header->last_transport;
    put_u32(&p[36], header->pending); put_u32(&p[40], header->rx_overflows);
    put_u32(&p[44], header->crc_errors); put_u32(&p[48], header->tx_drops);
    put_u32(&p[52], header->uart_error);
    for (unsigned i = 0U; i < count; ++i) {
        const MotorTelemetry_Joint_t *j = &joints[i];
        uint8_t *r = &p[MOTOR_TELEMETRY_HEADER_SIZE + MOTOR_TELEMETRY_RECORD_SIZE*i];
        r[0] = j->id; r[1] = j->flags;
        put_u16(&r[2], j->can_id); put_u16(&r[4], j->feedback_id);
        r[6] = j->disable_status;
        put_u32(&r[8], j->received_ms); put_u32(&r[12], j->received_count);
        put_float(&r[16], j->position); put_float(&r[20], j->velocity);
        put_float(&r[24], j->torque);
        r[28] = j->state; r[29] = j->mos_c; r[30] = j->rotor_c;
        put_float(&r[32], j->reference); put_float(&r[36], j->target);
        put_float(&r[40], j->target_velocity); put_float(&r[44], j->kp);
        put_float(&r[48], j->kd); put_float(&r[52], j->ff);
        put_float(&r[56], j->target_reference); put_float(&r[60], j->tracking_error);
    }
    put_u16(&out[total - 2U], MotorTelemetry_CRC16(&out[2], total - 4U));
    return total;
}
