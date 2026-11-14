#include "motor_snapshot.h"

#include <math.h>
#include <string.h>

static void put16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)value; out[1] = (uint8_t)(value >> 8);
}

static void put32(uint8_t *out, uint32_t value)
{
    for (unsigned i = 0U; i < 4U; ++i) { out[i] = (uint8_t)(value >> (8U*i)); }
}

static void put_float(uint8_t *out, float value)
{
    uint32_t bits;
    _Static_assert(sizeof(float) == 4U, "wire format requires binary32");
    memcpy(&bits, &value, sizeof(bits));
    put32(out, bits);
}

void MotorSnapshot_Encode(uint8_t out[MOTOR_SNAPSHOT_LENGTH],
    uint8_t count, uint8_t index, uint8_t bus, uint32_t sequence,
    uint32_t tick_ms, uint16_t max_age_ms, const DM_Motor_Config_t *config,
    const DM_Motor_Feedback_t *feedback)
{
    memset(out, 0, MOTOR_SNAPSHOT_LENGTH);
    out[0] = 'Y'; out[1] = 'L'; out[2] = MOTOR_SNAPSHOT_VERSION; out[3] = count;
    put32(&out[4], sequence); put32(&out[8], tick_ms); put16(&out[12], max_age_ms);
    out[14] = MOTOR_SNAPSHOT_RECORD_SIZE; out[15] = index;
    uint8_t *r = &out[16];
    r[0] = (uint8_t)(index + 1U); r[1] = bus; r[2] = (uint8_t)config->protocol;
    put16(&r[4], config->can_id); put16(&r[6], config->master_id);
    put_float(&r[16], NAN); put_float(&r[20], NAN); put_float(&r[24], NAN);
    r[28] = r[29] = r[30] = 0xFFU;
    if (feedback != NULL) {
        r[3] = (uint8_t)(1U | (tick_ms - feedback->received_at_ms <= max_age_ms ? 2U : 0U));
        put32(&r[8], feedback->received_at_ms); put32(&r[12], feedback->received_count);
        put_float(&r[16], feedback->position_rad); put_float(&r[20], feedback->velocity_rad_s);
        put_float(&r[24], feedback->torque_nm);
        r[28] = feedback->state; r[29] = feedback->mos_temperature_c;
        r[30] = feedback->rotor_temperature_c;
    }
}
