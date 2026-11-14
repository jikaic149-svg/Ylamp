#include "motor_telemetry.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

int main(int argc, char **argv)
{
    uint8_t buffer[MOTOR_TELEMETRY_FRAME_SIZE(MOTOR_TELEMETRY_MAX_JOINTS) + 1U];
    MotorTelemetry_Header_t header = {.tick_ms = 1234U, .sequence = 7U, .policy = 1U};
    MotorTelemetry_Joint_t joints[MOTOR_TELEMETRY_MAX_JOINTS] = {0};
    for (unsigned i = 0U; i < MOTOR_TELEMETRY_MAX_JOINTS; ++i) {
        joints[i] = (MotorTelemetry_Joint_t) {
            .id = (uint8_t)(i+1U), .flags = 1U, .can_id = (uint16_t)(i+1U),
            .feedback_id = (uint16_t)(0x11U+i), .received_ms = 1200U,
            .received_count = i+20U, .position = (float)i+.5f, .velocity = -.25f,
            .torque = .125f, .mos_c = 25U, .rotor_c = 26U,
            .reference = (float)i+.5f, .target = (float)i+.75f,
            .target_velocity = .125f, .kp = 5.0f, .kd = .5f, .ff = .25f,
            .target_reference = (float)i+.75f, .tracking_error = .25f
        };
    }
    const unsigned counts[] = {2U, 3U, 6U};
    for (unsigned n = 0U; n < 3U; ++n) {
        const unsigned count = counts[n];
        const size_t size = MOTOR_TELEMETRY_FRAME_SIZE(count);
        memset(buffer, 0xCD, sizeof(buffer));
        assert(MotorTelemetry_Encode(buffer, size-1U, &header, joints, count) == 0U);
        for (unsigned i = 0U; i < sizeof(buffer); ++i) { assert(buffer[i] == 0xCDU); }
        assert(MotorTelemetry_Encode(buffer, size, &header, joints, count) == size);
        assert(buffer[size] == 0xCDU && buffer[2] == 2U && buffer[3] == 1U);
        assert((unsigned)(buffer[4] | ((unsigned)buffer[5] << 8)) == size-8U);
        assert(buffer[17] == count && buffer[20] == MOTOR_TELEMETRY_RECORD_SIZE);
        const uint16_t crc = MotorTelemetry_CRC16(&buffer[2], size-4U);
        assert(buffer[size-2U] == (uint8_t)crc && buffer[size-1U] == (uint8_t)(crc >> 8));
        if (argc == 2 && strcmp(argv[1], "--vectors") == 0) {
            for (size_t i = 0U; i < size; ++i) { printf("%02x", buffer[i]); }
            putchar('\n');
        }
    }
    memset(buffer, 0xCD, sizeof(buffer));
    assert(MotorTelemetry_Encode(buffer, sizeof(buffer), &header, joints, 0U) == 0U);
    assert(MotorTelemetry_Encode(buffer, sizeof(buffer), &header, joints, 7U) == 0U);
    joints[1].id = 1U;
    assert(MotorTelemetry_Encode(buffer, sizeof(buffer), &header, joints, 2U) == 0U);
    joints[1].id = 7U;
    assert(MotorTelemetry_Encode(buffer, sizeof(buffer), &header, joints, 2U) == 0U);
    joints[1].id = 2U; joints[1].position = NAN;
    assert(MotorTelemetry_Encode(buffer, sizeof(buffer), &header, joints, 2U) == 0U);
    for (unsigned i = 0U; i < sizeof(buffer); ++i) { assert(buffer[i] == 0xCDU); }
    return 0;
}
