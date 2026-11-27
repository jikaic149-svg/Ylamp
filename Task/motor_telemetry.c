#include "motor_telemetry.h"

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
