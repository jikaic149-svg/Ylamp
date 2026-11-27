#include "motor_telemetry.h"

#include <assert.h>
int main(void)
{
    static const uint8_t check[] = "123456789";
    assert(MotorTelemetry_CRC16(check, sizeof(check) - 1U) == 0x29B1U);
    assert(MotorTelemetry_CRC16(check, 0U) == 0xFFFFU);
    return 0;
}
