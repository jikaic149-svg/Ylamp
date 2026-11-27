#include "bsp_can.h"
#include "bsp_uart.h"
#include "bsp_rgb.h"
#include "bsp_key.h"
#include "bsp_power.h"
#include "dm_motor.h"
#include "motor_config.h"
#include "five_control.h"
#include "motor_telemetry.h"
#include "bsp_usb.h"
#include "usb_telemetry.h"
#include "five_dynamics.h"
#include "joint_trajectory.h"
#include "joint_calibration.h"
#include "yummy_imu.h"
#include "bmi088_port.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "bsp_rgb.h"
#include "bsp_key.h"
#include "bsp_power.h"
#include "dm_motor.h"
#include "motor_config.h"

static_assert(sizeof(BSP_CAN_Frame_t::data) == 8, "Classic CAN payload");

extern "C" int bsp_cpp_headers_check(void)
{
    // 真实引用 C 符号，检查的不只是 C++ 语法，也包括 extern "C" 链接。
    return BSP_POWER_SetMotorPower(false) == HAL_ERROR &&
           BSP_POWER_GetEnabled(BSP_POWER_OUT1, nullptr) == HAL_ERROR &&
           DM_Motor_GetFeedback(nullptr, nullptr) == BSP_CAN_INVALID_ARG ? 0 : 1;
}
#include "motor_usb.h"
