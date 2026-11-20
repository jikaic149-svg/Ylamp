#include "bsp_can.h"
#include "bsp_uart.h"
#include "bsp_rgb.h"
#include "bsp_key.h"
#include "bsp_power.h"
#include "dm_motor.h"
#include "motor_config.h"
#include "j25_task.h"
#include "motor_task.h"
#include "drag_mode.h"
#include "motor_tuning.h"
#include "motor_serial.h"
#include "motor_telemetry.h"
#include "can3_telemetry.h"
#include "bsp_usb.h"
#include "usb_telemetry.h"
#include "robot_model.h"
#include "joint_trajectory.h"
#include "motor_mode.h"
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
#include "motor_task.h"

static_assert(sizeof(BSP_CAN_Frame_t::data) == 8, "Classic CAN payload");
static_assert(sizeof(BSP_CAN_FDFrame_t::data) == 64, "CAN3 FD capacity");

extern "C" int bsp_cpp_headers_check(void)
{
    // 真实引用 C 符号，检查的不只是 C++ 语法，也包括 extern "C" 链接。
    return BSP_POWER_SetMotorPower(false) == HAL_ERROR &&
           BSP_POWER_GetEnabled(BSP_POWER_OUT1, nullptr) == HAL_ERROR &&
           DM_Motor_GetFeedback(nullptr, nullptr) == BSP_CAN_INVALID_ARG &&
           MotorTask_GetStatus(nullptr) == BSP_CAN_INVALID_ARG &&
           !DragGeometry_Evaluate(nullptr, 0.0f, 0.0f, nullptr) ? 0 : 1;
}
#include "motor_usb.h"
