#include "main.h"
#include "bsp_key.h"
#include "bsp_rgb.h"
#include "five_control.h"
#include "lookaround_task.h"
#include "bmi088_port.h"
#include "motor_usb.h"
#include "usb_telemetry.h"
#include "joint_calibration.h"
#include "yummy_imu.h"
#include "fault.h"

static void show_control_mode(const FiveControl_Status *s, bool local_trajectory)
{
    static const uint8_t colors[][3]={
        [FIVE_WAIT]      ={ 0U, 0U, 8U}, /* Blue */
        [FIVE_IDLE]      ={ 0U, 8U, 0U}, /* Green */
        [FIVE_MOTION]    ={ 6U, 6U, 6U}, /* White */
        [FIVE_POSITION]  ={12U, 3U, 0U}, /* Orange */
        [FIVE_DEGRADED]  ={ 8U, 0U, 0U}, /* Red */
        [FIVE_IMPEDANCE] ={ 8U, 8U, 0U}, /* Yellow */
        [FIVE_DRAG]      ={ 0U, 8U, 8U}, /* Cyan */
    };
    if (!s) { return; }
    const FivePhase phase=s->fault ? FIVE_DEGRADED : s->phase;
    if (phase==FIVE_MOTION && local_trajectory) {
        (void)BSP_RGB_Set(8U,0U,8U); /* Purple only during local playback. */
    } else if ((unsigned)phase<sizeof colors/sizeof colors[0]) {
        (void)BSP_RGB_Set(colors[phase][0],colors[phase][1],colors[phase][2]);
    }
}

/* One iteration of YlampControl, called after initialization, not from ISR. */
void Ylamp_ControlStep(uint32_t now)
{
    static uint32_t report;
    g_fault_stage=YLAMP_STAGE_MOTOR;
    FiveControl_Tick(now);
    const bool refresh_rgb=now-report>=500U; /* Keep existing SPI polling budget. */
    /* Ownership paired with the phase from the motor tick, before KEY/USB can change it. */
    const bool local_trajectory=refresh_rgb && FiveControl_LocalTrajectoryOwned();
    (void)LookaroundTask_Run(BSP_KEY_Scan(now)==BSP_KEY_EVENT_PRESSED);
    g_fault_stage=YLAMP_STAGE_IMU;
    BMI088_Run(now);
    YummyIMU_Run();
    g_fault_stage=YLAMP_STAGE_CALIBRATION;
    JointCalibration_Run(now);
    if (MotorUSB_Run()) { USBTelemetry_Run(HAL_GetTick()); }
    if (refresh_rgb) {
        report=now;
        show_control_mode(FiveControl_StatusNow(),local_trajectory);
    }
    g_fault_stage=YLAMP_STAGE_WAIT;
}
