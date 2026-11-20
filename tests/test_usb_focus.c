#include "motor_usb.h"
#include "bsp_usb.h"
#include "five_control.h"
#include "joint_calibration.h"
#include "motor_telemetry.h"
#include "yummy_imu.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

uint32_t test_ipsr, test_primask, test_tick=100;
FDCAN_HandleTypeDef hfdcan1, hfdcan2;
static uint8_t input[256], output[512];
static unsigned head, tail, output_size, writes, actuator_calls;
static unsigned sensor_starts, sensor_stops;
static uint32_t epoch;
static bool dtr=true, busy;
static FiveControl_Status control;
static YummyIMU_Status_t imu;
static BMI088_Sample_t bmi;
static uint8_t bmi_flags;
uint32_t HAL_GetTick(void) { return test_tick; }
const FiveControl_Status *FiveControl_StatusNow(void) { return &control; }
void FiveControl_Gravity(void) { ++actuator_calls; }
BSP_CAN_Status_t FiveControl_ColdStart(void) { ++actuator_calls;return BSP_CAN_OK; }
BSP_CAN_Status_t FiveControl_Acknowledge(void) { ++actuator_calls;return BSP_CAN_OK; }
BSP_CAN_Status_t FiveControl_ReadLegacyJ3(void) { ++actuator_calls;return BSP_CAN_OK; }
BSP_CAN_Status_t FiveControl_Move(const double q[5],uint32_t ms)
{ (void)q;(void)ms;++actuator_calls;return BSP_CAN_OK; }
#ifdef YLAMP_J3_REVERSE_TEST
BSP_CAN_Status_t FiveControl_ReverseJ3(void) { ++actuator_calls;return BSP_CAN_OK; }
#endif
BSP_CAN_Status_t JointCalibration_SetIMU(bool enabled,uint32_t now)
{
    (void)now;
    if (enabled) { ++sensor_starts;bmi_flags|=CAL_IMU_ACTIVE; }
    else { ++sensor_stops;bmi_flags=0; }
    return BSP_CAN_OK;
}
BSP_CAN_Status_t JointCalibration_GetStatus(uint32_t now,JointCalibration_Status_t *out)
{ (void)now;*out=(JointCalibration_Status_t){.flags=bmi_flags};return BSP_CAN_OK; }
HAL_StatusTypeDef BMI088_GetSample(BMI088_Sample_t *out)
{ *out=bmi;return bmi_flags & CAL_IMU_ACTIVE ? HAL_OK : HAL_ERROR; }
void YummyIMU_GetStatus(YummyIMU_Status_t *out) { *out=imu; }
uint32_t BSP_USB_RxEpoch(void) { return epoch; }
BSP_USB_Status_t BSP_USB_GetDiagnostics(BSP_USB_Diagnostics_t *out)
{ *out=(BSP_USB_Diagnostics_t){.configured=true,.dtr=dtr,.tx_busy=busy};return BSP_USB_OK; }
BSP_USB_Status_t BSP_USB_Send(const uint8_t *p,uint16_t n)
{
    if (!dtr) { return BSP_USB_NOT_READY; }
    if (busy) { return BSP_USB_BUSY; }
    assert(n<=sizeof output);memcpy(output,p,n);output_size=n;++writes;
    return BSP_USB_OK;
}
BSP_USB_Status_t BSP_USB_Read(uint8_t *b,uint32_t *at,uint32_t *e)
{
    *e=epoch;
    if (tail==head || !dtr) { return BSP_USB_NOT_READY; }
    *b=input[tail++];*at=test_tick;return BSP_USB_OK;
}
static uint32_t u32(const uint8_t *p)
{ return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24; }
static float scalar(const uint8_t *p)
{ uint32_t bits=u32(p);float v;memcpy(&v,&bits,4);return v; }
static void crc(uint8_t *p,unsigned n)
{
    uint16_t value=MotorTelemetry_CRC16(p+2,n-4);
    p[n-2]=(uint8_t)value;p[n-1]=(uint8_t)(value>>8);
}
static void command(uint8_t mode)
{
    uint8_t frame[16]={0xA5,0x5A,1,0x13,9,1,2,3,4,5,6,7,8,0};
    frame[13]=mode;crc(frame,sizeof frame);
    memcpy(input,frame,sizeof frame);head=sizeof frame;tail=0;
    assert(!MotorUSB_Run()); /* Response pending, never leak normal telemetry. */
    assert(!MotorUSB_Run()); /* ACK has priority over sample. */
    assert(output_size==25 && output[3]==0x93 && output[4]==18);
    assert(memcmp(output+5,frame+5,9)==0);
    assert(output[14]==(mode<=4 ? BSP_CAN_OK : BSP_CAN_INVALID_ARG));
    assert(u32(output+15)==(mode==3 || mode==4 ? MOTOR_J2_FOCUS_VERSION : MOTOR_FOCUS_VERSION) &&
           u32(output+19)==test_tick);
    assert(MotorTelemetry_CRC16(output+2,output_size-4)==
           (uint16_t)(output[output_size-2]|output[output_size-1]<<8));
}
int main(void)
{
    MotorUSB_Init();
    control.feedback[2]=(DM_Motor_Feedback_t){.position_rad=.5f,.received_at_ms=98,
        .received_count=5,.state=1};
    imu=(YummyIMU_Status_t){.sequence=7,.received_at_ms=97,.pitch_deg=4.5f,.rx_running=true};
    assert(MotorUSB_Run()); /* Original five-axis default unchanged. */
    command(2);
    unsigned before=writes;
    for (unsigned i=0;i<20;++i) { ++test_tick;assert(!MotorUSB_Run()); }
    assert(writes==before && !actuator_calls);
    control.feedback[2].received_at_ms=test_tick-2;imu.received_at_ms=test_tick-3;
    command(1);
    assert(!MotorUSB_Run() && output_size==43 && output[3]==0x94 && output[4]==36);
    assert(u32(output+5)==1 && u32(output+9)==test_tick);
    assert(u32(output+17)==5 && u32(output+25)==7);
    assert(fabsf(scalar(output+29)-28.6478898f)<.00001f && scalar(output+33)==4.5f);
    assert(output[37]==15 && output[38]==1 && output[39]==40 && output[40]==0);
    assert(MotorTelemetry_CRC16(output+2,39)==(uint16_t)(output[41]|output[42]<<8));
    printf("GOLDEN ");
    for (unsigned i=0;i<output_size;++i) { printf("%02x",output[i]); }
    puts("");
    before=writes;
    test_tick+=9;assert(!MotorUSB_Run() && writes==before);
    ++test_tick;assert(!MotorUSB_Run() && writes==before+1);
    command(2);before=writes;
    test_tick+=100;assert(!MotorUSB_Run() && writes==before);
    command(1);assert(!MotorUSB_Run());
    assert(output[37]==5); /* Both received but now stale, not fabricated zeros. */
    command(7);assert(!MotorUSB_Run()); /* Invalid command must preserve running mode. */
    control.feedback[2].received_count=0;imu.sequence=0;
    test_tick+=10;assert(!MotorUSB_Run());
    assert(output[37]==0 && output[38]==255 && isnan(scalar(output+29)) && isnan(scalar(output+33)));
    busy=true;test_tick+=10;before=writes;assert(!MotorUSB_Run() && writes==before);
    busy=false;assert(!MotorUSB_Run() && writes==before); /* No catch-up retry. */
    test_tick+=10;assert(!MotorUSB_Run() && writes==before+1);
    command(2);dtr=false;++epoch;assert(MotorUSB_Run()); /* Restores normal, no actuator. */
    dtr=true;assert(MotorUSB_Run());
    command(1);
    command(0);assert(MotorUSB_Run());
    /* Corrupted observer frame cannot change the stream profile. */
    uint8_t bad[16]={0xA5,0x5A,1,0x13,9};
    memcpy(input,bad,sizeof bad);head=sizeof bad;tail=0;
    assert(MotorUSB_Run() && !actuator_calls);
    assert(!sensor_starts && !sensor_stops);
    command(4);assert(!sensor_starts); /* Connect-to-pause never powers BMI. */
    control.feedback[1]=(DM_Motor_Feedback_t){.position_rad=2.5f,
        .received_at_ms=test_tick,.received_count=9,.state=1};
    bmi=(BMI088_Sample_t){.accel_m_s2={-1,0,1},.received_at_ms=test_tick,.sequence=3};
    command(3);assert(sensor_starts==1 && !actuator_calls);
    assert(!MotorUSB_Run() && output[37]==7); /* Fresh but not warmed: not calibratable. */
    bmi_flags|=CAL_WARM|CAL_IMU_FRESH;
    test_tick+=10;control.feedback[1].received_at_ms=bmi.received_at_ms=test_tick;
    assert(!MotorUSB_Run() && output[37]==15);
    assert(fabsf(scalar(output+29)-143.23945f)<.0001f && fabsf(scalar(output+33)-45.f)<.0001f);
    command(4);before=writes;test_tick+=20;
    assert(!MotorUSB_Run() && writes==before && sensor_stops==0);
    command(3);assert(sensor_starts==1); /* Resume does not reopen/retry the sensor. */
    command(0);assert(sensor_stops==1 && MotorUSB_Run());
    /* An externally owned session is BUSY, never stolen or stopped. */
    bmi_flags=CAL_IMU_ACTIVE;
    uint8_t held[16]={0xA5,0x5A,1,0x13,9,1,2,3,4,5,6,7,8,3};
    crc(held,sizeof held);memcpy(input,held,sizeof held);head=sizeof held;tail=0;
    assert(!MotorUSB_Run() && !MotorUSB_Run() && output[14]==BSP_CAN_BUSY);
    assert(sensor_starts==1 && sensor_stops==1 && !actuator_calls);
    bmi_flags=0;command(3);assert(sensor_starts==2);
    dtr=false;++epoch;assert(MotorUSB_Run() && sensor_stops==2 && !actuator_calls);
    puts("USB focus: both profiles/start/pause/warmup/ownership/DTR/CRC/no motor control PASS");
    return 0;
}
