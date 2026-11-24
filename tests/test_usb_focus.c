#include "motor_usb.h"
#include "bsp_usb.h"
#include "five_control.h"
#include "joint_calibration.h"
#include "motor_telemetry.h"
#include "yummy_imu.h"
#include "five_history.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

uint32_t test_ipsr, test_primask, test_tick=100;
FDCAN_HandleTypeDef hfdcan1, hfdcan2;
static uint8_t input[256], output[512];
static unsigned head, tail, output_size, writes, actuator_calls;
static unsigned gravity_calls, release_calls, impedance_calls, impedance_test_calls;
static unsigned speed_joint;
static float speed_value;
static uint32_t speed_duration, speed_ramp;
static unsigned sensor_starts, sensor_stops;
static uint32_t epoch;
static bool dtr=true, busy;
static FiveControl_Status control;
static YummyIMU_Status_t imu;
static BMI088_Sample_t bmi;
static uint8_t bmi_flags;
uint32_t HAL_GetTick(void) { return test_tick; }
const FiveControl_Status *FiveControl_StatusNow(void) { return &control; }
const FiveHistory_Status *FiveHistory_StatusNow(void)
{
    static const FiveHistory_Status history={.flags=FIVE_HISTORY_READY|FIVE_HISTORY_LOADED,
        .sequence=7,.writes=3,.saved_mask=31,.restored_mask=5,
        .saved_rad={0,1,2,3,4},.step_deg={150,150,150,150,150}};
    return &history;
}
void FiveControl_Gravity(void) { ++actuator_calls; ++gravity_calls; }
void FiveControl_Release(void) { ++actuator_calls; ++release_calls; }
BSP_CAN_Status_t FiveControl_ColdStart(void) { ++actuator_calls;return BSP_CAN_OK; }
BSP_CAN_Status_t FiveControl_Acknowledge(void) { ++actuator_calls;return BSP_CAN_OK; }
BSP_CAN_Status_t FiveControl_ReadLegacyJ3(void) { ++actuator_calls;return BSP_CAN_OK; }
BSP_CAN_Status_t FiveControl_Drag(void) { ++actuator_calls;return BSP_CAN_OK; }
BSP_CAN_Status_t FiveControl_Impedance(void) { ++actuator_calls;++impedance_calls;return BSP_CAN_OK; }
BSP_CAN_Status_t FiveControl_ImpedanceTest(unsigned joint,float offset)
{
    assert(joint==4 && offset==1.f);
    control.impedance_test_duration_ms=24000;
    ++actuator_calls;++impedance_test_calls;return BSP_CAN_OK;
}
BSP_CAN_Status_t FiveControl_Move(const double q[5],uint32_t ms)
{ (void)q;(void)ms;++actuator_calls;return BSP_CAN_OK; }
BSP_CAN_Status_t FiveControl_Speed(unsigned joint,float speed,uint32_t ms,uint32_t ramp)
{
    speed_joint=joint;speed_value=speed;speed_duration=ms;speed_ramp=ramp;
    ++actuator_calls;return BSP_CAN_OK;
}
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
static void set32(uint8_t *p,uint32_t value)
{ for (unsigned i=0;i<4;++i) { p[i]=(uint8_t)(value>>(8*i)); } }
static unsigned reliable(unsigned id,unsigned scope,unsigned field,uint32_t value)
{
    uint8_t frame[31]={0xA5,0x5A,1,0x11,24,1,2,3,4,5,6,7,8};
    set32(frame+13,id);set32(frame+17,test_tick);
    frame[21]=(uint8_t)id;frame[22]=16;frame[23]=(uint8_t)scope;frame[24]=(uint8_t)field;
    set32(frame+25,value);crc(frame,sizeof frame);
    memcpy(input,frame,sizeof frame);head=sizeof frame;tail=0;
    assert(!MotorUSB_Run() && !MotorUSB_Run());
    assert(output_size==27 && output[3]==0x91 && u32(output+13)==id);
    return output[18];
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
    assert(output[14]==(mode<=2 ? BSP_CAN_OK : BSP_CAN_INVALID_ARG));
    assert(u32(output+15)==MOTOR_FOCUS_VERSION && u32(output+19)==test_tick);
    assert(MotorTelemetry_CRC16(output+2,output_size-4)==
           (uint16_t)(output[output_size-2]|output[output_size-1]<<8));
}
static const uint8_t *motor(unsigned i) { return output+19+i*32; }
static const uint8_t *sensor(unsigned i) { return output+179+i*16; }
static void refresh_sources(void)
{
    for (unsigned i=0;i<5;++i) { control.feedback[i].received_at_ms=test_tick-2; }
    bmi.received_at_ms=test_tick-1;imu.received_at_ms=test_tick-3;
}
static void check_sample(void)
{
    assert(output_size==213 && output[3]==0x94 && output[4]==206);
    assert(u32(output+9)==test_tick);
    assert(output[13]==5 && output[14]==2 && output[15]==32 && output[16]==16);
    assert(output[17]==40 && output[18]==0);
    assert(MotorTelemetry_CRC16(output+2,209)==(uint16_t)(output[211]|output[212]<<8));
    for (unsigned i=0;i<5;++i) {
        const uint8_t *r=motor(i);
        assert(r[0]==i+1 && r[1]==(i<2 ? 2 : 1) && r[2]==(i==2 ? 1 : 0));
        assert(r[4]==i+1 && r[6]==0x11+i);
    }
    assert(sensor(0)[0]==1 && sensor(1)[0]==2);
}
int main(void)
{
    MotorUSB_Init();
    for (unsigned i=0;i<5;++i) {
        control.feedback[i]=(DM_Motor_Feedback_t){.position_rad=(i+1)*.25f,
            .received_at_ms=98,.received_count=i+11,.state=1};
    }
    imu=(YummyIMU_Status_t){.sequence=7,.pitch_deg=4.5f,.rx_running=true};
    bmi=(BMI088_Sample_t){.accel_m_s2={-1,0,1},.sequence=3};
    assert(MotorUSB_Run());
    command(2); /* Connect-to-pause does not start sensors or motors. */
    unsigned before=writes;
    for (unsigned i=0;i<20;++i) { ++test_tick;assert(!MotorUSB_Run()); }
    assert(writes==before && !sensor_starts && !actuator_calls);
    refresh_sources();command(1);
    assert(sensor_starts==1 && !MotorUSB_Run());
    check_sample();
    for (unsigned i=0;i<5;++i) {
        assert(motor(i)[3]==3 && scalar(motor(i)+16)==(i+1)*.25f);
        assert(u32(motor(i)+12)==i+11);
    }
    assert(sensor(0)[1]==1 && scalar(sensor(0)+12)==45.f); /* BMI warmup is local. */
    assert(sensor(1)[1]==3 && scalar(sensor(1)+12)==4.5f);
    bmi_flags|=CAL_WARM|CAL_IMU_FRESH;
    test_tick+=10;refresh_sources();assert(!MotorUSB_Run());
    check_sample();assert(sensor(0)[1]==3 && sensor(1)[1]==3);
    printf("GOLDEN ");
    for (unsigned i=0;i<output_size;++i) { printf("%02x",output[i]); }
    puts("");
    before=writes;
    test_tick+=9;assert(!MotorUSB_Run() && writes==before);
    ++test_tick;assert(!MotorUSB_Run() && writes==before+1);
    command(2);before=writes;test_tick+=100;
    assert(!MotorUSB_Run() && writes==before && !sensor_stops);
    command(1);assert(sensor_starts==1 && !MotorUSB_Run());
    assert(motor(1)[3]==1 && motor(2)[3]==1 && sensor(0)[1]==1 && sensor(1)[1]==1);
    command(3);command(4);command(7); /* Retired profiles/invalid requests preserve run. */
    test_tick+=10;refresh_sources();
    control.feedback[2].received_count=0;imu.sequence=0;
    assert(!MotorUSB_Run());check_sample();
    assert(motor(1)[3]==3 && motor(2)[3]==0 && motor(2)[28]==255);
    assert(isnan(scalar(motor(2)+16)) && sensor(0)[1]==3 &&
           sensor(1)[1]==0 && isnan(scalar(sensor(1)+12)));
    busy=true;test_tick+=10;before=writes;assert(!MotorUSB_Run() && writes==before);
    busy=false;assert(!MotorUSB_Run() && writes==before); /* No catch-up retry. */
    test_tick+=10;assert(!MotorUSB_Run() && writes==before+1);
    command(2);dtr=false;++epoch;assert(MotorUSB_Run());
    assert(sensor_stops==1 && !actuator_calls);
    dtr=true;assert(MotorUSB_Run());
    command(2);assert(sensor_starts==1);
    command(1);assert(sensor_starts==2);
    command(0);assert(MotorUSB_Run() && sensor_stops==2);
    uint8_t bad[16]={0xA5,0x5A,1,0x13,9};
    memcpy(input,bad,sizeof bad);head=sizeof bad;tail=0;
    assert(MotorUSB_Run() && !actuator_calls);
    /* An external BMI session is not stolen or stopped. */
    bmi_flags=CAL_IMU_ACTIVE;
    uint8_t held[16]={0xA5,0x5A,1,0x13,9,1,2,3,4,5,6,7,8,1};
    crc(held,sizeof held);memcpy(input,held,sizeof held);head=sizeof held;tail=0;
    assert(!MotorUSB_Run() && !MotorUSB_Run() && output[14]==BSP_CAN_BUSY);
    assert(sensor_starts==2 && sensor_stops==2 && !actuator_calls);
    bmi_flags=0;command(1);assert(sensor_starts==3);
    dtr=false;++epoch;assert(MotorUSB_Run() && sensor_stops==3 && !actuator_calls);
    /* Velocity trials use the reliable session, not observation mode.
     * Missing/expired stages cannot execute; replay gets cached ACK only. */
    dtr=true;MotorUSB_Init();
    uint8_t session[15]={0xA5,0x5A,1,0x12,8,1,2,3,4,5,6,7,8};
    crc(session,sizeof session);
    memcpy(input,session,sizeof session);head=sizeof session;tail=0;
    assert(!MotorUSB_Run() && !MotorUSB_Run() && !actuator_calls);
    assert(reliable(1,0,252,0)==BSP_CAN_OK && u32(output+21)==MOTOR_CRUISE_VERSION);
    assert(reliable(2,0,13,5000)==BSP_CAN_NOT_READY && !actuator_calls);
    const float speed=-.08f;uint32_t speed_bits;memcpy(&speed_bits,&speed,4);
    assert(reliable(3,4,12,speed_bits)==BSP_CAN_OK);
    assert(reliable(4,0,14,1000)==BSP_CAN_OK);
    assert(reliable(5,0,13,5000)==BSP_CAN_OK && actuator_calls==1);
    assert(speed_joint==4 && speed_value==speed && speed_duration==5000 && speed_ramp==1000);
    assert(reliable(5,0,13,5000)==BSP_CAN_OK && actuator_calls==1);
    assert(reliable(6,0,13,5000)==BSP_CAN_NOT_READY && actuator_calls==1);
    assert(reliable(7,4,12,speed_bits)==BSP_CAN_OK);
    assert(reliable(8,0,14,1000)==BSP_CAN_OK);
    test_tick+=2001;
    assert(reliable(9,0,13,5000)==BSP_CAN_NOT_READY && actuator_calls==1);
    assert(reliable(10,4,12,speed_bits)==BSP_CAN_OK);
    assert(reliable(11,0,14,1000)==BSP_CAN_OK);
    assert(reliable(12,0,7,0)==BSP_CAN_OK && actuator_calls==2);
    assert(reliable(13,0,13,5000)==BSP_CAN_NOT_READY && actuator_calls==2);
    assert(reliable(14,0,251,0)==BSP_CAN_OK && u32(output+21)==MOTOR_DRAG_VERSION);
    assert(reliable(15,0,15,0)==BSP_CAN_INVALID_ARG && actuator_calls==2);
    assert(reliable(16,1,15,MOTOR_DRAG_REQUEST)==BSP_CAN_INVALID_ARG && actuator_calls==2);
    assert(reliable(17,0,15,MOTOR_DRAG_REQUEST)==BSP_CAN_OK && actuator_calls==3);
    assert(reliable(17,0,15,MOTOR_DRAG_REQUEST)==BSP_CAN_OK && actuator_calls==3);
    assert(reliable(18,0,250,0)==BSP_CAN_OK && u32(output+21)==MOTOR_HISTORY_VERSION);
    assert(reliable(19,0,16,0)==BSP_CAN_OK &&
        u32(output+21)==(FIVE_HISTORY_READY|FIVE_HISTORY_LOADED));
    assert(reliable(20,0,20,0)==BSP_CAN_OK && u32(output+21)==(31U|(5U<<8)));
    control.fresh=31;control.reference_rad[0]=-.1f;
    assert(reliable(21,1,20,0)==BSP_CAN_OK && u32(output+21)==0xffffffffU);
    assert(reliable(22,1,21,0)==BSP_CAN_OK && scalar(output+21)>6.f);
    assert(reliable(23,1,18,0)==BSP_CAN_OK && scalar(output+21)==150.f);
    assert(actuator_calls==3); /* History/turn diagnostics are read only. */
    assert(reliable(24,0,249,0)==BSP_CAN_OK && u32(output+21)==MOTOR_IMPEDANCE_TEST_VERSION);
    assert(reliable(25,0,22,0)==BSP_CAN_INVALID_ARG && actuator_calls==3);
    assert(reliable(26,1,22,MOTOR_IMPEDANCE_REQUEST)==BSP_CAN_INVALID_ARG && actuator_calls==3);
    assert(reliable(27,0,22,MOTOR_IMPEDANCE_REQUEST)==BSP_CAN_OK && impedance_calls==1);
    assert(reliable(27,0,22,MOTOR_IMPEDANCE_REQUEST)==BSP_CAN_OK && impedance_calls==1);
    const float offset=1.f;uint32_t offset_bits;memcpy(&offset_bits,&offset,4);
    assert(reliable(28,0,23,offset_bits)==BSP_CAN_INVALID_ARG && !impedance_test_calls);
    assert(reliable(29,4,23,offset_bits)==BSP_CAN_OK && u32(output+21)==24000);
    assert(reliable(29,4,23,offset_bits)==BSP_CAN_OK && impedance_test_calls==1);
    control.phase=FIVE_IMPEDANCE;control.impedance_test_joint=4;
    control.impedance_test_started_ms=1234;control.impedance_target_raw_rad[3]=3.f;
    assert(reliable(30,0,24,0)==BSP_CAN_OK && u32(output+21)==4);
    assert(reliable(31,0,25,0)==BSP_CAN_OK && u32(output+21)==1234);
    assert(reliable(32,4,24,0)==BSP_CAN_OK && scalar(output+21)==3.f);
    control.phase=FIVE_DRAG;
    assert(reliable(33,4,24,0)==BSP_CAN_INVALID_ARG && actuator_calls==5);
    dtr=false;++epoch;assert(MotorUSB_Run() && actuator_calls==6);
    assert(gravity_calls==1 && release_calls==1); /* Session exit != pure-G selection. */
    puts("USB unified: five motors/both Pitches/start/pause/warmup/ownership/DTR/CRC/no motor control PASS");
    return 0;
}
