#include "bmi088_port.h"
#include "bsp_key.h"
#include "bsp_rgb.h"
#include "fault.h"
#include "five_control.h"
#include "joint_calibration.h"
#include "lookaround_task.h"
#include "motor_usb.h"
#include "usb_telemetry.h"
#include "yummy_imu.h"
#include <assert.h>

static FiveControl_Status status;
static bool local;
static uint32_t now, key_at;
static unsigned updates;
static uint8_t rgb[3];
volatile uint32_t g_fault_stage;

void Ylamp_ControlStep(uint32_t now_ms);
uint32_t HAL_GetTick(void) { return now; }
void FiveControl_Tick(uint32_t ms) { (void)ms; }
const FiveControl_Status *FiveControl_StatusNow(void) { return &status; }
bool FiveControl_LocalTrajectoryOwned(void) { return local; }
BSP_KEY_Event_t BSP_KEY_Scan(uint32_t ms)
{
    return ms==key_at ? BSP_KEY_EVENT_PRESSED : BSP_KEY_EVENT_NONE;
}
BSP_CAN_Status_t LookaroundTask_Run(bool key_pressed)
{
    if (key_pressed) { local=false; } /* Cancel after the current motor tick. */
    return BSP_CAN_OK;
}
void BMI088_Run(uint32_t ms) { (void)ms; }
void YummyIMU_Run(void) {}
void JointCalibration_Run(uint32_t ms) { (void)ms; }
bool MotorUSB_Run(void) { return false; }
void USBTelemetry_Run(uint32_t ms) { (void)ms; }
HAL_StatusTypeDef BSP_RGB_Set(uint8_t r, uint8_t g, uint8_t b)
{
    ++updates;
    rgb[0]=r; rgb[1]=g; rgb[2]=b;
    return HAL_OK;
}

static void expect(FivePhase phase, bool owned, uint8_t r, uint8_t g, uint8_t b)
{
    status.phase=phase;
    local=owned;
    const unsigned before=updates;
    Ylamp_ControlStep(++now);
    assert(updates==before);
    now+=499U;
    Ylamp_ControlStep(now);
    assert(updates==before+1U && rgb[0]==r && rgb[1]==g && rgb[2]==b);
}

int main(void)
{
    expect(FIVE_WAIT,false,0,0,8);
    expect(FIVE_IDLE,false,0,8,0);
    expect(FIVE_DRAG,false,0,8,8);
    expect(FIVE_DRAG,true,0,8,8); /* Accepted intent, still planning. */
    expect(FIVE_IMPEDANCE,false,8,8,0);
    expect(FIVE_MOTION,false,6,6,6);
    expect(FIVE_POSITION,false,12,3,0);
    expect(FIVE_MOTION,true,8,0,8);
    key_at=now+500U;
    expect(FIVE_MOTION,true,8,0,8); /* KEY cancel at refresh: no false white. */
    assert(!local);
    expect(FIVE_IDLE,false,0,8,0);
    expect(FIVE_DEGRADED,false,8,0,0);
    status.fault=FIVE_BAD_FEEDBACK;
    expect(FIVE_MOTION,true,8,0,0); /* Latched fault overrides mode and owner. */
}
