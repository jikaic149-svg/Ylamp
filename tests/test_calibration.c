#include "joint_calibration.h"
#include "motor_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define DEG(x) ((x) * 0.01745329251994329577f)
uint32_t test_ipsr, test_primask, test_tick;
FDCAN_HandleTypeDef hfdcan1, hfdcan2;
static MotorTask_Config_t config;
static MotorTask_Status_t motor;
static BMI088_Sample_t sample;
static bool enabled, available = true, stopping_busy;
static unsigned starts, stops;

BSP_CAN_Status_t MotorTask_GetConfig(MotorTask_Config_t *out)
{
    *out = config;
    return BSP_CAN_OK;
}
BSP_CAN_Status_t MotorTask_GetStatus(MotorTask_Status_t *out)
{
    *out = motor;
    return BSP_CAN_OK;
}
HAL_StatusTypeDef BMI088_GetSample(BMI088_Sample_t *out)
{
    if (!enabled || !available || sample.sequence == 0U) { return HAL_ERROR; }
    *out = sample;
    return HAL_OK;
}
bool BMI088_IsSuspended(void) { return available && !enabled; }
HAL_StatusTypeDef BMI088_SetEnabled(bool on, uint32_t now)
{
    (void)now;
    if (!on && stopping_busy) { return HAL_BUSY; }
    if (!available) { return HAL_ERROR; }
    if (on) { ++starts; sample.sequence = 0U; }
    else { ++stops; }
    enabled = on;
    return HAL_OK;
}

static void feed(uint32_t now, float x, float y, float z)
{
    sample.accel_m_s2[0] = x;
    sample.accel_m_s2[1] = y;
    sample.accel_m_s2[2] = z;
    sample.received_at_ms = now;
    ++sample.sequence;
    motor.feedback[0].received_at_ms = now;
    motor.feedback[1].received_at_ms = now;
    JointCalibration_Run(now);
}

int main(int argc, char **argv)
{
    config.feedback_max_age_ms = 100U;
    config.joint[0].lower_rad = DEG(-210);
    config.joint[0].upper_rad = DEG(0);
    config.joint[1].lower_rad = DEG(-200);
    config.joint[1].upper_rad = DEG(-60);
    const bool saved = argc == 2 && strcmp(argv[1], "saved") == 0;
    if (saved) {
        config.drag_reference_confirmed = 1U;
        config.drag_j2_zero_rad = -2.640383005142212f;
    }
    motor.state = MOTOR_TASK_IDLE;
    motor.feedback_valid_mask = 3U;
    motor.feedback[0].position_rad = DEG(260); /* Calibration preserves raw turns. */
    motor.feedback[1].position_rad = DEG(210); /* Same posture, +360 wire branch. */
    motor.reference_rad[0] = DEG(-100);
    motor.reference_rad[1] = DEG(-150);
    JointCalibration_Status_t status;
    assert(JointCalibration_Init() == BSP_CAN_OK);
    assert(JointCalibration_GetStatus(0U, &status) == BSP_CAN_OK);
    assert(status.zero_rad[0] == g_joint_reference_config[0].zero_rad);
    assert(status.raw_rad[0] == DEG(260) && status.reference_rad[0] == status.raw_rad[0]);
    if (saved) {
        assert(status.valid_mask == 3U && status.axis == 0U);
        assert(status.zero_rad[1] == config.drag_j2_zero_rad);
        assert(isnan(status.vertical_error_rad));
        assert(!(status.flags & (CAL_IMU_ACTIVE | CAL_IMU_FRESH | CAL_STABLE)));
        assert(!enabled && starts == 0U && stops == 0U);
        motor.feedback_valid_mask = 0U; /* Disconnected CAN must not prevent read-only IMU. */
        motor.state = MOTOR_TASK_RUNNING;
        assert(JointCalibration_SetIMU(true, 0U) == BSP_CAN_BUSY);
        motor.state = MOTOR_TASK_HOLDING;
        assert(JointCalibration_SetIMU(true, 0U) == BSP_CAN_BUSY);
        motor.state = MOTOR_TASK_IDLE;
        assert(JointCalibration_SetIMU(true, 0U) == BSP_CAN_OK);
        for (uint32_t ms = 10U; ms <= 6100U; ms += 10U) {
            feed(ms, 0, 0, 9.80665f);
        }
        assert(JointCalibration_GetStatus(6100U, &status) == BSP_CAN_OK);
        assert(status.flags == (CAL_IMU_ACTIVE | CAL_IMU_FRESH | CAL_WARM));
        assert(JointCalibration_Mark(2U, 5U, 6100U) == BSP_CAN_BUSY);
        assert(status.zero_rad[1] == config.drag_j2_zero_rad && status.axis == 0U);
        assert(JointCalibration_SetIMU(false, 6100U) == BSP_CAN_OK);
        JointCalibration_Run(6101U);
        assert(!enabled && starts == 1U && stops == 1U);
        puts("PASS: restore manual reference, explicit IMU without CAN, no false zero approval.");
        return 0;
    }
    assert(status.valid_mask == 1U && isnan(status.zero_rad[1]));
    assert(!(status.flags & CAL_IMU_ACTIVE) && starts == 0U);
    assert(JointCalibration_Mark(2U, 5U, 0U) == BSP_CAN_NOT_READY);
    motor.state = MOTOR_TASK_RUNNING;
    assert(JointCalibration_SetIMU(true, 0U) == BSP_CAN_BUSY);
    motor.state = MOTOR_TASK_IDLE;
    assert(JointCalibration_SetIMU(true, 0U) == BSP_CAN_OK);
    assert(starts == 1U);
    for (uint32_t ms = 10U; ms <= 6100U; ms += 10U) {
        feed(ms, 0.0f, 0.0f, 9.80665f);
    }
    assert(JointCalibration_GetStatus(6100U, &status) == BSP_CAN_OK);
    assert(status.flags == 31U);
    assert(JointCalibration_Mark(2U, 6U, 6100U) == BSP_CAN_INVALID_ARG);
    sample.gyro_rad_s[0] = DEG(4);
    feed(6110U, 0, 0, 9.80665f);
    assert(JointCalibration_Mark(2U, 5U, 6110U) == BSP_CAN_NOT_READY);
    sample.gyro_rad_s[0] = 0;
    for (uint32_t ms = 6120U; ms <= 8500U; ms += 10U) {
        feed(ms, 0, 0, 9.80665f);
    }
    assert(JointCalibration_Mark(2U, 5U, 8500U) == BSP_CAN_OK);
    assert(enabled && stops == 0U); /* Drain, not premature hardware-off claim. */
    stopping_busy = true;
    JointCalibration_Run(8501U);
    JointCalibration_GetStatus(8501U, &status);
    assert((status.flags & CAL_IMU_STOPPING) && !(status.flags & CAL_IMU_FRESH));
    stopping_busy = false;
    JointCalibration_Run(8502U);
    JointCalibration_GetStatus(8502U, &status);
    assert(!enabled && stops == 1U && status.valid_mask == 3U);
    assert(!(status.flags & (CAL_IMU_ACTIVE | CAL_IMU_STOPPING)));
    assert(fabsf(status.zero_rad[1] - DEG(-150)) < 1e-6f);
    assert(fabsf(motor.feedback[1].position_rad - DEG(210)) < 1e-6f);
    assert(fabsf(status.reference_rad[1] - DEG(-150)) < 1e-6f);
    for (uint32_t ms = 8510U; ms < 9000U; ms += 10U) {
        JointCalibration_Run(ms);
    }
    assert(starts == 1U); /* Never reopen automatically after capture. */
    motor.feedback[0].received_at_ms = motor.feedback[1].received_at_ms = 9000U;
    assert(JointCalibration_SetIMU(true, 9000U) == BSP_CAN_OK);
    for (uint32_t ms = 9010U; ms <= 16000U; ms += 10U) {
        feed(ms, 9.80665f * sinf(DEG(5)), 0, 9.80665f * cosf(DEG(5)));
    }
    assert(JointCalibration_Mark(2U, 5U, 16000U) == BSP_CAN_INVALID_ARG);
    assert(JointCalibration_Mark(2U, 0U, 16000U) == BSP_CAN_INVALID_ARG);
    JointCalibration_Run(16050U);
    JointCalibration_GetStatus(16050U, &status);
    assert(!(status.flags & (CAL_IMU_FRESH | CAL_STABLE)));
    config.joint[0].lower_rad = DEG(-200);
    assert(JointCalibration_GetStatus(16000U, &status) == BSP_CAN_OK);
    assert(status.zero_rad[0] == g_joint_reference_config[0].zero_rad);
    assert(JointCalibration_Mark(1U, 0U, 16000U) == BSP_CAN_OK);
    JointCalibration_GetStatus(16000U, &status);
    assert(status.zero_rad[0] == g_joint_reference_config[0].zero_rad);
    test_ipsr = 1U;
    assert(JointCalibration_SetIMU(true, 16000U) == BSP_CAN_INVALID_ARG);
    assert(JointCalibration_Mark(1U, 0U, 16000U) == BSP_CAN_INVALID_ARG);
    test_ipsr = 0U;
    assert(JointCalibration_SetIMU(false, 16000U) == BSP_CAN_OK);
    JointCalibration_Run(16000U);
    available = false;
    assert(JointCalibration_SetIMU(true, 16000U) == BSP_CAN_NOT_READY);
    puts("PASS: modeling-only midpoint/upward zero, rest/warm/tilt/fresh gates,"
         " explicit sessions, drain-before-suspend, no auto-reopen.");
}
