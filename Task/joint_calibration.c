#include "joint_calibration.h"

#include <math.h>
#include <string.h>

static struct {
    bool initialized, filtered, resting, active, stop_requested, stop_failed;
    uint32_t sequence, first_ms, last_ms, rest_ms;
    float low_pass[3], anchor[3], gyro[3];
    float encoder_min, encoder_max;
    float zero[2];
    uint8_t valid_mask, axis;
} calibration;

static float norm(const float v[3])
{
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static float angle(const float a[3], const float b[3])
{
    const float denominator = norm(a) * norm(b);
    if (!isfinite(denominator) || denominator < 0.001f) { return INFINITY; }
    const float dot = (a[0]*b[0] + a[1]*b[1] + a[2]*b[2]) / denominator;
    return acosf(fmaxf(-1.0f, fminf(1.0f, dot)));
}

static bool motors_still(uint32_t now_ms, MotorTask_Status_t *status,
                         MotorTask_Config_t *config)
{
    if (MotorTask_GetStatus(status) != BSP_CAN_OK ||
        MotorTask_GetConfig(config) != BSP_CAN_OK ||
        (status->state != MOTOR_TASK_IDLE && status->state != MOTOR_TASK_FAULT) ||
        status->feedback_valid_mask != 3U) { return false; }
    for (unsigned i = 0U; i < 2U; ++i) {
        const DM_Motor_Feedback_t *fb = &status->feedback[i];
        if (fb->state != 0U || now_ms - fb->received_at_ms > config->feedback_max_age_ms ||
            !isfinite(fb->position_rad) || !isfinite(status->reference_rad[i]) ||
            !isfinite(fb->velocity_rad_s) ||
            status->reference_rad[i] < config->joint[i].lower_rad ||
            status->reference_rad[i] > config->joint[i].upper_rad ||
            fabsf(fb->velocity_rad_s) > g_calibration_config.motor_still_rad_s) {
            return false;
        }
    }
    return true;
}

static float tilt(uint8_t axis)
{
    if (axis < 1U || axis > 6U || !calibration.filtered) { return NAN; }
    float up[3] = {0};
    up[(axis - 1U) / 2U] = (axis & 1U) != 0U ? 1.0f : -1.0f;
    /* A stationary accelerometer measures specific force, pointing UP. */
    return angle(calibration.low_pass, up);
}

BSP_CAN_Status_t JointCalibration_Init(void)
{
    if (__get_IPSR() != 0U) { return BSP_CAN_INVALID_ARG; }
    if (calibration.initialized) { return BSP_CAN_BUSY; }
    MotorTask_Config_t config;
    const BSP_CAN_Status_t result = MotorTask_GetConfig(&config);
    if (result != BSP_CAN_OK) { return result; }
    calibration.zero[0] = 0.5f * (config.joint[0].lower_rad + config.joint[0].upper_rad);
    const bool saved = config.drag_reference_confirmed == 1U &&
        isfinite(config.drag_j2_zero_rad);
    calibration.zero[1] = saved ? config.drag_j2_zero_rad : NAN;
    calibration.valid_mask = saved ? 3U : 1U;
    /* axis remains unknown: a saved manual reference is not an IMU measurement. */
    calibration.initialized = true;
    return BSP_CAN_OK;
}

void JointCalibration_Run(uint32_t now_ms)
{
    if (!calibration.initialized || __get_IPSR() != 0U) { return; }
    if (calibration.stop_requested) {
        const HAL_StatusTypeDef result = BMI088_SetEnabled(false, now_ms);
        if (result == HAL_BUSY) { return; }
        calibration.stop_requested = false;
        calibration.stop_failed = result != HAL_OK;
        if (result == HAL_OK) { calibration.active = false; }
        return; /* Failure is reported, not automatically retried/reset. */
    }
    if (!calibration.active || calibration.stop_failed) { return; }
    const JointCalibration_Config_t *cfg = &g_calibration_config;
    BMI088_Sample_t sample;
    if (BMI088_GetSample(&sample) != HAL_OK ||
        now_ms - sample.received_at_ms > cfg->max_age_ms) {
        calibration.resting = false;
        calibration.filtered = false;
        return;
    }
    if (sample.sequence == calibration.sequence) { return; }
    float accel[3];
    for (unsigned i = 0U; i < 3U; ++i) {
        accel[i] = (sample.accel_m_s2[i] - cfg->accel_bias_m_s2[i]) * cfg->accel_scale[i];
        calibration.gyro[i] = sample.gyro_rad_s[i] - cfg->gyro_bias_rad_s[i];
        if (!isfinite(accel[i]) || !isfinite(calibration.gyro[i])) {
            calibration.resting = calibration.filtered = false;
            return;
        }
    }
    const float magnitude = norm(accel);
    if (magnitude < 0.1f) {
        calibration.resting = calibration.filtered = false;
        return;
    }
    const uint32_t gap = sample.received_at_ms - calibration.last_ms;
    if (!calibration.filtered || gap == 0U || gap > cfg->max_age_ms) {
        memcpy(calibration.low_pass, accel, sizeof(accel));
        calibration.first_ms = sample.received_at_ms;
        calibration.resting = false;
        calibration.filtered = true;
    } else {
        const float dt = (float)gap * 0.001f;
        const float alpha = dt / (cfg->accel_tau_s + dt);
        for (unsigned i = 0U; i < 3U; ++i) {
            calibration.low_pass[i] += alpha * (accel[i] - calibration.low_pass[i]);
        }
    }
    calibration.last_ms = sample.received_at_ms;
    calibration.sequence = sample.sequence;
    MotorTask_Status_t status;
    MotorTask_Config_t config;
    const bool good = fabsf(magnitude - cfg->gravity_m_s2) <= cfg->gravity_tolerance_m_s2 &&
        norm(calibration.gyro) <= cfg->gyro_still_rad_s &&
        angle(accel, calibration.low_pass) <= cfg->accel_residual_rad &&
        motors_still(now_ms, &status, &config);
    if (!good) {
        calibration.resting = false;
        return;
    }
    const float encoder = status.reference_rad[1];
    if (calibration.resting) {
        calibration.encoder_min = fminf(calibration.encoder_min, encoder);
        calibration.encoder_max = fmaxf(calibration.encoder_max, encoder);
        if (angle(calibration.low_pass, calibration.anchor) > cfg->gravity_drift_rad ||
            calibration.encoder_max - calibration.encoder_min > cfg->encoder_span_rad) {
            calibration.resting = false;
        }
    }
    if (!calibration.resting) {
        calibration.resting = true;
        calibration.rest_ms = sample.received_at_ms;
        calibration.encoder_min = calibration.encoder_max = encoder;
        memcpy(calibration.anchor, calibration.low_pass, sizeof(calibration.anchor));
    }
}

BSP_CAN_Status_t JointCalibration_GetStatus(uint32_t now_ms,
                                           JointCalibration_Status_t *status)
{
    if (status == NULL || __get_IPSR() != 0U) { return BSP_CAN_INVALID_ARG; }
    if (!calibration.initialized) { return BSP_CAN_NOT_READY; }
    memset(status, 0, sizeof(*status));
    status->raw_rad[0] = status->raw_rad[1] = NAN;
    status->reference_rad[0] = status->reference_rad[1] = NAN;
    if (calibration.active) { status->flags |= CAL_IMU_ACTIVE; }
    if (calibration.stop_requested) { status->flags |= CAL_IMU_STOPPING; }
    if (calibration.stop_failed ||
        (!calibration.active && !BMI088_IsSuspended())) {
        status->flags |= CAL_IMU_STOP_FAILED;
    }
    status->valid_mask = calibration.valid_mask;
    status->axis = calibration.axis;
    memcpy(status->zero_rad, calibration.zero, sizeof(status->zero_rad));
    memcpy(status->accel_m_s2, calibration.low_pass, sizeof(status->accel_m_s2));
    memcpy(status->gyro_rad_s, calibration.gyro, sizeof(status->gyro_rad_s));
    status->vertical_error_rad = tilt(calibration.axis);
    status->imu_age_ms = now_ms - calibration.last_ms;
    BMI088_Sample_t sample;
    if (calibration.active && !calibration.stop_requested && !calibration.stop_failed &&
        calibration.filtered && BMI088_GetSample(&sample) == HAL_OK &&
        status->imu_age_ms <= g_calibration_config.max_age_ms) {
        status->flags |= CAL_IMU_FRESH;
        if (calibration.last_ms - calibration.first_ms >= g_calibration_config.warmup_ms) {
            status->flags |= CAL_WARM;
        }
        if (calibration.resting) {
            status->stable_ms = calibration.last_ms - calibration.rest_ms;
            if (status->stable_ms >= g_calibration_config.stable_ms) {
                status->flags |= CAL_STABLE;
            }
        }
    }
    MotorTask_Status_t motor;
    MotorTask_Config_t config;
    if (motors_still(now_ms, &motor, &config)) { status->flags |= CAL_MOTORS_STILL_DISABLED; }
    if (MotorTask_GetStatus(&motor) == BSP_CAN_OK &&
        MotorTask_GetConfig(&config) == BSP_CAN_OK) {
        for (unsigned i = 0U; i < 2U; ++i) {
            status->raw_rad[i] = (motor.feedback_valid_mask & (1U << i)) != 0U &&
                now_ms - motor.feedback[i].received_at_ms <= config.feedback_max_age_ms ?
                motor.feedback[i].position_rad : NAN;
            if (isfinite(status->raw_rad[i])) {
                status->reference_rad[i] = motor.reference_rad[i];
            }
        }
    }
    return BSP_CAN_OK;
}

BSP_CAN_Status_t JointCalibration_SetIMU(bool enabled, uint32_t now_ms)
{
    if (__get_IPSR() != 0U) { return BSP_CAN_INVALID_ARG; }
    if (!calibration.initialized) { return BSP_CAN_NOT_READY; }
    if (!enabled) {
        if (calibration.stop_failed) { return BSP_CAN_ERROR; }
        if (calibration.active) { calibration.stop_requested = true; }
        calibration.filtered = calibration.resting = false;
        return BSP_CAN_OK;
    }
    if (calibration.active || calibration.stop_requested || calibration.stop_failed) {
        return BSP_CAN_BUSY;
    }
    MotorTask_Status_t motor;
    /* Sampling is read-only and need not depend on CAN health. Capturing a zero
     * still uses motors_still(), including fresh disabled feedback. */
    if (MotorTask_GetStatus(&motor) != BSP_CAN_OK ||
        (motor.state != MOTOR_TASK_IDLE && motor.state != MOTOR_TASK_FAULT)) {
        return BSP_CAN_BUSY;
    }
    const HAL_StatusTypeDef result = BMI088_SetEnabled(true, now_ms);
    if (result != HAL_OK) { return result == HAL_BUSY ? BSP_CAN_BUSY : BSP_CAN_NOT_READY; }
    calibration.active = true;
    calibration.filtered = calibration.resting = false;
    calibration.sequence = 0U;
    return BSP_CAN_OK;
}

BSP_CAN_Status_t JointCalibration_Mark(uint8_t joint, uint8_t axis, uint32_t now_ms)
{
    if (__get_IPSR() != 0U || joint < 1U || joint > 2U ||
        (joint == 2U && (axis < 1U || axis > 6U))) { return BSP_CAN_INVALID_ARG; }
    if (!calibration.initialized) { return BSP_CAN_NOT_READY; }
    MotorTask_Status_t motor;
    MotorTask_Config_t config;
    if (!motors_still(now_ms, &motor, &config)) { return BSP_CAN_BUSY; }
    if (joint == 1U) {
        calibration.zero[0] = 0.5f * (config.joint[0].lower_rad + config.joint[0].upper_rad);
        calibration.valid_mask |= 1U;
        return BSP_CAN_OK;
    }
    JointCalibration_Status_t status;
    if (JointCalibration_GetStatus(now_ms, &status) != BSP_CAN_OK ||
        status.flags != (CAL_IMU_FRESH | CAL_WARM | CAL_STABLE |
                         CAL_MOTORS_STILL_DISABLED | CAL_IMU_ACTIVE)) {
        return BSP_CAN_NOT_READY;
    }
    const float error = tilt(axis);
    if (!isfinite(error) || error > g_calibration_config.vertical_tolerance_rad) {
        return BSP_CAN_INVALID_ARG;
    }
    calibration.zero[1] = motor.reference_rad[1];
    calibration.axis = axis;
    calibration.valid_mask |= 2U;
    (void)JointCalibration_SetIMU(false, now_ms);
    return BSP_CAN_OK;
}
