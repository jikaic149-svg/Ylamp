#include "calibration_config.h"

#define DEG(x) ((x) * 0.01745329251994329577f)

const JointCalibration_Config_t g_calibration_config = {
    .warmup_ms = 5000U, .stable_ms = 2000U, .max_age_ms = 40U,
    .accel_tau_s = 0.5f,
    .gravity_m_s2 = 9.80665f, .gravity_tolerance_m_s2 = 0.5f,
    .gyro_still_rad_s = DEG(2.0f),
    .accel_residual_rad = DEG(0.3f), .gravity_drift_rad = DEG(0.15f),
    .vertical_tolerance_rad = DEG(0.5f),
    .motor_still_rad_s = DEG(0.2f), .encoder_span_rad = DEG(0.1f),
    .accel_scale = {1.0f, 1.0f, 1.0f},
    .accel_bias_m_s2 = {0.0f, 0.0f, 0.0f},
    .gyro_bias_rad_s = {0.0f, 0.0f, 0.0f}
};
