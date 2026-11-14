#ifndef YLAMP_CALIBRATION_CONFIG_H
#define YLAMP_CALIBRATION_CONFIG_H

#include <stdint.h>

typedef struct {
    uint32_t warmup_ms, stable_ms, max_age_ms;
    float accel_tau_s;
    float gravity_m_s2, gravity_tolerance_m_s2;
    float gyro_still_rad_s;
    float accel_residual_rad, gravity_drift_rad, vertical_tolerance_rad;
    float motor_still_rad_s, encoder_span_rad;
    /* This board's calibration only. Identity/zero = not six-face calibrated.
     * Never copy the reference board's affine/gyro-bias coefficients. */
    float accel_scale[3], accel_bias_m_s2[3], gyro_bias_rad_s[3];
} JointCalibration_Config_t;

#ifdef __cplusplus
extern "C" {
#endif
extern const JointCalibration_Config_t g_calibration_config;
#ifdef __cplusplus
}
#endif
#endif
