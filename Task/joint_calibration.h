#ifndef YLAMP_JOINT_CALIBRATION_H
#define YLAMP_JOINT_CALIBRATION_H

#include "motor_task.h"
#include "bmi088.h"
#include "calibration_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define JOINT_CALIBRATION_VERSION 0x594C4302U
#define CAL_IMU_FRESH 1U
#define CAL_WARM 2U
#define CAL_STABLE 4U
#define CAL_MOTORS_STILL_DISABLED 8U
#define CAL_IMU_ACTIVE 16U
#define CAL_IMU_STOPPING 32U
#define CAL_IMU_STOP_FAILED 64U

typedef struct {
    uint8_t flags, valid_mask, axis; /* axis 0=unknown; 1..6=+X,-X,+Y,-Y,+Z,-Z */
    uint32_t imu_age_ms, stable_ms;
    float accel_m_s2[3], gyro_rad_s[3];
    float zero_rad[2], raw_rad[2];
    float vertical_error_rad; /* NaN until an axis has been selected by a mark. */
    float reference_rad[2]; /* Continuous reference branch, not raw wire angle. */
} JointCalibration_Status_t;

/* Main loop only. J1 starts at the configured range midpoint; J2 restores an
 * approved configured reference, otherwise remains uncalibrated. A restored
 * manual reference does not select an IMU axis or open an IMU session.
 * These modeling offsets do not change raw feedback, motor zeros, limits,
 * trajectories, gains, power, enable state or motor Flash. */
BSP_CAN_Status_t JointCalibration_Init(void);
/* Explicit read-only temporary session. START requires an inactive motor Task
 * (IDLE/FAULT), not fresh CAN feedback. This does not approve a zero capture.
 * STOP drains an in-flight SPI transaction, then suspends both sensors.
 * Successful J2 capture requests STOP automatically; reboot never reopens. */
BSP_CAN_Status_t JointCalibration_SetIMU(bool enabled, uint32_t now_ms);
void JointCalibration_Run(uint32_t now_ms);
BSP_CAN_Status_t JointCalibration_GetStatus(uint32_t now_ms,
                                           JointCalibration_Status_t *status);
/* Explicit one-shot action; both motors must already be disabled and still.
 * J1: axis ignored, records current configured range midpoint.
 * J2: user-confirmed link-UP sensor axis 1..6, fresh/warm/stable IMU,
 * near-vertical gravity and stable reference-frame J2 encoder window required.
 * Success is RAM only; no motion, no automatic seek/retry/driver zero write. */
BSP_CAN_Status_t JointCalibration_Mark(uint8_t joint, uint8_t axis, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
#endif
