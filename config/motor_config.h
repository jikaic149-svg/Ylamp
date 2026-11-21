#ifndef YLAMP_MOTOR_CONFIG_H
#define YLAMP_MOTOR_CONFIG_H

#include <stdbool.h>
#include "dm_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Physical owner, model, calibration and bus addresses have the same order. */
#define MOTOR_CAN_JOINT_COUNT 5U
/* 500 Hz motor-owner tick; the existing RTOS task still wakes every 1 ms.
 * Leave CAN headroom instead of sending all five at the 1 kHz task rate. */
#define FIVE_PERIOD_MS 2U
#define FIVE_QUERY_MS 10U
#define FIVE_MAX_SKEW_MS 20U
#define FIVE_MAX_AGE_MS 100U
#define FIVE_ENABLE_TIMEOUT_MS 500U
#define FIVE_TEMP_LIMIT_C 60U
/* Software coordinates, never a write to a drive's encoder zero. */
#define MOTOR_REFERENCE_CANDIDATE 1U
#define MOTOR_REFERENCE_MIDPOINT 2U
#define MOTOR_REFERENCE_USER_APPROVED 3U
typedef struct {
    float zero_rad, observed_lower_rad, observed_upper_rad;
    uint8_t kind; /* 0=unconfigured, 1=candidate, 2=midpoint, 3=user accepted. */
} MotorJointReference_Config_t;
extern const MotorJointReference_Config_t
    g_joint_reference_config[MOTOR_CAN_JOINT_COUNT];
extern const DM_Motor_Config_t g_motor_can_config[MOTOR_CAN_JOINT_COUNT];
#ifdef __cplusplus
}
#endif
#endif
