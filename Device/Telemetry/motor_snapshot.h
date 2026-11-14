#ifndef YLAMP_MOTOR_SNAPSHOT_H
#define YLAMP_MOTOR_SNAPSHOT_H

#include "dm_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_SNAPSHOT_VERSION 1U
#define MOTOR_SNAPSHOT_LENGTH 48U
#define MOTOR_SNAPSHOT_RECORD_SIZE 32U
#define MOTOR_SNAPSHOT_MAX_JOINTS 6U

/* Pure shared CAN FD / USB wire encoder, little-endian binary32.
 * Caller supplies valid count 1..6, index<count, bus 1/2, nonzero max_age.
 * feedback=NULL encodes never-received/NaNs; stale samples keep original values.
 * All other pointers non-NULL; no I/O, state mutation, allocation or waiting. */
void MotorSnapshot_Encode(uint8_t out[MOTOR_SNAPSHOT_LENGTH],
    uint8_t count, uint8_t index, uint8_t bus, uint32_t sequence,
    uint32_t tick_ms, uint16_t max_age_ms, const DM_Motor_Config_t *config,
    const DM_Motor_Feedback_t *feedback);

#ifdef __cplusplus
}
#endif
#endif
