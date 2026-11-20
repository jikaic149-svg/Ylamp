#ifndef YLAMP_MOTOR_USB_H
#define YLAMP_MOTOR_USB_H
#include "dm_motor.h"
#ifdef __cplusplus
extern "C" {
#endif
/* op16: one physical owner, no standalone J3 streaming/disable endpoint. */
#define MOTOR_FIVE_VERSION 0x594C3501UL
#define MOTOR_COLD_SUPPORTED 0x434F4C44UL /* Explicit declaration, not a status query. */
#define MOTOR_J3_READ_VERSION 0x594C4A01UL
#define MOTOR_J3_READ_SUPPORTED 0x4A335244UL /* Explicit supported legacy acquisition. */
#ifdef YLAMP_J3_REVERSE_TEST
/* op16 scope0/field254/value0 queries this optional capability.
 * scope3/field10/value J3RV requests one target-timed trial, never enables. */
#define MOTOR_J3_REVERSE_VERSION 0x594C3301UL
#define MOTOR_J3_REVERSE_SUPPORTED 0x4A335256UL
#endif
/* op14/scope0/value0, field255=version; field0 freezes a 1s snapshot:
 * u32 B1/00/format/flags (bits0 RX healthy,1 received,2 age<=500ms).
 * Fields1 age_ms (UINT32_MAX if absent),2 count,3..5 float Pitch/Roll/Yaw
 * degrees,6 RX bytes,7 bad frames,8 ring overflows,9 HAL UART error bits.
 * Not an angular-rate packet; the factory USE protocol carries angles only. */
#define MOTOR_UART_IMU_VERSION 0x594C4901UL
/* Read-only op15/value0: scope0/field255=version; scope1..5 fields0=kind
 * (1=link-UP candidate, 2=user midpoint, 3=user accepted), 1=zero,
 * 2=observed raw lower, 3=observed raw upper. Floats are radians.
 * Narrow ranges select an equivalent 2*pi branch before subtracting zero;
 * multi-turn J1/J4 preserve raw. J2 also supplies legacy control defaults.
 * These references do not certify collision-free combinations.
 * Older builds may omit scope1. No configuration or actuator writes. */
#define MOTOR_JOINT_REFERENCE_VERSION 0x594C5202UL
/* Read-only observer, no control SESSION/enable/disable/motion/zero write:
 * v1 frame type13, payload nonce[8]+mode[1]: 0 normal,1 run,2 pause.
 * type93 ACK: nonce[8], mode, result, version u32, board tick u32 (18B).
 * type94 sample (36B): u32 seq,tick,j3_at,j3_count,imu_at,imu_count;
 * f32 J3 RAW encoder degrees, Yummy Pitch degrees; u8 flags,state; u16 age limit.
 * flags bits0/1 motor received/fresh, bits2/3 IMU received/fresh.
 * Missing angles are NaN; stale angles retain source time but not fresh flags.
 * Pausing suppresses all periodic USB telemetry, not control/IMU acquisition.
 * DTR close/RX epoch loss restores normal five-axis USB reporting.
 * One already-submitted transfer may precede the pause ACK. No replay/recovery. */
#define MOTOR_FOCUS_VERSION 0x594C5001UL
#define MOTOR_J2_FOCUS_VERSION 0x594C3202UL
/* Modes 3/4 select J2 encoder + BMI088 Pitch degrees, run/pause.
 * Pitch = atan2(-ax, hypot(ay,az)), no mounting/bias correction.
 * Same sample layout; IMU-fresh additionally requires the existing warmup.
 * Explicit run acquires the temporary BMI session; pause retains it.
 * Leaving this profile or disconnecting requests normal asynchronous STOP.
 * No motor enable, trajectory, zero capture or calibration write. */
void MotorUSB_Init(void);
bool MotorUSB_Run(void);
#ifdef __cplusplus
}
#endif
#endif
