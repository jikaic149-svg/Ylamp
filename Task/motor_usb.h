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
/* Unified observer: every sample carries all five motors + both IMU Pitches.
 * No control SESSION, motor query/enable/disable/motion or zero write.
 * v1 type13 request: nonce[8]+mode: 0 normal,1 run,2 pause; old 3/4 rejected.
 * type93 ACK (18B): nonce[8],mode,result,u32 version,board tick.
 * type94 payload: u32 seq,tick; u8 motor_count,imu_count,motor_size,imu_size;
 * u16 max_age_ms, then count*32B MotorSnapshot records (rad,rad/s,Nm),
 * then two 16B IMU records: u8 id,flags; u16 reserved=0; u32 at,count; f32 Pitch deg.
 * IMU IDs 1=BMI088,2=Yummy; flags bit0 received,bit1 fresh; absent angle=NaN.
 * Actual five-axis payload=206B, total frame=213B; same-envelope, NOT simultaneous
 * sensor acquisition. Original source times/counts and independent flags remain.
 * BMI Pitch=atan2(-ax,hypot(ay,az)); freshness includes existing warmup.
 * Run acquires only an unowned temporary BMI session; pause retains it.
 * Normal/close/epoch loss requests normal async STOP, never steals another owner.
 * Pause suppresses periodic USB data, not motor control or sensor acquisition.
 * An in-flight sample may precede the pause ACK; no replay/recovery.
 * New version rejects old pair-only hosts rather than reinterpreting old data. */
#define MOTOR_FOCUS_VERSION 0x594C5002UL
void MotorUSB_Init(void);
bool MotorUSB_Run(void);
#ifdef __cplusplus
}
#endif
#endif
