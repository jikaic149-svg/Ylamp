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
/* op16 scope0/field252/value0 queries finite velocity-trial support.
 * field12/scope1..5 stages signed float rad/s for that joint; field14/scope0
 * stages ramp_ms; field13/scope0=total_ms commits once. Start/held pose is
 * captured by FiveControl, not stale host targets. Same 2s staging/500ms age;
 * commit (including rejection), cancel and session loss consume staged data. */
#define MOTOR_CRUISE_VERSION 0x594C5601UL
/* op16 scope0/field251/value0 queries persistent manual-idle support (v2).
 * field15/value DRAG selects healthy G/DRAG, idempotently, never enables/clears.
 * field7 explicitly selects persistent pure G. KEY toggles drag/impedance;
 * session release preserves idle selection, or cancels an active task to drag.
 * This op16 capability is distinct from retired op11 tuning diagnostics. */
#define MOTOR_DRAG_VERSION 0x594C4402UL
#define MOTOR_DRAG_REQUEST 0x44524147UL
/* Remote native-MIT commissioning; still the sole FiveControl output owner.
 * op16 field249/scope0/value0: capability; field22/scope0/value IMIT:
 * capture current raw centers (idempotent; no cold/enable/ACK).
 * v2 field23/scope1..5: float offset_deg starts one finite +/- offset cycle,
 * ramp outward but STEP to the identical center for both returns;
 * ACK value is configured duration_ms, NOT physical arrival.
 * Read-only value0: field24/scope0 active test joint (0 idle);
 * field24/scope1..5 current native raw target (requires IMPEDANCE);
 * field25/scope0 actual target-side test start_ms.
 * Session/KEY/fault exits retain existing manual-idle/G handover semantics. */
#define MOTOR_IMPEDANCE_TEST_VERSION 0x594C5402UL
#define MOTOR_IMPEDANCE_REQUEST 0x494D4954UL
/* Read-only op16/value0: scope0/field250=capability; scope0 fields16 flags,
 * 17 committed sequence,18 errors,19 boot-local successful writes,
 * 20 saved_mask | restored_mask<<8. Flags: ready1,loaded2,writing4,erasing8,
 * io_error16,foreign32,no_chip64. scope1..5 fields16 continuous reference rad,
 * 17 saved reference rad (requires saved bit),18 trigger degrees,
 * 19 running whole-turn corrections,20 signed turn N,21 single-turn rad [0,2pi).
 * q_reference = phase + 2*pi*N. Original raw telemetry remains untouched.
 * These queries never format storage, reset, move, enable or clear faults. */
#define MOTOR_HISTORY_VERSION 0x594C4801UL
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
