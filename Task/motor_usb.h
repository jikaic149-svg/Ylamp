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
/* op16 scope0/field251/value0 queries manual-idle support (v2).
 * field15/value DRAG enters healthy DRAG, idempotently, never enables/clears.
 * field7 selects IDLE (legacy pure G), without clearing faults or enabling.
 * KEY toggles drag/impedance;
 * session release preserves idle support, or cancels an active task to drag.
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
/* Preloaded five-axis Ruckig trajectory, op16 capability field248/value0.
 * field34/scope0=count begins a non-actuating upload (1..32 future states).
 * For EACH knot: fields30/31/32, scopes1..5 stage float q_deg/v_rad_s/a_rad_s2;
 * field33/scope0=relative_ms appends all 15 components, returns stored count.
 * field35/scope0=SPNL commits once after the full upload; actual q0/v0 captured
 * by the owner, initial a0 unknown -> 0, final v/a zero. Original 500ms request freshness,
 * sequence/dedup, KEY/session/fault/hold semantics; no enable/reset/retry.
 * Read-only value0: field36=count|expected<<8|uploading<<16|active<<17,
 * 37=start_ms,38=duration_ms,39=FNV1a32 of canonical LE (u32 time,15 f32 q/v/a).
 * field40/scopes1..5=actual captured origin software degrees (requires active).
 * field41/scopes1..count=actual cumulative knot_ms, field42/scopes1..5=actual
 * captured initial rad/s (requires active). Requested section times are minima,
 * not fixed-time demands: Ruckig can extend them under unchanged v/a/j limits.
 * Hash covers uploaded future knots, not captured origin or execution proof. */
#define MOTOR_TRAJECTORY_VERSION 0x594C5302UL
#define MOTOR_TRAJECTORY_REQUEST 0x53504C4EUL
/* op16 field247/value0: bundled motion capability. Same v1 framing/CRC.
 * All payloads begin with session[8], u32 ID. Writes share the legacy
 * sequence and append u32 sampled board tick; QUERY has its own IDs.
 * BEGIN: u8 count. POINT: u32 plan_id, u8 zero-based index, 64B LE knot
 * (u32 relative_ms, 5*f32 software deg, 5*f32 rad/s, 5*f32 rad/s^2).
 * START: u32 plan_id, u32 checksum. Plan ID is the accepted BEGIN write ID.
 * See README for replies. QUERY never mutates or refreshes a write. */
#define MOTOR_MOTION_BUNDLE_VERSION 0x594C4202UL
#define MOTOR_LOOKAROUND_VERSION 0x594C5001UL
enum {
    MOTOR_USB_TRAJ_BEGIN=0x18, MOTOR_USB_TRAJ_POINT=0x19,
    MOTOR_USB_TRAJ_START=0x1A, MOTOR_USB_MOTION_QUERY=0x1B,
    MOTOR_USB_TRAJ_ACK=0x98, MOTOR_USB_TRAJ_RESULT=0x99, MOTOR_USB_MOTION_STATE=0x9A,
    MOTOR_USB_TRAJ_BEGIN_LEN=17, MOTOR_USB_TRAJ_POINT_LEN=85,
    MOTOR_USB_TRAJ_START_LEN=24, MOTOR_USB_MOTION_QUERY_LEN=12,
    MOTOR_USB_TRAJ_ACK_LEN=24, MOTOR_USB_MOTION_STATE_LEN=244,
    /* Explicit storage only. Same BEGIN/POINT/SAVE layouts as the motion
     * bundle, but bound to a separate preset_id, never a trajectory start.
     * Query is independent/read-only. SAVE accepts an asynchronous commit;
     * successful persistence requires sequence/hash + SAVING clear, no error.
     * Prefix 12B, rc/count/expected/loaded_count (4B), then six u32:
     * flags, saved_sequence, saved_hash, staged_hash, preset_id, errors.
     * op16 scope0 field246/value0 queries this capability. */
    MOTOR_USB_PRESET_BEGIN=0x1C, MOTOR_USB_PRESET_POINT=0x1D,
    MOTOR_USB_PRESET_SAVE=0x1E, MOTOR_USB_PRESET_QUERY=0x1F,
    MOTOR_USB_PRESET_ACK=0x9C, MOTOR_USB_PRESET_STATE=0x9F,
    MOTOR_USB_PRESET_STATE_LEN=40,
    /* Read-only 8KiB preset partition backup: session+query_id+u32 offset.
     * Reply prefix, rc/u8 length/2 reserved, u32 offset, <=224 bytes padded
     * to 224. No addressing outside the dedicated partition. */
    MOTOR_USB_PRESET_READ=0x20, MOTOR_USB_PRESET_DATA=0xA0,
    MOTOR_USB_PRESET_READ_LEN=16, MOTOR_USB_PRESET_DATA_LEN=244
};
/* Read-only op16/value0: scope0/field250=capability; scope0 fields16 flags,
 * 17 committed sequence,18 errors,19 boot-local successful writes,
 * 20 saved_mask | restored_mask<<8. Flags: ready1,loaded2,writing4,erasing8,
 * io_error16,foreign32,no_chip64. scope1..5 fields16 continuous reference rad,
 * 17 saved reference rad (requires saved bit),18 trigger degrees,
 * 19 running whole-turn corrections,20 signed turn N,21 single-turn rad [0,2pi).
 * q_reference = phase + 2*pi*N. Original raw telemetry remains untouched.
 * These queries never format storage, reset, move, enable or clear faults. */
#define MOTOR_HISTORY_VERSION 0x594C4801UL
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
