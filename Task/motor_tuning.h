#ifndef YLAMP_MOTOR_TUNING_H
#define YLAMP_MOTOR_TUNING_H

#include "motor_task.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_TUNING_REQUEST_ID  0x7E0U
#define MOTOR_TUNING_RESPONSE_ID 0x7E1U
#define MOTOR_TUNING_VERSION     0x594C0003U
#define MOTOR_MOTION_VERSION     0x594C5002U

/* RPC 8 bytes (Classic CAN, or UART payload): seq/op/scope/field/value32.
 * All multibyte values are little endian.
 * Scope 0=global, 1=J1, 2=J2; parameter value = float in UI units.
 * op 0=hello, 1=read, 2=begin(expected revision u32), 3=stage,
 * 4=commit, 5=abort, 6=status, 7=Task command (field=command).
 * Response: seq/BSP_CAN_Status/scope/field/value32. op0 returns VERSION;
 * op4 and status field1 return revision. status field0 returns
 * state/reason/policy_ready/feedback_valid bytes. No C struct on the wire.
 * One PC owns the transaction; this RPC has no authentication/dedup.
 * UART's reliable envelope separately deduplicates identical retries.
 * BEGIN copies current config; all public fields must be staged before COMMIT.
 * Idle staging expires after 3 seconds. A failed/lost reply means action outcome
 * UNKNOWN; it must never trigger an automatic retry.
 * CAN ISR uses an 8-frame FIFO; Run handles <=1 request per pass.
 * UART uses MotorSerial's bounded byte ring and calls HandleRequest directly.
 * Overflow/failed response submission is not retried (PC reports timeout).
 * Read/apply operate MCU RAM only, not motor registers/Flash. */
/* Single query owner: MCU alone sends motor requests on 0x7FF.
 * op1 scope1/2 fields240/241/242 read configured PMAX/VMAX/TMAX (f32,
 * rad/rad/s/Nm). These are MCU decode scales, NOT fresh motor-register reads;
 * stage/write to these fields is rejected. Older firmware rejects the reads:
 * PC must stop, never fall back to direct motor queries on a shared bus. */
/* Calibration extension: op8 read-only, scope0/id255 capability 0x594C4302;
 * scope0/id0 captures a coherent 1s snapshot, returns C1/valid-mask/axis/flags.
 * id1=IMU age ms, id2=stable ms (u32); id3=vertical tolerance deg (f32);
 * id4..6=filtered accel m/s2,
 * id7..9=gyro deg/s (f32). Scope1/2 id1=zero, id2=raw, id3=reference-zero,
 * id4=continuous reference angle, deg f32.
 * NaN means unavailable, never substitute zero. op9 scope1/id0 marks midpoint;
 * scope2/id1..6 marks link-UP +X,-X,+Y,-Y,+Z,-Z. value32=current config revision.
 * Mark is a one-shot explicit RAM-only operation, never retry/seek/enable.
 * op10 scope0/id1 starts a temporary IMU session (value=current revision);
 * scope0/id0 requests stop/drain/suspend. Neither action is auto-retried.
 * Flags: bit0 fresh, 1 warmed, 2 stable, 3 motors disabled/still,
 * 4 session active, 5 stopping, 6 suspend failed/unknown hardware state.
 * Boot defaults OFF; successful J2 mark requests OFF, not automatic reopen.
 * Calibration does not change control config revision or reference limits. */
/* Drag diagnostics: op11, scope0/id255 capability 0x594C4402.
 * scope0/id0 freezes a 1s snapshot: D1/state/phase/valid bytes (u32).
 * scope0/id1..3 = nominal A/B/D, kg*m² (f32).
 * scope1/2 id1=raw deg, 2=raw deg/s, 3=current-estimated torque Nm,
 * 4=gravity Nm, 5=c Nm (diagnostic), 6=M*ddq Nm (diagnostic),
 * 7=requested FF Nm, 8=requested Kd, 9=requested Kp,
 * 10=requested wire/raw target deg, 11=reference-target-minus-reference-actual deg,
 * 12=nominal damping -Kd*raw_velocity Nm (f32),
 * 13=continuous reference angle deg, 14=integer-turn reference-minus-raw deg.
 * Stale/invalid data is NaN; OFF has no active command (id7..12 NaN).
 * No control side effects; requested fields are NOT measured output torque.
 * Capture expiry returns NOT_READY. c/M*ddq are unscaled model values;
 * id7 includes bounded partial dynamics, so do not sum id4+id5+id6 as output. */
BSP_CAN_Status_t MotorTuning_Init(FDCAN_HandleTypeDef *bus);
/* Motion extension, op12:
 * scope0/id255 capability MOTOR_MOTION_VERSION; id0 current Mode u32;
 * id1 select Mode value0=DRAG/1=MOTION, inactive only, never enables.
 * id8 capture-current impedance (value=config revision): explicit ONE action,
 * enables if idle, or switches a live Mode without disable/re-enable. Captures
 * once; displacement never changes the center. No trajectory limits needed.
 * Does not clear device/communication faults. Added in capability 0x594C5002.
 * id2 BEGIN one job (value=config revision), only Motion HOLDING.
 * scope1/2 id1 target reference encoder deg, id2 speed deg/s,
 * id3 acceleration deg/s2, id4 jerk deg/s3 (f32, caps finite positive).
 * scope0/id3 COMMIT (value=same revision): all 8 fields required; explicit,
 * once only, never implicitly enables or repeats. id4 discards a draft.
 * Draft expires after 3s inactivity, or on mode generation/revision change,
 * STOP/disable/config BEGIN. Rejected commit preserves existing hold.
 * id5 freezes Motion status 1s (E1/Mode/phase/valid bytes); id6/7 elapsed/
 * duration seconds. scope1/2 id11 gravity,12 c,13 M*ddq Nm,14 ddq rad/s2.
 * These model parts are diagnostics, not measured output. Invalid is NaN.
 * Existing telemetry v2 carries all 5 MIT inputs, actual angle and error.
 * Trajectory completion means reference finished and HOLDING, not settled. */
BSP_CAN_Status_t MotorTuning_Run(uint32_t now_ms);
/* Init(NULL) initializes UART-only RPC; no CAN RPC callback is registered.
 * HandleRequest is called only by the same control task as Run (never ISR).
 * Request/response are eight bytes in the existing wire format, non-NULL,
 * non-overlapping. arrived_ms is the receive timestamp, not dispatch time.
 * OK means a response was produced; response[1] is the business result.
 * No transport send, CAN readiness dependency, retry or automatic motion.
 * One host/transport owns the config transaction; do not run two writers. */
BSP_CAN_Status_t MotorTuning_HandleRequest(const uint8_t request[8],
    uint32_t arrived_ms, uint32_t now_ms, uint8_t response[8]);

#ifdef __cplusplus
}
#endif
#endif
