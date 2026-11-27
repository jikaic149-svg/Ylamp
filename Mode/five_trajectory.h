#ifndef YLAMP_FIVE_TRAJECTORY_H
#define YLAMP_FIVE_TRAJECTORY_H
#include "five_mit.h"

#ifdef __cplusplus
extern "C" {
#endif
enum { FIVE_TRAJECTORY_CAPACITY=32 };
#define FIVE_TRAJECTORY_VERSION 0x594c5302U
typedef struct { double velocity_rad_s, acceleration_rad_s2, jerk_rad_s3; } FiveTrajectory_Limit;
/* Active Ruckig planning only; not drag/KEY gains, protocol limits or ratings. */
extern const FiveTrajectory_Limit g_five_trajectory_limits[5];
/* Explicit wire units: relative milliseconds, software degrees, rad/s, rad/s^2.
 * This is an in-memory object, NOT a packed wire format. */
typedef struct {
    uint32_t at_ms;
    float position_deg[5], velocity_rad_s[5], acceleration_rad_s2[5];
} FiveTrajectory_Point;
typedef struct {
    /* Mode-owned: task callers use the functions below, not these members. */
    FiveTrajectory_Point points[FIVE_TRAJECTORY_CAPACITY], origin, pending;
    unsigned expected, count, pending_mask;
    uint32_t start_ms, checksum, duration_ms, segment_ms[FIVE_TRAJECTORY_CAPACITY];
    bool uploading, active, initialized, prepared;
    /* Caller-owned static storage, placement constructed once. Never copy a
     * live state. The board places the 32-profile state in AXI SRAM;
     * no heap/cloud or planning in the running sample path. */
    union { double alignment; unsigned char bytes[120*1024]; } engine;
} FiveTrajectory_State;
typedef struct {
    unsigned expected, count;
    uint32_t start_ms, duration_ms, checksum;
    uint32_t knot_ms[FIVE_TRAJECTORY_CAPACITY];
    float origin_deg[5], origin_velocity_rad_s[5];
    bool uploading, active;
} FiveTrajectory_Status;

/* Staging never changes actuator output. All input is copied; no allocation,
 * hardware, waiting, implicit restart or automatic retry. Cancel invalidates
 * both the incomplete upload and active trajectory. */
void FiveTrajectory_Cancel(FiveTrajectory_State *state);
bool FiveTrajectory_Begin(FiveTrajectory_State *state,unsigned points);
bool FiveTrajectory_Set(FiveTrajectory_State *state,unsigned axis,
                        unsigned derivative,float value);
bool FiveTrajectory_Append(FiveTrajectory_State *state,uint32_t relative_ms);
/* Atomically validate and append a complete point, using the same checks as Append. */
bool FiveTrajectory_AppendPoint(FiveTrajectory_State *state,const FiveTrajectory_Point *point);
/* Canonical 64-byte LE point, shared by USB and the external preset store. */
void FiveTrajectory_EncodePoint(uint8_t bytes[64],const FiveTrajectory_Point *point);
void FiveTrajectory_DecodePoint(FiveTrajectory_Point *point,const uint8_t bytes[64]);
bool FiveTrajectory_ValidatePoint(const FiveTrajectory_Point *point,uint32_t previous_ms,bool last);
/* q_out = captured_origin + (template_q - first_q); v/a/time unchanged.
 * No workspace clamp or admission bypass; AppendPoint validates the result.
 * Output unchanged on nonfinite input. Allows aliased point/first/output. */
bool FiveTrajectory_RelativePoint(FiveTrajectory_Point *out,const FiveTrajectory_Point *point,
                                 const FiveTrajectory_Point *first,const double origin_deg[5]);
/* Append precomputes one interior segment. PrepareStart computes only the
 * actual-origin -> first target segment, before the owner starts its clock.
 * All segments use local five-DOF Ruckig, synchronized q/v/a and finite v/a/j.
 * Requested times are MINIMUM section durations; actual rounded 2ms durations
 * are reported. No calculation or entry ramp restart at an interior knot.
 * Capture measured initial q/v (initial a unknown -> 0); final v/a are zero.
 * Interior supplied v/a are retained. Position extrema
 * keep original travel/inset, not just endpoints. Not collision clearance. */
bool FiveTrajectory_PrepareStart(FiveTrajectory_State *state,const FiveMIT_State *measured);
bool FiveTrajectory_Start(FiveTrajectory_State *state,uint32_t now_ms);
bool FiveTrajectory_Active(const FiveTrajectory_State *state);
FiveTrajectory_Status FiveTrajectory_StatusNow(const FiveTrajectory_State *state);
/* Exact Ruckig profile sampling at owner time, 500 Hz on the board.
 * Final sample holds the last q with zero v/a. No entry blend per knot.
 * Output unchanged on failure. */
bool FiveTrajectory_Sample(const FiveTrajectory_State *state,uint32_t elapsed_ms,
                           FiveMIT_Reference *out);
#ifdef __cplusplus
}
#endif
#endif
