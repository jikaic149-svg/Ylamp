#ifndef YLAMP_FIVE_SUPPORT_H
#define YLAMP_FIVE_SUPPORT_H
#include "five_mit.h"

typedef struct { float kp, kd; } FiveSupport_Gains;
extern const FiveSupport_Gains g_five_support_gains[FIVE_AXES];
/* KEY-only sampled total-torque budgets [Nm], separate from pure gravity. */
extern const float g_five_impedance_max_nm[FIVE_AXES];
/* KEY/native hold only: sampled PD effort [Nm], never a reduction of G. */
extern const float g_five_impedance_pd_max_nm[FIVE_AXES];
/* Remote commissioning only, not a manual KEY angle restriction. */
typedef struct { uint32_t ramp_ms, hold_ms; float max_offset_deg; } FiveSupport_TestConfig;
extern const FiveSupport_TestConfig g_five_impedance_test;
/* Commissioning support residual: only its J2 torque is applied. The shared
 * five-link geometry still uses every measured CAD angle; J4/J5 outputs and
 * nominal M/c are unchanged. Effective [kg, kg*m X/Y/Z], not physical labels. */
extern const float g_five_j2_gravity_delta[FIVE_AXES][4];
/* Applied once to observed J3 G before the existing support budget.
 * Not a rigid-body parameter, encoder scale or active-motion inertia gain. */
extern const float g_five_j3_gravity_scale;
/* Support-only empirical coefficient [Nm] times cos(J3 software radians).
 * Applied after the scale, once before the existing budget; not a new zero. */
extern const float g_five_j3_gravity_cosine_nm;
/* One continuous support formula, before the owner's existing torque budget.
 * qCAD and J3 software angle are radians from the same measured pose.
 * Includes the J2 candidate and the configured J3 empirical trim exactly once.
 * No velocity, PD, enable, clipping or I/O; failure leaves gravity_nm unchanged. */
bool FiveSupport_GravityModel(const float qCAD[FIVE_AXES],float j3_software_rad,
                             float gravity_nm[FIVE_AXES]);
typedef struct {
    float anchor_raw_rad[FIVE_AXES]; /* Captured drive coordinates, not CAD/history turns. */
    bool impedance;
    unsigned test_joint; /* 0 idle, otherwise 1..5; same native-MIT support path. */
    uint32_t test_started_ms;
    float test_center_raw_rad, test_offset_rad;
#ifdef YLAMP_J3_REVERSE_TEST
    bool reverse_j3;
    uint32_t reverse_started_ms;
#endif
} FiveSupport_State;

#ifdef YLAMP_J3_REVERSE_TEST
typedef struct { uint32_t total_ms, ramp_ms; float max_nm; } FiveSupport_ReverseConfig;
extern const FiveSupport_ReverseConfig g_five_j3_reverse;
/* Explicit entry only, no enable. Target-side elapsed time, not USB timeout.
 * Changes torque direction only; does NOT calibrate qCAD or encoder signs. */
bool FiveSupport_EnterReverseJ3(FiveSupport_State *mode,uint32_t now,float gravity_j3_nm);
void FiveSupport_ApplyReverseJ3(FiveSupport_State *mode,uint32_t now,float torque_nm[FIVE_AXES]);
#endif

/* Pure mode lifecycle, called by the sole motor owner; no CAN or enable. */
void FiveSupport_Gravity(FiveSupport_State *mode);
bool FiveSupport_EnterImpedance(FiveSupport_State *mode,const FiveMIT_State *state);
/* One finite center -> +offset -> CENTER STEP -> -offset -> CENTER STEP cycle.
 * Owner validates both directed paths against original continuous travel first.
 * Ramp outward only; each return holds the original fixed center for one slot.
 * The ramp duration does not bound physical speed after a center step.
 * No software PD/dynamics/F overlay, enable or I/O. Completion holds the saved
 * center in impedance; Gravity/KEY/session/fault cancellation stops the cycle. */
bool FiveSupport_StartTest(FiveSupport_State *mode,unsigned joint,float offset_deg,uint32_t now);
void FiveSupport_TestTick(FiveSupport_State *mode,uint32_t now);
uint32_t FiveSupport_TestDuration(void);
/* Gravity: torque-only MIT G. Impedance: fixed captured raw position, zero
 * desired velocity, native drive Kp/Kd and G feedforward, no software PD.
 * Reduce both gains together when predicted PD exceeds the smaller of its
 * independent PD ceiling and remaining KEY total budget. Report that normal
 * yielding in limited, without moving the saved target or reducing G.
 * This is a pre-encoding, current-feedback prediction, NOT a drive-side
 * torque clamp or a bound between feedback frames. Validate all five before
 * returning; command/limited remain unchanged on failure. No CAN/enable. */
bool FiveSupport_Sample(const FiveSupport_State *mode,const FiveMIT_State *state,
                        const float gravity_nm[FIVE_AXES],
                        DM_Motor_MIT_Command_t command[FIVE_AXES],unsigned *limited);
#endif
