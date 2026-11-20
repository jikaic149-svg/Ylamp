#ifndef YLAMP_FIVE_SUPPORT_H
#define YLAMP_FIVE_SUPPORT_H
#include "five_mit.h"

typedef struct { float kp, kd; } FiveSupport_Gains;
extern const FiveSupport_Gains g_five_support_gains[FIVE_AXES];
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
 * Includes the J2 candidate and the unchanged J3 empirical trim exactly once.
 * No velocity, PD, enable, clipping or I/O; failure leaves gravity_nm unchanged. */
bool FiveSupport_GravityModel(const float qCAD[FIVE_AXES],float j3_software_rad,
                             float gravity_nm[FIVE_AXES]);
typedef struct {
    double anchor_deg[FIVE_AXES];
    bool impedance;
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
/* Gravity: G only, independent of anchor and velocity. Impedance: G plus
 * software spring/damping about the entry pose, within the existing budget.
 * Both modes use torque-only MIT frames; never duplicate PD in the drive. */
bool FiveSupport_Sample(const FiveSupport_State *mode,const FiveMIT_State *state,
                        const float gravity_nm[FIVE_AXES],float torque_nm[FIVE_AXES]);
#endif
