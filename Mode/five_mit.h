#ifndef YLAMP_FIVE_MIT_H
#define YLAMP_FIVE_MIT_H
#include "dm_motor.h"
#include "five_dynamics.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    float kp, kd, coulomb_nm, viscous_nm_s_rad, smooth_rad_s;
    float model_max_nm, cad_offset_deg;
} FiveMIT_Axis;
extern const FiveMIT_Axis g_five_mit_hw[FIVE_AXES];
typedef struct { float kp, kd, max_nm; } FiveMIT_Tracking;
/* Active commissioning only; never used by pure G or manual impedance hold. */
extern const FiveMIT_Tracking g_five_mit_tracking[FIVE_AXES];
typedef struct { float kp, ki; } FiveMIT_SpeedPI;
/* Velocity trial axis only: kp [Nm*s/rad], ki [Nm/rad]. Effort ceilings remain
 * g_five_mit_tracking[].max_nm; zero-travel/held axes keep normal tracking. */
extern const FiveMIT_SpeedPI g_five_speed_pi[FIVE_AXES];
typedef struct {
    float log_amplitude_nm, speed_exponent, hinge_016, hinge_048;
} FiveMIT_SpeedFriction;
/* URDF-informed TOTAL friction: log amplitude at .08 rad/s, log speed slope,
 * and slope increments above .16/.48. Replaces all old base/fast/high terms.
 * Speed bounds freeze only amplitude, never motion admission or velocity. */
extern const FiveMIT_SpeedFriction g_five_speed_friction[FIVE_AXES];
extern const float g_five_speed_friction_range_rad_s[2];
extern const float g_five_speed_friction_knots_rad_s[2];
/* J2 log-amplitude coefficients [bend, abs nominal G], both 1/Nm.
 * PCA centering is already folded into J2 log_amplitude_nm. These nominal
 * CAD wrench proxies are not measured bearing normal forces. */
extern const float g_five_speed_friction_j2[2];
/* Pure finite calculation; smooth reference sign, never measured sign chatter.
 * At zero reference speed the result is zero. Output unchanged on failure. */
bool FiveMIT_SpeedFrictionTorque(unsigned axis,float reference_rad_s,
    float nominal_gravity_nm,float bending_nm,float *torque_nm);
/* Seconds, >=0. Active PD only; 0 disables the entry blend, not supervision. */
extern const float g_five_tracking_ramp_s;
#ifdef YLAMP_HOST_FIXTURES
extern const FiveMIT_Axis g_five_mit_sim[FIVE_AXES];
#endif
typedef struct {
    double start_deg[5], end_deg[5], seconds;
    double cruise_ramp_s; /* 0: original quintic; >0: smooth velocity ramps + plateau. */
} FiveMIT_Plan; /* caller-owned; initialize through Prepare only, then read-only */
typedef struct {
    double software_deg[5]; /* measured reference coordinates; never wrap J4 */
    float raw_rad[5], velocity_rad_s[5]; /* matching positions, same sample */
} FiveMIT_State;
/* One coordinated interpolated target; never interpolated feedback. */
typedef struct {
    double position_deg[5];
    float velocity_rad_s[5], acceleration_rad_s2[5];
} FiveMIT_Reference;
typedef struct {
    DM_Motor_MIT_Command_t motor[5]; /* ordered J1..J5 */
    BSP_CAN_Frame_t frame[5];
} FiveMIT_Output;
/* Nonperiodic travel/protocol-speed checks, NOT a collision or dynamic proof.
 * A rejected replan invalidates the plan. */
bool FiveMIT_Prepare(FiveMIT_Plan *plan,const double from[5],const double to[5],double seconds);
/* Finite velocity trial: ramp seconds >0, total seconds >2*ramp.
 * Plateau velocity[i] = (to[i]-from[i])*pi/180/(seconds-ramp).
 * Position is the analytic integral of the same smooth velocity reference.
 * Speed feedback uses its configured P/I, with position error providing the
 * speed-error integral. No second integrator; held axes keep normal gains. */
bool FiveMIT_PrepareCruise(FiveMIT_Plan *plan,const double from[5],const double to[5],
                           double seconds,double ramp_s);
typedef struct { double lo[5],hi[5]; bool valid; } FiveMIT_Box;
/* Initialize once via CheckBox; treat the result as immutable. PrepareInBox
 * performs bounded endpoint and speed checks; neither is a collision check. */
bool FiveMIT_CheckBox(FiveMIT_Box *box,const double lo[5],const double hi[5]);
bool FiveMIT_PrepareInBox(FiveMIT_Plan *p,const FiveMIT_Box *box,const double from[5],
                          const double to[5],double seconds);
/* Pure command calculation; no CAN/power/enable. Fails the whole batch,
 * leaving out unchanged. Never reuse a previous output on failure.
 * Motion owner must provide fresh feedback and the calibrated same-turn raw
 * reference; this calculation does not implement hardware supervision. */
bool FiveMIT_Sample(const FiveMIT_Plan *plan,double elapsed_s,const FiveMIT_State *state,
                    const FiveMIT_Axis profile[5],FiveMIT_Output *out);
/* Active commissioning only: measured support G + nominal dynamics +
 * software feedback + ONE total reference-velocity F on Move/Speed/Ruckig.
 * Cruise travel axes use speed PI; ordinary moves/held axes keep their
 * position/velocity gains. Zero reference speed gives zero F, even if the
 * measured joint moves; no static kick or measured-sign feedback chatter.
 * PD builds smoothly from zero over g_five_tracking_ramp_s after plan start;
 * G/dynamics remain direct. No interpolation of feedback or its source time.
 * MIT gains stay zero; reject the batch before exceeding torque budgets. */
bool FiveMIT_SampleFeedback(const FiveMIT_Plan *plan,double elapsed_s,
    const FiveMIT_State *state,const float support_gravity_nm[5],FiveMIT_Output *out);
/* Continuous ordinary-position tracking, same G/M/C/F/PD/budgets as Move.
 * elapsed_s is time from the WHOLE trajectory start, not the current knot.
 * No speed-trial PI, native-drive PD or second entry ramp. */
bool FiveMIT_SampleReferenceFeedback(const FiveMIT_Reference *reference,double elapsed_s,
    const FiveMIT_State *state,const float support_gravity_nm[5],FiveMIT_Output *out);
BSP_CAN_Status_t FiveMIT_SendReference(const FiveMIT_Reference *reference,double elapsed_s,
    const FiveMIT_State *state,const float support_gravity_nm[5],
    DM_Motor_t *const motors[5],unsigned *submitted);
/* Owner-only transport. No automatic enable.
 * Validate all five before sending. On bus failure *submitted is the accepted
 * prefix (not an acknowledgement); stop, never replay an old batch. */
BSP_CAN_Status_t FiveMIT_SendSample(const FiveMIT_Plan *plan,double elapsed_s,
    const FiveMIT_State *state,const FiveMIT_Axis profile[5],
    const float *support_gravity_nm,DM_Motor_t *const motors[5],unsigned *submitted);
#ifdef __cplusplus
}
#endif
#endif
