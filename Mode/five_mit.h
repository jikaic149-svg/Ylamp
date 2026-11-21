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
/* Seconds, >=0. Active PD only; 0 disables the entry blend, not supervision. */
extern const float g_five_tracking_ramp_s;
#ifdef YLAMP_HOST_FIXTURES
extern const FiveMIT_Axis g_five_mit_sim[FIVE_AXES];
#endif
typedef struct {
    double start_deg[5], end_deg[5], seconds;
} FiveMIT_Plan; /* caller-owned; initialize through Prepare only, then read-only */
typedef struct {
    double software_deg[5]; /* measured reference coordinates; never wrap J4 */
    float raw_rad[5], velocity_rad_s[5]; /* matching positions, same sample */
} FiveMIT_State;
typedef struct {
    DM_Motor_MIT_Command_t motor[5]; /* ordered J1..J5 */
    BSP_CAN_Frame_t frame[5];
} FiveMIT_Output;
/* Nonperiodic travel/protocol-speed checks, NOT a collision or dynamic proof.
 * A rejected replan invalidates the plan. */
bool FiveMIT_Prepare(FiveMIT_Plan *plan,const double from[5],const double to[5],double seconds);
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
 * software position/velocity feedback using the active tracking profile.
 * PD builds smoothly from zero over g_five_tracking_ramp_s after plan start;
 * G/dynamics remain direct. No interpolation of feedback or its source time.
 * MIT gains stay zero; reject the batch before exceeding torque budgets. */
bool FiveMIT_SampleFeedback(const FiveMIT_Plan *plan,double elapsed_s,
    const FiveMIT_State *state,const float support_gravity_nm[5],FiveMIT_Output *out);
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
