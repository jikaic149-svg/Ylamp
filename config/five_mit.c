#include "five_mit.h"
#include "five_support.h"
#include "five_coordinates.h"

/* qCAD = reference_raw - shared_zero + CAD offset, with unit encoder scale.
 * The 2026-11-19 Pitch LS slopes describe IMU observations, not joint gearing.
 * Shared calibrated CAD offsets feed G/M/c together, once.
 * Historical two-axis sine/friction fits are not this five-axis model. */
/* Gravity-model commissioning: MIT Kp, Kd, Fc and B are all zero.
 * Torque budgets are software ceilings
 * on predicted total torque, NOT motor protocol/rating or a validated hold.
 * A rejected motion sample falls back to gravity, never to a disable. */
const FiveMIT_Axis g_five_mit_hw[FIVE_AXES] = {
    {0,0, 0,0,.02f, .4f,0},
    {0,0, 0,0,.02f, 3.0f,J2_CAD},
    {0,0, 0,0,.02f, 1.8f,J3_CAD},
    {0,0, 0,0,.02f, .8f,0},
    {0,0, 0,0,.02f, .4f,0}
};
/* Active identification only: software kp [Nm/rad], kd [Nm*s/rad], total Nm.
 * The original 0.4/3 Nm J1/J2 HOLD budgets stopped the measured active tests;
 * keep pure-G/impedance budgets untouched and give active tracking its own
 * bounded effort. These are commissioning limits, not a mechanism rating.
 * MIT actuator gains stay zero and the whole batch is rejected above max_nm. */
const FiveMIT_Tracking g_five_mit_tracking[FIVE_AXES] = {
    {20,1,2}, {30,1.5f,5}, {5,.5f,1.8f}, {3,.3f,.8f}, {10,.5f,.4f}
};
/* User-requested smoothing candidate: ease active PD in with the same
 * quintic used by the trajectory. G/dynamics and fault handover are not
 * filtered/delayed. J3/J4 gains reuse the lower manual-hold baseline;
 * this is an initial tuning candidate, not a hardware stability result. */
const float g_five_tracking_ramp_s = .1f;
/* User-authorized J2 support candidate, 2026-11-20; NOT hardware validated.
 * G2 = G2_CAD + (Yg(qCAD)*delta)_2, using all five measured angles.
 * Effective potential residual [kg, kg*m X/Y/Z] per link; never scale M/c.
 * Source: gravity_residual_02.json SHA256
 * 775435E883954E79F132AC148D1391ABD510238C37B22F7A142B6B94B9081816.
 * Only J2 output is applied. J3 keeps its existing trim; J1/J4/J5 unchanged.
 * This scoped support correction is not a newly identified full-body model. */
const float g_five_j2_gravity_delta[FIVE_AXES][4] = {
    {0,0,0,0},
    {0,-.0164182597156f,0,-.000946406295559f},
    {.126940341852f,.00336448940255f,0,.00710592716843f},
    {.0583312670578f,-.00823985302446f,-.00370807413993f,-.0107413542675f},
    {.030915274808f,.000715258259845f,.00202653249103f,-.00911195010189f}
};
/* User-requested small J3 support increase, 2026-11-19.
 * Pure G and the impedance G base only; retain the 1.8 Nm budget and zero PD. */
const float g_five_j3_gravity_scale = 1.18f;
/* User-requested angle-side comparison, 2026-11-19, NOT identified COM.
 * Add coefficient*cos(J3 software radians) AFTER the support gain, before the
 * original support budget. Smooth at zero, |delta torque| <= .01 Nm.
 * This local empirical trim does not replace the five-axis CAD mapping. */
const float g_five_j3_gravity_cosine_nm = -.01f;
/* Impedance ONLY: software Kp [Nm/rad], Kd [Nm*s/rad].
 * Reuse the BEE414C8 deployed hold gains, not the high-gain simulation profile.
 * The center is captured on every explicit entry, never the calibrated zero. */
const FiveSupport_Gains g_five_support_gains[FIVE_AXES] = {
    {5,.5f}, {5,.5f}, {5,.5f}, {3,.3f}, {3,.3f}
};
#ifdef YLAMP_J3_REVERSE_TEST
/* Supported comparison only: total/ramp [ms], absolute J3 torque [N*m].
 * Outside this envelope leave the original five-axis gravity unchanged. */
const FiveSupport_ReverseConfig g_five_j3_reverse = {600U,100U,.5f};
#endif
#ifdef YLAMP_HOST_FIXTURES
/* Simulation only. kp [Nm/rad], kd [Nm*s/rad], Fc [Nm], B [Nm*s/rad],
 * smoothing speed [rad/s], scene total torque [Nm], baseline CAD zero [deg].
 * Protocol limits and actual CAN addresses live in motor_config.c. */
const FiveMIT_Axis g_five_mit_sim[FIVE_AXES] = {
    {140,5,    0,0,.02f,27,  0},
    {130,5,    0,0,.02f,27, J2_CAD},
    {150,3,    0,0,.02f, 7,J3_CAD},
    { 90,1,    0,0,.02f, 7,  0},
    { 60,.75f, 0,0,.02f, 7,  0}
};

#endif
