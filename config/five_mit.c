#include "five_mit.h"
#include "five_support.h"
#include "five_drag.h"
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
 * User explicitly doubled all five active total budgets after the J5 .6 rad/s
 * return hit .4 Nm, 2026-11-22. This changes ceilings, not commanded gains or
 * friction amplitude; pure-G/impedance budgets remain untouched.
 * These are commissioning limits, not a validated mechanism rating.
 * MIT actuator gains stay zero and the whole batch is rejected above max_nm. */
const FiveMIT_Tracking g_five_mit_tracking[FIVE_AXES] = {
    {20,1,4}, {30,1.5f,10}, {5,.5f,3.6f}, {3,.3f,1.6f}, {10,.5f,.8f}
};
/* Axis-specific candidate after speed_smooth_validation_01:
 * P [Nm*s/rad], I [Nm/rad]. Keep the improved J2 tune; restore baseline
 * integral on J1/J3/J4 because the uniform half-I tune increased lag.
 * J4 gets more velocity damping; J5 returns to its better first-trial gains.
 * q_ref-q is the velocity-error integral. Only the nonzero-travel cruise
 * axis uses this table, including its timed endpoint hold; other four axes,
 * ordinary Move, manual impedance and pure G retain their existing gains.
 * Keep the original shared effort ceilings; not a hardware-validated tune. */
const FiveMIT_SpeedPI g_five_speed_pi[FIVE_AXES] = {
    {1.5f,20}, {2.25f,15}, {.75f,5}, {.6f,3}, {.5f,10}
};
/* User-authorized global friction candidate02 deployment, 2026-11-23.
 * Source friction_global_model_02.json SHA256
 * D950FF3F2F803D953889EAFECB1E52510BE575558895E17E22CE6E359F2D8C37.
 * log A = intercept + slope*log(v/.08) + hinge16*max(0,log(v/.16))
 *         + hinge48*max(0,log(v/.48)); total F=A*tanh(v/.02).
 * Replaces the old base/fast/high chain, never adds a second total F.
 * Retain low-speed amplitude freezing at .08; freeze above .8. These are
 * model extrapolation bounds, NOT commanded-speed or motion-admission gates.
 * Same callers: moving cruise axes and existing partial J1/J2 manual drag.
 * Ordinary Move/held axes/pure G/PI/drag fractions/budgets stay unchanged.
 * Coverage and J1 local tail limitations remain; this is not global acceptance. */
const FiveMIT_SpeedFriction g_five_speed_friction[FIVE_AXES] = {
    {-.3529545390540462f,-.1094118938571156f,.256320566976014f,.016621367861598263f},
    {-.6264370238064128f,-.16367067413011038f,.3254983958136558f,.07914225028485657f},
    {-2.067608595539022f,0,0,0},
    {-2.0190656854614013f,-.08758386997159276f,0,0},
    {-2.006567438632663f,0,0,0}
};
const float g_five_speed_friction_range_rad_s[2] = {.08f,.8f};
const float g_five_speed_friction_knots_rad_s[2] = {.16f,.48f};
/* Exact collapse of J2's two trained PCs into nominal physical wrench units.
 * Normalizer W*reach=5.308704988371524 Nm; PC centering contributes
 * -.0656988255731292 to its intercept above. No second load offset or G trim. */
const float g_five_speed_friction_j2[2] = {
    -.12727473763230213f,.18466618285566616f
};
/* Explicit J1/J2-only manual-drag trial, 2026-11-22: partial compensation of the
 * SAME total friction map using filtered measured speed, not a velocity PI.
 * Fraction, positive damping [Nm*s/rad], velocity signal deadband [rad/s],
 * assist cap [Nm], total G+overlay budget [Nm]. Deadbands exceed the drive's
 * half-LSB zero-speed bias. No static kick, gravity change or enable.
 * Total budgets are below existing active budgets; pure-G/impedance budgets
 * remain unchanged. User keeps J3/J4/J5 at exact G, no added damping or assist.
 * Initial feel tune, NOT validated passivity or calibration. */
const FiveDrag_Axis g_five_drag[FIVE_AXES] = {
    {.80f,.04f,.008f,.55f,1.0f},
    {.65f,.08f,.008f,.55f,4.0f},
    {0,0,.016f,0,1.8f},
    {0,0,.016f,0,.8f},
    {0,0,.016f,0,.4f}
};
const float g_five_drag_filter_s = .015f;
const float g_five_drag_ramp_s = .2f;
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
/* User-authorized J3 gravity trial, 2026-11-22. Scale the complete previous
 * 1.18*G_CAD3-.01*cos(theta3) by 1.0825531721890982 exactly once.
 * Source gravity_from_friction_02.json SHA256
 * 797F798E63E6E6570D4E56B44616652E80299D13E12F96435A81B8BF1AF49049.
 * This is a commissioning candidate, not accepted global gravity calibration.
 * Pure G/impedance/active G base share it; retain the 1.8 Nm G cap, zero PD,
 * all other joints, nominal dynamics, friction and control budgets. */
const float g_five_j3_gravity_scale = 1.2774127431831357f;
/* Same relative scaling of the previous trim, not a second compensation.
 * Add once after the CAD gain and before the existing owner's effort cap.
 * Software angle is radians; zero/CAD datum and encoder sign stay unchanged. */
const float g_five_j3_gravity_cosine_nm = -.010825531721891f;
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
