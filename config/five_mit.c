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
 * User requested wider effort budgets after a latched FIVE_LIMIT, 2026-11-23:
 * shared G ceilings +50% from the deployed ADFC baseline, not compounded
 * with an intermediate candidate; no G formula or fault/exit change.
 * This shared table affects G, drag and KEY support, not KEY alone.
 * A rejected motion sample falls back to gravity, never to a disable. */
const FiveMIT_Axis g_five_mit_hw[FIVE_AXES] = {
    {0,0, 0,0,.02f, .6f,0},
    {0,0, 0,0,.02f, 4.5f,J2_CAD},
    {0,0, 0,0,.02f, 2.7f,J3_CAD},
    {0,0, 0,0,.02f, 1.2f,0},
    {0,0, 0,0,.02f, .6f,0}
};
/* Active trajectories only: software kp [Nm/rad], kd [Nm*s/rad], total Nm.
 * User-approved J1/J4 position tuning, 2026-11-24: kp 20->30 and 3->6.
 * Drag and KEY impedance use their own profiles below; their gains stay put.
 * User explicitly doubled all five active total budgets after the J5 .6 rad/s
 * return hit .4 Nm, 2026-11-22. This changes ceilings, not commanded gains or
 * friction amplitude. That earlier change was active-only; the later shared
 * support/KEY budget relaxation is configured separately above/below.
 * These are commissioning limits, not a validated mechanism rating.
 * MIT actuator gains stay zero and the whole batch is rejected above max_nm. */
const FiveMIT_Tracking g_five_mit_tracking[FIVE_AXES] = {
    {30,1,4}, {30,1.5f,10}, {5,.5f,3.6f}, {6,.3f,1.6f}, {10,.5f,.8f}
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
 * 2026-11-25 user explicitly requested F on ordinary Move/Ruckig too:
 * all active references use this same map ONCE; zero v_ref stays zero F.
 * Existing partial J1/J2 manual drag, pure G/KEY, PI, drag fractions and
 * budgets stay unchanged. No newly identified coefficients or static kick.
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
 * Total budgets stay below existing active budgets. J1 remains 1 Nm;
 * J2 rises only from 4 to 4.5 Nm to accommodate its relaxed G ceiling.
 * J3/J4/J5 match their relaxed G ceilings, still exact G with no added
 * damping or assist. Fractions, assist caps, filter and ramp are unchanged.
 * Initial feel tune, NOT validated passivity or calibration. */
const FiveDrag_Axis g_five_drag[FIVE_AXES] = {
    {.80f,.04f,.008f,.55f,1.0f},
    {.65f,.08f,.008f,.55f,4.5f},
    {0,0,.016f,0,2.7f},
    {0,0,.016f,0,1.2f},
    {0,0,.016f,0,.6f}
};
const float g_five_drag_filter_s = .015f;
const float g_five_drag_ramp_s = .2f;
/* User-requested smoothing candidate: ease active PD in with the same
 * quintic used by the trajectory. G/dynamics and fault handover are not
 * filtered/delayed. Active gains are independent of KEY/manual hold;
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
 * Pure G/impedance/active G base share it; the formula, zero PD and nominal
 * dynamics/friction are unchanged. Effort ceilings are configured separately. */
const float g_five_j3_gravity_scale = 1.2774127431831357f;
/* Same relative scaling of the previous trim, not a second compensation.
 * Add once after the CAD gain and before the existing owner's effort cap.
 * Software angle is radians; zero/CAD datum and encoder sign stay unchanged. */
const float g_five_j3_gravity_cosine_nm = -.010825531721891f;
/* KEY impedance ONLY: native drive Kp [Nm/rad], Kd [Nm*s/rad].
 * User-approved yielding hold, 2026-11-24: retain deployed 859D Kp to preserve
 * the near-center spring slope. Finite large-offset screening rejected uniform
 * Kd*.7 (8.58 deg overshoot): J1/J5 keep *.7, J2 keeps its 859D damping,
 * J3/J4 use *1.15 for braking. Low PD effort ceilings, not weak return braking,
 * provide the large-offset yielding; this is still not hardware acceptance.
 * Independent PD effort ceilings below soften large deflections without
 * reducing G. No integral, static kick, center drift or second software PD.
 * The 1-2 deg bidirectional return target still needs real-drive validation.
 * Original lag-stress source impedance_stress_01.json SHA256
 * 9309E8E463C219DDB2AC86F672413C4DFA9981CBEA11C0076D485D515484E097.
 * Offline finite-scenario selection, NOT measured real-drive stability.
 * Preserve G/F coefficients, drag fractions, speed PI and the mode lifecycle.
 * Capture each drive's raw position on explicit entry; no duplicate software PD.
 * Scale gains together for the separate KEY total/PD sampled budgets.
 * The drive's inner-loop torque between frames is not MCU torque-clamped. */
const FiveSupport_Gains g_five_support_gains[FIVE_AXES] = {
    {84.84f,3.1248f}, {63.84f,5}, {15.75f,1.6054f}, {16.66f,.6969f}, {16.66f,.4207f}
};
/* User-requested KEY total-effort relaxation, 2026-11-23: previous ceilings
 * [1,3,1.8,.8,.4] +50% from ADFC, never compounded with a draft candidate.
 * Shared G ceilings above also rise +50%. The later yielding tune adds a
 * separate PD ceiling; these TOTAL ceilings and active budgets stay unchanged.
 * These commissioning ceilings remain below the existing active ceilings;
 * NOT a measured mechanism rating or a drive-side hard torque clamp.
 * No automatic fault clear, re-enable or change to the fault handover. */
const float g_five_impedance_max_nm[FIVE_AXES] = {1.5f,4.5f,2.7f,1.2f,.6f};
/* Approved hand-yielding tune: maximum sampled |Kp*error-Kd*velocity| [Nm].
 * room=min(this, total_budget-|G|); never clip G or move the captured center.
 * Preserve small-error Kp, reduce large-offset pushing; normal yielding is
 * limited telemetry, not a new fault, time limit or angle gate.
 * Initial commissioning values, not a drive-side hard torque clamp, measured
 * breakaway/hand-force bound or a guarantee of static accuracy/stability. */
const float g_five_impedance_pd_max_nm[FIVE_AXES] = {1.f,1.f,.4f,.3f,.25f};
/* Remote trial v2, user requested large-angle return/overshoot tests.
 * Four 6s slots: +target (4s quintic + 2s hold), fixed-center STEP/hold 6s,
 * -target (4s + 2s), same fixed-center STEP/hold 6s; total 24s.
 * At most 60 deg REQUESTED offset, still subject to original travel/CAD
 * planning and unchanged sampled torque budgets, not permission to use 60 deg
 * on every joint/pose. Physical return speed is not bounded by the outward ramp.
 * No new manual KEY restriction, gain/force increase or fault/exit change. */
const FiveSupport_TestConfig g_five_impedance_test = {4000U,2000U,60.f};
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
