"""Offline position-even/inertia separation on the frozen 12-leg batch."""
import argparse
import json
from pathlib import Path

import numpy as np

from five_friction_global import balanced, plain, predict, score, sha
from five_friction_map import load_features
from five_gravity_residual_report import support_gravity
from five_identify import HW, ROOT, load_run, scaled_lstsq
from five_inertia_identify import nominal_torque


def position_design(q, velocity, weights, degree, speed_term=False, center=None):
    """Learn the position center on training data only; never tune on holdout."""
    if center is None:
        center = float(np.average(q, weights=weights))
    columns = [np.ones(len(q)), np.tanh(velocity/.02)]
    for power in range(1, degree+1):
        columns.append(((q-center)/.25)**power)
    if speed_term:
        columns.append(velocity)
    return np.column_stack(columns), center


def partial_inertia(acceleration, nuisance, target, groups):
    """Weighted residual-on-residual fit, checked against the joint solution."""
    w = balanced(groups)
    root = np.sqrt(w)
    acceleration_fit, _ = scaled_lstsq(nuisance*root[:, None], acceleration*root)
    target_fit, _ = scaled_lstsq(nuisance*root[:, None], target*root)
    a_residual = acceleration-nuisance@acceleration_fit
    y_residual = target-nuisance@target_fit
    information = float(np.sum(w*a_residual**2))
    variance = float(np.sum(w*(acceleration-np.average(acceleration, weights=w))**2))
    if information <= max(1e-12, variance*1e-10):
        raise ValueError("Acceleration is not separable from the nuisance columns")
    delta = float(np.sum(w*a_residual*y_residual)/information)
    x = np.column_stack((acceleration, nuisance))
    beta, condition = scaled_lstsq(x*root[:, None], target*root)
    if not np.isclose(delta, beta[0], rtol=1e-9, atol=1e-11):
        raise ValueError("Partial and joint least-squares solutions disagree")
    return beta, dict(scaled_condition=condition,
                      acceleration_information_fraction=information/variance,
                      residual_acceleration_rms_rad_s2=information**.5)


def matched_acceleration(q, v, a, residual, ramps, groups, times, axis, q_tolerance_deg=.5):
    """No holdout points or sample reuse; same signed speed and all-five pose."""
    fast, slow = np.flatnonzero(ramps == 300), np.flatnonzero(ramps == 1200)
    dq = np.rad2deg(np.max(abs(q[fast, None]-q[None, slow]), axis=2))
    dv = abs(v[fast, None]-v[None, slow])
    allowance = np.minimum(.03, .1*(abs(v[fast, None])+abs(v[None, slow]))/2)
    da = a[fast, None]-a[None, slow]
    eligible = ((dq <= q_tolerance_deg) & (dv <= allowance) &
                (v[fast, None]*v[None, slow] > 0) & (abs(da) >= .25))
    candidates = np.argwhere(eligible)
    if not len(candidates):
        return dict(pose_tolerance_deg=q_tolerance_deg, pairs=0,
                    extra_inertia_kg_m2=None, matched_pairs=[])
    cost = (dq/q_tolerance_deg)**2+(dv/np.maximum(allowance, 1e-12))**2
    candidates = sorted(candidates, key=lambda pair: cost[tuple(pair)])
    used_fast, used_slow, pairs = set(), set(), []
    for fi, si in candidates:
        if fi in used_fast or si in used_slow:
            continue
        used_fast.add(fi)
        used_slow.add(si)
        i, j = int(fast[fi]), int(slow[si])
        pairs.append(dict(fast_sample=i, slow_sample=j, fast_record=str(groups[i]),
            slow_record=str(groups[j]), fast_source_ms=float(times[i]),
            slow_source_ms=float(times[j]), max_pose_difference_deg=float(dq[fi, si]),
            velocity_difference_rad_s=float(dv[fi, si]),
            delta_acceleration_rad_s2=float(a[i]-a[j]),
            delta_residual_nm=float(residual[i]-residual[j]),
            block=f"{int(np.floor(np.rad2deg((q[i, axis]+q[j, axis])/2)))}:{int(np.sign(v[i]))}"))
    w = balanced([p["block"] for p in pairs])
    delta_a = np.array([p["delta_acceleration_rad_s2"] for p in pairs])
    delta_y = np.array([p["delta_residual_nm"] for p in pairs])
    delta = float(np.sum(w*delta_a*delta_y)/np.sum(w*delta_a**2))
    return dict(pose_tolerance_deg=q_tolerance_deg, pairs=len(pairs),
                angular_direction_blocks=len(set(p["block"] for p in pairs)),
                extra_inertia_kg_m2=delta, matched_pairs=pairs,
                difference_fit_rmse_nm=float(np.sqrt(np.sum(w*(delta*delta_a-delta_y)**2))))


def collect(baseline_path):
    baseline = json.loads(baseline_path.read_text(encoding="utf-8"))
    if sha(baseline_path) != "6a08baf10405a4341fee8697b346cf11376458dd7cec28ccdcda521e99171c14":
        raise ValueError("Frozen first-stage result changed")
    for path, expected in baseline["source_sha256"].items():
        if sha(Path(path)) != expected:
            raise ValueError(f"Frozen source changed: {path}")
    frozen_path = ROOT/"build/v12-default-drag-20261123/friction_global_model_02.json"
    if sha(frozen_path) != baseline["frozen_friction_sha256"]:
        raise ValueError("Frozen friction changed")
    models = json.loads(frozen_path.read_text(encoding="utf-8"))["models"]
    parts = {1: [], 2: []}
    for capture in baseline["captures"]:
        path = Path(capture["file"])
        report, data = load_run(path, with_time=True)
        if report["issues"] or data is None or report["sha256"] != capture["sha256"]:
            raise ValueError(f"Unusable original: {path}")
        q, v, a, tau, relative = data
        source_ms = relative+report["analysis_origin_tick_ms"]
        joint = capture["joint"]
        start, duration = capture["verified_start_tick_ms"], capture["duration_ms"]
        keep = ((source_ms >= start+40) & (source_ms <= start+duration-40) &
                (abs(v[:, joint-1]) >= .05))
        if int(keep.sum()) != capture["retained_sliding_samples"]:
            raise ValueError("Original eligibility changed")
        q, v, a, tau, source_ms = (value[keep] for value in (q, v, a, tau, source_ms))
        zero, axis = np.zeros_like(v), joint-1
        dynamics = nominal_torque(q, v, a)-nominal_torque(q, zero, zero)
        z = load_features(q)[0][:, axis]
        friction = predict(models[axis]["final_model"], z, np.clip(abs(v[:, axis]), .08, .8))
        friction *= np.tanh(v[:, axis]/HW[axis, 4])
        residual = tau[:, axis]-support_gravity(q)[:, axis]-dynamics[:, axis]-friction
        parts[joint].append(dict(q=q, v=v[:, axis], a=a[:, axis], y=residual,
            ramp=np.full(len(q), capture["ramp_ms"]), time=source_ms,
            group=np.full(len(q), path.name, dtype=object)))
    return {joint: {key: np.concatenate([part[key] for part in legs])
                    for key in legs[0]} for joint, legs in parts.items()}, baseline


def evaluate(data, axis, degree, speed_term, heldout_ramp):
    train, test = data["ramp"] != heldout_ramp, data["ramp"] == heldout_ramp
    z, center = position_design(data["q"][train, axis], data["v"][train],
        balanced(data["group"][train]), degree, speed_term)
    z_test, _ = position_design(data["q"][test, axis], data["v"][test], None,
                               degree, speed_term, center)
    beta, information = partial_inertia(data["a"][train], z, data["y"][train],
                                        data["group"][train])
    root = np.sqrt(balanced(data["group"][train]))
    no_inertia, _ = scaled_lstsq(z*root[:, None], data["y"][train]*root)
    baseline_error = z_test@no_inertia-data["y"][test]
    candidate_error = data["a"][test]*beta[0]+z_test@beta[1:]-data["y"][test]
    return dict(holdout_ramp_ms=heldout_ramp, training_position_center_rad=center,
        training_coefficients=beta, **information,
        without_new_inertia=score(baseline_error, data["group"][test]),
        with_new_inertia=score(candidate_error, data["group"][test]))


def analyze(baseline_path):
    parts, baseline = collect(baseline_path)
    joints = []
    specifications = [("offset_direction", 0, False), ("linear_position", 1, False),
                      ("quadratic_position", 2, False), ("linear_position_speed", 1, True)]
    for joint, data in parts.items():
        axis, models = joint-1, []
        for name, degree, speed_term in specifications:
            parameter_order = ["extra_inertia_kg_m2", "constant_bias_nm", "direction_residual_nm"]
            parameter_order += [f"normalized_position_power_{power}_nm"
                                for power in range(1, degree+1)]
            if speed_term:
                parameter_order.append("signed_velocity_coefficient_nm_s_rad")
            z, center = position_design(data["q"][:, axis], data["v"],
                balanced(data["group"]), degree, speed_term)
            beta, information = partial_inertia(data["a"], z, data["y"], data["group"])
            models.append(dict(name=name, position_degree=degree, speed_term=speed_term,
                parameter_order=parameter_order,
                final_coefficients=beta, final_position_center_rad=center, **information,
                whole_ramp_holdouts=[evaluate(data, axis, degree, speed_term, ramp)
                                    for ramp in (600, 300, 1200)]))
        pairs = [matched_acceleration(data["q"], data["v"], data["a"], data["y"],
            data["ramp"], data["group"], data["time"], axis, tolerance)
            for tolerance in (.5, .25)]
        joints.append(dict(joint=joint, sliding_samples=len(data["q"]), models=models,
                           matched_acceleration_sensitivity=pairs))
    return plain(dict(status="offline_position_acceleration_separation_not_deployable",
        deploy=False, baseline=str(baseline_path), baseline_sha256=sha(baseline_path),
        captures=baseline["captures"], joints=joints,
        comparison_policy="Linear-position model is primary; others are sensitivity checks, not selected by best holdout.",
        feature_definitions=dict(normalized_position="(q_axis-training_center_rad)/.25",
            direction="tanh(actual_velocity_rad_s/.02)", signed_velocity="actual_velocity_rad_s"),
        score_weighting="Equal total weight per capture for RMSE/bias/MAE/fraction; P95 is unweighted.",
        paired_rules=dict(training_ramps_ms=[300,1200], holdout_excluded_ms=600,
            same_signed_velocity=True, max_velocity_difference_rad_s=.03,
            max_relative_velocity_difference=.10, min_acceleration_difference_rad_s2=.25,
            pose_tolerances_deg=[.5,.25], all_five_pose=True, sample_reuse=False),
        source_sha256={str(p):sha(p) for p in [Path(__file__), ROOT/"tools/five_friction_map.py",
            ROOT/"tools/five_inertia_identify.py", ROOT/"tools/five_identify.py",
            ROOT/"tools/five_friction_global.py", ROOT/"tools/five_gravity_fit.py",
            ROOT/"tools/five_gravity_residual_report.py", ROOT/"config/five_coordinates.h",
            ROOT/"config/five_scene.inc", ROOT/"config/five_mit.c", ROOT/"config/joint_references.def"]},
        limitations=[
            "Only the same 603 eligible samples and 12 completed legs; no new motion, labels or relaxed acquisition rules.",
            "Position-even nuisance is empirical, not a new gravity calibration; no firmware changes.",
            "Whole-profile holdouts are retrospective diagnostics, not independent new physical experiments.",
            "Matched pairs are adjacent/conditionally dependent observations, not independent trials or global confidence.",
            "Current-derived torque, filtered velocity derivative, frozen G/F and nominal coupled dynamics remain uncertain.",
            "No rotor/link separation, full five-axis inertia matrix identification or automatic deployment."]))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--baseline", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    result = analyze(args.baseline)
    with args.output.open("x", encoding="utf-8") as output:
        json.dump(result, output, ensure_ascii=False, indent=2, allow_nan=False)
    for joint in result["joints"]:
        for model in joint["models"]:
            fold = model["whole_ramp_holdouts"][0]
            print(json.dumps(dict(joint=joint["joint"], model=model["name"],
                extra_inertia_kg_m2=model["final_coefficients"][0],
                retained_acceleration_fraction=model["acceleration_information_fraction"],
                heldout_without_I=fold["without_new_inertia"]["rmse_nm"],
                heldout_with_I=fold["with_new_inertia"]["rmse_nm"])))
        print(json.dumps(dict(joint=joint["joint"], matched=[
            {key:value for key,value in pairs.items() if key!="matched_pairs"}
            for pairs in joint["matched_acceleration_sensitivity"]])))


if __name__ == "__main__":
    main()
