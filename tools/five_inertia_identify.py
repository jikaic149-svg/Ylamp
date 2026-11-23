"""Offline local J1/J2 equivalent-inertia residual fit; never controls hardware."""
import argparse
import json
from pathlib import Path
import re

import numpy as np

from five_friction_global import balanced, plain, predict, score, sha
from five_friction_map import load_features
from five_gravity_residual_report import support_gravity
from five_identify import HW, NOMINAL, REFS, ROOT, load_run, regressors, scaled_lstsq


def local_design(acceleration, velocity):
    return np.column_stack((acceleration, np.ones(len(velocity)), np.tanh(velocity/.02)))


def local_fit(x, residual, groups):
    weight = np.sqrt(balanced(groups))
    return scaled_lstsq(x*weight[:, None], residual*weight)


def nominal_torque(q, v, a):
    fixed, basis, _ = regressors(q, v, a)
    return fixed+basis@NOMINAL


def analyze(journal):
    rows = [json.loads(s) for s in journal.read_text(encoding="utf-8").splitlines()]
    if (not rows or rows[-1].get("stage") != "inertia_batch_complete" or
            rows[-1]["completed_measurement_legs"] != 12):
        raise ValueError("A complete 12-leg batch is required; never fit a failed prefix")
    timing = {Path(r["capture"]): r for r in rows if r["stage"] == "verified_timing"}
    if len(timing) != 12:
        raise ValueError("Each completed leg needs its independently verified board start")
    frozen_path = ROOT/"build/v12-default-drag-20261123/friction_global_model_02.json"
    if sha(frozen_path) != "d950ff3f2f803d953889eafecb1e52510be575558895e17e22ce6e359f2d8c37":
        raise ValueError("Frozen global friction model changed")
    models = json.loads(frozen_path.read_text(encoding="utf-8"))["models"]
    captures, parts = [], {1: [], 2: []}
    identities = set()
    for path, verified in timing.items():
        identity = re.fullmatch(r"inertia\d+_j([12])_r(1200|0600|0300)_(out|back)", path.stem)
        if not identity:
            raise ValueError("Unexpected first-stage capture identity")
        joint, ramp, direction = identity.groups()
        joint, ramp = int(joint), int(ramp)
        if (joint, ramp, direction) in identities:
            raise ValueError("Duplicate first-stage logical leg")
        identities.add((joint, ramp, direction))
        report, data = load_run(path, with_time=True)
        if report["issues"] or data is None or report["sha256"] != verified["capture_sha256"]:
            raise ValueError(f"Unusable or changed original: {path}: {report['issues']}")
        q, v, a, tau, relative_ms = data
        source_ms = relative_ms+report["analysis_origin_tick_ms"]
        start, duration = verified["board_start_tick_ms"], verified["duration_ms"]
        # Actual measured acceleration, not the planned .5/1/2 labels.
        keep = ((source_ms >= start+40) & (source_ms <= start+duration-40) &
                (np.abs(v[:, joint-1]) >= .05))
        if np.count_nonzero(keep) < 15:
            raise ValueError(f"Insufficient moving data: {path}")
        report.update(joint=joint, ramp_ms=ramp, direction=direction,
                      retained_sliding_samples=int(keep.sum()),
                      verified_start_tick_ms=start, duration_ms=duration)
        captures.append(report)
        parts[joint].append((q[keep], v[keep], a[keep], tau[keep], ramp, path.name))
    result = []
    for joint, legs in parts.items():
        q, v, a, tau = (np.concatenate([leg[k] for leg in legs]) for k in range(4))
        ramps = np.concatenate([np.full(len(leg[0]), leg[4]) for leg in legs])
        groups = np.concatenate([np.full(len(leg[0]), leg[5], dtype=object) for leg in legs])
        axis, zero = joint-1, np.zeros_like(v)
        nominal_g = nominal_torque(q, zero, zero)
        dynamics = nominal_torque(q, v, a)-nominal_g
        z = load_features(q)[0][:, axis]
        friction = predict(models[axis]["final_model"], z, np.clip(abs(v[:, axis]), .08, .8))
        friction *= np.tanh(v[:, axis]/HW[axis, 4])
        residual = tau[:, axis]-support_gravity(q)[:, axis]-dynamics[:, axis]-friction
        x = local_design(a[:, axis], v[:, axis])
        train, test = ramps != 600, ramps == 600
        frozen_beta, frozen_condition = local_fit(x[train], residual[train], groups[train])
        baseline, _ = local_fit(x[train, 1:], residual[train], groups[train])
        heldout = dict(
            fixed_GF_and_nominal_dynamics=score(-residual[test], groups[test]),
            nuisance_bias_and_direction_only=score(x[test, 1:]@baseline-residual[test], groups[test]),
            added_local_inertia=score(x[test]@frozen_beta-residual[test], groups[test]))
        beta, condition = local_fit(x, residual, groups)
        angle_beta, angle_condition = local_fit(
            np.column_stack((x, q[:, axis])), residual, groups)
        acceleration_one = np.zeros_like(a)
        acceleration_one[:, axis] = 1.
        diagonal = (nominal_torque(q, zero, acceleration_one)-nominal_g)[:, axis]
        result.append(dict(joint=joint, sliding_samples=len(q),
            parameter_order=["extra_equivalent_inertia_kg_m2", "constant_bias_nm",
                             "direction_residual_nm"],
            final_refit=beta, scaled_condition=condition,
            frozen_training_coefficients=frozen_beta, frozen_training_condition=frozen_condition,
            holdout_ramp_ms=600, training_ramps_ms=[300, 1200], heldout_residuals=heldout,
            in_sample_residual=score(x@beta-residual, groups),
            linear_gravity_trend_sensitivity=dict(extra_inertia_kg_m2=angle_beta[0],
                delta_from_primary_kg_m2=angle_beta[0]-beta[0], scaled_condition=angle_condition),
            nominal_diagonal_inertia_kg_m2=np.percentile(diagonal, [0, 50, 100]),
            implied_local_diagonal_inertia_kg_m2=np.percentile(diagonal+beta[0], [0, 50, 100]),
            actual_velocity_rad_s=np.percentile(v[:, axis], [0, 50, 100]),
            actual_acceleration_rad_s2=np.percentile(a[:, axis], [0, 5, 50, 95, 100]),
            continuous_reference_ranges_deg=np.rad2deg(
                np.column_stack((q.min(axis=0), q.max(axis=0))))-HW[:, 6, None]+
                np.rad2deg(REFS[:, :1])))
    return plain(dict(status="local_equivalent_inertia_exploratory_not_deployable", deploy=False,
        journal=str(journal), journal_sha256=sha(journal), captures=captures, joints=result,
        frozen_friction_sha256=sha(frozen_path),
        source_sha256={str(p): sha(p) for p in
            [Path(__file__), ROOT/"tools/five_identify.py", ROOT/"tools/five_friction_global.py",
             ROOT/"tools/five_gravity_residual_report.py", ROOT/"config/five_mit.c",
             ROOT/"config/five_scene.inc", ROOT/"config/joint_references.def"]},
        limitations=[
            "Single load configuration and 12 legs, not full five-axis dynamics identification.",
            "Joint-side local extra inertia is confounded with correlated unmodelled dynamics; no rotor/link separation.",
            "Frozen G/F and nominal cross-axis/velocity dynamics are assumptions, not independent torque truth.",
            "CAN current-derived torque and velocity-derived acceleration; 100 Hz grid, 80 ms derivative support.",
            "Whole 600 ms ramp pairs are held out, not randomized frames or a blind old/new controller A/B.",
            "Only |measured velocity| >= .05 rad/s; no static-friction claim.",
            "Command components are not from one atomic control tick; no firmware parameters changed."]))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--journal", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    result = analyze(args.journal)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
    for joint in result["joints"]:
        print(json.dumps(dict(joint=joint["joint"], final_refit=joint["final_refit"],
            implied_local_inertia=joint["implied_local_diagonal_inertia_kg_m2"],
            heldout=joint["heldout_residuals"], sensitivity=joint["linear_gravity_trend_sensitivity"])))


if __name__ == "__main__":
    main()
