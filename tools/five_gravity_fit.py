"""Offline coupled gravity-residual identification. No serial, motion or deployment.

G_new(q) = G_CAD(q) + Yg(q) @ delta. Fit against measured drive torque minus
nominal inverse dynamics, with separate odd-velocity friction nuisance terms.
The existing J3 manual scale/cosine is NOT part of the nominal CAD baseline:
a validated deployed candidate must REPLACE, not double-count, that trim.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from dm_monitor import JOINT_REFERENCES
from five_identify import HW, NOMINAL, ROOT, SCENE, load_run, regressors, scaled_lstsq

PARAMETERS = [f"delta_{name}_J{i}" for i in range(1, 6)
              for name in ("mass_kg", "hx_kg_m", "hy_kg_m", "hz_kg_m")]
PARAMETERS += ["nominal_load_gravity_x_m_s2", "nominal_load_gravity_z_m_s2"]


def gravity_basis(q):
    """Return (N,5,22) analytic potential-gradient basis, q in CAD radians.

Twenty mass/first-moment columns share the five-axis geometry. Two additional
columns use nominal load under unit base-X/Z acceleration; they permit a
conservative yaw-dependent residual without asserting a measured base tilt.
Coefficients are effective combinations, not unique mass/COM measurements.
"""
    q = np.asarray(q, dtype=float)
    if q.ndim != 2 or q.shape[1] != 5 or not np.isfinite(q).all():
        raise ValueError("Expected finite N by 5 CAD radians")
    n = len(q)
    rotation = np.broadcast_to(np.eye(3), (n, 3, 3)).copy()
    origin = np.zeros((n, 3))
    origins, axes = [], []
    y = np.zeros((n, 5, 22))
    acceleration = np.array([0.0, -9.81, 0.0])
    for i, link in enumerate(SCENE):
        origin = origin + np.einsum("nij,j->ni", rotation, link[:3])
        fixed = rotation @ link[6:15].reshape(3, 3)
        axis = -fixed[:, :, 1]
        origins.append(origin.copy())
        axes.append(axis)
        c, s = np.cos(q[:, i]), np.sin(q[:, i])
        local = np.zeros((n, 3, 3))
        local[:, 0, 0] = local[:, 2, 2] = c
        local[:, 0, 2], local[:, 2, 0], local[:, 1, 1] = -s, s, 1
        rotation = fixed @ local
        nominal_h = link[-2] * link[3:6]
        for j in range(i + 1):
            lever = origin - origins[j]
            y[:, j, 4*i] = np.cross(axes[j], lever) @ acceleration
            for k in range(3):
                y[:, j, 4*i+1+k] = np.cross(axes[j], rotation[:, :, k]) @ acceleration
            first_moment = link[-2] * lever + np.einsum("nij,j->ni", rotation, nominal_h)
            force_basis = np.cross(axes[j], first_moment)
            y[:, j, 20] += force_basis[:, 0]
            y[:, j, 21] += force_basis[:, 2]
    return y


def nominal_parameters():
    p = np.zeros(22)
    for i, link in enumerate(SCENE):
        p[4*i:4*i+4] = np.r_[link[-2], link[-2] * link[3:6]]
    return p


def structural_basis():
    """Remove intrinsic parameter ambiguities, not missing measured excitation."""
    rng = np.random.default_rng(20261119)
    q = np.column_stack([
        rng.uniform(r.lower_rad, r.upper_rad, 512) + np.radians(HW[i, 6])
        for i, r in enumerate(JOINT_REFERENCES.values())])
    reference = gravity_basis(q).reshape(-1, 22)
    norm = np.linalg.norm(reference, axis=0)
    norm[norm < 1e-12] = 1.0
    _, singular, vh = np.linalg.svd(reference / norm, full_matrices=False)
    rank = int(np.sum(singular > singular[0] * 1e-10))
    return vh[:rank].T / norm[:, None], singular.tolist()


def design(data, transform, max_dynamic_nm):
    q, v, a, torque, times = [np.asarray(x, dtype=float) for x in data]
    if (q.ndim != 2 or q.shape[1] != 5 or any(x.shape != q.shape for x in (v, a, torque))
            or times.shape != (len(q),) or len(q) < 15
            or any(not np.isfinite(x).all() for x in (q, v, a, torque, times))
            or np.any(np.diff(times) <= 0)):
        raise ValueError("Expected finite measured five-axis arrays and increasing MCU times")
    fixed, dynamic, friction = regressors(q, v, a)
    inverse = fixed + dynamic @ NOMINAL
    nominal_g = gravity_basis(q) @ nominal_parameters()
    # Exclude stationary equations: direction of static friction is unobserved.
    # Keep dynamic corrections small: their nominal model is not re-identified here.
    active = ((np.abs(v) >= .02) & (np.abs(v) <= .20)
              & (np.abs(inverse - nominal_g) <= max_dynamic_nm))
    for i in range(5):
        if min(np.sum(active[:, i] & (v[:, i] > 0)),
               np.sum(active[:, i] & (v[:, i] < 0))) < 15:
            raise ValueError(f"J{i+1}: insufficient measured low-speed bidirectional excitation")
    x = np.concatenate((gravity_basis(q) @ transform, friction), axis=2)
    weights = np.zeros_like(v)
    for i in range(5):
        for direction in (-1, 1):
            keep = active[:, i] & (v[:, i] * direction > 0)
            weights[keep, i] = 1.0 / np.sqrt(np.sum(keep))
    return dict(x=x, target=torque-inverse, active=active, weights=weights,
                nominal_g=nominal_g, inverse=inverse, friction=friction,
                q=q, v=v, times=times)


def solve(parts, selections=None):
    rows, target = [], []
    if selections is None:
        selections = [(i, np.arange(len(p["q"]))) for i, p in enumerate(parts)]
    for run, selected in selections:
        p = parts[run]
        keep = p["active"][selected]
        w = p["weights"][selected][keep]
        rows.append(p["x"][selected][keep] * w[:, None])
        target.append(p["target"][selected][keep] * w)
    return scaled_lstsq(np.concatenate(rows), np.concatenate(target))


def score(part, coeff, gravity_rank):
    pred = part["x"] @ coeff
    correction = part["x"][:, :, :gravity_rank] @ coeff[:gravity_rank]
    def rms(values):
        return [float(np.sqrt(np.mean(values[part["active"][:, i], i]**2))) for i in range(5)]
    return dict(eligible_per_joint=part["active"].sum(axis=0).tolist(),
                nominal_inverse_rmse_nm=rms(part["target"]),
                corrected_with_friction_rmse_nm=rms(part["target"] - pred),
                gravity_without_friction_rmse_nm=rms(part["target"] - correction),
                peak_gravity_correction_nm=np.max(np.abs(correction), axis=0).tolist(),
                cad_span_deg=np.degrees(np.ptp(part["q"], axis=0)).tolist())


def identify(training, validation, *, bootstrap=400, block_seconds=2.0,
             max_dynamic_nm=.03, seed=20261119):
    if not training or not validation:
        raise ValueError("Independent training and holdout runs are required")
    if bootstrap < 20 or not .1 <= block_seconds <= 60 or not 0 < max_dynamic_nm <= .2:
        raise ValueError("Invalid confidence/dynamic-selection settings")
    transform, structural_singular = structural_basis()
    rank = transform.shape[1]
    train = [design(d, transform, max_dynamic_nm) for d in training]
    test = [design(d, transform, max_dynamic_nm) for d in validation]
    coeff, condition = solve(train)  # Missing measured rank is an error, never silently discarded.
    blocks = []
    for i, p in enumerate(train):
        labels = np.floor((p["times"]-p["times"][0])/(1000*block_seconds)).astype(int)
        blocks.extend((i, np.flatnonzero(labels == b)) for b in np.unique(labels))
    if len(blocks) < 8:
        raise ValueError("Fewer than eight measured time blocks: confidence not established")
    rng = np.random.default_rng(seed)
    bootstrap_coeff, failed = [], 0
    for _ in range(bootstrap):
        draw = [blocks[i] for i in rng.integers(0, len(blocks), len(blocks))]
        try:
            bootstrap_coeff.append(solve(train, draw)[0])
        except ValueError:
            failed += 1
    if len(bootstrap_coeff) < max(20, .8 * bootstrap):
        raise ValueError("Too many rank-deficient bootstrap draws")
    delta = transform @ coeff[:rank]
    cloud = np.asarray(bootstrap_coeff)
    delta_cloud = cloud[:, :rank] @ transform.T
    reports = [score(p, coeff, rank) for p in test]
    warnings = []
    if condition > 1e4:
        warnings.append("Poor measured normalized conditioning")
    if np.any(coeff[rank:] < 0):
        warnings.append("Negative friction nuisance coefficient; no physical friction claim")
    if any(any(a >= b for a, b in zip(r["corrected_with_friction_rmse_nm"],
                                     r["nominal_inverse_rmse_nm"])) for r in reports):
        warnings.append("Held-out improvement not demonstrated on every joint/run")
    if failed:
        warnings.append(f"{failed}/{bootstrap} bootstrap draws had insufficient rank")
    representative = test[0]["q"][len(test[0]["q"])//2].copy()
    conditional = []
    for i in range(5):
        cosine_pose, sine_pose = representative.copy(), representative.copy()
        cosine_pose[i], sine_pose[i] = 0, np.pi/2
        cosine = (gravity_basis(cosine_pose[None]) @ delta)[0, i]
        sine = (gravity_basis(sine_pose[None]) @ delta)[0, i]
        conditional.append(dict(joint=i+1, cos_nm=float(cosine), sin_nm=float(sine)))
    validation_intervals = []
    for p in test:
        residual = p["x"][:, :, :rank] @ cloud[:, :rank].T
        validation_intervals.append(dict(
            median_conditional_95_width_nm=np.median(
                np.percentile(residual, 97.5, axis=2)-np.percentile(residual, 2.5, axis=2),
                axis=0).tolist()))
    return dict(status="candidate_not_deployed", deploy=False,
                baseline="Nominal CAD inverse dynamics, NOT J3 manual support trim",
                correction_formula="delta_G(qCAD)=gravity_basis(qCAD) @ potential_delta",
                implementation="tools/five_gravity_fit.py:gravity_basis",
                parameter_names=PARAMETERS, potential_delta=delta.tolist(),
                effective_parameter_conditional_95_interval=np.percentile(
                    delta_cloud, [2.5, 97.5], axis=0).T.tolist(),
                gravity_structural_rank=rank, structural_singular_values=structural_singular,
                measured_scaled_condition=condition,
                friction_nuisance_fc_b=coeff[rank:].tolist(),
                training=[score(p, coeff, rank) for p in train], holdout=reports,
                conditional_sine_cosine_example=dict(
                    other_angles_cad_deg=np.degrees(representative).tolist(), coefficients=conditional,
                    warning="Conditional slice only; coefficients change with other joint angles"),
                holdout_conditional_95_intervals=validation_intervals,
                confidence=dict(block_seconds=block_seconds, blocks=len(blocks), requested=bootstrap,
                                successful=len(cloud), seed=seed,
                                scope="Conditional on geometry, torque scale, data and nuisance model"),
                selection=dict(min_abs_speed_rad_s=.02, max_abs_speed_rad_s=.20,
                               max_nominal_dynamic_torque_nm=max_dynamic_nm),
                candidate_warnings=warnings,
                limitations=["Drive-current torque is not independently calibrated shaft torque",
                             "Static friction/unknown external loads are not identified by this fit",
                             "Effective potential parameters are not unique physical mass/COM values",
                             "Nominal M/c are subtracted, not identified or scaled",
                             "No full-workspace coverage or motion-safety claim",
                             "Replace absorbed J3 manual scale/cosine when deploying, never double count"])


def fit_files(training, validation, **options):
    all_runs = [load_run(path, with_time=True) for path in training + validation]
    reports = [r for r, _ in all_runs]
    result = dict(status="insufficient_data", deploy=False, captures=reports)
    if any(r["issues"] for r in reports):
        return result
    for i, r in enumerate(reports):
        for previous in reports[:i]:
            lo, hi = r["unix_interval"]
            a, b = previous["unix_interval"]
            if (r["session"] == previous["session"] or r["sha256"] == previous["sha256"]
                    or max(lo, a) <= min(hi, b)):
                result["reason"] = "Duplicate/overlapping run, not independent validation"
                return result
    try:
        result.update(identify([d for _, d in all_runs[:len(training)]],
                               [d for _, d in all_runs[len(training):]], **options))
    except ValueError as exc:
        result["reason"] = str(exc)
    result["source_sha256"] = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                              for p in (ROOT/"config/five_scene.inc",
                                        ROOT/"config/five_coordinates.h",
                                        ROOT/"config/five_mit.c",
                                        ROOT/"config/joint_references.def",
                                        ROOT/"tools/five_identify.py",
                                        Path(__file__))}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--train", nargs="+", type=Path, required=True)
    parser.add_argument("--holdout", nargs="+", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--bootstrap", type=int, default=400)
    parser.add_argument("--block-seconds", type=float, default=2.0)
    args = parser.parse_args()
    if not args.train or not args.holdout:
        parser.error("Independent input runs are required")
    report = fit_files(args.train, args.holdout, bootstrap=args.bootstrap,
                       block_seconds=args.block_seconds)
    with args.out.open("x", encoding="utf-8") as out:
        json.dump(report, out, indent=2, allow_nan=False)
    print(json.dumps({k: v for k, v in report.items() if k != "captures"}, indent=2))
    return 0 if report["status"] == "candidate_not_deployed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
