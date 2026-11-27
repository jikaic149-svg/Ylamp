"""Exploratory J2..J5 gravity/friction analysis of existing healthy motion windows.

Never opens a serial port, changes capture labels, writes firmware or authorizes
deployment. Incomplete runs and unconfirmed contact remain explicit limitations.
"""
import argparse
import csv
import hashlib
import json
import math
import re
from pathlib import Path

import numpy as np

from dm_monitor import JOINT_REFERENCES
from five_control import CONTROL_VERSION, EXPECTED, PURE_GRAVITY_PHASES
from five_gravity_fit import gravity_basis, nominal_parameters, solve
from five_identify import HW, MASK, NOMINAL, REFS, ROOT, align_feedback, regressors, scaled_lstsq

FRICTION_COLUMNS = [1, 2, 3, 4, 6, 7, 8, 9]


def healthy(record):
    return (record.get("phase") in (*PURE_GRAVITY_PHASES, "MOTION", "POSITION")
            and not record.get("fault") and not record.get("limited")
            and all(record.get(k) == 31 for k in ("fresh", "enabled", "submitted"))
            and record.get("gravity_age_ms") is not None
            and 0 <= record["gravity_age_ms"] <= 100)


def read_analysis_run(path):
    """Keep original audit/provenance; use only bounded, observed healthy windows."""
    digest = hashlib.sha256()
    meta, fast, states, waypoints = {}, [], [], []
    origin = latest = None
    end = handover = None
    with path.open("rb") as source:
        for raw in source:
            digest.update(raw)
            obj = json.loads(raw.decode("utf-8-sig"))
            stage = obj.get("stage")
            if stage == "capture":
                if meta:
                    raise ValueError("Multiple capture headers")
                meta = obj
            if stage == "end":
                end = obj
            if stage == "handover":
                handover = obj
            telemetry = obj.get("telemetry")
            if telemetry:
                tick = telemetry["tick_ms"]
                if origin is None:
                    origin = tick
                relative = (tick-origin) & MASK
                if latest is not None and relative < latest:
                    raise ValueError("MCU timestamp went backwards")
                latest = relative
                if [(j["joint"], j["bus"], j["protocol"], j["command_id"], j["feedback_id"])
                        for j in telemetry["joints"]] != EXPECTED:
                    raise ValueError("Not the V1.2 five-axis motor layout")
                if stage == "feedback":
                    fast.append(telemetry)
                elif "phase" in obj:
                    states.append((relative, healthy(obj)))
            if stage == "waypoint":
                if latest is None or not 1 <= obj["joint"] <= 5 or not 100 <= obj["duration_ms"] <= 120000:
                    raise ValueError("Invalid waypoint metadata")
                waypoints.append(dict(at_ms=latest, joint=obj["joint"], leg=obj["leg"],
                                      duration_ms=obj["duration_ms"]))
    if (meta.get("schema") != "ylamp-five-id-1" or meta.get("control_version") != CONTROL_VERSION
            or meta.get("action") != "move"
            or not re.fullmatch("[0-9a-f]{16}", str(meta.get("session", "")))):
        raise ValueError("Missing verified five-axis motion session")
    refs = np.asarray(meta.get("references_rad", []))
    if refs.shape != (5, 3) or not np.allclose(refs, REFS, rtol=0, atol=2e-6):
        raise ValueError("Mismatched board references")
    if not waypoints or not states or not states[0][1] or latest is None or not 100 <= latest < 86400000:
        raise ValueError("Insufficient healthy motion/timeline metadata")
    report = dict(file=str(path), sha256=digest.hexdigest(), session=meta["session"],
                  analysis_only=True, deploy=False,
                  sequence_completed=bool(end and handover and healthy(handover)
                                          and handover["phase"] in PURE_GRAVITY_PHASES),
                  free_space_confirmed=meta.get("free_space") is True,
                  issues=[], waypoints=waypoints)
    note = path.with_suffix(".operator_confirmation.json")
    if note.is_file():
        confirmation = json.loads(note.read_text(encoding="utf-8"))
        if (confirmation.get("capture_sha256", "").lower() == report["sha256"]
                and confirmation.get("free_space_confirmed") is True
                and confirmation.get("original_capture_modified") is False):
            report["free_space_confirmed"] = True
            report["confirmation"] = str(note)
    # This API does NOT call the strict deployment loader or erase its issues.
    report, data = align_feedback(fast, origin, latest, report, with_time=True)
    if data is None:
        raise ValueError("; ".join(report["issues"]))
    q, v, a, torque, times = data
    cutoff = min((tick-1000 for tick, ok in states if not ok), default=latest-1000)
    report["first_unhealthy_cutoff_ms"] = float(cutoff)
    status_t = np.array([s[0] for s in states])
    status_ok = np.array([s[1] for s in states])
    preceding = np.searchsorted(status_t, times, side="right")-1
    following = np.minimum(preceding+1, len(states)-1)
    observed = ((preceding >= 0) & (times <= cutoff)
                & status_ok[np.maximum(preceding, 0)] & status_ok[following])
    moving_joint = np.zeros(len(times), int)
    for i, point in enumerate(waypoints):
        right = min(point["at_ms"]+point["duration_ms"]-1000, cutoff)
        if i+1 < len(waypoints):
            right = min(right, waypoints[i+1]["at_ms"]-1000)
        keep = observed & (times >= point["at_ms"]+1000) & (times <= right)
        moving_joint[keep] = point["joint"]
    report["unconfirmed_contact_warning"] = not report["free_space_confirmed"]
    return report, (q, v, a, torque, times, moving_joint)


def gravity_transform():
    """Eight structural gravity combinations; no base-tilt/G1 correction."""
    rng = np.random.default_rng(20261119)
    q = np.column_stack([rng.uniform(r.lower_rad, r.upper_rad, 512)+math.radians(HW[i, 6])
                         for i, r in enumerate(JOINT_REFERENCES.values())])
    reference = gravity_basis(q)[:, 1:, :20].reshape(-1, 20)
    norm = np.linalg.norm(reference, axis=0)
    norm[norm < 1e-12] = 1
    _, singular, vh = np.linalg.svd(reference/norm, full_matrices=False)
    rank = int(np.sum(singular > singular[0]*1e-10))
    transform = np.zeros((22, rank))
    transform[:20] = vh[:rank].T/norm[:, None]
    return transform


def support_gravity(q, *, clip=True):
    """Current configured support; clip=False exposes raw planning demand.

    Not a claim about a capture's flashed image. The default preserves the
    historical bounded-support analysis, never use it to prove budget margin.
    """
    basis = gravity_basis(q)
    result = basis @ nominal_parameters()
    config = (ROOT/"config/five_mit.c").read_text(encoding="utf-8")
    table = config.split("g_five_j2_gravity_delta[FIVE_AXES][4] = {", 1)[1].split("};", 1)[0]
    delta = np.array([[float(value.strip().rstrip("f")) for value in row.split(",")]
                      for row in re.findall(r"\{([^{}]+)\}", table)])
    if delta.shape != (5, 4) or not np.isfinite(delta).all():
        raise ValueError("Invalid current J2 gravity residual configuration")
    # The commissioning residual is intentionally scoped to J2 output. Do not
    # reinterpret it as measured masses, modify M/c or apply the weak J4/J5 fit.
    result[:, 1] += (basis[:, :, :20] @ delta.ravel())[:, 1]
    def scalar(name):
        return float(re.search(name+r"\s*=\s*([-+.\deE]+)f?\s*;", config)[1])
    result[:, 2] *= scalar("g_five_j3_gravity_scale")
    result[:, 2] += scalar("g_five_j3_gravity_cosine_nm")*np.cos(q[:, 2]-math.radians(HW[2, 6]))
    return np.clip(result, -HW[:, 5], HW[:, 5]) if clip else result


def prepare(data, transform, max_dynamic_nm=.03):
    q, v, a, torque, times, moving_joint = data
    fixed, y, friction = regressors(q, v, a)
    inverse = fixed+y@NOMINAL
    nominal_g = gravity_basis(q)@nominal_parameters()
    active = ((moving_joint[:, None] == np.arange(1, 6))
              & (np.abs(v) >= .02) & (np.abs(v) <= .20)
              & (np.abs(inverse-nominal_g) <= max_dynamic_nm))
    active[:, 0] = False
    return dict(x=np.concatenate((gravity_basis(q)@transform,
                                  friction[:, :, FRICTION_COLUMNS]), axis=2),
                target=torque-inverse, active=active, weights=np.zeros_like(v),
                q=q, v=v, times=times, nominal_dynamic=inverse-nominal_g,
                current_trim=support_gravity(q)-nominal_g)


def set_weights(parts):
    for joint in range(1, 5):
        for direction in (-1, 1):
            selections = [p["active"][:, joint] & (p["v"][:, joint]*direction > 0) for p in parts]
            count = sum(int(np.sum(keep)) for keep in selections)
            if count < 15:
                raise ValueError(f"J{joint+1}: fewer than 15 moving samples in direction {direction}")
            for p, keep in zip(parts, selections):
                p["weights"][keep, joint] = 1/np.sqrt(count)


def fit_analysis(data, *, bootstrap=400, seed=20261119):
    if bootstrap < 20:
        raise ValueError("At least 20 block bootstrap draws required")
    transform = gravity_transform()
    rank = transform.shape[1]
    all_parts = [prepare(d, transform) for d in data]
    # Choose whole time blocks using excitation counts only, never torque
    # residuals. A fixed modulo split can miss a short positive J5 sweep.
    candidates = []
    for run, part in enumerate(all_parts):
        label = np.floor(part["times"]/2000).astype(int)
        for block in np.unique(label[np.any(part["active"], axis=1)]):
            selected = label == block
            counts = np.array([[np.sum(selected & part["active"][:, j] & (part["v"][:, j]*d > 0))
                                for d in (-1, 1)] for j in range(1, 5)])
            candidates.append((run, selected, counts))
    total = sum((b[2] for b in candidates), np.zeros((4, 2), int))
    if np.any(total < 30):
        raise ValueError("Fewer than 30 samples in a joint/direction; cannot split train/holdout")
    desired = np.maximum(15, np.rint(total*.2).astype(int))
    held = np.zeros((4, 2), int)
    holdout_mask = [np.zeros(len(p["times"]), bool) for p in all_parts]
    while np.any(held < desired):
        available = [(i, float(np.sum(np.minimum(b[2], np.maximum(desired-held, 0))/desired)))
                     for i, b in enumerate(candidates) if np.all(total-held-b[2] >= 15)]
        if not available or max(score for _, score in available) <= 0:
            raise ValueError("Whole-block holdout cannot preserve bidirectional training excitation")
        index = max(available, key=lambda item: item[1])[0]
        run, selected, counts = candidates.pop(index)
        holdout_mask[run] |= selected
        held += counts
    train, test = [], []
    for part, held_mask in zip(all_parts, holdout_mask):
        for group, keep in ((train, ~held_mask), (test, held_mask)):
            copy = dict(part, active=part["active"] & keep[:, None], weights=np.zeros_like(part["v"]))
            group.append(copy)
    set_weights(train)
    set_weights(test)
    coeff, condition = solve(train)
    # Fit friction alone against CURRENT support as a fair baseline.
    rows = np.concatenate([p["x"][:, :, rank:][p["active"]]*p["weights"][p["active"]][:, None]
                           for p in train])
    target = np.concatenate([(p["target"]-p["current_trim"])[p["active"]]
                             *p["weights"][p["active"]] for p in train])
    baseline_friction, _ = scaled_lstsq(rows, target)
    blocks = []
    for run, p in enumerate(train):
        label = np.floor(p["times"]/2000).astype(int)
        for b in np.unique(label[np.any(p["active"], axis=1)]):
            blocks.append((run, np.flatnonzero(label == b)))
    if len(blocks) < 8:
        raise ValueError("Fewer than eight measured time blocks")
    rng = np.random.default_rng(seed)
    cloud, failed = [], 0
    for _ in range(bootstrap):
        draw = [blocks[i] for i in rng.integers(0, len(blocks), len(blocks))]
        try:
            cloud.append(solve(train, draw)[0])
        except ValueError:
            failed += 1
    if len(cloud) < max(20, .8*bootstrap):
        raise ValueError("Too many rank-deficient block bootstrap draws")
    cloud = np.asarray(cloud)
    delta = transform@coeff[:rank]
    no_dynamic_coeff, _ = solve([dict(p, target=p["target"]+p["nominal_dynamic"]) for p in train])
    cross_record = []
    for held_run, held_part in enumerate(all_parts):
        active = held_part["active"][:, 1]
        if min(np.sum(active & (held_part["v"][:, 1] > 0)),
               np.sum(active & (held_part["v"][:, 1] < 0))) < 15:
            continue
        remaining = [dict(p, weights=np.zeros_like(p["v"]))
                     for i, p in enumerate(all_parts) if i != held_run]
        try:
            set_weights(remaining)
            other_coeff, _ = solve(remaining)
            x = np.concatenate([p["x"][:, :, rank:][p["active"]]*p["weights"][p["active"]][:, None]
                                for p in remaining])
            y = np.concatenate([(p["target"]-p["current_trim"])[p["active"]]
                                *p["weights"][p["active"]] for p in remaining])
            old_friction, _ = scaled_lstsq(x, y)
            current = (held_part["target"]-held_part["current_trim"])[:, 1]
            before = current-held_part["x"][:, 1, rank:]@old_friction
            after = held_part["target"][:, 1]-held_part["x"][:, 1]@other_coeff
            cross_record.append(dict(held_run=held_run, joint=2, selected_samples=int(sum(active)),
                                     current_with_friction_rmse_nm=float(np.mean(before[active]**2)**.5),
                                     candidate_with_friction_rmse_nm=float(np.mean(after[active]**2)**.5)))
        except ValueError as exc:
            cross_record.append(dict(held_run=held_run, joint=2, unavailable=str(exc)))
    reports, slices = [], []
    for joint in range(1, 5):
        keep = [p["active"][:, joint] for p in test]
        def rms(values):
            vector = np.concatenate([value[k] for value, k in zip(values, keep)])
            return float(np.sqrt(np.mean(vector**2)))
        current_error = [p["target"][:, joint]-p["current_trim"][:, joint] for p in test]
        friction_error = [e-p["x"][:, joint, rank:]@baseline_friction for e, p in zip(current_error, test)]
        candidate_error = [p["target"][:, joint]-p["x"][:, joint]@coeff for p in test]
        selected_q = np.concatenate([p["q"][p["active"][:, joint]] for p in all_parts])
        extra = ((gravity_basis(selected_q)@delta)[:, joint]
                 -(support_gravity(selected_q)-gravity_basis(selected_q)@nominal_parameters())[:, joint])
        sensitivity = np.abs((gravity_basis(selected_q)@transform
                              @(no_dynamic_coeff[:rank]-coeff[:rank]))[:, joint])
        reports.append(dict(joint=joint+1, selected_samples=len(selected_q),
                            selected_raw_range_deg=(np.degrees([selected_q[:, joint].min(),
                                                               selected_q[:, joint].max()])
                                                    -HW[joint, 6]+math.degrees(REFS[joint, 0])).tolist(),
                            extra_gravity_5_50_95_percentile_nm=np.percentile(extra, [5, 50, 95]).tolist(),
                            current_without_friction_holdout_rmse_nm=rms(current_error),
                            current_with_friction_holdout_rmse_nm=rms(friction_error),
                            candidate_with_friction_holdout_rmse_nm=rms(candidate_error),
                            nominal_dynamic_subtraction_sensitivity95_nm=float(np.percentile(sensitivity, 95)),
                            fc_nm=float(coeff[rank+joint-1]), b_nm_s_rad=float(coeff[rank+4+joint-1])))
        for run, part in enumerate(all_parts):
            use = part["active"][:, joint]
            if np.sum(use) < 30:
                continue
            pose = np.median(part["q"][use], axis=0)
            theta = part["q"][use, joint]-math.radians(HW[joint, 6])
            grid = np.linspace(theta.min(), theta.max(), 9)
            poses = np.tile(pose, (len(grid), 1))
            poses[:, joint] = grid+math.radians(HW[joint, 6])
            basis = (gravity_basis(poses)@transform)[:, joint]
            old_trim = (support_gravity(poses)-gravity_basis(poses)@nominal_parameters())[:, joint]
            estimate = basis@coeff[:rank]-old_trim
            interval = np.percentile(basis@cloud[:, :rank].T-old_trim[:, None], [2.5, 97.5], axis=1).T
            harmonic = np.tile(pose, (2, 1))
            harmonic[:, joint] = np.array([0., np.pi/2])+math.radians(HW[joint, 6])
            hb = (gravity_basis(harmonic)@transform)[:, joint]
            ht = (support_gravity(harmonic)-gravity_basis(harmonic)@nominal_parameters())[:, joint]
            hc = hb@coeff[:rank]-ht
            hi = np.percentile(hb@cloud[:, :rank].T-ht[:, None], [2.5, 97.5], axis=1).T
            slices.append(dict(joint=joint+1, run=run,
                               other_angles_raw_deg=(np.degrees(pose)-HW[:, 6]+np.degrees(REFS[:, 0])).tolist(),
                               raw_deg=(np.degrees(grid)+math.degrees(REFS[joint, 0])).tolist(),
                               extra_nm=estimate.tolist(), conditional_95_nm=interval.tolist(),
                               conditional_formula="delta_G=A_cos*cos(q_software_rad)+A_sin*sin(q_software_rad)",
                               a_cos_nm=float(hc[0]), a_sin_nm=float(hc[1]),
                               a_cos_conditional95_nm=hi[0].tolist(), a_sin_conditional95_nm=hi[1].tolist()))
    warnings = ["Not independently repeated on J3/J4/J5; internal block holdout is not independent validation"]
    if condition > 1e4:
        warnings.append("Poor measured normalized conditioning")
    for item in reports:
        if item["fc_nm"] < 0 or item["b_nm_s_rad"] < 0:
            warnings.append(f"J{item['joint']}: negative friction nuisance coefficient; no physical friction claim")
        if item["candidate_with_friction_holdout_rmse_nm"] >= item["current_with_friction_holdout_rmse_nm"]:
            warnings.append(f"J{item['joint']}: no gravity improvement over the friction-only baseline")
    for item in cross_record:
        if "unavailable" in item or item["candidate_with_friction_rmse_nm"] >= item["current_with_friction_rmse_nm"]:
            warnings.append(f"J2 held recording {item['held_run']}: gravity improvement not independently demonstrated")
    return dict(status="exploratory_not_deployable", deploy=False,
                formula="G_candidate=G_CAD+gravity_basis(qCAD)@potential_delta; replace J3 manual trim",
                correction_relative_to="CURRENT support G, including J3 1.18/cosine trim",
                potential_delta=delta.tolist(), gravity_rank=rank, measured_scaled_condition=condition,
                coefficient_cloud_conditional_95=np.percentile(cloud, [2.5, 97.5], axis=0).T.tolist(),
                friction_coefficients=coeff[rank:].tolist(), joints=reports, conditional_slices=slices,
                cross_record_j2=cross_record, candidate_warnings=warnings,
                selection_per_capture=[dict(counts_positive_negative=[
                    [int(np.sum(p["active"][:, j] & (p["v"][:, j] > 0))),
                     int(np.sum(p["active"][:, j] & (p["v"][:, j] < 0)))]
                    for j in range(1, 5)]) for p in all_parts],
                validation="Whole 2 s block holdout stratified by joint/direction excitation, NOT an independent repeat",
                confidence=dict(bootstrap=bootstrap, successful=len(cloud), failed=failed,
                                blocks=len(blocks), block_seconds=2, seed=seed),
                selection=dict(target_joint_speed_rad_s=[.02, .20], max_nominal_dynamic_nm=.03,
                               waypoint_edge_exclusion_ms=1000, pre_fault_exclusion_ms=1000),
                limitations=["No J1 gravity correction or excitation is fitted",
                             "Only swept-joint equations, with synchronous measured five-axis poses",
                             "Drive-reported torque is not independently calibrated shaft torque",
                             "Nominal CAD dynamic subtraction is NOT identified acceleration/coupling compensation",
                             "Confidence is conditional on geometry, torque scale, contact and friction model",
                             "Incomplete run data are healthy prefixes, never relabelled completed",
                             "Any unconfirmed contact prevents deployment",
                             "Conditional angle slices are not full-workspace correction laws",
                             "Negative friction coefficients or nonimproving holdout require model review"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="+", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--bootstrap", type=int, default=400)
    args = parser.parse_args()
    loaded = [read_analysis_run(p) for p in args.paths]
    for field in ("sha256", "session"):
        if len({r[field] for r, _ in loaded}) != len(loaded):
            raise ValueError("Duplicate capture/session")
    result = fit_analysis([d for _, d in loaded], bootstrap=args.bootstrap)
    result["captures"] = [r for r, _ in loaded]
    if any(not r["free_space_confirmed"] for r, _ in loaded):
        result["candidate_warnings"].append("One or more J2 recordings lack explicit whole-capture no-contact confirmation")
    result["source_sha256"] = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                              for p in (Path(__file__), ROOT/"tools/five_identify.py",
                                        ROOT/"tools/five_gravity_fit.py", ROOT/"config/five_mit.c",
                                        ROOT/"config/five_scene.inc", ROOT/"config/five_coordinates.h",
                                        ROOT/"config/joint_references.def")}
    with args.out.open("x", encoding="utf-8") as out:
        json.dump(result, out, ensure_ascii=False, indent=2, allow_nan=False)
    table = args.out.with_suffix(".csv")
    with table.open("x", newline="", encoding="utf-8-sig") as out:
        writer = csv.writer(out)
        writer.writerow(["joint", "capture", "raw_deg", "additional_nm", "conditional95_low_nm",
                         "conditional95_high_nm", "other_angles_raw_deg"])
        for example in result["conditional_slices"]:
            for degree, force, interval in zip(example["raw_deg"], example["extra_nm"], example["conditional_95_nm"]):
                writer.writerow([example["joint"], example["run"], degree, force, *interval,
                                 json.dumps(example["other_angles_raw_deg"])])
    print(json.dumps(dict(status=result["status"], joints=result["joints"],
                          confidence=result["confidence"],
                          measured_scaled_condition=result["measured_scaled_condition"]), indent=2))


if __name__ == "__main__":
    main()
