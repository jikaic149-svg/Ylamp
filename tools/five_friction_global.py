"""Offline URDF-informed J1..J5 friction candidate; never controls hardware."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np

import five_friction_map as fm
from five_identify import HW, REFS, ROOT, SCENE


def sha(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(k): plain(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [plain(v) for v in value]
    return value


def urdf_audit():
    path = ROOT.parent/"Ylamp-URDF-package(1)/Ylamp.urdf"
    robot = ET.parse(path).getroot()
    errors = []
    for i in range(5):
        joint = robot.find(f"./joint[@name='J{i+1}']")
        body = robot.find(f"./link[@name='{joint.find('child').get('link')}']/inertial")
        xyz = lambda node: np.fromstring(node.get("xyz"), sep=" ")
        roll, pitch, yaw = np.fromstring(joint.find("origin").get("rpy"), sep=" ")
        cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
        rotation = np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                             [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                             [-sp, cp*sr, cp*cr]])
        inertia = body.find("inertia")
        tensor = np.array([[float(inertia.get("i"+a+b if a <= b else "i"+b+a))
                            for b in "xyz"] for a in "xyz"])
        errors.extend((np.max(abs(xyz(joint.find("origin"))-SCENE[i, :3])),
                       np.max(abs(xyz(body.find("origin"))-SCENE[i, 3:6])),
                       np.max(abs(rotation-SCENE[i, 6:15].reshape(3, 3))),
                       np.max(abs(tensor-SCENE[i, 15:24].reshape(3, 3))),
                       abs(float(body.find("mass").get("value"))-SCENE[i, -2])))
        if not np.allclose(xyz(joint.find("axis")), [0, -1, 0], atol=1e-12):
            raise ValueError("URDF joint axis differs from the native model")
    # The existing native export rounds joint origins/Euler angles before
    # forming rotations. Mass, COM and inertia are copied without that rounding.
    origin_error = max(errors[0::5])
    rotation_error = max(errors[2::5])
    other_error = max(e for k, e in enumerate(errors) if k % 5 not in (0, 2))
    if origin_error > 5e-7 or rotation_error > 2e-6 or other_error > 2e-10:
        raise ValueError("URDF and native scene differ; do not invent load features")
    return dict(file=str(path), sha256=sha(path), max_scene_element_difference=max(errors),
                max_native_origin_rounding_m=origin_error,
                max_native_rotation_element_rounding=rotation_error,
                max_other_scene_element_difference=other_error,
                feature_order=["abs_axial/weight", "radial/weight",
                               "bending/(weight*reach)", "abs_drive/(weight*reach)"],
                bearing_normal_force_measured=False)


def collect(directory):
    """Re-audit originals, preserving both directions and independent test tags."""
    frozen_path = ROOT/"build/v12-friction-budget2x-20261122/gravity_from_friction_02.json"
    frozen = json.loads(frozen_path.read_text(encoding="utf-8"))
    specs = []
    for c in frozen["captures"]:
        if c["domain"] == "wide":
            continue  # Gravity profiles remain diagnostics, not extra friction truth.
        specs.append(dict(path=Path(c["file"]), domain=c["domain"], pose=c["pose"],
                          repeat=c["repeat"], grid=c["analysis_grid_ms"],
                          width=c["angle_bin_deg"], tick=c.get("verified_start_tick_ms"),
                          interval=None, expected=c["sha256"]))
    timing, completed = {}, set()
    journals = [directory/name for name in (
        "global_pose01_j12_campaign_01.jsonl", "global_j2_rejoin_01.jsonl",
        "global_pose01_j12_resume_01.jsonl", "global_pose01_j345_campaign_01.jsonl",
        "global_pose01_j345_resume_01.jsonl")]
    pattern = r"global_pose01_r([12])_j([1-5])_0(?:16|48|80)_(?:pos|neg)(?:_unsent_replan_01)?"
    for journal in journals:
        for line in journal.read_text(encoding="utf-8").splitlines():
            r = json.loads(line)
            if "capture" not in r:
                continue
            path = Path(r["capture"])
            if not re.fullmatch(pattern, path.stem):
                continue
            if r["stage"] == "completed":
                completed.add(path)
            if r["stage"] == "verified_timing":
                timing[path] = dict(tick=r["board_start_tick_ms"], interval=None,
                                    expected=r["capture_sha256"])
            elif "board_start_tick_interval_ms" in r and path not in timing:
                timing[path] = dict(tick=None, interval=r["board_start_tick_interval_ms"],
                                    expected=r["capture_sha256"])
    for path in sorted(completed):
        match = re.fullmatch(pattern, path.stem)
        if path not in timing:
            raise ValueError(f"No audited timing for {path}")
        specs.append(dict(path=path, domain="new_global", pose=0, repeat=int(match[1]),
                          grid=4, width=1., **timing[path]))
    for folder in ("v12-j3-gravity-ls-20261122", "v12-five-history-20261122",
                   "v12-friction-high-tail-20261122"):
        for path in sorted((ROOT/"build"/folder).glob("*.jsonl")):
            with path.open("rb") as stream:
                try:
                    header = json.loads(stream.readline())
                except (ValueError, UnicodeError):
                    continue
            if (header.get("action") == "move" and header.get("constant_speed_rad_s") == .8):
                from five_global_friction_scan import source_clock_audit
                timing_audit = source_clock_audit(path, require_free_space=False)
                specs.append(dict(path=path, domain="retest_"+folder, pose=0,
                                  repeat=2, grid=4, width=1., tick=None,
                                  interval=timing_audit["board_start_tick_interval_ms"],
                                  expected=timing_audit["capture_sha256"]))
    pending, audits, points = defaultdict(list), [], []
    seen = set()
    for number, spec in enumerate(specs, 1):
        path = spec["path"].resolve()
        if path in seen:
            raise ValueError("Repeated original capture")
        seen.add(path)
        audit, legs = fm.load_legs(path, spec["pose"], grid_ms=spec["grid"],
            verified_start_tick_ms=spec["tick"], start_tick_interval_ms=spec["interval"])
        if spec["expected"] and audit["sha256"].lower() != spec["expected"].lower():
            raise ValueError(f"Original changed: {path}")
        audit.update(domain=spec["domain"], repeat=spec["repeat"], pose=spec["pose"],
                     derived_legs=len(legs), analysis_grid_ms=spec["grid"])
        audits.append(audit)
        if len(legs) == 1:
            leg = legs[0]
            key = (spec["domain"], spec["pose"], spec["repeat"], leg["joint"], leg["group"][1])
            pending[key].append((leg, audit))
            if len(pending[key]) < 2:
                continue
            if len(pending[key]) != 2:
                raise ValueError("Duplicated directional leg")
            both = pending.pop(key)
            derived = fm.pair_legs([item[0] for item in both], spec["width"], components=True)
            record = "|".join(Path(item[1]["file"]).name for item in both)
        else:
            derived = fm.pair_legs(legs, spec["width"], components=True)
            record = path.name
        for p in derived:
            p.update(domain=spec["domain"], repeat=spec["repeat"], record=record)
            points.append(p)
        if number % 8 == 0:
            print(f"READ {number}/{len(specs)} originals, {len(points)} paired cells", flush=True)
    if pending:
        raise ValueError("Unpaired original single-leg capture")
    return audits, points, {str(p): sha(p) for p in [frozen_path, *journals]}


def balanced(groups):
    counts = Counter(groups)
    w = np.array([1/counts[g] for g in groups])
    return w/w.sum()


def design(z, speed, shape, rank, interaction=False, basis=None, w=None):
    """Training-only URDF feature projection; no full five-angle lookup table."""
    if basis is None:
        w = np.ones(len(speed))/len(speed) if w is None else w
        center = np.average(z, axis=0, weights=w)
        spread = np.sqrt(np.average((z-center)**2, axis=0, weights=w))
        active = spread > 1e-5
        projection = np.zeros((4, rank))
        if rank:
            scaled = (z[:, active]-center[active])/spread[active]
            _, sv, vt = np.linalg.svd(scaled*np.sqrt(w[:, None]), full_matrices=False)
            if len(sv) < rank or sv[rank-1] < sv[0]*.03:
                raise ValueError("Insufficient independent load excitation")
            projection[active] = vt[:rank].T/spread[active, None]
        basis = dict(center=center, projection=projection)
    columns, names = [np.ones(len(speed))], ["intercept"]
    u = np.log(speed/.08)
    if shape != "constant":
        columns.append(u)
        names.append("log(v/.08)")
    if shape == "hinge":
        for knot in (.16, .48):
            columns.append(np.maximum(0., np.log(speed/knot)))
            names.append(f"max(0,log(v/{knot}))")
    pcs = (z-np.asarray(basis["center"]))@np.asarray(basis["projection"]).reshape(4, rank)
    for j in range(rank):
        columns.append(pcs[:, j])
        names.append(f"loadPC{j+1}")
    if interaction:
        columns.append(pcs[:, 0]*u)
        names.append("loadPC1*log(v/.08)")
    return np.column_stack(columns), basis, names


def fit_model(z, speed, target, groups, candidate):
    w = balanced(groups)
    x, basis, names = design(z, speed, *candidate, w=w)
    beta, condition = fm.fit(x, target, w, tuple(range(x.shape[1])))
    return dict(shape=candidate[0], load_rank=candidate[1], interaction=candidate[2],
                coefficients=beta, load_basis=basis, terms=names,
                scaled_condition=condition)


def predict(model, z, speed):
    x, _, _ = design(z, speed, model["shape"], model["load_rank"],
                     model["interaction"], basis=model["load_basis"])
    return np.exp(x@np.asarray(model["coefficients"]))


def score(error, groups):
    if not len(error):
        return None
    w = balanced(groups)
    return dict(cells=len(error), groups=len(set(groups)),
                rmse_nm=float(np.sqrt(np.average(error**2, weights=w))),
                bias_nm=float(np.average(error, weights=w)),
                mae_nm=float(np.average(abs(error), weights=w)),
                abs_p95_nm=float(np.percentile(abs(error), 95)),
                max_abs_nm=float(np.max(abs(error))),
                below_005_fraction=float(np.average(abs(error) < .05, weights=w)))


def analyze(points):
    output = []
    candidates = [(shape, rank, False) for shape in ("constant", "power", "hinge")
                  for rank in (0, 1, 2)]+[("hinge", 1, True)]
    for joint in range(1, 6):
        all_points = [p for p in points if p["joint"] == joint]
        # The user excluded very-low-speed work from this main model. Keep the
        # original low-speed records, but do not mix stick/slip into sliding fit.
        ps = [p for p in all_points if p["planned"] >= .08 and p["speed"] >= .05]
        q = np.array([p["q"] for p in ps])
        z = fm.load_features(q)[0][:, joint-1]
        speed = np.array([p["speed"] for p in ps])
        targets = np.array([p["targets"] for p in ps])
        y = targets[:, 1]
        group = [(p["domain"], p["pose"], p["repeat"], p["planned"], p["record"]) for p in ps]
        train = np.array([p["repeat"] == 1 and p["domain"] in
                          ("four_pose", "baseline", "old80", "new50", "new_global") for p in ps])
        blocks = [(p["domain"], p["pose"]) for p in ps]
        # Compare every structure on the same four recorded load-pose folds.
        # Holding out the only high-speed domain would make hinge coefficients
        # unidentifiable, not provide a fair comparison of their generalization.
        names = sorted(set(blocks[i] for i in np.flatnonzero(train)
                           if ps[i]["domain"] == "four_pose"))
        cv_mask = train & np.array([p["domain"] == "four_pose" for p in ps])
        trials = []
        for candidate in candidates:
            error = np.full(len(ps), np.nan)
            try:
                for held in names:
                    test = train & np.array([b == held for b in blocks])
                    fit = train & ~test
                    model = fit_model(z[fit], speed[fit], y[fit],
                                      [group[i] for i in np.flatnonzero(fit)], candidate)
                    error[test] = predict(model, z[test], speed[test])-y[test]
                result = score(error[cv_mask], [group[i] for i in np.flatnonzero(cv_mask)])
                trials.append(dict(candidate=candidate, cv=result,
                                   parameters=len(model["coefficients"])))
            except ValueError:
                continue
        if not trials:
            raise ValueError(f"J{joint}: no rank-qualified group validation")
        best = min(t["cv"]["rmse_nm"] for t in trials)
        selected = min((t for t in trials if t["cv"]["rmse_nm"] <= best+.002),
                       key=lambda t: (t["parameters"], t["cv"]["rmse_nm"]))
        candidate = tuple(selected["candidate"])
        train_ids = np.flatnonzero(train)
        frozen = fit_model(z[train], speed[train], y[train],
                           [group[i] for i in train_ids], candidate)
        frozen_error = predict(frozen, z, speed)-y
        final = fit_model(z, speed, y, group, candidate)
        final_error = predict(final, z, speed)-y
        external = ~train
        domains = sorted({p["domain"] for p in ps})
        validation = {}
        for domain in domains:
            mask = external & np.array([p["domain"] == domain for p in ps])
            validation[domain] = score(frozen_error[mask],
                                      [group[i] for i in np.flatnonzero(mask)])
        raw = np.rad2deg(q-np.deg2rad(HW[:, 6])+REFS[:, 0])
        sensitivity = []
        for scale in (0, 2):
            alt = fit_model(z, speed, targets[:, scale], group, candidate)
            sensitivity.append(dict(nominal_dynamic_multiplier=scale,
                prediction_shift_rmse_nm=score(predict(alt, z, speed)-predict(final, z, speed),
                                               group)["rmse_nm"]))
        output.append(dict(joint=joint, final_model=final, frozen_validation_model=frozen,
            paired_cells_all=len(all_points), usable_sliding_cells=len(ps),
            excluded_very_low_speed_cells=len(all_points)-len(ps),
            nonpositive_targets=int(np.sum(y <= 0)), training_cells=int(train.sum()),
            independent_test_cells=int(external.sum()), model_selection=trials,
            selected_cross_condition_score=selected["cv"],
            final_refit_in_sample_residual=score(final_error, group),
            frozen_independent_residual=score(frozen_error[external],
                [group[i] for i in np.flatnonzero(external)]),
            independent_by_domain=validation,
            current_global_wide_residual=validation.get("new_global"),
            observed_speed_rad_s=[float(speed.min()), float(speed.max())],
            actual_raw_reference_ranges_deg=np.column_stack((raw.min(axis=0), raw.max(axis=0))),
            load_feature_ranges=np.column_stack((z.min(axis=0), z.max(axis=0))),
            temperature_range_c=[min(p["temperature"] for p in ps), max(p["temperature"] for p in ps)],
            nominal_dynamic_sensitivity=sensitivity,
            cell_residuals=[dict(domain=p["domain"], pose=p["pose"], repeat=p["repeat"],
                record=p["record"], speed_rad_s=p["speed"],
                raw_angle_center_deg=p["raw_angle_center_deg"], observed_friction_nm=float(y[i]),
                final_prediction_nm=float(final_error[i]+y[i]), final_residual_nm=float(final_error[i]),
                frozen_residual_nm=float(frozen_error[i]), independent=bool(external[i]))
                for i, p in enumerate(ps)]))
        print("JOINT", joint, plain(output[-1]["final_refit_in_sample_residual"]),
              "INDEPENDENT", plain(output[-1]["frozen_independent_residual"]), flush=True)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError("Never overwrite a frozen model")
    urdf = urdf_audit()
    audits, points, reports = collect(args.directory)
    models = analyze(points)
    result = dict(status="urdf_informed_global_friction_candidate_not_deployed", deploy=False,
        formula="tau_Fj=tanh(v_j/.02)*exp(X(abs(v_j),loadPC(qCAD))@beta_j)",
        velocity_unit="rad/s", torque_unit="N*m",
        coordinates="qCAD=continuous_reference-zero+CAD; original protocol feedback unchanged",
        friction_quantity="TOTAL direction-odd inferred drive load; replace old F once, not an extra trim",
        urdf=urdf, frozen_input_reports_sha256=reports, captures=audits,
        complete_friction_legs=sum(a["derived_legs"] for a in audits),
        paired_cells=len(points), models=models,
        source_sha256={str(p): sha(p) for p in (
            Path(__file__), ROOT/"tools/five_friction_map.py", ROOT/"tools/five_identify.py",
            ROOT/"tools/five_gravity_residual_report.py", ROOT/"config/five_coordinates.h",
            ROOT/"config/five_scene.inc", ROOT/"config/joint_references.def")},
        limitations=[
            "Drive torque estimates are not independently calibrated shaft torque.",
            "Sliding model only: planned speed >=.08 and measured speed >=.05 rad/s; no static friction claim.",
            "URDF wrench features are load proxies, not measured bearing contact normal force.",
            "First-round old/new data train; the same four old load-pose folds select structure; second rounds/timed/retests are held out.",
            "New wide J3/J4 have only first rounds; J5 has no new wide capture. Their new full-range independent validation is missing.",
            "The first exploratory report was inspected before correcting grid density; reused validation is retrospective, not a fresh blind physical test.",
            "Final coefficients refit all usable data after frozen-model independent scores; its training residual is not independent.",
            "Different condition tags do not guarantee mechanically independent load poses.",
            "J4 nonperiodic cable winding/history is not identified; never fold turns to assert cable-friction equivalence.",
            "Source start intervals are conservatively trimmed, not relabelled as exact timing.",
            "No arbitrary payload/temperature/velocity extrapolation; unsupported domain retains commissioned baseline.",
            "Original no-contact metadata remain explicit; historical false is not silently changed.",
            "No gravity/zero/firmware changes, serial access, flash, reset, enable or motion."])
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(plain(result), stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    print("MODEL", args.output, "SHA256", sha(args.output), flush=True)


if __name__ == "__main__":
    main()
