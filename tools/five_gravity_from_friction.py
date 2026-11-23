"""Offline J2..J5 gravity re-identification from audited bidirectional data.

Reuses the measured-clock/SG/friction pipeline. No port, firmware or motion I/O.
Gravity and constant torque bias are separate; a small training residual alone
does not establish absolute gravity. Outputs one reproducible numeric artifact.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path

import numpy as np

import five_friction_map as fm
from five_gravity_fit import gravity_basis, nominal_parameters
from five_gravity_residual_report import gravity_transform, support_gravity
from five_identify import HW, REFS, ROOT


RIDGES = (0., 1e-6, 1e-4, .001, .01, .1, 1.)
WIDE_RUNS = (
    "v12-gravity-sweep-unlocked-20261119/j345_sweep_01.jsonl",
    "v12-j4-raw-range-20261121/j2_validation_sweep_01.jsonl",
    "v12-smooth-500hz-20261121/j45_sweep_01.jsonl",
)


def sha(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def capture_path(path):
    path = Path(path)
    return path if path.is_absolute() else ROOT.parent/path


def jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(k): jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(v) for v in value]
    return value


def collect():
    """Keep all completed captures; never relabel an aborted sequence as complete."""
    folder = ROOT/"build/v12-speed-friction-20261121"
    base_path = folder/"friction_map_4pose_2round_01.json"
    fast_path = ROOT/"build/v12-friction-budget2x-20261122/friction_integrated_080_02.json"
    base, fast = [json.loads(p.read_text(encoding="utf-8")) for p in (base_path, fast_path)]
    expected = {str(capture_path(c["file"]).resolve()): c["sha256"].lower()
                for c in base["captures"]+fast["previous_captures"]+fast["timed_captures"]}
    specs = [(path, "four_pose", pose, repeat, 10, .5, None, None)
             for pose, repeat, path in fm.campaign_paths(folder)]
    for speed in ("002", "008", "016"):
        for repeat in (1, 2):
            path = ROOT/f"build/v12-speed-pi-axis-tune-20261121/friction_baseline_{speed}_{repeat:02d}.jsonl"
            specs.append((path, "baseline", 10, repeat, 10, .5, None, None))
    for c in fast["previous_captures"]:
        specs.append((capture_path(c["file"]).resolve(), c["population"], 20,
                      c["repeat"], 4, 1., None, None))
    for index, rel in enumerate(WIDE_RUNS):
        specs.append((ROOT/"build"/rel, "wide", 100+index, None, 10, 1., None, None))
    for joint in range(1, 6):
        entries = sorted((r for r in fast["verified_timing"] if r["joint"] == joint),
                         key=lambda r: r["leg"])
        for entry in entries:
            specs.append((Path(entry["capture"]), "timed", 30, 1, 4, 1.,
                          entry["board_start_tick_ms"], joint))
    audits, points, pending = [], [], defaultdict(list)
    for index, (path, domain, pose, repeat, grid, width, tick, timed_joint) in enumerate(specs):
        audit, legs = fm.load_legs(path, pose, grid_ms=grid,
                                  verified_start_tick_ms=tick, historical_ranges=domain == "wide")
        key = str(path.resolve())
        if key in expected and audit["sha256"] != expected[key]:
            raise ValueError(f"Capture changed since frozen analysis: {path}")
        audit.update(domain=domain, pose=pose, repeat=repeat, analysis_grid_ms=grid,
                     angle_bin_deg=width, verified_start_tick_ms=tick)
        if domain == "wide":
            note = path.with_suffix(".operator_confirmation.json")
            if note.is_file():
                confirmation = json.loads(note.read_text(encoding="utf-8"))
                if (confirmation.get("capture_sha256", "").lower() == audit["sha256"]
                        and confirmation.get("original_capture_modified") is False
                        and confirmation.get("free_space_confirmed") is True):
                    audit["free_space_confirmed"] = True
                    audit["operator_confirmation"] = dict(file=str(note), sha256=sha(note))
        audits.append(audit)
        groups = []
        if domain == "timed":
            pending[timed_joint].extend(legs)
            if len(pending[timed_joint]) == 2:
                groups.append((pending.pop(timed_joint), 1, f"timed_J{timed_joint}"))
        elif domain == "wide":
            for joint in sorted({leg["joint"] for leg in legs}):
                both = [leg for leg in legs if leg["joint"] == joint]
                if len(both) != 4:
                    raise ValueError("Expected four completed historical sweep legs")
                groups.extend((both[i:i+2], i//2+1, f"{path.name}:J{joint}:pair{i//2+1}")
                              for i in (0, 2))
        else:
            groups.append((legs, repeat, path.name))
        for both, repetition, record in groups:
            for point in fm.pair_legs(both, angle_bin_deg=width, components=True):
                if point["joint"] != 1:
                    point.update(domain=domain, repeat=repetition, record=record,
                                 file=str(path), file_sha256=audit["sha256"])
                    points.append(point)
        if index % 6 == 5:
            print(f"READ {index+1}/{len(specs)} captures, {len(points)} J2..J5 pairs", flush=True)
    if pending or len({a["file"] for a in audits}) != len(audits):
        raise ValueError("Duplicate/missing capture or timed leg")
    return audits, points, {
        str(p): sha(p) for p in (base_path, fast_path)
    }


def balanced(points, mask):
    """Each joint/domain receives equal mass, then each recorded condition."""
    indexes = np.flatnonzero(mask)
    if not len(indexes):
        raise ValueError("No eligible observations")
    group = [(points[i]["joint"], points[i]["domain"], points[i]["pose"],
              points[i]["planned"], points[i]["repeat"], points[i]["record"]) for i in indexes]
    cells = Counter(group)
    groups = Counter((g[0], g[1]) for g in set(group))
    domains = Counter(g[0] for g in groups)
    w = np.zeros(len(points))
    w[indexes] = [1/(cells[g]*groups[g[:2]]*domains[g[0]]) for g in group]
    return w/w.sum()


def matrices(points):
    qp = np.array([p["positive"]["q"] for p in points])
    qn = np.array([p["negative"]["q"] for p in points])
    joint = np.array([p["joint"]-1 for p in points])
    rows = np.arange(len(points))
    yp = np.array([p["positive"]["y"] for p in points])
    yn = np.array([p["negative"]["y"] for p in points])
    yp_basis, yn_basis = gravity_basis(qp)[rows, joint], gravity_basis(qn)[rows, joint]
    basis = (yp_basis+yn_basis)/2
    oldp, oldn = support_gravity(qp)[rows, joint], support_gravity(qn)[rows, joint]
    current = (oldp+oldn)/2
    speed = np.array([[p["positive"]["v"], p["negative"]["v"]] for p in points])
    dynamic = np.maximum(np.max(abs(yp[:, [0, 2]]-yp[:, 1, None]), axis=1),
                         np.max(abs(yn[:, [0, 2]]-yn[:, 1, None]), axis=1))
    qualified = (speed.min(axis=1) >= .02) & (speed.max(axis=1) <= .25) & (dynamic <= .03)
    rawp = qp-np.deg2rad(HW[:, 6])+REFS[:, 0]
    rawn = qn-np.deg2rad(HW[:, 6])+REFS[:, 0]
    current_range = np.all((rawp >= REFS[:, 1]) & (rawp <= REFS[:, 2])
                           & (rawn >= REFS[:, 1]) & (rawn <= REFS[:, 2]), axis=1)
    domain = np.array([p["domain"] for p in points])
    repeat = np.array([p["repeat"] for p in points])
    return dict(qp=qp, qn=qn, joint=joint, basis=basis, transform=gravity_transform(),
                oldp=oldp, oldn=oldn, current=current,
                nominal=basis@nominal_parameters(),
                target_scales=current[:, None]+(yp+yn)/2, dynamic=dynamic,
                qualified=qualified, within_current_raw_ranges=current_range,
                train=qualified & np.isin(domain, ("four_pose", "wide")) & (repeat == 1),
                independent=qualified & (domain == "four_pose") & (repeat == 2),
                wide_repeat=qualified & (domain == "wide") & (repeat == 2),
                domain=domain, repeat=repeat, speed=speed)


def friction_adjustment(points, data, gravity_training):
    """Fit odd-load priors on training first-round four-pose cells only.

Do not extrapolate this prior: outside its observed speed/feature box, keep
the paired average uncorrected and flag it. The no-prior sensitivity is saved.
"""
    correction = np.zeros(len(points))
    outside = np.ones(len(points), bool)
    models = []
    for joint in range(2, 6):
        ids = np.array([i for i, p in enumerate(points) if p["joint"] == joint])
        training = np.array([gravity_training[i] and points[i]["domain"] == "four_pose" for i in ids])
        if sum(training) < 8:
            raise ValueError(f"J{joint}: insufficient training-only friction prior")
        q = np.array([points[i]["q"] for i in ids])
        z = fm.load_features(q)[0][:, joint-1]
        v = np.array([points[i]["speed"] for i in ids])
        odd = np.array([points[i]["targets"][1] for i in ids])
        local = [points[i] for i in ids]
        w = balanced(local, training)[training]
        name = "speed_load_2" if joint == 2 else "speed_only"
        try:
            beta, basis, _ = fm.campaign_fit(z[training], v[training], odd[training], w, name)
        except ValueError:
            name = "speed_only"  # Missing load excitation is reported, not fabricated.
            beta, basis, _ = fm.campaign_fit(z[training], v[training], odd[training], w, name)
        pp, pn = data["qp"][ids], data["qn"][ids]
        zp, zn = fm.load_features(pp)[0][:, joint-1], fm.load_features(pn)[0][:, joint-1]
        vp, vn = data["speed"][ids].T
        lo, hi = v[training].min(), v[training].max()
        inside = (vp >= lo) & (vp <= hi) & (vn >= lo) & (vn <= hi)
        active = np.ptp(z[training], axis=0) > 1e-5
        if "load" in name:
            zlo, zhi = z[training].min(axis=0), z[training].max(axis=0)
            inside &= np.all((zp[:, active] >= zlo[active]-1e-8)
                             & (zp[:, active] <= zhi[active]+1e-8)
                             & (zn[:, active] >= zlo[active]-1e-8)
                             & (zn[:, active] <= zhi[active]+1e-8), axis=1)
        selected = ids[inside]
        if len(selected):
            fp = fm.campaign_predict(zp[inside], vp[inside], name, beta, basis)
            fn = fm.campaign_predict(zn[inside], vn[inside], name, beta, basis)
            correction[selected] = (fp-fn)/2
            outside[selected] = False
        models.append(dict(joint=joint, model=name, coefficients=beta, load_basis=basis,
                           training_cells=int(sum(training)), speed_domain=[lo, hi],
                           corrected_pairs=int(sum(inside)), out_of_domain_pairs=int(sum(~inside))))
    return correction, outside, models


def fit_linear(basis, target, weights, ridge=0., include_bias=True):
    """Weighted SVD, with gravity-only ridge; never silently discard a column."""
    if basis.ndim != 2 or target.shape != (len(basis),) or weights.shape != target.shape:
        raise ValueError("Invalid regression dimensions")
    use = weights > 0
    if not np.isfinite(basis).all() or not np.isfinite(target).all() or not np.isfinite(weights).all():
        raise ValueError("Nonfinite regression input")
    scale = np.sqrt(np.sum(weights[:, None]*basis**2, axis=0))
    if np.any(scale < 1e-12) or sum(use) < basis.shape[1]+2:
        raise ValueError("Unexcited gravity/bias design")
    x = basis[use]/scale*np.sqrt(weights[use, None])
    y = target[use]*np.sqrt(weights[use])
    sv = np.linalg.svd(x, compute_uv=False)
    rank = int(sum(sv > sv[0]*1e-10))
    if rank < basis.shape[1]:
        raise ValueError(f"Measured rank deficient: {rank}/{basis.shape[1]}")
    penalty = np.eye(basis.shape[1])
    if include_bias:
        penalty[-4:, -4:] = 0
    augmented_x = np.vstack((x, np.sqrt(ridge)*penalty))
    augmented_y = np.r_[y, np.zeros(basis.shape[1])]
    scaled = np.linalg.lstsq(augmented_x, augmented_y, rcond=1e-12)[0]
    return dict(coefficients=scaled/scale, column_scale=scale, rank=rank,
                condition=float(sv[0]/sv[-1]), ridge=ridge)


def design(data, include_bias=True):
    physical = data["basis"]@data["transform"]
    return np.column_stack((physical, np.eye(4)[data["joint"]-1])) if include_bias else physical


def rms_score(points, values, selected):
    if not np.any(selected):
        return None
    w = balanced(points, selected)
    return float(np.sqrt(np.sum(w*values**2)))


def tune(points, data, training):
    """Leave whole first-round load poses out; never use repeat two for tuning."""
    x = design(data)
    scores = defaultdict(list)
    for pose in sorted({p["pose"] for i, p in enumerate(points)
                        if training[i] and p["domain"] == "four_pose"}):
        held = np.array([p["domain"] == "four_pose" and p["pose"] == pose for p in points])
        tr, te = training & ~held, training & held
        fc, _, _ = friction_adjustment(points, data, tr)
        y = data["target_scales"][:, 1]-fc-data["nominal"]
        w = balanced(points, tr)
        for ridge in RIDGES:
            try:
                model = fit_linear(x, y, w, ridge)
                error = y-x@model["coefficients"]
                scores[ridge].append(rms_score(points, error, te))
            except ValueError:
                pass
    complete = {r: float(np.mean(v)) for r, v in scores.items() if len(v) == 4}
    if not complete:
        raise ValueError("No fully identified four-pose cross-validation candidate")
    best = min(complete.values())
    selected = max(r for r, score in complete.items() if score <= 1.05*best+1e-12)
    return selected, [dict(ridge=r, fold_rmse_nm=scores[r], mean_rmse_nm=complete[r])
                      for r in sorted(complete)]


def evaluate(points, data, training, testing, ridge):
    correction, outside, priors = friction_adjustment(points, data, training)
    target = data["target_scales"][:, 1]-correction
    x, w = design(data), balanced(points, training)
    model = fit_linear(x, target-data["nominal"], w, ridge)
    rank = data["transform"].shape[1]
    coeff = model["coefficients"]
    gravity = data["nominal"]+x[:, :rank]@coeff[:rank]
    bias = coeff[rank:]
    baseline_bias = np.array([np.average((target-data["current"])[training & (data["joint"] == j)],
                                weights=w[training & (data["joint"] == j)]) for j in range(1, 5)])
    reports = []
    for joint in range(2, 6):
        take = testing & (data["joint"] == joint-1)
        reports.append(dict(joint=joint, cells=int(sum(take)),
            current_gravity_only_rmse_nm=rms_score(points, target-data["current"], take),
            current_plus_training_bias_rmse_nm=rms_score(
                points, target-data["current"]-baseline_bias[data["joint"]-1], take),
            candidate_gravity_only_rmse_nm=rms_score(points, target-gravity, take),
            candidate_plus_training_bias_rmse_nm=rms_score(
                points, target-gravity-bias[data["joint"]-1], take),
            candidate_torque_bias_nm=float(bias[joint-2]),
            current_fitted_bias_nm=float(baseline_bias[joint-2]),
            friction_prior_outside_cells=int(sum(take & outside)),
            within_current_raw_ranges_cells=int(sum(take & data["within_current_raw_ranges"])),
            current_range_only_current_gravity_rmse_nm=rms_score(
                points, target-data["current"], take & data["within_current_raw_ranges"]),
            current_range_only_candidate_gravity_rmse_nm=rms_score(
                points, target-gravity, take & data["within_current_raw_ranges"])))
    return model, gravity, target, correction, outside, priors, reports


def scalar_comparison(points, data, target, draws):
    """A low-complexity diagnostic: scale current G, with/without nuisance bias.

The two models are predeclared; no test-driven choice is silently deployed.
"""
    output = []
    for joint in range(2, 6):
        axis = data["joint"] == joint-1
        tr, test = data["train"] & axis, data["independent"] & axis
        w = balanced(points, tr)
        g = data["current"]
        def coefficients(weights, with_bias):
            use = weights > 0
            design = np.column_stack((g, np.ones(len(g)))) if with_bias else g[:, None]
            scale = np.sqrt(np.sum(weights[:, None]*design**2, axis=0))
            x = design[use]/scale*np.sqrt(weights[use, None])
            value, _, rank, _ = np.linalg.lstsq(x, target[use]*np.sqrt(weights[use]), rcond=1e-10)
            if rank != design.shape[1]:
                raise ValueError("Unidentified scalar gravity/bias")
            return value/scale
        for with_bias in (False, True):
            beta = coefficients(w, with_bias)
            held = []
            for file in sorted({p["file"] for i, p in enumerate(points) if axis[i] and p["domain"] == "wide"}):
                mask = np.array([p["file"] == file for p in points])
                training, testing = tr & ~mask, data["qualified"] & axis & mask
                b = coefficients(balanced(points, training), with_bias)
                held.append(dict(file=file, cells=int(sum(testing)), coefficients=b,
                    current_gravity_rmse_nm=rms_score(points, target-g, testing),
                    candidate_gravity_rmse_nm=rms_score(points, target-g*b[0], testing),
                    candidate_with_bias_rmse_nm=rms_score(
                        points, target-g*b[0]-(b[1] if with_bias else 0), testing),
                    current_range_current_gravity_rmse_nm=rms_score(
                        points, target-g, testing & data["within_current_raw_ranges"]),
                    current_range_candidate_gravity_rmse_nm=rms_score(
                        points, target-g*b[0], testing & data["within_current_raw_ranges"])))
            uncertainty = []
            for block_degrees in (4., 12.):
                groups = defaultdict(lambda: defaultdict(list))
                for i in np.flatnonzero(tr):
                    p = points[i]
                    key = (p["domain"], p["pose"], p["planned"], p["record"])
                    groups[key][int(np.floor(p["raw_angle_center_deg"]/block_degrees))].append(i)
                rng = np.random.default_rng(2026112220+joint)
                cloud = []
                for _ in range(draws):
                    ww = np.zeros(len(points))
                    for block in groups.values():
                        labels = list(block)
                        for key in rng.choice(labels, len(labels), replace=True):
                            ww[block[key]] += w[block[key]]
                    cloud.append(coefficients(ww/ww.sum(), with_bias))
                uncertainty.append(dict(block_degrees=block_degrees,
                    conditional_95=np.percentile(cloud, [2.5, 97.5], axis=0).T))
            output.append(dict(joint=joint, formula="G_est=scale*G_current(q); exclude any fitted bias",
                with_nuisance_bias=with_bias, scale=float(beta[0]),
                nuisance_bias_nm=float(beta[1]) if with_bias else 0.,
                independent_current_gravity_rmse_nm=rms_score(points, target-g, test),
                independent_candidate_gravity_rmse_nm=rms_score(points, target-g*beta[0], test),
                independent_candidate_with_bias_rmse_nm=rms_score(
                    points, target-g*beta[0]-(beta[1] if with_bias else 0), test),
                whole_capture_holdout=held, conditional_intervals=uncertainty,
                physical_warning=("Negative scale: reject as a physical gravity update" if beta[0] <= 0
                                  else "Effective actuator feedforward estimate, not a calibrated mass/COM")))
    return output


def analyze(points, draws=500):
    data = matrices(points)
    train = data["train"]
    ridge, cv = tune(points, data, train)
    model, gravity, target, fc, outside, priors, independent = evaluate(
        points, data, train, data["independent"], ridge)
    _, _, _, _, _, _, wide_repeat = evaluate(
        points, data, train, data["wide_repeat"], ridge)
    rank = data["transform"].shape[1]
    x, w = design(data), balanced(points, train)
    coeff = model["coefficients"]
    delta = data["transform"]@coeff[:rank]
    nominal = nominal_parameters()
    wide_sources = sorted({p["file"] for p in points if p["domain"] == "wide"})
    external = []
    for source in wide_sources:
        held = np.array([p["file"] == source for p in points])
        tr, te = train & ~held, data["qualified"] & held
        try:
            lam, _ = tune(points, data, tr)
            _, _, _, _, _, _, scores = evaluate(points, data, tr, te, lam)
            external.append(dict(file=source, ridge=lam, joints=scores,
                                 validation="whole capture excluded from gravity and friction fitting/tuning"))
        except ValueError as exc:
            external.append(dict(file=source, unavailable=str(exc)))
    dynamic_models = []
    for scale in (0, 2):
        alt = fit_linear(x, data["target_scales"][:, scale]-fc-data["nominal"], w, ridge)
        dynamic_models.append(alt["coefficients"])
    no_prior = fit_linear(x, data["target_scales"][:, 1]-data["nominal"], w, ridge)["coefficients"]
    no_bias = fit_linear(x[:, :rank], target-data["nominal"], w, ridge,
                         include_bias=False)["coefficients"]
    rng = np.random.default_rng(2026112211)
    intervals, clouds, bootstrap_examples = [], [], []
    for block_degrees in (4., 12.):
        groups = defaultdict(lambda: defaultdict(list))
        for i in np.flatnonzero(train):
            p = points[i]
            group = (p["joint"], p["domain"], p["pose"], p["planned"], p["record"])
            groups[group][int(np.floor(p["raw_angle_center_deg"]/block_degrees))].append(i)
        cloud, failures = [], 0
        for draw in range(draws):
            ww = np.zeros(len(points))
            for block in groups.values():
                labels = list(block)
                for key in rng.choice(labels, len(labels), replace=True):
                    ids = block[key]
                    ww[ids] += w[ids]
            ww /= ww.sum()
            try:
                fitted = fit_linear(x, target-data["nominal"], ww, ridge)["coefficients"]
                cloud.append(fitted)
                if draw < 3:
                    bootstrap_examples.append(dict(block_degrees=block_degrees,
                        weights=ww, coefficients=fitted))
            except ValueError:
                failures += 1
        if len(cloud) < max(20, .8*draws):
            raise ValueError("Too many rank-deficient spatial block draws")
        cloud = np.asarray(cloud)
        intervals.append(dict(block_degrees=block_degrees, requested=draws,
            successful=len(cloud), failed=failures, strata=len(groups),
            block_counts=[len(g) for g in groups.values()],
            coefficient_conditional_95=np.percentile(cloud, [2.5, 97.5], axis=0).T))
        clouds.append(cloud)
    joint_reports, harmonic_x, harmonic_nominal = [], [], []
    for joint in range(2, 6):
        i = joint-1
        use = data["qualified"] & (data["joint"] == i)
        raw = np.rad2deg((data["qp"]+data["qn"])/2-np.deg2rad(HW[:, 6])+REFS[:, 0])
        candidates = [k for k, p in enumerate(points) if p["joint"] == joint
                      and p["domain"] == "four_pose" and p["pose"] == 0 and p["planned"] == .16]
        representative = candidates[len(candidates)//2]
        pose = (data["qp"][representative]+data["qn"][representative])/2
        query = np.tile(pose, (2, 1))
        query[:, i] = np.deg2rad(HW[i, 6])+np.array([0, np.pi/2])
        harmonic_basis = gravity_basis(query)[:, i]
        harmonic = harmonic_basis@(nominal+delta)
        hb = harmonic_basis@data["transform"]
        harmonic_x.append(np.column_stack((hb, np.zeros((2, 4)))))
        harmonic_nominal.append(harmonic_basis@nominal)
        bounds = [np.percentile(harmonic_basis@nominal[:, None]+hb@c[:, :rank].T,
                               [2.5, 97.5], axis=1).T for c in clouds]
        prediction = data["nominal"][:, None]+x[:, :rank]@clouds[-1][:, :rank].T
        width = np.ptp(np.percentile(prediction[use], [2.5, 97.5], axis=1), axis=0)
        sensitivity = [float(np.percentile(abs(x[use, :rank]@(a[:rank]-coeff[:rank])), 95))
                       for a in dynamic_models]
        no_bias_change = x[use, :rank]@(no_bias-coeff[:rank])
        no_prior_change = x[use, :rank]@(no_prior[:rank]-coeff[:rank])
        tests = [r for record in external for r in record.get("joints", [])
                 if r["joint"] == joint and r["cells"]]
        joint_reports.append(dict(joint=joint, train_cells=int(sum(train & (data["joint"] == i))),
            independent_cells=int(sum(data["independent"] & (data["joint"] == i))),
            eligible_raw_ranges_deg=np.column_stack((raw[use].min(axis=0), raw[use].max(axis=0))),
            current_to_candidate_change_5_50_95_nm=np.percentile((gravity-data["current"])[use], [5, 50, 95]),
            inferred_gravity_5_50_95_nm=np.percentile(gravity[use], [5, 50, 95]),
            fitted_constant_bias_nm=coeff[rank+joint-2],
            median_conditional_95_gravity_width_nm=float(np.median(width)),
            zero_bias_assumption_gravity_change95_nm=float(np.percentile(abs(no_bias_change), 95)),
            no_friction_prior_gravity_change95_nm=float(np.percentile(abs(no_prior_change), 95)),
            nominal_dynamic_0_2_gravity_change95_nm=sensitivity,
            independent_repeat=independent[joint-2], wide_same_record_repeat=wide_repeat[joint-2],
            whole_capture_holdout=tests,
            conditional_formula="G=A_cos*cos(theta_software_rad)+A_sin*sin(theta_software_rad); NO bias included",
            conditional_cos_sin_nm=harmonic,
            conditional_cos_sin_95_by_4deg_12deg_block=bounds,
            conditional_other_raw_deg=np.rad2deg(pose)-HW[:, 6]+np.rad2deg(REFS[:, 0]),
            conditional_source_file=points[representative]["file"],
            warning="Conditional slice, not an independent global one-axis sine; torque bias is NOT gravity."))
    friction_feedback = []
    for joint in range(2, 6):
        ids = np.array([k for k, p in enumerate(points) if p["joint"] == joint])
        jp = gravity_basis(data["qp"][ids])[:, joint-1]@(nominal+delta)
        jn = gravity_basis(data["qn"][ids])[:, joint-1]@(nominal+delta)
        change = ((data["oldp"][ids]-jp)-(data["oldn"][ids]-jn))/2
        friction_feedback.append(dict(joint=joint,
            corrected_direction_odd_change_5_50_95_nm=np.percentile(change, [5, 50, 95]),
            max_abs_change_nm=float(np.max(abs(change))),
            explanation="Recompute paired odd load after replacing current G once; a common torque bias cancels."))
    all_external = ~np.isin(data["domain"], ("four_pose", "wide")) & (data["speed"].min(axis=1) >= .02)
    _, _, _, _, _, _, other = evaluate(points, data, train, all_external, ridge)
    # Store the exact numerical problem for independent MATLAB reproduction.
    numerical = dict(x=x, y=target-data["nominal"], weights=w,
        gravity_rank=rank, ridge=ridge, coefficients=coeff,
        nominal_gravity=data["nominal"], current_gravity=data["current"],
        target_even_nm=target, joint=data["joint"]+1, train_mask=train,
        independent_mask=data["independent"], wide_repeat_mask=data["wide_repeat"],
        independent_weights=balanced(points, data["independent"]),
        bootstrap_examples=bootstrap_examples,
        bootstrap_strata_12deg=[dict(blocks=[dict(rows=ids) for ids in block.values()])
                               for block in groups.values()],
        harmonic_x=np.vstack(harmonic_x), harmonic_nominal=np.concatenate(harmonic_nominal),
        harmonic_conditional_95_12deg=np.percentile(
            np.concatenate(harmonic_nominal)[:, None]+np.vstack(harmonic_x)@clouds[-1].T,
            [2.5, 97.5], axis=1).T)
    return dict(status="measured_torque_gravity_estimates_not_deployed", deploy=False,
        formula="G_est(qCAD)=gravity_basis(qCAD)@(nominal_parameters+potential_delta); replace old G2/G3 trims once",
        input_coordinates="qCAD_rad=(raw-reference_zero)+CAD_offset, five actual joints, J4 never wrapped",
        potential_delta=delta, potential_delta_rows_link_cols_mass_hxyz=delta[:20].reshape(5, 4),
        gravity_rank=rank, measured_rank=model["rank"], measured_scaled_condition=model["condition"],
        selected_ridge=ridge, training_only_pose_cv=cv, friction_training_priors=priors,
        candidate_constant_bias_nm=coeff[rank:], joints=joint_reports,
        whole_capture_validation=external, other_friction_captures_validation=other,
        friction_label_feedback=friction_feedback, conditional_bootstrap=intervals,
        scalar_diagnostic_models=scalar_comparison(points, data, target, draws),
        numerical_crosscheck=numerical,
        selection=dict(min_each_direction_actual_speed=.02, max_primary_actual_speed=.25,
                       max_each_direction_nominal_dynamic_nm=.03, gravity_joints=[2, 3, 4, 5],
                       gravity_train="first repeat four-pose + first two legs of each completed historical wide sweep",
                       independent_test="second-repeat four-pose; separately report within-capture wide repeats",
                       prior="odd-load friction fits trained only on training first-repeat four-pose cells",
                       domain_policy="No friction extrapolation; outside prior domain retain uncorrected even average"),
        limitations=[
            "Actual means inferred from drive-reported torque, not independently calibrated shaft torque.",
            "A direction-even torque bias, cable force or asymmetric friction is not uniquely gravity.",
            "Constant biases are separately estimated and never included in the gravity feedforward.",
            "Regularization and limited excitation affect absolute-gravity estimates; zero-bias sensitivity is reported.",
            "95% intervals are conditional spatial-block uncertainty, not probabilities of global correctness.",
            "Nominal dynamic subtraction is not identified physical acceleration/inertia compensation.",
            "Whole no-contact confirmation and current-range coverage remain explicit per capture.",
            "Historical raw ranges are analysis-only and do not authorize old J4 travel.",
            "No new J1 gravity, zero calibration, firmware change, flash, serial or motion operation.",
            "Independent model verification and a friction campaign plan do not establish global hardware validation."])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--bootstrap", type=int, default=500)
    parser.add_argument("--paired-input", type=Path,
                        help="reuse audited pair observations; rehash every raw capture and fixed geometry first")
    args = parser.parse_args()
    if args.output.exists() or args.bootstrap < 20:
        parser.error("Use a new output file and at least 20 bootstrap draws")
    if args.paired_input:
        frozen = json.loads(args.paired_input.read_text(encoding="utf-8"))
        audits, points = frozen["captures"], frozen["paired_observations"]
        provenance = frozen["frozen_input_reports_sha256"]
        for audit in audits:
            if sha(audit["file"]) != audit["sha256"]:
                raise ValueError("Cached pair original capture changed")
        for rel, expected in frozen["source_sha256"].items():
            if Path(rel).as_posix() != "tools/five_gravity_from_friction.py" and sha(ROOT/rel) != expected:
                raise ValueError("Cached pair geometry/extraction implementation changed")
        for path, expected in provenance.items():
            if sha(path) != expected:
                raise ValueError("Frozen input report changed")
        print("REUSED_AUDITED_PAIRS", len(audits), len(points), flush=True)
    else:
        audits, points, provenance = collect()
    result = analyze(points, args.bootstrap)
    result.update(captures=audits, paired_cells=len(points),
        complete_friction_legs=sum(a["counts"]["waypoint"] for a in audits if a["domain"] != "wide"),
        complete_historical_gravity_legs=sum(a["counts"]["waypoint"] for a in audits if a["domain"] == "wide"),
        frozen_input_reports_sha256=provenance, paired_observations=points)
    sources = [Path(__file__), ROOT/"tools/five_friction_map.py", ROOT/"tools/five_gravity_fit.py",
               ROOT/"tools/five_gravity_residual_report.py", ROOT/"tools/five_identify.py",
               ROOT/"config/five_scene.inc", ROOT/"config/five_coordinates.h",
               ROOT/"config/five_mit.c", ROOT/"config/joint_references.def"]
    result["source_sha256"] = {str(p.relative_to(ROOT)): sha(p) for p in sources}
    if args.paired_input:
        result["audited_pair_input"] = dict(file=str(args.paired_input), sha256=sha(args.paired_input),
                                          all_originals_and_fixed_sources_rehashed=True)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(jsonable(result), stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    print("GRAVITY_RESULT", args.output, sha(args.output), flush=True)
    print(json.dumps(jsonable({k: result[k] for k in ("paired_cells", "gravity_rank",
        "measured_scaled_condition", "selected_ridge", "candidate_constant_bias_nm", "joints")}),
        ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
