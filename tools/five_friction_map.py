"""Offline two-load-pose friction map. Never opens a port or writes firmware."""
import argparse
from array import array
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from five_control import CONTROL_VERSION, EXPECTED
from five_identify import HW, NOMINAL, REFS, ROOT, SCENE, regressors
from five_gravity_residual_report import support_gravity


MODELS = {
    "constant": (0,),
    "load_only": (0, 2),
    "speed_only": (0, 1),
    "speed_load": (0, 1, 2),
    "speed_load_interaction": (0, 1, 2, 3),
}
SPEED_BINS = [.005, .015, .03, .06, .10, .14, .20, .30, .50]


def capture_paths(directory):
    for pose, prefix in ((0, "friction_overlay"), (1, "friction_pose02")):
        for speed in ("002", "008", "016"):
            scopes = ("j1234", "j5") if pose or speed == "016" else ("",)
            for scope in scopes:
                suffix = f"_{scope}" if scope else ""
                yield pose, directory / f"{prefix}_{speed}{suffix}_01.jsonl"


def derivative_kernels(grid_ms=10):
    """Same +/-100 ms cubic SG support at 100 Hz or 250 Hz."""
    if grid_ms not in (4, 10):
        raise ValueError("Use a 4 or 10 ms analysis grid")
    half = 100//grid_ms
    dt = np.arange(-half, half+1)*(grid_ms*.001)
    inverse = np.linalg.pinv(np.column_stack([dt**p for p in range(4)]))
    return half, (inverse[0], inverse[1], 2*inverse[2])


def load_legs(path, pose, grid_ms=10, verified_start_tick_ms=None, *,
              historical_ranges=False, start_tick_interval_ms=None):
    """Audit originals; retain source clock, all five angles and reported torque."""
    half, kernels = derivative_kernels(grid_ms)
    legs, active, latest, header, end, final = [], None, None, None, None, None
    digest, counts = hashlib.sha256(), Counter()
    previous, gaps = [None]*5, []
    raw_low, raw_high = np.full(5, np.inf), np.full(5, -np.inf)
    max_temperature, max_source_age = 0, 0
    with path.open("rb") as source:
        for raw in source:
            digest.update(raw)
            r = json.loads(raw)
            stage = r["stage"]
            counts[stage] += 1
            if stage == "capture":
                if header is not None:
                    raise ValueError("Duplicate capture header")
                header = r
                references = np.asarray(r["references_rad"])
                if (r["control_version"] != CONTROL_VERSION
                        or r["schema"] != "ylamp-five-id-1" or r["action"] != "move"
                        or references.shape != (5, 3)
                        or not np.isfinite(references).all()
                        or np.any(references[:, 2] <= references[:, 1])
                        or not np.allclose(references[:, :1] if historical_ranges else references,
                                           REFS[:, :1] if historical_ranges else REFS,
                                           rtol=0, atol=2e-6)):
                    raise ValueError(f"Incompatible capture: {path}")
            if "phase" in r and (r["fault"] or r["limited"]
                    or any(r[k] != 31 for k in ("fresh", "enabled", "submitted"))):
                raise ValueError(f"Unhealthy capture: {path}")
            if stage == "waypoint":
                active = dict(r, t0=None, streams=[array("d") for _ in range(5)])
                legs.append(active)
            if stage == "rpc_accepted":
                active["t0"] = latest
            tel = r.get("telemetry")
            if tel:
                latest = tel["tick_ms"]
                if stage == "feedback":
                    layout = [(j["joint"], j["bus"], j["protocol"],
                               j["command_id"], j["feedback_id"]) for j in tel["joints"]]
                    if layout != EXPECTED:
                        raise ValueError("Wrong motor layout")
                    for i, j in enumerate(tel["joints"]):
                        if not (j["fresh"] and j["received"] and j["state"] == 1
                                and 0 <= j["age_ms"] <= 20
                                and latest-j["received_tick_ms"] == j["age_ms"]):
                            raise ValueError("Invalid motor state/source time")
                        raw_low[i] = min(raw_low[i], j["position_rad"])
                        raw_high[i] = max(raw_high[i], j["position_rad"])
                        max_temperature = max(max_temperature, j["mos_c"], j["rotor_c"])
                        max_source_age = max(max_source_age, j["age_ms"])
                        now = j["received_tick_ms"], j["received_count"]
                        if now == previous[i]:
                            continue
                        if previous[i]:
                            if not all(a > b for a, b in zip(now, previous[i])):
                                raise ValueError("Source clock/count reversed")
                            if now[0]-previous[i][0] > 25:
                                gaps.append((previous[i][0], now[0]))
                        previous[i] = now
                        if active:
                            active["streams"][i].extend((now[0], j["position_rad"],
                                j["torque_nm"], max(j["mos_c"], j["rotor_c"])))
            if stage == "waypoint_handover":
                active = None
            if stage == "handover":
                final = r
            if stage == "end":
                end = r
    if not header or not end or not final or final["phase"] != "GRAVITY":
        raise ValueError("Incomplete capture")
    if end["malformed"] or end["incomplete"]:
        raise ValueError("Malformed/incomplete telemetry")
    if verified_start_tick_ms is not None:
        if (len(legs)!=1 or not isinstance(verified_start_tick_ms, int)
                or not 0<=verified_start_tick_ms<=0xffffffff
                or abs(verified_start_tick_ms-legs[0]["t0"])>1000):
            raise ValueError("Verified start tick needs a matching single-leg capture")
        legs[0]["t0"] = verified_start_tick_ms
    if start_tick_interval_ms is not None:
        if (verified_start_tick_ms is not None or len(legs) != 1
                or len(start_tick_interval_ms) != 2
                or any(not isinstance(t, int) or not 0 <= t <= 0xffffffff
                       for t in start_tick_interval_ms)
                or not 0 <= start_tick_interval_ms[1]-start_tick_interval_ms[0] <= 1000
                or max(abs(t-legs[0]["t0"]) for t in start_tick_interval_ms) > 1000):
            raise ValueError("Start interval needs one matching single-leg capture")
    audit = dict(file=str(path), sha256=digest.hexdigest(), counts=dict(counts),
                 free_space_confirmed=header.get("free_space") is True,
                 command_components_same_tick=header.get("command_components_same_tick"),
                 observation_gaps=len(gaps), max_source_age_ms=max_source_age,
                 max_temperature_c=max_temperature,
                 all_feedback_raw_ranges_deg=np.rad2deg(
                     np.column_stack((raw_low, raw_high))).tolist(),
                 final_tick_ms=final["telemetry"]["tick_ms"],
                 final_state={key: final[key] for key in
                     ("phase", "fresh", "enabled", "submitted", "fault", "limited", "gravity_age_ms")})
    if start_tick_interval_ms is not None:
        audit["conservative_start_tick_interval_ms"] = list(start_tick_interval_ms)
    continuous = header.get("target_coordinates") == "board_continuous_reference"
    if continuous:
        audit["continuous_reference_reconstruction"] = []
    if historical_ranges:
        audit.update(historical_reference_ranges_rad=references.tolist(),
                     current_reference_ranges_differ=not np.allclose(references, REFS, rtol=0, atol=2e-6),
                     historical_ranges_are_analysis_only=True)
    derived = []
    for leg in legs:
        streams = [np.asarray(s).reshape(-1, 4) for s in leg["streams"]]
        if continuous:
            anchor = np.deg2rad(leg["start_software_deg"])+REFS[:, 0]
            offsets = []
            for i, stream in enumerate(streams):
                if np.max(np.diff(stream[:, 0])) > 1000:
                    raise ValueError("Cannot infer continuous turns across a long observation gap")
                unwrapped = np.unwrap(stream[:, 1])
                turns = round((anchor[i]-unwrapped[0])/(2*np.pi))
                stream[:, 1] = unwrapped+turns*2*np.pi
                offsets.append(turns)
            audit["continuous_reference_reconstruction"].append(
                dict(joint=leg["joint"], leg=leg["leg"], initial_raw_to_reference_turns=offsets))
        grid = np.arange(math.ceil(max(s[0, 0] for s in streams)/grid_ms)*grid_ms,
                         min(s[-1, 0] for s in streams), float(grid_ms))
        values = [np.column_stack([np.interp(grid, s[:, 0], s[:, k])
                                   for s in streams]) for k in (1, 2, 3)]
        rawq, tau, temp = values
        q = rawq-REFS[:, 0]+np.deg2rad(HW[:, 6])
        def filt(x, kernel):
            return np.column_stack([np.convolve(x[:, i], kernel[::-1], "valid")
                                    for i in range(5)])
        qs, v, a = [filt(q, k) for k in kernels]
        tau, temp = filt(tau, kernels[0]), filt(temp, kernels[0])
        time = grid[half:-half]
        ramp = leg.get("ramp_ms", 0)
        margin = 200 if ramp else 1000
        lo, hi = start_tick_interval_ms or (leg["t0"], leg["t0"])
        keep = ((time >= hi+ramp+margin)
                & (time <= lo+leg["duration_ms"]-ramp-margin))
        for lo, hi in gaps:
            keep &= (time+100 < lo) | (time-100 > hi)
        j = leg["joint"]-1
        reference_velocity = leg.get("constant_velocity_rad_s")
        if reference_velocity is None:
            reference_velocity = (leg["target_software_deg"][j]-leg["start_software_deg"][j])
        keep &= (abs(v[:, j]) > .005) & (v[:, j]*reference_velocity > 0)
        if historical_ranges:
            # Do not fold the old J4 multi-turn angle into today's range.
            # Historical bounds validate only the recorded data, never motion.
            tolerance = np.deg2rad(.05)
            raw = qs-np.deg2rad(HW[:, 6])+REFS[:, 0]
            keep &= np.all((raw >= references[:, 1]-tolerance)
                           & (raw <= references[:, 2]+tolerance), axis=1)
        fixed, basis, _ = regressors(qs, v, a)
        zero = np.zeros_like(qs)
        g0, gb, _ = regressors(qs, zero, zero)
        dynamic = fixed+basis@NOMINAL-g0-gb@NOMINAL
        residual = tau-support_gravity(qs)
        derived.append(dict(pose=pose, joint=j+1, leg=leg["leg"],
            group=(pose, round(abs(leg.get("constant_velocity_rad_s") or 0), 3)),
            sign=1 if reference_velocity > 0 else -1,
            q=qs[keep], v=v[keep, j], r=residual[keep, j], dyn=dynamic[keep, j],
            temp=temp[keep, j], source_tick_ms=time[keep]))
    return audit, derived


def pair_legs(legs, angle_bin_deg=.5, *, components=False):
    """Same record, raw-angle/speed cells; do not call time samples independent."""
    if not math.isfinite(angle_bin_deg) or angle_bin_deg <= 0:
        raise ValueError("Positive finite angle bin required")
    groups = defaultdict(list)
    for leg in legs:
        groups[(leg["group"], leg["joint"])].append(leg)
    points = []
    for (group, joint), both in groups.items():
        if len(both) != 2 or {r["sign"] for r in both} != {-1, 1}:
            raise ValueError("Missing/duplicated forward-reverse leg")
        cells = {}
        for r in both:
            raw = np.rad2deg(r["q"][:, joint-1]-np.deg2rad(HW[joint-1, 6])+REFS[joint-1, 0])
            angle, speed = np.floor(raw/angle_bin_deg).astype(int), np.digitize(abs(r["v"]), SPEED_BINS)
            for key in set(zip(angle, speed)):
                use = (angle == key[0]) & (speed == key[1])
                if sum(use) < 3:
                    continue
                cells[(r["sign"], key)] = dict(
                    q=np.median(r["q"][use], axis=0), v=float(np.median(abs(r["v"][use]))),
                    y=np.array([np.median((r["r"]-scale*r["dyn"])[use])
                                for scale in (0, 1, 2)]),
                    temp=float(np.median(r["temp"][use])))
        for sign, key in cells:
            if sign != 1 or (-1, key) not in cells:
                continue
            pos, neg = cells[(1, key)], cells[(-1, key)]
            if (np.max(abs(np.rad2deg(pos["q"]-neg["q"]))) > .5
                    or abs(pos["v"]-neg["v"]) > .3*max(pos["v"], neg["v"])):
                continue
            point = dict(pose=group[0], planned=group[1], joint=joint,
                raw_angle_center_deg=(key[0]+.5)*angle_bin_deg, q=(pos["q"]+neg["q"])/2,
                speed=(pos["v"]+neg["v"])/2, targets=(pos["y"]-neg["y"])/2,
                temperature=(pos["temp"]+neg["temp"])/2)
            if components:
                # Keep both measured poses/residuals for gravity-even analysis;
                # the default friction map and its matching criteria are unchanged.
                point.update(positive=pos, negative=neg)
            points.append(point)
    return points


def load_features(q):
    """Dimensionless CAD wrench proxies, NOT bearing contact normal forces."""
    q = np.asarray(q)
    n = len(q)
    rotation, position = np.tile(np.eye(3), (n, 1, 1)), np.zeros((n, 3))
    origins, axes, centers = [], [], []
    for i, row in enumerate(SCENE):
        position = position+np.einsum("nij,j->ni", rotation, row[:3])
        fixed = np.einsum("nij,jk->nik", rotation, row[6:15].reshape(3, 3))
        c, s = np.cos(q[:, i]), np.sin(q[:, i])
        turn = np.zeros((n, 3, 3))
        turn[:, 0, 0] = turn[:, 2, 2] = c
        turn[:, 1, 1] = 1
        turn[:, 0, 2], turn[:, 2, 0] = -s, s
        rotation = fixed@turn
        origins.append(position.copy())
        axes.append(-fixed[:, :, 1])
        centers.append(position+np.einsum("nij,j->ni", rotation, row[3:6]))
    gravity, result, signed = np.array([0., -9.81, 0.]), [], []
    lengths = []
    for i in range(5):
        force = SCENE[i:, -2].sum()*gravity
        moment = sum((np.cross(centers[k]-origins[i], SCENE[k, -2]*gravity)
                      for k in range(i, 5)), np.zeros((n, 3)))
        axial = axes[i]@force
        drive = np.einsum("ni,ni->n", moment, axes[i])
        radial = np.sqrt(np.maximum(0., force@force-axial**2))
        bend = np.sqrt(np.maximum(0., np.sum(moment**2, axis=1)-drive**2))
        reach = np.linalg.norm(SCENE[i+1:, :3], axis=1).sum()+np.linalg.norm(SCENE[i:, 3:6], axis=1).max()
        lengths.append(float(reach))
        weight = np.linalg.norm(force)
        result.append(np.column_stack((abs(axial)/weight, radial/weight,
                                       bend/(weight*reach), abs(drive)/(weight*reach))))
        signed.append(drive)
    return np.stack(result, axis=1), lengths, np.column_stack(signed)


def weights(pose, planned):
    groups = list(zip(pose, planned))
    count = Counter(groups)
    w = np.array([1/count[g] for g in groups])
    return w/w.sum()


def load_coordinate(z, pose, w):
    mu = np.array([np.average(z[pose == p], axis=0, weights=w[pose == p]) for p in (0, 1)])
    direction = mu[1]-mu[0]
    if direction@direction < 1e-12:
        raise ValueError("No independent load-pose contrast")
    coordinate = (z-mu[0])@direction/(direction@direction)
    perpendicular = np.linalg.norm(z-mu[0]-coordinate[:, None]*direction, axis=1)
    return coordinate, mu, direction, perpendicular


def design(speed, coordinate):
    u = np.log(speed/.08)
    return np.column_stack((np.ones(len(u)), u, coordinate, coordinate*u))


def fit(x, y, w, columns):
    """Robust, group-balanced weighted least squares in log-amplitude space."""
    positive = y > 0
    if sum(positive) < len(columns)+2:
        raise ValueError("Insufficient positive directional observations")
    a, target, base = x[positive][:, columns], np.log(y[positive]), w[positive]
    norm = np.linalg.norm(a*np.sqrt(base[:, None]), axis=0)
    if np.any(norm < 1e-12):
        raise ValueError("Unexcited mapping coefficient")
    a = a/norm
    effective = base.copy()
    for _ in range(12):
        beta, _, rank, sv = np.linalg.lstsq(a*np.sqrt(effective[:, None]),
                                           target*np.sqrt(effective), rcond=1e-10)
        if rank != len(columns):
            raise ValueError("Rank-deficient mapping")
        residual = target-a@beta
        scale = 1.4826*np.median(abs(residual-np.median(residual)))
        if scale < 1e-10:
            break
        effective = base*np.minimum(1., 1.345*scale/np.maximum(abs(residual), 1e-12))
    full = np.zeros(x.shape[1])
    full[list(columns)] = beta/norm
    return full, float(sv[0]/sv[-1])


def predict(x, beta):
    return np.exp(x@beta)


def analyze(points, draws=500):
    models = []
    for joint in range(1, 6):
        ps = [p for p in points if p["joint"] == joint]
        q = np.array([p["q"] for p in ps])
        speed, pose, planned, target = [np.array([p[k] for p in ps])
                                        for k in ("speed", "pose", "planned", "targets")]
        z = load_features(q)[0][:, joint-1]
        w = weights(pose, planned)
        coordinate, mu, direction, perpendicular = load_coordinate(z, pose, w)
        x, y = design(speed, coordinate), target[:, 1]
        scores = {}
        for name, columns in MODELS.items():
            folds = []
            for held in (.02, .08, .16):
                train, test = planned != held, planned == held
                if not np.any(test) or set(pose[train]) != {0, 1}:
                    continue
                wt = weights(pose[train], planned[train])
                _, mt, dt, _ = load_coordinate(z[train], pose[train], wt)
                lt = (z-mt[0])@dt/(dt@dt)
                xt = design(speed, lt)
                try:
                    beta, _ = fit(xt[train], y[train], wt, columns)
                    error = predict(xt[test], beta)-y[test]
                    folds.append(float(np.sqrt(np.average(error**2,
                        weights=weights(pose[test], planned[test])))))
                except ValueError:
                    folds = []
                    break
            if len(folds) == 3:
                scores[name] = dict(fold_rmse_nm=folds, mean=float(np.mean(folds)),
                                   se=float(np.std(folds, ddof=1)/math.sqrt(3)))
        if not scores:
            raise ValueError(f"J{joint}: no complete leave-speed-out validation")
        best = min(scores, key=lambda k: scores[k]["mean"])
        threshold = scores[best]["mean"]+scores[best]["se"]
        selected = min((k for k in scores if scores[k]["mean"] <= threshold+1e-12),
                       key=lambda k: (len(MODELS[k]), scores[k]["mean"]))
        columns = MODELS[selected]
        beta, condition = fit(x, y, w, columns)
        prediction = predict(x, beta)
        fc = [.64, .54, .12, .12, .13][joint-1]
        current = fc*np.tanh(planned/.02)
        cloud, failures = [], 0
        rng = np.random.default_rng(20261122+joint)
        group = list(zip(pose, planned))
        raw_angle = np.array([p["raw_angle_center_deg"] for p in ps])
        for _ in range(draws):
            chosen = []
            for g in sorted(set(group)):
                indexes = np.flatnonzero((pose == g[0]) & (planned == g[1]))
                blocks = np.floor(raw_angle[indexes]/2).astype(int)
                names = np.unique(blocks)
                for b in rng.choice(names, len(names), replace=True):
                    chosen.extend(indexes[blocks == b])
            ids = np.asarray(chosen)
            try:
                cloud.append(fit(x[ids], y[ids], weights(pose[ids], planned[ids]), columns)[0])
            except ValueError:
                failures += 1
        cloud = np.asarray(cloud)
        evaluations = []
        for p in (0, 1):
            for v in (.02, .08, .16):
                xx = design(np.array([v]), np.array([p]))
                interval = (np.percentile(np.exp(cloud@xx[0]), [2.5, 97.5]).tolist()
                            if len(cloud) else None)
                evaluations.append(dict(load_coordinate=p, speed_rad_s=v,
                    amplitude_nm=float(predict(xx, beta)[0]), conditional_95_interval_nm=interval,
                    velocity_in_observed_domain=bool(speed.min() <= v <= speed.max())))
        sensitivity = []
        for scale in (0, 2):
            b, _ = fit(x, target[:, scale], w, columns)
            sensitivity.append(dict(nominal_dynamic_multiplier=scale, coefficients=b.tolist()))
        groups = [dict(pose=int(p), planned_speed=float(v),
                       cells=int(sum((pose == p) & (planned == v))))
                  for p, v in sorted(set(group))]
        raw_q_deg = np.rad2deg(q-np.deg2rad(HW[:, 6])+REFS[:, 0])
        models.append(dict(joint=joint, selected_model=selected, coefficients=beta.tolist(),
            formula="tau_comp=sign(v)*exp(a+b*ln(abs(v)/0.08)+c*lambda(q)+d*lambda(q)*ln(abs(v)/0.08))",
            lambda_formula="lambda=(z(q)-load_center_0) dot load_direction / (load_direction dot load_direction)",
            load_center_0=mu[0].tolist(), load_center_1=mu[1].tolist(),
            load_direction=direction.tolist(), cells=len(ps), nonpositive_targets=int(sum(y <= 0)),
            groups=groups, scaled_design_condition=condition, cv=scores,
            train_rmse_nm=float(np.sqrt(np.average((prediction-y)**2, weights=w))),
            current_reference_ff_rmse_nm=float(np.sqrt(np.average((current-y)**2, weights=w))),
            observed_speed_range_rad_s=[float(speed.min()), float(speed.max())],
            observed_lambda_range=[float(coordinate.min()), float(coordinate.max())],
            perpendicular_load_distance_95=float(np.percentile(perpendicular, 95)),
            raw_angle_ranges_deg=np.column_stack((raw_q_deg.min(axis=0),
                                                   raw_q_deg.max(axis=0))).tolist(),
            temperature_range_c=[float(min(p["temperature"] for p in ps)),
                                 float(max(p["temperature"] for p in ps))],
            bootstrap=dict(draws=draws, successful=len(cloud), failed=failures,
                method="2-degree raw-angle blocks, stratified by pose/planned-speed; model and load axis fixed",
                scope="conditional on these two captures/model/selection; NOT independent pose/temperature validation"),
            coefficient_conditional_95_interval=(np.percentile(cloud, [2.5, 97.5], axis=0).tolist()
                                                 if len(cloud) else None),
            evaluations=evaluations, nominal_dynamic_sensitivity=sensitivity))
    return models


def self_test():
    rng = np.random.default_rng(17)
    speed = np.exp(rng.uniform(np.log(.01), np.log(.2), 200))
    coordinate = np.tile([0., 1.], 100)
    x = design(speed, coordinate)
    truth = np.array([math.log(.4), -.12, .2, .07])
    recovered, _ = fit(x, predict(x, truth), np.ones(200)/200, (0, 1, 2, 3))
    assert np.max(abs(recovered-truth)) < 1e-8
    assert np.all(predict(x, recovered) > 0)
    q = rng.uniform(-1, 1, (15, 5))
    z, _, signed = load_features(q)
    zero = np.zeros_like(q)
    fixed, basis, _ = regressors(q, zero, zero)
    assert np.max(abs(signed-fixed-basis@NOMINAL)) < 2e-6
    assert np.all(np.isfinite(z)) and np.all(z >= 0)
    features = rng.normal(size=(200, 4))
    features[:, 2:] = .1
    w = np.ones(200)/200
    xx, load_basis, columns = campaign_design(features, speed, w, "speed_load_2")
    truth = np.array([math.log(.2), -.05, .13, -.07])
    yy = predict(xx, truth)
    beta, fitted_basis, _ = campaign_fit(features, speed, yy, w, "speed_load_2")
    assert np.max(abs(campaign_predict(features, speed, "speed_load_2", beta, fitted_basis)-yy)) < 1e-9
    held = features[:10]+.4
    center_before = load_basis["center"].copy()
    campaign_design(held, speed[:10], None, "speed_load_2", load_basis)
    assert np.array_equal(center_before, load_basis["center"])
    print("PASS: synthetic maps, positive amplitude, CAD/RNE consistency, two-load-rank recovery, frozen validation basis")


CAMPAIGN_MODELS = {
    "constant": (0, (0,), False),
    "speed_only": (0, (0, 1), False),
    "load_1": (1, (0, 2), False),
    "speed_load_1": (1, (0, 1, 2), False),
    "speed_load_2": (2, (0, 1, 2, 3), False),
    "speed_load_interaction_1": (1, (0, 1, 2, 3), True),
}


def campaign_paths(directory):
    for pose, path in capture_paths(directory):
        yield pose, 1, path
    for pose, repeat in ((2, 2), (1, 2), (3, 1), (4, 1), (3, 2), (4, 2)):
        for speed in ("002", "008", "016"):
            for scope in ("j1234", "j5"):
                yield pose-1, repeat, directory/f"friction_pose{pose:02d}_r{repeat}_{speed}_{scope}.jsonl"


def condition_weights(pose, planned, repeat):
    groups = list(zip(pose, planned, repeat))
    count = Counter(groups)
    w = np.array([1/count[g] for g in groups])
    return w/w.sum()


def campaign_design(z, speed, w, name, basis=None):
    """Fit load centering/scaling on training data only; at most two load directions."""
    rank, columns, interaction = CAMPAIGN_MODELS[name]
    if basis is None:
        center = np.average(z, axis=0, weights=w)
        spread = np.sqrt(np.average((z-center)**2, axis=0, weights=w))
        active = spread > 1e-5
        projection = np.zeros((4, rank))
        if rank:
            scaled = (z[:, active]-center[active])/spread[active]
            _, sv, vt = np.linalg.svd(scaled*np.sqrt(w[:, None]), full_matrices=False)
            if len(sv) < rank or sv[rank-1] < sv[0]*.03:
                raise ValueError("Insufficient independent observed load variation")
            projection[active] = vt[:rank].T/spread[active, None]
        basis = dict(center=center, projection=projection)
    u = np.log(speed/.08)
    x = np.zeros((len(speed), 4))
    x[:, 0], x[:, 1] = 1, u
    if rank:
        components = (z-basis["center"])@basis["projection"]
        x[:, 2:2+rank] = components
        if interaction:
            x[:, 3] = components[:, 0]*u
    return x, basis, columns


def campaign_fit(z, speed, y, w, name):
    x, basis, columns = campaign_design(z, speed, w, name)
    beta, condition = fit(x, y, w, columns)
    return beta, basis, condition


def campaign_predict(z, speed, name, beta, basis):
    x, _, _ = campaign_design(z, speed, None, name, basis)
    return predict(x, beta)


def matched_repeat(ps):
    """Strict actual-q/speed overlap audit, not a condition on issuing motion."""
    matched, counts = [], []
    for pose in range(4):
        for planned in (.02, .08, .16):
            first = [p for p in ps if p["pose"] == pose and p["planned"] == planned and p["repeat"] == 1]
            second = [p for p in ps if p["pose"] == pose and p["planned"] == planned and p["repeat"] == 2]
            used, errors, distances = set(), [], []
            for a in first:
                choices = []
                for k, b in enumerate(second):
                    distance = float(np.max(abs(np.rad2deg(a["q"]-b["q"]))))
                    speed_error = abs(a["speed"]-b["speed"])/max(a["speed"], b["speed"])
                    if k not in used and distance <= .5 and speed_error <= .3:
                        choices.append((distance+speed_error, k, distance))
                if choices:
                    _, k, distance = min(choices)
                    used.add(k)
                    errors.append(float(second[k]["targets"][1]-a["targets"][1]))
                    distances.append(distance)
            counts.append(dict(pose=pose+1, planned_speed=planned, matched_cells=len(errors),
                               first_cells=len(first), second_cells=len(second)))
            matched.extend(errors)
    return dict(max_all_joint_distance_deg=.5, max_relative_speed_difference=.3,
                one_to_one=True, groups=counts, cells=len(matched),
                rmse_nm=float(np.sqrt(np.mean(np.square(matched)))) if matched else None,
                bias_nm=float(np.mean(matched)) if matched else None)


def analyze_campaign(points, previous, draws):
    output = []
    for joint in range(1, 6):
        ps = [p for p in points if p["joint"] == joint]
        q = np.array([p["q"] for p in ps])
        z = load_features(q)[0][:, joint-1]
        speed, pose, planned, repeat, targets = [np.array([p[k] for p in ps])
            for k in ("speed", "pose", "planned", "repeat", "targets")]
        y = targets[:, 1]
        w = condition_weights(pose, planned, repeat)
        training, independent = repeat == 1, repeat == 2
        expected = {(p, s, r) for p in range(4) for s in (.02, .08, .16) for r in (1, 2)}
        observed = set(zip(pose, planned, repeat))
        missing = [dict(pose=p+1, planned_speed=s, repeat=r)
                   for p, s, r in sorted(expected-observed)]
        scores = {}
        for name in CAMPAIGN_MODELS:
            folds = []
            for held in range(4):
                tr, te = training & (pose != held), training & (pose == held)
                if not tr.any() or not te.any():
                    folds = []
                    break
                try:
                    beta, basis, _ = campaign_fit(z[tr], speed[tr], y[tr],
                        condition_weights(pose[tr], planned[tr], repeat[tr]), name)
                    pred = campaign_predict(z[te], speed[te], name, beta, basis)
                    folds.append(float(np.sqrt(np.average((pred-y[te])**2,
                        weights=condition_weights(pose[te], planned[te], repeat[te])))))
                except ValueError:
                    folds = []
                    break
            if len(folds) == 4:
                scores[name] = dict(held_pose_rmse_nm=folds, mean=float(np.mean(folds)),
                                   se=float(np.std(folds, ddof=1)/2))
        if not scores:
            raise ValueError(f"J{joint}: no full independent-pose CV model")
        best = min(scores, key=lambda k: scores[k]["mean"])
        threshold = scores[best]["mean"]+scores[best]["se"]
        selected = min((k for k in scores if scores[k]["mean"] <= threshold+1e-12),
            key=lambda k: (len(CAMPAIGN_MODELS[k][1]), scores[k]["mean"]))
        beta1, basis1, _ = campaign_fit(z[training], speed[training], y[training],
            condition_weights(pose[training], planned[training], repeat[training]), selected)
        held_prediction = campaign_predict(z, speed, selected, beta1, basis1)
        old = previous["models"][joint-1]
        direction = np.asarray(old["load_direction"])
        coordinate = (z-np.asarray(old["load_center_0"]))@direction/(direction@direction)
        frozen_prediction = predict(design(speed, coordinate), np.asarray(old["coefficients"]))
        current_prediction = [.64, .54, .12, .12, .13][joint-1]*np.tanh(planned/.02)
        def score(prediction, mask):
            if not mask.any():
                return None
            return float(np.sqrt(np.average((prediction[mask]-y[mask])**2,
                weights=condition_weights(pose[mask], planned[mask], repeat[mask]))))
        validation = []
        for p in range(4):
            test = independent & (pose == p)
            validation.append(dict(pose=p+1, second_repeat_cells=int(test.sum()),
                first_round_model_rmse_nm=score(held_prediction, test),
                frozen_two_pose_model_rmse_nm=score(frozen_prediction, test),
                current_reference_ff_rmse_nm=score(current_prediction, test)))
        beta, basis, condition = campaign_fit(z, speed, y, w, selected)
        x, _, columns = campaign_design(z, speed, None, selected, basis)
        rng, cloud, failures = np.random.default_rng(20261122+joint), [], 0
        raw_angle = np.array([p["raw_angle_center_deg"] for p in ps])
        group = list(zip(pose, planned, repeat))
        for _ in range(draws):
            chosen = []
            for g in sorted(set(group)):
                indexes = np.flatnonzero((pose == g[0]) & (planned == g[1]) & (repeat == g[2]))
                blocks = np.floor(raw_angle[indexes]/2).astype(int)
                names = np.unique(blocks)
                for b in rng.choice(names, len(names), replace=True):
                    chosen.extend(indexes[blocks == b])
            ids = np.asarray(chosen)
            try:
                cloud.append(fit(x[ids], y[ids],
                    condition_weights(pose[ids], planned[ids], repeat[ids]), columns)[0])
            except ValueError:
                failures += 1
        cloud = np.asarray(cloud)
        raw = np.rad2deg(q-np.deg2rad(HW[:, 6])+REFS[:, 0])
        sensitivity = []
        for scale in (0, 2):
            try:
                alternative, _ = fit(x, targets[:, scale], w, columns)
                sensitivity.append(dict(nominal_dynamic_multiplier=scale,
                    coefficients=alternative.tolist(),
                    prediction_change_rms_nm=float(np.sqrt(np.average(
                        (predict(x, alternative)-predict(x, beta))**2, weights=w)))))
            except ValueError as exc:
                sensitivity.append(dict(nominal_dynamic_multiplier=scale, unavailable=str(exc)))
        evaluations = []
        for p in range(4):
            selected_pose = pose == p
            center_z = np.average(z[selected_pose], axis=0, weights=w[selected_pose])
            for v in (.02, .08, .16):
                xx, _, _ = campaign_design(center_z[None, :], np.array([v]), None, selected, basis)
                amplitude = float(predict(xx, beta)[0])
                evaluations.append(dict(pose=p+1, actual_load_feature_mean=center_z.tolist(),
                    speed_rad_s=v, amplitude_nm=amplitude,
                    delta_vs_current_reference_ff_nm=amplitude-[.64, .54, .12, .12, .13][joint-1]*math.tanh(v/.02),
                    conditional_95_interval_nm=(np.percentile(np.exp(cloud@xx[0]), [2.5, 97.5]).tolist()
                                                if len(cloud) else None),
                    velocity_in_observed_domain=bool(speed[selected_pose].min() <= v <= speed[selected_pose].max())))
        output.append(dict(joint=joint, selected_model=selected, coefficients=beta.tolist(),
            log_amplitude_formula="log(A)=beta0+beta1*log(abs(v)/.08)+beta2*loadPC1+beta3*loadPC2; interaction model uses loadPC1*log(abs(v)/.08) as last term",
            load_center=basis["center"].tolist(), load_projection=basis["projection"].tolist(),
            load_feature_names=previous["feature_order"], cells=len(ps),
            conditions_without_qualified_pairs=missing,
            condition_cell_counts=[dict(pose=p+1, planned_speed=s, repeat=r,
                cells=int(np.sum((pose == p) & (planned == s) & (repeat == r))))
                for p, s, r in sorted(expected)],
            nonpositive_targets=int(sum(y <= 0)), scaled_design_condition=condition,
            first_repeat_leave_pose_out=scores, independent_second_repeat=validation,
            independent_validation_model=dict(coefficients=beta1.tolist(),
                load_center=basis1["center"].tolist(), load_projection=basis1["projection"].tolist(),
                training_repeats=[1], tested_repeats=[2]),
            independent_all_second_repeats_rmse_nm=score(held_prediction, independent),
            frozen_two_pose_new_first_repeats_rmse_nm=score(frozen_prediction, training & (pose >= 2)),
            strict_actual_pose_repeat_comparison=matched_repeat(ps),
            observed_speed_range_rad_s=[float(speed.min()), float(speed.max())],
            raw_angle_ranges_deg=np.column_stack((raw.min(axis=0), raw.max(axis=0))).tolist(),
            temperature_range_c=[float(min(p["temperature"] for p in ps)),
                                 float(max(p["temperature"] for p in ps))],
            bootstrap=dict(draws=draws, successful=len(cloud), failed=failures,
                method="2-degree angle blocks stratified by pose/speed/repeat; fixed selected basis/model",
                scope="conditional intervals; only two independent rounds per nominal pose"),
            coefficient_conditional_95_interval=(np.percentile(cloud, [2.5, 97.5], axis=0).tolist()
                                                 if len(cloud) else None),
            representative_measured_load_evaluations=evaluations,
            nominal_dynamic_sensitivity=sensitivity,
            train_rmse_nm=score(predict(x, beta), np.ones(len(ps), dtype=bool))))
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, default=ROOT/"build/v12-speed-friction-20261121")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--bootstrap", type=int, default=500)
    parser.add_argument("--campaign", action="store_true",
                        help="audit all four load poses/two rounds; freeze previous two-pose map for validation")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if args.output is None or args.bootstrap < 0:
        parser.error("pass a new --output path and nonnegative --bootstrap")
    if args.output.exists():
        parser.error("output already exists; no overwrite")
    if args.campaign:
        original = args.directory/"friction_map_2pose_01.json"
        previous = json.loads(original.read_text(encoding="utf-8"))
        audits, points, total_legs = [], [], 0
        for pose, repeat, path in campaign_paths(args.directory):
            audit, legs = load_legs(path, pose)
            audit.update(pose=pose+1, repeat=repeat)
            audits.append(audit)
            total_legs += len(legs)
            paired = pair_legs(legs)
            for point in paired:
                point["repeat"] = repeat
            points.extend(paired)
        if len(audits) != 46 or total_legs != 240:
            raise ValueError("Incomplete four-pose/two-round campaign")
        result = dict(status="four_pose_two_round_exploratory_not_deployed", deploy=False,
            physical_files=len(audits), trajectory_legs=total_legs,
            frozen_two_pose_map_sha256=hashlib.sha256(original.read_bytes()).hexdigest(),
            captures=audits, models=analyze_campaign(points, previous, args.bootstrap),
            limitations=[s for s in previous["limitations"]
                if not s.startswith(("Two load poses", "Leave-speed-out"))]+[
                "Pose labels are nominal: repeat comparisons require actual q/speed overlap",
                "Independent second rounds are not used for model selection or load-basis fitting",
                "Final reported coefficients refit all observations only after independent scores are computed",
                "Four nominal poses do not guarantee identification of four independent load effects",
                "Leave-pose-out model selection uses first rounds only; second-round testing is independent of that selection"])
        with args.output.open("x", encoding="utf-8") as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
        print("CAMPAIGN_MAP", args.output, "files=", len(audits), "legs=", total_legs,
              "paired_cells=", len(points))
        for m in result["models"]:
            print(json.dumps({k: m[k] for k in ("joint", "selected_model",
                "independent_all_second_repeats_rmse_nm",
                "strict_actual_pose_repeat_comparison")}, separators=(",", ":")))
        return
    audits, legs = [], []
    for pose, path in capture_paths(args.directory):
        audit, derived = load_legs(path, pose)
        audits.append(audit)
        legs.extend(derived)
    points = pair_legs(legs)
    result = dict(status="local_empirical_exploratory_not_deployable", deploy=False,
        logical_conditions=6, physical_files=len(audits), trajectory_legs=len(legs),
        feature_order=["abs_axial_force/weight", "radial_force/weight",
                       "bending_moment/(weight*reach)", "abs_nominal_G/(weight*reach)"],
        characteristic_reach_m=load_features(np.zeros((1, 5)))[1],
        input_coordinates="qCAD radians; raw-reference zero and current CAD offset exactly once",
        captures=audits, models=analyze(points, args.bootstrap),
        limitations=["Drive-reported torque is not independently calibrated torque truth",
            "Only direction-odd average load; asymmetric friction and gravity/cable even residual not uniquely separated",
            "Two load poses span one modeled load direction, not a unique full axial/radial/bending/temperature law",
            "Actual sliding speed, not planned speed, is the regressor; no zero-speed/static-friction claim",
            "SG-filtered nominal M/C correction is not newly identified physical dynamics",
            "Leave-speed-out is internal validation; no independent repeat/third-pose validation",
            "Power law is only local; no extrapolation to zero speed, unseen loads or arbitrary raw J4 turns",
            "Confidence intervals are conditional, not a probability the whole physical model is correct",
            "Original contact/command-atomicity metadata remain unchanged",
            "This is a TOTAL direction load map: replace/compare existing friction once, never blindly add twice"])
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    print("MAP", args.output, "logical_conditions=6", "files=", len(audits),
          "legs=", len(legs), "paired_cells=", len(points))
    for m in result["models"]:
        print(json.dumps({k: m[k] for k in ("joint", "selected_model", "coefficients",
            "cells", "train_rmse_nm", "current_reference_ff_rmse_nm",
            "observed_speed_range_rad_s", "cv")}, separators=(",", ":")))


if __name__ == "__main__":
    main()
