"""Offline J1..J5 inverse-dynamics identification. Reads captures; never opens a port or writes firmware."""
import argparse
import hashlib
import json
import math
import re
from pathlib import Path

import numpy as np

from dm_monitor import JOINT_REFERENCES
from five_control import CONTROL_VERSION, EXPECTED

ROOT = Path(__file__).resolve().parents[1]
NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
MASK = 0xffffffff


def model_config():
    rows = [list(map(float, re.findall(NUMBER, line))) for line in
            (ROOT/"config/five_scene.inc").read_text().splitlines() if line.lstrip().startswith("{{")]
    scene = np.array(rows)
    if scene.shape != (5, 26):
        raise ValueError("Unexpected rigid-body scene")
    text = (ROOT/"config/five_mit.c").read_text()
    coordinate_text = (ROOT/"config/five_coordinates.h").read_text()
    macros = dict(re.findall(r"#define (\w+_CAD)\s+\(?("+NUMBER+r")f?", coordinate_text))
    hw = text.split("g_five_mit_hw[FIVE_AXES] = {", 1)[1].split("};", 1)[0]
    profile = np.array([[float(macros.get(v.strip(), v.strip()).rstrip("f"))
                         for v in row.split(",")] for row in re.findall(r"\{([^{}]+)\}", hw)])
    if profile.shape != (5, 7) or np.any(profile[:, 4] <= 0):
        raise ValueError("Unexpected five-axis MIT profile")
    return scene, profile


SCENE, HW = model_config()
WORKSPACE_CAD = np.array([float(v) for v in re.search(
    r"#define FIVE_WORKSPACE_CAD_OFFSETS \{([^}]+)\}",
    (ROOT/"config/five_coordinates.h").read_text())[1].split(",")])
NOMINAL = np.r_[np.ones(4), SCENE[:, -1]]
PARAMS = (["link_scale_J"+str(j) for j in range(2, 6)] +
          [f"{name}_J{j}" for name in ("armature_kg_m2", "Fc_Nm", "B_Nm_s_rad") for j in range(1, 6)])
REFS = np.array([[r.zero_rad, r.raw_lower_rad, r.raw_upper_rad] for r in JOINT_REFERENCES.values()])


def regressors(q, v, a):
    """CAD radians, rad/s, rad/s². Exact link G, M*a and velocity coupling, not diagonal-joint fits.

    Geometry, COM and normalized inertia stay CAD. A positive link scale varies mass and its tensor
    together. Link 1's own yaw inertia is inseparable from rotor 1, so keep link 1 fixed.
    """
    q, v, a = [np.asarray(x, dtype=float) for x in (q, v, a)]
    if q.ndim != 2 or q.shape[1] != 5 or q.shape != v.shape or q.shape != a.shape:
        raise ValueError("Expected N by 5 measured vectors")
    n = len(q)
    omega = np.zeros((n, 3))
    alpha = omega.copy()
    acc = np.tile([0., -9.81, 0.], (n, 1))
    axis = np.array([0., -1., 0.])
    rotate = lambda r, x: np.einsum("nij,nj->ni", r, x)
    rotations, basis = [], np.zeros((n, 5, 5))
    for i, row in enumerate(SCENE):
        origin, com = row[:3], row[3:6]
        fixed, inertia = row[6:15].reshape(3, 3), row[15:24].reshape(3, 3)
        c, s = np.cos(q[:, i, None]), np.sin(q[:, i, None])
        r = np.empty((n, 3, 3))
        r[:, :, 0], r[:, :, 1], r[:, :, 2] = (
            c*fixed[:, 0] + s*fixed[:, 2], fixed[:, 1], -s*fixed[:, 0] + c*fixed[:, 2])
        rotations.append(r)
        rt = r.transpose(0, 2, 1)
        acc = rotate(rt, acc + np.cross(alpha, origin) + np.cross(omega, np.cross(omega, origin)))
        w = rotate(rt, omega)
        rate = v[:, i, None]*axis
        alpha = rotate(rt, alpha) + a[:, i, None]*axis + np.cross(w, rate)
        omega = w + rate
        f = row[-2]*(acc + np.cross(alpha, com) + np.cross(omega, np.cross(omega, com)))
        m = np.cross(com, f) + alpha@inertia.T + np.cross(omega, omega@inertia.T)
        for j in range(i, -1, -1):
            basis[:, j, i] = -m[:, 1]
            if j:
                f = rotate(rotations[j], f)
                m = rotate(rotations[j], m) + np.cross(SCENE[j, :3], f)
    eye = np.eye(5)
    y = np.concatenate([basis[:, :, 1:], eye*a[:, :, None]], axis=2)
    f = np.concatenate([eye*np.tanh(v/HW[:, 4])[:, :, None], eye*v[:, :, None]], axis=2)
    return basis[:, :, 0], y, f


def snapshot(record):
    return record.get("telemetry", record.get("snapshot", record))


def load_run(path, *, with_time=False):
    """Audit old logs too; only complete, labelled V1.2 motor data may enter the regression."""
    h = hashlib.sha256()
    meta, status, fast, end, ack, handover = {}, [], [], None, None, None
    test_gravity = None
    issues, last, updates = [], [None]*5, np.zeros(5, int)
    seen = all_enabled = 0
    with path.open("rb") as src:
        for raw in src:
            h.update(raw)
            obj = json.loads(raw.decode("utf-8-sig"))
            stage = obj.get("stage")
            if stage == "capture":
                if meta:
                    issues.append("multiple capture headers")
                meta = obj
            elif stage == "end":
                end = obj
            elif stage == "rpc_accepted":
                ack = obj.get("action")
            elif stage == "handover":
                handover = obj
            elif stage == "test_gravity_handover":
                test_gravity = obj
            if stage in ("before", "sample"):
                status.append(obj)
            t = snapshot(obj)
            joints = t.get("joints")
            if not joints:
                continue
            if [(j["joint"], j["bus"], j["protocol"], j["command_id"], j["feedback_id"])
                    for j in joints] != EXPECTED:
                raise ValueError(f"{path}: motor layout does not match V1.2")
            seen += 1
            if all(j["fresh"] and j["state"] == 1 for j in joints):
                all_enabled += 1
            for i, j in enumerate(joints):
                if j["received"] and j["received_count"] != last[i]:
                    updates[i] += 1
                    last[i] = j["received_count"]
            if stage == "feedback":
                fast.append(t)
    report = dict(file=str(path), sha256=h.hexdigest(), snapshots=seen,
                  fresh_enabled_5_snapshots=all_enabled, distinct_feedback=updates.tolist())
    if (meta.get("schema") != "ylamp-five-id-1" or meta.get("control_version") != CONTROL_VERSION
            or not re.fullmatch("[0-9a-f]{16}", str(meta.get("session", "")))):
        issues.append("missing verified five-axis session")
    refs = np.asarray(meta.get("references_rad", []))
    if refs.shape != (5, 3) or not np.allclose(refs, REFS, atol=2e-6, rtol=0):
        issues.append("missing or mismatched board calibration")
    if meta.get("free_space") is not True:
        issues.append("external torque / hand or support contact not excluded")
    if not fast or not end or ack not in ("gravity", "status", "move") or ack != meta.get("action"):
        issues.append("not a completed high-rate measurement run")
    if end and (end.get("malformed", 0) or end.get("incomplete", 0)):
        issues.append("malformed or incomplete telemetry")
    if not all_enabled:
        issues.append("no fresh, enabled five-axis snapshot")
    if not status or status[0]["stage"] != "before" or len(status) < 3:
        issues.append("insufficient controller observations")
    def healthy(s):
        phase_ok = (s["phase"] in ("GRAVITY", "MOTION", "POSITION") or
                    (s.get("stage") == "before" and s["phase"] == "DRAG"))
        return (phase_ok and not s["fault"] and not s["limited"]
                and s["fresh"] == s["enabled"] == s["submitted"] == 31
                and s["gravity_age_ms"] is not None and 0 <= s["gravity_age_ms"] <= 100)
    if any(not healthy(s) for s in status):
        issues.append("controller not supplying all five axes throughout")
    if status and status[0]["phase"] == "DRAG" and (
            not test_gravity or test_gravity["phase"] != "GRAVITY" or not healthy(test_gravity)):
        issues.append("manual idle was not explicitly handed to pure gravity")
    if ack == "move" and (not handover or not healthy(handover) or handover["phase"] != "GRAVITY"
            or not any(s["phase"] == "MOTION" for s in status) or status[-1]["phase"] != "POSITION"):
        issues.append("trajectory or handover not confirmed")
    report.update(issues=issues, session=meta.get("session"), eligible_samples=0)
    if issues:
        return report, None
    start = status[1]["telemetry"]["tick_ms"]
    length = (status[-1]["telemetry"]["tick_ms"]-start) & MASK
    if not 100 <= length <= 425000:
        issues.append("invalid MCU clock span")
        return report, None
    report["unix_interval"] = [status[1]["telemetry"]["unix_s"], status[-1]["telemetry"]["unix_s"]]
    report["analysis_origin_tick_ms"] = start
    report["command_components_same_tick"] = meta.get("command_components_same_tick")
    return align_feedback(fast, start, length, report, with_time=with_time)


def align_feedback(fast, start, length, report, *, with_time=False):
    """Shared measured-clock alignment; callers retain their own capture audit."""
    issues = report["issues"]
    streams, gaps, previous = [[] for _ in range(5)], [], [None]*5
    for t in fast:
        dt = (t["tick_ms"]-start) & MASK
        if dt > length:
            continue
        for i, j in enumerate(t["joints"]):
            if not (j["received"] and j["fresh"] and j["state"] == 1 and 0 <= j["age_ms"] <= 20
                    and ((t["tick_ms"]-j["received_tick_ms"]) & MASK) == j["age_ms"]):
                issues.append("invalid drive state or time during run")
                return report, None
            stamp, count = dt-j["age_ms"], j["received_count"]
            if previous[i] == (stamp, count):
                continue
            if previous[i]:
                old, oldcount = previous[i]
                if stamp <= old or not 0 < ((count-oldcount) & MASK) < 0x80000000:
                    issues.append("motor timestamp/count went backwards")
                    return report, None
                if stamp-old > 25 or ((count-oldcount) & MASK) != 1:
                    gaps.append((old, stamp))
            previous[i] = stamp, count
            angle = JOINT_REFERENCES[i+1].angle(j["position_rad"])
            if (angle is None or not JOINT_REFERENCES[i+1].lower_rad <= angle <= JOINT_REFERENCES[i+1].upper_rad
                    or not all(map(math.isfinite, (j["velocity_rad_s"], j["torque_nm"])))):
                issues.append("invalid measured position, velocity or drive torque")
                return report, None
            streams[i].append([stamp, angle+math.radians(HW[i, 6]), j["velocity_rad_s"], j["torque_nm"]])
    if min(map(len, streams)) < 11:
        issues.append("too few independent motor samples")
        return report, None
    # Common 100 Hz MCU clock, no extrapolation. Discard every differentiation window spanning a gap.
    st = list(map(np.asarray, streams))
    times = np.arange(max(x[0, 0] for x in st), min(x[-1, 0] for x in st)+1, 10.)
    if len(times) < 11:
        issues.append("insufficient common timeline")
        return report, None
    q, v, tau = [np.column_stack([np.interp(times, x[:, 0], x[:, k]) for x in st]) for k in (1, 2, 3)]
    # Local linear derivative of measured velocity (9 points, 80 ms); never use planned acceleration.
    half = 4
    w = np.arange(-half, half+1)*.01
    a = sum(weight*v[k:k+len(v)-2*half] for k, weight in enumerate(w))/(w@w)
    center = times[half:-half]
    good = np.ones(len(center), dtype=bool)
    for lo, hi in gaps:
        good &= (center+40 < lo) | (center-40 > hi)
    report.update(gaps=len(gaps), eligible_samples=int(sum(good)), excluded_windows=int(sum(~good)))
    if not np.any(good):
        issues.append("no continuous differentiation window")
        return report, None
    result = (q[half:-half][good], v[half:-half][good], a[good], tau[half:-half][good])
    return report, result + (center[good],) if with_time else result


def scaled_lstsq(x, target):
    norm = np.linalg.norm(x, axis=0)
    if np.any(norm <= 1e-12):
        raise ValueError("unexcited regressor column")
    coeff, _, rank, sv = np.linalg.lstsq(x/norm, target, rcond=None)
    if rank != x.shape[1]:
        raise ValueError(f"rank deficient: {rank}/{x.shape[1]}")
    return coeff/norm, float(sv[0]/sv[-1])


def design(data):
    q, v, a, torque = data
    k, y, f = regressors(q, v, a)
    # At rest the sign of stiction isn't measured. Exclude that joint's equation, not its other four axes.
    moving = np.abs(v) > .04
    return k, y, f, torque, moving


def rmse(res, keep):
    return [float(np.sqrt(np.mean(res[keep[:, i], i]**2))) if keep[:, i].any() else None
            for i in range(5)]


def identify(train, test, stribeck=False):
    for name, data in (("train", train), ("validation", test)):
        v = data[1]
        if np.any(np.minimum((v > .04).sum(axis=0), (v < -.04).sum(axis=0)) < 15):
            raise ValueError(f"{name}: missing measured bidirectional movement on one or more axes")
    k, y, f, tau, active = design(train)
    xd, xf, target = y[active], f[active], (tau-k)[active]
    nofriction, _ = scaled_lstsq(xd, target)
    inertia, trace = nofriction.copy(), []
    for _ in range(2):
        friction, _ = scaled_lstsq(xf, target-xd@inertia)
        inertia, _ = scaled_lstsq(xd, target-xf@friction)
        trace.append(dict(inertia=inertia.tolist(), friction=friction.tolist(),
                          train_rmse_nm=float(np.mean((xd@inertia+xf@friction-target)**2)**.5)))
    joint, cond = scaled_lstsq(np.column_stack((xd, xf)), target)
    q, v, a, tau2 = test
    k2, y2, f2, _, mask = design(test)
    nominal = k2+y2@NOMINAL
    pred = k2+y2@joint[:9]+f2@joint[9:]
    scores = dict(cad=rmse(nominal-tau2, mask), inertia_only=rmse(k2+y2@nofriction-tau2, mask),
                  alternating=rmse(k2+y2@inertia+f2@friction-tau2, mask), joint=rmse(pred-tau2, mask))
    zero = np.zeros_like(v)
    kg, yg, _ = regressors(q, zero, zero)
    kv, yv, _ = regressors(q, v, zero)
    gravity = kg+yg@joint[:9]
    coupling = kv+yv@joint[:9]-gravity
    accel = k2+y2@joint[:9]-gravity-coupling
    # Effective drive torque is a current-derived label; these are comparisons, not sensor calibration.
    scores.update(without_velocity_coupling=rmse(pred-coupling-tau2, mask),
                  without_acceleration=rmse(pred-accel-tau2, mask),
                  without_friction=rmse(pred-f2@joint[9:]-tau2, mask))
    static = np.all(np.abs(v) <= .04, axis=1) & np.all(np.abs(a) < .12, axis=1)
    warnings = []
    if cond > 1e4:
        warnings.append("poor normalized conditioning")
    if np.any(joint < 0) or np.any(joint[:4] <= 0):
        warnings.append("unconstrained parameters are not physical; not clipped")
    if max(np.subtract(scores["joint"], scores["cad"])) >= 0:
        warnings.append("held-out error did not improve over CAD on every joint")
    if max(np.subtract(scores["joint"], scores["without_friction"])) >= 0:
        warnings.append("friction improvement not demonstrated on every joint")
    result = dict(parameter_order=PARAMS, no_friction_fit=nofriction.tolist(), alternating=trace,
                  joint_fit=joint.tolist(), scaled_condition=cond, evaluation_rmse_nm=scores,
                  stationary_samples=int(sum(static)),
                  stationary_gravity_residual_rmse_nm=rmse(gravity-tau2, np.broadcast_to(static[:, None], v.shape)),
                  validation_peak_velocity_coupling_nm=np.max(np.abs(coupling), axis=0).tolist(),
                  validation_peak_acceleration_torque_nm=np.max(np.abs(accel), axis=0).tolist(),
                  candidate_warnings=warnings, deploy=False)
    if stribeck:
        # Learn delta and vs on TRAIN ONLY; the separate recordings never select the speed.
        residual = tau-(k+y@joint[:9]+f@joint[9:])
        extras = []
        for i in range(5):
            vel = train[1][:, i]
            slow = (np.abs(vel) > .02) & (np.abs(vel) < .12)
            if min(sum(slow & (vel > 0)), sum(slow & (vel < 0))) < 15:
                extras.append(None)
                continue
            u = np.abs(vel) > .02
            def term(speed, vs):
                return np.tanh(speed/HW[i, 4])*np.exp(-(speed/vs)**2)
            def candidate(vs):
                z = term(vel[u], vs)
                delta = max(0., float(z@residual[u, i]/(z@z+1e-20)))
                return float(np.mean((residual[u, i]-delta*z)**2)), float(vs), delta
            _, vs, delta = min(candidate(vs) for vs in np.geomspace(.025, .20, 25))
            val = (np.abs(v[:, i]) > .02) & (np.abs(v[:, i]) < .12)
            if min(sum(val & (v[:, i] > 0)), sum(val & (v[:, i] < 0))) < 15:
                extras.append(None)
                continue
            e = (pred-tau2)[val, i]
            after = e+delta*term(v[val, i], vs)
            extras.append(dict(delta_nm=delta, vs_rad_s=vs, n_validation=int(sum(val)),
                               validation_rmse_without_nm=float(np.mean(e**2)**.5),
                               validation_rmse_with_nm=float(np.mean(after**2)**.5), deployed=False))
        result["stribeck_residual_only"] = extras
    return result


def fit_files(training, validation, stribeck):
    reports, groups = [], []
    for files in (training, validation):
        runs = [load_run(p) for p in files]
        reports.extend(r for r, _ in runs)
        groups.append([d for _, d in runs if d is not None])
    result = dict(source="drive-reported torque, not independently calibrated shaft torque",
                  captures=reports, status="insufficient_data", deployed=False)
    if any(r["issues"] for r in reports):
        return result
    for i, r in enumerate(reports):
        for other in reports[:i]:
            lo, hi = r["unix_interval"]
            l2, h2 = other["unix_interval"]
            if (r["session"] == other["session"] or r["sha256"] == other["sha256"]
                    or max(lo, l2) <= min(hi, h2)):
                result["reason"] = "duplicate session or overlapping run; no independent validation"
                return result
    try:
        data = [tuple(np.concatenate(parts, axis=0) for parts in zip(*g)) for g in groups]
        result["fit"] = identify(*data, stribeck=stribeck)
        result["status"] = "candidate_not_deployed"
    except ValueError as exc:
        result["reason"] = str(exc)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    audit = sub.add_parser("audit")
    audit.add_argument("paths", nargs="+", type=Path)
    fit = sub.add_parser("fit")
    fit.add_argument("--train", type=Path, nargs="+", required=True)
    fit.add_argument("--holdout", type=Path, nargs="+", required=True)
    fit.add_argument("--stribeck", action="store_true")
    for p in (audit, fit):
        p.add_argument("--out", type=Path)
    args = parser.parse_args()
    result = (dict(captures=[load_run(p)[0] for p in args.paths]) if args.action == "audit"
              else fit_files(args.train, args.holdout, args.stribeck))
    text = json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False)
    if args.out:
        with args.out.open("x", encoding="utf8") as out:
            print(text, file=out)
    else:
        print(text)
    return int(args.action == "fit" and result["status"] == "insufficient_data")


if __name__ == "__main__":
    raise SystemExit(main())
