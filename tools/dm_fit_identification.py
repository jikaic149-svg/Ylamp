"""Offline effective friction/gravity identification. Never controls hardware.

Uses measured velocity/acceleration, subtracts nominal inertia/coupling, fits
bidirectional motion and validates on entirely separate recordings. The drive
current estimate does not identify an independently calibrated torque scale.
"""
from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
import re
import tempfile
from pathlib import Path

import numpy as np


def nominal_parameters():
    source = (Path(__file__).resolve().parents[1] / "config/robot_model.c").read_text(
        encoding="utf-8")
    names = ("a0", "a_cos2", "a_sin2", "b_cos", "b_sin", "d")
    return {name: float(re.search(r"\."+name+r"\s*=\s*([-+\deE.]+)f", source)[1])
            for name in names}


def smooth_velocity(rows, window_s=.12):
    t = np.array([float(r["feedback_ms"])*.001 for r in rows])
    v = np.array([float(r["velocity_rad_s"]) for r in rows])
    # Local measured-velocity regression, not commanded acceleration.
    velocity, acceleration = np.empty(len(rows)), np.empty(len(rows))
    for i, at in enumerate(t):
        low, high = np.searchsorted(t, (at-window_s, at+window_s))
        x = t[low:high]-at
        if len(x) < 3 or np.ptp(x) == 0:
            velocity[i], acceleration[i] = v[i], math.nan
        else:
            velocity[i], acceleration[i] = np.linalg.lstsq(
                np.column_stack((np.ones(len(x)), x)), v[low:high], rcond=None)[0]
    return velocity, acceleration


def read_recordings(paths, model, *, dynamic=False, derivative_window_s=.12,
                    extra_inertia=(0., 0.)):
    # Columns: q2 relative to UP, measured v1/v2/a1/a2, estimated tau1/tau2.
    by_joint = {1: [], 2: []}
    for path in paths:
        path = Path(path)
        report = json.loads((path / "report.json").read_text(encoding="utf-8"))
        if report["result"] != "complete_holding":
            raise ValueError(f"Not a complete recording: {path}")
        zero = math.radians(report["configuration"]["values"]["0.4"])
        with (path / "samples.csv").open(encoding="utf-8-sig", newline="") as stream:
            records = list(csv.DictReader(stream))
        axes = {j: [r for r in records if int(r["logical_joint"]) == j] for j in (1, 2)}
        smoothed = {j: smooth_velocity(axes[j], derivative_window_s) for j in (1, 2)}
        paired = {
            j: {r["telemetry_sequence"]: (r, v, a)
                for r, v, a in zip(axes[j], *smoothed[j])} for j in (1, 2)}
        for sequence in paired[1].keys() & paired[2].keys():
            r1, v1, a1 = paired[1][sequence]
            r2, v2, a2 = paired[2][sequence]
            if any(int(r["state"]) != 1 or int(r["mode"]) != 1 or
                   int(r["task_state"]) not in (2, 3, 4) for r in (r1, r2)):
                continue
            q = float(r2["reference_rad"])-zero
            if not all(math.isfinite(x) for x in (q, v1, v2, a1, a2)):
                continue
            a = model["a0"]+model["a_cos2"]*math.cos(2*q)+model["a_sin2"]*math.sin(2*q)
            b = model["b_cos"]*math.cos(q)+model["b_sin"]*math.sin(q)
            ap = -2*model["a_cos2"]*math.sin(2*q)+2*model["a_sin2"]*math.cos(2*q)
            bp = -model["b_cos"]*math.sin(q)+model["b_sin"]*math.cos(q)
            inertial = ((a+extra_inertia[0])*a1+b*a2,
                        b*a1+(model["d"]+extra_inertia[1])*a2)
            coupling = (ap*v1*v2+bp*v2*v2, -.5*ap*v1*v1)
            for j, row, v, acceleration in ((1, r1, v1, a1), (2, r2, v2, a2)):
                # Quasi-steady moving samples only: static friction is an
                # interval, so a stationary torque is NOT a dynamic friction label.
                if .025 <= abs(v) and (dynamic or abs(acceleration) < .5):
                    torque = float(row["estimated_torque_nm"])
                    values = (q, v, torque-inertial[j-1]-coupling[j-1])
                    by_joint[j].append(values+(acceleration,) if dynamic else values)
    return {j: np.array(rows, dtype=float).reshape(-1, 4 if dynamic else 3)
            for j, rows in by_joint.items()}


def design(data, joint, vs=None):
    q, v = data[:, 0], data[:, 1]
    columns, names = [], []
    if joint == 2:
        columns.append(np.sin(q))
        names.append("gravity_sin_nm")
    columns.append(np.sign(v))
    names.append("coulomb_nm")
    if vs is not None:
        columns.append(np.sign(v)*np.exp(-(np.abs(v)/vs)**2))
        names.append("stribeck_delta_nm")
    columns.extend((v, np.ones(len(v))))
    names.extend(("viscous_nm_s", "bias_nm"))
    if data.shape[1] == 4:
        columns.append(data[:, 3])
        names.append("extra_diagonal_inertia_kg_m2")
    return np.column_stack(columns), names


def constrained_fit(x, y, nonnegative):
    """Tiny NNLS by active-set enumeration of the few physical coefficients."""
    best = None
    for flags in itertools.product((False, True), repeat=len(nonnegative)):
        free = [i for i in range(x.shape[1])
                if i not in nonnegative or flags[nonnegative.index(i)]]
        beta = np.zeros(x.shape[1])
        beta[free] = np.linalg.lstsq(x[:, free], y, rcond=None)[0]
        if any(beta[i] < -1e-9 for i in nonnegative):
            continue
        mse = float(np.mean((x@beta-y)**2))
        if best is None or mse < best[0]:
            best = mse, beta
    return best


def fit(data, joint, stribeck=False):
    if len(data) < 40 or min(np.count_nonzero(data[:, 1] > 0),
                             np.count_nonzero(data[:, 1] < 0)) < 15:
        raise ValueError(f"J{joint}: insufficient actual bidirectional motion")
    best = None
    for vs in np.geomspace(.02, .7, 45) if stribeck else (None,):
        x, names = design(data, joint, vs)
        indices = [i for i, name in enumerate(names) if name not in ("gravity_sin_nm", "bias_nm")]
        mse, beta = constrained_fit(x, data[:, 2], indices)
        if best is None or mse < best["training_rmse_nm"]**2:
            active = [i for i, b in enumerate(beta) if abs(b) > 1e-8]
            scaled = x[:, active]/np.linalg.norm(x[:, active], axis=0)
            best = dict(parameters=dict(zip(names, map(float, beta))),
                        stribeck_speed_rad_s=None if vs is None else float(vs),
                        training_rmse_nm=math.sqrt(mse),
                        scaled_condition=float(np.linalg.cond(scaled)),
                        sample_count=len(data))
    return best


def evaluate(data, joint, fitted):
    x, names = design(data, joint, fitted["stribeck_speed_rad_s"])
    predicted = x@np.array([fitted["parameters"][name] for name in names])
    error = predicted-data[:, 2]
    rmse = float(np.sqrt(np.mean(error**2)))
    span = float(np.ptp(data[:, 2]))
    return dict(rmse_nm=rmse, rrmse_percent=100*rmse/span if span else None,
                mean_error_nm=float(np.mean(error)), samples=len(data),
                positive_samples=int(np.count_nonzero(data[:, 1] > 0)),
                negative_samples=int(np.count_nonzero(data[:, 1] < 0)),
                speed_range_rad_s=[float(np.min(data[:, 1])), float(np.max(data[:, 1]))],
                q2_up_range_deg=list(map(float, np.degrees(
                    [np.min(data[:, 0]), np.max(data[:, 0])]))))


def imu_stationary(paths):
    """Coherent IMU/encoder snapshots, last part of each settled segment only."""
    rows = []
    for path in paths:
        report = json.loads((Path(path)/"report.json").read_text(encoding="utf-8"))
        snapshots = report.get("imu_snapshots", [])
        if not snapshots:
            continue
        zero = report["configuration"]["values"]["0.4"]
        for start, end in zip(report["markers"], report["markers"][1:]):
            if not start["name"].endswith("_settle"):
                continue
            for c in snapshots:
                if (start["elapsed_s"]+2.5 <= c["elapsed_s"] < end["elapsed_s"]
                        and c["flags"] & 19 == 19 and c["age_ms"] <= 40
                        and math.hypot(*c["gyro"]) < 2):
                    rows.append((math.radians(c["joints"][1][3]-zero), *c["accel"]))
    return np.array(rows, dtype=float).reshape(-1, 4)


def imu_consistency(train, validation):
    """Learn the sensor mounting orbit, not an absolute link-UP calibration."""
    if len(train) < 8 or len(validation) < 4:
        return None
    q = train[:, 0]
    x = np.column_stack((np.ones(len(q)), np.cos(q), np.sin(q)))
    coefficients = np.linalg.lstsq(x, train[:, 1:], rcond=None)[0]
    orbit = coefficients[1:].T
    if np.linalg.matrix_rank(orbit) < 2:
        raise ValueError("IMU poses do not identify the joint rotation plane")
    result = dict(
        interpretation="Encoder/IMU relative-angle consistency, NOT absolute vertical accuracy",
        formula="a_sensor = c + u*cos(q2_user_UP) + v*sin(q2_user_UP)",
        coefficients_m_s2=coefficients.tolist(),
        limitations=["Accelerometer offset/scale and sensor-to-link mounting not independently calibrated",
                     "Do not overwrite the user UP zero from the largest gravity component"],
        scaled_condition=float(np.linalg.cond(x/np.linalg.norm(x, axis=0))))
    for name, data in (("training", train), ("validation", validation)):
        circular = (np.linalg.pinv(orbit)@(data[:, 1:]-coefficients[0]).T).T
        inferred = np.arctan2(circular[:, 1], circular[:, 0])
        error = np.degrees(np.arctan2(np.sin(inferred-data[:, 0]),
                                      np.cos(inferred-data[:, 0])))
        result[name] = dict(samples=len(data), angle_rmse_deg=float(np.sqrt(np.mean(error**2))),
                            max_absolute_error_deg=float(np.max(np.abs(error))),
                            encoder_up_range_deg=list(map(float, np.degrees(
                                [np.min(data[:, 0]), np.max(data[:, 0])]))))
    return result


def inertia_check(train_paths, validation_paths, model):
    """Candidate drive inertia only; CAD coupling is not reidentified here."""
    train = read_recordings(train_paths, model, dynamic=True)
    validation = read_recordings(validation_paths, model, dynamic=True)
    result = dict(interpretation="Extra effective diagonal inertia above nominal CAD; not deployed",
                  limitations=["Current estimate and acceleration differentiation limit accuracy",
                               "No full coupled inertial parameter identification"], joints={})
    for joint in (1, 2):
        data = train[joint]
        counts = [int(np.count_nonzero(data[:, 3] > .3)),
                  int(np.count_nonzero(data[:, 3] < -.3))]
        if min(counts) < 15 or not len(validation[joint]):
            result["joints"][str(joint)] = dict(
                result="Insufficient bidirectional acceleration excitation", counts=counts)
            continue
        fitted = fit(data, joint)
        fitted["acceleration_counts_above_0_3"] = counts
        fitted["validation"] = evaluate(validation[joint], joint, fitted)
        # Compare on exactly the same dynamic samples with the same CAD model.
        no_extra = fit(data[:, :3], joint)
        fitted["validation_without_extra_inertia"] = evaluate(
            validation[joint][:, :3], joint, no_extra)
        result["joints"][str(joint)] = fitted
    # Differentiation bandwidth can masquerade as inertia. Report sensitivity,
    # rather than automatically deploying a single attractive regression.
    for window_s in (.08, .18):
        alternate = read_recordings(train_paths, model, dynamic=True,
                                    derivative_window_s=window_s)
        for joint in (1, 2):
            if "parameters" in result["joints"][str(joint)]:
                candidate = fit(alternate[joint], joint)
                result["joints"][str(joint)].setdefault(
                    "differentiation_window_sensitivity", []).append(dict(
                        half_window_s=window_s,
                        extra_inertia_kg_m2=candidate["parameters"][
                            "extra_diagonal_inertia_kg_m2"]))
    return result


def impulse_windows(paths, model, window_s=.25):
    """Integral Lagrange equation: subtract CAD momentum, no differentiated v."""
    windows = {1: [], 2: []}
    for path in paths:
        path = Path(path)
        report = json.loads((path/"report.json").read_text(encoding="utf-8"))
        if report["result"] != "complete_holding":
            raise ValueError(f"Not a complete recording: {path}")
        zero = math.radians(report["configuration"]["values"]["0.4"])
        with (path/"samples.csv").open(encoding="utf-8-sig", newline="") as stream:
            pairs = {}
            for row in csv.DictReader(stream):
                pairs.setdefault(row["telemetry_sequence"], {})[
                    int(row["logical_joint"])] = row
        rows = sorted((p for p in pairs.values() if 1 in p and 2 in p),
                      key=lambda p: float(p[1]["feedback_ms"]))
        t = np.array([float(p[1]["feedback_ms"])*.001 for p in rows])
        q = np.array([float(p[2]["reference_rad"])-zero for p in rows])
        v = np.array([[float(p[j]["velocity_rad_s"]) for j in (1, 2)] for p in rows])
        tau = np.array([[float(p[j]["estimated_torque_nm"]) for j in (1, 2)]
                        for p in rows])
        valid = np.array([all(int(p[j]["state"]) == 1 and int(p[j]["mode"]) == 1
                              and int(p[j]["task_state"]) in (2, 3, 4)
                              for j in (1, 2)) for p in rows])
        a = model["a0"]+model["a_cos2"]*np.cos(2*q)+model["a_sin2"]*np.sin(2*q)
        b = model["b_cos"]*np.cos(q)+model["b_sin"]*np.sin(q)
        ap = -2*model["a_cos2"]*np.sin(2*q)+2*model["a_sin2"]*np.cos(2*q)
        bp = -model["b_cos"]*np.sin(q)+model["b_sin"]*np.cos(q)
        momentum = np.column_stack((a*v[:, 0]+b*v[:, 1],
                                    b*v[:, 0]+model["d"]*v[:, 1]))
        kinetic_q2 = .5*ap*v[:, 0]**2+bp*v[:, 0]*v[:, 1]
        for low in range(0, len(t), 5):
            high = int(np.searchsorted(t, t[low]+window_s))
            if high >= len(t) or high-low < 5:
                continue
            at = slice(low, high+1)
            dt = np.diff(t[at])
            if not valid[at].all() or np.max(dt) > .06 or np.min(dt) <= 0:
                continue
            duration = t[high]-t[low]

            def mean_integral(values):
                return float(np.sum((values[low:high]+values[low+1:high+1])*dt*.5)
                             / duration)

            for joint in (1, 2):
                j = joint-1
                # Exclude stick and reversals: sign(v) is not static friction.
                if np.min(np.abs(v[at, j])) < .025 or np.ptp(np.sign(v[at, j])) > 0:
                    continue
                x = ([mean_integral(np.sin(q))] if joint == 2 else []) + [
                    mean_integral(np.sign(v[:, j])), mean_integral(v[:, j]), 1.,
                    (v[high, j]-v[low, j])/duration]
                y = mean_integral(tau[:, j]) - (
                    momentum[high, j]-momentum[low, j])/duration
                if joint == 2:
                    y += mean_integral(kinetic_q2)
                windows[joint].append((*x, y))
    return {j: np.array(values).reshape(-1, 6 if j == 2 else 5)
            for j, values in windows.items()}


def impulse_inertia_check(train_paths, validation_paths, model):
    result = dict(
        interpretation="Integrated torque/momentum cross-check; candidates are not deployed",
        limitations=["Overlapping windows are not independent statistical samples",
                     "Torque scale, driver filtering and fixed-J3 approximation remain uncalibrated"],
        windows=[])
    for window_s in (.15, .25, .4):
        train = impulse_windows(train_paths, model, window_s)
        validation = impulse_windows(validation_paths, model, window_s)
        entry = dict(window_s=window_s, joints={})
        for j in (1, 2):
            names = (["gravity_sin_nm"] if j == 2 else []) + [
                "coulomb_nm", "viscous_nm_s", "bias_nm", "extra_diagonal_inertia_kg_m2"]
            data, val = train[j], validation[j]
            if min(len(data), len(val)) < 20:
                entry["joints"][str(j)] = dict(result="Insufficient moving windows")
                continue
            x, y = data[:, :-1], data[:, -1]
            nonnegative = [i for i, name in enumerate(names)
                           if name not in ("gravity_sin_nm", "bias_nm")]
            _, beta = constrained_fit(x, y, nonnegative)
            error = val[:, :-1]@beta-val[:, -1]
            _, no_extra = constrained_fit(x[:, :-1], y, nonnegative[:-1])
            entry["joints"][str(j)] = dict(
                parameters=dict(zip(names, map(float, beta))),
                training_windows=len(data), validation_windows=len(val),
                validation_rmse_nm=float(np.sqrt(np.mean(error**2))),
                validation_rmse_without_extra_nm=float(np.sqrt(np.mean(
                    (val[:, :-2]@no_extra-val[:, -1])**2))))
        result["windows"].append(entry)
    return result


def deployed_coefficients():
    source = (Path(__file__).resolve().parents[1]/"config/robot_model.c").read_text(
        encoding="utf-8")
    number = r"([-+\deE.]+)f"
    extra = re.search(r"\.extra_inertia_kg_m2\s*=\s*\{([^}]+)\}", source)[1]
    return dict(
        extra_inertia_kg_m2=[float(v) for v in re.findall(number, extra)],
        gravity_sin_nm=float(re.search(r"\.gravity_sin_nm\s*=\s*"+number, source)[1]),
        friction=[dict(coulomb_nm=float(a), viscous_nm_s=float(b)) for a, b in re.findall(
            r"\.coulomb_nm\s*=\s*"+number+r",\s*\.viscous_nm_s\s*=\s*"+number, source)])


def identify(train_paths, validation_paths, *, use_deployed_inertia=False, test_paths=()):
    model = nominal_parameters()
    deployed = deployed_coefficients()
    extra = deployed["extra_inertia_kg_m2"] if use_deployed_inertia else (0., 0.)
    train = read_recordings(train_paths, model, extra_inertia=extra)
    validation = read_recordings(validation_paths, model, extra_inertia=extra)
    result = dict(
        scope="effective friction and J2 sine gravity; nominal inertia/coupling subtracted",
        limitations=["Current-based estimated torque is not independently calibrated",
                     "J3 fixed-payload assumption not independently measured",
                     "No static friction value inferred from stationary samples",
                     "No inertial parameters identified by this quasi-steady experiment",
                     "No hardware changes or automatic coefficient deployment"],
        nominal_inertia_model=model, training=list(train_paths), validation=list(validation_paths),
        final_unseen_test=list(test_paths),
        subtracted_extra_inertia_kg_m2=extra, deployed_coefficients=deployed,
        joints={})
    for joint in (1, 2):
        options = {}
        if not len(validation[joint]):
            raise ValueError(f"J{joint}: no independent moving validation data")
        for name, stribeck in (("coulomb_viscous", False), ("stribeck", True)):
            fitted = fit(train[joint], joint, stribeck)
            fitted["validation"] = evaluate(validation[joint], joint, fitted)
            without_bias = dict(fitted, parameters=dict(fitted["parameters"], bias_nm=0.))
            fitted["validation_without_nuisance_bias"] = evaluate(
                validation[joint], joint, without_bias)
            options[name] = fitted
        # Prefer the smaller model unless Stribeck improves independent error.
        simple, detailed = options.values()
        selected = ("stribeck" if detailed["validation"]["rmse_nm"] <
                    .95*simple["validation"]["rmse_nm"] else "coulomb_viscous")
        baseline = dict(parameters=dict(deployed["friction"][joint-1], bias_nm=0.),
                        stribeck_speed_rad_s=None)
        if joint == 2:
            baseline["parameters"]["gravity_sin_nm"] = deployed["gravity_sin_nm"]
        result["joints"][str(joint)] = dict(
            candidates=options, selected=selected,
            deployed_effective_model_validation=evaluate(validation[joint], joint, baseline))
        per_record = []
        for path in validation_paths:
            held_out = read_recordings([path], model, extra_inertia=extra)[joint]
            chosen = options[selected]
            without_bias = dict(chosen, parameters=dict(chosen["parameters"], bias_nm=0.))
            per_record.append(dict(
                recording=str(path),
                deployed=evaluate(held_out, joint, baseline),
                candidate_without_bias=evaluate(held_out, joint, without_bias)))
        stability = []
        if len(train_paths) >= 3:
            for excluded in train_paths:
                subset = read_recordings([p for p in train_paths if p != excluded],
                                         model, extra_inertia=extra)[joint]
                stability.append(dict(excluded=str(excluded), parameters=fit(
                    subset, joint, selected == "stribeck")["parameters"]))
        result["joints"][str(joint)].update(
            per_record_validation=per_record, leave_one_training_record_out=stability,
            candidate_improves_every_validation_record=all(
                x["candidate_without_bias"]["rmse_nm"] < x["deployed"]["rmse_nm"]
                for x in per_record),
            deployment_status="offline candidates only; no firmware or coefficient write")
        # Test records are evaluated only after model selection; never used to
        # tune coefficients or choose between simple/Stribeck candidates.
        chosen = options[selected]
        without_bias = dict(chosen, parameters=dict(chosen["parameters"], bias_nm=0.))
        unseen = []
        for path in test_paths:
            data = read_recordings([path], model, extra_inertia=extra)[joint]
            unseen.append(dict(recording=str(path),
                               deployed=evaluate(data, joint, baseline),
                               candidate_without_bias=evaluate(data, joint, without_bias)))
        result["joints"][str(joint)]["final_unseen_test"] = unseen
    result["imu_consistency"] = imu_consistency(
        imu_stationary(train_paths), imu_stationary(validation_paths))
    result["inertia_check"] = inertia_check(train_paths, validation_paths, model)
    result["impulse_inertia_check"] = impulse_inertia_check(
        train_paths, validation_paths, model)
    all_paths = list(train_paths)+list(validation_paths)+list(test_paths)
    result["matched_center_returns"] = matched_center_returns(all_paths)
    result["direction_model_check"] = direction_model_check(train, validation)
    reports = [json.loads((Path(p)/"report.json").read_text(encoding="utf-8"))
               for p in all_paths]
    centers = {tuple(round(math.degrees(q), 5) for q in job["center_rad"])
               for r in reports for job in r["plan"] if "center_rad" in job}
    result["experiment_audit"] = dict(
        recordings=len(reports), centers_deg=sorted(centers),
        return_segments=sum("center_rad" in job for r in reports for job in r["plan"]),
        coupled_return_segments=sum("test_joints" in job for r in reports for job in r["plan"]),
        samples=sum(r["samples"] for r in reports),
        can_observations=[r["final_status"]["can_observations"] for r in reports],
        uart_errors=[r["final_status"]["uart_error"] for r in reports],
        uart_crc_errors=[r["final_status"]["crc_errors"] for r in reports],
        uart_rx_overflows=[r["final_status"]["rx_overflows"] for r in reports],
        capture_anchor_checks=[dict(
            recording=str(p), acknowledged_capture="capture_ack" in r,
            declared_capture_rad=r["captured_target_rad"],
            baseline_hold_target_deg=[next(
                x["target_deg"] for x in r["settled_measurements"]
                if x["segment"] == "baseline_settle" and x["joint"] == j) for j in (1, 2)],
            interpretation=("ACK plus newer telemetry" if "capture_ack" in r else
                            "Legacy pre-ACK anchor metadata; explicit test-center targets remain checked"))
            for p, r in zip(all_paths, reports)],
        final_status=reports[-1]["final_status"],
        final_status_age_s=reports[-1]["final_status_age_s"],
        firmware_modified_by_this_tool=False, parameters_deployed_by_this_tool=False,
        manual_push_release=False)
    return result


def matched_center_returns(paths):
    """Separate within-direction scatter from SAME-target bidirectional difference."""
    groups = {}
    for path in paths:
        report = json.loads((Path(path)/"report.json").read_text(encoding="utf-8"))
        if report["result"] != "complete_holding":
            raise ValueError(f"Incomplete recording: {path}")
        jobs = {j["name"]+"_settle": j for j in report["plan"]
                if "test_joint" in j or "test_joints" in j}
        for row in report["settled_measurements"]:
            job = jobs.get(row["segment"])
            if job is None or row["joint"] not in job.get(
                    "test_joints", [job.get("test_joint")]):
                continue
            joint = row["joint"]
            center = tuple(round(math.degrees(q), 5) for q in job["center_rad"])
            if abs(row["target_deg"]-center[joint-1]) > .001:
                raise ValueError("Mismatched target: cannot call this a same-center return")
            coupled = bool(job.get("test_joints"))
            family = ("diagonal_same" if job["from_offset_signs"][0] ==
                      job["from_offset_signs"][1] else "diagonal_opposed") if coupled else "single_joint"
            key = center, joint, report["test_speed_rad_s"], family
            sign = (job["from_offset_signs"][joint-1] if "test_joints" in job
                    else job["from_offset_sign"])
            groups.setdefault(key, {1: [], -1: []})[sign].append(dict(
                recording=str(path), repeat=job["repeat"], **row))
    results = []
    for (center, joint, speed, family), sides in groups.items():
        if not all(sides.values()):
            raise ValueError("Incomplete bidirectional pair")
        item = dict(center_deg=center, joint=joint, speed_rad_s=speed,
                    coupled=family != "single_joint", path_family=family)
        for sign, name in ((1, "from_plus"), (-1, "from_minus")):
            rows = sides[sign]
            errors = np.array([r["error_deg"] for r in rows])
            item[name] = dict(repeats=len(rows), mean_error_deg=float(np.mean(errors)),
                             repeat_std_deg=float(np.std(errors)),
                             repeat_span_deg=float(np.ptp(errors)), trials=rows)
        p, m = (item[name]["mean_error_deg"] for name in ("from_plus", "from_minus"))
        item.update(bidirectional_difference_deg=p-m, midpoint_error_deg=(p+m)/2)
        results.append(item)
    return dict(
        interpretation="Signed error=target-actual; same target, opposite trajectory approaches",
        limitations=["Not a manual push/release experiment",
                     "Midpoint and directional difference do not uniquely identify gravity or static friction",
                     "Output/motor encoder is not an independent end-effector measurement"],
        conditions=results)


def direction_model_check(train, validation):
    """Check extra moving-friction/phase terms, not a static-friction label."""
    result = {}
    for joint in (1, 2):
        models = {}
        for split_viscosity, gravity_cos in ((False, False), (True, False),
                                           (False, True), (True, True)):
            if gravity_cos and joint != 2:
                continue

            def matrix(data):
                q, v = data[:, 0], data[:, 1]
                columns, names = [], []
                if joint == 2:
                    columns.append(np.sin(q)); names.append("gravity_sin_nm")
                    if gravity_cos:
                        columns.append(np.cos(q)); names.append("gravity_cos_nm")
                columns.append(np.sign(v)); names.append("coulomb_nm")
                if split_viscosity:
                    columns.extend((np.maximum(v, 0), np.minimum(v, 0)))
                    names.extend(("positive_viscous_nm_s", "negative_viscous_nm_s"))
                else:
                    columns.append(v); names.append("viscous_nm_s")
                columns.append(np.ones(len(v))); names.append("bias_nm")
                return np.column_stack(columns), names

            x, names = matrix(train[joint])
            xv, _ = matrix(validation[joint])
            nonnegative = [i for i, n in enumerate(names) if
                           n not in ("gravity_sin_nm", "gravity_cos_nm", "bias_nm")]
            mse, beta = constrained_fit(x, train[joint][:, 2], nonnegative)
            error = xv@beta-validation[joint][:, 2]
            label = ("direction_viscosity" if split_viscosity else "symmetric")
            if gravity_cos:
                label += "_gravity_phase"
            models[label] = dict(
                parameters=dict(zip(names, map(float, beta))),
                training_rmse_nm=math.sqrt(mse),
                validation_rmse_nm=float(np.sqrt(np.mean(error**2))),
                validation_positive_rmse_nm=float(np.sqrt(np.mean(
                    error[validation[joint][:, 1] > 0]**2))),
                validation_negative_rmse_nm=float(np.sqrt(np.mean(
                    error[validation[joint][:, 1] < 0]**2))),
                scaled_condition=float(np.linalg.cond(x/np.linalg.norm(x, axis=0))))
        result[str(joint)] = dict(candidates=models,
            limitations=["Asymmetric Coulomb constants cannot be separated from constant bias with these data",
                         "Moving-velocity models do not remove static HOLD hysteresis automatically",
                         "Additional terms are not deployed; independent validation decides usefulness"])
    return result


def self_test():
    rows = [dict(feedback_ms=str(i*20), velocity_rad_s=str(.4+1.2*i*.02))
            for i in range(50)]
    for window in (.08, .12, .18):
        _, acceleration = smooth_velocity(rows, window)
        assert np.allclose(acceleration, 1.2)
    v = np.r_[np.linspace(-.6, -.03, 150), np.linspace(.03, .6, 150)]
    q = np.sin(np.arange(len(v))*.27)*.6
    data = np.column_stack((q, v, np.zeros(len(v))))
    x, _ = design(data, 2)
    expected = [-.49, .55, .08, -.012]
    data[:, 2] = x@expected
    fitted = fit(data, 2)
    assert np.allclose(list(fitted["parameters"].values()), expected)
    assert evaluate(data, 2, fitted)["rmse_nm"] < 1e-12
    assert fit(data, 2, True)["training_rmse_nm"] < 1e-10
    assert direction_model_check({1: data, 2: data}, {1: data, 2: data})[
        "2"]["candidates"]["symmetric"]["validation_rmse_nm"] < 1e-10
    with tempfile.TemporaryDirectory(prefix="ylamp_bidirectional_test_") as directory:
        path = Path(directory)
        jobs, rows = [], []
        for sign in (1, -1):
            name = f"return_{sign}"
            jobs.append(dict(name=name, test_joint=1, repeat=1,
                             from_offset_sign=sign, center_rad=[0, 0]))
            rows.append(dict(segment=name+"_settle", joint=1, target_deg=0,
                             error_deg=sign*.4+.1))
        (path/"report.json").write_text(json.dumps(dict(
            result="complete_holding", plan=jobs, settled_measurements=rows,
            test_speed_rad_s=.2)), encoding="utf-8")
        condition = matched_center_returns([path])["conditions"][0]
        assert abs(condition["bidirectional_difference_deg"]-.8) < 1e-10
        assert abs(condition["midpoint_error_deg"]-.1) < 1e-10
        rows[0]["target_deg"] = 1
        (path/"report.json").write_text(json.dumps(dict(
            result="complete_holding", plan=jobs, settled_measurements=rows,
            test_speed_rad_s=.2)), encoding="utf-8")
        try:
            matched_center_returns([path])
        except ValueError:
            pass
        else:
            raise AssertionError("Different targets called same-center returns")
    q = np.linspace(-.7, 1.1, 40)
    imu = np.column_stack((q, .2+0*q, 9.8*np.sin(q+.05), 9.8*np.cos(q+.05)))
    consistency = imu_consistency(imu[::2], imu[1::2])
    assert consistency["validation"]["angle_rmse_deg"] < 1e-10
    dynamic = np.column_stack((data, 2*np.cos(np.arange(len(v))*.17)))
    x, names = design(dynamic, 2)
    expected = [-.49, .55, .08, -.012, .05]
    dynamic[:, 2] = x@expected
    assert np.allclose(list(fit(dynamic, 2)["parameters"].values()), expected)
    # End-to-end integral regression against known extra inertia, including
    # both moving joints and the CAD velocity-coupling terms.
    model = nominal_parameters()
    with tempfile.TemporaryDirectory(prefix="ylamp_impulse_test_") as directory:
        path = Path(directory)
        (path/"report.json").write_text(json.dumps(dict(
            result="complete_holding", configuration=dict(values={"0.4": 0}))),
            encoding="utf-8")
        with (path/"samples.csv").open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(("telemetry_sequence", "logical_joint", "feedback_ms",
                             "reference_rad", "velocity_rad_s", "estimated_torque_nm",
                             "state", "mode", "task_state"))
            for i in range(3000):
                t = i*.01
                q2, v1, v2 = .3+.5*math.sin(.6*t), .96*math.cos(1.2*t), .3*math.cos(.6*t)
                a1, a2 = -1.152*math.sin(1.2*t), -.18*math.sin(.6*t)
                a = model["a0"]+model["a_cos2"]*math.cos(2*q2)+model["a_sin2"]*math.sin(2*q2)
                b = model["b_cos"]*math.cos(q2)+model["b_sin"]*math.sin(q2)
                ap = -2*model["a_cos2"]*math.sin(2*q2)+2*model["a_sin2"]*math.cos(2*q2)
                bp = -model["b_cos"]*math.sin(q2)+model["b_sin"]*math.cos(q2)
                torque = [(a+.03)*a1+b*a2+ap*v1*v2+bp*v2*v2+.4*np.sign(v1)+.1*v1,
                          b*a1+(model["d"]+.04)*a2-.5*ap*v1*v1
                          -.6*math.sin(q2)+.5*np.sign(v2)+.2*v2]
                for joint, v in ((1, v1), (2, v2)):
                    writer.writerow((i, joint, i*10, q2, v, torque[joint-1], 1, 1, 2))
        for joint, data in impulse_windows([path], model).items():
            beta = np.linalg.lstsq(data[:, :-1], data[:, -1], rcond=None)[0]
            assert abs(beta[-1]-(.03 if joint == 1 else .04)) < 1e-4
    print("Identification fit self-test passed (synthetic data, no hardware)")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--train", nargs="+")
    parser.add_argument("--validate", nargs="+")
    parser.add_argument("--test", nargs="+", default=[],
                        help="Final unseen records; evaluation only, not fitting/model selection")
    parser.add_argument("--output")
    parser.add_argument("--returns", nargs="+",
                        help="Summarize matched-center return recordings only; no model fit")
    parser.add_argument("--deployed-inertia", action="store_true",
                        help="Subtract the existing extra inertia when refining friction/gravity")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
    elif args.returns:
        if not args.output:
            parser.error("--output is required")
        result = matched_center_returns(args.returns)
        Path(args.output).write_text(json.dumps(result, indent=2, allow_nan=False), encoding="utf-8")
        for row in result["conditions"]:
            print(f"J{row['joint']} center={row['center_deg']} "
                  f"+side={row['from_plus']['mean_error_deg']:+.5f}deg "
                  f"-side={row['from_minus']['mean_error_deg']:+.5f}deg "
                  f"difference={row['bidirectional_difference_deg']:+.5f}deg")
    else:
        if not args.train or not args.validate or not args.output:
            parser.error("--train, --validate and --output are required")
        groups = [set(map(lambda p: Path(p).resolve(), paths))
                  for paths in (args.train, args.validate, args.test)]
        if any(groups[a] & groups[b] for a, b in ((0, 1), (0, 2), (1, 2))):
            parser.error("Training, validation and final test recordings must be separate")
        result = identify(args.train, args.validate, use_deployed_inertia=args.deployed_inertia,
                          test_paths=args.test)
        Path(args.output).write_text(json.dumps(result, indent=2, allow_nan=False), encoding="utf-8")
        print(json.dumps(result["joints"], indent=2))
