"""Conditional wrist intervals and scheduling experiments. OFFLINE ONLY."""
import argparse
from datetime import datetime, timezone
import hashlib
import itertools
import math
from pathlib import Path
import time

import numpy as np

from workspace_geometry import Checker, line, read, write


def sampled_intervals(clear, low, high, step=.5, inset=1., tolerance=.005):
    """Finite sampled runs, refined at detected transitions; NOT a certificate."""
    if not all(math.isfinite(v) for v in (low, high, step, inset, tolerance)):
        raise ValueError("Nonfinite interval argument")
    if high < low or step <= 0 or inset < 0 or tolerance <= 0:
        raise ValueError("Invalid interval arguments")
    xs = np.linspace(low, high, max(1, math.ceil((high-low)/step))+1)
    flags = [bool(clear(float(x))) for x in xs]
    runs, i = [], 0

    def boundary(good, bad):
        while abs(good-bad) > tolerance:
            middle = (good+bad)/2
            if clear(float(middle)):
                good = middle
            else:
                bad = middle
        return float(good)

    while i < len(xs):
        if not flags[i]:
            i += 1
            continue
        start = i
        while i+1 < len(xs) and flags[i+1]:
            i += 1
        a = xs[start] if start == 0 else boundary(xs[start], xs[start-1])
        b = xs[i] if i+1 == len(xs) else boundary(xs[i], xs[i+1])
        if b-a >= 2*inset:
            runs.append([float(a+inset), float(b-inset)])
        i += 1
    return runs


def conditional_intervals(checker, config, software, joint=4, step=.5, inset=1.):
    software = np.asarray(software, dtype=float)
    bounds = np.array(config["recordedSoftwareRangesDeg"])
    if software.shape != (5,) or not np.isfinite(software).all():
        raise ValueError("Expected five finite software joint angles, in degrees")
    if joint not in (3, 4) or software[0] != 0:
        raise ValueError("Only wrist scans with J1 fixed at zero are supported")
    fixed = [j for j in range(1, 5) if j != joint]
    if np.any(software[fixed] < bounds[fixed, 0]) or np.any(software[fixed] > bounds[fixed, 1]):
        raise ValueError("Fixed joints outside recorded domain")
    cad = software+np.array(config["offsetCadDeg"])

    def clear(x):
        cad[joint] = x+config["offsetCadDeg"][joint]
        return not checker.hits(cad)
    return sampled_intervals(clear, *bounds[joint], step, inset)


def schedule_reference(q, velocity, acceleration, offset, parameters):
    """Compose a C2 wrist schedule with a CAD-radian shoulder/elbow reference."""
    q, velocity, acceleration = [np.array(a, dtype=float, copy=True)
                                 for a in (q, velocity, acceleration)]
    start, end = parameters["sumStartDeg"], parameters["sumEndDeg"]
    if end <= start:
        raise ValueError("Schedule end must exceed start")
    total = math.degrees(q[1]+q[2])-offset[1]-offset[2]
    u = np.clip((total-start)/(end-start), 0., 1.)
    r = 10*u**3-15*u**4+6*u**5
    first = (30*u**2-60*u**3+30*u**4)/(end-start)
    second = (60*u-180*u**2+120*u**3)/(end-start)**2
    speed = math.degrees(velocity[1]+velocity[2])
    acc = math.degrees(acceleration[1]+acceleration[2])
    amplitude = np.array([parameters["J4AmplitudeDeg"], parameters["J5AmplitudeDeg"]])
    q[3:] = np.radians(np.array(offset[3:])+amplitude*r)
    velocity[3:] = np.radians(amplitude*first*speed)
    acceleration[3:] = np.radians(amplitude*(second*speed**2+first*acc))
    return q, velocity, acceleration


def policy_pose(q2, q3, offset, parameters):
    q = np.radians(np.array(offset)+[0, q2, q3, 0, 0])
    return np.degrees(schedule_reference(q, np.zeros(5), np.zeros(5), offset, parameters)[0])


def axis_samples(low, high, step):
    return np.unique([*np.linspace(low, high, math.ceil((high-low)/step)+1), 0.])


def study(source, out):
    started = time.perf_counter()
    config = read(source / "config.json")
    offset = np.array(config["offsetCadDeg"])
    bounds = np.array(config["recordedSoftwareRangesDeg"])
    checker = Checker(source)
    print(f"Meshes ready in {time.perf_counter()-started:.1f}s", flush=True)

    def validate(points, parameters, perturb=False):
        count, collisions, examples = 0, 0, []
        for q2, q3 in points:
            q = policy_pose(q2, q3, offset, parameters)
            candidates = [q]
            if perturb:
                # ponytail: finite angular box corners, not a continuous or distance certificate.
                candidates = [q+np.array([0, *delta])*.5
                              for delta in itertools.product((-1, 1), repeat=4)]
            for candidate in candidates:
                if (np.any(candidate[1:] < offset[1:]+bounds[1:, 0]-1e-8)
                        or np.any(candidate[1:] > offset[1:]+bounds[1:, 1]+1e-8)):
                    continue
                found = checker.hits(candidate)
                count += 1
                if found:
                    collisions += 1
                    if len(examples) < 8:
                        examples.append(dict(softwareDegrees=(candidate-offset).tolist(), pairs=found))
        return dict(checked=count, colliding=collisions, examples=examples)

    # A small hypothesis search, followed by denser and independent validation.
    candidates = [(90., 130.), (80., 125.), (60., 120.)]
    fit_points = list(itertools.product(*(axis_samples(*bounds[j], 5.) for j in (1, 2))))
    fitting, chosen = [], None
    for start, end in candidates:
        profiles = [dict(sumStartDeg=start, sumEndDeg=end, J4AmplitudeDeg=sign*160., J5AmplitudeDeg=20.)
                    for sign in (-1, 1)]
        checks = [validate(fit_points, p) for p in profiles]
        fitting.append(dict(parameters=profiles, validation=checks))
        print(f"Fit {start:g}..{end:g}: {[c['colliding'] for c in checks]}", flush=True)
        if not any(c["colliding"] for c in checks):
            chosen = profiles
            break
    if chosen is None:
        raise RuntimeError("Simple sum schedules rejected; retain conditional interval method")
    grid_points = list(itertools.product(*(axis_samples(*bounds[j], 2.) for j in (1, 2))))
    rng = np.random.default_rng(20261116)
    random_points = rng.uniform(bounds[1:3, 0], bounds[1:3, 1], (5000, 2))
    near_points = rng.uniform([15, 65], bounds[1:3, 1], (400, 2))
    validation = []
    for parameters in chosen:
        result = dict(parameters=parameters, grid=validate(grid_points, parameters),
                      randomHoldout=validate(random_points, parameters),
                      angularCornerStress=validate(near_points, parameters, True))
        validation.append(result)
        print(f"Dense/holdout/stress: {result}", flush=True)
    assert not any(r[k]["colliding"] for r in validation
                   for k in ("grid", "randomHoldout", "angularCornerStress")), "Schedule validation failed"

    # This is an implicit interval function, not independently clamped joint limits.
    # Keep multiple components rather than joining their endpoints across collisions.
    interval_rows, interval_checks = [], dict(includedChecked=0, includedColliding=0, outsideButClear=0)
    for q2 in (15., 25., 35., bounds[1, 1]):
        for q3 in (80., 90., 100., bounds[2, 1]):
            for q4 in (0., -80., -160., 160.):
                software = np.array([0, q2, q3, q4, 0])
                intervals = conditional_intervals(checker, config, software)
                interval_rows.append(dict(J2SoftwareDeg=q2, J3SoftwareDeg=q3,
                                          J4SoftwareDeg=q4, J5SoftwareIntervalsDeg=intervals))
                for q5 in rng.uniform(*bounds[4], 80):
                    included = any(a <= q5 <= b for a, b in intervals)
                    found = checker.hits(offset+software+[0, 0, 0, 0, q5])
                    interval_checks["includedChecked"] += included
                    interval_checks["includedColliding"] += bool(included and found)
                    interval_checks["outsideButClear"] += bool(not included and not found)
    assert interval_checks["includedColliding"] == 0, "Sample interval missed an interior collision"
    print(f"Conditional interval validation: {interval_checks}", flush=True)

    def make_case(name, arms, profile):
        waypoints = [policy_pose(q2, q3, offset, profile).tolist() for q2, q3 in arms]
        # Check the actual scheduled curve, not just a line between wrist endpoints.
        checked = 0
        for a, b in zip(waypoints, waypoints[1:]):
            # <= .1 deg in shoulders plus the composed wrist; recheck dense curve directly.
            n = max(1, math.ceil(max(abs(np.array(b)-a))/.025))
            for t in np.linspace(0, 1, n+1):
                arms = (1-t)*np.array(a)[1:3]+t*np.array(b)[1:3]-offset[1:3]
                assert not checker.hits(policy_pose(*arms, offset, profile)), name
                checked += 1
        return dict(id=name, waypointsCadDeg=waypoints, wristSchedule=profile,
                    scheduledCurveChecked=checked)

    lo, hi = bounds[:, 0]+1, bounds[:, 1]-1
    corner, left, bottom = (hi[1], hi[2]), (lo[1], hi[2]), (hi[1], lo[2])
    paths = []
    for profile in chosen:
        side = "negative" if profile["J4AmplitudeDeg"] < 0 else "positive"
        for name, arms in (
                ("diagonal", [(0, 0), corner, (0, 0)]),
                ("shoulder_at_upper_elbow", [(0, 0), left, corner, left, (0, 0)]),
                ("elbow_at_upper_shoulder", [(0, 0), bottom, corner, bottom, (0, 0)])):
            paths.append(make_case(f"scheduled_{side}_{name}", arms, profile))
    negative, positive = [policy_pose(*corner, offset, p) for p in chosen]
    branch_switch_hits = [dict(cadDegrees=q.tolist(), pairs=found)
                          for q in line(negative, positive, .5) if (found := checker.hits(q))]
    assert branch_switch_hits, "Expected upper-corner direct branch switch counterexample"
    target_config = dict(config, coupledSource=str(source.resolve()),
                         scope="Offline coupled J2-J5 interval/schedule study; J1 target held at zero")
    out.mkdir(parents=True, exist_ok=False)
    write(out / "config.json", target_config)
    write(out / "geometry.json", dict(anchorCadDeg=offset.tolist(),
          paths=paths, coordinationDeltaCadDeg=[0, 8, -12, 20, 5]))
    write(out / "study.json", dict(generatedUtc=datetime.now(timezone.utc).isoformat(),
          source=str(source.resolve()), conditionalMapping=config["coordinateAssumption"],
          parameters=chosen, fitting=fitting, validation=validation,
          intervalFunction="q5 in union_k [L_k(q2,q3,q4),U_k(q2,q3,q4)]; degree software coordinates",
          intervalRows=interval_rows, intervalValidation=interval_checks,
          directUpperCornerBranchSwitch=dict(checked=math.ceil(320/.5)+1,
              colliding=len(branch_switch_hits), witnesses=branch_switch_hits[:5]),
          scheduledCurvesChecked=sum(p["scheduledCurveChecked"] for p in paths),
          uniquePoseChecks=checker.checks, elapsedSeconds=time.perf_counter()-started,
          limitations=["Finite sampled model experiment; not continuous collision or clearance certification",
                       "Interval boundaries inset 1 angular degree, not a millimeter distance margin",
                       "C2 wrist sum schedule is one tested posture preference, NOT the allowed workspace",
                       "No interpolation of interval tables is authorized as a safety rule",
                       *config["limitations"]]))
    print(f"Study ready: {out}; elapsed={time.perf_counter()-started:.1f}s; poses={checker.checks}", flush=True)


def finish(out):
    config, study_data = read(out / "config.json"), read(out / "study.json")
    dynamics, geometry = read(out / "dynamics.json"), read(out / "trajectory_geometry.json")
    trace = read(out / "actual_trace.json")
    expected = {f"{speed}/{p['id']}" for p in read(out / "geometry.json")["paths"]
                for speed in ("slow", "brisk")}
    expected.update(f"{speed}/{name}" for speed in ("slow", "brisk")
                    for name in [*(f"matched_solo_J{j}" for j in range(2, 6)),
                                 "matched_simultaneous", "matched_staggered"])
    assert dynamics["status"] == "completed"
    assert expected == {c["speed"]+"/"+c["id"] for c in dynamics["cases"]} == set(geometry["cases"])
    assert len(trace["samples"]) == geometry["samples"] == sum(c["checked"] for c in geometry["cases"].values())
    paths = {p["id"]: p for p in read(out / "geometry.json")["paths"]}
    previous, max_step, residual = {}, 0., np.zeros(2)
    for sample in trace["samples"]:
        q = np.array(sample["cadDegrees"])
        assert q.shape == (5,) and np.isfinite(q).all()
        if sample["case"] in previous:
            max_step = max(max_step, float(abs(q-previous[sample["case"]]).max()))
        previous[sample["case"]] = q
        name = sample["case"].split("/", 1)[1]
        if name in paths and "wristSchedule" in paths[name]:
            software = q-np.array(config["offsetCadDeg"])
            expected_pose = policy_pose(*software[1:3], config["offsetCadDeg"], paths[name]["wristSchedule"])
            residual = np.maximum(residual, abs(q[3:]-expected_pose[3:]))
    assert max_step <= .250001
    for filename, digest in ((config["scene"], config["sceneSha256"]),
                             (config["recordSource"], config["recordSha256"])):
        assert hashlib.sha256(Path(filename).read_bytes()).hexdigest() == digest
    source_input = read(Path(config["geometryDirectory"]) / "mesh_input.json")
    assert hashlib.sha256(Path(config["urdf"]).read_bytes()).hexdigest() == source_input["urdfSha256"]
    results = []
    for case in dynamics["cases"]:
        mesh = geometry["cases"][case["speed"]+"/"+case["id"]]
        accepted = (not mesh["colliding"] and not sum(case["saturationSamples"])
                    and not sum(case["recordedBoundaryBreachSamples"]) and not case["floorContactSteps"]
                    and case["nativeMITEquationErrorNm"] < 1e-8
                    and not dynamics["warnings"] and dynamics["sourceSceneUnchanged"])
        results.append(dict(case, acceptedUnderModel=accepted))
    result = dict(status="conditional_uncalibrated_model_only", parameters=study_data["parameters"],
                  cases=len(results), accepted=sum(c["acceptedUnderModel"] for c in results),
                  actualTraceSamples=geometry["samples"],
                  collidingTraceSamples=sum(c["colliding"] for c in geometry["cases"].values()),
                  maximumActualTraceStepDeg=max_step,
                  maximumActualWristScheduleResidualDeg=(
                      residual.tolist() if any("wristSchedule" in p for p in paths.values()) else None),
                  maximumTrackingErrorDeg=max(max(c["maxErrorDeg"]) for c in results),
                  maximumPeakSpeedDegS=np.degrees(np.array([c["peakSpeedRadS"] for c in results]).max(axis=0)).tolist(),
                  maximumTorqueNm=np.array([c["peakTorqueNm"] for c in results]).max(axis=0).tolist(),
                  results=results, limitations=study_data["limitations"])
    write(out / "coupled_summary.json", result)
    print({k: v for k, v in result.items() if k not in ("results", "limitations")}, flush=True)


def probe(source, out):
    started = time.perf_counter()
    config = read(source / "config.json")
    checker = Checker(source)
    print(f"Meshes ready in {time.perf_counter()-started:.1f}s", flush=True)
    rows = []
    for q2 in (15., 25., 35., config["recordedSoftwareRangesDeg"][1][1]):
        for q3 in (80., 90., 100., config["recordedSoftwareRangesDeg"][2][1]):
            slices = []
            for q5 in (0., 10., 20., config["recordedSoftwareRangesDeg"][4][1]-1):
                intervals = conditional_intervals(checker, config, [0, q2, q3, 0, q5],
                                                  joint=3, step=2, inset=1.)
                slices.append(dict(J5SoftwareDeg=q5, J4SoftwareIntervalsDeg=intervals))
            row = dict(J2SoftwareDeg=q2, J3SoftwareDeg=q3, slices=slices)
            rows.append(row)
            print(row, flush=True)
    out.mkdir(parents=True, exist_ok=False)
    write(out / "probe.json", dict(source=str(source.resolve()), rows=rows,
          elapsedSeconds=time.perf_counter()-started, poseChecks=checker.checks,
          note="Finite sampled intervals with 1 degree endpoint inset, NOT physical clearance or certification"))


def self_test():
    intervals = sampled_intervals(lambda x: -8 <= x <= -2 or 1 <= x <= 7,
                                 -10, 10, step=.25, inset=.5)
    assert np.allclose(intervals, [[-7.5, -2.5], [1.5, 6.5]], atol=.005)
    assert sampled_intervals(lambda x: False, -1, 1) == []
    assert sampled_intervals(lambda x: True, -2, 2) == [[-1, 1]]
    assert sampled_intervals(lambda x: abs(x) < .1, -1, 1, step=.1) == []
    turns = sampled_intervals(lambda x: math.cos(math.radians(x)) < -.5,
                              -227, 227, step=1, inset=1)
    assert len(turns) == 2 and turns[0][0] == -226 and turns[1][1] == 226
    assert turns[0][1] < -120 and turns[1][0] > 120
    try:
        sampled_intervals(lambda x: True, 1, -1)
    except ValueError:
        pass
    else:
        raise AssertionError("Reversed interval accepted")
    parameters = dict(sumStartDeg=90., sumEndDeg=130., J4AmplitudeDeg=-160., J5AmplitudeDeg=20.)
    q = np.radians([0, 30, 80, 0, 0])
    v, acc = np.array([0, .1, -.04, 0, 0]), np.array([0, -.03, .07, 0, 0])
    a, speed, acceleration = schedule_reference(q, v, acc, np.zeros(5), parameters)
    h = 1e-4
    plus = schedule_reference(q+v*h+acc*h*h/2, v+acc*h, acc, np.zeros(5), parameters)[0]
    minus = schedule_reference(q-v*h+acc*h*h/2, v-acc*h, acc, np.zeros(5), parameters)[0]
    assert np.allclose((plus-minus)/(2*h), speed, atol=1e-7)
    assert np.allclose((plus-2*a+minus)/h**2, acceleration, atol=1e-6)
    for total, wrist in ((89., [0., 0.]), (90., [0., 0.]), (130., [-160., 20.]), (131., [-160., 20.])):
        pose = policy_pose(total, 0., np.zeros(5), parameters)
        assert np.allclose(pose[3:], wrist)
        _, speed, acceleration = schedule_reference(np.radians(pose), v, acc, np.zeros(5), parameters)
        assert np.allclose(speed[3:], 0) and np.allclose(acceleration[3:], 0)
    print("PASS: disjoint intervals, refinement/inset, invalid inputs, C2 schedule and chain-rule derivatives")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=["probe", "study", "finish", "query", "self-test"])
    parser.add_argument("--source", type=Path, default=Path(__file__).resolve().parents[2] / "build/workspace_20261116_v3")
    parser.add_argument("--out", type=Path)
    parser.add_argument("--software", nargs=5, type=float, help="J1..J5 software degrees; scanned joint is ignored")
    parser.add_argument("--scan-joint", type=int, choices=(4, 5), default=5)
    parser.add_argument("--step", type=float, default=.5)
    parser.add_argument("--inset", type=float, default=1.)
    args = parser.parse_args()
    if args.action == "self-test":
        self_test()
    elif args.action == "query":
        assert args.software
        config = read(args.source / "config.json")
        result = conditional_intervals(Checker(args.source), config, args.software,
                                       args.scan_joint-1, args.step, args.inset)
        print(dict(softwareDegrees=args.software, scannedJoint=args.scan_joint,
                   sampledIntervalsDeg=result, stepDeg=args.step, endpointInsetDeg=args.inset,
                   status="Offline conditional model, not real-time or hardware authorization"), flush=True)
    else:
        assert args.out
        if args.action == "finish":
            finish(args.out)
        else:
            {"probe": probe, "study": study}[args.action](args.source, args.out)
