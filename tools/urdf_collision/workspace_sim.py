"""Recorded-domain study, OFFLINE ONLY. Reuses the installed native MuJoCo tools."""
import argparse
import ctypes
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, "D:/MuJoCo/Ylamp")
from gravity_model import GravityModel
from dynamics_model import DynamicFeedforward
from gravity_sim import state, sensor, LogMessage
import prepare
import tune_mit


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def upright(model):
    q = np.zeros(5)
    for j in (1, 2):
        def height(a):
            p = q.copy()
            p[j] = a
            origins = model.kinematics(p)[0]
            return float((origins[j + 1] - origins[j])[2])
        dc = (height(0) + height(math.pi)) / 2
        q[j] = math.atan2(height(math.pi / 2) - dc, height(0) - dc)
    return np.degrees(q)


def configuration(out, geometry_directory=None):
    model = GravityModel()
    source = ROOT / "config/joint_references.def"
    rows = re.findall(r"^JOINT_REFERENCE\((\d+),\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+),\s*(\w+)\)",
                      source.read_text(encoding="utf-8"), re.M)
    assert [int(r[0]) for r in rows] == [1, 2, 3, 4, 5]
    ranges = [[float(lo) - float(z), float(hi) - float(z)] for _, z, lo, hi, _ in rows[1:]]
    sys.path.insert(0, str(ROOT / "tools"))
    from five_identify import HW
    offset = HW[:, 6]
    config = dict(
        generatedUtc=datetime.now(timezone.utc).isoformat(),
        scope="Offline J2-J5 only; J1 fixed CAD 0; NO hardware or firmware access",
        recordSource=str(source), recordSha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        scene=str(model.scene), sceneSha256=hashlib.sha256(model.source).hexdigest(),
        urdf=str(ROOT.parent / "Ylamp-URDF-package(1)/Ylamp.urdf"),
        nativeRegression=str(ROOT / "build/solidworks_collision_v12/interferences_20261115_163720_526.json"),
        coordinateAssumption="qCAD=b+qSoftware; b comes from shared five_coordinates.h, including the "
            "recorded J2 pose for the J3 Pitch datum. Installation bias remains provisionally zero. "
            "J4/J5 midpoint mapped to CAD 0. J4 stays unwrapped; no new physical validation.",
        offsetCadDeg=offset.tolist(), signs=[1]*5,
        recordedSoftwareRangesDeg=[[0, 0], *ranges],
        mappedCadRangesDeg=[[0, 0], *[[a + b, c + b] for (a, c), b in zip(ranges, offset[1:])]],
        gridCounts=[9, 13, 17, 5], graphEdgeMaxStepDeg=3,
        finalPathMaxStepDeg=.5, trajectoryRecheckMaxStepDeg=.25,
        installationRules={p: dict(y=-.0005, axialTolerance=.00003, radius=.03)
                           for p in ("part3/part5", "part0/part2", "part1/part10")},
        limitations=["Finite surface intersection checks, no continuous/clearance/containment certificate",
                     "J1 held fixed; no cable, backlash, thermal or real torque-speed envelope",
                     "Current mass/COM and reflected unloaded J/B assumptions, ideal torque actuator",
                     "Native STEP/URDF joint-origin discrepancy about 1.57 mm"])
    if geometry_directory:
        config["geometryDirectory"] = str(geometry_directory.resolve())
    write(out / "config.json", config)
    print(json.dumps(config, ensure_ascii=False, indent=2), flush=True)


def quintic(a, b, duration, t):
    s = np.clip(t / duration, 0, 1)
    delta = b - a
    return (a + delta * (10*s**3 - 15*s**4 + 6*s**5),
            delta * (30*s**2 - 60*s**3 + 30*s**4) / duration,
            delta * (60*s - 180*s**2 + 120*s**3) / duration**2)


def simulate(out):
    config = json.loads((out / "config.json").read_text(encoding="utf-8"))
    geometry = json.loads((out / "geometry.json").read_text(encoding="utf-8"))
    model = GravityModel()
    assert hashlib.sha256(model.source).hexdigest() == config["sceneSha256"]
    root = ET.fromstring(model.source)
    # Isolated study model: old single-axis CAD bounds do not clip the recorded domain.
    # Pad simulation stops by 2 deg; commands and measured-domain breaches remain checked separately.
    bounds = np.array(config["mappedCadRangesDeg"])
    for j, name in enumerate(prepare.JOINTS):
        low, high = (bounds[j] + [-2, 2]) if j else (-118, 118)
        text = " ".join(map(str, np.radians([low, high])))
        root.find(f".//joint[@name='{name}']").set("range", text)
        root.find(f"./actuator/pid[@joint='{name}']").set("posrange", text)
    ET.SubElement(root.find("sensor"), "contact",
                  name="verification_moving_floor_contact", subtree1="link_j1",
                  geom2="floor", data="found", num="1")
    profile = np.array(tune_mit.gains(root))
    dll = prepare.native_library()
    warnings = []
    callback_type = ctypes.CFUNCTYPE(None, ctypes.POINTER(LogMessage))
    @callback_type
    def logger(pointer):
        message = pointer.contents
        if message.level >= 2:
            warnings.append(dict(level=message.level, body=(message.body or b"").decode(errors="replace")))
    dll.mju_setLogHandler.argtypes = [ctypes.c_void_p]
    dll.mju_setLogHandler.restype = ctypes.c_void_p
    previous = dll.mju_setLogHandler(ctypes.cast(logger, ctypes.c_void_p))
    sim = dynamic = None
    results, traces = [], []
    peaks = np.array([d[2] for d in prepare.DRIVES])
    report = dict(generatedUtc=datetime.now(timezone.utc).isoformat(), conditions=config,
                  gainsKpKd=profile.tolist(), dt=.002, cases=results,
                  controller="MIT PD + G(q_actual) + M(q_actual)*qdd_des + h(q_actual,v_actual); no friction FF",
                  commandPathSafety="Boundary-witness paths inset 1 deg in recorded domain for dynamic tests; "
                    "modified paths and actual trajectories require separate post-run geometry recheck. "
                    "Physical stops padded 2 deg only in study copy.",
                  status="running")
    try:
        sim = tune_mit.Simulation(dll, root, profile)
        dynamic = DynamicFeedforward(dll, sim.model, model)
        floor = dll.mj_name2id(sim.model, 20, b"verification_moving_floor_contact")
        assert floor >= 0
        clock = (ctypes.c_double * 1)()
        cases = list(geometry["paths"])
        anchor = np.array(geometry["anchorCadDeg"])
        # Matched solo vs simultaneous/staggered with identical signed per-axis excursions.
        local = np.array(geometry["coordinationDeltaCadDeg"])
        cases.extend(dict(id=f"matched_solo_J{j+1}", waypointsCadDeg=[anchor.tolist(), (anchor+np.eye(5)[j]*local[j]).tolist()])
                     for j in range(1, 5))
        cases.extend(dict(id=mode, waypointsCadDeg=[anchor.tolist(), (anchor+local).tolist()])
                     for mode in ("matched_simultaneous", "matched_staggered"))
        for speed, max_speed in (("slow", 25), ("brisk", 70)):
            for case in cases:
                command_degrees = np.array(case["waypointsCadDeg"], dtype=float)
                command_degrees[:, 1:] = np.clip(command_degrees[:, 1:], bounds[1:, 0]+1, bounds[1:, 1]-1)
                waypoints = np.radians(command_degrees)
                dll.mj_resetData(sim.model, sim.data)
                sim.q[:] = waypoints[0]
                dll.mj_setState(sim.model, sim.data, sim.q, 1 << 1)
                max_error, peak_torque, peak_speed = np.zeros(5), np.zeros(5), np.zeros(5)
                saturated, breached, held_error = np.zeros(5, int), np.zeros(5, int), np.zeros(5)
                count = floor_steps = 0
                equation_error = 0.
                actual_min, actual_max = np.full(5, np.inf), np.full(5, -np.inf)
                previous_actual = None
                for segment, (a, b) in enumerate(zip(waypoints, waypoints[1:])):
                    duration = max(.7, 1.875*float(np.degrees(abs(b-a)).max())/max_speed)
                    schedule = case.get("wristSchedule")
                    if schedule:
                        from coupled_workspace import schedule_reference
                        # Compose derivatives through the wrist curve; do not independently
                        # interpolate wrist endpoints through a potentially blocked branch.
                        maximum_derivative = max(float(abs(schedule_reference(
                            *quintic(a, b, 1., phase), config["offsetCadDeg"], schedule)[1]).max())
                            for phase in np.linspace(0, 1, 1001))
                        duration = max(.7, 1.005*math.degrees(maximum_derivative)/max_speed)
                    if case["id"].startswith("matched_"):
                        duration = max(.7, 1.875*float(abs(local).max())/max_speed)
                    shifts = np.arange(5)*duration*.15 if case["id"] == "matched_staggered" else np.zeros(5)
                    end = duration + float(shifts.max()) + .4
                    for step in range(math.ceil(end/.002)):
                        t = step*.002
                        q, v = state(sim)
                        desired, velocity, acceleration = quintic(a, b, duration, t-shifts)
                        if schedule:
                            desired, velocity, acceleration = schedule_reference(
                                desired, velocity, acceleration, config["offsetCadDeg"], schedule)
                        terms = dynamic.terms(q, v, acceleration)
                        sim.command(desired, velocity, terms["feedforward"])
                        dll.mj_step(sim.model, sim.data)
                        torque = np.array([sensor(sim, i) for i in sim.sensors])
                        expected = np.clip(profile[:, 0]*(desired-q)+profile[:, 1]*(velocity-v)+terms["feedforward"], -peaks, peaks)
                        equation_error = max(equation_error, float(abs(torque-expected).max()))
                        error = np.degrees(abs(desired-q))
                        max_error = np.maximum(max_error, error)
                        peak_torque = np.maximum(peak_torque, abs(torque))
                        peak_speed = np.maximum(peak_speed, abs(v))
                        saturated += abs(torque) >= peaks*.999
                        actual_deg = np.degrees(q)
                        actual_min = np.minimum(actual_min, actual_deg)
                        actual_max = np.maximum(actual_max, actual_deg)
                        breached += (actual_deg < bounds[:, 0]-1e-7) | (actual_deg > bounds[:, 1]+1e-7)
                        # J1 is intentionally held, not restricted to an exact mathematical point.
                        breached[0] = 0
                        held = abs(b-a) < 1e-10
                        held_error = np.maximum(held_error, np.where(held, error, 0))
                        floor_steps += sensor(sim, floor) > 0
                        count += 1
                        # Adaptive offline mesh trace: max .25 deg between saved actual poses.
                        if (previous_actual is None or abs(actual_deg-previous_actual).max() >= .20
                                or step == math.ceil(end/.002)-1):
                            if previous_actual is not None:
                                n = math.ceil(abs(actual_deg-previous_actual).max()/.25)
                                for k in range(1, max(n, 1)):
                                    traces.append(dict(case=f"{speed}/{case['id']}", cadDegrees=(previous_actual+(actual_deg-previous_actual)*k/n).tolist()))
                            traces.append(dict(case=f"{speed}/{case['id']}", cadDegrees=actual_deg.tolist()))
                            previous_actual = actual_deg
                        dll.mj_getState(sim.model, sim.data, clock, 1)
                        if abs(clock[0]-count*.002) > 1e-7:
                            raise RuntimeError("Simulation time discontinuity")
                final_q, _ = state(sim)
                final_deg = np.degrees(final_q)
                actual_min = np.minimum(actual_min, final_deg)
                actual_max = np.maximum(actual_max, final_deg)
                traces.append(dict(case=f"{speed}/{case['id']}", cadDegrees=final_deg.tolist()))
                results.append(dict(id=case["id"], speed=speed, simulatedSeconds=count*.002,
                    commandWaypointsCadDeg=command_degrees.tolist(),
                    actualCadRangesDeg=np.column_stack((actual_min, actual_max)).tolist(),
                    maxErrorDeg=max_error.tolist(), peakTorqueNm=peak_torque.tolist(),
                    peakSpeedRadS=peak_speed.tolist(), saturationSamples=saturated.tolist(),
                    recordedBoundaryBreachSamples=breached.tolist(), floorContactSteps=int(floor_steps),
                    heldAxisMaxErrorDeg=held_error.tolist(), nativeMITEquationErrorNm=equation_error))
                print(f"{speed}/{case['id']}: error={max_error.max():.4f} deg; saturation={saturated.sum()}; floor={floor_steps}", flush=True)
        report["status"] = "completed"
    except Exception as error:
        report.update(status="failed", error=str(error))
        raise
    finally:
        if dynamic:
            dynamic.close()
        if sim:
            sim.close()
        dll.mju_setLogHandler(previous)
        report["warnings"] = warnings
        report["sourceSceneUnchanged"] = prepare.SCENE.read_bytes() == model.source
        write(out / "dynamics.json", report)
        write(out / "actual_trace.json", dict(maxStepDeg=.25, samples=traces))


def native_plan(out):
    """Reuse independent assembly transforms; move all 24 instances together."""
    from solidworks_plan import from_sw, to_sw, moved_frames, verify_rigid_groups
    old = json.loads((ROOT / "build/solidworks_collision_v12/native_test_plan_v2.json").read_text(encoding="utf-8"))
    geometry = json.loads((out / "geometry.json").read_text(encoding="utf-8"))
    zero_pose = old["displayPoses"][0]
    by_part = {c["part"]: c for c in zero_pose["components"]}
    zero = [from_sw(by_part[p]["transform"]) for p in ["part7", "part7", "part6", "part5", "part2", "part10"]]
    chain = ["base_link", *[f"link_j{k}" for k in range(1, 6)]]
    original = {c["name"]: c["transform"] for c in zero_pose["components"]}
    tests = [("mapped_upright", geometry["anchorCadDeg"])]
    tests.extend((p["id"], p["waypointsCadDeg"][-1]) for p in geometry["paths"])
    tests.extend((f"collision_{k}", p["cadDegrees"]) for k, p in enumerate(geometry["collisionWitnesses"][:3]))
    poses = []
    for label, q in tests:
        frames = moved_frames(zero, q)
        components = []
        for c in zero_pose["components"]:
            k = chain.index(c["link"])
            components.append({**c, "transform": to_sw(frames[k] @ np.linalg.inv(zero[k]) @ from_sw(c["transform"]))})
        verify_rigid_groups(components, original)
        poses.append(dict(label=label, cadDegrees=q, components=components,
                          crossLinkScope=zero_pose["crossLinkScope"]))
    write(out / "native_plan.json", {**old, "poses": poses, "displayPoses": poses,
                                   "limitations": old["limitations"]+["Recorded-domain mapping is hypothetical, outside old single-axis limits"]})
    print("Native witness poses:", len(poses), flush=True)


def summarize(out):
    read = lambda name: json.loads((out / name).read_text(encoding="utf-8"))
    geometry, dynamics, trace = map(read, ("geometry.json", "dynamics.json", "trajectory_geometry.json"))
    config = read("config.json")
    offset = np.array(config["offsetCadDeg"])
    states = np.array([s["cadDegrees"] for s in read("connected_samples.json")["states"]])
    model = GravityModel()
    points = np.array([model.kinematics(np.radians(q))[2][-1] for q in states])
    cases = dynamics["cases"]
    by_trace = trace["cases"]
    expected = {f"{speed}/{p['id']}" for speed in ("slow", "brisk")
                for p in geometry["paths"]}
    expected.update(f"{speed}/{name}" for speed in ("slow", "brisk")
                    for name in [*(f"matched_solo_J{j}" for j in range(2, 6)),
                                 "matched_simultaneous", "matched_staggered"])
    assert dynamics["status"] == "completed"
    assert expected == {c["speed"]+"/"+c["id"] for c in cases} == set(by_trace)
    assert sum(c["checked"] for c in by_trace.values()) == trace["samples"]
    actual = read("actual_trace.json")
    previous, largest_trace_step = {}, 0.
    for sample in actual["samples"]:
        q, key = np.array(sample["cadDegrees"]), sample["case"]
        assert q.shape == (5,) and np.isfinite(q).all()
        if key in previous:
            largest_trace_step = max(largest_trace_step, float(abs(q-previous[key]).max()))
        previous[key] = q
    assert len(actual["samples"]) == trace["samples"]
    assert largest_trace_step <= config["trajectoryRecheckMaxStepDeg"]+1e-6
    for source, expected_hash in (
            (Path(config["scene"]), config["sceneSha256"]),
            (Path(config["recordSource"]), config["recordSha256"]),
            (Path(config["urdf"]), geometry["input"]["urdfSha256"])):
        assert hashlib.sha256(source.read_bytes()).hexdigest() == expected_hash
    accepted = []
    for c in cases:
        key = c["speed"]+"/"+c["id"]
        c["acceptedUnderModel"] = (not by_trace[key]["colliding"]
            and not sum(c["saturationSamples"]) and not sum(c["recordedBoundaryBreachSamples"])
            and not c["floorContactSteps"] and c["nativeMITEquationErrorNm"] < 1e-8
            and not dynamics["warnings"] and dynamics["sourceSceneUnchanged"])
        if c["acceptedUnderModel"]:
            accepted.append(c)
    comparisons = []
    for speed in ("slow", "brisk"):
        for mode in ("matched_simultaneous", "matched_staggered"):
            together = next(c for c in cases if c["speed"] == speed and c["id"] == mode)
            for j in range(1, 5):
                solo = next(c for c in cases if c["speed"] == speed and c["id"] == f"matched_solo_J{j+1}")
                comparisons.append(dict(speed=speed, mode=mode, joint=f"J{j+1}",
                    soloPeakErrorDeg=solo["maxErrorDeg"][j],
                    coupledPeakErrorDeg=together["maxErrorDeg"][j],
                    soloPeakTorqueNm=solo["peakTorqueNm"][j],
                    coupledPeakTorqueNm=together["peakTorqueNm"][j],
                    peakTorqueChangeNm=together["peakTorqueNm"][j]-solo["peakTorqueNm"][j]))
    envelope = []
    for shoulder in np.unique(states[:, 1]):
        group = states[abs(states[:, 1]-shoulder) < 1e-6]
        neutral = group[(abs(group[:, 3]) < 1e-6) & (abs(group[:, 4]) < 1e-6)]
        envelope.append(dict(J2SoftwareDeg=float(shoulder-offset[1]),
            reachableJ3SoftwareSamplesDeg=np.unique(group[:, 2]-offset[2]).tolist(),
            neutralWristJ3SoftwareSamplesDeg=np.unique(neutral[:, 2]-offset[2]).tolist()))
    valid_paths = [c for c in accepted if not c["id"].startswith("matched_")]
    tested_ranges = None
    if valid_paths:
        ranges = np.array([c["actualCadRangesDeg"] for c in valid_paths])
        tested_ranges = np.column_stack((ranges[:, :, 0].min(axis=0),
                                         ranges[:, :, 1].max(axis=0)))-offset[:, None]
    native = read("native_results.json") if (out / "native_results.json").exists() else None
    native_results = [] if native is None else [
        dict(label=r["label"], cadDegrees=r["cadDegrees"],
             confirmed=r["confirmedCrossLinkOverlaps"], possible=r["possibleCrossLinkOverlaps"],
             restored=r["restoredAndVerified"]) for r in native["results"]]
    summary = dict(generatedUtc=datetime.now(timezone.utc).isoformat(),
        status="conditional_model_results_not_calibrated_hardware_limits",
        assumptions=config, grid={k:geometry[k] for k in
            ("gridChecked", "gridFree", "anchorConnected", "unreachableFree", "uniquePoseChecks",
             "graphEdgesChecked", "graphEdgesBlocked")},
        marginalSoftwareRangesDeg=geometry["marginalSoftwareRangesDeg"],
        actualAcceptedPathProjectionSoftwareDeg=None if tested_ranges is None else tested_ranges.tolist(),
        cartesianPoint="J5 CAD center of mass, NOT calibrated TCP",
        connectedPointBoundingBoxMm=(np.column_stack((points.min(axis=0),points.max(axis=0)))*1000).tolist(),
        dynamicsCases=len(cases), acceptedDynamicsCases=len(accepted),
        trajectorySamplesChecked=trace["samples"],
        trajectorySamplesColliding=sum(c["colliding"] for c in by_trace.values()),
        maximumActualTraceStepDeg=largest_trace_step,
        dynamicAcceptance="Complete cases, no sampled geometry collision, no saturation, no recorded-boundary "
            "breach, no floor contact, MIT equation error <1e-8 Nm, no engine warning, source unchanged. "
            "Tracking errors are reported, not compared to an unspecified hardware accuracy requirement.",
        maximumTrackingErrorDeg=max(max(c["maxErrorDeg"]) for c in cases),
        maximumTorqueNm=np.array([c["peakTorqueNm"] for c in cases]).max(axis=0).tolist(),
        matchedComparisons=comparisons, conditionalEnvelope=envelope,
        nativeWitnessResults=native_results, engineWarnings=dynamics["warnings"],
        sourceSceneUnchanged=dynamics["sourceSceneUnchanged"], results=cases,
        limitations=geometry["guarantee"])
    # These static cautions do not silently turn into passed dynamic certificates.
    summary["nativeBoundaryCautions"] = [r for r in native_results
        if (r["confirmed"] or r["possible"]) and not r["label"].startswith("collision_")]
    write(out / "summary.json", summary)
    print(json.dumps({k:summary[k] for k in ("grid", "marginalSoftwareRangesDeg",
        "actualAcceptedPathProjectionSoftwareDeg", "dynamicsCases", "acceptedDynamicsCases",
        "maximumTrackingErrorDeg", "maximumTorqueNm", "nativeBoundaryCautions",
        "engineWarnings", "sourceSceneUnchanged")}, ensure_ascii=False, indent=2))
    print("Matched comparisons:", json.dumps(comparisons, indent=2))


def self_test():
    a, b = np.zeros(5), np.ones(5)
    for t in (-1, 0, 2, 3):
        q, v, acc = quintic(a, b, 2, t)
        assert np.allclose(q, a if t <= 0 else b)
        assert np.allclose(v, 0) and np.allclose(acc, 0)
    t, h = .7, 1e-5
    q, v, acc = quintic(a, b, 2, t)
    assert np.allclose((quintic(a, b, 2, t+h)[0]-quintic(a, b, 2, t-h)[0])/(2*h), v)
    assert np.allclose((quintic(a, b, 2, t+h)[1]-quintic(a, b, 2, t-h)[1])/(2*h), acc)
    model = GravityModel()
    q = np.radians(upright(model))
    for j in (1, 2):
        p = model.kinematics(q)[0]
        for delta in (-.01, .01):
            trial = q.copy(); trial[j] += delta
            pp = model.kinematics(trial)[0]
            assert p[j+1, 2]-p[j, 2] > pp[j+1, 2]-pp[j, 2]
    print("PASS: quintic endpoints/derivatives and upright mapping (hypothesis only)")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=["prepare", "simulate", "native-plan", "summarize", "self-test"])
    parser.add_argument("--out", type=Path, default=ROOT / "build/workspace_20261116")
    parser.add_argument("--geometry-directory", type=Path,
                        help="Reuse already exported mesh buffers without duplicating them")
    args = parser.parse_args()
    {"prepare": lambda out: configuration(out, args.geometry_directory),
     "simulate": simulate, "native-plan": native_plan, "summarize": summarize,
     "self-test": lambda _: self_test()}[args.action](args.out)
