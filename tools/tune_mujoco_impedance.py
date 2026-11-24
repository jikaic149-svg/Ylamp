"""Bounded simulation-only KEY impedance study; no serial, CAN or firmware writes.

Uses the immutable collision MJCF, its current budgets and the installed native
MuJoCo. A C access shim preserves native implicit PID while updating encoded
gains. Sliding F is ONLY a labelled dry-friction sensitivity surrogate; it does
not identify static friction, backlash or the real motor current loop.
"""
import argparse
import ctypes as ct
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np

import export_mujoco_identified as ex

SOURCE = ex.OUTPUT.with_stem(ex.OUTPUT.stem + "_collision")
OUT = ex.ROOT / "build/v12-mujoco-impedance-20261123"
CANDIDATE = ex.ROOT / "simulation/ylamp_v12_impedance_candidate.xml"
P = ct.POINTER(ct.c_double)
SCENARIOS = [
    dict(name="nominal_CAD_viscous", dry_scale=0., delay_ms=0),
    dict(name="sliding_amplitude_as_dry_proxy", dry_scale=1., delay_ms=0),
    dict(name="dry_proxy_120percent_delay4ms", dry_scale=1.2, delay_ms=4),
]


def vector(values):
    return (ct.c_double * len(values))(*map(float, values))


def encoded(value, maximum, bits, *, nearest=False, symmetric=True):
    value = np.asarray(value, float)
    count = (1 << bits) - 1
    code = (value + (maximum if symmetric else 0)) / (
        2 * maximum if symmetric else maximum) * count
    code = np.rint(code) if nearest else np.floor(code)
    code = np.clip(code, 0, count)
    return code / count * (2 * maximum if symmetric else maximum) - (
        maximum if symmetric else 0)


def build_shim():
    cache = (ex.BUILD / "host/CMakeCache.txt").read_text(encoding="utf-8")
    compiler = Path(re.search(r"^CMAKE_C_COMPILER:\w+=(.+)$", cache, re.M)[1])
    target = OUT / "native_01.dll"
    inputs = [ex.ROOT / p for p in (
        "tools/mujoco_impedance_native.c", "Model/five_dynamics.c", "config/five_mit.c")]
    if not target.exists() or target.stat().st_mtime < max(p.stat().st_mtime for p in inputs):
        OUT.mkdir(parents=True, exist_ok=True)
        cmd = [str(compiler), "-shared", "-O2", "-std=c11", "-static-libgcc",
               "-ID:/MuJoCo/4.0.1/include",
               *["-I" + str(ex.ROOT / p) for p in (
                   "Model", "Mode", "config", "Device/DM_Motor/Common", "BSP/CAN", "tests/fakes")],
               *map(str, inputs), "-o", str(target), "-lm", "-Wl,--no-insert-timestamp"]
        environment = dict(os.environ)
        environment["PATH"] = str(compiler.parent) + os.pathsep + environment.get("PATH", "")
        subprocess.run(cmd, env=environment, check=True, capture_output=True, text=True)
    if not target.is_file():
        raise RuntimeError("Host compiler did not create the requested DLL")
    return target


class Bench:
    def __init__(self, *, config=None, recorded=None):
        self.root = ET.parse(SOURCE).getroot()
        self.dll, self.model = ex.load_native(SOURCE)
        self.data = self.dll.mj_makeData(self.model)
        self.native_path = build_shim()
        self.native = ct.CDLL(str(self.native_path))
        for name, args, restype in (
                ("yi_layout", [ct.c_void_p], ct.c_int),
                ("yi_coefficients", [ct.c_void_p, ct.c_int, P], None),
                ("yi_gains", [ct.c_void_p, P, P], ct.c_int),
                ("yi_environment", [ct.c_void_p, ct.c_double, ct.c_int, P, P], None),
                ("yi_friction", [ct.c_void_p, P], None),
                ("yi_read", [ct.c_void_p, ct.c_void_p, P], None),
                ("yi_model", [P, P, P, P], ct.c_int)):
            fn = getattr(self.native, name)
            fn.argtypes, fn.restype = args, restype
        assert self.native.yi_layout(self.model)
        self.config = ex.parameters() if config is None else config
        self.baseline = self.config["g_five_support_gains"].reshape(5, 2)
        self.caps = self.config["g_five_impedance_max_nm"]
        self.g_caps = self.config["g_five_mit_hw"].reshape(5, 7)[:, 5]
        inputs = ex.inputs() if recorded is None else recorded
        self.bounds = inputs[1]
        self.center = inputs[2]
        snapshot = inputs[3]["snapshots"][-1]
        self.raw_offset = np.array(snapshot["raw_rad"]) - self.center
        self.vmax = np.array([10, 10, 30, 30, 30.])
        self.tmax = np.array([28, 28, 10, 10, 10.])
        self.old_damping = np.array([
            float(self.root.find(f".//joint[@name='J{i+1}']").get("damping")) for i in range(5)])
        self.sample = (ct.c_double * 18)()
        self.g, self.amp = (ct.c_double * 5)(), (ct.c_double * 5)()
        self.scratch_g, self.scratch_amp = (ct.c_double * 5)(), (ct.c_double * 5)()
        self.kp, self.kd, self.friction = [((ct.c_double * 5)()) for _ in range(3)]
        self.ctrl = (ct.c_double * 15)()
        self.coefficients = (ct.c_double * 20)()
        self.mass_buffer = (ct.c_double * 25)()
        self.contact_id = self.dll.mj_name2id(self.model, 20, b"collision_all")

    def close(self):
        self.dll.mj_deleteData(self.data)
        self.dll.mj_deleteModel(self.model)

    def model_terms(self, q, v=None):
        if not self.native.yi_model(vector(q), vector(np.zeros(5) if v is None else v),
                                    self.g, self.amp):
            raise RuntimeError("Original model calculation rejected a finite pose")
        return np.array(self.g), np.array(self.amp)

    def mass(self, q):
        self.dll.mj_resetData(self.model, self.data)
        self.dll.mj_setState(self.model, self.data, vector(q), 2)
        self.dll.mj_forward(self.model, self.data)
        self.dll.mj_fullM(self.model, self.data, self.mass_buffer)
        return np.array(self.mass_buffer).reshape(5, 5)

    def run(self, profile, pose, delta_deg, scenario, *, duration=2., dt=.002,
            contacts=False, keep_trace=False, pd_max_nm=None):
        m, d, dll, shim = self.model, self.data, self.dll, self.native
        dry = scenario["dry_scale"]
        damping = self.old_damping if dry == 0 else np.zeros(5)
        # The identified direction law is a TOTAL loss proxy: never add the old
        # unloaded viscous B again when using it as a dry-friction plant.
        shim.yi_environment(m, dt, int(contacts), vector(damping), vector(np.zeros(5)))
        dll.mj_resetData(m, d)
        q_start = pose + np.radians(delta_deg)
        if np.any(q_start < self.bounds[:, 0]) or np.any(q_start > self.bounds[:, 1]):
            raise ValueError("Offline perturbation outside original range")
        dll.mj_setState(m, d, vector(q_start), 2)
        anchor = encoded(pose + self.raw_offset, 12.5, 16, nearest=True).astype(np.float32)
        wire_target = encoded(anchor, 12.5, 16) - self.raw_offset
        wire_v = encoded(np.zeros(5), self.vmax, 12)
        delay = round(scenario["delay_ms"] * .001 / dt)
        control_every = round(.002 / dt)
        history = []
        steps = round(duration / dt)
        trace = np.zeros((steps, 16))
        limited = np.zeros(5)
        maximum_contacts = 0
        planned_kp, planned_kd = profile.T
        lag_s = scenario.get("torque_lag_ms", 0.) * .001
        if not math.isfinite(lag_s) or lag_s < 0:
            raise ValueError("Plant torque-lag scenario must be finite and nonnegative")
        alpha = dt / (dt + lag_s) if lag_s else 1.
        torque_state = wire_kp = wire_kd = wire_ctrl = None
        for step in range(steps):
            shim.yi_read(m, d, self.sample)
            actual = np.array(self.sample)
            q, v = actual[:5], actual[5:10]
            history.append((q.copy(), v.copy()))
            if dry:
                if not shim.yi_model(vector(q), vector(v), self.scratch_g, self.scratch_amp):
                    raise RuntimeError("Plant-friction load calculation failed")
                self.friction[:] = dry * np.array(self.scratch_amp)
                shim.yi_friction(m, self.friction)
            if step % control_every == 0:
                past_q, past_v = history[max(0, step - delay)]
                # Feedback rounding is an explicit measurement approximation;
                # command fields follow the project's truncating MIT encoder.
                raw = encoded(past_q + self.raw_offset, 12.5, 16, nearest=True).astype(np.float32)
                velocity = encoded(past_v, self.vmax, 12, nearest=True).astype(np.float32)
                measured_q = raw - self.raw_offset
                if not shim.yi_model(vector(measured_q), vector(velocity), self.g, self.amp):
                    raise RuntimeError("G calculation failed")
                gravity = np.array(self.g)
                pd = planned_kp * (anchor - raw) - planned_kd * velocity
                room = np.maximum(0, self.caps - np.abs(gravity))
                if pd_max_nm is not None:
                    room = np.minimum(room, pd_max_nm)
                scale = np.minimum(1., room / np.maximum(np.abs(pd), 1e-12))
                limited += (scale < 1) * control_every
                self.kp[:] = encoded(planned_kp * scale, 500., 12, symmetric=False)
                self.kd[:] = encoded(planned_kd * scale, 5., 12, symmetric=False)
                self.ctrl[:] = np.column_stack((
                    wire_target, wire_v, encoded(gravity, self.tmax, 12))).ravel()
                if lag_s:
                    wire_kp, wire_kd = np.array(self.kp), np.array(self.kd)
                    wire_ctrl = np.array(self.ctrl).reshape(5, 3)
                else:
                    assert shim.yi_gains(m, self.kp, self.kd)
                    dll.mj_setState(m, d, self.ctrl, 64)
            if lag_s:
                # ASSUMED PLANT, not a new controller: backward-Euler first-order
                # torque response. The encoded command remains unchanged.
                # Initialise from the held deflected equilibrium, not zero.
                if torque_state is None:
                    torque_state = (wire_kp * (wire_ctrl[:, 0] - q)
                                    + wire_kd * (wire_ctrl[:, 1] - v) + wire_ctrl[:, 2])
                self.kp[:], self.kd[:] = alpha * wire_kp, alpha * wire_kd
                filtered = wire_ctrl.copy()
                filtered[:, 2] = alpha * wire_ctrl[:, 2] + (1 - alpha) * torque_state
                self.ctrl[:] = filtered.ravel()
                assert shim.yi_gains(m, self.kp, self.kd)
                dll.mj_setState(m, d, self.ctrl, 64)
            dll.mj_step(m, d)
            shim.yi_read(m, d, self.sample)
            result = np.array(self.sample)
            if (not np.isfinite(result).all() or result[17] or np.max(abs(result[5:10])) > 10
                    or np.any(result[:5] < self.bounds[:, 0] - .01)
                    or np.any(result[:5] > self.bounds[:, 1] + .01)):
                return dict(failed=True, reason="nonfinite/warning/speed/range", step=step)
            maximum_contacts = max(maximum_contacts, int(result[16]))
            trace[step] = np.r_[result[15], result[:15]]
            if lag_s:
                torque_state = result[10:15].copy()
        if abs(trace[-1, 0] - duration) > 1e-8:
            raise RuntimeError("Native simulation reset or clock discontinuity")
        error = np.degrees(trace[:, 1:6] - pose)
        tail = max(1, round(.3 / dt))
        final = np.mean(error[-tail:], axis=0)
        moved = np.flatnonzero(np.asarray(delta_deg) != 0)
        after_crossing = [
            max(0., float(np.max(-np.sign(delta_deg[j]) * error[:, j]))) for j in moved]
        outside = np.flatnonzero(np.max(abs(error), axis=1) > .5)
        eq_outside = np.flatnonzero(np.max(abs(error - final), axis=1) > .1)
        metrics = dict(
            failed=False, final_error_deg=final.tolist(),
            iae_deg_s=float(np.mean(np.sum(abs(error[:, moved]), axis=0) * dt)),
            overshoot_deg=max(after_crossing, default=0.),
            settled_to_center_s=float((outside[-1] + 1) * dt if len(outside) else 0.),
            settled_to_equilibrium_s=float((eq_outside[-1] + 1) * dt if len(eq_outside) else 0.),
            tail_max_speed_rad_s=float(np.max(abs(trace[-tail:, 6:11]))),
            tail_position_std_deg=np.std(error[-tail:], axis=0).tolist(),
            peak_torque_nm=np.max(abs(trace[:, 11:16]), axis=0).tolist(),
            peak_speed_rad_s=float(np.max(abs(trace[:, 6:11]))),
            limited_fraction=(limited / steps).tolist(), max_contacts=maximum_contacts,
        )
        if keep_trace:
            metrics["trace_20ms"] = trace[::max(1, round(.02 / dt))].tolist()
        return metrics

    def self_test(self):
        """Small runnable checks before any gain search."""
        original = np.array([[float(a.get("kp")), float(a.get("kv"))]
                             for a in self.root.findall("./actuator/pid")])
        for i in range(5):
            self.native.yi_coefficients(self.model, i, self.coefficients)
            assert np.allclose(np.array(self.coefficients)[[11, 12]], -original[i])
        trial = np.array([[12, .7], [18, .8], [8, .4], [6, .3], [6, .3]])
        assert self.native.yi_gains(self.model, vector(trial[:, 0]), vector(trial[:, 1]))
        self.dll.mj_resetData(self.model, self.data)
        self.dll.mj_setState(self.model, self.data, vector(self.center), 2)
        self.dll.mj_setState(self.model, self.data, vector([.1] * 5), 4)
        ctrl = np.column_stack((self.center + .01, [.2] * 5, [.1] * 5)).ravel()
        self.dll.mj_setState(self.model, self.data, vector(ctrl), 64)
        self.dll.mj_forward(self.model, self.data)
        self.native.yi_read(self.model, self.data, self.sample)
        assert np.max(abs(np.array(self.sample)[10:15] -
                          (trial[:, 0] * .01 + trial[:, 1] * .1 + .1))) < 1e-10
        rng = np.random.default_rng(20261123)
        max_g = 0.
        for q in rng.uniform(self.center - .1, self.center + .1, size=(64, 5)):
            g, a = self.model_terms(q)
            max_g = max(max_g, float(np.max(abs(g - ex.support_gravity(q[None])[0]))))
            assert np.isfinite(a).all() and np.all(a > 0)
        assert max_g < 5e-6
        assert encoded(20, 500, 12, symmetric=False) < 20
        return dict(native_PID_gain_update_error_below_nm=1e-10,
                    C_G_max_error_nm=max_g, geometry_mass_unchanged=True)


def directions():
    for axis in range(5):
        for sign in (-1, 1):
            delta = np.zeros(5)
            delta[axis] = sign * 3.
            yield f"J{axis+1}_{sign:+}", delta
    for sign in (-1, 1):
        yield f"coupled_{sign:+}", sign * np.array([3., 3., -3., 3., 3.])


def summarize(cases):
    valid = [c for c in cases if not c["metrics"].get("failed")]
    if len(valid) != len(cases):
        return dict(score=1e9, failed=len(cases) - len(valid))
    groups = {}
    for case in valid:
        key = (case["pose"], case["scenario"], case["case"].rsplit("_", 1)[0])
        groups.setdefault(key, []).append(case["metrics"]["final_error_deg"])
    spans = np.array([np.abs(np.array(v[1]) - v[0]) for v in groups.values() if len(v) == 2])
    errors = np.array([c["metrics"]["final_error_deg"] for c in valid])
    iae = np.mean([c["metrics"]["iae_deg_s"] for c in valid])
    overshoot = max(c["metrics"]["overshoot_deg"] for c in valid)
    tail_speed = max(c["metrics"]["tail_max_speed_rad_s"] for c in valid)
    equilibrium = np.mean([c["metrics"]["settled_to_equilibrium_s"] for c in valid])
    # Explicit engineering ranking, not an inferred user acceptance criterion.
    score = float(2 * spans.max() + np.mean(spans) + np.max(abs(errors))
                  + .15 * iae + .5 * overshoot + .15 * equilibrium + 20 * tail_speed)
    return dict(score=score, failed=0, max_span_deg_by_axis=spans.max(axis=0).tolist(),
                worst_abs_final_error_deg=float(np.max(abs(errors))),
                mean_iae_deg_s=float(iae), max_overshoot_deg=overshoot,
                max_tail_speed_rad_s=tail_speed,
                mean_settled_to_equilibrium_s=float(equilibrium))


def cases_for(bench, profile, poses, scenarios, *, coupled_only=False,
              contacts=False, dt=.002, traces=False):
    cases = []
    for pose_id, pose in enumerate(poses):
        for scenario in scenarios:
            for name, delta in directions():
                if coupled_only and not name.startswith("coupled"):
                    continue
                result = bench.run(profile, pose, delta, scenario,
                                   contacts=contacts, dt=dt, keep_trace=traces)
                cases.append(dict(pose=pose_id, scenario=scenario["name"], case=name,
                                  metrics=result))
                if contacts:
                    print(json.dumps(dict(stage="SDF_case", case=name,
                                          scenario=scenario["name"], failed=result["failed"])),
                          flush=True)
    return cases


def export_candidate(profile, report_path, candidate_path=CANDIDATE):
    root = ET.parse(SOURCE).getroot()
    root.set("model", "Ylamp V1.2 simulation-only tuned impedance candidate")
    for actuator, (kp, kd) in zip(root.findall("./actuator/pid"), profile):
        actuator.set("kp", ex.numbers([encoded(kp, 500, 12, symmetric=False)]))
        actuator.set("kv", ex.numbers([encoded(kd, 5, 12, symmetric=False)]))
    custom = root.find("custom")
    ET.SubElement(custom, "numeric", name="candidate_key_kp_kd", data=ex.numbers(profile))
    schema = custom.find("text[@name='parameter_schema']")
    schema_data = json.loads(schema.get("data"))
    schema_data["g_five_support_gains"] = (
        "archived deployed KEY kp,kd; candidate XML instead uses encoded candidate_key_kp_kd")
    schema.set("data", json.dumps(schema_data, separators=(",", ":")))
    scope = custom.find("text[@name='simulation_scope']")
    scope_data = json.loads(scope.get("data"))
    scope_data["executed"] = [v.replace("encoded native KEY PD", "encoded simulation-only candidate PD")
                              for v in scope_data["executed"]]
    scope.set("data", json.dumps(scope_data, separators=(",", ":")))
    ET.SubElement(custom, "text", name="impedance_candidate_status", data=json.dumps(dict(
        simulation_only=True, deployed=False, report=str(report_path),
        report_sha256=ex.sha(report_path), source_xml_sha256=ex.sha(SOURCE),
        budget_changed=False, added_friction_compensation=False,
        unchanged_original_tables="g_five_* remain the archived deployed b5d5 baseline",
        execution="Native PD is the candidate; standalone XML has fixed saved-pose FF only. "
                  "The tuning harness supplies live G, encoded budget-dependent gain reduction "
                  "and labelled dry-friction/delay surrogate scenarios. Those are not "
                  "automatically executed by loading this XML.",
    ), separators=(",", ":")))
    ET.indent(root, space="  ")
    candidate_path.parent.mkdir(parents=True, exist_ok=True)
    with candidate_path.open("xb") as stream:
        ET.ElementTree(root).write(stream, encoding="utf-8", xml_declaration=True)
    dll, model = ex.load_native(candidate_path)
    dll.mj_deleteModel(model)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true", help="run the finite offline gain search")
    parser.add_argument("--report", type=Path, default=OUT / "tuning_01.json")
    parser.add_argument("--candidate", type=Path, default=CANDIDATE,
                        help="new output XML path; existing candidates are never overwritten")
    args = parser.parse_args()
    if args.run and (args.report.exists() or args.candidate.exists()):
        raise FileExistsError("Keep existing results; select new --report and --candidate paths")
    start = time.monotonic()
    bench = Bench()
    try:
        self_check = bench.self_test()
        print(json.dumps(dict(stage="self_test", result=self_check)), flush=True)
        if not args.run:
            return
        mass = bench.mass(bench.center)
        _, amplitude = bench.model_terms(bench.center)
        kp_reference = np.maximum(bench.baseline[:, 0],
                                  np.ceil(2 * 1.2 * amplitude / math.radians(1)))
        profiles = [("baseline", bench.baseline)]
        for scale in (.75, 1., 1.25):
            for zeta in (.65, .9, 1.15):
                kp = np.round(kp_reference * scale, 1)
                kd = np.minimum(5., np.round(2 * zeta * np.sqrt(kp * np.diag(mass)), 3))
                profiles.append((f"k{scale}_z{zeta}", np.column_stack((kp, kd))))
        training = []
        for name, profile in profiles:
            cases = cases_for(bench, profile, [bench.center], SCENARIOS)
            summary = summarize(cases)
            training.append(dict(name=name, gains=profile.tolist(), summary=summary, cases=cases))
            print(json.dumps(dict(stage="profile", name=name, **summary)), flush=True)
        chosen = min(training[1:], key=lambda p: p["summary"]["score"])
        profile = np.array(chosen["gains"])
        validation_poses = [
            bench.center + np.radians([0, 6, -10, 15, 3]),
            bench.center + np.radians([0, -6, 10, -15, -3]),
        ]
        validation = {}
        collision = {}
        for name, gains in (("baseline", bench.baseline), ("candidate", profile)):
            cases = cases_for(bench, gains, validation_poses, SCENARIOS)
            validation[name] = dict(summary=summarize(cases), cases=cases)
            print(json.dumps(dict(stage="validation", name=name, **validation[name]["summary"])),
                  flush=True)
            # All native SDF contacts active every step for an independent
            # coupled ±3deg pose, under each labelled uncertainty scenario.
            cases = cases_for(bench, gains, validation_poses[:1], SCENARIOS,
                              coupled_only=True, contacts=True, traces=True)
            collision[name] = dict(summary=summarize(cases), cases=cases)
        dt_check = cases_for(bench, profile, validation_poses[1:], SCENARIOS[-1:],
                            coupled_only=True, dt=.001)
        assert chosen["summary"]["failed"] == 0
        assert validation["candidate"]["summary"]["failed"] == 0
        assert collision["candidate"]["summary"]["failed"] == 0
        report = dict(
            schema="ylamp-mujoco-impedance-1", generated_utc=datetime.now(timezone.utc).isoformat(),
            simulation_only=True, deployed=False, source_xml=str(SOURCE),
            source_xml_sha256=ex.sha(SOURCE), source_firmware=ex.inputs()[3]["firmware_sha256"],
            source_sha256={str(p): ex.sha(p) for p in (
                Path(__file__), Path(__file__).with_name("mujoco_impedance_native.c"),
                ex.CONFIG, ex.ROOT / "Model/five_dynamics.c", bench.native_path)},
            baseline_gains=bench.baseline.tolist(), candidate_gains=profile.tolist(),
            unchanged_key_budget_nm=bench.caps.tolist(), mass_matrix_at_training_pose=mass.tolist(),
            sliding_F_amplitude_proxy_nm=amplitude.tolist(), selected_profile=chosen["name"],
            engineering_target="1deg bilateral span used for the search seed; not a user-specified acceptance",
            scenarios=SCENARIOS, self_test=self_check, training=training,
            held_out_validation=validation, native_SDF_coupled_validation=collision,
            timestep_1ms_stress_cases=dt_check, elapsed_seconds=time.monotonic() - start,
            limitations=[
                "Dry thresholds are surrogate multiples of moving F, NOT measured static friction.",
                "Dry proxy replaces unloaded viscous B to avoid double-counting total loss.",
                "Measurement rounding and 4ms delay are scenarios, not identified drive dynamics.",
                "G feedforward remains commissioned, plant mass/COM/M/c remain nominal.",
                "No current-loop, elasticity, backlash, cable or thermal identification.",
                "KEY gain scaling is sampled prediction, not a drive hard torque clamp.",
                "SDF has known thin-wall misses; no real collision/safety acceptance.",
                "Search contacts disabled for speed; only the explicit SDF validation cases "
                "have contact solving active every step.",
                "Candidate XML alone does not execute the live controller or surrogate plant.",
            ])
        args.report.parent.mkdir(parents=True, exist_ok=True)
        with args.report.open("x", encoding="utf-8") as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        export_candidate(profile, args.report, args.candidate)
        print(json.dumps(dict(stage="done", report=str(args.report), candidate=str(args.candidate),
                              gains=profile.tolist(), elapsed_seconds=report["elapsed_seconds"])),
              flush=True)
    finally:
        bench.close()


if __name__ == "__main__":
    main()
