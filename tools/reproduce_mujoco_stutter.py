"""Offline stutter diagnosis using real01 Ruckig and the production C controller.

No COM/SWD, target build, parameter writes or hardware commands.
python -B tools/reproduce_mujoco_stutter.py --output build/v12-stutter-20261125/run01
--quick runs the three causal controls; default adds finite sensitivity cases.
"""
from __future__ import annotations

import argparse
import ctypes as ct
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np

import export_mujoco_identified as ex
from five_trajectory import RuckigTrajectory, DEFAULT_NATIVE, reference_friction

ROOT = Path(__file__).resolve().parents[1]
REAL = ROOT / "build/v12-ruckig-waypoints-20261125/real01"
XML = ROOT / "simulation/ylamp_v12_identified_collision.xml"
SDK = Path("D:/MuJoCo/4.0.1")
GCC = Path("C:/msys64/ucrt64/bin/gcc.exe")
P = ct.POINTER(ct.c_double)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def vector(x):
    return np.ascontiguousarray(x, dtype=np.float64).ctypes.data_as(P)


def build_native(out):
    sources = [
        "tools/mujoco_stutter_native.c", "tools/mujoco_impedance_native.c",
        "Mode/five_mit.c", "Mode/five_support.c", "Model/five_dynamics.c",
        "Model/five_workspace.c", "config/five_mit.c", "config/motor_config.c",
        "Device/DM_Motor/Common/dm_motor.c",
    ]
    library = out / "stutter_native.dll"
    cmd = [str(GCC), "-shared", "-O2", "-std=c11", "-Wall", "-Wextra", "-Werror",
           "-static-libgcc", f"-I{SDK / 'include'}"]
    cmd += [f"-I{ROOT / p}" for p in (
        "Mode", "Model", "config", "Device/DM_Motor", "Device/DM_Motor/Common",
        "BSP/CAN", "tests/fakes")]
    cmd += [str(ROOT / p) for p in sources]
    cmd += [str(SDK / "bin/mujoco.dll"), "-lm", "-Wl,--no-insert-timestamp", "-o", str(library)]
    env = dict(os.environ)
    env["PATH"] = f"{SDK / 'bin'};{GCC.parent};{env.get('PATH', '')}"
    result = subprocess.run(cmd, env=env, text=True, capture_output=True)
    (out / "compile.log").write_text(result.stdout + result.stderr, encoding="utf-8")
    result.check_returncode()
    native = ct.CDLL(str(library))
    for name, args, restype in (
        ("ys_info", [P], None),
        ("ys_setup", [ct.c_void_p, ct.c_double, ct.c_int, ct.c_int], ct.c_int),
        ("ys_control", [P, P, P, P, P, ct.c_double, ct.c_int, ct.c_int, P], ct.c_int),
        ("ys_plant", [ct.c_void_p, ct.c_void_p, ct.c_double, ct.c_int], ct.c_int),
        ("yi_read", [ct.c_void_p, ct.c_void_p, P], None),
    ):
        fn = getattr(native, name)
        fn.argtypes, fn.restype = args, restype
    return native, library, sources


def plateau(t, q, reference, vr, *, gap=.02000001):
    """Same-code source observations, not a continuous-freeze bound."""
    best = dict(seconds=0., start_s=None, end_s=None, target_travel_deg=0., observations=0)
    begin = None
    for k in range(len(t)):
        good = abs(vr[k]) > .04
        continuous = (k > 0 and t[k]-t[k-1] <= gap and q[k] == q[k-1]
                      and good and abs(vr[k-1]) > .04)
        begin = (begin if continuous and begin is not None else k) if good else None
        if begin is not None and t[k]-t[begin] > best["seconds"]:
            best = dict(seconds=float(t[k]-t[begin]), start_s=float(t[begin]),
                        end_s=float(t[k]), target_travel_deg=float(abs(reference[k]-reference[begin])),
                        observations=k-begin+1)
    return best


def real_observations(curve, audit):
    csv = np.loadtxt(REAL / "tracking.csv", delimiter=",", skiprows=1)
    joints = []
    for i in range(5):
        rows = csv[csv[:, 1] == i+1]
        rows = np.array(list({int(r[3]): r for r in rows}.values()))
        rows = rows[(rows[:, 4] >= 6) & (rows[:, 4] <= curve.times[-1])]
        vr = curve.sample(rows[:, 4])[1][:, i]
        measured = plateau(rows[:, 4], rows[:, 6], rows[:, 5], vr)
        old = audit["joints"][i]["longest_observed_same_code_while_abs_vref_gt_004"]
        assert abs(measured["seconds"] - old["seconds"]) < 1e-9
        joints.append(dict(joint=i+1, plateau=measured,
                           rms_deg=float(np.sqrt(np.mean(rows[:, 7]**2)))))
    return joints


def self_test(native, info, q, reference):
    output = np.zeros(40)
    qr, vr, ar = reference
    for extra in (0, 1):
        assert native.ys_control(vector(q), vector(vr), vector(qr), vector(vr),
                                 vector(ar), 1., 1, extra, vector(output))
        assert np.isfinite(output).all()
    assert np.any(abs(output[30:35]) > .001)
    for i in range(5):
        p = plateau(np.arange(6)*.01, np.array([0, 0, 0, 1, 1, 1.]),
                    np.arange(6)*.1, np.ones(6))
        assert abs(p["seconds"]-.02) < 1e-9 and p["observations"] == 3
        assert np.all(info[i, :3] > 0)
    # Reject invalid references rather than silently continuing/clamping a run.
    bad = qr.copy()
    bad[0] = np.nan
    assert not native.ys_control(vector(q), vector(vr), vector(bad), vector(vr),
                                  vector(ar), 1., 1, 0, vector(output))
    np.testing.assert_allclose(output[30:35],reference_friction(output[35:40],vr),
                               atol=2e-6,rtol=1e-6)
    # The causal reference is the already-completed old-controller simulation.
    # Its bridge could add F counterfactually; now the candidate does so itself.
    old_path=ROOT/"build/v12-stutter-20261125/baseline02/stutter_native.dll"
    old=ct.CDLL(str(old_path))
    old.ys_control.argtypes=native.ys_control.argtypes
    old.ys_control.restype=ct.c_int
    baseline=np.zeros(40)
    max_difference=0.
    for enabled in (0,1):
        assert native.ys_control(vector(q),vector(vr),vector(qr),vector(vr),vector(ar),
                                 1.,1,enabled,vector(output))
        assert old.ys_control(vector(q),vector(vr),vector(qr),vector(vr),vector(ar),
                              1.,1,enabled,vector(baseline))
        max_difference=max(max_difference,float(np.max(abs(output-baseline))))
        np.testing.assert_allclose(output,baseline,atol=2e-6,rtol=1e-6)
    return dict(production_C_and_encoder=True, candidate_F_exactly_once=True,
                baseline_bridge_max_difference=max_difference,
                invalid_reference_rejected=True, plateau_self_check=True)


def run_case(native, info, curve, scenario, out, *, dt=.002):
    dll, model = ex.load_native(XML)
    data = dll.mj_makeData(model)
    try:
        assert native.ys_setup(model, dt, scenario.get("viscous", 0),
                               scenario.get("stiff", 0))
        count = round((curve.times[-1]+1.5)/dt)+1
        t = np.arange(count)*dt
        qr, vr, ar = curve.sample(t)
        cad = np.radians(info[:, 4])
        q0 = np.radians(curve.sample(0)[0])+cad
        v0 = curve.sample(0)[1]
        dll.mj_setState(model, data, vector(q0), 2)
        dll.mj_setState(model, data, vector(v0), 4)
        dll.mj_forward(model, data)
        read = np.zeros(18)
        command = np.zeros(40)
        ctrl = np.zeros((5, 3))
        rows, history = [], []
        every = round(.002/dt)
        delay = round(scenario.get("delay_ms", 0)*.001/dt)
        quantized = scenario.get("quantized", 1)
        for k, seconds in enumerate(t):
            native.yi_read(model, data, vector(read))
            q, v = read[:5].copy(), read[5:10].copy()
            if not np.isfinite(read).all() or read[17]:
                raise RuntimeError(f"{scenario['name']}: invalid state/warning at {seconds}")
            raw = q-cad+info[:, 3]
            if np.any(raw < info[:, 8]-.0004) or np.any(raw > info[:, 9]+.0004):
                raise RuntimeError(f"{scenario['name']}: actual simulated travel exceeded")
            history.append((q, v))
            if k % every == 0:
                past_q, past_v = history[max(0, k-delay)]
                ok = native.ys_control(vector(past_q), vector(past_v), vector(qr[k]),
                    vector(vr[k]), vector(ar[k]), min(seconds, curve.times[-1]),
                    quantized, scenario.get("extra_F", 0), vector(command))
                if not ok:
                    raise RuntimeError(f"{scenario['name']}: production command/budget rejected at {seconds}")
                ctrl[:, 2] = command[:5]
                dll.mj_setState(model, data, vector(ctrl.ravel()), 64)
            # Diagnostic encoder stream is current, not the delayed control sample.
            # Float decode matches the drive reader; measurement rounding is assumed.
            code = np.rint((raw/info[:, 0]+1)*.5*65535)
            raw_wire = ((code.astype(np.float32)/np.float32(65535)*2-1)
                        * info[:, 0].astype(np.float32))
            observed = np.degrees(raw_wire-info[:, 3].astype(np.float32))
            true_deg = np.degrees(q-cad)
            rows.append(np.r_[seconds, qr[k], vr[k], true_deg, v, observed,
                              command[:5], command[10:20], command[30:35]])
            if k+1 < count:
                assert native.ys_plant(model, data, scenario["dry_scale"], scenario["matched_G"])
                dll.mj_step(model, data)
        trace = np.array(rows)
        assert abs(read[15]-t[-1]) < 1e-7
        mask = (t >= 6) & (t <= curve.times[-1])
        errors = qr-trace[:, 11:16]
        joints = []
        for i in range(5):
            same = plateau(t[mask], trace[mask, 21+i], qr[mask, i], vr[mask, i])
            slow = (abs(vr[:, i]) > .04) & mask
            joints.append(dict(joint=i+1, rms_deg=float(np.sqrt(np.mean(errors[mask, i]**2))),
                peak_abs_error_deg=float(np.max(abs(errors[mask, i]))), plateau=same,
                near_stall_fraction=float(np.mean(abs(trace[slow, 16+i]) < .005)) if slow.any() else 0.,
                peak_torque_nm=float(np.max(abs(trace[:, 26+i])))))
        columns = ["time_s"]+[f"{name}_J{i+1}" for name in (
            "qref_deg", "vref_rad_s", "true_q_deg", "true_v_rad_s", "encoder_q_deg",
            "wire_torque_nm", "G_nm", "moving_F_amplitude_nm", "added_F_nm") for i in range(5)]
        path = out / f"{scenario['name']}_dt{round(dt*1000)}ms.csv"
        np.savetxt(path, trace, delimiter=",", header=",".join(columns), comments="", fmt="%.10g")
        return dict(scenario=scenario, timestep_s=dt, joints=joints, samples=len(trace),
                    warnings=int(read[17]), contacts_disabled=True,
                    controller_rejections=0, csv=str(path), csv_sha256=sha(path)), trace
    finally:
        dll.mj_deleteData(data)
        dll.mj_deleteModel(model)


def plot(cases, traces, real, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True, layout="constrained")
    colors = ("#757575", "#cc3311", "#0077bb")
    names = ("No dry friction, F off", "Moving F as dry proxy, legacy F off",
             "Same proxy, candidate production F on")
    for j, ax in enumerate(axes):
        i = (0, 1, 2)[j]
        first = next(iter(traces.values()))
        ax.plot(first[:, 0], first[:, 1+i], "k--", lw=1, label="Same Ruckig reference")
        for (key, trace), color, name in zip(traces.items(), colors, names):
            ax.plot(trace[:, 0], trace[:, 11+i], color=color, lw=1.1, label=name)
        ax.set_ylabel(f"J{i+1} software deg")
        ax.grid(alpha=.2)
    axes[-1].set_xlim(6, 16.4)
    axes[-1].set_xlabel("Time from trajectory start (s)")
    axes[0].legend(fontsize=8, ncol=2)
    fig.suptitle("Simulation-only causal controls: G/M/C/production software PD retained\n"
                 "Dry threshold is a sliding-F proxy, NOT identified static friction")
    fig.savefig(out / "comparison.png", dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--quick", action="store_true")
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    start = time.monotonic()
    plan = json.loads((REAL / "plan.json").read_text(encoding="utf-8"))
    audit = json.loads((REAL / "audit_01.json").read_text(encoding="utf-8"))
    for name in ("plan.json", "capture.jsonl", "tracking.csv"):
        assert sha(REAL / name) == audit["sha256"][name]
    scoped_changes={"Mode/five_mit.c","config/five_mit.c","tools/five_trajectory.py",
                    "tools/five_continuous_motion.py","tools/five_happy_motion.py"}
    changed={}
    for name, expected in plan["source_sha256"].items():
        actual=sha(ROOT/name)
        if actual != expected:
            assert name in scoped_changes, f"Unexpected deployed-source change: {name}"
            changed[name]=dict(old_deployed_sha256=expected,current_candidate_sha256=actual)
    assert sha(DEFAULT_NATIVE) == plan["native_library_sha256"]
    commit = audit["actual_commit"]
    curve = RuckigTrajectory(commit["origin_deg"], plan["points"],
        origin_velocity_rad_s=commit["origin_velocity_rad_s"])
    assert curve.knot_ms == commit["knot_ms"] and curve.duration_ms == commit["duration_ms"]
    native, library, sources = build_native(out)
    info = np.zeros((5, 10))
    native.ys_info(vector(info))
    real = real_observations(curve, audit)
    check = self_test(native, info, np.radians(curve.sample(.5)[0]+info[:, 4]), curve.sample(.5))
    print(json.dumps(dict(stage="self_check", result=check)), flush=True)
    scenarios = [
        dict(name="no_dry_matchedG", dry_scale=0., matched_G=1),
        dict(name="dry_matchedG", dry_scale=1., matched_G=1),
        dict(name="dry_matchedG_addedF", dry_scale=1., matched_G=1, extra_F=1),
    ]
    if not args.quick:
        scenarios += [
            dict(name="nominal_viscous", dry_scale=0., matched_G=0, viscous=1),
            dict(name="dry_nominalG", dry_scale=1., matched_G=0),
            dict(name="dry_nominalG_addedF", dry_scale=1., matched_G=0, extra_F=1),
            dict(name="dry_matchedG_idealIO", dry_scale=1., matched_G=1, quantized=0),
            dict(name="dry_matchedG_delay4ms", dry_scale=1., matched_G=1, delay_ms=4),
            dict(name="dry_matchedG_stiff", dry_scale=1., matched_G=1, stiff=1),
        ]
    cases, traces = [], {}
    for scenario in scenarios:
        metrics, trace = run_case(native, info, curve, scenario, out)
        cases.append(metrics)
        if len(traces) < 3:
            traces[scenario["name"]] = trace
        print(json.dumps(dict(stage="case", name=scenario["name"], joints=metrics["joints"])), flush=True)
    if not args.quick:
        for scenario in scenarios[1:3]:
            metrics, _ = run_case(native, info, curve, scenario, out, dt=.001)
            cases.append(metrics)
            print(json.dumps(dict(stage="dt_check", name=scenario["name"], joints=metrics["joints"])), flush=True)
    plot(cases, traces, real, out)
    report = dict(schema="ylamp-stutter-reproduction-2", simulation_only=True,
        hardware_access=False, production_source_F_added=True, candidate_deployed=False,
        generated_utc=datetime.now(timezone.utc).isoformat(), reference_firmware=plan["firmware_sha256"],
        scoped_source_changes_since_reference=changed,
        source_sha256={str(p.relative_to(ROOT)): sha(p) for p in
            [ROOT / name for name in sources]+[Path(__file__), XML, REAL / "plan.json",
                REAL / "capture.jsonl", REAL / "tracking.csv", DEFAULT_NATIVE, library]},
        self_check=check, real_observations=real, current_active_kp_kd_caps=info[:, 5:8].tolist(),
        curve=dict(origin_deg=commit["origin_deg"], origin_velocity_rad_s=commit["origin_velocity_rad_s"],
                   knots_ms=curve.knot_ms, same_reference_for_all_cases=True),
        cases=cases, elapsed_seconds=time.monotonic()-start,
        limitations=[
            "Dry threshold is the adopted sliding-F amplitude, not measured static friction or Stribeck data.",
            "Matched-G scenarios cancel nominal/accepted G mismatch only to isolate friction; both still apply full G.",
            "Dry proxy replaces no-load viscous B; it is never added to a second total-loss model.",
            "Nearest measurement rounding, solver stiffness, 4ms delay and ideal IO are explicit assumptions.",
            "Candidate production C contains one F; legacy causal cases subtract it, never double-add.",
            "Production F wiring changed only after explicit user request; parameters/PI/budgets untouched.",
            "Contact disabled to isolate joint behavior; no collision/environment/cable guarantee.",
            "500Hz sampled software torque and production float32 C/encoding, not native KEY PD.",
            "Real same-code plateaus are source observations, not continuous-freeze bounds or unique causal proof.",
            "No current loop, gear elasticity, backlash, torque scale, temperature or MCU WCET identification.",
        ])
    with (out / "report.json").open("x", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, ensure_ascii=False, allow_nan=False)
    print(f"REPORT: {out / 'report.json'}\nPLOT: {out / 'comparison.png'}", flush=True)


if __name__ == "__main__":
    main()
