"""Continue the offline impedance study with explicitly ASSUMED plant lag.

Never changes firmware or the preceding report/candidate. Torque response time
is a sensitivity knob, not an identification of the motor's actual current loop.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import time

import numpy as np

import tune_mujoco_impedance as tune

REPORT = tune.OUT / "impedance_stress_01.json"
XML = tune.CANDIDATE.with_name("ylamp_v12_impedance_robust_candidate.xml")


def scenarios(lags=(0, 2, 5, 10), delays=(0, 4, 8), friction=(0, 1.2)):
    return [
        dict(name=f"lag{lag}_feedback{delay}_dry{dry}", torque_lag_ms=lag,
             delay_ms=delay, dry_scale=dry)
        for lag in lags for delay in delays for dry in friction]


def evaluate(bench, gains, pose, conditions, amplitudes=(3, 5), *, contacts=False, dt=.002):
    cases = []
    for scenario in conditions:
        for amplitude in amplitudes:
            for sign in (-1, 1):
                delta = sign * amplitude * np.array([1., 1., -1., 1., 1.])
                result = bench.run(gains, pose, delta, scenario, contacts=contacts,
                                   dt=dt, keep_trace=contacts)
                cases.append(dict(pose=0, scenario=scenario["name"],
                                  case=f"a{amplitude}_coupled_{sign:+}", metrics=result))
                if contacts:
                    print(json.dumps(dict(stage="SDF_stress", scenario=scenario["name"],
                                          sign=sign, failed=result["failed"])), flush=True)
    summary = tune.summarize(cases)
    if not summary["failed"]:
        peak = np.max([c["metrics"]["peak_torque_nm"] for c in cases], axis=0)
        summary["peak_torque_nm"] = peak.tolist()
        summary["peak_over_predicted_budget_ratio"] = float(np.max(peak / bench.caps))
        summary["max_contacts"] = max(c["metrics"]["max_contacts"] for c in cases)
        # Prefer smoother response/less predicted-budget overshoot when the
        # uncertain plant gets slower. This is a stated engineering ranking.
        summary["stress_score"] = float(
            summary["score"] + .5 * summary["max_overshoot_deg"]
            + 5 * max(0., summary["peak_over_predicted_budget_ratio"] - 1.05))
    else:
        summary["stress_score"] = 1e9
    return dict(summary=summary, cases=cases)


def zero_lag_regression(bench, old_report):
    archived = tune.OUT / "tune_mujoco_impedance_executed_01.py"
    expected = old_report["source_sha256"][str(Path(tune.__file__))]
    assert tune.ex.sha(archived) == expected
    spec = importlib.util.spec_from_file_location("executed_impedance_01", archived)
    previous = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(previous)
    old = previous.Bench()
    try:
        profile = np.array(old_report["candidate_gains"])
        delta = np.array([3., 3., -3., 3., 3.])
        scenario = dict(name="zero_lag_regression", dry_scale=1.2, delay_ms=4)
        a = old.run(profile, old.center, delta, scenario, keep_trace=True)
        b = bench.run(profile, bench.center, delta, scenario, keep_trace=True)
        error = float(np.max(abs(np.array(a["trace_20ms"]) - b["trace_20ms"])))
        assert error < 1e-12, error
        return dict(execution_snapshot_sha256=expected, zero_lag_trace_max_difference=error)
    finally:
        old.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true", help="execute the finite offline pressure study")
    args = parser.parse_args()
    if args.run and (REPORT.exists() or XML.exists()):
        raise FileExistsError("Keep existing stress evidence; do not overwrite")
    previous_path = tune.OUT / "tuning_01.json"
    old_report = json.loads(previous_path.read_text(encoding="utf-8"))
    start = time.monotonic()
    bench = tune.Bench()
    try:
        regression = zero_lag_regression(bench, old_report)
        print(json.dumps(dict(stage="zero_lag_regression", **regression)), flush=True)
        if not args.run:
            return
        names = ("baseline", old_report["selected_profile"],
                 "k1.0_z0.9", "k1.0_z1.15", "k1.25_z1.15")
        profiles = {r["name"]: np.array(r["gains"]) for r in old_report["training"] if r["name"] in names}
        search_pose = bench.center + np.radians([0, 4, 8, -8, 2])
        holdout_pose = bench.center + np.radians([0, -4, -8, 8, -2])
        conditions = scenarios()
        rows = {}
        for name in names:
            result = evaluate(bench, profiles[name], search_pose, conditions)
            rows[name] = dict(gains=profiles[name].tolist(), **result)
            print(json.dumps(dict(stage="stress_profile", name=name, **result["summary"])), flush=True)
        selected = min(names[1:], key=lambda n: rows[n]["summary"]["stress_score"])
        assert rows[selected]["summary"]["failed"] == 0
        final_names = list(dict.fromkeys((old_report["selected_profile"], selected)))
        holdout, full_contact = {}, {}
        worst = scenarios(lags=(10,), delays=(8,))
        for name in final_names:
            holdout[name] = evaluate(bench, profiles[name], holdout_pose, worst)
            print(json.dumps(dict(stage="stress_holdout", name=name,
                                  **holdout[name]["summary"])), flush=True)
            full_contact[name] = evaluate(bench, profiles[name], holdout_pose, worst,
                                          amplitudes=(5,), contacts=True)
            assert full_contact[name]["summary"]["failed"] == 0
        dt = evaluate(bench, profiles[selected], holdout_pose, worst,
                      amplitudes=(5,), dt=.001)
        assert dt["summary"]["failed"] == 0
        report = dict(
            schema="ylamp-impedance-lag-stress-1", simulation_only=True, deployed=False,
            previous_report=str(previous_path), previous_sha256=tune.ex.sha(previous_path),
            source_xml=str(tune.SOURCE), source_xml_sha256=tune.ex.sha(tune.SOURCE),
            source_sha256={str(p): tune.ex.sha(p) for p in (
                Path(__file__), Path(tune.__file__),
                Path(tune.__file__).with_name("mujoco_impedance_native.c"), bench.native_path)},
            scenarios=conditions, perturbations_deg=[3, 5],
            search_pose_qCAD_rad=search_pose.tolist(), holdout_pose_qCAD_rad=holdout_pose.tolist(),
            zero_lag_regression=regression, profiles=rows, selected_profile=selected,
            candidate_gains=profiles[selected].tolist(), unchanged_key_budget_nm=bench.caps.tolist(),
            held_out_validation=holdout, full_SDF_validation=full_contact,
            timestep_1ms_worst_case=dt, elapsed_seconds=time.monotonic() - start,
            limitations=[
                "0/2/5/10ms first-order torque-response constants are stress assumptions, not identified current-loop dynamics.",
                "The implicit-Euler actuator-lag surrogate initializes at the held deflected torque.",
                "Native torque-lag gains are plant dynamics, not changes to the encoded MCU gain commands.",
                "No new control strategy, friction FF, position integral, drag tuning, budgets or hardware writes.",
                "Dry thresholds are proxies from sliding F; native soft friction may creep.",
                "Screening contacts disabled; only the labelled full-SDF runs solve contacts every step.",
                "SDF misses thin walls; no CAD/CCD or real-hardware safety approval.",
                "Standalone XML has candidate PD/fixed FF only, not this live controller or assumed plant lag.",
            ])
        with REPORT.open("x", encoding="utf-8") as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        tune.export_candidate(profiles[selected], REPORT, XML)
        print(json.dumps(dict(stage="done", selected=selected, gains=profiles[selected].tolist(),
                              report=str(REPORT), xml=str(XML),
                              elapsed_seconds=report["elapsed_seconds"])), flush=True)
    finally:
        bench.close()


if __name__ == "__main__":
    main()
