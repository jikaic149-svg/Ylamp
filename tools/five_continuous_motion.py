"""Precompute five-axis Ruckig through-states; explicit one commit, original exits."""
import argparse
import csv
import json
import math
from pathlib import Path
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np

import five_happy_motion as shared
from five_control import Board, gravity_output_confirmed, idle_output_confirmed
from five_trajectory import DEFAULT_NATIVE, RuckigTrajectory, normalize_points, trajectory_hash, reference_friction
from five_identify import HW, REFS
from five_gravity_residual_report import support_gravity
from five_inertia_identify import nominal_torque
from five_impedance_test import drain_observation
from usb_monitor import serial_port

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BUILD = ROOT/"build/v12-ruckig-waypoints-20261125"
DEFAULT_REFERENCE = ROOT/"build/v12-happy-motion-20261124/real04/preview_reference.csv"


def make_points(reference, origin_deg, time_scale=2., prepare_ms=6000):
    if not math.isfinite(time_scale) or time_scale <= 0 or prepare_ms < 100:
        raise ValueError("Invalid finite trajectory timing")
    data = np.loadtxt(reference, delimiter=",", skiprows=1)
    if (data.ndim != 2 or data.shape[1] != 16 or not np.isfinite(data).all() or
            np.any(np.diff(data[:,0]) <= 0) or data[0,0] != 0):
        raise ValueError("Expected monotonic analytic CAD q/v/a reference")
    origin = np.asarray(origin_deg, float)
    shift = np.zeros(5)
    shift[[0,3]] = (origin+HW[:,6]-data[0,1:6])[[0,3]]
    # A sparse set of meaningful q/v/a states, not zero-speed Move commands.
    # Ruckig owns the path/time between them; sample the SAME profiles at 2ms.
    # 0.4s source spacing -> 14 states fits the fixed 16-profile MCU budget.
    selected = np.unique(np.r_[np.arange(0,len(data),200),len(data)-1])
    points = []
    for index in selected:
        row = data[index]
        points.append(dict(at_ms=prepare_ms+round(row[0]*time_scale*1000),
            position_deg=(row[1:6]+shift-HW[:,6]).tolist(),
            velocity_rad_s=(row[6:11]/time_scale).tolist(),
            acceleration_rad_s2=(row[11:16]/time_scale**2).tolist()))
    return normalize_points(points)


def check_curve(origin, points, geometry, velocity=None):
    curve = RuckigTrajectory(origin,points,origin_velocity_rad_s=velocity)
    times = np.arange(0,curve.duration_ms+1,2)*.001
    q,v,a = curve.sample(times)
    velocity_ceiling=np.maximum(curve.limits[:,0],
        np.zeros(5) if velocity is None else np.abs(velocity))
    sampled_jerk=np.max(abs(np.diff(a,axis=0)),axis=0)/.002
    if (np.any(np.max(abs(v),axis=0)>velocity_ceiling+1e-5) or
            np.any(np.max(abs(a),axis=0)>curve.limits[:,1]+1e-5) or
            np.any(sampled_jerk>curve.limits[:,2]+1e-3)):
        raise ValueError("Ruckig sample/limit consistency failed; no commit")
    # Keep actual continuous J1/J4 branches. Finite CAD samples, NOT a
    # continuous collision certificate; all expensive work is BEFORE commit.
    indices = [0]
    for i in range(1,len(q)):
        if np.max(abs(q[i]-q[indices[-1]])) >= 2.5:
            indices.append(i)
    if indices[-1] != len(q)-1:
        indices.append(len(q)-1)
    paths = [[(q[i]+HW[:,6]).tolist(),(q[j]+HW[:,6]).tolist()]
             for i,j in zip(indices,indices[1:])]
    geometric = geometry.check(paths)
    cad = np.deg2rad(q+HW[:,6]);zero = np.zeros_like(q)
    g = support_gravity(cad,clip=False)
    friction = reference_friction(cad,v)
    demand = nominal_torque(cad,v,a)-nominal_torque(cad,zero,zero)+g+friction
    peak = np.max(abs(demand),axis=0)
    if np.any(np.max(abs(g),axis=0)>HW[:,5]) or np.any(peak>shared.CAPS):
        raise ValueError("Continuous nominal G/M/C/F exceeds unchanged budgets; no commit")
    return dict(geometry=geometric,samples=len(times),duration_ms=curve.duration_ms,
        actual_knot_ms=curve.knot_ms,limits_rad_si=curve.limits.tolist(),
        nominal_zero_feedback_peak_nm=peak.tolist(),
        reference_friction_included=True,
        peak_reference_friction_nm=np.max(abs(friction),axis=0).tolist(),
        peak_velocity_rad_s=np.max(abs(v),axis=0).tolist(),
        peak_acceleration_rad_s2=np.max(abs(a),axis=0).tolist(),
        sampled_peak_jerk_rad_s3=sampled_jerk.tolist(),
        assumed_zero_tracking_error=True,continuous_collision_proof=False)


def prepare(directory, reference, origin, time_scale):
    points = make_points(reference,origin,time_scale)
    with shared.Geometry() as geometry:
        check = check_curve(origin,points,geometry)
    directory.mkdir(parents=True,exist_ok=False)
    plan = dict(points=points,origin_deg=list(origin),prepare_ms=6000,
        reference_sha256=shared.sha(reference),time_scale=time_scale,
        native_library_sha256=shared.sha(DEFAULT_NATIVE),
        firmware_sha256=shared.sha(DEFAULT_BUILD/"ylamp_v12_five_axis.bin"),
        checksum=trajectory_hash(points),check=check,simulation_only=True,
        source_sha256={name:shared.sha(ROOT/name) for name in (
            "tools/five_continuous_motion.py","tools/five_trajectory.py",
            "tools/five_control.py","tools/five_happy_motion.py","Mode/five_trajectory.cpp",
            "Mode/five_trajectory.h","tools/five_ruckig_native.cpp","config/five_trajectory.c",
            "Model/five_workspace.c","Model/five_workspace.h","Mode/five_mit.c",
            "Task/five_control.c","Task/motor_usb.c","config/five_mit.c")})
    with (directory/"plan.json").open("x",encoding="utf-8") as stream:
        json.dump(plan,stream,indent=2,allow_nan=False)
    print(json.dumps(dict(points=len(points),duration_ms=check["duration_ms"],
                          check=check,hardware_motion_sent=False),indent=2),flush=True)


def execute(directory):
    plan = json.loads((directory/"plan.json").read_text())
    if plan["firmware_sha256"] != shared.EXPECTED:
        raise ValueError("Prepared image mismatch; no motion")
    display = shared.LiveDisplay()
    records, pending = [], []
    active = None
    last_ui = 0.
    with (directory/"capture.jsonl").open("x",encoding="utf-8") as log, \
            (directory/"tracking.csv").open("x",newline="") as trace:
        writer = csv.writer(trace)
        writer.writerow(["tick_ms","joint","source_tick_ms","received_count",
                         "elapsed_s","target_deg","actual_deg","error_deg"])
        def note(**row):
            print(json.dumps(row,separators=(",",":"),allow_nan=False),file=log)
        def consume(sample):
            nonlocal last_ui
            if active is None:
                pending.append(sample)
                return
            target,actual,error,velocity,torque = [],[],[],[],[]
            source_times = np.array([((axis["received_tick_ms"]-active["start_ms"]) &
                                     0xffffffff)*.001 for axis in sample["joints"]])
            if np.any(source_times > 86400):
                return
            goals = curve.sample(source_times)[0]
            for i,axis in enumerate(sample["joints"]):
                elapsed = float(source_times[i])
                goal = goals[i,i]
                # Select only the measured branch adjacent to the preserved
                # origin; the curve never folds a multi-turn joint.
                raw = math.degrees(axis["position_rad"])
                raw += 360*round((reference_branch[i]-raw)/360)
                reference_branch[i] = raw
                actual_deg = raw-math.degrees(REFS[i,0])
                delta = float(goal-actual_deg)
                writer.writerow([sample["tick_ms"],i+1,axis["received_tick_ms"],
                                 axis["received_count"],elapsed,goal,actual_deg,delta])
                records.append((i,axis["received_count"],elapsed,delta))
                target.append(float(goal));actual.append(actual_deg);error.append(delta)
                velocity.append(axis["velocity_rad_s"]);torque.append(axis["torque_nm"])
            if time.monotonic()-last_ui >= .15:
                last_ui = time.monotonic()
                display.update("Ruckig 连续主动轨迹",target,actual,error,velocity,torque)
                log.flush();trace.flush()
        def feedback(sample):
            note(stage="feedback",telemetry=sample)
            if committing:
                consume(sample)
        # CAD loading is outside USB ownership; its finite path work runs in
        # the worker while the one USB owner continues draining all telemetry.
        with shared.Geometry() as geometry, serial_port("COM36") as port, \
                ThreadPoolExecutor(max_workers=1) as pool:
            board = Board(port);board.connect()
            refs = [[board.scalar(field,joint,op=15) for field in (1,2,3)]
                    for joint in range(1,6)]
            if not np.allclose(refs,REFS,rtol=0,atol=2e-6):
                raise ValueError("Live calibration differs; no motion")
            initial = board.status()
            if initial["phase"] != "DRAG" or not idle_output_confirmed(initial):
                raise ValueError("Healthy existing drag absent; no takeover or recovery")
            history = board.history()
            origin = np.array([j["reference_deg"] for j in history["joints"]])-np.rad2deg(REFS[:,0])
            reference_branch = origin+np.rad2deg(REFS[:,0])
            committing = False
            board.on_feedback = feedback
            note(stage="capture",schema="ylamp-continuous-tracking-1",
                 firmware_sha256=shared.EXPECTED,plan_sha256=shared.sha(directory/"plan.json"),
                 references_rad=refs,initial=initial,operator_released=True,
                 free_space=False,exact_CAN_command_or_atomic_components=False)
            owned,success,failure = False,False,None
            try:
                job = pool.submit(check_curve,origin,plan["points"],geometry,
                                  [axis["velocity_rad_s"] for axis in initial["telemetry"]["joints"]])
                while not job.done():
                    board.pump();display.pump()
                    if display.stop:
                        raise RuntimeError("Cancelled before upload")
                note(stage="precommit_check",**job.result())
                drain_observation(board)
                points = board.trajectory_upload(plan["points"])
                note(stage="upload_verified",count=len(points),checksum=trajectory_hash(points))
                drain_observation(board)
                owned = committing = True  # A lost commit ACK may still mean motion.
                active = board.trajectory_start()
                curve = RuckigTrajectory(active["origin_deg"],points,
                                        origin_velocity_rad_s=active["origin_velocity_rad_s"])
                if (active["checksum"] != plan["checksum"] or
                        active["duration_ms"] != curve.duration_ms or
                        active["knot_ms"] != curve.knot_ms):
                    raise RuntimeError("Committed plan binding differs; no retry")
                note(stage="committed",**active)
                for sample in pending:
                    consume(sample)
                pending.clear()
                next_status = 0.
                deadline = time.monotonic()+active["duration_ms"]*.001+4.5
                while True:
                    board.pump();display.pump()
                    if display.stop:
                        raise RuntimeError("Operator cancelled continuous motion")
                    if time.monotonic() >= next_status:
                        status = board.status();note(stage="status",**status)
                        if (status["fault"] or status["limited"] or
                                any(status[k]!=31 for k in ("fresh","enabled","submitted")) or
                                status["phase"] not in ("MOTION","POSITION")):
                            raise RuntimeError("Continuous motion interrupted; no recovery")
                        tick = status["tick_ms"] if "tick_ms" in status else status["telemetry"]["tick_ms"]
                        elapsed = (tick-active["start_ms"]) & 0xffffffff
                        if active["duration_ms"]+1500 <= elapsed < 0x80000000 and status["phase"]=="POSITION":
                            success = True
                            break
                        next_status = time.monotonic()+.1
                    if time.monotonic() > deadline:
                        raise TimeoutError("Continuous completion unconfirmed; no replay")
            except BaseException as exc:
                failure = str(exc);note(stage="abort",error=failure)
            finally:
                committing = False
                if owned:
                    try:
                        board.rpc(7)
                        handover = board.observe_transition(("MOTION","POSITION","DRAG"))
                        note(stage="gravity_handover",**handover)
                        if not gravity_output_confirmed(handover):
                            raise RuntimeError("G handover unconfirmed")
                        board.drag()
                        final = board.observe_transition(("GRAVITY",))
                        note(stage="manual_handover",**final)
                        if final["phase"]!="DRAG" or not idle_output_confirmed(final):
                            raise RuntimeError("Drag handover unconfirmed")
                    except BaseException as exc:
                        success = False;note(stage="handover_unconfirmed",error=str(exc))
                note(stage="end",success=success,malformed=board.stream.malformed,
                     incomplete=board.collector.incomplete,gaps=board.collector.gaps)
        joints = []
        for i in range(5):
            unique = {count:(elapsed,error) for joint,count,elapsed,error in records
                      if joint==i and elapsed>=curve.knot_ms[0]*.001}
            errors = np.array([value[1] for value in unique.values()])
            held = [error for elapsed,error in unique.values() if active and
                    active["duration_ms"]*.001+1<=elapsed<=active["duration_ms"]*.001+1.5]
            joints.append(dict(joint=i+1,samples=len(errors),
                rms_error_deg=float(np.sqrt(np.mean(errors**2))) if len(errors) else None,
                max_absolute_error_deg=float(np.max(abs(errors))) if len(errors) else None,
                final_hold_mean_error_deg=float(np.mean(held)) if held else None))
        with (directory/"results.json").open("x",encoding="utf-8") as stream:
            json.dump(dict(success=success,failure=failure,joints=joints,
                atomic_command_claimed=False,no_parameter_changes=True),stream,indent=2,allow_nan=False)
    display.title.set("连续轨迹结束；USB已关闭，参数未修改")
    display.button.configure(text="关闭结果窗口",command=display.root.destroy)
    display.root.protocol("WM_DELETE_WINDOW",display.root.destroy)
    display.root.mainloop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory",type=Path,required=True)
    parser.add_argument("--reference",type=Path,default=DEFAULT_REFERENCE)
    parser.add_argument("--origin-deg",type=float,nargs=5)
    parser.add_argument("--time-scale",type=float,default=2.)
    parser.add_argument("--execute",action="store_true")
    parser.add_argument("--released",action="store_true")
    args = parser.parse_args()
    directory = args.directory.resolve()
    if not args.execute:
        if args.origin_deg is None:
            parser.error("Offline preparation requires five --origin-deg values")
        prepare(directory,args.reference,args.origin_deg,args.time_scale)
        return
    if not args.released:
        parser.error("--execute requires this run's --released confirmation")
    if any((directory/name).exists() for name in ("capture.jsonl","results.json","before_execute")):
        raise FileExistsError("Already attempted; never replay this directory")
    plan = json.loads((directory/"plan.json").read_text())
    for name,digest in plan["source_sha256"].items():
        if shared.sha(ROOT/name) != digest:
            raise ValueError("Prepared source changed; regenerate offline")
    if shared.sha(DEFAULT_NATIVE)!=plan["native_library_sha256"]:
        raise ValueError("Prepared native Ruckig library changed; regenerate offline")
    shared.FIRMWARE = DEFAULT_BUILD
    shared.EXPECTED = shared.sha(DEFAULT_BUILD/"ylamp_v12_five_axis.bin")
    shared.STATE_BINDING = DEFAULT_BUILD/"after_flash_state_01.json"
    preflight = directory/"before_execute";preflight.mkdir()
    shared.preflight(preflight)
    execute(directory)


if __name__ == "__main__":
    main()
