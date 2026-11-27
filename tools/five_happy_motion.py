"""7B32 finite happy motion and tracking. Never enables, resets, clears or tunes."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import re
import struct
import subprocess
import math
import os
import sys
import shlex
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np

from five_friction_campaign import GEOMETRY
from five_gravity_residual_report import support_gravity
from five_identify import HW, REFS
from five_inertia_identify import nominal_torque
from five_trajectory import reference_friction
from five_control import Board, idle_output_confirmed, gravity_output_confirmed
from usb_monitor import serial_port
from five_impedance_test import drain_observation

ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "build/v12-active-kp-isolated-20261124"
EXPECTED = "7b32d0a9b421aec9c380dc5ed2c601363365dd920f53a3b8f426e65e74db7111"
STATE_BINDING = FIRMWARE / "after_j3_restore_read_01/after_flash_state_01.json"
OPENOCD = Path("D:/OpenOCD/openocd-custom-20260922")
PREVIEW = ROOT / "build/v12-happy-preview-20261124/v2_amplitude2x_01"
CAPS = np.array([4., 10., 3.6, 1.6, .8])


def quintic(start, end, seconds, elapsed):
    """Same rest-to-rest q/v/a as the existing ordinary MCU Move, no Speed PI."""
    t = np.clip(np.asarray(elapsed, dtype=float)/seconds, 0, 1)
    s = t*t*t*(10+t*(-15+6*t))
    ds = 30*t*t*(1-t)*(1-t)/seconds
    dds = 60*t*(1-t)*(1-2*t)/seconds**2
    delta = np.asarray(end)-start
    return (np.asarray(start)+s[..., None]*delta,
            ds[..., None]*np.deg2rad(delta), dds[..., None]*np.deg2rad(delta))


class Geometry:
    def __init__(self):
        self.process = subprocess.Popen(
            ["node", "--input-type=module", "-e", GEOMETRY],
            cwd=ROOT/"tools/urdf_collision", stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, text=True, encoding="utf-8")
        while True:
            line = self.process.stdout.readline()
            if not line:
                raise RuntimeError("Existing CAD geometry loader failed")
            if line.startswith("{"):
                self.source = json.loads(line)
                break

    def check(self, paths):
        self.process.stdin.write(json.dumps(paths)+"\n")
        self.process.stdin.flush()
        row = self.process.stdout.readline()
        if not row:
            raise RuntimeError("Existing CAD checker exited")
        result = json.loads(row)
        if result["witness"]:
            raise ValueError(f"CAD intersection witness; path not sent: {result['witness']}")
        return result

    def close(self):
        self.process.stdin.close()
        self.process.wait(timeout=10)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def check_leg(start, target, milliseconds):
    """Existing travel/effort checks, not a new runtime error lock or rating."""
    low, high = np.rad2deg(REFS[:, 1:]).T-np.rad2deg(REFS[:, 0])
    if np.any(target < low+1) or np.any(target > high-1):
        raise ValueError("Target outside existing one-degree inset")
    if np.any(start < low-.022) or np.any(start > high+.022):
        raise ValueError("Observed start outside original travel")
    seconds = milliseconds*.001
    q, v, a = quintic(start, target, seconds, np.linspace(0, seconds, 501))
    q = np.deg2rad(q+HW[:, 6])
    zero = np.zeros_like(q)
    gravity = support_gravity(q, clip=False)
    friction = reference_friction(q,v)
    demand = nominal_torque(q, v, a)-nominal_torque(q, zero, zero)+gravity+friction
    peak, gpeak = np.max(abs(demand), axis=0), np.max(abs(gravity), axis=0)
    if np.any(gpeak > HW[:, 5]) or np.any(peak > CAPS):
        raise ValueError("Nominal G/trajectory demand exceeds retained budget")
    return dict(nominal_zero_feedback_peak_nm=peak.tolist(),
                reference_friction_included=True,
                peak_reference_friction_nm=np.max(abs(friction),axis=0).tolist(),
                unclipped_gravity_peak_nm=gpeak.tolist(),
                peak_reference_velocity_rad_s=np.max(abs(v), axis=0).tolist(),
                assumed_zero_tracking_error=True, actual_torque_not_guaranteed=True)


def make_plan(directory):
    report = json.loads((directory/"preflight.json").read_text())
    live = report["snapshots"][-1]
    if (live["phase"] != 7 or live["fault"] or live["limited"] or live["moving"] or
            live["test_joint"] or any(live[k] != 31 for k in ("fresh", "enabled", "submitted"))):
        raise ValueError("Current snapshot is not healthy existing DRAG; no recovery")
    # Compile only host simulation/pure model sources. Never builds/flashes MCU.
    sdk = Path("D:/MuJoCo/4.0.1")
    compiler = Path("C:/msys64/ucrt64/bin/gcc.exe")
    executable = directory/"reference_exporter.exe"
    cmd = [str(compiler), "-O2", "-std=c11", "-Wall", "-Wextra", "-static-libgcc",
           f"-I{sdk/'include'}"]
    cmd += [f"-I{ROOT/p}" for p in (
        "Model", "Mode", "config", "Device/DM_Motor/Common", "BSP/CAN", "tests/fakes")]
    cmd += [str(ROOT/p) for p in ("tools/mujoco_happy_preview.c",
        "tools/mujoco_impedance_native.c", "Model/five_dynamics.c", "config/five_mit.c")]
    cmd += [str(sdk/"bin/mujoco.dll"), "-lopengl32", "-lgdi32", "-luser32", "-lm",
            "-Wl,--no-insert-timestamp", "-o", str(executable)]
    env = os.environ.copy()
    env["PATH"] = f"{sdk/'bin'};{compiler.parent};{env.get('PATH', '')}"
    compiled = subprocess.run(cmd, env=env, capture_output=True, text=True)
    (directory/"reference_compile.log").write_text(compiled.stdout+compiled.stderr)
    compiled.check_returncode()
    reference_file = directory/"preview_reference.csv"
    subprocess.run([str(executable), "--reference", str(PREVIEW/"ylamp_happy_v2.xml"),
                    str(reference_file)], env=env, check=True)
    source = np.loadtxt(reference_file, delimiter=",", skiprows=1)
    anchor = np.rad2deg(live["reference_rad"])-np.rad2deg(REFS[:, 0])
    # Keep actual continuous yaw/wrist branch; do not move to the camera yaw.
    shift = np.zeros(5)
    shift[[0, 3]] = (anchor+HW[:, 6]-source[0, 1:6])[[0, 3]]
    times = (0., .9, 1.15, 1.82, 2.19, 2.60, 2.83, 3.27, 4.10)
    names = ("prepare", "look", "gaze_pause", "perk", "response_1",
             "return_1", "response_2", "return_2", "relax")
    current, legs = anchor.copy(), []
    for i, (t, name) in enumerate(zip(times, names)):
        cad = np.array([np.interp(t, source[:, 0], source[:, 1+j]) for j in range(5)])+shift
        target = (cad-HW[:, 6]).astype(np.float32).astype(float)
        seconds = (max(6., 1.875*np.max(abs(np.deg2rad(target-current)))/.12)
                   if i == 0 else max(.8, 2*(t-times[i-1])))
        milliseconds = math.ceil(seconds*1000)
        checked = check_leg(current, target, milliseconds)
        legs.append(dict(name=name, preview_time_s=t, target_software_deg=target.tolist(),
                         duration_ms=milliseconds, hold_ms=200 if i<len(times)-1 else 1500,
                         nominal_start_software_deg=current.tolist(), **checked))
        current = target
    paths = [[(np.asarray(leg["nominal_start_software_deg"])+HW[:, 6]).tolist(),
              (np.asarray(leg["target_software_deg"])+HW[:, 6]).tolist()] for leg in legs]
    geometry = Geometry()
    try:
        geometric = geometry.check(paths)
    finally:
        geometry.close()
    result = dict(firmware_sha256=EXPECTED, action="finite slow segmented approximation",
        reference_source_sha256=sha(PREVIEW/"ylamp_happy_v2.xml"),
        reference_csv_sha256=sha(reference_file), source_sha256={
            p:sha(ROOT/p) for p in ("tools/five_happy_motion.py",
                "tools/mujoco_happy_preview.c", "config/five_mit.c",
                "Mode/five_mit.c", "Task/five_control.c", "config/joint_references.def")},
        actual_anchor_software_deg=anchor.tolist(), preserved_CAD_shift_deg=shift.tolist(),
        legs=legs, geometry=geometric, geometry_source=geometry.source,
        controller="existing MCU ordinary Move; drive PD zero; accepted G + nominal M*a+C + software PD",
        not_exact_continuous_preview=True, new_firmware_or_parameters=False,
        free_space_confirmed=False, execution_permitted_by_plan_only=False)
    with (directory/"plan.json").open("x", encoding="utf-8") as out:
        json.dump(result, out, indent=2, allow_nan=False)
    with (directory/"planned_trajectory.csv").open("x", newline="") as stream:
        out = csv.writer(stream)
        out.writerow(["sequence_time_s", "segment"]+[f"target_J{j}_software_deg" for j in range(1,6)]
                     +[f"velocity_J{j}_rad_s" for j in range(1,6)]
                     +[f"accel_J{j}_rad_s2" for j in range(1,6)])
        offset = 0.
        for leg in legs:
            seconds = leg["duration_ms"]*.001
            ts = np.linspace(0, seconds, math.ceil(seconds/.002)+1)
            q, v, a = quintic(np.array(leg["nominal_start_software_deg"]),
                             leg["target_software_deg"], seconds, ts)
            for t, qq, vv, aa in zip(ts,q,v,a):
                out.writerow([offset+t,leg["name"],*qq,*vv,*aa])
            offset += seconds+leg["hold_ms"]*.001
    print(json.dumps(dict(segments=len(legs), planned_seconds=round(offset,3),
        first_target_software_deg=legs[0]["target_software_deg"],
        geometry=geometric, hardware_motion_sent=False),indent=2),flush=True)
    return result


def motion_layout():
    """Ask the actual ARM compiler for the current 7B32 private layout."""
    commands = subprocess.check_output(
        ["D:/Ninja/ninja.exe", "-C", str(FIRMWARE), "-t", "commands", "ylamp_task"], text=True)
    command = next(line for line in commands.splitlines()
                   if re.search(r" -c .*[/\\]Task[/\\]five_control\.c$", line))
    arguments, flags, i = shlex.split(command), [], 0
    while i < len(arguments):
        word = arguments[i]
        if word in ("-MT", "-MF", "-o", "-c"):
            i += 2
        elif word in ("-MD", "-fstack-usage"):
            i += 1
        else:
            flags.append(word)
            i += 1
    source = (f'#include "{(ROOT/"Task/five_control.c").as_posix()}"\n'
              '_Static_assert(sizeof(c)==560,"7B32 c");\n'
              '_Static_assert(sizeof(c.plan)==96,"7B32 plan");\n'
              'const unsigned motion_layout[]={__builtin_offsetof(__typeof__(c),plan),'
              '__builtin_offsetof(__typeof__(c),start),'
              '__builtin_offsetof(__typeof__(c),duration)};\n')
    assembly = subprocess.check_output([*flags, "-S", "-x", "c", "-o", "-", "-"],
                                       input=source, text=True, timeout=30)
    fields = re.findall(r"^\s*\.word\s+(\d+)\s*$",
                        assembly.split("\nmotion_layout:\n", 1)[1], re.M)
    if len(fields) != 3:
        raise ValueError("ARM plan layout incomplete")
    return dict(zip(("plan", "start", "duration"), map(int, fields)))


def read_motion(base, layout):
    """Ordinary non-halting SWD; USB is pumped independently during this read."""
    commands = [
        "gdb port disabled", "tcl port disabled", "telnet port disabled",
        "adapter speed 1000", "reset_config none",
        "stm32h7x.cpu0 configure -event examine-end {} -work-area-size 0",
        "init", "targets stm32h7x.cpu0", "poll",
        'set u [read_memory 0x1ff1e800 32 3]; '
        'if {[lindex $u 0]!=0x420030 || [lindex $u 1]!=0x31345106 || '
        '[lindex $u 2]!=0x38373332} {error WRONG_BOARD}; echo UID_OK',
        'set d [lindex [read_memory 0xe000edf0 32 1] 0]; '
        'if {$d & 0x20000} {error CORE_HALTED}; echo DHCSR_BEFORE_$d',
        f"echo PLAN_[read_memory {base+layout['plan']:#x} 32 24]",
        f"echo START_[read_memory {base+layout['start']:#x} 32 1]",
        f"echo DURATION_[read_memory {base+layout['duration']:#x} 32 1]",
        'set d [lindex [read_memory 0xe000edf0 32 1] 0]; '
        'if {$d & 0x20000} {error CORE_HALTED}; echo DHCSR_AFTER_$d',
        "echo FINAL_STATE_[stm32h7x.cpu0 curstate]", "shutdown",
    ]
    command = [str(OPENOCD/"bin/openocd.exe"), "-s", str(OPENOCD/"share/openocd/scripts"),
               "-f", "interface/stlink.cfg", "-f", "target/stm32h7x.cfg"]
    for line in commands:
        command += ["-c", line]
    result = subprocess.run(command, capture_output=True, text=True, timeout=15)
    text = result.stdout+result.stderr
    if result.returncode or "UID_OK" not in text or "FINAL_STATE_running" not in text:
        raise RuntimeError(f"Non-halting plan read failed; no retry:\n{text}")
    words = re.search(r"^PLAN_(.*)$", text, re.M)[1].split()
    values = struct.unpack("<12d", struct.pack("<24I", *(int(v,16) for v in words)))
    if len(words) != 24 or not all(map(math.isfinite, values)):
        raise ValueError("Invalid bound live plan")
    return dict(start_software_deg=list(values[:5]), target_software_deg=list(values[5:10]),
                seconds=values[10], cruise_ramp_s=values[11],
                start_tick_ms=int(re.search(r"^START_(0x[0-9a-f]+)", text, re.M)[1],16),
                duration_ms=int(re.search(r"^DURATION_(0x[0-9a-f]+)", text, re.M)[1],16),
                core_running=True, atomic_feedback_or_CAN_command_claimed=False,
                probe_output=text)


class LiveDisplay:
    """A single display/stop button; the existing Board remains sole USB owner."""
    def __init__(self):
        import tkinter as tk
        self.root = tk.Tk()
        self.root.title("Ylamp 实机｜目标 / 实际 / 误差（软件角 °）")
        self.stop = False
        self.title = tk.StringVar(value="准备中：参数不改，只播放一次")
        tk.Label(self.root, textvariable=self.title, font=("Microsoft YaHei UI",13)).grid(
            row=0,column=0,columnspan=6,padx=14,pady=12)
        for c, text in enumerate(("关节","目标 °","实际 °","误差 °","速度 rad/s","电流估矩 Nm")):
            tk.Label(self.root,text=text).grid(row=1,column=c,padx=12,pady=5)
        self.cells = [[tk.StringVar(value="—") for _ in range(5)] for _ in range(5)]
        for j in range(5):
            tk.Label(self.root,text=f"J{j+1}").grid(row=j+2,column=0,pady=7)
            for c, value in enumerate(self.cells[j]):
                tk.Label(self.root,textvariable=value,font=("Consolas",13)).grid(
                    row=j+2,column=c+1,padx=12,pady=7)
        self.button = tk.Button(self.root,text="取消轨迹，保留重力支撑",command=self.cancel)
        self.button.grid(row=7,column=0,columnspan=6,pady=12)
        self.root.protocol("WM_DELETE_WINDOW",self.cancel)
        self.root.update()

    def cancel(self):
        self.stop = True

    def update(self, tag, target, actual, error, velocity, torque):
        self.title.set(f"实机：{tag}｜目标按已核验的板端轨迹计算")
        for j in range(5):
            for cell, number in zip(self.cells[j],
                                   (target[j],actual[j],error[j],velocity[j],torque[j])):
                cell.set(f"{number:+.3f}")
        self.pump()

    def pump(self):
        self.root.update_idletasks()
        self.root.update()


def execute(directory):
    """One finite nine-segment run. No tuning, enable, ACK, reset or retry."""
    plan = json.loads((directory/"plan.json").read_text())
    if plan["firmware_sha256"] != EXPECTED:
        raise ValueError("Prepared plan belongs to another firmware; no motion")
    layout = motion_layout()
    binding = json.loads(STATE_BINDING.read_text())
    display = LiveDisplay()
    records, segment_results = [], []
    state = dict(active=None, tag="before", pending=[], last_ref=None, last_ui=0.)
    with (directory/"capture.jsonl").open("x",encoding="utf-8") as journal, \
            (directory/"tracking.csv").open("x",newline="") as trace:
        writer = csv.writer(trace)
        writer.writerow(["segment","snapshot_tick_ms"]+
            [f"{key}_J{j}" for key in ("source_tick_ms","target_software_deg","actual_software_deg",
                "error_deg","velocity_rad_s","torque_est_nm","received_count") for j in range(1,6)])

        def note(**record):
            print(json.dumps(record,allow_nan=False,separators=(",",":")),file=journal)

        def consume(sample, actual):
            active = state["active"]
            elapsed = np.array([((j["received_tick_ms"]-active["start_tick_ms"]) & 0xffffffff)*.001
                                for j in sample["joints"]])
            if np.any(elapsed > 86400):  # pre-commit source ticks, never retime them
                return
            target = np.array([quintic(np.array(active["start_software_deg"]),
                active["target_software_deg"],active["seconds"],t)[0][j]
                for j,t in enumerate(elapsed)])
            error = target-actual
            velocity = np.array([j["velocity_rad_s"] for j in sample["joints"]])
            torque = np.array([j["torque_nm"] for j in sample["joints"]])
            counts = [j["received_count"] for j in sample["joints"]]
            row = dict(segment=state["tag"],tick_ms=sample["tick_ms"],elapsed_s=elapsed.tolist(),
                       target=target.tolist(),actual=actual.tolist(),error=error.tolist(),
                       velocity=velocity.tolist(),torque=torque.tolist(),counts=counts)
            records.append(row)
            writer.writerow([state["tag"],sample["tick_ms"],
                *[j["received_tick_ms"] for j in sample["joints"]],
                *target,*actual,*error,*velocity,*torque,*counts])
            if time.monotonic()-state["last_ui"] >= .15:
                state["last_ui"] = time.monotonic()
                display.update(state["tag"],target,actual,error,velocity,torque)
                print(f"LIVE {state['tag']}: target={np.round(target,2).tolist()} "
                      f"actual={np.round(actual,2).tolist()} error={np.round(error,2).tolist()} deg",
                      flush=True)
                journal.flush()
                trace.flush()

        def feedback(sample):
            note(stage="feedback",segment=state["tag"],telemetry=sample)
            raw = np.rad2deg([j["position_rad"] for j in sample["joints"]])
            ref = raw+360*np.round((state["last_ref"]-raw)/360)
            state["last_ref"] = ref
            actual = ref-np.rad2deg(REFS[:,0])
            if state["active"] is None:
                state["pending"].append((sample,actual.copy()))
            else:
                consume(sample,actual)

        # Load expensive CAD before opening USB. Runtime checks run off-thread
        # while this thread continues reading every original telemetry record.
        with Geometry() as geometry, serial_port("COM36") as port, \
                ThreadPoolExecutor(max_workers=1) as pool:
            class ObservedBoard(Board):
                def rpc(self,field,scope=0,value=0,op=16):
                    observed=op==16 and field in (5,6,7,15)
                    if observed:
                        note(stage="request_intent",field=field,scope=scope,
                             value=value,request_id=self.ident+1)
                    try:
                        result=super().rpc(field,scope,value,op)
                    except BaseException as exc:
                        if observed:
                            note(stage="request_failed",field=field,scope=scope,error=str(exc))
                        raise
                    if observed:
                        note(stage="request_ack",field=field,scope=scope,request_id=self.ident)
                    return result

            b = ObservedBoard(port)
            b.connect()
            refs = [[b.scalar(f,j,op=15) for f in (1,2,3)] for j in range(1,6)]
            if not np.allclose(refs,REFS,rtol=0,atol=2e-6):
                raise ValueError("Board references changed")
            initial, history = b.status(), b.history()
            if not idle_output_confirmed(initial) or initial["phase"]!="DRAG":
                raise ValueError("Current healthy DRAG absent; no automatic recovery")
            state["last_ref"] = np.array([j["reference_deg"] for j in history["joints"]])
            b.on_feedback = feedback
            note(stage="capture",schema="ylamp-happy-tracking-1",session=b.session.hex(),
                 firmware_sha256=EXPECTED,references_rad=refs,plan_sha256=sha(directory/"plan.json"),
                 control_layout=layout,operator_released=True,free_space=False,
                 exact_CAN_command_or_atomic_components=False,initial=initial)
            owned, success, failure = False, False, None
            try:
                for leg in plan["legs"]:
                    if display.stop:
                        raise RuntimeError("Operator cancelled before next segment")
                    state.update(active=None,tag=leg["name"],pending=[])
                    fresh_history = b.history()
                    current = np.array([j["reference_deg"] for j in fresh_history["joints"]])-np.rad2deg(REFS[:,0])
                    target = np.array(leg["target_software_deg"])
                    def actual_precheck():
                        geometric=geometry.check([[(current+HW[:,6]).tolist(),
                                                   (target+HW[:,6]).tolist()]])
                        return geometric,check_leg(current,target,leg["duration_ms"])
                    checked=pool.submit(actual_precheck)
                    display.title.set(f"检查 {state['tag']} 路径；持续收包，尚未发送本段目标")
                    while not checked.done():
                        b.pump()
                        display.pump()
                        if display.stop:
                            raise RuntimeError("Operator cancelled during path check")
                    geometry_check,dynamic=checked.result()
                    drain_observation(b)  # Existing bounded read-only queue drain; no RPC replay.
                    state["pending"]=[]
                    note(stage="segment_plan",**leg,actual_start_software_deg=current.tolist(),
                         current_geometry=geometry_check,current_nominal_check=dynamic)
                    # Own cancellation before a commit could be sent: a lost
                    # commit ACK must not leave a possibly active task behind.
                    owned = True
                    b.move(leg["target_software_deg"],leg["duration_ms"])
                    future = pool.submit(read_motion,binding["control_address"],layout)
                    deadline = time.monotonic()+(leg["duration_ms"]+leg["hold_ms"])*.001+3
                    next_status = 0.
                    status = b.observe_transition(("GRAVITY","DRAG","POSITION"))
                    while True:
                        b.pump()
                        display.pump()
                        if display.stop:
                            raise RuntimeError("Operator requested cancellation")
                        if state["active"] is None and future.done():
                            active = future.result()
                            if (active["duration_ms"]!=leg["duration_ms"] or active["cruise_ramp_s"]!=0 or
                                    not np.allclose(active["target_software_deg"],target,rtol=0,atol=1e-6)):
                                raise ValueError("Observed MCU plan differs from requested segment")
                            state["active"] = active
                            note(stage="verified_mcu_plan",segment=state["tag"],**active)
                            for sample, actual in state["pending"]:
                                consume(sample,actual)
                            state["pending"] = []
                        if time.monotonic()>=next_status:
                            status = b.status()
                            note(stage="status",segment=state["tag"],**status)
                            if (status["fault"] or status["limited"] or
                                    any(status[k]!=31 for k in ("fresh","enabled","submitted")) or
                                    status["phase"] not in ("MOTION","POSITION")):
                                raise RuntimeError("Motion interrupted; no retry or fault clear")
                            next_status = time.monotonic()+.1
                        if state["active"] is not None:
                            elapsed = (status["telemetry"]["tick_ms"]-
                                       state["active"]["start_tick_ms"]) & 0xffffffff
                            if elapsed>=leg["duration_ms"]+leg["hold_ms"] and status["phase"]=="POSITION":
                                segment_results.append(dict(name=leg["name"],plan=state["active"]))
                                note(stage="segment_complete",name=leg["name"],
                                     physical_arrival_claimed=False,**status)
                                break
                        if time.monotonic()>deadline:
                            raise TimeoutError("Segment completion not confirmed; no replay")
                success = True
            except BaseException as exc:
                note(stage="abort",error=str(exc),no_retry_or_recovery=True)
                failure = str(exc)
            finally:
                state["active"] = None  # No target-error samples after releasing task ownership.
                if owned:
                    try:
                        b.rpc(7)  # Original explicit trajectory cancel/G handover, never disable.
                        handover = b.observe_transition(("POSITION","MOTION","DRAG"))
                        note(stage="gravity_handover",**handover)
                        if not gravity_output_confirmed(handover):
                            raise RuntimeError("Gravity handover unconfirmed")
                        if not handover["fault"]:
                            b.drag()
                            final = b.observe_transition(("GRAVITY",))
                            note(stage="manual_handover",**final)
                            if not idle_output_confirmed(final) or final["phase"]!="DRAG":
                                raise RuntimeError("Original drag handover unconfirmed")
                    except BaseException as exc:
                        success = False
                        note(stage="handover_unconfirmed",error=str(exc),no_retry=True)
                note(stage="end",completed_segments=len(segment_results),success=success,
                     malformed=b.stream.malformed,incomplete=b.collector.incomplete,
                     collector_gaps=b.collector.gaps,parameters_changed=False)
                journal.flush()
                trace.flush()
        summary = tracking_summary(records,segment_results)
        summary.update(completed_segments=len(segment_results),success=success,
                       failure=failure,
                       capture_sha256=sha(directory/"capture.jsonl"),
                       tracking_sha256=sha(directory/"tracking.csv"),
                       no_parameter_changes=True,command_timing_atomicity_claimed=False)
        with (directory/"results.json").open("x",encoding="utf-8") as out:
            json.dump(summary,out,indent=2,allow_nan=False)
        print("RESULTS "+json.dumps(summary,allow_nan=False),flush=True)
    display.title.set("本次采集结束，已关闭USB；参数未修改")
    display.button.configure(text="关闭结果窗口",command=display.root.destroy)
    display.root.protocol("WM_DELETE_WINDOW",display.root.destroy)
    display.root.mainloop()


def tracking_summary(records, segments):
    rows = []
    for j in range(5):
        selected, seen = [], set()
        for row in records:
            key=(row["segment"],row["counts"][j])
            if key not in seen and row["segment"]!="prepare":
                selected.append(row)
                seen.add(key)
        if not selected:
            rows.append(dict(joint=j+1,samples=0))
            continue
        error = np.array([r["error"][j] for r in selected])
        last = segments[-1] if segments else None
        held = [r for r in selected if last and r["segment"]==last["name"] and
                last["plan"]["seconds"]+1.<=r["elapsed_s"][j]<=last["plan"]["seconds"]+1.5]
        overshoots = []
        for seg in segments:
            delta = seg["plan"]["target_software_deg"][j]-seg["plan"]["start_software_deg"][j]
            if seg["name"]=="prepare" or abs(delta)<.1:
                continue
            values = [np.sign(delta)*(r["actual"][j]-seg["plan"]["target_software_deg"][j])
                      for r in selected if r["segment"]==seg["name"]]
            if values:
                overshoots.append(max(0.,max(values)))
        rows.append(dict(joint=j+1,samples=len(selected),
            rms_error_deg=float(np.sqrt(np.mean(error**2))),
            max_absolute_error_deg=float(np.max(abs(error))),
            p95_absolute_error_deg=float(np.quantile(abs(error),.95)),
            final_hold_mean_error_deg=float(np.mean([r["error"][j] for r in held])) if held else None,
            final_hold_max_absolute_error_deg=max(abs(r["error"][j]) for r in held) if held else None,
            max_measured_overshoot_deg=max(overshoots) if overshoots else None,
            peak_feedback_torque_est_nm=max(abs(r["torque"][j]) for r in selected)))
    return dict(joints=rows,statistics_exclude_preparation=True,
                final_hold_window="after final segment duration +1s, through +1.5s",
                torque_is_drive_current_estimate_not_external_truth=True)


def verify_io(directory):
    """Read-only workload check: no SESSION, RPC or motor command is sent."""
    plan=json.loads((directory/"plan.json").read_text())
    leg=plan["legs"][0]
    current=np.array(leg["nominal_start_software_deg"])
    target=np.array(leg["target_software_deg"])
    with Geometry() as geometry, serial_port("COM36") as port, \
            ThreadPoolExecutor(max_workers=1) as pool:
        b=Board(port)  # Intentionally no connect(): receive only, no SESSION.
        def work():
            result=geometry.check([[(current+HW[:,6]).tolist(),(target+HW[:,6]).tolist()]])
            return result,check_leg(current,target,leg["duration_ms"])
        started=time.monotonic()
        job=pool.submit(work)
        first_tick=None
        max_pump_interval=0.
        previous=started
        while not job.done():
            b.pump()
            now=time.monotonic()
            max_pump_interval=max(max_pump_interval,now-previous)
            previous=now
            if first_tick is None and b.latest:
                first_tick=b.latest["tick_ms"]
        result,dynamic=job.result()
        drain_observation(b)
        if b.latest is None or first_tick is None:
            raise RuntimeError("No live passive telemetry during the workload")
        report=dict(no_session_rpc_or_motor_command=True,first_board_tick_ms=first_tick,
            last_board_tick_ms=b.latest["tick_ms"],host_workload_seconds=time.monotonic()-started,
            max_pump_interval_ms=max_pump_interval*1000,remaining_rx_bytes=port.in_waiting,
            malformed=b.stream.malformed,incomplete=b.collector.incomplete,
            collector_gaps=b.collector.gaps,geometry=result,nominal_check=dynamic,
            final_telemetry=b.latest,script_sha256=sha(__file__))
    with (directory/"passive_io_check.json").open("x",encoding="utf-8") as out:
        json.dump(report,out,indent=2,allow_nan=False)
    print(json.dumps({k:v for k,v in report.items() if k not in
        ("final_telemetry","nominal_check","geometry")},indent=2),flush=True)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def preflight(directory):
    """Read running image and three bounded state windows; no USB SESSION."""
    binding = json.loads(STATE_BINDING.read_text(encoding="utf-8"))
    audit = json.loads((FIRMWARE / "candidate_audit_01.json").read_text(encoding="utf-8"))
    if (binding["firmware_sha256"] != EXPECTED or
            sha(FIRMWARE / "ylamp_v12_five_axis.bin") != EXPECTED or
            sha(FIRMWARE / "ylamp_v12_five_axis.elf") != binding["elf_sha256"]):
        raise ValueError("Frozen ELF/BIN binding changed")
    for name, digest in audit["source_sha256"].items():
        if sha(ROOT / name) != digest:
            raise ValueError(f"Audited firmware source changed: {name}")
    base, size = binding["control_address"], binding["control_size"]
    symbols = subprocess.check_output(
        ["D:/GCC/ARMGCC/10 2021.10/bin/arm-none-eabi-nm.exe", "-S", "--defined-only",
         str(FIRMWARE/"ylamp_v12_five_axis.elf")], text=True)
    match = re.search(r"^([0-9a-f]+)\s+([0-9a-f]+)\s+b\s+c$", symbols, re.M)
    if not match or (int(match[1],16), int(match[2],16)) != (base,size) or size % 4:
        raise ValueError("Bound control object differs from the verified ELF")
    application = directory / "running_application.bin"
    commands = [
        "gdb port disabled", "tcl port disabled", "telnet port disabled",
        "adapter speed 1000", "reset_config none",
        "stm32h7x.cpu0 configure -event examine-end {} -work-area-size 0",
        "init", "targets stm32h7x.cpu0", "poll",
        'set u [read_memory 0x1ff1e800 32 3]; '
        'if {[lindex $u 0]!=0x420030 || [lindex $u 1]!=0x31345106 || '
        '[lindex $u 2]!=0x38373332} {error WRONG_BOARD}; echo UID_OK',
        'set d [lindex [read_memory 0xe000edf0 32 1] 0]; '
        'if {$d & 0x20000} {error CORE_HALTED}; echo DHCSR_BEFORE_$d',
        "echo CFSR_HFSR_[read_memory 0xe000ed28 32 2]",
        f"dump_image {{{application.as_posix()}}} 0x08000000 "
        f"{(FIRMWARE/'ylamp_v12_five_axis.bin').stat().st_size}",
    ]
    for i in range(3):
        commands.append(f"echo CONTROL_{i}_[read_memory {base:#x} 32 {size//4}]")
    commands += [
        'set d [lindex [read_memory 0xe000edf0 32 1] 0]; '
        'if {$d & 0x20000} {error CORE_HALTED}; echo DHCSR_AFTER_$d',
        "echo FINAL_STATE_[stm32h7x.cpu0 curstate]", "shutdown",
    ]
    command = [str(OPENOCD / "bin/openocd.exe"), "-s", str(OPENOCD / "share/openocd/scripts"),
               "-f", "interface/stlink.cfg", "-f", "target/stm32h7x.cfg"]
    for line in commands:
        command += ["-c", line]
    result = subprocess.run(command, capture_output=True, text=True, timeout=30)
    text = result.stdout + result.stderr
    with (directory / "preflight_probe.log").open("x", encoding="utf-8") as log:
        log.write(text)
    if result.returncode or "UID_OK" not in text or "FINAL_STATE_running" not in text:
        raise RuntimeError("Read-only preflight failed; no retry or recovery")
    if sha(application) != EXPECTED:
        raise ValueError("Actual application differs; RAM not decoded")
    snapshots = []
    layout = binding["control_layout"]
    formats = dict(phase="B", fresh="B", enabled="B", submitted="B", limited="B",
                   fault="I", reference_rad="5f", raw_rad="5f", velocity_rad_s="5f",
                   gravity_valid="?", gravity_nm="5f", moving="?", j3_read_used="?",
                   test_joint="I", drag_active="?", impedance="?")
    for j in range(1, 6):
        formats[f"j{j}_state"] = "B"
        formats[f"j{j}_received_count"] = "I"
    for i in range(3):
        words = re.search(rf"^CONTROL_{i}_(.*)$", text, re.M)
        if not words or len(words[1].split()) != size//4:
            raise ValueError("Incomplete live state")
        raw = struct.pack("<" + "I"*(size//4), *(int(v, 16) for v in words[1].split()))
        row = {}
        for name, fmt in formats.items():
            values = struct.unpack_from("<"+fmt, raw, layout[name])
            row[name] = values[0] if len(values) == 1 else list(values)
        snapshots.append(row)
    report = dict(firmware_sha256=sha(application), elf_sha256=binding["elf_sha256"],
                  layout_binding_sha256=sha(STATE_BINDING),
                  source_sha256=audit["source_sha256"], snapshots=snapshots,
                  no_halt_reset_usb_session_or_motor_command=True,
                  atomic_snapshot=False, script_sha256=sha(__file__))
    with (directory / "preflight.json").open("x", encoding="utf-8") as out:
        json.dump(report, out, indent=2, allow_nan=False)
    print(json.dumps(report["snapshots"][-1], indent=2), flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, required=True)
    parser.add_argument("--plan", action="store_true", help="offline plan from the existing preflight")
    parser.add_argument("--execute", action="store_true", help="one real, finite sequence; no tuning")
    parser.add_argument("--released", action="store_true",
                        help="operator explicitly confirmed released and outside travel for this run")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--verify-io",action="store_true",
                        help="passive receive-only workload test, no SESSION or motion")
    args = parser.parse_args()
    if sum((args.plan,args.execute,args.self_test,args.verify_io))>1:
        parser.error("choose one action")
    if args.verify_io:
        verify_io(args.directory.resolve())
        return
    if args.self_test:
        start,end=np.array([1,2,3,4,5.]),np.array([2,0,4,7,1.])
        q,v,a=quintic(start,end,2.,np.array([0.,1.,2.]))
        assert np.array_equal(q[0],start) and np.array_equal(q[-1],end)
        assert np.max(abs(v[[0,-1]]))==np.max(abs(a[[0,-1]]))==0
        eps=1e-5
        qm,vm,_=quintic(start,end,2.,np.array([.7-eps]))
        qp,vp,_=quintic(start,end,2.,np.array([.7+eps]))
        _,vv,aa=quintic(start,end,2.,np.array([.7]))
        assert np.max(abs(np.deg2rad((qp-qm)/(2*eps))-vv))<1e-8
        assert np.max(abs((vp-vm)/(2*eps)-aa))<1e-7
        print("PASS: native-Move quintic q/v/a, endpoints and finite derivatives; no hardware")
        return
    if args.execute:
        if not args.released or args.plan:
            parser.error("--execute needs this run's --released confirmation, not --plan")
        if args.directory.exists():
            if any((args.directory/name).exists() for name in
                   ("capture.jsonl","results.json","before_execute")):
                raise FileExistsError("This prepared run was already attempted; never replay it")
            prepared=json.loads((args.directory/"plan.json").read_text())
            for name,digest in prepared["source_sha256"].items():
                if sha(ROOT/name)!=digest:
                    raise ValueError(f"Prepared source changed: {name}; regenerate offline")
            fresh_dir=args.directory/"before_execute"
            fresh_dir.mkdir()
            fresh=preflight(fresh_dir.resolve())["snapshots"][-1]
            if (fresh["moving"] or fresh["test_joint"] or fresh["fault"] or
                    fresh["limited"] or fresh["phase"]!=7 or
                    any(fresh[k]!=31 for k in ("fresh","enabled","submitted"))):
                raise ValueError("Fresh existing DRAG absent; no recovery or takeover")
        else:
            args.directory.mkdir(parents=True,exist_ok=False)
            preflight(args.directory.resolve())
            make_plan(args.directory.resolve())
        execute(args.directory.resolve())
        return
    if args.plan:
        make_plan(args.directory.resolve())
        return
    args.directory.mkdir(parents=True, exist_ok=False)
    preflight(args.directory.resolve())


if __name__ == "__main__":
    main()
