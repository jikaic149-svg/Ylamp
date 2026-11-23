"""One explicit full-angle/load-pose friction batch; no reset, enable or recovery."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess

import numpy as np

from five_control import Board, DRAG_VERSION, idle_output_confirmed
from five_friction_campaign import Campaign
from five_gravity_sweep import baseline_goal
from five_identify import REFS, ROOT
from usb_monitor import serial_port


def source_clock_audit(capture, *, require_free_space=True):
    """Bound start from same-board observation/commit order, never invent an exact tick."""
    latest = lower = upper = duration = end = header = final = None
    digest = hashlib.sha256()
    with capture.open("rb") as source:
        for line in source:
            digest.update(line)
            r = json.loads(line)
            stage = r["stage"]
            if stage == "capture":
                header = r
            if stage == "waypoint":
                if lower is not None:
                    raise ValueError("Timing audit requires one completed leg")
                lower, duration = latest, r["duration_ms"]
            if "phase" in r:
                if (r["fault"] or r["limited"] or
                        any(r[k] != 31 for k in ("fresh", "enabled", "submitted"))):
                    raise ValueError("Cannot resume an unhealthy capture")
            tel = r.get("telemetry")
            if tel:
                latest = tel["tick_ms"]
                if stage == "sample" and r["phase"] == "MOTION" and upper is None:
                    # Status flags/time are not atomic; keep the declared
                    # 100 ms telemetry freshness allowance, rather than
                    # claiming the first observed MOTION tick is the start.
                    upper = latest+tel["max_age_ms"]
            if stage == "handover":
                final = r
            if stage == "end":
                end = r
    if (not header or (require_free_space and header.get("free_space") is not True) or not end or
            end["malformed"] or end["incomplete"] or not final or
            final["phase"] != "GRAVITY" or lower is None or upper is None or upper < lower):
        raise ValueError("Incomplete capture; cannot skip or relabel a failed movement")
    return dict(capture=str(capture), capture_sha256=digest.hexdigest(),
                free_space_confirmed=header.get("free_space") is True,
                board_start_tick_interval_ms=[lower, upper],
                requested_duration_ms=duration, exact_start_verified=False,
                interval_requires_conservative_plateau_trimming=True)


def full_leg(raw, joint, target_deg, speed, ramp_ms):
    """Integrate the existing finite velocity profile toward a continuous endpoint."""
    raw = np.asarray(raw, dtype=float)
    if (raw.shape != (5,) or not np.isfinite(raw).all() or joint not in range(1, 6)
            or not math.isfinite(target_deg) or not math.isfinite(speed) or speed <= 0
            or not isinstance(ramp_ms, int) or ramp_ms <= 0):
        raise ValueError("Invalid full-angle leg")
    low, high = np.rad2deg(REFS[joint-1, 1:])+[1.1, -1.1]
    if not low <= target_deg <= high:
        raise ValueError("Endpoint outside retained target inset")
    delta = target_deg-raw[joint-1]
    if not delta:
        raise ValueError("Zero-distance leg")
    direction = 1 if delta > 0 else -1
    plateau = math.floor(math.radians(abs(delta))/speed*1000)-ramp_ms
    software = (raw-np.rad2deg(REFS[:, 0])).tolist()
    goal, _, duration = baseline_goal(software, joint, speed, plateau, ramp_ms, direction)
    return np.asarray(goal)+np.rad2deg(REFS[:, 0]), plateau, direction, duration


def verify_timing_layout(elf, nm, objdump):
    """Bind this read-only address to the supplied, currently deployed ELF."""
    symbols = subprocess.check_output([nm, "-S", "--defined-only", str(elf)], text=True)
    match = re.search(r"^([0-9a-f]+)\s+[0-9a-f]+\s+b\s+c$", symbols, re.M)
    asm = subprocess.check_output([objdump, "-d", "--disassemble=FiveControl_Speed",
                                   str(elf)], text=True)
    if (not match or int(match[1], 16) != 0x20002AE8 or
            not re.search(r"str\.w\s+r\d+,\s*\[r7,\s*#440\]", asm) or
            not re.search(r"str\.w\s+r\d+,\s*\[r7,\s*#444\]", asm) or
            "20002ae8" not in asm):
        raise ValueError("Timing layout differs; do not use a historical RAM address")
    return 0x20002AE8+440


class GlobalBatch(Campaign):
    def state(self):
        with serial_port(self.args.port) as port:
            b = Board(port)
            b.connect()
            refs = [[b.scalar(f, j, op=15) for f in (1, 2, 3)] for j in range(1, 6)]
            if not np.allclose(refs, REFS, rtol=0, atol=2e-6):
                raise ValueError("Board references differ")
            state, history = b.status(), b.history()
            if not idle_output_confirmed(state) or b.rpc(251) != DRAG_VERSION:
                raise RuntimeError("Healthy persistent idle absent; no recovery")
        raw = np.array([j["reference_deg"] for j in history["joints"]])
        if not np.isfinite(raw).all():
            raise ValueError("Missing continuous reference")
        self.note(stage="continuous_status", history=history,
                  raw_to_reference_turns=np.rint((raw-np.rad2deg(
                      [j["position_rad"] for j in state["telemetry"]["joints"]]))/360).astype(int).tolist(),
                  **state)
        return raw

    def timing(self, capture, expected_ms):
        # Ordinary SWD after the capture has closed; never steals an active USB
        # session, halts the core, executes a CRC algorithm or changes RAM.
        cmd = [
            self.args.openocd, "-s", self.args.openocd_scripts,
            "-f", "interface/stlink.cfg", "-f", "target/stm32h7x.cfg",
            "-c", "gdb port disabled", "-c", "tcl port disabled",
            "-c", "telnet port disabled", "-c", "adapter speed 1000",
            "-c", "reset_config none",
            "-c", "stm32h7x.cpu0 configure -event examine-end {}",
            "-c", "init", "-c", "targets stm32h7x.cpu0", "-c", "poll",
            "-c", 'set u [read_memory 0x1ff1e800 32 3]; '
                  'if {[lindex $u 0]!=0x420030 || [lindex $u 1]!=0x31345106 || '
                  '[lindex $u 2]!=0x38373332} {error WRONG_BOARD}; echo UID_OK',
            "-c", f"echo PLAN_START_DURATION_[read_memory {self.args.timing_address:#x} 32 2]",
            "-c", 'set d [lindex [read_memory 0xe000edf0 32 1] 0]; '
                  'if {$d & 0x20000} {error CORE_HALTED}; echo DHCSR_$d',
            "-c", "shutdown"]
        result = subprocess.run(cmd, text=True, capture_output=True, timeout=15)
        text = result.stdout+result.stderr
        if result.returncode:
            self.note(stage="timing_query_failed", capture=str(capture),
                      returncode=result.returncode, probe_output=text, no_motor_command=True)
            raise RuntimeError("Read-only timing query failed; see preserved probe output")
        values = re.search(r"PLAN_START_DURATION_(0x[0-9a-f]+) (0x[0-9a-f]+)", text)
        if not values or "UID_OK" not in text:
            raise RuntimeError("Read-only plan timing not returned")
        start, duration = [int(x, 16) for x in values.groups()]
        if duration != expected_ms:
            raise RuntimeError("Board duration differs from completed capture")
        self.note(stage="verified_timing", capture=str(capture),
                  capture_sha256=hashlib.sha256(capture.read_bytes()).hexdigest(),
                  board_start_tick_ms=start, duration_ms=duration, core_running=True,
                  probe_output=text)

    def leg(self, joint, endpoint, speed, tag):
        capture = self.args.directory/f"{tag}.jsonl"
        logical_name = capture.name
        if capture.name in self.previous:
            capture, expected_sha = self.previous[capture.name]
            audit = source_clock_audit(capture)
            if audit["capture_sha256"] != expected_sha:
                raise ValueError("Completed capture changed; no motion replay")
            self.note(stage="previous_capture_retained", **audit)
            return
        if capture.exists() and self.args.replan_unsent:
            records = [json.loads(s) for s in capture.read_text(encoding="utf-8").splitlines()]
            if (any(r["stage"] in ("waypoint", "rpc_accepted", "end") for r in records)
                    or not any(r["stage"] == "abort" and "no motion sent" in r["error"]
                               for r in records)):
                raise ValueError("Only a proven unsent leg can be replanned; no motion replay")
            self.note(stage="unsent_capture_retained", capture=str(capture),
                      sha256=hashlib.sha256(capture.read_bytes()).hexdigest(),
                      no_motion_submitted=True)
            capture = capture.with_stem(capture.stem+"_unsent_replan_01")
        current = self.state()
        goal, plateau, direction, duration = full_leg(
            current, joint, endpoint, speed, self.args.ramp_ms)
        self.check([[current.tolist(), goal.tolist()]])
        name = capture.name
        self.note(stage="planned_full_leg", joint=joint, endpoint_reference_deg=endpoint,
                  predicted_reference_deg=goal.tolist(), speed_rad_s=direction*speed,
                  duration_ms=duration, label="actual_endpoint_not_guaranteed")
        self.run_tool("five_gravity_sweep.py", name, [
            "--continuous-reference", "--free-space", "--single-leg",
            "--joints", str(joint), "--constant-speed", str(speed),
            "--ramp-ms", str(self.args.ramp_ms), "--plateau-ms", str(plateau),
            "--target-reference-deg", str(endpoint)])
        self.note(stage="completed_logical", capture=str(capture), logical_name=logical_name)
        if self.args.source_clock_only:
            self.note(stage="source_timing_interval", **source_clock_audit(capture))
        else:
            duration = source_clock_audit(capture)["requested_duration_ms"]
            self.timing(capture, duration)

    def execute(self):
        anchor = self.state()
        self.previous = {}
        for journal in self.args.resume_from:
            records = [json.loads(s) for s in journal.read_text(encoding="utf-8").splitlines()]
            for r in records:
                if r["stage"] == "batch_started":
                    if (r["firmware_sha256"] != self.args.firmware_sha or
                            r["joints"] != self.args.joints or r["speeds"] != self.args.speeds or
                            r["rounds"] != self.args.rounds):
                        raise ValueError("Resume matrix/profile differs")
                    anchor = np.asarray(r["pose_anchor_reference_deg"])
                if r["stage"] in ("completed", "completed_logical"):
                    path = Path(r["capture"])
                    audit = source_clock_audit(path)
                    self.previous[r.get("logical_name", path.name)] = (path, audit["capture_sha256"])
                    self.note(stage="retained_completed_capture", source_journal=str(journal), **audit)
        self.note(stage="batch_started", pose_anchor_reference_deg=anchor.tolist(),
                  joints=self.args.joints, speeds=self.args.speeds, rounds=self.args.rounds,
                  free_space=True, firmware_sha256=self.args.firmware_sha,
                  ramp_ms=self.args.ramp_ms, source_clock_only=self.args.source_clock_only,
                  full_global_matrix_complete=False)
        for repeat in range(1, self.args.rounds+1):
            for joint in self.args.joints:
                main_names = [f"global_{self.args.tag}_r{repeat}_j{joint}_{round(v*100):03d}_{d}.jsonl"
                              for v in self.args.speeds for d in ("pos", "neg")]
                if all(name in self.previous for name in main_names):
                    self.note(stage="joint_repeat_previously_complete", joint=joint, repeat=repeat)
                    continue
                # Source firmware requires 1 degree. These task endpoints leave
                # 0.2 degree for quantization between two fresh host readings;
                # this is not a new movement gate or a changed mechanical range.
                low, high = np.rad2deg(REFS[joint-1, 1:])+[1.2, -1.2]
                current = self.state()
                minimum = math.degrees(.16*(self.args.ramp_ms+100)/1000)
                if abs(current[joint-1]-low) > minimum:
                    self.leg(joint, low, .16,
                             f"global_{self.args.tag}_r{repeat}_j{joint}_entry")
                for speed in self.args.speeds:
                    for direction, endpoint in ((1, high), (-1, low)):
                        self.leg(joint, endpoint, speed,
                            f"global_{self.args.tag}_r{repeat}_j{joint}_{round(speed*100):03d}_{'pos' if direction>0 else 'neg'}")
                # Restore the measured load-pose anchor before the next axis/
                # repeat. Fresh actual poses, not these nominal tags, decide
                # whether independent repeats can be matched in analysis.
                current = self.state()
                if abs(current[joint-1]-anchor[joint-1]) > minimum:
                    self.leg(joint, anchor[joint-1], .16,
                             f"global_{self.args.tag}_r{repeat}_j{joint}_return")
                self.note(stage="joint_repeat_complete", joint=joint, repeat=repeat)
        self.state()
        self.note(stage="batch_complete",
                  matched_direction_legs=len(self.args.joints)*len(self.args.speeds)*2*self.args.rounds,
                  full_global_matrix_complete=False)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", required=True)
    p.add_argument("--directory", type=Path, required=True)
    p.add_argument("--log", type=Path, required=True)
    p.add_argument("--tag", required=True)
    p.add_argument("--joints", type=int, nargs="+", choices=range(1, 6), required=True)
    p.add_argument("--speeds", type=float, nargs="+", default=[.16, .48, .8])
    p.add_argument("--rounds", type=int, choices=(1, 2), default=1)
    p.add_argument("--ramp-ms", type=int, default=300)
    p.add_argument("--resume-from", type=Path, nargs="+", default=[],
                   help="retain successful hashed captures; never replay a failed movement")
    p.add_argument("--source-clock-only", action="store_true",
                   help="keep explicit board-time start intervals instead of requiring SWD")
    p.add_argument("--replan-unsent", action="store_true",
                   help="preserve a proven pre-submit refusal and replan once in a new file")
    p.add_argument("--execute", action="store_true", required=True)
    p.add_argument("--free-space", action="store_true", required=True)
    p.add_argument("--openocd", required=True)
    p.add_argument("--openocd-scripts", required=True)
    p.add_argument("--nm", required=True)
    p.add_argument("--objdump", required=True)
    args = p.parse_args()
    if (len(set(args.joints)) != len(args.joints) or not re.fullmatch(r"[a-z0-9_]+", args.tag)
            or not all(math.isfinite(v) and v in (.16, .48, .8) for v in args.speeds)
            or len(set(args.speeds)) != len(args.speeds) or args.ramp_ms <= 0):
        p.error("Invalid finite batch")
    binary = args.directory/"ylamp_v12_five_axis.bin"
    args.firmware_sha = hashlib.sha256(binary.read_bytes()).hexdigest()
    if args.firmware_sha != "5d788f4436092380f4b068c375c91c9cf37d4bc3772273e96017ef1c0dbd1b47":
        raise ValueError("Batch is bound to the frozen, deployed 5D788 profile")
    args.timing_address = verify_timing_layout(
        args.directory/"ylamp_v12_five_axis.elf", args.nm, args.objdump)
    with args.log.open("x", encoding="utf-8") as log:
        batch = GlobalBatch(args, log)
        try:
            batch.execute()
        except BaseException as exc:
            batch.note(stage="batch_abort", error=str(exc),no_retry_or_recovery=True)
            raise
        finally:
            batch.geometry.stdin.close()
            batch.geometry.wait(timeout=10)


if __name__ == "__main__":
    main()
