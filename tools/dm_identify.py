"""Bounded identification recordings through the existing MCU Motion/RPC owner.

No motion without --run. Closing this program does NOT disable the motors:
successful runs return to the captured target and remain in impedance hold.
Torque feedback is a current-based estimate, not calibrated external force.
"""
from __future__ import annotations

import argparse
import json
import math
import queue
import time
from datetime import datetime, timezone
from pathlib import Path

import dm_monitor
import dm_tuning


def make_plan(origin, bounds, amplitude_deg, speed, acceleration, jerk, *,
              axes=(0, 1), center_j1=None, center_j2=None,
              bidirectional=False, repetitions=1, coupled=False):
    """Rest-to-rest, one axis at a time. Ruckig runs on the MCU."""
    if len(origin) != 2 or len(bounds) != 2 or any(
            not math.isfinite(x) or x <= 0
            for x in (amplitude_deg, speed, acceleration, jerk)):
        raise ValueError("Invalid two-axis experiment")
    if (acceleration > 2 or speed > 160 * 2 * math.pi / 60 or
            not isinstance(repetitions, int) or repetitions < 1):
        raise ValueError("Experiment exceeds the user-authorized v/a limits")
    jobs = []
    center = list(origin)
    if center_j1 is not None:
        center[0] = math.radians(center_j1)
    if center_j2 is not None:
        center[1] = math.radians(center_j2)

    def append(name, target, **metadata):
        if any(not math.isfinite(q) or not low <= q <= high
               for q, (low, high) in zip(target, bounds)):
            raise ValueError("Experiment leaves the configured position range")
        values = [[math.degrees(q), math.degrees(speed),
                   math.degrees(acceleration), math.degrees(jerk)] for q in target]
        dm_tuning.validate_motion(values)
        jobs.append(dict(name=name, target_rad=target, request=values, **metadata))

    if center_j1 is not None or center_j2 is not None:
        append("move_to_center", center)
    if coupled:
        if tuple(axes) != (0, 1):
            raise ValueError("Coupled experiments require both joints")
        for repeat in range(repetitions):
            directions = ((1, 1), (-1, -1), (1, -1), (-1, 1))
            for signs in directions if repeat % 2 == 0 else reversed(directions):
                tag = f"coupled_r{repeat+1}_from_{signs[0]:+d}_{signs[1]:+d}"
                target = [q+math.radians(s*amplitude_deg) for q, s in zip(center, signs)]
                append(tag+"_offset", target)
                append(tag+"_return", list(center), test_joints=[1, 2],
                       repeat=repeat+1, from_offset_signs=list(signs),
                       center_rad=list(center))
        if center_j1 is not None or center_j2 is not None:
            append("return_capture", list(origin))
        return jobs
    for axis in axes:
        if bidirectional:
            for repeat in range(repetitions):
                # Reverse the order between repeats to expose order/thermal drift.
                signs = (1, -1) if repeat % 2 == 0 else (-1, 1)
                for sign in signs:
                    tag = f"J{axis+1}_r{repeat+1}_from_{'plus' if sign > 0 else 'minus'}"
                    target = list(center)
                    target[axis] += math.radians(sign * amplitude_deg)
                    append(tag+"_offset", target)
                    append(tag+"_return", list(center), test_joint=axis+1,
                           repeat=repeat+1, from_offset_sign=sign,
                           center_rad=list(center))
            continue
        for offset in (amplitude_deg, -amplitude_deg, 0):
            target = list(center)
            target[axis] += math.radians(offset)
            append(f"J{axis+1}_{offset:+g}deg", target)
    if center_j1 is not None or center_j2 is not None:
        append("return_capture", list(origin))
    return jobs


class Recording:
    def __init__(self, port):
        self.origin = time.perf_counter()
        self.samples = queue.Queue(maxsize=8192)
        self.events = queue.SimpleQueue()
        self.reader = dm_monitor.SerialReader(
            {"port": port}, self.samples, self.events, self.origin)
        self.reader.tuning.poll_details = False
        self.records, self.markers, self.messages, self.calibrations = [], [], [], []
        self.motion_acks = []
        self.observe_imu = False

    def pump(self, *, active=False):
        while True:
            try:
                self.records.append(self.samples.get_nowait())
            except queue.Empty:
                break
        failures = []
        while True:
            try:
                kind, value = self.events.get_nowait()
            except queue.Empty:
                break
            if kind in ("error", "tuning_error", "motion_error", "calibration_error"):
                failures.append(f"{kind}: {value}")
                self.messages.append(failures[-1])
            elif kind == "calibration_status":
                self.calibrations.append(dict(
                    value, elapsed_s=value["at"]-self.origin,
                    roll_pitch_deg=[x if math.isfinite(x) else None for x in
                                    dm_tuning.gravity_angles(value["accel"])[:2]]))
            elif kind == "motion_ack":
                self.motion_acks.append(dict(
                    kind=value, sequence=self.reader.telemetry["sequence"],
                    elapsed_s=time.perf_counter()-self.origin))
        if failures:
            raise RuntimeError("; ".join(failures))
        if self.reader.stop_requested.is_set():
            raise RuntimeError("Reader disconnected; motor state may be unknown")
        status = self.reader.telemetry
        if status is not None and active:
            if time.perf_counter() - self.reader.telemetry_at > .3:
                raise RuntimeError("No fresh MCU telemetry")
            if status["state"] in (5, 6):
                raise RuntimeError(f"Task state={status['state']} reason={status['reason']}")
            if status["valid_mask"] != 3 or any(
                    j["age_ms"] > 100 for j in status["joints"]):
                raise RuntimeError("Motor feedback is not fresh")
        return status

    def wait(self, predicate, timeout, *, active=False):
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            status = self.pump(active=active)
            if predicate(status):
                return status
            self.reader.stop_requested.wait(.02)
        raise TimeoutError("Timed out waiting for MCU operation")

    def collect(self, seconds, *, active=True):
        deadline = time.perf_counter() + seconds
        next_imu = time.perf_counter()
        last_read_count = None
        while time.perf_counter() < deadline:
            self.pump(active=active)
            if self.observe_imu and time.perf_counter() >= next_imu:
                last_read_count = len(self.calibrations)
                self.action("calibration_read")
                next_imu = time.perf_counter()+.8
            self.reader.stop_requested.wait(.02)
        if last_read_count is not None:
            self.wait(lambda s: len(self.calibrations) > last_read_count, 3, active=active)

    def start_imu(self):
        self.action("calibration_read")
        self.wait(lambda s: bool(self.calibrations), 3)
        if not self.calibrations[-1]["flags"] & 16:
            self.action("imu", (True, self.reader.tuning.snapshot["revision"]))
            self.wait(lambda s: bool(self.calibrations[-1]["flags"] & 16), 3)
        self.observe_imu = True
        self.collect(7, active=False)
        if self.calibrations[-1]["flags"] & 3 != 3:
            raise RuntimeError("IMU has no fresh warmed measurement")

    def action(self, kind, payload=None):
        error = self.reader.tuning.request(kind, payload)
        if error:
            raise RuntimeError(error)

    def capture_hold(self):
        count = len(self.motion_acks)
        self.action("impedance")
        return self.wait(lambda s: capture_completed(s, self.motion_acks, count), 8)

    def mark(self, name, **fields):
        self.markers.append(dict(name=name, elapsed_s=time.perf_counter()-self.origin,
                                 tick_ms=self.reader.telemetry["tick_ms"], **fields))
        print(name, flush=True)


def holding_at(status, target=None):
    if not status or (status["mode"], status["state"], status["phase"]) != (1, 4, 4):
        return False
    return target is None or all(
        abs(j["target_reference"] - target[j["id"]-1]) < 2e-5
        for j in status["joints"])


def capture_completed(status, acknowledgements, previous_count):
    """An old HOLD is not completion of a newly queued capture."""
    return (len(acknowledgements) > previous_count and
            acknowledgements[-1]["kind"] == "impedance" and holding_at(status) and
            0 < ((status["sequence"]-acknowledgements[-1]["sequence"]) & 0xFFFFFFFF)
            < 0x80000000)


def summarize(records, markers):
    result = []
    for start, end in zip(markers, markers[1:]):
        if not start["name"].endswith("_settle"):
            continue
        for joint in (1, 2):
            rows = [s for s in records if s.motor == joint and
                    start["elapsed_s"] <= s.elapsed_s < end["elapsed_s"]]
            if not rows:
                continue
            tail = rows[len(rows)//2:]
            errors = [math.degrees(s.tracking_error) for s in tail]
            mean_error = sum(errors)/len(errors)
            result.append(dict(
                segment=start["name"], joint=joint, samples=len(rows),
                position_deg=sum(math.degrees(s.reference) for s in tail)/len(tail),
                error_deg=mean_error,
                error_std_deg=math.sqrt(sum((e-mean_error)**2 for e in errors)/len(errors)),
                error_span_deg=max(errors)-min(errors),
                target_deg=sum(math.degrees(s.target_reference) for s in tail)/len(tail),
                torque_estimate_nm=sum(s.torque_nm for s in tail)/len(tail),
                ff_nm=sum(s.ff for s in tail)/len(tail),
                peak_speed_rad_s=max(abs(s.velocity_rad_s) for s in rows),
                tail_peak_speed_rad_s=max(abs(s.velocity_rad_s) for s in tail),
                tail_span_s=tail[-1].elapsed_s-tail[0].elapsed_s))
    return result


def run(args):
    recording = Recording(args.port)
    output = Path(args.output) if args.output else Path(__file__).resolve().parents[1] / (
        "build/identification/" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ"))
    output.mkdir(parents=True, exist_ok=False)
    report = dict(start_utc=datetime.now(timezone.utc).isoformat(),
                  transport="existing USART1 reliable RPC / MCU-only motor control",
                  torque_source="motor current estimate; not external force",
                  user_limits=dict(speed_rpm=160, acceleration_rad_s2=2),
                  amplitude_deg=args.amplitude, test_speed_rad_s=args.speed,
                  test_acceleration_rad_s2=args.acceleration, test_jerk_rad_s3=args.jerk,
                  experiment=("matched_center_coupled_trajectory_approach" if args.coupled else
                              "matched_center_bidirectional_trajectory_approach"
                              if args.bidirectional else "sequential_endpoint_tracking"),
                  repetitions=args.repetitions,
                  manual_push_release=False,
                  result="not_started")
    armed = False
    recording.reader.start()
    try:
        status = recording.wait(lambda s: recording.reader.ready.is_set(), 15)
        if not recording.reader.tuning.motion_supported:
            raise RuntimeError("Firmware has no compatible Motion endpoint")
        inactive = status["state"] == 5 and status["reason"] == 12 and all(
            j["state"] == 0 for j in status["joints"])
        if not (status["state"] in (0, 4) or inactive) or status["valid_mask"] != 3 or any(
                j["age_ms"] > 100 for j in status["joints"]):
            raise RuntimeError("Expected fresh IDLE or HOLDING feedback")
        snapshot = recording.reader.tuning.snapshot
        report["configuration"] = snapshot
        report["initial_status"] = status
        origin = [j["reference"] for j in sorted(status["joints"], key=lambda j: j["id"])]
        bounds = [(math.radians(snapshot["values"][f"{j}.1"]),
                   math.radians(snapshot["values"][f"{j}.2"])) for j in (1, 2)]
        plan = make_plan(origin, bounds, args.amplitude, args.speed,
                         args.acceleration, args.jerk,
                         axes=(args.joint-1,) if args.joint else (0, 1),
                         center_j1=args.center_j1, center_j2=args.center_j2,
                         bidirectional=args.bidirectional, repetitions=args.repetitions,
                         coupled=args.coupled)
        if any(args.speed > limit[1] for limit in recording.reader.limits.values()):
            raise ValueError("Test speed exceeds the configured MIT velocity encoding")
        report["plan"] = plan
        print("Connected; origin_deg =", [math.degrees(q) for q in origin], flush=True)
        if args.imu:
            recording.start_imu()
            print("IMU", recording.calibrations[-1], flush=True)
        if not args.run:
            if args.imu:
                recording.collect(args.settle, active=False)
            report["result"] = "read_only_plan"
            return
        # The existing endpoint captures a fresh position and ramps support.
        # Do not call the RPC client directly: its TX thread owns sequencing.
        armed = True
        recording.mark("capture_hold")
        # The queued capture may clear an existing, acknowledged F6 lock.
        # Do not treat its pre-command telemetry as a new active-motion fault.
        status = recording.capture_hold()
        recording.pump(active=True)
        captured = [j["target_reference"] for j in sorted(
            status["joints"], key=lambda j: j["id"])]
        plan = make_plan(captured, bounds, args.amplitude, args.speed,
                         args.acceleration, args.jerk,
                         axes=(args.joint-1,) if args.joint else (0, 1),
                         center_j1=args.center_j1, center_j2=args.center_j2,
                         bidirectional=args.bidirectional, repetitions=args.repetitions,
                         coupled=args.coupled)
        report["captured_target_rad"], report["plan"] = captured, plan
        report["capture_ack"] = recording.motion_acks[-1]
        recording.mark("baseline_settle")
        recording.collect(args.settle)
        for job in plan:
            recording.mark(job["name"], **{k: v for k, v in job.items()
                                          if k not in ("name", "request")})
            recording.action("motion", job["request"])
            # Completion means reference completion, not zero physical error.
            # This deliberately permits compliance and measures the static error.
            distance = max(abs(a-b) for a, b in zip(
                job["target_rad"],
                [j["target_reference"] for j in sorted(
                    recording.reader.telemetry["joints"], key=lambda j: j["id"])]))
            timeout = 8 + 2*distance/args.speed + 4*args.acceleration/args.jerk
            recording.wait(lambda s: holding_at(s, job["target_rad"]), timeout, active=True)
            recording.mark(job["name"]+"_settle")
            recording.collect(args.settle)
        recording.mark("complete_returned_to_captured_target")
        report["result"] = "complete_holding"
    except (Exception, KeyboardInterrupt) as exc:
        report["result"] = "interrupted" if isinstance(exc, KeyboardInterrupt) else "failed"
        report["error"] = f"{type(exc).__name__}: {exc}"
        print(report["error"], flush=True)
        if armed and not recording.reader.stop_requested.is_set():
            # One normal reference-braking STOP. No automatic fault clear,
            # re-enable, repeated action or F6 (which would remove support).
            try:
                recording.mark("abort_stop_requested")
                recording.action("command", (2, 0))
                recording.wait(lambda s: holding_at(s), 6, active=True)
                report["abort_stop"] = "holding_observed"
            except Exception as stop_error:
                report["abort_stop"] = f"not_confirmed: {stop_error}"
    finally:
        try:
            recording.pump()
        except Exception as exc:
            report.setdefault("closing_messages", []).append(str(exc))
        report["final_status"] = recording.reader.telemetry
        report["final_status_age_s"] = time.perf_counter()-recording.reader.telemetry_at
        recording.reader.stop_requested.set()
        recording.reader.join(3)
        if recording.reader.is_alive():
            report["closing_messages"] = ["Reader thread did not finish within 3 seconds"]
        # Drain samples already queued while the reader was being closed.
        while not recording.samples.empty():
            recording.records.append(recording.samples.get_nowait())
        report["statistics"] = recording.reader.statistics()
        report["markers"] = recording.markers
        report["messages"] = recording.messages
        report["imu_snapshots"] = recording.calibrations
        report["settled_measurements"] = summarize(recording.records, recording.markers)
        report["samples"] = len(recording.records)
        dm_monitor.write_csv(output / "samples.csv", recording.records)
        (output / "report.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
        print("Recording:", output, "result:", report["result"], flush=True)
    if report["result"] != "complete_holding":
        raise SystemExit(1)


def self_test():
    recording = Recording("unused-self-test")
    recording.events.put(("calibration_status", dict(
        at=recording.origin, accel=[0, 0, 0])))
    recording.pump()
    assert recording.calibrations[0]["roll_pitch_deg"] == [None, None]
    json.dumps(recording.calibrations, allow_nan=False)
    origin = [-1., -2.]
    plan = make_plan(origin, [(-3, 0), (-3.5, -1)], 3, .05, 2, 4)
    assert len(plan) == 6 and plan[-1]["target_rad"] == origin
    assert plan[2]["target_rad"] == origin
    assert math.isclose(plan[1]["target_rad"][0], -1-math.radians(3))
    centered = make_plan(origin, [(-3, 0), (-3.5, -1)], 20, .2, 2, 8,
                         axes=(1,), center_j2=-151.2828)
    assert len(centered) == 5 and centered[-1]["target_rad"] == origin
    assert all(job["target_rad"][0] == origin[0] for job in centered)
    paired = make_plan(origin, [(-3, 0), (-3.5, -1)], 8, .2, 2, 8,
                       center_j1=-60, center_j2=-140,
                       bidirectional=True, repetitions=3)
    returns = [job for job in paired if "test_joint" in job]
    assert len(returns) == 12 and len(paired) == 26
    assert all(job["target_rad"] == job["center_rad"] for job in returns)
    assert [j["from_offset_sign"] for j in returns[:6]] == [1, -1, -1, 1, 1, -1]
    assert paired[-1]["target_rad"] == origin
    diagonal = make_plan(origin, [(-3, 0), (-3.5, -1)], 8, .4, 2, 8,
                         center_j1=-60, center_j2=-140, coupled=True)
    assert len(diagonal) == 10
    assert [j["from_offset_signs"] for j in diagonal if "test_joints" in j] == [
        [1, 1], [-1, -1], [1, -1], [-1, 1]]
    assert all(j["target_rad"] == j["center_rad"] for j in diagonal if "test_joints" in j)
    for amplitude, speed, accel, jerk in ((200, .05, 2, 4), (3, 17, 2, 4),
                                         (3, .05, 2.1, 4), (3, .05, 2, 0)):
        try:
            make_plan(origin, [(-3, 0), (-3.5, -1)], amplitude, speed, accel, jerk)
        except ValueError:
            pass
        else:
            raise AssertionError("Invalid experiment accepted")
    s = dict(mode=1, state=4, phase=4,
             sequence=10,
             joints=[dict(id=i+1, target_reference=q) for i, q in enumerate(origin)])
    assert holding_at(s, origin)
    assert not holding_at(s, [-1.1, -2])
    assert not holding_at(dict(s, state=5), origin)
    assert not capture_completed(s, [], 0)
    acks = [dict(kind="impedance", sequence=10)]
    assert not capture_completed(s, acks, 0)
    assert capture_completed(dict(s, sequence=11), acks, 0)
    assert not capture_completed(dict(s, sequence=11), acks, 1)
    assert not capture_completed(dict(s, sequence=9), acks, 0)
    assert not capture_completed(dict(s, sequence=11, phase=1), acks, 0)
    assert capture_completed(dict(s, sequence=1),
                             [dict(kind="impedance", sequence=0xFFFFFFFF)], 0)
    print("Identification plan self-test passed (no hardware)")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM34")
    parser.add_argument("--run", action="store_true", help="Capture/enable and execute the plan")
    parser.add_argument("--imu", action="store_true",
                        help="Explicitly start/read IMU; does not overwrite the modeling zero")
    parser.add_argument("--amplitude", type=float, default=3, help="Degrees, relative to captured point")
    parser.add_argument("--speed", type=float, default=.05, help="Test velocity in rad/s")
    parser.add_argument("--acceleration", type=float, default=2, help="rad/s^2; user limit is 2")
    parser.add_argument("--jerk", type=float, default=4, help="Test profile rad/s^3, not a hardware rating")
    parser.add_argument("--settle", type=float, default=2, help="Seconds of post-reference observation")
    parser.add_argument("--joint", type=int, choices=(0, 1, 2), default=0,
                        help="0=both, otherwise excite only the selected joint")
    parser.add_argument("--center-j2", type=float, help="J2 experiment center in reference degrees")
    parser.add_argument("--center-j1", type=float, help="J1 experiment center in reference degrees")
    parser.add_argument("--bidirectional", action="store_true",
                        help="Return to the SAME center from both sides; not a manual release test")
    parser.add_argument("--repetitions", type=int, default=1,
                        help="Paired trials per joint; alternates approach order")
    parser.add_argument("--coupled", action="store_true",
                        help="Both joints together: ++, --, +-, -+ offsets and same-center returns")
    parser.add_argument("--output")
    parser.add_argument("--self-test", action="store_true")
    arguments = parser.parse_args()
    if arguments.self_test:
        self_test()
    else:
        if not math.isfinite(arguments.settle) or arguments.settle < 0:
            parser.error("--settle must be finite and nonnegative")
        run(arguments)
