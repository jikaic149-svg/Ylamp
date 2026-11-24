"""Explicit native-MIT hold/finite commissioning; never enables, clears or retries."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import subprocess
import time

import numpy as np

from check_v12_image import support_bytes
from five_control import Board, IMPEDANCE_TEST_VERSION, idle_output_confirmed
from five_friction_campaign import GEOMETRY
from five_gravity_residual_report import support_gravity
from five_identify import HW, REFS, ROOT
from usb_monitor import serial_port


def drain_observation(board):
    """After CAD computation, consume queued telemetry before a fresh command.

    No pending RPC here. Preserve received records; never replay a request.
    """
    deadline = time.monotonic()+.7
    while board.port.in_waiting:
        if time.monotonic()>deadline:
            raise TimeoutError("Telemetry backlog unresolved; no motion sent")
        board.pump()


def segmented_paths(points, step_deg=.25):
    paths = []
    for a, b in zip(points, points[1:]):
        a, b = np.asarray(a, float), np.asarray(b, float)
        samples = np.linspace(a, b, max(1, math.ceil(max(abs(b-a))/step_deg))+1)
        paths.extend([[x.tolist(), y.tolist()] for x, y in zip(samples, samples[1:])])
    return paths


def planned_paths(center, joint, offset):
    center = np.asarray(center, float)
    if (center.shape != (5,) or not np.isfinite(center).all() or joint not in range(1, 6)
            or not math.isfinite(offset) or not offset):
        raise ValueError("Invalid finite trial geometry")
    points = [center.copy() for _ in range(5)]
    points[1][joint-1] += offset
    points[3][joint-1] -= offset
    lo, hi = np.rad2deg(REFS[:, 1:]).T + np.array([[1.], [-1.]])
    if any(np.any(p < lo) or np.any(p > hi) for p in points):
        raise ValueError("Trial does not fit original 1-degree target inset; no motion sent")
    # Feed short segments to the original finite CAD checker; do not change
    # its mounting-surface rules, whole-turn basis or pretend continuous proof.
    return segmented_paths(points)


def geometry_check(geometry, paths):
    shift = HW[:, 6]-np.rad2deg(REFS[:, 0])
    encoded = [[(np.asarray(x)+shift).tolist() for x in pair] for pair in paths]
    geometry.stdin.write(json.dumps(encoded)+"\n");geometry.stdin.flush()
    result = json.loads(geometry.stdout.readline())
    poses = np.array([point for pair in encoded for point in pair])
    demand = np.max(abs(support_gravity(np.deg2rad(poses), clip=False)), axis=0)
    return result, demand


def return_metrics(samples, center, start, ramp, hold, offset_deg=None):
    values = np.asarray(samples, float)
    if (values.ndim != 2 or values.shape[1] != 5 or not len(values)
            or not np.isfinite(values).all()):
        raise ValueError("No actual feedback samples")
    # [source time, source count, raw angle rad, speed rad/s, estimated Nm].
    elapsed = (values[:, 0]-start) % 2**32
    summaries = []
    for leg in (1, 3):
        begin, end = leg*(ramp+hold), (leg+1)*(ramp+hold)
        returning = values[(elapsed >= begin) & (elapsed < end)]
        times = elapsed[(elapsed >= begin) & (elapsed < end)]-begin
        before = values[(elapsed >= begin-250) & (elapsed < begin)]
        initial = math.degrees(float(np.median(before[:, 2]))-center) if len(before)>=5 else None
        window = values[(elapsed >= end-min(1000, hold)) & (elapsed < end)]
        if len(window) < 5:
            raise ValueError("Insufficient source feedback in center hold")
        error = np.rad2deg(window[:, 2]-center)
        all_error = np.rad2deg(returning[:, 2]-center)
        source_gap = float(max(np.diff(times), default=0))
        covered = bool(len(times)>1 and times[0]<=100 and times[-1]>=end-begin-100 and source_gap<=100)
        direction = (math.copysign(1., offset_deg)*(1 if leg==1 else -1)
                     if offset_deg else math.copysign(1., initial or 1))
        settle = {}
        for band in (1, 2):
            outside = np.flatnonzero(abs(all_error)>band)
            index = int(outside[-1])+1 if len(outside) else 0
            settle[str(band)] = (float(times[index]*.001)
                if covered and index<len(times) and end-begin-times[index]>=500 else None)
        summaries.append(dict(samples=len(window), median_error_deg=float(np.median(error)),
            absolute_error_p95_deg=float(np.percentile(abs(error), 95)),
            absolute_error_max_deg=float(max(abs(error))),
            peak_to_peak_deg=float(np.ptp(error)),
            velocity_rms_rad_s=float(np.sqrt(np.mean(window[:, 3]**2))),
            estimated_torque_median_nm=float(np.median(window[:, 4])),
            initial_displacement_deg=initial,
            peak_return_overshoot_deg=(float(max(0., max(-direction*all_error)))
                                       if len(all_error) else None),
            return_peak_estimated_torque_nm=(float(max(abs(returning[:, 4])))
                                             if len(returning) else None),
            settling_time_s_by_band_deg=settle, return_source_max_gap_ms=source_gap,
            return_coverage_sufficient_for_settling=covered,
            steady_p95_within_1_deg=bool(np.percentile(abs(error),95)<=1),
            steady_p95_within_2_deg=bool(np.percentile(abs(error),95)<=2)))
    difference = summaries[0]["median_error_deg"]-summaries[1]["median_error_deg"]
    return dict(after_plus_target=summaries[0], after_minus_target=summaries[1],
        return_difference_deg=difference, side_difference_within_1_deg=bool(abs(difference)<=1),
        side_difference_within_2_deg=bool(abs(difference)<=2),
        peak_estimated_torque_nm=float(max(abs(values[:, 4]))),
        actual_offset_range_deg=np.rad2deg([values[:, 2].min()-center,
                                          values[:, 2].max()-center]).tolist(),
        feedback_resolution_deg=math.degrees(25/65535),
        independent_output_angle_measured=False, external_release_test=False,
        position_acceptance_claimed=False)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", required=True)
    p.add_argument("--log", type=Path, required=True)
    p.add_argument("--firmware", type=Path, required=True, help="declared local image, not an on-board hash query")
    action = p.add_mutually_exclusive_group(required=True)
    action.add_argument("--hold-only", action="store_true")
    action.add_argument("--preflight-only", action="store_true")
    action.add_argument("--execute", action="store_true")
    p.add_argument("--released", action="store_true", help="operator has released and left planned travel")
    p.add_argument("--seconds", type=float, default=2, help="static native hold duration")
    p.add_argument("--joints", type=int, nargs="+", default=[1, 2, 3, 4, 5])
    p.add_argument("--offset-deg", type=float, default=20.)
    p.add_argument("--offsets-deg", type=float, nargs=5, help="J1..J5 requested amplitudes")
    p.add_argument("--plan-center-reference-deg", type=float, nargs=5,
                   help="preflight ONLY: prospective continuous raw center, never moves there")
    a = p.parse_args()
    config = support_bytes(manual_drag=True)
    ramp, hold, maximum = struct.unpack("<IIf", config["g_five_impedance_test"])
    offsets = a.offsets_deg if a.offsets_deg is not None else [a.offset_deg]*5
    if (a.execute and not a.released or not math.isfinite(a.seconds) or not 0<a.seconds<=60
            or not a.joints or len(a.joints)!=len(set(a.joints))
            or any(j not in range(1, 6) for j in a.joints)
            or any(not math.isfinite(v) or not 0<abs(v)<=maximum for v in offsets)
            or a.plan_center_reference_deg is not None and
                (not a.preflight_only or not all(map(math.isfinite,a.plan_center_reference_deg)))):
        p.error("Valid finite trials require unique J1..J5, configured offset and --released for execution")
    digest = hashlib.sha256(a.firmware.read_bytes()).hexdigest()
    with a.log.open("x", encoding="utf-8") as log:
        def note(**row):
            print(json.dumps(dict(unix_s=time.time(), **row), allow_nan=False), file=log,
                  flush=row.get("stage")!="feedback")

        note(stage="capture", schema="ylamp-native-impedance-test-2",
             declared_local_firmware_sha256=digest, free_space=False,
             operator_released_at_start=a.released, command_components_same_tick=False,
             planned_trial_capability=IMPEDANCE_TEST_VERSION,
             tool_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        with serial_port(a.port) as port:
            b = Board(port)
            b.connect()
            samples, active, last_count = [], 0, None
            torque_peak = np.zeros(5)

            def feedback(sample):
                nonlocal last_count
                note(stage="feedback", telemetry=sample)
                if active:
                    torque_peak[:] = np.maximum(torque_peak,
                        [abs(v["torque_nm"]) for v in sample["joints"]])
                    j = sample["joints"][active-1]
                    if j["received_count"] != last_count:
                        last_count = j["received_count"]
                        samples.append([j["received_tick_ms"], j["received_count"],
                                        j["position_rad"], j["velocity_rad_s"], j["torque_nm"]])

            b.on_feedback = feedback
            initial = b.status()
            note(stage="before", **initial)
            live_capability = b.rpc(249)
            note(stage="capability", actual=live_capability, planned=IMPEDANCE_TEST_VERSION)
            if not idle_output_confirmed(initial) or (
                    not a.preflight_only and live_capability!=IMPEDANCE_TEST_VERSION):
                raise RuntimeError("Healthy idle/matching capability absent; no recovery")
            if not a.preflight_only:
                b.impedance()
                state = b.observe_transition(("GRAVITY", "DRAG"))
                if state["phase"]!="IMPEDANCE" or state["fault"]:
                    raise RuntimeError("Native hold entry unconfirmed; no replay")
                note(stage="native_entry", native=b.impedance_status(), **state)
            if a.hold_only:
                deadline = time.monotonic()+a.seconds
                while time.monotonic()<deadline:
                    state = b.status()
                    if state["phase"]!="IMPEDANCE" or state["fault"] or state["fresh"]!=31:
                        raise RuntimeError("Native hold interrupted; no recovery")
                    note(stage="sample", native=b.impedance_status(), **state)
                    until = min(deadline, time.monotonic()+.12)
                    while time.monotonic()<until:
                        b.pump()
            else:
                with subprocess.Popen(["node", "--input-type=module", "-e", GEOMETRY],
                        cwd=ROOT/"tools/urdf_collision", stdin=subprocess.PIPE,
                        stdout=subprocess.PIPE, text=True, encoding="utf-8") as geometry:
                    while True:
                        line = geometry.stdout.readline()
                        if not line:
                            raise RuntimeError("Geometry loader failed")
                        if line.startswith("{"):
                            note(stage="geometry_input", **json.loads(line));break
                    try:
                        for joint in a.joints:
                            reference = np.rad2deg([b.scalar(16, j) for j in range(1, 6)])
                            state = b.status()
                            target = ([j["position_rad"] for j in state["telemetry"]["joints"]]
                                      if a.preflight_only else b.impedance_status()["target_raw_rad"])
                            raw = np.array([j["position_rad"] for j in state["telemetry"]["joints"]])
                            center = reference+np.rad2deg(np.asarray(target)-raw)
                            if a.plan_center_reference_deg is not None:
                                candidate = np.asarray(a.plan_center_reference_deg)
                                if joint==a.joints[0]:
                                    prep = segmented_paths([center, candidate])
                                    result, demand = geometry_check(geometry, prep)
                                    note(stage="preparation_preflight", from_reference_deg=center.tolist(),
                                         to_reference_deg=candidate.tolist(), no_motion_sent=True,
                                         unclipped_max_gravity_nm=demand.tolist(), **result)
                                    if result["witness"] or np.any(demand>HW[:, 5]):
                                        raise ValueError("Preparation CAD/G rejected; no motion sent")
                                    drain_observation(b)
                                center = candidate
                            amplitude = offsets[joint-1]
                            paths = planned_paths(center, joint, amplitude)
                            result, demand = geometry_check(geometry, paths)
                            note(stage="preflight", joint=joint, reference_center_deg=center.tolist(),
                                 measured_native_start_raw_rad=list(target), offset_deg=amplitude,
                                 max_segment_deg=.25, unclipped_max_gravity_nm=demand.tolist(), **result)
                            if result["witness"] or np.any(demand>HW[:, 5]):
                                raise ValueError("Finite CAD/G preflight rejected; no motion sent")
                            drain_observation(b)
                            if a.preflight_only:
                                continue
                            samples.clear();active=joint;last_count=None;torque_peak[:]=0
                            limited_mask=0
                            duration = b.impedance_test(joint, amplitude)
                            if duration!=4*(ramp+hold):
                                raise RuntimeError("Target duration differs; no replay")
                            native = b.impedance_status()
                            start = native["test_started_ms"]
                            note(stage="cycle_started", joint=joint, native=native, duration_ms=duration)
                            deadline = time.monotonic()+duration*.001+2
                            while True:
                                state = b.status()
                                if (state["phase"]!="IMPEDANCE" or state["fault"] or
                                        state["fresh"]!=31 or state["enabled"]!=31):
                                    raise RuntimeError("Native cycle interrupted; no recovery")
                                native = b.impedance_status()
                                limited_mask |= state["limited"]
                                note(stage="sample", native=native, **state)
                                if native["test_started_ms"]!=start:
                                    raise RuntimeError("Trial identity changed; no replay")
                                if not native["test_joint"]:
                                    if (b.latest["tick_ms"]-start)%2**32<duration:
                                        raise RuntimeError("Trial cancelled before target-time completion")
                                    break
                                if time.monotonic()>deadline:
                                    raise TimeoutError("Target completion unconfirmed; no replay")
                                until = time.monotonic()+.12
                                while time.monotonic()<until:
                                    b.pump()
                            note(stage="cycle_result", joint=joint,
                                 metrics=return_metrics(samples, target[joint-1], start, ramp, hold, amplitude),
                                 peak_driver_estimated_torque_nm_all_axes=torque_peak.tolist(),
                                 software_impedance_budget_nm=list(struct.unpack("<5f",
                                     config["g_five_impedance_max_nm"])),
                                 software_pd_budget_nm=list(struct.unpack("<5f",
                                     config["g_five_impedance_pd_max_nm"])),
                                 sampled_limited_mask=limited_mask,
                                 torque_measurement_is_independent=False)
                            active=0
                            print(f"J{joint} native finite cycle captured", flush=True)
                    finally:
                        geometry.stdin.close()
                        geometry.wait(timeout=10)
            if not a.preflight_only:
                b.rpc(7)  # Explicit normal completion handover, never fault ACK.
                state = b.observe_transition(("IMPEDANCE",))
                if initial["phase"]=="DRAG":
                    b.drag();state=b.observe_transition(("GRAVITY",))
                note(stage="handover", **state)
                if not idle_output_confirmed(state):
                    raise RuntimeError("Idle handover unconfirmed; no recovery")
            else:
                drain_observation(b)
                state=b.status()
                note(stage="preflight_complete", no_motion_sent=True, **state)
                if not idle_output_confirmed(state):
                    raise RuntimeError("Idle changed during preflight; no recovery")
            note(stage="end", tick_ms=b.latest["tick_ms"], malformed=b.stream.malformed,
                 incomplete=b.collector.incomplete, observed_gaps=b.collector.gaps,
                 resyncs=b.collector.resyncs)
            print(json.dumps(state, allow_nan=False))


if __name__=="__main__":
    main()
