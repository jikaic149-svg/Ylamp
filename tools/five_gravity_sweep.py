"""Explicit J1..J5 bidirectional commissioning sweep. No automatic enable/reset/ACK."""
import argparse
import json
import math
import struct
import time
from pathlib import Path

from dm_monitor import JOINT_REFERENCES
from five_control import Board, CONTROL_VERSION, gravity_output_confirmed
from usb_monitor import serial_port


def angles(state):
    return [math.degrees(JOINT_REFERENCES[j["joint"]].angle(j["position_rad"]))
            for j in state["telemetry"]["joints"]]


def healthy(state):
    return (not state["fault"] and not state["limited"] and
            all(state[k] == 31 for k in ("fresh", "enabled", "submitted")))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--log", required=True, type=Path)
    parser.add_argument("--peak-speed", type=float, default=.16, help="quintic peak rad/s")
    parser.add_argument("--joints", type=int, nargs="+", choices=range(1, 6),
                        default=[1, 2, 3, 4, 5], help="explicit test order; never retries a failed axis")
    parser.add_argument("--free-space", action="store_true",
                        help="ONLY after operator confirms no support/contact during the entire capture")
    args = parser.parse_args()
    if not math.isfinite(args.peak_speed) or not .02 <= args.peak_speed <= .2:
        parser.error("--peak-speed must be .02..0.2 rad/s")
    with args.log.open("x", encoding="utf-8") as log, serial_port(args.port) as port:
        def note(**record):
            print(json.dumps(record, allow_nan=False, separators=(",", ":")), file=log,
                  flush=record.get("stage") != "feedback")

        board = Board(port)
        board.connect()
        refs = [[board.scalar(f, j, op=15) for f in (1, 2, 3)] for j in range(1, 6)]
        expected = [[r.zero_rad, r.raw_lower_rad, r.raw_upper_rad]
                    for r in JOINT_REFERENCES.values()]
        if any(abs(a-b) > 2e-6 for row, want in zip(refs, expected)
               for a, b in zip(row, want)):
            raise ValueError("Board references differ from current five-axis source")
        note(stage="capture", schema="ylamp-five-id-1", control_version=CONTROL_VERSION,
             action="move", session=board.session.hex(), free_space=args.free_space,
             references_rad=refs, order=args.joints, peak_speed_rad_s=args.peak_speed)
        board.on_feedback = lambda sample: note(stage="feedback", telemetry=sample)
        state = board.status()
        note(stage="before", **state)
        if not gravity_output_confirmed(state):
            raise RuntimeError("Five-axis gravity not healthy; no automatic recovery")
        try:
            for joint in args.joints:
                ref = JOINT_REFERENCES[joint]
                # Keep original references/turns. The retained native target inset is 1 degree.
                start = angles(state)[joint-1]
                low, high = math.degrees(ref.lower_rad)+1.1, math.degrees(ref.upper_rad)-1.1
                for leg, target in enumerate((low, high, low, start), 1):
                    state = board.status()
                    if not healthy(state):
                        raise RuntimeError("Controller fault before next waypoint")
                    current = angles(state)
                    goal = current.copy()
                    goal[joint-1] = target
                    milliseconds = max(3000, math.ceil(
                        1.875*abs(target-current[joint-1])*math.pi/180/args.peak_speed*1000))
                    if milliseconds > 120000:
                        raise ValueError("Requested peak speed exceeds native 120 s segment duration")
                    note(stage="waypoint", joint=joint, leg=leg, start_software_deg=current,
                         target_software_deg=goal, duration_ms=milliseconds)
                    for j, degree in enumerate(goal, 1):
                        board.rpc(5, j, struct.unpack("<I", struct.pack("<f", degree))[0])
                    board.rpc(6, value=milliseconds)  # Exactly once; no replay.
                    note(stage="rpc_accepted", action="move", joint=joint, leg=leg)
                    deadline = time.monotonic()+milliseconds/1000+1.5
                    progress = 0.
                    state = board.observe_transition(("GRAVITY", "POSITION"))
                    while True:
                        note(stage="sample", **state)
                        if not healthy(state) or state["phase"] not in ("MOTION", "POSITION"):
                            raise RuntimeError(f"J{joint} leg {leg}: interrupted, see capture")
                        now = time.monotonic()
                        if now >= progress:
                            actual = state["telemetry"]["joints"][joint-1]
                            print(json.dumps(dict(joint=joint, leg=leg, phase=state["phase"],
                                  raw_deg=math.degrees(actual["position_rad"]),
                                  velocity_rad_s=actual["velocity_rad_s"],
                                  torque_nm=actual["torque_nm"])), flush=True)
                            progress = now+3
                        if now >= deadline:
                            break
                        until = min(deadline, now+.12)
                        while time.monotonic() < until:
                            board.pump()
                        state = board.status()
                    if state["phase"] != "POSITION":
                        raise RuntimeError("Trajectory time did not complete")
                    actual = angles(state)
                    note(stage="trajectory_result", joint=joint, leg=leg,
                         target_software_deg=goal, actual_software_deg=actual,
                         error_deg=[goal[i]-actual[i] for i in range(5)],
                         physical_arrival_claimed=False)
                    # End this segment's active hold before admitting the next
                    # one. Holding-PD velocity ripple is not a new sweep target.
                    board.rpc(7)
                    state = board.observe_transition(("POSITION", "MOTION"))
                    note(stage="waypoint_handover", joint=joint, leg=leg, **state)
                    if not gravity_output_confirmed(state):
                        raise RuntimeError("Waypoint gravity handover unconfirmed")
            board.rpc(7)
            state = board.observe_transition(("POSITION", "MOTION"))
            note(stage="handover", **state)
            if not gravity_output_confirmed(state):
                raise RuntimeError("Final gravity handover unconfirmed")
            note(stage="end", tick_ms=board.latest["tick_ms"], malformed=board.stream.malformed,
                 incomplete=board.collector.incomplete)
            print(f"Completed {args.joints} bidirectional sweep; returned to pure gravity.", flush=True)
        except BaseException as exc:
            note(stage="abort", error=str(exc))
            try:
                board.rpc(7)  # Cancel to existing G; never clear faults or enable.
                note(stage="abort_gravity",
                     **board.observe_transition(("POSITION", "MOTION")))
            except Exception as cancel_error:
                note(stage="cancel_unconfirmed", error=str(cancel_error))
            raise


if __name__ == "__main__":
    main()
