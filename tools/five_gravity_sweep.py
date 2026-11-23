"""Explicit J1..J5 bidirectional commissioning sweep. No automatic enable/reset/ACK."""
import argparse
import json
import math
import time
from pathlib import Path

from dm_monitor import JOINT_REFERENCES
from five_control import Board, CONTROL_VERSION, HISTORY_VERSION, gravity_output_confirmed, idle_output_confirmed
from usb_monitor import serial_port


def angles(state, board=None):
    if board is not None:
        return [math.degrees(board.scalar(16, joint)-ref.zero_rad)
                for joint, ref in JOINT_REFERENCES.items()]
    return [math.degrees(JOINT_REFERENCES[j["joint"]].angle(j["position_rad"]))
            for j in state["telemetry"]["joints"]]


def healthy(state):
    return (not state["fault"] and not state["limited"] and
            all(state[k] == 31 for k in ("fresh", "enabled", "submitted")))

def baseline_goal(current, joint, speed, plateau_ms, ramp_ms, direction=None):
    """One single-axis finite velocity trial, preserving actual other-axis pose."""
    if (len(current) != 5 or not all(map(math.isfinite, current)) or joint not in range(1, 6)
            or not math.isfinite(speed) or speed <= 0 or plateau_ms <= 0 or ramp_ms <= 0
            or not 100 <= plateau_ms+2*ramp_ms <= 120000):
        raise ValueError("Invalid velocity baseline")
    ref = JOINT_REFERENCES[joint]
    low, high = math.degrees(ref.lower_rad)+1.1, math.degrees(ref.upper_rad)-1.1
    start = current[joint-1]
    delta = math.degrees(speed)*(plateau_ms+ramp_ms)/1000
    if direction is None:
        direction = 1 if high-start >= start-low else -1
    if direction not in (-1, 1) or not low <= start+direction*delta <= high:
        raise ValueError(f"J{joint}: baseline leg does not fit retained raw travel; no motion sent")
    goal = current.copy()
    goal[joint-1] += direction*delta
    return goal, direction, plateau_ms+2*ramp_ms


def endpoint_goal(current, joint, endpoint_reference_deg, speed, ramp_ms):
    """Recompute a finite leg from the latest pose, without relaxing travel."""
    if (joint not in range(1, 6) or len(current) != 5
            or not all(map(math.isfinite, current))
            or not math.isfinite(endpoint_reference_deg)
            or not math.isfinite(speed) or speed <= 0 or ramp_ms <= 0):
        raise ValueError("Invalid endpoint leg")
    ref = JOINT_REFERENCES[joint]
    if not (math.degrees(ref.raw_lower_rad)+1.1 <= endpoint_reference_deg
            <= math.degrees(ref.raw_upper_rad)-1.1):
        raise ValueError("Endpoint outside retained raw travel")
    delta = endpoint_reference_deg-math.degrees(ref.zero_rad)-current[joint-1]
    plateau = math.floor(math.radians(abs(delta))/speed*1000)-ramp_ms
    return baseline_goal(current, joint, speed, plateau, ramp_ms, 1 if delta > 0 else -1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--log", required=True, type=Path)
    speed = parser.add_mutually_exclusive_group()
    speed.add_argument("--peak-speed", type=float, default=.16, help="original full-sweep quintic peak rad/s")
    speed.add_argument("--constant-speed", type=float,
                       help="first-round baseline: finite velocity PI, one forward/reverse pair per axis, rad/s")
    parser.add_argument("--plateau-ms", type=int, default=3000)
    parser.add_argument("--ramp-ms", type=int, default=1000)
    parser.add_argument("--single-leg", action="store_true",
                        help="one finite velocity leg on one joint; same gravity handover")
    parser.add_argument("--direction", type=int, choices=(-1, 1),
                        help="explicit direction for --single-leg only")
    parser.add_argument("--target-reference-deg", type=float,
                        help="single-leg endpoint; recompute duration from the final fresh pose")
    parser.add_argument("--joints", type=int, nargs="+", choices=range(1, 6),
                        default=[1, 2, 3, 4, 5], help="explicit test order; never retries a failed axis")
    parser.add_argument("--free-space", action="store_true",
                        help="ONLY after operator confirms no support/contact during the entire capture")
    parser.add_argument("--continuous-reference", action="store_true",
                        help="use the history-capable board's continuous angle for targets; raw feedback stays unchanged")
    args = parser.parse_args()
    if args.single_leg and (args.constant_speed is None or len(args.joints)!=1):
        parser.error("--single-leg needs --constant-speed and exactly one joint")
    if args.direction is not None and not args.single_leg:
        parser.error("--direction is only for --single-leg")
    endpoint = getattr(args, "target_reference_deg", None)
    if endpoint is not None and (not args.single_leg or not math.isfinite(endpoint)):
        parser.error("--target-reference-deg needs a finite single-leg endpoint")
    if not math.isfinite(args.peak_speed) or not .02 <= args.peak_speed <= .2:
        parser.error("--peak-speed must be .02..0.2 rad/s")
    if args.constant_speed is not None and (
            not math.isfinite(args.constant_speed) or args.constant_speed <= 0 or
            args.plateau_ms <= 0 or args.ramp_ms <= 0 or
            not 100 <= args.plateau_ms+2*args.ramp_ms <= 120000):
        parser.error("invalid finite velocity baseline")
    with args.log.open("x", encoding="utf-8") as log, serial_port(args.port) as port:
        def note(**record):
            print(json.dumps(record, allow_nan=False, separators=(",", ":")), file=log,
                  flush=record.get("stage") != "feedback")

        board = Board(port)
        board.connect()
        if args.continuous_reference and board.rpc(250) != HISTORY_VERSION:
            raise ValueError("Firmware has no matching continuous-reference capability")
        pose_board = board if args.continuous_reference else None
        refs = [[board.scalar(f, j, op=15) for f in (1, 2, 3)] for j in range(1, 6)]
        expected = [[r.zero_rad, r.raw_lower_rad, r.raw_upper_rad]
                    for r in JOINT_REFERENCES.values()]
        if any(abs(a-b) > 2e-6 for row, want in zip(refs, expected)
               for a, b in zip(row, want)):
            raise ValueError("Board references differ from current five-axis source")
        note(stage="capture", schema="ylamp-five-id-1", control_version=CONTROL_VERSION,
             action="move", session=board.session.hex(), free_space=args.free_space,
             references_rad=refs, order=args.joints,
             peak_speed_rad_s=args.peak_speed if args.constant_speed is None else None,
             constant_speed_rad_s=args.constant_speed,
             target_coordinates="board_continuous_reference" if args.continuous_reference else "raw_reference",
             command_components_same_tick=False, actual_speed_requires_validation=True)
        board.on_feedback = lambda sample: note(stage="feedback", telemetry=sample)
        state = board.status()
        note(stage="before", **state)
        resume_drag = state["phase"] == "DRAG"
        state = board.test_gravity(state)
        note(stage="test_gravity_handover", **state)
        try:
            for joint in args.joints:
                ref = JOINT_REFERENCES[joint]
                # Keep original references/turns. The retained native target inset is 1 degree.
                start = angles(state, pose_board)[joint-1]
                low, high = math.degrees(ref.lower_rad)+1.1, math.degrees(ref.upper_rad)-1.1
                direction = args.direction if args.single_leg else None
                targets = ((low, high, low, start) if args.constant_speed is None else
                           (None,) if args.single_leg else (None, None))
                for leg, target in enumerate(targets, 1):
                    state = board.status()
                    if not healthy(state):
                        raise RuntimeError("Controller fault before next waypoint")
                    current = angles(state, pose_board)
                    ramp = 0
                    if args.constant_speed is not None:
                        if endpoint is not None:
                            goal, direction, milliseconds = endpoint_goal(
                                current, joint, endpoint, args.constant_speed, args.ramp_ms)
                        else:
                            goal, direction, milliseconds = baseline_goal(
                                current, joint, args.constant_speed, args.plateau_ms, args.ramp_ms,
                                direction if args.single_leg else None if leg == 1 else -direction)
                        ramp = args.ramp_ms
                    else:
                        goal = current.copy()
                        goal[joint-1] = target
                        milliseconds = max(3000, math.ceil(
                            1.875*abs(target-current[joint-1])*math.pi/180/args.peak_speed*1000))
                    if milliseconds > 120000:
                        raise ValueError("Requested peak speed exceeds native 120 s segment duration")
                    note(stage="waypoint", joint=joint, leg=leg, start_software_deg=current,
                         target_software_deg=goal, duration_ms=milliseconds, ramp_ms=ramp,
                         constant_velocity_rad_s=direction*args.constant_speed if ramp else None)
                    if ramp:
                        board.speed(joint, direction*args.constant_speed, milliseconds, ramp)
                    else:
                        board.move(goal, milliseconds)  # Exactly once; no replay.
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
                    actual = angles(state, pose_board)
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
            if resume_drag:
                board.drag()
                state = board.observe_transition(("GRAVITY",))
                note(stage="manual_handover", **state)
                if state["phase"] != "DRAG" or not idle_output_confirmed(state):
                    raise RuntimeError("Final manual-idle handover unconfirmed")
            note(stage="end", tick_ms=board.latest["tick_ms"], malformed=board.stream.malformed,
                 incomplete=board.collector.incomplete)
            kind = "single leg" if args.single_leg else "bidirectional sweep"
            print(f"Completed {args.joints} {kind}; returned to {state['phase']}.", flush=True)
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
