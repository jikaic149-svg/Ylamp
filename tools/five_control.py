"""Board02 five-axis USB operator/test client. No disable, reset or automatic enable."""
import argparse
import json
import math
import secrets
import struct
import time
from contextlib import nullcontext
from pathlib import Path

from dm_serial import encode, SESSION, SESSION_ACK, RELIABLE_REQUEST, RELIABLE_RESPONSE
from dm_serial import TRAJ_BEGIN, TRAJ_POINT, TRAJ_START, MOTION_QUERY, TRAJ_ACK, TRAJ_RESULT, MOTION_STATE
from usb_monitor import Collector, Stream, serial_port

PHASE = ("WAIT", "IDLE", "MOTION", "POSITION", "DEGRADED", "IMPEDANCE", "REVERSE_J3", "DRAG")
PURE_GRAVITY_PHASES = ("IDLE", "GRAVITY")  # Also accept the phase in archived captures.
EXPECTED = [(i, 2 if i <= 2 else 1, int(i == 3), i, 0x10+i) for i in range(1, 6)]
CONTROL_VERSION = 0x594c3501
CRUISE_VERSION = 0x594c5601
DRAG_VERSION = 0x594c4402
HISTORY_VERSION = 0x594c4801
IMPEDANCE_TEST_VERSION = 0x594c5402
TRAJECTORY_VERSION = 0x594c5302
MOTION_BUNDLE_VERSION = 0x594c4202
INVALID_ARG = 1


class RPCRejected(RuntimeError):
    def __init__(self, action, rc):
        super().__init__(f"Board rejected {action}: rc={rc}")
        self.rc = rc


class ReplyTimeout(TimeoutError):
    """Entire request written, but no matching response observed."""


def decode_motion(reply):
    rc, phase, fresh, enabled, submitted, limited, flags, count, expected = reply[12:21]
    if rc:
        raise RPCRejected("bundled motion", rc)
    if (len(reply) not in (180,244) or phase >= len(PHASE) or flags not in (0, 1, 2) or
            not count <= expected <= (len(reply)-116)//4):
        raise ValueError("Invalid bundled motion state")
    tick, fault, age, plan_id, start_id, checksum, start, duration = struct.unpack_from("<8I", reply, 24)
    active = flags == 2
    state = dict(phase=PHASE[phase], fresh=fresh, enabled=enabled, submitted=submitted,
                 limited=limited, fault=fault, tick_ms=tick,
                 gravity_age_ms=None if age == 0xffffffff else age,
                 model_gravity_nm=None if age == 0xffffffff else list(struct.unpack_from("<5f", reply, 56)))
    path = dict(plan_id=plan_id, start_id=start_id, checksum=checksum,
                count=count, expected=expected, active=active, uploading=flags == 1,
                start_ms=start, duration_ms=duration,
                origin_deg=list(struct.unpack_from("<5f", reply, 76)) if active else None,
                origin_velocity_rad_s=list(struct.unpack_from("<5f", reply, 96)) if active else None,
                knot_ms=list(struct.unpack_from(f"<{count}I", reply, 116)) if active else None)
    return state, path


def gravity_output_confirmed(state):
    return state["phase"] in PURE_GRAVITY_PHASES and idle_output_confirmed(state)


def idle_output_confirmed(state):
    return (state["phase"] in (*PURE_GRAVITY_PHASES, "DRAG") and
            state["fresh"] == state["enabled"] == state["submitted"] == 31
            and not state["fault"] and not state["limited"] and state["gravity_age_ms"] is not None
            and 0 <= state["gravity_age_ms"] <= 100)


class Board:
    def __init__(self, port):
        self.port, self.stream, self.collector = port, Stream(), Collector()
        self.session, self.ident = secrets.token_bytes(8), 0
        self.query_ident, self.motion_bundle, self._trajectory_plan = 0, False, None
        self.latest = self.at = None
        self.on_feedback = self.last_counts = None

    def pump(self):
        replies = []
        data = self.port.read(min(8192, max(1, self.port.in_waiting)))
        for kind, record in self.stream.messages(data):
            if kind is not None:
                replies.append((kind, record))
                continue
            now = time.monotonic()
            sample = self.collector.add(record, now)
            if sample:
                ids = [(j["joint"], j["bus"], j["protocol"], j["command_id"], j["feedback_id"])
                       for j in sample["joints"]]
                if ids != EXPECTED:
                    raise ValueError("Not the five-axis Board02 motor layout")
                if self.latest and ((sample["tick_ms"] - self.latest["tick_ms"]) & 0xffffffff) > 0x7fffffff:
                    raise RuntimeError("MCU restarted; session invalid")
                self.latest, self.at = sample, now
                if self.on_feedback:
                    counts = tuple(j["received_count"] for j in sample["joints"])
                    if counts != self.last_counts:
                        self.on_feedback(sample)
                        self.last_counts = counts
        return replies

    def exchange(self, kind, payload, expected, prefix):
        frame = encode(kind, payload)
        if self.port.write(frame) != len(frame):
            raise OSError("Partial USB write; action not retried")
        deadline = time.monotonic() + 0.7
        while time.monotonic() < deadline:
            for k, p in self.pump():
                if k == expected and p.startswith(prefix):
                    return p
        raise ReplyTimeout("Board reply absent; outcome unknown, no replay")

    def connect(self):
        deadline = time.monotonic() + 2
        while self.latest is None and time.monotonic() < deadline:
            self.pump()
        if self.latest is None:
            raise TimeoutError("No five-axis telemetry")
        self.exchange(SESSION, self.session, SESSION_ACK, self.session)
        if self.rpc(255) != CONTROL_VERSION:
            raise ValueError("Wrong five-axis control version")
        try:
            version = self.rpc(247)
        except RPCRejected as exc:
            if exc.rc != INVALID_ARG:
                raise
        else:
            if version not in (0x594c4201, MOTION_BUNDLE_VERSION):
                raise ValueError("Unknown bundled motion capability")
            self.motion_bundle = True

    def write_prefix(self):
        # RPC responses can split/starve a five-frame telemetry snapshot.
        # Read to completion within the existing I/O budget; never transmit
        # with an old timestamp or replay a request.
        deadline = time.monotonic() + .7
        while self.at is None or time.monotonic() - self.at > .1:
            if time.monotonic() >= deadline:
                raise TimeoutError("Stale board time; no request sent")
            self.pump()
        self.ident += 1
        prefix = self.session + struct.pack("<I", self.ident)
        return prefix + struct.pack("<I", self.latest["tick_ms"])

    def rpc(self, field, scope=0, value=0, op=16):
        header = self.write_prefix()
        token = (self.ident - 1) % 255 + 1
        prefix = header[:12]
        frame = header + struct.pack("<BBBBI", token, op, scope, field, value)
        reply = self.exchange(RELIABLE_REQUEST, frame, RELIABLE_RESPONSE, prefix)
        seq, rc, sc, f, val = struct.unpack_from("<BBBBI", reply, 12)
        if (seq, sc, f) != (token, scope, field):
            raise ValueError("Reply mismatch")
        if rc:
            raise RPCRejected(f"op={op} scope={scope} field={field}", rc)
        return val

    def scalar(self, field, joint, op=16):
        return struct.unpack("<f", struct.pack("<I", self.rpc(field, joint, op=op)))[0]

    def move(self, degrees, milliseconds):
        """One original finite move; no retry or enable."""
        if (len(degrees) != 5 or not all(map(math.isfinite, degrees)) or
                not 100 <= milliseconds <= 120000):
            raise ValueError("Invalid finite trajectory")
        for joint, deg in enumerate(degrees, 1):
            self.rpc(5, joint, struct.unpack("<I", struct.pack("<f", deg))[0])
        self.rpc(6, value=milliseconds)

    def speed(self, joint, rad_s, milliseconds, ramp_ms=1000):
        """One single-axis velocity PI trial; board captures actual start/hold pose."""
        if (joint not in range(1, 6) or not math.isfinite(rad_s) or not rad_s or
                not 100 <= milliseconds <= 120000 or not 0 < ramp_ms < milliseconds/2):
            raise ValueError("Invalid finite velocity trial")
        if self.rpc(252) != CRUISE_VERSION:
            raise ValueError("Firmware has no matching finite velocity-loop capability")
        self.rpc(12, scope=joint, value=struct.unpack("<I", struct.pack("<f", rad_s))[0])
        self.rpc(14, value=ramp_ms)
        self.rpc(13, value=milliseconds)

    def status(self):
        if self.motion_bundle:
            state, _ = self.motion_query()
            return dict(state, telemetry=self.latest)
        flags = self.rpc(0)
        phase, fresh, enabled, submitted = flags.to_bytes(4, "little")
        fault, age, limited = self.rpc(1), self.rpc(2), self.rpc(3)
        g = None if age == 0xffffffff else [self.scalar(4, i) for i in range(1, 6)]
        return dict(phase=PHASE[phase], fresh=fresh, enabled=enabled, submitted=submitted,
                    fault=fault, gravity_age_ms=None if g is None else age, limited=limited,
                    model_gravity_nm=g, telemetry=self.latest)

    def drag(self):
        """Select persistent manual idle; no enable, trajectory or automatic retry."""
        if self.rpc(251) != DRAG_VERSION:
            raise ValueError("Firmware has no matching manual-drag capability")
        self.rpc(15, value=0x44524147)

    def impedance(self):
        """Capture raw centers once; repeated entry never recaptures or enables."""
        if self.rpc(249) != IMPEDANCE_TEST_VERSION:
            raise ValueError("Firmware has no matching native-impedance test capability")
        self.rpc(22, value=0x494d4954)

    def trajectory_upload(self, points):
        """Stage all q/v/a knots, verify count/hash; never starts or retries."""
        from five_trajectory import FIELDS, pack_point, normalize_points, trajectory_hash
        points = normalize_points(points)  # Validate EVERYTHING before any I/O.
        if self.motion_bundle:
            self._trajectory_plan = None
            checksum = trajectory_hash(points)
            header = self.write_prefix()
            plan_id = self.ident
            self.trajectory_ack(TRAJ_BEGIN, header, bytes((len(points),)), plan_id, 0, len(points))
            for index, point in enumerate(points):
                body = struct.pack("<IB", plan_id, index) + pack_point(point)
                stored = self.trajectory_ack(TRAJ_POINT, self.write_prefix(), body,
                                             plan_id, index+1, len(points))
            if stored != checksum:
                raise ValueError("Complete trajectory upload/hash unconfirmed; no commit")
            self._trajectory_plan = plan_id, checksum
            return points
        if self.rpc(248) != TRAJECTORY_VERSION:
            raise ValueError("Firmware has no matching preloaded five-axis Ruckig trajectory")
        self.rpc(34, value=len(points))
        for count, point in enumerate(points, 1):
            for derivative, name in enumerate(FIELDS):
                for joint, value in enumerate(point[name], 1):
                    self.rpc(30+derivative, scope=joint,
                             value=struct.unpack("<I", struct.pack("<f", value))[0])
            if self.rpc(33, value=point["at_ms"]) != count:
                raise ValueError("Stored knot count differs; no commit")
        status = self.trajectory_status()
        if (status["count"] != len(points) or status["expected"] != len(points) or
                not status["uploading"] or status["active"] or
                status["checksum"] != trajectory_hash(points)):
            raise ValueError("Complete trajectory upload/hash unconfirmed; no commit")
        return points

    def trajectory_ack(self, kind, header, body, plan_id, count, expected):
        reply = self.exchange(kind, header + body, TRAJ_ACK, header[:12])
        rc, stored, total, flags, accepted_plan, checksum = struct.unpack_from("<4B2I", reply, 12)
        if rc:
            raise RPCRejected("trajectory upload", rc)
        if (stored, total, flags, accepted_plan) != (count, expected, 1, plan_id):
            raise ValueError("Stored trajectory identity/count differs; no commit")
        return checksum

    def motion_query(self):
        # Distinct reply type and ID namespace. Safe even with a missing write ID.
        self.query_ident += 1
        prefix = self.session + struct.pack("<I", self.query_ident)
        return decode_motion(self.exchange(MOTION_QUERY, prefix, MOTION_STATE, prefix))

    def trajectory_start(self):
        """One explicit commit; no enable/clear/reset/retry."""
        if self.motion_bundle:
            if self._trajectory_plan is None:
                raise ValueError("No verified bundled plan to start")
            plan_id, checksum = self._trajectory_plan
            header = self.write_prefix()
            start_id = self.ident
            self._trajectory_plan = None
            try:
                reply = self.exchange(TRAJ_START, header + struct.pack("<II", plan_id, checksum),
                                      TRAJ_RESULT, header[:12])
            except ReplyTimeout:
                # Check the exact accepted start while this plan still owns the path.
                # No reconnect, sequence repair or second write.
                try:
                    _, path = self.motion_query()
                except (ReplyTimeout, RPCRejected) as exc:
                    raise TimeoutError("Trajectory start outcome unknown; no replay") from exc
            else:
                _, path = decode_motion(reply)
            if (not path["active"] or path["uploading"] or
                    (path["plan_id"], path["start_id"], path["checksum"]) != (plan_id, start_id, checksum)):
                raise RuntimeError("Trajectory start outcome unconfirmed; no replay")
            return path
        self.rpc(35, value=0x53504c4e)
        status = self.trajectory_status()
        if not status["active"] or status["uploading"]:
            raise RuntimeError("Trajectory commit outcome unconfirmed; no replay")
        return status

    def trajectory_status(self):
        if self.motion_bundle:
            return self.motion_query()[1]
        flags = self.rpc(36)
        active = bool(flags & (1 << 17))
        return dict(count=flags & 255, expected=(flags >> 8) & 255,
                    uploading=bool(flags & (1 << 16)), active=active,
                    start_ms=self.rpc(37), duration_ms=self.rpc(38), checksum=self.rpc(39),
                    origin_deg=[self.scalar(40, j) for j in range(1, 6)] if active else None,
                    origin_velocity_rad_s=[self.scalar(42,j) for j in range(1,6)]
                        if active else None,
                    knot_ms=[self.rpc(41,scope=k) for k in range(1,(flags & 255)+1)]
                        if active else None)

    def impedance_test(self, joint, offset_deg):
        """One target-timed native cycle; return duration, never retry/enable."""
        if (joint not in range(1, 6) or not math.isfinite(offset_deg) or not offset_deg):
            raise ValueError("Invalid native-impedance trial")
        if self.rpc(249) != IMPEDANCE_TEST_VERSION:
            raise ValueError("Firmware has no matching native-impedance test capability")
        return self.rpc(23, scope=joint,
                        value=struct.unpack("<I", struct.pack("<f", offset_deg))[0])

    def impedance_status(self):
        """Read current native target and target-side timing within this SESSION."""
        return dict(test_joint=self.rpc(24), test_started_ms=self.rpc(25),
                    target_raw_rad=[self.scalar(24, j) for j in range(1, 6)])

    def test_gravity(self, state):
        """A measurement explicitly selects pure G; status/connect never does."""
        if not idle_output_confirmed(state):
            raise RuntimeError("Healthy idle support absent; no automatic recovery")
        if state["phase"] == "DRAG":
            if self.rpc(251) != DRAG_VERSION:
                raise ValueError("Firmware has no persistent manual-idle capability")
            self.rpc(7)
            state = self.observe_transition(("DRAG",))
        if not gravity_output_confirmed(state):
            raise RuntimeError("Pure-gravity measurement handover unconfirmed")
        return state

    def history(self):
        """Read checkpoint/turn diagnostics; never erase or change storage."""
        if self.rpc(250) != HISTORY_VERSION:
            raise ValueError("Firmware has no matching position-history capability")
        masks = self.rpc(20)
        joints = []
        for joint in range(1, 6):
            fresh = self.latest["joints"][joint-1]["fresh"]
            saved = bool(masks & (1 << (joint-1)))
            signed = self.rpc(20, joint) if fresh else None
            joints.append(dict(
                joint=joint,
                reference_deg=math.degrees(self.scalar(16, joint)) if fresh else None,
                turns=struct.unpack("<i", struct.pack("<I", signed))[0] if fresh else None,
                single_turn_deg=math.degrees(self.scalar(21, joint)) if fresh else None,
                saved_reference_deg=math.degrees(self.scalar(17, joint)) if saved else None,
                trigger_deg=self.scalar(18, joint),
                corrections=self.rpc(19, joint)))
        return dict(flags=self.rpc(16), sequence=self.rpc(17), errors=self.rpc(18),
                    writes_this_boot=self.rpc(19), saved_mask=masks & 31,
                    restored_mask=(masks >> 8) & 31, joints=joints)

    def observe_transition(self, previous):
        # RPC acceptance and the control tick that publishes the phase are asynchronous.
        if any(phase in previous for phase in PURE_GRAVITY_PHASES):
            previous = (*previous, *PURE_GRAVITY_PHASES)
        deadline = time.monotonic() + .1
        state = self.status()
        while (state["phase"] in previous and not state["fault"] and not state["limited"]
               and state["fresh"] == state["enabled"] == state["submitted"] == 31
               and time.monotonic() < deadline):
            state = self.status()
        return state


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", required=True)
    p.add_argument("--log", type=Path, help="new JSONL; status, actual feedback and board G")
    p.add_argument("--free-space", action="store_true",
                   help="mark ONLY an independently verified run with no external contact/support load")
    p.add_argument("--seconds", type=float, default=1, help="capture duration after request")
    sub = p.add_subparsers(dest="action", required=True)
    for name in ("status", "idle", "acknowledge", "drag", "history"):
        sub.add_parser(name)
    cold = sub.add_parser("cold", help="supported cold commissioning only")
    cold.add_argument("--supported-all-off", action="store_true", required=True,
                      help="operator confirms arm is unloaded on independent support and all drives are known OFF")
    read_j3 = sub.add_parser("read-j3", help="one supported legacy-J3 neutral/enable handshake, no reset/fault clear")
    read_j3.add_argument("--supported", action="store_true", required=True,
                         help="operator supports the arm and explicitly authorizes J3 neutral/one enable")
    move = sub.add_parser("move", help="all five targets are calibrated software degrees")
    move.add_argument("--ms", type=int, required=True)
    move.add_argument("degrees", type=float, nargs=5)
    a = p.parse_args()
    if not math.isfinite(a.seconds) or not 0 <= a.seconds <= 300:
        p.error("--seconds: 0..300")
    if a.action == "read-j3" and (not a.log or a.seconds < 1):
        p.error("read-j3 requires --log and --seconds >= 1")
    if a.action == "move" and (not all(map(math.isfinite, a.degrees)) or not 100 <= a.ms <= 120000):
        p.error("invalid trajectory")
    if a.free_space and (not a.log or a.action not in ("status", "idle", "move")):
        p.error("--free-space needs --log and a measurement run")
    with a.log.open("x", encoding="utf8") if a.log else nullcontext(None) as log:
        def note(**obj):
            if log:
                print(json.dumps(obj, allow_nan=False, separators=(",", ":")), file=log,
                      flush=obj.get("stage") != "feedback")
        with serial_port(a.port) as port:
            b = Board(port)
            b.connect()
            if log or a.action == "move":
                references = [[b.scalar(field, joint, op=15) for field in (1, 2, 3)]
                              for joint in range(1, 6)]
            if log:
                note(stage="capture", schema="ylamp-five-id-1", control_version=CONTROL_VERSION,
                     action=a.action, session=b.session.hex(), free_space=a.free_space,
                     references_rad=references)
                b.on_feedback = lambda sample: note(stage="feedback", telemetry=sample)
            initial = b.status()
            note(stage="before", **initial)
            if a.action == "cold":
                if initial["phase"] != "WAIT" or initial["enabled"] or initial["fault"]:
                    raise ValueError("Not a cold start")
                b.rpc(9, value=0x434f4c44)
            elif a.action == "idle":
                b.rpc(7)
            elif a.action == "acknowledge":
                b.rpc(8)
            elif a.action == "drag":
                if not idle_output_confirmed(initial):
                    raise ValueError("Manual drag requires existing healthy five-axis idle support")
                b.drag()
            elif a.action == "read-j3":
                if (initial["fresh"] != 27 or initial["enabled"] != 27 or
                        initial["submitted"] or initial["gravity_age_ms"] is not None or
                        initial["fault"] & ~1 or initial["telemetry"]["joints"][2]["received_count"]):
                    raise ValueError("Legacy acquisition requires only J3 missing after controller restart")
                if b.rpc(253) != 0x594c4a01:
                    raise ValueError("No matching legacy-J3 acquisition capability")
                b.rpc(11, scope=3, value=0x4a335244)  # Explicit once; ACK is not feedback.
            elif a.action == "move":
                if initial["phase"] not in (*PURE_GRAVITY_PHASES, "DRAG", "POSITION") or initial["fault"] or initial["limited"]:
                    raise ValueError("All axes must already have healthy support")
                if initial["phase"] == "DRAG" and b.rpc(251) != DRAG_VERSION:
                    raise ValueError("Firmware has no persistent manual-idle capability")
                b.move(a.degrees, a.ms)
            history = b.history() if a.action == "history" else None
            if history is not None:
                note(stage="history", **history)
            note(stage="rpc_accepted", action=a.action)
            if a.action == "idle":
                print("IDLE request ACK; verifying pure-gravity output.", flush=True)
            deadline = time.monotonic() + a.seconds + (a.ms/1000 if a.action == "move" else 0)
            if a.action == "idle":
                state = b.observe_transition(("DRAG", "MOTION", "POSITION", "IMPEDANCE"))
                if initial["phase"] == "DRAG" and gravity_output_confirmed(state):
                    note(stage="test_gravity", **state)
            elif a.action in ("move", "drag"):
                state = b.observe_transition(("IDLE", "POSITION"))
            else:
                state = b.status()
            while True:
                note(stage="sample", **state)
                if a.action == "move" and (state["fault"] or state["enabled"] != 31 or state["fresh"] != 31
                                           or state["phase"] not in ("MOTION", "POSITION")):
                    raise RuntimeError("Five-axis move interrupted; see capture")
                if a.action == "drag" and (
                        state["fault"] or state["fresh"] != 31 or state["enabled"] != 31 or
                        state["submitted"] != 31 or state["phase"] != "DRAG"):
                    raise RuntimeError("Manual drag interrupted; no retry")
                if time.monotonic() >= deadline:
                    break
                until = min(deadline, time.monotonic() + .12)
                while time.monotonic() < until:
                    b.pump()
                state = b.status()
            if a.action == "move":
                if state["phase"] != "POSITION":
                    raise RuntimeError("No trajectory-time completion acknowledgement")
                # POSITION is timed completion plus feedback hold, NOT a
                # claim that the mechanism reached an exact commanded angle.
                actual = [math.degrees(j["position_rad"] - references[i][0])
                          for i, j in enumerate(state["telemetry"]["joints"])]
                note(stage="trajectory_result", target_software_deg=a.degrees,
                     actual_software_deg=actual,
                     error_deg=[a.degrees[i] - actual[i] for i in range(5)],
                     physical_arrival_claimed=False)
                b.rpc(7)   # Explicit handover to gravity, not a motor disable.
                handover = b.observe_transition(("POSITION",))
                note(stage="handover", **handover)
                if not gravity_output_confirmed(handover):
                    raise RuntimeError("Five-axis gravity handover unconfirmed; no retry")
                if initial["phase"] == "DRAG":
                    b.drag()
                    state = b.observe_transition(("IDLE",))
                    note(stage="manual_handover", **state)
                    if state["phase"] != "DRAG" or not idle_output_confirmed(state):
                        raise RuntimeError("Manual-idle handover unconfirmed; no retry")
            note(stage="end", tick_ms=b.latest["tick_ms"], malformed=b.stream.malformed,
                 incomplete=b.collector.incomplete)
            if history is not None:
                state["history"] = b.history()
            print(json.dumps(state, allow_nan=False))
            if a.action == "idle" and not gravity_output_confirmed(state):
                raise SystemExit("Five-axis IDLE output NOT confirmed; request not repeated.")
            if a.action == "cold" and not idle_output_confirmed(state):
                raise SystemExit("Five-axis support NOT confirmed; request not repeated.")
            if a.action == "read-j3" and not (
                    state["fresh"] == state["enabled"] == state["submitted"] == 31 and
                    state["telemetry"]["joints"][2]["received_count"] > 0 and
                    not state["fault"] & ~1 and not state["limited"]):
                raise SystemExit("Fresh legacy-J3 feedback/handover NOT confirmed; request not repeated.")


if __name__ == "__main__":
    main()
