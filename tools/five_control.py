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
from usb_monitor import Collector, Stream, serial_port

PHASE = ("WAIT", "GRAVITY", "MOTION", "POSITION", "DEGRADED", "IMPEDANCE", "REVERSE_J3", "DRAG")
EXPECTED = [(i, 2 if i <= 2 else 1, int(i == 3), i, 0x10+i) for i in range(1, 6)]
CONTROL_VERSION = 0x594c3501
CRUISE_VERSION = 0x594c5601
DRAG_VERSION = 0x594c4402
HISTORY_VERSION = 0x594c4801


def gravity_output_confirmed(state):
    return state["phase"] == "GRAVITY" and idle_output_confirmed(state)


def idle_output_confirmed(state):
    return (state["phase"] in ("GRAVITY", "DRAG") and
            state["fresh"] == state["enabled"] == state["submitted"] == 31
            and not state["fault"] and not state["limited"] and state["gravity_age_ms"] is not None
            and 0 <= state["gravity_age_ms"] <= 100)


class Board:
    def __init__(self, port):
        self.port, self.stream, self.collector = port, Stream(), Collector()
        self.session, self.ident = secrets.token_bytes(8), 0
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
        raise TimeoutError("Board reply absent; outcome unknown, no replay")

    def connect(self):
        deadline = time.monotonic() + 2
        while self.latest is None and time.monotonic() < deadline:
            self.pump()
        if self.latest is None:
            raise TimeoutError("No five-axis telemetry")
        self.exchange(SESSION, self.session, SESSION_ACK, self.session)
        if self.rpc(255) != CONTROL_VERSION:
            raise ValueError("Wrong five-axis control version")

    def rpc(self, field, scope=0, value=0, op=16):
        # RPC responses can split/starve a five-frame telemetry snapshot.
        # Read to completion within the existing I/O budget; never transmit
        # with an old timestamp or replay a request.
        deadline = time.monotonic() + .7
        while self.at is None or time.monotonic() - self.at > .1:
            if time.monotonic() >= deadline:
                raise TimeoutError("Stale board time; no request sent")
            self.pump()
        self.ident += 1
        token = (self.ident - 1) % 255 + 1
        prefix = self.session + struct.pack("<I", self.ident)
        frame = prefix + struct.pack("<IBBBBI", self.latest["tick_ms"], token, op, scope, field, value)
        reply = self.exchange(RELIABLE_REQUEST, frame, RELIABLE_RESPONSE, prefix)
        seq, rc, sc, f, val = struct.unpack_from("<BBBBI", reply, 12)
        if (seq, sc, f) != (token, scope, field):
            raise ValueError("Reply mismatch")
        if rc:
            raise RuntimeError(f"Board rejected op={op} scope={scope} field={field}: rc={rc}")
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
    for name in ("status", "gravity", "acknowledge", "drag", "history"):
        sub.add_parser(name)
    reverse = sub.add_parser("reverse-j3", help="optional supported 600 ms direction comparison, no enable")
    reverse.add_argument("--supported", action="store_true", required=True,
                         help="operator is continuously supporting the arm for this short comparison")
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
    if a.action == "reverse-j3" and (not a.log or a.seconds < 1):
        p.error("reverse-j3 requires --log and --seconds >= 1")
    if a.action == "read-j3" and (not a.log or a.seconds < 1):
        p.error("read-j3 requires --log and --seconds >= 1")
    if a.action == "move" and (not all(map(math.isfinite, a.degrees)) or not 100 <= a.ms <= 120000):
        p.error("invalid trajectory")
    if a.free_space and (not a.log or a.action not in ("status", "gravity", "move")):
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
            elif a.action == "gravity":
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
            elif a.action == "reverse-j3":
                if not gravity_output_confirmed(initial):
                    raise ValueError("Reverse comparison needs healthy five-axis pure gravity first")
                if b.rpc(254) != 0x594c3301:
                    raise ValueError("No matching J3 comparison capability")
                b.rpc(10, scope=3, value=0x4a335256)  # Exactly once, never replay/extend.
            elif a.action == "move":
                if initial["phase"] not in ("GRAVITY", "DRAG", "POSITION") or initial["fault"] or initial["limited"]:
                    raise ValueError("All axes must already have healthy support")
                if initial["phase"] == "DRAG" and b.rpc(251) != DRAG_VERSION:
                    raise ValueError("Firmware has no persistent manual-idle capability")
                b.move(a.degrees, a.ms)
            history = b.history() if a.action == "history" else None
            if history is not None:
                note(stage="history", **history)
            note(stage="rpc_accepted", action=a.action)
            if a.action == "gravity":
                print("Gravity/trajectory-cancel request ACK; verifying actual output.", flush=True)
            deadline = time.monotonic() + a.seconds + (a.ms/1000 if a.action == "move" else 0)
            state = b.observe_transition(("GRAVITY", "POSITION")) if a.action in ("move", "drag") else b.status()
            reverse_seen = False
            while True:
                note(stage="sample", **state)
                reverse_seen |= state["phase"] == "REVERSE_J3"
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
                    state = b.observe_transition(("GRAVITY",))
                    note(stage="manual_handover", **state)
                    if state["phase"] != "DRAG" or not idle_output_confirmed(state):
                        raise RuntimeError("Manual-idle handover unconfirmed; no retry")
            note(stage="end", tick_ms=b.latest["tick_ms"], malformed=b.stream.malformed,
                 incomplete=b.collector.incomplete)
            if history is not None:
                state["history"] = b.history()
            print(json.dumps(state, allow_nan=False))
            if a.action == "reverse-j3" and (not reverse_seen or not gravity_output_confirmed(state)):
                raise SystemExit("J3 comparison or return to original gravity unconfirmed; no retry.")
            if a.action in ("gravity", "cold") and not gravity_output_confirmed(state):
                raise SystemExit("Five-axis gravity output NOT confirmed at end of capture; request not repeated.")
            if a.action == "read-j3" and not (
                    state["fresh"] == state["enabled"] == state["submitted"] == 31 and
                    state["telemetry"]["joints"][2]["received_count"] > 0 and
                    not state["fault"] & ~1 and not state["limited"]):
                raise SystemExit("Fresh legacy-J3 feedback/handover NOT confirmed; request not repeated.")


if __name__ == "__main__":
    main()
