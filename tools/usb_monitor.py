"""Receive Board02 native USB CDC telemetry, NOT a USB-CAN adapter.

This CLI is read-only: DTR opens telemetry, no data/control commands sent.
The GUI also uses Stream.messages for reliable control reply multiplexing.
Five adjacent 48-byte v1 records share one bounded snapshot decoder/assembler.
USB Full Speed 12Mbps; the nominal serial baud does not set USB throughput.
"""
from __future__ import annotations

import argparse
import binascii
import json
import math
import struct
import time
from contextlib import contextmanager
from dm_serial import (FOCUS_SAMPLE, LENGTHS, TRAJ_ACK, TRAJ_RESULT, MOTION_STATE,
                       PRESET_ACK, PRESET_STATE, PRESET_DATA, valid_v1_length)

HEADER = struct.Struct("<2sBBIIHBB")
RECORD = struct.Struct("<BBBBHHIIfffBBBB")
MASK = 0xFFFFFFFF


def decode(data: bytes) -> dict:
    if len(data) != 48:
        raise ValueError("Expected one 48-byte motor snapshot record")
    magic, version, count, sequence, tick, limit, size, index = HEADER.unpack_from(data)
    if magic != b"YL" or version != 1 or not 1 <= count <= 6 or size != 32 or index >= count:
        raise ValueError("Unsupported telemetry header")
    joint, bus, protocol, flags, command, feedback, rx_tick, rx_count, p, v, t, state, mos, rotor, reserved = RECORD.unpack_from(data, 16)
    if (not 1 <= joint <= 6 or bus not in (1, 2) or protocol not in (0, 1)
            or flags not in (0, 1, 3) or reserved or command > 0x7FF or feedback > 0x7FF
            or not limit):
        raise ValueError("Invalid joint metadata")
    received = bool(flags & 1)
    age = (tick - rx_tick) & MASK if received else None
    if received:
        if not all(math.isfinite(x) for x in (p, v, t)) or bool(flags & 2) != (age <= limit):
            raise ValueError("Invalid values/freshness flag")
    elif rx_count or rx_tick or not all(math.isnan(x) for x in (p, v, t)):
        raise ValueError("Invalid unreceived record")
    return dict(sequence=sequence, tick_ms=tick, count=count, max_age_ms=limit, index=index,
                joint=joint, bus=bus, protocol=protocol, received=received,
                fresh=bool(flags & 2), command_id=command, feedback_id=feedback,
                received_tick_ms=rx_tick, received_count=rx_count, age_ms=age,
                position_rad=p if received else None, velocity_rad_s=v if received else None,
                torque_nm=t if received else None, state=state if received else None,
                mos_c=mos if received else None, rotor_c=rotor if received else None)


class Collector:
    """One bounded assembly; never combine records from different snapshots."""
    def __init__(self):
        self.key = None
        self.parts = {}
        self.first_at = 0.0
        self.last_seen = 0.0
        self.complete = False
        self.incomplete = self.gaps = self.resyncs = self.duplicates = 0

    def add(self, data, now):
        record = decode(data)
        key = tuple(record[k] for k in ("sequence", "tick_ms", "count", "max_age_ms"))
        if self.key is not None and key != self.key:
            forward = (key[0] - self.key[0]) & MASK
            if now - self.last_seen <= 0.5:
                if not 0 < forward < 0x80000000:
                    raise ValueError("Old/conflicting snapshot")
                self.gaps += forward - 1
            else:
                self.resyncs += 1
        if key != self.key:
            if self.key is not None and not self.complete:
                self.incomplete += 1
            self.key, self.parts, self.first_at, self.complete = key, {}, now, False
        self.last_seen = now
        index = record["index"]
        if index in self.parts:
            if self.parts[index] != record:
                raise ValueError("Conflicting duplicate")
            self.duplicates += 1
            return None
        if any(r["joint"] == record["joint"] for r in self.parts.values()):
            raise ValueError("Duplicate joint in snapshot")
        self.parts[index] = record
        if len(self.parts) != record["count"]:
            return None
        self.complete = True
        if now - self.first_at > record["max_age_ms"] / 1000:
            self.incomplete += 1
            return None
        return dict(sequence=key[0], tick_ms=key[1], max_age_ms=key[3],
                    first_received_monotonic=self.first_at, unix_s=time.time(),
                    joints=sorted(self.parts.values(), key=lambda r: r["joint"]))


def display(snapshot, now):
    if snapshot is None:
        return "Waiting for a complete five-joint snapshot..."
    extra_ms = max(0, now - snapshot["first_received_monotonic"]) * 1000
    rows = []
    for r in snapshot["joints"]:
        age = None if r["age_ms"] is None else r["age_ms"] + extra_ms
        label = "NO_FEEDBACK" if age is None else "FRESH" if age <= snapshot["max_age_ms"] else "STALE"
        position = "--" if age is None else f"{math.degrees(r['position_rad']):.2f}deg"
        age_text = "--" if age is None else f"{age:.0f}ms"
        rows.append(f"J{r['joint']} {position} {label} age={age_text} rx={r['received_count']}")
    return f"seq={snapshot['sequence']} | " + " | ".join(rows)


class Stream:
    """Bounded byte-stream resynchronization; USB read boundaries are not frames."""
    def __init__(self):
        self.buffer = bytearray()
        self.discarded = self.malformed = 0

    def feed(self, data):
        for kind, payload in self.messages(data):
            if kind is None:
                yield payload

    def messages(self, data):
        """Multiplex unchanged YL snapshots and CRC-protected RPC responses."""
        # Carry <=250 bytes; consume incoming data in bounded 4096-byte chunks.
        for offset in range(0, len(data), 4096):
            self.buffer.extend(data[offset:offset + 4096])
            while self.buffer:
                starts = [i for i in (self.buffer.find(b"YL"), self.buffer.find(b"\xa5\x5a")) if i >= 0]
                start = min(starts) if starts else len(self.buffer) - int(self.buffer[-1] in (ord("Y"), 0xA5))
                if start:
                    self.discarded += start
                    del self.buffer[:start]
                if len(self.buffer) < 2:
                    break
                rpc = self.buffer[:2] == b"\xa5\x5a"
                if rpc and len(self.buffer) < 5:
                    break
                total = self.buffer[4] + 7 if rpc else 48
                if rpc and not (self.buffer[2] == 1
                    and self.buffer[3] in (0x92, 0x91, 0x93, FOCUS_SAMPLE,
                                           TRAJ_ACK, TRAJ_RESULT, MOTION_STATE, PRESET_ACK, PRESET_STATE, PRESET_DATA)
                    and valid_v1_length(self.buffer[3], self.buffer[4])):
                    self.malformed += 1
                    del self.buffer[0]
                    continue
                if len(self.buffer) < total:
                    break
                record = bytes(self.buffer[:total])
                try:
                    if rpc:
                        if binascii.crc_hqx(record[2:-2], 0xFFFF) != int.from_bytes(record[-2:], "little"):
                            raise ValueError("USB RPC CRC")
                    else:
                        decode(record)
                except ValueError:
                    self.malformed += 1
                    self.discarded += 1
                    del self.buffer[0]
                else:
                    del self.buffer[:total]
                    yield (record[3], record[5:-2]) if rpc else (None, record)


@contextmanager
def serial_port(name):
    import serial  # Existing pyserial; self-test/decoder need only stdlib.
    port = serial.Serial(port=None, baudrate=1_000_000, timeout=.02, write_timeout=.1)
    try:
        port.dtr = True
        port.rts = False
        port.port = name
        port.open()
        yield port
    finally:
        port.close()


def self_test():
    golden = bytes.fromhex(
        "594c010501000000fbffffff64002000"
        "0102000301001100fbffffff01000000"
        "00004841000a20bb001ce03b001e1f00")
    r = decode(golden)
    assert r["position_rad"] == 12.5 and r["fresh"] and r["joint"] == 1
    assert r["velocity_rad_s"] == -0.002442002296447754
    assert r["torque_nm"] == 0.006839275360107422
    def group(seq, count=5):
        return b"".join(
            HEADER.pack(b"YL", 1, count, seq, 10, 100, 32, j - 1)
            + RECORD.pack(j, 2 if j <= 2 else 1, int(j == 3), 0, j, 0x10 + j,
                          0, 0, math.nan, math.nan, math.nan, 255, 255, 255, 0)
            for j in range(1, count + 1))

    # Preserve the common assembly checks when removing the old FD adapter tool.
    collector = Collector()
    first = group(1)[:48]
    assert collector.add(first, 0) is None
    assert collector.add(first, .001) is None and collector.duplicates == 1
    conflict = bytearray(first);conflict[20] = 2
    try:
        collector.add(conflict, .002)
    except ValueError:
        pass
    else:
        raise AssertionError("Conflicting duplicate accepted")
    collector.add(group(2)[:48], .01)
    assert collector.incomplete == 1
    try:
        collector.add(first, .02)
    except ValueError:
        pass
    else:
        raise AssertionError("Old snapshot accepted")
    collector.add(group(0)[:48], 1)
    assert collector.resyncs == 1
    wrap = Collector();wrap.add(group(MASK)[:48], 0);wrap.add(group(0)[:48], .01)
    assert wrap.gaps == 0
    payload = group(1) + group(2) + group(4, 6)
    for size in (1, 7, 48, 64, 240, 4096):
        stream, collector, snapshots = Stream(), Collector(), []
        for offset in range(0, len(payload), size):
            for record in stream.feed(payload[offset:offset + size]):
                snapshot = collector.add(record, .01)
                if snapshot:
                    snapshots.append(snapshot)
            assert len(stream.buffer) < 48
        assert [s["sequence"] for s in snapshots] == [1, 2, 4]
        assert len(snapshots[-1]["joints"]) == 6 and collector.gaps == 1
        assert "NO_FEEDBACK" in display(snapshots[-1], .01)
    stream = Stream()
    damaged = bytearray(group(1)[:48])
    damaged[2] = 99
    recovered = list(stream.feed(b"x" * 10000 + b"Y" + damaged + b"noise" + group(2)))
    assert len(recovered) == 5 and all(decode(r)["sequence"] == 2 for r in recovered)
    assert stream.discarded and stream.malformed and len(stream.buffer) < 48
    stream = Stream()
    list(stream.feed(b"x" * 50 + b"Y"))
    assert list(stream.feed(group(3)[1:])) == [group(3)[i:i + 48] for i in range(0, 240, 48)]
    # Execute actual serial-port lifetime against a fake with NO write method.
    import sys
    from types import SimpleNamespace
    from unittest.mock import patch
    events = []

    class FakePort:
        def __init__(self, **kwargs):
            assert kwargs == dict(port=None, baudrate=1_000_000, timeout=.02, write_timeout=.1)

        def open(self):
            assert self.dtr and not self.rts and self.port == "FAKE"
            events.append("open")

        def close(self):
            events.append("close")

    with patch.dict(sys.modules, serial=SimpleNamespace(Serial=FakePort)):
        with serial_port("FAKE"):
            pass
        assert events == ["open", "close"]
    import dm_serial
    reply = dm_serial.encode(dm_serial.SESSION_ACK, b"1234YL78")
    mixed = group(1) + reply + group(2)
    for size in (1, 7, 64, 4096):
        stream = Stream()
        messages = [m for i in range(0, len(mixed), size) for m in stream.messages(mixed[i:i+size])]
        assert len(messages) == 11 and messages[5] == (dm_serial.SESSION_ACK, b"1234YL78")
    print("USB CDC stream fragmentation/resync/5+6 joints/port lifetime: PASS")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--list", action="store_true", help="List serial ports, no connection")
    parser.add_argument("--port", help="Board02 native USB CDC port, e.g. COM36")
    parser.add_argument("--seconds", type=float, default=0, help="0: until Ctrl+C")
    parser.add_argument("--log", help="Create a NEW JSONL file; never overwrite")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if args.list:
        from serial.tools import list_ports
        for port in list_ports.comports():
            print(f"{port.device}: {port.description} [{port.hwid}]")
        return
    if not args.port:
        parser.error("Select the board's native USB port using --port; --list lists ports")
    if not math.isfinite(args.seconds) or args.seconds < 0:
        parser.error("--seconds must be finite and nonnegative")
    log = open(args.log, "x", encoding="utf-8") if args.log else None
    stream, collector, latest = Stream(), Collector(), None
    snapshots = received_bytes = conflicts = 0
    start = last_display = time.monotonic()
    try:
        with serial_port(args.port) as port:
            print(f"{args.port}: native USB CDC, receive only; DTR ON", flush=True)
            while not args.seconds or time.monotonic() - start < args.seconds:
                data = port.read(min(4096, max(1, port.in_waiting)))
                now = time.monotonic()
                received_bytes += len(data)
                for record in stream.feed(data):
                    try:
                        snapshot = collector.add(record, now)
                    except ValueError:
                        conflicts += 1
                        continue
                    if snapshot:
                        snapshots += 1
                        latest = snapshot
                        if log:
                            log.write(json.dumps(snapshot, allow_nan=False) + "\n")
                if now - last_display >= .1:
                    last_display = now
                    print(display(latest, now), flush=True)
    finally:
        if log:
            log.close()
        elapsed = max(time.monotonic() - start, 1e-6)
        print(f"snapshots={snapshots} avg_hz={snapshots/elapsed:.1f} "
              f"payload_kB_s={received_bytes/elapsed/1000:.1f} sequence_gaps={collector.gaps} "
              f"incomplete={collector.incomplete} resyncs={collector.resyncs} "
              f"malformed={stream.malformed} discarded_bytes={stream.discarded} conflicts={conflicts}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        raise SystemExit(f"USB monitor: {exc}") from exc
