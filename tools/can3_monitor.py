"""Read-only CAN3 FD telemetry: GS_USB FD at 1M/2M, or offline --decode HEX.

No CAN transmit API or motor commands. USB control requests configure only the
adapter. Existing dm_monitor.py remains the separate Classic CAN/RPC interface.
GS_USB wire definitions: Linux drivers/net/can/usb/gs_usb.c (2026-11-14).
"""
from __future__ import annotations

import argparse
import json
import math
import struct
import time
from contextlib import contextmanager

CAN_ID = 0x600
DLC_LENGTHS = (*range(9), 12, 16, 20, 24, 32, 48, 64)
HEADER = struct.Struct("<2sBBIIHBB")
RECORD = struct.Struct("<BBBBHHIIfffBBBB")
USB_HEADER = struct.Struct("<IIBBBB")
MASK = 0xFFFFFFFF


def decode(data: bytes) -> dict:
    if len(data) != 48:
        raise ValueError("Expected one 48-byte CAN FD record")
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
                self.gaps += forward - 1  # Missing snapshot sequences, NOT motor packets.
            else:
                self.resyncs += 1  # May be a reboot; no delivery guarantee across it.
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
        # A late fragment must not revive an old snapshot.
        if now - self.first_at > record["max_age_ms"] / 1000:
            self.incomplete += 1
            return None
        return dict(sequence=key[0], tick_ms=key[1], max_age_ms=key[3],
                    first_received_monotonic=self.first_at, unix_s=time.time(),
                    joints=sorted(self.parts.values(), key=lambda r: r["joint"]))


def unpack_usb(packet):
    if len(packet) < USB_HEADER.size:
        raise ValueError("Truncated GS_USB header")
    echo, can_id, dlc, channel, flags, reserved = USB_HEADER.unpack_from(packet)
    if echo != MASK or channel != 0 or can_id != CAN_ID:
        return None, bool(flags & 1)  # Includes error/extended/RTR IDs and TX echoes.
    if dlc >= len(DLC_LENGTHS) or not flags & 2 or not flags & 4 or flags & ~15 or reserved:
        raise ValueError("Expected FD+BRS on channel 0")
    length = DLC_LENGTHS[dlc]
    if len(packet) < 12 + length or len(packet) > 80:
        raise ValueError("Truncated/oversize GS_USB data")
    return bytes(packet[12:12 + length]), bool(flags & 1)


def bit_timing(clock, limits, bitrate):
    """Exact bitrate, closest to 75% sample point, within advertised bounds."""
    s1min, s1max, s2min, s2max, sjwmax, bmin, bmax, binc = limits
    if (not clock or not binc or not 1 <= bmin <= bmax <= 1024
            or not 1 <= s1min <= s1max <= 512
            or not 1 <= s2min <= s2max <= 256 or not sjwmax):
        raise ValueError("Invalid adapter timing limits")
    choices = []
    for prescaler in range(bmin, bmax + 1, binc):
        if clock % (bitrate * prescaler):
            continue
        total = clock // (bitrate * prescaler)
        for seg2 in range(s2min, s2max + 1):
            seg1 = total - 1 - seg2
            if s1min <= seg1 <= s1max:
                choices.append((abs((1 + seg1) / total - .75), prescaler, seg1, seg2))
    if not choices:
        raise ValueError(f"Adapter cannot provide exact {bitrate} bit/s")
    _, prescaler, seg1, seg2 = min(choices)
    prop = min(1, seg1 - 1)
    return struct.pack("<5I", prop, seg1 - prop, seg2, min(seg2, sjwmax), prescaler)


def fd_configuration(constants, extended=None):
    if len(constants) != 40:
        raise ValueError("Invalid GS_USB capabilities")
    fields = struct.unpack("<10I", constants)
    features, clock = fields[:2]
    if not features & (1 << 8):
        raise ValueError("This USB-CAN is Classic-only; CAN3 requires a CAN FD adapter")
    if features & (1 << 9):
        raise ValueError("LPC546xx GS_USB quirk unsupported; use its vendor FD tool")
    data_limits = fields[2:]
    if features & (1 << 10):
        if extended is None or len(extended) != 72:
            raise ValueError("Missing extended FD timing capabilities")
        ext = struct.unpack("<18I", extended)
        if ext[:10] != fields:
            raise ValueError("Inconsistent adapter capabilities")
        data_limits = ext[10:]
    flags = (1 << 8) | (features & ((1 << 3) | (1 << 4)))
    return bit_timing(clock, fields[2:], 1_000_000), bit_timing(clock, data_limits, 2_000_000), flags


@contextmanager
def gs_usb_fd(adapter_index):
    # Lazy import: --decode/--self-test need only Python's standard library.
    from dm_monitor import usb_modules, scan_adapters
    core, util, backend = usb_modules()
    adapters = scan_adapters()
    if not adapters:
        raise ValueError("No supported GS_USB adapter found; vendor USB-CANFD needs its vendor driver/tool")
    if adapter_index is None and len(adapters) != 1:
        raise ValueError("Multiple adapters: select one explicitly using --adapter INDEX")
    index = 0 if adapter_index is None else adapter_index
    if not 0 <= index < len(adapters):
        raise ValueError("Adapter index out of range")
    selected = adapters[index]
    dev = core.find(idVendor=selected["vendor"], idProduct=selected["product"],
                    bus=selected["bus"], address=selected["address"], backend=backend)
    if dev is None:
        raise ValueError("Selected adapter disconnected")
    claimed = configured = False

    def control(direction, request, value):
        return dev.ctrl_transfer(direction, request, 0, 0, value, timeout=500)

    try:
        util.claim_interface(dev, 0)
        claimed = True
        # Must precede capability queries. Does not transmit any CAN data.
        dev.ctrl_transfer(0x41, 0, 1, 0, struct.pack("<I", 0xBEEF), timeout=500)
        constants = bytes(control(0xC1, 4, 40))
        if len(constants) != 40:
            raise ValueError("Short GS_USB capability reply")
        features = struct.unpack_from("<I", constants)[0]
        extended = bytes(control(0xC1, 11, 72)) if features & (1 << 10) else None
        nominal, data, flags = fd_configuration(constants, extended)
        interface = dev.get_active_configuration()[(0, 0)]
        endpoint = next((ep for ep in interface
                         if util.endpoint_direction(ep.bEndpointAddress) == util.ENDPOINT_IN
                         and util.endpoint_type(ep.bmAttributes) == util.ENDPOINT_TYPE_BULK), None)
        if endpoint is None:
            raise ValueError("No GS_USB bulk-IN endpoint")
        configured = True  # Finally resets even if a later configuration request fails.
        control(0x41, 2, struct.pack("<II", 0, 0))
        control(0x41, 1, nominal)
        control(0x41, 10, data)
        control(0x41, 2, struct.pack("<II", 1, flags))
        print(f"{selected['label']}: CAN FD 1M/2M BRS, receive-only application", flush=True)

        def receive():
            try:
                return bytes(dev.read(endpoint.bEndpointAddress, 80, timeout=20))
            except core.USBTimeoutError:
                return None
        yield receive
    finally:
        if configured:
            try:
                control(0x41, 2, struct.pack("<II", 0, 0))
            except core.USBError:
                pass
        try:
            if claimed:
                util.release_interface(dev, 0)
        finally:
            util.dispose_resources(dev)


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


def self_test():
    golden = bytes.fromhex(
        "594c010501000000fbffffff64002000"
        "0102000301001100fbffffff01000000"
        "00004841000a20bb001ce03b001e1f00")
    r = decode(golden)
    assert r["position_rad"] == 12.5 and r["velocity_rad_s"] == -0.002442002296447754
    assert r["torque_nm"] == 0.006839275360107422 and r["fresh"]

    def packet(seq=1, joint=1, index=None, tick=3, rx=MASK - 4, received=True):
        age = (tick - rx) & MASK
        flags = 1 | (2 if age <= 100 else 0) if received else 0
        return HEADER.pack(b"YL", 1, 5, seq, tick, 100, 32, joint - 1 if index is None else index) + RECORD.pack(
            joint, 2 if joint <= 2 else 1, int(joint == 3), flags, joint, 0x10 + joint,
            rx if received else 0, 1 if received else 0,
            1.0 if received else math.nan, 0 if received else math.nan,
            0 if received else math.nan, 0 if received else 255,
            30 if received else 255, 31 if received else 255, 0)

    c = Collector()
    assert c.add(packet(joint=2), 0) is None
    assert c.add(packet(joint=2), .001) is None and c.duplicates == 1
    for j in (5, 1, 3):
        assert c.add(packet(joint=j, received=j != 5), .002) is None
    s = c.add(packet(joint=4), .003)
    assert s and len(s["joints"]) == 5 and s["joints"][0]["age_ms"] == 8
    assert s["joints"][2]["protocol"] == 1 and s["joints"][4]["position_rad"] is None
    assert "STALE" in display(s, .2) and "NO_FEEDBACK" in display(s, .2)
    assert c.add(packet(seq=2), .01) is None
    assert c.add(packet(seq=4), .03) is None and c.incomplete == 1 and c.gaps == 1
    try:
        c.add(packet(seq=3), .04)
        raise AssertionError("Accepted older sequence")
    except ValueError:
        pass
    assert c.add(packet(seq=0), 1) is None and c.resyncs == 1
    wrap = Collector()
    wrap.add(packet(seq=MASK), 0)
    wrap.add(packet(seq=0), .01)
    assert wrap.gaps == 0
    assert not decode(packet(tick=200))["fresh"]
    for malformed in (golden[:-1], b"XX" + golden[2:], golden[:2] + b"\x02" + golden[3:],
                      packet(index=5), packet(joint=2, index=0)[:19] + b"\x02" + packet(joint=2, index=0)[20:]):
        try:
            decode(malformed)
            raise AssertionError("Accepted invalid packet")
        except ValueError:
            pass
    usb = USB_HEADER.pack(MASK, CAN_ID, 14, 0, 6, 0) + golden
    assert unpack_usb(usb) == (golden, False)
    assert unpack_usb(usb[:12] + golden + bytes(20)) == (golden, False)
    assert unpack_usb(USB_HEADER.pack(0, CAN_ID, 14, 0, 6, 0) + golden)[0] is None
    try:
        unpack_usb(usb[:-1])
        raise AssertionError("Accepted short USB packet")
    except ValueError:
        pass
    fields = (1 << 8, 48_000_000, 1, 16, 1, 8, 4, 1, 1024, 1)
    nominal, data, flags = fd_configuration(struct.pack("<10I", *fields))
    for timing, rate in ((nominal, 1_000_000), (data, 2_000_000)):
        prop, phase, seg2, sjw, pre = struct.unpack("<5I", timing)
        assert 48_000_000 == rate * pre * (1 + prop + phase + seg2) and sjw <= seg2
    assert flags == 1 << 8
    try:
        fd_configuration(struct.pack("<10I", 0, *fields[1:]))
        raise AssertionError("Accepted Classic-only adapter")
    except ValueError:
        pass
    # Execute the actual adapter lifetime against fakes: no USB library/device.
    import io
    import sys
    from contextlib import redirect_stdout
    from types import SimpleNamespace
    from unittest.mock import patch

    class FakeUSBError(Exception):
        pass

    class FakeDevice:
        def __init__(self, features):
            self.features, self.requests = features, []

        def ctrl_transfer(self, direction, request, value, index, data, timeout):
            self.requests.append((direction, request, value, index, data))
            assert timeout == 500
            if direction == 0xC1:
                base = (self.features, *fields[1:])
                if request == 4:
                    return struct.pack("<10I", *base)
                assert request == 11
                return struct.pack("<18I", *base, *fields[2:])
            return len(data)

        def get_active_configuration(self):
            return {(0, 0): [SimpleNamespace(bEndpointAddress=0x81, bmAttributes=2)]}

        def read(self, endpoint, size, timeout):
            assert (endpoint, size, timeout) == (0x81, 80, 20)
            return usb
        # Deliberately NO write() / CAN bulk-OUT.

    lifecycle = []
    dev = FakeDevice((1 << 8) | (1 << 10) | (1 << 3) | (1 << 4))
    core = SimpleNamespace(find=lambda **kw: dev, USBError=FakeUSBError,
                           USBTimeoutError=FakeUSBError)
    util = SimpleNamespace(
        claim_interface=lambda *a: lifecycle.append("claim"),
        release_interface=lambda *a: lifecycle.append("release"),
        dispose_resources=lambda *a: lifecycle.append("dispose"),
        endpoint_direction=lambda ep: ep & 0x80, ENDPOINT_IN=0x80,
        endpoint_type=lambda attr: attr & 3, ENDPOINT_TYPE_BULK=2)
    fake_module = SimpleNamespace(
        usb_modules=lambda: (core, util, None),
        scan_adapters=lambda: [dict(vendor=0x1D50, product=0x606F, bus=1, address=1, label="fake")])
    with patch.dict(sys.modules, dm_monitor=fake_module), redirect_stdout(io.StringIO()):
        with gs_usb_fd(None) as receive:
            assert receive() == usb
        assert lifecycle == ["claim", "release", "dispose"]
        assert [r[1] for r in dev.requests] == [0, 4, 11, 2, 1, 10, 2, 2]
        assert struct.unpack("<II", dev.requests[-2][4]) == (1, 0x118)
        assert struct.unpack("<II", dev.requests[-1][4]) == (0, 0)
        # Failure does not start/reset a Classic-only adapter's CAN channel.
        dev = FakeDevice(0)
        lifecycle.clear()
        try:
            with gs_usb_fd(None):
                raise AssertionError("Started Classic adapter in FD")
        except ValueError:
            pass
        assert [r[1] for r in dev.requests] == [0, 4]
        assert lifecycle == ["claim", "release", "dispose"]
    print("CAN3 FD decoder/assembly/USB framing/timing/adapter lifetime self-test: PASS")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--decode", metavar="HEX", help="Decode one 48-byte payload, no USB access")
    parser.add_argument("--adapter", type=int, help="GS_USB adapter index (channel 0)")
    parser.add_argument("--seconds", type=float, default=0, help="0: until Ctrl+C")
    parser.add_argument("--log", help="Create a NEW JSONL file; never overwrite")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if args.decode:
        print(json.dumps(decode(bytes.fromhex(args.decode)), allow_nan=False, indent=2))
        return
    if not math.isfinite(args.seconds) or args.seconds < 0:
        parser.error("--seconds must be finite and nonnegative")
    log = open(args.log, "x", encoding="utf-8") if args.log else None
    collector, latest = Collector(), None
    malformed = overflows = snapshots = 0
    start = last_display = time.monotonic()
    try:
        with gs_usb_fd(args.adapter) as receive:
            while not args.seconds or time.monotonic() - start < args.seconds:
                packet = receive()
                now = time.monotonic()
                if packet is not None:
                    try:
                        data, overflow = unpack_usb(packet)
                        overflows += int(overflow)
                        snapshot = collector.add(data, now) if data is not None else None
                    except ValueError:
                        malformed += 1
                        snapshot = None
                    if snapshot is not None:
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
        print(f"snapshots={snapshots} sequence_gaps={collector.gaps} incomplete={collector.incomplete} "
              f"pending_records={0 if collector.complete else len(collector.parts)} "
              f"resyncs={collector.resyncs} malformed={malformed} usb_overflows={overflows}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        raise SystemExit(f"CAN3 monitor: {exc}") from exc
