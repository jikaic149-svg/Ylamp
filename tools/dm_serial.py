"""Ylamp USART1 framing, shared by the serial monitor and offline tests."""

from __future__ import annotations

import binascii
import math
import queue
import secrets
import struct
import time

BAUDRATE = 230400
MAGIC = b"\xa5\x5a"
TELEMETRY, REQUEST, RESPONSE = 0x01, 0x10, 0x90
RELIABLE_REQUEST, RELIABLE_RESPONSE, SESSION, SESSION_ACK = 0x11, 0x91, 0x12, 0x92
FOCUS_REQUEST, FOCUS_ACK, FOCUS_SAMPLE = 0x13, 0x93, 0x94
TRAJ_BEGIN, TRAJ_POINT, TRAJ_START, MOTION_QUERY = 0x18, 0x19, 0x1a, 0x1b
TRAJ_ACK, TRAJ_RESULT, MOTION_STATE = 0x98, 0x99, 0x9a
PRESET_BEGIN, PRESET_POINT, PRESET_SAVE, PRESET_QUERY = 0x1c, 0x1d, 0x1e, 0x1f
PRESET_ACK, PRESET_STATE = 0x9c, 0x9f
PRESET_READ, PRESET_DATA = 0x20, 0xa0
LENGTHS = {TELEMETRY: 144, REQUEST: 8, RESPONSE: 8, RELIABLE_REQUEST: 24,
           RELIABLE_RESPONSE: 20, SESSION: 8, SESSION_ACK: 8,
           FOCUS_REQUEST: 9, FOCUS_ACK: 18, FOCUS_SAMPLE: 206,
           TRAJ_BEGIN: 17, TRAJ_POINT: 85, TRAJ_START: 24, MOTION_QUERY: 12,
           TRAJ_ACK: 24, TRAJ_RESULT: (180, 244), MOTION_STATE: (180, 244),
           PRESET_BEGIN: 17, PRESET_POINT: 85, PRESET_SAVE: 24, PRESET_QUERY: 12,
           PRESET_ACK: 40, PRESET_STATE: 40, PRESET_READ: 16, PRESET_DATA: 244}
MAX_JOINTS, HEADER_SIZE, RECORD_SIZE = 6, 64, 64


def valid_v1_length(kind, length):
    expected = LENGTHS.get(kind)
    return length in expected if isinstance(expected, tuple) else length == expected


def encode(kind, payload, *, version=1):
    if version == 2 and kind == TELEMETRY and (
            HEADER_SIZE + RECORD_SIZE <= len(payload) <= HEADER_SIZE + MAX_JOINTS*RECORD_SIZE
            and (len(payload) - HEADER_SIZE) % RECORD_SIZE == 0):
        body = struct.pack("<BBH", version, kind, len(payload)) + bytes(payload)
    elif version == 1 and kind in LENGTHS and valid_v1_length(kind, len(payload)):
        body = bytes((1, kind, len(payload))) + bytes(payload)
    else:
        raise ValueError("Invalid serial frame type/length")
    return MAGIC + body + struct.pack("<H", binascii.crc_hqx(body, 0xFFFF))


class Parser:
    """Bounded stream parser: noise, split frames, CRC rejection and resync."""

    def __init__(self):
        self.buffer = bytearray()
        self.errors = 0

    def feed(self, data):
        # The I/O reader bounds each chunk to 512 bytes. Processing incrementally
        # also keeps memory bounded if a test/caller supplies a larger chunk.
        frames = []
        for offset in range(0, len(data), 512):
            self.buffer.extend(data[offset:offset + 512])
            while self.buffer:
                start = self.buffer.find(MAGIC)
                if start < 0:
                    self.buffer[:] = self.buffer[-1:] if self.buffer[-1] == 0xA5 else b""
                    break
                if start:
                    del self.buffer[:start]
                if len(self.buffer) < 5:
                    break
                version, kind = self.buffer[2:4]
                header = 6 if version == 2 else 5
                if len(self.buffer) < header:
                    break
                length = (struct.unpack_from("<H", self.buffer, 4)[0]
                          if version == 2 else self.buffer[4])
                valid = (version == 1 and valid_v1_length(kind, length) or
                         version == 2 and kind == TELEMETRY and
                         HEADER_SIZE + RECORD_SIZE <= length <= HEADER_SIZE + MAX_JOINTS*RECORD_SIZE
                         and (length - HEADER_SIZE) % RECORD_SIZE == 0)
                if not valid:
                    self.errors += 1
                    del self.buffer[0]
                    continue
                total = length + header + 2
                if len(self.buffer) < total:
                    break
                crc = struct.unpack_from("<H", self.buffer, total - 2)[0]
                if binascii.crc_hqx(self.buffer[2:total - 2], 0xFFFF) != crc:
                    self.errors += 1
                    del self.buffer[0]
                    continue
                frames.append((kind, bytes(self.buffer[header:total - 2])))
                del self.buffer[:total]
        return frames


class ReliableRPC:
    """One TX owner, RX queue only; bounded stop-and-wait within one connection.

    Retry identical bytes, including session, 32-bit ID and original MCU tick.
    Exhausted/failed transport is latched: a new command must not skip an
    unresolved ID. Reconnect explicitly; never resubmit that action implicitly.
    """

    RETRY_SECONDS = 0.12
    MAX_ATTEMPTS = 3

    def __init__(self, write, stop):
        self.write, self.stop = write, stop
        self.session = secrets.token_bytes(8)
        self.replies = queue.Queue(maxsize=32)
        self.identifier = 0
        self.started = self.failed = False

    def receive(self, kind, payload):
        if kind in (SESSION_ACK, RELIABLE_RESPONSE):
            try:
                self.replies.put_nowait((kind, bytes(payload)))
            except queue.Full:
                pass

    def _exchange(self, kind, payload, expected, key):
        packet = encode(kind, payload)  # Build once: retries cannot alter intent.
        for _ in range(self.MAX_ATTEMPTS):
            if self.stop.is_set():
                raise RuntimeError("连接正在关闭，未继续补发")
            if self.write(packet) != len(packet):
                raise OSError("串口请求未完整提交；结果未知，请手动重连")
            deadline = time.perf_counter() + self.RETRY_SECONDS
            while not self.stop.is_set() and time.perf_counter() < deadline:
                try:
                    reply_kind, reply = self.replies.get(timeout=min(0.02, self.RETRY_SECONDS))
                except queue.Empty:
                    continue
                if reply_kind == expected and reply.startswith(key):
                    return reply
        raise TimeoutError("串口有限补发后仍无应答；实际结果未知，请手动重连核对")

    def start(self):
        if self.started or self.failed:
            raise RuntimeError("不可在当前连接重新握手；请手动重连")
        try:
            self._exchange(SESSION, self.session, SESSION_ACK, self.session)
        except Exception:
            self.failed = True
            raise
        self.started = True

    def request(self, data, issued_tick):
        if not self.started or self.failed or self.identifier == 0xFFFFFFFF:
            raise RuntimeError("可靠串口会话不可用；请手动重连，不重放旧动作")
        if len(data) != 8:
            raise ValueError("RPC must contain eight bytes")
        self.identifier += 1
        payload = self.session + struct.pack("<II", self.identifier, issued_tick) + data
        try:
            reply = self._exchange(RELIABLE_REQUEST, payload, RELIABLE_RESPONSE, payload[:12])
            rpc = reply[12:]
            if len(rpc) != 8 or (rpc[0], rpc[2:4]) != (data[0], data[2:4]):
                raise ValueError("可靠串口应答匹配失败；请手动重连")
        except Exception:
            self.failed = True
            raise
        return rpc


def _telemetry_v1(payload):
    if len(payload) != LENGTHS[TELEMETRY]:
        raise ValueError("Invalid telemetry length")
    tick, sequence = struct.unpack_from("<II", payload)
    state, reason, policy, valid, disabled, tx, disable1, disable2 = payload[8:16]
    if state > 6 or reason > 12 or policy > 1 or valid > 3 or disabled > 3:
        raise ValueError("Invalid MCU status")
    joints = []
    for motor in range(2):
        received, count, position, velocity, torque, status, mos, rotor, _ = (
            struct.unpack_from("<IIfff4B", payload, 16 + 24 * motor))
        reference, target, kp, kd, ff = struct.unpack_from("<5f", payload, 64 + 20 * motor)
        if valid & (1 << motor) and not all(
                math.isfinite(v) for v in (position, velocity, torque)):
            raise ValueError("Non-finite motor feedback")
        joints.append(dict(id=motor+1, can_id=motor+1, feedback_id=0x11+motor,
                           valid=bool(valid & (1 << motor)),
                           disable_submitted=bool(disabled & (1 << motor)),
                           disable_status=(disable1, disable2)[motor],
                           received_ms=received, count=count,
                           age_ms=(tick - received) & 0xFFFFFFFF,
                           position=position, velocity=velocity, torque=torque,
                           state=status, mos=mos, rotor=rotor, reference=reference,
                           target=target, kp=kp, kd=kd, ff=ff,
                           target_velocity=None, target_reference=None, tracking_error=None))
    observations = struct.unpack_from("<I", payload, 104)[0]
    flags, first_lec, first_tec, first_rec = payload[108:112]
    first_ms, bo_ms = struct.unpack_from("<II", payload, 112)
    bo_lec, bo_tec, bo_rec, phase = payload[120:124]
    pending, rx_overflows, crc_errors, tx_drops, uart_error = struct.unpack_from("<5I", payload, 124)
    return dict(version=1, joint_count=2, mode=0,
                tick_ms=tick, sequence=sequence, state=state, reason=reason,
                policy=policy, valid_mask=valid, disable_mask=disabled, tx=tx,
                disable_status=(disable1, disable2), joints=joints, phase=phase,
                can_observations=observations, can_flags=flags,
                rx_running=bool(flags & 16), rx_rearm_failed=bool(flags & 32), first_ms=first_ms,
                first_lec=first_lec, first_tec=first_tec, first_rec=first_rec,
                bo_ms=bo_ms, bo_lec=bo_lec, bo_tec=bo_tec, bo_rec=bo_rec,
                pending=pending, rx_overflows=rx_overflows, crc_errors=crc_errors,
                tx_drops=tx_drops, uart_error=uart_error)


def telemetry(payload):
    """v1 read compatibility; v2 validates layout/identities before any indexing."""
    if len(payload) == LENGTHS[TELEMETRY]:
        return _telemetry_v1(payload)
    if len(payload) < HEADER_SIZE:
        raise ValueError("Truncated telemetry header")
    tick, sequence = struct.unpack_from("<II", payload)
    state, reason, policy, count, mode, phase, record_size, _ = payload[8:16]
    if (not 1 <= count <= MAX_JOINTS or record_size != RECORD_SIZE or
            len(payload) != HEADER_SIZE + count*record_size or
            state > 6 or reason > 12 or policy > 1 or mode > 1 or phase > 5):
        raise ValueError("Unsupported telemetry layout/status")
    joints, seen, valid, disabled = [], set(), 0, 0
    for index in range(count):
        at = HEADER_SIZE + record_size*index
        motor, flags, can_id, feedback_id, disable_status, _ = struct.unpack_from(
            "<BBHHBB", payload, at)
        if (not 1 <= motor <= MAX_JOINTS or motor in seen or flags & ~3 or
                can_id > 0x7FF or feedback_id > 0x7FF):
            raise ValueError("Invalid/duplicate joint identity")
        seen.add(motor)
        received, samples, position, velocity, torque, status, mos, rotor, _ = (
            struct.unpack_from("<IIfff4B", payload, at+8))
        reference, target, target_velocity, kp, kd, ff, target_reference, error = (
            struct.unpack_from("<8f", payload, at+32))
        if flags & 1 and not all(math.isfinite(v) for v in (position, velocity, torque)):
            raise ValueError("Non-finite motor feedback")
        if state in (1, 2, 3, 4) and not all(
                math.isfinite(v) for v in (target, target_velocity, kp, kd, ff)):
            raise ValueError("Non-finite active command")
        valid |= (flags & 1) << (motor-1)
        disabled |= ((flags >> 1) & 1) << (motor-1)
        joints.append(dict(id=motor, can_id=can_id, feedback_id=feedback_id,
                           valid=bool(flags & 1), disable_submitted=bool(flags & 2),
                           disable_status=disable_status, received_ms=received,
                           count=samples, age_ms=(tick-received) & 0xFFFFFFFF,
                           position=position, velocity=velocity, torque=torque,
                           state=status, mos=mos, rotor=rotor, reference=reference,
                           target=target, target_velocity=target_velocity, kp=kp, kd=kd,
                           ff=ff, target_reference=target_reference, tracking_error=error))
    observations, flags, first_lec, first_tec, first_rec = struct.unpack_from("<I4B", payload, 16)
    first_ms, bo_ms, bo_lec, bo_tec, bo_rec, tx = struct.unpack_from("<II4B", payload, 24)
    pending, overflows, crc_errors, tx_drops, uart_error = struct.unpack_from("<5I", payload, 36)
    return dict(version=2, joint_count=count, mode=mode, phase=phase,
                tick_ms=tick, sequence=sequence, state=state, reason=reason, policy=policy,
                valid_mask=valid, disable_mask=disabled, tx=tx,
                disable_status=tuple(j["disable_status"] for j in joints), joints=joints,
                can_observations=observations, can_flags=flags, rx_running=bool(flags & 16),
                rx_rearm_failed=bool(flags & 32), first_ms=first_ms, first_lec=first_lec,
                first_tec=first_tec, first_rec=first_rec, bo_ms=bo_ms, bo_lec=bo_lec,
                bo_tec=bo_tec, bo_rec=bo_rec, pending=pending, rx_overflows=overflows,
                crc_errors=crc_errors, tx_drops=tx_drops, uart_error=uart_error)


def self_test():
    # Identical vector is checked against the real C UART path.
    request = encode(REQUEST, b"\x01" + bytes(7))
    assert request.hex() == "a55a0110080100000000000000fcf6"
    for split in range(len(request) + 1):
        parser = Parser()
        assert parser.feed(request[:split]) + parser.feed(request[split:]) == [
            (REQUEST, b"\x01" + bytes(7))]
    bad = bytearray(request)
    bad[-1] ^= 1
    parser = Parser()
    assert parser.feed(b"old text\r\n" + bad + request + request) == [
        (REQUEST, b"\x01" + bytes(7))] * 2
    assert parser.errors == 1 and not parser.buffer
    parser.feed(b"x" * 10000 + b"\xa5")
    assert parser.buffer == b"\xa5"
    assert parser.feed(request[1:]) == [(REQUEST, b"\x01" + bytes(7))]
    payload = bytearray(144)
    struct.pack_into("<II", payload, 0, 10, 3)
    payload[11] = 1
    struct.pack_into("<IIfff4B", payload, 16, 0xFFFFFFF0, 9, 1, 2, 3, 1, 25, 26, 0)
    status = telemetry(payload)
    assert status["joints"][0]["age_ms"] == 26
    assert status["joints"][0]["state"] == 1
    payload[8] = 6
    assert telemetry(payload)["state"] == 6  # Automatic CAN recovery, not a motor state.
    payload[8] = 7
    try:
        telemetry(payload)
    except ValueError:
        pass
    else:
        raise AssertionError("Unknown Task state accepted")
    payload[8] = 0
    struct.pack_into("<f", payload, 24, math.nan)
    try:
        telemetry(payload)
    except ValueError:
        pass
    else:
        raise AssertionError("Invalid feedback accepted")

    for count in (2, 3, 6):
        payload = bytearray(HEADER_SIZE + count*RECORD_SIZE)
        struct.pack_into("<II8B", payload, 0, 1234, 7, 0, 0, 1, count, 0, 0, RECORD_SIZE, 0)
        for i in range(count):
            at = HEADER_SIZE + i*RECORD_SIZE
            struct.pack_into("<BBHHBBIIfff4B", payload, at,
                             i+1, 1, i+1, 0x11+i, 0, 0, 1200, 20+i,
                             i+.5, -.25, .125, 0, 25, 26, 0)
            struct.pack_into("<8f", payload, at+32,
                             i+.5, i+.75, .125, 5, .5, .25, i+.75, .25)
        frame = encode(TELEMETRY, payload, version=2)
        assert len(frame) == 72+64*count
        for split in range(len(frame)+1):
            parser = Parser()
            assert parser.feed(frame[:split]) + parser.feed(frame[split:]) == [(TELEMETRY, payload)]
        status = telemetry(payload)
        assert status["joint_count"] == count and status["valid_mask"] == (1 << count)-1
        assert status["joints"][-1]["id"] == count
        assert status["joints"][-1]["target_velocity"] == .125
        parser = Parser()
        damaged = bytearray(frame)
        damaged[-1] ^= 1
        assert parser.feed(damaged + request + frame) == [(REQUEST, b"\x01"+bytes(7)), (TELEMETRY, payload)]
        assert parser.errors >= 1
        invalid = bytearray(payload)
        invalid[11] = count+1
        for bad_payload in (invalid, payload[:-1], payload+bytes(RECORD_SIZE)):
            try:
                telemetry(bad_payload)
            except ValueError:
                pass
            else:
                raise AssertionError("Invalid count/length accepted")
        invalid = bytearray(payload)
        invalid[HEADER_SIZE+RECORD_SIZE] = 1
        try:
            telemetry(invalid)
        except ValueError:
            pass
        else:
            raise AssertionError("Duplicate identity accepted")
        # Record order is not joint identity.
        reversed_payload = payload[:HEADER_SIZE] + b"".join(
            payload[HEADER_SIZE+i*RECORD_SIZE:HEADER_SIZE+(i+1)*RECORD_SIZE]
            for i in reversed(range(count)))
        assert telemetry(reversed_payload)["joints"][0]["id"] == count

    # Lose request, then lose reply: identical retry, one execution.
    import threading
    writes, executions, cached = [], [], {}
    def write(packet):
        kind, body = Parser().feed(packet)[0]
        writes.append(packet)
        if kind == SESSION:
            reliable.receive(SESSION_ACK, body)
        elif len(writes) > 2:  # First request never reaches the MCU.
            identifier = struct.unpack_from("<I", body, 8)[0]
            if identifier not in cached:
                executions.append(identifier)
                cached[identifier] = body[:12] + bytes((body[16], 0)) + body[18:24]
            else:  # Its first reply was lost; now deliver the cached response.
                reliable.receive(RELIABLE_RESPONSE, cached[identifier])
        return len(packet)
    reliable = ReliableRPC(write, threading.Event())
    reliable.RETRY_SECONDS = 0.001
    reliable.start()
    assert reliable.request(b"\x01" + bytes(7), 1234)[1] == 0
    assert executions == [1] and writes[1] == writes[2] == writes[3]
    dead = ReliableRPC(lambda packet: len(packet), threading.Event())
    dead.RETRY_SECONDS = 0.001
    try:
        dead.start()
    except TimeoutError:
        assert dead.failed
    else:
        raise AssertionError("Missing session ACK accepted")

    attempts = []
    def no_reply(packet):
        kind, body = Parser().feed(packet)[0]
        attempts.append(packet)
        if kind == SESSION:
            unresolved.receive(SESSION_ACK, body)
        else:
            # Wrong ID/session must never satisfy the current RPC.
            unresolved.receive(RELIABLE_RESPONSE, bytes(20))
        return len(packet)
    unresolved = ReliableRPC(no_reply, threading.Event())
    unresolved.RETRY_SECONDS = 0.001
    unresolved.start()
    try:
        unresolved.request(b"\x01" + bytes(7), 1234)
    except TimeoutError:
        assert unresolved.failed and len(attempts) == 4
        assert attempts[1] == attempts[2] == attempts[3]
    else:
        raise AssertionError("Unmatched reply accepted")
    try:
        unresolved.request(b"\x02" + bytes(7), 1235)
    except RuntimeError:
        assert len(attempts) == 4  # Never skip an unresolved transaction ID.
    else:
        raise AssertionError("Unresolved action was silently bypassed")


if __name__ == "__main__":
    self_test()
    print("PASS: serial framing/CRC, bounds/ages, bounded retry, matching and unresolved-ID latch.")
