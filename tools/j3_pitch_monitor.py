"""Unified USB five-motor + dual-Pitch monitor; no motor control.

The historical filename remains the entry point; no per-joint capture mode.
"""
import argparse
import csv
import math
import secrets
import struct
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

from dm_serial import encode, FOCUS_REQUEST, FOCUS_ACK, FOCUS_SAMPLE
from usb_monitor import Stream, serial_port, HEADER, RECORD, decode

VERSION = 0x594C5002
SAMPLE = struct.Struct("<IIBBBBH")
IMU = struct.Struct("<BBHIIf")
MOTOR_FIELDS = ("at_ms", "count", "raw_deg", "velocity_rad_s", "torque_nm",
                "flags", "state", "mos_c", "rotor_c")
CSV_FIELDS = (
    ("host_utc", "sequence", "tick_ms", "max_age_ms", "imu_mount_offset_deg")
    + tuple(f"j{j}_{name}" for j in range(1, 6) for name in MOTOR_FIELDS)
    + tuple(f"{source}_{name}" for source in ("bmi", "yummy")
            for name in ("at_ms", "count", "pitch_deg", "flags")))
LOG_DIR = Path(__file__).resolve().parents[1] / "build" / "j3-pitch-capture"


def decode_sample(payload):
    if len(payload) < SAMPLE.size:
        raise ValueError("Truncated unified sample")
    seq, tick, motors, imus, motor_size, imu_size, limit = SAMPLE.unpack_from(payload)
    if (motors, imus, motor_size, imu_size) != (5, 2, RECORD.size, IMU.size) or not limit:
        raise ValueError("Expected five motors and both IMU Pitches")
    if len(payload) != SAMPLE.size + motors * motor_size + imus * imu_size:
        raise ValueError("Invalid unified sample length")
    row = dict(host_utc=datetime.now(timezone.utc).isoformat(), sequence=seq,
               tick_ms=tick, max_age_ms=limit, imu_mount_offset_deg=0.0)
    offset = SAMPLE.size
    for i in range(motors):
        r = decode(HEADER.pack(b"YL", 1, motors, seq, tick, limit, motor_size, i)
                   + payload[offset:offset+motor_size])
        joint = i+1
        if (r["joint"], r["bus"], r["protocol"], r["command_id"], r["feedback_id"]) != (
                joint, 2 if joint <= 2 else 1, int(joint == 3), joint, 0x10+joint):
            raise ValueError("Motor identity does not match the five-axis board")
        if r["received"] and not r["received_count"]:
            raise ValueError("Received motor without a source count")
        values = (r["received_tick_ms"], r["received_count"],
                  math.degrees(r["position_rad"]) if r["received"] else None,
                  r["velocity_rad_s"], r["torque_nm"],
                  int(r["received"]) | int(r["fresh"]) << 1,
                  r["state"], r["mos_c"], r["rotor_c"])
        row.update((f"j{joint}_{key}", value) for key, value in zip(MOTOR_FIELDS, values))
        offset += motor_size
    for expected, source in enumerate(("bmi", "yummy"), 1):
        identity, flags, reserved, at, count, angle = IMU.unpack_from(payload, offset)
        if identity != expected or flags not in (0, 1, 3) or reserved:
            raise ValueError("Invalid IMU metadata")
        if flags & 1:
            if not count or not math.isfinite(angle):
                raise ValueError("Invalid received IMU")
            if flags & 2 and ((tick-at) & 0xffffffff) > limit:
                raise ValueError("IMU freshness contradicts its source time")
        elif at or count or not math.isnan(angle):
            raise ValueError("Absent IMU must not look like zero")
        row.update({f"{source}_at_ms": at, f"{source}_count": count,
                    f"{source}_pitch_deg": angle if flags & 1 else None,
                    f"{source}_flags": flags})
        offset += imu_size
    return row


class Link:
    """One observational writer; no control SESSION, retry, motor RPC or recovery."""
    def __init__(self, port):
        self.port, self.stream = port, Stream()
        self.pending = None
        self.mode = None
        self.last_tick = None

    def command(self, mode):
        if self.pending:
            raise RuntimeError("上一条采集命令尚未确认")
        if mode not in (0, 1, 2):
            raise ValueError("Invalid focus mode")
        nonce = secrets.token_bytes(8)
        frame = encode(FOCUS_REQUEST, nonce + bytes([mode]))
        self.pending = (nonce, mode, time.monotonic() + .8)
        if self.port.write(frame) != len(frame):
            raise OSError("USB 写入不完整，结果未知；不自动重发")

    def pump(self):
        events = []
        waiting = min(8192, self.port.in_waiting)
        data = self.port.read(waiting) if waiting else b""
        for kind, payload in self.stream.messages(data):
            if kind == FOCUS_ACK:
                nonce, mode, rc, version, tick = struct.unpack("<8sBBII", payload)
                if self.pending and nonce == self.pending[0]:
                    if mode != self.pending[1] or version != VERSION or rc:
                        raise RuntimeError(f"采集确认失败：mode={mode}, rc={rc}, version={version:08x}")
                    self.pending, self.mode = None, mode
                    events.append(("ack", mode))
            elif kind == FOCUS_SAMPLE and self.mode == 1:
                row = decode_sample(payload)
                if self.last_tick is not None and ((row["tick_ms"]-self.last_tick) & 0xffffffff) > 0x7fffffff:
                    raise RuntimeError("MCU 已重启；采集停止，不自动恢复")
                self.last_tick = row["tick_ms"]
                events.append(("sample", row))
        if self.pending and time.monotonic() >= self.pending[2]:
            raise TimeoutError("采集命令无应答；需要五轴/双IMU统一观测固件，不自动重试")
        return events


class App:
    def __init__(self, root, port=""):
        import tkinter as tk
        from tkinter import ttk
        self.root = root
        self.link = self.context = self.log = None
        self.writer = None
        self.points = deque(maxlen=2000)
        self.count = 0
        self.last_draw = self.last_sample = 0.0
        self.latest = None
        self.start_on_connect = False
        root.title("Ylamp · 五轴编码器 / BMI088 + Yummy Pitch")
        root.geometry("1080x730")
        root.minsize(840, 580)
        outer = ttk.Frame(root, padding=14)
        outer.pack(fill="both", expand=True)
        row = ttk.Frame(outer)
        row.pack(fill="x")
        self.port_name = tk.StringVar(value=port)
        self.ports = ttk.Combobox(row, textvariable=self.port_name, width=14)
        self.ports.pack(side="left")
        ttk.Button(row, text="刷新串口", command=self.refresh).pack(side="left", padx=6)
        self.connect_button = ttk.Button(row, text="连接", command=self.connect)
        self.connect_button.pack(side="left")
        self.start_button = ttk.Button(row, text="开始 / 继续", command=self.start, state="disabled")
        self.start_button.pack(side="left", padx=6)
        self.pause_button = ttk.Button(row, text="暂停", command=self.pause, state="disabled")
        self.pause_button.pack(side="left")
        ttk.Label(outer, text="仅采集：开始/暂停不启停电机 · IMU 安装偏角暂按 0° · 不写零点").pack(anchor="w", pady=8)
        self.status = tk.StringVar(value="未连接")
        ttk.Label(outer, textvariable=self.status).pack(anchor="w")
        values = ttk.Frame(outer)
        values.pack(fill="x", pady=12)
        self.angles = [tk.StringVar(value=f"J{j} 原始编码器：--") for j in range(1, 6)]
        for i, angle in enumerate(self.angles):
            ttk.Label(values, textvariable=angle, font=("Consolas", 13)).grid(
                row=i//3, column=i%3, sticky="w", padx=(0, 20), pady=3)
        self.pitches = {name: tk.StringVar(value=label+"：--")
                        for name, label in (("bmi", "BMI088 Pitch"), ("yummy", "Yummy Pitch"))}
        for i, pitch in enumerate(self.pitches.values()):
            ttk.Label(values, textvariable=pitch, font=("Consolas", 14)).grid(
                row=2, column=i, sticky="w", pady=7)
        self.canvas = tk.Canvas(outer, bg="#101824", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.csv_path = tk.StringVar(value="CSV：首次开始后自动记录，暂停保留同一文件")
        ttk.Label(outer, textvariable=self.csv_path, wraplength=870).pack(anchor="w", pady=8)
        self.refresh()
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(20, self.poll)

    def refresh(self):
        from serial.tools import list_ports
        names = [p.device for p in list_ports.comports()]
        self.ports["values"] = names
        if not self.port_name.get() and names:
            self.port_name.set("COM36" if "COM36" in names else names[0])

    def connect(self):
        if self.link:
            self.disconnect()
            return
        try:
            self.context = serial_port(self.port_name.get())
            port = self.context.__enter__()
            port.timeout = 0
            self.link = Link(port)
            self.points.clear()
            self.latest = None
            self.count = 0
            self.last_sample = 0
            for j, angle in enumerate(self.angles, 1):
                angle.set(f"J{j} 原始编码器：--")
            for name, pitch in self.pitches.items():
                pitch.set(("BMI088" if name=="bmi" else "Yummy")+" Pitch：--")
            self.link.command(2)  # Explicit connect selects PAUSE, never starts motors/data.
            self.connect_button["text"] = "断开"
            self.status.set("连接中：请求暂停并核对固件版本…")
        except Exception as exc:
            self.disconnect(str(exc))

    def start(self):
        try:
            if self.log is None:
                LOG_DIR.mkdir(parents=True, exist_ok=True)
                path = LOG_DIR / f"five_pitch_{datetime.now():%Y%m%d_%H%M%S_%f}.csv"
                self.log = path.open("x", newline="", encoding="utf-8-sig")
                self.writer = csv.DictWriter(self.log, CSV_FIELDS)
                self.writer.writeheader()
                self.csv_path.set(f"CSV：{path}")
            self.link.command(1)
            self.set_pending("正在开始…")
        except Exception as exc:
            self.disconnect(str(exc))

    def pause(self):
        try:
            self.link.command(2)
            self.set_pending("正在暂停，等待板端确认…")
        except Exception as exc:
            self.disconnect(str(exc))

    def set_pending(self, text):
        self.start_button["state"] = self.pause_button["state"] = "disabled"
        self.status.set(text)

    def poll(self):
        try:
            if self.link:
                for kind, value in self.link.pump():
                    if kind == "ack":
                        running = value == 1
                        self.start_button["state"] = "disabled" if running else "normal"
                        self.pause_button["state"] = "normal" if running else "disabled"
                        self.status.set("采集中 · 五轴 + 双Pitch" if running else f"已暂停 · 已记录 {self.count} 帧")
                        if self.log:
                            self.log.flush()
                        if value == 2 and self.start_on_connect:
                            self.start_on_connect = False
                            self.start()
                    else:
                        self.latest = value
                        self.last_sample = time.monotonic()
                        self.count += 1
                        if self.writer:
                            self.writer.writerow(value)
                            if self.count % 100 == 0:
                                self.log.flush()
                        self.points.append(value)  # Each channel keeps its own freshness.
                now = time.monotonic()
                if now - self.last_draw >= .1:
                    self.last_draw = now
                    self.draw()
                    if self.link.mode == 1 and self.last_sample and now - self.last_sample > .3:
                        self.status.set("数据流过期 · 不自动重连/使能")
        except Exception as exc:
            self.disconnect(str(exc))
        self.root.after(20, self.poll)

    def draw(self):
        if self.latest:
            r = self.latest
            extra_ms = max(0, time.monotonic()-self.last_sample)*1000
            def shown(source, name):
                if not self.link or self.link.mode != 1:
                    return "--（采集已暂停）"
                fresh = (r[f"{source}_flags"] & 2 and
                         ((r["tick_ms"]-r[f"{source}_at_ms"]) & 0xffffffff)+extra_ms <= r["max_age_ms"])
                return f"{r[name]:+.4f}°" if fresh else "--（无新鲜数据）"
            for j, angle in enumerate(self.angles, 1):
                angle.set(f"J{j} 原始编码器："+shown(f"j{j}", f"j{j}_raw_deg"))
            for name, pitch in self.pitches.items():
                pitch.set(("BMI088" if name=="bmi" else "Yummy")+" Pitch："+shown(name, f"{name}_pitch_deg"))
            if self.link and self.link.mode == 1:
                self.status.set(f"统一采集 · {self.count} 帧 · 五轴及双IMU独立新鲜度 · CRC/解析丢弃={self.link.stream.malformed}"
                                + (" · BMI088预热/等待新数据" if not r["bmi_flags"] & 2 else ""))
        c = self.canvas
        c.delete("all")
        w, h = c.winfo_width(), c.winfo_height()
        if w < 100 or h < 180:
            return
        points = list(self.points)
        for panel, (joint, source, label) in enumerate(((2, "bmi", "BMI088"), (3, "yummy", "Yummy"))):
            top, height = panel*h/2, h/2
            c.create_text(14, top+12, anchor="nw", fill="#69b8ff", text=f"蓝：J{joint} 原始编码器")
            c.create_text(250, top+12, anchor="nw", fill="#ffc86b", text=f"橙：{label} Pitch")
            columns = ((f"j{joint}_raw_deg", f"j{joint}_flags", "#69b8ff"),
                       (f"{source}_pitch_deg", f"{source}_flags", "#ffc86b"))
            values = [p[key] for p in points for key, flags, _ in columns if p[flags] & 2]
            if len(points) < 2 or not values:
                continue
            lo, hi = min(values), max(values)
            margin = max(.1, (hi-lo)*.1)
            lo, hi = lo-margin, hi+margin
            span = max(1, (points[-1]["tick_ms"]-points[0]["tick_ms"]) & 0xffffffff)
            for key, flags, color in columns:
                xy = []
                for p in points:
                    if not p[flags] & 2:
                        if len(xy) >= 4:
                            c.create_line(*xy, fill=color, width=2)
                        xy = []
                        continue
                    xy.extend((55+((p["tick_ms"]-points[0]["tick_ms"]) & 0xffffffff)/span*(w-70),
                               top+height-25-(p[key]-lo)/(hi-lo)*(height-75)))
                if len(xy) >= 4:
                    c.create_line(*xy, fill=color, width=2)
            c.create_text(7, top+40, anchor="nw", fill="#aab7c5", text=f"{hi:.2f}°")
            c.create_text(7, top+height-30, anchor="nw", fill="#aab7c5", text=f"{lo:.2f}°")

    def disconnect(self, message="已断开；电机状态未被工具改变"):
        self.start_on_connect = False
        self.link = None
        if self.context:
            self.context.__exit__(None, None, None)
            self.context = None
        if self.log:
            self.log.close()
            self.log = self.writer = None
        self.start_button["state"] = self.pause_button["state"] = "disabled"
        self.connect_button["text"] = "连接"
        self.status.set(message)
        self.latest = None

    def close(self):
        self.disconnect()
        self.root.destroy()


def self_test():
    from unittest.mock import patch
    def sample(missing=False, warm=True, tick=100, at=98):
        motors = b"".join(RECORD.pack(j, 2 if j<=2 else 1, int(j==3),
            0 if missing else 3, j, 0x10+j, 0 if missing else at, 0 if missing else j,
            math.nan if missing else j*.1, math.nan if missing else 0,
            math.nan if missing else 0, 255 if missing else 1, 30, 31, 0)
            for j in range(1, 6))
        return (SAMPLE.pack(1, tick, 5, 2, 32, 16, 40)+motors
                + IMU.pack(1, 3 if warm else 1, 0, at, 3, 45)
                + IMU.pack(2, 3, 0, at, 7, 4.5))
    data = sample()
    decoded = decode_sample(data)
    assert abs(decoded["j3_raw_deg"]-math.degrees(.3)) < 1e-5
    assert decoded["bmi_pitch_deg"]==45 and decoded["yummy_pitch_deg"]==4.5
    assert set(decoded)==set(CSV_FIELDS)
    absent = decode_sample(sample(missing=True, warm=False))
    assert absent["j3_raw_deg"] is None and absent["j1_flags"]==0 and absent["bmi_flags"]==1
    assert decode_sample(sample(tick=3, at=0xfffffffb))["j5_flags"]==3
    bad_count = bytearray(data);bad_count[8]=6
    bad_imu = bytearray(data);bad_imu[SAMPLE.size+5*32]=2
    stale = bytearray(data);struct.pack_into("<I", stale, 4, 500)
    for payload in (data[:-1], bad_count, bad_imu, stale):
        try:
            decode_sample(payload)
        except ValueError:
            pass
        else:
            raise AssertionError("Bad unified sample accepted")
    for size in (1, 7, 64, 213, 4096):
        stream = Stream()
        wire = encode(FOCUS_SAMPLE, data)
        result = [m for i in range(0, len(wire), size) for m in stream.messages(wire[i:i+size])]
        assert result==[(FOCUS_SAMPLE, data)] and not stream.buffer
    events, incoming = [], bytearray()
    class FakePort:
        @property
        def in_waiting(self):
            return len(incoming)
        def read(self, n):
            chunk = bytes(incoming[:n]); del incoming[:n]; return chunk
        def write(self, frame):
            events.append(frame); return len(frame)
    link = Link(FakePort())
    for mode in (2, 1, 2, 1, 0):
        link.command(mode)
        assert events[-1][3] == FOCUS_REQUEST
        nonce = link.pending[0]
        ack = encode(FOCUS_ACK, struct.pack("<8sBBII", nonce, mode, 0, VERSION, 100))
        incoming.extend(ack[:3]); assert not link.pump()
        incoming.extend(ack[3:] + encode(FOCUS_SAMPLE, data))
        result = link.pump()
        assert result[0] == ("ack", mode)
        assert sum(kind == "sample" for kind, _ in result) == (mode == 1)
    assert len(events) == 5  # No retry, control SESSION or actuator request.
    link.command(2)
    with patch("time.monotonic", return_value=link.pending[2]+1):
        try:
            link.pump()
        except TimeoutError:
            pass
        else:
            raise AssertionError("No timeout")
    assert len(events) == 6
    # Old pair-only firmware must not be mistaken for the new unified payload.
    incoming.clear()
    old = Link(FakePort());old.command(2)
    incoming.extend(encode(FOCUS_ACK, struct.pack("<8sBBII", old.pending[0], 2, 0, 0x594C5001, 100)))
    try:
        old.pump()
    except RuntimeError:
        pass
    else:
        raise AssertionError("Old pair-only firmware accepted")
    assert all(frame[3]==FOCUS_REQUEST for frame in events)
    print("Unified five-motor/dual-Pitch codec/fragmentation/absence/wrap/start-pause/version/no motor RPC: PASS")


def probe(port, seconds):
    with serial_port(port) as serial:
        serial.timeout = 0
        link = Link(serial)
        def wait_ack():
            while link.pending:
                link.pump()
                time.sleep(.005)
        link.command(1); wait_ack()
        count, last = 0, None
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            for kind, value in link.pump():
                if kind == "sample":
                    count += 1; last = value
            time.sleep(.003)
        link.command(2); wait_ack()
        quiet_end = time.monotonic()+.2
        # Pause ACK follows any prior sample on this single USB IN stream.
        while time.monotonic() < quiet_end:
            assert not any(k == FOCUS_SAMPLE for k, _ in link.stream.messages(
                serial.read(min(8192, serial.in_waiting)))), "New data after PAUSE ACK"
            time.sleep(.005)
        print(f"samples={count} avg_hz={count/seconds:.1f} pause_verified=True last={last}")
        assert count, "No focus samples"


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", default="")
    p.add_argument("--self-test", action="store_true")
    p.add_argument("--probe-seconds", type=float)
    p.add_argument("--start", action="store_true", help="Connect and start DATA after version ACK")
    a = p.parse_args()
    if a.self_test:
        self_test()
    elif a.probe_seconds is not None:
        if not a.port or not math.isfinite(a.probe_seconds) or not .1 <= a.probe_seconds <= 30:
            p.error("probe needs --port and 0.1..30 seconds")
        probe(a.port, a.probe_seconds)
    else:
        import tkinter as tk
        root = tk.Tk()
        app = App(root, a.port)
        if a.start:
            app.start_on_connect = True
            root.after(100, app.connect)
        root.mainloop()


if __name__ == "__main__":
    main()
