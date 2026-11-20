"""USB J3/Yummy or J2/BMI angle monitor; no motor control."""
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
from usb_monitor import Stream, serial_port

VERSION = 0x594C5001
J2_VERSION = 0x594C3202
SAMPLE = struct.Struct("<6I2fBBH")
CSV_FIELDS = ("host_utc", "sequence", "tick_ms", "j3_at_ms", "j3_count",
              "imu_at_ms", "imu_count", "j3_raw_deg", "pitch_deg", "flags",
              "j3_state", "max_age_ms", "imu_mount_offset_deg")
LOG_DIR = Path(__file__).resolve().parents[1] / "build" / "j3-pitch-capture"


def decode_sample(payload):
    if len(payload) != SAMPLE.size:
        raise ValueError("Invalid focus sample length")
    values = SAMPLE.unpack(payload)
    row = dict(zip(CSV_FIELDS[1:-1], values))
    if row["flags"] & ~15 or not row["max_age_ms"]:
        raise ValueError("Invalid sample flags/age limit")
    for name, received, fresh in (("j3_raw_deg", 1, 2), ("pitch_deg", 4, 8)):
        value = row[name]
        if row["flags"] & fresh and not row["flags"] & received:
            raise ValueError("Fresh without a source")
        if row["flags"] & received:
            if not math.isfinite(value):
                raise ValueError("Non-finite received angle")
            at = row["j3_at_ms" if received == 1 else "imu_at_ms"]
            if row["flags"] & fresh and ((row["tick_ms"]-at) & 0xffffffff) > row["max_age_ms"]:
                raise ValueError("Fresh flag contradicts source timestamp")
        elif not math.isnan(value):
            raise ValueError("Absent source must not look like zero")
        else:
            row[name] = None
    row["imu_mount_offset_deg"] = 0.0  # User-requested working assumption only.
    row["host_utc"] = datetime.now(timezone.utc).isoformat()
    return row


class Link:
    """One observational writer; no control SESSION, retry, motor RPC or recovery."""
    def __init__(self, port, joint=3):
        self.joint = joint
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
        wire_mode = {0: 0, 1: 3, 2: 4}[mode] if self.joint == 2 else mode
        frame = encode(FOCUS_REQUEST, nonce + bytes([wire_mode]))
        self.pending = (nonce, wire_mode, time.monotonic() + .8)
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
                    expected = J2_VERSION if mode in (3, 4) else VERSION
                    if mode != self.pending[1] or version != expected or rc:
                        raise RuntimeError(f"采集确认失败：mode={mode}, rc={rc}, version={version:08x}")
                    logical = {3: 1, 4: 2}.get(mode, mode)
                    self.pending, self.mode = None, logical
                    events.append(("ack", logical))
            elif kind == FOCUS_SAMPLE and self.mode == 1:
                row = decode_sample(payload)
                if self.last_tick is not None and ((row["tick_ms"]-self.last_tick) & 0xffffffff) > 0x7fffffff:
                    raise RuntimeError("MCU 已重启；采集停止，不自动恢复")
                self.last_tick = row["tick_ms"]
                events.append(("sample", row))
        if self.pending and time.monotonic() >= self.pending[2]:
            raise TimeoutError("采集命令无应答；需要双通道新版固件，不自动重试")
        return events


class App:
    def __init__(self, root, port="", joint=3):
        import tkinter as tk
        from tkinter import ttk
        self.root = root
        self.joint = joint
        self.encoder_label = f"J{joint} 原始编码器"
        self.imu_label = "BMI088 Pitch" if joint == 2 else "Yummy Pitch"
        self.log_dir = LOG_DIR if joint == 3 else LOG_DIR.parent / "j2-pitch-capture"
        self.csv_fields = [self.csv_name(k) for k in CSV_FIELDS]
        self.link = self.context = self.log = None
        self.writer = None
        self.points = deque(maxlen=2000)
        self.count = 0
        self.last_draw = self.last_sample = 0.0
        self.latest = None
        self.start_on_connect = False
        root.title(f"Ylamp · J{joint} 编码器 / {self.imu_label}")
        root.geometry("920x570")
        root.minsize(720, 420)
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
        self.angle = tk.StringVar(value=self.encoder_label + "：--")
        self.pitch = tk.StringVar(value=self.imu_label + "：--")
        ttk.Label(values, textvariable=self.angle, font=("Consolas", 17)).pack(side="left")
        ttk.Label(values, textvariable=self.pitch, font=("Consolas", 17)).pack(side="right")
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

    def csv_name(self, name):
        if self.joint == 2:
            return name.replace("j3", "j2").replace("pitch_deg", "bmi_pitch_deg")
        return name

    def connect(self):
        if self.link:
            self.disconnect()
            return
        try:
            self.context = serial_port(self.port_name.get())
            port = self.context.__enter__()
            port.timeout = 0
            self.link = Link(port, self.joint)
            self.points.clear()
            self.latest = None
            self.count = 0
            self.last_sample = 0
            self.angle.set(self.encoder_label + "：--（等待新数据）")
            self.pitch.set(self.imu_label + "：--（等待新数据）")
            self.link.command(2)  # Explicit connect selects PAUSE, never starts motors/data.
            self.connect_button["text"] = "断开"
            self.status.set("连接中：请求暂停并核对固件版本…")
        except Exception as exc:
            self.disconnect(str(exc))

    def start(self):
        try:
            if self.log is None:
                self.log_dir.mkdir(parents=True, exist_ok=True)
                path = self.log_dir / f"j{self.joint}_pitch_{datetime.now():%Y%m%d_%H%M%S_%f}.csv"
                self.log = path.open("x", newline="", encoding="utf-8-sig")
                self.writer = csv.DictWriter(self.log, self.csv_fields)
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
                        self.status.set("采集中 · 两路均为度" if running else f"已暂停 · 已记录 {self.count} 帧")
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
                            self.writer.writerow({self.csv_name(k): v for k, v in value.items()})
                            if self.count % 100 == 0:
                                self.log.flush()
                        # Invalid/stale samples remain documented, but never plot as fresh.
                        if value["flags"] == 15:
                            self.points.append((value["tick_ms"], value["j3_raw_deg"], value["pitch_deg"]))
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
            def shown(name, bit):
                if not self.link or self.link.mode != 1:
                    return "--（采集已暂停）"
                if self.joint == 2 and name == "pitch_deg" and not r["flags"] & bit:
                    return "--（BMI088预热 / 等待数据）"
                return f"{r[name]:+.4f}°" if r["flags"] & bit else "--（无新鲜反馈）"
            self.angle.set(self.encoder_label + "：" + shown("j3_raw_deg", 2))
            self.pitch.set(self.imu_label + "：" + shown("pitch_deg", 8))
            if self.link and self.link.mode == 1:
                self.status.set(f"采集中 · {self.count} 帧 · J{self.joint} state={r['j3_state']} · CRC/解析丢弃={self.link.stream.malformed}")
                if self.joint == 2 and not r["flags"] & 8:
                    self.status.set(f"BMI088预热约5秒 / 等待新数据 · 已记录 {self.count} 帧")
        c = self.canvas
        c.delete("all")
        w, h = c.winfo_width(), c.winfo_height()
        c.create_text(14, 14, anchor="nw", fill="#69b8ff", text="蓝：" + self.encoder_label)
        c.create_text(220, 14, anchor="nw", fill="#ffc86b", text="橙：" + self.imu_label)
        if len(self.points) < 2 or w < 100 or h < 90:
            return
        points = list(self.points)
        lo = min(min(p[1:]) for p in points)
        hi = max(max(p[1:]) for p in points)
        margin = max(.1, (hi-lo)*.1)
        lo, hi = lo-margin, hi+margin
        span = max(1, (points[-1][0]-points[0][0]) & 0xffffffff)
        for column, color in ((1, "#69b8ff"), (2, "#ffc86b")):
            xy = []
            for p in points:
                xy.extend((45+((p[0]-points[0][0]) & 0xffffffff)/span*(w-60),
                           h-25-(p[column]-lo)/(hi-lo)*(h-70)))
            c.create_line(*xy, fill=color, width=2)
        c.create_text(7, 40, anchor="nw", fill="#aab7c5", text=f"{hi:.2f}°")
        c.create_text(7, h-30, anchor="nw", fill="#aab7c5", text=f"{lo:.2f}°")

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
    data = SAMPLE.pack(1, 100, 98, 5, 97, 7, 19.5, 4.5, 15, 1, 40)
    assert decode_sample(data)["j3_raw_deg"] == 19.5
    absent = SAMPLE.pack(2, 100, 0, 0, 97, 7, math.nan, 4.5, 12, 255, 40)
    assert decode_sample(absent)["j3_raw_deg"] is None
    for payload in (data[:-1], SAMPLE.pack(1, 100, 0, 0, 97, 7, math.nan, 4.5, 15, 1, 40)):
        try:
            decode_sample(payload)
        except ValueError:
            pass
        else:
            raise AssertionError("Bad sample accepted")
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
    # Identical framing, but J2 has an explicit distinct profile/version.
    incoming.clear()
    j2 = Link(FakePort(), 2)
    for logical, wire in ((2, 4), (1, 3), (2, 4), (1, 3)):
        j2.command(logical)
        assert events[-1][13] == wire
        nonce = j2.pending[0]
        incoming.extend(encode(FOCUS_ACK, struct.pack("<8sBBII", nonce, wire, 0, J2_VERSION, 100)))
        incoming.extend(encode(FOCUS_SAMPLE, data))
        result = j2.pump()
        assert result[0] == ("ack", logical)
        assert sum(kind == "sample" for kind, _ in result) == (logical == 1)
    print("Focus codec/fragmentation/missing source/start-pause-resume/nonce/no replay: PASS")


def probe(port, seconds, joint=3):
    with serial_port(port) as serial:
        serial.timeout = 0
        link = Link(serial, joint)
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
    p.add_argument("--joint", type=int, choices=(2, 3), default=3)
    p.add_argument("--self-test", action="store_true")
    p.add_argument("--probe-seconds", type=float)
    p.add_argument("--start", action="store_true", help="Connect and start DATA after version ACK")
    a = p.parse_args()
    if a.self_test:
        self_test()
    elif a.probe_seconds is not None:
        if not a.port or not math.isfinite(a.probe_seconds) or not .1 <= a.probe_seconds <= 30:
            p.error("probe needs --port and 0.1..30 seconds")
        probe(a.port, a.probe_seconds, a.joint)
    else:
        import tkinter as tk
        root = tk.Tk()
        app = App(root, a.port, a.joint)
        if a.start:
            app.start_on_connect = True
            root.after(100, app.connect)
        root.mainloop()


if __name__ == "__main__":
    main()
