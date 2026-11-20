"""Transport-independent MCU Task RPC; USB quick controls, CAN/UART full editor."""
from __future__ import annotations

import csv
import json
import math
import queue
import re
import struct
import time
from dataclasses import dataclass
from pathlib import Path

REQUEST_ID, RESPONSE_ID, VERSION = 0x7E0, 0x7E1, 0x594C0003
PROTECTIONS = ((2, "跟随"), (4, "过温"))
ALL_PROTECTIONS = sum(mask for mask, _ in PROTECTIONS)
CALIBRATION_VERSION = 0x594C4302
DRAG_VERSION = 0x594C4402
MOTION_VERSION = 0x594C5002
J3_FEEDBACK_VERSION = 0x594C4A01
UART_IMU_VERSION = 0x594C4901
JOINT_REFERENCE_VERSION = 0x594C5202
J3_FEEDBACK_PHASES = ("未采集", "确认失能", "预装中性MIT", "确认使能",
                      "采集中（100Hz中性MIT）", "停止：确认失能", "失能未确认")
J3_FEEDBACK_REASONS = ("无", "反馈过期", "电机状态异常", "过温", "CAN提交失败",
                       "使能确认超时", "失能确认超时")
DRAG_PHASES = ("关闭/仅预览", "建立支撑", "自由拖动", "转保持", "保持", "释放保持")
IMU_AXES = ("未确认", "+X", "-X", "+Y", "-Y", "+Z", "-Z")
TASK_STATES = ("空闲（未使能）", "使能确认中", "运行", "转保持中", "保持（有力矩）", "故障锁定",
               "CAN自动恢复（可能自动出力）")
REASONS = ("无", "策略未放行", "缺少反馈", "反馈过期", "实际角度越界", "电机/反馈状态异常",
           "过温", "使能前未静止", "跟随误差", "调度超时", "CAN提交失败", "使能确认超时",
           "人工失能")
RESULTS = ("成功", "参数/组合不合法", "设备未就绪或请求过期",
           "忙：需要失能近静止，或参数版本已变化", "资源不足", "Task拒绝：查看状态/原因")


@dataclass(frozen=True)
class Field:
    scope: str
    id: int
    member: str
    storage: str
    unit: str
    low: float
    high: float
    label: str
    group: str


def load_fields():
    path = Path(__file__).resolve().parents[1] / "config/motor_tuning_fields.def"
    fields = []
    for line in path.read_text(encoding="utf-8").splitlines():
        match = re.fullmatch(r"TUNE_(JOINT|GLOBAL)\((.*)\)", line.strip())
        if match:
            values = next(csv.reader([match[2]], skipinitialspace=True))
            id_, member, storage, unit, low, high, label, group = values
            fields.append(Field(match[1], int(id_), member, storage, unit,
                                float(low), float(high), label, group))
    if len(fields) != 24 or len({(f.scope, f.id) for f in fields}) != 24:
        raise RuntimeError("调参字段定义损坏或版本不兼容")
    return tuple(fields)


FIELDS = load_fields()
PARAMETERS = tuple((joint, field) for field in FIELDS
                   for joint in ((1, 2) if field.scope == "JOINT" else (0,)))


def gravity_angles(accel, axis=0):
    """Static sensor-frame roll/pitch and selected link-UP axis tilt, in degrees.

    Right-handed XYZ, level = +Z up; roll about X, pitch about Y.
    Gravity cannot observe yaw. These are not joint encoder angles.
    """
    if len(accel) != 3 or not all(math.isfinite(x) for x in accel):
        return (math.nan,) * 3
    x, y, z = accel
    magnitude = math.hypot(x, y, z)
    if not math.isfinite(magnitude) or magnitude <= 0.1:
        return (math.nan,) * 3
    yz = math.hypot(y, z)
    # At pitch +/-90 degrees gravity cannot determine roll.
    roll = math.degrees(math.atan2(y, z)) if yz > magnitude * 1e-6 else math.nan
    pitch = math.degrees(math.atan2(-x, yz))
    tilt = math.nan
    if 1 <= axis <= 6:
        component = accel[(axis - 1) // 2] * (1 if axis % 2 else -1) / magnitude
        tilt = math.degrees(math.acos(max(-1, min(1, component))))
    return roll, pitch, tilt


def key(joint, field):
    return f"{joint}.{field.id}"


def validate(values):
    if set(values) != {key(j, f) for j, f in PARAMETERS}:
        raise ValueError("参数不完整或含未知字段；请先读取设备")
    for joint, field in PARAMETERS:
        value = values[key(joint, field)]
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or not field.low <= value <= field.high
                or field.storage != "F32" and int(value) != value):
            raise ValueError(f"{'公共' if joint == 0 else f'J{joint}'} {field.label}："
                             f"范围 {field.low:g}..{field.high:g}，不得 NaN/Inf")
    global_values = {f.member: values[key(0, f)] for f in FIELDS if f.scope == "GLOBAL"}
    if global_values["feedback_max_age_ms"] > global_values["enable_timeout_ms"]:
        raise ValueError("使能确认超时必须 ≥ 反馈过期时间")
    for joint in (1, 2):
        v = {f.member: values[key(joint, f)] for f in FIELDS if f.scope == "JOINT"}
        if v["upper_rad"] - v["lower_rad"] <= math.degrees(100 / 65535):
            raise ValueError(f"J{joint} 上限必须高于下限，并留出位置量化余量")
        if v["upper_rad"] - v["lower_rad"] >= 360 - math.degrees(100 / 65535):
            raise ValueError(f"J{joint} 参考区间必须小于 360°，否则上电无法唯一选圈")
        if int(v["protection_flags"]) & ~ALL_PROTECTIONS:
            raise ValueError(f"J{joint} 保护开关仅支持跟随/过温；旧 SPEED 位已删除")
        if v["drag_kd_low"] > v["drag_kd_high"]:
            raise ValueError(f"J{joint} 低速阻尼不得大于高速阻尼")
        if v["drag_speed_low_rad_s"] >= v["drag_speed_high_rad_s"]:
            raise ValueError(f"J{joint} 阻尼低速阈值必须小于高速阈值")
    return dict(values)

def validate_motion(values):
    """Eight explicit UI values; no guessed mechanical v/a/j defaults."""
    if len(values) != 2 or any(len(axis) != 4 for axis in values):
        raise ValueError("每轴需要目标角、速度、加速度和 jerk")
    for axis in values:
        if any(isinstance(v, bool) or not isinstance(v, (int, float))
               or not math.isfinite(v) for v in axis) or any(v <= 0 for v in axis[1:]):
            raise ValueError("目标必须有限；速度、加速度和 jerk 必须为正数")
        # Avoid a packing overflow being mistaken for an unknown MCU action.
        for value in axis:
            struct.pack("<f", value)
    return values


class RpcRejected(RuntimeError):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status


class Client:
    """TX thread owns RPC; RX thread only delivers matched replies to a queue."""
    def __init__(self, send, events, stop):
        self.send, self.events, self.stop = send, events, stop
        self.replies = queue.Queue(maxsize=32)
        self.jobs = queue.Queue(maxsize=4)
        self.seq = 0
        self.supported = None
        self.calibration_supported = False
        self.drag_supported = False
        self.motion_supported = False
        self.capture_supported = False
        self.j3_feedback_supported = False
        self.j3_feedback_status = None
        self.j3_feedback_at = 0.0
        self.online = False
        self.next_status = 0.0
        self.snapshot = None
        self.task_status = None
        self.task_status_at = 0.0
        self.motion = None
        # Recorders already consume the fast telemetry. They can omit bulky
        # diagnostic snapshots without pretending the firmware lacks features.
        self.poll_details = True
        self.poll_imu = False
        self.uart_imu_supported = False  # USB-only, read-only external sensor

    def receive(self, data):
        if len(data) == 8:
            try:
                self.replies.put_nowait(bytes(data))
            except queue.Full:
                pass  # bounded; no automatic replay of a stale action

    def request(self, kind, payload=None):
        if self.stop.is_set():
            return "连接正在关闭"
        if not self.supported:
            return "尚未识别支持调参的 MCU 固件"
        stopping = (kind == "command" and (
            payload[0] if isinstance(payload, tuple) else payload) in (2, 3)
            or kind == "j3_feedback" and payload is False
            or kind == "imu" and (payload[0] if isinstance(payload, tuple) else payload) is False)
        if not self.online and kind != "read" and not stopping:
            return "MCU 状态未知，请先显式重新读取"
        try:
            self.jobs.put_nowait((kind, payload, time.perf_counter()))
        except queue.Full:
            return "操作队列已满"
        return ""

    def rpc(self, op, scope=0, field=0, value=0, floating=False):
        readonly = op in (0, 1, 6, 8, 11, 14, 15) or (
            op == 12 and (scope == 0 and field in (0, 5, 6, 7, 255)
                          or scope in (1, 2) and field in (11, 12, 13, 14))) or (
            op == 13 and scope == 3 and field in (0, 255))
        # Only reads may make fresh-sequence attempts here. The reliable UART
        # transport may retry an identical envelope with MCU deduplication;
        # CAN actions/commits are still submitted only once.
        for attempt in range(3 if readonly else 1):
            if self.stop.is_set():
                raise RuntimeError("连接正在关闭，未发送")
            self.seq = self.seq % 255 + 1
            data = bytes((self.seq, op, scope, field)) + struct.pack("<f" if floating else "<I", value)
            self.send(data)
            deadline = time.perf_counter() + (0.25 if readonly else 0.75)
            while not self.stop.is_set() and time.perf_counter() < deadline:
                try:
                    reply = self.replies.get(timeout=0.02)
                except queue.Empty:
                    continue
                if (reply[0], reply[2], reply[3]) != (self.seq, scope, field):
                    continue
                if reply[1] != 0:
                    reason = RESULTS[reply[1]] if reply[1] < len(RESULTS) else f"未知状态{reply[1]}"
                    raise RpcRejected(reply[1], f"{reason}（op={op}, scope={scope}, field={field}）")
                return struct.unpack("<f" if floating else "<I", reply[4:])[0]
        raise TimeoutError("MCU 只读查询连续超时/已断开；状态未知，请重新读取" if readonly else
                           "MCU 应答超时/已断开；未重发，实际结果未知，请重新读取")

    def read_all(self, emit=True):
        self.snapshot = None
        revision = self.rpc(6, field=1)
        values = {key(j, f): self.rpc(1, j, f.id, floating=True) for j, f in PARAMETERS}
        if self.rpc(6, field=1) != revision:
            raise RuntimeError("读取期间参数版本变化；本次快照无效，请重新读取")
        validate(values)
        self.online = True
        snapshot = {"revision": revision, "values": values}
        self.snapshot = snapshot
        if emit:
            self.events.put(("tuning_readback", snapshot))
        return snapshot

    def discover(self):
        try:
            if self.rpc(0) != VERSION:
                raise RuntimeError("MCU 调参协议版本不匹配")
            self.supported = True
            self.online = True
            self.events.put(("tuning_supported", True))
            self.read_all()
            self.drag_supported = self.rpc(11, field=255) == DRAG_VERSION
            if not self.drag_supported:
                raise RuntimeError("固件未提供本版拖动状态")
            try:
                self.calibration_supported = self.rpc(8, field=255) == CALIBRATION_VERSION
                if not self.calibration_supported:
                    raise RuntimeError("固件不支持本版零点标定")
            except Exception as exc:
                self.events.put(("calibration_error", str(exc)))
            try:
                motion_version = self.rpc(12, field=255)
                self.motion_supported = motion_version in (0x594C5001, MOTION_VERSION)
                self.capture_supported = motion_version == MOTION_VERSION
            except Exception:
                self.motion_supported = False  # old firmware stays monitor/Drag capable
                self.capture_supported = False
        except Exception as exc:
            # No auto reconnect/retry. Monitoring and physical KEY still work.
            if self.supported is None:
                self.supported = False
            self.online = False
            self.events.put(("tuning_error", str(exc)))

    def read_calibration(self, include_joints=True):
        captured_at = time.perf_counter()
        packed = self.rpc(8)  # freeze one coherent MCU snapshot, valid for 1s
        if packed >> 24 != 0xC1:
            raise RuntimeError("标定快照版本不匹配")
        snapshot = {
            "flags": packed & 255, "axis": (packed >> 8) & 255,
            "valid_mask": (packed >> 16) & 255,
            "age_ms": self.rpc(8, field=1), "stable_ms": self.rpc(8, field=2),
            "tolerance_deg": self.rpc(8, field=3, floating=True),
            "accel": [self.rpc(8, field=f, floating=True) for f in (4, 5, 6)],
            "gyro": [self.rpc(8, field=f, floating=True) for f in (7, 8, 9)],
            "joints": [[self.rpc(8, j, f, floating=True) for f in (1, 2, 3, 4)]
                       for j in (1, 2)] if include_joints else [],
            "at": captured_at,
        }
        if snapshot["axis"] > 6 or snapshot["valid_mask"] > 3:
            raise RuntimeError("标定快照损坏")
        self.events.put(("calibration_status", snapshot))
        return snapshot

    def read_drag(self):
        captured_at = time.perf_counter()
        packed = self.rpc(11)
        if packed >> 24 != 0xD1:
            raise RuntimeError("拖动快照版本不匹配")
        snapshot = {
            "valid": bool(packed & 1), "phase": (packed >> 8) & 255,
            "inertia": [self.rpc(11, field=f, floating=True) for f in (1, 2, 3)],
            "joints": [[self.rpc(11, j, f, floating=True) for f in range(1, 15)]
                       for j in (1, 2)],
            "at": captured_at,
        }
        if snapshot["phase"] >= len(DRAG_PHASES):
            raise RuntimeError("拖动阶段无效")
        self.events.put(("drag_status", snapshot))
        return snapshot

    def read_uart_imu(self):
        captured_at = time.perf_counter()
        packed = self.rpc(14)
        if packed >> 16 != 0xB100 or (packed >> 8) & 255 not in (0, 1) or packed & 255 > 7:
            raise RuntimeError("UART10 IMU 快照版本不匹配")
        snapshot = {
            "flags": packed & 255, "format": (packed >> 8) & 255,
            "age_ms": self.rpc(14, field=1), "count": self.rpc(14, field=2),
            "angles": [self.rpc(14, field=f, floating=True) for f in (3, 4, 5)],
            "rx_bytes": self.rpc(14, field=6), "bad_frames": self.rpc(14, field=7),
            "overflows": self.rpc(14, field=8), "uart_error": self.rpc(14, field=9),
            "at": captured_at,
        }
        if snapshot["flags"] & 2 and any(not math.isfinite(v) or abs(v) > 180
                                         for v in snapshot["angles"]):
            raise RuntimeError("UART10 IMU 姿态角无效")
        self.events.put(("uart_imu_status", snapshot))
        return snapshot

    def read_motion(self):
        captured = time.perf_counter()
        mode = self.rpc(12)
        snapshot = {"mode": mode, "at": captured, "valid": False}
        if mode == 1:
            packed = self.rpc(12, field=5)
            if packed >> 24 != 0xE1 or (packed >> 16) & 255 != mode:
                raise RuntimeError("位置阻抗快照不匹配")
            snapshot.update(valid=bool(packed & 1), phase=(packed >> 8) & 255,
                elapsed=self.rpc(12, field=6, floating=True),
                duration=self.rpc(12, field=7, floating=True),
                model=[[self.rpc(12, j, f, floating=True) for f in (11, 12, 13, 14)]
                       for j in (1, 2)])
        self.motion = snapshot
        self.events.put(("motion_status", snapshot))
        return snapshot

    def service(self):
        try:
            kind, payload, requested = self.jobs.get_nowait()
        except queue.Empty:
            kind, payload = None, None
        if kind:
            try:
                if kind == "read":
                    self.read_all()
                elif kind == "calibration_read":
                    self.read_calibration()
                elif kind == "apply":
                    values = validate(payload["values"])
                    self.rpc(2, value=payload["revision"])
                    try:
                        for j, f in PARAMETERS:
                            self.rpc(3, j, f.id, values[key(j, f)], floating=True)
                        revision = self.rpc(4)
                    except Exception:
                        # Abort only discards a partial RAM draft, never retries COMMIT.
                        try:
                            self.rpc(5)
                        except Exception:
                            pass
                        raise
                    snapshot = self.read_all(emit=False)
                    if snapshot["revision"] != revision or any(
                            not math.isclose(values[k], v, rel_tol=2e-6, abs_tol=2e-5)
                            for k, v in snapshot["values"].items()):
                        raise RuntimeError("提交后回读不一致；请勿启动，重新读取")
                    self.events.put(("tuning_applied", snapshot))
                elif kind == "j3_feedback":
                    if not self.j3_feedback_supported:
                        raise RuntimeError("固件未提供J3手动采集，请更新配套固件")
                    if type(payload) is not bool:
                        raise ValueError("J3采集只接受开始/停止")
                    if payload and time.perf_counter() - requested > 0.5:
                        raise RuntimeError("J3启动请求排队过期；未发送")
                    self.rpc(13, scope=3, field=1 if payload else 2)
                    self.events.put(("j3_feedback_ack", payload))
                elif kind == "command":
                    action, revision = payload if isinstance(payload, tuple) else (payload, 0)
                    if action == 1 and time.perf_counter() - requested > 0.5:
                        raise RuntimeError("启动请求在上位机排队过期；未发送")
                    self.rpc(7, field=action, value=revision)
                    self.events.put(("task_ack", action))
                elif kind in ("mode", "motion", "impedance"):
                    if not self.motion_supported:
                        raise RuntimeError("固件未提供本版位置阻抗功能")
                    if time.perf_counter() - requested > 0.5:
                        raise RuntimeError("位置控制请求排队过期；未发送")
                    if kind == "impedance":
                        if not self.capture_supported:
                            raise RuntimeError("固件未提供一键原位阻抗；需更新固件")
                        revision = self.rpc(6, field=1)
                        self.rpc(12, field=8, value=revision)
                    elif kind == "mode":
                        if payload not in (0, 1):
                            raise ValueError("未知 Mode")
                        self.rpc(12, field=1, value=payload)
                    else:
                        values = validate_motion(payload)
                        revision = self.rpc(6, field=1)
                        self.rpc(12, field=2, value=revision)
                        try:
                            for j, axis in enumerate(values, 1):
                                for field, value in enumerate(axis, 1):
                                    self.rpc(12, j, field, value, floating=True)
                            self.rpc(12, field=3, value=revision)
                        except Exception:
                            try:
                                self.rpc(12, field=4)
                            except Exception:
                                pass
                            raise  # never repeat a possibly accepted move
                    self.read_motion()
                    self.events.put(("motion_ack", kind))
                elif kind in ("calibration", "imu"):
                    if not self.calibration_supported:
                        raise RuntimeError("固件未提供零点标定功能")
                    if time.perf_counter() - requested > 0.5:
                        raise RuntimeError("标定/IMU 请求排队过期，未发送")
                    if kind == "calibration":
                        joint, axis, revision = payload
                        self.rpc(9, joint, axis, revision)
                    else:
                        enabled, revision = ((payload, self.rpc(6, field=1))
                                             if type(payload) is bool else payload)
                        self.rpc(10, field=int(enabled), value=revision)
                    self.read_calibration(include_joints=self.poll_details)
                    self.events.put(("calibration_ack", kind))
                else:
                    raise ValueError("未知调参操作")
            except Exception as exc:
                self.events.put(("calibration_error" if kind in ("calibration", "imu")
                                 else "j3_feedback_error" if kind == "j3_feedback"
                                 else "tuning_error", str(exc)))
        if self.supported and self.online and time.perf_counter() >= self.next_status:
            self.next_status = time.perf_counter() + 0.5
            try:
                packed = self.rpc(6)
                revision = self.rpc(6, field=1)
                self.task_status = (*struct.pack("<I", packed), revision)
                self.task_status_at = time.perf_counter()
                self.events.put(("task_status", self.task_status))
                if self.j3_feedback_supported:
                    packed = self.rpc(13, scope=3)
                    phase, reason, state, tag = struct.pack("<I", packed)
                    if (tag != 0xA3 or phase >= len(J3_FEEDBACK_PHASES) or
                            reason >= len(J3_FEEDBACK_REASONS) or state not in (*range(16), 255)):
                        raise RuntimeError("J3采集状态格式无效")
                    self.j3_feedback_status = (phase, reason, state)
                    self.j3_feedback_at = time.perf_counter()
                    self.events.put(("j3_feedback_status", self.j3_feedback_status))
            except Exception as exc:
                self.events.put(("tuning_error", str(exc)))
                self.online = False  # keep capability known; don't fall back to bare enable
            if self.online and (self.poll_details or self.poll_imu) and self.calibration_supported:
                try:
                    self.read_calibration(include_joints=self.poll_details)
                except Exception as exc:
                    self.events.put(("calibration_error", str(exc)))
            if self.online and self.uart_imu_supported:
                try:
                    self.read_uart_imu()
                except Exception as exc:
                    self.events.put(("uart_imu_error", str(exc)))
            if self.online and self.poll_details and self.drag_supported:
                try:
                    self.read_drag()
                except Exception as exc:
                    self.events.put(("drag_error", str(exc)))
            if self.online and self.poll_details and self.motion_supported:
                try:
                    self.read_motion()
                except Exception as exc:
                    self.events.put(("motion_error", str(exc)))


def capture_ready(worker, task_state, status_at, busy=False):
    """Same existing one-click impedance gate for the main and advanced views."""
    return bool(worker and worker.ready.is_set() and not worker.stop_requested.is_set()
                and worker.tuning.supported and worker.tuning.online
                and worker.tuning.capture_supported and not busy
                and time.perf_counter() - status_at < 1.5
                and task_state in (0, 2, 3, 4, 5))


class Panel:
    def __init__(self, monitor):
        import tkinter as tk
        from tkinter import ttk
        self.monitor, self.tk = monitor, tk
        self.window = tk.Toplevel(monitor.root)
        self.window.title("Ylamp · J1/J2 高级设置（参数 / 标定 / 诊断）")
        width = min(1120, monitor.root.winfo_screenwidth() - 80)
        height = min(900, monitor.root.winfo_screenheight() - 100)
        self.window.geometry(f"{width}x{height}")
        self.window.minsize(min(1080, width), min(700, height))
        self.window.protocol("WM_DELETE_WINDOW", self.window.withdraw)
        self.variables, self.readbacks, self.flags = {}, {}, {}
        self.snapshot = None
        self.busy = False
        self.status_at = 0.0
        self.task_state = None
        self.dirty = False
        self.filling = False
        self.calibration = None
        self.drag = None
        self.motion = None
        self.drag_text = tk.StringVar(value="拖动/模型状态尚未读取")
        outer = ttk.Frame(self.window, padding=14)
        outer.pack(fill="both", expand=True)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(3, weight=1)
        self.status = tk.StringVar(value="等待连接并读取 MCU 参数；空白不代表零。")
        ttk.Label(outer, textvariable=self.status, wraplength=1050).grid(row=0, column=0, sticky="ew")
        self.task_text = tk.StringVar(value="Task 状态未知")
        ttk.Label(outer, textvariable=self.task_text, foreground="#ffcf7a").grid(
            row=1, column=0, sticky="w", pady=6)
        actions = ttk.Frame(outer)
        actions.grid(row=2, column=0, sticky="ew", pady=5)
        self.read_button = ttk.Button(actions, text="读取设备（覆盖草稿）", command=self.read)
        self.read_button.pack(side="left", padx=3)
        self.apply_button = ttk.Button(actions, text="写入 RAM 并回读", command=self.apply)
        self.apply_button.pack(side="left", padx=3)
        ttk.Button(actions, text="保存本机方案", command=self.save).pack(side="left", padx=3)
        ttk.Button(actions, text="加载方案（不下发）", command=self.load).pack(side="left", padx=3)
        book = ttk.Notebook(outer)
        book.grid(row=3, column=0, sticky="nsew", pady=8)
        canvases = []
        for group in ("拖动", "保持", "位置阻抗", "行程", "保护", "零点标定"):
            tab = ttk.Frame(book)
            book.add(tab, text=group)
            canvas = tk.Canvas(tab, background="#101723", highlightthickness=0,
                               width=1, height=1)
            scroll = ttk.Scrollbar(tab, orient="vertical", command=canvas.yview)
            canvas.configure(yscrollcommand=scroll.set)
            scroll.pack(side="right", fill="y")
            canvas.pack(side="left", fill="both", expand=True)
            page = ttk.Frame(canvas, padding=12)
            content = canvas.create_window(0, 0, anchor="nw", window=page)
            page.bind("<Configure>", lambda event, c=canvas: c.configure(scrollregion=c.bbox("all")))
            canvas.bind("<Configure>", lambda event, c=canvas, item=content:
                        c.itemconfigure(item, width=event.width))
            canvases.append(canvas)
            if group == "零点标定":
                self.build_calibration(page)
                continue
            if group == "位置阻抗":
                self.build_motion(page)
                continue
            for column, title in enumerate(("参数", "J1 编辑", "J1 当前回读", "J2 编辑", "J2 当前回读")):
                ttk.Label(page, text=title).grid(row=0, column=column, sticky="w", padx=8, pady=6)
                page.columnconfigure(column, weight=1)
            rows = [f for f in FIELDS if f.group == group and f.scope == "JOINT"]
            for row, field in enumerate(rows, 1):
                ttk.Label(page, text=field.label).grid(row=row, column=0, sticky="w", padx=8, pady=5)
                for joint in (1, 2):
                    k = key(joint, field)
                    variable = tk.StringVar()
                    self.variables[k] = variable
                    variable.trace_add("write", self.changed)
                    if field.member == "protection_flags":
                        holder = ttk.Frame(page)
                        holder.grid(row=row, column=joint * 2 - 1, sticky="w", padx=8)
                        flags = [tk.BooleanVar(value=True) for _ in PROTECTIONS]
                        self.flags[joint] = flags
                        for (_, label), flag in zip(PROTECTIONS, flags):
                            ttk.Checkbutton(holder, text=label, variable=flag,
                                            command=lambda j=joint, v=variable:
                                            v.set(str(sum(mask for (mask, _), f in
                                                          zip(PROTECTIONS, self.flags[j])
                                                          if f.get())))).pack(side="left")
                    else:
                        ttk.Entry(page, textvariable=variable, width=14).grid(
                            row=row, column=joint * 2 - 1, sticky="ew", padx=8, pady=5)
                    readback = tk.StringVar(value="—")
                    self.readbacks[k] = readback
                    ttk.Label(page, textvariable=readback, foreground="#52d6e8").grid(
                        row=row, column=joint * 2, sticky="w", padx=8)
            row = len(rows) + 1
            for field in (f for f in FIELDS if f.scope == "GLOBAL" and f.group == group):
                ttk.Label(page, text=f"公共 · {field.label}").grid(row=row, column=0, sticky="w", padx=8, pady=5)
                k = key(0, field)
                self.variables[k] = tk.StringVar()
                self.variables[k].trace_add("write", self.changed)
                ttk.Entry(page, textvariable=self.variables[k], width=14).grid(row=row, column=1, padx=8, sticky="ew")
                self.readbacks[k] = tk.StringVar(value="—")
                ttk.Label(page, textvariable=self.readbacks[k], foreground="#52d6e8").grid(row=row, column=2, padx=8)
                row += 1
            if group == "行程":
                ttk.Label(page, text="角度单位 °：原始反馈不改写；限位使用标定参考圈，区间必须小于 360°。\n"
                          "上电按参考区间选等价圈，运行连续跟踪；目标反变换到电机当前圈发送。\n"
                          "上下限同时用于参考角目标钳位和越界检查；请留机构、惯性与停止余量。\n"
                          "主动单次运动见位置阻抗页；无自动往返。",
                          wraplength=970).grid(row=row, column=0, columnspan=5, sticky="w", pady=20)
            elif group == "拖动":
                ttk.Label(page, text="拖动：Kp=0，v_des=0；速度越大阻尼越大，J2 重力前馈随角度变化。\n"
                          "已确认的朝上参考随固件保存，重启沿用；更改安装或电机编码器零点后须重新核对。\n"
                          "保存人工参考不等于 IMU 复核或实机补偿方向已通过。\n"
                          "J1 理想竖轴 g1=0；新动态版将25%×(M·ddq+c)限幅加入FF，停止保持时撤出。\n"
                          "c、M·ddq显示未缩放模型值，FF显示合成请求；不含摩擦/电流估算助力。\n"
                          "运行 SPEED 超速保护已删除；速度仍用于阻尼，前馈限幅不是总输出力矩限幅。",
                          wraplength=970).grid(row=row, column=0, columnspan=5, sticky="w", pady=14)
                ttk.Label(page, textvariable=self.drag_text, wraplength=970).grid(
                    row=row+1, column=0, columnspan=5, sticky="w", pady=14)
            elif group == "保持":
                ttk.Label(page, text="正常停止：捕获当前位置，渐增保持刚度/阻尼，同时保留重力支撑。\n"
                          "继续：保持目标不跳变，Kp 渐退至零再回到自由拖动。\n"
                          "F6/故障失能会失去支撑，必须手动托住负载。",
                          wraplength=970).grid(row=row, column=0, columnspan=5, sticky="w", pady=14)
            else:
                ttk.Label(page, text="运行 SPEED 超速保护已删除；使能前近静止检查保留。\n"
                          "开关只影响 MCU 的跟随误差/软件过温检查，关闭会失去相应保护。\n"
                          "角度越界、反馈有效性/失联、电机故障、协议数值校验仍保留；"
                          "不修改电机内置过流/过温/过压保护。\n"
                          "不是认证安全限位；反馈估算力矩不是实测，也不作为总力矩硬限制。",
                          wraplength=970).grid(row=row, column=0, columnspan=5, sticky="w", pady=14)
        self.allow_motion = tk.BooleanVar(value=False)
        ttk.Checkbutton(outer, text="我已核对已回读参数、负载支撑和行程，允许启动所选 Mode",
                        variable=self.allow_motion).grid(row=4, column=0, sticky="w", pady=5)
        movement = ttk.Frame(outer)
        movement.grid(row=5, column=0, sticky="ew")
        self.start_button = ttk.Button(movement, text="启动 / 继续所选 Mode", command=lambda: self.command(1))
        self.start_button.pack(side="left", padx=3)
        self.stop_button = ttk.Button(movement, text="停止转保持 [F5]", command=lambda: self.command(2))
        self.stop_button.pack(side="left", padx=3)
        self.disable_button = ttk.Button(movement, text="双轴失能 [F6] · 释放力矩", command=lambda: self.command(3))
        self.disable_button.pack(side="left", padx=3)
        self.clear_button = ttk.Button(movement, text="解除 Task 锁定（不使能）", command=lambda: self.command(4))
        self.clear_button.pack(side="left", padx=3)
        ttk.Label(outer, text="仅 RAM 生效：MCU 重启恢复固件默认值，可加载本机方案重新写入。"
                  "关闭上位机/拔 USB 不会停止板上任务；无独立硬件急停保障。",
                  wraplength=1050).grid(row=6, column=0, sticky="ew", pady=(8, 0))
        self.window.bind("<MouseWheel>", lambda event:
                         canvases[book.index(book.select())].yview_scroll(
                             -int(event.delta / 120), "units"))
        for keyname in ("F5", "F6"):
            self.window.bind(f"<KeyPress-{keyname}>", monitor.hotkey)
            self.window.bind(f"<KeyRelease-{keyname}>", monitor.release_hotkey)
        self.window.bind("<FocusOut>", lambda event: monitor.pressed_keys.clear())
        if monitor.tuning_snapshot:
            self.fill(monitor.tuning_snapshot)
        self.update_buttons()
        self.window.after(100, self.tick)

    def changed(self, *args):
        if not self.filling:
            self.dirty = True
            self.allow_motion.set(False)
            self.status.set("草稿已修改，尚未下发；右列仍是上次回读，点击写入后确认。")

    def build_motion(self, page):
        from tkinter import ttk
        ttk.Label(page, text="位置阻抗：MIT 内部 PD + 完整名义 M·ddq_des + c + g；不含摩擦补偿。\n"
                  "点击下面的阻抗按钮：自动锁定 MCU 收到请求时的双轴当前位置并进入阻抗。\n"
                  "无需再选 Mode、按 KEY 或填写运动参数；从拖动切入不先失能。\n"
                  "推开后原位目标不变，松手后按 Kp 回拉、Kd 阻尼返回；再次点击才重新记录原位。\n"
                  "本次不规划移动目标，不配置速度、加速度、jerk 上限；Ruckig 移动留待后续。",
                  wraplength=970).pack(anchor="w", pady=8)
        self.hold_button = ttk.Button(page, text="阻抗：锁定当前位置",
                                      command=lambda: self.submit("impedance"))
        self.hold_button.pack(anchor="w", pady=12)
        actions = ttk.Frame(page)
        actions.pack(anchor="w", pady=8)
        self.mode_buttons = [
            ttk.Button(actions, text=text, command=lambda m=mode: self.submit("mode", m))
            for mode, text in ((0, "失能时切回 Drag（不使能）"),)]
        for button in self.mode_buttons:
            button.pack(side="left", padx=4)
        self.motion_text = self.tk.StringVar(value="位置阻抗状态尚未读取")
        ttk.Label(page, textvariable=self.motion_text, wraplength=970).pack(anchor="w", pady=8)
        ttk.Label(page, text="初始 Kp=5 N·m/rad、Kd=0.5 N·m·s/rad（两轴），1 秒渐入，尚未实机整定。\n"
                  "固定目标速度/加速度为零；回位由阻抗反馈产生，不是 Ruckig 限速回位。\n"
                  "实际回位受摩擦、负载和模型误差影响，请看目标、实际角和误差。\n"
                  "KEY/F5 不改变已锁定原位；F6 失能会失去重力支撑。\n"
                  "Motion 持续柔顺，不使用 Drag 的5°跟随跳闸。不要靠误差测试代替限位或停机手段。",
                  wraplength=970).pack(anchor="w", pady=8)

    def build_calibration(self, page):
        from tkinter import ttk
        tk = self.tk
        ttk.Label(page, text="建模角度 = 参考圈角度 − 建模零点（°）；参考角 = 原始角 + 整数圈 × 360°\n"
                  "J1：实测行程中点，保留原始圈数。J2：连杆朝上。不会改电机零点、原始限位或驱动电机。",
                  wraplength=950).pack(anchor="w", pady=8)
        self.cal_text = tk.StringVar(value="等待标定状态；IMU 默认关闭。")
        ttk.Label(page, textvariable=self.cal_text, wraplength=950).pack(anchor="w", pady=8)
        self.cal_attitude = tk.StringVar(value="Roll：—    Pitch：—    竖直偏差：—")
        ttk.Label(page, textvariable=self.cal_attitude, font=("Segoe UI", 16),
                  wraplength=950).pack(anchor="w", pady=8)
        ttk.Label(page, text="Roll / Pitch 为传感器坐标系的静态重力倾角（+Z 朝上为水平），不是 J2 关节角。\n"
                  "Yaw：重力无法观测，不伪报为 0°。J2 朝上原点使用所选轴的竖直偏差，不依赖航向。",
                  wraplength=950).pack(anchor="w", pady=4)
        self.cal_values = tk.StringVar(value="加速度 / 陀螺仪 / 建模角度：—")
        ttk.Label(page, textvariable=self.cal_values, wraplength=950).pack(anchor="w", pady=8)
        row = ttk.Frame(page)
        row.pack(anchor="w", pady=8)
        self.imu_start = ttk.Button(row, text="开启 IMU 只读采样",
                                   command=lambda: self.imu_session(True))
        self.imu_start.pack(side="left", padx=4)
        self.imu_stop = ttk.Button(row, text="关闭 IMU", command=lambda: self.imu_session(False))
        self.imu_stop.pack(side="left", padx=4)
        self.cal_j1 = ttk.Button(row, text="恢复 J1 软件零点", command=lambda: self.mark(1))
        self.cal_j1.pack(side="left", padx=4)
        ttk.Label(page, text="连杆朝上时，传感器哪个正/负轴应当朝上？\n"
                  "必须按板上轴向与实际安装确认；不是自动选最大分量。",
                  wraplength=950).pack(anchor="w", pady=8)
        self.cal_axis = tk.StringVar(value=IMU_AXES[0])
        axis = ttk.Combobox(page, textvariable=self.cal_axis, values=IMU_AXES,
                            state="readonly", width=12)
        axis.pack(anchor="w")
        self.cal_confirm = tk.BooleanVar(value=False)
        self.cal_axis.trace_add("write", lambda *args: self.cal_confirm.set(False))
        ttk.Checkbutton(page, variable=self.cal_confirm,
                        text="我确认板随 J2 输出连杆刚性运动，连杆朝上与所选 IMU 轴向一致").pack(
                            anchor="w", pady=10)
        self.cal_j2 = ttk.Button(page, text="将稳定竖直点标为 J2 零点（完成后关闭 IMU）",
                                command=lambda: self.mark(2))
        self.cal_j2.pack(anchor="w", pady=8)
        self.cal_export = ttk.Button(page, text="导出建模零点 JSON", command=self.save_calibration)
        self.cal_export.pack(anchor="w", pady=8)
        ttk.Label(page, text="先用机械支撑固定负载，再失能并手动摆正；本页不会自动释放力矩或寻找零位。\n"
                  "等待重力低通滤波预热及连续静止；运动/振动/失联时拒绝标定。\n"
                  "未做这块板的六面加速度误差标定，倾角阈值不是绝对精度保证；安装不对齐也需校正。\n"
                  "重启恢复固件保存的零点；本页新标定仍仅修改 RAM，不会自动覆盖拖动参考或写 Flash。\n"
                  "IMU 默认休眠，J2 成功标定后自动休眠；以后仅由你手动开启。",
                  wraplength=950).pack(anchor="w", pady=12)

    def imu_session(self, enabled):
        if enabled and (self.snapshot is None or self.dirty):
            self.status.set("先读取并确认设备参数；当前编辑草稿不能用于标定")
            return
        revision = self.snapshot["revision"] if self.snapshot else 0
        self.submit("imu", (enabled, revision))

    def mark(self, joint):
        if self.snapshot is None or self.dirty:
            self.status.set("先读取设备，标定不使用尚未下发的行程草稿")
            return
        axis = IMU_AXES.index(self.cal_axis.get()) if joint == 2 else 0
        if joint == 2 and (not axis or not self.cal_confirm.get()):
            self.status.set("先确认连杆朝上以及 IMU 轴向")
            return
        self.submit("calibration", (joint, axis, self.snapshot["revision"]))

    def calibration_ready(self):
        c = self.calibration
        worker = self.monitor.worker
        return bool(c and worker and worker.tuning.online
                    and worker.tuning.calibration_supported
                    and time.perf_counter() - c["at"] < 1.5)

    def update_calibration(self):
        fresh = self.calibration_ready()
        c = self.calibration
        if not fresh:
            self.cal_text.set("标定状态过期/未知；不代表 IMU 已关闭或标定已成功。")
            self.cal_attitude.set("Roll：—    Pitch：—    竖直偏差：—（状态过期/未知）")
            self.cal_values.set("加速度 / 陀螺仪 / 建模角度：—（状态过期/未知）")
            return False
        flags = c["flags"]
        state = ("IMU 不可用/休眠未确认" if flags & 64 else
                 "正在结束传输并休眠" if flags & 32 else "本次采样中" if flags & 16 else "已休眠")
        tests = ("新鲜", "预热完成", "连续静止", "两轴失能静止")
        gates = "  ".join(f"{name}:{'是' if flags & (1 << i) else '否'}"
                          for i, name in enumerate(tests))
        axis = IMU_AXES.index(self.cal_axis.get())
        sensor_fresh = flags & 113 == 17  # fresh + active, not stopping/failed
        accel = c["accel"] if sensor_fresh else [math.nan] * 3
        gyro = c["gyro"] if sensor_fresh else [math.nan] * 3
        roll, pitch, error = gravity_angles(accel, axis)
        def degrees(value):
            return f"{value:+.3f}°" if math.isfinite(value) else "—"
        self.cal_attitude.set(f"Roll：{degrees(roll)}    Pitch：{degrees(pitch)}"
                              f"    竖直偏差：{degrees(error)}")
        shown_error = f"{error:.3f}°" if math.isfinite(error) else "—"
        reference = ("J2：沿用已保存的人工朝上参考，IMU 轴向未复核。\n"
                     if c["valid_mask"] & 2 and c["axis"] == 0 else "")
        self.cal_text.set(reference + f"IMU：{state}  数据年龄 {c['age_ms']} ms  静止 {c['stable_ms']} ms\n"
                          f"{gates}\n所选轴偏离竖直：{shown_error}，允许 ≤ {c['tolerance_deg']:g}°")
        def shown(value):
            return f"{value:.4f}" if math.isfinite(value) else "未标定/无新鲜反馈"
        sensor = (f"a XYZ (m/s²)：{' / '.join(shown(x) for x in accel)}\n"
                  f"ω XYZ (°/s)：{' / '.join(shown(x) for x in gyro)}")
        joints = [f"J{j}: 零点 {shown(z)}°  原始 {shown(raw)}°"
                  f"  参考 {shown(reference)}°  建模 {shown(model)}°"
                  for j, (z, raw, model, reference) in enumerate(c["joints"], 1)]
        self.cal_values.set(sensor + "\n" + "\n".join(joints))
        return flags == 31 and math.isfinite(error) and error <= c["tolerance_deg"]

    def save_calibration(self):
        from tkinter import filedialog, messagebox
        if not self.calibration_ready() or self.calibration["valid_mask"] != 3 or not self.snapshot:
            self.status.set("需要本连接的新鲜零点回读，以及 J1/J2 都已标定")
            return
        c = self.calibration
        payload = {"schema": 2, "units": "degree",
                   "formula": "model = reference - zero; J1 reference = raw; J2 reference = raw + 360*k",
                   "created_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                   "j2_reference": "link_up", "imu_up_axis": IMU_AXES[c["axis"]],
                   "imu_vertical_verified": 1 <= c["axis"] <= 6,
                   "ram_only": True, "joints": [
                       {"joint": j, "can_id": j, "zero_deg": c["joints"][j-1][0],
                        "reference_limits_deg": [self.snapshot["values"][f"{j}.1"],
                                           self.snapshot["values"][f"{j}.2"]]}
                       for j in (1, 2)]}
        try:
            path = filedialog.asksaveasfilename(parent=self.window, defaultextension=".json",
                initialfile="joint_model_zeros.json", filetypes=[("建模零点", "*.json")])
            if path:
                Path(path).write_text(json.dumps(payload, ensure_ascii=False, indent=2,
                                                 allow_nan=False), encoding="utf-8")
                self.status.set("建模零点已导出；未写电机 Flash，重启 MCU 不自动导入。")
        except (OSError, ValueError) as exc:
            messagebox.showerror("导出零点", str(exc), parent=self.window)

    def values(self):
        try:
            return validate({k: float(v.get()) for k, v in self.variables.items()})
        except ValueError as exc:
            raise ValueError(f"输入有误：{exc}") from exc

    def fill(self, snapshot):
        self.filling = True
        self.snapshot = snapshot
        for k, value in snapshot["values"].items():
            self.variables[k].set(f"{value:.7g}")
            self.readbacks[k].set(f"{value:.7g}")
        for joint, flags in self.flags.items():
            mask = int(snapshot["values"][f"{joint}.9"])
            for (bit, _), flag in zip(PROTECTIONS, flags):
                flag.set(bool(mask & bit))
        self.filling = False
        self.dirty = self.busy = False
        self.allow_motion.set(False)
        self.status.set(f"设备 RAM 已回读，参数版本 {snapshot['revision']}；未自动使能/启动。")

    def submit(self, kind, payload=None):
        worker = self.monitor.worker
        reason = worker.tuning.request(kind, payload) if worker else "尚未连接 MCU"
        if reason:
            self.status.set(reason)
            return False
        self.busy = True
        self.status.set("操作已排入，等待 MCU 应答；尚未确认生效。")
        return True

    def read(self):
        from tkinter import messagebox
        if self.dirty and not messagebox.askyesno("读取设备", "覆盖未下发的编辑草稿？", parent=self.window):
            return
        self.submit("read")

    def apply(self):
        from tkinter import messagebox
        try:
            values = self.values()
            if self.snapshot is None:
                raise ValueError("请先读取当前设备参数")
            disabled = [f"J{j}" for j in (1, 2) if values[f"{j}.9"] != ALL_PROTECTIONS]
            if disabled and not messagebox.askyesno(
                    "保护开关", f"{'、'.join(disabled)} 将关闭部分软件保护，确认写入？", parent=self.window):
                return
            self.allow_motion.set(False)
            self.submit("apply", {"revision": self.snapshot["revision"], "values": values})
        except ValueError as exc:
            messagebox.showerror("参数校验", str(exc), parent=self.window)

    def command(self, action):
        if action == 1 and (not self.allow_motion.get() or self.dirty or self.snapshot is None
                            or self.snapshot["values"]["0.5"] != 1):
            self.status.set("启动拒绝：先写入/回读，核对参数并勾选运动确认")
            return
        if action == 3:
            self.allow_motion.set(False)
        self.submit("command", (action, self.snapshot["revision"]) if action == 1 else action)

    def event(self, kind, value):
        if kind == "tuning_readback":
            self.fill(value)
        elif kind == "tuning_applied":
            self.fill(value)
            self.status.set(f"整组写入并回读一致，RAM 版本 {value['revision']}；未启动。")
        elif kind == "task_status":
            self.status_at = time.perf_counter()
            self.task_state = value[0]
            if self.snapshot and self.snapshot["revision"] != value[4]:
                self.snapshot = None
                self.allow_motion.set(False)
                self.dirty = True
                self.status.set("MCU 参数版本已变化或已重启；请重新读取，旧方案未生效。")
            state = TASK_STATES[value[0]] if value[0] < len(TASK_STATES) else str(value[0])
            reason = REASONS[value[1]] if value[1] < len(REASONS) else str(value[1])
            self.task_text.set(f"MCU Task：{state}    原因：{reason}    有效反馈掩码 {value[3]:02b}")
        elif kind == "task_ack":
            self.busy = False
            self.status.set("MCU 已处理请求；使能/失能/停稳请看电机新反馈与 Task 状态。")
        elif kind == "calibration_status":
            self.calibration = value
        elif kind == "drag_status":
            self.drag = value
        elif kind == "drag_error":
            self.drag = None
        elif kind == "motion_status":
            self.motion = value
        elif kind == "motion_ack":
            self.busy = False
            self.status.set("MCU 已接受原位阻抗请求；使能确认及保持状态请看新鲜反馈。"
                            if value == "impedance" else
                            "MCU 已接受模式/单次运动请求；请核对当前阶段、目标和误差。")
        elif kind == "motion_error":
            self.motion = None
            self.motion_text.set(f"位置阻抗状态未知：{value}")
        elif kind == "calibration_ack":
            self.busy = False
            self.status.set("标定/IMU 请求已应答；请核对零点、IMU 状态回读。未驱动电机。")
        elif kind == "calibration_error":
            self.busy = False
            self.calibration = None
            self.cal_confirm.set(False)
            self.status.set(f"标定状态未确认：{value}；未自动重试动作。")
        elif kind == "tuning_error":
            self.busy = False
            self.allow_motion.set(False)
            self.snapshot = None
            self.dirty = True
            self.status.set(f"未确认：{value}")

    def update_buttons(self):
        worker = self.monitor.worker
        connected = bool(worker and worker.ready.is_set() and worker.tuning.supported
                         and not worker.stop_requested.is_set())
        for button in (self.read_button, self.apply_button, self.clear_button):
            button.configure(state="normal" if connected and not self.busy else "disabled")
        fresh = connected and worker.tuning.online and time.perf_counter() - self.status_at < 1.5
        idle = fresh and self.task_state in (0, 5)
        motion_fresh = bool(fresh and worker.tuning.motion_supported and self.motion
                            and time.perf_counter() - self.motion["at"] < 1.5)
        for button in self.mode_buttons:
            button.configure(state="normal" if idle and not self.busy
                             and worker.tuning.motion_supported else "disabled")
        self.hold_button.configure(
            state="normal" if capture_ready(worker, self.task_state, self.status_at,
                                            self.busy or self.monitor.quick_busy)
            else "disabled")
        if motion_fresh:
            lines = [f"当前 Mode：{'Motion 位置阻抗' if self.motion['mode'] == 1 else 'Drag 拖动'}"]
            if self.motion.get("valid"):
                phases = ("关闭", "渐入原位保持", "轨迹执行", "减速停止", "保持", "恢复")
                phase = self.motion["phase"]
                lines.append(f"阶段：{phases[phase] if phase < len(phases) else '未知'}  "
                             f"轨迹时间 {self.motion['elapsed']:.3f}/{self.motion['duration']:.3f} s")
                for joint in (1, 2):
                    sample = self.monitor.latest.get(joint)
                    if (sample and sample.mode == 1 and sample.reference is not None
                            and sample.target_reference is not None and sample.tracking_error is not None
                            and time.perf_counter() - self.monitor.origin - sample.elapsed_s < .5):
                        lines.append(f"J{joint} 目标 {math.degrees(sample.target_reference):+.3f}°  "
                                     f"实际 {math.degrees(sample.reference):+.3f}°  "
                                     f"误差 {math.degrees(sample.tracking_error):+.3f}°")
                    else:
                        lines.append(f"J{joint} 目标/实际/误差：无新鲜遥测")
                    g, c, inertia, a = self.motion["model"][joint - 1]
                    lines.append(f"  g {g:+.5f}  c {c:+.5f}  M·ddq {inertia:+.5f} N·m  "
                                 f"ddq_des {a:+.4f} rad/s²")
            self.motion_text.set("\n".join(lines))
        elif not self.motion:
            self.motion_text.set("位置阻抗状态未知/固件不支持；未自动运动")
        else:
            self.motion_text.set("位置阻抗状态过期；历史值不代表当前状态")
        self.apply_button.configure(state="normal" if idle and not self.busy and self.snapshot else "disabled")
        self.clear_button.configure(state="normal" if idle and not self.busy else "disabled")
        self.start_button.configure(state="normal" if fresh and self.task_state in (0, 4)
                                    and not self.busy and not self.dirty and self.snapshot
                                    and self.snapshot["values"]["0.5"] == 1
                                    and self.allow_motion.get() else "disabled")
        for button in (self.stop_button, self.disable_button):
            button.configure(state="normal" if connected else "disabled")
        if not fresh:
            self.task_text.set("Task 状态过期/未知；不要据历史回读判断运动已停止。")
        if self.drag and self.drag["valid"] and time.perf_counter() - self.drag["at"] < 1.5:
            lines = [f"Mode：{DRAG_PHASES[self.drag['phase']]}    A/B/D [kg·m²]：" +
                     " / ".join(f"{v:.6g}" for v in self.drag["inertia"])]
            for j, v in enumerate(self.drag["joints"], 1):
                lines += [f"J{j} 原始 {v[0]:+.3f}°  速度 {v[1]:+.3f}°/s  电流估算力矩 {v[2]:+.4f} N·m",
                          f"  参考 {v[12]:+.3f}°  换圈偏移 {v[13]:+.0f}°",
                          f"  g {v[3]:+.4f}  c(名义原值) {v[4]:+.5f}  M·ddq(名义原值) {v[5]:+.5f}",
                          f"  请求 FF {v[6]:+.4f}  Kd {v[7]:.3f}  Kp {v[8]:.3f}"
                          f"  目标 {v[9]:+.3f}°  误差 {v[10]:+.3f}°"]
            self.drag_text.set("\n".join(lines).replace("nan", "—"))
        else:
            self.drag_text.set("拖动/模型状态过期或无有效反馈；数值未知。")
        vertical = self.update_calibration()
        cal_fresh = self.calibration_ready()
        flags = self.calibration["flags"] if cal_fresh else 0
        allowed = bool(idle and not self.busy and not self.dirty and self.snapshot and cal_fresh)
        self.imu_start.configure(state="normal" if allowed and not flags & 112 else "disabled")
        self.imu_stop.configure(state="normal" if connected and worker.tuning.calibration_supported
                                and not self.busy else "disabled")
        self.cal_j1.configure(state="normal" if allowed and flags & 8 else "disabled")
        self.cal_j2.configure(state="normal" if allowed and vertical and self.cal_confirm.get() else "disabled")
        self.cal_export.configure(state="normal" if cal_fresh and self.calibration["valid_mask"] == 3
                                  and self.snapshot else "disabled")

    def tick(self):
        if not self.window.winfo_exists():
            return
        self.update_buttons()
        self.window.after(100, self.tick)

    def save(self):
        from tkinter import filedialog, messagebox
        try:
            values = self.values()
            path = filedialog.asksaveasfilename(parent=self.window, defaultextension=".json",
                initialdir=str(Path(__file__).resolve().parents[1] / "config"),
                initialfile="motor_tuning_profile.json", filetypes=[("调参方案", "*.json")])
            if path:
                Path(path).write_text(json.dumps({"schema": 3, "values": values},
                                                ensure_ascii=False, indent=2), encoding="utf-8")
                self.status.set("方案已保存到本机；不代表设备已写入。")
        except (OSError, ValueError) as exc:
            messagebox.showerror("保存", str(exc), parent=self.window)

    def load(self):
        from tkinter import filedialog, messagebox
        path = filedialog.askopenfilename(parent=self.window, filetypes=[("调参方案", "*.json")])
        if not path:
            return
        try:
            profile = json.loads(Path(path).read_text(encoding="utf-8"))
            if profile["schema"] != 3:
                raise ValueError("方案版本不支持")
            values = validate(profile["values"])
            self.filling = True
            for k, value in values.items():
                self.variables[k].set(f"{value:.7g}")
            for j, flags in self.flags.items():
                for (bit, _), flag in zip(PROTECTIONS, flags):
                    flag.set(bool(int(values[f"{j}.9"]) & bit))
            self.filling = False
            self.changed()
            self.status.set("方案已加载到草稿，未下发；右列仍是设备上次回读。")
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.filling = False
            messagebox.showerror("加载", str(exc), parent=self.window)


def self_test():
    """Offline executable check of the actual RPC/client and input contract."""
    import threading
    for axis, accel in enumerate(((1, 0, 0), (-1, 0, 0), (0, 1, 0),
                                  (0, -1, 0), (0, 0, 1), (0, 0, -1)), 1):
        roll, pitch, tilt = gravity_angles(accel, axis)
        assert abs(tilt) < 1e-6
        assert abs(gravity_angles(accel, axis + (1 if axis % 2 else -1))[2] - 180) < 1e-6
        if axis <= 2:
            assert math.isnan(roll) and abs(abs(pitch) - 90) < 1e-6
    for roll_deg, pitch_deg in ((0, 0), (30, 20), (-45, -35), (170, 60)):
        r, p = map(math.radians, (roll_deg, pitch_deg))
        accel = [-math.sin(p), math.sin(r) * math.cos(p), math.cos(r) * math.cos(p)]
        roll, pitch, _ = gravity_angles([v * 9.80665 for v in accel])
        assert math.isclose(roll, roll_deg, abs_tol=1e-6)
        assert math.isclose(pitch, pitch_deg, abs_tol=1e-6)
    for accel in ((0, 0, 0), (math.nan, 0, 1), (0, math.inf, 1), (1, 2)):
        assert all(math.isnan(v) for v in gravity_angles(accel, 5))
    assert math.isnan(gravity_angles((0, 0, 9.80665))[2]), "Unknown UP axis inferred"
    joint = {1: -210, 2: 0, 3: 5, 4: 5, 5: 5, 7: 1, 8: 60,
             9: ALL_PROTECTIONS, 10: .05, 11: 5, 12: 1, 13: 8, 14: .2, 15: 10}
    globals_ = [100, 500, 20, -151.2828025, 1, 1, 1000, 1, 30, 100]
    values = {key(j, f): (joint[f.id] if j else globals_[f.id-1])
              for j, f in PARAMETERS}
    values["2.1"], values["2.2"] = -200, -60
    values["2.14"] = .7
    validate(values)
    assert len(PARAMETERS) == 38
    assert "1.6" not in values and "2.6" not in values
    for mask in (0, 2, 4, 6):
        validate({**values, "1.9": mask, "2.9": mask})
    assert validate(json.loads(json.dumps(values))) == values
    for k, v in (("1.3", float("nan")), ("2.4", 6), ("1.8", 2.5),
                 ("1.1", 1), ("1.1", -400), ("1.12", 10), ("0.1", 501), ("2.9", 8),
                 ("1.9", 1), ("1.9", 3), ("1.9", 5), ("1.9", 7), ("1.6", 10)):
        try:
            validate({**values, k: v})
        except ValueError:
            pass
        else:
            raise AssertionError((k, v))
    try:
        validate({**values, "unknown": 1})
    except ValueError:
        pass
    else:
        raise AssertionError("Unknown field accepted")
    events, stop, sent = queue.SimpleQueue(), threading.Event(), []
    memory, staged, revision, actions = dict(values), {}, 1, []
    calibration_actions = []
    corrupt_readback = False
    motion_draft, motion_jobs, mode = {}, [], 0

    def send(data):
        nonlocal revision, mode
        sent.append(bytes(data))
        seq, op, scope, field = data[:4]
        k = f"{scope}.{field}"
        raw = struct.unpack("<I", data[4:])[0]
        result, output, floating = 0, 0, False
        if op == 0:
            output = VERSION
        elif op == 1:
            output, floating = memory[k], True
            if corrupt_readback and k == "1.3":
                output += 1
        elif op == 2:
            if raw != revision:
                result = 3
            else:
                staged.clear()
        elif op == 3:
            staged[k] = struct.unpack("<f", data[4:])[0]
        elif op == 4:
            assert set(staged) == set(memory)
            memory.update(staged)
            revision += 1
            output = revision
        elif op == 5:
            staged.clear()
        elif op == 6:
            output = revision if field else 0x03010000
        elif op == 7:
            if field == 1 and raw != revision:
                result = 3
            else:
                actions.append(field)
        elif op == 8:
            if field == 255:
                output = CALIBRATION_VERSION
            elif scope:
                output, floating = (-105 if scope == 1 else float("nan")), True
            elif field == 0:
                output = 0xC1010008  # J1 valid, IMU off, both motors disabled/still
            elif field >= 3:
                output, floating = (0.5 if field == 3 else 0), True
        elif op in (9, 10):
            if raw != revision:
                result = 3
            else:
                calibration_actions.append((op, scope, field))
        elif op == 11:
            if field == 255:
                output = DRAG_VERSION
            elif scope == 0 and field == 0:
                output = 0xD1000001
            else:
                output, floating = 0, True
        elif op == 12:
            if scope:
                if field <= 4:
                    motion_draft[k] = struct.unpack("<f", data[4:])[0]
                else:
                    output, floating = 0, True
            elif field == 255:
                output = MOTION_VERSION
            elif field == 0:
                output = mode
            elif field == 1:
                mode = raw
            elif field == 2:
                motion_draft.clear()
            elif field == 3:
                assert raw == revision and len(motion_draft) == 8
                motion_jobs.append(dict(motion_draft))
            elif field == 4:
                motion_draft.clear()
            elif field == 8:
                assert raw == revision
                mode = 1
                actions.append("impedance")
            elif field == 5:
                output = 0xE1010401
            else:
                output, floating = 1, True
        else:
            raise AssertionError(op)
        # A mismatched reply must never satisfy an RPC.
        client.receive(bytes((0, 0, scope, field)) + bytes(4))
        client.receive(bytes((seq, result, scope, field))
                       + struct.pack("<f" if floating else "<I", output))

    client = Client(send, events, stop)
    initial_send = client.send
    dropped = []
    def lose_first_read(data):
        dropped.append(bytes(data))
        if len(dropped) > 1:
            initial_send(data)
    client.send = lose_first_read
    assert client.rpc(0) == VERSION
    assert len(dropped) == 2 and dropped[0][0] != dropped[1][0]
    client.send = initial_send
    client.discover()
    assert client.supported and client.online
    assert client.calibration_supported
    assert client.motion_supported and client.capture_supported
    assert not actions and not calibration_actions and all(data[1] in (0, 1, 6, 8, 11, 12) for data in sent)
    client.poll_details = False
    first = len(sent)
    client.next_status = 0
    client.service()
    assert [data[1] for data in sent[first:]] == [6, 6]
    first = len(sent)
    client.next_status = time.perf_counter()+10
    assert not client.request("calibration_read")
    client.service()
    assert all(data[1] == 8 for data in sent[first:])
    assert len(sent) > first and not calibration_actions and not actions
    client.poll_details = True
    drag = client.read_drag()
    assert drag["valid"] and len(drag["joints"][0]) == 14 and not actions
    cal = client.read_calibration()
    assert len(cal["joints"][0]) == 4
    assert cal["flags"] == 8 and cal["valid_mask"] == 1 and math.isnan(cal["joints"][1][0])
    updated = dict(values)
    updated["1.3"] = 30
    updated["2.10"], updated["2.14"] = .1, .6
    assert not client.request("apply", {"revision": 1, "values": updated})
    client.service()
    assert revision == 2 and memory["1.3"] == 30 and not actions
    assert not client.request("command", (1, revision))
    client.service()
    assert actions == [1]
    assert not client.request("command", (1, 1))
    client.service()
    assert actions == [1], "Stale revision started motion"
    client.jobs.put_nowait(("command", (1, revision), time.perf_counter() - 1))
    client.service()
    assert actions == [1], "Expired host request started motion"
    assert not client.request("imu", (True, revision))
    client.service()
    assert calibration_actions == [(10, 0, 1)]
    assert not client.request("calibration", (2, 5, revision))
    client.service()
    assert calibration_actions[-1] == (9, 2, 5)
    before_actions = len(calibration_actions)
    client.jobs.put_nowait(("calibration", (2, 5, revision), time.perf_counter() - 1))
    client.service()
    assert len(calibration_actions) == before_actions, "Expired calibration was executed"
    before = list(actions)
    assert not client.request("mode", 1)
    client.service()
    assert mode == 1 and actions == before
    action_count = len(sent)
    assert not client.request("impedance")
    client.service()
    assert actions == [*before, "impedance"]
    issued = [data[1:4] for data in sent[action_count:] if data[1] not in (6, 8, 11)
              and not (data[1] == 12 and data[3] in (0, 5, 6, 7, 11, 12, 13, 14))]
    assert issued == [bytes((12, 0, 8))], "Shortcut staged a move or separately enabled"
    before = list(actions)
    client.jobs.put_nowait(("impedance", None, time.perf_counter() - 1))
    client.service()
    assert actions == before, "Expired capture recentered the arm"
    assert not client.request("motion", [[-100, 5, 10, 20], [-150, 5, 10, 20]])
    client.service()
    assert len(motion_jobs) == 1 and actions == before, "Move implicitly enabled or repeated"
    client.jobs.put_nowait(("motion", [[-100, 5, 10, 20], [-150, 5, 10, 20]],
                            time.perf_counter() - 1))
    client.service()
    assert len(motion_jobs) == 1, "Expired move executed"
    for invalid in ([[math.nan, 1, 1, 1]] * 2, [[0, 0, 1, 1]] * 2, [[0, 1, 1]] * 2):
        try:
            validate_motion(invalid)
        except ValueError:
            pass
        else:
            raise AssertionError("Invalid Motion job accepted")
    # Drain success messages, then check an unverified readback is not reported applied.
    while not events.empty():
        events.get_nowait()
    corrupt_readback = True
    assert not client.request("apply", {"revision": 2, "values": updated})
    client.service()
    messages = []
    while not events.empty():
        messages.append(events.get_nowait())
    assert any(k == "tuning_error" for k, v in messages)
    assert not any(k in ("tuning_applied", "tuning_readback") for k, v in messages)
    from unittest.mock import patch
    unanswered = []
    client.send = lambda data: unanswered.append(bytes(data))
    with patch("dm_tuning.time.perf_counter", side_effect=(0.0, 1.0)):
        try:
            client.rpc(7, field=3)
        except TimeoutError:
            pass
        else:
            raise AssertionError("Unanswered action reported success")
    assert len(unanswered) == 1, "An action was automatically retried"
    client.send = initial_send
    client.online = False
    assert client.request("command", (1, revision)), "Unknown status allowed START"
    assert not client.request("command", 3), "Known firmware must still accept explicit F6"
    client.service()
    assert actions[-1] == 3
    def old_firmware(data):
        if data[1:4] == bytes((12, 0, 255)):
            client.receive(bytes((data[0], 1, 0, 255)) + bytes(4))
        else:
            initial_send(data)
    client.send = old_firmware
    client.discover()
    assert client.online and client.supported and not client.motion_supported
    assert not client.capture_supported
    client.send = initial_send
    stop.set()
    before = len(sent)
    try:
        client.rpc(7, field=1, value=revision)
    except RuntimeError:
        pass
    else:
        raise AssertionError("Closed connection sent an action")
    assert len(sent) == before
    print("PASS: tuning schema/validation, RPC matching, atomic apply/readback,"
          " version/start guards, expiry, no auto-motion and unknown-result handling (offline).")
    return values


if __name__ == "__main__":
    self_test()
