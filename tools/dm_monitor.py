"""Five-motor monitoring: native board USB CDC; explicit J1/J2 quick controls.

Run: python dm_monitor.py
Checks: python dm_monitor.py --self-test
Hardware check: python dm_monitor.py --usb --probe --port COM36 --seconds 5
CAN1 read-only: python dm_monitor.py --can --bus 1
UART alternative: python dm_monitor.py --serial --probe --port COM5 --seconds 5
Standalone CAN only: python dm_monitor.py --probe --can --direct --hz 100

Default GUI uses board USB CDC, NOT USB-CAN or the UART adapter.
Explicit --can retains PyUSB/WinUSB GS_USB at Classic CAN 1 Mbps.
PC only listens to motor feedback/MIT; MCU alone queries/commands the motors.
The MCU CAN RPC must be enabled in the matching firmware.
Explicit --serial uses pyserial at 230400 8N1; never use two configuration writers.
Connecting and --probe only query/listen; legacy J3 remains passive.
GUI controls require explicit user action.
MCU tuning uses explicit manual operations; connecting still only reads.
Never zeroes encoders or writes motor parameters/Flash.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import queue
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, replace
from datetime import datetime, timezone
from pathlib import Path
import dm_tuning
import dm_serial
import usb_monitor


# Match config/motor_config.c. These are software identities, not motor-ID writes.
MOTORS = {1: 0x11, 2: 0x12, 3: 0x13, 4: 0x14, 5: 0x15}
MOTOR_BUSES = {1: 2, 2: 2, 3: 1, 4: 1, 5: 1}
MOTOR_MODELS = {1: "4340P", 2: "4340P", 3: "4310", 4: "4310", 5: "4310"}
CONFIG_LIMITS = {m: (12.5, 10, 28) if m <= 2 else (12.5, 30, 10) for m in MOTORS}
LEGACY_MOTORS = frozenset({3})  # J3 5011/007: command replies only, no CC/register reads.
BUS_LABELS = {2: "CAN2 · J1/J2 · 操作与监控", 1: "CAN1 · J3/J4/J5 · 仅查看"}
LIMIT_REGISTERS = {21: "PMAX", 22: "VMAX", 23: "TMAX"}
FRAME = struct.Struct("<IIBBBB8s")
MAX_RECORDS = 120_000
COLORS = {1: "#52d6e8", 2: "#ffb35b"}
PALETTE = (*COLORS.values(), "#a4dc83", "#c3a0ff", "#fb8faa", "#f1dd77")
BG, PANEL, TEXT, MUTED = "#101723", "#192334", "#e8eff9", "#9aaec6"
STATES = {
    0: "失能",
    1: "已使能",
    2: "电机侧未识别",
    3: "输出轴未识别",
    5: "编码器读取错误",
    7: "编码器读取错误",
    8: "过压",
    9: "欠压",
    10: "过流",
    11: "MOS 过温",
    12: "线圈过温",
    13: "通信丢失",
    14: "过载",
}


@dataclass(frozen=True, slots=True)
class Sample:
    motor: int
    elapsed_s: float
    unix_s: float
    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    mos_c: int
    rotor_c: int
    state: int
    raw: bytes
    can_id: int | None = None
    feedback_id: int | None = None
    tick_ms: int | None = None
    feedback_ms: int | None = None
    sequence: int | None = None
    reference: float | None = None
    target: float | None = None
    target_velocity: float | None = None
    target_reference: float | None = None
    tracking_error: float | None = None
    kp: float | None = None
    kd: float | None = None
    ff: float | None = None
    task_state: int | None = None
    task_reason: int | None = None
    mode: int | None = None
    phase: int | None = None
    source: str = "CAN"
    reference_source: str | None = None
    command_elapsed_s: float | None = None
    command_age_ms: float | None = None
    status_age_ms: float | None = None
    command_raw: bytes = b""
    can_bus: int | None = None
    limits_source: str | None = None

    @property
    def position_deg(self):
        return math.degrees(self.position_rad)


def joint_color(motor):
    return PALETTE[(motor - 1) % len(PALETTE)]


def query_frame(motor: int, register: int | None = None, echo: int = 0) -> bytes:
    """Allowlisted, read-only requests; also used by the query-only CLI probe."""
    if motor not in MOTORS or register is not None and register not in LIMIT_REGISTERS:
        raise ValueError("Only motors J1..J5 and read-only range registers are allowed")
    if motor in LEGACY_MOTORS:
        raise ValueError("J3旧协议不支持0xCC/0x33查询；只接收显式控制命令的反馈")
    if not 0 <= echo < 0xFFFFFFFF:
        raise ValueError("Invalid echo ID")
    payload = bytes((motor, 0, 0xCC if register is None else 0x33,
                     0 if register is None else register, 0, 0, 0, 0))
    return FRAME.pack(echo, 0x7FF, 8, 0, 0, 0, payload)


def unpack_frame(packet):
    # GS_USB: 20 bytes, or 24 bytes when a hardware timestamp is appended.
    if len(packet) not in (20, 24, 32):
        raise ValueError(f"Unexpected USB frame length: {len(packet)}")
    echo, can_id, dlc, channel, flags, reserved, data = FRAME.unpack_from(packet)
    if dlc > 8 or flags & ~1:  # Only Classic CAN plus the RX-overflow flag.
        raise ValueError("Not a Classic CAN frame")
    return echo, can_id, dlc, channel, data, flags


def decode_sample(can_id, data, limits, elapsed_s, unix_s):
    motor = next((m for m, master in MOTORS.items() if master == can_id), None)
    if motor is None or len(data) != 8 or data[0] & 0x0F != motor:
        raise ValueError("Feedback ID/payload mismatch")
    if motor not in LEGACY_MOTORS and data[:3] == bytes((motor, 0, 0x33)) and data[3] in LIMIT_REGISTERS:
        raise ValueError("Motor register reply is not motion feedback")
    pmax, vmax, tmax = limits
    if any(not math.isfinite(v) or v <= 0 for v in limits):
        raise ValueError("Invalid protocol ranges")
    pos = (data[1] << 8) | data[2]
    vel = (data[3] << 4) | (data[4] >> 4)
    torque = ((data[4] & 15) << 8) | data[5]
    return Sample(motor, elapsed_s, unix_s,
                  pos * (2 * pmax) / 65535 - pmax,
                  vel * (2 * vmax) / 4095 - vmax,
                  torque * (2 * tmax) / 4095 - tmax,
                  data[6], data[7], data[0] >> 4, bytes(data))


def decode_mit(data, limits):
    """Decode observed wire values, not requested floats or measured torque."""
    if len(data) != 8 or any(not math.isfinite(v) or v <= 0 for v in limits):
        raise ValueError("Invalid MIT frame/ranges")
    if data[:7] == b"\xff" * 7 and 0xFB <= data[7] <= 0xFE:
        return None  # Clear/enable/disable/zero are not MIT targets.
    pmax, vmax, tmax = limits
    return dict(target=((data[0] << 8) | data[1]) * (2*pmax)/65535 - pmax,
                target_velocity=((data[2] << 4) | (data[3] >> 4)) * (2*vmax)/4095 - vmax,
                kp=(((data[3] & 15) << 8) | data[4]) * 500/4095,
                kd=((data[5] << 4) | (data[6] >> 4)) * 5/4095,
                ff=(((data[6] & 15) << 8) | data[7]) * (2*tmax)/4095 - tmax)


def usb_modules():
    try:
        import usb.core
        import usb.util
        import usb.backend.libusb1
    except ImportError as exc:
        raise RuntimeError("缺少 PyUSB；请使用本机已安装 PyUSB 的 Python 运行。") from exc
    backend = usb.backend.libusb1.get_backend()
    if backend is None:
        workspace = Path(__file__).resolve().parents[2]
        sdk = workspace / "arm_learning" / "projects" / "damiao_motor_sdk"
        for dll in sdk.glob("*/u2canfd/libusb-1.0.dll"):
            backend = usb.backend.libusb1.get_backend(find_library=lambda name: str(dll))
            if backend is not None:
                break
    if backend is None:
        raise RuntimeError("找不到 libusb；请保留工程内达妙 SDK 的 libusb-1.0.dll。")
    return usb.core, usb.util, backend


def scan_adapters():
    core, util, backend = usb_modules()
    result = []
    for vendor, product in ((0x1D50, 0x606F), (0x1209, 0x2323)):
        for dev in core.find(find_all=True, idVendor=vendor, idProduct=product, backend=backend):
            try:
                serial = dev.serial_number or ""
                result.append({
                    "vendor": vendor, "product": product, "bus": dev.bus,
                    "address": dev.address, "serial": serial,
                    "label": f"{dev.product or 'GS_USB'} · {serial or '无序列号'}"
                             f" · USB {dev.bus}:{dev.address}",
                })
            except core.USBError as exc:
                raise RuntimeError("适配器被占用或驱动不可访问；请先关闭达妙/CAN调试工具的设备。") from exc
            finally:
                util.dispose_resources(dev)
    return result


class Reader(threading.Thread):
    """Owns USB lifetime; separate bulk-IN and bulk-OUT avoid Windows read stalls."""

    def __init__(self, adapter, hz, samples, events, origin, direct=False, can_bus=2):
        super().__init__(name="dm-monitor", daemon=False)
        if can_bus not in BUS_LABELS:
            raise ValueError("CAN bus must be 1 or 2")
        self.adapter, self.samples, self.events, self.origin = adapter, samples, events, origin
        self.hz = hz
        self.direct = direct
        # GS_USB channel 0 is the adapter's connector, NOT board CAN1/CAN2.
        # The user must attach that connector to the selected physical bus.
        self.can_bus = can_bus
        self.motors = {m: master for m, master in MOTORS.items() if MOTOR_BUSES[m] == can_bus}
        self.limits_source = ("mixed: J3 local config (legacy); J4/J5 motor registers"
                              if direct and can_bus == 1 else "motor registers" if direct else
                              "MCU configured decode scales" if can_bus == 2 else
                              "local config; NOT read back or verified")
        self.stop_requested = threading.Event()
        self.paused = threading.Event()
        self.limits = {}
        self.counts = {m: 0 for m in self.motors}
        self.arrivals = {m: deque(maxlen=2000) for m in self.motors}
        self.tx_usb = self.tx_echo = self.tx_rpc = self.invalid = self.other = 0
        self.bus_errors = self.ui_drops = self.missed_rounds = self.adapter_overflows = 0
        self.echo = 0
        self.dev = None
        self.core = None
        self.last_stats = {}
        self.ready = threading.Event()
        self.tuning = dm_tuning.Client(self.send_tuning, events, self.stop_requested)
        self.commands, self.references, self.offsets = {}, {}, {}

    def send_query(self, motor, register=None):
        if not self.direct:
            raise RuntimeError("共享总线由 MCU 独占电机查询；禁止 PC 发送 0x7FF")
        if motor not in self.motors:
            raise ValueError("电机不在当前选择的 CAN 总线")
        packet = query_frame(motor, register, self.echo)
        if self.dev.write(0x02, packet, timeout=300) != len(packet):
            raise RuntimeError("USB 查询帧未完整提交")
        self.echo = (self.echo + 1) % 0xFFFFFFFF
        if register is None:
            self.tx_usb += 1

    def send_tuning(self, data):
        if self.can_bus != 2 or self.direct:
            raise RuntimeError("MCU 操作仅限 CAN2 共享总线；CAN1 只查看")
        packet = FRAME.pack(self.echo, dm_tuning.REQUEST_ID, 8, 0, 0, 0, data)
        if self.dev.write(0x02, packet, timeout=300) != len(packet):
            raise RuntimeError("USB 调参命令未完整提交；结果未知")
        self.echo = (self.echo + 1) % 0xFFFFFFFF
        self.tx_rpc += 1

    def receive(self, timeout_ms):
        try:
            packet = bytes(self.dev.read(0x81, 32, timeout=timeout_ms))
        except self.core.USBTimeoutError:
            return None
        try:
            frame = unpack_frame(packet)
        except ValueError:
            self.invalid += 1
            return None
        echo, can_id, dlc, channel, data, flags = frame
        if flags & 1:
            self.adapter_overflows += 1
        if can_id & 0x20000000:
            self.bus_errors += 1
            return None
        if echo != 0xFFFFFFFF:
            if can_id == 0x7FF and dlc == 8 and data[2] == 0xCC:
                self.tx_echo += 1
            return None  # A TX echo is NOT a motor response.
        if channel == 0 and can_id == dm_tuning.RESPONSE_ID and self.can_bus == 2:
            self.tuning.receive(data[:dlc])
            return None
        if channel == 0 and can_id in self.motors:
            if can_id in self.limits:
                try:
                    command = decode_mit(data[:dlc], self.limits[can_id])
                except ValueError:
                    self.invalid += 1
                else:
                    if command is None:
                        self.commands.pop(can_id, None)
                    else:
                        command.update(at=time.perf_counter(), raw=bytes(data),
                                       offset=self.offsets.get(can_id))
                        self.commands[can_id] = command
            return None
        if channel != 0 or can_id not in self.motors.values():
            self.other += 1
            return None
        return can_id, data[:dlc]

    def make_sample(self, can_id, data, now, unix):
        sample = decode_sample(can_id, data, self.limits[can_id - 0x10],
                               now - self.origin, unix)
        motor = sample.motor
        values = self.tuning.snapshot
        reference = None
        if values is not None and motor in (1, 2):
            values = values["values"]
            anchor = (self.references[motor] if sample.state == 1 and motor in self.references
                      else math.radians((values[f"{motor}.1"] + values[f"{motor}.2"]) / 2))
            turns = (anchor - sample.position_rad) / math.tau
            # Same nearest-turn convention as MCU roundf (ties away from zero).
            offset = math.copysign(math.floor(abs(turns) + .5), turns) * math.tau
            reference = sample.position_rad + offset
            self.references[motor], self.offsets[motor] = reference, offset
        fields = dict(can_id=motor, feedback_id=can_id, reference=reference,
                      can_bus=self.can_bus, limits_source=self.limits_source,
                      reference_source="host CAN branch" if reference is not None else None)
        if self.direct:
            fields["limits_source"] = ("local config; legacy J3, NOT read back or verified"
                                      if motor in LEGACY_MOTORS else "motor registers")
        command = self.commands.get(motor)
        if sample.state != 1:
            self.commands.pop(motor, None)
        elif command is not None and 0 <= now - command["at"] <= .1:
            fields.update({k: command[k] for k in ("target", "target_velocity", "kp", "kd", "ff")})
            fields.update(command_elapsed_s=command["at"] - self.origin,
                          command_age_ms=(now - command["at"]) * 1000,
                          command_raw=command["raw"])
            if reference is not None:
                offset = command["offset"] if command["offset"] is not None else self.offsets[motor]
                target = command["target"] + offset
                fields.update(target_reference=target, tracking_error=target-reference)
        if self.tuning.task_status and now - self.tuning.task_status_at < 1.5:
            fields.update(task_state=self.tuning.task_status[0],
                          task_reason=self.tuning.task_status[1],
                          status_age_ms=(now-self.tuning.task_status_at)*1000)
        motion = self.tuning.motion
        if motion and now - motion["at"] < 1.5:
            fields.update(mode=motion["mode"], phase=motion.get("phase"))
        return replace(sample, **fields)

    def read_limits(self):
        for motor, master in self.motors.items():
            if motor in LEGACY_MOTORS:
                self.limits[motor] = CONFIG_LIMITS[motor]
                continue  # No unsupported read, implicit FD or zero-MIT fallback.
            values = []
            for register, name in LIMIT_REGISTERS.items():
                # Only these read-only range queries may retry after USB/CAN
                # startup loss. Never retry tuning commits or motor commands.
                for attempt in range(3):
                    if self.stop_requested.is_set():
                        return False
                    self.send_query(motor, register)
                    deadline = time.perf_counter() + 0.6
                    while time.perf_counter() < deadline:
                        if self.stop_requested.is_set():
                            return False
                        reply = self.receive(20)
                        if reply is None:
                            continue
                        can_id, data = reply
                        if can_id == master and len(data) == 8 and data[:4] == bytes((motor, 0, 0x33, register)):
                            value = struct.unpack("<f", data[4:])[0]
                            if not math.isfinite(value) or value <= 0:
                                raise RuntimeError(f"0x{motor:02X} {name} 返回无效量程")
                            values.append(value)
                            break
                    else:
                        continue
                    break  # A matched read ended the inner loop.
                else:
                    raise RuntimeError(f"0x{motor:02X} 读取 {name} 三次超时；未使用猜测量程启动监控。")
            self.limits[motor] = tuple(values)
        self.events.put(("limits", self.limits.copy()))
        return True

    def read_mcu_limits(self):
        """Read the MCU's decode scales, not the motor registers."""
        limits = {}
        for motor in self.motors:
            try:
                values = tuple(self.tuning.rpc(1, motor, field, floating=True)
                               for field in (240, 241, 242))
            except Exception as exc:
                raise RuntimeError("MCU 解码量程读取失败；请核对连接并升级配套固件，"
                                   "不会回退为直接电机查询") from exc
            if any(not math.isfinite(value) or value <= 0 for value in values):
                raise RuntimeError(f"J{motor} MCU 协议量程无效")
            limits[motor] = values
        self.limits = limits
        self.events.put(("limits", limits.copy()))

    def statistics(self):
        now = time.perf_counter()
        rates = {}
        for motor, times in self.arrivals.items():
            while times and times[0] < now - 1:
                times.popleft()
            rates[motor] = len(times)  # Actual received frames in the last 1 second.
        return {
            "rx": self.counts.copy(), "hz": rates, "tx_usb": self.tx_usb,
            "tx_echo": self.tx_echo, "tx_rpc": self.tx_rpc,
            "invalid": self.invalid, "other": self.other,
            "bus_errors": self.bus_errors, "ui_drops": self.ui_drops,
            "missed_rounds": self.missed_rounds, "adapter_overflows": self.adapter_overflows,
            "transport": "CAN",
            "can_bus": self.can_bus,
        }

    def transmit_loop(self):
        """Shared bus: RPC only. Explicit standalone mode owns motor queries."""
        slot = 0
        query_motors = tuple(m for m in self.motors if m not in LEGACY_MOTORS)
        deadline = time.perf_counter()
        try:
            if not self.direct and self.can_bus == 1:
                # No RPC endpoint on CAN1. Use explicitly labelled local decode
                # scales; never compete with MCU queries or guess a readback.
                self.limits = {m: CONFIG_LIMITS[m] for m in self.motors}
                self.events.put(("limits", self.limits.copy()))
                self.ready.set()
                self.events.put(("connected", None))
                while not self.stop_requested.wait(0.02):
                    pass
                return
            if not self.direct:
                self.tuning.discover()  # Read-only; never auto START or take over.
                if not self.tuning.supported or not self.tuning.online:
                    raise RuntimeError("MCU 未就绪；未回退为 PC 电机查询")
                self.read_mcu_limits()
                self.ready.set()
                self.events.put(("connected", None))
            while not self.stop_requested.is_set():
                if not self.direct:
                    self.tuning.service()
                if not self.direct or self.paused.is_set():
                    self.stop_requested.wait(0.02)
                    deadline = time.perf_counter()
                    continue
                interval = 1 / (len(query_motors) * self.hz)
                now = time.perf_counter()
                if now < deadline:
                    time.sleep(min(deadline - now, 0.02))
                    continue
                skipped = max(0, int((now - deadline) / interval))
                self.missed_rounds += skipped
                if not self.stop_requested.is_set() and not self.paused.is_set():
                    self.send_query(query_motors[slot % len(query_motors)])
                    slot += 1
                deadline += (skipped + 1) * interval
        except Exception as exc:
            self.events.put(("error", f"监控初始化/发送失败：{type(exc).__name__}: {exc}"))
            self.stop_requested.set()

    def run(self):
        util = None
        claimed = started = False
        cleanup_error = None
        transmitter = None
        try:
            self.core, util, backend = usb_modules()
            self.dev = self.core.find(
                idVendor=self.adapter["vendor"], idProduct=self.adapter["product"],
                bus=self.adapter["bus"], address=self.adapter["address"], backend=backend)
            if self.dev is None or (self.adapter["serial"] and self.dev.serial_number != self.adapter["serial"]):
                raise RuntimeError("选中的适配器已拔出或地址改变，请刷新后重选。")
            util.claim_interface(self.dev, 0)
            claimed = True
            capability = struct.unpack("<10I", bytes(
                self.dev.ctrl_transfer(0xC1, 4, 0, 0, 40, timeout=500)))
            # This adapter was physically verified: 48 MHz, Classic CAN 1 Mbps.
            if capability[1] != 48_000_000 or not capability[0] & 8:
                raise RuntimeError("当前仅支持已验证的 48MHz、支持单次发送的 GS_USB 适配器。")
            self.dev.ctrl_transfer(0x41, 2, 0, 0, struct.pack("<II", 0, 0), timeout=500)
            self.dev.ctrl_transfer(0x41, 1, 0, 0, struct.pack("<5I", 1, 10, 4, 1, 3), timeout=500)
            started = True  # Attempt cleanup even if the START reply times out.
            self.dev.ctrl_transfer(0x41, 2, 0, 0, struct.pack("<II", 1, 8), timeout=500)
            # One-shot TX; no USB reset, auto reconnect or automatic motor commands.
            if self.direct:
                if not self.read_limits():
                    return
                self.ready.set()
                self.events.put(("connected", None))
            transmitter = threading.Thread(target=self.transmit_loop, name="dm-query-tx", daemon=False)
            transmitter.start()
            next_stats = time.perf_counter()
            while not self.stop_requested.is_set():
                reply = self.receive(20)
                if reply is not None:
                    can_id, data = reply
                    motor = next(m for m in self.motors if self.motors[m] == can_id)
                    if motor not in self.limits:
                        continue  # Decode only after the matching scales arrive.
                    now = time.perf_counter()
                    try:
                        sample = self.make_sample(can_id, data, now, time.time())
                    except ValueError:
                        self.invalid += 1
                    else:
                        self.counts[motor] += 1
                        self.arrivals[motor].append(now)
                        try:
                            self.samples.put_nowait(sample)
                        except queue.Full:
                            self.ui_drops += 1
                if time.perf_counter() >= next_stats:
                    self.last_stats = self.statistics()
                    self.events.put(("stats", self.last_stats))
                    next_stats = time.perf_counter() + 0.25
        except Exception as exc:
            self.events.put(("error", f"{type(exc).__name__}: {exc}"))
        finally:
            self.ready.clear()
            self.stop_requested.set()
            self.tuning.online = False
            if transmitter is not None:
                transmitter.join()  # Every USB write has a finite 300ms timeout.
            if self.dev is not None:
                try:
                    if started:
                        self.dev.ctrl_transfer(0x41, 2, 0, 0, struct.pack("<II", 0, 0), timeout=500)
                except Exception as exc:
                    cleanup_error = f"适配器停止未确认：{exc}；请拔下 USB 确认停止查询。"
                finally:
                    if util is not None:
                        try:
                            if claimed:
                                util.release_interface(self.dev, 0)
                        except Exception as exc:
                            cleanup_error = cleanup_error or f"释放 USB 接口失败：{exc}"
                        finally:
                            try:
                                util.dispose_resources(self.dev)
                            except Exception as exc:
                                cleanup_error = cleanup_error or f"关闭 USB 句柄失败：{exc}"
            self.last_stats = self.statistics()
            self.events.put(("stats", self.last_stats))
            if cleanup_error:
                self.events.put(("error", cleanup_error))
            self.events.put(("stopped", cleanup_error))


def scan_serial_ports():
    from serial.tools import list_ports
    return [{"port": p.device, "label": f"{p.device} · {p.description}"}
            for p in sorted(list_ports.comports(), key=lambda p: p.device)]


def open_serial(port):
    import serial
    # Set control lines BEFORE opening. Do not reset the MCU to connect.
    device = serial.Serial(port=None, baudrate=dm_serial.BAUDRATE,
                           bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                           stopbits=serial.STOPBITS_ONE, xonxoff=False, rtscts=False,
                           dsrdtr=False, timeout=0.02, write_timeout=0.3)
    device.dtr = device.rts = False
    device.port = port
    try:
        device.open()
    except Exception:
        device.close()
        raise
    return device


class SerialReader(threading.Thread):
    """UART owns telemetry/RPC; never opens CAN or directly commands a motor."""

    def __init__(self, adapter, samples, events, origin):
        super().__init__(name="dm-serial-monitor", daemon=False)
        self.adapter, self.samples, self.events, self.origin = adapter, samples, events, origin
        self.stop_requested, self.ready = threading.Event(), threading.Event()
        self.direct = False
        self.limits, self.last_stats = {}, {}
        self.counts, self.arrivals = {}, {}
        self.received_counts = {}
        self.tx_rpc = self.invalid = self.other = self.ui_drops = self.sequence_gaps = 0
        self.parser, self.dev = dm_serial.Parser(), None
        self.telemetry = None
        self.telemetry_at = time.perf_counter()
        self.reliable = dm_serial.ReliableRPC(self.write_serial, self.stop_requested)
        self.tuning = dm_tuning.Client(self.send_tuning, events, self.stop_requested)

    def write_serial(self, packet):
        written = self.dev.write(packet)
        self.tx_rpc += 1
        return written

    def send_tuning(self, payload):
        # Firmware prioritizes pending RPC replies over the 20ms telemetry.
        # Leave a telemetry slot during a burst of snapshot reads; otherwise
        # those reads can starve the very heartbeat needed by the next RPC.
        # RX remains running, and neither intent lifetime nor retries change.
        deadline = time.perf_counter() + .25
        while (self.telemetry is None or
               time.perf_counter() - self.telemetry_at >= .020):
            if time.perf_counter() >= deadline or self.stop_requested.wait(.002):
                raise RuntimeError("MCU 时间戳不新鲜，未发送控制请求")
        status = self.telemetry
        self.tuning.receive(self.reliable.request(payload, status["tick_ms"]))

    def transmit_loop(self):
        try:
            deadline = time.perf_counter() + 1
            while self.telemetry is None and not self.stop_requested.is_set():
                if time.perf_counter() >= deadline:
                    raise TimeoutError("未收到 MCU 遥测，未建立可靠串口会话")
                self.stop_requested.wait(0.01)
            if self.stop_requested.is_set():
                return
            self.reliable.start()
            if {joint["id"] for joint in self.telemetry["joints"]} != {1, 2}:
                # The current RAM editor is an explicit 38-parameter 2-axis
                # contract. New counted layouts remain useful read-only;
                # never apply a two-axis configuration to a different robot.
                self.ready.set()
                self.events.put(("connected", None))
                self.events.put(("monitor_only", "多轴遥测已连接；当前调参协议仍为双轴38项，未下发配置或运动。"))
                while not self.stop_requested.wait(0.02):
                    pass
                return
            self.tuning.discover()  # Read-only. Connecting never enables a motor.
            if not self.tuning.supported or not self.tuning.online:
                raise RuntimeError("未识别配套串口固件；不会回退到 USB-CAN")
            for motor in sorted(joint["id"] for joint in self.telemetry["joints"]):
                values = tuple(self.tuning.rpc(1, motor, field, floating=True)
                               for field in (240, 241, 242))
                if any(not math.isfinite(v) or v <= 0 for v in values):
                    raise RuntimeError(f"J{motor} MCU 解码量程无效")
                self.limits[motor] = values
            self.events.put(("limits", self.limits.copy()))
            self.ready.set()
            self.events.put(("connected", None))
            while not self.stop_requested.is_set():
                self.tuning.service()
                self.stop_requested.wait(0.02)
        except Exception as exc:
            if not self.stop_requested.is_set():
                self.events.put(("error", f"串口初始化/发送失败：{type(exc).__name__}: {exc}"))
            self.stop_requested.set()

    def accept_telemetry(self, payload):
        status = dm_serial.telemetry(payload)
        if self.telemetry is not None:
            identity = lambda s: sorted((j["id"], j["can_id"], j["feedback_id"]) for j in s["joints"])
            if identity(status) != identity(self.telemetry):
                raise RuntimeError("当前连接的关节身份发生变化；请重新连接核对设备")
            # Unsigned subtraction tolerates HAL tick rollover, not a reboot.
            if ((status["tick_ms"] - self.telemetry["tick_ms"]) & 0xFFFFFFFF) > 0x80000000:
                raise RuntimeError("MCU 时钟回退/重启；请手动重连，不沿用旧状态")
            gap = (status["sequence"] - self.telemetry["sequence"]) & 0xFFFFFFFF
            if gap > 0x80000000:
                raise RuntimeError("MCU 串口序号回退；请手动重连")
            if gap == 0:
                return  # Duplicate/replayed frame is not a fresh MCU heartbeat.
            self.sequence_gaps += max(0, gap - 1)
        now, unix = time.perf_counter(), time.time()
        self.telemetry, self.telemetry_at = status, now
        self.events.put(("serial_status", (status, now)))
        for joint in status["joints"]:
            motor = joint["id"]
            self.counts.setdefault(motor, 0)
            self.arrivals.setdefault(motor, deque(maxlen=2000))
            if not joint["valid"] or self.received_counts.get(motor) == joint["count"]:
                continue
            self.received_counts[motor] = joint["count"]
            age_s = joint["age_ms"] / 1000
            sample = Sample(motor, now - self.origin - age_s, unix - age_s,
                            joint["position"], joint["velocity"], joint["torque"],
                            joint["mos"], joint["rotor"], joint["state"], b"",
                            can_id=joint["can_id"], feedback_id=joint["feedback_id"],
                            tick_ms=status["tick_ms"], feedback_ms=joint["received_ms"],
                            sequence=status["sequence"], reference=joint["reference"],
                            target=joint["target"], target_velocity=joint["target_velocity"],
                            target_reference=joint["target_reference"], tracking_error=joint["tracking_error"],
                            kp=joint["kp"], kd=joint["kd"], ff=joint["ff"],
                            task_state=status["state"], task_reason=status["reason"],
                            mode=status["mode"], phase=status["phase"],
                            source="UART", reference_source="MCU")
            self.counts[motor] += 1
            if age_s > 0.5:
                self.arrivals[motor].clear()
            if age_s <= 0.5:
                self.arrivals[motor].append(now)
            try:
                self.samples.put_nowait(sample)
            except queue.Full:
                self.ui_drops += 1

    def statistics(self):
        now = time.perf_counter()
        for times in self.arrivals.values():
            while times and times[0] < now - 1:
                times.popleft()
        return dict(rx=self.counts.copy(), hz={m: len(t) for m, t in self.arrivals.items()},
                    tx_rpc=self.tx_rpc, invalid=self.invalid + self.parser.errors,
                    other=self.other, ui_drops=self.ui_drops, serial_gaps=self.sequence_gaps,
                    mcu_rx=self.received_counts.copy(), transport="USART1")

    def run(self):
        transmitter = None
        try:
            self.dev = open_serial(self.adapter["port"])
            transmitter = threading.Thread(target=self.transmit_loop,
                                           name="dm-serial-rpc", daemon=False)
            transmitter.start()
            next_stats = time.perf_counter()
            while not self.stop_requested.is_set():
                data = self.dev.read(min(512, self.dev.in_waiting or 1))
                for kind, payload in self.parser.feed(data):
                    if kind in (dm_serial.SESSION_ACK, dm_serial.RELIABLE_RESPONSE):
                        self.reliable.receive(kind, payload)
                    elif kind == dm_serial.TELEMETRY:
                        try:
                            self.accept_telemetry(payload)
                        except ValueError:
                            self.invalid += 1
                    else:
                        self.other += 1
                now = time.perf_counter()
                if now - self.telemetry_at > (1 if self.telemetry is not None else 3):
                    raise TimeoutError("串口状态回传超时；电机实际状态未知，请检查接线/固件")
                if now >= next_stats:
                    self.last_stats = self.statistics()
                    self.events.put(("stats", self.last_stats))
                    next_stats = now + 0.25
        except Exception as exc:
            self.events.put(("error", f"{type(exc).__name__}: {exc}"))
        finally:
            self.stop_requested.set()
            self.ready.clear()
            self.tuning.online = False
            if transmitter is not None:
                transmitter.join()  # Finite write timeout, RPC wait checks stop.
            cleanup_error = None
            if self.dev is not None:
                try:
                    self.dev.close()  # No motor action/reset on disconnect.
                except Exception as exc:
                    cleanup_error = f"关闭串口失败：{exc}"
                    self.events.put(("error", cleanup_error))
            self.last_stats = self.statistics()
            self.events.put(("stats", self.last_stats))
            self.events.put(("stopped", cleanup_error))


class USBReader(threading.Thread):
    """CDC RX plus one bounded reliable RPC TX owner; no direct motor queries."""

    def __init__(self, adapter, samples, events, origin):
        super().__init__(name="dm-usb-cdc-monitor", daemon=False)
        self.adapter, self.samples, self.events, self.origin = adapter, samples, events, origin
        self.stop_requested, self.ready = threading.Event(), threading.Event()
        self.direct = False
        self.tuning = dm_tuning.Client(self.send_tuning, events, self.stop_requested)
        self.tuning.poll_details = False
        self.dev = None
        self.reliable = dm_serial.ReliableRPC(self.write_serial, self.stop_requested)
        self.parser, self.collector = usb_monitor.Stream(), usb_monitor.Collector()
        self.telemetry, self.telemetry_at = None, time.perf_counter()
        self.counts, self.arrivals, self.received_counts, self.motors = {}, {}, {}, {}
        self.limits, self.last_stats = {}, {}
        self.invalid = self.other = self.ui_drops = self.tx_rpc = 0
        self.bytes_received = self.snapshots = 0
        self.started_at = time.perf_counter()
        self.identity = None

    write_serial = SerialReader.write_serial
    send_tuning = SerialReader.send_tuning

    def transmit_loop(self):
        try:
            while self.telemetry is None and not self.stop_requested.wait(.01):
                pass
            if self.stop_requested.is_set():
                return
            self.reliable.start()
            if self.tuning.rpc(0) != dm_tuning.VERSION:
                raise RuntimeError("USB控制协议版本不匹配")
            self.tuning.capture_supported = self.tuning.rpc(12, field=255) == dm_tuning.MOTION_VERSION
            self.tuning.motion_supported = self.tuning.capture_supported
            self.tuning.supported = self.tuning.online = True
            self.events.put(("tuning_supported", True))
            while not self.stop_requested.is_set():
                self.tuning.service()
                if self.reliable.failed:
                    raise RuntimeError("USB控制应答丢失，结果未知；需手动重连，不能重放动作")
                self.stop_requested.wait(.02)
        except Exception as exc:
            self.tuning.supported = self.tuning.online = False
            if not self.stop_requested.is_set():
                self.events.put(("monitor_only", f"USB遥测继续；控制不可用：{exc}"))

    def accept_snapshot(self, snapshot, now):
        identity = tuple((j["joint"], j["bus"], j["protocol"], j["command_id"], j["feedback_id"])
                         for j in snapshot["joints"])
        if self.identity is not None and identity != self.identity:
            raise RuntimeError("USB关节身份发生变化，请手动重连核对")
        if self.telemetry is not None and (
                (snapshot["tick_ms"] - self.telemetry["tick_ms"]) & 0xFFFFFFFF) > 0x80000000:
            raise RuntimeError("MCU时钟回退/重启，请手动重连")
        self.identity = identity
        self.telemetry, self.telemetry_at = snapshot, now
        self.snapshots += 1
        unix = time.time()
        for joint in snapshot["joints"]:
            motor = joint["joint"]
            self.motors[motor] = joint["feedback_id"]
            self.counts.setdefault(motor, 0)
            self.arrivals.setdefault(motor, deque(maxlen=2000))
            old_count = self.received_counts.get(motor)
            self.received_counts[motor] = joint["received_count"]
            # Fast repeated snapshots must not fabricate new encoder samples.
            if not joint["received"] or old_count == joint["received_count"]:
                continue
            age_s = joint["age_ms"] / 1000
            sample = Sample(motor, now - self.origin - age_s, unix - age_s,
                            joint["position_rad"], joint["velocity_rad_s"], joint["torque_nm"],
                            joint["mos_c"], joint["rotor_c"], joint["state"], b"",
                            can_id=joint["command_id"], feedback_id=joint["feedback_id"],
                            tick_ms=snapshot["tick_ms"], feedback_ms=joint["received_tick_ms"],
                            sequence=snapshot["sequence"], source="USB",
                            can_bus=joint["bus"], limits_source="MCU decoded; no motor register read")
            self.counts[motor] += 1
            if joint["fresh"]:
                self.arrivals[motor].append(now)
            else:
                self.arrivals[motor].clear()
            try:
                self.samples.put_nowait(sample)
            except queue.Full:
                self.ui_drops += 1
        if not self.ready.is_set():
            self.ready.set()
            self.events.put(("connected", None))

    def statistics(self):
        now = time.perf_counter()
        for times in self.arrivals.values():
            while times and times[0] < now - 1:
                times.popleft()
        elapsed = max(now - self.started_at, 1e-6)
        return dict(rx=self.counts.copy(), hz={m: len(t) for m, t in self.arrivals.items()},
                    tx_rpc=self.tx_rpc, invalid=self.invalid + self.parser.malformed,
                    ui_drops=self.ui_drops, serial_gaps=self.collector.gaps,
                    incomplete=self.collector.incomplete, resyncs=self.collector.resyncs,
                    discarded_bytes=self.parser.discarded, snapshots=self.snapshots,
                    snapshot_hz=self.snapshots / elapsed,
                    payload_kB_s=self.bytes_received / elapsed / 1000,
                    mcu_rx=self.received_counts.copy(), transport="USB CDC")

    def run(self):
        transmitter = None
        port_context = None
        try:
            self.started_at = self.telemetry_at = time.perf_counter()
            port_context = usb_monitor.serial_port(self.adapter["port"])
            port = port_context.__enter__()
            if port is not None:
                self.dev = port
                transmitter = threading.Thread(target=self.transmit_loop, name="dm-usb-rpc", daemon=False)
                transmitter.start()
                next_stats = self.started_at
                while not self.stop_requested.is_set():
                    data = port.read(min(4096, port.in_waiting or 1))
                    self.bytes_received += len(data)
                    now = time.perf_counter()
                    for kind, record in self.parser.messages(data):
                        if kind is not None:
                            self.reliable.receive(kind, record)
                            continue
                        try:
                            snapshot = self.collector.add(record, now)
                        except ValueError:
                            self.invalid += 1
                            continue
                        if snapshot is not None:
                            self.accept_snapshot(snapshot, now)
                    if now - self.telemetry_at > (1 if self.telemetry is not None else 3):
                        raise TimeoutError("USB遥测超时，请检查板载USB数据线及配套固件；电机状态未知")
                    if now >= next_stats:
                        self.last_stats = self.statistics()
                        self.events.put(("stats", self.last_stats))
                        if self.telemetry:
                            # Coalesce status events to 4Hz, not 1000Hz.
                            self.events.put(("usb_status", (self.telemetry, self.telemetry_at)))
                        next_stats = now + .25
        except Exception as exc:
            if not self.stop_requested.is_set():
                self.events.put(("error", f"USB {type(exc).__name__}: {exc}"))
        finally:
            self.stop_requested.set()
            self.ready.clear()
            self.tuning.online = False
            if transmitter is not None:
                transmitter.join()  # Bounded write/RPC waits observe stop.
            if port_context is not None and self.dev is not None:
                try:
                    port_context.__exit__(None, None, None)
                except Exception as exc:
                    self.events.put(("error", f"关闭USB失败：{exc}"))
            self.dev = None
            self.last_stats = self.statistics()
            self.events.put(("stats", self.last_stats))
            self.events.put(("stopped", None))


def envelope(points, columns):
    """Keep extrema when reducing a dense trace to screen pixels."""
    if len(points) <= columns * 2:
        return points
    start, end = points[0][0], points[-1][0]
    if end <= start:
        return points[:1]
    result, bucket, index = [], [], -1
    for point in points:
        col = min(columns - 1, int((point[0] - start) / (end - start) * columns))
        if col != index and bucket:
            lo = min(range(len(bucket)), key=lambda i: bucket[i][1])
            hi = max(range(len(bucket)), key=lambda i: bucket[i][1])
            result.extend(bucket[i] for i in sorted({0, lo, hi, len(bucket) - 1}))
            bucket = []
        bucket.append(point)
        index = col
    if bucket:
        lo = min(range(len(bucket)), key=lambda i: bucket[i][1])
        hi = max(range(len(bucket)), key=lambda i: bucket[i][1])
        result.extend(bucket[i] for i in sorted({0, lo, hi, len(bucket) - 1}))
    return result


CSV_FIELDS = ("time_utc", "elapsed_s", "motor_can_id", "feedback_can_id",
              "position_rad", "position_deg", "velocity_rad_s", "estimated_torque_nm",
              "mos_c", "rotor_c", "state", "logical_joint", "mcu_tick_ms", "feedback_ms",
              "telemetry_sequence", "reference_rad", "target_raw_rad", "target_velocity_rad_s",
              "target_reference_rad", "tracking_error_rad", "kp", "kd", "torque_ff_nm",
              "task_state", "task_reason", "mode", "phase", "raw_hex",
              "data_source", "reference_source", "command_elapsed_s", "command_age_ms",
              "status_age_ms", "command_raw_hex", "can_bus", "limits_source")


def write_csv(path, records):
    with open(path, "w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.writer(stream)
        writer.writerow(CSV_FIELDS)
        for s in records:
            writer.writerow((
                datetime.fromtimestamp(s.unix_s, timezone.utc).isoformat(timespec="milliseconds"),
                f"{s.elapsed_s:.6f}", f"0x{(s.can_id if s.can_id is not None else s.motor):02X}",
                (f"0x{s.feedback_id:02X}" if s.feedback_id is not None
                 else f"0x{MOTORS[s.motor]:02X}" if s.motor in MOTORS else ""),
                f"{s.position_rad:.8f}", f"{s.position_deg:.6f}", f"{s.velocity_rad_s:.8f}",
                f"{s.torque_nm:.8f}", s.mos_c, s.rotor_c, s.state, s.motor,
                s.tick_ms, s.feedback_ms, s.sequence, s.reference, s.target,
                s.target_velocity, s.target_reference, s.tracking_error, s.kp, s.kd, s.ff,
                s.task_state, s.task_reason, s.mode, s.phase, s.raw.hex(" ").upper(),
                s.source, s.reference_source, s.command_elapsed_s, s.command_age_ms,
                s.status_age_ms, s.command_raw.hex(" ").upper(),
                f"CAN{s.can_bus}" if s.can_bus is not None else "", s.limits_source))


class Monitor:
    def __init__(self, root, hz=100, autostart=False, can=True, can_bus=2, usb=False, port=None):
        import tkinter as tk
        from tkinter import ttk, font as tkfont
        self.tk, self.ttk, self.root = tk, ttk, root
        self.is_usb = usb
        self.is_can = can = can and not usb
        self.preferred_port = port
        self.can_bus = can_bus
        self.plot_font = tkfont.Font(root=root, family="Consolas", size=9)
        self.origin = time.perf_counter()
        self.samples, self.events = queue.Queue(maxsize=8192), queue.SimpleQueue()
        self.records = deque(maxlen=MAX_RECORDS)
        self.history = {m: deque() for m in MOTORS}
        self.latest, self.limits, self.stats = {}, {}, {}
        self.serial_status = None
        self.usb_status = None
        self.adapters, self.worker = [], None
        self.closing = self.exporting = False
        self.pressed_keys = set()
        self.error_text = ""
        self.tuning_panel = None
        self.tuning_snapshot = None
        self.quick_busy = False
        self.detail_widgets = []
        self.show_details = tk.BooleanVar(value=False)
        self.received = 0
        root.title("Ylamp · 五电机控制台")
        scale = root.winfo_fpixels("1i") / 96
        width = min(round(1420 * scale), root.winfo_screenwidth() - 80)
        height = min(round(920 * scale), root.winfo_screenheight() - 100)
        root.geometry(f"{width}x{height}")
        root.minsize(min(round(1120 * scale), width), min(round(780 * scale), height))
        root.configure(bg=BG)
        root.protocol("WM_DELETE_WINDOW", self.close)
        style = ttk.Style(root)
        style.theme_use("clam")
        style.configure(".", font=("Microsoft YaHei UI", 10), background=BG, foreground=TEXT)
        style.configure("TFrame", background=BG)
        style.configure("Card.TFrame", background=PANEL)
        style.configure("TLabel", background=BG, foreground=TEXT)
        style.configure("Muted.TLabel", foreground=MUTED)
        style.configure("Card.TLabel", background=PANEL)
        style.configure("TButton", padding=(12, 7), background="#2a3a52", foreground=TEXT)
        style.map("TButton", background=[("active", "#405879")], foreground=[("disabled", "#75849a")])
        style.configure("Accent.TButton", background="#176b7a")
        style.configure("TCombobox", fieldbackground=PANEL, background=PANEL,
                        foreground=TEXT, arrowcolor=TEXT)
        style.map("TCombobox", fieldbackground=[("readonly", PANEL)],
                  foreground=[("readonly", TEXT)])
        style.configure("TSpinbox", fieldbackground=PANEL, background=PANEL, foreground=TEXT)
        style.configure("TEntry", fieldbackground=PANEL, foreground=TEXT, insertcolor=TEXT)
        style.configure("TNotebook", background=BG)
        style.configure("TNotebook.Tab", background=PANEL, foreground=TEXT, padding=(12, 5))
        style.map("TNotebook.Tab", background=[("selected", "#2a3a52"), ("active", "#405879")])
        style.configure("TCheckbutton", background=BG, foreground=TEXT)
        outer = ttk.Frame(root, padding=18)
        outer.pack(fill="both", expand=True)
        title = ttk.Frame(outer)
        title.pack(fill="x")
        ttk.Label(title, text="Ylamp / 电机控制台", font=("Microsoft YaHei UI", 21, "bold")).pack(side="left")
        ttk.Label(title, text="板载 USB CDC · 五电机遥测 · J1/J2 简单控制" if usb else
                  "USB-CAN · Classic CAN 1 Mbps" if can
                  else f"USART1 {dm_serial.BAUDRATE} · MCU→电机 CAN",
                  style="Muted.TLabel").pack(side="right")
        ttk.Label(outer, text="连接不会启动电机。两路电源由固件联动，本界面不切换电源。",
                  style="Muted.TLabel").pack(anchor="w", pady=(5, 8))
        bus_row = ttk.Frame(outer)
        bus_row.pack(fill="x", pady=(0, 8))
        self.bus_var = tk.StringVar(value=BUS_LABELS[can_bus])
        self.bus_box = ttk.Combobox(bus_row, textvariable=self.bus_var, width=30,
                                    values=tuple(BUS_LABELS.values()), state="readonly")
        if can:
            self.bus_box.pack(side="left", padx=(0, 10))
        self.bus_box.bind("<<ComboboxSelected>>", self.select_bus)
        self.bus_hint = tk.StringVar()
        ttk.Label(bus_row, textvariable=self.bus_hint, style="Muted.TLabel").pack(side="left")
        toolbar = ttk.Frame(outer)
        toolbar.pack(fill="x")
        self.adapter_var = tk.StringVar()
        self.adapter_box = ttk.Combobox(toolbar, textvariable=self.adapter_var, width=34, state="readonly")
        self.adapter_box.pack(side="left", padx=(0, 6))
        self.refresh_button = ttk.Button(toolbar, text="刷新", command=self.refresh_adapters)
        self.refresh_button.pack(side="left", padx=3)
        self.connect_button = ttk.Button(toolbar, text="连接", style="Accent.TButton", command=self.connect)
        self.connect_button.pack(side="left", padx=3)
        self.disconnect_button = ttk.Button(toolbar, text="断开", command=self.disconnect, state="disabled")
        self.disconnect_button.pack(side="left", padx=3)
        self.export_button = ttk.Button(toolbar, text="导出记录", command=self.export)
        self.export_button.pack(side="right")
        self.tuning_button = ttk.Button(toolbar, text="高级设置…", command=self.open_tuning)
        self.tuning_button.pack(side="right", padx=8)
        controls = ttk.Frame(outer)
        controls.pack(fill="x", pady=(10, 6))
        ttk.Label(controls, text="J1 / J2：").pack(side="left")
        self.capture_button = ttk.Button(controls, text="锁定当前位置",
                                         command=lambda: self.quick_command("impedance"), state="disabled")
        self.capture_button.pack(side="left", padx=3)
        self.hold_button = ttk.Button(controls, text="停止并保持 [F5]",
                                      command=lambda: self.quick_command(2), state="disabled")
        self.hold_button.pack(side="left", padx=3)
        self.release_button = ttk.Button(controls, text="释放力矩 [F6]",
                                         command=lambda: self.quick_command(3), state="disabled")
        self.release_button.pack(side="left", padx=3)
        ttk.Checkbutton(controls, text="诊断详情", variable=self.show_details,
                        command=self.toggle_details).pack(side="right")
        ttk.Label(controls, text="曲线 / 秒").pack(side="left", padx=(15, 5))
        self.span_var = tk.StringVar(value="10")
        ttk.Combobox(controls, textvariable=self.span_var, values=("5", "10", "30", "60"),
                     width=4, state="readonly").pack(side="left")
        ttk.Label(outer, text="锁定会出力保持，推开后会回拉；释放力矩会失去支撑，请托住负载。断开连接不会停机。",
                  style="Muted.TLabel").pack(anchor="w", pady=(0, 6))
        self.status_var = tk.StringVar(value="未连接；选择开发板原生 USB CDC 串口。" if usb else
                                       "未连接；先选总线，再点连接。" if can
                                       else "未连接；选择 USART1 的 USB-TTL 串口。")
        self.status_label = ttk.Label(outer, textvariable=self.status_var, foreground=COLORS[1],
                                      wraplength=width - 50)
        self.status_label.pack(anchor="w", pady=(0, 10))
        self.task_var = tk.StringVar(value="J1/J2 状态：未连接；J3～J5 仅查看反馈。")
        ttk.Label(outer, textvariable=self.task_var).pack(anchor="w", pady=(0, 6))
        self.diagnostics = ttk.Frame(outer)
        self.serial_var = tk.StringVar(value="MCU 状态：未连接")
        ttk.Label(self.diagnostics, textvariable=self.serial_var, wraplength=width - 50,
                  style="Muted.TLabel").pack(anchor="w")
        self.cards = ttk.Frame(outer)
        self.cards.pack(fill="x")
        self.card_vars, self.state_labels = {}, {}
        self.joint_layout = ()
        self.configure_joints(tuple((m, m, feedback) for m, feedback in MOTORS.items()
                                    if usb or can or m in (1, 2)))
        self.select_bus()
        charts = ttk.Frame(outer)
        charts.pack(fill="both", expand=True, pady=(12, 8))
        for n in (0, 1):
            charts.columnconfigure(n, weight=1)
            charts.rowconfigure(n, weight=1)
        self.canvases = []
        for i, (heading, unit) in enumerate((
                ("反馈位置", "°"), ("速度", "rad/s"), ("估算力矩 · 非外部力传感器", "N·m"),
                ("温度 · 实线 MOS / 虚线绕组", "°C"))):
            canvas = tk.Canvas(charts, background=PANEL, highlightthickness=0, height=200)
            canvas.grid(row=i // 2, column=i % 2, sticky="nsew", padx=(0, 8) if i % 2 == 0 else (0, 0),
                        pady=(0, 8) if i < 2 else (0, 0))
            self.canvases.append((canvas, heading, unit))
        self.count_var = tk.StringVar(value="尚无数据")
        ttk.Label(self.diagnostics, textvariable=self.count_var, style="Muted.TLabel",
                  wraplength=width - 50).pack(anchor="w")
        self.buffer_var = tk.StringVar(value="内存最多保留 120,000 条；超出覆盖最旧数据。CSV 仅导出当前保留数据。")
        ttk.Label(outer, textvariable=self.buffer_var, style="Muted.TLabel").pack(anchor="w", pady=(4, 0))
        self.last_plot = 0
        for key in ("F5", "F6"):
            root.bind(f"<KeyPress-{key}>", self.hotkey)
            root.bind(f"<KeyRelease-{key}>", self.release_hotkey)
        root.bind("<FocusOut>", lambda event: self.pressed_keys.clear())
        self.refresh_adapters()
        root.after(100 if usb else 33, self.tick)
        if autostart:
            root.after(200, self.connect)

    def select_bus(self, event=None):
        if self.worker is not None:
            return
        self.can_bus = next(bus for bus, label in BUS_LABELS.items() if label == self.bus_var.get())
        self.latest, self.limits, self.stats = {}, {}, {}
        for history in self.history.values():
            history.clear()
        self.bus_hint.set(
            ("适配器接 CAN2 H/L；J1/J2 配 VCC_OUT1 / PC13。" if self.can_bus == 2 else
             "适配器接 CAN1 H/L；J3～J5 配 VCC_OUT0 / PC14；ID、量程待核对。")
            if self.is_can else "板载 USB：汇总 CAN1/2；仅 J1/J2 原位锁定、保持、释放，连接不自动动作。"
            if self.is_usb else "USART1 备用链路；当前固件只回传 J1/J2。")
        self.tuning_button.configure(
            state="normal" if not self.is_usb and (not self.is_can or self.can_bus == 2) else "disabled")
        for variables in self.card_vars.values():
            for variable in variables.values():
                variable.set("—")
        self.update_quick_buttons()

    def toggle_details(self):
        for widget in self.detail_widgets:
            if self.show_details.get():
                widget.pack(anchor="w", pady=2)
            else:
                widget.pack_forget()
        if self.show_details.get():
            self.diagnostics.pack(fill="x", before=self.cards, pady=(0, 6))
        else:
            self.diagnostics.pack_forget()

    def control_connected(self):
        return bool((not self.is_can or self.can_bus == 2) and self.worker
                    and self.worker.ready.is_set() and self.worker.tuning.supported
                    and not self.worker.stop_requested.is_set())

    def update_quick_buttons(self):
        connected = self.control_connected()
        client = self.worker.tuning if connected else None
        busy = self.quick_busy or bool(self.tuning_panel and self.tuning_panel.busy)
        allowed = bool(connected and client.task_status and dm_tuning.capture_ready(
            self.worker, client.task_status[0], client.task_status_at, busy))
        self.capture_button.configure(state="normal" if allowed else "disabled")
        for button in (self.hold_button, self.release_button):
            button.configure(state="normal" if connected else "disabled")

    def quick_command(self, action):
        if not self.control_connected():
            self.status_var.set("J1/J2控制尚不可用；请等待USB控制握手或CAN2读取完成。")
            return
        if action == "impedance":
            self.update_quick_buttons()
            if str(self.capture_button["state"]) == "disabled":
                self.status_var.set("尚不能锁定：请等待新鲜 MCU 状态，或查看高级设置中的提示。")
                return
        if action == 3 and self.tuning_panel:
            self.tuning_panel.allow_motion.set(False)
        reason = self.worker.tuning.request("impedance" if action == "impedance" else "command",
                                            None if action == "impedance" else action)
        if reason:
            self.status_var.set(reason)
            return
        if action == "impedance":
            self.quick_busy = True
        self.status_var.set("请求已排队，等待 MCU 应答；电机是否执行请看新反馈。")
        self.update_quick_buttons()

    def configure_joints(self, layout):
        """One card per announced identity, no per-axis special cases."""
        layout = tuple(sorted(layout))
        if layout == self.joint_layout:
            return
        self.joint_layout = layout
        for child in self.cards.winfo_children():
            child.destroy()
        self.card_vars, self.state_labels = {}, {}
        self.detail_widgets = []
        for column in range(dm_serial.MAX_JOINTS):
            self.cards.columnconfigure(column, weight=1 if column < len(layout) else 0)
        # Compact typography for 3..6 joints keeps the existing plot area usable.
        many = len(layout) > 2
        for column, (motor, can_id, feedback_id) in enumerate(layout):
            card = self.ttk.Frame(self.cards, style="Card.TFrame", padding=8 if many else 14)
            card.grid(row=0, column=column, sticky="nsew", padx=3)
            heading = (f"J{motor} · {MOTOR_MODELS.get(motor, '电机')} · CAN{MOTOR_BUSES.get(motor, '?')}"
                       if self.is_can else f"J{motor}")
            self.ttk.Label(card, text=heading,
                           style="Card.TLabel", foreground=joint_color(motor),
                           font=("Microsoft YaHei UI", 10 if many else 12, "bold")).pack(anchor="w")
            self.ttk.Label(card, text=f"命令 0x{can_id:02X} / 反馈 0x{feedback_id:02X}",
                           style="Card.TLabel").pack(anchor="w")
            variables = {key: self.tk.StringVar(value="—") for key in
                         ("position", "motion", "temperature", "state", "raw", "limits", "control")}
            self.card_vars[motor] = variables
            self.history.setdefault(motor, deque())
            for key in ("position", "motion", "temperature", "state", "control", "limits", "raw"):
                label = self.ttk.Label(
                    card, textvariable=variables[key], style="Card.TLabel",
                    wraplength=180 if many else 550,
                    font=("Consolas", 14 if many else 25, "bold") if key == "position"
                    else ("Microsoft YaHei UI", 9 if many else 10))
                if key in ("limits", "raw"):
                    self.detail_widgets.append(label)
                    if self.show_details.get():
                        label.pack(anchor="w", pady=2)
                else:
                    label.pack(anchor="w", pady=2)
                if key == "state":
                    self.state_labels[motor] = label
            variables["control"].set("等待连接；不会自动使能")

    def hotkey(self, event):
        if event.keysym in ("F5", "F6") and event.keysym not in self.pressed_keys:
            self.pressed_keys.add(event.keysym)
            if self.worker is not None:
                self.quick_command(2 if event.keysym == "F5" else 3)
        return "break"

    def open_tuning(self):
        if self.is_usb:
            self.status_var.set("USB已提供主界面三个简单控制；高级参数/零点编辑仍使用原CAN2或USART1入口。")
            return
        if self.is_can and self.can_bus != 2:
            self.status_var.set("CAN1 仅查看 J3～J5；高级设置和 J1/J2 操作请接 CAN2。")
            return
        if self.tuning_panel is None:
            self.tuning_panel = dm_tuning.Panel(self)
        self.tuning_panel.window.deiconify()
        self.tuning_panel.window.lift()

    def release_hotkey(self, event):
        self.pressed_keys.discard(event.keysym)
        return "break"

    def refresh_adapters(self):
        if self.worker is not None:
            return
        try:
            self.adapters = scan_adapters() if self.is_can else scan_serial_ports()
            self.adapter_box.configure(values=[a["label"] for a in self.adapters])
            if self.adapters:
                preferred = next((i for i, a in enumerate(self.adapters)
                                  if a.get("port", "").casefold() == (self.preferred_port or "").casefold()), None)
                self.adapter_box.current(0 if preferred is None else preferred)
                self.status_var.set("请选择板载 USB CDC 串口，不是 ST-Link/USB-TTL/USB-CAN。" if self.is_usb else
                                    "已发现 USB-CAN；确认接线与所选 CAN 一致，再点连接。" if self.is_can
                                    else f"已发现串口；请选择 USART1 的 USB-TTL，{dm_serial.BAUDRATE} / 8N1。")
            else:
                self.adapter_var.set("")
                self.status_var.set("未发现串口；请连接板载USB数据口，并烧录配套USB固件。" if self.is_usb else
                                    "未发现 USB-CAN，请检查适配器与 WinUSB 驱动。" if self.is_can
                                    else "未发现串口，请检查 USB-TTL 连接与驱动。")
        except Exception as exc:
            self.status_var.set(str(exc))

    def connect(self):
        from tkinter import messagebox
        if self.worker is not None:
            return
        selected = self.adapter_box.current()
        if selected < 0 or selected >= len(self.adapters):
            messagebox.showerror("连接", "请先插入并选择板载 USB CDC 串口。" if self.is_usb else
                                 "请先插入并选择 USB-CAN。" if self.is_can
                                 else "请先插入并选择 USART1 的 USB-TTL 串口。", parent=self.root)
            return
        self.error_text = ""
        self.pressed_keys.clear()
        self.latest, self.limits, self.stats = {}, {}, {}
        self.serial_status = None
        self.usb_status = None
        self.tuning_snapshot = None
        self.quick_busy = False
        if self.tuning_panel:
            self.tuning_panel.snapshot = None
            self.tuning_panel.calibration = None
            self.tuning_panel.cal_confirm.set(False)
            self.tuning_panel.allow_motion.set(False)
            self.tuning_panel.status.set("新连接：等待当前设备回读；不使用旧连接参数启动。")
        for variables in self.card_vars.values():
            variables["control"].set("等待连接；不会自动使能")
        self.status_var.set("正在连接板载USB；只接收，不发送电机命令……" if self.is_usb else
                            (f"正在连接 CAN{self.can_bus}；不会启动电机……") if self.is_can
                            else f"正在连接 {dm_serial.BAUDRATE} 串口并读取 MCU……")
        self.status_label.configure(foreground=COLORS[1])
        self.worker = (USBReader(self.adapters[selected], self.samples, self.events, self.origin) if self.is_usb else
                       Reader(self.adapters[selected], 100, self.samples, self.events, self.origin,
                              can_bus=self.can_bus)
                       if self.is_can else
                       SerialReader(self.adapters[selected], self.samples, self.events, self.origin))
        self.adapter_box.configure(state="disabled")
        self.bus_box.configure(state="disabled")
        self.refresh_button.configure(state="disabled")
        self.connect_button.configure(state="disabled")
        self.disconnect_button.configure(state="normal")
        self.worker.start()

    def disconnect(self):
        if self.worker is not None:
            self.worker.stop_requested.set()
            self.disconnect_button.configure(state="disabled")
            self.update_quick_buttons()
            self.status_var.set("正在关闭监控连接；不改变电机状态……")

    def export(self):
        from tkinter import filedialog, messagebox
        if not self.records:
            messagebox.showinfo("导出", "尚无真实回传数据。", parent=self.root)
            return
        path = filedialog.asksaveasfilename(
            parent=self.root, title="导出当前保留数据", defaultextension=".csv",
            filetypes=[("CSV 数据", "*.csv")],
            initialfile=datetime.now().strftime("dm_motors_%Y%m%d_%H%M%S.csv"))
        if not path:
            return
        snapshot = tuple(self.records)
        self.exporting = True
        self.export_button.configure(state="disabled")

        def save():
            try:
                write_csv(path, snapshot)
                self.events.put(("export", (path, len(snapshot))))
            except Exception as exc:
                self.events.put(("export_error", str(exc)))
        threading.Thread(target=save, name="csv-export", daemon=False).start()

    def handle_events(self):
        from tkinter import messagebox
        for _ in range(200):
            try:
                kind, value = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == "limits":
                self.limits = value
                for m, (p, v, t) in value.items():
                    if m in self.card_vars:
                        source = ("配置值，未实机核对" if self.is_can and self.can_bus == 1
                                  else "MCU 配置回读，非电机寄存器")
                        self.card_vars[m]["limits"].set(
                            f"{source}：P ±{p:g} / V ±{v:g} / T ±{t:g}（非安全限位）")
            elif kind in ("tuning_readback", "tuning_applied", "tuning_error", "task_status", "task_ack",
                          "tuning_supported", "calibration_status", "calibration_ack", "calibration_error",
                          "drag_status", "drag_error", "motion_status", "motion_ack", "motion_error"):
                if kind in ("tuning_readback", "tuning_applied"):
                    self.tuning_snapshot = value
                elif kind == "task_status" and self.tuning_snapshot and self.tuning_snapshot["revision"] != value[4]:
                    self.tuning_snapshot = None
                elif kind == "tuning_error":
                    self.tuning_snapshot = None
                if kind in ("task_ack", "motion_ack", "tuning_error", "motion_error"):
                    if kind != "task_ack":
                        self.quick_busy = False
                    self.status_var.set(
                        f"操作未确认：{value}" if kind.endswith("error") else
                        "MCU 已应答；是否已保持或释放力矩，请看电机新反馈。")
                if self.tuning_panel:
                    self.tuning_panel.event(kind, value)
            elif kind == "connected":
                self.status_var.set(
                    "板载USB遥测已连接；控制握手后可手动锁定、保持、释放 J1/J2，不自动动作。" if self.is_usb else
                    ("CAN2 已连接：查看 J1/J2；可手动锁定、保持或释放力矩。"
                     if self.can_bus == 2 else
                     "CAN1 仅监听：J3旧协议随命令回包，空闲无连续反馈；解码量程为配置值。") if self.is_can
                                    else "串口连接成功；不自动启动、恢复或使能。")
            elif kind == "stats":
                self.stats = value
            elif kind == "serial_status":
                self.serial_status = value
                self.configure_joints(tuple((j["id"], j["can_id"], j["feedback_id"])
                                            for j in value[0]["joints"]))
            elif kind == "usb_status":
                self.usb_status = value
                self.configure_joints(tuple((j["joint"], j["command_id"], j["feedback_id"])
                                            for j in value[0]["joints"]))
            elif kind == "monitor_only":
                self.status_var.set(value)
            elif kind == "error":
                self.quick_busy = False
                self.error_text = value
                self.status_var.set(value)
                self.status_label.configure(foreground="#ff7b87")
                for variables in self.card_vars.values():
                    variables["control"].set("通信失败；电机实际状态未确认，不会自动重发或失能")
            elif kind == "stopped":
                if self.worker is not None and self.worker.is_alive():
                    self.events.put((kind, value))
                    break
                self.worker = None
                self.quick_busy = False
                self.adapter_box.configure(state="readonly")
                self.bus_box.configure(state="readonly")
                self.refresh_button.configure(state="normal")
                self.connect_button.configure(state="normal")
                self.disconnect_button.configure(state="disabled")
                for variables in self.card_vars.values():
                    variables["control"].set("已断开；未确认状态仅供参考，断开不会自动失能")
                if not self.error_text:
                    self.status_var.set("监控已断开；曲线与数据保留，未改变 MCU 或电机状态。")
            elif kind in ("export", "export_error"):
                self.exporting = False
                self.export_button.configure(state="normal")
                if kind == "export":
                    path, count = value
                    self.buffer_var.set(f"已导出 {count:,} 条 → {path}")
                else:
                    messagebox.showerror("导出失败", value, parent=self.root)

    def tick(self):
        self.handle_events()
        for _ in range(8192):
            try:
                sample = self.samples.get_nowait()
            except queue.Empty:
                break
            self.records.append(sample)
            history = self.history.setdefault(sample.motor, deque())
            # ponytail: plots only decimated to <=50Hz; retain every received
            # unique source sample in records/CSV. Plot peaks are not guaranteed.
            if not self.is_usb or not history or sample.elapsed_s - history[-1].elapsed_s >= .020:
                history.append(sample)
            self.latest[sample.motor] = sample
            self.received += 1
        now = time.perf_counter() - self.origin
        history_end = max((s.elapsed_s for s in self.latest.values()), default=now)
        for history in self.history.values():
            while history and history[0].elapsed_s < history_end - 65:
                history.popleft()
        for motor, variables in self.card_vars.items():
            if self.is_can and MOTOR_BUSES[motor] != self.can_bus:
                variables["position"].set(f"请接 CAN{MOTOR_BUSES[motor]}")
                variables["motion"].set("本路不接此电机")
                variables["temperature"].set("—")
                variables["state"].set("未接入当前总线，不判断在线状态")
                variables["control"].set("J1/J2 可操作" if motor <= 2 else "J3～J5 仅查看")
                self.state_labels[motor].configure(foreground=MUTED)
                continue
            sample = self.latest.get(motor)
            if sample is None:
                variables["position"].set("等待真实反馈")
                variables["motion"].set("速度 —    估算力矩 —")
                variables["temperature"].set("MOS —    绕组 —")
                variables["state"].set(
                    "旧协议：等待命令回包，不支持独立查询"
                    if (self.is_can or self.is_usb) and motor in LEGACY_MOTORS else "状态：未收到本次连接反馈")
                variables["raw"].set("RAW —")
                variables["control"].set("USB反馈未知 · 控制应答不代表已执行" if self.is_usb else
                                         "仅查看 · 量程未核对" if self.is_can and self.can_bus == 1
                                         else "不会自动启动")
                continue
            age = max(0, now - sample.elapsed_s)
            max_age_s = self.usb_status[0]["max_age_ms"] / 1000 if self.is_usb and self.usb_status else .5
            stale = age > max_age_s or self.worker is None
            rate = self.stats.get("hz", {}).get(motor, 0) if not stale else 0
            variables["position"].set(f"{sample.position_deg:+.3f}°   {sample.position_rad:+.5f} rad")
            variables["motion"].set(f"速度 {sample.velocity_rad_s:+.4f} rad/s    估算力矩 {sample.torque_nm:+.4f} N·m")
            variables["temperature"].set(f"MOS {sample.mos_c}°C    绕组 {sample.rotor_c}°C    上位机采样 {rate} Hz")
            state = STATES.get(sample.state, f"未知 0x{sample.state:X}")
            variables["state"].set(f"状态：{state}    数据龄 {age * 1000:.0f} ms"
                                   + ("  [历史/过期]" if stale else ""))
            self.state_labels[motor].configure(
                foreground="#ff7b87" if sample.state > 1 else "#ffcf7a" if sample.state == 1 else MUTED if stale else "#8ed5b0")
            variables["raw"].set(f"RAW {sample.raw.hex(' ').upper()}" if sample.raw
                                 else "MCU 解码回传（无原始 CAN 帧）")
            if self.is_can:
                variables["control"].set(
                    f"CAN MIT：Kp {sample.kp:.3g} / Kd {sample.kd:.3g}"
                    f" / 前馈 {sample.ff:+.4f} N·m（线值，非实测）"
                    if not stale and sample.kp is not None else "未收到新鲜 MIT 指令；不沿用历史出力")
                if self.can_bus == 1:
                    variables["control"].set("仅查看 · 数值按配置量程解码，未实机核对")
            elif self.is_usb:
                variables["control"].set("USB MCU缓存值 · 无MIT请求回传")
            if self.serial_status:
                status, arrived = self.serial_status
                joint = next((j for j in status["joints"] if j["id"] == motor), None)
                if joint is None:
                    continue
                historical = (self.worker is None or time.perf_counter() - arrived > 0.5
                              or status["state"] in (0, 5))
                variables["control"].set(
                    f"{'历史/残留' if historical else ''}MIT请求："
                    f"Kp {joint['kp']:.3g} / Kd {joint['kd']:.3g}"
                    f" / 前馈 {joint['ff']:+.4f} N·m（非实测）")
        if self.is_usb:
            snapshot, arrived = self.usb_status or (None, 0)
            stale = self.worker is None or time.perf_counter() - arrived > .5
            self.serial_var.set(
                f"USB {'[历史/过期]' if stale else '[数据流在线]'}"
                f" · 快照序号 {snapshot['sequence'] if snapshot else '—'}"
                " · 流在线不代表五台电机在线；源采样次数未变化不会生成新样本。")
        elif self.serial_status:
            status, arrived = self.serial_status
            stale = self.worker is None or time.perf_counter() - arrived > 0.5
            counters = (f"TEC {status['bo_tec']} / REC {status['bo_rec']}"
                        if status["can_flags"] & 8 else "计数未取得")
            bo = (f"曾 Bus-Off @{status['bo_ms']}ms：LEC {status['bo_lec']} / {counters}"
                  if status["can_flags"] & 2 else "尚无 Bus-Off 记录")
            first = (f"首错 @{status['first_ms']}ms LEC {status['first_lec']}"
                     if status["can_flags"] & 1 else "尚无首错记录")
            self.serial_var.set(
                f"MCU {'[历史/过期]' if stale else '[串口在线]'} "
                f"{dm_tuning.TASK_STATES[status['state']]} / {dm_tuning.REASONS[status['reason']]}"
                f" · {first} · {bo} · 失能提交 {status['disable_mask']:02b}"
                "（不代表电机已失能）"
                f" · UART接收 {'运行' if status['rx_running'] else '停止'}"
                f"{'/续接失败' if status['rx_rearm_failed'] else ''}"
                f" · UART RX溢出 {status['rx_overflows']} CRC错 {status['crc_errors']}"
                f" TX丢弃 {status['tx_drops']} RX错误 0x{status['uart_error']:X}")
        else:
            client = self.worker.tuning if self.worker else None
            if self.is_can and client and client.task_status:
                status = client.task_status
                stale = time.perf_counter() - client.task_status_at > 1.5 or not client.online
                self.serial_var.set(
                    f"MCU {'[历史/过期]' if stale else '[CAN RPC]'} "
                    f"{dm_tuning.TASK_STATES[status[0]]} / {dm_tuning.REASONS[status[1]]}"
                    " · 首错/Bus-Off完整取证仍可用 --serial 单独读取")
            else:
                self.serial_var.set("MCU 状态：尚无本次连接的有效回传")
        if self.is_can and self.can_bus == 1:
            self.task_var.set("本路仅查看 J3/J4/J5；操作 J1/J2 请断开后改接 CAN2。")
        else:
            client = self.worker.tuning if self.worker else None
            if client and client.task_status and client.online and (
                    time.perf_counter() - client.task_status_at < 1.5):
                state, reason = client.task_status[:2]
                self.task_var.set(f"J1/J2：{dm_tuning.TASK_STATES[state]} · {dm_tuning.REASONS[reason]}")
            else:
                self.task_var.set("J1/J2 状态未知；不能据此判断已停机或已失能。")
        self.update_quick_buttons()
        s = self.stats
        axis_counts = " / ".join(f"J{m} {s.get('rx', {}).get(m, 0):,}" for m in self.card_vars)
        mcu_counts = " / ".join(f"J{m} {s.get('mcu_rx', {}).get(m, 0):,}" for m in self.card_vars)
        self.count_var.set((
            f"USB {s.get('snapshot_hz', 0):.1f} 组/s · {s.get('payload_kB_s', 0):.1f} kB/s"
            f" · 源样本 {axis_counts} · MCU累计 {mcu_counts}"
            f" · 序号缺口 {s.get('serial_gaps', 0)} / 残组 {s.get('incomplete', 0)}"
            f" / 坏帧 {s.get('invalid', 0)} · 显示队列丢弃 {s.get('ui_drops', 0)}"
        ) if self.is_usb else (
            f"CAN MCU请求 {s.get('tx_rpc', 0):,} · 实收 {axis_counts}"
            f" · 无效帧 {s.get('invalid', 0)} / CAN错误帧 {s.get('bus_errors', 0)}"
            f" · 适配器溢出 {s.get('adapter_overflows', 0)}"
            f" · 显示队列丢弃 {s.get('ui_drops', 0)}"
        ) if self.is_can else (
            f"串口 MCU 请求 {s.get('tx_rpc', 0):,}"
            f" · 上位机采样 {axis_counts}"
            f" · MCU累计CAN回传 {mcu_counts}"
            f" · 无效串口帧 {s.get('invalid', 0)} / 回传序号缺口 {s.get('serial_gaps', 0)}"
            f" · 显示队列丢弃 {s.get('ui_drops', 0)}"))
        if not self.exporting and not self.buffer_var.get().startswith("已导出"):
            overwritten = max(0, self.received - MAX_RECORDS)
            self.buffer_var.set(f"缓存 {len(self.records):,} / {MAX_RECORDS:,} 条，已覆盖最旧 {overwritten:,} 条；"
                                "CSV 仅导出当前保留数据。显示队列丢弃不计入 CSV。"
                                + ("曲线≤50Hz抽样，接收/CSV不降采样。" if self.is_usb else ""))
        if now - self.last_plot >= (.1 if self.is_usb else 1 / 30):
            self.draw_plots()
            self.last_plot = now
        if self.closing and self.worker is None and not self.exporting:
            self.root.destroy()
            return
        self.root.after(100 if self.is_usb else 33, self.tick)

    def draw_plots(self):
        end = max((s.elapsed_s for s in self.latest.values()), default=0)
        span = float(self.span_var.get())
        begin = end - span
        visible = {}
        for motor, history in self.history.items():
            if self.is_can and MOTOR_BUSES.get(motor) != self.can_bus:
                continue
            points = []
            for sample in reversed(history):
                if sample.elapsed_s < begin:
                    break
                points.append(sample)
            visible[motor] = list(reversed(points))
        fields = ("position_deg", "velocity_rad_s", "torque_nm", "mos_c")
        for chart, (canvas, heading, unit) in enumerate(self.canvases):
            canvas.delete("all")
            width, height = canvas.winfo_width(), canvas.winfo_height()
            if width < 100 or height < 100:
                continue
            left, top, right, bottom = 65, 42, width - 16, height - 32
            canvas.create_text(14, 17, text=f"{heading}  [{unit}]", anchor="w",
                               fill=TEXT, font=("Microsoft YaHei UI", 10, "bold"))
            plotted = [m for m in self.card_vars if not self.is_can or MOTOR_BUSES.get(m) == self.can_bus]
            for index, motor in enumerate(plotted):
                canvas.create_text(width - 35*(len(plotted)-index), 32, text=f"J{motor}",
                                   fill=joint_color(motor), font=("Consolas", 9, "bold"))
            traces = []
            for motor, samples in visible.items():
                traces.append(([(s.elapsed_s, getattr(s, fields[chart])) for s in samples], joint_color(motor), False))
                if chart == 3:
                    traces.append(([(s.elapsed_s, s.rotor_c) for s in samples], joint_color(motor), True))
            values = [v for points, _, _ in traces for _, v in points]
            if values:
                low, high = min(values), max(values)
                pad = max((high - low) * 0.12, (0.02, 0.02, 0.02, 1)[chart])
                low, high = low - pad, high + pad
            else:
                low, high = (-1, 1) if chart != 3 else (0, 50)
            ticks = [f"{high - i * (high - low) / 4:.3g}" for i in range(5)]
            left = max(left, max(self.plot_font.measure(label) for label in ticks) + 14)
            for i in range(5):
                y = top + i * (bottom - top) / 4
                canvas.create_line(left, y, right, y, fill="#29394e")
                canvas.create_text(left - 7, y, text=ticks[i], fill=MUTED, anchor="e", font=self.plot_font)
            for i in range(5):
                x = left + i * (right - left) / 4
                canvas.create_line(x, top, x, bottom, fill="#233247")
                canvas.create_text(x, bottom + 14, text=f"{-span + i * span / 4:g}s",
                                   fill=MUTED, font=("Consolas", 9))
            for points, color, dashed in traces:
                # ponytail: bounded history + pixel extrema; no plotting dependency needed.
                points = envelope(points, max(1, int((right - left) / 2)))
                segment, previous = [], None
                for t, value in points:
                    if previous is not None and t - previous > 0.5:
                        if len(segment) >= 4:
                            canvas.create_line(*segment, fill=color, width=1.7, dash=(4, 3) if dashed else ())
                        segment = []
                    segment.extend((left + (t - begin) / span * (right - left),
                                    bottom - (value - low) / (high - low) * (bottom - top)))
                    previous = t
                if len(segment) >= 4:
                    canvas.create_line(*segment, fill=color, width=1.7, dash=(4, 3) if dashed else ())
                elif len(segment) == 2:
                    x, y = segment
                    canvas.create_oval(x - 2, y - 2, x + 2, y + 2, fill=color, outline="")
            if not values:
                canvas.create_text((left + right) / 2, (top + bottom) / 2,
                                   text="等待真实回传数据", fill=MUTED, font=("Microsoft YaHei UI", 12))

    def close(self):
        self.closing = True
        self.disconnect()
        if self.worker is None and not self.exporting:
            self.root.destroy()


def usb_self_test():
    """Actual RX/TX workers and GUI, fake CDC only; no device enumeration."""
    from unittest.mock import patch
    from contextlib import contextmanager, redirect_stdout
    from io import StringIO
    import tkinter as tk

    class Port:
        def __init__(self):
            self.lock, self.pending = threading.Lock(), bytearray()
            self.parser, self.seq, self.actions = dm_serial.Parser(), 0, []
            self.cached, self.last_session, self.dropped = {}, None, False
            self.requests = []
            self.closed = False

        @property
        def in_waiting(self):
            return 4096

        def read(self, length):
            time.sleep(.0005)
            with self.lock:
                if not self.pending:
                    self.seq += 1
                    tick = int(time.perf_counter() * 1000) & 0xFFFFFFFF
                    for j in range(1, 6):
                        valid = j != 3  # Legacy J3 has no unsolicited samples.
                        self.pending.extend(usb_monitor.HEADER.pack(
                            b"YL", 1, 5, self.seq, tick, 100, 32, j-1)
                            + usb_monitor.RECORD.pack(j, 2 if j <= 2 else 1, int(j == 3),
                                3 if valid else 0, j, 0x10+j, tick if valid else 0,
                                self.seq if valid else 0, .1 if valid else math.nan,
                                0. if valid else math.nan, 0. if valid else math.nan,
                                0 if valid else 255, 25, 25, 0))
                result = bytes(self.pending[:min(length, 97)])
                del self.pending[:len(result)]
                return result

        def write(self, packet):
            assert not self.closed
            with self.lock:
                for kind, data in self.parser.feed(packet):
                    if kind == dm_serial.SESSION:
                        self.last_session = data
                        self.pending.extend(dm_serial.encode(dm_serial.SESSION_ACK, data))
                        continue
                    assert kind == dm_serial.RELIABLE_REQUEST and data[:8] == self.last_session
                    _, op, scope, field = data[16:20]
                    self.requests.append((op, scope, field))
                    key = data[:12]
                    if key not in self.cached:
                        value = (dm_tuning.VERSION if op == 0 else
                            dm_tuning.MOTION_VERSION if op == 12 and field == 255 else
                            0x03010000 if op == 6 and field == 0 else
                            0xE1010001 if op == 12 and field == 5 else 1)
                        if (op == 12 and field == 8) or op == 7:
                            self.actions.append((op, field))
                        reply = data[:12] + bytes((data[16], 0, scope, field)) + struct.pack("<I", value)
                        self.cached[key] = dm_serial.encode(dm_serial.RELIABLE_RESPONSE, reply)
                        if op == 12 and field == 8 and not self.dropped:
                            self.dropped = True
                            continue  # First action reply lost: identical retry only.
                    self.pending.extend(self.cached[key])
            return len(packet)

    port = Port()
    @contextmanager
    def open_fake(name):
        assert name == "FAKE"
        try:
            yield port
        finally:
            port.closed = True

    samples, events = queue.Queue(maxsize=8192), queue.SimpleQueue()
    reader = USBReader({"port": "FAKE"}, samples, events, time.perf_counter())
    def until(condition):
        deadline = time.perf_counter() + 3
        while not condition():
            assert reader.is_alive() and time.perf_counter() < deadline, reader.last_stats
            time.sleep(.005)

    with patch.object(usb_monitor, "serial_port", open_fake):
        reader.start()
        try:
            until(lambda: reader.tuning.task_status is not None)
            assert reader.tuning.supported and reader.ready.is_set() and not port.actions
            assert set(reader.motors) == {1,2,3,4,5} and reader.counts[3] == 0
            root = tk.Tk()
            root.withdraw()
            with patch.dict(globals(), scan_serial_ports=lambda: [{"port":"FAKE", "label":"fake"}]):
                gui = Monitor(root, can=False, usb=True, port="FAKE")
            gui.worker, gui.samples, gui.events = reader, samples, events
            gui.tick()
            assert len(gui.card_vars) == 5 and gui.control_connected()
            assert str(gui.capture_button["state"]) == "normal"
            gui.quick_command("impedance")
            until(lambda: not reader.tuning.jobs.qsize() and reader.tuning.motion is not None)
            assert port.actions == [(12,8)] and port.dropped
            for action in (2,3):
                gui.quick_command(action)
                until(lambda: (7,action) in port.actions)
            assert port.actions == [(12,8), (7,2), (7,3)]
            # Repeated cache snapshots are not new encoder data.
            reader.stop_requested.set()
            reader.join(2)
            before = samples.qsize()
            reader.accept_snapshot(reader.telemetry, time.perf_counter())
            assert samples.qsize() == before
            gui.worker = None
            root.destroy()
        finally:
            reader.stop_requested.set()
            reader.join(2)
    assert not reader.is_alive() and port.closed
    assert all(item[0] in (0,6,7,12) for item in port.requests)
    assert reader.last_stats["ui_drops"] == 0 and reader.last_stats["invalid"] == 0
    # USB reports protocol capability from MCU metadata, just like CAN:
    # passive legacy is unverified, not a transport failure or an online pass.
    port, output = Port(), StringIO()
    with patch.object(usb_monitor, "serial_port", open_fake), patch.dict(
            globals(), scan_serial_ports=lambda: [{"port": "FAKE"}]), redirect_stdout(output):
        result = probe(100, .5, can=False, port="FAKE", usb=True)
    report = json.loads(output.getvalue())
    assert result == 1 and not report["all_motors_verified"]
    assert report["legacy_passive_motors"] == [3]
    assert report["per_motor_feedback"]["3"]["status"] == "WAITING_COMMAND_REPLY"
    assert all(report["per_motor_feedback"][str(m)]["status"] == "FRESH" for m in (1,2,4,5))
    assert report["errors"] == [] and not port.actions and port.closed
    # One second of target payload, not a USB speed measurement. Keep all 5000
    # unique source records off-screen; repeating a snapshot creates no sample.
    bulk = USBReader({"port":"FAKE"}, queue.Queue(maxsize=8192), queue.SimpleQueue(), time.perf_counter())
    for seq in range(1, 1001):
        data = b"".join(usb_monitor.HEADER.pack(b"YL", 1, 5, seq, seq, 100, 32, j-1)
            + usb_monitor.RECORD.pack(j, 2 if j <= 2 else 1, int(j == 3), 3, j, 0x10+j,
                seq, seq, .1, 0., 0., 0, 25, 25, 0) for j in range(1,6))
        for record in bulk.parser.feed(data):
            snapshot = bulk.collector.add(record, .01)
            if snapshot:
                bulk.accept_snapshot(snapshot, time.perf_counter())
    assert bulk.snapshots == 1000 and bulk.samples.qsize() == 5000 and bulk.ui_drops == 0
    print("USB GUI/RX+TX/5 joints/explicit quick commands/deduplicated retry/cleanup: PASS")


def self_test():
    usb_self_test()
    import tempfile
    dm_serial.self_test()
    tuning_values = dm_tuning.self_test()
    vectors = (
        (0x11, bytes.fromhex("01 65 F1 7F E7 FE 1B 19"), -2.544632639047837),
        (0x12, bytes.fromhex("02 67 0A 7F F7 FF 1C 19"), -2.437438010223545),
    )
    records = []
    for can_id, data, expected in vectors:
        s = decode_sample(can_id, data, (12.5, 10, 28), 0.1, 1_700_000_000)
        assert math.isclose(s.position_rad, expected, abs_tol=1e-12)
        assert s.state == 0 and s.raw == data and s.motor == can_id - 0x10
        records.append(s)
    for motor in MOTORS:
        for register in (None, *LIMIT_REGISTERS):
            if motor in LEGACY_MOTORS:
                try:
                    query_frame(motor, register, 4)
                except ValueError:
                    pass
                else:
                    raise AssertionError("Legacy J3 sent unsupported CC/register query")
                continue
            frame = query_frame(motor, register, 4)
            echo, cid, dlc, ch, data, flags = unpack_frame(frame)
            assert (echo, cid, dlc, ch) == (4, 0x7FF, 8, 0)
            assert data[2] in (0x33, 0xCC) and data[:2] == bytes((motor, 0))
            assert unpack_frame(frame + bytes(4)) == unpack_frame(frame)
    for motor, register in ((0, None), (6, None), (1, 10), (2, 81)):
        try:
            query_frame(motor, register)
        except ValueError:
            pass
        else:
            raise AssertionError("Transmit allowlist bypass")
    for cid, data, ranges in ((0x12, vectors[0][1], (12.5, 10, 28)),
                              (0x11, b"", (12.5, 10, 28)),
                              (0x11, vectors[0][1], (float("nan"), 10, 28))):
        try:
            decode_sample(cid, data, ranges, 0, 0)
        except ValueError:
            pass
        else:
            raise AssertionError("Invalid feedback accepted")
    legacy = decode_sample(0x13, bytes.fromhex("03 84 C6 7F F8 01 1B 1A"),
                           CONFIG_LIMITS[3], 0, 0)
    assert legacy.motor == 3 and legacy.state == 0 and legacy.mos_c == 27
    legacy_collision = bytes((3, 0, 0x33, 21, 0, 0, 27, 26))
    assert decode_sample(0x13, legacy_collision, CONFIG_LIMITS[3], 0, 0).raw == legacy_collision
    for n in (0, 65535):
        data = bytes((1, n >> 8, n & 255, 0, 0, 0, 0, 0))
        assert decode_sample(0x11, data, (12.5, 10, 28), 0, 0).position_rad == (-12.5 if n == 0 else 12.5)
    points = [(i / 1000, 30 if i == 102 else -20 if i == 103 else 0) for i in range(1000)]
    reduced = envelope(points, 20)
    assert max(v for _, v in reduced) == 30 and min(v for _, v in reduced) == -20
    assert reduced == sorted(reduced) and len(reduced) <= 80
    with tempfile.TemporaryDirectory(prefix="dm-monitor-test-") as directory:
        path = Path(directory) / "feedback.csv"
        write_csv(path, records)
        with path.open(encoding="utf-8-sig", newline="") as stream:
            rows = list(csv.reader(stream))
        assert rows[0] == list(CSV_FIELDS) and len(rows) == 3
        assert rows[1][CSV_FIELDS.index("raw_hex")] == vectors[0][1].hex(" ").upper()
    # Exercise the REAL worker against a fake USB endpoint, including failed init.
    from types import SimpleNamespace
    from unittest.mock import patch

    # No host path can send bare enable, disable or MIT motion frames.
    guarded = Reader({}, 100, queue.Queue(), queue.SimpleQueue(), time.perf_counter())
    guarded.ready.set()
    assert not hasattr(guarded, "request_state") and "state_frame" not in globals()
    assert guarded.tuning.request("command", 1)  # Undiscovered firmware cannot start.

    def mit_frame(p=0x7000, v=0x800, kp=40, kd=409, t=0x800):
        return bytes((p >> 8, p & 255, v >> 4, (v & 15) << 4 | kp >> 8,
                      kp & 255, kd >> 4, (kd & 15) << 4 | t >> 8, t & 255))

    wire_command = mit_frame()
    command = decode_mit(wire_command, (12.5, 10, 28))
    assert math.isclose(command["kp"], 40*500/4095)
    assert math.isclose(command["kd"], 409*5/4095)
    assert math.isclose(command["ff"], 28/4095)
    for special in range(0xFB, 0xFF):
        assert decode_mit(b"\xff"*7 + bytes((special,)), (12.5, 10, 28)) is None
    for data in (b"", wire_command[:-1]):
        try:
            decode_mit(data, (12.5, 10, 28))
        except ValueError:
            pass
        else:
            raise AssertionError("Truncated MIT frame accepted")
    guarded.limits = {1: (12.5, 10, 28), 2: (12.5, 10, 28)}
    guarded.tuning.snapshot = {"revision": 1, "values": tuning_values}
    test_now = time.perf_counter()
    guarded.commands[1] = {**command, "raw": wire_command, "at": test_now, "offset": None}
    active_feedback = bytes((0x11,)) + vectors[0][1][1:]
    active = guarded.make_sample(0x11, active_feedback, test_now+.005, 1_700_000_000)
    assert active.source == "CAN" and active.reference_source == "host CAN branch"
    assert active.tick_ms is None and active.sequence is None
    assert active.command_raw == wire_command and math.isclose(active.command_age_ms, 5, abs_tol=1e-5)
    assert math.isclose(active.tracking_error, active.target_reference-active.reference)
    assert guarded.make_sample(0x11, active_feedback, test_now+.2, 0).kp is None
    disabled = guarded.make_sample(0x11, vectors[0][1], test_now+.01, 0)
    assert disabled.kp is None and not guarded.commands
    # Encoder raw branch jumps by one turn; host reference stays continuous.
    def feedback_position(q):
        p = round((q+12.5)/25*65535)
        return bytes((0x11, p >> 8, p & 255, 0x80, 0x08, 0, 27, 25))
    before = guarded.make_sample(0x11, feedback_position(-2.5), test_now, 0)
    after = guarded.make_sample(0x11, feedback_position(-2.5+math.tau), test_now+.005, 0)
    assert abs(before.reference-after.reference) < 25/65535
    register_reply = bytes((1, 0, 0x33, 21)) + struct.pack("<f", 12.5)
    try:
        guarded.make_sample(0x11, register_reply, test_now, 0)
    except ValueError:
        pass
    else:
        raise AssertionError("CAN range reply published as motion feedback")
    with tempfile.TemporaryDirectory(prefix="dm-can-test-") as directory:
        path = Path(directory) / "can.csv"
        write_csv(path, (active, disabled))
        with path.open(encoding="utf-8-sig", newline="") as stream:
            rows = list(csv.DictReader(stream))
        assert rows[0]["data_source"] == "CAN" and rows[0]["mcu_tick_ms"] == ""
        assert rows[0]["command_raw_hex"] == wire_command.hex(" ").upper()
        assert rows[0]["kp"] and rows[1]["kp"] == ""

    class FakeDevice:
        serial_number = "self-test"

        def __init__(self, fail_start=False, drop_first_limit=False,
                     mcu=False, reject_scales=False, invalid_scale=False, active=False):
            self.incoming, self.written, self.modes = queue.Queue(), [], []
            self.fail_start = fail_start
            self.drop_first_limit = drop_first_limit
            self.states = {m: 0 for m in MOTORS}
            self.mcu, self.reject_scales = mcu, reject_scales
            self.invalid_scale = invalid_scale
            self.drop_status = False
            self.active = active

        def ctrl_transfer(self, direction, request, value, index, data, timeout=0):
            if direction == 0xC1:
                return struct.pack("<10I", 8, 48_000_000, 1, 16, 1, 8, 4, 1, 1024, 1)
            if request == 2:
                self.modes.append(struct.unpack("<II", data))
                if self.fail_start and self.modes[-1][0] == 1:
                    raise TimeoutError("Injected START timeout")
            return len(data)

        def write(self, endpoint, packet, timeout=0):
            assert endpoint == 2
            _, cid, dlc, ch, payload, flags = unpack_frame(packet)
            assert dlc == 8 and ch == 0
            self.written.append(packet)
            if cid == 0x7FF and payload[2] == 0x33:
                if self.drop_first_limit:
                    self.drop_first_limit = False
                    return len(packet)  # Inject one read-only startup loss.
                motor = payload[0]
                response = payload[:4] + struct.pack("<f", CONFIG_LIMITS[motor][payload[3] - 21])
            elif cid == dm_tuning.REQUEST_ID:
                seq, op, scope, field = payload[:4]
                if not self.mcu:
                    assert op == 0  # Legacy board ignores MCU discovery.
                    return len(packet)
                assert op in (0, 1, 6, 8, 11, 12), "Automatic motor/config action"
                assert op != 12 or scope == 0 and field in (0, 255), "Automatic Motion action"
                if self.drop_status and op == 6:
                    return len(packet)
                result, output, floating = 0, 0, False
                if op == 0:
                    output = dm_tuning.VERSION
                elif op == 1:
                    floating = True
                    if field in (240, 241, 242):
                        result = 1 if self.reject_scales else 0
                        output = float("nan") if self.invalid_scale else {
                            240: 12.5, 241: 10, 242: 28}[field]
                    else:
                        output = tuning_values[f"{scope}.{field}"]
                elif op == 6:
                    output = 1 if field else 0x03010000
                elif op == 8:
                    if field == 255:
                        output = dm_tuning.CALIBRATION_VERSION
                    elif not scope and not field:
                        output = 0xC1010008
                    else:
                        output, floating = 0, True
                elif op == 11:
                    if field == 255:
                        output = dm_tuning.DRAG_VERSION
                    elif not scope and not field:
                        output = 0xD1000001
                    else:
                        output, floating = 0, True
                elif op == 12:
                    output = dm_tuning.MOTION_VERSION if field == 255 else 0
                response = bytes((seq, result, scope, field)) + struct.pack(
                    "<f" if floating else "<I", output)
                self.incoming.put(FRAME.pack(
                    0xFFFFFFFF, dm_tuning.RESPONSE_ID, 8, 0, 0, 0, response))
                for motor, (master, feedback, _) in enumerate(vectors, 1):
                    if self.active:
                        self.incoming.put(FRAME.pack(
                            0xFFFFFFFF, motor, 8, 0, 0, 0, wire_command))
                        feedback = bytes((0x10 | motor,)) + feedback[1:]
                    self.incoming.put(FRAME.pack(
                        0xFFFFFFFF, master, 8, 0, 0, 0, feedback))
                return len(packet)
            else:
                assert cid == 0x7FF and payload[2] == 0xCC
                motor = payload[0]
                response = bytes(((self.states[motor] << 4) | motor,)) + (
                    vectors[motor - 1][1][1:] if motor <= 2 else
                    bytes.fromhex("7F FF 7F F7 FF 1B 19"))
            self.incoming.put(FRAME.pack(0xFFFFFFFF, MOTORS[motor], 8, 0, 0, 0, response))
            return len(packet)

        def read(self, endpoint, size, timeout=0):
            assert endpoint == 0x81
            try:
                return self.incoming.get(timeout=timeout / 1000)
            except queue.Empty as exc:
                raise TimeoutError from exc

    for fail_start, drop_first in ((False, False), (True, False), (False, True)):
        device = FakeDevice(fail_start, drop_first)
        released = []
        core = SimpleNamespace(find=lambda **kwargs: device, USBTimeoutError=TimeoutError)
        util = SimpleNamespace(claim_interface=lambda *args: None,
                               release_interface=lambda *args: released.append("release"),
                               dispose_resources=lambda *args: released.append("dispose"))
        events, samples = queue.SimpleQueue(), queue.Queue(maxsize=1000)
        adapter = dict(vendor=0x1D50, product=0x606F, bus=1, address=1, serial="self-test")
        with patch.dict(globals(), usb_modules=lambda: (core, util, None)):
            reader = Reader(adapter, 100, samples, events, time.perf_counter(), direct=True)
            reader.start()
            deadline = time.perf_counter() + 2
            while reader.is_alive() and min(reader.counts.values()) < 3 and time.perf_counter() < deadline:
                time.sleep(0.01)
            assert all(unpack_frame(packet)[1] in (0x7FF, dm_tuning.REQUEST_ID)
                       for packet in device.written), "Automatic motor command"
            reader.paused.set()
            time.sleep(0.04)
            before = len(device.written)
            time.sleep(0.04)
            assert len(device.written) == before, "Queries continued while paused"
            reader.stop_requested.set()
            reader.join(2)
            assert not reader.is_alive() and device.modes[-1] == (0, 0)
            assert released == ["release", "dispose"]
            if fail_start:
                assert not device.written
            else:
                assert min(reader.counts.values()) >= 3
                expected_reads = 7 if drop_first else 6
                assert sum(unpack_frame(p)[1] == 0x7FF and unpack_frame(p)[4][2] == 0x33
                           for p in device.written) == expected_reads

    # Shared-bus startup, steady state, missing extension and offline MCU must
    # NEVER produce 0x7FF, including the former six startup range reads.
    try:
        guarded.send_query(1)
    except RuntimeError:
        pass
    else:
        raise AssertionError("Shared-bus direct-query guard bypassed")
    for mcu, reject, invalid in ((True, False, False), (True, True, False),
                                 (True, False, True), (False, False, False)):
        device = FakeDevice(mcu=mcu, reject_scales=reject, invalid_scale=invalid, active=True)
        released = []
        core = SimpleNamespace(find=lambda **kwargs: device, USBTimeoutError=TimeoutError)
        util = SimpleNamespace(claim_interface=lambda *args: None,
                               release_interface=lambda *args: released.append("release"),
                               dispose_resources=lambda *args: released.append("dispose"))
        events, samples = queue.SimpleQueue(), queue.Queue(maxsize=1000)
        with patch.dict(globals(), usb_modules=lambda: (core, util, None)):
            reader = Reader(adapter, 100, samples, events, time.perf_counter())
            reader.start()
            deadline = time.perf_counter() + 3
            while (reader.is_alive() and min(reader.counts.values()) < 3
                   and time.perf_counter() < deadline):
                time.sleep(0.01)
            if mcu and not reject and not invalid:
                assert reader.ready.is_set() and min(reader.counts.values()) >= 3
                assert reader.limits == {1: (12.5, 10, 28), 2: (12.5, 10, 28)}
                assert any(sample.kp is not None and sample.command_raw == wire_command
                           for sample in samples.queue), "CAN MIT inputs missing from feedback records"
                # More than one UART period's samples must survive; no 50Hz decimation.
                before_count = reader.counts[1]
                for _ in range(60):
                    device.incoming.put(FRAME.pack(
                        0xFFFFFFFF, 1, 8, 0, 0, 0, wire_command))
                    device.incoming.put(FRAME.pack(
                        0xFFFFFFFF, 0x11, 8, 0, 0, 0, active_feedback))
                deadline = time.perf_counter() + 1
                while reader.counts[1] < before_count + 60 and time.perf_counter() < deadline:
                    time.sleep(.005)
                assert reader.counts[1] >= before_count + 60
                device.drop_status = True
                deadline = time.perf_counter() + 2
                while reader.tuning.online and time.perf_counter() < deadline:
                    time.sleep(0.01)
                assert not reader.tuning.online and reader.tuning.supported
                assert reader.is_alive(), "Offline MCU caused an automatic reconnect"
            else:
                assert not reader.ready.is_set() and not reader.is_alive()
            reader.stop_requested.set()
            reader.join(2)
            assert not reader.is_alive() and released == ["release", "dispose"]
            assert device.written and all(unpack_frame(p)[1] == dm_tuning.REQUEST_ID
                                          for p in device.written)
            assert reader.tx_usb == 0, "PC took over MCU motor queries"

    # CAN1 is entirely passive: no RPC, no 0x7FF, even with a tuning snapshot
    # from the old two-axis editor. Filter the other physical bus's identities.
    device = FakeDevice()
    core = SimpleNamespace(find=lambda **kwargs: device, USBTimeoutError=TimeoutError)
    released = []
    util = SimpleNamespace(claim_interface=lambda *args: None,
                           release_interface=lambda *args: released.append("release"),
                           dispose_resources=lambda *args: released.append("dispose"))
    samples, events = queue.Queue(), queue.SimpleQueue()
    with patch.dict(globals(), usb_modules=lambda: (core, util, None)):
        reader = Reader(adapter, 100, samples, events, time.perf_counter(), can_bus=1)
        reader.tuning.snapshot = {"revision": 1, "values": tuning_values}
        reader.start()
        try:
            assert reader.ready.wait(1)
            for motor, master in MOTORS.items():
                feedback = bytes((motor, 255, 255, 255, 255, 255, 27, 25))
                device.incoming.put(FRAME.pack(0xFFFFFFFF, master, 8, 0, 0, 0, feedback))
            # A correct feedback ID with the wrong low-nibble motor is invalid.
            device.incoming.put(FRAME.pack(
                0xFFFFFFFF, 0x13, 8, 0, 0, 0, bytes((4, 255, 255, 255, 255, 255, 27, 25))))
            deadline = time.perf_counter() + 1
            while min(reader.counts.values()) < 1 or reader.invalid < 1:
                assert reader.is_alive() and time.perf_counter() < deadline
                time.sleep(.005)
            assert reader.counts == {3: 1, 4: 1, 5: 1}
            assert reader.other == 2 and reader.invalid == 1
            assert reader.limits == {m: (12.5, 30, 10) for m in (3, 4, 5)}
            can1_records = tuple(samples.queue)
            assert {s.motor for s in can1_records} == {3, 4, 5}
            for s in can1_records:
                assert (s.position_rad, s.velocity_rad_s, s.torque_nm) == (12.5, 30, 10)
                assert s.can_bus == 1 and "NOT read back" in s.limits_source
                assert s.reference is None and s.kp is None
            assert reader.tuning.request("command", 1)
            for send in (lambda: reader.send_query(3), lambda: reader.send_tuning(bytes(8))):
                try:
                    send()
                except RuntimeError:
                    pass
                else:
                    raise AssertionError("CAN1 passive mode sent a request")
            assert not device.written, "CAN1 passive mode transmitted CAN traffic"
        finally:
            reader.stop_requested.set()
            reader.join(2)
        assert not reader.is_alive() and released == ["release", "dispose"]
        assert not device.written
    with tempfile.TemporaryDirectory(prefix="dm-can1-test-") as directory:
        path = Path(directory) / "can1.csv"
        write_csv(path, (*can1_records, active))
        with path.open(encoding="utf-8-sig", newline="") as stream:
            rows = list(csv.DictReader(stream))
        assert {r["can_bus"] for r in rows} == {"CAN1", "CAN2"}
        assert rows[0]["limits_source"] == "local config; NOT read back or verified"
        assert rows[0]["motor_can_id"] == "0x03" and rows[0]["feedback_can_id"] == "0x13"
    device = FakeDevice()
    with patch.dict(globals(), usb_modules=lambda: (core, util, None)):
        standalone = Reader(adapter, 100, queue.Queue(), queue.SimpleQueue(),
                            time.perf_counter(), direct=True, can_bus=1)
        standalone.start()
        try:
            assert standalone.ready.wait(1)
            device.incoming.put(FRAME.pack(
                0xFFFFFFFF, 0x13, 8, 0, 0, 0, bytes.fromhex("03 84 C6 7F F8 01 1B 1A")))
            deadline = time.perf_counter() + 2
            while standalone.counts[3] != 1 or min(standalone.counts[m] for m in (4, 5)) < 3:
                assert standalone.is_alive() and time.perf_counter() < deadline
                time.sleep(.005)
            assert standalone.limits == {m: (12.5, 30, 10) for m in (3, 4, 5)}
            assert "J3 local config" in standalone.limits_source
            assert "NOT read back" in next(s for s in list(standalone.samples.queue)
                                           if s.motor == 3).limits_source
        finally:
            standalone.stop_requested.set()
            standalone.join(2)
        assert not standalone.is_alive()
        assert len([p for p in device.written if unpack_frame(p)[4][2] == 0x33]) == 6
        assert all(unpack_frame(p)[1] == 0x7FF and unpack_frame(p)[4][0] in (4, 5)
                   for p in device.written)
    # Passive legacy J3 has no response here: never claim all three passed.
    from contextlib import redirect_stdout
    from io import StringIO
    device, output = FakeDevice(), StringIO()
    with patch.dict(globals(), usb_modules=lambda: (core, util, None),
                    scan_adapters=lambda: [adapter]), redirect_stdout(output):
        result = probe(100, .5, direct=True, can_bus=1)
    report = json.loads(output.getvalue())
    assert result == 1 and set(report["latest"]) == {"4", "5"}
    assert report["selected_can_bus"] == "CAN1" and "J3 local config" in report["limits_source"]
    assert set(report["readback_limits"]) == {"4", "5"}
    # Keep the small host mirror honest against the production five-motor table.
    import re
    config = (Path(__file__).resolve().parents[1] / "config" / "motor_config.c").read_text(encoding="utf-8")
    devices = re.findall(
        r"\.model\s*=\s*DM_MOTOR_DM(\d+),\s*\.bus\s*=\s*&hfdcan(\d),\s*\\?\s*"
        r"\.can_id\s*=\s*(0x[\dA-Fa-f]+)U,\s*\.master_id\s*=\s*(0x[\dA-Fa-f]+)U,\s*\\?\s*"
        r"\.pmax\s*=\s*([\d.]+)f,\s*\.vmax\s*=\s*([\d.]+)f,\s*\.tmax\s*=\s*([\d.]+)f", config)
    assert len(devices) == len(MOTORS)
    for motor, (model, bus, command_id, feedback_id, *limits) in enumerate(devices, 1):
        assert model == MOTOR_MODELS[motor].removesuffix("P")
        assert int(bus) == MOTOR_BUSES[motor]
        assert int(command_id, 16) == motor and int(feedback_id, 16) == MOTORS[motor]
        assert tuple(map(float, limits)) == CONFIG_LIMITS[motor]
    assert config.count(".protocol = DM_MOTOR_PROTOCOL_LEGACY_MIT") == 1
    assert LEGACY_MOTORS == {3}

    # Exercise the actual UART worker/RPC, without opening COM or USB-CAN.
    class FakeSerial:
        def __init__(self, fail_write=False, silent=False, joint_count=2):
            self.mcu = FakeDevice(mcu=True)
            self.incoming, self.buffer = queue.Queue(), bytearray()
            self.parser = dm_serial.Parser()
            self.written, self.closed = [], False
            self.start = time.perf_counter()
            self.next_frame = self.start
            self.sequence = self.count = 0
            self.freeze = self.bo = False
            self.fail_write, self.silent = fail_write, silent
            self.session, self.rpc_cache = None, {}
            self.joint_count = joint_count

        @property
        def in_waiting(self):
            return len(self.buffer)

        def write(self, data):
            if self.fail_write:
                raise OSError("Injected UART write failure")
            self.written.append(bytes(data))
            for kind, payload in self.parser.feed(data):
                if kind == dm_serial.SESSION:
                    if self.session != payload:
                        self.session, self.rpc_cache = payload, {}
                    self.incoming.put(dm_serial.encode(dm_serial.SESSION_ACK, payload))
                    continue
                assert kind == dm_serial.RELIABLE_REQUEST and payload[:8] == self.session
                key = payload[:12]
                if key in self.rpc_cache:
                    self.incoming.put(self.rpc_cache[key])
                    continue
                self.mcu.write(2, FRAME.pack(
                    0, dm_tuning.REQUEST_ID, 8, 0, 0, 0, payload[16:]))
                while not self.mcu.incoming.empty():
                    _, cid, _, _, answer, _ = unpack_frame(self.mcu.incoming.get_nowait())
                    if cid == dm_tuning.RESPONSE_ID:
                        self.rpc_cache[key] = dm_serial.encode(
                            dm_serial.RELIABLE_RESPONSE, key + answer)
                        self.incoming.put(self.rpc_cache[key])
            return len(data)

        def read(self, size):
            if not self.buffer:
                now = time.perf_counter()
                if not self.silent and now >= self.next_frame:
                    self.next_frame = now + 0.02
                    self.sequence += 1
                    tick = 1000 + int((now - self.start) * 1000)
                    if not self.freeze:
                        self.count += 1
                        self.last_feedback = tick
                    payload = bytearray(dm_serial.HEADER_SIZE + dm_serial.RECORD_SIZE*self.joint_count)
                    struct.pack_into("<II", payload, 0, tick, self.sequence)
                    payload[8:15] = bytes((5 if self.bo else 0, 10 if self.bo else 0, 1,
                                          self.joint_count, 0, 0, dm_serial.RECORD_SIZE))
                    for motor in range(self.joint_count):
                        at = dm_serial.HEADER_SIZE + dm_serial.RECORD_SIZE*motor
                        struct.pack_into("<BBHHBB", payload, at,
                                         motor+1, 1, motor+1, 0x11+motor, 0, 0)
                        struct.pack_into("<IIfff4B", payload, at+8,
                                         self.last_feedback, self.count, -2.6, 0, 0, 1, 25, 26, 0)
                        struct.pack_into("<8f", payload, at+32,
                                         -2.6, -2.5, .125, 0, .25, .1, -2.5, .1)
                    if self.bo:
                        payload[20], payload[32], payload[33] = 10, 5, 248
                    self.buffer.extend(dm_serial.encode(dm_serial.TELEMETRY, payload, version=2))
                else:
                    try:
                        self.buffer.extend(self.incoming.get(timeout=0.005))
                    except queue.Empty:
                        return b""
            result = bytes(self.buffer[:size])
            del self.buffer[:size]
            return result

        def close(self):
            self.closed = True

    serial_device = FakeSerial()
    serial_events, serial_samples = queue.SimpleQueue(), queue.Queue(maxsize=1000)
    with patch.dict(globals(), open_serial=lambda port: serial_device,
                    usb_modules=lambda: (_ for _ in ()).throw(AssertionError("UART opened CAN"))):
        serial_reader = SerialReader(dict(port="FAKE"), serial_samples, serial_events,
                                     time.perf_counter())
        serial_reader.start()
        try:
            deadline = time.perf_counter() + 3
            while (not serial_reader.ready.is_set() or min(serial_reader.counts.values()) < 3
                   or serial_reader.telemetry is None):
                assert serial_reader.is_alive() and time.perf_counter() < deadline
                time.sleep(0.01)
            assert serial_reader.limits == {1: (12.5, 10, 28), 2: (12.5, 10, 28)}
            serial_device.freeze = serial_device.bo = True
            time.sleep(0.08)  # Drain the last in-flight feedback.
            before = serial_reader.counts.copy()
            last_elapsed = serial_samples.queue[-1].elapsed_s
            sequence = serial_reader.telemetry["sequence"]
            time.sleep(0.6)
            assert serial_reader.counts == before, "UART refreshed frozen CAN feedback"
            assert serial_reader.telemetry["sequence"] > sequence
            assert serial_reader.telemetry["can_flags"] & 2
            assert serial_reader.telemetry["joints"][0]["age_ms"] > 500
            assert time.perf_counter() - serial_reader.origin - last_elapsed > 0.5
            assert serial_reader.tuning.online, "Bus-Off broke independent UART RPC"
            serial_device.mcu.drop_status = True
            deadline = time.perf_counter() + 3
            while serial_reader.tuning.online and time.perf_counter() < deadline:
                time.sleep(0.01)
            assert not serial_reader.tuning.online and serial_reader.is_alive()
            assert all(sample.raw == b"" for sample in serial_samples.queue)
        finally:
            serial_reader.stop_requested.set()
            serial_reader.join(2)
        assert not serial_reader.is_alive() and serial_device.closed
        assert serial_device.written and all(
            dm_serial.Parser().feed(p)[0][0] in (dm_serial.SESSION, dm_serial.RELIABLE_REQUEST)
            for p in serial_device.written)
    multi_status = {}
    for joint_count in (3, 6):
        serial_device = FakeSerial(joint_count=joint_count)
        samples = queue.Queue()
        with patch.dict(globals(), open_serial=lambda port: serial_device):
            reader = SerialReader(dict(port="FAKE"), samples, queue.SimpleQueue(),
                                  time.perf_counter())
            reader.start()
            try:
                deadline = time.perf_counter()+2
                while not reader.ready.is_set() or len(reader.counts) != joint_count:
                    assert reader.is_alive() and time.perf_counter() < deadline
                    time.sleep(.01)
                multi_status[joint_count] = reader.telemetry
                assert set(reader.counts) == set(range(1, joint_count+1))
                assert not reader.tuning.supported
            finally:
                reader.stop_requested.set()
                reader.join(2)
            assert not reader.is_alive() and serial_device.closed
            assert all(dm_serial.Parser().feed(p)[0][0] == dm_serial.SESSION
                       for p in serial_device.written)
        with tempfile.TemporaryDirectory(prefix="dm-multi-test-") as directory:
            path = Path(directory)/"feedback.csv"
            write_csv(path, tuple(samples.queue))
            with path.open(encoding="utf-8-sig", newline="") as stream:
                rows = list(csv.DictReader(stream))
            assert {int(r["logical_joint"]) for r in rows} == set(range(1, joint_count+1))
            last = next(r for r in rows if int(r["logical_joint"]) == joint_count)
            assert last["feedback_can_id"] == f"0x{0x10+joint_count:02X}"
            assert float(last["target_velocity_rad_s"]) == .125 and last["mcu_tick_ms"]
    for fail_write, silent in ((True, False), (False, True)):
        serial_device = FakeSerial(fail_write=fail_write, silent=silent)
        with patch.dict(globals(), open_serial=lambda port: serial_device):
            reader = SerialReader(dict(port="FAKE"), queue.Queue(), queue.SimpleQueue(),
                                  time.perf_counter())
            reader.start()
            reader.join(4)
            assert not reader.is_alive() and serial_device.closed
            assert not reader.ready.is_set()
    with patch.dict(globals(), open_serial=lambda port: (_ for _ in ()).throw(OSError("port busy"))):
        reader = SerialReader(dict(port="FAKE"), queue.Queue(), queue.SimpleQueue(),
                              time.perf_counter())
        reader.start()
        reader.join(1)
        assert not reader.is_alive() and not reader.ready.is_set()

    # RPC bursts wait for an RX telemetry slot instead of starving the stream.
    reader = SerialReader({}, queue.Queue(), queue.SimpleQueue(), time.perf_counter())
    reader.telemetry, reader.telemetry_at = {"tick_ms": 10}, 99.
    def heartbeat_slot(_):
        reader.telemetry, reader.telemetry_at = {"tick_ms": 20}, 100.
        return False
    with patch("dm_monitor.time.perf_counter", return_value=100.), \
         patch.object(reader.stop_requested, "wait", side_effect=heartbeat_slot), \
         patch.object(reader.reliable, "request", return_value=bytes(8)) as request:
        reader.send_tuning(bytes(8))
        request.assert_called_once_with(bytes(8), 20)
    reader.telemetry_at = 99.
    with patch("dm_monitor.time.perf_counter", side_effect=(100., 100., 100.3)), \
         patch.object(reader.reliable, "request") as request:
        try:
            reader.send_tuning(bytes(8))
        except RuntimeError:
            pass
        else:
            raise AssertionError("Silent MCU accepted a new RPC")
        request.assert_not_called()

    # MCU reboot is not normal tick rollover, and never triggers reconnect.
    reader = SerialReader({}, queue.Queue(), queue.SimpleQueue(), time.perf_counter())
    packet = bytearray(144)
    struct.pack_into("<II", packet, 0, 0xFFFFFFF0, 1)
    reader.accept_telemetry(packet)
    struct.pack_into("<II", packet, 0, 10, 2)
    reader.accept_telemetry(packet)
    heartbeat_at = reader.telemetry_at
    reader.accept_telemetry(packet)
    assert reader.telemetry_at == heartbeat_at, "Duplicate frame refreshed MCU heartbeat"
    struct.pack_into("<II", packet, 0, 5, 3)
    try:
        reader.accept_telemetry(packet)
    except RuntimeError:
        pass
    else:
        raise AssertionError("MCU reset accepted as continuous telemetry")

    # Construct the actual GUI offline; exercise bound shortcuts and repeat suppression.
    import tkinter as tk
    root = tk.Tk()
    root.withdraw()
    with patch.dict(globals(), scan_serial_ports=lambda: [], scan_adapters=lambda: []):
        ui = Monitor(root, can=False)
    for joint_count, status in multi_status.items():
        ui.events.put(("serial_status", (status, time.perf_counter())))
        ui.handle_events()
        assert set(ui.card_vars) == set(range(1, joint_count+1))
        assert len(ui.cards.winfo_children()) == joint_count
        ui.draw_plots()  # All colors and legends must work, not only J1/J2.
    ui.configure_joints(tuple((m, m, feedback) for m, feedback in MOTORS.items()))
    for key in ("F1", "F2", "F3", "F4"):
        assert not root.bind(f"<KeyPress-{key}>")
    ui.worker = guarded
    guarded.tuning.supported = guarded.tuning.online = True
    ui.events.put(("connected", None))
    ui.handle_events()
    assert ui.tuning_panel is None, "Advanced settings opened by default"
    assert not hasattr(ui, "pause_button") and not hasattr(ui, "rate_box")
    assert not ui.show_details.get()
    for key, action in (("F5", 2), ("F6", 3)):
        assert root.bind(f"<KeyPress-{key}>") and root.bind(f"<KeyRelease-{key}>")
        event = SimpleNamespace(keysym=key)
        ui.hotkey(event)
        ui.hotkey(event)
        ui.release_hotkey(event)
        assert guarded.tuning.jobs.get_nowait()[:2] == ("command", action)
        assert guarded.tuning.jobs.empty()
    ui.worker = None
    ui.open_tuning()
    panel = ui.tuning_panel
    panel.window.withdraw()
    assert len(panel.variables) == 38 and panel.snapshot is None
    panel.fill({"revision": 1, "values": tuning_values})
    assert not panel.dirty and not panel.allow_motion.get()
    assert set(dm_tuning.PROTECTIONS) == {(2, "跟随"), (4, "过温")}
    for joint, flags in panel.flags.items():
        assert len(flags) == 2 and all(flag.get() for flag in flags)
        for mask in (0, 2, 4, 6):
            panel.fill({"revision": 1, "values": {**tuning_values, f"{joint}.9": mask}})
            assert [flag.get() for flag in flags] == [bool(mask & 2), bool(mask & 4)]
    panel.fill({"revision": 1, "values": tuning_values})
    with tempfile.TemporaryDirectory(prefix="dm-reference-test-") as directory:
        path = Path(directory) / "profile.json"
        with patch("tkinter.filedialog.asksaveasfilename", return_value=str(path)):
            panel.save()
        saved = json.loads(path.read_text(encoding="utf-8"))
        assert saved["schema"] == 3 and saved["values"]["0.5"] == 1
        panel.variables["0.5"].set("0")
        with patch("tkinter.filedialog.askopenfilename", return_value=str(path)):
            panel.load()
        assert panel.values()["0.5"] == 1
        assert panel.dirty and not panel.allow_motion.get()
        assert guarded.tuning.jobs.empty(), "Loading a profile submitted a motor action"
    panel.fill({"revision": 1, "values": tuning_values})
    panel.variables["1.3"].set("20")
    assert panel.dirty and panel.readbacks["1.3"].get() == "5"
    assert panel.values()["1.3"] == 20
    panel.event("tuning_error", "Injected timeout")
    assert panel.snapshot is None and not panel.allow_motion.get()
    panel.fill({"revision": 1, "values": tuning_values})
    panel.event("task_status", (0, 0, 1, 3, 2))
    assert panel.snapshot is None, "MCU parameter revision change did not invalidate UI"
    ui.worker = guarded
    guarded.stop_requested.clear()
    guarded.tuning.supported = guarded.tuning.calibration_supported = guarded.tuning.online = True
    panel.fill({"revision": 1, "values": tuning_values})
    panel.event("task_status", (0, 0, 1, 3, 1))
    cal = {"flags": 31, "axis": 0, "valid_mask": 1, "age_ms": 2, "stable_ms": 2100,
           "accel": [0, 0, 9.80665], "gyro": [0, 0, 0],
           "joints": [[-105, -100, 5, -100],
                      [float("nan"), 210, float("nan"), -150]],
           "at": time.perf_counter(), "tolerance_deg": 0.5}
    panel.event("calibration_status", cal)
    panel.update_buttons()
    assert "原始 210.0000°  参考 -150.0000°" in panel.cal_values.get()
    assert str(panel.cal_j2["state"]) == "disabled", "Unknown IMU axis accepted"
    panel.event("calibration_status", {**cal, "valid_mask": 3, "flags": 8})
    panel.update_buttons()
    assert "人工朝上参考" in panel.cal_text.get()
    assert "轴向未复核" in panel.cal_text.get()
    panel.event("calibration_status", {**cal, "valid_mask": 3, "flags": 0})
    panel.update_buttons()
    assert str(panel.imu_start["state"]) == "normal", "Read-only IMU incorrectly tied to CAN"
    assert str(panel.cal_j2["state"]) == "disabled", "Zero capture allowed without motor feedback"
    panel.event("calibration_status", cal)
    panel.cal_axis.set("+Z")
    panel.cal_confirm.set(True)
    panel.update_buttons()
    assert str(panel.cal_j2["state"]) == "normal"
    drag = {"valid": True, "phase": 4, "inertia": [.001, 0, .007],
            "joints": [[210, 0, 0, .2, 0, 0, .2, 5, 5, 210, 0, 0, -150, -360]] * 2,
            "at": time.perf_counter()}
    panel.event("drag_status", drag)
    panel.update_buttons()
    assert "参考 -150.000°" in panel.drag_text.get()
    assert "换圈偏移 -360°" in panel.drag_text.get()
    assert "Roll：+0.000°" in panel.cal_attitude.get()
    assert "Pitch：+0.000°" in panel.cal_attitude.get()
    assert "竖直偏差：+0.000°" in panel.cal_attitude.get()
    panel.event("calibration_status", {**cal, "flags": 8})
    panel.update_buttons()
    assert "0.000" not in panel.cal_attitude.get(), "Sleeping IMU shown as live attitude"
    assert "9.8066" not in panel.cal_values.get(), "Sleeping IMU shown as live acceleration"
    panel.event("calibration_status", {**cal, "at": time.perf_counter() - 2})
    panel.update_buttons()
    assert "过期" in panel.cal_attitude.get() and "过期" in panel.cal_values.get()
    assert str(panel.cal_j2["state"]) == "disabled", "Expired attitude accepted"
    panel.event("calibration_status", {**cal, "flags": 16})
    panel.update_buttons()
    assert str(panel.cal_j2["state"]) == "disabled", "Unstable IMU accepted"
    panel.event("calibration_error", "Injected unknown result")
    assert panel.calibration is None and not panel.cal_confirm.get()
    # One explicit impedance click, no checkbox/Mode/KEY/eight-value sequence.
    guarded.tuning.motion_supported = True
    guarded.tuning.capture_supported = True
    panel.fill({"revision": 1, "values": tuning_values})
    panel.event("task_status", (4, 0, 1, 3, 1))
    panel.event("motion_status", {"mode": 1, "valid": True, "phase": 4,
                                "elapsed": 1, "duration": 1, "model": [[.1, .02, .03, .4]] * 2,
                                "at": time.perf_counter()})
    saved_latest = ui.latest
    ui.latest = {j: SimpleNamespace(
        mode=1, reference=-2.6, target_reference=-2.5, tracking_error=.1,
        elapsed_s=time.perf_counter() - ui.origin) for j in (1, 2)}
    panel.update_buttons()
    assert "目标 -143.239°" in panel.motion_text.get()
    assert "实际 -148.969°" in panel.motion_text.get()
    assert "误差 +5.730°" in panel.motion_text.get()
    assert str(panel.hold_button["state"]) == "normal"
    ui.latest = saved_latest
    while not guarded.tuning.jobs.empty():
        guarded.tuning.jobs.get_nowait()
    assert not panel.allow_motion.get()
    panel.hold_button.invoke()
    kind, payload, _ = guarded.tuning.jobs.get_nowait()
    assert kind == "impedance" and payload is None
    assert guarded.tuning.jobs.empty(), "Capture also queued a separate enable"
    panel.busy = False
    panel.event("motion_status", {**panel.motion, "at": time.perf_counter() - 2})
    panel.update_buttons()
    assert str(panel.hold_button["state"]) == "normal", "Capture depends on fresh Task, not old target"
    assert "过期" in panel.motion_text.get()
    panel.status_at = time.perf_counter() - 2
    panel.update_buttons()
    assert str(panel.hold_button["state"]) == "disabled", "Stale Task allowed capture"
    panel.event("task_status", (0, 0, 1, 3, 1))
    panel.update_buttons()
    assert all(str(b["state"]) == "normal" for b in panel.mode_buttons)
    panel.mode_buttons[0].invoke()
    kind, payload, _ = guarded.tuning.jobs.get_nowait()
    assert kind == "mode" and payload == 0
    assert guarded.tuning.jobs.empty(), "Mode selection implicitly enabled"
    guarded.tuning.capture_supported = False
    panel.busy = False
    panel.update_buttons()
    assert str(panel.hold_button["state"]) == "disabled", "Old firmware allowed unsupported action"
    ui.worker = None
    panel.window.deiconify()
    panel.window.geometry("1080x700")
    root.update()
    assert panel.disable_button.winfo_ismapped()
    assert (panel.disable_button.winfo_rooty() + panel.disable_button.winfo_height()
            < panel.window.winfo_rooty() + panel.window.winfo_height()), "Motion controls clipped"
    for after_id in root.tk.splitlist(root.tk.call("after", "info")):
        # Cancel timer events only; destroy() deletes each widget's own Tcl
        # commands. root.after_cancel() would double-delete Toplevel callbacks.
        root.tk.call("after", "cancel", after_id)
    root.destroy()
    can_root = tk.Tk()
    can_root.withdraw()
    can_worker = SimpleNamespace(tuning=guarded.tuning, start=lambda: None,
                                 ready=threading.Event(), stop_requested=threading.Event())
    with patch.dict(globals(), scan_adapters=lambda: [dict(adapter, label="offline CAN")],
                    scan_serial_ports=lambda: (_ for _ in ()).throw(AssertionError("CAN scanned UART")),
                    Reader=lambda *args, **kwargs: can_worker):
        can_ui = Monitor(can_root)
        assert can_ui.is_can and can_ui.can_bus == 2
        assert set(can_ui.card_vars) == set(MOTORS) and len(can_ui.cards.winfo_children()) == 5
        assert can_ui.tuning_panel is None and not can_ui.diagnostics.winfo_manager()
        can_ui.connect()
        assert can_ui.worker is can_worker
        assert str(can_ui.bus_box["state"]) == "disabled"
        can_ui.samples.put(active)
        can_ui.tick()
        assert can_ui.latest[1].command_raw == wire_command
        assert "CAN MIT" in can_ui.card_vars[1]["control"].get()
        assert "CAN1" in can_ui.card_vars[3]["position"].get()
        can_worker.ready.set()
        can_worker.tuning.supported = can_worker.tuning.online = True
        can_worker.tuning.capture_supported = True
        can_worker.tuning.task_status = (0, 0, 1, 3, 1)
        can_worker.tuning.task_status_at = time.perf_counter()
        can_ui.update_quick_buttons()
        assert str(can_ui.capture_button["state"]) == "normal"
        can_ui.capture_button.invoke()
        assert guarded.tuning.jobs.get_nowait()[:2] == ("impedance", None)
        can_ui.capture_button.invoke()
        assert guarded.tuning.jobs.empty(), "Repeated capture bypassed pending-action gate"
        can_ui.hold_button.invoke()
        assert guarded.tuning.jobs.get_nowait()[:2] == ("command", 2)
        can_ui.events.put(("task_ack", 2))
        can_ui.handle_events()
        assert can_ui.quick_busy, "Stop ACK incorrectly acknowledged pending capture"
        can_ui.release_button.invoke()
        assert guarded.tuning.jobs.get_nowait()[:2] == ("command", 3)
        can_ui.events.put(("motion_ack", "impedance"))
        can_ui.handle_events()
        assert not can_ui.quick_busy
        can_worker.tuning.task_status_at -= 2
        can_ui.update_quick_buttons()
        assert str(can_ui.capture_button["state"]) == "disabled"
        assert str(can_ui.release_button["state"]) == "normal", "Stale MCU blocked stop/disable"
        can_ui.worker = None
        can_ui.bus_var.set(BUS_LABELS[1])
        can_ui.select_bus()
        can_ui.tick()
        assert can_ui.can_bus == 1 and not can_ui.latest
        assert str(can_ui.tuning_button["state"]) == "disabled"
        assert all(str(b["state"]) == "disabled" for b in (
            can_ui.capture_button, can_ui.hold_button, can_ui.release_button))
        can_ui.open_tuning()
        can_ui.hotkey(SimpleNamespace(keysym="F6"))
        assert can_ui.tuning_panel is None and guarded.tuning.jobs.empty()
        assert "CAN2" in can_ui.card_vars[1]["position"].get()
        assert "未核对" in can_ui.card_vars[3]["control"].get()
        can_root.deiconify()
        can_root.geometry("1120x780")
        can_root.update()
        can_ui.draw_plots()
        for canvas, _, _ in can_ui.canvases:
            legends = {canvas.itemcget(item, "text") for item in canvas.find_all()
                       if canvas.type(item) == "text"
                       and re.fullmatch(r"J[1-5]", canvas.itemcget(item, "text"))}
            assert legends == {"J3", "J4", "J5"}, "Chart included another bus's legend"
        for widget in (can_ui.capture_button, can_ui.hold_button, can_ui.release_button, can_ui.export_button):
            assert widget.winfo_ismapped()
            assert widget.winfo_rootx() + widget.winfo_width() < can_root.winfo_rootx() + can_root.winfo_width()
            assert widget.winfo_rooty() + widget.winfo_height() < can_root.winfo_rooty() + can_root.winfo_height()
        can_ui.show_details.set(True)
        can_ui.toggle_details()
        can_root.update()
        assert can_ui.diagnostics.winfo_manager() and all(w.winfo_manager() for w in can_ui.detail_widgets)
        can_ui.show_details.set(False)
        can_ui.toggle_details()
        assert not can_ui.diagnostics.winfo_manager()
    for after_id in can_root.tk.splitlist(can_root.tk.call("after", "info")):
        can_root.tk.call("after", "cancel", after_id)
    can_root.destroy()
    print("PASS: five-motor configuration, CAN1 passive/CAN2 RPC isolation, simple GUI/actions/CSV,"
          " primary CAN feedback/MIT, v1/v2 UART alternative, 2/3/6-joint worker/cards/CSV,"
          " RPC, frozen CAN feedback, Bus-Off, port failures/timeouts/reboot,"
          " shared-bus single query owner (startup/offline/old firmware),"
          " standalone read-only queries, no bare motor control, feedback, CSV,"
          " worker/pause/cleanup, 38-field GUI, F5/F6 MCU commands and calibration (offline).")


def probe(hz, seconds, direct=False, can=True, port=None, can_bus=2, usb=False):
    adapters = scan_adapters() if can else scan_serial_ports()
    if port:
        adapters = [a for a in adapters if a["port"].casefold() == port.casefold()]
    if len(adapters) != 1:
        raise RuntimeError(f"匹配到 {len(adapters)} 个设备；请用 --port COMx 或图形界面明确选择。")
    samples, events = queue.Queue(maxsize=8192), queue.SimpleQueue()
    reader = (Reader(adapters[0], hz, samples, events, time.perf_counter(),
                     direct=direct, can_bus=can_bus) if can else
              (USBReader if usb else SerialReader)(adapters[0], samples, events, time.perf_counter()))
    reader.start()
    started = None
    errors, latest, limits, tuning = [], {}, {}, None
    try:
        while reader.is_alive():
            try:
                kind, value = events.get(timeout=0.05)
            except queue.Empty:
                kind, value = None, None
            if kind == "connected":
                started = time.perf_counter()
            elif kind == "limits":
                limits = value
            elif kind == "error":
                errors.append(value)
            elif kind in ("tuning_readback", "tuning_applied"):
                tuning = value
            while True:
                try:
                    sample = samples.get_nowait()
                except queue.Empty:
                    break
                latest[sample.motor] = sample
            if started is not None and time.perf_counter() - started >= seconds:
                reader.stop_requested.set()
    finally:
        reader.stop_requested.set()
        reader.join(timeout=3)
    while not events.empty():
        kind, value = events.get_nowait()
        if kind == "error":
            errors.append(value)
    expected = set(reader.motors) if can else (
        {j["joint" if usb else "id"] for j in reader.telemetry["joints"]} if reader.telemetry else set())
    legacy = (LEGACY_MOTORS.intersection(expected) if can else
              {j["joint"] for j in reader.telemetry["joints"] if j["protocol"] == 1}
              if usb and reader.telemetry else set())
    now = time.perf_counter()
    per_motor = {}
    for motor in sorted(expected):
        sample = latest.get(motor)
        age_s = max(0., now - reader.origin - sample.elapsed_s) if sample else None
        per_motor[motor] = {
            "status": ("FRESH" if age_s <= .5 else "STALE") if sample else (
                "WAITING_COMMAND_REPLY" if motor in legacy else "NO_FEEDBACK"),
            "age_ms": age_s * 1000 if sample else None,
            "state": sample.state if sample else None,
        }
    fresh = bool(expected) and set(latest) == expected and all(
        v["status"] == "FRESH" for v in per_motor.values())
    result = {
        "requested_hz_per_motor": hz if direct else None, "duration_s": seconds,
        "transport": "USB CDC" if usb else "USB-CAN" if can else f"USART1 {dm_serial.BAUDRATE} 8N1",
        "selected_can_bus": f"CAN{can_bus}" if can else None,
        "query_owner": "PC (standalone only)" if direct else "MCU",
        "limits_source": reader.limits_source if can else "MCU configured decode scales",
        "decode_limits": limits,
        "readback_limits": (None if can and can_bus == 1 and not direct else
                            {m: value for m, value in limits.items()
                             if m not in LEGACY_MOTORS} if can and direct else limits),
        "legacy_passive_motors": sorted(legacy),
        "per_motor_feedback": per_motor,
        "all_motors_verified": fresh,
        "stats": reader.last_stats,
        "mcu_tuning": tuning,
        "serial_status": None if can else reader.telemetry,
        "latest": {m: {"position_deg": s.position_deg, "position_rad": s.position_rad,
                       "velocity_rad_s": s.velocity_rad_s, "estimated_torque_nm": s.torque_nm,
                       "state": s.state, "mos_c": s.mos_c, "rotor_c": s.rotor_c,
                       "raw": s.raw.hex(" ").upper()} for m, s in latest.items()},
        "errors": errors, "adapter_thread_stopped": not reader.is_alive(),
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if not errors and fresh and not reader.is_alive() else 1


def main():
    parser = argparse.ArgumentParser(description="Ylamp 默认板载USB五电机监控与J1/J2简单控制")
    parser.add_argument("--self-test", action="store_true", help="离线自检，不访问硬件")
    parser.add_argument("--probe", action="store_true", help="短时硬件只读测试并输出 JSON")
    parser.add_argument("--port", help="明确选择板载USB/备用UART串口，如 COM5")
    transport = parser.add_mutually_exclusive_group()
    transport.add_argument("--usb", action="store_true", help="板载USB CDC（默认），不需要USB-CAN")
    transport.add_argument("--can", action="store_true", help="显式使用原USB-CAN链路")
    transport.add_argument("--serial", action="store_true",
                           help="显式使用 USART1 备用链路；勿同时连接两个调参程序")
    parser.add_argument("--bus", type=int, choices=(1, 2), default=2,
                        help="适配器实际接入的板上 CAN 总线；2=J1/J2，1=J3/J4/J5 仅查看")
    parser.add_argument("--direct", action="store_true",
                        help="仅 --probe --can：独立电机总线查询；必须先断开 MCU/其他查询源")
    parser.add_argument("--hz", type=int, default=100,
                        help="仅 --probe --direct 的每台查询 Hz，10–500，默认 100")
    parser.add_argument("--seconds", type=float, default=5, help="--probe 测试秒数，0.5–60")
    parser.add_argument("--autostart", action="store_true", help="打开界面后开始只读查询")
    parser.add_argument("--tune", action="store_true", help="同时打开 MCU 参数、行程和保护面板")
    args = parser.parse_args()
    if not 10 <= args.hz <= 500:
        parser.error("--hz 必须在 10–500 之间")
    if not math.isfinite(args.seconds) or not 0.5 <= args.seconds <= 60:
        parser.error("--seconds 必须在 0.5–60 之间")
    if args.self_test:
        self_test()
        return 0
    if args.port and args.can:
        parser.error("--port 不能与 --can 同时使用")
    use_can, use_usb = args.can, not (args.can or args.serial)
    if not use_can and args.bus != 2:
        parser.error("--bus 仅用于 USB-CAN，不能改变串口回传的关节")
    if args.direct and (not args.probe or not use_can):
        parser.error("--direct 仅用于 CAN --probe，不能与 MCU 共享查询总线")
    if args.probe:
        try:
            return probe(args.hz, args.seconds, args.direct, use_can, args.port, args.bus, usb=use_usb)
        except Exception as exc:
            print(f"{type(exc).__name__}: {exc}", file=sys.stderr)
            return 1
    if sys.platform == "win32":
        import ctypes
        try:
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except (AttributeError, OSError):
            pass
    import tkinter as tk
    root = tk.Tk()
    monitor = Monitor(root, args.hz, args.autostart, can=use_can, can_bus=args.bus, usb=use_usb, port=args.port)
    if args.tune:
        monitor.open_tuning()
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
