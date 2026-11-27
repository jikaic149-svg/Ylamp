"""Read-only preflight for a Ylamp V1.2 ELF/BIN pair (GNU ARM, current ABI).

This detects the wrong application/profile. It does not certify a mechanism,
prove that a runtime branch was taken, or authorize flashing or enabling.
"""
import argparse
import csv
import hashlib
import math
from pathlib import Path
import re
import struct
import subprocess
import tempfile


FLASH = 0x08000000
REFS = Path(__file__).resolve().parents[1] / "config" / "joint_references.def"
HEADER = re.compile(r"^([0-9a-fA-F]+) <([^>]+)>:$")
BRANCH = re.compile(r"\s(?:blx?|b(?:eq|ne|cs|cc|mi|pl|vs|vc|hi|ls|ge|lt|gt|le)?(?:\.[wn])?)\s+[\da-fA-F]+ <([^>+]+)>")


def call_graph(disassembly):
    graph, owner = {}, None
    for line in disassembly.splitlines():
        head = HEADER.match(line)
        if head:
            owner = head[2]
            graph.setdefault(owner, set())
        elif owner and (branch := BRANCH.search(line)):
            graph[owner].add(branch[1])
    return graph


def reachable(graph, root):
    seen, todo = set(), [root] if root in graph else []
    while todo:
        name = todo.pop()
        if name not in seen:
            seen.add(name)
            todo.extend(graph.get(name, ()))
    return seen


def check_calls(graph, velocity_loop=False, manual_drag=False,
                continuous_trajectory=False, active_friction=False, lookaround=False):
    # FreeRTOS starts control_task by pointer; check the registration separately.
    required = {
        "Reset_Handler": ("main",),
        "main": ("Ylamp_StartScheduler", "DM_Motor_Init"),
        "Ylamp_StartScheduler": ("xTaskCreateStatic", "vTaskStartScheduler"),
        "control_task": ("Ylamp_ControlStep",),
        "Ylamp_ControlStep": ("FiveControl_Tick", "FiveMIT_SendSample", "MotorUSB_Run",
                             "BSP_KEY_Scan", "LookaroundTask_Run" if lookaround else "FiveControl_KeyPress"),
        "FiveControl_Tick": ("FiveDynamics_Inverse", "FiveSupport_GravityModel",
                             "FiveDynamics_GravityResidual", "DM_Motor_SendMIT", "FiveSupport_Sample"),
        "FiveControl_KeyPress": ("FiveControl_ColdStart",),
        "FiveSupport_Sample": ("DM_Motor_EncodeMIT",),
        "MotorUSB_Run": ("FiveControl_ColdStart", "FiveControl_Move"),
        "FiveMIT_SendSample": ("FiveMIT_Sample", "DM_Motor_SendMIT"),
        "FiveMIT_Sample": ("FiveDynamics_Inverse",),
        "DM_Motor_SendMIT": ("BSP_CAN_Send",),
    }
    required["MotorUSB_Run"] += ("FiveControl_Impedance", "FiveControl_ImpedanceTest")
    required["FiveControl_Impedance"] = ("FiveSupport_EnterImpedance",)
    required["FiveControl_ImpedanceTest"] = ("FiveMIT_Prepare", "FiveSupport_StartTest")
    required["FiveControl_Tick"] += ("FiveSupport_TestTick",)
    if lookaround:
        required["LookaroundTask_Run"] = ("FiveControl_KeyPress", "FiveControl_LocalTrajectoryBegin",
                                          "FiveControl_LocalTrajectoryStep", "LookaroundStore_Points")
        required["main"] += ("LookaroundStore_Init", "W25Q64_Read")
        required["LookaroundTask_Run"] += ("LookaroundStore_Run",)
        required["MotorUSB_Run"] += ("LookaroundStore_Begin", "LookaroundStore_Append", "LookaroundStore_Save")
        required["FiveControl_LocalTrajectoryBegin"] = ("FiveTrajectory_Begin",)
        # ARM Release inlines FiveControl_Idle here; inspect its gravity/cancel calls.
        required["FiveControl_LocalTrajectoryStep"] = ("FiveTrajectory_AppendPoint",
            "FiveTrajectory_RelativePoint", "FiveTrajectory_PrepareStart", "FiveTrajectory_Start",
            "FiveSupport_Gravity", "FiveDrag_Stop", "FiveTrajectory_Cancel")
    else:
        required["FiveControl_KeyPress"] += ("FiveSupport_EnterImpedance",)
    if velocity_loop:
        required["MotorUSB_Run"] += ("FiveControl_Speed",)
        required["FiveControl_Speed"] = ("FiveMIT_PrepareCruise",)
        required["FiveMIT_SampleFeedback"] = ("FiveMIT_SpeedFrictionTorque",
                                             "FiveDynamics_GravityLoad")
    if manual_drag:
        required["MotorUSB_Run"] += ("FiveControl_Drag", "FiveControl_Idle", "FiveControl_Release")
        required["FiveControl_Drag"] = ("FiveDrag_Enter",)
        required["FiveControl_Tick"] += ("FiveDrag_Sample", "FiveDrag_Enter")
        required["FiveDrag_Sample"] = ("FiveMIT_SpeedFrictionTorque", "FiveDynamics_GravityLoad")
    if continuous_trajectory:
        required["MotorUSB_Run"] += ("FiveControl_TrajectoryBegin", "FiveControl_TrajectorySet",
                                    "FiveControl_TrajectoryAppend", "FiveControl_TrajectoryStart")
        required["FiveControl_TrajectoryAppend"] = ("FiveTrajectory_Append",)
        required["FiveControl_TrajectoryStart"] = ("FiveTrajectory_PrepareStart","FiveTrajectory_Start")
        required["FiveControl_Tick"] += ("FiveTrajectory_Sample", "FiveMIT_SendReference")
        required["FiveMIT_SendReference"] = ("FiveMIT_SampleReferenceFeedback", "DM_Motor_SendMIT")
    if active_friction:
        required["FiveMIT_SampleFeedback"] = ("FiveMIT_SpeedFrictionTorque", "FiveDynamics_GravityLoad")
        required["FiveMIT_SampleReferenceFeedback"] = ("FiveMIT_SpeedFrictionTorque",
                                                      "FiveDynamics_GravityLoad")
    failures = []
    for root, targets in required.items():
        calls = reachable(graph, root)
        failures.extend(f"missing {root} -> {target}" for target in targets if target not in calls)
    if lookaround:
        calls = reachable(graph, "LookaroundTask_Run")
        if "FiveSupport_EnterImpedance" in calls:
            failures.append("KEY still enters impedance")
        if {"DM_Motor_SendMIT", "BSP_CAN_Send"} & reachable(graph, "FiveControl_LocalTrajectoryStep"):
            failures.append("local trajectory preparation has its own CAN output")
    if continuous_trajectory:
        # Mangled native solver names deliberately inspected; a renamed custom
        # spline or a dead uncalled Ruckig archive must not pass this check.
        planning = reachable(graph,"FiveControl_TrajectoryAppend") | reachable(
            graph,"FiveControl_TrajectoryStart")
        if not any("ruckig" in name and "PositionThirdOrderStep" in name for name in planning):
            failures.append("five-axis path has no reachable native Ruckig third-order solver")
        if not any("TargetCalculatorILj5E" in name or "TargetCalculatorILm5E" in name
                   for name in planning):
            failures.append("Ruckig is not instantiated for five DOFs")
        if any("ruckig" in name and "calculate" in name
               for name in reachable(graph,"FiveControl_Tick")):
            failures.append("Ruckig replanning appears in the 500Hz motion tick")
    old = {"MotorTask_Run", "J25Shadow_Tick", "CanCheck_Run", "J5Test_Run",
           "DM_Motor_Disable", "BSP_CAN_RecoverBusOff", "NVIC_SystemReset"}
    failures.extend(f"wrong control owner: {name}" for name in sorted(old & reachable(graph, "Ylamp_ControlStep")))
    return failures


def control_registered(asm, image):
    """Fail closed if the current GNU/Thumb task-registration pattern changes."""
    task = re.search(r"^([\da-fA-F]+) <control_task>:$", asm, re.M)
    code = re.search(r"^[\da-fA-F]+ <Ylamp_StartScheduler>:\n((?:[^\n]+\n)+)", asm, re.M)
    if not task or not code:
        return False
    # AAPCS arg0 is r0. In this build its last write is the literal load
    # immediately before the call, so no instruction can replace the pointer.
    for word in re.finditer(r"\bldr(?:\.w)?\s+r0,\s*\[pc,[^]]+\][^\n]*?;\s*\(([\da-fA-F]+) [^)\n]*\)\n"
                            r"[^\n]*?\bbl(?:\.w)?\s+[\da-fA-F]+ <xTaskCreateStatic>", code[1]):
        off = int(word[1], 16) - FLASH
        if 0 <= off <= len(image)-4 and struct.unpack_from("<I", image, off)[0] == (int(task[1], 16) | 1):
            return True
    return False


def reference_bytes():
    # Mirror DEG_RAD: the degree literals are double, its factor is an f32.
    rad, = struct.unpack("<f", struct.pack("<f", math.pi / 180))
    kinds = {"CANDIDATE": 1, "MIDPOINT": 2, "USER_APPROVED": 3}
    rows = []
    for record in re.findall(r"^JOINT_REFERENCE\(([^)]+)\)$", REFS.read_text(encoding="utf-8"), re.M):
        joint, zero, low, high, kind = next(csv.reader([record], skipinitialspace=True))
        if int(joint) != len(rows) + 1:
            raise ValueError("invalid joint order in joint_references.def")
        rows.append(struct.pack("<fffB3x", *(float(v) * rad for v in (zero, low, high)), kinds[kind]))
    if len(rows) != 5:
        raise ValueError("expected five calibrated joints")
    return b"".join(rows)


def support_bytes(manual_drag=False):
    """Match support and active smoothing configuration, not just function names."""
    text = (REFS.parent / "five_mit.c").read_text(encoding="utf-8")
    table = re.search(r"g_five_j2_gravity_delta\[FIVE_AXES\]\[4\]\s*=\s*\{(.*?)\};", text, re.S)
    if not table:
        raise ValueError("missing configured J2 gravity residual")
    rows = [[float(v.strip().rstrip("f")) for v in row.split(",")]
            for row in re.findall(r"\{([^{}]+)\}", table[1])]
    if len(rows) != 5 or any(len(row) != 4 for row in rows):
        raise ValueError("invalid configured J2 gravity residual")
    result = {"g_five_j2_gravity_delta": struct.pack("<20f", *(v for row in rows for v in row))}
    tracking = re.search(r"g_five_mit_tracking\[FIVE_AXES\]\s*=\s*\{(.*?)\};", text, re.S)
    if not tracking:
        raise ValueError("missing configured active tracking profile")
    gains = [[float(v.strip().rstrip("f")) for v in row.split(",")]
             for row in re.findall(r"\{([^{}]+)\}", tracking[1])]
    if len(gains) != 5 or any(len(row) != 3 for row in gains):
        raise ValueError("invalid configured active tracking profile")
    result["g_five_mit_tracking"] = struct.pack("<15f", *(v for row in gains for v in row))
    speed_pi = re.search(r"g_five_speed_pi\[FIVE_AXES\]\s*=\s*\{(.*?)\};", text, re.S)
    if not speed_pi:
        raise ValueError("missing configured finite velocity PI gains")
    speeds = [[float(v.strip().rstrip("f")) for v in row.split(",")]
              for row in re.findall(r"\{([^{}]+)\}", speed_pi[1])]
    if len(speeds) != 5 or any(len(row) != 2 for row in speeds):
        raise ValueError("invalid configured finite velocity PI gains")
    result["g_five_speed_pi"] = struct.pack("<10f", *(v for row in speeds for v in row))
    support = re.search(r"g_five_support_gains\[FIVE_AXES\]\s*=\s*\{(.*?)\};", text, re.S)
    rows = [[float(v.strip().rstrip("f")) for v in row.split(",")]
            for row in re.findall(r"\{([^{}]+)\}", support[1])] if support else []
    if (len(rows)!=5 or any(len(row)!=2 or not all(map(math.isfinite,row)) or
                           not 0<row[0]<=500 or not 0<=row[1]<=5 for row in rows)):
        raise ValueError("invalid configured native-MIT impedance gains")
    result["g_five_support_gains"] = struct.pack("<10f", *(v for row in rows for v in row))
    impedance = re.search(r"g_five_impedance_max_nm\[FIVE_AXES\]\s*=\s*\{([^}]+)\};", text)
    caps = [float(v.strip().rstrip("f")) for v in impedance[1].split(",")] if impedance else []
    if len(caps)!=5 or any(not math.isfinite(v) or v<=0 for v in caps):
        raise ValueError("invalid configured KEY-only impedance budgets")
    result["g_five_impedance_max_nm"] = struct.pack("<5f", *caps)
    pd_limit = re.search(r"g_five_impedance_pd_max_nm\[FIVE_AXES\]\s*=\s*\{([^}]+)\};", text)
    pd_caps = [float(v.strip().rstrip("f")) for v in pd_limit[1].split(",")] if pd_limit else []
    if len(pd_caps)!=5 or any(
            not math.isfinite(v) or not 0<v<=total for v, total in zip(pd_caps, caps)):
        raise ValueError("invalid configured native-MIT PD-only budgets")
    result["g_five_impedance_pd_max_nm"] = struct.pack("<5f", *pd_caps)
    trial = re.search(r"g_five_impedance_test\s*=\s*\{(\d+)U,(\d+)U,([-+.\deE]+)f?\};", text)
    if not trial:
        raise ValueError("missing native-impedance trial configuration")
    ramp, hold, offset = int(trial[1]), int(trial[2]), float(trial[3])
    if not (ramp>=100 and hold>0 and 4*(ramp+hold)<=120000 and
            math.isfinite(offset) and offset>0):
        raise ValueError("invalid native-impedance trial configuration")
    result["g_five_impedance_test"] = struct.pack("<IIf", ramp, hold, offset)
    friction = re.search(r"g_five_speed_friction\[FIVE_AXES\]\s*=\s*\{(.*?)\};", text, re.S)
    if not friction:
        raise ValueError("missing configured velocity-trial friction overlay")
    values = [[float(v.strip().rstrip("f")) for v in row.split(",")]
              for row in re.findall(r"\{([^{}]+)\}", friction[1])]
    if (len(values) != 5 or any(len(row) != 4 for row in values) or
            any(not all(map(math.isfinite,row)) for row in values)):
        raise ValueError("invalid configured velocity-trial friction overlay")
    result["g_five_speed_friction"] = struct.pack("<20f", *(v for row in values for v in row))
    for name in ("g_five_speed_friction_range_rad_s",
                 "g_five_speed_friction_knots_rad_s", "g_five_speed_friction_j2"):
        match = re.search(name+r"\[2\]\s*=\s*\{([^}]+)\};", text, re.S)
        if not match:
            raise ValueError(f"missing configured {name}")
        row = [float(v.strip().rstrip("f")) for v in match[1].split(",")]
        if len(row)!=2 or not all(map(math.isfinite,row)):
            raise ValueError(f"invalid configured {name}")
        if name!="g_five_speed_friction_j2" and not 0<row[0]<row[1]:
            raise ValueError(f"invalid amplitude-domain bounds in {name}")
        result[name] = struct.pack("<2f", *row)
    lo, hi = struct.unpack("<2f", result["g_five_speed_friction_range_rad_s"])
    k16, k48 = struct.unpack("<2f", result["g_five_speed_friction_knots_rad_s"])
    if not lo<k16<k48<hi:
        raise ValueError("friction shape knots must lie inside its amplitude domain")
    for name in ("g_five_j3_gravity_scale", "g_five_j3_gravity_cosine_nm",
                 "g_five_tracking_ramp_s"):
        match = re.search(name+r"\s*=\s*([-+.\deE]+)f?\s*;", text)
        if not match:
            raise ValueError(f"missing configured {name}")
        result[name] = struct.pack("<f", float(match[1]))
    if manual_drag:
        match = re.search(r"g_five_drag\[FIVE_AXES\]\s*=\s*\{(.*?)\};", text, re.S)
        if not match:
            raise ValueError("missing configured manual-drag profile")
        rows = [[float(v.strip().rstrip("f")) for v in row.split(",")]
                for row in re.findall(r"\{([^{}]+)\}", match[1])]
        if (len(rows)!=5 or any(len(row)!=5 or not all(map(math.isfinite,row)) or
                not 0<=row[0]<1 or row[1]<0 or row[2]<=0 or row[3]<0 or row[4]<=0
                for row in rows)):
            raise ValueError("invalid manual-drag profile")
        result["g_five_drag"] = struct.pack("<25f", *(v for row in rows for v in row))
        for name in ("g_five_drag_filter_s", "g_five_drag_ramp_s"):
            match = re.search(name+r"\s*=\s*([-+.\deE]+)f?\s*;", text)
            if not match or not math.isfinite(float(match[1])) or float(match[1])<=0:
                raise ValueError(f"invalid configured {name}")
            result[name] = struct.pack("<f", float(match[1]))
    return result


def inspect(elf, binary, objdump, objcopy, velocity_loop=False, manual_drag=False,
            continuous_trajectory=False, active_friction=False, lookaround=False):
    def dump(*flags):
        return subprocess.check_output([objdump, *flags, elf], encoding="utf-8", errors="replace")

    if "architecture: armv7e-m," not in dump("-f"):
        raise ValueError("not the expected ARMv7E-M executable")
    image = binary.read_bytes()
    if not 8 <= len(image) <= 1024 * 1024:
        raise ValueError("not an STM32H723 flash image")
    with tempfile.TemporaryDirectory() as tmp:
        extracted = Path(tmp) / "image.bin"
        subprocess.run([objcopy, "-O", "binary", elf, extracted], check=True)
        if extracted.read_bytes() != image:
            raise ValueError("BIN differs from ELF load image")
    asm, symbols = dump("-d"), dump("-t")
    failures = check_calls(call_graph(asm), velocity_loop, manual_drag,
                           continuous_trajectory or lookaround, active_friction, lookaround)
    if re.search(r"\b(?:FiveControl_ReverseJ3|FiveSupport_ApplyReverseJ3)\b", symbols):
        failures.append("retired J3 reverse trial present")
    if lookaround and re.search(r"\bFiveTrajectory_EntryWithin\b", symbols):
        failures.append("retired fixed-preset entry window present")
    if not control_registered(asm, image):
        failures.append("cannot verify control_task is registered with FreeRTOS")
    reset = re.search(r"^([\da-fA-F]+) <Reset_Handler>:$", asm, re.M)
    sp, entry = struct.unpack_from("<II", image)
    if not reset or entry != int(reset[1], 16) | 1 or sp != 0x20020000:
        failures.append("unexpected H723 reset vector / initial stack")
    tables = {"g_joint_reference_config": reference_bytes(), **support_bytes(manual_drag)}
    if lookaround:
        if re.search(r"\bg_five_lookaround$", symbols, re.M):
            failures.append("built-in lookaround point fallback present")
        cache = re.search(r"^([\da-fA-F]+)\s+l\s+O\s+\.trajectory_buffer\s+([\da-fA-F]+)\s+trajectory$",
                          symbols, re.M)
        if not cache or not 0x24000000<=int(cache[1],16)<0x24050000 or int(cache[2],16)<120*1024:
            failures.append("32-profile cache is not in bounded AXI SRAM")
    for name, expected in tables.items():
        table = re.search(r"^([\da-fA-F]+)\s+g\s+O\s+\.rodata\s+([\da-fA-F]+)\s+"+name+"$", symbols, re.M)
        offset = int(table[1], 16) - FLASH if table else -1
        if not table or int(table[2], 16) != len(expected) or offset < 0 or image[offset:offset+len(expected)] != expected:
            failures.append(f"missing or stale {name}")
    return failures, hashlib.sha256(image).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("elf", type=Path)
    p.add_argument("bin", type=Path)
    p.add_argument("--objdump", default="arm-none-eabi-objdump")
    p.add_argument("--objcopy", default="arm-none-eabi-objcopy")
    p.add_argument("--velocity-loop", action="store_true", help="also require native finite velocity-PI call chains")
    p.add_argument("--manual-drag", action="store_true", help="also require manual-drag call chains and profile")
    p.add_argument("--continuous-trajectory", action="store_true",
                   help="also require preloaded five-axis Ruckig and existing output-owner call chains")
    p.add_argument("--active-friction", action="store_true",
                   help="also require total F reachable from ordinary Move and Ruckig feedback")
    p.add_argument("--lookaround", action="store_true",
                   help="require KEY/W25Q64 preset loader, explicit save and AXI planning cache")
    args = p.parse_args()
    try:
        failures, digest = inspect(args.elf, args.bin, args.objdump, args.objcopy,
                                   args.velocity_loop, args.manual_drag,
                                   args.continuous_trajectory, args.active_friction, args.lookaround)
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        p.exit(1, f"V1.2 BLOCKED: {exc}\n")
    if failures:
        p.exit(1, f"V1.2 BLOCKED [{digest}]: " + "; ".join(failures) + "\n")
    print(f"V1.2 structural preflight PASS [{digest}]; physical commissioning still required.")


if __name__ == "__main__":
    main()
