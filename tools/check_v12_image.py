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


def check_calls(graph, reverse_test=False):
    # FreeRTOS starts control_task by pointer; check the registration separately.
    required = {
        "Reset_Handler": ("main",),
        "main": ("Ylamp_StartScheduler", "DM_Motor_Init"),
        "Ylamp_StartScheduler": ("xTaskCreateStatic", "vTaskStartScheduler"),
        "control_task": ("Ylamp_ControlStep",),
        "Ylamp_ControlStep": ("FiveControl_Tick", "FiveMIT_SendSample", "MotorUSB_Run",
                             "BSP_KEY_Scan", "FiveControl_KeyPress"),
        "FiveControl_Tick": ("FiveDynamics_Inverse", "FiveSupport_GravityModel",
                             "FiveDynamics_GravityResidual", "DM_Motor_SendTorque", "FiveSupport_Sample"),
        "FiveControl_KeyPress": ("FiveControl_ColdStart", "FiveSupport_EnterImpedance"),
        "MotorUSB_Run": ("FiveControl_ColdStart", "FiveControl_Move"),
        "FiveMIT_SendSample": ("FiveMIT_Sample", "DM_Motor_SendMIT"),
        "FiveMIT_Sample": ("FiveDynamics_Inverse",),
        "DM_Motor_SendMIT": ("BSP_CAN_Send",),
    }
    if reverse_test:
        required["MotorUSB_Run"] += ("FiveControl_ReverseJ3",)
        required["FiveControl_Tick"] += ("FiveSupport_ApplyReverseJ3",)
    failures = []
    for root, targets in required.items():
        calls = reachable(graph, root)
        failures.extend(f"missing {root} -> {target}" for target in targets if target not in calls)
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


def support_bytes():
    """Match the configured support candidate, not just its function names."""
    text = (REFS.parent / "five_mit.c").read_text(encoding="utf-8")
    table = re.search(r"g_five_j2_gravity_delta\[FIVE_AXES\]\[4\]\s*=\s*\{(.*?)\};", text, re.S)
    if not table:
        raise ValueError("missing configured J2 gravity residual")
    rows = [[float(v.strip().rstrip("f")) for v in row.split(",")]
            for row in re.findall(r"\{([^{}]+)\}", table[1])]
    if len(rows) != 5 or any(len(row) != 4 for row in rows):
        raise ValueError("invalid configured J2 gravity residual")
    result = {"g_five_j2_gravity_delta": struct.pack("<20f", *(v for row in rows for v in row))}
    for name in ("g_five_j3_gravity_scale", "g_five_j3_gravity_cosine_nm"):
        match = re.search(name+r"\s*=\s*([-+.\deE]+)f?\s*;", text)
        if not match:
            raise ValueError(f"missing configured {name}")
        result[name] = struct.pack("<f", float(match[1]))
    return result


def inspect(elf, binary, objdump, objcopy, reverse_test=False):
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
    failures = check_calls(call_graph(asm), reverse_test)
    if not control_registered(asm, image):
        failures.append("cannot verify control_task is registered with FreeRTOS")
    reset = re.search(r"^([\da-fA-F]+) <Reset_Handler>:$", asm, re.M)
    sp, entry = struct.unpack_from("<II", image)
    if not reset or entry != int(reset[1], 16) | 1 or sp != 0x20020000:
        failures.append("unexpected H723 reset vector / initial stack")
    for name, expected in {"g_joint_reference_config": reference_bytes(), **support_bytes()}.items():
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
    p.add_argument("--j3-reverse-test", action="store_true", help="also require the optional test call chains")
    args = p.parse_args()
    try:
        failures, digest = inspect(args.elf, args.bin, args.objdump, args.objcopy, args.j3_reverse_test)
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        p.exit(1, f"V1.2 BLOCKED: {exc}\n")
    if failures:
        p.exit(1, f"V1.2 BLOCKED [{digest}]: " + "; ".join(failures) + "\n")
    print(f"V1.2 structural preflight PASS [{digest}]; physical commissioning still required.")


if __name__ == "__main__":
    main()
