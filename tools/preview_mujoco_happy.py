"""Simulation-only happy v2; native MuJoCo, no hardware dependencies.

Run: D:/python/3.13/python.exe tools/preview_mujoco_happy.py --amplitude 2 --open
Writes only a new build directory. Space pauses, R restarts, Esc closes viewer.
The copied MJCF is a scene/trajectory asset, not a self-contained controller:
replay through the linked native player to execute the live model feedforward.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
from datetime import datetime

ROOT = Path(__file__).resolve().parents[1]
SDK = Path("D:/MuJoCo/4.0.1")
GCC = Path("C:/msys64/ucrt64/bin/gcc.exe")
SOURCE = ROOT / "simulation/ylamp_v12_identified_collision.xml"
FPS, WIDTH, HEIGHT = 30, 960, 720

# CAD angles, degrees; a chosen SIMULATION pose, NOT a real-machine move.
# The native v2 reference combines C2 attention/perk/relax and two diminishing
# pulses with a 80/120ms torso delay. These are endpoints, not stop-at-knot poses.
KEYS = [
    (0.0, 6, 49, -70, 120, 2),
    (5.2, 20, 36, -85, 123, 14),
]


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def scene(output: Path, amplitude: float) -> None:
    tree = ET.parse(SOURCE)
    root = tree.getroot()
    root.set("model", "Ylamp V1.2 HAPPY v2 - SIMULATION ONLY")
    # Remove stale b5d5 control metadata/keyframe, never alter the source asset.
    for tag in ("custom", "keyframe"):
        old = root.find(tag)
        if old is not None:
            root.remove(old)
    custom = ET.SubElement(root, "custom")
    ET.SubElement(custom, "numeric", name="happy_style", data="2")
    ET.SubElement(custom, "numeric", name="happy_response_scale", data=str(amplitude))
    flat = [x for row in KEYS for x in (row[0], *(math.radians(v) for v in row[1:]))]
    ET.SubElement(custom, "numeric", name="happy_keyframes",
                  data=" ".join(format(x, ".17g") for x in flat))
    ET.SubElement(custom, "text", name="simulation_scope", data=(
        f"Parameterized head-led happy v2, response amplitude x{amplitude:g}; "
        "not reinforcement learning. q=CAD rad, v=rad/s. "
        "Replay with tools/preview_mujoco_happy.py's native player for live G+M*a+C "
        "and frozen native MIT gains. Not MCU active/KEY sampled yielding. "
        "Original SDF contacts/ranges/mass/inertia preserved. Nominal damping only; "
        "nonlinear friction, drive delays/current loop/backlash not validated. "
        "No real-hardware commands. Standalone XML is NOT controller-equivalent."))
    ET.SubElement(custom, "text", name="source_sha256", data=json.dumps({
        str(p.relative_to(ROOT)): digest(p) for p in (
            SOURCE, ROOT / "config/five_mit.c", ROOT / "Model/five_dynamics.c",
            ROOT / "config/joint_references.def",
            ROOT / "tools/mujoco_happy_preview.c", Path(__file__),
            ROOT / "tools/mujoco_impedance_native.c", ROOT / "config/five_coordinates.h",
        )
    }, sort_keys=True))
    ET.SubElement(custom, "numeric", name="simulation_active_caps_nm", data="4 10 3.6 1.6 .8")
    keyframe = ET.SubElement(root, "keyframe")
    q0 = flat[1:6]
    ET.SubElement(keyframe, "key", name="happy_start_CAD",
                  qpos=" ".join(map(str, q0)), qvel="0 0 0 0 0",
                  ctrl=" ".join(str(x) for q in q0 for x in (q, 0, 0)))
    for actuator, kp, kd, cap in zip(root.find("actuator"),  # type: ignore[arg-type]
                                   (84.84, 63.84, 15.75, 16.66, 16.66),
                                   (3.1248, 5, 1.6054, .6969, .4207),
                                   (4, 10, 3.6, 1.6, .8)):
        actuator.set("kp", str(kp))
        actuator.set("kv", str(kd))
        actuator.set("forcerange", f"{-cap} {cap}")
        actuator.set("forcelimited", "true")
    # Presentation only: neutral studio floor. Collision filters are untouched.
    floor = root.find(".//geom[@type='plane']")
    if floor is not None:
        floor.set("rgba", ".14 .17 .20 1")
    visual = root.find("visual")
    if visual is None:
        visual = ET.SubElement(root, "visual")
    visual_global = visual.find("global")
    if visual_global is None:
        visual_global = ET.SubElement(visual, "global")
    visual_global.set("offwidth", str(WIDTH))
    visual_global.set("offheight", str(HEIGHT))
    ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--open", action="store_true")
    parser.add_argument("--amplitude", type=float, default=2.0,
                        help="response-only amplitude multiplier relative to the original v2")
    args = parser.parse_args()
    if not math.isfinite(args.amplitude) or args.amplitude <= 0:
        parser.error("--amplitude must be finite and positive")
    out = (args.output or ROOT / "build/v12-happy-preview-20261124" /
           datetime.now().strftime(f"v2_amp{args.amplitude:g}_%H%M%S")).resolve()
    out.mkdir(parents=True, exist_ok=False)  # refuse to overwrite prior evidence
    xml, exe = out / "ylamp_happy_v2.xml", out / "happy_preview.exe"
    scene(xml, args.amplitude)
    sources = ("tools/mujoco_happy_preview.c", "tools/mujoco_impedance_native.c",
               "Model/five_dynamics.c", "config/five_mit.c")
    command = [str(GCC), "-O2", "-std=c11", "-Wall", "-Wextra", "-static-libgcc",
               f"-I{SDK / 'include'}"]
    command += [f"-I{ROOT / p}" for p in (
        "Model", "Mode", "config", "Device/DM_Motor/Common", "BSP/CAN", "tests/fakes")]
    command += [str(ROOT / s) for s in sources]
    command += [str(SDK / "bin/mujoco.dll"), "-lopengl32", "-lgdi32", "-luser32",
                "-lm", "-Wl,--no-insert-timestamp", "-o", str(exe)]
    env = os.environ.copy()
    env["PATH"] = f"{SDK / 'bin'};{GCC.parent};{env.get('PATH', '')}"
    result = subprocess.run(command, env=env, capture_output=True, text=True)
    (out / "compile.log").write_text(result.stdout + result.stderr, encoding="utf-8")
    result.check_returncode()
    states = out / "simulation_states.bin"
    subprocess.run([str(exe), "--check", str(xml), str(out / "check.json"), str(states)],
                   env=env, check=True)
    check = json.loads((out / "check.json").read_text())
    print(json.dumps(check, indent=2), flush=True)
    import imageio_ffmpeg  # already installed; no MuJoCo Python package needed
    ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    video = out / "ylamp_happy_v2.mp4"
    with (out / "render.log").open("wb") as log:
        renderer = subprocess.Popen(
            [str(exe), "--record", str(xml), str(states)],
            env=env, stdout=subprocess.PIPE, stderr=log)
        encoder = subprocess.run([
            ffmpeg, "-n", "-loglevel", "warning", "-f", "rawvideo", "-pix_fmt",
            "rgb24", "-s", f"{WIDTH}x{HEIGHT}", "-r", str(FPS), "-i", "pipe:0",
            "-vf", "vflip", "-an", "-c:v", "libx264", "-crf", "20",
            "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(video),
        ], stdin=renderer.stdout, stdout=log, stderr=log)
        renderer.stdout.close()
        status = renderer.wait()
        encoder.check_returncode()
        if status:
            raise RuntimeError(f"Native rendering failed: {status}; see {out / 'render.log'}")
    subprocess.run([ffmpeg, "-n", "-loglevel", "error", "-ss", "1.4", "-i",
                    str(video), "-frames:v", "1", str(out / "poster.png")], check=True)
    manifest = {
        "simulation_only": True, "trained": False, "hardware_access": False,
        "response_amplitude_scale": args.amplitude,
        "keyframes_CAD_deg": KEYS, "controller_sources": sources,
        "trajectory": "Native C head-led overlapping C2 components; endpoint KEYS are not the full curve",
        "rendering": "Replay of audited 500Hz physical states, not a second independent simulation",
        "video": str(video), "xml": str(xml),
        "sha256": {p.name: digest(p) for p in (xml, exe, states, video)},
    }
    if args.open:
        with (out / "viewer.log").open("wb") as log:
            process = subprocess.Popen(
                [str(exe), "--view", str(xml), str(states)],
                env=env, stdout=log, stderr=log,
                creationflags=subprocess.CREATE_NO_WINDOW)
        manifest["viewer_pid"] = process.pid
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(f"VIDEO: {video}\nXML: {xml}", flush=True)


if __name__ == "__main__":
    main()
