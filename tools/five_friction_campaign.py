"""Explicit remaining friction trials; reuses existing moves/sweeps, never enables or clears faults."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

from five_control import Board, idle_output_confirmed
from five_identify import HW, REFS, ROOT
from five_gravity_residual_report import support_gravity
from usb_monitor import serial_port


# Same installed contact-plane rules as urdf_collision/workspace_sim.py.
# Only the existing bounded mounting surfaces are removed, never whole pairs.
GEOMETRY = r"""
import {loadModel} from './check.mjs';
import {resolve} from 'node:path';
import {createInterface} from 'node:readline';
const rules=Object.fromEntries(['part3/part5','part0/part2','part1/part10']
  .map(k=>[k,{y:-.0005,axialTolerance:.00003,radius:.03}]));
const m=loadModel(resolve('../../../Ylamp-URDF-package(1)/Ylamp.urdf'),rules);
console.log(JSON.stringify({ready:true,input:m.input}));
for await(const text of createInterface({input:process.stdin})) {
  const paths=JSON.parse(text); let checked=0, witness=null;
  for(const [a,b] of paths) {
    const n=Math.max(1,Math.ceil(Math.max(...a.map((x,i)=>Math.abs(x-b[i])))/3));
    for(let k=0;k<=n;k++) {
      const q=a.map((x,i)=>x+(b[i]-x)*k/n);
      const hits=[...m.collisions(q),...m.floorHits()]; checked++;
      if(hits.length) {witness={q,hits};break;}
    }
    if(witness) break;
  }
  console.log(JSON.stringify({checked,witness,finite_step_deg:3}));
}
"""


def anchors(directory, pose, speed, scope):
    prefix = "friction_overlay" if pose == 1 else "friction_pose02"
    suffix = f"_{scope}" if pose == 2 or speed == "016" else ""
    path = directory/f"{prefix}_{speed}{suffix}_01.jsonl"
    with path.open(encoding="utf-8") as source:
        for line in source:
            r = json.loads(line)
            if r["stage"] == "waypoint" and r["joint"] == (1 if scope == "j1234" else 5):
                return np.array(r["start_software_deg"])+np.rad2deg(REFS[:, 0])
    raise ValueError("No original start pose")


class Campaign:
    def __init__(self, args, log):
        self.args, self.log, self.number = args, log, 0
        self.geometry = subprocess.Popen(
            ["node", "--input-type=module", "-e", GEOMETRY],
            cwd=ROOT/"tools/urdf_collision", stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, text=True, encoding="utf-8")
        while True:
            line = self.geometry.stdout.readline()
            if not line:
                raise RuntimeError("Geometry loader failed")
            print(line.rstrip(), flush=True)
            if line.startswith("{"):
                self.note(stage="geometry_input", **json.loads(line))
                break

    def note(self, **record):
        print(json.dumps(dict(unix_s=time.time(), **record), allow_nan=False),
              file=self.log, flush=True)

    def state(self):
        with serial_port(self.args.port) as port:
            b = Board(port)
            b.connect()
            references = [[b.scalar(f, j, op=15) for f in (1, 2, 3)] for j in range(1, 6)]
            if not np.allclose(references, REFS, rtol=0, atol=2e-6):
                raise ValueError("Board references differ from the planned raw-coordinate basis")
            state = b.status()
        self.note(stage="status", **state)
        if not idle_output_confirmed(state):
            raise RuntimeError("Healthy idle support absent; no automatic recovery")
        return np.rad2deg([j["position_rad"] for j in state["telemetry"]["joints"]])

    def check(self, paths):
        shift = HW[:, 6]-np.rad2deg(REFS[:, 0])
        encoded = [[(np.asarray(a)+shift).tolist(), (np.asarray(b)+shift).tolist()]
                   for a, b in paths]
        self.geometry.stdin.write(json.dumps(encoded)+"\n")
        self.geometry.stdin.flush()
        line = self.geometry.stdout.readline()
        if not line:
            raise RuntimeError("Geometry check failed")
        result = json.loads(line)
        self.note(stage="geometry_check", raw_paths=paths, **result)
        if result["witness"]:
            raise ValueError(f"Actual directed path intersects CAD: {result['witness']}")
        # The existing gravity budgets, not a new torque/velocity gate.
        q = np.concatenate([np.linspace(a, b, 25)+shift for a, b in paths])
        demand = np.max(abs(support_gravity(np.deg2rad(q))), axis=0)
        self.note(stage="static_model", max_gravity_nm=demand.tolist())
        if np.any(demand > np.array([.4, 3., 1.8, .8, .4])):
            raise ValueError("Planned pose exceeds retained gravity budget")

    def run_tool(self, tool, log_name, options):
        path = self.args.directory/log_name
        if path.exists():
            raise FileExistsError(f"Capture already exists: {path}")
        self.note(stage="launch", capture=str(path), tool=tool, arguments=options)
        print(f"RUN {log_name}", flush=True)
        subprocess.run([sys.executable, "-B", str(ROOT/"tools"/tool),
                        "--port", self.args.port, "--log", str(path), *options], check=True)
        self.note(stage="completed", capture=str(path))

    def transfer(self, target, tag):
        current = self.state()
        target = np.asarray(target)
        if np.any(target < np.rad2deg(REFS[:, 1])+1.1) or np.any(target > np.rad2deg(REFS[:, 2])-1.1):
            raise ValueError("Pose target outside retained raw travel")
        self.check([[current.tolist(), target.tolist()]])
        ms = max(6000, math.ceil(1.875*np.max(abs(np.deg2rad(target-current)))/.12*1000))
        self.run_tool("five_control.py", f"{tag}_transfer.jsonl",
            ["--seconds", ".25", "move", "--ms", str(ms),
             *map(str, target-np.rad2deg(REFS[:, 0]))])
        return self.state()

    def sweep(self, raw, tag, speed, scope, plateau, ramp_ms=1000):
        joints = [1, 2, 3, 4] if scope == "j1234" else [5]
        # Check only the directed inward/return path, not unrelated outward limits.
        current = np.array(raw)
        paths = []
        for j in joints:
            lo, hi = np.rad2deg(REFS[j-1, 1:])-np.rad2deg(REFS[j-1, 0])
            start = current[j-1]-math.degrees(REFS[j-1, 0])
            sign = 1 if hi-1.1-start >= start-lo-1.1 else -1
            end = current.copy()
            end[j-1] += sign*math.degrees(speed)*(plateau+ramp_ms)/1000
            if not math.degrees(REFS[j-1, 1])+1.1 <= end[j-1] <= math.degrees(REFS[j-1, 2])-1.1:
                raise ValueError("Directed speed trial does not fit retained travel")
            paths.extend([[current.tolist(), end.tolist()], [end.tolist(), current.tolist()]])
        self.check(paths)
        self.run_tool("five_gravity_sweep.py", f"{tag}_{scope}.jsonl",
            ["--constant-speed", str(speed), "--plateau-ms", str(plateau),
             "--ramp-ms", str(ramp_ms), "--joints", *map(str, joints)])

    def execute(self):
        # Six new 30-leg records complete the original four-pose/two-repeat plan.
        for pose, repeat in ((2, 2), (1, 2), (3, 1), (4, 1), (3, 2), (4, 2)):
            for speed_tag, speed, plateau in (("016", .16, 2000), ("008", .08, 3000),
                                               ("002", .02, 5000)):
                for scope in ("j1234", "j5"):
                    tag = f"friction_pose{pose:02d}_r{repeat}_{speed_tag}"
                    if pose in (1, 2):
                        target = anchors(self.args.directory, pose, speed_tag, scope)
                    else:
                        target = anchors(self.args.directory, 1, speed_tag, scope)
                        # Independent J4-only and J3-only changes of the two existing load poses.
                        other = anchors(self.args.directory, 2, speed_tag, scope)
                        target[3 if pose == 3 else 2] = other[3 if pose == 3 else 2]
                    raw = self.transfer(target, f"{tag}_{scope}")
                    self.sweep(raw, tag, speed, scope, 1300 if speed == .16 and scope == "j5" else plateau)
            self.note(stage="pose_repeat_complete", pose=pose, repeat=repeat)
            print(f"POSE {pose} REPEAT {repeat}: 30 legs completed; returned to gravity", flush=True)
        self.state()
        self.note(stage="campaign_complete", new_legs=180)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--directory", type=Path, default=ROOT/"build/v12-speed-friction-20261121")
    parser.add_argument("--log", type=Path)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        for pose in (1, 2):
            for speed in ("002", "008", "016"):
                for scope in ("j1234", "j5"):
                    raw = anchors(args.directory, pose, speed, scope)
                    assert raw.shape == (5,) and np.isfinite(raw).all()
                    assert np.all(raw > np.rad2deg(REFS[:, 1]))
                    assert np.all(raw < np.rad2deg(REFS[:, 2]))
        assert abs(anchors(args.directory, 1, "008", "j5")[2]-44.81765) < .001
        assert abs(anchors(args.directory, 2, "016", "j5")[2]-102.45437) < .001
        print("PASS: 12 original pose/speed/scope anchors, J5 split identity, retained raw travel")
        return
    if not args.execute or not args.port or args.log is None:
        parser.error("explicit --execute, --port and new --log required")
    with args.log.open("x", encoding="utf-8") as log:
        campaign = Campaign(args, log)
        try:
            campaign.execute()
        except BaseException as exc:
            campaign.note(stage="campaign_abort", error=str(exc))
            raise
        finally:
            campaign.geometry.stdin.close()
            campaign.geometry.wait(timeout=10)


if __name__ == "__main__":
    main()
