"""Explicit first J1/J2 acceleration batch; reuses the deployed cruise mode."""
import argparse
import hashlib
from pathlib import Path
import re

import numpy as np

from five_global_friction_scan import GlobalBatch, verify_timing_layout
from five_identify import REFS, ROOT


SPEED = .32
RAMPS_MS = (1200, 600, 300)  # Nominal peak accelerations .5, 1, 2 rad/s².
FIRMWARE_SHA = "9ec65d69b5260badf7fa526f367b3ca63e31646a17b8b87138bf5f27a0689514"


def pair_endpoints(raw, joint):
    """One finite 30-degree interval; retain every other measured coordinate."""
    raw = np.asarray(raw, dtype=float)
    if raw.shape != (5,) or not np.isfinite(raw).all() or joint not in (1, 2):
        raise ValueError("Invalid first-stage inertia anchor")
    lo, hi = np.rad2deg(REFS[:, 1:]).T + np.array([[1.1], [-1.1]])
    if np.any(raw < lo) or np.any(raw > hi):
        raise ValueError("Anchor is outside retained target inset; no motion sent")
    i = joint-1
    direction = 1 if hi[i]-raw[i] >= raw[i]-lo[i] else -1
    far = raw.copy()
    far[i] += direction*30.
    if not lo[i] <= far[i] <= hi[i]:
        raise ValueError("Finite inertia interval does not fit; no motion sent")
    return raw.copy(), far


class InertiaBatch(GlobalBatch):
    def preflight(self):
        anchor = self.state()
        lo, hi = np.rad2deg(REFS[4, 1:])+[1.1, -1.1]
        if not lo <= anchor[4] <= hi:
            target = anchor.copy()
            target[4] = -38.
            self.check([[anchor.tolist(), target.tolist()]])
            anchor = target
        for joint in (1, 2):
            near, far = pair_endpoints(anchor, joint)
            self.check([[near.tolist(), far.tolist()], [far.tolist(), near.tolist()]])
            self.note(stage="inertia_preflight_interval", joint=joint,
                      near_reference_deg=near.tolist(), far_reference_deg=far.tolist(),
                      speed_rad_s=SPEED, ramps_ms=RAMPS_MS, no_motion_sent=True)
        self.note(stage="inertia_preflight_complete", firmware_sha256=FIRMWARE_SHA,
                  no_motion_sent=True, actual_start_requires_recheck=True)

    def execute(self):
        self.previous = {}
        anchor = self.state()
        self.note(stage="inertia_batch_started", firmware_sha256=FIRMWARE_SHA,
                  joints=[1, 2], speed_rad_s=SPEED, ramps_ms=RAMPS_MS,
                  expected_measurement_legs=12, free_space=True,
                  command_components_same_tick=False, full_dynamics_identified=False)
        # Existing observed travel and target inset, not a new entry threshold.
        # This wrist was left near its upper endpoint before flashing.
        lo, hi = np.rad2deg(REFS[4, 1:])+[1.1, -1.1]
        if not lo <= anchor[4] <= hi:
            target = anchor.copy()
            target[4] = -38.
            self.transfer(target, f"{self.args.tag}_wrist_inward")
        for joint in (1, 2):
            near, far = pair_endpoints(self.state(), joint)
            self.check([[near.tolist(), far.tolist()], [far.tolist(), near.tolist()]])
            self.note(stage="inertia_axis_interval", joint=joint,
                      planned_near_reference_deg=near.tolist(),
                      planned_far_reference_deg=far.tolist(),
                      actual_repeat_requires_validation=True)
            for ramp in RAMPS_MS:
                self.args.ramp_ms = ramp
                self.note(stage="inertia_profile", joint=joint, ramp_ms=ramp,
                          nominal_peak_acceleration_rad_s2=1.875*SPEED/(ramp*.001))
                for direction, endpoint in (("out", far[joint-1]), ("back", near[joint-1])):
                    self.leg(joint, float(endpoint), SPEED,
                             f"{self.args.tag}_j{joint}_r{ramp:04d}_{direction}")
                self.note(stage="inertia_pair_complete", joint=joint, ramp_ms=ramp)
                print(f"INERTIA J{joint} ramp {ramp} ms: pair complete", flush=True)
        self.state()
        self.note(stage="inertia_batch_complete", completed_measurement_legs=12,
                  full_dynamics_identified=False, parameters_changed=False)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", required=True)
    p.add_argument("--directory", type=Path, default=ROOT/"build/v12-friction-global-20261123")
    p.add_argument("--log", type=Path, required=True)
    p.add_argument("--tag", default="inertia01",
                   help="explicit new batch identity; old failed captures are never replayed")
    action = p.add_mutually_exclusive_group(required=True)
    action.add_argument("--execute", action="store_true")
    action.add_argument("--preflight-only", action="store_true", help="read/plan only, no motor request")
    p.add_argument("--free-space", action="store_true")
    args = p.parse_args()
    if not re.fullmatch(r"[a-z0-9_]+", args.tag) or (args.execute and not args.free_space):
        p.error("A valid explicit batch identity and --free-space for execution are required")
    binary = args.directory/"ylamp_v12_five_axis.bin"
    if hashlib.sha256(binary.read_bytes()).hexdigest() != FIRMWARE_SHA:
        raise ValueError("First-stage batch is bound to verified deployed 9EC6")
    if args.execute and list(args.directory.glob(f"{args.tag}_*.jsonl")):
        raise FileExistsError("Existing first-stage capture: no automatic replay")
    args.timing_address = verify_timing_layout(
        args.directory/"ylamp_v12_five_axis.elf", "arm-none-eabi-nm", "arm-none-eabi-objdump")
    args.openocd = "D:/OPENOCD/openocd-custom-20260922/bin/openocd.exe"
    args.openocd_scripts = "D:/OPENOCD/openocd-custom-20260922/share/openocd/scripts"
    args.source_clock_only = False
    args.replan_unsent = False
    with args.log.open("x", encoding="utf-8") as log:
        batch = InertiaBatch(args, log)
        try:
            if args.preflight_only:
                batch.preflight()
            else:
                batch.execute()
        except BaseException as exc:
            batch.note(stage="inertia_batch_abort", error=str(exc), no_retry_or_recovery=True)
            raise
        finally:
            batch.geometry.stdin.close()
            batch.geometry.wait(timeout=10)


if __name__ == "__main__":
    main()
