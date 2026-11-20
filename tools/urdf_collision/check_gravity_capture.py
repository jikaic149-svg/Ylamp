"""Compare five-axis board model telemetry to native MuJoCo at the logged pose.

Reads completed status captures only. This checks the model calculation, not
the physical load or the motor command. Status scalars and encoders are read
asynchronously; their discrepancy is reported, not treated as a torque fit.
"""
import argparse
import ctypes
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from export_five_profile import ROOT, SCENE_SHA, pinned


def pose_diagnostics(software_deg):
    """Nominal CAD contributions and local sensitivity, NOT measured gravity."""
    from five_identify import HW, SCENE, regressors

    software = np.asarray(software_deg, dtype=float)
    if software.shape != (5,) or not np.isfinite(software).all():
        raise ValueError("Expected five finite software angles in degrees")
    q = np.radians(software + HW[:, 6])
    step_deg = .1
    delta = np.radians(step_deg)*np.eye(5)
    poses = np.vstack((q, q+delta, q-delta))
    zero = np.zeros_like(poses)
    fixed, y, _ = regressors(poses, zero, zero)
    contributions = np.concatenate((fixed[:, :, None], y[:, :, :4]), axis=2)
    gravity = contributions.sum(axis=2)
    sensitivity = ((gravity[1:6]-gravity[6:])/(2*step_deg)).T
    return dict(cad_deg=np.degrees(q).tolist(),
                link_mass_kg=SCENE[:, -2].tolist(),
                gravity_nm=gravity[0].tolist(),
                link_contributions_nm_rows_joint_cols_body=contributions[0].tolist(),
                sensitivity_nm_per_deg_rows_torque_cols_angle=sensitivity.tolist(),
                sensitivity_step_deg=step_deg)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--sim-dir", required=True, type=Path)
    p.add_argument("--output", type=Path)
    p.add_argument("captures", nargs="+", type=Path)
    args = p.parse_args()
    sys.path.insert(0, str(ROOT/"tools"))
    from five_control import CONTROL_VERSION, EXPECTED, gravity_output_confirmed
    from five_identify import HW
    from dm_monitor import JOINT_REFERENCES, JointReference
    sys.path.insert(0, str(args.sim_dir.resolve()))
    import prepare

    pinned(prepare.SCENE, SCENE_SHA)
    dll = prepare.native_library()
    ptr, num = ctypes.c_void_p, ctypes.c_double
    vec = num*5
    dll.mj_rne.argtypes = [ptr, ptr, ctypes.c_int, ctypes.POINTER(num)]
    dll.mj_rne.restype = None
    error = ctypes.create_string_buffer(4096)
    model = dll.mj_loadXML(str(prepare.SCENE).encode(), None, error, len(error))
    if not model:
        raise RuntimeError(error.value.decode(errors="replace"))
    data = None
    runs = []
    try:
        data = dll.mj_makeData(model)
        if not data or any(dll.mj_stateSize(model, state) != 5 for state in (2, 4)):
            raise ValueError("Expected five joint positions and velocities")
        for path in args.captures:
            raw = path.read_bytes()
            records = [json.loads(line) for line in raw.splitlines()]
            head = records[0]
            if (head["stage"] != "capture" or head["action"] != "status" or
                    head["control_version"] != CONTROL_VERSION or records[-1]["stage"] != "end"):
                raise ValueError(f"{path}: expected a complete five-axis status capture")
            if len(head["references_rad"]) != 5:
                raise ValueError(f"{path}: missing board calibration")
            refs = [JointReference(*r, JOINT_REFERENCES[j].kind, "board")
                    for j,r in enumerate(head["references_rad"], 1)]
            angles, board, sim, fb, ticks = [], [], [], [], []
            for s in records:
                if s["stage"] != "sample":
                    continue
                js = s["telemetry"]["joints"]
                if (not gravity_output_confirmed(s) or not all(
                        j["received"] and j["fresh"] and j["state"] == 1 for j in js) or
                        [(j["joint"], j["bus"], j["protocol"], j["command_id"], j["feedback_id"])
                         for j in js] != EXPECTED):
                    raise ValueError(f"{path}: feedback/state not valid")
                software = np.array([r.angle(j["position_rad"]) for r, j in zip(refs, js)])
                q = software + np.radians(HW[:, 6])
                dll.mj_resetData(model, data)
                dll.mj_setState(model, data, vec(*q), 2)
                dll.mj_forward(model, data)
                g = vec()
                dll.mj_rne(model, data, 0, g)
                angles.append(np.degrees(software))
                board.append(s["model_gravity_nm"])
                sim.append(list(g))
                fb.append([j["torque_nm"] for j in js])
                ticks.append(s["telemetry"]["tick_ms"])
            if len(ticks) < 2:
                raise ValueError(f"{path}: too few observations")
            runs.append(dict(file=path.name, sha256=hashlib.sha256(raw).hexdigest(),
                             status_samples=len(ticks), span_ms=(ticks[-1]-ticks[0]) & 0xffffffff,
                             software_deg_median=np.median(angles, axis=0).tolist(),
                             software_deg_ptp=np.ptp(angles, axis=0).tolist(),
                             board_g_nm_median=np.median(board, axis=0).tolist(),
                             board_g_nm_ptp=np.ptp(board, axis=0).tolist(),
                             mujoco_g_nm_median=np.median(sim, axis=0).tolist(),
                             abs_difference_nm_max=np.max(np.abs(np.subtract(board, sim)), axis=0).tolist(),
                             drive_feedback_nm_median=np.median(fb, axis=0).tolist(),
                             representative_pose=pose_diagnostics(np.median(angles, axis=0))))
    finally:
        if data:
            dll.mj_deleteData(data)
        dll.mj_deleteModel(model)
    out = json.dumps(dict(scene_sha256=SCENE_SHA,
                         scope="Calculation check only; CAD coordinates, masses and COM remain "
                               "unverified on hardware. Drive feedback is not measured load torque.",
                         runs=runs), indent=2, allow_nan=False)
    if args.output:
        with args.output.open("x", encoding="utf-8") as f:
            f.write(out+"\n")
    print(out)


if __name__ == "__main__":
    main()
