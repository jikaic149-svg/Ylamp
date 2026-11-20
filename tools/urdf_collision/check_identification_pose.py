"""Compare the compiled five-axis inverse dynamics to native MuJoCo. Offline only.

Build --library from Model/five_dynamics.c and config/five_mit.c with the host
compiler and the project's host include paths (see README). No device I/O.
"""
import argparse
import ctypes as ct
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from export_five_profile import ROOT, SCENE_SHA, pinned


def yaw_audit(dynamic, firmware, postures, offset, yaw_deg):
    """Vary yaw only. Use native bias without passive damping, contact or limits."""
    z = np.zeros(5)
    spin = np.array([1., 0., 0., 0., 0.])  # rad/s, also unit J1 acceleration
    rest = np.array([0., -.3, .2, .4, -.2])
    def native(q, v):
        m = dynamic.terms(q, v, z)["matrix"]
        return m, np.array(dynamic.bias)

    max_g = max_m = max_h = max_spin_sym = max_c_error = max_abs_g1 = 0.
    examples = []
    for pose in postures:
        def cad(yaw):
            q = np.radians(pose + offset)
            q[0] = np.radians(yaw + offset[0])
            return q
        m0, g0 = native(cad(0), z)
        _, total0 = native(cad(0), spin + rest)
        for yaw in yaw_deg:
            q = cad(yaw)
            m, g = native(q, z)
            _, b1 = native(q, spin)
            _, bn = native(q, -spin)
            _, b2 = native(q, 2 * spin)
            _, br = native(q, rest)
            _, total = native(q, spin + rest)
            h1, cross = b1 - g, total - br - b1 + g
            max_abs_g1 = max(max_abs_g1, abs(g[0]))
            max_g = max(max_g, abs(g - g0).max())
            max_m = max(max_m, abs(m - m0).max())
            max_h = max(max_h, abs((total - g) - (total0 - g0)).max())
            max_spin_sym = max(max_spin_sym, abs(bn - b1).max(), abs((b2 - g) - 4*h1).max())
            for v, a, expected in ((z, z, g), (z, spin, g + m[:, 0]),
                                   (spin, z, b1), (-spin, z, bn), (rest, z, br),
                                   (spin + rest, z, total)):
                max_c_error = max(max_c_error, abs(firmware(q, v, a) - expected).max())
        examples.append(dict(software_deg=pose.tolist(), M11_kg_m2=m[0, 0],
                             unit_j1_accel_nm=m[:, 0].tolist(), pure_j1_spin_nm=h1.tolist(),
                             mixed_j1_velocity_nm=cross.tolist()))
    if (max(max_abs_g1, max_g, max_m, max_h, max_spin_sym) > 1e-10 or max_c_error > 5e-6
            or np.ptp([p["M11_kg_m2"] for p in examples]) < 1e-5
            or max(abs(x) for p in examples for x in p["pure_j1_spin_nm"][1:]) < 1e-5
            or max(abs(x) for p in examples for x in p["mixed_j1_velocity_nm"]) < 1e-5):
        raise ValueError("J1 yaw invariance, inertia/velocity excitation or native/C parity failed")
    return dict(yaw_deg=yaw_deg, rest_velocity_rad_s=rest.tolist(), examples=examples,
                max_abs_g1_nm=max_abs_g1, max_G_yaw_change_nm=max_g,
                max_M_yaw_change_kg_m2=max_m, max_h_yaw_change_nm=max_h,
                max_spin_even_quadratic_error_nm=max_spin_sym, max_native_C_error_nm=max_c_error,
                note="Rigid-body calculations, not motion, friction identification or collision checks.")


def audit(args):
    sys.path.insert(0, str(ROOT / "tools"))
    from dm_monitor import JOINT_REFERENCES
    sys.path.insert(0, str(args.sim_dir.resolve()))
    import prepare
    import tune_mit
    from dynamics_model import DynamicFeedforward, GravityModel

    lib = ct.CDLL(str(args.library.resolve()))
    vector = ct.c_float * 5
    inv = lib.FiveDynamics_Inverse
    inv.argtypes, inv.restype = [ct.POINTER(ct.c_float)] * 4, ct.c_bool

    def firmware(q, v, a):
        out = vector()
        if not inv(vector(*q), vector(*v), vector(*a), out):
            raise ValueError("C inverse dynamics rejected finite input")
        return np.array(out, dtype=float)

    axes = np.array(((ct.c_float * 7) * 5).in_dll(lib, "g_five_mit_sim"))
    boxes = np.array((((ct.c_double * 5) * 2) * 3).in_dll(lib, "g_j25_preview_boxes"))
    snapshot = json.loads(args.snapshot.read_text(encoding="utf-8-sig"))["five_axis_snapshot"]
    joints = sorted(snapshot["joints"], key=lambda row: row["joint"])
    if (len(joints) != 5 or [j["joint"] for j in joints] != [1, 2, 3, 4, 5] or
            not all(j["fresh"] and j["received"] and j["sequence"] == snapshot["sequence"]
                    for j in joints) or max(j["age_ms"] for j in joints) > 100 or
            np.ptp([j["age_ms"] for j in joints]) > 20):
        raise ValueError("Expected one complete, fresh five-axis device snapshot")
    sw = np.degrees([JOINT_REFERENCES[j["joint"]].angle(j["position_rad"]) for j in joints])
    if not np.isfinite(sw).all():
        raise ValueError("Invalid encoder angles")
    rng = np.random.default_rng(20261118)
    poses = np.vstack([sw, rng.uniform(*boxes[0], size=(32, 5))])
    root = ET.fromstring(pinned(prepare.SCENE, SCENE_SHA))
    dll = prepare.native_library()
    dll.mj_versionString.argtypes, dll.mj_versionString.restype = [], ct.c_char_p
    version = dll.mj_versionString().decode()
    sim = tune_mit.Simulation(dll, root, tune_mit.gains(root))
    maxima = np.zeros(4)
    least_eig = np.inf
    dynamic = None
    try:
        dynamic = DynamicFeedforward(dll, sim.model, GravityModel())
        z, unit = np.zeros(5), np.eye(5)
        for i, software_deg in enumerate(poses):
            q = np.radians(software_deg + axes[:, -1])
            v, a = rng.uniform(-.5, .5, 5), rng.uniform(-1.5, 1.5, 5)
            mass = dynamic.terms(q, z, z)["matrix"]
            g = np.array(dynamic.bias)  # native mj_rne(q, 0), not analytic G
            dynamic.terms(q, v, a)
            bias = np.array(dynamic.bias)  # native mj_rne(q, v)
            g_c = firmware(q, z, z)
            m_c = np.column_stack([firmware(q, z, e) - g_c for e in unit])
            tau = firmware(q, v, a)
            errors = [g_c - g, m_c - mass, firmware(q, v, z) - g_c - (bias - g),
                      tau - (mass @ a + bias)]
            maxima = np.maximum(maxima, [abs(d).max() for d in errors])
            if not np.allclose(mass, mass.T, atol=1e-12, rtol=0):
                raise ValueError("Nonsymmetric MuJoCo mass matrix")
            least_eig = min(least_eig, np.linalg.eigvalsh(mass).min())
            if i == 0:
                measured = dict(software_deg=software_deg.tolist(), cad_deg=np.degrees(q).tolist(),
                                native_gravity_nm=g.tolist(), mass_matrix_kg_m2=mass.tolist(),
                                diag_kg_m2=np.diag(mass).tolist(),
                                inside_any_preview_box=bool(np.any(np.all(
                                    (sw >= boxes[:, 0]) & (sw <= boxes[:, 1]), axis=1))))
        j1 = JOINT_REFERENCES[1]
        yaw = yaw_audit(dynamic, firmware, np.vstack([sw, boxes.mean(axis=1)]), axes[:, -1],
                        [np.degrees(j1.lower_rad), -180., -90., 0., sw[0], 90., 180.,
                         np.degrees(j1.upper_rad)])
    finally:
        if dynamic is not None:
            dynamic.close()
        sim.close()
    if not least_eig > 0 or not (maxima < 5e-6).all():
        raise ValueError(f"Model parity failed: maxima={maxima}, min eigenvalue={least_eig}")
    return dict(scope="Model-to-model parity only; measured angles, not measured load or friction",
                scene_sha256=SCENE_SHA, native_version=version,
                snapshot_sha256=hashlib.sha256(args.snapshot.read_bytes()).hexdigest(),
                library_sha256=hashlib.sha256(args.library.read_bytes()).hexdigest(),
                random_seed=20261118, pose_count=len(poses), snapshot=measured, yaw_audit=yaw,
                max_abs_error=dict(zip(("gravity_nm", "M_element_kg_m2", "velocity_bias_nm",
                                        "inverse_torque_nm"), maxima.tolist())),
                minimum_native_mass_eigenvalue_kg_m2=float(least_eig))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for flag in ("sim-dir", "library", "snapshot", "output"):
        p.add_argument("--" + flag, type=Path, required=True)
    args = p.parse_args()
    result = audit(args)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False), encoding="utf-8")
    print("Model parity:", result["max_abs_error"])
    print("Measured-pose prediction: G [Nm]", result["snapshot"]["native_gravity_nm"],
          "diag(M) [kg m2]", result["snapshot"]["diag_kg_m2"])
