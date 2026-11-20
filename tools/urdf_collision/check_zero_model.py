"""Offline post-zero-update formula check: compiled C, Python RNE and local CAD.

No serial/SWD, motion, model-file changes or parameter fitting. Uses the
current shared reference-to-CAD mapping; does not establish its physical datum.
"""
import argparse
import ctypes as ct
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))


def review(args):
    from dm_monitor import JOINT_REFERENCES
    from five_identify import HW, NOMINAL, regressors

    sys.path.insert(0, str(args.sim_dir.resolve()))
    from gravity_model import GravityModel
    from dynamics_model import DynamicFeedforward
    import prepare
    import tune_mit

    records = [json.loads(line) for line in args.capture.read_bytes().splitlines()]
    old = np.array(records[0]["references_rad"])[:, 0]
    last = next(r for r in reversed(records) if r.get("stage") == "sample")
    raw = np.array([j["position_rad"] for j in last["telemetry"]["joints"]])
    new = np.array([JOINT_REFERENCES[j].zero_rad for j in range(1, 6)])
    reference = np.array([JOINT_REFERENCES[j].angle(value) + JOINT_REFERENCES[j].zero_rad
                          for j, value in enumerate(raw, 1)])
    q_now = reference - new + np.radians(HW[:, 6])
    previous_cad = (HW[:, 6] if args.previous_cad_deg is None else
                    np.array([float(v) for v in args.previous_cad_deg.split(",")]))
    if previous_cad.shape != (5,) or not np.isfinite(previous_cad).all():
        raise ValueError("Expected five finite previous CAD offsets")
    q_old = reference - old + np.radians(previous_cad)
    rng = np.random.default_rng(20261119)
    poses = np.vstack((q_now, q_old, np.radians(HW[:, 6]), rng.uniform(-2, 2, (100, 5))))
    native_c = ct.CDLL(str(args.library.resolve()))
    vec = ct.c_float * 5
    native_c.FiveDynamics_Inverse.argtypes = [ct.POINTER(ct.c_float)] * 4
    native_c.FiveDynamics_Inverse.restype = ct.c_bool

    def c_inverse(q, v, a):
        out = vec()
        assert native_c.FiveDynamics_Inverse(vec(*q), vec(*v), vec(*a), out)
        return np.array(out)

    model = GravityModel()
    root = ET.fromstring(model.source)
    sim = tune_mit.Simulation(prepare.native_library(), root, tune_mit.gains(root))
    dynamic = DynamicFeedforward(sim.dll, sim.model, model)
    zero = np.zeros(5)
    maxima = dict(c_gravity_nm=0., c_dynamic_nm=0., python_rne_nm=0.,
                  energy_gradient_nm=0., mass_symmetry_kg_m2=0.)
    minimum_eigenvalue = float("inf")
    try:
        for q in poses:
            v, a = rng.uniform(-1, 1, (2, 5))
            terms = dynamic.terms(q, v, a)
            g = terms["gravity"]
            fixed, y, _ = regressors(q[None], v[None], a[None])
            predicted = (fixed + y @ NOMINAL)[0]
            maxima["c_gravity_nm"] = max(maxima["c_gravity_nm"], float(np.max(abs(c_inverse(q, zero, zero)-g))))
            maxima["c_dynamic_nm"] = max(maxima["c_dynamic_nm"], float(np.max(abs(c_inverse(q, v, a)-terms["feedforward"]))))
            maxima["python_rne_nm"] = max(maxima["python_rne_nm"], float(np.max(abs(predicted-terms["feedforward"]))))
            eps = 1e-5
            gradient = np.array([(model.potential(q+e*eps)-model.potential(q-e*eps))/(2*eps)
                                 for e in np.eye(5)])
            maxima["energy_gradient_nm"] = max(maxima["energy_gradient_nm"], float(np.max(abs(g-gradient))))
            matrix = terms["matrix"]
            maxima["mass_symmetry_kg_m2"] = max(maxima["mass_symmetry_kg_m2"], float(np.max(abs(matrix-matrix.T))))
            minimum_eigenvalue = min(minimum_eigenvalue, float(np.linalg.eigvalsh(matrix).min()))
        assert max(maxima.values()) < 1e-5 and minimum_eigenvalue > 0
        assert model.scene.read_bytes() == model.source
    finally:
        dynamic.close()
        sim.close()
    old_g, new_g = model.gravity(q_old), model.gravity(q_now)
    return dict(status="passed_model_consistency_only", poses=len(poses), max_errors=maxima,
                minimum_mass_eigenvalue_kg_m2=minimum_eigenvalue,
                old_zero_deg=np.degrees(old).tolist(), new_zero_deg=np.degrees(new).tolist(),
                old_cad_offsets_deg=previous_cad.tolist(), new_cad_offsets_deg=HW[:, 6].tolist(),
                same_raw_encoder_deg=np.degrees(reference).tolist(),
                new_cad_deg=np.degrees(q_now).tolist(),
                same_pose_old_gravity_nm=old_g.tolist(), same_pose_new_gravity_nm=new_g.tolist(),
                same_pose_gravity_change_nm=(new_g-old_g).tolist(),
                formulas=dict(coordinate="qCAD_rad=(reference_raw_deg-zero_deg+CAD_offset_deg)*pi/180",
                              gravity="G(qCAD)=dU/dqCAD; pure support is +G, Kp=Kd=Fc=B=0",
                              dynamic="tau=G(qCAD)+M(qCAD)*qdd_des+c(qCAD,qd_actual); no double gravity",
                              old_fit_coordinate="delta=z_new-z_old; q_old=q_new+delta",
                              old_sine_reexpression="k*sin(q_old)=k*cos(delta)*sin(q_new)+k*sin(delta)*cos(q_new)",
                              old_harmonic_reexpression="A*cos(n*q_old)+B*sin(n*q_old): A'=A*cos(n*delta)+B*sin(n*delta), B'=-A*sin(n*delta)+B*cos(n*delta)",
                              distinction="Reexpressing an OLD empirical fit preserves its predictions; correcting a calibration in the CAD model changes its physical input phase. Do not do both."),
                unchanged=["J1/J4/J5 zeros", "raw travel endpoints", "rigid-body parameters",
                           "hardware/software gravity gains and torque budgets", "controller lifecycle/failure policy"],
                historical_models=dict(two_axis="Old load/assembly; not linked into V1.2. Do not transplant its sine/friction coefficients.",
                                       idealized_arm5="output/arm5_dof is a separate example, not the current CAD.",
                                       old_fit_reestimated=False,
                                       reason="New angle-only captures contain no torque/load labels. Old identification recordings were removed before this task."),
                scope="Model-vs-model/energy consistency only, not physical torque or mechanical calibration validation.",
                inputs_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
                               (args.capture, args.library, model.scene, ROOT/"config/joint_references.def",
                                ROOT/"config/five_scene.inc", ROOT/"config/five_mit.c",
                                ROOT/"config/five_coordinates.h")})


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("sim-dir", "library", "capture", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--previous-cad-deg", help="Five comma-separated offsets for the actual previous image")
    args = parser.parse_args()
    result = review(args)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps(result, indent=2, allow_nan=False))
