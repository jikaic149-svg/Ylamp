"""Offline J3 model audit and zero-PD counterfactuals; never accesses hardware.

Reuses the installed MuJoCo and existing gravity model. Perturbations are
hypotheses, NOT fitted parameters. Dynamic copies have no contacts or stops;
their excursions are not certified hardware paths.
"""
import argparse
import copy
import ctypes
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

from export_five_profile import ROOT, SCENE_SHA, pinned
from solidworks_plan import from_sw, origin


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def refresh(model):
    model.subtree_mass = np.cumsum(model.mass[::-1])[::-1]
    model.first_moment = model.mass[:, None]*model.com
    return model


def urdf_model(nominal, robot):
    """Read original URDF transforms directly, not the rounded MJCF export."""
    model = copy.deepcopy(nominal)
    for i, joint in enumerate(robot.findall("joint")):
        transform = origin(joint.find("origin"))
        model.translations[i] = transform[:3, 3]
        model.rotations[i] = transform[:3, :3]
        assert joint.find("axis").get("xyz") == "0 -1 0"
        inertial = robot.find("link[@name='%s']/inertial" % joint.find("child").get("link"))
        assert inertial.find("origin").get("rpy", "0 0 0") == "0 0 0"
        model.mass[i] = float(inertial.find("mass").get("value"))
        model.com[i] = origin(inertial.find("origin"))[:3, 3]
    assert np.all(model.mass > 0)
    return refresh(model)


def native_frame_model(nominal, robot, plan):
    """A geometry-only hypothesis preserving URDF zero-pose world COMs.

    Motor component frames are a CAD reference, not measured shaft metrology.
    This does NOT reconstruct independently measured component COMs.
    """
    parts = {c["part"]: c for c in plan["displayPoses"][0]["components"]}
    native = [from_sw(parts[p]["transform"]) for p in
              ("part7", "part7", "part6", "part5", "part2", "part10")]
    native = [np.linalg.inv(native[0])@t for t in native]
    urdf = [np.eye(4)]
    model, rows = copy.deepcopy(nominal), []
    for i, joint in enumerate(robot.findall("joint")):
        urdf.append(urdf[-1]@origin(joint.find("origin")))
        delta = native[i+1][:3, 3]-urdf[-1][:3, 3]
        axis = urdf[-1][:3, :3]@np.array([0., -1., 0.])
        relative = urdf[-1][:3, :3].T@native[i+1][:3, :3]
        angle = math.degrees(math.acos(np.clip((np.trace(relative)-1)/2, -1, 1)))
        rows.append(dict(joint=i+1, origin_delta_mm=(delta*1000).tolist(),
                         radial_axis_distance_mm=float(np.linalg.norm(delta-axis*(axis@delta))*1000),
                         zero_frame_rotation_difference_deg=angle))
        transform = np.linalg.inv(native[i])@native[i+1]
        model.translations[i], model.rotations[i] = transform[:3, 3], transform[:3, :3]
        model.com[i] = (np.linalg.inv(native[i+1])@urdf[-1]@np.r_[nominal.com[i], 1])[:3]
    return refresh(model), rows


def capture_pose(path):
    from dm_monitor import JOINT_REFERENCES, JointReference
    records = [json.loads(line) for line in path.read_bytes().splitlines()]
    assert records[0]["stage"] == "capture" and records[-1]["stage"] == "end"
    refs = [JointReference(*r, JOINT_REFERENCES[j].kind, "board")
            for j, r in enumerate(records[0]["references_rad"], 1)]
    samples = [r for r in records if r["stage"] == "sample"]
    angles = np.array([[math.degrees(ref.angle(j["position_rad"]))
                        for ref, j in zip(refs, r["telemetry"]["joints"])] for r in samples])
    assert angles.shape[1] == 5 and np.isfinite(angles).all()
    return np.median(angles, axis=0), dict(
        source=str(path), sha256=sha(path), status_samples=len(samples),
        software_deg_median=np.median(angles, axis=0).tolist(),
        software_deg_ptp=np.ptp(angles, axis=0).tolist(),
        scope="Recorded pose only; no gravity labels or parameter fitting.")


def physics_copy(scene):
    root = ET.fromstring(scene)
    for tag in ("actuator", "sensor", "asset"):
        element = root.find(tag)
        if element is not None:
            root.remove(element)
    for parent in root.iter():
        for element in list(parent):
            if element.tag == "geom":
                parent.remove(element)
    for joint in root.findall(".//joint"):
        joint.set("limited", "false")
        joint.set("damping", "0")
        joint.set("frictionloss", "0")
    actuators = ET.SubElement(root, "actuator")
    for j in range(1, 6):
        ET.SubElement(actuators, "motor", joint=f"J{j}", gear="1")
    return root


class Native:
    def __init__(self, dll, root):
        self.dll, self.model, self.data = dll, None, None
        error = ctypes.create_string_buffer(4096)
        spec = dll.mj_parseXMLString(ET.tostring(root, encoding="utf-8"), None, error, len(error))
        if not spec:
            raise RuntimeError(error.value.decode(errors="replace"))
        try:
            self.model = dll.mj_compile(spec, None)
            if not self.model:
                raise RuntimeError(dll.mjs_getError(spec).decode(errors="replace"))
        finally:
            dll.mj_deleteSpec(spec)
        self.data = dll.mj_makeData(self.model)
        assert self.data and dll.mj_stateSize(self.model, 2) == 5

    def close(self):
        if self.data:
            self.dll.mj_deleteData(self.data)
        if self.model:
            self.dll.mj_deleteModel(self.model)

    def set(self, values, flag):
        self.dll.mj_setState(self.model, self.data, (ctypes.c_double*5)(*values), flag)

    def get(self, flag):
        result = (ctypes.c_double*5)()
        self.dll.mj_getState(self.model, self.data, result, flag)
        return np.array(result)

    def gravity(self, q):
        self.dll.mj_resetData(self.model, self.data)
        self.set(q, 2)
        self.dll.mj_forward(self.model, self.data)
        result = (ctypes.c_double*5)()
        self.dll.mj_rne(self.model, self.data, 0, result)
        return np.array(result)


def jacobian(model, q):
    step = np.radians(.01)
    return np.column_stack([(model.gravity(q+e*step)-model.gravity(q-e*step))/(2*step)
                            for e in np.eye(5)])


def loop_work(model, q):
    """Virtual 5-degree J2/J3 rectangle, not a motion plan."""
    e2, e3 = np.radians(5)*np.eye(5)[[1, 2]]
    start = q-(e2+e3)/2
    corners = [start, start+e2, start+e2+e3, start+e3, start]
    work = np.zeros(2)
    for a, b in zip(corners, corners[1:]):
        torques = np.array([model.gravity(a+(b-a)*s) for s in np.linspace(0, 1, 401)])
        average = (torques[:-1]+torques[1:])/2
        work[0] += np.sum(average@(b-a))/400
        average[:, 2] *= 2
        work[1] += np.sum(average@(b-a))/400
    assert abs(work[0]) < 1e-9 and abs(work[1]) > 1e-4
    return dict(nominal_closed_loop_joule=float(work[0]), x2_closed_loop_joule=float(work[1]))


def dynamic_case(sim, model, q0, budgets, scale=1., phase_deg=0., pd=False):
    sim.dll.mj_resetData(sim.model, sim.data)
    sim.set(q0, 2)
    kp = np.array([140, 130, 150, 90, 60]) if pd else np.zeros(5)
    kd = np.array([5, 5, 3, 1, .75]) if pd else np.zeros(5)
    phase = np.eye(5)[2]*np.radians(phase_deg)
    peak, clipped = np.zeros(5), np.zeros(5, dtype=int)
    for _ in range(500):
        q, v = sim.get(2), sim.get(4)
        command = model.gravity(q+phase)
        command[2] *= scale
        command += kp*(q0-q)-kd*v
        clipped += np.abs(command) > budgets
        sim.set(np.clip(command, -budgets, budgets), 64)
        sim.dll.mj_step(sim.model, sim.data)
        assert np.isfinite(sim.get(2)).all()
        peak = np.maximum(peak, np.abs(np.degrees(sim.get(2)-q0)))
    return dict(seconds=1., kp=kp.tolist(), kd=kd.tolist(), j3_g_scale=scale,
                controller_j3_phase_error_deg=phase_deg,
                peak_excursion_deg=peak.tolist(),
                final_excursion_deg=np.degrees(sim.get(2)-q0).tolist(),
                final_speed_rad_s=sim.get(4).tolist(), clipped_steps=clipped.tolist())


def audit(args):
    sys.path[:0] = [str(args.sim_dir.resolve()), str(ROOT/"tools")]
    import prepare
    from gravity_model import GravityModel
    from five_identify import HW
    from workspace_sim import upright
    from check_gravity_capture import pose_diagnostics

    nominal = GravityModel()
    pinned(prepare.SCENE, SCENE_SHA)
    urdf_path = ROOT.parent/"Ylamp-URDF-package(1)/Ylamp.urdf"
    plan_path = ROOT/"build/solidworks_collision_v12/native_test_plan_v2.json"
    inputs = {str(p): sha(p) for p in (urdf_path, prepare.SCENE, plan_path, args.capture,
        ROOT/"config/five_scene.inc", ROOT/"config/five_mit.c", ROOT/"config/joint_references.def",
        Path(__file__).resolve(), args.sim_dir/"gravity_model.py", args.sim_dir/"prepare.py")}
    robot = ET.parse(urdf_path).getroot()
    direct = urdf_model(nominal, robot)
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    assert plan["urdfSha256"] == sha(urdf_path)
    native_geometry, frames = native_frame_model(direct, robot, plan)
    software, capture = capture_pose(args.capture)
    q = np.radians(software+HW[:, 6])
    poses = np.vstack([q, np.radians(HW[:, 6]), np.zeros(5),
                       np.random.default_rng(20261118).uniform(nominal.ranges[:, 0],
                                                              nominal.ranges[:, 1], (200, 5))])
    dll = prepare.native_library()
    dll.mj_rne.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int,
                          ctypes.POINTER(ctypes.c_double)]
    dll.mj_rne.restype = None
    xml = physics_copy(nominal.source)
    sim = Native(dll, xml)
    try:
        errors = np.zeros(4)
        for point in poses:
            g = nominal.gravity(point)
            step = 1e-6
            gradient = np.array([(direct.potential(point+e*step)-direct.potential(point-e*step))/(2*step)
                                 for e in np.eye(5)])
            errors = np.maximum(errors, [
                np.max(np.abs(direct.gravity(point)-g)),
                np.max(np.abs(direct.geometric_gravity(point)-gradient)),
                np.max(np.abs(sim.gravity(point)-g)),
                np.max(np.abs(direct.gravity(point)-direct.geometric_gravity(point)))])
        assert np.max(errors) < 2e-6
        variants = []
        base = nominal.gravity(q)
        for name, model in (("STEP_frame_reference_preserve_zero_COM", native_geometry),):
            differences = np.array([model.gravity(p)-direct.gravity(p) for p in poses])
            variants.append(dict(hypothesis=name, current_delta_nm=(model.gravity(q)-base).tolist(),
                                 sampled_max_abs_delta_nm=np.max(np.abs(differences), axis=0).tolist()))
        geometry_xml = copy.deepcopy(xml)
        dll.mju_mat2Quat.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double)]
        dll.mju_mat2Quat.restype = None
        for i in range(5):
            body = geometry_xml.find(f".//body[@name='link_j{i+1}']")
            quaternion = (ctypes.c_double*4)()
            matrix = (ctypes.c_double*9)(*native_geometry.rotations[i].ravel())
            dll.mju_mat2Quat(quaternion, matrix)
            body.set("quat", " ".join(map(str, quaternion)))
            body.set("pos", " ".join(map(str, native_geometry.translations[i])))
            body.find("inertial").set("pos", " ".join(map(str, native_geometry.com[i])))
        geometry_sim = Native(dll, geometry_xml)
        try:
            geometry_error = max(np.max(np.abs(geometry_sim.gravity(p)-native_geometry.gravity(p)))
                                 for p in poses)
            assert geometry_error < 1e-10
        finally:
            geometry_sim.close()
        perturbations = []
        for joint, angles in ((1, (-5, 5)), (2, (-10, -5, 5, 10)), (3, (-10, 10)), (4, (-10, 10))):
            for degrees in angles:
                trial = q+np.eye(5)[joint]*np.radians(degrees)
                perturbations.append(dict(hypothesis=f"J{joint+1}_phase_{degrees:+}deg",
                                          delta_nm=(nominal.gravity(trial)-base).tolist()))
        for body in (2, 3, 4):
            for factor in (.8, 1.2):
                trial = copy.deepcopy(nominal)
                trial.mass[body] *= factor
                refresh(trial)
                perturbations.append(dict(hypothesis=f"link_J{body+1}_mass_x{factor}",
                                          delta_nm=(trial.gravity(q)-base).tolist()))
        for axis in (0, 2):
            for delta in (-.01, .01):
                trial = copy.deepcopy(nominal)
                trial.com[2, axis] += delta
                refresh(trial)
                perturbations.append(dict(hypothesis=f"link_J3_COM_{'xyz'[axis]}_{delta*1000:+}mm",
                                          delta_nm=(trial.gravity(q)-base).tolist()))
        doubled_mechanics = copy.deepcopy(xml)
        for element in doubled_mechanics.findall(".//inertial"):
            element.set("fullinertia", " ".join(str(2*float(v)) for v in element.get("fullinertia").split()))
        for joint in doubled_mechanics.findall(".//joint"):
            joint.set("armature", str(2*float(joint.get("armature", "0"))))
        other = Native(dll, doubled_mechanics)
        try:
            inertial_error = np.max([np.max(np.abs(other.gravity(p)-sim.gravity(p))) for p in poses[:3]])
            assert inertial_error < 1e-12
        finally:
            other.close()
        curves = []
        for label, point in (("recorded_wrist", q), ("software_zero", np.radians(HW[:, 6]))):
            rows = []
            for degrees in (-30, -15, 0, 15, 30):
                trial = point.copy()
                trial[2] = np.radians(degrees+HW[2, 6])
                rows.append(dict(j3_software_deg=degrees, nominal_nm=float(nominal.gravity(trial)[2])))
            curves.append(dict(condition=label, samples=rows))
        a, b = base[2], nominal.gravity(q+np.eye(5)[2]*math.pi/2)[2]
        amplitude = math.hypot(a, b)
        phase_candidates = []
        if abs(2*a) <= amplitude:
            phi, target = math.atan2(a, b), math.asin(2*a/amplitude)
            phase_candidates = [((math.degrees(x-phi)+180) % 360)-180
                                for x in (target, math.pi-target)]
            phase = min(phase_candidates, key=abs)
            assert abs(nominal.gravity(q+np.eye(5)[2]*np.radians(phase))[2]-2*a) < 1e-12
        hessian = jacobian(nominal, q)
        scaled = hessian.copy()
        scaled[2] *= 2
        assert np.max(np.abs(hessian-hessian.T)) < 1e-7
        assert abs(scaled[2, 1]-scaled[1, 2]) > .1
        dynamics = [dynamic_case(sim, nominal, q, HW[:, 5], **case) for case in (
            {}, dict(scale=2), dict(scale=2, pd=True),
            dict(phase_deg=5), dict(phase_deg=5, pd=True))]
        assert max(dynamics[0]["peak_excursion_deg"]) < 1e-7
        assert dynamics[1]["peak_excursion_deg"][2] > 1
        assert dynamics[1]["peak_excursion_deg"][2] > 10*dynamics[2]["peak_excursion_deg"][2]
        assert dynamics[3]["peak_excursion_deg"][2] > 10*dynamics[4]["peak_excursion_deg"][2]
    finally:
        sim.close()
    inertia_checks = []
    for link in robot.findall("link"):
        i = link.find("inertial/inertia").attrib
        tensor = np.array([[float(i["ixx"]), float(i["ixy"]), float(i["ixz"])],
                           [float(i["ixy"]), float(i["iyy"]), float(i["iyz"])],
                           [float(i["ixz"]), float(i["iyz"]), float(i["izz"])]])
        eigen = np.linalg.eigvalsh(tensor)
        assert eigen[0] > 0 and eigen[0]+eigen[1] >= eigen[2]
        inertia_checks.append(dict(link=link.get("name"), principal_inertia_kg_m2=eigen.tolist()))
    report = dict(
        scope="OFFLINE ONLY. No model fitting or deployment. Same-CAD checks do not validate reality. "
              "Perturbations are sensitivity hypotheses; x2 is NOT a measured required load. "
              "Dynamic copies remove contacts, stops, damping and all MIT gains unless explicitly listed; "
              "retain nominal CAD masses/COM/inertias and identified armature. Not a workspace certificate.",
        inputs=inputs,
        recorded_pose=capture, representative_pose=pose_diagnostics(software),
        numerical_checks=dict(poses=len(poses),
            direct_urdf_vs_rounded_mjcf_max_nm=float(errors[0]),
            direct_urdf_potential_gradient_max_nm=float(errors[1]),
            native_mujoco_vs_formula_max_nm=float(errors[2]),
            native_geometry_mujoco_vs_formula_max_nm=float(geometry_error),
            direct_urdf_geometric_vs_recursive_max_nm=float(errors[3]),
            doubled_inertia_armature_static_delta_nm=float(inertial_error)),
        inertia_checks=inertia_checks, native_frame_comparison=frames,
        geometry_hypotheses=variants,
        original_upright_offsets_deg=upright(nominal).tolist(),
        native_frame_upright_offsets_deg=upright(native_geometry).tolist(),
        parameter_sensitivities=perturbations, j3_curves=curves,
        j3_local=dict(gravity_nm=float(a), amplitude_with_fixed_other_axes_nm=amplitude,
            derivative_nm_per_rad=float(hessian[2, 2]),
            equivalent_phase_for_x2_at_this_pose_deg=min(phase_candidates, key=abs) if phase_candidates else None,
            minimum_collective_downstream_COM_shift_to_change_G3_by_abs_nominal_mm=
                float(abs(a)/(9.81*nominal.subtree_mass[2])*1000),
            x2_artificial_restoring_stiffness_at_pose_nm_per_rad=float(-hessian[2, 2])),
        gravity_jacobian_nm_per_rad=hessian.tolist(),
        x2_field_cross_partial_asymmetry_nm_per_rad=float(scaled[2, 1]-scaled[1, 2]),
        virtual_loop_work=loop_work(nominal, q),
        dynamic_cases=dynamics)
    for text, digest in report["inputs"].items():
        assert sha(Path(text)) == digest, f"Input changed: {text}"
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps({k: report[k] for k in ("numerical_checks", "geometry_hypotheses",
                                           "j3_local", "dynamic_cases")}, indent=2))
    print("Report:", args.output)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sim-dir", type=Path, required=True)
    parser.add_argument("--capture", type=Path, default=ROOT/
                        "build/identification/five_gravity_impedance_handover_20261118_01.jsonl")
    parser.add_argument("--output", type=Path, required=True, help="New JSON path; refuses overwrite")
    audit(parser.parse_args())
