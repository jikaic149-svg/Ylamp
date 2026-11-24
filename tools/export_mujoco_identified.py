"""Export the adopted V1.2 parameters to MJCF, offline; never accesses hardware.

Requires the existing D:/MuJoCo/4.0.1 installation and D:/MuJoCo/Ylamp assets.
The XML executes native mechanics and MIT PD. Nonlinear support/friction and
other control modes are explicitly stored as metadata, NOT silently simulated.
"""
import argparse
import copy
import ctypes as ct
import hashlib
import itertools
import json
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np

from five_identify import HW, ROOT, SCENE, regressors, NOMINAL
from five_gravity_residual_report import support_gravity

SIM = Path("D:/MuJoCo/Ylamp")
BUILD = ROOT / "build/v12-j1-native-impedance-tune-20261123"
OUTPUT = ROOT / "simulation/ylamp_v12_identified.xml"
CONFIG = ROOT / "config/five_mit.c"
DIAGNOSTIC = BUILD / "impedance_diagnostic_01.json"
sys.path.insert(0, str(SIM))
import prepare


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def numbers(values):
    return " ".join(format(float(x), ".17g") for x in np.asarray(values).ravel())


def parameters():
    """Read the production C tables, excluding host-only high-gain fixtures."""
    text = CONFIG.read_text(encoding="utf-8").split("#ifdef YLAMP_J3_REVERSE_TEST")[0]
    result = {}
    for name, body in re.findall(
            r"const\s+\w+\s+(g_five_\w+)(?:\[[^\]]+\])*\s*=\s*([^;]+);", text):
        body = re.sub(r"/\*.*?\*/", "", body, flags=re.S)
        for macro, value in (("J2_CAD", HW[1, 6]), ("J3_CAD", HW[2, 6])):
            body = body.replace(macro, repr(float(value)))
        values = [float(x.strip().rstrip("f")) for x in
                  body.translate(str.maketrans({"{": "", "}": ""})).split(",")
                  if x.strip()]
        # These are const float tables in the deployed image, not binary64 fits.
        result[name] = np.asarray(values, dtype=np.float32).astype(float)
    expected = {
        "g_five_mit_hw": 35, "g_five_mit_tracking": 15,
        "g_five_speed_pi": 10, "g_five_speed_friction": 20,
        "g_five_speed_friction_range_rad_s": 2,
        "g_five_speed_friction_knots_rad_s": 2, "g_five_speed_friction_j2": 2,
        "g_five_drag": 25, "g_five_drag_filter_s": 1, "g_five_drag_ramp_s": 1,
        "g_five_tracking_ramp_s": 1, "g_five_j2_gravity_delta": 20,
        "g_five_j3_gravity_scale": 1, "g_five_j3_gravity_cosine_nm": 1,
        "g_five_support_gains": 10, "g_five_impedance_max_nm": 5,
    }
    if set(result) != set(expected) or any(
            len(result[k]) != n or not np.isfinite(result[k]).all()
            for k, n in expected.items()):
        raise ValueError("Production parameter table layout changed")
    return result


def inputs():
    reference_path = ROOT / "config/joint_references.def"
    rows = re.findall(
        r"^JOINT_REFERENCE\((\d+),\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+),\s*(\w+)\)",
        reference_path.read_text(encoding="utf-8"), re.M)
    if [int(row[0]) for row in rows] != list(range(1, 6)):
        raise ValueError("Expected current five joint references")
    refs = np.array([[float(x) for x in row[1:4]] for row in rows])
    diag = json.loads(DIAGNOSTIC.read_text(encoding="utf-8"))
    if diag["source_sha256"]["config/five_mit.c"] != sha(CONFIG):
        raise ValueError("Current configuration differs from the archived b5d5 diagnostic")
    image = BUILD / "ylamp_v12_five_axis.bin"
    if diag["firmware_sha256"] != sha(image):
        raise ValueError("Archived firmware hash mismatch")
    snapshot = diag["snapshots"][-1]
    if (snapshot["fault"] or snapshot["limited"]
            or any(snapshot[k] != 31 for k in ("fresh", "enabled", "submitted"))):
        raise ValueError("Saved pose does not have healthy complete feedback")
    q = np.asarray(snapshot["reference_rad"]) - np.radians(refs[:, 0]) + np.radians(HW[:, 6])
    bounds = np.radians(refs[:, 1:] - refs[:, :1] + HW[:, 6, None])
    if np.any(q < bounds[:, 0]) or np.any(q > bounds[:, 1]):
        raise ValueError("Saved continuous pose is outside the configured range")
    return refs, bounds, q, diag


def configure_collisions(root):
    """Native SDF volumes preserve concavities; no extra package or new masses.

    ponytail: use MuJoCo's built-in mesh SDF, not a new decomposition pipeline.
    Finite SDF resolution/search and soft contacts are not a CAD safety proof.
    """
    bodies = root.findall(".//body")
    assert [b.get("name") for b in bodies] == ["base_link", *[f"link_j{i}" for i in range(1, 6)]]
    root.set("model", "Ylamp V1.2 b5d5 - SDF collision volumes and native MIT PD")
    root.find("compiler").set("inertiafromgeom", "false")
    option = root.find("option")
    option.attrib.update(sdf_iterations="20", sdf_initpoints="40")
    flag = option.find("flag")
    if flag is None:
        flag = ET.SubElement(option, "flag")
    # Test neighboring main bodies too, including the J4/J5 wrist.
    flag.attrib.update(filterparent="disable", contact="enable")
    count = len(bodies)
    all_bits = (1 << (2 * count + 1)) - 1
    volumes = {}
    for i, body in enumerate(bodies):
        visual = [g for g in body.findall("geom") if g.get("contype") == "0"]
        volumes[body.get("name")] = []
        for original in visual:
            name = "collision_" + original.get("name")
            geom = body.find(f"./geom[@name='{name}']")
            if geom is None:
                geom = copy.deepcopy(original)
                geom.set("name", name)
                body.append(geom)
            hardware = original.get("name").startswith("hardware")
            denied = (1 << i) | (1 << (count + i))
            for neighbor in (i - 1, i + 1):
                if 0 <= neighbor < count:
                    # Fasteners are fixed assembly hardware, not separate links.
                    # Ignore their intended contact with the neighboring joint
                    # assembly, but retain every main-body/main-body pair.
                    denied |= 1 << (count + neighbor)
                    if hardware:
                        denied |= 1 << neighbor
            geom.attrib.update(
                type="sdf", contype=str(1 << (i + (count if hardware else 0))),
                conaffinity=str(all_bits & ~denied), group="3", density="0",
                margin="0", gap="0", condim="3", solref=".008 1",
                solimp=".95 .99 .001", rgba=".2 .65 1 .25")
            volumes[body.get("name")].append(name)
    floor = root.find("./worldbody/geom[@name='floor']")
    floor.attrib.update(contype=str(1 << (2 * count)),
                        conaffinity=str((1 << (2 * count)) - 1),
                        solref=".008 1", solimp=".95 .99 .001")
    sensors = root.find("sensor")
    ET.SubElement(sensors, "contact", name="collision_all",
                  subtree1="base_link", data="found force", num="1")
    for a, b in itertools.combinations(bodies, 2):
        ET.SubElement(sensors, "contact",
                      name=f"collision_{a.get('name')}_{b.get('name')}",
                      body1=a.get("name"), body2=b.get("name"), data="found", num="1")
    custom = root.find("custom")
    scope = custom.find("text[@name='simulation_scope']")
    scope_data = json.loads(scope.get("data"))
    scope_data["executed"].append("Native SDF robot self-contact and floor contact")
    scope_data["collision"] = (
        "24 native mesh SDF volumes on six rigid bodies, including fixed hardware; "
        "all main-body pairs including neighbors enabled. Fixed hardware versus "
        "adjacent joint assemblies is filtered; nonadjacent hardware remains checked. "
        "Finite SDF search/resolution and soft contacts can miss thin intersections; "
        "not exact CAD, continuous collision detection or real-hardware protection.")
    scope.set("data", json.dumps(scope_data, separators=(",", ":")))
    ET.SubElement(custom, "text", name="collision_volumes", data=json.dumps({
        "representation": "MuJoCo 4.0.1 built-in mesh SDF, not filled convex hulls",
        "bodies": volumes, "units": "metres", "visual_group": 1, "collision_group": 3,
        "extra_mass_kg": 0, "parent_main_body_contacts_enabled": True,
        "filtered_contacts": "same rigid body; hardware against adjacent joint assembly",
        "margin_m": 0, "solref_seconds_damping_ratio": [.008, 1],
        "material": "MuJoCo default contact friction, not newly identified",
        "limitations": "Finite SDF approximation; no exact-surface/CCD/cable/safety guarantee",
    }, separators=(",", ":")))
    return root


def make_scene(collisions=False):
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    root = ET.parse(SIM / "ylamp_learning.xml", parser=parser).getroot()
    refs, bounds, q, diag = inputs()
    params = parameters()
    gains = params["g_five_support_gains"].reshape(5, 2)
    # Model the actual 12-bit MIT Kp/Kd encoding, not the old simulation tune.
    encoded = np.floor(gains / [500., 5.] * 4095) / 4095 * [500., 5.]
    root.set("model", "Ylamp V1.2 identified archive b5d5 - native MIT PD plus metadata")
    for child in list(root):
        if child.tag is ET.Comment:
            root.remove(child)
    root.insert(0, ET.Comment(
        "Offline parameter export, 2026-11-23. qpos is CAD radians; "
        "qCAD=continuous_reference-zero+CAD_offset. "
        "Native mechanics and encoded KEY PD execute. Nonlinear G/F, mode "
        "transitions and MCU sampled budget logic require an external controller. "
        "Custom numeric/text fields are data, not executable force laws. "
        "CAD mass/COM/inertia remain nominal; no new full-body inertia fit. "
        "No hardware access; original XML and firmware are unchanged."))
    for i, name in enumerate(prepare.JOINTS):
        joint = root.find(f".//joint[@name='{name}']")
        joint.set("range", numbers(bounds[i]))
        joint.set("limited", "true")
        # Keep unloaded J and B already present, with the N^2 assumption stated.
        # Do not turn sliding F into an unidentified static friction threshold.
        joint.set("frictionloss", "0")
        actuator = root.find(f"./actuator/pid[@joint='{name}']")
        actuator.set("kp", numbers([encoded[i, 0]]))
        actuator.set("kv", numbers([encoded[i, 1]]))
        actuator.set("posrange", numbers(bounds[i]))
        actuator.set("ki", "0")
        # Catalog peak force range is retained, NOT the MCU sampled KEY budget.
        # The latter reduces native gains; it is not a drive hard force clamp.
    root.find("option").set("timestep", ".002")
    custom = root.find("custom")
    for element in list(custom):
        if not element.get("name", "").startswith("damiao_noload_"):
            custom.remove(element)

    def text(name, data):
        if not isinstance(data, str):
            data = json.dumps(data, ensure_ascii=True, separators=(",", ":"), allow_nan=False)
        ET.SubElement(custom, "text", name=name, data=data)

    def numeric(name, data):
        ET.SubElement(custom, "numeric", name=name, data=numbers(data))

    for name, values in params.items():
        numeric(name, values)
    numeric("joint_reference_zero_deg", refs[:, 0])
    numeric("joint_continuous_range_deg", refs[:, 1:])
    numeric("cad_offsets_deg", HW[:, 6])
    numeric("encoded_key_kp_kd", encoded)
    numeric("saved_pose_qCAD_rad", q)
    numeric("nominal_link_mass_kg", SCENE[:, -2])
    numeric("nominal_link_com_m", SCENE[:, 3:6])
    numeric("nominal_armature_kg_m2", SCENE[:, -1])
    # A keyframe initializes one saved offline pose and its G feedforward.
    # Its FF is fixed; it must not be mistaken for G(q) evaluated continuously.
    g = support_gravity(q[None])[0]
    ctrl = np.column_stack((q, np.zeros(5), g)).ravel()
    if root.find("keyframe") is not None:
        root.remove(root.find("keyframe"))
    keys = ET.SubElement(root, "keyframe")
    ET.SubElement(keys, "key", name="saved_pose_G_at_pose_only",
                  qpos=numbers(q), qvel=numbers(np.zeros(5)), ctrl=numbers(ctrl))
    text("parameter_schema", {
        "order": "J1,J2,J3,J4,J5; C tables flattened row-major",
        "units": "qpos/ctrl position rad CAD; velocity rad/s; torque Nm",
        "g_five_mit_hw": "kp,kd,Fc,B,smoothing_rad_s,pure_G_cap_Nm,CAD_offset_deg",
        "g_five_mit_tracking": "software_kp,kd,active_total_budget_Nm",
        "g_five_speed_pi": "P_Nm_s_rad,I_Nm_rad; moving cruise axis only",
        "g_five_speed_friction": "log_A,slope,hinge_016,hinge_048",
        "g_five_j2_gravity_delta": "effective mass,hx,hy,hz; apply ONLY J2 torque",
        "g_five_drag": "fraction,damping,deadband,assist_cap,total_budget",
        "g_five_support_gains": "configured KEY kp,kd; native XML uses encoded_key_kp_kd",
        "g_five_impedance_max_nm": "MCU predicted KEY budget; NOT a native hard clamp",
        "ctrl_layout": "15 inputs: J1 pos,vel,ff; J2 pos,vel,ff; ...",
        "joint_continuous_range_deg": "min,max per axis; J4 never modulo 2*pi",
        "friction_smoothing": "g_five_mit_hw column5, currently .02rad/s",
    })
    text("support_gravity_law",
         "G=G_CAD; G2+=(Yg(qCAD)*g_five_j2_gravity_delta.ravel())[J2]; "
         "G3=g_five_j3_gravity_scale*G_CAD3+g_five_j3_gravity_cosine_nm*"
         "cos(qCAD3-radians(cad_offsets_deg[J3])); clip each axis at g_five_mit_hw[:,5]. "
         "Yg is the nominal full-chain gravity potential basis (mass,hx,hy,hz per link), "
         "with world gravity=(0,0,-9.81) / base-frame gravity=(0,9.81,0). "
         "Effective residuals do not change physical masses, M(q), or c(q,v). "
         "Reference implementation: tools/five_gravity_residual_report.py:support_gravity.")
    text("total_friction_law",
         "s=clip(abs(v_reference),.08,.8); "
         "logA=c0+c1*log(s/.08)+c2*max(0,log(s/.16))+c3*max(0,log(s/.48)); "
         "J2 adds g_five_speed_friction_j2[0]*nominal_bending_Nm + "
         "g_five_speed_friction_j2[1]*abs(G_CAD2); "
         "F=exp(logA)*tanh(v_reference/.02). "
         "nominal_bending_Nm=norm((I-aa^T)*subtree_gravity_moment) at J2. "
         "Total F replaces, not adds to, previous friction fits. "
         "Application: moving cruise axis; partial filtered measured-speed J1/J2 drag. "
         "KEY hold, pure G, held axes and ordinary Move do not add F. "
         "NOT applied by this XML. Do not sum with noload B as a second identified total loss. "
         "Reference implementation: Mode/five_mit.c:FiveMIT_SpeedFrictionTorque.")
    text("simulation_scope", {
        "executed": ["CAD mass/COM/inertia", "unloaded N^2 armature and reference-only viscous B",
                     "current recorded ranges mapped to CAD radians", "encoded native KEY PD",
                     "catalog peak actuator saturation", "fixed FF stored in saved-pose keyframe"],
        "data_only": ["full current G/F coefficients and formulas", "speed PI/active gains",
                      "KEY predicted budgets and drag parameters", "unloaded electrical R/L/flux/settings"],
        "not_identified": ["static friction thresholds", "backlash", "compliance",
                           "closed-loop electrical/thermal behavior", "full-body reidentified M/c"],
        "not_implemented": ["live G/F controller", "MCU budget-dependent gain reduction",
                            "KEY/drag state machine", "protocol enable/fault/position quantization"],
        "frictionloss_zero_means": "Unidentified stiction, not measured zero mechanical friction.",
        "collision": "Original self-collision disabled; convex hull floor contact only, not CAD certification.",
        "initialization": "GUI: pause, select keyframe 0 and Load key. FF is evaluated only at the saved pose.",
        "assets": "Uses existing absolute local mesh paths. XML is not a mesh bundle.",
    })
    text("unadopted_candidates",
         "The 2026-11-23 inertia position/acceleration delta-I candidate was not robust "
         "and remains deploy=false. Do not replace nominal armature or masses with it. "
         "Wrist retest is validation, not a newly adopted friction table. "
         "Earlier negative-B regressions are not physical negative damping.")
    sources = [CONFIG, ROOT / "config/five_coordinates.h",
               ROOT / "config/joint_references.def", ROOT / "config/five_scene.inc",
               ROOT / "Mode/five_mit.c", ROOT / "Mode/five_support.c",
               ROOT / "Model/five_dynamics.c", DIAGNOSTIC,
               BUILD / "ylamp_v12_five_axis.bin",
               SIM / "ylamp_learning.xml", SIM / "damiao_noload.json", Path(__file__)]
    text("source_sha256", {str(path): sha(path) for path in sources})
    text("firmware_sha256", diag["firmware_sha256"])
    text("original_firmware_reference_text",
         (ROOT / "config/joint_references.def").read_text(encoding="utf-8"))
    for path in (
            ROOT / "build/v12-default-drag-20261123/friction_global_model_02.json",
            ROOT / "build/v12-friction-global-20261123/inertia_position_acceleration_01.json",
            BUILD / "wrist_friction_retest_analysis_01.json"):
        text(path.stem + "_provenance", {"file": str(path), "sha256": sha(path)})
    return configure_collisions(root) if collisions else root


def load_native(path):
    dll = prepare.native_library()
    dll.mj_versionString.restype = ct.c_char_p
    if dll.mj_versionString().decode() != "4.0.1":
        raise ValueError("This exporter is checked against the installed MuJoCo 4.0.1")
    error = ct.create_string_buffer(4096)
    model = dll.mj_loadXML(str(path).encode(), None, error, len(error))
    if not model:
        raise ValueError(error.value.decode(errors="replace"))
    return dll, model


def check(path, full=False):
    root = ET.parse(path).getroot()
    params = parameters()
    refs, bounds, q0, _ = inputs()
    for name, value in params.items():
        stored = np.fromstring(root.find(f"./custom/numeric[@name='{name}']").get("data"), sep=" ")
        assert np.array_equal(stored, value), name
    for i, name in enumerate(prepare.JOINTS):
        joint = root.find(f".//joint[@name='{name}']")
        assert np.array_equal(np.fromstring(joint.get("range"), sep=" "), bounds[i])
        assert float(joint.get("armature")) == SCENE[i, -1]
        assert float(joint.get("frictionloss")) == 0
    dll, model = load_native(path)
    ptr, num = ct.c_void_p, ct.c_double
    dll.mj_resetDataKeyframe.argtypes = [ptr, ptr, ct.c_int]
    dll.mj_resetDataKeyframe.restype = None
    dll.mj_rne.argtypes, dll.mj_rne.restype = [ptr, ptr, ct.c_int, ct.POINTER(num)], None
    data = None
    collision_enabled = root.find("./custom/text[@name='collision_volumes']") is not None
    report = {"xml": str(path), "sha256": sha(path), "native_version": "4.0.1",
              "production_tables": len(params), "hardware_access": False,
              "scope": "Native model consistency, not physical identification acceptance"}
    try:
        assert [dll.mj_stateSize(model, 1 << i) for i in (1, 2, 6)] == [5, 5, 15]
        assert abs(dll.mj_getTotalmass(model) - 2.32467047218) < 1e-10
        data = dll.mj_makeData(model)
        assert data
        dll.mj_resetDataKeyframe(model, data, 0)
        position, control = (num * 5)(), (num * 15)()
        dll.mj_getState(model, data, position, 1 << 1)
        dll.mj_getState(model, data, control, 1 << 6)
        assert np.array_equal(position, q0)
        assert np.allclose(np.array(control)[2::3], support_gravity(q0[None])[0], atol=1e-14)
        report["keyframe_cad_deg"] = np.degrees(q0).tolist()
        report["keyframe_fixed_ff_nm"] = np.array(control)[2::3].tolist()
        if collision_enabled:
            bodies = root.findall(".//body")
            geometry = []
            for body in bodies:
                for visual in body.findall("geom"):
                    if visual.get("contype") != "0":
                        continue
                    geom = body.find(f"./geom[@name='collision_{visual.get('name')}']")
                    assert geom is not None and geom.get("type") == "sdf"
                    assert geom.get("mesh") == visual.get("mesh")
                    assert float(geom.get("density")) == 0
                    geometry.append((body, geom))
            assert len(geometry) == 24
            assert root.find("./option/flag").get("filterparent") == "disable"
            assert not root.findall("./contact/exclude")
            for (a, ga), (b, gb) in itertools.combinations(geometry, 2):
                if a is b:
                    continue
                if "hardware" not in ga.get("name") + gb.get("name"):
                    assert (int(ga.get("contype")) & int(gb.get("conaffinity"))
                            or int(gb.get("contype")) & int(ga.get("conaffinity")))
            sid = dll.mj_name2id(model, 20, b"collision_all")
            assert sid >= 0

            def contact_state():
                value = (num * 4)()
                pointer = dll.mj_readSensor(model, data, sid, 0, value, 0)
                return np.array(pointer[:4] if pointer else value)

            dll.mj_forward(model, data)
            initial_contact = contact_state()
            assert initial_contact[0] == 0, initial_contact
            report["collision"] = dict(rigid_bodies=6, sdf_volumes=24,
                                       keyframe_contacts=int(initial_contact[0]),
                                       all_main_body_pairs_enabled=True)
            if full:
                # Find and verify a real native contact/force in the recorded
                # range; this is a simulation test, never a hardware command.
                rng = np.random.default_rng(20261124)
                witness = None
                for trial in range(128):
                    q = rng.uniform(bounds[:, 0], bounds[:, 1])
                    dll.mj_resetData(model, data)
                    dll.mj_setState(model, data, (num * 5)(*q), 1 << 1)
                    target = np.column_stack((q, np.zeros(5), support_gravity(q[None])[0])).ravel()
                    dll.mj_setState(model, data, (num * 15)(*target), 1 << 6)
                    dll.mj_forward(model, data)
                    contact = contact_state()
                    if contact[0] > 0 and np.linalg.norm(contact[1:]) > 1e-6:
                        witness = dict(trial=trial, qCAD_rad=q.tolist(),
                                       contacts=int(contact[0]), force_sensor=contact[1:].tolist())
                        break
                assert witness is not None, "No native contact force found in 128 bounded poses"
                report["collision"]["native_force_witness"] = witness
        if full:
            rng = np.random.default_rng(20261123)
            max_error = 0.
            for q in np.vstack((q0, rng.uniform(bounds[:, 0], bounds[:, 1], size=(128, 5)))):
                v, a = rng.uniform(-.8, .8, 5), rng.uniform(-1, 1, 5)
                dll.mj_setState(model, data, (num * 5)(*q), 1 << 1)
                dll.mj_setState(model, data, (num * 5)(*v), 1 << 2)
                dll.mj_forward(model, data)
                bias, matrix = (num * 5)(), (num * 25)()
                dll.mj_rne(model, data, 0, bias)
                dll.mj_fullM(model, data, matrix)
                mass = np.array(matrix).reshape(5, 5)
                assert np.linalg.eigvalsh(mass).min() > 0
                fixed, basis, _ = regressors(q[None], v[None], a[None])
                expected = (fixed + basis @ NOMINAL)[0]
                max_error = max(max_error, float(np.max(abs(mass @ a + bias - expected))))
            assert max_error < 1e-10, max_error
            # A numerical smoke test only: live G updated by the offline harness,
            # not by the standalone XML; no new F or double-counted damping.
            dll.mj_resetDataKeyframe(model, data, 0)
            for _ in range(1500):
                dll.mj_getState(model, data, position, 1 << 1)
                control[2::3] = support_gravity(np.array(position)[None])[0]
                dll.mj_setState(model, data, control, 1 << 6)
                dll.mj_step(model, data)
            dll.mj_getState(model, data, position, 1 << 1)
            velocity = (num * 5)()
            dll.mj_getState(model, data, velocity, 1 << 2)
            assert np.isfinite(position).all() and np.isfinite(velocity).all()
            report.update(inverse_dynamics_poses=129, max_inverse_error_nm=max_error,
                          smoke_simulated_seconds=3,
                          smoke_final_position_rad=list(position),
                          smoke_final_velocity_rad_s=list(velocity),
                          smoke_controller="Offline harness updates G; XML alone does not")
    finally:
        if data:
            dll.mj_deleteData(data)
        dll.mj_deleteModel(model)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--collisions", action="store_true",
                        help="export a separate SDF self-collision XML; preserve the baseline XML")
    parser.add_argument("--check", action="store_true", help="read existing XML; never regenerate")
    parser.add_argument("--full", action="store_true", help="129-pose dynamics and 3s numerical smoke")
    args = parser.parse_args()
    if args.collisions and args.output == OUTPUT:
        args.output = OUTPUT.with_stem(OUTPUT.stem + "_collision")
    if not args.check:
        root = make_scene(args.collisions)
        ET.indent(root, space="  ")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        # Exclusive creation preserves both the original scene and user edits.
        with args.output.open("xb") as stream:
            ET.ElementTree(root).write(stream, encoding="utf-8", xml_declaration=True)
    print(json.dumps(check(args.output, args.full), ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
