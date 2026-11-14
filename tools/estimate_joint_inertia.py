"""Offline initial yaw/pitch inertias. Never communicates with hardware.

Requires Python and NumPy already used by the project. STEP lengths and slicer
coordinates are mm; the exported mass properties and inertias are SI.
"""

from __future__ import annotations

import argparse
from array import array
import hashlib
import json
import math
import mmap
from pathlib import Path
import re
import xml.etree.ElementTree as ET
import zipfile

import numpy as np


MODEL_DIR = Path(__file__).resolve().parents[2] / "Ylamp-V1.2"
CACHE_DIR = Path.home() / "AppData/Local/Temp/ylamp-mass-e92183306c6c47ab825d012caf625535"
NUMBER = r"[-+]?(?:\d*\.?\d+)(?:[Ee][-+]?\d+)?"
REF = re.compile(rb"#(\d+)")


class Step:
    """Targeted AP203 reader, not a general CAD tessellator."""

    def __init__(self, path):
        self.file = Path(path).open("rb")
        self.data = mmap.mmap(self.file.fileno(), 0, access=mmap.ACCESS_READ)
        self.offsets = array("q", [-1]) * 2_000_000
        for match in re.finditer(rb"^#(\d+)\s*=", self.data, re.M):
            i = int(match[1])
            if i >= len(self.offsets):
                self.offsets.extend([-1] * (i + 1 - len(self.offsets)))
            self.offsets[i] = match.start()

    def close(self):
        self.data.close()
        self.file.close()

    def entity(self, i):
        start = self.offsets[i]
        if start < 0:
            raise ValueError(f"Missing STEP entity #{i}")
        return self.data[start : self.data.find(b";", start) + 1]

    def refs(self, i):
        return [int(x) for x in REF.findall(self.entity(i).split(b"=", 1)[1])]

    def vector(self, i):
        raw = self.entity(i).decode("ascii")
        match = re.search(r",\s*\(\s*(" + NUMBER + r".*?)\)\s*\)", raw)
        return np.array([float(v) for v in re.findall(NUMBER, match[1])])

    def frame(self, i):
        p, z, x = (self.vector(j) for j in self.refs(i))
        z /= np.linalg.norm(z)
        x -= z * np.dot(z, x)
        x /= np.linalg.norm(x)
        return np.column_stack((x, np.cross(z, x), z)), p

    def envelope(self, representation):
        pending = self.refs(representation)[:-2]  # omit reference frame/context
        visited, vertices, cylinders = set(), [], []
        while pending:
            i = pending.pop()
            if i in visited:
                continue
            visited.add(i)
            raw = self.entity(i)
            refs = self.refs(i)
            if b"VERTEX_POINT" in raw:
                vertices.append(self.vector(refs[0]))
            if b"CYLINDRICAL_SURFACE" in raw:
                radius = float(re.findall(NUMBER.encode(), raw.split(b"=", 1)[1])[-1])
                r, p = self.frame(refs[0])
                if radius > 20:
                    cylinders.append((radius, p.tolist(), r[:, 2].tolist()))
            pending.extend(refs)
        vertices = np.array(vertices)
        return {
            "vertices_min_mm": vertices.min(axis=0).tolist(),
            "vertices_max_mm": vertices.max(axis=0).tolist(),
            "large_cylinders": sorted(cylinders),
        }


def rotate(axis, theta):
    axis = np.asarray(axis, dtype=float)
    x, y, z = axis / np.linalg.norm(axis)
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(theta) * k + (1 - math.cos(theta)) * k @ k


def grip_model(frames):
    """J3 frame origin, not a measured hand contact; never invert a Jacobian."""
    p1, p2, p3 = (frames[name][1] / 1000 for name in ("J1", "J2", "J3"))
    u1, u2 = (frames[name][0][:, 1] for name in ("J1", "J2"))
    assert abs(np.dot(u1, u2)) < 1e-12
    transverse = p3 - p2 - u2 * np.dot(u2, p3 - p2)
    up = rotate(u2, math.atan2(np.dot(u2, np.cross(transverse, u1)),
                             np.dot(transverse, u1)))
    basis = np.column_stack((np.cross(u2, u1), u2, u1))
    origin = basis.T @ (p2 - p1)
    point = basis.T @ up @ (p3 - p2)
    ez, ey = np.array([0., 0., 1.]), np.array([0., 1., 0.])

    def evaluate(q1, q2):
        yaw, pitch = rotate(ez, q1), rotate(ey, q2)
        position = yaw @ (origin + pitch @ point)
        jacobian = np.column_stack((np.cross(ez, position),
                                   yaw @ np.cross(ey, pitch @ point)))
        return position, jacobian

    poses = {}
    for degrees in (-45, 0, 15, 30, 45, 60, 90):
        q = math.radians(degrees)
        position, jacobian = evaluate(.7, q)
        for axis in (0, 1):
            step = np.eye(2)[axis] * 1e-6
            plus = evaluate(.7 + step[0], q + step[1])[0]
            minus = evaluate(.7 - step[0], q - step[1])[0]
            assert np.allclose((plus-minus)/2e-6, jacobian[:, axis], atol=1e-9, rtol=0)
        poses[str(degrees)] = {
            "position_at_q1_0p7_rad_m": position.tolist(),
            "translational_jacobian_m_per_rad": jacobian.tolist(),
            "radius_m": np.linalg.norm(jacobian, axis=0).tolist(),
            "rank": int(np.linalg.matrix_rank(jacobian, tol=1e-10)),
        }
    r2 = float(np.hypot(point[0], point[2]))
    return {
        "point": "J3 STEP frame origin, fixed in J2 link, not measured hand contact",
        "axes": "J1 +Z / J2 +Y in yaw basis; encoder polarities UNVERIFIED",
        "j2_origin_m": origin.tolist(),
        "point_from_j2_m_at_upright": point.tolist(),
        "reference_radius_m": [float(np.hypot(abs(origin[0])+r2, origin[1]+point[1])), r2],
        "formula": "p=Rz(q1)*(origin+Ry(q2)*point); J=[ez cross p,Rz(q1)*(ey cross Ry(q2)*point)]",
        "poses_q2_deg": poses,
        "boundary": "No inverse; upright translation Jacobian rank may drop. No drive inertia or friction identification.",
    }


def segment_moments(a, b, arc=None):
    """First and raw second spatial moments of a uniform deposited path."""
    a, b = np.asarray(a), np.asarray(b)
    if arc is None:
        return (a + b) / 2, (
            (np.outer(a, a) + np.outer(b, b)) / 3
            + (np.outer(a, b) + np.outer(b, a)) / 6
        )
    center, clockwise = arc
    center = np.asarray(center)
    radius = np.linalg.norm(a[:2] - center)
    start = math.atan2(a[1] - center[1], a[0] - center[0])
    end = math.atan2(b[1] - center[1], b[0] - center[0])
    sweep = (end - start) % (2 * math.pi)
    if clockwise:
        sweep = sweep - 2 * math.pi
    elif sweep < 1e-12:
        sweep = 2 * math.pi
    end = start + sweep
    c = (math.sin(end) - math.sin(start)) / sweep
    s = (math.cos(start) - math.cos(end)) / sweep
    cc = 0.5 + (math.sin(2 * end) - math.sin(2 * start)) / (4 * sweep)
    ss = 1 - cc
    cs = (math.cos(2 * start) - math.cos(2 * end)) / (4 * sweep)
    mean = np.array([center[0] + radius * c, center[1] + radius * s, (a[2] + b[2]) / 2])
    second = np.outer(mean, mean)
    second[:2, :2] += radius**2 * np.array([[cc - c*c, cs - c*s], [cs - c*s, ss - s*s]])
    # Production G-code arcs are planar. Reject helical extrusion, not silently
    # apply the planar moment formula to a new slicer profile.
    if abs(a[2] - b[2]) > 1e-8:
        raise ValueError("Helical extrusion is not supported")
    return mean, second


def slicer_transform(path):
    with zipfile.ZipFile(path) as archive:
        root = ET.fromstring(archive.read("3D/3dmodel.model"))
        item = root.find("{*}build/{*}item")
        values = np.array([float(v) for v in item.attrib["transform"].split()]).reshape(4, 3)
        component = root.find("{*}resources/{*}object/{*}components/{*}component")
        if component is not None:
            transform = np.array([float(v) for v in component.attrib["transform"].split()]).reshape(4, 3)
            if not np.allclose(transform, np.vstack((np.eye(3), np.zeros(3)))):
                raise ValueError("Unexpected nonidentity component placement")
        return values[:3].T, values[3]


def source_offset(path):
    with zipfile.ZipFile(path) as archive:
        root = ET.fromstring(archive.read("Metadata/model_settings.config"))
        part = root.find("object/part")
        metadata = {item.attrib["key"]: item.attrib["value"] for item in part.findall("metadata")}
        return np.array([float(metadata["source_offset_" + axis]) for axis in "xyz"])


def integrate_gcode(path, original_3mf, sliced_3mf):
    position = np.zeros(3)
    e_position = 0.0
    absolute_xyz, relative_e = True, True
    feature, height = "Custom", 0.2
    with zipfile.ZipFile(original_3mf) as archive:
        settings = json.loads(archive.read("Metadata/project_settings.config"))
    density = float(settings["filament_density"][0])
    diameter = float(settings["filament_diameter"][0])
    # g/cm3 to kg/mm3.
    kg_per_e_mm = math.pi * (diameter / 2)**2 * density * 1e-6
    totals = {}
    arc_count = 0
    with Path(path).open(encoding="utf-8", errors="replace") as stream:
        for raw in stream:
            if raw.startswith("; FEATURE:"):
                feature = raw.split(":", 1)[1].strip()
            if raw.startswith("; LAYER_HEIGHT:"):
                height = float(raw.split(":", 1)[1])
            command = raw.split(";", 1)[0].strip()
            if not command:
                continue
            tokens = command.split()
            op = tokens[0]
            if op == "G90":
                absolute_xyz = True
            elif op == "G91":
                absolute_xyz = False
            elif op == "M82":
                relative_e = False
            elif op == "M83":
                relative_e = True
            if op not in {"G0", "G1", "G2", "G3", "G92"}:
                continue
            params = {m[0]: float(m[1:]) for m in re.findall(r"[XYZEFIJKR]" + NUMBER, command)}
            if op == "G92":
                for j, axis in enumerate("XYZ"):
                    position[j] = params.get(axis, position[j])
                e_position = params.get("E", e_position)
                continue
            previous = position.copy()
            for j, axis in enumerate("XYZ"):
                if axis in params:
                    position[j] = params[axis] if absolute_xyz else position[j] + params[axis]
            extrusion = params.get("E", 0) if relative_e else params.get("E", e_position) - e_position
            if "E" in params:
                e_position = e_position + params["E"] if relative_e else params["E"]
            is_arc = op in {"G2", "G3"}
            if extrusion <= 0 or (np.linalg.norm(position - previous) < 1e-10 and not is_arc):
                continue
            a, b = previous.copy(), position.copy()
            a[2] -= height / 2
            b[2] -= height / 2
            arc = None
            if is_arc:
                if "R" in params:
                    raise ValueError("R-format arcs are unsupported")
                arc = (previous[:2] + [params.get("I", 0), params.get("J", 0)], op == "G2")
                arc_count += 1
            mean, second = segment_moments(a, b, arc)
            mass = extrusion * kg_per_e_mm
            if feature not in totals:
                totals[feature] = [0.0, np.zeros(3), np.zeros((3, 3))]
            totals[feature][0] += mass
            totals[feature][1] += mass * mean
            totals[feature][2] += mass * second
    excluded = {"Custom", "Brim", "Skirt", "Support", "Support interface", "Prime tower", "Flush"}
    allowed = {
        "Inner wall", "Outer wall", "Overhang wall", "Gap infill", "Top surface",
        "Bottom surface", "Internal solid infill", "Sparse infill", "Bridge",
        "Internal bridge", "Floating vertical shell",
    }
    unknown = set(totals) - excluded - allowed
    if unknown:
        raise ValueError(f"Unreviewed slicer extrusion features: {unknown}")
    included = [value for key, value in totals.items() if key not in excluded]
    mass = sum(v[0] for v in included)
    mean_bed = sum((v[1] for v in included), np.zeros(3)) / mass
    covariance_bed = sum((v[2] for v in included), np.zeros((3, 3))) - mass * np.outer(mean_bed, mean_bed)
    r, p = slicer_transform(sliced_3mf)
    center_native = r.T @ (mean_bed - p) + source_offset(original_3mf)
    covariance_native = r.T @ covariance_bed @ r * 1e-6
    assert np.linalg.eigvalsh(covariance_native).min() > 0
    inertia = np.trace(covariance_native) * np.eye(3) - covariance_native
    assert np.linalg.eigvalsh(inertia).min() > 0
    result = json.loads((Path(path).parent / "result.json").read_text(encoding="utf-8"))
    slicer_mass = result["sliced_plates"][0]["filaments"][0]["main_used_g"]
    mass_error_g = sum(v[0] for v in totals.values()) * 1000 - slicer_mass
    assert abs(mass_error_g) < 2e-5, mass_error_g
    return {
        "mass_kg": mass,
        "com_native_mm": center_native.tolist(),
        "I_com_native_kg_m2": inertia.tolist(),
        "feature_masses_g": {key: value[0] * 1000 for key, value in totals.items()},
        "arc_count": arc_count,
        "mass_audit_error_g": mass_error_g,
        "saved_print_profile": {
            key: settings[key] for key in [
                "wall_loops", "sparse_infill_density", "sparse_infill_pattern",
                "top_surface_pattern", "layer_height", "top_shell_layers", "bottom_shell_layers",
            ]
        },
        "PLA_density_g_cm3": density,
        "filament_diameter_mm": diameter,
        "deposition_assumptions": "uniform mass along XY extrusion path, Z at layer mid-height; bead cross-section omitted",
    }


def cylinder(mass, radius_mm, height_mm, center_y_mm, shell=False):
    """Explicit whole-motor envelope approximation, NOT drive rotor inertia."""
    r, h = radius_mm / 1000, height_mm / 1000
    axial = mass * r*r * (1 if shell else 0.5)
    transverse = mass * (r*r * (0.5 if shell else 0.25) + h*h / 12)
    return {
        "mass_kg": mass,
        "com_native_mm": [0, center_y_mm, 0],
        "I_com_native_kg_m2": np.diag([transverse, axial, transverse]).tolist(),
        "assumption": "uniform thin cylindrical shell" if shell else "uniform solid envelope cylinder",
        "radius_mm": radius_mm,
        "height_mm": height_mm,
    }


def inertia_model(parts, frames, *, shell=False, center_shifts_mm=(0., 0.), point_motors=False):
    """M with CAD +Y motor axes positive, J2-J3 axis centerline upright at q2=0."""
    motors = {
        "J2_motor_body": cylinder(0.375, 28.5, 56.5, 28.25 + center_shifts_mm[0], shell),
        "J3_motor_body": cylinder(0.300, 28.5, 46.0, 22.5 + center_shifts_mm[1], shell),
    }
    p1, p2, p3 = (frames[key][1] / 1000 for key in ["J1", "J2", "J3"])
    u1, u2 = (frames[key][0][:, 1] for key in ["J1", "J2"])
    assert abs(np.dot(u1, u2)) < 1e-12
    assert np.dot(u2, frames["J3"][0][:, 1]) > 1 - 1e-12
    l = p3 - p2
    l -= u2 * np.dot(u2, l)
    theta_up = math.atan2(np.dot(u2, np.cross(l, u1)), np.dot(l, u1))
    r_up = rotate(u2, theta_up)
    assert np.allclose(r_up @ l / np.linalg.norm(l), u1)
    basis = np.column_stack((np.cross(u2, u1), u2, u1))
    bodies = []
    for name, properties in {**parts, **motors}.items():
        frame_name = {"J2_motor_body": "J2", "J3_motor_body": "J3"}.get(name, name)
        r, p_mm = frames[frame_name]
        center = p_mm / 1000 + r @ (np.array(properties["com_native_mm"]) / 1000)
        inertia = r @ np.array(properties["I_com_native_kg_m2"]) @ r.T
        moving = name in {"D3", "J3_motor_body"}
        if moving:
            center = p2 + r_up @ (center - p2)
            inertia = r_up @ inertia @ r_up.T
        if point_motors and name in motors:
            inertia = np.zeros((3, 3))
        bodies.append((name, properties["mass_kg"], center, inertia, moving))

    def evaluate(angle, breakdown=False):
        r = rotate(u2, angle)
        contributions = {}
        for name, mass, center0, inertia0, moving in bodies:
            center = p2 + r @ (center0 - p2) if moving else center0
            inertia = r @ inertia0 @ r.T if moving else inertia0
            jv = np.column_stack((
                np.cross(u1, center - p1),
                np.cross(u2, center - p2) if moving else np.zeros(3),
            ))
            jw = np.column_stack((u1, u2 if moving else np.zeros(3)))
            contributions[name] = mass * jv.T @ jv + jw.T @ inertia @ jw
        return contributions if breakdown else sum(contributions.values(), np.zeros((2, 2)))

    angles = np.arange(32) * (2 * math.pi / 32)
    values = np.array([evaluate(q) for q in angles])
    a_basis = lambda q: np.column_stack((np.ones_like(q), np.cos(q), np.sin(q), np.cos(2*q), np.sin(2*q)))
    b_basis = lambda q: np.column_stack((np.cos(q), np.sin(q)))
    a = np.linalg.lstsq(a_basis(angles), values[:, 0, 0], rcond=None)[0]
    b = np.linalg.lstsq(b_basis(angles), values[:, 0, 1], rcond=None)[0]
    d = values[:, 1, 1].mean()
    dense = np.linspace(-math.pi, math.pi, 1441)
    matrices = np.array([evaluate(q) for q in dense])
    residual = max(
        np.max(np.abs(a_basis(dense) @ a - matrices[:, 0, 0])),
        np.max(np.abs(b_basis(dense) @ b - matrices[:, 0, 1])),
        np.max(np.abs(d - matrices[:, 1, 1])),
    )
    assert residual < 1e-12
    assert np.linalg.eigvalsh(matrices).min() > 0
    # A point-mass-only model can be compared against the complete rigid-body
    # result, but never used silently as the motor body inertia.
    h = 1e-4
    q = 0.371
    derivatives = (evaluate(q+h) - evaluate(q-h)) / (2*h)
    analytic_a = -a[1]*math.sin(q) + a[2]*math.cos(q) - 2*a[3]*math.sin(2*q) + 2*a[4]*math.cos(2*q)
    analytic_b = -b[0]*math.sin(q) + b[1]*math.cos(q)
    assert abs(derivatives[0, 0] - analytic_a) < 1e-9
    assert abs(derivatives[0, 1] - analytic_b) < 1e-9
    upper = [body for body in bodies if body[4]]
    first_moment = sum((mass * (center-p2) for _, mass, center, _, _ in upper), np.zeros(3))
    canonical_moment = basis.T @ first_moment
    return {
        "A_coefficients_kg_m2": dict(zip(["1", "cos(q2)", "sin(q2)", "cos(2*q2)", "sin(2*q2)"], a.tolist())),
        "B_coefficients_kg_m2": dict(zip(["cos(q2)", "sin(q2)"], b.tolist())),
        "D_kg_m2": d,
        "harmonic_fit_max_error_kg_m2": residual,
        "mass_matrix_min_eigenvalue_kg_m2": float(np.linalg.eigvalsh(matrices).min()),
        "A_min_max_kg_m2": [float(matrices[:, 0, 0].min()), float(matrices[:, 0, 0].max())],
        "max_abs_B_kg_m2": float(np.max(np.abs(matrices[:, 0, 1]))),
        "sample_matrices_kg_m2": {str(deg): evaluate(math.radians(deg)).tolist() for deg in [0, 30, 45, 60, 90]},
        "contributions_at_q2_zero_kg_m2": {name: value.tolist() for name, value in evaluate(0, True).items()},
        "motor_body_assumptions": motors,
        "body_com_at_zero_in_yaw_basis_mm": {name: (basis.T @ (center-p1) * 1000).tolist() for name, _, center, _, _ in bodies},
        "J2_axis_point_in_yaw_basis_mm": (basis.T @ (p2-p1) * 1000).tolist(),
        "J2_to_J3_transverse_distance_mm": float(np.linalg.norm(l)*1000),
        "CAD_pose_to_upright_rotation_deg": math.degrees(theta_up),
        "gravity_crosscheck": {
            "upper_mass_kg": sum(body[1] for body in upper),
            "first_moment_xyz_kg_m": canonical_moment.tolist(),
            "G2_sin_coefficient_Nm": -9.81 * float(canonical_moment[2]),
            "G2_cos_coefficient_Nm": -9.81 * float(canonical_moment[0]),
        },
    }


def build_result(parts, frames, model_dir, slice_dir):
    nominal = inertia_model(parts, frames)
    point = inertia_model(parts, frames, point_motors=True)
    shell = inertia_model(parts, frames, shell=True)
    center_cases = {
        f"J2_shift_{s2:+g}_J3_shift_{s3:+g}_mm": inertia_model(parts, frames, center_shifts_mm=(s2, s3))
        for s2 in [-5, 0, 5] for s3 in [-5, 0, 5] if s2 or s3
    }
    inputs = [model_dir / "装配体2.step"]
    inputs += [model_dir / (name+".3mf") for name in ["D3", "D4"]]
    inputs += [slice_dir / name / "plate_1.gcode" for name in ["D3", "D4"]]
    hashes = {}
    for path in inputs:
        with path.open("rb") as stream:
            hashes[str(path)] = hashlib.file_digest(stream, "sha256").hexdigest()
    return {
        "status": "offline_initial_estimate_not_identified_not_applied",
        "date": "2026-11-08",
        "units": {"angle": "rad", "mass": "kg", "inertia": "kg*m^2"},
        "scope": "installed D4 + J2 DM4340P body + D3 + passive J3 DM4310; excludes boards/screws/cables",
        "special_zero": {
            "J2_encoder_zero_rad": -2.640383005142212,
            "meaning": "J2-to-J3 axis centerline vertically upward; not upper assembly COM exactly upward",
            "q2_encoder_model": "q2_raw - (-2.640383005142212)",
            "encoder_vs_CAD_axis_signs": "UNVERIFIED",
            "sign_mapping": "q2_CAD=sigma2*q2_model; A_model(q)=A_CAD(sigma2*q); B_model(q)=sigma1*sigma2*B_CAD(sigma2*q); D unchanged",
        },
        "mass_matrix": "M=[[A(q2),B(q2)],[B(q2),D]]",
        "coordinate_convention": "positive about each motor CAD local +Y; q2=0 is centerline upright; J1 CAD +Y is treated as vertical",
        "printed_parts": parts,
        "motor_data_basis": {
            "DM4340P": "375 g retained from prior DM-J4340P-2EC V1.0 manual review; CAD envelope diameter 57 mm, Y in [0,56.5] mm",
            "DM4310": "300 g / nominal 46 mm height verified against manufacturer V1.0 manual page 9; CAD envelope diameter 57 mm, Y in [-0.5,45.5] mm",
            "DM4310_manual_primary_blob": "https://api.github.com/repos/dmBots/DM-J4310-2EC/git/blobs/943735da12556fb86ea2c14e7bdd83ec81c282f6",
            "motor_inertia_measurement": "manufacturer whole-body COM/inertia not obtained; cylinder inertia is explicitly approximate",
        },
        "nominal_rigid_body_only": nominal,
        "nominal_grip_geometry": grip_model(frames),
        "motor_COM_only_diagnostic_DO_NOT_USE": {
            "A_at_zero_kg_m2": point["sample_matrices_kg_m2"]["0"][0][0],
            "D_kg_m2": point["D_kg_m2"],
            "missing_D_kg_m2": nominal["D_kg_m2"] - point["D_kg_m2"],
            "missing_A_at_zero_kg_m2": nominal["sample_matrices_kg_m2"]["0"][0][0] - point["sample_matrices_kg_m2"]["0"][0][0],
        },
        "sensitivity_not_confidence_bounds": {
            "thin_shell_same_envelope": {
                "A_coefficients_kg_m2": shell["A_coefficients_kg_m2"],
                "B_coefficients_kg_m2": shell["B_coefficients_kg_m2"],
                "D_kg_m2": shell["D_kg_m2"],
            },
            "axial_COM_shift_plus_minus_5_mm": {
                name: {
                    "A_at_zero_kg_m2": case["sample_matrices_kg_m2"]["0"][0][0],
                    "B_coefficients_kg_m2": case["B_coefficients_kg_m2"],
                    "D_kg_m2": case["D_kg_m2"],
                } for name, case in center_cases.items()
            },
        },
        "unidentified_drive_inertia": {
            "J1": "add Jdrive1 >= 0 to A; UNKNOWN, not assumed measured zero",
            "J2": "add Jdrive2 >= 0 to D; UNKNOWN, not assumed measured zero",
            "formula_if_input_rotor_inertia_known": "Jdrive_i approx N_i^2*Jrotor_i + reflected gear/output inertia; clarify reference side before multiplying",
            "not_equal_to": "whole motor body cylinder inertia or m*COM_distance^2",
            "passive_J3": "nominal treats all J3 mass as co-moving rigid body; disabled does not lock its output/internal rotor; do not add N3^2*Jrotor3 as an active J2 drive",
        },
        "limitations": [
            "motor mass and variant must be checked against the actual units; body COM/inertia are assumptions, not manufacturer measurements",
            "J1 and J2 encoder positive directions vs CAD +Y require safe physical verification before applying B or odd sine terms",
            "PLA density/profile and cached slicer reproduce estimated printed mass, not scale measurements",
            "uniform extrusion centerline model omits small bead width/height second moments",
            "J3 passive output/internal motion not identified",
            "no firmware parameters, software zeros, hardware state, motor enable or control mode were changed",
        ],
        "input_SHA256": hashes,
    }


def self_test():
    mean, second = segment_moments([0., 0., 0.], [2., 0., 0.])
    assert np.allclose(mean, [1, 0, 0])
    assert np.allclose(second, np.diag([4/3, 0, 0]))
    for clockwise in [False, True]:
        mean, second = segment_moments([2., 0., 1.], [2., 0., 1.], ([0., 0.], clockwise))
        assert np.allclose(mean, [0, 0, 1])
        assert np.allclose(second, np.diag([2, 2, 1]))
    r = rotate([0, 1, 0], 0.4)
    assert np.allclose(r @ r.T, np.eye(3))
    assert np.isclose(np.linalg.det(r), 1)
    # Check quarter arc and translation; catches confusing COM with raw moments.
    mean, second = segment_moments([1., 0., 0.], [0., 1., 0.], ([0., 0.], False))
    assert np.allclose(mean, [2/math.pi, 2/math.pi, 0])
    assert np.isclose(second[0, 1], 1/math.pi)
    offset = np.array([3., -4., 2.])
    shifted_mean, shifted_second = segment_moments(offset, offset + [2., 0., 0.])
    assert np.allclose(shifted_second - np.outer(shifted_mean, shifted_mean), np.diag([1/3, 0, 0]))
    # Independent quadrature of partial CW/CCW arcs with translated centers.
    for sweep in [0.7, 2.5, -0.8, -4.1]:
        start, radius, center = 0.31, 2.3, np.array([3., -2.])
        end = start + sweep
        a = np.r_[center + radius*np.array([math.cos(start), math.sin(start)]), 1.7]
        b = np.r_[center + radius*np.array([math.cos(end), math.sin(end)]), 1.7]
        mean, second = segment_moments(a, b, (center, sweep < 0))
        theta = start + sweep * (np.arange(20000) + 0.5) / 20000
        points = np.column_stack((center[0]+radius*np.cos(theta), center[1]+radius*np.sin(theta), np.full_like(theta, 1.7)))
        assert np.allclose(mean, points.mean(axis=0), atol=1e-8, rtol=0)
        assert np.allclose(second, points.T@points/len(points), atol=1e-7, rtol=0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-dir", type=Path, default=MODEL_DIR)
    parser.add_argument("--slice-dir", type=Path, default=CACHE_DIR)
    parser.add_argument("--inspect", action="store_true")
    parser.add_argument("--grip", action="store_true", help="only extract/verify nominal J3 grip geometry from STEP")
    parser.add_argument("--self-test", action="store_true", help="run analytical/quadrature unit checks without loading CAD")
    parser.add_argument("--output", type=Path, help="optional offline JSON result, never used by firmware")
    args = parser.parse_args()
    self_test()
    if args.self_test:
        print("PASS: line/arc mass moments, translation and rotation checks")
        return
    assembly = max(args.model_dir.glob("*.step"), key=lambda p: p.stat().st_size)
    step = Step(assembly)
    try:
        frames = {name: step.frame(i) for name, i in {
            "J1": 1579768, "J2": 1548686, "J3": 1700325, "D3": 640474, "D4": 337262,
        }.items()}
        if args.grip:
            result = grip_model(frames)
            encoded = json.dumps(result, indent=2, ensure_ascii=False) + "\n"
            if args.output:
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.write_text(encoded, encoding="utf-8")
            print(encoded)
        elif args.inspect:
            for name, (r, p) in frames.items():
                print(name, "R=", r.tolist(), "p_mm=", p.tolist())
            for name, i in [("DM4340P", 1694781), ("DM4310", 223692)]:
                print(name, json.dumps(step.envelope(i)))
        else:
            parts = {}
            for name in ["D3", "D4"]:
                parts[name] = integrate_gcode(
                    args.slice_dir / name / "plate_1.gcode",
                    args.model_dir / (name + ".3mf"),
                    args.slice_dir / name / (name + ".gcode.3mf"),
                )
            result = build_result(parts, frames, args.model_dir, args.slice_dir)
            encoded = json.dumps(result, indent=2, ensure_ascii=False) + "\n"
            if args.output:
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.write_text(encoded, encoding="utf-8")
            print(json.dumps({
                "nominal": result["nominal_rigid_body_only"],
                "motor_COM_only_diagnostic_DO_NOT_USE": result["motor_COM_only_diagnostic_DO_NOT_USE"],
                "sensitivity": result["sensitivity_not_confidence_bounds"],
                "output": str(args.output) if args.output else None,
            }, indent=2, ensure_ascii=True))
    finally:
        step.close()


if __name__ == "__main__":
    main()
