"""Build offline native-CAD test poses; never opens a port or controls hardware."""
import argparse
import hashlib
import itertools
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


SOURCE_SHA256 = "9b7e1588cee22b75f198261e0a6c97dbd56b3bdd7e2e10b1c5b68f9e39c2d61b"
# Explicit instance mapping verified against the imported V1.2 STEP and mesh centroids.
SUFFIXES = {
    "part8": "_X2_5e955ea7_X0_-1", "part7": "DM4340P-2", "part9": "D4-1",
    "hardware11": "_X2_70ed719487ba6bcd_X0_-2",
    "hardware12": "F3-M8_X2_63a8529b74038f74627f_X0_-2",
    "hardware13": "618-3 GB 276-94-2", "hardware14": "_X2_8f74627f63215708_X0_-1",
    "part6": "DM4340P-1", "part5": "DM4310-2", "part4": "D3-1",
    "part11": "MC02_complete-1",
    "part12": "_X2_52067535677f_X0_202603 (1)^_X2_88c5914d4f53_X0_1-1",
    "part13": "_X2_52067535677f_X0_202603 (1)^_X2_88c5914d4f53_X0_1-2",
    "part2": "DM4310-1", "part3": "D2-1",
    "hardware6": "_X2_70ed719487ba6bcd_X0_-1",
    "hardware7": "F3-M8_X2_63a8529b74038f74627f_X0_-1",
    "hardware9": "618-3 GB 276-94-1", "hardware15": "_X2_8f74627f63215708_X0_-2",
    "part0": "_X2_7684_X0_-1", "part10": "DM4310-3",
    "hardware22": "F3-M8_X2_63a8529b74038f74627f_X0_-3",
    "hardware23": "618-3 GB 276-94-3", "part1": "_X2_53f0706f593490e8706f7f69_X0_-1",
}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def origin(element):
    if element is None:
        return np.eye(4)
    r, p, y = map(float, element.get("rpy", "0 0 0").split())
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    t = np.eye(4)
    t[:3, :3] = (
        np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    )
    t[:3, 3] = list(map(float, element.get("xyz", "0 0 0").split()))
    return t


def from_sw(values):
    a = np.array(values, dtype=float)
    require(a.shape == (16,) and np.isfinite(a).all(), "Invalid native transform")
    require(abs(a[12] - 1) < 1e-10, "Scaled CAD components are not supported")
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = a[:9].reshape(3, 3).T, a[9:12]
    require(np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), rtol=0, atol=1e-9), "Non-rigid transform")
    require(abs(np.linalg.det(t[:3, :3]) - 1) < 1e-9, "Reflected component")
    return t


def to_sw(t):
    return [*t[:3, :3].T.flatten().tolist(), *t[:3, 3].tolist(), 1, 0, 0, 0]


def negative_y(degrees):
    a = np.deg2rad(degrees)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, -s, 0], [0, 1, 0, 0], [s, 0, c, 0], [0, 0, 0, 1]])


def moved_frames(zero, degrees):
    require(len(degrees) == 5 and np.isfinite(degrees).all(), "Invalid pose")
    require(degrees[0] == 0, "J1 remains outside this validation scope")
    frames = [zero[0]]
    for k, q in enumerate(degrees):
        frames.append(frames[k] @ np.linalg.inv(zero[k]) @ zero[k + 1] @ negative_y(q))
    return frames


def verify_rigid_groups(components, original):
    """All members of a link must receive one identical assembly-space motion."""
    motions = {}
    require(len(components) == len(original), "Incomplete pose")
    require(len({c["name"] for c in components}) == len(original), "Duplicate pose instances")
    for c in components:
        require(c["name"] in original, "Unknown pose instance")
        delta = from_sw(c["transform"]) @ np.linalg.inv(from_sw(original[c["name"]]))
        previous = motions.setdefault(c["link"], delta)
        require(np.allclose(delta, previous, rtol=0, atol=1e-9),
                f"Rigid link pulled apart: {c['link']}: {c['name']}")


def mesh_properties(path, scale):
    data = path.read_bytes()
    count = int.from_bytes(data[80:84], "little")
    require(len(data) == 84 + 50 * count, f"Expected binary STL: {path}")
    dtype = np.dtype({"names": ["v"], "formats": [("<f4", (3, 3))], "offsets": [12], "itemsize": 50})
    v = np.frombuffer(data, dtype=dtype, count=count, offset=84)["v"].astype(float) * scale
    tetrahedra = np.einsum("ij,ij->i", v[:, 0], np.cross(v[:, 1], v[:, 2])) / 6
    volume = tetrahedra.sum()
    require(abs(volume) > 1e-15, f"Degenerate mesh volume: {path}")
    center = ((v.sum(axis=1) / 4) * tetrahedra[:, None]).sum(axis=0) / volume
    return center, abs(volume)


def build(inspection, package, source, grid=0, mesh_report=None):
    require(hashlib.sha256(source.read_bytes()).hexdigest() == SOURCE_SHA256, "Wrong source STEP")
    native = json.loads(inspection.read_text(encoding="utf-8"))
    require(len(native["components"]) == 24, "Expected the complete 24-instance assembly")
    robot = ET.parse(package / "Ylamp.urdf").getroot()
    joints, links = robot.findall("joint"), robot.findall("link")
    require([j.get("name") for j in joints] == ["J1", "J2", "J3", "J4", "J5"], "Unexpected joint order")
    for j in joints:
        require(j.find("axis").get("xyz") == "0 -1 0", "Unexpected joint axis")
    chain = ["base_link"] + [j.find("child").get("link") for j in joints]
    cad, members, mapping = {}, {}, []
    urdf_zero = [np.eye(4)]
    for j in joints:
        urdf_zero.append(urdf_zero[-1] @ origin(j.find("origin")))
    for link in links:
        k = chain.index(link.get("name"))
        for visual in link.findall("visual"):
            part = visual.get("name")
            candidates = [c for c in native["components"] if c["Name"].endswith(SUFFIXES[part])]
            require(len(candidates) == 1, f"Ambiguous native instance: {part}")
            c = candidates[0]
            cad[part], members[part] = c, k
    require(len({c["Name"] for c in cad.values()}) == 24, "Duplicate CAD instance assignment")
    root = from_sw(cad["part7"]["Transform"])
    for link in links:
        k = chain.index(link.get("name"))
        for visual in link.findall("visual"):
            part = visual.get("name")
            mesh = visual.find("geometry/mesh")
            path = package / mesh.get("filename").split("package://ylamp_description/")[1]
            center, volume = mesh_properties(path, np.array(list(map(float, mesh.get("scale").split()))))
            expected = root @ urdf_zero[k] @ origin(visual.find("origin")) @ np.r_[center, 1]
            actual = from_sw(cad[part]["Transform"]) @ np.r_[cad[part]["Centroid"], 1]
            distance = np.linalg.norm(actual[:3] - expected[:3]) * 1000
            # The centroid checks identify instances, not an exact mesh/B-rep equivalence proof.
            require(distance < 4, f"Native/mesh mapping exceeds 4 mm: {part}: {distance}")
            require(abs(cad[part]["Volume"] / volume - 1) < .15, f"Volume mismatch: {part}")
            mapping.append(dict(part=part, name=cad[part]["Name"], link=chain[k],
                                centroidDifferenceMm=float(distance),
                                meshVolumeM3=float(volume), nativeVolumeM3=cad[part]["Volume"]))
    frames = [from_sw(cad[p]["Transform"]) for p in
              ["part7", "part7", "part6", "part5", "part2", "part10"]]
    differences = []
    for k in range(1, 6):
        expected = root @ urdf_zero[k]
        axis_error = np.linalg.norm(frames[k][:3, 1] - expected[:3, 1])
        require(axis_error < 1e-8, f"Joint axis direction mismatch: J{k}")
        delta = (frames[k][:3, 3] - expected[:3, 3]) * 1000
        differences.append(dict(joint=f"J{k}", originDifferenceMm=delta.tolist(),
                                originDistanceMm=float(np.linalg.norm(delta)),
                                axisDirectionError=float(axis_error)))
    main = [m["part"] for m in mapping if m["part"].startswith("part")]
    zero_inverse = [np.linalg.inv(t) for t in frames]
    original = {c["Name"]: c["Transform"] for c in cad.values()}

    def pose(label, degrees, selected):
        moved = moved_frames(frames, degrees)
        components = []
        # SolidWorks' manager can inspect the entire assembly: move the hardware too.
        # Filter the reporting scope afterwards, never leave unselected bodies at CAD zero.
        for part in cad:
            k = members[part]
            t = moved[k] @ zero_inverse[k] @ from_sw(cad[part]["Transform"])
            components.append(dict(part=part, name=cad[part]["Name"], link=chain[k], transform=to_sw(t)))
        verify_rigid_groups(components, original)
        return dict(label=label, cadDegrees=degrees, components=components,
                    crossLinkScope=[cad[part]["Name"] for part in selected])

    tests = [
        ("zero_full", [0, 0, 0, 0, 0], main),
        ("single_J2", [0, 68.5, 0, 0, 0], ["part1", "part8"]),
        ("single_J3", [0, 0, 9.75, 0, 0], ["part1", "part8"]),
        ("coupled_edge_full", [0, 68.5, 9.75, 0, 0], main),
        ("coupled_interior_full", [0, 65, 8, 0, 0], main),
        ("changed_wrist", [0, 68.5, 9.75, 0, 26.75], ["part1", "part8"]),
    ]
    limits = [[float(j.find("limit").get(side)) * 180 / np.pi for side in ("lower", "upper")]
              for j in joints]
    seen = {tuple(t[1]) for t in tests}

    def add_pose(label, degrees):
        require(len(degrees) == 5 and np.isfinite(degrees).all() and degrees[0] == 0, "Invalid added pose")
        require(all(lo - 1e-6 <= q <= hi + 1e-6 for q, (lo, hi) in zip(degrees, limits)),
                f"Pose outside CAD limits: {label}")
        key = tuple(round(q, 8) for q in degrees)
        if key not in seen:
            seen.add(key)
            tests.append((label, degrees, list(cad)))

    require(grid == 0 or 2 <= grid <= 5, "Grid must be zero (disabled) or 2..5 points per axis")
    if grid:
        for k, values in enumerate(itertools.product(*(np.linspace(*v, grid).tolist() for v in limits[1:]))):
            add_pose(f"grid_{k:04d}", [0, *values])
    mesh_source = None
    if mesh_report:
        mesh_data = json.loads(mesh_report.read_text(encoding="utf-8"))
        require(mesh_data["input"]["urdfSha256"] ==
                hashlib.sha256((package / "Ylamp.urdf").read_bytes()).hexdigest(), "Mesh report URDF mismatch")
        for k, sample in enumerate(mesh_data["additionalCollisionSamples"]):
            add_pose(f"mesh_witness_{k:04d}", sample["cadDegrees"])
        mesh_source = dict(path=str(mesh_report), sha256=hashlib.sha256(mesh_report.read_bytes()).hexdigest())
    return dict(
        schema="ylamp-solidworks-native-test-plan-v2",
        sourceStep=str(source), sourceStepSha256=SOURCE_SHA256,
        urdfSha256=hashlib.sha256((package / "Ylamp.urdf").read_bytes()).hexdigest(),
        inspectionPath=str(inspection), inspectionSha256=hashlib.sha256(inspection.read_bytes()).hexdigest(),
        coordinateMethod="Exact imported STEP zero transforms; five axes through native motor local Y",
        limitations=["Motor-axis inference, not recovered SolidWorks mates or a Motion study",
                     "CAD angles, not encoder angles; J1 fixed at zero",
                     "Native STEP and URDF axis origins differ; no firmware limits approved",
                     "Discrete solid checks do not certify continuous motion or a safe workspace"],
        componentMapping=mapping, jointOriginComparison=differences,
        rigidGroupsVerified=True, cadLimitsDegrees=limits, gridPointsPerAxis=grid, meshReport=mesh_source,
        poses=[pose(*test) for test in tests],
        displayPoses=[pose(label, degrees, list(cad)) for label, degrees, _ in tests],
    )


def self_test():
    t = origin(ET.fromstring('<origin xyz=".1 .2 .3" rpy="1.5707963267948966 0 1.5707963267948966"/>'))
    assert np.allclose(t @ [0, 0, 1, 1], [1.1, .2, .3, 1])
    assert np.allclose(from_sw(to_sw(t)), t)
    for index, value in [(0, -1), (1, 1e-6), (12, 2), (9, float("nan"))]:
        invalid = to_sw(np.eye(4))
        invalid[index] = value
        try:
            from_sw(invalid)
        except ValueError:
            pass
        else:
            raise AssertionError("Invalid native transform accepted")
    assert np.allclose(negative_y(90) @ [1, 0, 0, 1], [0, 0, 1, 1])
    zeros = [np.eye(4), np.eye(4)] + [origin(ET.fromstring(
        f'<origin xyz="{k/10} .02 .03" rpy=".2 .3 .4"/>')) for k in range(1, 5)]
    assert all(np.allclose(a, b) for a, b in zip(zeros, moved_frames(zeros, [0] * 5)))
    moved = moved_frames(zeros, [0, 30, 0, 0, 0])
    delta = moved[2] @ np.linalg.inv(zeros[2])
    assert all(np.allclose(moved[k], delta @ zeros[k]) for k in range(2, 6))
    try:
        moved_frames(zeros, [1, 0, 0, 0, 0])
    except ValueError:
        pass
    else:
        raise AssertionError("Nonzero J1 accepted")
    original = {"a": to_sw(zeros[0]), "b": to_sw(zeros[2]), "c": to_sw(zeros[4])}
    components = [dict(name=n, link="rigid", transform=to_sw(delta @ from_sw(t)))
                  for n, t in original.items()]
    verify_rigid_groups(components, original)
    components[1]["transform"][9] += .01
    try:
        verify_rigid_groups(components, original)
    except ValueError:
        pass
    else:
        raise AssertionError("Pulled-apart rigid group accepted")
    print("SELF_TEST PASS: URDF RPY, native transforms, negative-Y axes, rigid motion and pulled-apart group rejection")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--inspection", type=Path)
    parser.add_argument("--package", type=Path)
    parser.add_argument("--step", type=Path)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--grid", type=int, choices=[0, 2, 3, 4, 5], default=0,
                        help="Native J2..J5 grid points per axis: 0 or 2..5")
    parser.add_argument("--mesh-report", type=Path, help="Also verify counterexamples from the same-URDF mesh report")
    args = parser.parse_args()
    if args.self_test:
        self_test()
    else:
        require(all([args.inspection, args.package, args.step, args.out]), "Pass --inspection --package --step --out")
        result = build(args.inspection, args.package, args.step, args.grid, args.mesh_report)
        with args.out.open("x", encoding="utf-8") as stream:
            json.dump(result, stream, indent=2, ensure_ascii=False, allow_nan=False)
            stream.write("\n")
        print(json.dumps(dict(output=str(args.out), poses=len(result["poses"]),
                             rigidGroupsVerified=result["rigidGroupsVerified"],
                             joints=result["jointOriginComparison"]), indent=2))
