"""Historical all-yaw geometry check for config/five_yaw_box.def; no device I/O.

Moving/moving distances and floor heights are yaw-invariant. Check their
entire projected box with the archived J1=0 table library, NOT today's
per-axis admission. For every fixed/moving pair separately prove separation.
"""
import argparse
import ctypes as ct
import hashlib
import itertools
import json
from pathlib import Path
import re

import numpy as np
from export_five_profile import ROOT, TABLE_SHA, pinned
from workspace_geometry import read, write
from workspace_verify import BoundsChecker, motion_bound

GEOMETRY_LIBRARY_SHA = "ac04a62711a6afff3cb0aa9e61da5bead731983ae79ee9b218cd9c4ce408c02e"


def triangles(directory, mesh, transform):
    v = np.fromfile(directory / (mesh+".vertices"), "<f4").reshape(-1, 3)
    faces = np.fromfile(directory / (mesh+".indices"), "<u4").reshape(-1, 3)
    return (v @ transform[:3, :3].T + transform[:3, 3])[faces]


def base_mount(directory, pair, guard):
    # Use precisely the manifest's pair meshes, including its installation masks.
    def mesh(tag):
        return triangles(directory, pair[tag+"Mesh"], pair[tag]["matrix"])
    a, b = mesh("a"), mesh("b")
    ymin, ymax = a[:, :, 1].min(), b[:, :, 1].max()
    if ymin - ymax > guard:
        return "masked_axial", float(ymin-ymax)
    # Only these faces can approach within guard in height. A unit horizontal
    # projection bounds radius from below across an entire triangle, not just
    # the minimum vertex radius. The maximum vertex radius bounds it above.
    fixed = a[a[:, :, 1].min(axis=1) <= ymax+guard][:, :, [0, 2]]
    moving = b[b[:, :, 1].max(axis=1) >= ymin-guard][:, :, [0, 2]]
    n = fixed.mean(axis=1)
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-30)
    return "radial_in_axial_overlap", float((fixed*n[:, None, :]).sum(axis=-1).min()
                                             - np.linalg.norm(moving, axis=-1).max())


def audit(source, library, snapshot):
    if hashlib.sha256(library.read_bytes()).hexdigest() != GEOMETRY_LIBRARY_SHA:
        raise ValueError("Historical geometry audit requires its archived table library; "
                         "per-axis travel admission is not a collision certificate")
    import sys
    sys.path.insert(0, str(ROOT / "tools"))
    from dm_monitor import JOINT_REFERENCES
    from five_identify import HW, WORKSPACE_CAD
    box_def = ROOT / "config/five_yaw_box.def"
    half = np.array([float(x) for x in re.search(r"^FIVE_YAW_BOX\(([^)]+)\)$",
                    box_def.read_text(), re.M)[1].split(",")])
    if half.shape != (5,) or not np.isfinite(half).all() or not (half > 0).all():
        raise ValueError("Invalid five-axis box")
    software_center = WORKSPACE_CAD-HW[:, 6]
    for j, radius in enumerate(half, 1):
        ref = JOINT_REFERENCES[j]
        if (software_center[j-1]-radius < np.degrees(ref.lower_rad)+1 or
                software_center[j-1]+radius > np.degrees(ref.upper_rad)-1):
            raise ValueError(f"J{j} lacks the measured travel inset")
    arr = ct.c_double * 5
    band = (ct.c_double * 2)()
    box_query = ct.CDLL(str(library.resolve())).FiveWorkspace_BoxJ3Band
    box_query.argtypes, box_query.restype = [ct.POINTER(ct.c_double)]*3, ct.c_bool
    a, b = software_center-half, software_center+half
    a[0] = b[0] = 0.; b[2] = a[2]
    if (not box_query(arr(*a), arr(*b), band) or band[0] > a[2] or
            band[1] < software_center[2]+half[2]):
        raise ValueError("Original J1=0 tables do not cover this entire projected box")
    base = source / "baseline"
    directory = Path(read(base/"config.json")["geometryDirectory"]) / "fcl_meshes"
    manifest = read(directory/"manifest.json")
    pairs = {p["name"]: p for p in manifest["pairs"]}
    ch = BoundsChecker(base)
    if not np.array_equal(ch.origins[0], np.eye(4)):
        raise ValueError("Yaw origin/axis changed; axial/radial argument not valid")
    guard = 1e-6
    cases = {}
    mesh_hashes = {f.name: hashlib.sha256(f.read_bytes()).hexdigest()
                   for f in directory.iterdir() if f.suffix in (".indices", ".vertices") or f.name=="manifest.json"}
    for label, hashes in zip(("baseline", "alignment"), TABLE_SHA):
        cfg = read(source/label/"config.json")
        evidence = json.loads(pinned(source.parent/f"workspace_factored_regions_20261117_{label}_v10"
                                     /"factored_regions.json", hashes[0]))
        pinned_meshes = {Path(k).name: h for k, h in evidence["priorInputFileSha256"].items()
                         if Path(k).parent.name == "fcl_meshes"}
        if (cfg["signs"] != [1]*5 or cfg["offsetCadDeg"] != evidence["offsetCadDeg"] or
                Path(cfg["geometryDirectory"])/"fcl_meshes" != directory or mesh_hashes != pinned_meshes):
            raise ValueError("Geometry or mapping differs from the zero-yaw projection evidence")
        center = np.array(cfg["offsetCadDeg"])
        ch.place(center)
        rows = {}
        for name, constraint in ch.constraints.items():
            p = constraint.get("pair")
            if p is None or p["low"] != 0:
                continue
            if p["a"]["linkIndex"] != 0 or p["b"]["linkIndex"] == 0:
                raise ValueError(f"Unexpected fixed/moving order: {name}")
            def heights(body):
                t = body["matrix"]
                return body["vertices"] @ t[1, :3] + t[1, 3]
            ah, bh = heights(p["a"]), heights(p["b"])
            rho = np.radians(half); rho[0] = 0.  # yaw changes no vertex height
            gap = max(ah.min()-bh.max(), bh.min()-ah.max()) - motion_bound(constraint["radius"], rho)
            how = "axial_after_all_non_yaw_joint_excursions"
            if gap <= guard and p["high"] == 1:
                how, gap = base_mount(directory, {**p, "aMesh": pairs[name]["aMesh"],
                                                   "bMesh": pairs[name]["bMesh"]}, guard)
            if not gap > guard:
                raise ValueError(f"{label}: not proven for {name}, {gap} m")
            rows[name] = dict(test=how, separation_test_mm=1000*gap)
        if len(rows) != 24 or len(ch.constraints) != 89:
            raise ValueError("Unexpected V1.2 fixed/total constraint count")
        for signs in itertools.product((-1, 1), repeat=5):
            if ch.hits(center+half*signs):
                raise ValueError(f"{label}: native FCL hit at box corner {signs}")
        sw = np.array(read(snapshot)["snapshot"]["software_deg"])
        hits = {}
        for yaw in (sw[0], 0., 45., 90., 180., -90., -180.):
            q = sw.copy(); q[0] = yaw
            hits[str(yaw)] = ch.hits(q + HW[:, 6])
        cases[label] = dict(fixed_pairs=rows, recorded_pose_hits_by_yaw=hits, fcl_clear_corners=32,
                            projected_evidence_sha256=hashes[0])
    return dict(half_width_deg=half.tolist(), current_software_center_deg=software_center.tolist(),
                snapshot_cad_offsets_deg=HW[:, 6].tolist(),
                guard_m=guard, constraints=len(ch.constraints),
                old_zero_yaw_J3_band=list(band), cases=cases,
                manifest_sha256=hashlib.sha256((directory/"manifest.json").read_bytes()).hexdigest(),
                library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),
                snapshot_sha256=hashlib.sha256(snapshot.read_bytes()).hexdigest(),
                scope="Nominal mesh box only; installation masks retained; excludes tracking, cables, obstacles and hardware.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for flag in ("source", "library", "snapshot", "output"):
        parser.add_argument("--"+flag, required=True, type=Path)
    args = parser.parse_args()
    result = audit(args.source, args.library, args.snapshot)
    write(args.output, result)
    print("PASS: ±", result["half_width_deg"], "deg; constraints:", result["constraints"],
          "; fixed pairs per mapping:", len(result["cases"]["baseline"]["fixed_pairs"]))
