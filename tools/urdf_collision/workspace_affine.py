"""Offline shoulder/elbow slabs, using the existing mesh/motion-bound verifier."""
import argparse
import hashlib
import itertools
import math
from pathlib import Path
import time

import numpy as np

from workspace_geometry import read, write
from workspace_survey import (audit_partition_cover, inherited_box_partition,
                              inherited_constraint_scope, joint3_tables,
                              projected_gaps)
from workspace_verify import BoundsChecker, factored_membership, motion_bound, partition_regions


def enclosure(bounds, weight):
    """Original software-angle AABB enclosing u=(q1,q2,q3+w*q2,q4,q5)."""
    bounds = np.asarray(bounds, dtype=float)
    if (bounds.shape != (5, 2) or not np.isfinite(bounds).all()
            or np.any(bounds[:, 0] > bounds[:, 1]) or not math.isfinite(weight)):
        raise ValueError("Expected finite ordered transformed bounds and weight")
    result = bounds.copy()
    shifts = weight*bounds[1]
    result[2] = [math.nextafter(bounds[2, 0]-max(shifts), -math.inf),
                 math.nextafter(bounds[2, 1]-min(shifts), math.inf)]
    return result


class SlabChecker:
    """Conservative adapter; the underlying checker always receives CAD angles."""
    def __init__(self, checker, weight, offset):
        self.checker, self.weight = checker, float(weight)
        self.offset = np.asarray(offset, dtype=float)
        if not math.isfinite(self.weight) or self.offset.shape != (5,) or not np.isfinite(self.offset).all():
            raise ValueError("Invalid affine coordinates")
        self.constraints = {}
        for name, item in checker.constraints.items():
            radius = np.asarray(item["radius"]).copy()
            # f(a)=2*sin(min(a,pi)/2) is increasing, subadditive and concave:
            # f(h3+|w|*h2) <= f(h3)+max(1,|w|)*f(h2). No cancellation assumed.
            radius[1] += max(1., abs(self.weight))*radius[2]
            self.constraints[name] = dict(radius=radius)

    def place(self, u):
        self.u = np.asarray(u, dtype=float).copy()
        self.cad = self.u.copy()
        self.cad[2] -= self.weight*self.u[1]
        self.cad += self.offset
        self.checker.place(self.cad)

    def lower_distance(self, name, u, needed=math.inf):
        assert np.array_equal(u, self.u), "Distance must refer to the placed center"
        return self.checker.lower_distance(name, self.cad, needed)


def inherited_slabs(known, bounds, weight):
    """Inscribed u-boxes only; an image AABB is NOT a valid inherited safe box."""
    result, sources = [], []
    for index, original in enumerate(np.asarray(known)):
        clipped = original.copy()
        for axis in (0, 1, 3, 4):
            clipped[axis, 0] = max(clipped[axis, 0], bounds[axis, 0])
            clipped[axis, 1] = min(clipped[axis, 1], bounds[axis, 1])
        if np.any(clipped[[1, 3, 4], 0] >= clipped[[1, 3, 4], 1]):
            continue
        width = clipped[2, 1]-clipped[2, 0]
        if width <= 0:
            continue
        count = min(8, max(1, math.ceil(2*abs(weight)*np.ptp(clipped[1])/width)))
        cuts = np.linspace(*clipped[1], count+1)
        for low, high in zip(cuts, cuts[1:]):
            box = clipped.copy()
            box[1] = [low, high]
            shifts = weight*box[1]
            box[2] = [max(bounds[2, 0], clipped[2, 0]+max(shifts)+1e-10),
                      min(bounds[2, 1], clipped[2, 1]+min(shifts)-1e-10)]
            if box[2, 0] >= box[2, 1]:
                continue
            original_enclosure = enclosure(box, weight)
            assert np.all(original_enclosure[:, 0] >= original[:, 0])
            assert np.all(original_enclosure[:, 1] <= original[:, 1])
            result.append(box)
            sources.append(index)
    return np.asarray(result).reshape(-1, 5, 2), sources


def self_test():
    box = np.array([[0, 0], [-2, 3], [10, 20], [-30, 30], [-4, 5]], dtype=float)
    for weight in (-2., -.5, 0., .5, 1., 1.5, 2.):
        outer = enclosure(box, weight)
        for corner in itertools.product(*box):
            q = np.asarray(corner)
            q[2] -= weight*q[1]
            assert np.all((q >= outer[:, 0]) & (q <= outer[:, 1]))
        known = np.array([outer])
        inherited, sources = inherited_slabs(known, box, weight)
        assert len(inherited) and len(inherited) == len(sources)
        for item, index in zip(inherited, sources):
            original = enclosure(item, weight)
            assert np.all(original[:, 0] >= known[index, :, 0])
            assert np.all(original[:, 1] <= known[index, :, 1])
        radius = np.array([.1, .2, .3, .4, .5])
        transformed = radius.copy()
        transformed[1] += max(1., abs(weight))*radius[2]
        for half in np.random.default_rng(7).uniform(0, 4*math.pi, (1000, 5)):
            original_half = half.copy()
            original_half[2] += abs(weight)*half[1]
            assert motion_bound(radius, original_half) <= motion_bound(transformed, half)+1e-12
    class Fake:
        constraints = {"test": {"radius": np.array([0., 1., 2., 0., 0.])}}
        def place(self, q):
            self.placed = q.copy()
        def lower_distance(self, name, q, needed):
            assert np.array_equal(q, self.placed)
            return 10.
    fake = Fake()
    adapter = SlabChecker(fake, 1.5, [0, 2, -3, 0, 4])
    u = np.array([0., 4., 12., -20., 2.])
    adapter.place(u)
    assert np.array_equal(fake.placed, [0, 6, 3, -20, 6])
    assert adapter.lower_distance("test", u) == 10.
    assert np.array_equal(adapter.constraints["test"]["radius"], [0, 4, 2, 0, 0])
    try:
        enclosure(box, math.inf)
    except ValueError:
        pass
    else:
        raise AssertionError("Invalid affine weight accepted")
    print("affine self-test passed", flush=True)


def study(build, extension, out, seconds):
    started = time.perf_counter()
    print("Loading original verified tables", flush=True)
    tables, identities = joint3_tables(build, extension)
    configs = [read(extension / t["mapping"] / "config.json") for t in tables]
    assert configs[0]["geometryDirectory"] == configs[1]["geometryDirectory"]
    print("Loading shared high-resolution mesh checker", flush=True)
    checker = BoundsChecker(extension / tables[0]["mapping"])
    scopes = []
    for table, config in zip(tables, configs):
        parent_path = extension / table["mapping"] / "gap_refinement.json"
        parent = read(parent_path)
        assert parent["constraintsAccountedFor"] == 89 and parent["completeOtherGroupsInherited"] == 7
        identities.update(parent["inputFileSha256"])
        config_path = parent_path.with_name("config.json")
        identities[str(config_path.resolve())] = hashlib.sha256(config_path.read_bytes()).hexdigest()
        scopes.append(inherited_constraint_scope(table, checker, config, identities))
    print("Checking", len(identities), "input identities", flush=True)
    for name, digest in identities.items():
        with Path(name).open("rb") as stream:
            assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, name
    domains = np.asarray([t["admissibleSoftwareDomainDeg"] for t in tables])
    domain = np.column_stack((domains[:, :, 0].max(axis=0), domains[:, :, 1].min(axis=0)))
    survey_path = build / "workspace_frontier_survey_20261117_v1/frontier_survey.json"
    survey = read(survey_path)
    identities[str(survey_path.resolve())] = hashlib.sha256(survey_path.read_bytes()).hexdigest()
    # Broad shoulder/elbow slabs, not neighborhoods centered on individual targets.
    candidates = [
        ("positive_wrist", 1.5, [23, 44], [110, 139], [20, 120], [-15, 15]),
        ("negative_wrist", 1.5, [23, 44], [110, 139], [-120, -20], [-15, 15]),
        ("all_wrist", 1.5, [22, 44], [110, 136], [-226, 226], [-15, 20]),
        ("low_elbow_negative_wrist", 1.5, [-34, -25], [-134, -119], [-210, -130], [-14, 14]),
        ("low_elbow_positive_wrist", 1.5, [-34, -25], [-134, -119], [130, 210], [-14, 14]),
    ]
    out.mkdir(parents=True, exist_ok=False)
    rows, certified = [], []
    for index, (label, weight, shoulder, slab, wrist, fifth) in enumerate(candidates):
        bounds = np.asarray([[0, 0], shoulder, slab, wrist, fifth], dtype=float)
        enclosing = enclosure(bounds, weight)
        assert np.all(enclosing[:, 0] >= domain[:, 0]) and np.all(enclosing[:, 1] <= domain[:, 1])
        witnesses = []
        for row in survey["pointRows"]:
            if row["state"] != "inside_collision":
                continue
            u = np.asarray(row["softwareDeg"]).copy()
            u[2] += weight*u[1]
            if np.all((u >= bounds[:, 0]) & (u <= bounds[:, 1])):
                witnesses.append(row["sampleIndex"])
        proofs, complete = [], []
        if not witnesses:
            for table, config, scope in zip(tables, configs, scopes):
                phase = time.perf_counter()
                known = table["groups"][-1]["verifiedSoftwareBoxesDeg"]
                inscribed, sources = inherited_slabs(known, bounds, weight)
                # ponytail: cap rectangular inheritance before subtraction; the
                # remainder is still verified numerically, never presumed safe.
                volumes = np.prod(np.diff(inscribed[:, 1:], axis=2).squeeze(2), axis=1)
                selected = np.argsort(-volumes, kind="stable")[:32]
                inscribed, sources = inscribed[selected], [sources[i] for i in selected]
                inherited, pending = inherited_box_partition(bounds, inscribed, scope["newlyBoundConstraints"])
                print(label, table["mapping"], "inherited", len(inherited), "pending", len(pending), flush=True)
                adapter = SlabChecker(checker, weight, config["offsetCadDeg"])
                proof = partition_regions(adapter, bounds, scope["newlyBoundConstraints"], 100000,
                                          min_width=.02, max_seconds=seconds, seeds=pending)
                proof.update(inheritedLeaves=len(inherited), newlyVerifiedLeaves=len(proof["verified"]),
                             inheritedSourceBoxCount=len(known),
                             inscribedTransformedBoxesDeg=inscribed.tolist(),
                             inscribedSourceIndices=sources, mapping=table["mapping"],
                             coordinateSystem="u=(q1,q2,q3+w*q2,q4,q5); NOT CAD")
                proof["verified"] = inherited+proof["verified"]
                proof["verifiedFraction"] = 1.-proof["unknownFraction"]
                audit_partition_cover(dict(verified=proof["verified"]+proof["unknown"], unknown=[]), bounds)
                if not proof["unknown"]:
                    audit_partition_cover(proof, bounds)
                write(out / f"slab_{index}_{table['mapping']}.json", proof)
                complete.append(proof)
                proofs.append(dict(mapping=table["mapping"], unknownLeaves=len(proof["unknown"]),
                                   inheritedLeaves=len(inherited), newlyVerifiedLeaves=proof["newlyVerifiedLeaves"],
                                   verifiedFraction=proof["verifiedFraction"], nodes=proof["nodes"],
                                   elapsedSeconds=time.perf_counter()-phase,
                                   centerWitnesses=[w for leaf in proof["unknown"]
                                                    for w in leaf.get("centerWitnesses", [])][:10]))
                print(label, table["mapping"], "done", proofs[-1], flush=True)
        rows.append(dict(label=label, weight=weight, transformedBoundsDeg=bounds.tolist(),
                         originalSoftwareEnclosureDeg=enclosing.tolist(), obstacleSampleIndices=witnesses,
                         wholeSlabProven=len(proofs) == 2 and all(not p["unknownLeaves"] for p in proofs),
                         perMapping=proofs))
        if len(complete) == 2:
            gaps = [bounds[2].tolist()] if rows[-1]["wholeSlabProven"] else projected_gaps(
                [leaf["cadBoundsDeg"] for proof in complete for leaf in proof["unknown"]], bounds, 0.)
            for gap_index, gap in enumerate(gaps):
                if gap[1]-gap[0] < 1.:
                    continue
                sub = bounds.copy()
                sub[2] = gap
                for proof in complete:
                    clipped = []
                    for leaf_index, leaf in enumerate(proof["verified"]):
                        box = np.asarray(leaf["cadBoundsDeg"]).copy()
                        box[:, 0] = np.maximum(box[:, 0], sub[:, 0])
                        box[:, 1] = np.minimum(box[:, 1], sub[:, 1])
                        if np.all(box[1:, 0] < box[1:, 1]):
                            clipped.append(dict(cadBoundsDeg=box.tolist(), sourceLeafIndex=leaf_index))
                    audit_partition_cover(dict(verified=clipped, unknown=[]), sub)
                    write(out / f"certified_{index}_{gap_index}_{proof['mapping']}.json",
                          dict(verified=clipped, unknown=[], transformedBoundsDeg=sub.tolist(),
                               originalPartition=f"slab_{index}_{proof['mapping']}.json"))
                certified.append(dict(label=f"{label}_{gap_index}", weight=weight,
                                      transformedBoundsDeg=sub.tolist(),
                                      originalCandidateIndex=index, gapIndex=gap_index))
        write(out / f"candidate_{index}.json", rows[-1])
    # Independent finite membership regression; no point sample substitutes for a slab proof.
    recorded = np.asarray(configs[0]["recordedSoftwareRangesDeg"])
    points = np.random.default_rng(2026111731).uniform(recorded[:, 0], recorded[:, 1], (12000, 5))
    old = np.all((points >= domain[:, 0]) & (points <= domain[:, 1]), axis=1)
    for table in tables:
        print("Global regression", table["mapping"], flush=True)
        old &= factored_membership(table["groups"], points)
    additional = np.zeros(len(points), dtype=bool)
    for row in certified:
        u = points.copy()
        u[:, 2] += row["weight"]*u[:, 1]
        bounds = np.asarray(row["transformedBoundsDeg"])
        additional |= np.all((u >= bounds[:, 0]) & (u <= bounds[:, 1]), axis=1)
    corner_checks = []
    for row in certified:
        bounds = np.asarray(row["transformedBoundsDeg"])
        corners = np.asarray(list(itertools.product([0.], *bounds[1:])))
        corners = np.vstack((corners, bounds.mean(axis=1)))
        corners[:, 2] -= row["weight"]*corners[:, 1]
        for table, config in zip(tables, configs):
            collisions = [dict(softwareDeg=q.tolist(), pairs=hits) for q in corners
                          if (hits := checker.hits(q+config["offsetCadDeg"]))]
            assert not collisions, collisions
            corner_checks.append(dict(label=row["label"], mapping=table["mapping"],
                                      samples=len(corners), collisions=collisions))
    for path in out.glob("*.json"):
        identities[str(path.resolve())] = hashlib.sha256(path.read_bytes()).hexdigest()
    result = dict(previousExtension=str(extension.resolve()), coordinateDomainSoftwareDeg=domain.tolist(),
                  candidates=rows, certifiedSlabs=certified, independentCornerChecks=corner_checks,
                  constraintScopesPerMapping=scopes,
                  inputFileSha256=identities, elapsedSeconds=time.perf_counter()-started,
                  globalRegression=dict(seed=2026111731, samples=len(points), before=int(old.sum()),
                                        after=int((old | additional).sum()), added=int((additional & ~old).sum())),
                  implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  verifierSha256=hashlib.sha256(Path(__file__).with_name("workspace_verify.py").read_bytes()).hexdigest(),
                  limitations=["Whole slabs require both mappings and zero unknown leaves.",
                               "Inherited 84 constraints only inside the original common domain; J1=0.",
                               "No dynamic replay or hardware calibration for these candidate slabs.",
                               "Not a global maximum or physical-clearance/containment certificate."])
    write(out / "affine_study.json", result)
    print("affine study", result["globalRegression"], "whole slabs",
          sum(r["wholeSlabProven"] for r in rows), "certified subslabs", len(certified), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("self-test", "study"))
    parser.add_argument("--build", type=Path, default=Path("build"))
    parser.add_argument("--extension", type=Path)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--seconds", type=float, default=8.)
    args = parser.parse_args()
    if args.action == "self-test":
        self_test()
    else:
        assert args.extension is not None and args.out is not None
        assert math.isfinite(args.seconds) and args.seconds > 0
        study(args.build, args.extension, args.out, args.seconds)
