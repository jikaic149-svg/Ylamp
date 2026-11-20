"""Continuous, pose-dependent tip/base separation; offline, no physical control."""
import argparse
import hashlib
import itertools
import math
from pathlib import Path
import time

import numpy as np

from workspace_geometry import Checker, read, write
from workspace_survey import (certified_slice_intervals, inherited_constraint_scope,
                              joint3_query, joint3_tables, observed_segment_span)
from workspace_verify import (check_curve, factored_intervals, factored_membership, intersect_intervals,
                              merge_intervals, obb_gap, partition_regions)

BASES = ("part6", "part7", "part8", "part9")
GUARD = 1e-6  # Numerical separation, NOT a physical safety/registration margin.


def projection_gap(a, b, axis):
    """Positive projection separation proves the two complete vertex hulls disjoint."""
    axis = np.asarray(axis, dtype=float)
    norm = float(np.linalg.norm(axis))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("Expected a finite, nonzero separating direction")
    axis = axis/norm
    values = []
    for body in (a, b):
        m = body["matrix"]
        projected = body["vertices"] @ (m[:3, :3].T @ axis)+axis @ m[:3, 3]
        values.append((float(projected.min()), float(projected.max())))
    return max(values[0][0]-values[1][1], values[1][0]-values[0][1])


def candidate_axes(a, b):
    """Three most promising OBB axes; all are actual directions, never guessed gaps."""
    ra, rb = a["matrix"][:3, :3], b["matrix"][:3, :3]
    axes = np.concatenate((ra.T, rb.T, np.cross(ra.T[:, None, :], rb.T[None, :, :]).reshape(-1, 3)))
    norms = np.linalg.norm(axes, axis=1)
    axes = axes[norms > 1e-12]/norms[norms > 1e-12, None]
    gaps = (abs(axes @ (a["centerWorld"]-b["centerWorld"]))
            -abs(axes @ ra) @ a["half"]-abs(axes @ rb) @ b["half"])
    return axes[np.argsort(-gaps, kind="stable")[:3]]


def search_gap(a, b, needed, iterations=160):
    """Propose support directions; only full-vertex projection gaps are evidence."""
    pa = a["vertices"] @ a["matrix"][:3, :3].T+a["matrix"][:3, 3]
    pb = b["vertices"] @ b["matrix"][:3, :3].T+b["matrix"][:3, 3]
    v, best = pa.mean(axis=0)-pb.mean(axis=0), 0.
    # ponytail: bounded segment iterations can miss a separator; unknown stays rejected.
    for _ in range(iterations):
        norm = float(np.linalg.norm(v))
        if norm < 1e-12:
            break
        n = v/norm
        support = pa[int(np.argmin(pa@n))]-pb[int(np.argmax(pb@n))]
        if float(n@support) > best:
            best = max(best, projection_gap(a, b, n))
            if best > needed:
                break
        delta = support-v
        length2 = float(delta@delta)
        if length2 < 1e-30:
            break
        step = float(np.clip(-v@delta/length2, 0., 1.))
        if step < 1e-12:
            break
        v = v+step*delta
    return best


class EnvelopeChecker:
    """No BVH construction: original mesh vertices, rigid transforms and support bounds."""
    def __init__(self, source, search_directions=True):
        self.search_directions = search_directions
        self.config = read(source / "config.json")
        folder = Path(self.config["geometryDirectory"]) / "fcl_meshes"
        manifest = read(folder / "manifest.json")
        assert manifest["axes"] == [[0, -1, 0]]*5
        self.origins = [np.asarray(x).reshape(4, 4, order="F") for x in manifest["origins"]]
        self.bodies = {}
        for item in manifest["bodies"]:
            vertices = np.unique(np.fromfile(folder / (item["mesh"]+".vertices"),
                                            dtype="<f4").reshape(-1, 3), axis=0).astype(float)
            local = np.asarray(item["local"]).reshape(4, 4, order="F")
            lower, upper = vertices.min(axis=0), vertices.max(axis=0)
            extent = float(np.linalg.norm(vertices @ local[:3, :3].T+local[:3, 3], axis=1).max())
            radius = np.zeros(5)
            for j in range(item["linkIndex"]):
                radius[j] = extent+sum(np.linalg.norm(o[:3, 3])
                                      for o in self.origins[j+1:item["linkIndex"]])
            self.bodies[item["name"]] = dict(item, local=local, center=(lower+upper)/2,
                half=(upper-lower)/2, radius=radius,
                vertices=vertices if item["name"] in ("part1", *BASES) else None)
        self.constraints = {}
        for item in manifest["pairs"]:
            a, b = (self.bodies[item[k]] for k in ("a", "b"))
            low, high = sorted((a["linkIndex"], b["linkIndex"]))
            radius = max((a, b), key=lambda x: x["linkIndex"])["radius"].copy()
            radius[:low] = 0
            self.constraints[item["name"]] = dict(pair=dict(low=low, high=high), radius=radius)
        for body in self.bodies.values():
            if body["linkIndex"]:
                self.constraints[body["name"]+"/floor"] = dict(body=body, radius=body["radius"])

    def place(self, q):
        Checker.place(self, q)

    def lower_distance(self, name, q, needed=math.inf):
        if name == "part1/floor":
            tip = self.bodies["part1"]
            lower = .05874511-tip["centerWorld"][1]-tip["halfWorld"][1]
            if lower > needed:
                return float(lower)
            m = tip["matrix"]
            return float(.05874511-m[1, 3]-np.max(tip["vertices"] @ m[1, :3]))
        assert name in {f"part1/{base}" for base in BASES}
        a, b = self.bodies["part1"], self.bodies[name.split("/")[1]]
        lower = obb_gap(a, b)
        if lower > needed:
            return lower
        for axis in candidate_axes(a, b):
            lower = max(lower, projection_gap(a, b, axis))
            if lower > needed:
                break
        if self.search_directions and lower <= needed:
            lower = max(lower, search_gap(a, b, needed))
        return lower

    def margins(self, cad, names):
        self.place(cad)
        return [self.lower_distance(name, cad, GUARD) for name in names]


def context(build, extension):
    tables, identities = joint3_tables(build, extension)
    configs = [read(extension / table["mapping"] / "config.json") for table in tables]
    assert configs[0]["geometryDirectory"] == configs[1]["geometryDirectory"]
    checker = EnvelopeChecker(extension / tables[0]["mapping"])
    scopes = []
    for table, config in zip(tables, configs):
        path = extension / table["mapping"] / "gap_refinement.json"
        parent = read(path)
        assert parent["constraintsAccountedFor"] == 89 and parent["completeOtherGroupsInherited"] == 7
        identities.update(parent["inputFileSha256"])
        path = path.with_name("config.json")
        identities[str(path.resolve())] = hashlib.sha256(path.read_bytes()).hexdigest()
        scopes.append(inherited_constraint_scope(table, checker, config, identities))
    expected = {"part1/floor", *(f"part1/{base}" for base in BASES)}
    assert all(set(scope["newlyBoundConstraints"]) == expected for scope in scopes)
    for name, digest in identities.items():
        with Path(name).open("rb") as stream:
            assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, name
    domains = np.asarray([table["admissibleSoftwareDomainDeg"] for table in tables])
    domain = np.column_stack((domains[:, :, 0].max(axis=0), domains[:, :, 1].min(axis=0)))
    return tables, configs, checker, scopes, domain, identities


def self_test():
    vertices = np.asarray(list(itertools.product((-1., 1.), repeat=3)))
    def body(center):
        matrix = np.eye(4)
        matrix[:3, 3] = center
        return dict(vertices=vertices, matrix=matrix, centerWorld=np.asarray(center), half=np.ones(3))
    a, b = body([0, 0, 0]), body([5, 0, 0])
    assert projection_gap(a, b, [1, 0, 0]) == 3.
    assert projection_gap(a, b, [-2, 0, 0]) == 3.
    assert projection_gap(a, body([.5, 0, 0]), [1, 0, 0]) < 0
    assert max(projection_gap(a, b, n) for n in candidate_axes(a, b)) == 3.
    assert obb_gap(a, b) == 3.
    for invalid in ([0, 0, 0], [math.nan, 0, 0], [math.inf, 0, 0]):
        try:
            projection_gap(a, b, invalid)
        except ValueError:
            pass
        else:
            raise AssertionError("Invalid direction accepted")
    tetra = np.vstack((np.zeros(3), np.eye(3)))
    ta, tb = body([0, 0, 0]), body([.8, .8, .8])
    for item in (ta, tb):
        item["vertices"] = tetra
        item["centerWorld"] = item["matrix"][:3, 3]+.5
        item["half"] = np.full(3, .5)
    assert max(projection_gap(ta, tb, n) for n in candidate_axes(ta, tb)) < 0
    assert search_gap(ta, tb, GUARD) > .8
    assert search_gap(tb, ta, GUARD) > .8
    tb["matrix"][:3, 3] = [.2, .2, .2]
    assert search_gap(ta, tb, GUARD) == 0  # Overlap, not just failed OBB directions.
    tb["vertices"] = tetra*.1
    tb["matrix"][:3, 3] = [.1, .1, .1]
    assert search_gap(ta, tb, GUARD) == 0  # Full containment must not be admitted.
    tb["matrix"][:3, 3] = [2., 2., 2.]
    assert search_gap(ta, tb, GUARD, iterations=0) == 0
    assert search_gap(ta, tb, GUARD) > 0
    print("envelope self-test passed", flush=True)


def study(build, extension, out, native, seed=2026111732, samples=12000):
    started = time.perf_counter()
    print("Loading original tables, vertex supports and inherited constraint scope", flush=True)
    tables, configs, checker, scopes, domain, identities = context(build, extension)
    names = scopes[0]["newlyBoundConstraints"]
    recorded = np.asarray(configs[0]["recordedSoftwareRangesDeg"])
    survey_path = build / "workspace_frontier_survey_20261117_v1/frontier_survey.json"
    survey = read(survey_path)
    identities[str(survey_path.resolve())] = hashlib.sha256(survey_path.read_bytes()).hexdigest()
    validation, accepted_clear = [], []
    for row in survey["pointRows"]:
        if not row["state"].startswith("inside"):
            continue
        margins = [checker.margins(np.asarray(row["softwareDeg"])+c["offsetCadDeg"], names) for c in configs]
        passed = min(map(min, margins)) > GUARD
        assert not (passed and row["state"] == "inside_collision"), row
        if passed:
            accepted_clear.append(row["sampleIndex"])
        validation.append(dict(sampleIndex=row["sampleIndex"], state=row["state"], envelopePassed=passed,
                               marginsPerMappingMeters=margins))
    assert samples > 0 and seed >= 0
    points = np.random.default_rng(seed).uniform(recorded[:, 0], recorded[:, 1], (samples, 5))
    inside = np.all((points >= domain[:, 0]) & (points <= domain[:, 1]), axis=1)
    own = [factored_membership(table["groups"], points) & inside for table in tables]
    old = own[0] & own[1]
    after = [allowed.copy() for allowed in own]
    candidates = np.flatnonzero(inside & ~old)
    print("Checking", len(candidates), "old-unadmitted poses against continuous envelope", flush=True)
    novel = []
    for index in candidates:
        margins = []
        for i, config in enumerate(configs):
            values = checker.margins(points[index]+config["offsetCadDeg"], names)
            margins.append(values)
            after[i][index] |= min(values) > GUARD
        if after[0][index] and after[1][index]:
            novel.append(dict(sampleIndex=int(index), softwareDeg=points[index].tolist(),
                              oldAllowedPerMapping=[bool(x[index]) for x in own],
                              marginsPerMappingMeters=margins))
    both = after[0] & after[1]
    assert int(both.sum()-old.sum()) == len(novel)
    out.mkdir(parents=True, exist_ok=False)
    native_checks = []
    if native and novel:
        print("Loading independent full native FCL for", 2*len(novel), "novel-pose checks", flush=True)
        native_checker = Checker(extension / tables[0]["mapping"])
        for row in novel:
            for table, config in zip(tables, configs):
                hits = native_checker.hits(np.asarray(row["softwareDeg"])+config["offsetCadDeg"])
                assert not hits, (row, table["mapping"], hits)
                native_checks.append(dict(sampleIndex=row["sampleIndex"], mapping=table["mapping"], pairs=hits))
    write(out / "envelope_study.json", dict(
        previousExtension=str(extension.resolve()), constraintScopesPerMapping=scopes,
        coordinateDomainSoftwareDeg=domain.tolist(), guardMeters=GUARD,
        directionSearch="OBB, three vertex-support axes, then at most 160 support-direction iterations",
        formula="AND over mappings: old_allowed(q) OR min(analytic_support_separations(q)) > guard",
        validationAgainstPriorFcl=validation, priorClearTargetsAccepted=accepted_clear,
        globalRegression=dict(seed=seed, samples=len(points), insideCommonDomain=int(inside.sum()),
                              before=int(old.sum()), after=int(both.sum()), added=len(novel)),
        novelPoses=novel, independentNativeChecks=native_checks,
        inputFileSha256=identities, elapsedSeconds=time.perf_counter()-started,
        implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        limitations=["Continuous separating-support region, not arbitrary endpoint-to-endpoint path safety.",
                     "Original common domain and inherited 84 constraints remain mandatory; nominal J1=0.",
                     "Guard is numerical only; no physical registration/load/clearance/containment certification.",
                     "Not full-domain dynamics or a global-maximum certificate."]))
    print("envelope global", int(old.sum()), "->", int(both.sum()), "added", len(novel),
          "prior clear accepted", len(accepted_clear), "native checks", len(native_checks), flush=True)


def query(build, extension, software, out, seconds):
    tables, configs, checker, scopes, domain, identities = context(build, extension)
    q = np.asarray(software, dtype=float)
    if q.shape != (5,) or not np.isfinite(q).all() or q[0] != 0:
        raise ValueError("Expected five finite software angles with J1=0")
    fixed = [0, 1, 3, 4]
    if np.any(q[fixed] < domain[fixed, 0]) or np.any(q[fixed] > domain[fixed, 1]):
        raise ValueError("Fixed angles lie outside the original approved domain")
    out.mkdir(parents=True, exist_ok=False)
    own, rows = [], []
    for table, config, scope in zip(tables, configs, scopes):
        bounds = np.column_stack((q, q))+np.asarray(config["offsetCadDeg"])[:, None]
        bounds[2] = domain[2]+config["offsetCadDeg"][2]
        proof = partition_regions(checker, bounds, scope["newlyBoundConstraints"], 10000,
                                  min_width=.02, max_seconds=seconds)
        intervals = certified_slice_intervals(proof, bounds, config["offsetCadDeg"][2])
        old = factored_intervals(table, q, 2)
        own.append(merge_intervals(old+intervals))
        write(out / f"{table['mapping']}.json", proof)
        rows.append(dict(mapping=table["mapping"], oldJ3IntervalsDeg=old,
                         supportCertifiedJ3IntervalsDeg=intervals, unknownLeaves=len(proof["unknown"])))
    result = dict(softwareDeg=q.tolist(), commonJ3IntervalsDeg=intersect_intervals(*own),
                  perMapping=rows, inputFileSha256=identities,
                  note="Fixed J2/J4/J5 only; unknown support leaves not admitted, old proved intervals retained.")
    write(out / "envelope_query.json", result)
    print({key: value for key, value in result.items() if key != "inputFileSha256"}, flush=True)


def completeness(build, extension, source, out):
    """Classify the previous formula's rejected samples; never admit them by this audit."""
    started = time.perf_counter()
    path = source / "envelope_study.json"
    previous = read(path)
    assert previous["previousExtension"] == str(extension.resolve())
    tables, configs, _, scopes, domain, identities = context(build, extension)
    identities.update(previous["inputFileSha256"])
    identities[str(path.resolve())] = hashlib.sha256(path.read_bytes()).hexdigest()
    for name, digest in identities.items():
        with Path(name).open("rb") as stream:
            assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, name
    regression = previous["globalRegression"]
    recorded = np.asarray(configs[0]["recordedSoftwareRangesDeg"])
    points = np.random.default_rng(regression["seed"]).uniform(
        recorded[:, 0], recorded[:, 1], (regression["samples"], 5))
    inside = np.all((points >= domain[:, 0]) & (points <= domain[:, 1]), axis=1)
    old = inside.copy()
    for table in tables:
        old &= factored_membership(table["groups"], points)
    accepted = old.copy()
    for row in previous["novelPoses"]:
        assert np.array_equal(points[row["sampleIndex"]], row["softwareDeg"])
        accepted[row["sampleIndex"]] = True
    assert int(old.sum()) == regression["before"] and int(accepted.sum()) == regression["after"]
    indices = np.flatnonzero(inside & ~accepted)
    print("Native completeness audit:", len(indices), "previously rejected in-domain poses", flush=True)
    checker = Checker(extension / tables[0]["mapping"])
    rows = []
    for k, index in enumerate(indices):
        hits = [checker.hits(points[index]+config["offsetCadDeg"]) for config in configs]
        rows.append(dict(sampleIndex=int(index), softwareDeg=points[index].tolist(),
                         collisionPairsPerMapping=hits,
                         classification="native_collision" if any(hits) else "native_clear_unproved"))
        if k % 50 == 0:
            print("Classified", k+1, "/", len(indices), flush=True)
    survey_path = build / "workspace_frontier_survey_20261117_v1/frontier_survey.json"
    target = next(row for row in read(survey_path)["pointRows"] if row["sampleIndex"] == 5655)
    target_hits = [checker.hits(np.asarray(target["softwareDeg"])+config["offsetCadDeg"]) for config in configs]
    identities[str(survey_path.resolve())] = hashlib.sha256(survey_path.read_bytes()).hexdigest()
    clear = [row for row in rows if row["classification"] == "native_clear_unproved"]
    out.mkdir(parents=True, exist_ok=False)
    write(out / "completeness.json", dict(
        originalRegression=regression, originalDomainSoftwareDeg=domain.tolist(),
        rejectedInDomain=len(rows), rejectedNativeCollision=len(rows)-len(clear),
        rejectedNativeClearUnproved=len(clear), rejectedRows=rows,
        coveredFractionOfSampledNativeClear=regression["after"]/(regression["after"]+len(clear)),
        priorTarget5655=dict(softwareDeg=target["softwareDeg"], collisionPairsPerMapping=target_hits),
        constraintScopesPerMapping=scopes, inputFileSha256=identities,
        implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        elapsedSeconds=time.perf_counter()-started,
        limitations=["Diagnostic only: a native-clear sample does not add a surrounding allowed region.",
                     "Same fixed sample universe as previous report; not a fresh independent random draw.",
                     "Finite triangle-surface classification, not containment/physical clearance or maximality."]))
    print("Rejected:", len(rows)-len(clear), "native collision;", len(clear), "native clear/unproved", flush=True)


def replay(build, extension, source, out, previous_source=None):
    """Six new-domain loops; existing MuJoCo/FCL engines and acceptance checks."""
    from workspace_sim import simulate
    from workspace_geometry import run as native_trace
    from coupled_workspace import finish

    started = time.perf_counter()
    report_path = source / "envelope_study.json"
    previous = read(report_path)
    assert previous["previousExtension"] == str(extension.resolve())
    tables, configs, checker, scopes, domain, identities = context(build, extension)
    identities.update(previous["inputFileSha256"])
    identities[str(report_path.resolve())] = hashlib.sha256(report_path.read_bytes()).hexdigest()
    excluded = set()
    previous_checker = None
    if previous_source is not None:
        earlier_path = previous_source / "envelope_study.json"
        earlier = read(earlier_path)
        assert earlier["previousExtension"] == previous["previousExtension"]
        assert earlier["globalRegression"]["seed"] == previous["globalRegression"]["seed"]
        assert earlier["globalRegression"]["samples"] == previous["globalRegression"]["samples"]
        excluded = {row["sampleIndex"] for row in earlier["novelPoses"]}
        for row in previous["novelPoses"]:
            old_row = next((r for r in earlier["novelPoses"] if r["sampleIndex"] == row["sampleIndex"]), None)
            if old_row is not None:
                assert row["softwareDeg"] == old_row["softwareDeg"]
        assert excluded.issubset({row["sampleIndex"] for row in previous["novelPoses"]})
        identities[str(earlier_path.resolve())] = hashlib.sha256(earlier_path.read_bytes()).hexdigest()
        previous_checker = EnvelopeChecker(extension / tables[0]["mapping"], search_directions=False)
    for name, digest in identities.items():
        with Path(name).open("rb") as stream:
            assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, name
    inset = np.asarray(configs[0]["recordedSoftwareRangesDeg"]).copy()
    inset[1:, 0] += 1.
    inset[1:, 1] -= 1.
    candidates = sorted((r for r in previous["novelPoses"] if r["sampleIndex"] not in excluded),
                        key=lambda r: min(map(min, r["marginsPerMappingMeters"])), reverse=True)
    delta = np.array([0., 2., .75, 6., .75])
    selected, attempts = [], []
    for row in candidates[:30]:
        point = np.asarray(row["softwareDeg"])
        a, b = point-delta, point+delta
        if np.any(a < inset[:, 0]) or np.any(b > inset[:, 1]):
            continue
        intervals = joint3_query(tables, point)[0]
        if not intervals:
            continue
        anchor = point.copy()
        choices = [min(max(point[2], low+min(1., (high-low)/4)),
                       high-min(1., (high-low)/4)) for low, high in intervals]
        anchor[2] = min(choices, key=lambda x: abs(x-point[2]))
        route = np.asarray([anchor, a, b, anchor])
        if np.any(route < inset[:, 0]) or np.any(route > inset[:, 1]):
            continue
        assert np.all(route >= domain[:, 0]) and np.all(route <= domain[:, 1])
        proofs = []
        for table, config, scope in zip(tables, configs, scopes):
            offset = np.asarray(config["offsetCadDeg"])
            for segment, (start, end) in enumerate(zip(route, route[1:])):
                result = check_curve(checker, lambda t: start+t*(end-start)+offset,
                                     max_nodes=1500, names=scope["newlyBoundConstraints"])
                proofs.append(dict(mapping=table["mapping"], segment=segment, result=result))
                if result["status"] != "numerical_motion_bound_pass":
                    break
            if proofs[-1]["result"]["status"] != "numerical_motion_bound_pass":
                break
        passed = len(proofs) == 6 and all(
            p["result"]["status"] == "numerical_motion_bound_pass" for p in proofs)
        attempt = dict(sampleIndex=row["sampleIndex"], wholeBothMappingsProven=passed,
                       routeSoftwareDeg=route.tolist(), proofs=proofs)
        attempts.append(attempt)
        print("Support route", row["sampleIndex"], "whole proof", passed, flush=True)
        if passed:
            selected.append(dict(attempt, id=f"support_{row['sampleIndex']}_return",
                                 aSoftwareDeg=a.tolist(), bSoftwareDeg=b.tolist()))
        if len(selected) == 6:
            break
    assert selected, "No new whole path: never substitute the old successful test batch"
    out.mkdir(parents=True, exist_ok=False)
    write(out / "nominal_paths.json", dict(selectedRoutes=selected, attempts=attempts,
          constraintScopesPerMapping=scopes, inputFileSha256=identities,
          implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()))
    summaries = []
    for table, config in zip(tables, configs):
        mapping, offset = table["mapping"], np.asarray(config["offsetCadDeg"])
        own = out / mapping
        own.mkdir()
        geometry_path = build / f"workspace_outer_rule_20261117_{mapping}_v1/geometry.json"
        geometry = read(geometry_path)
        identities[str(geometry_path.resolve())] = hashlib.sha256(geometry_path.read_bytes()).hexdigest()
        geometry["paths"] = [dict(id=row["id"],
            waypointsCadDeg=(np.asarray(row["routeSoftwareDeg"])+offset).tolist()) for row in selected]
        write(own / "config.json", config)
        write(own / "geometry.json", geometry)
        write(own / "study.json", dict(parameters=[dict(type="continuous_support_envelope_paths")],
              selectedRoutes=selected, constraintScopesPerMapping=scopes, inputFileSha256=identities,
              limitations=["Original D, 84 inherited and 5 continuous nominal constraints, J1=0 only.",
                           "Finite actual full-five-axis FCL, not continuous actual tracking-tube safety.",
                           "Two unregistered mappings; no hardware safety or full-domain dynamics certification."]))
        simulate(own)
        native_trace(own, trace=True)
        dynamics = read(own / "dynamics.json")
        paths = {path["id"]: path for path in geometry["paths"]}
        for case in dynamics["cases"]:
            if case["id"] in paths:
                assert np.array_equal(case["commandWaypointsCadDeg"], paths[case["id"]]["waypointsCadDeg"])
        trace = read(own / "actual_trace.json")["samples"]
        projected = np.asarray([sample["cadDegrees"] for sample in trace])-offset
        j1_error = float(abs(projected[:, 0]).max())
        projected[:, 0] = 0.  # Membership only; independent native_trace kept actual J1.
        assert np.all(projected >= domain[:, 0]) and np.all(projected <= domain[:, 1])
        old_per_map = [factored_membership(t["groups"], projected) for t in tables]
        old_common = old_per_map[0] & old_per_map[1]
        previous_per_map = [allowed.copy() for allowed in old_per_map]
        support_checked = []
        for allowed, previous_allowed, target_config, scope in zip(
                old_per_map, previous_per_map, configs, scopes):
            indices = np.flatnonzero(~allowed)
            for index in indices:
                values = checker.margins(projected[index]+target_config["offsetCadDeg"],
                                         scope["newlyBoundConstraints"])
                assert min(values) > GUARD, (mapping, index, values)
                if previous_checker is not None:
                    previous_values = previous_checker.margins(
                        projected[index]+target_config["offsetCadDeg"], scope["newlyBoundConstraints"])
                    previous_allowed[index] |= min(previous_values) > GUARD
            support_checked.append(len(indices))
        previous_common = previous_per_map[0] & previous_per_map[1]
        labels = np.asarray([sample["case"] for sample in trace])
        counts, previous_counts, spans = {}, {}, {}
        for row in selected:
            for speed in ("slow", "brisk"):
                label = speed+"/"+row["id"]
                mask = labels == label
                counts[label] = int(np.sum(mask & ~old_common))
                assert counts[label] > 0, (mapping, label)
                previous_counts[label] = int(np.sum(mask & ~previous_common))
                assert previous_counts[label] > 0, (mapping, label, "Did not enter previous support exterior")
                case = next(c for c in dynamics["cases"] if c["speed"]+"/"+c["id"] == label)
                spans[label] = observed_segment_span(projected[mask], np.asarray(row["aSoftwareDeg"]),
                    np.asarray(row["bSoftwareDeg"]), 2*max(case["maxErrorDeg"])+1e-6)
        write(own / "support_actual_audit.json", dict(actualSamples=len(trace),
              projectedJ1MaximumErrorDeg=j1_error, supportChecksPerMapping=support_checked,
              novelSamplesPerNewRouteSpeed=counts, fourAxisObservedSpan=spans,
              previousSupportNovelSamplesPerNewRouteSpeed=previous_counts,
              newCommandsIdenticalToNominal=True, comparisonToleranceDeg=0.))
        finish(own)
        summary = read(own / "coupled_summary.json")
        assert summary["cases"] == summary["accepted"] == 2*(len(selected)+6)
        summaries.append(dict(mapping=mapping, cases=summary["cases"], accepted=summary["accepted"],
                              actualFclStates=summary["actualTraceSamples"],
                              maximumTrackingErrorDeg=summary["maximumTrackingErrorDeg"],
                              novelRouteSpeedLabels=len(counts), novelActualSamples=sum(counts.values()),
                              previousSupportNovelActualSamples=sum(previous_counts.values())))
    for path in out.rglob("*.json"):
        identities[str(path.resolve())] = hashlib.sha256(path.read_bytes()).hexdigest()
    write(out / "support_replay.json", dict(selectedRoutes=len(selected), perMapping=summaries,
          previousSource=str(previous_source.resolve()) if previous_source is not None else None,
          previousNovelTargetsExcluded=len(excluded), newCandidateTargets=len(candidates),
          totalCases=sum(s["cases"] for s in summaries),
          actualFclStates=sum(s["actualFclStates"] for s in summaries),
          inputFileSha256=identities, elapsedSeconds=time.perf_counter()-started,
          implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
          limitations=["Six representative new-domain paths, not full-domain dynamics or hardware clearance."]))
    print("Support replay complete", summaries, flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("self-test", "study", "query", "replay", "completeness"))
    parser.add_argument("--build", type=Path, default=Path("build"))
    parser.add_argument("--extension", type=Path)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--source", type=Path)
    parser.add_argument("--previous-source", type=Path)
    parser.add_argument("--native-audit", action="store_true")
    parser.add_argument("--seed", type=int, default=2026111732)
    parser.add_argument("--samples", type=int, default=12000)
    parser.add_argument("--software", nargs=5, type=float)
    parser.add_argument("--seconds", type=float, default=3.)
    args = parser.parse_args()
    if args.action == "self-test":
        self_test()
    else:
        assert args.extension is not None and args.out is not None
        if args.action == "study":
            study(args.build, args.extension, args.out, args.native_audit, args.seed, args.samples)
        elif args.action == "completeness":
            assert args.source is not None
            completeness(args.build, args.extension, args.source, args.out)
        elif args.action == "replay":
            assert args.source is not None
            replay(args.build, args.extension, args.source, args.out, args.previous_source)
        else:
            assert args.software is not None and math.isfinite(args.seconds) and args.seconds > 0
            query(args.build, args.extension, args.software, args.out, args.seconds)
