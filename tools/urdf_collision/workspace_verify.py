"""Clearance and conservative motion-bound experiments; offline, no CAD application."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import heapq
import html
import itertools
import json
import math
from pathlib import Path
import re
import time
import urllib.request

import fcl
import numpy as np

from coupled_workspace import conditional_intervals, policy_pose, sampled_intervals
from workspace_geometry import Checker, line, read, write


SOURCES = {
    "joint_limit_tables": "https://scikit-robot.readthedocs.io/en/latest/examples_guide/joint_limit_table.html",
    "ompl_validation": "https://ompl.kavrakilab.org/stateValidation.html",
    "drake_iris": "https://drake.mit.edu/doxygen_cxx/group__planning__iris.html",
    "drake_cspace_free_box": "https://drake.mit.edu/doxygen_cxx/classdrake_1_1geometry_1_1optimization_1_1_cspace_free_box.html",
    "mink_collision": "https://kevinzakka.github.io/mink/limits.html",
}


def sources(out):
    out.mkdir(parents=True, exist_ok=False)
    result = []
    for name, url in SOURCES.items():
        row = dict(name=name, url=url, retrievedUtc=datetime.now(timezone.utc).isoformat())
        try:
            request = urllib.request.Request(url, headers={"User-Agent": "Ylamp-offline-study/1.0"})
            with urllib.request.urlopen(request, timeout=25) as response:
                raw = response.read()
                row.update(status=response.status, finalUrl=response.url,
                           sha256=hashlib.sha256(raw).hexdigest())
            text = raw.decode("utf-8", errors="replace")
            text = re.sub(r"<(script|style)\b.*?</\1>", "", text, flags=re.S | re.I)
            text = html.unescape(re.sub(r"<[^>]+>", " ", text))
            row["text"] = re.sub(r"\s+", " ", text).strip()
        except Exception as error:
            row["error"] = str(error)
        result.append(row)
        print(name, row.get("status", row.get("error")), flush=True)
    write(out / "official_sources.json", dict(sources=result,
          note="Primary-source snapshots. Retrieval failure is not research evidence."))


def motion_bound(radius, half_angles):
    """Hausdorff displacement upper bound, radians; remains valid beyond pi."""
    return float(np.dot(radius, 2*np.sin(np.minimum(abs(half_angles), math.pi)/2)))


def obb_gap(a, b):
    """Projection separation is a lower bound on distance, never an estimate above it."""
    ra, rb = a["matrix"][:3, :3], b["matrix"][:3, :3]
    axes = np.concatenate((ra.T, rb.T, np.cross(ra.T[:, None, :], rb.T[None, :, :]).reshape(-1, 3)))
    length = np.linalg.norm(axes, axis=1)
    axes = axes[length > 1e-12]/length[length > 1e-12, None]
    gap = (abs(axes @ (a["centerWorld"]-b["centerWorld"]))
           -abs(axes @ ra) @ a["half"]-abs(axes @ rb) @ b["half"])
    return max(0., float(gap.max()))


class BoundsChecker(Checker):
    def __init__(self, source):
        super().__init__(source)
        buffers = Path(read(source / "config.json")["geometryDirectory"]) / "fcl_meshes"
        self.distance_request = fcl.DistanceRequest()
        self.distance_cache, self.floor_cache = {}, {}
        self.exact_queries = Counter()
        for body in self.bodies.values():
            vertices = np.fromfile(buffers / (body["mesh"]+".vertices"), dtype="<f4").reshape(-1, 3)
            # Deduplication changes neither the original triangle support nor its minimum height.
            body["vertices"] = np.unique(vertices, axis=0).astype(float)
            local = body["vertices"] @ body["local"][:3, :3].T+body["local"][:3, 3]
            extent = float(np.linalg.norm(local, axis=1).max())
            radius = np.zeros(5)
            for j in range(body["linkIndex"]):
                radius[j] = extent+sum(np.linalg.norm(o[:3, 3])
                                      for o in self.origins[j+1:body["linkIndex"]])
            body["radius"] = radius
        self.constraints = {}
        for pair in self.pairs:
            moving = max((pair["a"], pair["b"]), key=lambda b: b["linkIndex"])
            radius = moving["radius"].copy()
            radius[:pair["low"]] = 0
            self.constraints[pair["name"]] = dict(pair=pair, radius=radius)
        for body in self.bodies.values():
            if body["linkIndex"]:
                self.constraints[body["name"]+"/floor"] = dict(body=body, radius=body["radius"])

    def lower_distance(self, name, q, needed=math.inf):
        constraint = self.constraints[name]
        if "pair" in constraint:
            pair = constraint["pair"]
            key = (name, *np.round(q[pair["low"]:pair["high"]], 10))
            if key in self.distance_cache:
                return self.distance_cache[key]
            a, b = pair["a"], pair["b"]
            aabb = float(np.linalg.norm(np.maximum(0, abs(a["centerWorld"]-b["centerWorld"])
                                                       -a["halfWorld"]-b["halfWorld"])))
            if aabb > needed:
                return aabb
            oriented = obb_gap(a, b)
            if oriented > needed:
                return oriented
            pair["objA"].setTransform(a["transform"])
            pair["objB"].setTransform(b["transform"])
            # Intersecting detailed STL meshes can be slow for an unsigned
            # distance query; collision-first rejects the center without it.
            distance = (-1. if self.overlap(pair["objA"], pair["objB"]) else
                        float(fcl.distance(pair["objA"], pair["objB"],
                                           self.distance_request, fcl.DistanceResult())))
            self.distance_cache[key] = distance
        else:
            body = constraint["body"]
            key = (name, *np.round(q[:body["linkIndex"]], 10))
            if key in self.floor_cache:
                return self.floor_cache[key]
            lower = .05874511-body["centerWorld"][1]-body["halfWorld"][1]
            if lower > needed:
                return float(lower)
            m = body["matrix"]
            distance = float(.05874511-m[1, 3]-np.max(body["vertices"] @ m[1, :3]))
            self.floor_cache[key] = distance
        self.exact_queries[name] += 1
        return distance


def check_curve(checker, pose, guard=1e-6, max_nodes=30000, max_depth=24, names=None):
    """Bound all poses of a component-monotone curve, not just sampled points.

    Caller must supply a curve whose each coordinate is monotone on [0,1].
    The current linear shoulder path composed with the clipped quintic has
    this property. FCL and floating-point bounds are numerical, not formal
    interval arithmetic or a real-world safety certificate.
    """
    stack = [(0., 1., tuple(checker.constraints if names is None else names), 0)]
    nodes, bound_checks, leaves, lowest_slack = 0, 0, [], math.inf
    failures = []
    while stack:
        t0, t1, names, depth = stack.pop()
        q0, q1, qc = [np.array(pose(t)) for t in (t0, t1, (t0+t1)/2)]
        if any(q.shape != (5,) or not np.isfinite(q).all() for q in (q0, q1, qc)):
            raise ValueError("Invalid curve pose")
        rho = np.radians(np.maximum(abs(q0-qc), abs(q1-qc)))
        checker.place(qc)
        nodes += 1
        pending = []
        for name in names:
            movement = motion_bound(checker.constraints[name]["radius"], rho)
            distance = checker.lower_distance(name, qc, movement+guard)
            bound_checks += 1
            slack = distance-movement-guard
            if slack > 0:
                lowest_slack = min(lowest_slack, slack)
            else:
                pending.append(name)
            if distance <= 0:
                failures.append(dict(t=(t0+t1)/2, cadDegrees=qc.tolist(), constraint=name,
                                     centerDistanceMm=distance*1000))
        if failures:
            break
        if not pending:
            leaves.append([t0, t1])
        elif depth >= max_depth or nodes+len(stack)+2 > max_nodes:
            failures.append(dict(interval=[t0, t1], unresolved=pending, reason="resource_or_depth_limit"))
            break
        else:
            mid = (t0+t1)/2
            stack.extend(((mid, t1, tuple(pending), depth+1), (t0, mid, tuple(pending), depth+1)))
        if nodes % 2000 == 0:
            print(f"Motion bounds: nodes={nodes}, stack={len(stack)}, leaves={len(leaves)}", flush=True)
    leaves.sort()
    covered = sum(b-a for a, b in leaves)
    if not failures:
        assert leaves and leaves[0][0] == 0 and leaves[-1][1] == 1
        assert all(a[1] == b[0] for a, b in zip(leaves, leaves[1:]))
        assert abs(covered-1) < 1e-12
    return dict(status="numerical_motion_bound_pass" if not failures else "not_proven",
                nodes=nodes, boundChecks=bound_checks, leaves=len(leaves), coveredParameterFraction=covered,
                lowestPositiveSlackMm=None if not math.isfinite(lowest_slack) else lowest_slack*1000,
                failures=failures, floatingPointGuardMm=guard*1000)


def verify(source, out, limit):
    start = time.perf_counter()
    config = read(source / "config.json")
    model = BoundsChecker(source)
    print(f"Meshes and support vertices ready: {time.perf_counter()-start:.1f}s", flush=True)
    offset = config["offsetCadDeg"]
    reference = read(source / "geometry.json")["paths"]
    results, seen = [], set()
    for route in reference:
        for a, b in zip(route["waypointsCadDeg"], route["waypointsCadDeg"][1:]):
            key = (route["wristSchedule"]["J4AmplitudeDeg"],
                   *sorted((tuple(np.round(a, 8)), tuple(np.round(b, 8)))))
            if key in seen:
                continue
            seen.add(key)
            a, b = np.array(a), np.array(b)
            def pose(t):
                arms = a[1:3]+(b[1:3]-a[1:3])*t-np.array(offset[1:3])
                return policy_pose(*arms, offset, route["wristSchedule"])
            result = check_curve(model, pose, max_nodes=limit)
            results.append(dict(route=route["id"], startCadDegrees=a.tolist(),
                                endCadDegrees=b.tolist(), result=result))
            print(route["id"], result, flush=True)
            if result["status"] != "numerical_motion_bound_pass":
                break
        if results and results[-1]["result"]["status"] != "numerical_motion_bound_pass":
            break
    out.mkdir(parents=True, exist_ok=False)
    write(out / "motion_bounds.json", dict(source=str(source.resolve()), results=results,
          generatedUtc=datetime.now(timezone.utc).isoformat(), exactQueries=model.exact_queries,
          elapsedSeconds=time.perf_counter()-start,
          conditionalMapping=config["coordinateAssumption"],
          limitations=["Numerical surface-distance bounds, not formal interval arithmetic",
                       "Checks nominal composed curves; not actual tracking tubes, arbitrary regions or braking",
                       "Installation-surface masking and all input model/calibration limitations remain"]))


def j5_alignment(source, out):
    """Diagnose whether the measured span fits the J5-only adjacent collision pair."""
    config = read(source / "config.json")
    checker = Checker(source)
    q = np.array(config["offsetCadDeg"])
    observations = []
    def clear(angle):
        q[4] = angle
        hits = checker.hits(q)
        observations.append(dict(J5CadDeg=angle, pairs=hits))
        return "part0/part1" not in hits
    intervals = sampled_intervals(clear, -100, 100, step=.5, inset=0, tolerance=.0005)
    measured = config["recordedSoftwareRangesDeg"][4]
    offsets = [[a-measured[0], b-measured[1]] for a, b in intervals
               if b-a >= measured[1]-measured[0] and -100 < a and b < 100]
    zero_offset_contains_recorded_span = any(a <= measured[0] and measured[1] <= b for a, b in intervals)
    out.mkdir(parents=True, exist_ok=False)
    report = dict(generatedUtc=datetime.now(timezone.utc).isoformat(), source=str(source.resolve()),
                  recordedSoftwareRangeDeg=measured, adjacentPair="part0/part1",
                  searchWindowCadDeg=[-100, 100],
                  offsetCompatibilityScope="Fully bounded components in the search window only; not unique physical alignment",
                  pairDependsOnlyOn="J5 CAD angle, because both bodies share the first four joints",
                  sampledCadIntervalsDeg=intervals, compatibleOffsetIntervalsDeg=offsets,
                  currentZeroOffsetContainsRecordedSpan=zero_offset_contains_recorded_span,
                  observations=observations,
                  limitations=["Geometric diagnostic, not a physical calibration or motor motion",
                               "Endpoints and possible offset intervals are numerical and model-dependent",
                               "Other pairs, cable and environment can further restrict these intervals",
                               "Components touching the scan window endpoints can extend outside; not classified completely",
                               "No automatic offset change: raw observed endpoints are not measured CAD postures"])
    write(out / "j5_alignment.json", report)
    print({k: v for k, v in report.items() if k not in ("observations", "limitations")}, flush=True)


def alignment_hypothesis(source, diagnostic, out):
    config = read(source / "config.json")
    evidence = read(diagnostic)
    possible = evidence["compatibleOffsetIntervalsDeg"]
    if not possible:
        raise ValueError("No compatible offset candidate; do not manufacture a calibration")
    low, high = possible[0]
    offset = (low+high)/2
    config["offsetCadDeg"][4] = offset
    config["mappedCadRangesDeg"][4] = [x+offset for x in config["recordedSoftwareRangesDeg"][4]]
    config["coordinateAssumption"] += (
        f" SENSITIVITY HYPOTHESIS ONLY: J5 CAD offset={offset:.10f} deg inferred from "
        "fitting the observed span inside one adjacent-pair interval, NOT measured CAD alignment.")
    config["alignmentHypothesis"] = dict(diagnostic=str(diagnostic.resolve()),
        possibleOffsetIntervalDeg=[low, high], selectedOffsetDeg=offset,
        changesRealCalibration=False, status="UNVERIFIED_PARAMETER_SENSITIVITY_ONLY")
    out.mkdir(parents=True, exist_ok=False)
    write(out / "config.json", config)
    print(config["alignmentHypothesis"], flush=True)


def radius_audit(source, out):
    checker = BoundsChecker(source)
    config = read(source / "config.json")
    bounds = np.array(config["mappedCadRangesDeg"])
    rng = np.random.default_rng(2026111602)
    counts = 0
    def points(constraint):
        if "body" in constraint:
            body = constraint["body"]
            transform = body["matrix"]
        else:
            pair = constraint["pair"]
            body = max((pair["a"], pair["b"]), key=lambda b: b["linkIndex"])
            transform = np.linalg.inv(checker.frames[pair["low"]]) @ body["matrix"]
        vertices = body["vertices"]
        vertices = vertices[np.linspace(0, len(vertices)-1, min(64, len(vertices)), dtype=int)]
        return vertices @ transform[:3, :3].T+transform[:3, 3]
    for _ in range(30):
        q = rng.uniform(bounds[:, 0], bounds[:, 1])
        delta = rng.uniform(-20, 20, 5)
        checker.place(q)
        before = {name: points(c) for name, c in checker.constraints.items()}
        checker.place(q+delta)
        for name, constraint in checker.constraints.items():
            observed = float(np.linalg.norm(points(constraint)-before[name], axis=1).max())
            upper = motion_bound(constraint["radius"], np.radians(abs(delta)))
            assert observed <= upper+1e-10, (name, observed, upper)
            counts += 1
    request = fcl.DistanceRequest()
    for gap in (1e-6, 1e-5, .001, .1):
        a = fcl.CollisionObject(fcl.Box(1, 1, 1))
        b = fcl.CollisionObject(fcl.Box(1, 1, 1), fcl.Transform(np.array([1+gap, 0., 0.])))
        distance = fcl.distance(a, b, request, fcl.DistanceResult())
        assert abs(distance-gap) < 1e-10
    out.mkdir(parents=True, exist_ok=False)
    write(out / "radius_audit.json", dict(randomSeed=2026111602, constraints=len(checker.constraints),
          randomDisplacementChecks=counts, geometryPointsPerCheckUpTo=64, anglesTestedBeyondRecordedBounds=True,
          analyticBoxDistanceTests=4, status="passed",
          note="Implementation checks for conservative radii and native distances, not a global robot safety proof"))
    print("PASS:", counts, "relative/world point displacement checks and four analytic box distances", flush=True)


def adjacent_domains(source, out, limit):
    """Factor one-joint pairs out of the full four-dimensional admissible set."""
    started = time.perf_counter()
    checker = BoundsChecker(source)
    config = read(source / "config.json")
    bounds = np.array(config["mappedCadRangesDeg"])
    results = []
    for j in range(5):
        names = [p["name"] for p in checker.pairs if p["low"] == j and p["high"] == j+1]
        a = np.array(config["offsetCadDeg"])
        a[j] = bounds[j, 0]
        b = a.copy()
        b[j] = bounds[j, 1]
        result = check_curve(checker, lambda t: a+(b-a)*t, names=names, max_nodes=limit)
        results.append(dict(joint=j+1, cadRangeDeg=bounds[j].tolist(), constraints=names, result=result))
        print(f"Adjacent domain J{j+1}:", result, flush=True)
    out.mkdir(parents=True, exist_ok=False)
    write(out / "adjacent_domains.json", dict(source=str(source.resolve()),
          mapping=config["coordinateAssumption"], results=results, exactQueries=checker.exact_queries,
          elapsedSeconds=time.perf_counter()-started,
          note="Each tested pair is independent of all other joints in its common link frame; "
               "numerical motion bounds over complete recorded angle intervals, not hardware calibration. "
               "No nonadjacent pair or floor constraint is removed by this result."))


def intervals(source, out, limit, software, joint):
    """Keep a sampled component only if its complete fixed-joint interval passes."""
    started = time.perf_counter()
    config = read(source / "config.json")
    checker = BoundsChecker(source)
    offset = np.array(config["offsetCadDeg"])
    upper = np.array(config["recordedSoftwareRangesDeg"])[:, 1]
    queries = [(software, joint)] if software is not None else [
        ([0, 0, 0, 0, 0], 4),
        ([0, upper[1], upper[2], -160, 0], 4),
        ([0, upper[1], upper[2], 160, 0], 4),
        ([0, upper[1], upper[2], 0, 0], 4),
        ([0, 35, 90, 0, 0], 3),
        ([0, upper[1], upper[2], 0, 20], 3),
    ]
    results = []
    for fixed, axis in queries:
        candidates = conditional_intervals(checker, config, fixed, joint=axis,
                                          step=.5 if axis == 4 else 2., inset=1.)
        verified, checks = [], []
        for a, b in candidates:
            def pose(t):
                q = offset+np.array(fixed)
                q[axis] = offset[axis]+a+(b-a)*t
                return q
            check = check_curve(checker, pose, max_nodes=limit)
            checks.append(dict(candidateSoftwareIntervalDeg=[a, b], result=check))
            if check["status"] == "numerical_motion_bound_pass":
                verified.append([a, b])
        row = dict(fixedSoftwareDegrees=list(fixed), scannedJoint=axis+1,
                   candidateIntervalsDeg=candidates, verifiedIntervalsDeg=verified, checks=checks)
        results.append(row)
        print({k: v for k, v in row.items() if k != "checks"}, flush=True)
    out.mkdir(parents=True, exist_ok=False)
    write(out / "verified_intervals.json", dict(source=str(source.resolve()),
          generatedUtc=datetime.now(timezone.utc).isoformat(), results=results,
          offsetCadDeg=config["offsetCadDeg"], mapping=config["coordinateAssumption"],
          exactQueries=checker.exact_queries, elapsedSeconds=time.perf_counter()-started,
          floatingPointGuardMm=.001,
          limitations=["Only the scanned joint varies; other joints must equal each row's fixed values",
                       "No interpolation of rows or transitions between disconnected intervals is authorized",
                       "Sampled discovery can omit narrow free components; not a globally maximal set",
                       "An unproved candidate is excluded, not declared colliding",
                       "Numerical model bounds only; physical calibration, tracking and braking remain unverified"]))


def branch_switch(source, out, limit):
    """Find local withdrawal/sweep/return corridors without replacing the allowed set."""
    started = time.perf_counter()
    config = read(source / "config.json")
    checker = BoundsChecker(source)
    offset = np.array(config["offsetCadDeg"])
    bounds = np.array(config["recordedSoftwareRangesDeg"])
    top = np.array([0., bounds[1, 1]-1, bounds[2, 1]-1, -160., 20.])
    out.mkdir(parents=True, exist_ok=False)
    write(out / "config.json", config)
    direct = [dict(softwareDegrees=(q-offset).tolist(), pairs=found)
              for q in line(offset+top, offset+top+[0, 0, 0, 320, 0], .5)
              if (found := checker.hits(q))]
    results, paths = [], []
    for axis in (1, 2):
        searched = []
        def sampled_sweep(angle):
            q = top.copy()
            q[axis] = angle
            a = q+offset
            clear = checker.clear_line(a, a+[0, 0, 0, 320, 0], 1.)
            searched.append(dict(withdrawnSoftwareAngleDeg=float(angle), sampledSweepClear=clear))
            return clear
        good, bad = None, float(top[axis])
        for angle in np.linspace(top[axis], bounds[axis, 0]+1,
                                 math.ceil((top[axis]-bounds[axis, 0]-1)/5)+1):
            if sampled_sweep(float(angle)):
                good = float(angle)
                break
            bad = float(angle)
        row = dict(withdrawJoint=axis+1, search=searched)
        if good is not None:
            while bad-good > .05:
                middle = (good+bad)/2
                if sampled_sweep(middle):
                    good = middle
                else:
                    bad = middle
            withdrawn = max(bounds[axis, 0]+1, good-2)
            a = top.copy()
            a[axis] = withdrawn
            b, end = a.copy(), top.copy()
            b[3] = end[3] = 160.
            forward = [top, a, b, end]
            proofs = [check_curve(checker, lambda t, p=p, q=q: offset+p+(q-p)*t,
                                  max_nodes=limit)
                      for p, q in zip(forward, forward[1:])]
            row.update(localSampledThresholdBracketDeg=[good, bad],
                       selectedWithdrawalAngleDeg=float(withdrawn),
                       nominalSegmentChecks=proofs)
            if all(p["status"] == "numerical_motion_bound_pass" for p in proofs):
                waypoints = [*forward, b, a, top]
                paths.append(dict(id=f"switch_by_J{axis+1}_return",
                                  waypointsCadDeg=[(q+offset).tolist() for q in waypoints]))
        results.append(row)
        print(f"Branch corridor J{axis+1}:", {k: v for k, v in row.items() if k != "search"},
              flush=True)
    limitations = ["Conditional uncalibrated model, not hardware authorization",
                   "Search brackets one local sampled transition, not a global maximum",
                   "The selected complete nominal paths are separately distance-bound checked",
                   "Motion stops at intermediate vertices; arbitrary blending is not authorized",
                   "No tracking tube, braking or cable certification",
                   *config["limitations"]]
    write(out / "branch_switch.json", dict(source=str(source.resolve()),
          generatedUtc=datetime.now(timezone.utc).isoformat(),
          offsetCadDeg=config["offsetCadDeg"], topSoftwareDeg=top.tolist(),
          directSweep=dict(checked=641, colliding=len(direct), witnesses=direct[:5]),
          results=results, exactQueries=checker.exact_queries,
          elapsedSeconds=time.perf_counter()-started, limitations=limitations))
    write(out / "geometry.json", dict(anchorCadDeg=offset.tolist(), paths=paths,
          coordinationDeltaCadDeg=[0, 8, -12, 20, 5]))
    write(out / "study.json", dict(parameters=[dict(type="withdraw_sweep_return", fixedJ5Deg=20.)],
          limitations=limitations))


def equivalent_angles(angle, low, high):
    """Enumerate permitted unwrapped angles; never normalize the measured state."""
    if not all(math.isfinite(x) for x in (angle, low, high)) or high < low:
        raise ValueError("Invalid angle domain")
    return [angle+360*k for k in range(math.ceil((low-angle)/360),
                                     math.floor((high-angle)/360)+1)]


def equivalent_turns(source, out, limit):
    """Test explicit turn selection only for tasks accepting equal tool poses."""
    started = time.perf_counter()
    config = read(source / "config.json")
    checker = BoundsChecker(source)
    offset = np.array(config["offsetCadDeg"])
    bounds = np.array(config["recordedSoftwareRangesDeg"])
    top = np.array([0., bounds[1, 1]-1, bounds[2, 1]-1, 0., 20.])
    paths, results = [], []
    for current, target in ((-160., 160.), (160., -160.)):
        start = top.copy()
        start[3] = current
        selected = None
        options = sorted(equivalent_angles(target, bounds[3, 0]+1, bounds[3, 1]-1),
                         key=lambda angle: abs(angle-current))
        checks = []
        for angle in options:
            end, requested = top.copy(), top.copy()
            end[3], requested[3] = angle, target
            checker.place(offset+requested)
            expected = {n: b["matrix"].copy() for n, b in checker.bodies.items()}
            checker.place(offset+end)
            difference = max(float(abs(b["matrix"]-expected[n]).max())
                             for n, b in checker.bodies.items())
            assert difference < 1e-12, "Equivalent angle changed modeled body transforms"
            proof = check_curve(checker, lambda t: offset+start+(end-start)*t, max_nodes=limit)
            checks.append(dict(commandedContinuousAngleDeg=angle,
                               maximumBodyTransformEntryDifference=difference, result=proof))
            if selected is None and proof["status"] == "numerical_motion_bound_pass":
                selected = angle
                paths.append(dict(id=f"equivalent_turn_from_{int(current)}_return",
                                  waypointsCadDeg=[(offset+q).tolist() for q in (start, end, start)]))
        row = dict(currentContinuousAngleDeg=current, requestedOrientationAngleDeg=target,
                   selectedContinuousAngleDeg=selected, options=checks)
        results.append(row)
        print("Equivalent turn:", row, flush=True)
    limitations = ["Only when the task accepts equal tool poses, not an exact continuous-angle target",
                   "No modulo operation on feedback or reference; selected angles remain explicit",
                   "Recorded angle bounds are not proof of cable/twist allowance",
                   "Conditional uncalibrated model; no hardware, tracking tube or braking authorization",
                   *config["limitations"]]
    out.mkdir(parents=True, exist_ok=False)
    write(out / "config.json", config)
    write(out / "equivalent_turns.json", dict(source=str(source.resolve()),
          generatedUtc=datetime.now(timezone.utc).isoformat(), results=results,
          offsetCadDeg=config["offsetCadDeg"], fixedJ5SoftwareDeg=20.,
          fixedJ2J3SoftwareDeg=top[1:3].tolist(), exactQueries=checker.exact_queries,
          elapsedSeconds=time.perf_counter()-started, limitations=limitations))
    write(out / "geometry.json", dict(anchorCadDeg=offset.tolist(), paths=paths,
          coordinationDeltaCadDeg=[0, 8, -12, 20, 5]))
    write(out / "study.json", dict(parameters=[dict(type="explicit_equivalent_turn", fixedJ5Deg=20.)],
          limitations=limitations))


def sum_counterexample(source, out):
    """Search matched-sum poses; reject reducing the full constraint to q2+q3."""
    checker = Checker(source)
    config = read(source / "config.json")
    offset = np.array(config["offsetCadDeg"])
    bounds = np.array(config["recordedSoftwareRangesDeg"])
    witnesses = []
    for total in (110., 120., 125., 130., 135., 140.):
        for q4, q5 in ((0., 0.), (0., 20.), (0., 22.), (-80., 0.), (-160., 20.)):
            rows = []
            for q2 in (15., 20., 25., 30., 35., 40., 45.):
                software = np.array([0., q2, total-q2, q4, q5])
                if np.any(software < bounds[:, 0]) or np.any(software > bounds[:, 1]):
                    continue
                rows.append(dict(softwareDegrees=software.tolist(), pairs=checker.hits(offset+software)))
            clear = next((r for r in rows if not r["pairs"]), None)
            colliding = next((r for r in rows if r["pairs"]), None)
            if clear is not None and colliding is not None:
                witnesses.append(dict(sumDeg=total, sameJ4J5Deg=[q4, q5],
                                      clearPose=clear, collidingPose=colliding))
    out.mkdir(parents=True, exist_ok=False)
    write(out / "sum_counterexamples.json", dict(source=str(source.resolve()),
          generatedUtc=datetime.now(timezone.utc).isoformat(), offsetCadDeg=config["offsetCadDeg"],
          witnesses=witnesses, poseChecks=checker.checks,
          note="Same q2+q3 and same wrist angles, different surface/floor classifications. "
               "A counterexample refutes a universal sum-only limit; no witness would not prove one."))
    print("Matched-sum counterexamples:", len(witnesses), witnesses[:2], flush=True)


def box_measure(bounds):
    widths = np.diff(np.asarray(bounds), axis=1).ravel()
    return float(np.prod(widths[widths > 0]))


def focus_seeds(seeds, windows):
    """Partition saved unknown cells exactly; defer, never discard, outside pieces."""
    selected, deferred = [], list(seeds)
    for window in np.asarray(windows):
        remaining = []
        for row in deferred:
            box = np.asarray(row["cadBoundsDeg"], dtype=float).copy()
            cut = np.column_stack((np.maximum(box[:, 0], window[:, 0]),
                                   np.minimum(box[:, 1], window[:, 1])))
            active = box[:, 1] > box[:, 0]
            if np.any(cut[:, 1] < cut[:, 0]) or np.any(cut[active, 1] <= cut[active, 0]):
                remaining.append(row)
                continue
            # A parent's center witness is not a witness at any child's center.
            def child(bounds):
                return dict(cadBoundsDeg=bounds.tolist(), unresolved=list(row["unresolved"]),
                            reason="focus_partition")
            for j in range(5):
                if box[j, 0] < cut[j, 0]:
                    outside = box.copy()
                    outside[j, 1] = cut[j, 0]
                    remaining.append(child(outside))
                    box[j, 0] = cut[j, 0]
                if box[j, 1] > cut[j, 1]:
                    outside = box.copy()
                    outside[j, 0] = cut[j, 1]
                    remaining.append(child(outside))
                    box[j, 1] = cut[j, 1]
            selected.append(child(box))
        deferred = remaining
    return selected, deferred


def partition_regions(checker, bounds, names, max_nodes, min_width=.5, guard=1e-6,
                      max_seconds=None, seeds=None, focus=None):
    """Inner approximation: a collision at a center never rejects its whole box."""
    bounds = np.asarray(bounds, dtype=float)
    if (bounds.shape != (5, 2) or not np.isfinite(bounds).all()
            or np.any(bounds[:, 1] < bounds[:, 0]) or max_nodes < 1 or min_width <= 0):
        raise ValueError("Invalid region domain or budget")
    started = time.perf_counter()
    if max_seconds is not None and (not math.isfinite(max_seconds) or max_seconds < 0):
        raise ValueError("Invalid time budget")
    focus = np.empty((0, 5)) if focus is None else np.asarray(focus, dtype=float)
    if (focus.ndim != 2 or focus.shape[1] != 5 or not np.isfinite(focus).all()
            or np.any(focus < bounds[:, 0]) or np.any(focus > bounds[:, 1])):
        raise ValueError("Focus poses must be finite and inside the parent domain")
    def priority(box, colliding=False):
        contains_focus = np.any(np.all((focus >= box[:, 0]) & (focus <= box[:, 1]), axis=1))
        return -box_measure(box)*(.02 if colliding else 1.)*(1e12 if contains_focus else 1.)
    initial = seeds if seeds is not None else [dict(cadBoundsDeg=bounds.tolist(), unresolved=list(names))]
    pending = []
    for i, row in enumerate(initial):
        box = np.asarray(row["cadBoundsDeg"], dtype=float)
        if (box.shape != (5, 2) or not np.isfinite(box).all()
                or np.any(box[:, 0] < bounds[:, 0]) or np.any(box[:, 1] > bounds[:, 1])
                or np.any(box[:, 0] > box[:, 1])
                or not set(row["unresolved"]) <= set(names)):
            raise ValueError("Invalid saved subdomain")
        pending.append((priority(box), i, box.copy(), tuple(row["unresolved"])))
    heapq.heapify(pending)
    input_fraction = sum(box_measure(box) for _, _, box, _ in pending)/box_measure(bounds)
    serial, nodes, checks, safe, unknown = len(pending), 0, 0, [], []
    while pending and nodes < max_nodes and (max_seconds is None or time.perf_counter()-started < max_seconds):
        _, _, box, names = heapq.heappop(pending)
        center, half = box.mean(axis=1), np.diff(box, axis=1).ravel()/2
        checker.place(center)
        unresolved, witnesses, slack = [], [], math.inf
        for index, name in enumerate(names):
            movement = motion_bound(checker.constraints[name]["radius"], np.radians(half))
            distance = checker.lower_distance(name, center, movement+guard)
            checks += 1
            if distance-movement > guard:
                slack = min(slack, distance-movement-guard)
            else:
                unresolved.append(name)
            if distance <= 0:
                witnesses.append(dict(constraint=name, centerDistanceMm=distance*1000))
                unresolved.extend(names[index+1:])
                break
        nodes += 1
        if not unresolved:
            safe.append(dict(cadBoundsDeg=box.tolist(),
                             boundSlackMm=None if not math.isfinite(slack) else slack*1000))
            continue
        # A collision witness cannot certify rejection of the surrounding cell.
        # Stop refining its interior sooner; preserve it explicitly as unknown.
        eligible = 2*half > (max(min_width, .125) if witnesses else min_width)
        if not np.any(eligible):
            unknown.append(dict(cadBoundsDeg=box.tolist(), reason="resolution_limit",
                                unresolved=unresolved, centerWitnesses=witnesses))
            continue
        radii = np.max([checker.constraints[n]["radius"] for n in unresolved], axis=0)
        score = radii*2*np.sin(np.minimum(np.radians(half), math.pi)/2)
        axis = int(np.argmax(np.where(eligible, score, -1)))
        for side in (0, 1):
            child = box.copy()
            child[axis, 1-side] = center[axis]
            serial += 1
            # ponytail: volume-priority bisection; unresolved collision centers are
            # searched later, never mislabeled as whole-box collision certificates.
            heapq.heappush(pending, (priority(child, bool(witnesses)), serial, child, tuple(unresolved)))
        if nodes % 2000 == 0:
            print(f"Regions: nodes={nodes}, verified={len(safe)}, pending={len(pending)}", flush=True)
    unknown.extend(dict(cadBoundsDeg=box.tolist(), reason="node_or_time_budget", unresolved=list(names))
                   for _, _, box, names in pending)
    total = box_measure(bounds)
    safe_fraction = sum(box_measure(r["cadBoundsDeg"]) for r in safe)/total
    unknown_fraction = sum(box_measure(r["cadBoundsDeg"]) for r in unknown)/total
    assert abs(safe_fraction+unknown_fraction-input_fraction) < 1e-9
    return dict(verified=safe, unknown=unknown, nodes=nodes, boundChecks=checks,
                elapsedSeconds=time.perf_counter()-started,
                verifiedFraction=safe_fraction, unknownFraction=unknown_fraction)


def merge_intervals(intervals):
    result = []
    for low, high in sorted(intervals):
        if result and low <= result[-1][1]:
            result[-1][1] = max(high, result[-1][1])
        else:
            result.append([low, high])
    return result


def region_intervals(boxes, software, joint=4):
    """Conditional interval union for J2..J5; no interpolation or angle wrapping."""
    q = np.asarray(software, dtype=float)
    if q.shape != (5,) or not np.isfinite(q).all() or q[0] != 0 or joint not in (1, 2, 3, 4):
        raise ValueError("Expected finite software angles with J1 exactly zero")
    axes = [j for j in range(5) if j != joint]
    windows = np.asarray(boxes, dtype=float).reshape(-1, 5, 2)
    selected = (np.all(q[axes] >= windows[:, axes, 0], axis=1)
                & np.all(q[axes] <= windows[:, axes, 1], axis=1))
    return merge_intervals(windows[selected, joint].tolist())


def segment_in_regions(boxes, a, b):
    """Check the complete unwrapped joint-linear segment against a union of boxes."""
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    if a.shape != (5,) or b.shape != (5,) or not np.isfinite([a, b]).all():
        raise ValueError("Expected two finite five-joint poses")
    intervals = []
    for box in np.asarray(boxes):
        low, high = 0., 1.
        for j, delta in enumerate(b-a):
            if delta == 0:
                if not box[j, 0] <= a[j] <= box[j, 1]:
                    break
            else:
                ends = (box[j]-a[j])/delta
                low, high = max(low, float(min(ends))), min(high, float(max(ends)))
                if low > high:
                    break
        else:
            intervals.append([low, high])
    merged = merge_intervals(intervals)
    return bool(merged and merged[0][0] <= 0 and merged[0][1] >= 1)


def regions(source, out, limit):
    """Factor adjacent constraints, then bound full-dimensional allowed cells."""
    started = time.perf_counter()
    config = read(source / "config.json")
    checker = BoundsChecker(source)
    bounds = np.array(config["mappedCadRangesDeg"])
    offset = np.array(config["offsetCadDeg"])
    if config["signs"] != [1]*5 or not np.array_equal(bounds[0], [0., 0.]):
        raise ValueError("This study requires the documented +1 mapping and fixed J1")
    # Hash the exact geometry buffers used now, not merely an old report's filenames.
    geometry = Path(config["geometryDirectory"])
    files = [source / "config.json", geometry / "mesh_input.json",
             geometry / "fcl_meshes/manifest.json", Path(config["recordSource"]),
             Path(config["urdf"]), Path(config["scene"]), Path(__file__)]
    files.extend(sorted((geometry / "fcl_meshes").glob("*.vertices")))
    files.extend(sorted((geometry / "fcl_meshes").glob("*.indices")))
    identity = {}
    for p in files:
        with p.open("rb") as stream:
            identity[str(p.resolve())] = hashlib.file_digest(stream, "sha256").hexdigest()
    assert identity[str(Path(config["recordSource"]).resolve())] == config["recordSha256"]
    assert identity[str(Path(config["scene"]).resolve())] == config["sceneSha256"]
    adjacent, axes, factored = [], [], set()
    for j in range(5):
        names = [p["name"] for p in checker.pairs if p["low"] == j and p["high"] == j+1]
        domain = np.column_stack((offset, offset))
        domain[j] = bounds[j]
        result = partition_regions(checker, domain, names, 35000, min_width=.0005)
        intervals = merge_intervals([r["cadBoundsDeg"][j] for r in result["verified"]])
        adjacent.append(dict(joint=j+1, constraints=names, intervalsCadDeg=intervals, result=result))
        axes.append(intervals)
        factored.update(names)
        print(f"Adjacent J{j+1}: {result['nodes']} nodes, "
              f"{result['verifiedFraction']:.6%} verified, intervals={intervals}", flush=True)
    remaining = sorted(set(checker.constraints)-factored)
    cells, unresolved = [], []
    statistics = []
    for intervals in itertools.product(*axes):
        domain = np.array(intervals)
        result = partition_regions(checker, domain, remaining, limit)
        cells.extend(result.pop("verified"))
        unresolved.extend(result.pop("unknown"))
        statistics.append(dict(domainCadDeg=domain.tolist(), **result))
    software_boxes = [(np.array(r["cadBoundsDeg"])-offset[:, None]).tolist() for r in cells]
    covered = sum(box_measure(r["cadBoundsDeg"]) for r in cells)/box_measure(bounds)
    # Independent finite regression, in addition to conservative whole-cell bounds.
    rng = np.random.default_rng(2026111701)
    samples, failures = 0, []
    if cells:
        ids = np.unique(np.linspace(0, len(cells)-1, min(300, len(cells)), dtype=int))
        for i in ids:
            box = np.array(cells[i]["cadBoundsDeg"])
            for q in (box[:, 0], box[:, 1], *rng.uniform(box[:, 0], box[:, 1], (3, 5))):
                found = checker.hits(q)
                samples += 1
                if found:
                    failures.append(dict(cadDegrees=q.tolist(), pairs=found))
    assert not failures, failures
    for path, digest in identity.items():
        with Path(path).open("rb") as stream:
            assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, path
    out.mkdir(parents=True, exist_ok=False)
    write(out / "config.json", config)
    candidates = []
    for i, box in enumerate(software_boxes):
        inner = np.array(box)
        inner[1:, 0] = np.maximum(inner[1:, 0], np.array(config["recordedSoftwareRangesDeg"])[1:, 0]+1)
        inner[1:, 1] = np.minimum(inner[1:, 1], np.array(config["recordedSoftwareRangesDeg"])[1:, 1]-1)
        if np.all(np.diff(inner[1:], axis=1).ravel() > .1):
            candidates.append((i, inner))
    selected, paths = set(), []
    criteria = [lambda box: box_measure(box),
                lambda box: float(box[1:3].mean(axis=1).sum()) if box[3].mean() < 0 else -math.inf,
                lambda box: float(box[1:3].mean(axis=1).sum()) if box[3].mean() > 0 else -math.inf,
                lambda box: float(box[2, 1]-box[2, 0])]
    for score in criteria:
        available = [(i, b) for i, b in candidates if i not in selected]
        if not available:
            break
        i, box = max(available, key=lambda item: score(item[1]))
        selected.add(i)
        width = np.diff(box, axis=1).ravel()
        a, b = box[:, 0]+.2*width, box[:, 1]-.2*width
        assert segment_in_regions([software_boxes[i]], a, b)
        paths.append(dict(id=f"region_{i}_four_axis_return", regionIndex=i,
                          softwareRegionBoundsDeg=software_boxes[i],
                          waypointsCadDeg=[(offset+q).tolist() for q in (a, b, a)]))
    write(out / "geometry.json", dict(anchorCadDeg=offset.tolist(), paths=paths,
          coordinationDeltaCadDeg=[0, 8, -12, 20, 5]))
    write(out / "study.json", dict(parameters=[dict(type="verified_joint_region_diagonals")],
          limitations=config["limitations"]))
    report = dict(source=str(source.resolve()), generatedUtc=datetime.now(timezone.utc).isoformat(),
        fileSha256=identity, offsetCadDeg=offset.tolist(), mapping=config["coordinateAssumption"],
        recordedSoftwareRangesDeg=config["recordedSoftwareRangesDeg"],
        formula="q1=0 and q5 in union(Bk[4] for k where (q2,q3,q4) in Bk[1:4])",
        verifiedSoftwareBoxesDeg=software_boxes, adjacentConstraints=adjacent,
        remainingConstraints=remaining, partitions=statistics,
        unknownCadBoxes=unresolved, verifiedRecordedJointMeasureFraction=covered,
        unverifiedRecordedJointMeasureFraction=1-covered,
        independentPoseChecks=samples, independentCollisions=failures, dynamicRoutes=paths,
        exactQueries=checker.exact_queries, elapsedSeconds=time.perf_counter()-started,
        floatingPointGuardMm=.001,
        limitations=["Numerical inner approximation, not globally maximal or formal interval arithmetic",
                     "Unverified cells are forbidden by this table, not declared entirely colliding",
                     "Fractions are four-dimensional joint-coordinate measure, not Cartesian workspace volume",
                     "All listed regions permit independent joint variation; transitions need union path coverage",
                     "Only J1=0; actual tracking outside regions, braking, cables and calibration unverified",
                     *config["limitations"]])
    write(out / "regions.json", report)
    print(f"Verified {len(cells)} boxes, {covered:.6%} of recorded joint domain; "
          f"independent checks={samples}, seconds={report['elapsedSeconds']:.1f}", flush=True)


def regions_query(source, software):
    report = read(source / "regions.json")
    print(dict(softwareDegrees=software, scannedJoint=5,
               verifiedIntervalsDeg=region_intervals(report["verifiedSoftwareBoxesDeg"], software),
               mapping=report["mapping"],
               status="Numerical model inner approximation; empty means not authorized, not necessarily collision"),
          flush=True)


def compact_boxes(boxes):
    """Merge exact face-adjacent rectangles; never fill a hole in their union."""
    boxes = [[list(v) for v in b] for b in boxes]
    previous = -1
    while previous != len(boxes):
        previous = len(boxes)
        for axis in range(1, 5):
            groups = {}
            for box in boxes:
                key = tuple(tuple(v) for j, v in enumerate(box) if j != axis)
                groups.setdefault(key, []).append(box)
            merged = []
            for rows in groups.values():
                for interval in merge_intervals([b[axis] for b in rows]):
                    box = [v.copy() for v in rows[0]]
                    box[axis] = interval
                    merged.append(box)
            boxes = merged
    return boxes


def intersect_intervals(a, b):
    return merge_intervals([[max(x, u), min(y, v)] for x, y in a for u, v in b
                            if max(x, u) <= min(y, v)])


def factored_intervals(report, software, joint=4):
    q = np.asarray(software, dtype=float)
    if q.shape != (5,) or not np.isfinite(q).all() or q[0] != 0 or joint not in (1, 2, 3, 4):
        raise ValueError("Expected finite software angles with J1 zero")
    domain = np.array(report["admissibleSoftwareDomainDeg"])
    axes = [j for j in range(5) if j != joint]
    if np.any(q[axes] < domain[axes, 0]) or np.any(q[axes] > domain[axes, 1]):
        return []
    result = [domain[joint].tolist()]
    for group in report["groups"]:
        result = intersect_intervals(result, region_intervals(group["verifiedSoftwareBoxesDeg"], q, joint))
        if not result:
            break
    return result


def factored_membership(groups, points):
    points = np.asarray(points)
    allowed = np.ones(len(points), dtype=bool)
    for group in groups:
        boxes = np.asarray(group["verifiedSoftwareBoxesDeg"])
        if not len(boxes):
            return np.zeros(len(points), dtype=bool)
        for start in range(0, len(points), 64):
            q = points[start:start+64, None, :]
            inside = np.any(np.all((q >= boxes[None, :, :, 0])
                                  & (q <= boxes[None, :, :, 1]), axis=2), axis=1)
            allowed[start:start+64] &= inside
    return allowed


def factored_regions(previous, out, limit, max_seconds=90, focus=None, focus_boxes=None):
    """Avoid multiplying shoulder cells by wrist-only cells: intersect their sets."""
    started = time.perf_counter()
    prior = read(previous / "factored_regions.json") if (previous / "factored_regions.json").is_file() else None
    original = Path(prior["previousReport"]) if prior else previous / "regions.json"
    old = read(original)
    source = Path(old["source"])
    config = read(source / "config.json")
    # Reuse only same-model adjacent results. The old script hash is historical
    # implementation provenance, not an input geometry/calibration parameter.
    for filename, digest in old["fileSha256"].items():
        if Path(filename).resolve() == Path(__file__).resolve():
            continue
        with Path(filename).open("rb") as stream:
            if hashlib.file_digest(stream, "sha256").hexdigest() != digest:
                raise ValueError("Prior model/input changed: "+filename)
    if len(old["partitions"]) != 1:
        raise ValueError("This refinement expects one adjacent-approved product domain")
    domain = np.array(old["partitions"][0]["domainCadDeg"])
    offset = np.array(config["offsetCadDeg"])
    focus = np.empty((0, 5)) if focus is None else np.asarray(focus, dtype=float)
    if (focus.ndim != 2 or focus.shape[1] != 5 or not np.isfinite(focus).all()
            or np.any(focus < domain[:, 0]-offset) or np.any(focus > domain[:, 1]-offset)):
        raise ValueError("Focus poses must be inside the adjacent-approved software domain")
    windows = []
    for values in focus_boxes or []:
        box = np.vstack(([0., 0.], np.asarray(values, dtype=float).reshape(4, 2)))
        if (not np.isfinite(box).all() or np.any(box[1:, 1] <= box[1:, 0])
                or np.any(box[:, 0] < domain[:, 0]-offset)
                or np.any(box[:, 1] > domain[:, 1]-offset)):
            raise ValueError("Focus boxes must have positive widths inside the approved domain")
        windows.append(box)
    checker = BoundsChecker(source)
    grouped = {}
    for name in old["remainingConstraints"]:
        dependence = tuple(np.flatnonzero((np.diff(domain, axis=1).ravel() > 0)
                                         & (checker.constraints[name]["radius"] > 0)))
        grouped.setdefault(dependence, []).append(name)
    expected = set(checker.constraints)
    accounted = set(old["remainingConstraints"])
    for item in old["adjacentConstraints"]:
        accounted.update(item["constraints"])
    assert accounted == expected
    groups = []
    prior_groups = {tuple(g["dependentJoints"]): g for g in prior["groups"]} if prior else {}
    for dependence, names in sorted(grouped.items(), key=lambda item: (len(item[0]), item[0])):
        saved = prior_groups.get(tuple(int(j)+1 for j in dependence))
        if saved and saved["constraints"] != names:
            raise ValueError("Saved constraint identity differs")
        if saved and not saved["unknownReducedCadBoxes"]:
            groups.append(saved)
            print("Reused complete group", saved["dependentJoints"], flush=True)
            continue
        reduced = np.column_stack((offset, offset))
        for j in dependence:
            reduced[j] = domain[j]
        projected_focus = focus+offset
        for j in range(5):
            if j not in dependence:
                projected_focus[:, j] = offset[j]
        seeds = saved["unknownReducedCadBoxes"] if saved else None
        deferred = []
        if windows:
            projected_windows = np.array(windows)+offset[None, :, None]
            for j in range(5):
                if j not in dependence:
                    projected_windows[:, j] = offset[j]
            seeds, deferred = focus_seeds(
                seeds if seeds is not None else [dict(cadBoundsDeg=reduced.tolist(), unresolved=names)],
                projected_windows)
        result = partition_regions(checker, reduced, names, limit, min_width=.01, max_seconds=max_seconds,
                                   seeds=seeds, focus=projected_focus)
        result["unknown"].extend(deferred)
        result["unknownFraction"] += sum(box_measure(r["cadBoundsDeg"]) for r in deferred)/box_measure(reduced)
        boxes = list(saved["verifiedSoftwareBoxesDeg"]) if saved else []
        for row in result["verified"]:
            lifted = np.array(row["cadBoundsDeg"])
            for j in range(5):
                if j not in dependence:
                    lifted[j] = domain[j]
            boxes.append((lifted-offset[:, None]).tolist())
        if not result["unknown"]:
            boxes = [(domain-offset[:, None]).tolist()]
        else:
            boxes = compact_boxes(boxes)
        group = dict(dependentJoints=[int(j)+1 for j in dependence], constraints=names,
                     verifiedSoftwareBoxesDeg=boxes,
                     nodes=result["nodes"]+(saved["nodes"] if saved else 0),
                     boundChecks=result["boundChecks"]+(saved["boundChecks"] if saved else 0),
                     elapsedSeconds=result["elapsedSeconds"]+(saved["elapsedSeconds"] if saved else 0),
                     verifiedReducedDomainFraction=result["verifiedFraction"]+(
                         saved["verifiedReducedDomainFraction"] if saved else 0),
                     unknownReducedDomainFraction=result["unknownFraction"],
                     unknownReducedCadBoxes=result["unknown"])
        groups.append(group)
        assert abs(group["verifiedReducedDomainFraction"]+group["unknownReducedDomainFraction"]-1) < 1e-9
        print("Factored", group["dependentJoints"], "constraints", len(names),
              "verified", group["verifiedReducedDomainFraction"], "boxes", len(boxes), flush=True)
    soft_domain = domain-offset[:, None]
    recorded = np.array(config["recordedSoftwareRangesDeg"])
    rng = np.random.default_rng(2026111702)
    points = rng.uniform(recorded[:, 0], recorded[:, 1], (6000, 5))
    accepted = factored_membership(groups, points)
    accepted &= np.all((points >= soft_domain[:, 0]) & (points <= soft_domain[:, 1]), axis=1)
    if prior:
        before = factored_membership(prior["groups"], points)
        before &= np.all((points >= soft_domain[:, 0]) & (points <= soft_domain[:, 1]), axis=1)
        assert np.all(~before | accepted), "Refinement must not remove verified points"
    allowed_points = points[accepted]
    allowed_focus = focus[factored_membership(groups, focus)]
    window_points = np.array([q for box in windows
                              for q in itertools.product(*(np.unique(axis) for axis in box))])
    allowed_window_points = (window_points[factored_membership(groups, window_points)]
                             if windows else np.empty((0, 5)))
    checks, collisions = 0, []
    for q in np.vstack((allowed_focus, allowed_window_points, allowed_points[:300])):
        hits = checker.hits(offset+q)
        checks += 1
        if hits:
            collisions.append(dict(softwareDegrees=q.tolist(), pairs=hits))
    assert not collisions, collisions
    paths, used = [], set()
    # Find four true full-dimensional intersections, not a preferred 2-D curve.
    for q in sorted(np.vstack((allowed_focus, allowed_points)),
                    key=lambda q: float(q[1]+q[2]), reverse=True):
        box = soft_domain.copy()
        for group in groups:
            candidates = [np.array(b) for b in group["verifiedSoftwareBoxesDeg"]
                          if np.all(q >= np.array(b)[:, 0]) and np.all(q <= np.array(b)[:, 1])]
            chosen = max(candidates, key=box_measure)
            box[:, 0] = np.maximum(box[:, 0], chosen[:, 0])
            box[:, 1] = np.minimum(box[:, 1], chosen[:, 1])
        box[1:, 0] = np.maximum(box[1:, 0], recorded[1:, 0]+1)
        box[1:, 1] = np.minimum(box[1:, 1], recorded[1:, 1]-1)
        key = tuple(box.ravel())
        if key in used or np.any(np.diff(box[1:], axis=1).ravel() <= .2):
            continue
        used.add(key)
        width = np.diff(box, axis=1).ravel()
        a, b = box[:, 0]+.2*width, box[:, 1]-.2*width
        assert segment_in_regions([box.tolist()], a, b)
        assert factored_membership(groups, np.array([a, b])).all()
        paths.append(dict(id=f"factored_region_{len(paths)}_return",
                          softwareRegionBoundsDeg=box.tolist(),
                          waypointsCadDeg=[(offset+p).tolist() for p in (a, b, a)]))
        if len(paths) == 4:
            break
    focus_routes = []
    for index, window in enumerate(windows):
        box = window.copy()
        box[1:, 0] = np.maximum(box[1:, 0], recorded[1:, 0]+1)
        box[1:, 1] = np.minimum(box[1:, 1], recorded[1:, 1]-1)
        if np.any(box[1:, 1] <= box[1:, 0]):
            focus_routes.append(dict(index=index, status="no_inset_domain"))
            continue
        width = np.diff(box, axis=1).ravel()
        a, b = box[:, 0]+.1*width, box[:, 1]-.1*width
        covered = all(segment_in_regions(g["verifiedSoftwareBoxesDeg"], a, b) for g in groups)
        focus_routes.append(dict(index=index, status="whole_segment_covered" if covered else "not_proven",
                                 endpointsSoftwareDeg=[a.tolist(), b.tolist()]))
        if covered:
            paths.append(dict(id=f"focus_box_{index}_four_axis_return",
                              waypointsCadDeg=[(offset+p).tolist() for p in (a, b, a)]))
    out.mkdir(parents=True, exist_ok=False)
    write(out / "config.json", config)
    write(out / "geometry.json", dict(anchorCadDeg=offset.tolist(), paths=paths,
          coordinationDeltaCadDeg=[0, 8, -12, 20, 5]))
    write(out / "study.json", dict(parameters=[dict(type="factored_region_intersections")],
          limitations=config["limitations"]))
    report = dict(generatedUtc=datetime.now(timezone.utc).isoformat(),
        previousReport=str(original.resolve()),
        refinedFrom=(str((previous / "factored_regions.json").resolve()) if prior else None),
        refinedFromSha256=(hashlib.sha256((previous / "factored_regions.json").read_bytes()).hexdigest()
                           if prior else None),
        priorInputFileSha256=old["fileSha256"],
        implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        mapping=config["coordinateAssumption"], offsetCadDeg=offset.tolist(),
        recordedSoftwareRangesDeg=recorded.tolist(), admissibleSoftwareDomainDeg=soft_domain.tolist(),
        constraintsAccountedFor=len(accounted), groups=groups, dynamicRoutes=paths,
        perGroupNodeBudget=limit, perGroupTimeBudgetSeconds=max_seconds,
        focusSoftwareDegrees=focus.tolist(),
        focusSoftwareBoxesDeg=[b.tolist() for b in windows], focusRouteResults=focus_routes,
        formula="I5(q2,q3,q4) = admissible_J5 intersect intersection_g union_k I5_gk(q2,q3,q4)",
        coverageRandomSeed=2026111702, coverageSamples=len(points),
        coverageSamplesAccepted=int(accepted.sum()),
        previousCoverageSamplesAccepted=(int(before.sum()) if prior else None),
        estimatedRecordedJointMeasureFraction=float(accepted.mean()),
        coverageIsFiniteSampleEstimateNotCertifiedVolume=True,
        independentPoseChecks=checks, independentCollisions=collisions,
        exactQueries=checker.exact_queries, elapsedSeconds=time.perf_counter()-started,
        engine=f"python-fcl {fcl.__version__}", floatingPointGuardMm=.001,
        limitations=["All groups must pass; no union across different constraint groups",
                     "Numerical mesh surface bounds do not prove solid containment or hardware safety",
                     "Unproved cells are not authorized, not classified entirely colliding",
                     "Coverage fraction is a Monte Carlo estimate over recorded joint coordinates",
                     "No calibration, tracking tube, braking or cable verification", *config["limitations"]])
    report["focusResults"] = [dict(softwareDegrees=q.tolist(),
                                  verifiedJ5IntervalsDeg=factored_intervals(report, q))
                              for q in focus]
    # Verify inputs again after the calculation; script version may differ from
    # the preserved earlier report, whose empty/partial result is not rewritten.
    for filename, digest in old["fileSha256"].items():
        if Path(filename).resolve() != Path(__file__).resolve():
            with Path(filename).open("rb") as stream:
                assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, filename
    write(out / "factored_regions.json", report)
    print(f"Factored coverage estimate: {accepted.sum()}/{len(points)}; "
          f"independent poses {checks}, routes {len(paths)}, seconds {report['elapsedSeconds']:.1f}",
          flush=True)


def piecewise_transitions(previous, out):
    """Replay explicit cross-band paths only after whole-segment table coverage."""
    report = read(previous / "factored_regions.json")
    config = read(previous / "config.json")
    offset = np.asarray(report["offsetCadDeg"])
    assert config["offsetCadDeg"] == report["offsetCadDeg"]
    assert config["recordedSoftwareRangesDeg"] == report["recordedSoftwareRangesDeg"]
    domain = np.asarray(report["admissibleSoftwareDomainDeg"])
    recorded = np.asarray(report["recordedSoftwareRangesDeg"])
    for filename, digest in report["priorInputFileSha256"].items():
        if Path(filename).resolve() != Path(__file__).resolve():
            with Path(filename).open("rb") as stream:
                assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, filename
    paths = []
    for side, wrist in (("inner", [-191., -166., -151., -144.]),
                        ("outer", [-194., -194., -209., -216.])):
        for turn in (0., 360.):
            forward = np.array([[0., 36., 92., wrist[0]+turn, 18.2],
                                [0., 38., 96., wrist[1]+turn, 19.95],
                                [0., 42., 100., wrist[2]+turn, 21.55],
                                [0., 44., 103., wrist[3]+turn, 21.6]])
            if (np.any(forward < domain[:, 0]) or np.any(forward > domain[:, 1])
                    or np.any(forward[:, 1:] < recorded[1:, 0]+1)
                    or np.any(forward[:, 1:] > recorded[1:, 1]-1)):
                raise ValueError("Cross-band route leaves the approved/inset domain")
            waypoints = np.concatenate((forward, forward[-2::-1]))
            for a, b in zip(waypoints, waypoints[1:]):
                if not all(segment_in_regions(g["verifiedSoftwareBoxesDeg"], a, b)
                           for g in report["groups"]):
                    raise ValueError(f"Unproved cross-band segment: {side}, turn={turn}")
            paths.append(dict(id=f"piecewise_{side}_{int(turn)}_return",
                              waypointsCadDeg=(waypoints+offset).tolist()))
    out.mkdir(parents=True, exist_ok=False)
    write(out / "config.json", config)
    write(out / "geometry.json", dict(anchorCadDeg=offset.tolist(), paths=paths,
          coordinationDeltaCadDeg=[0, 8, -12, 20, 5]))
    write(out / "study.json", dict(parameters=[dict(type="verified_cross_band_transitions")],
          sourceReport=str((previous / "factored_regions.json").resolve()),
          sourceReportSha256=hashlib.sha256((previous / "factored_regions.json").read_bytes()).hexdigest(),
          implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
          limitations=["Every unwrapped nominal segment is covered by all constraint-group unions",
                       "Waypoint stops; no unverified corner smoothing",
                       "Two turn branches are geometrically equivalent, not independent poses",
                       "Actual trajectory collision checks still required", *config["limitations"]]))
    print("PASS: four cross-band returns, 24 directed segments covered by all table groups; dynamics pending")


def halfspace_in_regions(boxes, bounds, coefficients, lower, upper=math.inf):
    """Certify a halfspace/slab by bounding every piece outside the verified union."""
    bounds, coefficients = np.asarray(bounds), np.asarray(coefficients)
    if (bounds.shape != (5, 2) or coefficients.shape != (5,)
            or not np.isfinite(bounds).all() or not np.isfinite(coefficients).all()
            or not math.isfinite(lower) or math.isnan(upper) or upper < lower
            or np.any(bounds[:, 1] < bounds[:, 0])):
        raise ValueError("Expected finite box, linear coefficients and threshold")
    # Remove disjoint windows before exact subtraction; keep the original order.
    windows = np.asarray(boxes, dtype=float).reshape(-1, 5, 2)
    active = bounds[:, 1] > bounds[:, 0]
    low = np.maximum(windows[:, :, 0], bounds[:, 0])
    high = np.minimum(windows[:, :, 1], bounds[:, 1])
    intersects = np.all(high >= low, axis=1) & np.all(high[:, active] > low[:, active], axis=1)
    # Exact subtraction, pruning only pieces strictly outside the requested slab.
    # This avoids subdividing unrelated shoulder corners over thousands of windows.
    windows = windows[intersects]
    remaining, separated = [dict(cadBoundsDeg=bounds.tolist(), unresolved=[])], []
    for start in range(0, len(windows), 64):
        _, pieces = focus_seeds(remaining, windows[start:start+64])
        remaining = []
        for row in pieces:
            box = np.asarray(row["cadBoundsDeg"])
            high = float(np.dot(coefficients, np.where(coefficients >= 0, box[:, 1], box[:, 0])))
            low = float(np.dot(coefficients, np.where(coefficients >= 0, box[:, 0], box[:, 1])))
            (separated if max(lower-high, low-upper) > 0 else remaining).append(row)
        if not remaining:
            break
    maximum, minimum, separation, witness, conflicts = -math.inf, math.inf, math.inf, None, 0
    for row in separated+remaining:
        box = np.asarray(row["cadBoundsDeg"])
        high = float(np.dot(coefficients, np.where(coefficients >= 0, box[:, 1], box[:, 0])))
        low = float(np.dot(coefficients, np.where(coefficients >= 0, box[:, 0], box[:, 1])))
        maximum, minimum = max(maximum, high), min(minimum, low)
        gap = max(lower-high, low-upper)
        conflicts += gap <= 0
        if gap < separation:
            separation, witness = gap, box.tolist()
    return dict(proven=not conflicts, remainingBoxes=len(separated)+len(remaining),
                outsidePrunedBoxes=len(separated), intersectingRemainingBoxes=conflicts,
                maximumUncoveredLinearValue=None if witness is None else maximum,
                minimumUncoveredLinearValue=None if witness is None else minimum,
                linearSeparationDeg=None if witness is None else separation,
                worstRemainingSoftwareBoxDeg=witness)


def shoulder_polygon(bounds, lower, upper=math.inf, weight=1.5):
    """Clip the shoulder/elbow rectangle by lower <= weight*q2+q3 <= upper."""
    bounds = np.asarray(bounds)
    if (bounds.shape != (2, 2) or not np.isfinite(bounds).all()
            or np.any(bounds[:, 1] <= bounds[:, 0]) or not math.isfinite(lower)
            or math.isnan(upper) or upper < lower or not math.isfinite(weight)):
        raise ValueError("Invalid shoulder rectangle or weighted interval")
    polygon = [np.array([bounds[0, i], bounds[1, j]]) for i, j in ((0, 0), (1, 0), (1, 1), (0, 1))]
    planes = [(np.array([weight, 1.]), lower)]
    if math.isfinite(upper):
        planes.append((np.array([-weight, -1.]), -upper))
    for normal, threshold in planes:
        clipped = []
        if not polygon:
            break
        previous = polygon[-1]
        before = float(normal @ previous-threshold)
        for current in polygon:
            after = float(normal @ current-threshold)
            if (before >= 0) != (after >= 0):
                clipped.append(previous+before/(before-after)*(current-previous))
            if after >= 0:
                clipped.append(current)
            previous, before = current, after
        polygon = clipped
    return np.array(polygon).reshape(-1, 2)


def compact_wrist_intervals(band):
    """Continuous-angle branches; legacy bands are centred at abs(q4)=180."""
    center, radius = band.get("J4CenterAbsDeg", 180.), band["J4RadiusDeg"]
    if not all(math.isfinite(v) and v >= 0 for v in (center, radius)):
        raise ValueError("Expected finite nonnegative wrist centre and radius")
    return merge_intervals([[-center-radius, -center+radius], [center-radius, center+radius]])


def compact_rule(previous, out, variant="compact-rule", prior_rule=None):
    """Extract a simple sufficient rule without replacing the full allowed table."""
    assert variant in ("compact-rule", "extended-rule", "universal-rule", "piecewise-full-rule", "middle-rule", "negative-rule", "inner-rule", "affine-rule", "outer-rule")
    extended = variant == "extended-rule"
    piecewise_full = variant == "piecewise-full-rule"
    middle = variant == "middle-rule"
    negative = variant == "negative-rule"
    inner = variant == "inner-rule"
    affine = variant == "affine-rule"
    outer = variant == "outer-rule"
    full_wrist = variant in ("universal-rule", "piecewise-full-rule")
    report_path = previous / "factored_regions.json"
    report, config = read(report_path), read(previous / "config.json")
    offset = np.asarray(report["offsetCadDeg"])
    domain = np.asarray(report["admissibleSoftwareDomainDeg"])
    recorded = np.asarray(report["recordedSoftwareRangesDeg"])
    assert config["offsetCadDeg"] == report["offsetCadDeg"]
    assert config["recordedSoftwareRangesDeg"] == report["recordedSoftwareRangesDeg"]
    assert config["signs"] == [1]*5 and np.array_equal(domain[0], [0., 0.])
    assert report["constraintsAccountedFor"] == 89
    identity = {name: digest for name, digest in report["priorInputFileSha256"].items()
                if Path(name).resolve() != Path(__file__).resolve()}
    identity[str(report_path.resolve())] = hashlib.sha256(report_path.read_bytes()).hexdigest()
    inherited = None
    if middle or negative or inner or affine or outer:
        assert prior_rule is not None
        inherited_path = prior_rule / "compact_rule.json"
        inherited = read(inherited_path)
        assert read(prior_rule / "config.json") == config
        assert Path(inherited["sourceReport"]).resolve() == report_path.resolve()
        assert inherited["coordinateDomainSoftwareDeg"] == domain.tolist()
        assert inherited["coefficients"] == [0., 1.5, 1., 0., 0.]
        if negative or inner or affine or outer:
            comparison_path = prior_rule / "compact_rule_comparison.json"
            combined = read(comparison_path)["combinedRule"]
            assert combined["coordinateDomainSoftwareDeg"] == domain.tolist()
            assert combined["coefficients"] == inherited["coefficients"]
            identity[str(comparison_path.resolve())] = hashlib.sha256(comparison_path.read_bytes()).hexdigest()
            if negative:
                negative_bands = [dict(b, J5Deg=[-16.6, b["J5Deg"][1]])
                                  for b in combined["bands"] if b["J5Deg"][0] == -16]
                assert len(negative_bands) == 3
                assert {b["J4RadiusDeg"] for b in negative_bands} == {30., 120., 180.}
            indices = [i for i, b in enumerate(inherited["bands"]) if b["J4RadiusDeg"] == 180]
            if inner or affine or outer:
                # The negative extension retains both the old A and its larger replacement.
                indices = [i for i in indices if not any(
                    j != i and inherited["bands"][j]["J5Deg"] != inherited["bands"][i]["J5Deg"]
                    and inherited["bands"][j]["J5Deg"][0] <= inherited["bands"][i]["J5Deg"][0]
                    and inherited["bands"][j]["J5Deg"][1] >= inherited["bands"][i]["J5Deg"][1]
                    and inherited["bands"][j]["weightedLowerDeg"] == inherited["bands"][i]["weightedLowerDeg"]
                    and inherited["bands"][j].get("weightedUpperDeg") == inherited["bands"][i].get("weightedUpperDeg")
                    for j in indices)]
            inherited = dict(inherited, bands=[inherited["bands"][i] for i in indices],
                             audits=[dict(a, band=indices.index(a["band"])) for a in inherited["audits"]
                                     if a["band"] in indices])
        assert len(inherited["bands"]) == 4 and all(b["J4RadiusDeg"] == 180 for b in inherited["bands"])
        assert all(c["proven"] for a in inherited["audits"] for c in a["groupChecks"])
        for name, digest in inherited["sourceFileSha256"].items():
            assert name not in identity or identity[name] == digest, name
            identity[name] = digest
        identity[str(inherited_path.resolve())] = hashlib.sha256(inherited_path.read_bytes()).hexdigest()
    def check_inputs():
        for name, digest in identity.items():
            with Path(name).open("rb") as stream:
                assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, name
    check_inputs()
    bands = [dict(J5Deg=[18., 19.9], J4RadiusDeg=15., weightedLowerDeg=-138.5),
             dict(J5Deg=[19.9, 21.5], J4RadiusDeg=30., weightedLowerDeg=-137.1),
             dict(J5Deg=[21.5, 22.632856], J4RadiusDeg=38., weightedLowerDeg=-132.3)]
    if extended:
        bands[:0] = [dict(J5Deg=[-16., -10.], J4RadiusDeg=30., weightedLowerDeg=-113.5, weightedUpperDeg=144.3),
                     dict(J5Deg=[-10., 0.], J4RadiusDeg=30., weightedLowerDeg=-113.5, weightedUpperDeg=147.5),
                     dict(J5Deg=[0., 10.], J4RadiusDeg=30., weightedLowerDeg=-113.5, weightedUpperDeg=152.7),
                     dict(J5Deg=[10., 18.], J4RadiusDeg=30., weightedLowerDeg=-135., weightedUpperDeg=165.5)]
    if full_wrist:
        bands = [dict(J5Deg=[-16., 22.632856], J4RadiusDeg=180.,
                      weightedLowerDeg=-3., weightedUpperDeg=41.)]
    if piecewise_full:
        bands = [dict(J5Deg=[-16., 0.], J4RadiusDeg=180., weightedLowerDeg=-122.7, weightedUpperDeg=112.6),
                 dict(J5Deg=[0., 10.], J4RadiusDeg=180., weightedLowerDeg=-113.5, weightedUpperDeg=47.6),
                 dict(J5Deg=[10., 21.5], J4RadiusDeg=180., weightedLowerDeg=-15.6, weightedUpperDeg=42.9),
                 dict(J5Deg=[21.5, 22.632856], J4RadiusDeg=180., weightedLowerDeg=-3., weightedUpperDeg=42.9)]
    if middle:
        bands = [dict(b, J4RadiusDeg=120., weightedUpperDeg=120.3) for b in inherited["bands"]]
    if negative:
        bands = negative_bands
    if inner:
        bands = [dict(J5Deg=[-16.6, 22.632856], J4CenterAbsDeg=0., J4RadiusDeg=140.,
                      weightedLowerDeg=-122.7, weightedUpperDeg=42.9)]
    if affine:
        bands = [dict(J5Deg=fifth, J4CenterAbsDeg=0., J4RadiusDeg=60., J2Weight=.5,
                      weightedLowerDeg=lower, weightedUpperDeg=upper)
                 for fifth, lower, upper in (([-16.6, 0.], -108.2, 82.9),
                                              ([0., 10.], -112.9, 82.9),
                                              ([10., 22.632856], -115.2, 78.9))]
        bands += [dict(J5Deg=fifth, J4CenterAbsDeg=0., J4RadiusDeg=140., J2Weight=.5,
                       weightedLowerDeg=-94., weightedUpperDeg=upper)
                  for fifth, upper in (([-16.6, 10.], 82.9), ([10., 22.632856], 78.9))]
    if outer:
        survey_path = prior_rule.parent / "workspace_outer_wrist_survey_20261117_v1/candidate_survey.json"
        survey = read(survey_path)
        for name, digest in survey["inputFileSha256"].items():
            assert name not in identity or identity[name] == digest, name
            identity[name] = digest
        identity[str(survey_path.resolve())] = hashlib.sha256(survey_path.read_bytes()).hexdigest()
        selected = survey["greedySampleComparison"]
        assert len(selected) == 8 and selected[-1]["cumulativeNewSamples"] == 136
        assert survey["previousReadableAccepted"] == 4767
        assert all(row["J2Deg"] == domain[1].tolist() for row in selected)
        bands = [dict(J5Deg=row["J5Deg"], J4CenterAbsDeg=row["J4CenterAbsDeg"],
                      J4RadiusDeg=row["J4RadiusDeg"], J2Weight=row["weight"],
                      weightedLowerDeg=row["weightedLowerDeg"], weightedUpperDeg=row["weightedUpperDeg"])
                 for row in selected]
    witnesses = {}
    coefficients = np.array([0., 1.5, 1., 0., 0.])
    audits, paths = [], []
    for index, band in enumerate(bands):
        radius, threshold = band["J4RadiusDeg"], band["weightedLowerDeg"]
        center = band.get("J4CenterAbsDeg", 180.)
        weight = band.get("J2Weight", 1.5)
        band_coefficients = np.array([0., weight, 1., 0., 0.])
        upper = band.get("weightedUpperDeg", math.inf)
        for turn in ((0.,) if center == 0 else (0., 2*center)):
            target = domain.copy()
            target[3], target[4] = [-center-radius+turn, -center+radius+turn], band["J5Deg"]
            target[3, 0] = max(target[3, 0], domain[3, 0])
            target[3, 1] = min(target[3, 1], domain[3, 1])
            checks = [halfspace_in_regions(g["verifiedSoftwareBoxesDeg"], target, band_coefficients, threshold, upper)
                      for g in report["groups"]]
            assert all(c["proven"] for c in checks), (index, turn, checks)
            print(f"Verified whole slab: band={index}, turn={turn:g}", flush=True)
            audits.append(dict(band=index, turnDeg=turn, targetSoftwareBoxDeg=target.tolist(),
                               groupChecks=checks))
            box = target.copy()
            box[1:, 0] = np.maximum(box[1:, 0], recorded[1:, 0]+1)
            box[1:, 1] = np.minimum(box[1:, 1], recorded[1:, 1]-1)
            assert np.all(box[1:, 1] > box[1:, 0])
            low, high = box[:, 0], box[:, 1]
            wrist_low, wrist_high = low[3]+1, high[3]-1
            fifth_low, fifth_high = low[4]+.1*(high[4]-low[4]), high[4]-.1*(high[4]-low[4])
            if negative:
                fifth_low, fifth_high = -16.55, -16.05
            forward = np.array([[0., low[1], threshold-weight*low[1]+1, wrist_low, fifth_low],
                                [0., high[1], low[2], wrist_high, fifth_high],
                                [0., high[1], high[2], wrist_low, fifth_low],
                                [0., low[1], high[2], wrist_high, fifth_high]])
            if extended or full_wrist or middle or negative or inner or affine or outer:
                polygon = shoulder_polygon(box[1:3], threshold+1, upper-1, weight)
                assert len(polygon) >= 3
                forward = np.array([[0., q2, q3, wrist_low if k % 2 == 0 else wrist_high,
                                     fifth_low if k % 2 == 0 else fifth_high]
                                    for k, (q2, q3) in enumerate(polygon)])
            assert np.all(forward >= box[:, 0]) and np.all(forward <= box[:, 1])
            assert np.all(forward @ band_coefficients >= threshold+1-1e-10)
            assert np.all(forward @ band_coefficients <= upper-1+1e-10)
            waypoints = np.concatenate((forward, forward[:1]))
            if outer:
                probes = np.concatenate([a+(b-a)*np.linspace(0, 1, 101)[:, None]
                                         for a, b in zip(waypoints, waypoints[1:])])
                novel = np.flatnonzero(~compact_membership(combined, probes))
                assert len(novel), ("Loop does not enter the old complement", index, turn)
                witnesses[index, -1 if turn == 0 else 1] = probes[novel[len(novel)//2]].copy()
            paths.append(dict(id=f"compact_band_{index}_{int(turn)}_loop",
                              waypointsCadDeg=(waypoints+offset).tolist()))
        # Polygon area is joint-coordinate measure, not Cartesian workspace volume.
        polygon = shoulder_polygon(domain[1:3], threshold, upper, weight)
        area = abs(float(polygon[:, 0] @ np.roll(polygon[:, 1], 1)
                         -polygon[:, 1] @ np.roll(polygon[:, 0], 1)))/2
        band["retainedShoulderElbowFraction"] = area/float(np.prod(np.diff(domain[1:3], axis=1)))
    for turn in (() if negative or inner or affine or outer else ((0.,) if full_wrist else (0., 360.))):
        q2 = domain[1, 0]
        forward = np.array([[0., q2+1, -138.5-1.5*(q2+1)+2, -194+turn, 18.2],
                            [0., q2+2, -137.1-1.5*(q2+2)+2, -194+turn, 20.1],
                            [0., q2+3, -132.3-1.5*(q2+3)+2, -194+turn, 21.55],
                            [0., -20., -85., -217+turn, 21.6]])
        if extended:
            q5s = [float(np.mean(np.clip(b["J5Deg"], recorded[4, 0]+1, recorded[4, 1]-1))) for b in bands]
            forward = np.array([[0., q2, 10-1.5*q2, -194+turn if k % 2 == 0 else -166+turn, q5]
                                for k, (q2, q5) in enumerate(zip(np.linspace(-20., 20., len(bands)), q5s))])
        if full_wrist:
            forward = np.array([[0., -20., 40., recorded[3, 0]+1, -15.],
                                [0., 20., -20., recorded[3, 1]-1, recorded[4, 1]-1]])
        if piecewise_full:
            q5s = [float(np.mean(np.clip(b["J5Deg"], recorded[4, 0]+1, recorded[4, 1]-1))) for b in bands]
            forward = np.array([[0., q2, 10-1.5*q2,
                                 recorded[3, k % 2]+(1 if k % 2 == 0 else -1), q5]
                                for k, (q2, q5) in enumerate(zip(np.linspace(-20., 20., len(bands)), q5s))])
        if middle:
            q5s = [float(np.mean(np.clip(b["J5Deg"], recorded[4, 0]+1, recorded[4, 1]-1))) for b in bands]
            wrist = [max(recorded[3, 0]+1, -299+turn), min(recorded[3, 1]-1, -61+turn)]
            forward = np.array([[0., q2, 100-1.5*q2, wrist[k % 2], q5]
                                for k, (q2, q5) in enumerate(zip(np.linspace(0., 30., len(bands)), q5s))])
        waypoints = np.concatenate((forward, forward[-2::-1]))
        paths.append(dict(id=f"compact_cross_band_{int(turn)}_return",
                          waypointsCadDeg=(waypoints+offset).tolist()))
    if piecewise_full or middle:
        for index, (left, right) in enumerate(zip(bands, bands[1:])):
            boundary = left["J5Deg"][1]
            assert boundary == right["J5Deg"][0]
            polygon = shoulder_polygon(
                recorded[1:3]+[1., -1.],
                max(left["weightedLowerDeg"], right["weightedLowerDeg"])+1.,
                min(left["weightedUpperDeg"], right["weightedUpperDeg"])-1.)
            assert len(polygon) >= 3
            a, b = max(itertools.combinations(polygon, 2), key=lambda pair: np.linalg.norm(pair[1]-pair[0]))
            delta = min(.25, (left["J5Deg"][1]-left["J5Deg"][0])/4,
                        (right["J5Deg"][1]-right["J5Deg"][0])/4,
                        boundary-recorded[4, 0]-1, recorded[4, 1]-1-boundary)
            assert delta > 0
            wrists = [(recorded[3, 0]+1, recorded[3, 1]-1)]
            if middle:
                wrists = [(max(recorded[3, 0]+1, -299+turn), min(recorded[3, 1]-1, -61+turn))
                          for turn in (0., 360.)]
            for branch, (low, high) in enumerate(wrists):
                forward = np.array([[0., *a, low, boundary-delta],
                                    [0., *b, high, boundary+delta]])
                paths.append(dict(id=f"compact_boundary_{index}_{branch}_wrist_return" if middle
                                  else f"compact_boundary_{index}_full_wrist_return",
                                  waypointsCadDeg=(np.concatenate((forward, forward[:1]))+offset).tolist()))
    if middle:
        for index, band in enumerate(bands):
            q5 = float(np.mean(np.clip(band["J5Deg"], recorded[4, 0]+1, recorded[4, 1]-1)))
            forward = np.array([[0., 10., 100., -100., q5], [0., -10., 25., -80., q5],
                                [0., 10., -5., 80., q5], [0., 10., 100., 100., q5]])
            paths.append(dict(id=f"compact_middle_full_{index}_handoff_return",
                              waypointsCadDeg=(np.concatenate((forward, forward[-2::-1]))+offset).tolist()))
    if negative:
        for index, band in enumerate(bands):
            polygon = shoulder_polygon(recorded[1:3]+[1., -1.], band["weightedLowerDeg"]+1,
                                       band["weightedUpperDeg"]-1)
            a, b = max(itertools.combinations(polygon, 2), key=lambda pair: np.linalg.norm(pair[1]-pair[0]))
            radius = band["J4RadiusDeg"]
            wrists = [(recorded[3, 0]+1, recorded[3, 1]-1)] if radius == 180 else [
                (max(recorded[3, 0]+1, -180-radius+turn)+1,
                 min(recorded[3, 1]-1, -180+radius+turn)-1) for turn in (0., 360.)]
            for branch, (low, high) in enumerate(wrists):
                forward = np.array([[0., *a, low, -16.55], [0., *b, high, -15.5]])
                paths.append(dict(id=f"compact_negative_boundary_{index}_{branch}_return",
                                  waypointsCadDeg=(np.concatenate((forward, forward[:1]))+offset).tolist()))
        for sign in (-1., 1.):
            forward = np.array([[0., 30., 95., sign*180, -16.55],
                                [0., 20., 88., sign*150, -16.05],
                                [0., 20., 88., sign*100, -16.55],
                                [0., -10., 25., sign*80, -16.05],
                                [0., 10., -5., -sign*80, -16.55],
                                [0., 20., 88., -sign*100, -16.05],
                                [0., 20., 88., -sign*150, -15.5],
                                [0., 30., 95., -sign*180, -16.55]])
            paths.append(dict(id=f"compact_negative_handoff_{int(sign)}_return",
                              waypointsCadDeg=(np.concatenate((forward, forward[-2::-1]))+offset).tolist()))
        assert len(paths) == 13
        assert all(min(np.asarray(p["waypointsCadDeg"])[:, 4]-offset[4]) < -16 for p in paths)
    if inner:
        polygon = shoulder_polygon(recorded[1:3]+[1., -1.], -121.7, 41.9)
        for index, (low, high) in enumerate(((0., 10.), (10., 18.), (18., 19.9),
                                            (19.9, 21.5), (21.5, 22.632856))):
            low, high = max(low, recorded[4, 0]+1), min(high, recorded[4, 1]-1)
            assert high > low
            fifth = [low+.1*(high-low), high-.1*(high-low)]
            forward = np.array([[0., q2, q3, (-139., 139.)[k % 2], fifth[k % 2]]
                                for k, (q2, q3) in enumerate(polygon)])
            paths.append(dict(id=f"compact_inner_slice_{index}_loop",
                              waypointsCadDeg=(np.concatenate((forward, forward[:1]))+offset).tolist()))
        for index, boundary in enumerate((0., 10., 18., 19.9, 21.5)):
            delta = min(.25, (recorded[4, 1]-1-boundary)/2)
            assert delta > 0
            forward = np.array([[0., -25., -83.5, -139., boundary-delta],
                                [0., -20., -91., 139., boundary+delta]])
            paths.append(dict(id=f"compact_inner_boundary_{index}_return",
                              waypointsCadDeg=(np.concatenate((forward, forward[:1]))+offset).tolist()))
        for sign in (-1., 1.):
            outer = sign*min(-recorded[3, 0]-1, recorded[3, 1]-1)
            forward = np.array([[0., -25., -82.5, sign*139, 5.],
                                [0., -20., -90., sign*100, 12.],
                                [0., -20., 40., sign*139, 18.5],
                                [0., 0., 10., sign*141, 20.5],
                                [0., 20., -20., outer, 21.6],
                                [0., -10., 25., -outer, -16.55],
                                [0., 0., 10., -sign*141, .5],
                                [0., 0., 10., -sign*139, 5.],
                                [0., -25., -82.5, -sign*139, 12.]])
            paths.append(dict(id=f"compact_inner_full_{int(sign)}_handoff_return",
                              waypointsCadDeg=(np.concatenate((forward, forward[-2::-1]))+offset).tolist()))
        forward = np.array([[0., -25., -82.5, -139., -16.55],
                            [0., -20., -90., 139., 21.6]])
        paths.append(dict(id="compact_inner_whole_band_return",
                          waypointsCadDeg=(np.concatenate((forward, forward[:1]))+offset).tolist()))
        assert len(paths) == 14
        for path in paths:
            software = np.asarray(path["waypointsCadDeg"])-offset
            probes = np.concatenate([a+(b-a)*np.linspace(0, 1, 101)[:, None]
                                     for a, b in zip(software, software[1:])])
            assert np.any(~compact_membership(combined, probes)), path["id"]
    if affine:
        for index, boundary in enumerate((0., 10., 18., 19.9, 21.5)):
            delta = min(.25, (recorded[4, 1]-1-boundary)/2)
            assert delta > 0
            forward = np.array([[0., 30., 62.8, -59., boundary-delta],
                                [0., 40., 57.8, 59., boundary+delta]])
            paths.append(dict(id=f"compact_affine_boundary_{index}_return",
                              waypointsCadDeg=(np.concatenate((forward, forward[:1]))+offset).tolist()))
        for index, (fifth, lower) in enumerate(((-5., -108.2), (5., -112.9), (12., -115.2))):
            value = max(lower+1, -112.)
            forward = np.array([[0., -35., value+17.5, -59., fifth],
                                [0., -25., -80.5, -59., fifth],
                                [0., 35., 52.5, -61., fifth],
                                [0., 40., 50., -139., fifth]])
            paths.append(dict(id=f"compact_affine_wrist_boundary_{index}_return",
                              waypointsCadDeg=(np.concatenate((forward, forward[-2::-1]))+offset).tolist()))
        for sign in (-1., 1.):
            outer = sign*min(-recorded[3, 0]-1, recorded[3, 1]-1)
            forward = np.array([[0., -35., -94.5, sign*59, 12.],
                                [0., -25., -80.5, sign*59, 5.],
                                [0., 40., 57.8, sign*59, 5.],
                                [0., 40., 57.8, sign*61, 12.],
                                [0., 40., 57.8, sign*139, 21.6],
                                [0., 0., 10., sign*139, 21.6],
                                [0., 0., 10., sign*141, 20.5],
                                [0., 20., -20., outer, 21.6],
                                [0., -10., 25., -outer, -16.55],
                                [0., -10., 25., -sign*139, -16.55],
                                [0., 40., 57.8, -sign*139, -5.],
                                [0., 40., 57.8, -sign*59, 5.],
                                [0., -35., -94.5, -sign*59, 12.]])
            paths.append(dict(id=f"compact_affine_full_{int(sign)}_handoff_return",
                              waypointsCadDeg=(np.concatenate((forward, forward[-2::-1]))+offset).tolist()))
        forward = np.array([[0., 40., 57.8, -59., -16.55],
                            [0., 35., 60.3, 59., 21.6]])
        paths.append(dict(id="compact_affine_whole_fifth_return",
                          waypointsCadDeg=(np.concatenate((forward, forward[:1]))+offset).tolist()))
        assert len(paths) == 16
        for path in paths:
            software = np.asarray(path["waypointsCadDeg"])-offset
            probes = np.concatenate([a+(b-a)*np.linspace(0, 1, 101)[:, None]
                                     for a, b in zip(software, software[1:])])
            assert np.any(~compact_membership(combined, probes)), path["id"]
    if outer:
        for index, boundary in enumerate((-10., 0., 10., 18., 19.9, 21.5)):
            delta = min(.25, (recorded[4, 1]-1-boundary)/2)
            assert delta > 0
            sign = (-1., 1.)[index % 2]
            forward = np.array([[0., 38., 66., sign*225., boundary-delta],
                                [0., 43., 63.5, sign*223., boundary+delta]])
            paths.append(dict(id=f"compact_outer_fifth_boundary_{index}_return",
                              waypointsCadDeg=(np.concatenate((forward, forward[:1]))+offset).tolist()))
        for sign in (-1, 1):
            forward = []
            for index in range(len(bands)):
                witness = witnesses[index, sign]
                home = np.array([0., 0., 10., 0., witness[4]])
                retract = np.array([0., 0., 10., witness[3], witness[4]])
                forward.extend([home, retract, witness, retract, home])
            forward = np.asarray(forward)
            paths.append(dict(id=f"compact_outer_all_bands_{sign}_handoff_return",
                              waypointsCadDeg=(np.concatenate((forward, forward[-2::-1]))+offset).tolist()))
        assert len(paths) == 24
        for path in paths:
            software = np.asarray(path["waypointsCadDeg"])-offset
            probes = np.concatenate([a+(b-a)*np.linspace(0, 1, 101)[:, None]
                                     for a, b in zip(software, software[1:])])
            assert np.any(~compact_membership(combined, probes)), path["id"]
    if middle or negative or inner or affine or outer:
        audits.extend(dict(a, band=a["band"]+len(bands), inheritedFrom=str(inherited_path.resolve()))
                      for a in inherited["audits"])
        bands.extend(inherited["bands"])
    segments = 0
    for path in paths:
        software = np.asarray(path["waypointsCadDeg"])-offset
        assert np.all(software[:, 1:] >= recorded[1:, 0]+1)
        assert np.all(software[:, 1:] <= recorded[1:, 1]-1)
        for a, b in zip(software, software[1:]):
            assert all(segment_in_regions(g["verifiedSoftwareBoxesDeg"], a, b)
                       for g in report["groups"]), (path["id"], a, b)
            segments += 1
    check_inputs()
    out.mkdir(parents=True, exist_ok=False)
    provenance = dict(sourceReport=str(report_path.resolve()), sourceFileSha256=identity,
                      implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    write(out / "compact_rule.json", dict(generatedUtc=datetime.now(timezone.utc).isoformat(),
          **provenance, coordinateDomainSoftwareDeg=domain.tolist(), coefficients=coefficients.tolist(),
          bands=bands, audits=audits, wholeNominalSegments=segments,
          **(dict(negativeExtension=dict(priorLowerDeg=-16., newLowerDeg=-16.6)) if negative else {}),
          **(dict(innerExtension=dict(priorComparison=str(comparison_path.resolve()))) if inner else {}),
          **(dict(affineExtension=dict(priorComparison=str(comparison_path.resolve()))) if affine else {}),
          **(dict(outerExtension=dict(priorComparison=str(comparison_path.resolve()),
                                    candidateSurvey=str(survey_path.resolve()))) if outer else {}),
          formula="Within recorded/admissible domain, ANY closed band: abs(abs(q4)-center)<=radius AND lower<=weight*q2+q3<=upper (center defaults to 180, J2Weight to 1.5; omitted upper is unbounded)",
          limitations=["Inner approximation; full source table remains authoritative and larger",
                       "Shared J5 endpoints use union of both rows, each with its own shoulder condition",
                       "Same rule checked separately, NOT unioned across calibration hypotheses",
                       "Numerical model only; no hardware, tracking-tube or physical-clearance proof"]))
    write(out / "config.json", config)
    write(out / "geometry.json", dict(anchorCadDeg=offset.tolist(), paths=paths,
          coordinationDeltaCadDeg=[0, 8, -12, 20, 5]))
    write(out / "study.json", dict(parameters=[dict(type="compact_joint_dependent_rule")],
          **provenance, limitations=config["limitations"]))
    print(f"PASS: compact rule, {len(audits)} clipped boxes, {segments} whole nominal segments; dynamics pending",
          flush=True)


def compact_membership(rule, points, tolerance=0.):
    points = np.asarray(points)
    if (points.ndim != 2 or points.shape[1] != 5 or not np.isfinite(points).all()
            or not math.isfinite(tolerance) or tolerance < 0):
        raise ValueError("Expected finite five-joint points and nonnegative tolerance")
    domain = np.asarray(rule["coordinateDomainSoftwareDeg"])
    allowed = np.zeros(len(points), dtype=bool)
    for band in rule["bands"]:
        weight = band.get("J2Weight", 1.5)
        if not math.isfinite(weight):
            raise ValueError("Expected finite shoulder weight")
        weighted = weight*points[:, 1]+points[:, 2]
        wrist = np.zeros(len(points), dtype=bool)
        for low, high in compact_wrist_intervals(band):
            wrist |= (points[:, 3] >= low-tolerance) & (points[:, 3] <= high+tolerance)
        allowed |= ((points[:, 4] >= band["J5Deg"][0]-tolerance)
                    & (points[:, 4] <= band["J5Deg"][1]+tolerance)
                    & wrist
                    & (weighted >= band["weightedLowerDeg"]-tolerance)
                    & (weighted <= band.get("weightedUpperDeg", math.inf)+tolerance))
    return allowed & np.all((points >= domain[:, 0]-tolerance)
                            & (points <= domain[:, 1]+tolerance), axis=1)


def compact_audit(source):
    """Check whole nominal pieces and the actual four-axis projection separately."""
    rule = read(source / "compact_rule.json")
    for name, digest in rule["sourceFileSha256"].items():
        with Path(name).open("rb") as stream:
            assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, name
    assert rule["coefficients"] == [0., 1.5, 1., 0., 0.]
    config, geometry = read(source / "config.json"), read(source / "geometry.json")
    assert config == read(Path(rule["sourceReport"]).parent / "config.json")
    offset = np.asarray(config["offsetCadDeg"])
    segments = pieces = 0
    # Fixed linear segments are convex within each band/continuous-angle branch.
    # Split at J5 boundaries; testing both ends of each convex piece is sufficient.
    for path in geometry["paths"]:
        software = np.asarray(path["waypointsCadDeg"])-offset
        for a, b in zip(software, software[1:]):
            cuts = [0., 1.]
            if b[4] != a[4]:
                cuts.extend((v-a[4])/(b[4]-a[4]) for band in rule["bands"] for v in band["J5Deg"]
                            if 0 < (v-a[4])/(b[4]-a[4]) < 1)
            cuts = sorted(set(cuts))
            for start, end in zip(cuts, cuts[1:]):
                q = np.array([a+(b-a)*start, a+(b-a)*end])
                assert compact_membership(rule, q, 1e-10).all()
                assert any(np.all(q[:, 4] >= band["J5Deg"][0]-1e-10)
                           and np.all(q[:, 4] <= band["J5Deg"][1]+1e-10)
                           and np.all(band.get("J2Weight", 1.5)*q[:, 1]+q[:, 2] >= band["weightedLowerDeg"]-1e-10)
                           and np.all(band.get("J2Weight", 1.5)*q[:, 1]+q[:, 2] <= band.get("weightedUpperDeg", math.inf)+1e-10)
                           and any(np.all(q[:, 3] >= low-1e-10)
                                   and np.all(q[:, 3] <= high+1e-10)
                                   for low, high in compact_wrist_intervals(band))
                           for band in rule["bands"]), (path["id"], start, end)
                pieces += 1
            segments += 1
    assert segments == rule["wholeNominalSegments"]
    samples = [s for s in read(source / "actual_trace.json")["samples"]
               if s["case"].split("/", 1)[1].startswith("compact_")]
    assert samples
    q = np.asarray([s["cadDegrees"] for s in samples])-offset
    first_axis_error = float(abs(q[:, 0]).max())
    q[:, 0] = 0  # Only the 4-D projection is certified by the compact rule.
    allowed = compact_membership(rule, q)
    files = ("compact_rule.json", "config.json", "geometry.json", "actual_trace.json", "dynamics.json")
    identity = {name: hashlib.sha256((source / name).read_bytes()).hexdigest() for name in files}
    result = dict(wholeNominalSegments=segments, convexNominalPieces=pieces,
                  projectedActualSamples=len(samples), projectedActualViolations=int((~allowed).sum()),
                  maximumActualJ1DeviationDeg=first_axis_error, nominalComparisonToleranceDeg=1e-10,
                  actualComparisonToleranceDeg=0., inputFileSha256=identity,
                  implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  limitations=["Actual membership is J2..J5 projection only, not a 5-D tracking tube",
                               "Independent FCL must still use actual J1, without projection",
                               "Matched controls are separate regressions, excluded from the rule-specific trace subset"])
    if "negativeExtension" in rule:
        extension = q[:, 4] < rule["negativeExtension"]["priorLowerDeg"]
        entered = Counter(s["case"] for s, inside in zip(samples, extension) if inside)
        expected = {f"{speed}/{p['id']}" for p in geometry["paths"] for speed in ("slow", "brisk")}
        assert set(entered) == expected, (expected-set(entered), entered)
        result.update(extensionActualSamples=int(extension.sum()), extensionCaseSamples=dict(entered),
                      extensionCasesEntered=len(entered), expectedExtensionCases=len(expected),
                      minimumActualJ5Deg=float(q[:, 4].min()))
    if "innerExtension" in rule or "affineExtension" in rule or "outerExtension" in rule:
        extension = rule.get("outerExtension", rule.get("affineExtension", rule.get("innerExtension")))
        prior = read(Path(extension["priorComparison"]))["combinedRule"]
        novel = ~compact_membership(prior, q)
        entered = Counter(s["case"] for s, inside in zip(samples, novel) if inside)
        expected = {f"{speed}/{p['id']}" for p in geometry["paths"] for speed in ("slow", "brisk")}
        assert set(entered) == expected, (expected-set(entered), entered)
        result.update(newReadableActualSamples=int(novel.sum()), newReadableCaseSamples=dict(entered),
                      newReadableCasesEntered=len(entered), expectedNewReadableCases=len(expected))
    write(source / "compact_rule_audit.json", result)
    assert allowed.all(), result
    print(result, flush=True)


def compare_rules(source, folded_source):
    """Record the expanded union, retaining all prior folded-wrist conditions."""
    full, folded = read(source / "compact_rule.json"), read(folded_source / "compact_rule.json")
    assert read(source / "config.json") == read(folded_source / "config.json")
    assert full["coordinateDomainSoftwareDeg"] == folded["coordinateDomainSoftwareDeg"]
    assert full["coefficients"] == folded["coefficients"] == [0., 1.5, 1., 0., 0.]
    prior_comparison_path = folded_source / "compact_rule_comparison.json"
    prior_comparison = read(prior_comparison_path) if prior_comparison_path.is_file() else None
    prior_identity = {}
    if prior_comparison:
        prior_identity = dict(prior_comparison["inputFileSha256"])
        prior_identity[str(prior_comparison_path.resolve())] = hashlib.sha256(prior_comparison_path.read_bytes()).hexdigest()
        for name, digest in prior_identity.items():
            with Path(name).open("rb") as stream:
                assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, name
            if Path(name).name == "compact_rule.json":
                prior_rule = read(Path(name))
                assert prior_rule["coordinateDomainSoftwareDeg"] == full["coordinateDomainSoftwareDeg"]
                assert prior_rule["coefficients"] == full["coefficients"]
                assert prior_rule["audits"] and all(a["groupChecks"] for a in prior_rule["audits"])
                assert all(c["proven"] for a in prior_rule["audits"] for c in a["groupChecks"])
                for original, expected in prior_rule["sourceFileSha256"].items():
                    with Path(original).open("rb") as stream:
                        assert hashlib.file_digest(stream, "sha256").hexdigest() == expected, original
    for rule in (full, folded):
        assert all(c["proven"] for audit in rule["audits"] for c in audit["groupChecks"])
        for name, digest in rule["sourceFileSha256"].items():
            with Path(name).open("rb") as stream:
                assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, name
    # Exact interval coverage of the former uniform strip, not a sampled subset claim.
    full_intervals = merge_intervals([b["J5Deg"] for b in full["bands"]
        if b.get("J4CenterAbsDeg", 180.) == 180 and b.get("J2Weight", 1.5) == 1.5
        and b["J4RadiusDeg"] >= 180 and b["weightedLowerDeg"] <= -3
        and b.get("weightedUpperDeg", math.inf) >= 41])
    assert any(low <= -16 and high >= 22.632856 for low, high in full_intervals)
    old = dict(full, bands=folded["bands"]+[
        dict(J5Deg=[-16., 22.632856], J4RadiusDeg=180., weightedLowerDeg=-3., weightedUpperDeg=41.)])
    combined = dict(coordinateDomainSoftwareDeg=full["coordinateDomainSoftwareDeg"],
                    coefficients=full["coefficients"], bands=folded["bands"]+full["bands"])
    if prior_comparison:
        old = prior_comparison["combinedRule"]
        assert old["coordinateDomainSoftwareDeg"] == full["coordinateDomainSoftwareDeg"]
        assert old["coefficients"] == full["coefficients"]
        combined["bands"] = old["bands"]+[b for b in full["bands"] if b not in old["bands"]]
        assert all(b in combined["bands"] for b in old["bands"])
    report = read(Path(full["sourceReport"]))
    recorded = np.asarray(report["recordedSoftwareRangesDeg"])
    points = np.random.default_rng(report["coverageRandomSeed"]).uniform(
        recorded[:, 0], recorded[:, 1], (report["coverageSamples"], 5))
    before, after = compact_membership(old, points), compact_membership(combined, points)
    table = factored_membership(report["groups"], points)
    domain = np.asarray(report["admissibleSoftwareDomainDeg"])
    table &= np.all((points >= domain[:, 0]) & (points <= domain[:, 1]), axis=1)
    assert not np.any(before & ~after)
    assert not np.any(after & ~table)
    assert int(table.sum()) == report["coverageSamplesAccepted"]
    identity = dict(prior_identity)
    identity.update({str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest() for p in
                     (source / "compact_rule.json", folded_source / "compact_rule.json", Path(full["sourceReport"]))})
    result = dict(combinedRule=combined, priorUniformStripPreserved=True,
                  priorFoldedBandsRetainedUnchanged=True, fullWristJ5CoverageDeg=full_intervals,
                  comparisonBaseline="previous_compact_union" if prior_comparison else "folded_plus_uniform",
                  coverageRandomSeed=report["coverageRandomSeed"], coverageSamples=len(points),
                  previousReadableAccepted=int(before.sum()), expandedReadableAccepted=int(after.sum()),
                  newlyAccepted=int((after & ~before).sum()), fullTableAccepted=int(table.sum()),
                  samplesOutsideFullTable=int((after & ~table).sum()), inputFileSha256=identity,
                  implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  limitations=["Sample counts are joint-coordinate estimates, not certified Cartesian volume",
                               "Exact set growth follows unchanged prior bands, or retained folded bands plus coverage of the old uniform strip",
                               "Independent dynamics, FCL and actual projection audits remain required"])
    write(source / "compact_rule_comparison.json", result)
    print({k: v for k, v in result.items() if k not in ("combinedRule", "inputFileSha256", "limitations")}, flush=True)


def self_test():
    # A monotone artificial geometry whose moving body crosses an obstacle.
    class Dummy:
        constraints = {"pair": dict(radius=np.ones(5)*.1)}
        def place(self, q):
            pass
        def lower_distance(self, name, q, needed):
            return abs(q[1])*.001-.0001
    checker = Dummy()
    bad = check_curve(checker, lambda t: np.array([0, -1+2*t, 0, 0, 0]))
    assert bad["status"] == "not_proven" and bad["failures"]
    interior = check_curve(checker, lambda t: np.array([0, -1+3*t, 0, 0, 0]))
    assert interior["status"] == "not_proven" and interior["failures"]
    budget = check_curve(checker, lambda t: np.array([0, -1+3*t, 0, 0, 0]), max_nodes=1)
    assert budget["status"] == "not_proven"
    assert budget["failures"][0]["reason"] == "resource_or_depth_limit"
    good = check_curve(checker, lambda t: np.array([0, 100+t, 0, 0, 0]))
    assert good["status"] == "numerical_motion_bound_pass" and good["coveredParameterFraction"] == 1
    radius = np.array([.3, .2, 0, 0, 0])
    assert motion_bound(radius, np.zeros(5)) == 0
    assert abs(motion_bound(radius, np.array([4, 4, 0, 0, 0]))-1) < 1e-12
    assert equivalent_angles(160, -226, 226) == [-200, 160]
    assert equivalent_angles(-160, -226, 226) == [-160, 200]
    assert equivalent_angles(300, -20, 20) == []
    assert equivalent_angles(180, -180, 180) == [-180, 180]
    domain = np.array([[0., 0.], [100., 101.], [0., 0.], [0., 0.], [0., 0.]])
    region = partition_regions(checker, domain, ["pair"], 20)
    assert region["verifiedFraction"] == 1 and not region["unknown"]
    domain[1] = [-1, 2]
    region = partition_regions(checker, domain, ["pair"], 20)
    assert region["unknown"] and region["verifiedFraction"] < 1
    boxes = [np.array([[0., 0.], [0., 1.], [0., 1.], [0., 1.], [-2., -1.]]).tolist()]
    assert region_intervals(boxes, [0, .5, .5, .5, 0]) == [[-2., -1.]]
    assert not region_intervals(boxes, [0, 2, .5, .5, 0])
    assert region_intervals(boxes, [0, .5, .5, 100, -1.5], 3) == [[0., 1.]]
    assert not region_intervals(boxes, [0, .5, .5, 100, 0], 3)
    assert region_intervals(boxes, [0, .5, 100, .5, -1.5], 2) == [[0., 1.]]
    assert region_intervals(boxes, [0, 100, .5, .5, -1.5], 1) == [[0., 1.]]
    assert not region_intervals([], [0, .5, .5, .5, -1.5], 2)
    separated = np.array(boxes[0])
    separated[2] = [2., 3.]
    assert region_intervals([*boxes, separated.tolist()], [0, .5, 100, .5, -1.5], 2) == [
        [0., 1.], [2., 3.]]
    assert segment_in_regions(boxes, [0, 0, 0, 0, -2], [0, 1, 1, 1, -1])
    assert not segment_in_regions(boxes, [0, 0, 0, 0, -2], [0, 1, 1, 1, 0])
    second = np.array(boxes[0])
    second[1] = [2, 3]
    assert not segment_in_regions([*boxes, second.tolist()],
                                  [0, .5, .5, .5, -1.5], [0, 2.5, .5, .5, -1.5])
    second[1] = [1, 2]
    assert segment_in_regions([*boxes, second.tolist()],
                              [0, .5, .5, .5, -1.5], [0, 1.5, .5, .5, -1.5])
    assert merge_intervals([[1, 2], [0, 1], [4, 5]]) == [[0, 2], [4, 5]]
    assert intersect_intervals([[-3, -1], [1, 3]], [[-2, 2]]) == [[-2, -1], [1, 2]]
    compact = compact_boxes([*boxes, second.tolist()])
    assert len(compact) == 1 and compact[0][1] == [0, 2]
    hole = np.array(second)
    hole[1] = [2, 3]
    assert len(compact_boxes([*boxes, hole.tolist()])) == 2
    example = dict(admissibleSoftwareDomainDeg=[[0, 0], [0, 2], [0, 1], [0, 1], [-2, 2]],
                   groups=[dict(verifiedSoftwareBoxesDeg=compact)])
    assert factored_intervals(example, [0, .5, .5, .5, 0]) == [[-2, -1]]
    assert not factored_intervals(example, [0, 3, .5, .5, 0])
    assert factored_intervals(example, [0, .5, .5, 100, -1.5], 3) == [[0., 1.]]
    assert factored_intervals(example, [0, .5, 100, .5, -1.5], 2) == [[0., 1.]]
    assert not factored_intervals(example, [0, 3, 100, .5, -1.5], 2)
    assert factored_intervals(example, [0, 100, .5, .5, -1.5], 1) == [[0., 2.]]
    assert factored_membership(example["groups"], [[0, .5, .5, .5, -1.5],
                                                  [0, .5, .5, .5, 0]]).tolist() == [True, False]
    rng = np.random.default_rng(2026111703)
    for _ in range(100):
        bodies, objects = [], []
        for i in range(2):
            rotation, _ = np.linalg.qr(rng.normal(size=(3, 3)))
            rotation[:, 0] *= np.linalg.det(rotation)
            matrix = np.eye(4)
            matrix[:3, :3] = rotation
            center, half = rng.uniform(-2, 2, 3), rng.uniform(.05, .5, 3)
            bodies.append(dict(matrix=matrix, centerWorld=center, half=half))
            objects.append(fcl.CollisionObject(fcl.Box(*(2*half)), fcl.Transform(rotation, center)))
        lower = obb_gap(*bodies)
        distance = fcl.distance(*objects, fcl.DistanceRequest(), fcl.DistanceResult())
        assert lower <= (max(0., distance)+1e-8)
    timed = partition_regions(checker, domain, ["pair"], 100, max_seconds=0)
    assert not timed["verified"] and timed["unknownFraction"] == 1
    domain[1] = [100, 101]
    seed = domain.copy()
    seed[1, 1] = 100.5
    resumed = partition_regions(checker, domain, ["pair"], 100,
                                seeds=[dict(cadBoundsDeg=seed.tolist(), unresolved=["pair"])])
    assert resumed["verifiedFraction"] == .5 and not resumed["unknown"]
    domain[1] = [-1, 101]
    large, small = domain.copy(), domain.copy()
    large[1], small[1] = [-1, 99], [100, 101]
    seeds = [dict(cadBoundsDeg=box.tolist(), unresolved=["pair"]) for box in (large, small)]
    focused = partition_regions(checker, domain, ["pair"], 1, seeds=seeds,
                                focus=[[0, 100.5, 0, 0, 0]])
    assert abs(focused["verifiedFraction"]-1/102) < 1e-12
    assert focused["unknown"], "A focus result must not erase unprocessed regions"
    square = np.array([[0., 0.], [0., 4.], [0., 4.], [0., 0.], [0., 0.]])
    window = square.copy()
    window[1:3] = [[1., 3.], [1., 3.]]
    seed = dict(cadBoundsDeg=square.tolist(), unresolved=["pair"], centerWitnesses=["parent"])
    selected, deferred = focus_seeds([seed], [window, window])
    assert len(selected) == 1 and box_measure(selected[0]["cadBoundsDeg"]) == 4
    assert sum(box_measure(r["cadBoundsDeg"]) for r in selected+deferred) == 16
    assert all("centerWitnesses" not in r for r in selected+deferred)
    for i, row in enumerate(selected+deferred):
        for other in (selected+deferred)[i+1:]:
            a, b = np.array(row["cadBoundsDeg"]), np.array(other["cadBoundsDeg"])
            assert np.any(np.minimum(a[1:3, 1], b[1:3, 1])
                          <= np.maximum(a[1:3, 0], b[1:3, 0]))
    outside = window.copy()
    outside[1] = [4, 5]
    selected, deferred = focus_seeds([seed], [outside])
    assert not selected and deferred == [seed]
    selected, deferred = focus_seeds([seed], [window])
    resumed = partition_regions(checker, square, ["pair"], 1, max_seconds=0, seeds=selected)
    assert resumed["unknownFraction"]+sum(box_measure(r["cadBoundsDeg"]) for r in deferred)/16 == 1
    square[1:3] = [[0., 2.], [0., 2.]]
    first, second = square.copy(), square.copy()
    first[1], second[2] = [1., 2.], [1., 2.]
    coefficients = [0., 1., 1., 0., 0.]
    assert halfspace_in_regions([first, second], square, coefficients, 2.1)["proven"]
    assert not halfspace_in_regions([first, second], square, coefficients, 1.5)["proven"]
    assert halfspace_in_regions([square], square, coefficients, -10.)["proven"]
    assert not halfspace_in_regions([], square, [0., -1., 1., 0., 0.], 1.)["proven"]
    middle = square.copy()
    middle[1] = [.5, 1.5]
    assert halfspace_in_regions([middle], square, [0., 1., 0., 0., 0.], .6, 1.4)["proven"]
    unrelated = middle.copy()
    unrelated[1] = [10., 11.]
    assert (halfspace_in_regions([unrelated, middle], square, [0., 1., 0., 0., 0.], .6, 1.4)
            == halfspace_in_regions([middle], square, [0., 1., 0., 0., 0.], .6, 1.4))
    assert not halfspace_in_regions([middle], square, [0., 1., 0., 0., 0.], .4, 1.4)["proven"]
    assert not halfspace_in_regions([middle], square, [0., 1., 0., 0., 0.], .6)["proven"]
    left, right = square.copy(), square.copy()
    left[1], right[1] = [0., .75], [.75, 1.5]
    assert halfspace_in_regions([left]*64+[right], square, [0., 1., 0., 0., 0.], .6, 1.4)["proven"]
    for _ in range(40):
        windows = []
        for _ in range(5):
            window = square.copy()
            window[1:3] = np.sort(rng.uniform(0., 2., (2, 2)), axis=1)
            windows.append(window)
        coefficients = np.r_[0., rng.uniform(-1., 1., 2), 0., 0.]
        lower, upper = sorted(rng.uniform(-3., 3., 2))
        _, reference = focus_seeds([dict(cadBoundsDeg=square.tolist(), unresolved=[])], windows)
        disjoint = []
        for row in reference:
            box = np.asarray(row["cadBoundsDeg"])
            high = float(coefficients @ np.where(coefficients >= 0, box[:, 1], box[:, 0]))
            low = float(coefficients @ np.where(coefficients >= 0, box[:, 0], box[:, 1]))
            disjoint.append(high < lower or low > upper)
        assert halfspace_in_regions(windows, square, coefficients, lower, upper)["proven"] == all(disjoint)
    polygon = shoulder_polygon(square[1:3], 1., 3.)
    assert len(polygon) >= 3 and np.all(polygon >= 0) and np.all(polygon <= 2)
    assert np.all(polygon @ [1.5, 1.] >= 1-1e-12) and np.all(polygon @ [1.5, 1.] <= 3+1e-12)
    assert not len(shoulder_polygon(square[1:3], 6., 7.))
    for weight in (.5, -1.):
        polygon = shoulder_polygon(square[1:3], .2, .8, weight)
        assert len(polygon) >= 3 and np.all(polygon >= 0) and np.all(polygon <= 2)
        assert np.all(polygon @ [weight, 1.] >= .2-1e-12)
        assert np.all(polygon @ [weight, 1.] <= .8+1e-12)
    rule = dict(coordinateDomainSoftwareDeg=[[0, 0], [-36, 46], [-98, 105], [-227, 227], [-23, 23]],
                bands=[dict(J5Deg=[18, 19.9], J4RadiusDeg=15, weightedLowerDeg=-138.5),
                       dict(J5Deg=[19.9, 21.5], J4RadiusDeg=30, weightedLowerDeg=-137.1)])
    assert compact_membership(rule, [[0, -35, -85, -190, 19.9],
                                     [0, -35, -85, -205, 19.9],
                                     [0, -35, -85, -190, 20],
                                     [0, 0, 0, -205, 20],
                                     [0, 0, 0, 0, 20],
                                     [.01, 0, 0, -205, 20]]).tolist() == [True, False, False, True, False, False]
    rule["bands"][0]["weightedUpperDeg"] = 0.
    assert not compact_membership(rule, [[0, 1, 0, -190, 18.5]])[0]
    rule["bands"] = [dict(J5Deg=[-16, 22], J4RadiusDeg=180,
                          weightedLowerDeg=-3, weightedUpperDeg=41)]
    assert compact_membership(rule, [[0, 0, 10, q4, 0] for q4 in [-227, 0, 227]]).all()
    assert merge_intervals([[-360, 0], [0, 360]]) == [[-360, 360]]
    rule["bands"] = [dict(J5Deg=[-16, 0], J4RadiusDeg=180, weightedLowerDeg=-122.7, weightedUpperDeg=112.6),
                     dict(J5Deg=[0, 10], J4RadiusDeg=180, weightedLowerDeg=-113.5, weightedUpperDeg=47.6),
                     dict(J5Deg=[10, 21.5], J4RadiusDeg=180, weightedLowerDeg=-15.6, weightedUpperDeg=42.9),
                     dict(J5Deg=[21.5, 22.632856], J4RadiusDeg=180, weightedLowerDeg=-3, weightedUpperDeg=42.9)]
    assert compact_membership(rule, [[0, 0, 80, 0, -1], [0, 0, 80, 0, 0],
                                     [0, 0, 80, 0, 1], [0, -30, -75, 0, -1],
                                     [0, -30, -75, 0, 1], [0, 0, -10, 0, 20],
                                     [0, 0, -10, 0, 22], [0, 0, 10, 228, -1]]).tolist() == [
                                         True, True, False, True, False, True, False, False]
    full_bands = list(rule["bands"])
    rule["bands"] += [dict(b, J4RadiusDeg=120, weightedUpperDeg=120.3) for b in full_bands]
    assert compact_membership(rule, [[0, 10, 100, q4, -1] for q4 in
                                     [-227, -100, -60, -59.999, 0, 59.999, 60, 100, 227]]).tolist() == [
                                         True, True, True, False, False, False, True, True, True]
    assert compact_membership(rule, [[0, 20, 90.3, -100, 1],
                                     [0, 20, 90.4, -100, 1]]).tolist() == [True, False]
    assert compact_membership(rule, [[0, 0, 10, q4, q5]
                                     for q4 in [-227, 0, 227] for q5 in [-16, 0, 10, 21.5, 22]]).all()
    wrist = np.r_[np.linspace(-227, 227, 1001), -60., 60.]
    assert np.array_equal(abs(abs(wrist)-180) <= 120, abs(wrist) >= 60)
    previous_rule = dict(rule, bands=list(rule["bands"]))
    rule["bands"] = [dict(b, J5Deg=[-16.6, b["J5Deg"][1]]) if b["J5Deg"][0] == -16 else b
                     for b in rule["bands"]]
    negative_points = [[0, 0, 10, 0, q5] for q5 in [-16.600001, -16.6, -16.55, -16.05, -16, -15.5]]
    assert compact_membership(rule, negative_points).tolist() == [False, True, True, True, True, True]
    assert compact_membership(previous_rule, negative_points).tolist() == [False, False, False, False, True, True]
    assert compact_membership(rule, [[0, 10, 100, q4, -16.55] for q4 in [0, 59.999, 60, -60]]).tolist() == [
        False, False, True, True]
    inner_band = dict(J5Deg=[-16.6, 22.632856], J4CenterAbsDeg=0, J4RadiusDeg=140,
                      weightedLowerDeg=-122.7, weightedUpperDeg=42.9)
    assert compact_wrist_intervals(inner_band) == [[-140, 140]]
    assert compact_wrist_intervals(dict(J4RadiusDeg=30)) == [[-210, -150], [150, 210]]
    assert compact_wrist_intervals(dict(J4RadiusDeg=180)) == [[-360, 360]]
    rule["bands"] = [inner_band]
    assert compact_membership(rule, [[0, -25, -82.5, q4, 12] for q4 in
                                     [-140.000001, -140, 0, 140, 140.000001]]).tolist() == [
                                         False, True, True, True, False]
    assert compact_membership(rule, [[0, 0, s, 0, 12] for s in [-97, 42.9, 42.900001]]).tolist() == [
        True, True, False]
    for radius in (15, 30, 38, 120, 180):
        for tolerance in (0., 1e-10):
            legacy = dict(J4RadiusDeg=radius)
            wrist = np.r_[np.linspace(-360, 360, 1001), -180-radius, -180+radius, 180-radius, 180+radius]
            allowed = np.zeros(len(wrist), dtype=bool)
            for low, high in compact_wrist_intervals(legacy):
                allowed |= (wrist >= low-tolerance) & (wrist <= high+tolerance)
            assert np.array_equal(allowed, abs(abs(wrist)-180) <= radius+tolerance)
    affine_band = dict(J5Deg=[0, 10], J4CenterAbsDeg=0, J4RadiusDeg=60, J2Weight=.5,
                       weightedLowerDeg=-10, weightedUpperDeg=10)
    rule["bands"] = [affine_band]
    assert compact_membership(rule, [[0, 20, 0, 60, 0], [0, 20, .000001, 60, 0],
                                     [0, -20, 0, -60, 10], [0, -20, -.000001, -60, 10],
                                     [0, 20, 0, 60.000001, 0], [0, 20, 0, 0, -1]]).tolist() == [
                                         True, False, True, False, False, False]
    assert compact_membership(dict(rule, bands=[inner_band, affine_band]),
                              [[0, 40, 0, 0, 5], [0, -30, 15, 0, 5]]).tolist() == [False, True]
    outer_band = dict(J5Deg=[10, 18], J4CenterAbsDeg=360, J4RadiusDeg=180,
                      weightedLowerDeg=-113.5, weightedUpperDeg=142.4)
    assert compact_membership(dict(rule, bands=[outer_band]),
                              [[0, 40, 65, wrist, 12] for wrist in
                               [180, -180, 227, -227, 179.999999, -179.999999, 0]]).tolist() == [
                                   True, True, True, True, False, False, False]
    for invalid_weight in (math.inf, math.nan):
        try:
            compact_membership(dict(rule, bands=[dict(affine_band, J2Weight=invalid_weight)]),
                               [[0, 0, 0, 0, 5]])
            raise AssertionError("Invalid shoulder weight accepted")
        except ValueError:
            pass
    print("PASS: motion/box bounds, interior rejection, budgets, interval union and complete segment coverage")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("sources", "verify", "j5-alignment", "alignment-hypothesis", "radius-audit", "adjacent-domains", "intervals", "branch-switch", "equivalent-turns", "sum-counterexample", "regions", "regions-query", "factored-regions", "factored-query", "piecewise-transitions", "compact-rule", "extended-rule", "universal-rule", "piecewise-full-rule", "middle-rule", "negative-rule", "inner-rule", "affine-rule", "outer-rule", "compact-audit", "compare-rules", "self-test"))
    parser.add_argument("--source", type=Path, default=Path(__file__).resolve().parents[2] / "build/coupled_workspace_20261116_v1")
    parser.add_argument("--out", type=Path)
    parser.add_argument("--max-nodes", type=int, default=30000)
    parser.add_argument("--max-seconds", type=float, default=90, help="Per-group time budget for factored-regions")
    parser.add_argument("--diagnostic", type=Path)
    parser.add_argument("--software", nargs=5, type=float)
    parser.add_argument("--scan-joint", type=int, choices=(4, 5), default=5)
    parser.add_argument("--focus-software", nargs=5, type=float, action="append",
                        help="Prioritize saved cells containing this software pose; repeatable")
    parser.add_argument("--focus-software-box", nargs=8, type=float, action="append",
                        help="Refine only these J2..J5 low/high pairs; retain all other cells; repeatable")
    args = parser.parse_args()
    if args.action == "self-test":
        self_test()
    elif args.action == "sources":
        assert args.out
        sources(args.out)
    elif args.action == "j5-alignment":
        assert args.out
        j5_alignment(args.source, args.out)
    elif args.action == "alignment-hypothesis":
        assert args.out and args.diagnostic
        alignment_hypothesis(args.source, args.diagnostic, args.out)
    elif args.action == "radius-audit":
        assert args.out
        radius_audit(args.source, args.out)
    elif args.action == "adjacent-domains":
        assert args.out
        adjacent_domains(args.source, args.out, args.max_nodes)
    elif args.action == "intervals":
        assert args.out
        intervals(args.source, args.out, args.max_nodes, args.software, args.scan_joint-1)
    elif args.action == "branch-switch":
        assert args.out
        branch_switch(args.source, args.out, args.max_nodes)
    elif args.action == "equivalent-turns":
        assert args.out
        equivalent_turns(args.source, args.out, args.max_nodes)
    elif args.action == "sum-counterexample":
        assert args.out
        sum_counterexample(args.source, args.out)
    elif args.action == "regions":
        assert args.out
        regions(args.source, args.out, args.max_nodes)
    elif args.action == "regions-query":
        assert args.software
        regions_query(args.source, args.software)
    elif args.action == "factored-regions":
        assert args.out
        factored_regions(args.source, args.out, args.max_nodes, args.max_seconds,
                         args.focus_software, args.focus_software_box)
    elif args.action == "piecewise-transitions":
        assert args.out
        piecewise_transitions(args.source, args.out)
    elif args.action in ("compact-rule", "extended-rule", "universal-rule", "piecewise-full-rule", "middle-rule", "negative-rule", "inner-rule", "affine-rule", "outer-rule"):
        assert args.out
        assert args.action not in ("middle-rule", "negative-rule", "inner-rule", "affine-rule", "outer-rule") or args.diagnostic
        compact_rule(args.source, args.out, args.action, args.diagnostic)
    elif args.action == "compact-audit":
        compact_audit(args.source)
    elif args.action == "compare-rules":
        assert args.diagnostic
        compare_rules(args.source, args.diagnostic)
    elif args.action == "factored-query":
        assert args.software
        report = read(args.source / "factored_regions.json")
        print(dict(softwareDegrees=args.software,
                   scannedJoint=args.scan_joint,
                   **{f"verifiedJ{args.scan_joint}IntervalsDeg":
                      factored_intervals(report, args.software, args.scan_joint-1)},
                   mapping=report["mapping"], status="Offline numerical model only"), flush=True)
    else:
        assert args.out
        verify(args.source, args.out, args.max_nodes)
