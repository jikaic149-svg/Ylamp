"""Offline full-five-axis tracking tubes; no controller, scene or hardware changes."""
import argparse
import hashlib
from pathlib import Path
import time

import numpy as np

from workspace_geometry import read, write
from workspace_verify import BoundsChecker, check_curve, motion_bound


class TubeChecker:
    """Subtract a uniform-in-time angular-error displacement bound from each distance."""
    def __init__(self, checker, epsilon):
        epsilon = np.asarray(epsilon, dtype=float)
        if epsilon.shape != (5,) or not np.isfinite(epsilon).all() or np.any(epsilon < 0):
            raise ValueError("Expected five finite nonnegative angular errors in degrees")
        self.checker, self.constraints = checker, checker.constraints
        self.extra = {name: motion_bound(row["radius"], np.radians(epsilon))
                      for name, row in self.constraints.items()}

    def place(self, q):
        self.checker.place(q)

    def lower_distance(self, name, q, needed):
        extra = self.extra[name]
        return self.checker.lower_distance(name, q, needed+extra)-extra


def segment_membership(points, a, b, epsilon):
    """A witness on the closed segment for every accepted point; all five axes retained."""
    points, a, b, epsilon = map(np.asarray, (points, a, b, epsilon))
    if (points.ndim != 2 or points.shape[1] != 5 or
            any(x.shape != (5,) for x in (a, b, epsilon)) or
            not all(np.isfinite(x).all() for x in (points, a, b, epsilon)) or
            np.any(epsilon < 0)):
        raise ValueError("Expected finite five-axis points, segment and nonnegative errors")
    low, high, valid = np.zeros(len(points)), np.ones(len(points)), np.ones(len(points), bool)
    delta = b-a
    for j in range(5):
        if delta[j] == 0:
            valid &= abs(points[:, j]-a[j]) <= epsilon[j]
        else:
            x, y = (points[:, j]-a[j]-epsilon[j])/delta[j], (points[:, j]-a[j]+epsilon[j])/delta[j]
            low, high = np.maximum(low, np.minimum(x, y)), np.minimum(high, np.maximum(x, y))
    phase = (low+high)/2
    return valid & (low <= high) & np.all(abs(points-(a+phase[:, None]*delta)) <= epsilon, axis=1)


def self_test():
    class Fake:
        constraints = {"pair": dict(radius=np.asarray([1., 2., 0, 0, 0]))}
        distance = .01
        def place(self, q):
            pass
        def lower_distance(self, name, q, needed):
            self.needed = needed
            return self.distance
    core = Fake()
    tube = TubeChecker(core, [.01, .02, 0, 0, 0])
    extra = motion_bound(core.constraints["pair"]["radius"], np.radians([.01, .02, 0, 0, 0]))
    assert tube.lower_distance("pair", np.zeros(5), .001) == .01-extra
    assert core.needed == .001+extra
    assert check_curve(tube, lambda t: np.zeros(5))["status"] == "numerical_motion_bound_pass"
    core.distance = extra/2
    assert check_curve(tube, lambda t: np.zeros(5))["status"] == "not_proven"
    for invalid in ([0]*4, [0, 0, -1, 0, 0], [0, 0, np.nan, 0, 0]):
        try:
            TubeChecker(core, invalid)
        except ValueError:
            pass
        else:
            raise AssertionError("Invalid error tube admitted")
    a, b = np.zeros(5), np.array([1., 1., 0, 0, 0])
    points = np.array([[.5, .55, 0, 0, 0], [.5, .8, 0, 0, 0], [-.05, -.05, 0, 0, 0]])
    assert segment_membership(points, a, b, np.full(5, .1)).tolist() == [True, False, True]
    points = np.array([[.5, 0, 0, 0, 0], [1., .5, 0, 0, 0]])
    corner, end = np.array([1., 0, 0, 0, 0]), np.array([1., 1., 0, 0, 0])
    members = np.column_stack([segment_membership(points, x, y, np.full(5, .01))
                               for x, y in ((a, corner), (corner, end))])
    assert members.any(axis=1).all() and not (members[0] & members[1]).any()
    print("tracking-tube self-test passed", flush=True)


def study(source, out, max_nodes):
    started = time.perf_counter()
    nominal = read(source / "nominal_paths.json")
    routes = nominal["selectedRoutes"]
    ids = {row["id"] for row in routes}
    identities = dict(nominal["inputFileSha256"])
    configs, maxima = [], []
    for mapping in ("baseline", "alignment"):
        own = source / mapping
        configs.append(read(own / "config.json"))
        dynamics = read(own / "dynamics.json")
        maxima.extend(c["maxErrorDeg"] for c in dynamics["cases"] if c["id"] in ids)
        for filename in ("config.json", "dynamics.json", "actual_trace.json", "geometry.json"):
            path = own / filename
            identities[str(path.resolve())] = hashlib.sha256(path.read_bytes()).hexdigest()
    assert configs[0]["geometryDirectory"] == configs[1]["geometryDirectory"]
    observed = np.max(np.asarray(maxima), axis=0)
    epsilon = np.maximum(2*observed, 1e-4)
    for path in (source / "nominal_paths.json", Path(__file__),
                 Path(__file__).with_name("workspace_verify.py"),
                 Path(__file__).with_name("workspace_geometry.py")):
        identities[str(path.resolve())] = hashlib.sha256(path.read_bytes()).hexdigest()
    for name, digest in identities.items():
        with Path(name).open("rb") as stream:
            assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, name
    checker = TubeChecker(BoundsChecker(source / "baseline"), epsilon)
    assert len(checker.constraints) == 89
    print("Full-89-constraint angular tube:", epsilon.tolist(), flush=True)
    proofs, actual = [], []
    for mapping, config in zip(("baseline", "alignment"), configs):
        offset = np.asarray(config["offsetCadDeg"])
        for row in routes:
            route = np.asarray(row["routeSoftwareDeg"])+offset
            for segment, (a, b) in enumerate(zip(route, route[1:])):
                result = check_curve(checker, lambda t: a+t*(b-a), max_nodes=max_nodes)
                proofs.append(dict(mapping=mapping, route=row["id"], segment=segment, result=result))
                print(mapping, row["id"], segment, result["status"], result["nodes"], flush=True)
        samples = read(source / mapping / "actual_trace.json")["samples"]
        for row in routes:
            route = np.asarray(row["routeSoftwareDeg"])+offset
            for speed in ("slow", "brisk"):
                label = speed+"/"+row["id"]
                points = np.asarray([s["cadDegrees"] for s in samples if s["case"] == label])
                assert len(points) > 1
                members = np.column_stack([segment_membership(points, a, b, epsilon)
                                           for a, b in zip(route, route[1:])])
                shared = np.any(members[:-1] & members[1:], axis=1)
                actual.append(dict(mapping=mapping, case=label, points=len(points),
                    allPointsInTube=bool(members.any(axis=1).all()),
                    allEdgesInOneConvexSegmentTube=bool(shared.all()),
                    unresolvedEdges=np.flatnonzero(~shared).tolist()))
    passed = all(p["result"]["status"] == "numerical_motion_bound_pass" for p in proofs)
    edges = all(row["allPointsInTube"] and row["allEdgesInOneConvexSegmentTube"] for row in actual)
    for name, digest in identities.items():
        with Path(name).open("rb") as stream:
            assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, name
    out.mkdir(parents=True, exist_ok=False)
    write(out / "tracking_tube.json", dict(source=str(source.resolve()), constraints=89,
        observedMaximumNewRouteErrorDeg=observed.tolist(), angularTubeDeg=epsilon.tolist(),
        tubeRule="epsilon_i = max(2*observed_max_error_i, 0.0001 deg); conditional verification assumption",
        continuousNominalTubeProven=passed, savedActualPolylineCovered=passed and edges,
        proofs=proofs, actualPolylineChecks=actual, inputFileSha256=identities,
        elapsedSeconds=time.perf_counter()-started,
        limitations=["Numerical triangle-surface model bounds; not interval arithmetic or solid containment.",
                     "No guarantee that the finite observed error bounds unlogged continuous or hardware motion.",
                     "Only the six named routes and saved actual piecewise-linear traces, not the full workspace.",
                     "Original calibration, load/friction, physical clearance, cables and braking still unverified."]))
    print("Tube complete:", passed, "saved actual polyline:", passed and edges, flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("self-test", "study"))
    parser.add_argument("--source", type=Path)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--max-nodes", type=int, default=2000)
    args = parser.parse_args()
    if args.action == "self-test":
        self_test()
    else:
        assert args.source is not None and args.out is not None and args.max_nodes > 0
        study(args.source, args.out, args.max_nodes)
