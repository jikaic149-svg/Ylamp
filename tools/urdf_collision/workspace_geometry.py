"""Offline native triangle-surface checks. No serial/CAN or hardware access."""
import argparse
from collections import Counter, deque
from datetime import datetime, timezone
import itertools
import json
import math
from pathlib import Path
import time

import fcl
import numpy as np


def read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def write(path, data):
    with path.open("x", encoding="utf-8") as stream:
        json.dump(data, stream, indent=2, allow_nan=False)
        stream.write("\n")


def line(a, b, step):
    a, b = np.asarray(a), np.asarray(b)
    n = max(1, math.ceil(float(abs(b-a).max())/step))
    return (a+(b-a)*k/n for k in range(n+1))


class Checker:
    def __init__(self, out):
        source = Path(read(out / "config.json").get("geometryDirectory", str(out)))
        manifest = read(source / "fcl_meshes/manifest.json")
        self.input = read(source / "mesh_input.json")
        self.origins = [np.array(a).reshape(4, 4, order="F") for a in manifest["origins"]]
        assert manifest["axes"] == [[0, -1, 0]]*5
        models, boxes = {}, {}
        names = {b["mesh"] for b in manifest["bodies"]}
        names.update(p[k] for p in manifest["pairs"] for k in ("aMesh", "bMesh"))
        for name in sorted(names):
            vertices = np.fromfile(source / f"fcl_meshes/{name}.vertices", dtype="<f4").reshape(-1, 3).astype(float)
            faces = np.fromfile(source / f"fcl_meshes/{name}.indices", dtype="<u4").reshape(-1, 3).astype(np.int32)
            model = fcl.BVHModel()
            model.beginModel(len(faces), len(vertices))
            model.addSubModel(vertices, faces)
            model.endModel()
            models[name] = model
            lo, hi = vertices.min(axis=0), vertices.max(axis=0)
            boxes[name] = ((lo+hi)/2, (hi-lo)/2)
        self.bodies = {b["name"]: dict(b, local=np.array(b["local"]).reshape(4, 4, order="F"),
                                      center=boxes[b["mesh"]][0], half=boxes[b["mesh"]][1],
                                      object=fcl.CollisionObject(models[b["mesh"]]))
                       for b in manifest["bodies"]}
        self.pairs = []
        for p in manifest["pairs"]:
            a, b = self.bodies[p["a"]], self.bodies[p["b"]]
            self.pairs.append(dict(name=p["name"], a=a, b=b, cache={},
                low=min(a["linkIndex"], b["linkIndex"]), high=max(a["linkIndex"], b["linkIndex"]),
                objA=fcl.CollisionObject(models[p["aMesh"]]), objB=fcl.CollisionObject(models[p["bMesh"]])))
        self.request = fcl.CollisionRequest(num_max_contacts=1, enable_contact=False)
        # MuJoCo world z = .0587451 - URDF y; forbidden URDF y > .0587451.
        self.floor = fcl.CollisionObject(fcl.Halfspace(np.array([0., -1., 0.]), -.05874511))
        self.cache, self.counts, self.witnesses = {}, Counter(), []
        self.checks = 0

    def overlap(self, a, b):
        return fcl.collide(a, b, self.request, fcl.CollisionResult()) > 0

    def hits(self, q, floor=True):
        q = tuple(round(float(v), 8) for v in q)
        assert len(q) == 5 and all(math.isfinite(v) for v in q)
        if floor and q in self.cache:
            return self.cache[q]
        self.place(q)
        found = []
        for p in self.pairs:
            key = q[p["low"]:p["high"]]
            hit = p["cache"].get(key)
            if hit is None:
                a, b = p["a"], p["b"]
                hit = False
                if np.all(abs(a["centerWorld"]-b["centerWorld"]) <= a["halfWorld"]+b["halfWorld"]):
                    p["objA"].setTransform(a["transform"])
                    p["objB"].setTransform(b["transform"])
                    hit = self.overlap(p["objA"], p["objB"])
                p["cache"][key] = hit
            if hit:
                found.append(p["name"])
        if floor:
            for b in self.bodies.values():
                if not b["linkIndex"] or b["centerWorld"][1]+b["halfWorld"][1] <= .05874511:
                    continue
                b["object"].setTransform(b["transform"])
                if self.overlap(b["object"], self.floor):
                    found.append(b["name"]+"/floor")
        self.checks += 1
        if floor:
            self.cache[q] = found
            self.counts.update(found)
            if found and len(self.witnesses) < 50 and not any(w["pairs"] == found for w in self.witnesses):
                self.witnesses.append(dict(cadDegrees=q, pairs=found))
        return found

    def place(self, q):
        """Update transforms even if a collision/distance result was cached."""
        q = np.asarray(q, dtype=float)
        if q.shape != (5,) or not np.isfinite(q).all():
            raise ValueError("Expected five finite CAD angles in degrees")
        frames = [np.eye(4)]
        for a, origin in zip(q, self.origins):
            s, c = math.sin(math.radians(-a)), math.cos(math.radians(-a))
            rotation = np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])
            frames.append(frames[-1] @ origin @ rotation)
        self.frames = frames
        for b in self.bodies.values():
            m = frames[b["linkIndex"]] @ b["local"]
            b["matrix"] = m
            b["transform"] = fcl.Transform(m[:3, :3], m[:3, 3])
            b["centerWorld"] = m[:3, :3] @ b["center"]+m[:3, 3]
            b["halfWorld"] = abs(m[:3, :3]) @ b["half"]

    def clear_line(self, a, b, step):
        return all(not self.hits(q) for q in line(a, b, step))


def run(out, trace=False, regression_only=False):
    start = time.perf_counter()
    config = read(out / "config.json")
    checker = Checker(out)
    print(f"Native FCL {fcl.__version__}: meshes loaded in {time.perf_counter()-start:.2f}s", flush=True)
    if trace:
        samples = read(out / "actual_trace.json")["samples"]
        cases, violations = {}, []
        for k, sample in enumerate(samples):
            stat = cases.setdefault(sample["case"], dict(checked=0, colliding=0, pairs=Counter()))
            found = checker.hits(sample["cadDegrees"])
            stat["checked"] += 1
            if found:
                stat["colliding"] += 1
                stat["pairs"].update(found)
                if len(violations) < 30:
                    violations.append(dict(sample, pairs=found))
            if k % 1000 == 0:
                print(f"Actual trace {k}/{len(samples)}", flush=True)
        write(out / "trajectory_geometry.json", dict(samples=len(samples), cases=cases, violations=violations,
            input=checker.input, engine=f"python-fcl {fcl.__version__}",
            elapsedSeconds=time.perf_counter()-start,
            note="Finite triangle-surface states/interpolants; no continuous/clearance/containment certificate"))
        print(json.dumps(cases, indent=2), flush=True)
        return
    regression = []
    for r in read(Path(config["nativeRegression"]))["results"]:
        found = checker.hits(r["cadDegrees"], False)
        confirmed = any(not h["Possible"] and h["VolumeMm3"] > 0 for h in r["crossLinkOverlaps"])
        possible = any(h["Possible"] or h["VolumeMm3"] <= 0 for h in r["crossLinkOverlaps"])
        regression.append(dict(label=r["label"], cadDegrees=r["cadDegrees"], meshPairs=found,
            nativeConfirmed=confirmed, nativePossible=possible,
            agreement=None if possible else confirmed == bool(found)))
    regression_result = dict(results=regression, input=checker.input,
        agreement=sum(r["agreement"] is True for r in regression),
        disagreement=[r for r in regression if r["agreement"] is False],
        possible=sum(r["agreement"] is None for r in regression))
    write(out / "regression.json", regression_result)
    print(f"Native regression: {regression_result['agreement']}/{len(regression)}; "
          f"disagreements={len(regression_result['disagreement'])}; unknown={regression_result['possible']}", flush=True)
    if regression_only:
        return
    assert not regression_result["disagreement"], "Resolve native regression differences before scanning"
    axes = [sorted(set(round(v, 8) for v in [*np.linspace(lo, hi, n), 0.]))
            for (lo, hi), n in zip(config["recordedSoftwareRangesDeg"][1:], config["gridCounts"])]
    dims = tuple(map(len, axes))
    coords = list(itertools.product(*(range(d) for d in dims)))
    index = {c: i for i, c in enumerate(coords)}
    offset = np.array(config["offsetCadDeg"])
    poses = np.array([[0., *(axes[j][v]+offset[j+1] for j, v in enumerate(c))] for c in coords])
    free = np.zeros(len(poses), dtype=bool)
    for i, q in enumerate(poses):
        free[i] = not checker.hits(q)
        if i % 1000 == 0:
            print(f"Grid {i}/{len(poses)}; unique={checker.checks}; seconds={time.perf_counter()-start:.1f}", flush=True)
    seed = index[tuple(a.index(0.) for a in axes)]
    assert free[seed] and not checker.hits(offset), "Candidate upright anchor collides"
    parent = {seed: -1}
    queue = deque([seed])
    edges = blocked = 0
    while queue:
        i = queue.popleft()
        for j, direction in itertools.product(range(4), (-1, 1)):
            c = list(coords[i]); c[j] += direction
            k = index.get(tuple(c))
            if k is None or not free[k] or k in parent:
                continue
            edges += 1
            if checker.clear_line(poses[i], poses[k], config["graphEdgeMaxStepDeg"]):
                parent[k] = i
                queue.append(k)
            else:
                blocked += 1
        if len(parent) % 1000 == 0:
            print(f"Connected={len(parent)}; edges={edges}; unique={checker.checks}", flush=True)
    paths, marginals, refined_states = [], [], []
    connected = list(parent)
    def refined_path(i, endpoint=None):
        raw = []
        while i >= 0:
            raw.append(poses[i]); i = parent[i]
        raw.reverse()
        if endpoint is not None and not np.allclose(endpoint, raw[-1], atol=1e-9, rtol=0):
            raw.append(endpoint)
        compressed = [raw[0]]
        at = 0
        while at < len(raw)-1:
            nxt = len(raw)-1
            while nxt > at+1 and not checker.clear_line(raw[at], raw[nxt], config["finalPathMaxStepDeg"]):
                nxt -= 1
            assert checker.clear_line(raw[at], raw[nxt], config["finalPathMaxStepDeg"]), "Refined path failed"
            compressed.append(raw[nxt]); at = nxt
        return [p.tolist() for p in compressed]
    for j in range(1, 5):
        ids = [min(connected, key=lambda i: poses[i, j]), max(connected, key=lambda i: poses[i, j])]
        marginal = []
        for side, i in enumerate(ids):
            # Refine the unresolved grid-to-record boundary at fixed other axes.
            # This is a local extension, not a proof no other branch goes farther.
            end = poses[i].copy()
            limit = end.copy()
            limit[j] = config["mappedCadRangesDeg"][j][side]
            for trial in line(end, limit, .25):
                if checker.hits(trial):
                    lo, hi = end.copy(), trial.copy()
                    while abs(hi[j]-lo[j]) > .001:
                        mid = (lo+hi)/2
                        if checker.hits(mid):
                            hi = mid
                        else:
                            lo = mid
                    end = lo
                    break
                end = trial
            label = f"J{j+1}_{'max' if side else 'min'}"
            if abs(end[j]-poses[i, j]) > 1e-7:
                refined_states.append(dict(id="refined_"+label, cadDegrees=end.tolist()))
            marginal.append(float(end[j]-offset[j]))
            paths.append(dict(id=label, waypointsCadDeg=refined_path(i, end)))
            print(f"Refined {label}: endpoint={end[j]:.5f} CAD deg", flush=True)
        marginals.append(marginal)
    # Four large shoulder/elbow corner witnesses. Prefer minimal wrist motion
    # only after minimizing the shoulder/elbow distance to the recorded corner.
    for side2, side3 in itertools.product((0, 1), repeat=2):
        target = np.array([config["mappedCadRangesDeg"][j][side] for j, side in ((1, side2), (2, side3))])
        i = min(connected, key=lambda k: (
            round(float(((poses[k, 1:3]-target)**2).sum()), 6),
            float(((poses[k, 3:]/[227.4, 22.633])**2).sum())))
        label = f"coupled_J2_{'max' if side2 else 'min'}_J3_{'max' if side3 else 'min'}"
        paths.append(dict(id=label, waypointsCadDeg=refined_path(i)))
        print(f"{label}: endpoint={poses[i].tolist()}", flush=True)
    delta = np.array([0., 8., -12., 20., 5.])
    def coordinated_clear(d):
        for order in itertools.permutations(range(1, 5)):
            a = offset.copy()
            for j in order:
                b = a.copy(); b[j] += d[j]
                if not checker.clear_line(a, b, .5):
                    return False
                a = b
        if not checker.clear_line(offset, offset+d, .5):
            return False
        for phase in np.linspace(0, 1.6, 161):
            s = np.clip(phase-.15*np.arange(5), 0, 1)
            if checker.hits(offset+d*(10*s**3-15*s**4+6*s**5)):
                return False
        return True
    while not coordinated_clear(delta):
        delta /= 2
        assert abs(delta).max() > .1, "No useful matched coordination excursion"
    write(out / "connected_samples.json", dict(conditionalMapping=config["coordinateAssumption"],
        states=[dict(id=i, cadDegrees=poses[i].tolist()) for i in connected]+refined_states))
    result = dict(generatedUtc=datetime.now(timezone.utc).isoformat(), config=config, input=checker.input,
        engine=f"python-fcl {fcl.__version__}", calibrationStatus="HYPOTHESIS_ONLY",
        gridDimensions=dims, gridChecked=len(poses), gridFree=int(free.sum()), anchorConnected=len(parent),
        unreachableFree=int(free.sum())-len(parent), graphEdgesChecked=edges, graphEdgesBlocked=blocked,
        uniquePoseChecks=checker.checks, marginalSoftwareRangesDeg=marginals, anchorCadDeg=offset.tolist(),
        refinedBoundaryWitnesses=refined_states,
        paths=paths, coordinationDeltaCadDeg=delta.tolist(), pairCounts=checker.counts,
        collisionWitnesses=checker.witnesses, elapsedSeconds=time.perf_counter()-start,
        guarantee="Finite triangle surface and floor checks with conditional bounded installation-surface pruning. "
        "No containment, continuous-space, positive-clearance, calibrated-encoder or hardware-safety proof. "
        "Conservative remaining allowed-contact intersections may reject poses. "
        "Marginals are projections of a coupled set, NOT a safe Cartesian product.")
    write(out / "geometry.json", result)
    print(json.dumps({k: result[k] for k in ("gridChecked", "gridFree", "anchorConnected",
        "marginalSoftwareRangesDeg", "uniquePoseChecks", "elapsedSeconds")}, indent=2), flush=True)


def self_test():
    p = np.array(list(line([0, -227, 0], [0, 227, 1], .5)))
    assert len(p) == 909 and abs(np.diff(p, axis=0)).max() <= .500001
    assert p[0, 1] == -227 and p[-1, 1] == 227
    half = fcl.CollisionObject(fcl.Halfspace(np.array([0., -1., 0.]), -1))
    request = fcl.CollisionRequest()
    for y, expected in ((0, False), (2, True)):
        box = fcl.CollisionObject(fcl.Box(.1, .1, .1), fcl.Transform(np.array([0., y, 0.])))
        assert bool(fcl.collide(half, box, request, fcl.CollisionResult())) == expected
    print("PASS: unwrapped interpolation and forbidden-floor halfspace direction")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--trace", action="store_true")
    parser.add_argument("--regression-only", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
    else:
        assert args.out
        run(args.out, args.trace, args.regression_only)
