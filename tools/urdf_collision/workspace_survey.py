"""Offline candidate survey; projected unknown boxes are NOT an admission certificate."""
import argparse
from collections import Counter
from fractions import Fraction
import hashlib
import itertools
import math
from pathlib import Path
import re
import time

import numpy as np

from workspace_geometry import Checker, read, write
from workspace_verify import (compact_membership, compact_wrist_intervals,
                              factored_intervals, factored_membership,
                              intersect_intervals, merge_intervals, segment_in_regions,
                              BoundsChecker, check_curve, partition_regions, focus_seeds)


def projected_gaps(unknown, bounds, weight):
    """Open complement of closed unknown-box projections, rounded strictly inward."""
    clipped = np.asarray(unknown).copy()
    clipped[:, :, 0] = np.maximum(clipped[:, :, 0], bounds[:, 0])
    clipped[:, :, 1] = np.minimum(clipped[:, :, 1], bounds[:, 1])
    clipped = clipped[np.all(clipped[:, :, 1] >= clipped[:, :, 0], axis=1)]
    coefficients = np.array([0., weight, 1., 0., 0.])
    lower = np.where(coefficients >= 0, clipped[:, :, 0], clipped[:, :, 1]) @ coefficients
    upper = np.where(coefficients >= 0, clipped[:, :, 1], clipped[:, :, 0]) @ coefficients
    minimum = float(np.where(coefficients >= 0, bounds[:, 0], bounds[:, 1]) @ coefficients)
    maximum = float(np.where(coefficients >= 0, bounds[:, 1], bounds[:, 0]) @ coefficients)
    gaps, cursor = [], minimum
    for low, high in merge_intervals(np.column_stack((lower, upper)).tolist()):
        if low > cursor:
            gaps.append([cursor, min(low, maximum)])
        cursor = max(cursor, high)
    if cursor < maximum:
        gaps.append([cursor, maximum])
    return [[math.ceil((low+1e-8)*10)/10, math.floor((high-1e-8)*10)/10]
            for low, high in gaps if high-low > .21]


def wrist_projected_gaps(unknown, bounds, weight, intervals):
    """A shared slab must avoid unknown boxes in EVERY separate wrist window."""
    result = None
    for interval in intersect_intervals(intervals, [bounds[3].tolist()]):
        window = bounds.copy()
        window[3] = interval
        gaps = projected_gaps(unknown, window, weight)
        result = gaps if result is None else intersect_intervals(result, gaps)
        if not result:
            break
    return result or []


def survey(build, out, outer=False):
    identities, unknowns, tables = {}, [], []
    for mapping in ("baseline", "alignment"):
        directory = build / f"workspace_factored_regions_20261117_{mapping}_v10"
        report_path = directory / "factored_regions.json"
        prior_family = "affine" if outer else "inner"
        prior_path = build / f"workspace_{prior_family}_rule_20261117_{mapping}_v1/compact_rule_comparison.json"
        report, prior = read(report_path), read(prior_path)["combinedRule"]
        for path in (report_path, prior_path):
            with path.open("rb") as stream:
                identities[str(path.resolve())] = hashlib.file_digest(stream, "sha256").hexdigest()
        assert all(not group["unknownReducedCadBoxes"] for group in report["groups"][:-1])
        unknowns.append(np.asarray([row["cadBoundsDeg"]
                                    for row in report["groups"][-1]["unknownReducedCadBoxes"]])
                        -np.asarray(report["offsetCadDeg"])[None, :, None])
        if mapping == "baseline":
            recorded = np.asarray(report["recordedSoftwareRangesDeg"])
            points = np.random.default_rng(report["coverageRandomSeed"]).uniform(
                recorded[:, 0], recorded[:, 1], (report["coverageSamples"], 5))
            old = compact_membership(prior, points)
            domain = np.asarray(report["admissibleSoftwareDomainDeg"])
        else:
            assert np.array_equal(old, compact_membership(prior, points))
            domain[:, 0] = np.maximum(domain[:, 0], np.asarray(report["admissibleSoftwareDomainDeg"])[:, 0])
            domain[:, 1] = np.minimum(domain[:, 1], np.asarray(report["admissibleSoftwareDomainDeg"])[:, 1])
        inside = factored_membership(report["groups"], points)
        own_domain = np.asarray(report["admissibleSoftwareDomainDeg"])
        tables.append(inside & np.all((points >= own_domain[:, 0]) & (points <= own_domain[:, 1]), axis=1))
        del report
    common, unknown = tables[0] & tables[1], np.concatenate(unknowns)
    missing = common & ~old
    assert int(old.sum()) == (4767 if outer else 4585) and int(common.sum()) == 5097
    results = []
    fifths = [(-16.6, 0.), (0., 10.), (10., 18.), (18., 19.9),
              (19.9, 21.5), (21.5, 22.632856)]
    windows = [(0., radius) for radius in (30., 60., 90., 120., 140.)]
    if outer:
        fifths += [(-16.6, 22.632856), (-16.6, 10.), (0., 22.632856),
                   (10., 22.632856), (18., 22.632856)]
        windows = ([(0., radius) for radius in (140., 150., 165., 180., 210., 360.)]
                   +[(180., radius) for radius in (15., 30., 38., 45., 60., 90., 120.)]
                   +[(360., 360.-lower) for lower in (140., 150., 165., 180., 195., 210.)])
    shoulder_ranges = [(domain[1, 0], domain[1, 1]), (domain[1, 0], 0.),
                       (0., 15.), (15., 30.), (30., domain[1, 1]),
                       (domain[1, 0], -15.), (-15., 15.), (0., domain[1, 1]),
                       (15., domain[1, 1]), (domain[1, 0], 15.), (domain[1, 0], 30.)]
    if outer:
        shoulder_ranges = shoulder_ranges[:1]
    for shoulder_index, shoulder in enumerate(shoulder_ranges):
        weights = ((.5, 1.5) if outer else
                   ((0., .5, .75, 1., 1.25, 1.5, 1.75, 2., 2.5, 3.) if shoulder_index == 0 else (1.5,)))
        for weight in weights:
            for fifth in fifths:
                for center, radius in windows:
                    bounds = domain.copy()
                    bounds[1], bounds[4] = shoulder, fifth
                    intervals = compact_wrist_intervals(dict(J4CenterAbsDeg=center, J4RadiusDeg=radius))
                    intervals = intersect_intervals(intervals, [domain[3].tolist()])
                    wrist = np.zeros(len(points), dtype=bool)
                    for start, stop in intervals:
                        wrist |= (points[:, 3] >= start) & (points[:, 3] <= stop)
                    for low, high in wrist_projected_gaps(unknown, bounds, weight, intervals):
                        value = weight*points[:, 1]+points[:, 2]
                        admitted = (np.all((points >= bounds[:, 0]) & (points <= bounds[:, 1]), axis=1)
                                    & wrist & (value >= low) & (value <= high))
                        assert not np.any(admitted & ~common)
                        novel = admitted & ~old
                        if novel.any():
                            results.append(dict(weight=weight, J2Deg=list(shoulder), J5Deg=list(fifth),
                                                J4CenterAbsDeg=center, J4RadiusDeg=radius,
                                                weightedLowerDeg=low, weightedUpperDeg=high,
                                                newlyAccepted=int(novel.sum()),
                                                newSmallWrist=int((novel & (abs(points[:, 3]) < 60)).sum()),
                                                newSampleIndices=np.flatnonzero(novel).tolist()))
                if outer:
                    print(f"Surveyed weight={weight:g}, J5={fifth}, wrist windows={len(windows)}", flush=True)
    results.sort(key=lambda row: (-row["newlyAccepted"], -row["newSmallWrist"], row["weight"] != 1.5))
    # The best eight greedily chosen additions expose complexity versus sampled gain.
    covered, greedy = set(), []
    for _ in range(8 if results else 0):
        candidate = max(results, key=lambda row: len(set(row["newSampleIndices"])-covered))
        gain = len(set(candidate["newSampleIndices"])-covered)
        if not gain:
            break
        covered.update(candidate["newSampleIndices"])
        greedy.append(dict(candidate, marginalNewSamples=gain, cumulativeNewSamples=len(covered)))
    result = dict(previousReadableAccepted=int(old.sum()), commonFullTableAccepted=int(common.sum()),
                  commonUnutilized=int(missing.sum()), smallWristUnutilized=int((missing & (abs(points[:, 3]) < 60)).sum()),
                  outerWristUnutilized=int((missing & (abs(points[:, 3]) >= 140)).sum()),
                  candidates=results, greedySampleComparison=greedy, inputFileSha256=identities,
                  implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  limitations=["Candidate survey only; all eight groups must still be certified independently per mapping",
                               "Finite fixed joint-coordinate samples measure gain only, not Cartesian volume or maximality",
                               "No MuJoCo, actual trajectory, FCL or physical calibration admission in this report"])
    out.mkdir(parents=True, exist_ok=False)
    write(out / "candidate_survey.json", result)
    print({key: result[key] for key in ("previousReadableAccepted", "commonFullTableAccepted",
                                      "commonUnutilized", "smallWristUnutilized")}, flush=True)
    for row in results[:16]:
        print({key: value for key, value in row.items() if key != "newSampleIndices"}, flush=True)
    print("GREEDY", [{key: value for key, value in row.items() if key != "newSampleIndices"} for row in greedy],
          flush=True)


def self_test():
    bounds = np.array([[0., 0.], [0., 4.], [0., 4.], [-1., 1.], [0., 1.]])
    blocked = bounds.copy()
    blocked[1:3] = [[2., 3.], [1., 2.]]
    assert projected_gaps([blocked], bounds, 1.) == [[.1, 2.9], [5.1, 7.9]]
    assert projected_gaps([blocked], bounds, -1.) == [[-3.9, -2.1], [.1, 3.9]]
    outside = blocked.copy()
    outside[4] = [2., 3.]
    assert projected_gaps([outside], bounds, 1.) == [[.1, 7.9]]
    positive, negative = blocked.copy(), blocked.copy()
    positive[3], negative[3] = [.5, 1.], [-1., -.5]
    negative[1:3] = [[0., 1.], [0., 1.]]
    assert wrist_projected_gaps([positive, negative], bounds, 1., [[-1., -.5], [.5, 1.]]) == [
        [2.1, 2.9], [5.1, 7.9]]
    assert wrist_projected_gaps([positive, negative], bounds, 1., [[2., 3.]]) == []
    assert compact_wrist_intervals(dict(J4CenterAbsDeg=360., J4RadiusDeg=150.)) == [
        [-510., -210.], [210., 510.]]
    rule = dict(coordinateDomainSoftwareDeg=bounds.tolist(), bands=[
        dict(J5Deg=[0., 1.], J4CenterAbsDeg=0., J4RadiusDeg=1., weightedLowerDeg=0.,
             weightedUpperDeg=8.),
        dict(J5Deg=[0., .5], J4CenterAbsDeg=0., J4RadiusDeg=1., weightedLowerDeg=0.,
             weightedUpperDeg=8.),
        dict(J5Deg=[.5, 1.], J4CenterAbsDeg=0., J4RadiusDeg=1., weightedLowerDeg=0.,
             weightedUpperDeg=8.)])
    assert band_coverage(rule, 0, [1, 2])
    rule["bands"][2]["J5Deg"][0] = float(np.nextafter(.5, math.inf))
    assert band_coverage(rule, 0, [1, 2]) is None, "Even a sub-ULP real gap must remain unfilled"
    assert interval_contains([[0., 1.], [2., 3.]], 1.)
    assert not interval_contains([[0., 1.], [2., 3.]], 1.5)
    q = np.array([0., 2., 2., 0., .5])
    assert np.array_equal(gap_window(q, [1., 3.], bounds, 1.),
                          [[0., 0.], [1.5, 2.5], [.5, 3.5], [-1., 1.], [0., 1.]])
    assert np.array_equal(gap_window(q, [1., 3.], bounds, 20.), bounds)
    for scale in (0., -1., math.inf):
        try:
            gap_window(q, [1., 3.], bounds, scale)
        except ValueError:
            pass
        else:
            raise AssertionError("Invalid focus scale accepted")
    roomy = np.array([[0., 0.], [-40., 45.], [-96., 103.], [-226., 226.], [-21., 21.]])
    whole = [dict(groups=[dict(verifiedSoftwareBoxesDeg=np.array([roomy]))])]*2
    sample = [dict(index=1, newRegionWitnesses=[
        [0., -30., -85., -180., -10.], [0., -24., -90., -130., 10.]])]
    loops, selected = large_coordination_routes(sample, whole, roomy)
    assert len(loops) == 1 and selected[0]["selected"]["q3LiftDeg"] == 0.
    assert selected[0]["selected"]["fourAxisDeltaDeg"] == [6., 5., 50., 20.]
    assert all(segment_in_regions(np.array([roomy]), a, b)
               for points in loops.values() for a, b in zip(points, points[1:]))
    a, b = np.asarray(sample[0]["newRegionWitnesses"])
    actual = a+np.linspace(0., 1., 101)[:, None]*(b-a)
    observed = observed_segment_span(np.vstack((actual, a+[0, 0, 50, 0, 0])), a, b, 1e-7)
    assert observed["samples"] == 101 and np.allclose(observed["fourAxisObservedSpanDeg"], [6, 5, 50, 20])
    sample[0]["newRegionWitnesses"][1][1] = -29.
    assert not large_coordination_routes(sample, whole, roomy)[0]
    cells = corridor_windows([dict(category="test", selected=dict(
        aSoftwareDeg=a.tolist(), bSoftwareDeg=b.tolist()))], roomy)
    assert len(cells) == 16
    assert all(np.all((a+t*(b-a) >= cell[0][:, 0]) & (a+t*(b-a) <= cell[0][:, 1]))
               for cell, t in zip(cells, (np.arange(16)+.5)/16))
    assert all(np.array_equal(cell[0][0], [0., 0.]) for cell in cells)
    assert all(np.all(cell[0][:, 0] >= roomy[:, 0]) and np.all(cell[0][:, 1] <= roomy[:, 1])
               for cell in cells)
    original = [dict(category="test", selected=dict(aSoftwareDeg=a.tolist(),
                bSoftwareDeg=b.tolist(), q3LiftDeg=5.))]
    direct, selection = direct_coordination_routes(original)
    assert selection[0]["selected"]["q3LiftDeg"] == 0. and original[0]["selected"]["q3LiftDeg"] == 5.
    assert np.array_equal(direct["large_test_return"][1:3], [a, b])
    assert len(direct["large_test_return"]) == 5
    def fake_proof(box):
        ok = box[1, 1]-box[1, 0] <= 4.
        return [dict(verifiedFraction=float(ok), unknown=[] if ok else [{}])]
    joined, attempts = coarsen_windows(cells, roomy, fake_proof)
    assert joined and attempts
    assert all(any(np.all(old[0][:, 0] >= new[0][:, 0]) and
                       np.all(old[0][:, 1] <= new[0][:, 1]) for new in joined) for old in cells)
    whole, _ = coarsen_windows(cells, roomy,
                               lambda box: [dict(verifiedFraction=1., unknown=[])])
    assert len(whole) == 1
    assert np.all(whole[0][0][:, 0] >= roomy[:, 0]) and np.all(whole[0][0][:, 1] <= roomy[:, 1])
    anchor = np.array([0., -30., -85., -180., -10.])
    old = [dict(groups=[dict(verifiedSoftwareBoxesDeg=np.array([np.column_stack((anchor, anchor))]))])]
    routes, selected, diagnostics = coarse_boundary_routes([
        dict(category="test", focusSoftwareBoundsDeg=roomy.tolist(),
             anchorCandidatesSoftwareDeg=[anchor.tolist()])], old, roomy)
    assert len(routes) == len(selected) == 3 and diagnostics[0]["selectedPairs"] == 3
    assert all(np.array_equal(points[0], anchor) and np.array_equal(points[-1], anchor)
               for points in routes.values())
    skipped = dict(category="not_a_dynamic_representative", focusSoftwareBoundsDeg=roomy.tolist(),
                   anchorCandidatesSoftwareDeg=[anchor.tolist()], boundaryRepresentative=False)
    repeated, _, _ = coarse_boundary_routes([skipped, dict(
        category="test", focusSoftwareBoundsDeg=roomy.tolist(),
        anchorCandidatesSoftwareDeg=[anchor.tolist()])], old, roomy)
    assert set(repeated) == set(routes), "Static-only boxes must not silently become tested representatives"
    left, right = bounds.copy(), bounds.copy()
    left[1, 1] = right[1, 0] = 2.
    complete = dict(unknown=[], verified=[dict(cadBoundsDeg=b.tolist()) for b in (left, right)])
    assert audit_partition_cover(complete, bounds) == 2
    for leaves in ([left], [left, left, right]):
        try:
            audit_partition_cover(dict(unknown=[], verified=[dict(cadBoundsDeg=b.tolist()) for b in leaves]), bounds)
        except AssertionError:
            pass
        else:
            raise AssertionError("Incomplete or overlapping leaf cover accepted")
    overlap = bounds.copy()
    overlap[1] = [1., 3.]
    inherited, pending = inherited_box_partition(bounds, np.array([left, overlap]), ["test"])
    assert inherited and pending
    assert audit_partition_cover(dict(verified=inherited+pending, unknown=[]), bounds) >= 3
    assert all(row["inheritedSafeBoxIndex"] in (0, 1) for row in inherited)
    assert all(row["cadBoundsDeg"][1][0] >= 3. for row in pending)
    inherited, pending = inherited_box_partition(bounds, np.array([right, left, overlap]), ["test"])
    assert not pending and audit_partition_cover(dict(verified=inherited, unknown=[]), bounds)
    for row in inherited:
        known = np.array([right, left, overlap])[row["inheritedSafeBoxIndex"]]
        leaf = np.asarray(row["cadBoundsDeg"])
        assert np.all(leaf[:, 0] >= known[:, 0]) and np.all(leaf[:, 1] <= known[:, 1])
    missed = bounds.copy()
    missed[1] = [5., 6.]
    inherited, pending = inherited_box_partition(bounds, np.array([missed]), ["test"])
    assert not inherited and len(pending) == 1 and pending[0]["unresolved"] == ["test"]
    pool, excluded = frontier_candidates([
        dict(sampleIndex=1, softwareDeg=[0., 0., 0., 0., 0.], state="inside_clear"),
        dict(sampleIndex=2, softwareDeg=[0., 20., 20., 20., 0.], state="inside_collision")], roomy, roomy)
    assert pool and excluded and max(row["scale"] for row in pool) < 8.
    assert all(row["coveredSampleIndices"] == [1] for row in pool)
    sliced = np.column_stack((q, q))
    sliced[2] = [-3., 4.]
    leaves = []
    for low, high in ((-3., -1.), (-1., 1.), (2., 4.)):
        leaf = sliced.copy()
        leaf[2] = [low, high]
        leaves.append(dict(cadBoundsDeg=leaf.tolist()))
    intervals = certified_slice_intervals(dict(verified=leaves), sliced, .125)
    assert len(intervals) == 2 and intervals[0][0] > -3.125 and intervals[0][1] < .875
    assert not interval_contains(intervals, 1.375), "A gap between proved leaves must remain"
    assert certified_slice_intervals(dict(verified=[]), sliced, .125) == []
    leaves[0]["cadBoundsDeg"][1][0] -= .1
    try:
        certified_slice_intervals(dict(verified=leaves), sliced, .125)
    except ValueError:
        pass
    else:
        raise AssertionError("A non-slice proof was accepted as a fixed-axis interval")
    anchor_tables = [dict(admissibleSoftwareDomainDeg=roomy.tolist(),
                          groups=[dict(verifiedSoftwareBoxesDeg=np.array([roomy]))])]*2
    anchors = frontier_anchor_candidates(np.array([0., 20., 80., 90., 10.]), anchor_tables, roomy)
    assert anchors and all(np.all(abs(anchor[1:]-[20., 80., 90., 10.]) >= [3., 1., 10., 1.])
                           for anchor in anchors)
    small = np.array([[0., 0.], [-1., 1.], [-1., 1.], [-1., 1.], [-1., 1.]])
    domain = small*10
    history = []
    def admit(box):
        ok = bool(np.all(box[1:, 0] >= -4) and np.all(box[1:, 1] <= 4))
        history.append((box.copy(), ok))
        return len(history)-1 if ok else None
    grown, reference = inflate_faces(small, domain, admit)
    assert reference is not None and np.all(grown[1:, 0] >= -4) and np.all(grown[1:, 1] <= 4)
    assert np.all(grown[:, 0] <= small[:, 0]) and np.all(grown[:, 1] >= small[:, 1])
    assert np.array_equal(history[reference][0], grown) and history[reference][1]
    a, b = np.array([0., -20., 90., -120., -10.]), np.array([0., 20., 60., 120., 10.])
    options = connection_options(a, b)
    assert len(options) == 4 and len(options[0][1]) == 2
    for height, route in options[1:]:
        assert np.array_equal(route[0], a) and np.array_equal(route[-1], b)
        assert np.allclose(route[1:3, 2]+1.5*route[1:3, 1], height)
        assert np.all(abs(route[2, 1:]-route[1, 1:]) >= [10, 10, 60, 4])
    print("PASS: projection, separate wrist windows, exact closed-cell redundancy, disjoint interval queries")


def interval_contains(intervals, value):
    return any(low <= value <= high for low, high in intervals)


def band_coverage(rule, index, retained):
    """Exact same-weight box-union inclusion, including every closed boundary face."""
    band, domain = rule["bands"][index], rule["coordinateDomainSoftwareDeg"]
    others = [(i, rule["bands"][i]) for i in retained if i != index
              and rule["bands"][i].get("J2Weight", 1.5) == band.get("J2Weight", 1.5)]
    def atoms(interval, edges):
        cuts = sorted({Fraction(v) for v in [*interval, *edges] if interval[0] <= v <= interval[1]})
        # Fractions preserve even a real gap between adjacent representable floats.
        return cuts+[(a+b)/2 for a, b in zip(cuts, cuts[1:])]
    fifths = intersect_intervals([band["J5Deg"]], [domain[4]])
    checks, support = 0, set()
    for window in intersect_intervals(compact_wrist_intervals(band), [domain[3]]):
        wrists = atoms(window, [x for _, other in others
                                for pair in compact_wrist_intervals(other) for x in pair])
        for fifth in fifths:
            for q5 in atoms(fifth, [x for _, other in others for x in other["J5Deg"]]):
                for q4 in wrists:
                    active = [(i, other) for i, other in others
                              if other["J5Deg"][0] <= q5 <= other["J5Deg"][1]
                              and interval_contains(compact_wrist_intervals(other), q4)]
                    spans = merge_intervals([[other["weightedLowerDeg"],
                                              other.get("weightedUpperDeg", math.inf)]
                                             for _, other in active])
                    if not any(low <= band["weightedLowerDeg"]
                               and high >= band.get("weightedUpperDeg", math.inf)
                               for low, high in spans):
                        return None
                    checks += 1
                    support.update(i for i, _ in active)
    return dict(closedAndOpenCellsChecked=checks, coveredByOriginalBandIndices=sorted(support))


def rule_boundary_points(rule):
    domain = np.asarray(rule["coordinateDomainSoftwareDeg"])
    points = [np.random.default_rng(2026111711).uniform(domain[:, 0], domain[:, 1], (12000, 5))]
    def neighbors(values):
        values = np.unique([value for value in values if math.isfinite(value)])
        return np.unique(np.r_[values, np.nextafter(values, -math.inf), np.nextafter(values, math.inf)])
    for band in rule["bands"]:
        fifths = neighbors([*band["J5Deg"], np.mean(band["J5Deg"])])
        wrists = neighbors([*domain[3], 0., *[x for span in compact_wrist_intervals(band) for x in span]])
        for q2 in (*domain[1], -35., -20., 0., 20., 40.):
            thirds = neighbors([*domain[2], band["weightedLowerDeg"]-band.get("J2Weight", 1.5)*q2,
                                band.get("weightedUpperDeg", math.inf)-band.get("J2Weight", 1.5)*q2])
            q3, q4, q5 = (grid.ravel() for grid in np.meshgrid(thirds, wrists, fifths, indexing="ij"))
            points.append(np.column_stack((np.zeros(len(q3)), np.full(len(q3), q2), q3, q4, q5)))
    return np.concatenate(points)


def simplify(build, out):
    """Preserve the complete current rule; prove removals rather than sampling them."""
    results, identities = [], {}
    for mapping in ("baseline", "alignment"):
        directory = build / f"workspace_outer_rule_20261117_{mapping}_v1"
        path = directory / "compact_rule_comparison.json"
        original = read(path)["combinedRule"]
        retained = list(range(len(original["bands"])))
        for index in list(retained):
            if band_coverage(original, index, retained) is not None:
                retained.remove(index)
        removed = [i for i in range(len(original["bands"])) if i not in retained]
        proofs = {str(i): band_coverage(original, i, retained) for i in removed}
        assert all(proof is not None for proof in proofs.values())
        reduced = dict(original, bands=[original["bands"][i] for i in retained])
        # The retained OR is a subset; the removed-band proofs establish the reverse inclusion.
        domain = original["coordinateDomainSoftwareDeg"]
        for q2 in domain[1]:
            anchor = Fraction(10)-Fraction(1.5)*Fraction(q2)
            assert Fraction(domain[2][0]) <= anchor <= Fraction(domain[2][1])
            for band in reduced["bands"]:
                value = Fraction(10)+(Fraction(band.get("J2Weight", 1.5))-Fraction(1.5))*Fraction(q2)
                assert Fraction(band["weightedLowerDeg"]) <= value
                assert value <= band.get("weightedUpperDeg", math.inf)
        points = rule_boundary_points(original)
        before, after = compact_membership(original, points), compact_membership(reduced, points)
        assert np.array_equal(before, after)
        del points
        trace_path, config_path = directory / "actual_trace.json", directory / "config.json"
        trace, config = read(trace_path), read(config_path)
        points = np.asarray([sample["cadDegrees"] for sample in trace["samples"]])-config["offsetCadDeg"]
        # As in the original compact audit, only membership projects the held J1 to exactly zero.
        # Existing five-axis FCL results retain its actual tiny tracking deviation.
        points[:, 0] = 0.
        trace_before, trace_after = compact_membership(original, points), compact_membership(reduced, points)
        assert np.array_equal(trace_before, trace_after)
        result = dict(mapping=mapping, originalBands=len(original["bands"]), reducedBands=len(retained),
                      removedOriginalBandIndices=removed, retainedOriginalBandIndices=retained,
                      wholeRemovedBandCoverage=proofs, reducedRule=reduced,
                      boundaryAndRandomPoints=len(before), membershipMismatches=int((before != after).sum()),
                      actualTraceProjectionPoints=len(points),
                      actualTraceProjectionMismatches=int((trace_before != trace_after).sum()),
                      commonS10AnchorProvenAtBothJ2Endpoints=True)
        results.append(result)
        for identity_path in (path, trace_path, config_path):
            with identity_path.open("rb") as stream:
                identities[str(identity_path.resolve())] = hashlib.file_digest(stream, "sha256").hexdigest()
        print({key: value for key, value in result.items()
               if key not in ("reducedRule", "wholeRemovedBandCoverage")}, flush=True)
    out.mkdir(parents=True, exist_ok=False)
    write(out / "equivalent_limits.json", dict(results=results, inputFileSha256=identities,
          implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
          verifierSha256=hashlib.sha256(Path(__file__).with_name("workspace_verify.py").read_bytes()).hexdigest(),
          comparisonToleranceDeg=0.,
          limitations=["Exact box-union equivalence is not a new geometry or dynamics certificate",
                       "J3 min/max is real algebra; use the original S/T OR for floating-point membership",
                       "Existing geometry rounding, unregistered mappings and J1=0 restrictions still apply"]))


def joint3_tables(build, extension=None):
    """Keep verified arrays only; do not retain hundreds of MB of unknown-box JSON."""
    tables, identities = [], {}
    for mapping in ("baseline", "alignment"):
        path = build / f"workspace_factored_regions_20261117_{mapping}_v10/factored_regions.json"
        report = read(path)
        tables.append(dict(mapping=mapping, admissibleSoftwareDomainDeg=report["admissibleSoftwareDomainDeg"],
                           adjacentProofReport=report["previousReport"],
                           groups=[dict(constraints=group["constraints"],
                                        verifiedSoftwareBoxesDeg=np.asarray(group["verifiedSoftwareBoxesDeg"]))
                                   for group in report["groups"]]))
        with path.open("rb") as stream:
            identities[str(path.resolve())] = hashlib.file_digest(stream, "sha256").hexdigest()
        if extension is not None:
            extra_path = extension / mapping / "gap_refinement.json"
            extra = read(extra_path)
            assert extra["baseTableSha256"] == identities[str(path.resolve())]
            assert extra["mapping"] == mapping and extra["lastGroupConstraints"] == report["groups"][-1]["constraints"]
            boxes = np.asarray(extra["verifiedSoftwareBoxesDeg"], dtype=float).reshape(-1, 5, 2)
            domain = np.asarray(report["admissibleSoftwareDomainDeg"])
            assert np.isfinite(boxes).all() and np.all(boxes[:, :, 0] <= boxes[:, :, 1])
            assert np.all(boxes[:, :, 0] >= domain[:, 0]) and np.all(boxes[:, :, 1] <= domain[:, 1])
            tables[-1]["groups"][-1]["verifiedSoftwareBoxesDeg"] = np.concatenate(
                (tables[-1]["groups"][-1]["verifiedSoftwareBoxesDeg"], boxes))
            identities[str(extra_path.resolve())] = hashlib.sha256(extra_path.read_bytes()).hexdigest()
        del report
    return tables, identities


def joint3_query(tables, software):
    own = [factored_intervals(table, software, 2) for table in tables]
    return intersect_intervals(*own), own


def inherited_constraint_scope(table, checker, config, identities):
    """Account for 30 adjacent-axis proofs plus the 59 factored constraints."""
    domain = np.asarray(table["admissibleSoftwareDomainDeg"])
    assert len(table["groups"]) == 8 and all(
        np.array_equal(group["verifiedSoftwareBoxesDeg"], [domain]) for group in table["groups"][:-1])
    path = Path(table["adjacentProofReport"])
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    assert str(path.resolve()) not in identities or identities[str(path.resolve())] == digest
    identities[str(path.resolve())] = digest
    original = read(path)
    names = [name for group in table["groups"] for name in group["constraints"]]
    assert set(names) == set(original["remainingConstraints"]) and len(names) == len(set(names))
    cad = domain+np.asarray(config["offsetCadDeg"])[:, None]
    adjacent = []
    for item in original["adjacentConstraints"]:
        joint = item["joint"]-1
        intervals = merge_intervals([r["cadBoundsDeg"][joint] for r in item["result"]["verified"]])
        assert intervals == item["intervalsCadDeg"]
        assert any(low <= cad[joint, 0] <= cad[joint, 1] <= high for low, high in intervals)
        for name in item["constraints"]:
            pair = checker.constraints[name]["pair"]
            assert pair["low"] == joint and pair["high"] == joint+1
            assert np.all(np.delete(checker.constraints[name]["radius"], joint) == 0)
        adjacent.extend(item["constraints"])
    assert len(names)+len(adjacent) == len(set(names+adjacent)) == len(checker.constraints) == 89
    assert set(names+adjacent) == set(checker.constraints)
    return dict(totalConstraints=89, adjacentConstraints=len(adjacent),
                completeFactoredConstraints=len(names)-len(table["groups"][-1]["constraints"]),
                newlyBoundConstraints=table["groups"][-1]["constraints"],
                adjacentProofReport=str(path.resolve()))


def certified_slice_intervals(proof, cad_bounds, offset3):
    """Merge proved CAD leaves before inward conversion; never fill unknown gaps."""
    bounds = np.asarray(cad_bounds, dtype=float)
    fixed = [0, 1, 3, 4]
    if (bounds.shape != (5, 2) or not np.isfinite(bounds).all()
            or not np.array_equal(bounds[fixed, 0], bounds[fixed, 1])
            or bounds[2, 0] >= bounds[2, 1] or not math.isfinite(offset3)):
        raise ValueError("Expected a finite, fixed-other-axis J3 domain")
    cad_intervals = []
    for row in proof["verified"]:
        box = np.asarray(row["cadBoundsDeg"], dtype=float)
        if (box.shape != (5, 2) or not np.isfinite(box).all()
                or np.any(box[:, 0] > box[:, 1])
                or np.any(box[:, 0] < bounds[:, 0]) or np.any(box[:, 1] > bounds[:, 1])
                or not np.array_equal(box[fixed], bounds[fixed])):
            raise ValueError("Proved leaf does not belong to this fixed-axis slice")
        cad_intervals.append(box[2].tolist())
    result = []
    for low, high in merge_intervals(cad_intervals):
        a, b = math.nextafter(low-offset3, math.inf), math.nextafter(high-offset3, -math.inf)
        if a <= b:
            assert low <= a+offset3 <= b+offset3 <= high
            result.append([a, b])
    return result


def frontier_intervals(build, extension, source, out, software=None):
    """On-demand J3 intervals for every surveyed free frontier pose, not a 4-D table."""
    started = time.perf_counter()
    tables, identities = joint3_tables(build, extension)
    survey_path = None
    if software is None:
        survey_path = source / "frontier_survey.json"
        survey = read(survey_path)
        assert not survey["admittedRegressionViolations"]
        identities.update(survey["inputFileSha256"])
        identities[str(survey_path.resolve())] = hashlib.sha256(survey_path.read_bytes()).hexdigest()
        free = [row for row in survey["pointRows"] if row["state"] == "inside_clear"]
        assert len(free) == survey["classifications"]["inside_clear"]
    else:
        free = [dict(sampleIndex=0, softwareDeg=software)]
    parents, configs, checkers, scopes = [], [], [], []
    for table in tables:
        own = extension / table["mapping"]
        parent = read(own / "gap_refinement.json")
        assert parent["constraintsAccountedFor"] == 89 and parent["completeOtherGroupsInherited"] == 7
        domain = np.asarray(table["admissibleSoftwareDomainDeg"])
        assert all(np.array_equal(group["verifiedSoftwareBoxesDeg"], [domain])
                   for group in table["groups"][:-1])
        identities.update(parent["inputFileSha256"])
        config_path = own / "config.json"
        identities[str(config_path.resolve())] = hashlib.sha256(config_path.read_bytes()).hexdigest()
        parents.append(parent)
        configs.append(read(config_path))
        checkers.append(BoundsChecker(own))
        assert len(checkers[-1].constraints) == 89
        scopes.append(inherited_constraint_scope(table, checkers[-1], configs[-1], identities))
    for name, digest in identities.items():
        with Path(name).open("rb") as stream:
            assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, name
    domains = np.asarray([t["admissibleSoftwareDomainDeg"] for t in tables])
    domain = np.column_stack((domains[:, :, 0].max(axis=0), domains[:, :, 1].min(axis=0)))
    out.mkdir(parents=True, exist_ok=False)
    results, admitted, violations, checks = [], 0, [], 0
    for index, row in enumerate(free):
        q = np.asarray(row["softwareDeg"], dtype=float)
        assert q.shape == (5,) and np.isfinite(q).all() and q[0] == 0
        assert np.all(q >= domain[:, 0]) and np.all(q <= domain[:, 1])
        old, _ = joint3_query(tables, q)
        bounds, proofs, own_intervals = [], [], []
        for checker, config, parent in zip(checkers, configs, parents):
            offset = np.asarray(config["offsetCadDeg"])
            cad = np.column_stack((q+offset, q+offset))
            cad[2] = domain[2]+offset[2]
            proof = partition_regions(checker, cad, parent["lastGroupConstraints"], 2000,
                                      min_width=.005, max_seconds=1., focus=np.asarray([q+offset]))
            bounds.append(cad.tolist())
            proofs.append(proof)
            own_intervals.append(certified_slice_intervals(proof, cad, offset[2]))
        common = intersect_intervals(intersect_intervals(*own_intervals), [domain[2].tolist()])
        augmented = merge_intervals(old+common)
        probes = []
        for low, high in common:
            for angle in (low, (low+high)/2, high):
                probe = q.copy()
                probe[2] = angle
                hits = [checker.hits(probe+config["offsetCadDeg"])
                        for checker, config in zip(checkers, configs)]
                checks += 2
                probes.append(dict(softwareDeg=probe.tolist(), collisionPairsPerMapping=hits))
                if any(hits):
                    violations.append(dict(sampleIndex=row["sampleIndex"], probe=probes[-1]))
        gained = interval_contains(common, q[2]) and not interval_contains(old, q[2])
        admitted += int(gained)
        result = dict(sampleIndex=row["sampleIndex"], softwareDeg=q.tolist(),
                      oldCommonJ3IntervalsDeg=old, certifiedCommonJ3IntervalsDeg=common,
                      augmentedJ3IntervalsDeg=augmented, seedNewlyAdmitted=gained,
                      seedPreviouslyAllowed=interval_contains(old, q[2]),
                      cadBoundsPerMapping=bounds, proofsPerMapping=proofs,
                      certifiedIntervalsPerMapping=own_intervals, fullConstraintProbes=probes)
        write(out / f"slice_{index:03d}.json", result)
        results.append({key: value for key, value in result.items()
                        if key not in ("proofsPerMapping", "fullConstraintProbes")})
        print(f"J3 frontier slice {index+1}/{len(free)}: new seed={gained}, "
              f"total={admitted}, intervals={len(common)}, violations={len(violations)}", flush=True)
        assert not violations, violations
    for name, digest in identities.items():
        with Path(name).open("rb") as stream:
            assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, name
    write(out / "frontier_intervals.json", dict(
        status="conditional_fixed_slices_only", source=str(survey_path.resolve()) if survey_path else None,
        slices=len(results), newlyAdmittedSeeds=admitted, results=results,
        fullConstraintMapPoseChecks=checks, violations=violations,
        constraintsAccountedFor=89, completeOtherGroupsInherited=7,
        constraintScopesPerMapping=scopes,
        budgetPerMappingSlice=dict(maxNodes=2000, maxSeconds=1., minWidthDeg=.005),
        inputFileSha256=identities, implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        elapsedSeconds=time.perf_counter()-started,
        limitations=["Fixed J2/J4/J5 slices only; no interpolation or 4-D neighborhood admission",
                     "Old intervals retained; unknown parts never admitted or bridged",
                     "Joint motion requires separate whole-curve and actual-trace validation",
                     "Uncalibrated two-map geometry, not physical safety or global maximum"]))
    print(dict(slices=len(results), newlyAdmittedSeeds=admitted, checks=checks,
               violations=len(violations), elapsedSeconds=time.perf_counter()-started), flush=True)
    if software is not None:
        print(dict(softwareDegrees=software, augmentedJ3IntervalsDeg=results[0]["augmentedJ3IntervalsDeg"]),
              flush=True)


def frontier_anchor_candidates(q, tables, command):
    candidates = []
    for signs in itertools.product((-1., 1.), repeat=3):
        anchor = q.copy()
        anchor[[1, 3, 4]] += np.array(signs)*[3., 10., 1.]
        if np.any(anchor[[1, 3, 4]] < command[[1, 3, 4], 0]) or np.any(
                anchor[[1, 3, 4]] > command[[1, 3, 4], 1]):
            continue
        old, _ = joint3_query(tables, anchor)
        for low, high in intersect_intervals(old, [command[2].tolist()]):
            if high-low < 3.:
                continue
            point = anchor.copy()
            point[2] = float(np.clip(q[2]-math.copysign(5., q[2]), low+1.5, high-1.5))
            if abs(point[2]-q[2]) >= 1.:
                candidates.append(point)
    return sorted(candidates, key=lambda a: float(np.linalg.norm((a-q)/[1., 3., 5., 10., 1.])))[:8]


def frontier_paths(build, extension, source, out, resume=None):
    """Prove genuinely coordinated old-anchor/new-frontier lines, then prepare diverse replay."""
    started = time.perf_counter()
    source_path = source / "frontier_intervals.json"
    frontier = read(source_path)
    assert frontier["slices"] == len(frontier["results"]) and not frontier["violations"]
    tables, identities = joint3_tables(build, extension)
    identities.update(frontier["inputFileSha256"])
    identities[str(source_path.resolve())] = hashlib.sha256(source_path.read_bytes()).hexdigest()
    configs, checkers, names_per_mapping, scopes = [], [], [], []
    for table in tables:
        own = extension / table["mapping"]
        config_path = own / "config.json"
        identities[str(config_path.resolve())] = hashlib.sha256(config_path.read_bytes()).hexdigest()
        configs.append(read(config_path))
        checkers.append(BoundsChecker(own))
        assert len(checkers[-1].constraints) == 89
        parent = read(own / "gap_refinement.json")
        assert parent["completeOtherGroupsInherited"] == 7 and parent["constraintsAccountedFor"] == 89
        scopes.append(inherited_constraint_scope(table, checkers[-1], configs[-1], identities))
        assert table["groups"][-1]["constraints"] == parent["lastGroupConstraints"]
        names_per_mapping.append(parent["lastGroupConstraints"])
    for name, digest in identities.items():
        with Path(name).open("rb") as stream:
            assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, name
    command = np.asarray(configs[0]["recordedSoftwareRangesDeg"]).copy()
    command[1:, 0] += 1.
    command[1:, 1] -= 1.
    out.mkdir(parents=True, exist_ok=False)
    results, eligible = [], []
    for index, row in enumerate(frontier["results"]):
        q = np.asarray(row["softwareDeg"], dtype=float)
        assert row["seedNewlyAdmitted"] and not interval_contains(joint3_query(tables, q)[0], q[2])
        attempts, selected = [], None
        checkpoint = None if resume is None else resume / f"route_{index:03d}.json"
        if checkpoint is not None and checkpoint.is_file():
            previous = read(checkpoint)
            assert previous["sampleIndex"] == row["sampleIndex"] and np.array_equal(
                previous["targetSoftwareDeg"], q)
            attempts = previous["attempts"]
            if previous["nominalBothMappingsPassed"]:
                selected = np.asarray(previous["selectedAnchorSoftwareDeg"])
                assert all(p["status"] == "numerical_motion_bound_pass"
                           for p in attempts[-1]["proofsPerMapping"])
                assert attempts[-1]["passed"] and np.array_equal(attempts[-1]["anchorSoftwareDeg"], selected)
                assert interval_contains(joint3_query(tables, selected)[0], selected[2])
            identities[str(checkpoint.resolve())] = hashlib.sha256(checkpoint.read_bytes()).hexdigest()
        else:
            for anchor in frontier_anchor_candidates(q, tables, command):
                assert all(np.all((np.asarray([anchor, q]) >= np.asarray(t["admissibleSoftwareDomainDeg"])[:, 0])
                                  & (np.asarray([anchor, q]) <= np.asarray(t["admissibleSoftwareDomainDeg"])[:, 1]))
                           for t in tables), "Inherited root proofs only apply inside their full domains"
                proofs = []
                for checker, config, names in zip(checkers, configs, names_per_mapping):
                    offset = np.asarray(config["offsetCadDeg"])
                    proof = check_curve(checker, lambda t: anchor+t*(q-anchor)+offset,
                                        max_nodes=5000, names=names)
                    proof["newlyBoundConstraints"] = names
                    proof["otherConstraintsInheritedFromCompleteDomain"] = 84
                    proofs.append(proof)
                passed = all(p["status"] == "numerical_motion_bound_pass" for p in proofs)
                attempts.append(dict(anchorSoftwareDeg=anchor.tolist(), proofsPerMapping=proofs, passed=passed))
                if passed:
                    selected = anchor
                    break
        if selected is not None:
            assert np.all((selected >= command[:, 0]) & (selected <= command[:, 1]))
            assert np.all(abs(q[1:]-selected[1:]) >= np.array([3., 1., 10., 1.])-1e-12)
        inside = bool(np.all((q >= command[:, 0]) & (q <= command[:, 1])))
        result = dict(sampleIndex=row["sampleIndex"], targetSoftwareDeg=q.tolist(),
                      selectedAnchorSoftwareDeg=None if selected is None else selected.tolist(),
                      fourAxisSpanDeg=None if selected is None else abs(q[1:]-selected[1:]).tolist(),
                      nominalBothMappingsPassed=selected is not None,
                      originalDynamicInsetSatisfied=inside, attempts=attempts,
                      inheritedRouteFile=str(checkpoint.resolve()) if checkpoint and checkpoint.is_file() else None)
        write(out / f"route_{index:03d}.json", result)
        results.append(result)
        if selected is not None and inside:
            eligible.append(result)
        print(f"Frontier coupled route {index+1}/{frontier['slices']}: "
              f"proved={selected is not None}, dynamic inset={inside}, attempts={len(attempts)}", flush=True)
    # ponytail: twelve farthest-point targets are representative, not full-domain dynamics.
    remaining, selected = list(eligible), []
    while remaining and len(selected) < 12:
        row = remaining[0] if not selected else max(remaining, key=lambda r: min(
            float(np.linalg.norm((np.asarray(r["targetSoftwareDeg"])-s["targetSoftwareDeg"])
                                 /[1., 82., 202., 454., 39.])) for s in selected))
        remaining.remove(row)
        selected.append(row)
    assert selected, "No complete new-region coordinated route: do not replace with old-region tests"
    for name, digest in identities.items():
        with Path(name).open("rb") as stream:
            assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, name
    for table, config in zip(tables, configs):
        mapping = table["mapping"]
        original = build / f"workspace_outer_rule_20261117_{mapping}_v1"
        geometry = read(original / "geometry.json")
        geometry_path = original / "geometry.json"
        own_identities = dict(identities)
        own_identities[str(geometry_path.resolve())] = hashlib.sha256(geometry_path.read_bytes()).hexdigest()
        offset = np.asarray(config["offsetCadDeg"])
        geometry["paths"] = [
            dict(id=f"frontier_{row['sampleIndex']}_return",
                 waypointsCadDeg=(np.asarray([row["selectedAnchorSoftwareDeg"],
                                              row["targetSoftwareDeg"],
                                              row["selectedAnchorSoftwareDeg"]])+offset).tolist())
            for row in selected]
        own = out / mapping
        own.mkdir()
        write(own / "config.json", config)
        write(own / "geometry.json", geometry)
        write(own / "study.json", dict(
            parameters=[dict(type="frontier_four_axis_lines")], selectedRoutes=selected,
            inputFileSha256=own_identities,
            implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            verifierSha256=hashlib.sha256(Path(__file__).with_name("workspace_verify.py").read_bytes()).hexdigest(),
            constraintsAccountedFor=89, completeOtherGroupsInherited=7,
            constraintScopesPerMapping=scopes,
            limitations=["Two unregistered mappings; nominal J1 zero; actual J1 retained by independent FCL",
                         "Full 89-constraint numerical line bounds, no surrounding region admission",
                         "12 diverse new-domain loops, not all 85 targets or full-domain dynamics",
                         "Uncalibrated geometry, load, friction and tracking; no physical safety guarantee"]))
    write(out / "frontier_paths.json", dict(
        testedTargets=len(results), nominalBothMappingsPassed=sum(r["nominalBothMappingsPassed"] for r in results),
        dynamicEligible=len(eligible), selectedRoutes=selected,
        selectedSampleIndices=[r["sampleIndex"] for r in selected],
        results=results, inputFileSha256=identities,
        constraintsAccountedFor=89, completeOtherGroupsInherited=7,
        constraintScopesPerMapping=scopes,
        resumedRouteReports=sum(r["inheritedRouteFile"] is not None for r in results),
        implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        verifierSha256=hashlib.sha256(Path(__file__).with_name("workspace_verify.py").read_bytes()).hexdigest(),
        elapsedSeconds=time.perf_counter()-started))
    print(dict(targets=len(results), nominalPassed=sum(r["nominalBothMappingsPassed"] for r in results),
               dynamicEligible=len(eligible), selectedRoutes=len(selected),
               plannedCasesBothMappings=4*(len(selected)+6)), flush=True)


def frontier_paths_audit(build, extension, source):
    """Audit new-domain entry and actual joint travel; FCL/finish remain separate gates."""
    tables, identities = joint3_tables(build, extension)
    study, config = read(source / "study.json"), read(source / "config.json")
    for name, digest in study["inputFileSha256"].items():
        with Path(name).open("rb") as stream:
            assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, name
    trace, dynamics = read(source / "actual_trace.json"), read(source / "dynamics.json")
    points = np.asarray([row["cadDegrees"] for row in trace["samples"]])-config["offsetCadDeg"]
    j1 = float(abs(points[:, 0]).max())
    points[:, 0] = 0.  # Old-domain comparison only; actual five-axis FCL is separate.
    old = np.ones(len(points), dtype=bool)
    for table in tables:
        domain = np.asarray(table["admissibleSoftwareDomainDeg"])
        old &= factored_membership(table["groups"], points)
        old &= np.all((points >= domain[:, 0]) & (points <= domain[:, 1]), axis=1)
    labels = np.asarray([row["case"] for row in trace["samples"]])
    rows = []
    for route in study["selectedRoutes"]:
        a, b = np.asarray(route["selectedAnchorSoftwareDeg"]), np.asarray(route["targetSoftwareDeg"])
        for speed in ("slow", "brisk"):
            label = f"{speed}/frontier_{route['sampleIndex']}_return"
            case = next(c for c in dynamics["cases"] if c["speed"]+"/"+c["id"] == label)
            actual = points[labels == label]
            span = observed_segment_span(actual, a, b, max(case["maxErrorDeg"])+1e-6)
            count = int((~old & (labels == label)).sum())
            assert count, (label, "Actual trajectory did not enter previous-table exterior")
            rows.append(dict(case=label, previousTableExteriorSamples=count, observed=span))
    identities.update(study["inputFileSha256"])
    for name in ("study.json", "config.json", "geometry.json", "actual_trace.json", "dynamics.json"):
        path = source / name
        identities[str(path.resolve())] = hashlib.sha256(path.read_bytes()).hexdigest()
    write(source / "frontier_actual_audit.json", dict(
        samples=len(points), originalJ1MaximumActualErrorDeg=j1, routes=rows,
        previousTableExteriorSamples=int((~old).sum()), inputFileSha256=identities,
        implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        note="Actual new-domain entry and spans only; no 4-D region or tracking-tube certificate"))
    print(dict(samples=len(points), routeSpeedLabels=len(rows),
               previousTableExteriorSamples=int((~old).sum()), actualSpansPassed=True), flush=True)


def frontier_context(build, extension, source_path):
    """Reuse the exact inherited 84 constraints; load mesh bounds once per mapping."""
    tables, identities = joint3_tables(build, extension)
    identities[str(source_path.resolve())] = hashlib.sha256(source_path.read_bytes()).hexdigest()
    configs, checkers, scopes = [], [], []
    for table in tables:
        own = extension / table["mapping"]
        parent = read(own / "gap_refinement.json")
        assert parent["constraintsAccountedFor"] == 89 and parent["completeOtherGroupsInherited"] == 7
        identities.update(parent["inputFileSha256"])
        path = own / "config.json"
        identities[str(path.resolve())] = hashlib.sha256(path.read_bytes()).hexdigest()
        configs.append(read(path))
        checkers.append(BoundsChecker(own))
        scopes.append(inherited_constraint_scope(table, checkers[-1], configs[-1], identities))
    for name, digest in identities.items():
        with Path(name).open("rb") as stream:
            assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, name
    domains = np.asarray([table["admissibleSoftwareDomainDeg"] for table in tables])
    domain = np.column_stack((domains[:, :, 0].max(axis=0), domains[:, :, 1].min(axis=0)))
    return tables, identities, configs, checkers, scopes, domain


def inflate_faces(box, domain, admit):
    """Coordinate-wise certified inflation, not a global-volume optimum."""
    best, reference = np.asarray(box).copy(), None
    for axis in (1, 4, 3, 2):
        for side in (0, 1):
            initial, safe, unsafe = best[axis, side], 0., 1.
            for iteration in range(7):
                phase = 1. if iteration == 0 else (safe+unsafe)/2
                candidate = best.copy()
                candidate[axis, side] = initial+phase*(domain[axis, side]-initial)
                result = admit(candidate)
                if result is None:
                    unsafe = phase
                else:
                    best, reference, safe = candidate, result, phase
                    if phase == 1.:
                        break
    return best, reference


def inherited_box_partition(bounds, known, names):
    """Exact rectangle subtraction; every inherited leaf names its original safe box."""
    bounds, known = np.asarray(bounds, dtype=float), np.asarray(known, dtype=float).reshape(-1, 5, 2)
    if (bounds.shape != (5, 2) or not np.isfinite(bounds).all() or not np.isfinite(known).all()
            or np.any(bounds[:, 0] > bounds[:, 1]) or np.any(known[:, :, 0] > known[:, :, 1])):
        raise ValueError("Invalid inheritance rectangles")
    active = bounds[:, 1] > bounds[:, 0]
    if not active.any():
        raise ValueError("Inheritance requires at least one active axis")
    clipped = np.stack((np.maximum(known[:, :, 0], bounds[:, 0]),
                        np.minimum(known[:, :, 1], bounds[:, 1])), axis=2)
    valid = (np.all(clipped[:, :, 1] >= clipped[:, :, 0], axis=1)
             & np.all(clipped[:, active, 1] > clipped[:, active, 0], axis=1))
    indices = np.flatnonzero(valid)
    volumes = np.prod(clipped[indices][:, active, 1]-clipped[indices][:, active, 0], axis=1)
    indices = indices[np.argsort(-volumes, kind="stable")]
    inherited, pending = [], [dict(cadBoundsDeg=bounds.tolist(), unresolved=list(names))]
    for index in indices:
        selected, pending = focus_seeds(pending, [clipped[index]])
        for row in selected:
            leaf = np.asarray(row["cadBoundsDeg"])
            assert np.all(leaf[:, 0] >= known[index, :, 0]) and np.all(leaf[:, 1] <= known[index, :, 1])
            inherited.append(dict(cadBoundsDeg=leaf.tolist(), boundSlackMm=None,
                                  inheritedSafeBoxIndex=int(index)))
        if not pending:
            break
    # This checks the partition only; pending pieces are NOT certified collision-free.
    audit_partition_cover(dict(verified=inherited+pending, unknown=[]), bounds)
    return inherited, pending


def frontier_balanced(build, extension, source, out):
    """Balance all four widths, inheriting old safe subregions instead of reproving them."""
    started = time.perf_counter()
    source_path = source / "frontier_paths.json"
    previous = read(source_path)
    tables, identities, configs, checkers, scopes, domain = frontier_context(build, extension, source_path)
    identities.update(previous["inputFileSha256"])
    survey_path = build / "workspace_frontier_survey_20261117_v1/frontier_survey.json"
    survey = read(survey_path)
    identities[str(survey_path.resolve())] = hashlib.sha256(survey_path.read_bytes()).hexdigest()
    for name, digest in identities.items():
        with Path(name).open("rb") as stream:
            assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, name
    free = [row for row in survey["pointRows"] if row["state"] == "inside_clear"]
    obstacles = [row for row in survey["pointRows"] if row["state"] == "inside_collision"]
    candidates = [row for row in previous["results"] if row["nominalBothMappingsPassed"]
                  and row["originalDynamicInsetSatisfied"]]
    assert len(candidates) == 77 and len(free) == 85 and not survey["admittedRegressionViolations"]
    known = [table["groups"][-1]["verifiedSoftwareBoxesDeg"]+np.asarray(config["offsetCadDeg"])[None, :, None]
             for table, config in zip(tables, configs)]
    out.mkdir(parents=True, exist_ok=False)
    attempts, cache, initial, windows = [], {}, [], []
    def covered(box):
        return {row["sampleIndex"] for row in free if np.all(
            (row["softwareDeg"] >= box[:, 0]) & (row["softwareDeg"] <= box[:, 1]))}
    def prove(box, seed, phase):
        assert np.all(box[:, 0] >= domain[:, 0]) and np.all(box[:, 1] <= domain[:, 1])
        key = tuple(box.ravel())
        if key in cache:
            index = cache[key]
            return index if attempts[index]["wholeBoxProven"] else None
        witnesses = [row["sampleIndex"] for row in obstacles if np.all(
            (row["softwareDeg"] >= box[:, 0]) & (row["softwareDeg"] <= box[:, 1]))]
        proofs = []
        if not witnesses:
            for checker, config, scope, old in zip(checkers, configs, scopes, known):
                cad = box+np.asarray(config["offsetCadDeg"])[:, None]
                inherited, pending = inherited_box_partition(cad, old, scope["newlyBoundConstraints"])
                proof = partition_regions(checker, cad, scope["newlyBoundConstraints"], 5000,
                                          min_width=.01, max_seconds=1., seeds=pending)
                proof["newlyVerifiedLeaves"] = len(proof["verified"])
                proof["inheritedLeaves"] = len(inherited)
                proof["inheritedSourceBoxCount"] = len(old)
                proof["verified"] = inherited+proof["verified"]
                proof["verifiedFraction"] = 1.-proof["unknownFraction"]
                proofs.append(proof)
        passed = len(proofs) == 2 and all(not proof["unknown"] for proof in proofs)
        if passed:
            for proof, config in zip(proofs, configs):
                audit_partition_cover(proof, box+np.asarray(config["offsetCadDeg"])[:, None])
        index = len(attempts)
        path = out / f"attempt_{index:04d}.json"
        write(path, dict(softwareBoundsDeg=box.tolist(), seedSampleIndex=seed, phase=phase,
                         proofsPerMapping=proofs, collisionWitnessSampleIndices=witnesses,
                         wholeBoxProven=passed))
        attempts.append(dict(index=index, report=str(path.resolve()), seedSampleIndex=seed,
                             phase=phase, wholeBoxProven=passed, collisionWitnessSampleIndices=witnesses))
        cache[key] = index
        print(f"Balanced {index}: seed={seed}, phase={phase}, passed={passed}, "
              f"inherited/new/unknown={[(p['inheritedLeaves'], p['newlyVerifiedLeaves'], len(p['unknown'])) for p in proofs]}",
              flush=True)
        return index if passed else None
    for row in candidates:
        anchor, target = np.asarray(row["selectedAnchorSoftwareDeg"]), np.asarray(row["targetSoftwareDeg"])
        assert interval_contains(joint3_query(tables, anchor)[0], anchor[2])
        assert not interval_contains(joint3_query(tables, target)[0], target[2])
        padding = np.array([0., .2, .2, .2, .2])
        box = np.column_stack((np.maximum(np.minimum(anchor, target)-padding, domain[:, 0]),
                               np.minimum(np.maximum(anchor, target)+padding, domain[:, 1])))
        reference = prove(box, row["sampleIndex"], "initial")
        if reference is not None:
            initial.append(dict(row=row, box=box, reference=reference, covered=covered(box)))
    remaining, represented = list(initial), set()
    step = np.array([0., 1., 1., 5., 1.])
    while remaining and len(windows) < 8:
        choice = max(remaining, key=lambda item: (
            len(item["covered"]-represented), float(np.prod(np.diff(item["box"], axis=1)[1:]))))
        remaining.remove(choice)
        row, base, reference = choice["row"], choice["box"], choice["reference"]
        best, safe, unsafe = base.copy(), 0., None
        for scale in (1., 2., 4., 8., 16.):
            box = np.column_stack((np.maximum(base[:, 0]-scale*step, domain[:, 0]),
                                   np.minimum(base[:, 1]+scale*step, domain[:, 1])))
            result = prove(box, row["sampleIndex"], scale)
            if result is None:
                unsafe = scale
                break
            best, reference, safe = box, result, scale
        if unsafe is not None:
            for _ in range(6):
                scale = (safe+unsafe)/2
                box = np.column_stack((np.maximum(base[:, 0]-scale*step, domain[:, 0]),
                                       np.minimum(base[:, 1]+scale*step, domain[:, 1])))
                result = prove(box, row["sampleIndex"], scale)
                if result is None:
                    unsafe = scale
                else:
                    best, reference, safe = box, result, scale
        represented.update(covered(best))
        windows.append(dict(index=len(windows), category=f"balanced_{row['sampleIndex']}",
                            seedSampleIndex=row["sampleIndex"], focusSoftwareBoundsDeg=best.tolist(),
                            selectedAttempt=reference, simultaneousInflationScale=safe,
                            anchorCandidatesSoftwareDeg=[row["selectedAnchorSoftwareDeg"]],
                            anchorSource="previous_common_table_slice",
                            coveredFrontierSampleIndices=sorted(covered(best))))
    for name, digest in identities.items():
        with Path(name).open("rb") as stream:
            assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, name
    for index, (table, config) in enumerate(zip(tables, configs)):
        parent = read(extension / table["mapping"] / "gap_refinement.json")
        own = out / table["mapping"]
        own.mkdir()
        write(own / "config.json", config)
        write(own / "gap_refinement.json", dict(
            parent, refinementMode="frontier_balanced_common",
            inheritedVerifiedBoxes=len(parent["verifiedSoftwareBoxesDeg"]),
            verifiedSoftwareBoxesDeg=parent["verifiedSoftwareBoxesDeg"]+[w["focusSoftwareBoundsDeg"] for w in windows],
            windows=[dict(w, refinement=read(Path(attempts[w["selectedAttempt"]]["report"]))[
                "proofsPerMapping"][index]) for w in windows],
            inflationAttempts=attempts, constraintScopesPerMapping=scopes,
            inheritedSubregionSource=str(extension.resolve()), inputFileSha256=identities,
            implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            elapsedSeconds=time.perf_counter()-started))
    write(out / "frontier_balanced.json", dict(
        initialCandidates=len(candidates), initialWholeBoxesPassed=len(initial),
        windows=windows, attempts=attempts, coveredFrontierSampleIndices=sorted(represented),
        inputFileSha256=identities, constraintsAccountedFor=89, completeOtherGroupsInherited=7,
        budget=dict(perMapMaxNodes=5000, perMapSeconds=1., minWidthDeg=.01,
                    simultaneousStepDeg=step.tolist(), growthScales=[1, 2, 4, 8, 16], bisectionTrials=6),
        implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        verifierSha256=hashlib.sha256(Path(__file__).with_name("workspace_verify.py").read_bytes()).hexdigest(),
        elapsedSeconds=time.perf_counter()-started,
        limitations=["Whole-box numerical bounds, not formal arithmetic, physical calibration or a global maximum",
                     "Only previously hashed safe boxes may discharge inherited leaf constraints",
                     "Selected dynamic boundary loops and actual-trace membership remain separate gates"]))
    print(dict(boxes=len(windows), initialPasses=len(initial), attempts=len(attempts),
               coveredFrontier=len(represented), elapsedSeconds=time.perf_counter()-started), flush=True)


def frontier_grow(build, extension, source, out):
    """Inflate actual 4-D neighborhoods around new free targets; retain all old boxes."""
    started = time.perf_counter()
    source_path = source / "frontier_survey.json"
    survey = read(source_path)
    assert not survey["admittedRegressionViolations"]
    tables, identities, configs, checkers, scopes, domain = frontier_context(build, extension, source_path)
    free = [row for row in survey["pointRows"] if row["state"] == "inside_clear"]
    obstacles = [row for row in survey["pointRows"] if row["state"] == "inside_collision"]
    remaining, selected = list(free), []
    while remaining and len(selected) < 8:
        row = remaining[0] if not selected else max(remaining, key=lambda r: min(
            np.linalg.norm((np.asarray(r["softwareDeg"])-s["softwareDeg"])[1:]
                           /np.diff(domain, axis=1).ravel()[1:]) for s in selected))
        remaining.remove(row)
        selected.append(row)
    out.mkdir(parents=True, exist_ok=False)
    attempts, windows = [], []
    def prove(box):
        witnesses = [row["sampleIndex"] for row in obstacles if np.all(
            (row["softwareDeg"] >= box[:, 0]) & (row["softwareDeg"] <= box[:, 1]))]
        proofs = []
        if not witnesses:
            for checker, config, scope in zip(checkers, configs, scopes):
                cad = box+np.asarray(config["offsetCadDeg"])[:, None]
                proof = partition_regions(checker, cad, scope["newlyBoundConstraints"], 2000,
                                          min_width=.01, max_seconds=.4)
                proofs.append(proof)
        passed = len(proofs) == 2 and all(not proof["unknown"] for proof in proofs)
        if passed:
            for proof, config in zip(proofs, configs):
                audit_partition_cover(proof, box+np.asarray(config["offsetCadDeg"])[:, None])
        index = len(attempts)
        path = out / f"attempt_{index:04d}.json"
        write(path, dict(softwareBoundsDeg=box.tolist(), proofsPerMapping=proofs,
                         collisionWitnessSampleIndices=witnesses, wholeBoxProven=passed))
        attempts.append(dict(index=index, report=str(path.resolve()), wholeBoxProven=passed,
                             collisionWitnessSampleIndices=witnesses))
        return index if passed else None
    for row in selected:
        q = np.asarray(row["softwareDeg"])
        half = np.minimum([0., 1., 1., 8., 1.],
                          .99*np.minimum(q-domain[:, 0], domain[:, 1]-q))
        factor = 1.
        for checker, config, scope in zip(checkers, configs, scopes):
            cad = q+config["offsetCadDeg"]
            checker.place(cad)
            for name in scope["newlyBoundConstraints"]:
                distance = checker.lower_distance(name, cad)
                movement = float(np.dot(checker.constraints[name]["radius"], np.radians(half)))
                assert distance > 1e-6, (row["sampleIndex"], name, distance)
                if movement:
                    factor = min(factor, .45*(distance-1e-6)/movement)
        half *= factor
        box = np.column_stack((q-half, q+half))
        assert np.all(np.diff(box, axis=1).ravel()[1:] > 0)
        reference = prove(box)
        assert reference is not None, "Conservative seed proposal did not pass the actual verifier"
        grown, last = inflate_faces(box, domain, prove)
        reference = reference if last is None else last
        saved = read(Path(attempts[reference]["report"]))
        assert saved["wholeBoxProven"] and np.array_equal(saved["softwareBoundsDeg"], grown)
        command = np.asarray(configs[0]["recordedSoftwareRangesDeg"]).copy()
        command[1:, 0] += 1
        command[1:, 1] -= 1
        low = np.maximum(grown[:, 0]+[0., .1, .1, .1, .1], command[:, 0])
        high = np.minimum(grown[:, 1]-[0., .1, .1, .1, .1], command[:, 1])
        anchors = []
        if np.all(low <= high):
            for values in itertools.product(*((low[j], (low[j]+high[j])/2, high[j]) for j in (1, 3, 4))):
                point = np.array([0., values[0], 0., values[1], values[2]])
                intervals = intersect_intervals(joint3_query(tables, point)[0], [[low[2], high[2]]])
                if intervals:
                    point[2] = np.mean(max(intervals, key=lambda p: p[1]-p[0]))
                    anchors.append(point.tolist())
                    break
        windows.append(dict(index=len(windows), category=f"inflated_{row['sampleIndex']}",
                            seedSampleIndex=row["sampleIndex"], focusSoftwareBoundsDeg=grown.tolist(),
                            selectedAttempt=reference, anchorCandidatesSoftwareDeg=anchors,
                            anchorSource="previous_common_table_slice"))
        print(f"Inflated seed {row['sampleIndex']}: spans={np.diff(grown, axis=1).ravel()[1:].tolist()}, "
              f"attempts={len(attempts)}, old anchor={bool(anchors)}", flush=True)
    for name, digest in identities.items():
        with Path(name).open("rb") as stream:
            assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, name
    for index, (table, config) in enumerate(zip(tables, configs)):
        parent = read(extension / table["mapping"] / "gap_refinement.json")
        own = out / table["mapping"]
        own.mkdir()
        write(own / "config.json", config)
        write(own / "gap_refinement.json", dict(
            parent, refinementMode="frontier_inflated_common",
            inheritedVerifiedBoxes=len(parent["verifiedSoftwareBoxesDeg"]),
            verifiedSoftwareBoxesDeg=parent["verifiedSoftwareBoxesDeg"]+[w["focusSoftwareBoundsDeg"] for w in windows],
            windows=[dict(w, refinement=read(Path(attempts[w["selectedAttempt"]]["report"]))[
                "proofsPerMapping"][index]) for w in windows],
            inflationAttempts=attempts, constraintScopesPerMapping=scopes,
            inputFileSha256=identities, implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            elapsedSeconds=time.perf_counter()-started))
    write(out / "frontier_growth.json", dict(
        selectedSeedSampleIndices=[r["sampleIndex"] for r in selected], windows=windows, attempts=attempts,
        inputFileSha256=identities, constraintsAccountedFor=89, completeOtherGroupsInherited=7,
        budget=dict(perMapMaxNodes=2000, perMapSeconds=.4, faceTrials=7, axisOrder=[2, 5, 4, 3]),
        implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        elapsedSeconds=time.perf_counter()-started,
        limitations=["Coordinate-wise bounded-budget inflation, not global maximum or physical calibration",
                     "Static whole-box proof only; dynamics and actual-trace membership are separate gates"]))
    print(dict(boxes=len(windows), attempts=len(attempts), elapsedSeconds=time.perf_counter()-started), flush=True)


def frontier_collect(build, source, out):
    """Retain every proved initial neighborhood; eight enlarged boxes are dynamic representatives."""
    path = source / "frontier_balanced.json"
    report = read(path)
    identities = dict(report["inputFileSha256"])
    identities[str(path.resolve())] = hashlib.sha256(path.read_bytes()).hexdigest()
    paths_file = build / "workspace_frontier_paths_20261117_v2/frontier_paths.json"
    assert identities[str(paths_file.resolve())] == hashlib.sha256(paths_file.read_bytes()).hexdigest()
    routes = {row["sampleIndex"]: row for row in read(paths_file)["results"]}
    windows = [dict(window, boundaryRepresentative=True) for window in report["windows"]]
    proofs, initial_boxes = {}, []
    for attempt in report["attempts"]:
        if not attempt["wholeBoxProven"] or attempt["phase"] != "initial":
            continue
        attempt_path = Path(attempt["report"])
        item = read(attempt_path)
        identities[str(attempt_path.resolve())] = hashlib.sha256(attempt_path.read_bytes()).hexdigest()
        assert item["wholeBoxProven"] and item["phase"] == "initial"
        box = np.asarray(item["softwareBoundsDeg"])
        assert len(item["proofsPerMapping"]) == 2 and all(
            not proof["unknown"] for proof in item["proofsPerMapping"])
        initial_boxes.append(box)
        if any(np.all(box[:, 0] >= np.asarray(w["focusSoftwareBoundsDeg"])[:, 0]) and
               np.all(box[:, 1] <= np.asarray(w["focusSoftwareBoundsDeg"])[:, 1]) for w in windows):
            continue  # Exact containment, not a sampled or rounded simplification.
        seed = item["seedSampleIndex"]
        category = f"initial_{seed}"
        proofs[category] = item["proofsPerMapping"]
        windows.append(dict(index=len(windows), category=category, seedSampleIndex=seed,
                            focusSoftwareBoundsDeg=box.tolist(), selectedAttempt=attempt["index"],
                            anchorCandidatesSoftwareDeg=[routes[seed]["selectedAnchorSoftwareDeg"]],
                            anchorSource="previous_common_table_slice", boundaryRepresentative=False))
    assert len(initial_boxes) == report["initialWholeBoxesPassed"]
    assert all(any(np.all(box[:, 0] >= np.asarray(w["focusSoftwareBoundsDeg"])[:, 0]) and
                   np.all(box[:, 1] <= np.asarray(w["focusSoftwareBoundsDeg"])[:, 1])
                   for w in windows) for box in initial_boxes)
    for mapping in ("baseline", "alignment"):
        own_path = source / mapping / "gap_refinement.json"
        identities[str(own_path.resolve())] = hashlib.sha256(own_path.read_bytes()).hexdigest()
    for name, digest in identities.items():
        with Path(name).open("rb") as stream:
            assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, name
    out.mkdir(parents=True, exist_ok=False)
    for index, mapping in enumerate(("baseline", "alignment")):
        parent = read(source / mapping / "gap_refinement.json")
        old_windows = {window["category"]: window for window in parent["windows"]}
        assembled = [dict(window, refinement=(old_windows[window["category"]]["refinement"]
                     if window["boundaryRepresentative"] else proofs[window["category"]][index]))
                     for window in windows]
        own = out / mapping
        own.mkdir()
        write(own / "config.json", read(source / mapping / "config.json"))
        write(own / "gap_refinement.json", dict(
            parent, windows=assembled,
            verifiedSoftwareBoxesDeg=parent["verifiedSoftwareBoxesDeg"][:parent["inheritedVerifiedBoxes"]]+[
                window["focusSoftwareBoundsDeg"] for window in windows],
            allInitialSafeNeighborhoodsRetained=True, inputFileSha256=identities,
            implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()))
    write(out / "frontier_collection.json", dict(
        windows=windows, initialWholeBoxesPassed=report["initialWholeBoxesPassed"],
        allInitialSafeNeighborhoodsRetained=True, inputFileSha256=identities,
        dynamicRepresentativeBoxes=sum(window["boundaryRepresentative"] for window in windows),
        implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        note="Every initial whole-box proof retained directly or by exact containing enlarged box; dynamics remain representative"))
    print(dict(retainedBoxes=len(windows), initialWholeBoxesPassed=report["initialWholeBoxesPassed"],
               dynamicRepresentativeBoxes=sum(window["boundaryRepresentative"] for window in windows)), flush=True)


def connection_options(a, b):
    choices = [(None, np.array([a, b]))]
    for height in (10., 30., 60.):
        ra, rb = a.copy(), b.copy()
        ra[2], rb[2] = height-1.5*a[1], height-1.5*b[1]
        choices.append((height, np.array([a, ra, rb, b])))
    return choices


def frontier_connect(build, extension, source, out):
    """Prove long inter-target travel; failed direct lines remain explicit evidence."""
    started = time.perf_counter()
    source_path = source / "frontier_paths.json"
    previous = read(source_path)
    tables, identities, configs, checkers, scopes, domain = frontier_context(build, extension, source_path)
    points = [r for r in previous["results"] if r["nominalBothMappingsPassed"]
              and r["originalDynamicInsetSatisfied"]]
    pairs = []
    for a, b in itertools.combinations(points, 2):
        delta = abs(np.asarray(a["targetSoftwareDeg"])[1:]-np.asarray(b["targetSoftwareDeg"])[1:])
        if np.all(delta >= [10., 10., 60., 4.]):
            pairs.append((float(np.linalg.norm(delta/[82., 202., 454., 39.])), a, b))
    pairs.sort(key=lambda row: -row[0])
    # ponytail: 24 strongest diverse pairs, not a complete all-pairs roadmap.
    chosen, used = [], set()
    for _, a, b in pairs:
        if a["sampleIndex"] in used and b["sampleIndex"] in used:
            continue
        chosen.append((a, b))
        used.update((a["sampleIndex"], b["sampleIndex"]))
        if len(chosen) == 24:
            break
    command = np.asarray(configs[0]["recordedSoftwareRangesDeg"]).copy()
    command[1:, 0] += 1
    command[1:, 1] -= 1
    out.mkdir(parents=True, exist_ok=False)
    results, selected = [], []
    for arow, brow in chosen:
        a, b = np.asarray(arow["targetSoftwareDeg"]), np.asarray(brow["targetSoftwareDeg"])
        attempts, accepted = [], None
        for height, route in connection_options(a, b):
            inside = bool(np.all((route >= domain[:, 0]) & (route <= domain[:, 1]))
                          and np.all((route >= command[:, 0]) & (route <= command[:, 1])))
            proofs = []
            if inside:
                for checker, config, scope in zip(checkers, configs, scopes):
                    own = []
                    for start, end in zip(route, route[1:]):
                        own.append(check_curve(checker, lambda t: start+t*(end-start)+config["offsetCadDeg"],
                                               max_nodes=5000, names=scope["newlyBoundConstraints"]))
                        if own[-1]["status"] != "numerical_motion_bound_pass":
                            break
                    proofs.append(own)
            passed = inside and len(proofs) == 2 and all(
                len(own) == len(route)-1 and all(p["status"] == "numerical_motion_bound_pass" for p in own)
                for own in proofs)
            attempts.append(dict(transitHeightDeg=height, waypointsSoftwareDeg=route.tolist(),
                                 withinDomainAndInset=inside, proofsPerMapping=proofs, passed=passed))
            if passed:
                accepted = route
                break
        row = dict(pairSampleIndices=[arow["sampleIndex"], brow["sampleIndex"]], attempts=attempts,
                   nominalBothMappingsPassed=accepted is not None)
        if accepted is not None:
            middle = accepted if len(accepted) == 2 else accepted[1:3]
            row.update(sampleIndex=f"{arow['sampleIndex']}_{brow['sampleIndex']}",
                       selectedAnchorSoftwareDeg=middle[0].tolist(), targetSoftwareDeg=middle[1].tolist(),
                       fourAxisSpanDeg=abs(middle[1, 1:]-middle[0, 1:]).tolist(),
                       loopSoftwareDeg=np.vstack((accepted, accepted[-2::-1])).tolist(),
                       transitHeightDeg=attempts[-1]["transitHeightDeg"])
            assert np.all(np.asarray(row["fourAxisSpanDeg"]) >= [10., 10., 60., 4.])
            if len(selected) < 12:
                selected.append(row)
        write(out / f"connection_{len(results):03d}.json", row)
        results.append(row)
        print(f"Long connection {len(results)}/{len(chosen)}: "
              f"{row['pairSampleIndices']}, passed={accepted is not None}, attempts={len(attempts)}", flush=True)
    assert selected, "No new long coordinated path; no old-domain replacement"
    for name, digest in identities.items():
        with Path(name).open("rb") as stream:
            assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, name
    for table, config in zip(tables, configs):
        path = build / f"workspace_outer_rule_20261117_{table['mapping']}_v1/geometry.json"
        geometry = read(path)
        own_identities = dict(identities)
        own_identities[str(path.resolve())] = hashlib.sha256(path.read_bytes()).hexdigest()
        geometry["paths"] = [dict(id=f"frontier_{r['sampleIndex']}_return",
            waypointsCadDeg=(np.asarray(r["loopSoftwareDeg"])+config["offsetCadDeg"]).tolist()) for r in selected]
        own = out / table["mapping"]
        own.mkdir()
        write(own / "config.json", config)
        write(own / "geometry.json", geometry)
        write(own / "study.json", dict(
            parameters=[dict(type="long_frontier_connections")], selectedRoutes=selected,
            inputFileSha256=own_identities, constraintScopesPerMapping=scopes,
            constraintsAccountedFor=89, completeOtherGroupsInherited=7,
            implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            verifierSha256=hashlib.sha256(Path(__file__).with_name("workspace_verify.py").read_bytes()).hexdigest(),
            limitations=["Whole nominal lines only; not arbitrary endpoints or neighboring volume",
                         "Two unregistered mappings, numerical geometry, no physical safety guarantee"]))
    write(out / "frontier_connections.json", dict(
        eligiblePairs=len(pairs), testedPairs=len(results), results=results, selectedRoutes=selected,
        directPassed=sum(r["attempts"][0]["passed"] for r in results),
        detourPassed=sum(r["nominalBothMappingsPassed"] and not r["attempts"][0]["passed"] for r in results),
        inputFileSha256=identities, constraintScopesPerMapping=scopes,
        implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        elapsedSeconds=time.perf_counter()-started))
    print(dict(testedPairs=len(results), selected=len(selected), elapsedSeconds=time.perf_counter()-started), flush=True)


def frontier_survey(build, extension, out):
    """Diagnose remaining recorded-domain states, never infer a free region from a point."""
    started, identities, checkers, configs = time.perf_counter(), {}, [], []
    tables, table_identities = joint3_tables(build, extension)
    identities.update(table_identities)
    for table in tables:
        own = extension / table["mapping"]
        identities.update(read(own / "gap_refinement.json")["inputFileSha256"])
        config_path = own / "config.json"
        configs.append(read(config_path))
        identities[str(config_path.resolve())] = hashlib.sha256(config_path.read_bytes()).hexdigest()
        checkers.append(Checker(own))
    for name, digest in identities.items():
        with Path(name).open("rb") as stream:
            assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, name
    recorded = np.asarray(configs[0]["recordedSoftwareRangesDeg"])
    assert np.array_equal(recorded, configs[1]["recordedSoftwareRangesDeg"])
    points = np.random.default_rng(2026111730).uniform(recorded[:, 0], recorded[:, 1], (6000, 5))
    inside, allowed = np.ones(len(points), dtype=bool), np.ones(len(points), dtype=bool)
    for table in tables:
        domain = np.asarray(table["admissibleSoftwareDomainDeg"])
        inside &= np.all((points >= domain[:, 0]) & (points <= domain[:, 1]), axis=1)
        allowed &= factored_membership(table["groups"], points)
    allowed &= inside
    remaining = np.flatnonzero(~allowed)
    regression = np.flatnonzero(allowed)[:200]
    rows, violations, counts, pairs = [], [], Counter(), [Counter(), Counter()]
    for checked, index in enumerate(np.r_[remaining, regression]):
        hits = [checker.hits(points[index]+config["offsetCadDeg"])
                for checker, config in zip(checkers, configs)]
        if allowed[index]:
            if any(hits):
                violations.append(dict(sampleIndex=int(index), softwareDeg=points[index].tolist(),
                                       collisionPairsPerMapping=hits))
        else:
            state = ("inside" if inside[index] else "outside") + ("_collision" if any(hits) else "_clear")
            counts[state] += 1
            for counter, found in zip(pairs, hits):
                counter.update(found)
            rows.append(dict(sampleIndex=int(index), softwareDeg=points[index].tolist(), state=state,
                             collisionPairsPerMapping=hits))
        if checked % 100 == 0:
            print(f"Frontier states {checked}/{len(remaining)+len(regression)}: {dict(counts)}", flush=True)
    assert sum(counts.values()) == len(remaining) == len(points)-int(allowed.sum())
    for name, digest in identities.items():
        with Path(name).open("rb") as stream:
            assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, name
    out.mkdir(parents=True, exist_ok=False)
    write(out / "frontier_survey.json", dict(seed=2026111730, recordedSamples=len(points),
          previouslyAllowed=int(allowed.sum()), insideCommonDomain=int(inside.sum()),
          remainingStates=len(remaining), classifications=dict(counts), pointRows=rows,
          collisionPairCountsPerMapping=[dict(counter) for counter in pairs],
          admittedRegressionSamples=len(regression), admittedRegressionViolations=violations,
          inputFileSha256=identities, implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
          elapsedSeconds=time.perf_counter()-started,
          limitations=["Finite full-triangle/floor states only; neither whole-region rejection nor admission",
                       "Outside current domain is not synonymous with collision or physical impossibility",
                       "No source-record, mapping, controller, limits or hardware changes"]))
    assert not violations, violations
    print(dict(previouslyAllowed=int(allowed.sum()), insideCommonDomain=int(inside.sum()),
               classifications=dict(counts), regressionViolations=len(violations)), flush=True)


def frontier_candidates(rows, domain, command_domain):
    """Large-first multiscale candidates; a collision witness only disqualifies whole-box admission."""
    free = [row for row in rows if row["state"] == "inside_clear"]
    points = np.asarray([row["softwareDeg"] for row in free])
    colliding = [row for row in rows if row["state"] == "inside_collision"]
    obstacles = np.asarray([row["softwareDeg"] for row in colliding]).reshape(-1, 5)
    candidates, excluded, seen = [], [], set()
    for row in free:
        center = np.asarray(row["softwareDeg"])
        for scale in (.5, 1., 2., 4., 8.):
            half = scale*np.array([0., 4., 3., 30., 4.])
            box = np.column_stack((np.ceil(np.maximum(domain[:, 0], center-half)*10)/10,
                                   np.floor(np.minimum(domain[:, 1], center+half)*10)/10))
            key = tuple(box.ravel())
            if key in seen or np.any(box[:, 0] > box[:, 1]):
                continue
            seen.add(key)
            low = np.maximum(box[:, 0]+[0., .1, .1, .1, .1], command_domain[:, 0])
            high = np.minimum(box[:, 1]-[0., .1, .1, .1, .1], command_domain[:, 1])
            if np.any((high-low)[1:] < [3., 1., 10., 1.]):
                continue
            covered = np.all((points >= box[:, 0]) & (points <= box[:, 1]), axis=1)
            commanded = np.all((points >= low) & (points <= high), axis=1)
            if not commanded.any():
                continue
            witnesses = np.all((obstacles >= box[:, 0]) & (obstacles <= box[:, 1]), axis=1)
            result = dict(seedSampleIndex=row["sampleIndex"], scale=scale,
                          softwareBoundsDeg=box.tolist(),
                          coveredSampleIndices=[free[i]["sampleIndex"] for i in np.flatnonzero(covered)],
                          commandedWitnessIndices=[free[i]["sampleIndex"] for i in np.flatnonzero(commanded)],
                          volumeDeg4=float(np.prod((box[:, 1]-box[:, 0])[1:])))
            if witnesses.any():
                excluded.append(dict(result, collisionWitnessSampleIndices=[
                    colliding[i]["sampleIndex"] for i in np.flatnonzero(witnesses)]))
            else:
                candidates.append(result)
    return candidates, excluded


def frontier_regions(build, extension, source, out):
    started = time.perf_counter()
    survey_path = source / "frontier_survey.json"
    survey = read(survey_path)
    assert not survey["admittedRegressionViolations"]
    tables, identities = joint3_tables(build, extension)
    identities.update(survey["inputFileSha256"])
    identities[str(survey_path.resolve())] = hashlib.sha256(survey_path.read_bytes()).hexdigest()
    parents, configs, checkers = [], [], []
    for table in tables:
        own = extension / table["mapping"]
        parent = read(own / "gap_refinement.json")
        assert parent["constraintsAccountedFor"] == 89 and parent["completeOtherGroupsInherited"] == 7
        domain = np.asarray(table["admissibleSoftwareDomainDeg"])
        assert all(np.array_equal(group["verifiedSoftwareBoxesDeg"], [domain]) for group in table["groups"][:-1])
        identities.update(parent["inputFileSha256"])
        parents.append(parent)
        configs.append(read(own / "config.json"))
        checkers.append(BoundsChecker(own))
    for name, digest in identities.items():
        with Path(name).open("rb") as stream:
            assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, name
    domain = np.column_stack((np.max([t["admissibleSoftwareDomainDeg"] for t in tables], axis=0)[:, 0],
                              np.min([t["admissibleSoftwareDomainDeg"] for t in tables], axis=0)[:, 1]))
    command = np.asarray(configs[0]["recordedSoftwareRangesDeg"]).copy()
    command[1:, 0] += 1.
    command[1:, 1] -= 1.
    candidates, excluded = frontier_candidates(survey["pointRows"], domain, command)
    original_candidates = len(candidates)
    attempts, windows, covered = [], [], set()
    # ponytail: a declared 32-attempt/8-box greedy search, not a global volume optimum.
    while candidates and len(attempts) < 32 and len(windows) < 8:
        row = max(candidates, key=lambda r: (len(set(r["coveredSampleIndices"])-covered), r["volumeDeg4"]))
        candidates.remove(row)
        if not set(row["coveredSampleIndices"])-covered:
            break
        box, proofs = np.asarray(row["softwareBoundsDeg"]), []
        for checker, config, parent in zip(checkers, configs, parents):
            cad = box+np.asarray(config["offsetCadDeg"])[:, None]
            hits = checker.hits(cad.mean(axis=1))
            proof = partition_regions(checker, cad, parent["lastGroupConstraints"],
                                      1 if hits else 2000, min_width=.01, max_seconds=2)
            proof["centerCollisionPairs"] = hits
            proofs.append(proof)
        passed = all(not p["unknown"] and not p["centerCollisionPairs"] for p in proofs)
        attempts.append(dict(row, boundsPerMapping=proofs, wholeBoxProven=passed))
        print(f"Frontier box {len(attempts)}/32: scale={row['scale']:g}, "
              f"potential gain={len(set(row['coveredSampleIndices'])-covered)}, passed={passed}", flush=True)
        if not passed:
            continue
        for proof, config in zip(proofs, configs):
            audit_partition_cover(proof, box+np.asarray(config["offsetCadDeg"])[:, None])
        low = np.maximum(box[:, 0]+[0., .1, .1, .1, .1], command[:, 0])
        high = np.minimum(box[:, 1]-[0., .1, .1, .1, .1], command[:, 1])
        anchors = []
        for q2, q4, q5 in itertools.product(
                *([float((low[j]+high[j])/2), float(low[j]), float(high[j])] for j in (1, 3, 4))):
            point = np.array([0., q2, 0., q4, q5])
            old, _ = joint3_query(tables, point)
            intervals = intersect_intervals(old, [[float(low[2]), float(high[2])]])
            if intervals:
                point[2] = float(np.mean(max(intervals, key=lambda pair: pair[1]-pair[0])))
                anchors.append(point.tolist())
                break
        windows.append(dict(index=len(windows), category=f"frontier_{len(windows)}",
                            focusSoftwareBoundsDeg=box.tolist(), selectedAttempt=len(attempts)-1,
                            coveredSampleIndices=row["coveredSampleIndices"], anchorCandidatesSoftwareDeg=anchors,
                            anchorSource="previous_common_table_slice", observedSliceSoftwareDeg=box.mean(axis=1).tolist(),
                            verifiedBoxes=1))
        covered.update(row["coveredSampleIndices"])
    for name, digest in identities.items():
        with Path(name).open("rb") as stream:
            assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, name
    out.mkdir(parents=True, exist_ok=False)
    for index, (table, parent, config) in enumerate(zip(tables, parents, configs)):
        own = out / table["mapping"]
        own.mkdir()
        write(own / "config.json", config)
        write(own / "gap_refinement.json", dict(parent, refinementMode="recorded_frontier_common",
              inheritedVerifiedBoxes=len(parent["verifiedSoftwareBoxesDeg"]),
              verifiedSoftwareBoxesDeg=parent["verifiedSoftwareBoxesDeg"]+[w["focusSoftwareBoundsDeg"] for w in windows],
              windows=[dict(w, refinement=attempts[w["selectedAttempt"]]["boundsPerMapping"][index]) for w in windows],
              coarseningAttempts=attempts, mappingOrder=[t["mapping"] for t in tables],
              frontierCandidateCount=original_candidates, frontierExcludedByWitness=excluded,
              frontierSurvey=str(survey_path.resolve()), newlyCoveredSurveySampleIndices=sorted(covered),
              inputFileSha256=identities, implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              elapsedSeconds=time.perf_counter()-started))
    print(dict(candidateBoxes=original_candidates, excludedByWitness=len(excluded), attempts=len(attempts),
               verifiedCommonBoxes=len(windows), newlyCoveredSurveyStates=len(covered),
               missingAnchors=sum(not w["anchorCandidatesSoftwareDeg"] for w in windows)), flush=True)


def joint3_audit(build, out):
    tables, identities = joint3_tables(build)
    comparisons = [build / f"workspace_outer_rule_20261117_{mapping}_v1/compact_rule_comparison.json"
                   for mapping in ("baseline", "alignment")]
    rules = [read(path)["combinedRule"] for path in comparisons]
    for path in comparisons:
        identities[str(path.resolve())] = hashlib.sha256(path.read_bytes()).hexdigest()
    # Match the existing fixed sample exactly, including the untrimmed recorded J5 span.
    source_path = build / "workspace_factored_regions_20261117_baseline_v10/factored_regions.json"
    source = read(source_path)
    recorded = np.asarray(source["recordedSoftwareRangesDeg"])
    del source
    rows, witnesses, split_examples, boundary_checks = [], [], [], 0
    for seed in (2026111702, 2026111713):
        points = np.random.default_rng(seed).uniform(recorded[:, 0], recorded[:, 1], (6000, 5))
        expected = []
        for table in tables:
            domain = np.asarray(table["admissibleSoftwareDomainDeg"])
            expected.append(factored_membership(table["groups"], points)
                            & np.all((points >= domain[:, 0]) & (points <= domain[:, 1]), axis=1))
        expected = expected[0] & expected[1]
        compact = compact_membership(rules[0], points) & compact_membership(rules[1], points)
        assert not np.any(compact & ~expected), "Existing compact domain must remain inside both full tables"
        actual = np.zeros(len(points), dtype=bool)
        gap_count, multi_count = 0, 0
        for i, point in enumerate(points):
            common, own = joint3_query(tables, point)
            actual[i] = interval_contains(common, point[2])
            multi_count += len(common) > 1
            if len(common) > 1 and len(split_examples) < 4:
                split_examples.append(dict(softwareDeg=point.tolist(), commonJ3IntervalsDeg=common))
            if expected[i] and not compact[i]:
                assert actual[i]
                gap_count += 1
                if len(witnesses) < 16:
                    witnesses.append(dict(softwareDeg=point.tolist(), commonJ3IntervalsDeg=common,
                                          separateMappingJ3IntervalsDeg=own))
            if i < 32 or len(common) > 1:
                candidates = [float(np.mean(span)) for span in common]
                candidates += [float((left[1]+right[0])/2) for left, right in zip(common, common[1:])]
                edges = np.array([edge for span in common for edge in span])
                candidates += np.r_[edges, np.nextafter(edges, -math.inf),
                                     np.nextafter(edges, math.inf)].tolist()
                probes = np.tile(point, (len(candidates), 1))
                probes[:, 2] = candidates
                check = np.ones(len(probes), dtype=bool)
                for table in tables:
                    domain = np.asarray(table["admissibleSoftwareDomainDeg"])
                    check &= factored_membership(table["groups"], probes)
                    check &= np.all((probes >= domain[:, 0]) & (probes <= domain[:, 1]), axis=1)
                by_query = [interval_contains(common, value) for value in candidates]
                assert np.array_equal(check, by_query), (seed, i)
                boundary_checks += len(probes)
            if (i+1) % 1000 == 0:
                print(f"J3 query seed={seed}, {i+1}/{len(points)}", flush=True)
        assert np.array_equal(actual, expected), (seed, int((actual != expected).sum()))
        if seed == 2026111702:
            assert int(expected.sum()) == 5097 and int(compact.sum()) == 4903 and gap_count == 194
        rows.append(dict(seed=seed, checkedPoints=len(points), mismatches=int((actual != expected).sum()),
                         commonFullTableAccepted=int(expected.sum()), compactRuleAccepted=int(compact.sum()),
                         additionalFullTablePointsRecovered=gap_count, slicesWithMultipleIntervals=multi_count))
    examples = []
    for point in ([0., 30., 0., 225., 14.], [0., 40., 0., 225., 18.5], [0., 40., 0., 0., 12.]):
        common, own = joint3_query(tables, point)
        examples.append(dict(softwareDeg=point, commonJ3IntervalsDeg=common,
                             separateMappingJ3IntervalsDeg=own))
    out.mkdir(parents=True, exist_ok=False)
    write(out / "joint3_query_audit.json", dict(results=rows, boundaryAndGapProbePoints=boundary_checks,
          examples=examples, additionalRegionWitnesses=witnesses, multiIntervalExamples=split_examples,
          inputFileSha256=identities,
          implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
          verifierSha256=hashlib.sha256(Path(__file__).with_name("workspace_verify.py").read_bytes()).hexdigest(),
          comparisonToleranceDeg=0.,
          limitations=["Uses the existing v10 verified box union, not a larger newly certified geometry",
                       "Returns all common J3 segments; never bridges an unverified gap or interpolates rows",
                       "A legal static slice does not authorize an arbitrary inter-slice path or physical motion"]))
    print(rows, "boundary/gap probes", boundary_checks, "examples", examples, flush=True)


def large_coordination_routes(rows, tables, inset):
    """Bounded widest-first search; a missing table cover is not a collision proof."""
    minimum = np.array([3., 1., 10., 1.])
    points = [(row["index"], np.asarray(point, dtype=float))
              for row in rows for point in row["newRegionWitnesses"]
              if np.all((point >= inset[:, 0]) & (point <= inset[:, 1]))]
    categories = [(f"window_{index}", index) for index in sorted({index for index, _ in points})]
    if len(categories) > 1:
        categories.append(("between_windows", None))
    paths, diagnostics = {}, []
    for name, category in categories:
        pairs = []
        for i, (left, a) in enumerate(points):
            for right, b in points[i+1:]:
                if not ((category is None and left != right) or left == right == category):
                    continue
                delta = abs(b[1:]-a[1:])
                score = float(np.min(delta/minimum))
                if score >= 1.:
                    pairs.append((score, a, b, delta))
        pairs.sort(key=lambda item: -item[0])
        attempts, selected = [], None
        # ponytail: at most 24 existing witness pairs per category; expand only
        # when this bounded search leaves an important route unverified.
        for score, a, b, delta in pairs[:24]:
            anchor_a, anchor_b = a.copy(), b.copy()
            anchor_a[2], anchor_b[2] = 10.-1.5*a[1], 10.-1.5*b[1]
            for lift in (0., .5, 1., 2., 5., 10.):
                raised_a, raised_b = a.copy(), b.copy()
                raised_a[2] += lift
                raised_b[2] += lift
                route = np.array([anchor_a, a, raised_a, raised_b, b, anchor_b, anchor_a])
                route = route[np.r_[True, np.any(route[1:] != route[:-1], axis=1)]]
                inside = bool(np.all((route >= inset[:, 0]) & (route <= inset[:, 1])))
                covered = inside and all(segment_in_regions(group["verifiedSoftwareBoxesDeg"], start, end)
                                         for table in tables for group in table["groups"]
                                         for start, end in zip(route, route[1:]))
                attempt = dict(aSoftwareDeg=a.tolist(), bSoftwareDeg=b.tolist(),
                               fourAxisDeltaDeg=delta.tolist(), balancedSpanScore=score,
                               q3LiftDeg=lift, allSegmentsCovered=bool(covered),
                               insideCommandInset=inside)
                attempts.append(attempt)
                if covered:
                    paths[f"large_{name}_return"] = route.tolist()
                    selected = attempt
                    break
            if selected is not None:
                break
        diagnostics.append(dict(category=name, eligiblePairs=len(pairs), attempts=attempts, selected=selected))
    return paths, diagnostics


def observed_segment_span(points, a, b, tolerance):
    """Finite trace evidence for simultaneous four-axis travel, not a tracking-tube proof."""
    delta = b-a
    axis = 1+int(np.argmax(abs(delta[1:])))
    if delta[axis] == 0 or tolerance < 0:
        raise ValueError("Expected a nonconstant segment and nonnegative tolerance")
    phase = (points[:, axis]-a[axis])/delta[axis]
    residual = np.max(abs(points[:, 1:]-(a+phase[:, None]*delta)[:, 1:]), axis=1)
    selected = (phase >= 0.) & (phase <= 1.) & (residual <= tolerance)
    assert selected.sum() >= 2, "Actual trace did not traverse the selected coordinated segment"
    span = np.ptp(points[selected, 1:], axis=0)
    assert np.all(span >= .95*abs(delta[1:])), "Actual four-axis travel was smaller than the nominal test"
    return dict(samples=int(selected.sum()), fourAxisObservedSpanDeg=span.tolist(),
                phaseRange=[float(phase[selected].min()), float(phase[selected].max())],
                maximumLineResidualDeg=float(residual[selected].max()),
                classificationToleranceDeg=float(tolerance))


def direct_coordination_routes(selection):
    paths, selected = {}, []
    for row in selection:
        if row["selected"] is None:
            continue
        own = dict(row, selected=dict(row["selected"], q3LiftDeg=0.))
        selected.append(own)
        a, b = np.array(own["selected"]["aSoftwareDeg"]), np.array(own["selected"]["bSoftwareDeg"])
        anchor_a, anchor_b = a.copy(), b.copy()
        anchor_a[2], anchor_b[2] = 10.-1.5*a[1], 10.-1.5*b[1]
        paths[f"large_{row['category']}_return"] = np.array([anchor_a, a, b, anchor_b, anchor_a]).tolist()
    assert paths
    return paths, selected


def coarse_boundary_routes(windows, previous, command_inset):
    paths, selection, diagnostics = {}, [], []
    minimum = np.array([3., 1., 10., 1.])
    def previously_allowed(points):
        mask = np.ones(len(points), dtype=bool)
        for table in previous:
            mask &= factored_membership(table["groups"], points)
        return mask
    for window in windows:
        if not window.get("boundaryRepresentative", True):
            continue
        bounds = np.asarray(window["focusSoftwareBoundsDeg"])
        inset = bounds.copy()
        inset[1:, 0] = np.maximum(inset[1:, 0]+.1, command_inset[1:, 0])
        inset[1:, 1] = np.minimum(inset[1:, 1]-.1, command_inset[1:, 1])
        spans = inset[1:, 1]-inset[1:, 0]
        diagnostic = dict(category=window["category"], fourAxisSpanDeg=spans.tolist(), selectedPairs=0)
        diagnostics.append(diagnostic)
        if np.any(spans < minimum):
            diagnostic["reason"] = "Below declared four-axis span, no smaller replacement"
            continue
        if not window["anchorCandidatesSoftwareDeg"]:
            diagnostic["reason"] = "No previous-table anchor in this whole box; no unproved connector"
            continue
        anchors = np.asarray(window["anchorCandidatesSoftwareDeg"])
        valid = previously_allowed(anchors) & np.all(
            (anchors >= command_inset[:, 0]) & (anchors <= command_inset[:, 1]), axis=1)
        assert valid.any(), "No original admitted direct-line anchor inside command inset"
        anchor = anchors[valid][0]
        assert np.all((anchor >= bounds[:, 0]) & (anchor <= bounds[:, 1]))
        for mask in range(8):
            a, b = np.zeros(5), np.zeros(5)
            for j in range(1, 5):
                a[j], b[j] = inset[j, (mask >> (j-1)) & 1], inset[j, 1-((mask >> (j-1)) & 1)]
            probes = a+np.linspace(0., 1., 9)[:, None]*(b-a)
            novel = ~previously_allowed(probes)
            if not novel.any():
                continue
            name = f"{window['category']}_corner_{mask}"
            paths[f"large_{name}_return"] = np.array([anchor, a, b, anchor]).tolist()
            selection.append(dict(category=name, selected=dict(aSoftwareDeg=a.tolist(), bSoftwareDeg=b.tolist(),
                 q3LiftDeg=0., fourAxisDeltaDeg=abs(b[1:]-a[1:]).tolist(),
                 originalRouteAnchorSoftwareDeg=anchor.tolist(), previousTableNovelProbes=int(novel.sum()))))
            diagnostic["selectedPairs"] += 1
            if diagnostic["selectedPairs"] == 3:
                break
    assert paths, "No novel large boundary route; do not substitute old-domain loops"
    return paths, selection, diagnostics


def joint3_paths(build, out, extension=None, previous_extension=None, candidates=None,
                 large_coordination=False, corridors=None, coarse_boundaries=False):
    """Four small new-domain loops; reuse the established simulator and matched controls."""
    tables, identities = joint3_tables(build, extension)
    paths = {
        "joint3_small_wrist_positive": [[0, 35, 45, 0, 12], [0, 40, 70, 0, 12],
                                        [0, 43, 60, 30, 16], [0, 35, 45, 0, 12]],
        "joint3_small_wrist_negative": [[0, 35, 45, 0, 12], [0, 40, 70, 0, 12],
                                        [0, 43, 60, -30, 16], [0, 35, 45, 0, 12]],
        "joint3_positive_upper_tail": [[0, 30, 70, 225, 14], [0, 30, 103, 225, 14],
                                       [0, 35, 100, 220, 16], [0, 30, 70, 225, 14]],
        "joint3_fifth_boundary": [[0, 35, 50, 30, 8], [0, 40, 65, 15, 12],
                                  [0, 43, 60, 30, 16], [0, 35, 50, 30, 8]],
    }
    if extension is not None and candidates is None:
        examples = read(build / "workspace_joint3_query_20261117_v2/joint3_query_audit.json")["multiIntervalExamples"]
        paths = {}
        for index in (2, 3):  # The other two examples violate the simulator's existing 1-degree command inset.
            example = examples[index]
            point = np.asarray(example["softwareDeg"])
            low, high = example["commonJ3IntervalsDeg"][0][1], example["commonJ3IntervalsDeg"][1][0]
            anchor = point.copy()
            anchor[2] = 10.-1.5*point[1]
            a, b = point+[0, -.15, 0, -.3, -.15], point+[0, .15, 0, .3, .15]
            a[2], b[2] = low-.3, high-.2
            paths[f"gap_{index}_coupled_return"] = np.asarray([anchor, a, b, anchor]).tolist()
    selection, boundary_diagnostics = None, None
    if candidates is not None:
        assert extension is not None and previous_extension is not None
        survey = read(candidates)
        assert survey["previousExtension"] == str(previous_extension.resolve())
        for filename, digest in survey["inputFileSha256"].items():
            with Path(filename).open("rb") as stream:
                assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, filename
        previous, previous_identities = joint3_tables(build, previous_extension)
        identities.update(previous_identities)
        identities[str(candidates.resolve())] = hashlib.sha256(candidates.read_bytes()).hexdigest()
        config = read(build / "workspace_outer_rule_20261117_baseline_v1/config.json")
        inset = np.asarray(config["recordedSoftwareRangesDeg"], dtype=float)
        inset[1:, 0] += 1.
        inset[1:, 1] -= 1.
        witnesses = np.array([point for row in survey["results"] for point in row["newRegionWitnesses"]])
        old_allowed = np.ones(len(witnesses), dtype=bool)
        for table in previous:
            old_allowed &= factored_membership(table["groups"], witnesses)
        assert not old_allowed.any(), "New-domain witnesses must actually be outside the previous common table"
        paths = {}
        for row in (() if large_coordination else survey["results"]):
            for witness in row["newRegionWitnesses"]:
                point = np.asarray(witness, dtype=float)
                a = point.copy()
                a[2] = 10.-1.5*a[1]
                b = point+[0., .15, -.15, .3, .1]
                route = np.asarray([a, point, b, a])
                if not np.all((route >= inset[:, 0]) & (route <= inset[:, 1])):
                    continue
                if not all(segment_in_regions(group["verifiedSoftwareBoxesDeg"], start, end)
                           for table in tables for group in table["groups"]
                           for start, end in zip(route, route[1:])):
                    continue
                paths[f"wide_gap_{row['index']}_return"] = route.tolist()
                break
        if large_coordination:
            paths, selection = large_coordination_routes(survey["results"], tables, inset)
        assert paths, "No complete new-domain route; do not simulate an old-domain replacement"
    elif large_coordination:
        raise ValueError("Large coordination requires explicit expansion candidates and previous extension")
    if corridors is not None:
        assert candidates is None and not large_coordination and extension is not None
        assert previous_extension is not None
        for table in tables:
            path = corridors / table["mapping"] / "study.json"
            original = read(path)
            for filename, digest in original["inputFileSha256"].items():
                with Path(filename).open("rb") as stream:
                    assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, filename
            if selection is not None:
                assert original["largeCoordinationSelection"] == previous_selection
            previous_selection = original["largeCoordinationSelection"]
            paths, selection = direct_coordination_routes(previous_selection)
            identities[str(path.resolve())] = hashlib.sha256(path.read_bytes()).hexdigest()
    if coarse_boundaries:
        assert candidates is None and not large_coordination and corridors is None
        assert extension is not None and previous_extension is not None
        previous, previous_identities = joint3_tables(build, previous_extension)
        identities.update(previous_identities)
        reports = [read(extension / table["mapping"] / "gap_refinement.json") for table in tables]
        assert all(report["refinementMode"] in ("direct_corridors_coarse_common", "frontier_inflated_common",
                                               "frontier_balanced_common")
                   for report in reports)
        for report in reports:
            for filename, digest in report["inputFileSha256"].items():
                with Path(filename).open("rb") as stream:
                    assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, filename
            identities.update(report["inputFileSha256"])
        assert [w["focusSoftwareBoundsDeg"] for w in reports[0]["windows"]] == [
            w["focusSoftwareBoundsDeg"] for w in reports[1]["windows"]]
        config = read(extension / "baseline/config.json")
        inset = np.asarray(config["recordedSoftwareRangesDeg"], dtype=float)
        inset[1:, 0] += 1.
        inset[1:, 1] -= 1.
        paths, selection, boundary_diagnostics = coarse_boundary_routes(reports[0]["windows"], previous, inset)
    proofs = []
    for table in tables:
        for name, points in paths.items():
            for segment, (a, b) in enumerate(zip(points, points[1:])):
                groups = [segment_in_regions(group["verifiedSoftwareBoxesDeg"], a, b)
                          for group in table["groups"]]
                assert all(groups), (table["mapping"], name, segment)
                proofs.append(dict(mapping=table["mapping"], route=name, segment=segment,
                                   allEightGroupsCovered=groups))
    out.mkdir(parents=True, exist_ok=False)
    for table in tables:
        mapping = table["mapping"]
        source = build / f"workspace_outer_rule_20261117_{mapping}_v1"
        config, geometry = read(source / "config.json"), read(source / "geometry.json")
        rule = read(source / "compact_rule_comparison.json")["combinedRule"]
        identities_own = dict(identities)
        for name in ("config.json", "geometry.json", "compact_rule_comparison.json", "coupled_summary.json"):
            path = source / name
            identities_own[str(path.resolve())] = hashlib.sha256(path.read_bytes()).hexdigest()
        offset = np.asarray(config["offsetCadDeg"])
        for points in paths.values():
            assert np.any(~compact_membership(rule, points))
            cad = np.asarray(points)+offset
            bounds = np.asarray(config["mappedCadRangesDeg"])
            assert np.all(cad[:, 1:] >= bounds[1:, 0]+1) and np.all(cad[:, 1:] <= bounds[1:, 1]-1)
        geometry["paths"] = [dict(id=name, waypointsCadDeg=(np.asarray(points)+offset).tolist())
                             for name, points in paths.items()]
        own = out / mapping
        own.mkdir()
        write(own / "config.json", config)
        write(own / "geometry.json", geometry)
        write(own / "study.json", dict(parameters=[dict(type="conditional_full_table_J3_gap_paths" if extension
                                                          else "conditional_full_table_J3_paths")],
              largeCoordinationSelection=selection,
              directCoordinationSource=str(corridors.resolve()) if corridors else None,
              coarseBoundaryDiagnostics=boundary_diagnostics,
              wholeNominalSegmentsPerMapping=sum(len(points)-1 for points in paths.values()),
              wholeNominalProofs=proofs,
              inputFileSha256=identities_own,
              implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              verifierSha256=hashlib.sha256(Path(__file__).with_name("workspace_verify.py").read_bytes()).hexdigest(),
              limitations=["Existing full-table numerical bounds, both unregistered mappings, J1=0 nominal only",
                           "New selected trajectories, not full-domain dynamics or physical clearance certification"]))
        print(mapping, len(paths), "new loops,", sum(len(points)-1 for points in paths.values()),
              "whole segments, both mappings/all 8 groups checked;", 2*(len(paths)+6), "planned cases", flush=True)


def joint3_paths_audit(build, source, out=None, extension=None, previous_extension=None):
    tables, identities = joint3_tables(build, extension)
    study, config = read(source / "study.json"), read(source / "config.json")
    for name, digest in study["inputFileSha256"].items():
        with Path(name).open("rb") as stream:
            assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, name
    trace = read(source / "actual_trace.json")["samples"]
    points = np.asarray([sample["cadDegrees"] for sample in trace])-config["offsetCadDeg"]
    j1_error = float(abs(points[:, 0]).max())
    points[:, 0] = 0.  # Membership only; the full actual J1 remains in the independent FCL trace.
    for table in tables:
        domain = np.asarray(table["admissibleSoftwareDomainDeg"])
        allowed = factored_membership(table["groups"], points)
        allowed &= np.all((points >= domain[:, 0]) & (points <= domain[:, 1]), axis=1)
        assert allowed.all(), (table["mapping"], int((~allowed).sum()))
    rule_path = build / "workspace_outer_rule_20261117_baseline_v1/compact_rule_comparison.json"
    novel = ~compact_membership(read(rule_path)["combinedRule"], points)
    if extension is not None:
        previous, previous_identities = joint3_tables(build, previous_extension)
        was_allowed = np.ones(len(points), dtype=bool)
        for table in previous:
            domain = np.asarray(table["admissibleSoftwareDomainDeg"])
            was_allowed &= factored_membership(table["groups"], points)
            was_allowed &= np.all((points >= domain[:, 0]) & (points <= domain[:, 1]), axis=1)
        novel = ~was_allowed
        identities.update(previous_identities)
    labels = [sample["case"] for sample in trace]
    required = {f"{speed}/{path['id']}" for path in read(source / "geometry.json")["paths"]
                for speed in ("slow", "brisk")}
    counts = {label: int(sum(is_new and actual_label == label for actual_label, is_new in zip(labels, novel)))
              for label in required}
    assert all(counts.values()), counts
    coordinated = {}
    if study.get("largeCoordinationSelection"):
        dynamics = read(source / "dynamics.json")
        for row in study["largeCoordinationSelection"]:
            selected = row["selected"]
            if selected is None:
                continue
            a, b = np.array(selected["aSoftwareDeg"]), np.array(selected["bSoftwareDeg"])
            a[2] += selected["q3LiftDeg"]
            b[2] += selected["q3LiftDeg"]
            for speed in ("slow", "brisk"):
                label = f"{speed}/large_{row['category']}_return"
                case = next(case for case in dynamics["cases"] if case["speed"]+"/"+case["id"] == label)
                observed = points[np.asarray(labels) == label]
                coordinated[label] = observed_segment_span(observed, a, b, max(case["maxErrorDeg"])+1e-6)
    identities[str(rule_path.resolve())] = hashlib.sha256(rule_path.read_bytes()).hexdigest()
    for name in ("study.json", "config.json", "geometry.json", "actual_trace.json", "dynamics.json"):
        path = source / name
        identities[str(path.resolve())] = hashlib.sha256(path.read_bytes()).hexdigest()
    write(out or source / "joint3_actual_audit.json", dict(projectedSamples=len(points), fullTableViolations=0,
          originalJ1MaximumActualErrorDeg=j1_error, newDomainSamplesByRoute=counts,
          comparedAgainst=(str(previous_extension.resolve()) if previous_extension else
                           "v10_full_table_intersection") if extension else "outer_compact_rule",
          observedLargeCoordination=coordinated,
          inputFileSha256=identities, comparisonToleranceDeg=0.,
          implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
          verifierSha256=hashlib.sha256(Path(__file__).with_name("workspace_verify.py").read_bytes()).hexdigest(),
          note="Both mapping tables checked on the actual 4D projection; actual five-axis FCL remains required"))
    print(source.name, "all", len(points), "actual projection samples in both tables; novel route samples", counts, flush=True)


def direct_coordination_audit(source, out, motion_bounds=False):
    """Diagnose unlifted alternatives separately; zero sampled hits does not admit them."""
    study_path, config_path = source / "study.json", source / "config.json"
    study, config = read(study_path), read(config_path)
    identities = dict(study["inputFileSha256"])
    for path in (study_path, config_path):
        identities[str(path.resolve())] = hashlib.sha256(path.read_bytes()).hexdigest()
    checker = BoundsChecker(source) if motion_bounds else Checker(source)
    results = []
    for row in study["largeCoordinationSelection"]:
        if row["selected"] is None:
            continue
        a, b = np.array(row["selected"]["aSoftwareDeg"]), np.array(row["selected"]["bSoftwareDeg"])
        if motion_bounds:
            proof = check_curve(checker, lambda t: a+t*(b-a)+config["offsetCadDeg"], max_nodes=6000)
            results.append(dict(category=row["category"], constraints=len(checker.constraints), motionBound=proof))
            print(row["category"], "unlifted whole segment:", proof, flush=True)
            continue
        hits = []
        for phase in np.linspace(0., 1., 257):
            q = a+phase*(b-a)
            pairs = checker.hits(q+config["offsetCadDeg"])
            if pairs:
                hits.append(dict(phase=float(phase), softwareDeg=q.tolist(), pairs=pairs))
        results.append(dict(category=row["category"], checked=257, colliding=len(hits), witnesses=hits))
        print(row["category"], "unlifted alternative:", len(hits), "/257 sampled states colliding", flush=True)
    for filename, digest in identities.items():
        with Path(filename).open("rb") as stream:
            assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, filename
    out.mkdir(parents=True, exist_ok=False)
    write(out / ("direct_coordination_bounds.json" if motion_bounds else "direct_coordination_audit.json"),
          dict(results=results, inputFileSha256=identities,
          implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
          limitations=["Unlifted alternatives only, not the admitted lifted paths",
                       "A positive hit is a model collision witness; zero finite hits is not a motion certificate",
                       "A nominal numerical motion bound does not cover actual tracking tubes or a full region",
                       "Original triangle-surface rounding, containment and unregistered-model limitations apply"]))


def gap_audit(build, extension, out, previous_extension=None):
    before, identities = joint3_tables(build, previous_extension)
    after, extra_identities = joint3_tables(build, extension)
    identities.update(extra_identities)
    for old, new in zip(before, after):
        assert old["admissibleSoftwareDomainDeg"] == new["admissibleSoftwareDomainDeg"]
        for old_group, new_group in zip(old["groups"], new["groups"]):
            boxes, expanded = old_group["verifiedSoftwareBoxesDeg"], new_group["verifiedSoftwareBoxesDeg"]
            assert np.array_equal(boxes, expanded[:len(boxes)]), "Every old verified box must survive exactly"
    path = build / "workspace_joint3_query_20261117_v2/joint3_query_audit.json"
    focus = read(extension / "baseline/gap_refinement.json")
    inflated = focus.get("refinementMode") in ("frontier_inflated_common", "frontier_balanced_common")
    corridors = focus.get("refinementMode", "").startswith("direct_corridors") or inflated
    coarse = focus.get("refinementMode") == "direct_corridors_coarse_common" or inflated
    coarse_boxes = np.asarray([w["focusSoftwareBoundsDeg"] for w in focus["windows"]]) if coarse else None
    formula_checks, leaf_counts = 0, {}
    if coarse:
        alternate = read(extension / "alignment/gap_refinement.json")
        assert [w["focusSoftwareBoundsDeg"] for w in focus["windows"]] == [
            w["focusSoftwareBoundsDeg"] for w in alternate["windows"]]
        for table, report in zip(after, (focus, alternate)):
            config_path = extension / table["mapping"] / "config.json"
            offset = np.asarray(read(config_path)["offsetCadDeg"])
            identities[str(config_path.resolve())] = hashlib.sha256(config_path.read_bytes()).hexdigest()
            leaf_counts[table["mapping"]] = [audit_partition_cover(
                window["refinement"], np.asarray(window["focusSoftwareBoundsDeg"])+offset[:, None])
                for window in report["windows"]]
            if report["refinementMode"] == "frontier_balanced_common":
                assert Path(report["inheritedSubregionSource"]) == previous_extension.resolve()
                old = next(item for item in before if item["mapping"] == table["mapping"])
                known = old["groups"][-1]["verifiedSoftwareBoxesDeg"]+offset[None, :, None]
                for window in report["windows"]:
                    proof = window["refinement"]
                    assert proof["inheritedSourceBoxCount"] == len(known)
                    for leaf in proof["verified"]:
                        if "inheritedSafeBoxIndex" in leaf:
                            index = leaf["inheritedSafeBoxIndex"]
                            assert isinstance(index, int) and 0 <= index < len(known)
                            box = np.asarray(leaf["cadBoundsDeg"])
                            assert np.all(box[:, 0] >= known[index, :, 0])
                            assert np.all(box[:, 1] <= known[index, :, 1])
                        else:
                            assert leaf["boundSlackMm"] is not None and leaf["boundSlackMm"] > 0
    examples = ([dict(softwareDeg=(np.asarray(window["focusSoftwareBoundsDeg"]).mean(axis=1).tolist()
                                  if inflated else window["observedSliceSoftwareDeg"]),
                     endpoints=(np.asarray(window["focusSoftwareBoundsDeg"]).T.tolist() if inflated else
                                [window["probes"][0]["softwareDeg"], window["probes"][-1]["softwareDeg"]]))
                 for window in focus["windows"]] if corridors else read(path)["multiIntervalExamples"])
    identities[str(path.resolve())] = hashlib.sha256(path.read_bytes()).hexdigest()
    rows, all_local, query_checks, boundary_checks = [], [], 0, 0
    for index, example in enumerate(examples):
        point = np.asarray(example["softwareDeg"])
        old_intervals, _ = joint3_query(before, point)
        new_intervals, _ = joint3_query(after, point)
        if corridors:
            a, b = map(np.array, example["endpoints"])
        else:
            a, b = point.copy(), point.copy()
            a[2], b[2] = example["commonJ3IntervalsDeg"][0][1], example["commonJ3IntervalsDeg"][1][0]
        proof = [all(segment_in_regions(group["verifiedSoftwareBoxesDeg"], a, b)
                     for group in table["groups"]) for table in after]
        assert all(proof) and (corridors or len(new_intervals) == 1)
        window = np.asarray(focus["windows"][index]["focusSoftwareBoundsDeg"])
        points = np.random.default_rng(2026111720+index).uniform(window[:, 0], window[:, 1],
                                                               (64 if corridors and not coarse else 1500, 5))
        old_allowed, new_allowed = np.ones(len(points), dtype=bool), np.ones(len(points), dtype=bool)
        for tables, mask in ((before, old_allowed), (after, new_allowed)):
            for table in tables:
                domain = np.asarray(table["admissibleSoftwareDomainDeg"])
                mask &= factored_membership(table["groups"], points)
                mask &= np.all((points >= domain[:, 0]) & (points <= domain[:, 1]), axis=1)
        assert not np.any(old_allowed & ~new_allowed)
        if coarse:
            extra = np.any(np.all((points[:, None] >= coarse_boxes[None, :, :, 0]) &
                                 (points[:, None] <= coarse_boxes[None, :, :, 1]), axis=2), axis=1)
            assert np.array_equal(old_allowed | extra, new_allowed), "Common added-box OR formula mismatch"
            for q in points[:64]:
                own = [box[2].tolist() for box in coarse_boxes
                       if np.all((q[[0, 1, 3, 4]] >= box[[0, 1, 3, 4], 0]) &
                                 (q[[0, 1, 3, 4]] <= box[[0, 1, 3, 4], 1]))]
                composed = merge_intervals(joint3_query(before, q)[0]+own)
                assert composed == joint3_query(after, q)[0], "Composed interval union differs"
                formula_checks += 1
        if corridors:
            by_query = np.array([interval_contains(joint3_query(after, q)[0], q[2]) for q in points])
            assert np.array_equal(by_query, new_allowed), (index, "Local query/box mismatch")
            query_checks += len(points)
            edges = np.array([edge for span in new_intervals for edge in span])
            gaps = [(left[1]+right[0])/2 for left, right in zip(new_intervals, new_intervals[1:])]
            values = np.r_[edges, np.nextafter(edges, -math.inf), np.nextafter(edges, math.inf), gaps]
            probes = np.tile(point, (len(values), 1))
            probes[:, 2] = values
            expected = np.ones(len(probes), dtype=bool)
            for table in after:
                domain = np.asarray(table["admissibleSoftwareDomainDeg"])
                expected &= factored_membership(table["groups"], probes)
                expected &= np.all((probes >= domain[:, 0]) & (probes <= domain[:, 1]), axis=1)
            assert np.array_equal(expected, [interval_contains(new_intervals, value) for value in values])
            boundary_checks += len(probes)
        witnesses = points[~old_allowed & new_allowed][:64]
        rows.append(dict(index=index, observedSliceSoftwareDeg=point.tolist(),
                         beforeCommonJ3IntervalsDeg=old_intervals, afterCommonJ3IntervalsDeg=new_intervals,
                         fullGapSegmentProvenPerMapping=proof, localSampleSeed=2026111720+index,
                         localSamples=len(points), previouslyAllowed=int(old_allowed.sum()),
                         expandedAllowed=int(new_allowed.sum()), newlyAllowed=int((~old_allowed & new_allowed).sum()),
                         localWindowFullyAdmitted=bool(new_allowed.all()),
                         newRegionWitnesses=witnesses.tolist()))
        all_local.append(points)
        if (index+1) % 10 == 0:
            print(f"Gap audit: {index+1}/{len(examples)} windows, {query_checks} actual interval queries checked",
                  flush=True)
    global_rows = []
    for seed in (2026111702, 2026111722):
        config_path = extension / "baseline/config.json"
        recorded = np.asarray(read(config_path)["recordedSoftwareRangesDeg"])
        identities[str(config_path.resolve())] = hashlib.sha256(config_path.read_bytes()).hexdigest()
        points = np.random.default_rng(seed).uniform(recorded[:, 0], recorded[:, 1], (6000, 5))
        allowed = []
        for tables in (before, after):
            mask = np.ones(len(points), dtype=bool)
            for table in tables:
                domain = np.asarray(table["admissibleSoftwareDomainDeg"])
                mask &= factored_membership(table["groups"], points)
                mask &= np.all((points >= domain[:, 0]) & (points <= domain[:, 1]), axis=1)
            allowed.append(mask)
        assert not np.any(allowed[0] & ~allowed[1])
        if coarse:
            extra = np.any(np.all((points[:, None] >= coarse_boxes[None, :, :, 0]) &
                                 (points[:, None] <= coarse_boxes[None, :, :, 1]), axis=2), axis=1)
            assert np.array_equal(allowed[0] | extra, allowed[1])
        global_rows.append(dict(seed=seed, samples=len(points), before=int(allowed[0].sum()),
                                after=int(allowed[1].sum()), added=int((~allowed[0] & allowed[1]).sum())))
    out.mkdir(parents=True, exist_ok=False)
    write(out / ("corridor_audit.json" if corridors else "gap_audit.json"),
          dict(results=rows, allOriginalBoxesRetainedExactly=True,
          previousExtension=str(previous_extension.resolve()) if previous_extension else None,
          recordedDomainSamples=global_rows,
          queryCheckedLocalPoints=query_checks, queryBoundaryAndGapProbes=boundary_checks,
          composedIntervalFormulaChecks=formula_checks,
          wholeBoxLeafCountsPerMapping=leaf_counts,
          totalLocalRandomPoints=sum(len(points) for points in all_local), inputFileSha256=identities,
          implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
          verifierSha256=hashlib.sha256(Path(__file__).with_name("workspace_verify.py").read_bytes()).hexdigest(),
          limitations=["Local windows/corridors only, no claim of global maximality",
                       "Whole-box numerical bounds, not formal arithmetic or physical calibration",
                       "Dynamic paths and actual five-axis FCL remain separate gates"]))
    print(dict(windows=len(rows), localPoints=sum(len(points) for points in all_local),
               addedLocal=sum(row["newlyAllowed"] for row in rows),
               queryCheckedLocalPoints=query_checks, queryBoundaryAndGapProbes=boundary_checks,
               recordedDomainSamples=global_rows) if corridors else
          [{k: v for k, v in row.items() if k != "newRegionWitnesses"} for row in rows], flush=True)


def gap_window(q, gap, domain, scale):
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError("Focus scale must be positive and finite")
    bounds = np.column_stack((q-scale*np.array([0, .5, 0, 2, .5]),
                              q+scale*np.array([0, .5, 0, 2, .5])))
    bounds[2] = [gap[0]-.5*scale, gap[1]+.5*scale]
    bounds[:, 0] = np.maximum(bounds[:, 0], domain[:, 0])
    bounds[:, 1] = np.minimum(bounds[:, 1], domain[:, 1])
    if np.any(bounds[:, 1] < bounds[:, 0]):
        raise ValueError("Focus window is outside the approved domain")
    return bounds


def corridor_windows(selection, domain):
    """Thin overlapping axis boxes around the original direct lines, not larger replacement loops."""
    windows = []
    padding = np.array([0., .1, .1, .1, .1])
    for row in selection:
        if row["selected"] is None:
            continue
        a, b = np.array(row["selected"]["aSoftwareDeg"]), np.array(row["selected"]["bSoftwareDeg"])
        for segment in range(16):
            start, end = [a+t*(b-a) for t in (segment/16, (segment+1)/16)]
            bounds = np.column_stack((np.maximum(np.minimum(start, end)-padding, domain[:, 0]),
                                      np.minimum(np.maximum(start, end)+padding, domain[:, 1])))
            assert np.all(bounds[:, 0] <= bounds[:, 1])
            probes = start+np.linspace(0., 1., 3)[:, None]*(end-start)
            windows.append((bounds, probes, dict(category=row["category"], segment=segment)))
    return windows


def coarsen_windows(windows, domain, prove):
    """Try common rectangular supersets; an unresolved hull never removes its old cells."""
    attempts, merged = [], []
    def attempt(rows, bounds, padding):
        proofs = prove(bounds)
        passed = all(math.isclose(p["verifiedFraction"], 1., rel_tol=0., abs_tol=1e-12)
                     and not p["unknown"] and not p.get("centerCollisionPairs") for p in proofs)
        attempts.append(dict(sourceCells=[row[2]["sourceCell"] for row in rows],
                             softwareBoundsDeg=bounds.tolist(), nominalPaddingDeg=padding,
                             wholeBoxProven=passed, boundsPerMapping=proofs))
        return len(attempts)-1 if passed else None
    def visit(rows):
        boxes = np.asarray([row[0] for row in rows])
        bounds = np.column_stack((boxes[:, :, 0].min(axis=0), boxes[:, :, 1].max(axis=0)))
        proof = attempt(rows, bounds, 0.)
        if proof is not None:
            merged.append((rows, bounds, proof))
            return
        assert len(rows) > 1, "Even an inherited verified cell failed; do not replace its proof"
        categories = sorted({row[2]["category"] for row in rows})
        children = ([ [row for row in rows if row[2]["category"] == category] for category in categories]
                    if len(categories) > 1 else [rows[:len(rows)//2], rows[len(rows)//2:]])
        for child in children:
            visit(child)
    rows = [(box, probes, dict(metadata, sourceCell=i))
            for i, (box, probes, metadata) in enumerate(windows)]
    visit(rows)
    result = []
    for index, (rows, original, proof) in enumerate(merged):
        chosen = original
        # ponytail: five declared padding candidates, not a global volume optimizer.
        for padding in (.5, 1., 2., 5., 10.):
            delta = np.array([0., padding, padding, padding, padding])
            bounds = np.column_stack((np.ceil(np.maximum(original[:, 0]-delta, domain[:, 0])*10)/10,
                                      np.floor(np.minimum(original[:, 1]+delta, domain[:, 1])*10)/10))
            if np.any(bounds[:, 0] > original[:, 0]) or np.any(bounds[:, 1] < original[:, 1]):
                continue  # Inward decimal rounding must not shrink the parent.
            candidate = attempt(rows, bounds, padding)
            if candidate is not None:
                chosen, proof = bounds, candidate
        probes = np.array([chosen[:, 0], chosen.mean(axis=1), chosen[:, 1]])
        result.append((chosen, probes, dict(category=f"coarse_{index}",
                       sourceCells=[row[2]["sourceCell"] for row in rows], selectedAttempt=proof,
                       anchorCandidatesSoftwareDeg=[row[1][len(row[1])//2].tolist() for row in rows])))
    return result, attempts


def coarse_regions(build, extension, out):
    started = time.perf_counter()
    tables, identities = joint3_tables(build, extension)
    parents, checkers, configs = [], [], []
    for table in tables:
        mapping = table["mapping"]
        parent = read(extension / mapping / "gap_refinement.json")
        assert parent["refinementMode"] == "direct_corridors_16_cells_padding_0.1deg"
        assert parent["constraintsAccountedFor"] == 89 and parent["completeOtherGroupsInherited"] == 7
        assert len(parent["windows"]) == 48 and all(window["verifiedBoxes"] == 1 for window in parent["windows"])
        domain = np.asarray(table["admissibleSoftwareDomainDeg"])
        assert all(np.array_equal(group["verifiedSoftwareBoxesDeg"], [domain]) for group in table["groups"][:-1])
        for filename, digest in parent["inputFileSha256"].items():
            with Path(filename).open("rb") as stream:
                assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, filename
        identities.update(parent["inputFileSha256"])
        parents.append(parent)
        configs.append(read(extension / mapping / "config.json"))
        checkers.append(BoundsChecker(extension / mapping))
        assert len(checkers[-1].constraints) == 89
    domain = np.column_stack((np.max([table["admissibleSoftwareDomainDeg"] for table in tables], axis=0)[:, 0],
                              np.min([table["admissibleSoftwareDomainDeg"] for table in tables], axis=0)[:, 1]))
    cells = []
    for index, window in enumerate(parents[0]["windows"]):
        boxes = np.asarray([parent["verifiedSoftwareBoxesDeg"][-48+index] for parent in parents])
        bounds = np.column_stack((boxes[:, :, 0].min(axis=0), boxes[:, :, 1].max(axis=0)))
        assert np.all(bounds[:, 0] >= domain[:, 0]) and np.all(bounds[:, 1] <= domain[:, 1])
        assert all(parent["windows"][index]["category"] == window["category"] for parent in parents)
        cells.append((bounds, np.array([p["softwareDeg"] for p in window["probes"]]),
                      dict(category=window["category"])))
    def prove(bounds):
        results = []
        for checker, config, parent in zip(checkers, configs, parents):
            cad = bounds+np.asarray(config["offsetCadDeg"])[:, None]
            hits = checker.hits(cad.mean(axis=1))
            proof = partition_regions(checker, cad, parent["lastGroupConstraints"],
                                      1 if hits else 2000, min_width=.01, max_seconds=2)
            proof["centerCollisionPairs"] = hits
            results.append(proof)
        return results
    windows, attempts = coarsen_windows(cells, domain, prove)
    assert all(any(np.all(old[0][:, 0] >= new[0][:, 0]) and
                       np.all(old[0][:, 1] <= new[0][:, 1]) for new in windows) for old in cells)
    for filename, digest in identities.items():
        with Path(filename).open("rb") as stream:
            assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, filename
    out.mkdir(parents=True, exist_ok=False)
    for index, (table, parent, config) in enumerate(zip(tables, parents, configs)):
        own = out / table["mapping"]
        own.mkdir()
        report = dict(parent, refinementMode="direct_corridors_coarse_common",
                      inheritedVerifiedBoxes=len(parent["verifiedSoftwareBoxesDeg"]),
                      verifiedSoftwareBoxesDeg=parent["verifiedSoftwareBoxesDeg"]+[w[0].tolist() for w in windows],
                      coarseningAttempts=attempts, mappingOrder=[t["mapping"] for t in tables],
                      elapsedSeconds=time.perf_counter()-started,
                      inputFileSha256=identities,
                      implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                      verifierSha256=hashlib.sha256(Path(__file__).with_name("workspace_verify.py").read_bytes()).hexdigest(),
                      windows=[dict(index=i, focusSoftwareBoundsDeg=box.tolist(), **metadata,
                                    observedSliceSoftwareDeg=probes[1].tolist(),
                                    probes=[dict(softwareDeg=q.tolist()) for q in probes], verifiedBoxes=1,
                                    refinement=attempts[metadata["selectedAttempt"]]["boundsPerMapping"][index])
                               for i, (box, probes, metadata) in enumerate(windows)])
        write(own / "config.json", config)
        write(own / "gap_refinement.json", report)
    print(dict(originalCells=48, commonCoarseBoxes=len(windows), attempts=len(attempts),
               passedAttempts=sum(a["wholeBoxProven"] for a in attempts),
               selectedBounds=[window[0].tolist() for window in windows]), flush=True)


def gap_refine(build, mapping, out, scale=1., extension=None, corridors=None):
    """Refine four observed holes locally; unknowns remain unknown and old tables stay intact."""
    started = time.perf_counter()
    base_path = build / f"workspace_factored_regions_20261117_{mapping}_v10/factored_regions.json"
    sample_path = (corridors / mapping / "study.json" if corridors is not None else
                   build / "workspace_joint3_query_20261117_v2/joint3_query_audit.json")
    base, samples = read(base_path), read(sample_path)
    assert len(base["groups"]) == 8
    domain, offset = np.asarray(base["admissibleSoftwareDomainDeg"]), np.asarray(base["offsetCadDeg"])
    assert all(not group["unknownReducedCadBoxes"] for group in base["groups"][:-1])
    assert all(group["verifiedSoftwareBoxesDeg"] == [domain.tolist()] for group in base["groups"][:-1])
    original = read(Path(base["previousReport"]))
    source = Path(original["source"])
    config = read(source / "config.json")
    assert config["offsetCadDeg"] == offset.tolist()
    identities = {}
    for filename, digest in base["priorInputFileSha256"].items():
        path = Path(filename)
        if path.resolve() == Path(__file__).with_name("workspace_verify.py").resolve():
            continue  # Preserve its historical execution identity; geometry inputs must still match.
        with path.open("rb") as stream:
            assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, filename
        identities[str(path.resolve())] = digest
    for path in (base_path, sample_path):
        with path.open("rb") as stream:
            identities[str(path.resolve())] = hashlib.file_digest(stream, "sha256").hexdigest()
    if corridors is not None:
        proof_path = build / "workspace_large_direct_bounds_20261117_v1" / mapping / "direct_coordination_bounds.json"
        proof = read(proof_path)
        assert all(item["constraints"] == 89 and item["motionBound"]["status"] == "numerical_motion_bound_pass"
                   for item in proof["results"])
        selected = [row for row in samples["largeCoordinationSelection"] if row["selected"] is not None]
        assert {row["category"] for row in selected} == {row["category"] for row in proof["results"]}
        assert proof["inputFileSha256"][str(sample_path.resolve())] == identities[str(sample_path.resolve())]
        identities.update(proof["inputFileSha256"])
        identities[str(proof_path.resolve())] = hashlib.sha256(proof_path.read_bytes()).hexdigest()
        windows = corridor_windows(selected, domain)
    else:
        examples = samples["multiIntervalExamples"]
        assert len(examples) == 4
        windows = []
        for example in examples:
            q = np.asarray(example["softwareDeg"], dtype=float)
            gap = [example["commonJ3IntervalsDeg"][0][1], example["commonJ3IntervalsDeg"][1][0]]
            probes = np.tile(q, (33, 1))
            probes[:, 2] = np.linspace(*gap, len(probes))
            windows.append((gap_window(q, gap, domain, scale), probes, dict(originalGapDeg=gap)))
    inherited = []
    if extension is not None:
        parent_path = extension / mapping / "gap_refinement.json"
        parent = read(parent_path)
        assert parent["baseTableSha256"] == identities[str(base_path.resolve())]
        assert parent["mapping"] == mapping and parent["lastGroupConstraints"] == base["groups"][-1]["constraints"]
        for filename, digest in parent["inputFileSha256"].items():
            with Path(filename).open("rb") as stream:
                assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, filename
        identities[str(parent_path.resolve())] = hashlib.sha256(parent_path.read_bytes()).hexdigest()
        inherited = parent["verifiedSoftwareBoxesDeg"]
    checker = BoundsChecker(source)
    accounted = set(original["remainingConstraints"])
    for item in original["adjacentConstraints"]:
        accounted.update(item["constraints"])
    assert accounted == set(checker.constraints) and len(accounted) == base["constraintsAccountedFor"] == 89
    names, results, verified = base["groups"][-1]["constraints"], [], list(inherited)
    for index, (bounds, probes, metadata) in enumerate(windows):
        hits = [dict(softwareDeg=point.tolist(), pairs=checker.hits(point+offset)) for point in probes]
        local = partition_regions(checker, bounds+offset[:, None], names, 1000 if corridors else 6000,
                                  min_width=.01, max_seconds=1 if corridors else 30)
        software_boxes = [(np.asarray(row["cadBoundsDeg"])-offset[:, None]).tolist()
                          for row in local["verified"]]
        verified.extend(software_boxes)
        result = dict(index=index, focusSoftwareBoundsDeg=bounds.tolist(), **metadata,
                      observedSliceSoftwareDeg=probes[len(probes)//2].tolist(), probes=hits,
                      independentlyCollidingProbePoints=sum(bool(item["pairs"]) for item in hits),
                      verifiedBoxes=len(software_boxes), refinement=local)
        results.append(result)
        print(mapping, "gap", index, "colliding probes", result["independentlyCollidingProbePoints"],
              "new boxes", len(software_boxes), "unknown", len(local["unknown"]),
              "nodes", local["nodes"], "seconds", local["elapsedSeconds"], flush=True)
    for filename, digest in identities.items():
        with Path(filename).open("rb") as stream:
            assert hashlib.file_digest(stream, "sha256").hexdigest() == digest, filename
    out.mkdir(parents=True, exist_ok=False)
    write(out / "config.json", config)
    write(out / "gap_refinement.json", dict(mapping=mapping,
          baseTableSha256=identities[str(base_path.resolve())], baseTable=str(base_path.resolve()),
          originalDomainSoftwareDeg=domain.tolist(), constraintsAccountedFor=89,
          refinementMode="direct_corridors_16_cells_padding_0.1deg" if corridors else "observed_gaps",
          focusScale=scale, inheritedVerifiedBoxes=len(inherited),
          completeOtherGroupsInherited=7, lastGroupConstraints=names,
          verifiedSoftwareBoxesDeg=verified, windows=results, inputFileSha256=identities,
          implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
          verifierSha256=hashlib.sha256(Path(__file__).with_name("workspace_verify.py").read_bytes()).hexdigest(),
          elapsedSeconds=time.perf_counter()-started,
          limitations=["Local overlay of newly verified last-group boxes, not a replacement for v10",
                       "Original adjacent/all seven other group proofs remain required",
                       "Unresolved boxes and isolated collision-free samples are not admitted",
                       "No physical calibration, positive tracking margin or full-domain dynamics certificate"]))


def audit_partition_cover(proof, bounds):
    """Exact rational leaf volume plus disjoint interiors; no rounded coverage fraction."""
    bounds = np.asarray(bounds)
    leaves = np.asarray([row["cadBoundsDeg"] for row in proof["verified"]])
    assert not proof["unknown"] and not proof.get("centerCollisionPairs")
    assert bounds.shape == (5, 2) and np.isfinite(bounds).all()
    assert leaves.ndim == 3 and leaves.shape[1:] == bounds.shape and np.isfinite(leaves).all()
    assert np.all(bounds[:, 0] <= bounds[:, 1]) and np.all(leaves[:, :, 0] <= leaves[:, :, 1])
    assert np.all(leaves[:, :, 0] >= bounds[:, 0]) and np.all(leaves[:, :, 1] <= bounds[:, 1])
    active = np.flatnonzero(bounds[:, 1] > bounds[:, 0])
    assert len(active) and np.all(leaves[:, active, 1] > leaves[:, active, 0])
    def measure(box):
        return math.prod(Fraction(float(box[j, 1]))-Fraction(float(box[j, 0])) for j in active)
    assert sum(measure(leaf) for leaf in leaves) == measure(bounds), "Leaf union leaves a volume gap"
    for index, leaf in enumerate(leaves):
        overlap = np.all(np.maximum(leaves[:index][:, active, 0], leaf[active, 0]) <
                         np.minimum(leaves[:index][:, active, 1], leaf[active, 1]), axis=1)
        assert not overlap.any(), "Leaf interiors overlap; summed volume alone is insufficient"
    return len(leaves)


def readable_audit(build, document, out, extension=None):
    """Independently evaluate the written formula, including optional outer-wrist rows."""
    text = document.read_text(encoding="utf-8")
    body = text.split("## 当前可读公式\n", 1)[1].split("\n## ", 1)[0]
    marker = "### 大腕角追加公式\n"
    base_body = body.split(marker, 1)[0]
    def numbers(cell):
        return [float(value) for value in re.findall(
            r"[+-]?(?:\d+(?:\.\d*)?|\.\d+|inf)", cell.replace("∞", "inf"))]
    rows = []
    for line in base_body.splitlines():
        if line.startswith("| ["):
            fields = [numbers(cell) for cell in line.strip("|").split("|")]
            assert [len(field) for field in fields] == [2, 2, 1, 2, 1, 1]
            rows.append(fields)
    assert len(rows) == 7
    outer_rows = []
    if marker in body:
        for line in body.split(marker, 1)[1].splitlines():
            if line.startswith("| ["):
                cells = [cell.strip() for cell in line.strip("|").split("|")]
                fifth, wrist, kind, shoulder = numbers(cells[0]), numbers(cells[1]), cells[2], numbers(cells[3])
                assert len(fifth) == len(wrist) == len(shoulder) == 2 and kind in ("S", "T")
                outer_rows.append((fifth, wrist, kind, shoulder))
        assert len(outer_rows) == 8
    s_weight = float(re.search(r"S=([\d.]+)\*q2\+q3", body)[1])
    t_weight = float(re.search(r"T=([\d.]+)\*q2\+q3", body)[1])
    center = float(re.search(r"d4=abs\(abs\(q4\)-([\d.]+)\)", body)[1])
    middle_radius, middle_upper = map(float, re.search(
        r"abs\(q4\) ≥ ([\d.]+) 且 L_A ≤ S ≤ ([\d.]+)", body).groups())
    inner_radius, inner_lower, inner_upper = map(float, re.search(
        r"abs\(q4\) ≤ ([\d.]+) 且 ([-\d.]+) ≤ S ≤ ([-\d.]+)", body).groups())
    t_small_radius = float(re.search(r"abs\(q4\) ≤ ([\d.]+) 且 L_T ≤ T ≤ U_T", body)[1])
    t_large_radius, t_large_lower = map(float, re.search(
        r"abs\(q4\) ≤ ([\d.]+) 且 ([-\d.]+) ≤ T ≤ U_T", body).groups())
    domain = np.array([[0., 0.]]+[
        numbers(re.search(rf"q{joint} ∈ \[([^\]]+)\]", body)[1]) for joint in (2, 3, 4)
    ]+[[min(row[0][0] for row in rows), max(row[0][1] for row in rows)]])
    def written_membership(points):
        s = s_weight*points[:, 1]+points[:, 2]
        t = t_weight*points[:, 1]+points[:, 2]
        wrist, allowed = abs(points[:, 3]), np.zeros(len(points), dtype=bool)
        for fifth, a, radius, b, t_lower, t_upper in rows:
            allowed |= ((points[:, 4] >= fifth[0]) & (points[:, 4] <= fifth[1])
                        & (((s >= a[0]) & (s <= a[1]))
                           | ((wrist >= middle_radius) & (s >= a[0]) & (s <= middle_upper))
                           | ((abs(wrist-center) <= radius[0]) & (s >= b[0]) & (s <= b[1]))
                           | ((wrist <= inner_radius) & (s >= inner_lower) & (s <= inner_upper))
                           | ((wrist <= t_small_radius) & (t >= t_lower[0]) & (t <= t_upper[0]))
                           | ((wrist <= t_large_radius) & (t >= t_large_lower) & (t <= t_upper[0]))))
        for fifth, window, kind, shoulder in outer_rows:
            value = s if kind == "S" else t
            allowed |= ((points[:, 4] >= fifth[0]) & (points[:, 4] <= fifth[1])
                        & (wrist >= window[0]) & (wrist <= window[1])
                        & (value >= shoulder[0]) & (value <= shoulder[1]))
        return allowed & np.all((points >= domain[:, 0]) & (points <= domain[:, 1]), axis=1)
    wrist_edges = [*domain[3], 0., -middle_radius, middle_radius, -inner_radius, inner_radius]
    wrist_edges += [sign*(center+offset*row[2][0])
                    for row in rows for sign in (-1, 1) for offset in (-1, 1)]
    wrist_edges += [sign*edge for _, window, _, _ in outer_rows
                    for edge in window if math.isfinite(edge) for sign in (-1, 1)]
    wrist_edges = np.unique(wrist_edges)
    wrists = np.unique(np.r_[wrist_edges, np.nextafter(wrist_edges, -np.inf),
                            np.nextafter(wrist_edges, np.inf)])
    points = [np.random.default_rng(2026111707).uniform(domain[:, 0], domain[:, 1], (6000, 5))]
    for fifth, a, radius, b, t_lower, t_upper in rows:
        fifths = np.unique(np.r_[fifth, np.mean(fifth), np.nextafter(fifth, -np.inf),
                                np.nextafter(fifth, np.inf)])
        for q2 in (domain[1, 0], -35., -20., 0., 20., 40., domain[1, 1]):
            thirds = np.array([value-s_weight*q2 for value in [*a, *b, middle_upper, inner_lower, inner_upper]
                               if math.isfinite(value)]
                              +[value-t_weight*q2 for value in [t_lower[0], t_upper[0], t_large_lower]]
                              +list(domain[2]))
            thirds = np.unique(np.r_[thirds, np.nextafter(thirds, -np.inf), np.nextafter(thirds, np.inf)])
            q3, q4, q5 = (grid.ravel() for grid in np.meshgrid(thirds, wrists, fifths, indexing="ij"))
            points.append(np.column_stack((np.zeros(len(q3)), np.full(len(q3), q2), q3, q4, q5)))
            # In real algebra each T interval overlaps the shared inner-S interval.
            for lower in (t_lower[0], t_large_lower):
                assert lower-t_weight*q2 <= inner_upper-s_weight*q2
                assert inner_lower-s_weight*q2 <= t_upper[0]-t_weight*q2
    for fifth, _, kind, shoulder in outer_rows:
        fifths = np.unique(np.r_[fifth, np.mean(fifth), np.nextafter(fifth, -np.inf),
                                np.nextafter(fifth, np.inf)])
        weight = s_weight if kind == "S" else t_weight
        for q2 in (domain[1, 0], -35., -20., 0., 20., 40., domain[1, 1]):
            # Every added interval meets the original common S=10 anchor.
            assert shoulder[0] <= 10.+(weight-s_weight)*q2 <= shoulder[1]
            thirds = np.r_[np.asarray(shoulder)-weight*q2, domain[2]]
            thirds = np.unique(np.r_[thirds, np.nextafter(thirds, -np.inf), np.nextafter(thirds, np.inf)])
            q3, q4, q5 = (grid.ravel() for grid in np.meshgrid(thirds, wrists, fifths, indexing="ij"))
            points.append(np.column_stack((np.zeros(len(q3)), np.full(len(q3), q2), q3, q4, q5)))
    points = np.concatenate(points)
    points = np.vstack((points, [1e-9, 0., 0., 0., 5.], [-1e-9, 0., 0., 0., 5.]))
    expected = written_membership(points)
    results, identities = [], {}
    for mapping in ("baseline", "alignment"):
        family = "outer" if outer_rows else "affine"
        directory = build / f"workspace_{family}_rule_20261117_{mapping}_v1"
        comparison_path = directory / "compact_rule_comparison.json"
        summary_path = directory / "coupled_summary.json"
        comparison, summary = read(comparison_path), read(summary_path)
        assert summary["cases"] == summary["accepted"] == (60 if outer_rows else 44)
        assert summary["collidingTraceSamples"] == 0
        for q2 in domain[1]:
            anchor_q3 = 10.-s_weight*q2
            assert domain[2, 0] <= anchor_q3 <= domain[2, 1]
            for band in comparison["combinedRule"]["bands"]:
                value = (band.get("J2Weight", 1.5)-s_weight)*q2+10.
                assert band["weightedLowerDeg"] <= value <= band.get("weightedUpperDeg", math.inf)
        matched = set()
        for fifth, window, kind, shoulder in outer_rows:
            wrist_windows = intersect_intervals(
                [[-window[1], -window[0]], window], [domain[3].tolist()])
            indices = [index for index, band in enumerate(comparison["combinedRule"]["bands"])
                       if band["J5Deg"] == fifth
                       and band.get("J2Weight", 1.5) == (s_weight if kind == "S" else t_weight)
                       and band["weightedLowerDeg"] == shoulder[0]
                       and band.get("weightedUpperDeg", math.inf) == shoulder[1]
                       and intersect_intervals(compact_wrist_intervals(band), [domain[3].tolist()]) == wrist_windows]
            assert len(indices) == 1, (mapping, fifth, window, kind, shoulder, indices)
            matched.update(indices)
        assert len(matched) == len(outer_rows)
        machine = compact_membership(comparison["combinedRule"], points)
        assert np.array_equal(expected, machine), (mapping, int((expected != machine).sum()))
        for path in (comparison_path, summary_path):
            identities[str(path.resolve())] = hashlib.sha256(path.read_bytes()).hexdigest()
        results.append(dict(mapping=mapping, checkedPoints=len(points), mismatches=0,
                            acceptedPoints=int(machine.sum()), wholeNewBandIdentitiesMatched=len(matched),
                            commonAnchorVerifiedForAllBands=True))
    coarse_matches = []
    if extension is not None:
        coarse_body = text.split("## 八块增量公式与边界联动结果（2026-11-17）\n", 1)[1].split("\n## ", 1)[0]
        assert ("I3_new(q2,q4,q5) = I3_old(q2,q4,q5) ∪ "
                "⋃{ A3k | q2∈A2k, q4∈A4k, q5∈A5k }") in coarse_body
        boxes = []
        for line in coarse_body.splitlines():
            if line.startswith("| C"):
                fields = line.strip("|").split("|")
                assert fields[0].strip() == f"C{len(boxes)+1}" and len(fields) == 5
                second, fourth, fifth, third = [numbers(cell) for cell in fields[1:]]
                assert all(len(interval) == 2 and interval[0] <= interval[1]
                           for interval in (second, third, fourth, fifth))
                boxes.append([[0., 0.], second, third, fourth, fifth])
        assert np.asarray(boxes).shape == (8, 5, 2) and np.isfinite(boxes).all()
        for index, mapping in enumerate(("baseline", "alignment")):
            path = extension / mapping / "gap_refinement.json"
            report = read(path)
            assert report["refinementMode"] == "direct_corridors_coarse_common"
            assert report["constraintsAccountedFor"] == 89 and report["completeOtherGroupsInherited"] == 7
            assert report["mappingOrder"][index] == mapping
            assert np.array_equal(boxes, [window["focusSoftwareBoundsDeg"] for window in report["windows"]])
            leaf_counts = []
            offsets = [np.asarray(read(extension / name / "config.json")["offsetCadDeg"])
                       for name in report["mappingOrder"]]
            for window in report["windows"]:
                attempt = report["coarseningAttempts"][window["selectedAttempt"]]
                assert attempt["wholeBoxProven"] and attempt["softwareBoundsDeg"] == window["focusSoftwareBoundsDeg"]
                assert len(attempt["boundsPerMapping"]) == len(offsets) == 2
                counts = [audit_partition_cover(proof, np.asarray(attempt["softwareBoundsDeg"])+offset[:, None])
                          for proof, offset in zip(attempt["boundsPerMapping"], offsets)]
                leaf_counts.append(counts)
            coarse_matches.append(dict(mapping=mapping, exactWrittenBoxes=len(boxes),
                                       completeSelectedProofsInspected=len(report["windows"]),
                                       exactPartitionLeafCountsPerMapping=leaf_counts))
            for name in report["mappingOrder"]:
                config_path = extension / name / "config.json"
                identities[str(config_path.resolve())] = hashlib.sha256(config_path.read_bytes()).hexdigest()
            with path.open("rb") as stream:
                identities[str(path.resolve())] = hashlib.file_digest(stream, "sha256").hexdigest()
    identities[str(document.resolve())] = hashlib.sha256(document.read_bytes()).hexdigest()
    out.mkdir(parents=True, exist_ok=False)
    write(out / "readable_formula_audit.json", dict(results=results, inputFileSha256=identities,
          implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
          comparisonToleranceDeg=0.,
          outerWristRows=len(outer_rows),
          coarseWrittenRowsMatchedPerMapping=coarse_matches,
          limitations=["Written OR formula checked at random, nonzero-shoulder and nextafter boundary points",
                       "Translated J3 interval is real algebra, not an alternative floating-point implementation",
                       "Does not replace the independently saved whole-region proofs or physical calibration"]))
    print(dict(compactFormula=results, coarseWrittenRows=coarse_matches), flush=True)


def resample(build, out):
    """A fresh recorded-domain sample, never used in candidate selection."""
    identities, results = {}, []
    for mapping in ("baseline", "alignment"):
        report_path = build / f"workspace_factored_regions_20261117_{mapping}_v10/factored_regions.json"
        old_path = build / f"workspace_affine_rule_20261117_{mapping}_v1/compact_rule_comparison.json"
        new_path = build / f"workspace_outer_rule_20261117_{mapping}_v1/compact_rule_comparison.json"
        report, old, new = read(report_path), read(old_path)["combinedRule"], read(new_path)["combinedRule"]
        assert all(band in new["bands"] for band in old["bands"])
        recorded, domain = np.asarray(report["recordedSoftwareRangesDeg"]), np.asarray(new["coordinateDomainSoftwareDeg"])
        points = np.random.default_rng(2026111709).uniform(recorded[:, 0], recorded[:, 1], (6000, 5))
        before, after = compact_membership(old, points), compact_membership(new, points)
        table = factored_membership(report["groups"], points)
        table &= np.all((points >= domain[:, 0]) & (points <= domain[:, 1]), axis=1)
        assert not np.any(before & ~after) and not np.any(after & ~table)
        results.append(dict(mapping=mapping, previousReadableAccepted=int(before.sum()),
                            expandedReadableAccepted=int(after.sum()), newlyAccepted=int((after & ~before).sum()),
                            fullTableAccepted=int(table.sum()), samplesOutsideFullTable=int((after & ~table).sum()),
                            previousBandsRetained=len(old["bands"]), expandedBands=len(new["bands"])))
        for path in (report_path, old_path, new_path):
            with path.open("rb") as stream:
                identities[str(path.resolve())] = hashlib.file_digest(stream, "sha256").hexdigest()
        del report
    out.mkdir(parents=True, exist_ok=False)
    write(out / "independent_resample.json", dict(seed=2026111709, samples=6000, results=results,
          inputFileSha256=identities, implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
          limitations=["Joint-coordinate sample, not Cartesian volume, whole-region proof or maximality"]))
    print(results, flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("survey", "outer-survey", "self-test", "readable-audit", "resample",
                                          "simplify", "joint3-query", "joint3-audit",
                                          "joint3-paths", "joint3-paths-audit", "gap-refine", "gap-audit",
                                          "direct-coordination-audit", "coarse-refine", "frontier-survey",
                                          "frontier-refine", "frontier-intervals", "frontier-paths",
                                          "frontier-paths-audit", "joint3-certified", "frontier-grow",
                                          "frontier-connect", "frontier-balanced", "frontier-collect"))
    parser.add_argument("--build", type=Path, default=Path("build"))
    parser.add_argument("--out", type=Path)
    parser.add_argument("--document", type=Path, default=Path("tools/urdf_collision/workspace_study.md"))
    parser.add_argument("--software", nargs=5, type=float)
    parser.add_argument("--source", type=Path)
    parser.add_argument("--resume", type=Path)
    parser.add_argument("--mapping", choices=("baseline", "alignment"))
    parser.add_argument("--extension", type=Path)
    parser.add_argument("--previous-extension", type=Path)
    parser.add_argument("--candidates", type=Path)
    parser.add_argument("--corridors", type=Path)
    parser.add_argument("--large-coordination", action="store_true")
    parser.add_argument("--coarse-boundaries", action="store_true")
    parser.add_argument("--motion-bounds", action="store_true")
    parser.add_argument("--focus-scale", type=float, default=1.)
    args = parser.parse_args()
    if args.action == "self-test":
        self_test()
    elif args.action in ("survey", "outer-survey"):
        assert args.out is not None
        survey(args.build, args.out, outer=args.action == "outer-survey")
    elif args.action == "resample":
        assert args.out is not None
        resample(args.build, args.out)
    elif args.action == "simplify":
        assert args.out is not None
        simplify(args.build, args.out)
    elif args.action == "joint3-audit":
        assert args.out is not None
        joint3_audit(args.build, args.out)
    elif args.action == "frontier-survey":
        assert args.out is not None and args.extension is not None
        frontier_survey(args.build, args.extension, args.out)
    elif args.action == "frontier-refine":
        assert args.out is not None and args.extension is not None and args.source is not None
        frontier_regions(args.build, args.extension, args.source, args.out)
    elif args.action == "frontier-grow":
        assert args.out is not None and args.extension is not None and args.source is not None
        frontier_grow(args.build, args.extension, args.source, args.out)
    elif args.action == "frontier-balanced":
        assert args.out is not None and args.extension is not None and args.source is not None
        frontier_balanced(args.build, args.extension, args.source, args.out)
    elif args.action == "frontier-collect":
        assert args.out is not None and args.source is not None
        frontier_collect(args.build, args.source, args.out)
    elif args.action == "frontier-connect":
        assert args.out is not None and args.extension is not None and args.source is not None
        frontier_connect(args.build, args.extension, args.source, args.out)
    elif args.action == "frontier-intervals":
        assert args.out is not None and args.extension is not None and args.source is not None
        frontier_intervals(args.build, args.extension, args.source, args.out)
    elif args.action == "joint3-certified":
        assert args.out is not None and args.extension is not None and args.software is not None
        frontier_intervals(args.build, args.extension, None, args.out, args.software)
    elif args.action == "frontier-paths":
        assert args.out is not None and args.extension is not None and args.source is not None
        frontier_paths(args.build, args.extension, args.source, args.out, args.resume)
    elif args.action == "frontier-paths-audit":
        assert args.extension is not None and args.source is not None
        frontier_paths_audit(args.build, args.extension, args.source)
    elif args.action == "joint3-paths":
        assert args.out is not None
        joint3_paths(args.build, args.out, args.extension, args.previous_extension, args.candidates,
                     args.large_coordination, args.corridors, args.coarse_boundaries)
    elif args.action == "joint3-paths-audit":
        assert args.source is not None
        joint3_paths_audit(args.build, args.source, args.out, args.extension, args.previous_extension)
    elif args.action == "gap-refine":
        assert args.out is not None and args.mapping is not None
        gap_refine(args.build, args.mapping, args.out, args.focus_scale, args.extension, args.corridors)
    elif args.action == "gap-audit":
        assert args.out is not None and args.extension is not None
        gap_audit(args.build, args.extension, args.out, args.previous_extension)
    elif args.action == "coarse-refine":
        assert args.out is not None and args.extension is not None
        coarse_regions(args.build, args.extension, args.out)
    elif args.action == "direct-coordination-audit":
        assert args.source is not None and args.out is not None
        direct_coordination_audit(args.source, args.out, args.motion_bounds)
    elif args.action == "joint3-query":
        assert args.software is not None
        tables, identities = joint3_tables(args.build, args.extension)
        common, own = joint3_query(tables, args.software)
        print(dict(softwareDegrees=args.software, commonJ3IntervalsDeg=common,
                   separateMappingJ3IntervalsDeg=own, status="Offline conditional model only",
                   inputFileSha256=identities), flush=True)
    else:
        assert args.out is not None
        readable_audit(args.build, args.document, args.out, args.extension)
