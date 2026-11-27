"""Offline common-progress cruise/S-turn experiment. NO board or serial access.

ponytail: a finite straight-line family, not an arbitrary-path global solver.
Keep the current firmware/controller untouched until this representation has
an MCU implementation and a separately authorized physical comparison.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from functools import cached_property
import hashlib
import itertools
import json
import math
from pathlib import Path
import subprocess
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OLD = ROOT / "build/v12-lookaround-relative-arm-20261127/long_continuous_01"
RATIO = np.array([1., .4, -.8, -1., -.36])
VMAX = np.full(5, .8)
AMAX = np.array([2., 2., 4., 2., 4.])
JMAX = np.array([10., 10., 20., 10., 20.])
FLOOR = .16


def velocity_transition(v0, v1, acceleration, jerk):
    """Minimum-time zero-acceleration to zero-acceleration velocity change."""
    values = np.array([v0, v1, acceleration, jerk], float)
    if not np.isfinite(values).all() or acceleration <= 0 or jerk <= 0:
        raise ValueError("Expected finite velocity and positive A/J")
    dv = abs(v1-v0)
    if dv == 0:
        return []
    tj = min(acceleration/jerk, math.sqrt(dv/jerk))
    ta = max(0., dv/acceleration-tj)
    direction = math.copysign(1., v1-v0)
    return [(tj, direction*jerk), (ta, 0.), (tj, -direction*jerk)]


def integrate(state, seconds, jerk):
    x, v, a = state
    return np.array([x+v*seconds+a*seconds**2/2+jerk*seconds**3/6,
                     v+a*seconds+jerk*seconds**2/2, a+jerk*seconds])


@dataclass
class RawCurve:
    segments: np.ndarray  # start time, x, v, a, j, duration; SI scalar units.
    duration: float
    velocity: float
    acceleration: float
    jerk: float
    cycles: int
    first_turn_start: float
    turn_duration: float
    entry_duration: float
    cruise_duration: float

    @cached_property
    def position_integral_prefix(self):
        _, x, v, a, j, h = self.segments.T
        areas = x*h+v*h*h/2+a*h**3/6+j*h**4/24
        return np.r_[0., np.cumsum(areas)]

    def sample(self, seconds):
        t = np.asarray(seconds, float)
        if not np.isfinite(t).all():
            raise ValueError("Nonfinite sampling time")
        index = np.clip(np.searchsorted(self.segments[:, 0], t, side="right")-1,
                        0, len(self.segments)-1)
        row = self.segments[index]
        h = t-row[..., 0]
        x = row[..., 1]+row[..., 2]*h+row[..., 3]*h*h/2+row[..., 4]*h**3/6
        v = row[..., 2]+row[..., 3]*h+row[..., 4]*h*h/2
        a = row[..., 3]+row[..., 4]*h
        result = np.stack((x, v, a, row[..., 4]), axis=-1)
        # The finite experiment begins AND ends at its origin, at rest.
        return np.where(((t < 0) | (t >= self.duration))[..., None], 0., result)

    def integral(self, seconds):
        """Exact antiderivatives of finite rest-to-rest x/v/a/j."""
        t = np.asarray(seconds, float)
        index = np.clip(np.searchsorted(self.segments[:, 0], t, side="right")-1,
                        0, len(self.segments)-1)
        row = self.segments[index]
        h = np.clip(t-row[..., 0], 0., row[..., 5])
        area = (self.position_integral_prefix[index]+row[..., 1]*h+
                row[..., 2]*h*h/2+row[..., 3]*h**3/6+row[..., 4]*h**4/24)
        area = np.where(t <= 0, 0., area)
        area = np.where(t >= self.duration, self.position_integral_prefix[-1], area)
        return np.concatenate((area[..., None], self.sample(t)[..., :3]), axis=-1)


def make_raw(velocity, acceleration, jerk, *, cycles=None, target_seconds=78.):
    if velocity <= 0 or not math.isfinite(velocity):
        raise ValueError("Expected positive finite cruise")
    entry = velocity_transition(0., velocity, acceleration, jerk)
    turn = velocity_transition(velocity, -velocity, acceleration, jerk)
    te, tr = sum(p[0] for p in entry), sum(p[0] for p in turn)
    state = np.array([0., velocity, 0.])
    remaining = tr/2
    for duration, value in turn:
        step = min(remaining, duration)
        state = integrate(state, step, value)
        remaining -= step
        if remaining <= 1e-12:
            break
    turn_excursion = state[0]
    xe = velocity*te/2
    cruise = (1.-2*turn_excursion)/velocity
    partial = (.5-turn_excursion-xe)/velocity
    if cruise <= 0 or partial < -1e-12:
        raise ValueError("Insufficient retained excursion for entry/turn")
    period = 2*(cruise+tr)
    count = max(3, round((target_seconds-te)/period)) if cycles is None else cycles
    if not isinstance(count, int) or count < 1:
        raise ValueError("Expected positive integer cycles")
    rows, clock, state = [], 0., np.zeros(3)
    def append(duration, value=0.):
        nonlocal clock, state
        if duration <= 1e-12:
            return
        rows.append([clock, *state, value, duration])
        state = integrate(state, duration, value)
        clock += duration
    for duration, value in entry:
        append(duration, value)
    append(max(0., partial))
    first_turn = clock
    for cycle in range(count):
        for duration, value in turn:
            append(duration, value)
        append(cruise)
        for duration, value in velocity_transition(-velocity, velocity, acceleration, jerk):
            append(duration, value)
        append(cruise if cycle < count-1 else max(0., partial))
    for duration, value in velocity_transition(velocity, 0., acceleration, jerk):
        append(duration, value)
    if np.max(abs(state)) > 1e-9:
        raise ValueError(f"Finite curve does not close: {state}")
    return RawCurve(np.array(rows), clock, velocity, acceleration, jerk, count,
                    first_turn, tr, te, cruise)


class SmoothedCurve:
    """Exact continuous box average of the SAME position/derivative chain.

    Bounded, piecewise-constant raw jerk becomes continuous piecewise-linear
    jerk. Unlike a discrete FIR, there are no residual jerk steps or dead taps.
    The average is analytic, not a numerical moving average of sampled q.
    """
    def __init__(self, raw, half_window):
        if not math.isfinite(half_window) or half_window < 0:
            raise ValueError("Expected nonnegative finite smoothing window")
        self.raw = raw
        self.half_window = half_window
        self.width = 2*half_window
        self.duration = raw.duration+self.width
        self.scale = 1.
        peak_time = raw.first_turn_start+raw.turn_duration/2+self.half_window
        peak = self.sample(peak_time)[0]
        self.scale = .5/peak  # Restore identical geometric extrema, then recheck.

    def sample(self, seconds):
        t = np.asarray(seconds, float)
        if not np.isfinite(t).all():
            raise ValueError("Nonfinite sampling time")
        result = (self.raw.sample(t) if self.width == 0 else
                  (self.raw.integral(t)-self.raw.integral(t-self.width))/self.width)
        return result*self.scale

    def grid(self, dt):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("Expected positive finite sample period")
        times = np.r_[np.arange(0, self.duration, dt), self.duration]
        return times, self.sample(times)


def longest(mask, dt):
    edges = np.flatnonzero(np.diff(np.r_[False, mask, False]))
    return float(np.max(edges[1::2]-edges[::2], initial=0)*dt)


def metrics(curve, times, states):
    dt = times[1]-times[0]
    x, v, a, j = states.T
    ratios = abs(RATIO)
    # Whole experiment: no long low-speed interval can be hidden by labels.
    low = abs(v[:, None]*RATIO) < FLOOR-1e-10
    # Interval-weighted fractions include zero crossings, but not endpoint rest.
    fraction = (np.diff(times)[:, None]*low[:-1]).sum(axis=0)/curve.duration
    peak = np.max(abs(states[:, 1:]), axis=0)
    cruise = curve.raw.velocity*curve.scale*ratios
    body = ((times >= curve.raw.entry_duration+curve.width) &
            (times <= curve.raw.duration-curve.raw.entry_duration))
    return dict(duration_s=curve.duration, cycles=curve.raw.cycles,
        scalar_scale=curve.scale,
        cruise_abs_rad_s=cruise.tolist(),
        peak_velocity_rad_s=(peak[0]*ratios).tolist(),
        peak_acceleration_rad_s2=(peak[1]*ratios).tolist(),
        peak_jerk_rad_s3=(peak[2]*ratios).tolist(),
        low_speed_fraction=fraction.tolist(),
        longest_below_floor_s=[longest(low[:, i], dt) for i in range(5)],
        longest_reversal_below_floor_s=[
            longest(low[:, i] & body, dt) for i in range(5)],
        zero_velocity_dwell_s=max(0., longest(abs(v[1:-1]) < 1e-10, dt)-dt),
        scalar_jerk_rms=float(np.sqrt(np.mean(j*j))),
        scalar_max_sampled_jerk_step=float(np.max(abs(np.diff(j)), initial=0)),
        scalar_min=float(x.min()), scalar_max=float(x.max()),
        scalar_travel_rad=float(np.sum(abs(np.diff(x)))),
        low_speed_includes_start_end_and_reversal=True)


def feasible(m):
    return (np.min(m["cruise_abs_rad_s"]) >= FLOOR-1e-9 and
            np.all(np.asarray(m["peak_velocity_rad_s"]) <= VMAX+1e-9) and
            np.all(np.asarray(m["peak_acceleration_rad_s2"]) <= AMAX+1e-9) and
            np.all(np.asarray(m["peak_jerk_rad_s3"]) <= JMAX+1e-9) and
            m["scalar_min"] >= -.50000001 and m["scalar_max"] <= .50000001)


def objective(m, weights=(.55, .20, .15, .10)):
    # Explicit engineering tradeoff, NOT a user acceptance criterion.
    components = [max(m["low_speed_fraction"]),
                  (m["scalar_jerk_rms"]/10)**2,
                  (m["scalar_max_sampled_jerk_step"]/10)**2,
                  ((m["duration_s"]-78)/78)**2]
    return float(np.dot(weights, components))


def search():
    records, rejected = [], 0
    started = time.perf_counter()
    for velocity, acceleration, jerk in itertools.product(
            np.round(np.arange(.46, .801, .02), 8), (1., 1.5, 2.), (3., 5., 8.)):
        try:
            raw = make_raw(float(velocity), acceleration, jerk)
        except ValueError:
            rejected += 5
            continue
        for window in (0., .02, .04, .06, .08):
            curve = SmoothedCurve(raw, window)
            t, state = curve.grid(.004)
            m = metrics(curve, t, state)
            if not feasible(m):
                rejected += 1
                continue
            records.append(dict(velocity=float(velocity), acceleration=acceleration,
                jerk=jerk, half_window=window, objective=objective(m), metrics=m))
    if not records:
        raise ValueError("No feasible finite-family candidate")
    records.sort(key=lambda row: row["objective"])
    return records, dict(attempted=18*3*3*5, feasible=len(records), rejected=rejected,
        elapsed_s=time.perf_counter()-started, sample_period_s=.004,
        objective_weights=[.55, .20, .15, .10],
        objective_components=["worst_joint_whole_low_speed_fraction",
            "(scalar_jerk_rms/10)^2", "(max_4ms_jerk_step/10)^2",
            "((duration-78)/78)^2"],
        claim="finite_grid_minimum_only_not_global_or_time_optimal")


def native_check(executable, raw):
    reply = json.loads(subprocess.check_output(
        [str(executable.resolve()), str(raw.velocity), str(raw.acceleration), str(raw.jerk)],
        text=True))
    result = []
    for record, v0, v1 in zip(reply["transitions"], (0., raw.velocity),
                              (raw.velocity, -raw.velocity)):
        phases = velocity_transition(v0, v1, raw.acceleration, raw.jerk)
        rows, clock, state = [], 0., np.array([0., v0, 0.])
        for duration, value in phases:
            if duration > 1e-12:
                rows.append([clock, *state, value, duration])
                state = integrate(state, duration, value)
                clock += duration
        r = RawCurve(np.array(rows), clock, 0, 0, 0, 1, 0, 0, 0, 0)
        samples = np.array(record["samples"])
        expected = r.sample(samples[:, 0])
        # Native endpoint keeps target velocity; RawCurve's rest extension is
        # only for the full finite experiment. Compare interior samples here.
        interior = samples[:, 0] < clock-1e-10
        error = np.max(abs(expected[interior, :3]-samples[interior, 1:4]), axis=0)
        if abs(record["duration_s"]-clock) > 1e-10 or np.max(error) > 1e-9:
            raise ValueError("Independent local Ruckig transition mismatch")
        result.append(dict(v0=v0, v1=v1, duration_s=clock,
            max_abs_x_v_a_difference=error.tolist(), samples=int(interior.sum())))
    return dict(passed=True, local_ruckig_velocity_interface=result)


def verify_kinematics(curve):
    t, states = curve.grid(.001)
    m = metrics(curve, t, states)
    if not feasible(m):
        raise ValueError("Independent 1ms post-smoothing constraints failed")
    q, v, a, j = states.T
    # Central differences use uniform interior; exclude final fractional step.
    e1 = np.max(abs((q[2:-2]-q[:-4])/.002-v[1:-3]))
    e2 = np.max(abs((v[2:-2]-v[:-4])/.002-a[1:-3]))
    # Across jerk steps, derivative a is a time average, not endpoint jerk.
    derivative_a = abs(np.diff(a)/np.diff(t))
    if np.max(derivative_a) > curve.raw.jerk*curve.scale+1e-7:
        raise ValueError("Finite-difference jerk bound failed")
    if e1 > 5e-6 or e2 > .003:
        raise ValueError("Position/velocity/acceleration derivative chain mismatch")
    signs = np.sign(v[abs(v) > 1e-8])
    reversals = int(np.count_nonzero(signs[1:] != signs[:-1]))
    if reversals != 2*curve.raw.cycles or np.max(abs(states[[0, -1], :3])) > 1e-10:
        raise ValueError("Extra reversal or non-rest endpoint")
    return dict(passed=True, sample_period_s=.001, samples=len(t),
        metrics=m, max_central_q_derivative_error=e1,
        max_central_v_derivative_error=e2, sign_reversals=reversals,
        expected_sign_reversals=2*curve.raw.cycles,
        continuous_q_v_a=True, continuous_jerk=curve.width > 0,
        endpoint_origin_and_rest=True)


def continuous_extrema_check(curve):
    """All extrema of the piecewise-polynomial curve, not just a time grid."""
    raw = curve.raw
    boundaries = np.r_[raw.segments[:, 0], raw.duration]
    events = np.unique(np.r_[boundaries, boundaries+curve.width])
    candidates, polynomial_error, snap_peak = list(events), 0., 0.
    for start, stop in zip(events[:-1], events[1:]):
        if stop-start < 1e-12:
            continue
        state = curve.sample(start)
        middle = (start+stop)/2
        snap = (0. if curve.width == 0 else curve.scale*(
            raw.sample(middle)[3]-raw.sample(middle-curve.width)[3])/curve.width)
        snap_peak = max(snap_peak, abs(snap))
        x, v, a, j = state
        h = middle-start
        polynomial = np.array([x+v*h+a*h*h/2+j*h**3/6+snap*h**4/24,
            v+a*h+j*h*h/2+snap*h**3/6, a+j*h+snap*h*h/2, j+snap*h])
        polynomial_error = max(polynomial_error,
                               float(np.max(abs(polynomial-curve.sample(middle)))))
        # dx/dt = cubic; dv/dt = quadratic; da/dt = linear; dj/dt constant.
        for coefficients in ((snap/6, j/2, a, v), (snap/2, j, a), (snap, j)):
            nonzero = np.trim_zeros(np.asarray(coefficients), "f")
            if len(nonzero) < 2:
                continue
            for root in np.roots(nonzero):
                if abs(root.imag) < 1e-9 and 0 < root.real < stop-start:
                    candidates.append(start+float(root.real))
    values = curve.sample(np.array(candidates))
    peak = np.max(abs(values[:, 1:]), axis=0)
    limits_ok = (np.all(peak[0]*abs(RATIO) <= VMAX+1e-9) and
                 np.all(peak[1]*abs(RATIO) <= AMAX+1e-9) and
                 np.all(peak[2]*abs(RATIO) <= JMAX+1e-9))
    if (not limits_ok or np.max(abs(values[:, 0])) > .500000001 or
            polynomial_error > 1e-8):
        raise ValueError("Continuous polynomial extrema/identity check failed")
    return dict(passed=True, polynomial_intervals=len(events)-1,
        extrema_and_boundary_evaluations=len(candidates),
        scalar_min=float(values[:, 0].min()), scalar_max=float(values[:, 0].max()),
        peak_velocity_rad_s=(peak[0]*abs(RATIO)).tolist(),
        peak_acceleration_rad_s2=(peak[1]*abs(RATIO)).tolist(),
        peak_jerk_rad_s3=(peak[2]*abs(RATIO)).tolist(),
        scalar_peak_snap_rad_s4=snap_peak,
        max_midinterval_polynomial_identity_error=polynomial_error,
        method="all interval derivative roots plus all boundaries; floating-point not formal proof")


def model_check(origin_deg, curve):
    # Existing model/config, once-only F, NO hardware connection.
    from five_identify import HW, REFS
    from five_gravity_residual_report import support_gravity
    from five_inertia_identify import nominal_torque
    from five_trajectory import reference_friction
    from five_happy_motion import CAPS
    t, scalar = curve.grid(.002)
    q_sw = np.deg2rad(origin_deg)+scalar[:, :1]*RATIO
    v, a = scalar[:, 1:2]*RATIO, scalar[:, 2:3]*RATIO
    q_raw = q_sw+REFS[:, 0]
    margin = np.minimum(q_raw-REFS[:, 1], REFS[:, 2]-q_raw).min(axis=0)
    cad = q_sw+np.deg2rad(HW[:, 6])
    g = support_gravity(cad, clip=False)
    f = reference_friction(cad, v)
    zero = np.zeros_like(cad)
    dynamic = nominal_torque(cad, v, a)-nominal_torque(cad, zero, zero)
    total = g+f+dynamic
    peak, gp = np.max(abs(total), axis=0), np.max(abs(g), axis=0)
    passed = bool(np.all(margin >= math.radians(1)) and
                  np.all(gp <= HW[:, 5]) and np.all(peak <= CAPS))
    return dict(passed=passed, sample_period_s=.002, samples=len(t),
        min_original_travel_margin_deg=np.rad2deg(margin).tolist(),
        nominal_zero_feedback_peak_nm=peak.tolist(),
        peak_dynamic_nm=np.max(abs(dynamic), axis=0).tolist(),
        peak_gravity_nm=gp.tolist(), peak_friction_nm=np.max(abs(f), axis=0).tolist(),
        unchanged_total_budget_nm=CAPS.tolist(),
        budget_remaining_for_feedback_nm=(CAPS-peak).tolist(),
        friction_included_once=True, actual_torque_or_tracking_not_guaranteed=True)


def old_comparison():
    data = np.loadtxt(OLD/"tracking_rad.csv", delimiter=",", skiprows=1)
    rows = []
    for i in range(5):
        axis = data[(data[:, 1] == i+1) & (data[:, 4] >= 3000) & (data[:, 4] <= 75000)]
        rows.append(dict(joint=i+1, samples=len(axis),
            peak_reference_velocity_rad_s=float(np.max(abs(axis[:, 8]))),
            below_016_fraction=float(np.mean(abs(axis[:, 8]) < FLOOR))))
    return dict(source=str(OLD/"tracking_rad.csv"),
        sha256=sha(OLD/"tracking_rad.csv"), body_3_to_75_s=rows,
        different_path_and_cycle_not_causal_AB=True,
        previous_real_stalls_ms=[316, 492, 134, 214, 206],
        new_physical_stalls_and_angle_error_not_measured=True)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest().upper()


def export_csv(path, origin_deg, curve):
    t, scalar = curve.grid(.002)
    table = np.column_stack((t, np.deg2rad(origin_deg)+scalar[:, :1]*RATIO,
        scalar[:, 1:2]*RATIO, scalar[:, 2:3]*RATIO, scalar[:, 3:4]*RATIO))
    names = ["time_s"]+[f"{field}{i}" for field in ("q_rad_", "v_rad_s_",
             "a_rad_s2_", "j_rad_s3_") for i in range(1, 6)]
    np.savetxt(path, table, delimiter=",", header=",".join(names), comments="", fmt="%.12g")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--native", type=Path, required=True)
    parser.add_argument("--skip-cad", action="store_true",
                        help="Explicitly leave CAD unverified (never implies a pass)")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    origin = np.array(json.loads((OLD/"results.json").read_text())["active"]["origin_deg"])
    records, search_report = search()
    best = records[0]
    raw = make_raw(best["velocity"], best["acceleration"], best["jerk"])
    curve = SmoothedCurve(raw, best["half_window"])
    baseline = SmoothedCurve(make_raw(.5, 1., 4., cycles=raw.cycles), 0.)
    unsmoothed = SmoothedCurve(raw, 0.)
    for name, candidate in (("baseline", baseline), ("raw_selected", unsmoothed),
                             ("selected", curve)):
        export_csv(args.output/f"{name}_reference.csv", origin, candidate)
    kinematics = verify_kinematics(curve)
    report = dict(schema="ylamp-offline-cruise-planning-1",
        hardware_access=False, firmware_modified=False, not_deployed=True,
        geometry_family="q_sw=historical_origin+ratio*x, x in [-.5,+.5] rad",
        historical_origin_deg=origin.tolist(), ratio=RATIO.tolist(),
        old_phase_geometry_not_preserved=True, cruise_floor_rad_s=FLOOR,
        start_end_reversal_explicitly_allowed_to_cross_zero=True,
        search=search_report, selected=best, kinematics=kinematics,
        continuous_extrema=continuous_extrema_check(curve),
        local_ruckig_crosscheck=native_check(args.native, raw),
        baseline_ruckig_crosscheck=native_check(args.native, baseline.raw),
        baseline=metrics(baseline, *baseline.grid(.002)),
        baseline_same_excursion_and_cycle_count=True,
        selected_before_smoothing=metrics(unsmoothed, *unsmoothed.grid(.002)),
        model=model_check(origin, curve), old_reference=old_comparison(),
        selection_sensitivity={label:min(records, key=lambda r:objective(r["metrics"], weights))
            for label, weights in (("more_low_speed_weight", (.75, .10, .10, .05)),
                                   ("more_smoothness_weight", (.35, .30, .30, .05)))},
        limits=dict(v_rad_s=VMAX.tolist(), a_rad_s2=AMAX.tolist(), j_rad_s3=JMAX.tolist()),
        caveats=["Finite straight-path candidate family, not an arbitrary curved-path optimizer",
            "4ms search; selected curve independently checked at 1ms and polynomial extrema",
            "Exact positive moving average makes jerk continuous; snap still has steps",
            "Model assumes zero tracking error; actual friction/control and contact are unproved",
            "Existing 32-point Ruckig interface would reshape this curve; not ready to upload",
            "MCU integration, memory, WCET, live initial-state capture and physical AB remain undone"])
    if args.skip_cad:
        report["cad"] = dict(passed=False, skipped=True)
    else:
        from five_identify import HW
        from five_happy_motion import Geometry
        endpoints = [origin+np.rad2deg(RATIO*x)+HW[:, 6] for x in (-.5, .5)]
        with Geometry() as geometry:
            report["cad"] = dict(passed=True, finite_only=True,
                continuous_collision_proof=False, source=geometry.source,
                result=geometry.check([[point.tolist() for point in endpoints]]))
    report["offline_checks_passed"] = bool(report["model"]["passed"] and
        report["kinematics"]["passed"] and report["cad"]["passed"])
    with (args.output/"search.json").open("x", encoding="utf-8") as stream:
        json.dump(records, stream, indent=2, allow_nan=False)
    report["artifacts"] = {path.name:dict(bytes=path.stat().st_size, sha256=sha(path))
        for path in args.output.iterdir() if path.is_file()}
    report["native_executable_sha256"] = sha(args.native)
    report["sources"] = {name:sha(ROOT/name) for name in (
        "tools/five_cruise_plan.py", "tools/five_scalar_ruckig_check.cpp",
        "tools/five_trajectory.py", "tools/five_identify.py", "tools/five_inertia_identify.py",
        "tools/five_gravity_residual_report.py", "config/five_mit.c",
        "config/joint_references.def", "Mode/five_trajectory.cpp",
        "tests/test_five_cruise_plan.py")}
    with (args.output/"report.json").open("x", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    print(json.dumps(dict(output=str(args.output), selected=best, model=report["model"],
        cad=report["cad"], offline_checks_passed=report["offline_checks_passed"],
        hardware_access=False), indent=2, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
