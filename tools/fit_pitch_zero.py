"""Offline encoder/Pitch least squares. Never opens a port or changes references.

Pitch = slope * (encoder - zero). All fresh pairs participate, not just crossings.
The block-bootstrap intervals describe this recording and this affine model;
they do not include mounting, upstream-pose or angle-projection systematic error.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


def affine(q, pitch):
    center = float(np.mean(q))
    x = np.column_stack((q - center, np.ones(len(q))))
    beta, _, rank, _ = np.linalg.lstsq(x, pitch, rcond=None)
    if rank != 2 or abs(beta[0]) < .1:
        raise ValueError("Insufficient angle excitation or wrong Pitch channel")
    slope, intercept = map(float, beta)
    residual = pitch - x @ beta
    total = float(np.sum((pitch - np.mean(pitch)) ** 2))
    return dict(slope=slope, intercept_deg=intercept - slope * center,
                zero_deg=center - intercept / slope,
                rmse_deg=float(np.sqrt(np.mean(residual ** 2))),
                r_squared=1 - float(residual @ residual) / total)


def block_interval(q, pitch, seconds, block_s, repeats, seed):
    """Pairs bootstrap of contiguous time blocks; centered sufficient statistics."""
    center = float(np.mean(q))
    x = np.column_stack((q - center, np.ones(len(q))))
    groups = np.floor(seconds / block_s).astype(int)
    unique = np.unique(groups)
    if len(unique) < 8:
        raise ValueError("Too few time blocks for the requested interval")
    xx = np.array([x[groups == k].T @ x[groups == k] for k in unique])
    xy = np.array([x[groups == k].T @ pitch[groups == k] for k in unique])
    rng = np.random.default_rng(seed)
    selected = rng.integers(0, len(unique), (repeats, len(unique)))
    matrices, rhs = xx[selected].sum(axis=1), xy[selected].sum(axis=1)
    # A resample containing only a dwell has no estimable slope; report exclusions.
    good = np.linalg.det(matrices) > 1e-9
    beta = np.linalg.solve(matrices[good], rhs[good, :, None])[:, :, 0]
    beta = beta[np.abs(beta[:, 0]) > .1]
    if len(beta) < repeats * .95:
        raise ValueError("Too many unidentifiable bootstrap replicates")
    zeros = center - beta[:, 1] / beta[:, 0]
    return dict(block_seconds=block_s, blocks=len(unique), replicates=repeats,
                identifiable_replicates=len(beta), confidence_level=.95,
                zero_deg_interval=np.percentile(zeros, [2.5, 97.5]).tolist(),
                slope_interval=np.percentile(beta[:, 0], [2.5, 97.5]).tolist())


def load(path):
    with path.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames or []
        if "bmi_pitch_deg" in fields and "j2_raw_deg" in fields:
            joint, column = 2, "bmi_pitch_deg"
        elif "pitch_deg" in fields and "j3_raw_deg" in fields:
            joint, column = 3, "pitch_deg"
        else:
            raise ValueError(f"{path}: not a J2/J3 Pitch capture (Roll is not Pitch)")
        rows = list(reader)
    valid = []
    previous = None
    for row in rows:
        if int(row["flags"]) & 15 != 15:
            continue
        q, pitch = float(row[f"j{joint}_raw_deg"]), float(row[column])
        if not np.isfinite([q, pitch]).all() or float(row["imu_mount_offset_deg"]) != 0:
            raise ValueError("Nonfinite data or mixed mounting-offset convention")
        tick, motor_at, imu_at = map(int, (row["tick_ms"], row[f"j{joint}_at_ms"],
                                         row["imu_at_ms"]))
        maximum = int(row["max_age_ms"])
        if (int(row[f"j{joint}_state"]) != 1 or
                not (0 <= tick - motor_at <= maximum and 0 <= tick - imu_at <= maximum)):
            raise ValueError("Fresh flag contradicts drive state or source time")
        counts = (int(row[f"j{joint}_count"]), int(row["imu_count"]))
        if previous and (tick <= previous[0] or
                         any(new <= old for new, old in zip(counts, previous[1]))):
            raise ValueError("Repeated/backward source sample; cannot count as independent pairs")
        previous = tick, counts
        valid.append((q, pitch, tick, motor_at, imu_at))
    data = np.asarray(valid, dtype=float)
    if len(data) < 100 or np.ptp(data[:, 0]) < 10:
        raise ValueError("Insufficient valid angle span")
    return joint, len(rows), data


def analyze(path, repeats=4000):
    joint, total, data = load(path)
    q, pitch, tick, motor_at, imu_at = data.T
    seconds = (tick - tick[0]) / 1000
    result = affine(q, pitch)
    intervals = [block_interval(q, pitch, seconds, block, repeats, 20261119 + joint)
                 for block in (1., 2., 4.)]
    # q+Pitch is the constrained slope=-1 LS estimator, for comparison.
    fixed_zero = float(np.mean(q + pitch))
    fixed_residual = pitch + q - fixed_zero
    fixed = dict(slope=-1., zero_deg=fixed_zero,
                 rmse_deg=float(np.sqrt(np.mean(fixed_residual ** 2))))
    residual = pitch - result["slope"] * (q - result["zero_deg"])
    bands = []
    for low, high in zip(np.arange(q.min(), q.max(), 10),
                         np.arange(q.min() + 10, q.max() + 10, 10)):
        mask = (q >= low) & (q < high)
        if np.any(mask):
            bands.append(dict(encoder_deg=[float(low), float(high)], samples=int(mask.sum()),
                              mean_residual_deg=float(np.mean(residual[mask])),
                              rmse_deg=float(np.sqrt(np.mean(residual[mask] ** 2)))))
    # Sensitivity to long dwells and direction is not hidden inside a narrow CI.
    bins = np.floor((q - q.min()) / 2).astype(int)
    keys = np.unique(bins)
    balanced = affine(np.array([q[bins == k].mean() for k in keys]),
                      np.array([pitch[bins == k].mean() for k in keys]))
    velocity = np.gradient(q, seconds)
    directions = {}
    for name, mask in (("encoder_increasing", velocity > 2),
                       ("encoder_decreasing", velocity < -2)):
        if np.count_nonzero(mask) >= 100 and np.ptp(q[mask]) > 10:
            directions[name] = dict(samples=int(mask.sum()), **affine(q[mask], pitch[mask]))
    cv_error = []
    for fold in range(5):
        test = np.minimum((seconds / (seconds[-1] + .001) * 5).astype(int), 4) == fold
        model = affine(q[~test], pitch[~test])
        cv_error.extend((pitch[test] - model["slope"] * (q[test] - model["zero_deg"])).tolist())
    return dict(joint=joint, source=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                total_rows=total, fresh_pairs=len(q), excluded_warm_or_stale=total-len(q),
                duration_seconds=float(seconds[-1]), encoder_range_deg=[float(q.min()), float(q.max())],
                pitch_range_deg=[float(pitch.min()), float(pitch.max())],
                max_motor_age_ms=float(np.max(tick-motor_at)),
                max_imu_age_ms=float(np.max(tick-imu_at)),
                fit=result, constrained_unit_slope_fit=fixed,
                confidence=intervals[1], interval_block_size_sensitivity=intervals,
                balanced_two_degree_bins_fit=dict(bins=len(keys), **balanced),
                direction_fits=directions, residual_by_encoder_band=bands,
                blocked_five_fold_rmse_deg=float(np.sqrt(np.mean(np.square(cv_error)))),
                deploy=False,
                limitations=["95% is an interval procedure, not probability the physical zero is correct.",
                             "100 Hz rows are time-correlated; IID confidence is not used.",
                             "Intervals are conditional on this affine model and one recording.",
                             "Mounting offset assumed zero; projection/upstream-pose/dynamic errors remain.",
                             "Pitch regression slope is NOT an encoder scale or torque multiplier.",
                             "Angle-only CSV has no torque labels for mass/COM/inertia/friction refitting."])


def self_test():
    rng = np.random.default_rng(7)
    q = np.linspace(-40, 75, 1600)
    pitch = -1.02 * (q - 17.3) + rng.normal(0, .02, len(q))
    result = affine(q, pitch)
    assert abs(result["zero_deg"] - 17.3) < .01
    assert abs(result["slope"] + 1.02) < .001
    interval = block_interval(q, pitch, np.arange(len(q)) * .01, 1, 1000, 7)
    assert interval["zero_deg_interval"][0] < 17.3 < interval["zero_deg_interval"][1]
    try:
        affine(np.ones(100), np.arange(100))
    except ValueError:
        pass
    else:
        raise AssertionError("Unexcited encoder accepted")
    print("PASS: affine zero, block confidence and excitation rejection (synthetic, offline)")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("captures", type=Path, nargs="*")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
    if args.captures:
        report = dict(schema="ylamp-pitch-zero-ls-1", formula="Pitch_deg = slope*(encoder_deg-zero_deg)",
                      joints=[analyze(path) for path in args.captures])
        text = json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False)
        if args.output:
            with args.output.open("x", encoding="utf-8") as stream:
                stream.write(text + "\n")
        print(text)
    elif not args.self_test:
        parser.error("Provide Pitch captures or --self-test")
