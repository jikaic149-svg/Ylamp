"""Offline five-axis gravity-identification admission; never opens hardware."""
import argparse
import ctypes as ct
import hashlib
import json
import math
from pathlib import Path

from dm_monitor import JOINT_REFERENCES
from five_control import EXPECTED, PURE_GRAVITY_PHASES


def assess(pose, line):
    """All angles are live software degrees; J1/J4 remain unwrapped."""
    if len(pose) != 5 or not all(map(math.isfinite, pose)):
        raise ValueError("Expected five finite software angles")
    trials = []
    for joint in range(5):
        for step in (-1.0, 1.0):
            target = list(pose)
            target[joint] += step
            trials.append(dict(joint=joint + 1, step_deg=step,
                               admitted=bool(line(pose, target))))
    return dict(software_deg=pose, current_pose_admitted=bool(line(pose, pose)),
                local_one_degree_trials=trials,
                note="Per-axis range admission only, NOT collision clearance or a motion request; "
                     "no fixed staging/zero pose is required.")


def audit(capture, library):
    records = [json.loads(row) for row in capture.read_text(encoding="utf-8-sig").splitlines()]
    headers = [r for r in records if r.get("stage") == "capture"]
    if len(headers) != 1:
        raise ValueError("Expected exactly one verified capture header")
    refs = headers[0].get("references_rad", [])
    expected = [[r.zero_rad, r.raw_lower_rad, r.raw_upper_rad]
                for r in JOINT_REFERENCES.values()]
    if (len(refs) != 5 or any(len(a) != 3 for a in refs) or
            any(not math.isfinite(a) or abs(a - b) > 2e-6 for row, want in zip(refs, expected)
                for a, b in zip(row, want))):
        raise ValueError("Board calibration does not match current source")
    samples = [r for r in records if r.get("stage") == "sample"]
    if not samples:
        raise ValueError("No controller sample")
    sample = samples[-1]
    if (sample["phase"] not in PURE_GRAVITY_PHASES or sample["fault"] or sample["limited"] or
            any(sample[key] != 31 for key in ("fresh", "enabled", "submitted"))):
        raise ValueError("Capture did not finish with healthy five-axis gravity")
    joints = sample["telemetry"]["joints"]
    if ([(j["joint"], j["bus"], j["protocol"], j["command_id"], j["feedback_id"])
         for j in joints] != EXPECTED or
            not all(j["received"] and j["fresh"] and j["state"] == 1 for j in joints)):
        raise ValueError("Missing fresh enabled five-axis feedback")
    pose = [math.degrees(JOINT_REFERENCES[j["joint"]].angle(j["position_rad"]))
            for j in joints]
    reference = [math.radians(pose[i]) + JOINT_REFERENCES[i + 1].zero_rad
                 for i in range(5)]
    dll = ct.CDLL(str(library.resolve()))
    vector = ct.c_double * 5
    dll.FiveWorkspace_Line.argtypes = [ct.POINTER(ct.c_double)] * 2
    dll.FiveWorkspace_Line.restype = ct.c_bool
    result = assess(pose, lambda a, b: dll.FiveWorkspace_Line(vector(*a), vector(*b)))
    result.update(scope="Offline native per-axis travel admission, no collision proof or actuator writes",
                  capture_sha256=hashlib.sha256(capture.read_bytes()).hexdigest(),
                  workspace_library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),
                  board_tick_ms=sample["telemetry"]["tick_ms"],
                  board_phase=sample["phase"],
                  observed_raw_deg=[math.degrees(j["position_rad"]) for j in joints],
                  selected_reference_deg=list(map(math.degrees, reference)),
                  source_sha256={
                      name: hashlib.sha256((Path(__file__).resolve().parents[1] / name).read_bytes()).hexdigest()
                      for name in ("Model/five_workspace.c", "Model/five_workspace.h",
                                   "config/joint_references.def")},
                  raw_travel_violations=[
                      dict(joint=i + 1, raw_deg=math.degrees(j["position_rad"]),
                           reference_deg=math.degrees(reference[i]),
                           lower_deg=math.degrees(refs[i][1]), upper_deg=math.degrees(refs[i][2]))
                      for i, j in enumerate(joints)
                      if not refs[i][1] <= reference[i] <= refs[i][2]])
    return result


def self_test():
    pose = [60.0, -20.0, 35.0, -232.0, -10.0]
    checked = []

    def line(a, b):
        checked.append((list(a), list(b)))
        return all(math.degrees(ref.lower_rad)+1 <= p[i] <= math.degrees(ref.upper_rad)-1
                   for p in (a, b) for i, ref in enumerate(JOINT_REFERENCES.values()))

    result = assess(pose, line)
    assert result["current_pose_admitted"]
    assert all(r["admitted"] for r in result["local_one_degree_trials"])
    assert len(checked) == 11 and "staging_candidate_software_deg" not in result
    assert checked[0][0][3] == -232.0  # No equivalent-turn escape.
    assert pose == [60.0, -20.0, 35.0, -232.0, -10.0]
    j2, j4 = JOINT_REFERENCES[2], JOINT_REFERENCES[4]
    assert abs(j2.angle(j2.zero_rad - 2 * math.pi)) < 1e-6
    assert abs(j4.angle(j4.zero_rad - 2 * math.pi) + 2 * math.pi) < 1e-6
    try:
        assess([0, 0, math.nan, 0, 0], line)
    except ValueError:
        pass
    else:
        raise AssertionError("NaN must be rejected")
    print("Five-axis preflight report/no-wrap/no-mutation: PASS")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--capture", type=Path)
    parser.add_argument("--library", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return 0
    if not all((args.capture, args.library, args.output)):
        parser.error("--capture, --library and --output are required")
    result = audit(args.capture, args.library)
    with args.output.open("x", encoding="utf-8") as out:
        json.dump(result, out, indent=2, allow_nan=False)
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result["current_pose_admitted"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
