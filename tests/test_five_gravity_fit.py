"""Synthetic/offline checks only; no motor control or measured-data claims."""
import io
import hashlib
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from five_control import CONTROL_VERSION, EXPECTED
from five_gravity_fit import gravity_basis, identify, nominal_parameters, structural_basis
from five_gravity_residual_report import fit_analysis, gravity_transform, prepare, read_analysis_run, set_weights, support_gravity
from five_identify import HW, NOMINAL, REFS, load_run, regressors


class GravityFitTest(unittest.TestCase):
    def test_current_support_changes_only_j2_and_retains_j3(self):
        q = np.random.default_rng(40).uniform(-2, 2, (203, 5))
        nominal = gravity_basis(q) @ nominal_parameters()
        support = support_gravity(q)
        expected = nominal.copy()
        expected[:, 2] = 1.18*nominal[:, 2]-.01*np.cos(q[:, 2]-np.deg2rad(HW[2, 6]))
        expected = np.clip(expected, -HW[:, 5], HW[:, 5])
        np.testing.assert_array_equal(support[:, [0, 2, 3, 4]], expected[:, [0, 2, 3, 4]])
        self.assertGreater(np.max(abs(support[:, 1]-expected[:, 1])), .1)

    def test_j2_candidate_matches_recorded_pose_goldens_and_is_continuous(self):
        # Same two recorded wrist poses, not two angle-dependent branches.
        poses = [([-28.51236855812435, 0, 27.397681685578284,
                   -180.8771105814127, -46.63179150156308],
                  -.00678242792057368, -3.287249049659447),
                 ([-28.621651584844557, 0, -60.314245012986056,
                   -177.8827146681441, -71.11153099634728],
                  1.1415244366365485, -2.214092015647563)]
        angles = np.linspace(-.7, .7, 201)
        for raw, cosine, sine in poses:
            q = np.tile(np.deg2rad(raw)-REFS[:, 0]+np.deg2rad(HW[:, 6]), (len(angles), 1))
            q[:, 1] = angles+np.deg2rad(HW[1, 6])
            expected = cosine*np.cos(angles)+sine*np.sin(angles)
            np.testing.assert_allclose(support_gravity(q)[:, 1], expected, atol=1e-11)
            q[:, 3] -= 2*np.pi
            np.testing.assert_allclose(support_gravity(q)[:, 1], expected, atol=1e-11)
            q = q[:3].copy()
            q[:, 1] = np.array([-1e-8, 0, 1e-8])+np.deg2rad(HW[1, 6])
            self.assertLess(np.max(abs(np.diff(support_gravity(q)[:, 1]))), 4e-8)

    def test_nominal_basis_matches_existing_recursive_dynamics(self):
        q = np.random.default_rng(41).uniform(-2, 2, (203, 5))
        zero = np.zeros_like(q)
        fixed, y, _ = regressors(q, zero, zero)
        np.testing.assert_allclose(gravity_basis(q) @ nominal_parameters(),
                                   fixed+y@NOMINAL, atol=2e-14)

    def test_correction_is_one_shared_potential_gradient(self):
        rng = np.random.default_rng(42)
        delta = rng.normal(0, .01, 22)
        q = rng.uniform(-1, 1, (8, 5))
        h = 1e-5
        derivative = np.empty((len(q), 5, 5))
        for i in range(5):
            e = np.eye(5)[i]*h
            derivative[:, :, i] = ((gravity_basis(q+e)-gravity_basis(q-e)) @ delta)/(2*h)
        np.testing.assert_allclose(derivative, derivative.transpose(0, 2, 1), atol=2e-10)
        # Additional base-gravity columns permit yaw residual, rather than forcing G1=0.
        self.assertGreater(np.max(np.abs(gravity_basis(q)[:, 0, 20:])), .01)

    def test_each_conditional_joint_slice_is_sine_plus_cosine(self):
        rng = np.random.default_rng(43)
        delta = rng.normal(0, .01, 22)
        for i in range(5):
            q = rng.uniform(-1, 1, (30, 5))
            q[:] = q[0]
            angles = np.linspace(-4, 4, len(q))
            q[:, i] = angles
            c, s = q[:1].copy(), q[:1].copy()
            c[:, i], s[:, i] = 0, np.pi/2
            predicted = ((gravity_basis(c) @ delta)[0, i]*np.cos(angles)
                         + (gravity_basis(s) @ delta)[0, i]*np.sin(angles))
            np.testing.assert_allclose((gravity_basis(q) @ delta)[:, i], predicted, atol=1e-14)

    @staticmethod
    def synthetic(seed, delta, count=1000):
        rng = np.random.default_rng(seed)
        q = rng.uniform(-1.3, 1.3, (count, 5))
        v = rng.uniform(.04, .12, (count, 5))*rng.choice([-1, 1], (count, 5))
        a = np.zeros_like(q)
        fixed, y, friction = regressors(q, v, a)
        fc_b = np.r_[np.linspace(.02, .06, 5), np.linspace(.03, .07, 5)]
        torque = fixed+y@NOMINAL+gravity_basis(q)@delta+friction@fc_b
        return q, v, a, torque, np.arange(count)*20.0

    def test_identified_coupled_gravity_predicts_independent_data(self):
        transform, _ = structural_basis()
        delta = transform @ np.linspace(-.2, .2, transform.shape[1])
        train = self.synthetic(44, delta)
        holdout = self.synthetic(45, delta)
        report = identify([train], [holdout], bootstrap=20, block_seconds=1)
        self.assertEqual(report["status"], "candidate_not_deployed")
        self.assertFalse(report["deploy"])
        recovered = np.array(report["potential_delta"])
        np.testing.assert_allclose(gravity_basis(holdout[0]) @ recovered,
                                   gravity_basis(holdout[0]) @ delta, atol=1e-11)
        self.assertLess(max(report["holdout"][0]["corrected_with_friction_rmse_nm"]), 1e-10)
        self.assertEqual(report["confidence"]["successful"], 20)

    def test_missing_bidirection_and_pose_excitation_are_rejected(self):
        transform, _ = structural_basis()
        delta = transform @ np.zeros(transform.shape[1])
        data = list(self.synthetic(46, delta))
        validation = self.synthetic(47, delta)
        data[1] = abs(data[1])
        with self.assertRaisesRegex(ValueError, "bidirectional"):
            identify([tuple(data)], [validation], bootstrap=20, block_seconds=1)
        data = list(self.synthetic(48, delta))
        data[0][:] = data[0][0]
        with self.assertRaisesRegex(ValueError, "rank|unexcited"):
            identify([tuple(data)], [validation], bootstrap=20, block_seconds=1)

    def test_invalid_angles_are_not_used(self):
        for q in (np.zeros((3, 4)), np.full((2, 5), np.nan)):
            with self.assertRaises(ValueError):
                gravity_basis(q)

    def test_exploratory_four_axis_fit_excludes_j1_and_recovers_synthetic_gravity(self):
        rng = np.random.default_rng(20261119)
        transform = gravity_transform()
        delta = transform@np.linspace(-.1, .1, transform.shape[1])
        count = 4000
        q = rng.uniform(-1.2, 1.2, (count, 5))
        moving = np.repeat(np.arange(2, 6), count//4)
        velocity = np.zeros_like(q)
        velocity[np.arange(count), moving-1] = rng.uniform(.04, .12, count)*rng.choice([-1, 1], count)
        acceleration = np.zeros_like(q)
        fixed, y, friction = regressors(q, velocity, acceleration)
        fc_b = np.r_[0., [.05]*4, 0., [.1]*4]
        torque = fixed+y@NOMINAL+gravity_basis(q)@delta+friction@fc_b
        data = q, velocity, acceleration, torque, np.arange(count)*20., moving
        part = prepare(data, transform)
        self.assertFalse(np.any(part["active"][:, 0]))
        self.assertTrue(all(np.any(part["active"][:, i]) for i in range(1, 5)))
        result = fit_analysis([data], bootstrap=20)
        self.assertEqual(result["status"], "exploratory_not_deployable")
        self.assertFalse(result["deploy"])
        np.testing.assert_allclose(gravity_basis(q)@result["potential_delta"],
                                   gravity_basis(q)@delta, atol=1e-10)
        self.assertLess(max(j["candidate_with_friction_holdout_rmse_nm"]
                            for j in result["joints"]), 1e-10)
        self.assertLess(np.max(np.abs((gravity_basis(q)@result["potential_delta"])[:, 0])), 1e-10)
        part["active"][:, 4] = False
        with self.assertRaisesRegex(ValueError, "J5"):
            set_weights([part])

    def test_exploratory_prefix_preserves_incomplete_contact_and_fault_provenance(self):
        # Synthetic decoder fixture only; no persisted capture is claimed measured.
        records = [dict(stage="capture", schema="ylamp-five-id-1",
                        control_version=CONTROL_VERSION, action="move",
                        session="1234567890abcdef", free_space=False,
                        references_rad=REFS.tolist())]
        for tick in range(0, 3501, 10):
            joints = [dict(joint=j, bus=b, protocol=p, command_id=c, feedback_id=f,
                           received=True, fresh=True, state=1, age_ms=0,
                           received_tick_ms=tick, received_count=tick//10+1,
                           position_rad=float((REFS[i, 1]+REFS[i, 2])/2),
                           velocity_rad_s=.06, torque_nm=0)
                      for i, (j, b, p, c, f) in enumerate(EXPECTED)]
            telemetry = dict(tick_ms=tick, joints=joints)
            if tick == 0 or tick % 100 == 0:
                records.append(dict(stage="before" if tick == 0 else "sample",
                                    phase="MOTION" if tick < 3000 else "DEGRADED",
                                    fresh=31, enabled=31, submitted=31,
                                    fault=0 if tick < 3000 else 32, limited=0,
                                    gravity_age_ms=0, telemetry=telemetry))
            if tick == 0:
                records.append(dict(stage="waypoint", joint=2, leg=1, duration_ms=3000))
            records.append(dict(stage="feedback", telemetry=telemetry))
        payload = "\n".join(map(json.dumps, records)).encode()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"synthetic.jsonl"
            path.write_bytes(payload)
            report, data = read_analysis_run(path)
            self.assertFalse(report["sequence_completed"])
            self.assertFalse(report["free_space_confirmed"])
            self.assertFalse(report["deploy"])
            self.assertEqual(report["sha256"], hashlib.sha256(payload).hexdigest())
            self.assertEqual(path.read_bytes(), payload)
            selected = data[-1] == 2
            self.assertTrue(np.any(selected))
            self.assertTrue(np.all(data[-2][selected] <= 2000))

    def test_loader_preserves_measured_clock_after_excluding_a_gap(self):
        # Synthetic file format only; it is never persisted or labelled as a real capture.
        def telemetry(tick):
            joints = [dict(joint=j, bus=b, protocol=p, command_id=c, feedback_id=f,
                           received=True, fresh=True, state=1, age_ms=0,
                           received_tick_ms=tick, received_count=tick//10+1,
                           position_rad=float((REFS[i, 1]+REFS[i, 2])/2),
                           velocity_rad_s=.06, torque_nm=0)
                      for i, (j, b, p, c, f) in enumerate(EXPECTED)]
            return dict(tick_ms=tick, unix_s=1000+tick*.001, joints=joints)

        def status(stage, tick):
            return dict(stage=stage, phase="GRAVITY", fault=0, limited=0,
                        fresh=31, enabled=31, submitted=31, gravity_age_ms=0,
                        telemetry=telemetry(tick))

        records = [dict(stage="capture", schema="ylamp-five-id-1",
                        control_version=CONTROL_VERSION, action="status",
                        session="1234567890abcdef", free_space=True, references_rad=REFS.tolist()),
                   status("before", 0), dict(stage="rpc_accepted", action="status"),
                   status("sample", 20)]
        records += [dict(stage="feedback", telemetry=telemetry(t))
                    for t in range(20, 621, 10) if not 270 <= t <= 330]
        records += [status("sample", 620), dict(stage="end")]
        payload = "\n".join(map(json.dumps, records)).encode()
        path = Mock()
        path.open.side_effect = lambda *_: io.BytesIO(payload)
        report, legacy = load_run(path)
        timed_report, timed = load_run(path, with_time=True)
        self.assertEqual(report["issues"], [])
        self.assertEqual(timed_report["issues"], [])
        self.assertEqual(len(legacy), 4)
        self.assertEqual(len(timed), 5)
        for old, new in zip(legacy, timed):
            np.testing.assert_array_equal(old, new)
        times = timed[-1]
        self.assertEqual(len(times), report["eligible_samples"])
        self.assertTrue(np.all(np.diff(times) > 0))
        self.assertGreater(np.max(np.diff(times)), 100)
        self.assertFalse(np.any((times >= 200) & (times <= 360)))


if __name__ == "__main__":
    unittest.main()
