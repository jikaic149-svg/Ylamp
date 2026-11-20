"""Offline regression only. No serial port is opened and no capture is marked as measured."""
import sys
import struct
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools/urdf_collision"))
from five_control import Board, gravity_output_confirmed
from five_identify import HW, SCENE, WORKSPACE_CAD, identify, regressors
from check_gravity_capture import pose_diagnostics


class FiveAxisTest(unittest.TestCase):
    def test_gravity_contributions_and_angle_sensitivity_are_nominal_not_load_labels(self):
        pose = [45.593595917531964, 25.63823520066674, -47.38781318796074,
                82.20395628393392, -20.490909019496755]
        # Preserve the native-CAD golden pose, expressed in the new software basis.
        pose = np.array(pose)+WORKSPACE_CAD-HW[:, 6]
        report = pose_diagnostics(pose)
        contributions = np.array(report["link_contributions_nm_rows_joint_cols_body"])
        np.testing.assert_allclose(contributions.sum(axis=1), report["gravity_nm"], atol=1e-12)
        np.testing.assert_array_equal(np.tril(contributions, -1), np.zeros((5, 5)))
        # Native MuJoCo at this encoder-derived pose, not physical torque truth.
        np.testing.assert_allclose(report["gravity_nm"],
                                   [0, -.3851419015375672, .3555952880669767,
                                    -.014430698682056622, -.0334410704294464], atol=1e-12)
        sensitivity = np.array(report["sensitivity_nm_per_deg_rows_torque_cols_angle"])
        np.testing.assert_allclose(sensitivity, sensitivity.T, atol=1e-10)
        self.assertAlmostEqual(sensitivity[2, 1], -.015742702404278, places=10)
        heavier = SCENE.copy()
        heavier[4, -2] *= 1.4
        with patch("five_identify.SCENE", heavier):
            changed = pose_diagnostics(pose)
        np.testing.assert_allclose(np.subtract(changed["gravity_nm"], report["gravity_nm"]),
                                   .4*contributions[:, 4], atol=1e-12)
        for invalid in ([0]*4, [float("nan")]*5):
            with self.assertRaises(ValueError):
                pose_diagnostics(invalid)

    def test_pitch_zero_cad_datum_accounts_for_recorded_upstream_angle(self):
        from dm_monitor import JOINT_REFERENCES

        q = np.radians(HW[:, 6].copy())
        q[1] += np.radians(154.75808470469042)-JOINT_REFERENCES[2].zero_rad

        def height(j3):
            angles = q.copy()
            angles[2] = j3
            rotation = np.eye(3)
            for j in range(3):
                c, s = np.cos(angles[j]), np.sin(angles[j])
                rotation = rotation @ SCENE[j, 6:15].reshape(3, 3) @ np.array(
                    [[c, 0, -s], [0, 1, 0], [s, 0, c]])
            return -(rotation @ SCENE[3, :3])[1]  # base-frame up is -Y

        step = 1e-5
        self.assertLess(abs((height(q[2]+step)-height(q[2]-step))/(2*step)), 1e-10)
        self.assertGreater(height(q[2]), height(q[2]+step))
        self.assertGreater(height(q[2]), height(q[2]-step))
        # No sign hard switch: both sides use the same continuous CAD model.
        pose = np.degrees(q)-HW[:, 6]
        low, high = pose.copy(), pose.copy()
        low[2] -= 5
        high[2] += 5
        self.assertGreater(pose_diagnostics(low)["gravity_nm"][2], 0)
        self.assertLess(pose_diagnostics(high)["gravity_nm"][2], 0)

    def test_wait_is_not_gravity_and_phase_change_never_replays_action(self):
        ready = dict(phase="GRAVITY", fresh=31, enabled=31, submitted=31,
                     gravity_age_ms=5, limited=0, fault=0)
        self.assertTrue(gravity_output_confirmed(ready))
        for change in (dict(phase="WAIT"), dict(enabled=27), dict(submitted=0), dict(fresh=27),
                       dict(limited=1), dict(fault=1), dict(gravity_age_ms=None)):
            self.assertFalse(gravity_output_confirmed(dict(ready, **change)))
        board = Mock(spec=Board)
        board.status.side_effect = [dict(ready, phase="POSITION"), ready]
        self.assertEqual(Board.observe_transition(board, ("POSITION",)), ready)
        self.assertEqual(board.status.call_count, 2)
        board.rpc.assert_not_called()

    def test_rpc_consumes_pending_telemetry_before_rejecting_stale_clock(self):
        board = Board(Mock())
        board.at, board.latest = 0., dict(tick_ms=10)
        def refresh():
            if board.pump.call_count == 2:  # first read is only a partial snapshot
                board.at, board.latest = 1., dict(tick_ms=1000)
        board.pump = Mock(side_effect=refresh)
        board.exchange = Mock(side_effect=lambda kind, payload, expected, prefix:
                              prefix+struct.pack("<BBBBI", 1, 0, 0, 0, 42))
        with patch("five_control.time.monotonic", return_value=1.):
            self.assertEqual(board.rpc(0), 42)
        self.assertEqual(board.pump.call_count, 2)
        board.exchange.assert_called_once()
        payload = board.exchange.call_args.args[1]
        self.assertEqual(struct.unpack_from("<I", payload, 12)[0], 1000)

        for error in (None, RuntimeError("MCU restarted; session invalid")):
            board = Board(Mock())
            board.at, board.latest = 0., dict(tick_ms=10)
            def stalled():
                clock.return_value = 2.
            board.pump, board.exchange = Mock(side_effect=error or stalled), Mock()
            with patch("five_control.time.monotonic", return_value=1.) as clock:
                with self.assertRaises(RuntimeError if error else TimeoutError):
                    board.rpc(0)
            board.pump.assert_called_once()
            board.exchange.assert_not_called()
            self.assertEqual(board.ident, 0)

    def test_recovers_known_coupling_inertia_and_friction_on_separate_synthetic_data(self):
        truth = np.r_[[.9, 1.1, .95, 1.05], np.linspace(.001, .005, 5),
                      np.linspace(.02, .06, 5), np.linspace(.03, .07, 5)]
        def synthetic(seed):
            rng = np.random.default_rng(seed)
            q, v, a = (rng.uniform(-lim, lim, (350, 5)) for lim in (1.2, .35, .5))
            fixed, y, f = regressors(q, v, a)
            return q, v, a, fixed + y @ truth[:9] + f @ truth[9:]
        result = identify(synthetic(11), synthetic(20))
        self.assertEqual(len(result["alternating"]), 2)
        np.testing.assert_allclose(result["joint_fit"], truth, atol=1e-10, rtol=0)
        self.assertLess(max(result["evaluation_rmse_nm"]["joint"]), 1e-10)
        self.assertFalse(result["deploy"])


if __name__ == "__main__":
    unittest.main()
