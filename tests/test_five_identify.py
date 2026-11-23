"""Offline regression only. No serial port is opened and no capture is marked as measured."""
import sys
import struct
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools/urdf_collision"))
from five_control import Board, CRUISE_VERSION, DRAG_VERSION, gravity_output_confirmed, idle_output_confirmed
from five_gravity_sweep import angles, baseline_goal
from five_identify import HW, SCENE, WORKSPACE_CAD, identify, regressors
from check_gravity_capture import pose_diagnostics


class FiveAxisTest(unittest.TestCase):
    def test_position_partial_inertia_recovers_truth_and_rejects_confounding(self):
        from five_friction_global import balanced
        from five_inertia_separate import partial_inertia, position_design
        rng = np.random.default_rng(2026112303)
        position = rng.uniform(-.3, .3, 360)
        velocity = rng.uniform(.05, .4, 360)*rng.choice([-1, 1], 360)
        groups = np.array([f"synthetic_leg_{k//60}" for k in range(360)])
        nuisance, _ = position_design(position, velocity, balanced(groups), 1)
        acceleration = .8*position+rng.normal(0, .5, 360)
        truth = np.array([.037, -.012, .023, .081])
        residual = np.column_stack((acceleration, nuisance))@truth
        beta, info = partial_inertia(acceleration, nuisance, residual, groups)
        np.testing.assert_allclose(beta, truth, atol=1e-12)
        self.assertGreater(info["acceleration_information_fraction"], 0)
        self.assertLessEqual(info["acceleration_information_fraction"], 1+1e-12)
        with self.assertRaisesRegex(ValueError, "not separable"):
            partial_inertia(nuisance@np.array([.1, .3, .4]), nuisance, residual, groups)

    def test_position_inertia_holdout_never_changes_training_center_or_coefficients(self):
        from five_inertia_separate import evaluate
        rng = np.random.default_rng(2026112304)
        q = rng.uniform(-.3, .3, (360, 5))
        v = rng.uniform(.05, .4, 360)*rng.choice([-1, 1], 360)
        a = rng.uniform(-2, 2, 360)
        ramps = np.repeat([300, 600, 1200], 120)
        data = dict(q=q, v=v, a=a, ramp=ramps,
            y=.04*a+.03-.02*np.tanh(v/.02)+.3*q[:, 0],
            group=np.array([f"synthetic_leg_{k//60}" for k in range(360)]))
        first = evaluate(data, 0, 1, False, 600)
        data["q"] = q.copy()
        data["q"][ramps == 600, 0] += 10
        data["y"] = data["y"].copy()
        data["y"][ramps == 600] += 100
        second = evaluate(data, 0, 1, False, 600)
        self.assertEqual(first["training_position_center_rad"],
                         second["training_position_center_rad"])
        np.testing.assert_array_equal(first["training_coefficients"],
                                      second["training_coefficients"])
        self.assertLess(first["with_new_inertia"]["rmse_nm"], 1e-12)
        self.assertGreater(second["with_new_inertia"]["rmse_nm"], 1)

    def test_acceleration_pairs_exclude_holdout_wrong_pose_speed_and_reuse(self):
        from five_inertia_separate import matched_acceleration
        q = np.zeros((10, 5))
        q[:, 0] = np.deg2rad([0, .3, .1, .4, 0, 10, 10, 0, 20, 20])
        q[6, 4] = np.deg2rad(.6)  # Same active joint, wrong held-axis pose.
        v = np.full(10, .2)
        v[7], v[9] = -.2, .3
        a = np.array([1.2, 1.4, .2, .4, 3, 1.1, .1, .1, 1, .1])
        ramps = np.array([300, 300, 1200, 1200, 600, 300, 1200, 1200, 300, 1200])
        groups, times = np.array([f"sample_{k}" for k in range(10)]), np.arange(10)*10.
        result = matched_acceleration(q, v, a, .04*a+.08, ramps, groups, times, 0)
        self.assertEqual(result["pairs"], 2)
        self.assertAlmostEqual(result["extra_inertia_kg_m2"], .04, places=12)
        fast = [p["fast_sample"] for p in result["matched_pairs"]]
        slow = [p["slow_sample"] for p in result["matched_pairs"]]
        self.assertEqual(set(fast), {0, 1})
        self.assertEqual(set(slow), {2, 3})
        self.assertEqual(len(fast), len(set(fast)))
        self.assertEqual(len(slow), len(set(slow)))
        q[:, 4] = np.where(ramps == 1200, np.deg2rad(2.), 0.)
        rejected = matched_acceleration(q, v, a, .04*a+.08, ramps, groups, times, 0)
        self.assertEqual(rejected["pairs"], 0)
        self.assertIsNone(rejected["extra_inertia_kg_m2"])

    def test_local_inertia_recovers_synthetic_delta_and_keeps_holdout_separate(self):
        from five_inertia_identify import local_design, local_fit
        rng = np.random.default_rng(2026112302)
        acceleration = rng.uniform(-2, 2, 360)
        velocity = rng.uniform(.05, .4, 360)*rng.choice([-1, 1], 360)
        x = local_design(acceleration, velocity)
        truth = np.array([.037, -.012, .023])
        residual = x@truth
        groups = np.array([f"synthetic_leg_{k//60}" for k in range(360)])
        beta, condition = local_fit(x[:240], residual[:240], groups[:240])
        np.testing.assert_allclose(beta, truth, atol=1e-12)
        np.testing.assert_allclose(x[240:]@beta, residual[240:], atol=1e-12)
        self.assertLess(condition, 3)
        with self.assertRaises(ValueError):
            local_fit(local_design(np.zeros(360), velocity), residual, groups)

    def test_inertia_first_stage_keeps_finite_interval_and_varies_only_ramp(self):
        from five_inertia_scan import pair_endpoints, RAMPS_MS, SPEED
        from five_global_friction_scan import full_leg
        from five_identify import REFS
        middle = np.rad2deg(REFS[:, 1:]).mean(axis=1)
        for joint in (1, 2):
            for position in (np.rad2deg(REFS[joint-1, 1])+3,
                             middle[joint-1], np.rad2deg(REFS[joint-1, 2])-3):
                anchor = middle.copy()
                anchor[joint-1] = position
                near, far = pair_endpoints(anchor, joint)
                self.assertAlmostEqual(abs(far[joint-1]-near[joint-1]), 30.)
                np.testing.assert_array_equal(np.delete(near, joint-1), np.delete(far, joint-1))
                for ramp in RAMPS_MS:
                    for start, end in ((near, far), (far, near)):
                        goal, plateau, _, duration = full_leg(start, joint, end[joint-1], SPEED, ramp)
                        self.assertGreater(plateau, 0)
                        self.assertEqual(duration, plateau+2*ramp)
                        self.assertLessEqual(abs(goal[joint-1]-end[joint-1]),
                                             np.rad2deg(SPEED*.001)+1e-10)
        np.testing.assert_allclose([1.875*SPEED/(r*.001) for r in RAMPS_MS], [.5, 1, 2])
        middle[4] = np.rad2deg(REFS[4, 2])-.2
        with self.assertRaises(ValueError):
            pair_endpoints(middle, 1)
        with self.assertRaises(ValueError):
            pair_endpoints(np.zeros(5), 3)

    def test_inertia_preflight_never_launches_motion(self):
        from five_inertia_scan import InertiaBatch
        from five_identify import REFS
        batch = InertiaBatch.__new__(InertiaBatch)
        batch.state = Mock(return_value=np.rad2deg(REFS[:, 1:]).mean(axis=1))
        batch.check, batch.note = Mock(), Mock()
        batch.transfer = Mock(side_effect=AssertionError("Preflight must not move"))
        batch.leg = Mock(side_effect=AssertionError("Preflight must not launch a leg"))
        batch.preflight()
        self.assertEqual(batch.check.call_count, 2)
        batch.transfer.assert_not_called()
        batch.leg.assert_not_called()
        self.assertTrue(batch.note.call_args.kwargs["no_motion_sent"])

    def test_deployed_friction_matches_frozen_global_model_and_native_wrench(self):
        import hashlib
        import json
        import os
        import subprocess
        from five_friction_global import predict
        from five_friction_map import load_features
        from five_identify import ROOT
        native = Path(os.environ.get("YLAMP_NATIVE_TEST_EXE",
            ROOT/"build/v12-friction-global-20261123/host/bsp_tests.exe"))
        if not native.is_file():
            self.skipTest("Build the native five-wire executable for C/model cross-check")
        frozen = ROOT/"build/v12-default-drag-20261123/friction_global_model_02.json"
        payload = frozen.read_bytes()
        self.assertEqual(hashlib.sha256(payload).hexdigest(),
            "d950ff3f2f803d953889eafecb1e52510be575558895e17e22ce6e359f2d8c37")
        models = json.loads(payload)["models"]
        rng = np.random.default_rng(20261123)
        q = rng.uniform(-np.pi, np.pi, (500, 5))
        z, lengths, drive = load_features(q)
        native_load = subprocess.run([str(native), "five-wire"],
            input="".join("L "+" ".join(map(str, pose))+"\n" for pose in q),
            text=True, capture_output=True, check=True)
        wrench = np.array([[float(v) for v in line.split()]
                           for line in native_load.stdout.splitlines()]).reshape(-1, 5, 2)
        np.testing.assert_allclose(wrench[:, :, 0], drive, rtol=3e-5, atol=3e-6)
        speeds = np.array([0, .001, .05, .079999, .08, .080001, .159999, .16,
                           .160001, .32, .479999, .48, .480001, .6, .8, .800001, 1.2])
        commands, wanted = [], []
        for axis in range(5):
            normalizer = 9.81*SCENE[axis:, -2].sum()*lengths[axis]
            bend = z[:, axis, 2]*normalizer
            np.testing.assert_allclose(wrench[:, axis, 1], bend, rtol=3e-5, atol=3e-6)
            for speed in speeds:
                amplitude = predict(models[axis]["final_model"], z[:, axis],
                                    np.full(len(q), np.clip(speed, .08, .8)))
                for sign in (-1, 1):
                    commands.extend(f"F {axis} {sign*speed:.12g} {g:.12g} {b:.12g}\n"
                                    for g, b in wrench[:, axis])
                    wanted.extend(amplitude*np.tanh(sign*speed/.02))
        result = subprocess.run([str(native), "five-wire"], input="".join(commands),
                                text=True, capture_output=True, check=True)
        actual = np.array([float(line) for line in result.stdout.splitlines()])
        np.testing.assert_allclose(actual, wanted, rtol=2e-6, atol=4e-7)
        # Native tests separately cover this pure function's composition with
        # active commands and partial drag, plus unchanged pure-G/held axes.

    def test_endpoint_duration_uses_latest_pose_and_keeps_original_inset(self):
        from five_gravity_sweep import endpoint_goal
        from five_identify import REFS
        current = np.rad2deg(REFS[:, 1:]).mean(axis=1)-np.rad2deg(REFS[:, 0])
        endpoint = np.rad2deg(REFS[2, 1])+1.2
        for drift in (0, -.3, 2.5):
            actual = current.copy()
            actual[2] += drift
            goal, direction, duration = endpoint_goal(actual.tolist(), 3, endpoint, .16, 300)
            self.assertEqual(direction, -1)
            self.assertGreater(duration, 600)
            self.assertLessEqual(abs(goal[2]+np.rad2deg(REFS[2, 0])-endpoint),
                                 np.rad2deg(.16*.001)+1e-10)
        with self.assertRaises(ValueError):
            endpoint_goal(current.tolist(), 3, np.rad2deg(REFS[2, 1]), .16, 300)

    def test_source_clock_bounds_are_explicit_and_reject_failed_capture(self):
        import io
        import json
        from five_global_friction_scan import source_clock_audit
        state = dict(fresh=31, enabled=31, submitted=31, fault=0, limited=0)
        records = [
            dict(stage="capture", free_space=True),
            dict(stage="before", phase="GRAVITY", telemetry=dict(tick_ms=100, max_age_ms=100), **state),
            dict(stage="waypoint", duration_ms=2000),
            dict(stage="sample", phase="MOTION", telemetry=dict(tick_ms=130, max_age_ms=100), **state),
            dict(stage="handover", phase="GRAVITY", **state),
            dict(stage="end", malformed=0, incomplete=0)]
        capture = Mock()
        capture.__str__ = Mock(return_value="OFFLINE")
        def check(rows):
            capture.open.return_value = io.BytesIO(b"".join(
                (json.dumps(r)+"\n").encode() for r in rows))
            return source_clock_audit(capture)
        audit = check(records)
        self.assertEqual(audit["board_start_tick_interval_ms"], [100, 230])
        self.assertFalse(audit["exact_start_verified"])
        self.assertTrue(audit["interval_requires_conservative_plateau_trimming"])
        with self.assertRaises(ValueError):
            check([dict(records[0], free_space=False), *records[1:]])
        capture.open.return_value = io.BytesIO(b"".join(
            (json.dumps(r)+"\n").encode() for r in
            [dict(records[0], free_space=False), *records[1:]]))
        timing_only = source_clock_audit(capture, require_free_space=False)
        self.assertFalse(timing_only["free_space_confirmed"])
        with self.assertRaises(ValueError):
            check(records[:-1])
        with self.assertRaises(ValueError):
            check([*records[:3], dict(records[3], fault=32), *records[4:]])

    def test_full_angle_velocity_leg_keeps_continuous_branch_and_original_bounds(self):
        from five_global_friction_scan import full_leg
        from five_identify import REFS
        raw = np.rad2deg(REFS[:, 1:]).mean(axis=1)
        for joint in range(1, 6):
            for speed in (.16, .48, .8):
                for endpoint in np.rad2deg(REFS[joint-1, 1:])+[1.2, -1.2]:
                    goal, plateau, direction, duration = full_leg(raw, joint, endpoint, speed, 300)
                    self.assertGreater(plateau, 0)
                    self.assertEqual(duration, plateau+600)
                    self.assertLessEqual(abs(goal[joint-1]-endpoint), np.rad2deg(speed*.001)+1e-10)
                    self.assertEqual(direction, 1 if endpoint > raw[joint-1] else -1)
                    np.testing.assert_array_equal(np.delete(goal, joint-1), np.delete(raw, joint-1))
        raw[3] = -170
        goal, _, direction, _ = full_leg(raw, 4, 348.8, .16, 300)
        self.assertEqual(direction, 1)
        self.assertGreater(goal[3], 348.79)  # Do not replace a 519-degree path with a turn alias.
        for endpoint in (-180, 350, float("nan")):
            with self.assertRaises(ValueError):
                full_leg(raw, 4, endpoint, .16, 300)
        with self.assertRaises(ValueError):
                full_leg(raw, 4, raw[3], .16, 300)

    def test_fast_spatial_cells_need_the_existing_250hz_analysis_grid(self):
        from five_friction_map import pair_legs
        counts = []
        for dt in (.01, .004):
            time = np.arange(0, 1, dt)
            q = np.tile(np.deg2rad(HW[:, 6]), (len(time), 1))
            q[:, 0] += .8*time
            legs = [dict(group=(0, .8), joint=1, sign=sign, q=q,
                         v=np.full(len(time), .8), r=np.full(len(time), sign*.7),
                         dyn=np.zeros(len(time)), temp=np.full(len(time), 30.))
                    for sign in (1, -1)]
            counts.append(len(pair_legs(legs, angle_bin_deg=1.)))
        self.assertGreater(counts[1], 35)
        self.assertLess(counts[0], counts[1]/2)

    def test_sweep_restores_original_drag_only_after_success(self):
        import io
        import itertools
        from contextlib import nullcontext
        from types import SimpleNamespace
        from dm_monitor import JOINT_REFERENCES
        from five_gravity_sweep import main

        references = [[r.zero_rad, r.raw_lower_rad, r.raw_upper_rad]
                      for r in JOINT_REFERENCES.values()]
        ready = dict(phase="GRAVITY", fresh=31, enabled=31, submitted=31,
                     gravity_age_ms=2, limited=0, fault=0,
                     telemetry=dict(joints=[dict(joint=i+1,
                        position_rad=(r.raw_lower_rad+r.raw_upper_rad)/2,
                        velocity_rad_s=.08, torque_nm=.1)
                        for i, r in enumerate(JOINT_REFERENCES.values())]))
        drag, position = dict(ready, phase="DRAG"), dict(ready, phase="POSITION")
        for failure in (False, True):
            board, log = Mock(spec=Board), io.StringIO()
            board.status.side_effect = [drag, ready]
            board.rpc.side_effect = lambda field: DRAG_VERSION if field == 251 else 0
            board.scalar.side_effect = lambda field, joint, op: references[joint-1][field-1]
            board.test_gravity.side_effect = lambda state: Board.test_gravity(board, state)
            board.latest = dict(tick_ms=100)
            board.stream = SimpleNamespace(malformed=0)
            board.collector = SimpleNamespace(incomplete=0)
            board.session = b"12345678"
            board.observe_transition.side_effect = ([ready, ready] if failure else
                                                    [ready, position, ready, ready, drag])
            if failure:
                board.speed.side_effect = RuntimeError("single commit rejected")
            path = Mock()
            path.open.return_value = nullcontext(log)
            args = SimpleNamespace(port="UNUSED", log=path, peak_speed=.16,
                constant_speed=.08, single_leg=True, joints=[1], direction=1,
                plateau_ms=300, ramp_ms=100, continuous_reference=False, free_space=True)
            with patch("five_gravity_sweep.argparse.ArgumentParser.parse_args", return_value=args), \
                 patch("five_gravity_sweep.serial_port", return_value=nullcontext(Mock())), \
                 patch("five_gravity_sweep.Board", return_value=board), \
                 patch("five_gravity_sweep.time.monotonic", side_effect=itertools.count(0, 10)), \
                 patch("builtins.print"):
                if failure:
                    with self.assertRaisesRegex(RuntimeError, "single commit rejected"):
                        main()
                else:
                    main()
            board.speed.assert_called_once()
            if failure:
                board.drag.assert_not_called()
                board.rpc.assert_called_with(7)  # Failure stays G, no automatic assist retry.
            else:
                board.drag.assert_called_once_with()
                self.assertEqual(board.observe_transition.call_args.args, (("GRAVITY",),))

    def test_measurement_selects_pure_g_but_read_only_connect_does_not(self):
        ready = dict(phase="GRAVITY", fresh=31, enabled=31, submitted=31,
                     gravity_age_ms=5, limited=0, fault=0)
        drag = dict(ready, phase="DRAG")
        self.assertTrue(idle_output_confirmed(drag))
        self.assertFalse(gravity_output_confirmed(drag))
        board = Mock(spec=Board)
        board.rpc.side_effect = [DRAG_VERSION, 0]
        board.observe_transition.return_value = ready
        self.assertEqual(Board.test_gravity(board, drag), ready)
        self.assertEqual([call.args[0] for call in board.rpc.call_args_list], [251, 7])
        board.observe_transition.assert_called_once_with(("DRAG",))
        board.reset_mock()
        self.assertEqual(Board.test_gravity(board, ready), ready)
        board.rpc.assert_not_called()
        for change in (dict(phase="WAIT"), dict(phase="POSITION"), dict(fault=1),
                       dict(fresh=27), dict(limited=1), dict(gravity_age_ms=101)):
            board.reset_mock()
            with self.assertRaises(RuntimeError):
                Board.test_gravity(board, dict(drag, **change))
            board.rpc.assert_not_called()
        board.rpc.side_effect = None
        board.rpc.return_value = 0x594c4401
        with self.assertRaises(ValueError):
            Board.test_gravity(board, drag)
        board.rpc.assert_called_once_with(251)  # Reject old lifecycle before changing G/moving.

    def test_sweep_targets_use_board_continuous_angle_not_wrapped_raw(self):
        from dm_monitor import JOINT_REFERENCES
        raw = np.deg2rad([-42, 145, 39, 190, -64])
        continuous = raw.copy()
        continuous[3] -= 2*np.pi
        state = dict(telemetry=dict(joints=[
            dict(joint=i+1, position_rad=float(value)) for i, value in enumerate(raw)]))
        board = Mock()
        board.scalar.side_effect = lambda field, joint: float(continuous[joint-1])
        expected = np.rad2deg(continuous)-[
            np.rad2deg(r.zero_rad) for r in JOINT_REFERENCES.values()]
        np.testing.assert_allclose(angles(state, board), expected, atol=1e-12)
        self.assertAlmostEqual(angles(state)[3]-angles(state, board)[3], 360)
        self.assertTrue(all(call.args[0] == 16 for call in board.scalar.call_args_list))

    def test_drag_capability_is_checked_before_single_explicit_entry(self):
        board = Board(Mock())
        board.rpc = Mock(side_effect=[DRAG_VERSION, 0])
        board.drag()
        self.assertEqual([call.args[0] for call in board.rpc.call_args_list], [251, 15])
        self.assertEqual(board.rpc.call_args_list[-1].kwargs, dict(value=0x44524147))
        board.rpc = Mock(return_value=0)
        with self.assertRaises(ValueError):
            board.drag()
        board.rpc.assert_called_once_with(251)

    def test_single_leg_cli_rejects_ambiguous_requests_before_io(self):
        import subprocess
        tool = Path(__file__).resolve().parents[1]/"tools/five_gravity_sweep.py"
        for flags in (["--single-leg", "--joints", "1", "2"],
                      ["--direction", "1"]):
            run = subprocess.run([sys.executable, "-B", str(tool),
                "--port", "UNUSED", "--log", "UNUSED_CAPTURE.jsonl",
                "--constant-speed", ".8", *flags], capture_output=True, text=True)
            self.assertEqual(run.returncode, 2)
            self.assertIn("error:", run.stderr)

    def test_fast_analysis_keeps_same_time_support_and_derivative_units(self):
        from five_friction_map import derivative_kernels
        for grid_ms in (4, 10):
            half, kernels = derivative_kernels(grid_ms)
            dt = np.arange(-half, half+1)*grid_ms*.001
            values = .3+.7*dt+.4*dt**2+.1*dt**3
            np.testing.assert_allclose([k@values for k in kernels],
                                       [.3, .7, .8], atol=1e-12)
            self.assertAlmostEqual(dt[-1], .1)
        with self.assertRaises(ValueError):
            derivative_kernels(3)

    def test_campaign_uses_same_optional_ramp_for_path_and_command(self):
        from five_friction_campaign import Campaign
        from five_identify import REFS
        campaign = Campaign.__new__(Campaign)
        campaign.check, campaign.run_tool = Mock(), Mock()
        raw = np.rad2deg(REFS[:, 1:]).mean(axis=1)
        for speed, plateau, ramp in ((.48, 700, 400), (.08, 3000, 1000)):
            if ramp == 1000:
                campaign.sweep(raw, "offline", speed, "j5", plateau)
            else:
                campaign.sweep(raw, "offline", speed, "j1234", plateau, ramp)
            start, end = campaign.check.call_args.args[0][0]
            self.assertAlmostEqual(max(abs(np.array(end)-start)),
                                   np.rad2deg(speed)*(plateau+ramp)/1000)
            options = campaign.run_tool.call_args.args[2]
            self.assertEqual(options[options.index("--ramp-ms")+1], str(ramp))

    def test_fast_friction_bins_keep_original_actual_pose_tolerance(self):
        from five_friction_map import pair_legs
        from five_identify import REFS
        q = np.tile(np.deg2rad(HW[:, 6])-REFS[:, 0], (3, 1))
        q[:, 0] += np.deg2rad([.1, .4, .7])
        legs = [dict(group=(0, .48), joint=1, sign=sign, q=q.copy(),
                     v=np.full(3, .48), r=np.full(3, sign*.7),
                     dyn=np.zeros(3), temp=np.full(3, 30.))
                for sign in (1, -1)]
        self.assertEqual(pair_legs(legs), [])
        points = pair_legs(legs, angle_bin_deg=1.)
        self.assertEqual(len(points), 1)
        np.testing.assert_allclose(points[0]["targets"], [.7]*3)
        legs[1]["q"][:, 2] += np.deg2rad(.6)
        self.assertEqual(pair_legs(legs, angle_bin_deg=1.), [])
        for invalid in (0, -1, float("nan")):
            with self.assertRaises(ValueError):
                pair_legs(legs, angle_bin_deg=invalid)

    def test_speed_loop_command_is_capability_checked_single_commit_without_enable(self):
        board = Board(Mock())
        board.rpc = Mock(side_effect=lambda field, *args, **kwargs: CRUISE_VERSION if field == 252 else 0)
        board.speed(4, -.08, 5000, 1000)
        self.assertEqual(board.rpc.call_args_list[0].args, (252,))
        self.assertEqual(board.rpc.call_args_list[1].args, (12,))
        self.assertEqual(board.rpc.call_args_list[1].kwargs["scope"], 4)
        self.assertAlmostEqual(struct.unpack("<f", struct.pack("<I",
            board.rpc.call_args_list[1].kwargs["value"]))[0], -.08, places=7)
        self.assertEqual([call.args[0] for call in board.rpc.call_args_list], [252, 12, 14, 13])
        self.assertEqual(board.rpc.call_args_list[-1].kwargs, dict(value=5000))
        for kwargs in (dict(joint=0), dict(rad_s=0), dict(rad_s=float("nan")),
                       dict(ramp_ms=2500), dict(ramp_ms=0), dict(milliseconds=120001)):
            board.rpc.reset_mock()
            parameters = dict(joint=4, rad_s=.08, milliseconds=5000, ramp_ms=1000)
            with self.assertRaises(ValueError):
                board.speed(**(parameters | kwargs))
            board.rpc.assert_not_called()
        board.rpc = Mock(return_value=0)
        with self.assertRaises(ValueError):
            board.speed(4, .08, 5000)
        board.rpc.assert_called_once_with(252)

    def test_baseline_preserves_pose_raw_turn_and_original_range(self):
        from dm_monitor import JOINT_REFERENCES
        import math
        start = [math.degrees((r.lower_rad+r.upper_rad)/2)
                 for r in JOINT_REFERENCES.values()]
        for joint in range(1, 6):
            goal, direction, milliseconds = baseline_goal(start, joint, .08, 3000, 1000)
            self.assertEqual(milliseconds, 5000)
            self.assertAlmostEqual(math.radians(goal[joint-1]-start[joint-1]), direction*.32)
            for i in range(5):
                if i != joint-1:
                    self.assertEqual(goal[i], start[i])
            reverse, _, _ = baseline_goal(goal, joint, .08, 3000, 1000, -direction)
            np.testing.assert_allclose(reverse, start, atol=1e-13)
            # Newly requested fast trials: finite travel/time, not a new raw
            # range or a low-speed/fixed-angle admission gate.
            for fast in (.32, .48):
                goal, direction, milliseconds = baseline_goal(start, joint, fast, 400, 250)
                self.assertEqual(milliseconds, 900)
                self.assertAlmostEqual(math.radians(goal[joint-1]-start[joint-1]),
                                       direction*fast*.65)
                reverse, _, _ = baseline_goal(goal, joint, fast, 400, 250, -direction)
                np.testing.assert_allclose(reverse, start, atol=1e-13)
        near_upper = start.copy()
        near_upper[3] = math.degrees(JOINT_REFERENCES[4].upper_rad)-1.2
        _, direction, _ = baseline_goal(near_upper, 4, .08, 3000, 1000)
        self.assertEqual(direction, -1)  # Preserve +350 raw branch, never subtract a turn.
        with self.assertRaises(ValueError):
            baseline_goal(near_upper, 4, .08, 3000, 1000, 1)
        for values in ((.08, 0, 1000), (.08, 3000, 0), (float("nan"), 3000, 1000)):
            with self.assertRaises(ValueError):
                baseline_goal(start, 4, *values)

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
