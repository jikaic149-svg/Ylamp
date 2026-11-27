"""Pure offline tests; never open a device."""
from pathlib import Path
import sys
import unittest

import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"tools"))
from five_cruise_plan import (SmoothedCurve, make_raw, velocity_transition,
                              verify_kinematics, integrate, continuous_extrema_check)


class CruisePlanTests(unittest.TestCase):
    def test_transition_matches_triangular_and_acceleration_limited_integrals(self):
        for v0, v1, a, j in ((0., .1, 1., 4.), (.5, -.5, 1., 4.),
                              (0., .6, 2., 8.)):
            state = np.array([0., v0, 0.])
            for duration, value in velocity_transition(v0, v1, a, j):
                state = integrate(state, duration, value)
            np.testing.assert_allclose(state[1:], [v1, 0.], atol=1e-12)

    def test_post_smoothing_preserves_origin_range_derivatives_and_turn_count(self):
        curve = SmoothedCurve(make_raw(.5, 1., 4., cycles=3), .06)
        check = verify_kinematics(curve)
        self.assertTrue(check["passed"])
        self.assertEqual(check["sign_reversals"], 6)
        np.testing.assert_allclose(curve.sample([-1., curve.duration+1.]), 0., atol=1e-12)
        t, grid = curve.grid(.002)
        # Independent grid/continuous evaluation must agree.
        np.testing.assert_allclose(curve.sample(t[::113]), grid[::113], atol=2e-10)
        for row in curve.raw.segments:
            for boundary in (row[0], row[0]+curve.width):
                near = curve.sample([boundary-1e-8, boundary+1e-8])
                self.assertLess(abs(near[1, 3]-near[0, 3]), 1e-5)
        self.assertEqual(check["metrics"]["zero_velocity_dwell_s"], 0.)
        self.assertTrue(continuous_extrema_check(curve)["passed"])

    def test_rejects_unfit_excursion_and_bad_inputs(self):
        for args in ((.8, .1, .1), (0., 1., 4.), (np.nan, 1., 4.)):
            with self.assertRaises(ValueError):
                make_raw(*args)
        with self.assertRaises(ValueError):
            SmoothedCurve(make_raw(.5, 1., 4., cycles=1), -.02)


if __name__ == "__main__":
    unittest.main()
