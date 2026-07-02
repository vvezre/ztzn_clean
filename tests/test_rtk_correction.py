import unittest

from rtk_correction import compute_cte_dot, compute_heading_error, compute_linear_steering, low_pass_filter


class RTKCorrectionTest(unittest.TestCase):
    def test_heading_error_uses_shortest_signed_angle(self):
        self.assertEqual(compute_heading_error(2, 358), 4)
        self.assertEqual(compute_heading_error(358, 2), -4)

    def test_linear_steering_reverses_heading_and_cross_track_direction(self):
        result = compute_linear_steering(heading_error=3.5, cte=0.12)

        self.assertEqual(result, 85)

    def test_linear_steering_allows_legacy_observer_cte_gain(self):
        result = compute_linear_steering(heading_error=-2.0, cte=-0.05, cte_gain=800)

        self.assertEqual(result, -20)

    def test_cte_dot_returns_cross_track_velocity(self):
        self.assertAlmostEqual(compute_cte_dot(cte=0.16, last_cte=0.10, dt=0.2), 0.3)
        self.assertEqual(compute_cte_dot(cte=0.16, last_cte=0.10, dt=0), 0.0)

    def test_low_pass_filter_clamps_alpha(self):
        self.assertAlmostEqual(low_pass_filter(10, 20, alpha=0.8), 12.0)
        self.assertAlmostEqual(low_pass_filter(10, 20, alpha=2.0), 10.0)
        self.assertAlmostEqual(low_pass_filter(10, 20, alpha=-1.0), 20.0)

    def test_linear_steering_accepts_cte_derivative_term(self):
        result = compute_linear_steering(
            heading_error=1.0,
            cte=0.02,
            cte_gain=1000,
            cte_dot=0.5,
            cte_d_gain=40,
        )

        self.assertEqual(result, 30)


if __name__ == "__main__":
    unittest.main()
