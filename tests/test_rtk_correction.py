import unittest

from rtk_correction import compute_heading_error, compute_linear_steering


class RTKCorrectionTest(unittest.TestCase):
    def test_heading_error_uses_shortest_signed_angle(self):
        self.assertEqual(compute_heading_error(2, 358), 4)
        self.assertEqual(compute_heading_error(358, 2), -4)

    def test_linear_steering_matches_legacy_formula(self):
        result = compute_linear_steering(heading_error=3.5, cte=0.12)

        self.assertEqual(result, -85)

    def test_linear_steering_allows_legacy_observer_cte_gain(self):
        result = compute_linear_steering(heading_error=-2.0, cte=-0.05, cte_gain=800)

        self.assertEqual(result, 20)


if __name__ == "__main__":
    unittest.main()
