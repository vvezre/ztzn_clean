import io
import os
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def read_main():
    with io.open(os.path.join(ROOT, "main.py"), "r", encoding="utf-8", errors="ignore") as handle:
        return handle.read()


class PointToPointCorrectionParamsTest(unittest.TestCase):
    def test_short_range_heading_limit_and_cross_track_gain_are_tuned(self):
        source = read_main()

        self.assertIn("if distance_to_target < 1:", source)
        self.assertIn("compute_linear_steering(heading_error, cte, cte_gain=1000)", source)
        self.assertIn("cte_gain=1000 z={}", source)


if __name__ == "__main__":
    unittest.main()
