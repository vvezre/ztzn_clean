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

    def test_runtime_correction_debug_is_published_for_dev_console(self):
        source = read_main()

        self.assertIn("correctionDebug", source)
        self.assertIn("'headingError'", source)
        self.assertIn("'zSpeed'", source)
        self.assertIn("redis_cli.set('globalGo'", source)


if __name__ == "__main__":
    unittest.main()
