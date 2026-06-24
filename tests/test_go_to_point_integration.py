import io
import os
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def read_main():
    with io.open(os.path.join(ROOT, "main.py"), "r", encoding="utf-8", errors="ignore") as fh:
        return fh.read()


class GoToPointIntegrationTest(unittest.TestCase):
    def test_main_exposes_go_to_point_route(self):
        body = read_main()

        self.assertIn("from go_to_point import", body)
        self.assertIn('@app.route("/vehicle/goToPoint"', body)
        self.assertIn("def goToPoint():", body)
        self.assertIn("def goToPointThread(plan):", body)
        self.assertIn("build_go_to_point_plan(", body)
        self.assertIn("def pointToPointByRTKAutoHeading(", body)
        self.assertIn("util.get_distance_angle(", body)
        self.assertIn("current_start_lat", body)
        self.assertIn("current_start_lon", body)
        self.assertIn("return pointToPointByRTK(", body)
        self.assertIn("set_current_action('go_to_point')", body)

    def test_auto_heading_turns_before_driving(self):
        body = read_main()

        self.assertIn("turn_result = turn(", body)
        self.assertIn("heading * 10,", body)
        self.assertIn("source='point_to_point_auto_heading'", body)
        self.assertIn("if turn_result != 1:", body)

    def test_multi_waypoint_start_accepts_loop_options(self):
        body = read_main()

        self.assertIn("normalize_loop_options(", body)
        self.assertIn("iter_closed_loop_targets(", body)
        self.assertIn("'waypointLoopEnabled'", body)
        self.assertIn("'waypointLoopMode'", body)
        self.assertIn("'waypointLoopTarget'", body)
        self.assertIn("'waypointLoopCurrent'", body)

    def test_multi_waypoint_progress_exposes_loop_status(self):
        body = read_main()

        self.assertIn('"loopMode"', body)
        self.assertIn('"currentLoop"', body)
        self.assertIn('"targetLoop"', body)


if __name__ == "__main__":
    unittest.main()
