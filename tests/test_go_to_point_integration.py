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
        self.assertIn("pointToPointByRTK(", body)
        self.assertIn("set_current_action('go_to_point')", body)


if __name__ == "__main__":
    unittest.main()
