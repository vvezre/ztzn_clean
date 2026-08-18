import ast
import os
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load_arrival_function():
    with open(os.path.join(ROOT, "util.py"), "r", encoding="utf-8") as handle:
        tree = ast.parse(handle.read(), filename="util.py")
    function_node = next(
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "should_finish_point_to_point"
    )
    namespace = {}
    module = ast.Module(body=[function_node], type_ignores=[])
    exec(compile(module, "util.py", "exec"), namespace)
    return namespace["should_finish_point_to_point"]


should_finish_point_to_point = load_arrival_function()


class PointToPointArrivalTest(unittest.TestCase):
    def test_arrives_when_direct_distance_is_within_tolerance(self):
        self.assertTrue(should_finish_point_to_point(
            distance_to_target=0.08,
            signed_remaining=0.5,
            cte=1.0,
        ))

    def test_arrives_after_small_overshoot_near_target(self):
        self.assertTrue(should_finish_point_to_point(
            distance_to_target=0.12,
            signed_remaining=-0.03,
            cte=0.05,
        ))

    def test_does_not_arrive_when_passed_but_still_far_from_target(self):
        self.assertFalse(should_finish_point_to_point(
            distance_to_target=1.471,
            signed_remaining=-1.441,
            cte=-0.297,
        ))

    def test_does_not_arrive_when_target_is_passed_but_cross_track_is_large(self):
        self.assertFalse(should_finish_point_to_point(
            distance_to_target=0.12,
            signed_remaining=-0.01,
            cte=0.11,
        ))

    def test_does_not_arrive_before_target_when_not_close(self):
        self.assertFalse(should_finish_point_to_point(
            distance_to_target=0.2,
            signed_remaining=0.1,
            cte=0.0,
        ))


if __name__ == "__main__":
    unittest.main()
