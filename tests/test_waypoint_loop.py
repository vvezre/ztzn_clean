import unittest
from itertools import islice

from waypoint_loop import (
    build_closed_loop_targets,
    iter_closed_loop_targets,
    normalize_loop_options,
)


WAYPOINTS = [
    {"lat": 1.0, "lon": 1.0},
    {"lat": 2.0, "lon": 2.0},
    {"lat": 3.0, "lon": 3.0},
    {"lat": 4.0, "lon": 4.0},
]


class WaypointLoopTest(unittest.TestCase):
    def test_closed_loop_targets_approach_first_point_then_close_each_cycle(self):
        targets = build_closed_loop_targets(WAYPOINTS, loop_count=2)

        self.assertEqual(
            [target["index"] for target in targets],
            [0, 1, 2, 3, 0, 1, 2, 3, 0],
        )
        self.assertEqual(
            [target["completed_loop"] for target in targets],
            [0, 0, 0, 0, 1, 1, 1, 1, 2],
        )

    def test_continuous_loop_targets_repeat_after_each_return_to_first_point(self):
        targets = list(islice(iter_closed_loop_targets(WAYPOINTS), 9))

        self.assertEqual(
            [target["index"] for target in targets],
            [0, 1, 2, 3, 0, 1, 2, 3, 0],
        )
        self.assertEqual(targets[-1]["completed_loop"], 2)

    def test_normalize_count_loop_requires_positive_count_and_two_points(self):
        with self.assertRaisesRegex(ValueError, "at least two waypoints"):
            normalize_loop_options(
                {"loop": True, "loopMode": "count", "loopCount": 2},
                waypoint_count=1,
            )

        with self.assertRaisesRegex(ValueError, "loop count"):
            normalize_loop_options(
                {"loop": True, "loopMode": "count", "loopCount": 0},
                waypoint_count=2,
            )

    def test_normalize_continuous_loop_uses_zero_target(self):
        options = normalize_loop_options(
            {"loop": True, "loopMode": "continuous", "loopCount": 99},
            waypoint_count=2,
        )

        self.assertEqual(
            options,
            {
                "loop": True,
                "loopMode": "continuous",
                "loopCount": 0,
            },
        )

    def test_normalize_missing_options_preserves_single_pass_compatibility(self):
        options = normalize_loop_options({}, waypoint_count=1)

        self.assertEqual(
            options,
            {
                "loop": False,
                "loopMode": "count",
                "loopCount": 1,
            },
        )


if __name__ == "__main__":
    unittest.main()
