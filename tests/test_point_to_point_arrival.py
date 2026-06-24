import unittest

import util


class PointToPointArrivalTest(unittest.TestCase):
    def test_arrives_when_direct_distance_is_within_tolerance(self):
        self.assertTrue(
            util.should_finish_point_to_point(
                distance_to_target=0.02,
                signed_remaining=0.5,
                cte=1.0,
            )
        )

    def test_arrives_when_target_is_passed_and_cross_track_is_small(self):
        self.assertTrue(
            util.should_finish_point_to_point(
                distance_to_target=0.2,
                signed_remaining=-0.01,
                cte=0.3,
            )
        )

    def test_does_not_arrive_when_target_is_passed_but_cross_track_is_large(self):
        self.assertFalse(
            util.should_finish_point_to_point(
                distance_to_target=0.2,
                signed_remaining=-0.01,
                cte=0.31,
            )
        )

    def test_does_not_arrive_before_target_when_not_close(self):
        self.assertFalse(
            util.should_finish_point_to_point(
                distance_to_target=0.2,
                signed_remaining=0.1,
                cte=0.0,
            )
        )


if __name__ == "__main__":
    unittest.main()
