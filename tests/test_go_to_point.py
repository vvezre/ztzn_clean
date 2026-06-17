import unittest

from go_to_point import build_go_to_point_plan


class GoToPointTest(unittest.TestCase):
    def test_rejects_invalid_target_coordinate(self):
        plan = build_go_to_point_plan(
            current_lat=31.1,
            current_lon=121.1,
            target_lat="bad",
            target_lon=121.2,
        )

        self.assertFalse(plan["success"])
        self.assertEqual(plan["code"], "INVALID_TARGET")

    def test_rejects_missing_current_rtk(self):
        plan = build_go_to_point_plan(
            current_lat=None,
            current_lon=121.1,
            target_lat=31.2,
            target_lon=121.2,
        )

        self.assertFalse(plan["success"])
        self.assertEqual(plan["code"], "CURRENT_RTK_UNAVAILABLE")

    def test_rejects_target_that_is_too_close(self):
        plan = build_go_to_point_plan(
            current_lat=31.1,
            current_lon=121.1,
            target_lat=31.1,
            target_lon=121.1,
            min_distance_m=0.3,
        )

        self.assertFalse(plan["success"])
        self.assertEqual(plan["code"], "TARGET_TOO_CLOSE")
        self.assertEqual(plan["data"]["distance"], 0.0)

    def test_builds_go_to_point_plan_with_heading_and_distance(self):
        plan = build_go_to_point_plan(
            current_lat=31.1,
            current_lon=121.1,
            target_lat=31.1001,
            target_lon=121.1001,
            speed=260,
        )

        self.assertTrue(plan["success"])
        self.assertEqual(plan["code"], "GO_TO_POINT_READY")
        self.assertEqual(plan["data"]["startLat"], 31.1)
        self.assertEqual(plan["data"]["startLon"], 121.1)
        self.assertEqual(plan["data"]["targetLat"], 31.1001)
        self.assertEqual(plan["data"]["targetLon"], 121.1001)
        self.assertEqual(plan["data"]["speed"], 260)
        self.assertGreater(plan["data"]["distance"], 0.3)
        self.assertGreaterEqual(plan["data"]["heading"], 0)
        self.assertLess(plan["data"]["heading"], 360)


if __name__ == "__main__":
    unittest.main()
