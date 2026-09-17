import os
import sys
import unittest


sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from turn_heading_control import choose_turn_direction, plan_turn_timeout_recovery


class TurnHeadingControlTest(unittest.TestCase):
    def test_selects_right_turn_for_clockwise_target(self):
        direction, angle = choose_turn_direction(90, 180)
        self.assertEqual(direction, "right")
        self.assertAlmostEqual(angle, 90.0)

    def test_selects_short_right_turn_across_zero(self):
        direction, angle = choose_turn_direction(350, 10)
        self.assertEqual(direction, "right")
        self.assertAlmostEqual(angle, 20.0)

    def test_selects_short_left_turn_across_zero(self):
        direction, angle = choose_turn_direction(10, 350)
        self.assertEqual(direction, "left")
        self.assertAlmostEqual(angle, 20.0)

    def test_does_not_turn_when_already_at_target(self):
        direction, angle = choose_turn_direction(180, 180)
        self.assertEqual(direction, "none")
        self.assertAlmostEqual(angle, 0.0)

    def test_chooses_right_for_exact_half_turn(self):
        direction, angle = choose_turn_direction(90, 270)
        self.assertEqual(direction, "right")
        self.assertAlmostEqual(angle, 180.0)

    def test_timeout_waits_while_rtk_is_not_fixed(self):
        plan = plan_turn_timeout_recovery(90, 180, False)

        self.assertEqual(plan["action"], "wait_rtk")

    def test_timeout_waits_when_fresh_heading_is_unavailable(self):
        plan = plan_turn_timeout_recovery(None, 180, True)

        self.assertEqual(plan["action"], "wait_rtk")

    def test_timeout_completes_when_recheck_is_within_tolerance(self):
        plan = plan_turn_timeout_recovery(1.5, 0, True, 2.0)

        self.assertEqual(plan["action"], "complete")

    def test_timeout_recalculates_shortest_retry_from_current_heading(self):
        plan = plan_turn_timeout_recovery(350, 10, True, 2.0)

        self.assertEqual(plan["action"], "retry")
        self.assertEqual(plan["direction"], "right")
        self.assertAlmostEqual(plan["relativeAngle"], 20.0)


if __name__ == "__main__":
    unittest.main()
