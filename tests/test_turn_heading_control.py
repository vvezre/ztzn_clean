import os
import sys
import unittest


sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from turn_heading_control import choose_turn_direction


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


if __name__ == "__main__":
    unittest.main()
