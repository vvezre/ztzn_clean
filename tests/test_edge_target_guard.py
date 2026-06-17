import unittest
import os
import sys


sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from edge_target_guard import should_accept_edge_stop, should_recover_from_edge_stop


class EdgeTargetGuardTest(unittest.TestCase):
    def test_accepts_edge_when_near_segment_target(self):
        self.assertTrue(should_accept_edge_stop(0.3, tolerance_m=0.5))
        self.assertTrue(should_accept_edge_stop(0.5, tolerance_m=0.5))

    def test_rejects_edge_when_far_from_segment_target(self):
        self.assertFalse(should_accept_edge_stop(0.8, tolerance_m=0.5))

    def test_rejects_edge_when_distance_is_unknown(self):
        self.assertFalse(should_accept_edge_stop(None, tolerance_m=0.5))

    def test_recovers_only_when_far_from_segment_target(self):
        self.assertTrue(should_recover_from_edge_stop(0.8, tolerance_m=0.5))
        self.assertFalse(should_recover_from_edge_stop(0.5, tolerance_m=0.5))
        self.assertFalse(should_recover_from_edge_stop(None, tolerance_m=0.5))


if __name__ == "__main__":
    unittest.main()
