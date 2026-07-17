import os
import unittest
import io


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def read_main():
    with io.open(os.path.join(ROOT, "main.py"), "r", encoding="utf-8", errors="ignore") as fh:
        return fh.read()


class EdgeTargetGuardIntegrationTest(unittest.TestCase):
    def test_main_wires_edge_target_guard_into_motion_flow(self):
        body = read_main()
        self.assertIn("from edge_target_guard import", body)
        self.assertIn("should_accept_edge_stop", body)
        self.assertIn("def _distance_to_current_task_target", body)
        self.assertIn("def _handle_edge_stop_for_current_task", body)
        self.assertIn("def _recover_from_abnormal_edge", body)
        self.assertIn("_handle_edge_stop_for_current_task('moveDiatance')", body)

    def test_auto_clean_stops_when_distance_segment_fails(self):
        body = read_main()
        self.assertIn("if not moveDiatance(ser, length, 250):", body)
        self.assertIn("mode2 moveDiatance failed", body)

    def test_far_edge_recovers_and_resumes_distance_segment(self):
        body = read_main()
        self.assertIn("EDGE_STOP_ACTION_RECOVER", body)
        self.assertIn("_recover_from_abnormal_edge('moveDiatance')", body)
        self.assertIn("_send_distance_move_command(length, speed)", body)

    def test_main_latches_edge_until_sensor_is_released(self):
        body = read_main()

        self.assertIn("EdgeTriggerLatch", body)
        self.assertIn("global_edge_trigger_latch = EdgeTriggerLatch()", body)
        self.assertGreaterEqual(
            body.count("global_edge_trigger_latch.consume(getEdge())"),
            3,
        )


if __name__ == "__main__":
    unittest.main()
