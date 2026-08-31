# coding=utf-8
from contextlib import contextmanager
import shutil
import tempfile
import time
import unittest

from manual_steering import ManualSteeringController, ManualSteeringError
from motion_state import derive_motion_state


@contextmanager
def temporary_directory():
    """TemporaryDirectory equivalent that also runs on the robot's Python 2.7."""
    directory = tempfile.mkdtemp()
    try:
        yield directory
    finally:
        shutil.rmtree(directory, ignore_errors=True)


class ManualSteeringControllerTest(unittest.TestCase):
    def _controller(self, motion="forward", timeout=1.5, tap_duration=0.05):
        state = {
            "motion": motion,
            "now": 10.0,
            "speed": {"forward": 350, "reverse": 100},
        }
        events = []
        controller = ManualSteeringController(
            motion_provider=lambda: state["motion"],
            apply_trim=lambda value, mode, speed: events.append(("trim", value, mode, speed)),
            start_rotation=lambda direction: events.append(("rotate", direction)),
            stop_rotation=lambda: events.append(("stop_rotate",)),
            travel_speed_provider=lambda mode: state["speed"][mode],
            now=lambda: state["now"],
            moving_value=700,
            tap_duration=tap_duration,
            keepalive_timeout=timeout,
        )
        return controller, state, events

    def test_forward_tap_uses_fixed_left_diagonal_then_restores_straight(self):
        controller, state, events = self._controller("forward")
        try:
            first = controller.handle({"action": "tap", "direction": "left", "controlId": "tap-1", "sequence": 1})
            self.assertEqual(-700, first["data"]["manualCorrectionValue"])
            self.assertEqual("forward_tap", first["data"]["manualSteeringMode"])
            self.assertEqual(("trim", -700, "forward", 350), events[-1])

            time.sleep(0.08)
            self.assertEqual(("trim", 0, "forward", 350), events[-1])
            self.assertEqual("none", controller.snapshot()["manualSteeringMode"])
        finally:
            controller.close()

    def test_reverse_tap_uses_fixed_left_rear_diagonal(self):
        controller, state, events = self._controller("reverse")
        try:
            result = controller.handle({"action": "tap", "direction": "left", "controlId": "tap-r", "sequence": 1})
            self.assertEqual(-700, result["data"]["manualCorrectionValue"])
            self.assertEqual(("trim", -700, "reverse", 100), events[-1])
        finally:
            controller.close()

    def test_stopped_tap_is_rejected_but_hold_rotates_until_release(self):
        controller, state, events = self._controller("stopped")
        try:
            with self.assertRaises(ManualSteeringError) as raised:
                controller.handle({"action": "tap", "direction": "right", "controlId": "tap-s", "sequence": 1})
            self.assertEqual("TAP_REQUIRES_MOVEMENT", raised.exception.code)
            started = controller.handle({"action": "hold_start", "direction": "right", "controlId": "hold-s", "sequence": 1})
            self.assertEqual("turning", started["data"]["motionState"])
            controller.handle({"action": "hold_stop", "direction": "right", "controlId": "hold-s", "sequence": 2})
            self.assertEqual([("rotate", "right"), ("stop_rotate",)], events)
        finally:
            controller.close()

    def test_rotation_watchdog_stops_hardware(self):
        controller, state, events = self._controller("stopped", timeout=0.2)
        try:
            controller.handle({"action": "hold_start", "direction": "left", "controlId": "timeout", "sequence": 1})
            state["now"] += 1.0
            time.sleep(0.25)
            self.assertIn(("stop_rotate",), events)
            self.assertEqual("none", controller.snapshot()["manualSteeringMode"])
        finally:
            controller.close()

    def test_rotation_keepalive_accepts_real_lower_machine_motion_report(self):
        controller, state, events = self._controller("stopped")
        try:
            controller.handle({"action": "hold_start", "direction": "right", "controlId": "rotate", "sequence": 1})

            # The joystick-style rotation command can make real hardware
            # report a signed longitudinal speed even though the active manual
            # session is still an in-place rotation.
            state["motion"] = "forward"
            kept = controller.handle({"action": "keepalive", "direction": "right", "controlId": "rotate", "sequence": 2})

            self.assertTrue(kept["success"])
            self.assertEqual("in_place_rotate", kept["data"]["manualSteeringMode"])
            self.assertEqual("turning", kept["data"]["motionState"])
            self.assertNotIn(("stop_rotate",), events)
        finally:
            controller.close()

    def test_repeated_fixed_sequence_keeps_rotation_alive(self):
        controller, state, events = self._controller("stopped", timeout=0.2)
        try:
            controller.handle({"action": "hold_start", "direction": "right", "controlId": "fixed-sequence", "sequence": 1})
            state["motion"] = "forward"

            for _ in range(3):
                state["now"] += 0.15
                kept = controller.handle({"action": "keepalive", "direction": "right", "controlId": "fixed-sequence", "sequence": 2})
                self.assertTrue(kept["success"])

            time.sleep(0.25)
            self.assertEqual("in_place_rotate", controller.snapshot()["manualSteeringMode"])
            self.assertNotIn(("stop_rotate",), events)
        finally:
            controller.close()

    def test_repeated_fixed_sequence_keeps_one_fixed_moving_direction(self):
        controller, state, events = self._controller("forward")
        try:
            started = controller.handle({"action": "hold_start", "direction": "right", "controlId": "fixed-trim", "sequence": 1})
            self.assertEqual(700, started["data"]["manualCorrectionValue"])

            state["now"] += 0.5
            first = controller.handle({"action": "keepalive", "direction": "right", "controlId": "fixed-trim", "sequence": 2})
            self.assertEqual(700, first["data"]["manualCorrectionValue"])

            state["now"] += 0.05
            duplicate = controller.handle({"action": "keepalive", "direction": "right", "controlId": "fixed-trim", "sequence": 2})
            self.assertEqual(700, duplicate["data"]["manualCorrectionValue"])

            state["now"] += 0.5
            kept = controller.handle({"action": "keepalive", "direction": "right", "controlId": "fixed-trim", "sequence": 2})
            self.assertEqual(700, kept["data"]["manualCorrectionValue"])
            self.assertEqual([
                ("trim", 700, "forward", 350),
                ("trim", 700, "forward", 350),
                ("trim", 700, "forward", 350),
                ("trim", 700, "forward", 350),
            ], events)
        finally:
            controller.close()

    def test_moving_hold_release_sends_one_zero_and_clears_correction(self):
        controller, state, events = self._controller("forward")
        try:
            controller.handle({"action": "hold_start", "direction": "right", "controlId": "release", "sequence": 1})
            state["now"] += 0.5
            controller.handle({"action": "keepalive", "direction": "right", "controlId": "release", "sequence": 2})

            released = controller.handle({"action": "hold_stop", "direction": "right", "controlId": "release", "sequence": 3})

            self.assertEqual(("trim", 0, "forward", 350), events[-1])
            self.assertEqual(1, events.count(("trim", 0, "forward", 350)))
            self.assertEqual(0, released["data"]["manualCorrectionValue"])
            self.assertEqual(0, released["data"]["manualCorrectionLevel"])
            self.assertEqual("none", released["data"]["manualSteeringMode"])
        finally:
            controller.close()

    def test_moving_hold_locks_commanded_speed_while_feedback_or_setting_changes(self):
        controller, state, events = self._controller("forward")
        try:
            controller.handle({"action": "hold_start", "direction": "right", "controlId": "speed", "sequence": 1})
            # Simulate a lower X-speed drop or a concurrent setting update.  A
            # hold must keep using the 350 command captured at its start.
            state["speed"]["forward"] = 120
            state["now"] += 0.4
            controller.handle({"action": "keepalive", "direction": "right", "controlId": "speed", "sequence": 2})
            controller.handle({"action": "hold_stop", "direction": "right", "controlId": "speed", "sequence": 3})

            self.assertEqual([
                ("trim", 700, "forward", 350),
                ("trim", 700, "forward", 350),
                ("trim", 0, "forward", 350),
            ], events)
        finally:
            controller.close()

    def test_tap_then_different_control_keepalive_promotes_one_hold(self):
        controller, state, events = self._controller("forward", tap_duration=0.2)
        try:
            controller.handle({"action": "tap", "direction": "left", "controlId": "tap-id", "sequence": 1})
            promoted = controller.handle({"action": "keepalive", "direction": "left", "controlId": "hold-id", "sequence": 2})

            self.assertEqual("forward_trim", promoted["data"]["manualSteeringMode"])
            self.assertEqual("hold-id", promoted["data"]["manualSteeringControlId"])
            stray_tap = controller.handle({"action": "tap", "direction": "left", "controlId": "tap-id", "sequence": 1})
            self.assertEqual("tap ignored while hold is active", stray_tap["message"])
            self.assertEqual("hold-id", stray_tap["data"]["manualSteeringControlId"])
            controller.handle({"action": "keepalive", "direction": "left", "controlId": "hold-id", "sequence": 2})
            time.sleep(0.25)
            # The invalidated tap timer must not restore straight during hold.
            self.assertEqual(("trim", -700, "forward", 350), events[-1])

            controller.handle({"action": "hold_stop", "direction": "left", "controlId": "hold-id", "sequence": 6})
            self.assertEqual(("trim", 0, "forward", 350), events[-1])
        finally:
            controller.close()

    def test_hold_stop_tombstone_ignores_late_keepalive(self):
        controller, state, events = self._controller("forward")
        try:
            controller.handle({"action": "hold_start", "direction": "right", "controlId": "late", "sequence": 1})
            controller.handle({"action": "hold_stop", "direction": "right", "controlId": "late", "sequence": 6})
            event_count = len(events)

            stale = controller.handle({"action": "keepalive", "direction": "right", "controlId": "late", "sequence": 2})

            self.assertEqual("stale keepalive ignored", stale["message"])
            self.assertEqual(event_count, len(events))
            self.assertEqual("none", controller.snapshot()["manualSteeringMode"])
        finally:
            controller.close()

    def test_duplicate_tap_does_not_restart_pulse(self):
        controller, state, events = self._controller("forward", tap_duration=0.1)
        try:
            controller.handle({"action": "tap", "direction": "right", "controlId": "duplicate-tap", "sequence": 1})
            duplicate = controller.handle({"action": "tap", "direction": "right", "controlId": "duplicate-tap", "sequence": 1})

            self.assertEqual("duplicate tap ignored", duplicate["message"])
            self.assertEqual(1, events.count(("trim", 700, "forward", 350)))
            time.sleep(0.15)
            self.assertEqual(1, events.count(("trim", 0, "forward", 350)))
        finally:
            controller.close()

    def test_opposite_hold_switches_without_speed_step(self):
        controller, state, events = self._controller("forward")
        try:
            controller.handle({"action": "hold_start", "direction": "right", "controlId": "right", "sequence": 1})
            switched = controller.handle({"action": "hold_start", "direction": "left", "controlId": "left", "sequence": 1})
            controller.handle({"action": "hold_stop", "direction": "left", "controlId": "left", "sequence": 6})

            self.assertEqual("left", switched["data"]["manualSteeringDirection"])
            self.assertEqual([
                ("trim", 700, "forward", 350),
                ("trim", -700, "forward", 350),
                ("trim", 0, "forward", 350),
            ], events)
        finally:
            controller.close()

    def test_moving_hold_watchdog_restores_straight(self):
        controller, state, events = self._controller("forward", timeout=0.2)
        try:
            controller.handle({"action": "hold_start", "direction": "right", "controlId": "timeout-moving", "sequence": 1})
            state["now"] += 1.0
            time.sleep(0.25)

            self.assertEqual(("trim", 0, "forward", 350), events[-1])
            self.assertEqual("none", controller.snapshot()["manualSteeringMode"])
        finally:
            controller.close()

    def test_reset_invalidates_delayed_tap_restore(self):
        controller, state, events = self._controller("forward", tap_duration=0.1)
        try:
            controller.handle({"action": "tap", "direction": "right", "controlId": "tap-reset", "sequence": 1})
            controller.reset(send_hardware=False)
            time.sleep(0.15)

            self.assertEqual([("trim", 700, "forward", 350)], events)
            self.assertEqual("none", controller.snapshot()["manualSteeringMode"])
        finally:
            controller.close()

    def test_hold_stop_during_tap_restores_straight_once(self):
        controller, state, events = self._controller("forward", tap_duration=0.1)
        try:
            controller.handle({"action": "tap", "direction": "left", "controlId": "tap-stop", "sequence": 1})
            stopped = controller.handle({"action": "hold_stop", "direction": "left", "controlId": "tap-stop", "sequence": 2})
            time.sleep(0.15)

            self.assertEqual([
                ("trim", -700, "forward", 350),
                ("trim", 0, "forward", 350),
            ], events)
            self.assertEqual("none", stopped["data"]["manualSteeringMode"])
        finally:
            controller.close()

    def test_rotation_duplicate_start_is_idempotent_after_motion_report_changes(self):
        controller, state, events = self._controller("stopped")
        try:
            controller.handle({"action": "hold_start", "direction": "left", "controlId": "duplicate", "sequence": 1})
            state["motion"] = "turning"

            duplicate = controller.handle({"action": "hold_start", "direction": "left", "controlId": "duplicate", "sequence": 1})

            self.assertTrue(duplicate["success"])
            self.assertEqual("duplicate hold_start ignored", duplicate["message"])
            self.assertEqual([("rotate", "left")], events)
        finally:
            controller.close()

    def test_rotation_keepalive_still_stops_when_auto_control_takes_over(self):
        controller, state, events = self._controller("stopped")
        try:
            controller.handle({"action": "hold_start", "direction": "left", "controlId": "auto-takeover", "sequence": 1})
            state["motion"] = "auto"

            with self.assertRaises(ManualSteeringError) as changed:
                controller.handle({"action": "keepalive", "direction": "left", "controlId": "auto-takeover", "sequence": 2})

            self.assertEqual("MOTION_STATE_CHANGED", changed.exception.code)
            self.assertEqual("none", controller.snapshot()["manualSteeringMode"])
            self.assertIn(("stop_rotate",), events)
        finally:
            controller.close()

    def test_auto_is_blocked_and_motion_change_ends_active_hold(self):
        controller, state, events = self._controller("auto")
        try:
            with self.assertRaises(ManualSteeringError) as blocked:
                controller.handle({"action": "hold_start", "direction": "left", "controlId": "auto", "sequence": 1})
            self.assertEqual("MANUAL_STEERING_BLOCKED", blocked.exception.code)

            state["motion"] = "forward"
            controller.handle({"action": "hold_start", "direction": "right", "controlId": "change", "sequence": 1})
            state["motion"] = "reverse"
            with self.assertRaises(ManualSteeringError) as changed:
                controller.handle({"action": "keepalive", "direction": "right", "controlId": "change", "sequence": 2})
            self.assertEqual("MOTION_STATE_CHANGED", changed.exception.code)
            self.assertEqual("none", controller.snapshot()["manualSteeringMode"])
        finally:
            controller.close()

    def test_motion_state_derivation_uses_manual_rotation_session(self):
        self.assertEqual("reverse", derive_motion_state(-100, lower_status=1))
        self.assertEqual("stopped", derive_motion_state(0, lower_status=4))
        self.assertEqual("turning", derive_motion_state(0, lower_status=4, manual_mode="in_place_rotate"))
        self.assertEqual("auto", derive_motion_state(100, lower_status=1, control_state="RUNNING"))
        self.assertEqual("fault", derive_motion_state(0, lower_status=1, control_state="FAULT"))
        self.assertEqual("unknown", derive_motion_state(0, lower_status=1, control_state="DISABLED"))


class ManualSteeringSimulatorTest(unittest.TestCase):
    def test_simulator_accepts_full_mqtt_button_flow_without_moving_position(self):
        from modeling_simulator import ModelingSimulatorController
        from mqtt_handler import MQTTCommandHandler

        with temporary_directory() as directory:
            controller = ModelingSimulatorController(directory)
            handler = MQTTCommandHandler(controller)
            try:
                controller.drive()
                response = handler.handle({
                    "command": "manual_steering",
                    "params": {"action": "hold_start", "direction": "right", "controlId": "ui-1", "sequence": 1},
                })
                self.assertTrue(response["success"])
                self.assertEqual("forward_trim", response["data"]["manualSteeringMode"])
                self.assertEqual(700, response["data"]["manualCorrectionValue"])
                status = controller.status_snapshot()
                self.assertEqual("forward", status["motionState"])
                self.assertEqual(700, status["manualCorrectionValue"])

                kept = handler.handle({
                    "command": "manual_steering",
                    "params": {"action": "keepalive", "direction": "right", "controlId": "ui-1", "sequence": 2},
                })
                self.assertTrue(kept["success"])
                self.assertEqual(700, kept["data"]["manualCorrectionValue"])
                released = handler.handle({
                    "command": "manual_steering",
                    "params": {"action": "hold_stop", "direction": "right", "controlId": "ui-1", "sequence": 3},
                })
                self.assertTrue(released["success"])
                self.assertEqual("none", released["data"]["manualSteeringMode"])
                self.assertEqual(0, released["data"]["manualCorrectionValue"])

                controller.parking()
                rotating = handler.handle({
                    "command": "manual_steering",
                    "params": {"action": "hold_start", "direction": "left", "controlId": "ui-2", "sequence": 1},
                })
                self.assertTrue(rotating["success"])
                self.assertEqual("turning", rotating["data"]["motionState"])
                events = controller.status_snapshot()["simulation_manual_events"]
                self.assertEqual("rotation_start", events[-1]["event"])
                stopped = handler.handle({
                    "command": "manual_steering",
                    "params": {"action": "hold_stop", "direction": "left", "controlId": "ui-2", "sequence": 2},
                })
                self.assertTrue(stopped["success"])
                events = controller.status_snapshot()["simulation_manual_events"]
                self.assertEqual("rotation_stop", events[-1]["event"])
            finally:
                controller.manual_steering_controller.close()

    def test_simulator_mqtt_publishes_command_result_and_updated_status(self):
        from modeling_simulator import ModelingSimulatorController, ModelingSimulatorMQTTService

        class FakeMQTTClient(object):
            def __init__(self):
                self.callback = None
                self.status_messages = []
                self.position_messages = []

            def set_message_callback(self, callback):
                self.callback = callback

            def publish_status(self, message):
                self.status_messages.append(message)
                return True

            def publish_realtime(self, message):
                self.position_messages.append(message)
                return True

        with temporary_directory() as directory:
            controller = ModelingSimulatorController(directory)
            mqtt = FakeMQTTClient()
            service = ModelingSimulatorMQTTService({}, controller, mqtt_client=mqtt)
            try:
                service._on_message({
                    "command": "manual_steering",
                    "command_id": "cmd-test-1",
                    "trace_id": "trace-test-1",
                    "params": {
                        "action": "hold_start",
                        "direction": "right",
                        "controlId": "ui-mqtt-1",
                        "sequence": 1,
                    },
                })
                result = [item for item in mqtt.status_messages if item.get("type") == "command_result"][-1]
                status = [item for item in mqtt.status_messages if item.get("type") == "vehicle_status"][-1]
                self.assertTrue(result["result"]["success"])
                self.assertEqual("in_place_rotate", result["result"]["data"]["manualSteeringMode"])
                self.assertEqual("turning", status["data"]["motionState"])
                self.assertFalse(status["data"]["manualSteeringAllowed"])
            finally:
                controller.manual_steering_controller.close()


if __name__ == "__main__":
    unittest.main()
