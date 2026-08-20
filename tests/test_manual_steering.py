# coding=utf-8
import tempfile
import time
import unittest

from manual_steering import ManualSteeringController, ManualSteeringError
from motion_state import derive_motion_state


class ManualSteeringControllerTest(unittest.TestCase):
    def _controller(self, motion="forward", timeout=1.5):
        state = {"motion": motion, "now": 10.0}
        events = []
        controller = ManualSteeringController(
            motion_provider=lambda: state["motion"],
            apply_trim=lambda value, mode: events.append(("trim", value, mode)),
            start_rotation=lambda direction: events.append(("rotate", direction)),
            stop_rotation=lambda: events.append(("stop_rotate",)),
            now=lambda: state["now"],
            keepalive_timeout=timeout,
        )
        return controller, state, events

    def test_forward_tap_and_hold_increase_by_fixed_steps(self):
        controller, state, events = self._controller("forward")
        try:
            first = controller.handle({"action": "tap", "direction": "left", "controlId": "tap-1", "sequence": 1})
            self.assertEqual(-50, first["data"]["manualCorrectionValue"])
            controller.handle({"action": "hold_start", "direction": "left", "controlId": "hold-1", "sequence": 1})
            result = controller.handle({"action": "keepalive", "direction": "left", "controlId": "hold-1", "sequence": 2})
            self.assertEqual(-150, result["data"]["manualCorrectionValue"])
            self.assertEqual(("trim", -150, "forward"), events[-1])
        finally:
            controller.close()

    def test_reverse_reverses_raw_correction_sign(self):
        controller, state, events = self._controller("reverse")
        try:
            result = controller.handle({"action": "tap", "direction": "left", "controlId": "tap-r", "sequence": 1})
            self.assertEqual(50, result["data"]["manualCorrectionValue"])
            self.assertEqual(("trim", 50, "reverse"), events[-1])
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

        with tempfile.TemporaryDirectory() as directory:
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
                self.assertEqual(50, response["data"]["manualCorrectionValue"])
                status = controller.status_snapshot()
                self.assertEqual("forward", status["motionState"])
                self.assertEqual(50, status["manualCorrectionValue"])

                kept = handler.handle({
                    "command": "manual_steering",
                    "params": {"action": "keepalive", "direction": "right", "controlId": "ui-1", "sequence": 2},
                })
                self.assertTrue(kept["success"])
                self.assertEqual(100, kept["data"]["manualCorrectionValue"])
                released = handler.handle({
                    "command": "manual_steering",
                    "params": {"action": "hold_stop", "direction": "right", "controlId": "ui-1", "sequence": 3},
                })
                self.assertTrue(released["success"])
                self.assertEqual("none", released["data"]["manualSteeringMode"])

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

        with tempfile.TemporaryDirectory() as directory:
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
