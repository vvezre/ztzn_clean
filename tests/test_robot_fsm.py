import unittest


class RobotEventBusTest(unittest.TestCase):
    def test_event_bus_publishes_to_handlers_and_keeps_recent_events(self):
        from robot_fsm import RobotEventBus

        received = []
        bus = RobotEventBus(max_events=2)
        bus.subscribe("TASK_STARTED", received.append)

        first = bus.publish("INIT_STARTED", source="test", message="init", now=100)
        second = bus.publish("TASK_STARTED", source="test", message="run", payload={"task": "A"}, now=101)
        third = bus.publish("TASK_STARTED", source="test", message="run again", now=102)

        self.assertEqual(first["sequence"], 1)
        self.assertEqual(second["sequence"], 2)
        self.assertEqual(third["sequence"], 3)
        self.assertEqual(len(received), 2)
        self.assertEqual(received[0]["type"], "TASK_STARTED")
        self.assertEqual(received[0]["payload"]["task"], "A")
        self.assertEqual([event["sequence"] for event in bus.recent_events()], [2, 3])


class RobotLifecycleFSMTest(unittest.TestCase):
    def test_lifecycle_fsm_maps_events_to_runtime_state_fields(self):
        from robot_fsm import RobotLifecycleFSM

        fsm = RobotLifecycleFSM()

        init_state = fsm.apply_event("INIT_STARTED", message="initializing")
        self.assertEqual(init_state["controlState"], "INITIALIZING")
        self.assertEqual(init_state["healthState"], "OK")
        self.assertFalse(init_state["startReady"])

        running_state = fsm.apply_event("TASK_STARTED", message="running")
        self.assertEqual(running_state["controlState"], "RUNNING")
        self.assertEqual(running_state["healthState"], "OK")
        self.assertTrue(running_state["startReady"])

        paused_state = fsm.apply_event("RTK_LOST", message="rtk lost")
        self.assertEqual(paused_state["controlState"], "PAUSED")
        self.assertEqual(paused_state["healthState"], "WARN")
        self.assertEqual(paused_state["faultState"], "RTK_FIX_LOST")

        blocked_state = fsm.apply_event(
            "INIT_FAILED",
            message="bad config",
            payload={"faultState": "CONFIG_MISSING"},
        )
        self.assertEqual(blocked_state["controlState"], "BLOCKED")
        self.assertEqual(blocked_state["faultState"], "CONFIG_MISSING")


if __name__ == "__main__":
    unittest.main()
