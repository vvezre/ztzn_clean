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

        stopped_state = fsm.apply_event("INIT_SUCCEEDED", message="ready")
        self.assertEqual(stopped_state["controlState"], "STOPPED")

        ready_state = fsm.apply_event("START_CHECK_PASSED", message="ready")
        self.assertEqual(ready_state["controlState"], "READY")

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

    def test_lifecycle_fsm_keeps_current_state_after_events(self):
        from robot_fsm import EVENT_TASK_STARTED, RobotLifecycleFSM

        fsm = RobotLifecycleFSM()

        running_state = fsm.apply_event(
            EVENT_TASK_STARTED,
            message="auto drive started",
            payload={"action": "auto_drive"},
        )

        self.assertEqual(fsm.get_state()["controlState"], "RUNNING")
        self.assertEqual(fsm.get_state()["action"], "auto_drive")
        running_state["controlState"] = "BROKEN"
        self.assertEqual(fsm.get_state()["controlState"], "RUNNING")

    def test_legacy_redis_fields_are_derived_from_fsm_state(self):
        from robot_fsm import (
            EVENT_RTK_LOST,
            EVENT_RTK_RECOVERED,
            EVENT_TASK_FINISHED,
            EVENT_TASK_STARTED,
            RobotLifecycleFSM,
            legacy_fields_for_state,
        )

        fsm = RobotLifecycleFSM()

        running_state = fsm.apply_event(
            EVENT_TASK_STARTED,
            payload={"action": "go_to_point"},
        )
        self.assertEqual(
            legacy_fields_for_state(running_state),
            {
                "mission": "working",
                "parking": "0",
                "currentAction": "go_to_point",
            },
        )

        paused_state = fsm.apply_event(EVENT_RTK_LOST)
        self.assertEqual(
            legacy_fields_for_state(paused_state),
            {
                "mission": "working",
                "parking": "0",
                "currentAction": "go_to_point",
            },
        )

        recovered_state = fsm.apply_event(EVENT_RTK_RECOVERED)
        complete_state = fsm.apply_event(EVENT_TASK_FINISHED)
        self.assertEqual(recovered_state["controlState"], "RUNNING")
        self.assertEqual(
            legacy_fields_for_state(complete_state),
            {
                "mission": "complete",
                "parking": "1",
                "currentAction": "idle",
            },
        )

    def test_lifecycle_fsm_rejects_invalid_transition_without_changing_state(self):
        from robot_fsm import EVENT_INIT_FAILED, EVENT_TASK_STARTED, RobotLifecycleFSM

        fsm = RobotLifecycleFSM()
        fsm.apply_event(
            EVENT_INIT_FAILED,
            message="bad config",
            payload={"faultState": "CONFIG_MISSING"},
        )

        rejected = fsm.apply_event(
            EVENT_TASK_STARTED,
            message="must not run from blocked",
            payload={"action": "auto_drive"},
        )

        self.assertFalse(rejected["transitionAccepted"])
        self.assertEqual(rejected["controlState"], "BLOCKED")
        self.assertEqual(rejected["faultState"], "CONFIG_MISSING")
        self.assertEqual(rejected["rejectedEvent"], EVENT_TASK_STARTED)
        self.assertEqual(fsm.get_state()["controlState"], "BLOCKED")
        self.assertEqual(fsm.get_state()["action"], "idle")

    def test_lifecycle_fsm_accepts_normal_runtime_sequence(self):
        from robot_fsm import (
            EVENT_INIT_STARTED,
            EVENT_INIT_SUCCEEDED,
            EVENT_RTK_LOST,
            EVENT_RTK_RECOVERED,
            EVENT_START_CHECK_PASSED,
            EVENT_TASK_FINISHED,
            EVENT_TASK_STARTED,
            RobotLifecycleFSM,
        )

        fsm = RobotLifecycleFSM()

        init_state = fsm.apply_event(EVENT_INIT_STARTED)
        stopped_state = fsm.apply_event(EVENT_INIT_SUCCEEDED)
        ready_state = fsm.apply_event(EVENT_START_CHECK_PASSED)
        running_state = fsm.apply_event(EVENT_TASK_STARTED, payload={"action": "auto_drive"})
        paused_state = fsm.apply_event(EVENT_RTK_LOST)
        recovered_state = fsm.apply_event(EVENT_RTK_RECOVERED)
        complete_state = fsm.apply_event(EVENT_TASK_FINISHED)

        self.assertTrue(init_state["transitionAccepted"])
        self.assertEqual(stopped_state["controlState"], "STOPPED")
        self.assertEqual(ready_state["controlState"], "READY")
        self.assertEqual(running_state["controlState"], "RUNNING")
        self.assertEqual(paused_state["controlState"], "PAUSED")
        self.assertEqual(recovered_state["controlState"], "RUNNING")
        self.assertEqual(complete_state["controlState"], "COMPLETE")

    def test_terminal_events_do_not_keep_stale_runtime_detail_action(self):
        from robot_fsm import (
            EVENT_TASK_FINISHED,
            EVENT_TASK_STARTED,
            EVENT_TASK_STOPPED,
            RobotLifecycleFSM,
        )

        fsm = RobotLifecycleFSM()
        fsm.apply_event(
            EVENT_TASK_STARTED,
            message="multi waypoint running",
            payload={"action": "multi_go_to_point"},
        )

        stopped_state = fsm.apply_event(
            EVENT_TASK_STOPPED,
            message="parking",
            payload={"action": "multi_go_to_point"},
        )

        self.assertEqual(stopped_state["controlState"], "STOPPED")
        self.assertEqual(stopped_state["action"], "parking")

        fsm.apply_event(
            EVENT_TASK_STARTED,
            message="multi waypoint running again",
            payload={"action": "multi_go_to_point"},
        )
        complete_state = fsm.apply_event(
            EVENT_TASK_FINISHED,
            message="finished",
            payload={"action": "multi_go_to_point"},
        )

        self.assertEqual(complete_state["controlState"], "COMPLETE")
        self.assertEqual(complete_state["action"], "idle")

    def test_lifecycle_fsm_supports_non_rtk_pause_stopping_fault_and_disabled(self):
        from robot_fsm import (
            EVENT_DISABLED,
            EVENT_FAULT_OCCURRED,
            EVENT_INIT_SUCCEEDED,
            EVENT_START_CHECK_PASSED,
            EVENT_TASK_PAUSED,
            EVENT_TASK_STARTED,
            EVENT_TASK_STOPPING,
            RobotLifecycleFSM,
        )

        fsm = RobotLifecycleFSM()
        fsm.apply_event(EVENT_INIT_SUCCEEDED)
        fsm.apply_event(EVENT_START_CHECK_PASSED)
        fsm.apply_event(EVENT_TASK_STARTED, payload={"action": "auto_drive"})

        paused_state = fsm.apply_event(EVENT_TASK_PAUSED, payload={"action": "auto_drive"})
        self.assertTrue(paused_state["transitionAccepted"])
        self.assertEqual(paused_state["controlState"], "PAUSED")
        self.assertEqual(paused_state["faultState"], "")

        resumed_state = fsm.apply_event(EVENT_TASK_STARTED, payload={"action": "auto_drive"})
        self.assertEqual(resumed_state["controlState"], "RUNNING")

        stopping_state = fsm.apply_event(EVENT_TASK_STOPPING, payload={"action": "auto_drive"})
        self.assertTrue(stopping_state["transitionAccepted"])
        self.assertEqual(stopping_state["controlState"], "STOPPING")

        fault_state = fsm.apply_event(EVENT_FAULT_OCCURRED, payload={"faultState": "MOTOR_FAULT"})
        self.assertEqual(fault_state["controlState"], "FAULT")
        self.assertEqual(fault_state["healthState"], "ERROR")
        self.assertEqual(fault_state["faultState"], "MOTOR_FAULT")

        disabled_state = fsm.apply_event(EVENT_DISABLED, payload={"faultState": "LOWER_MACHINE_DISABLED"})
        self.assertEqual(disabled_state["controlState"], "DISABLED")
        self.assertEqual(disabled_state["faultState"], "LOWER_MACHINE_DISABLED")

    def test_lifecycle_fsm_honors_explicit_health_and_start_ready_overrides(self):
        from robot_fsm import EVENT_TASK_STARTED, RobotLifecycleFSM

        fsm = RobotLifecycleFSM()
        state = fsm.apply_event(
            EVENT_TASK_STARTED,
            payload={
                "action": "auto_drive",
                "healthState": "WARN",
                "startReady": False,
            },
        )

        self.assertEqual(state["controlState"], "RUNNING")
        self.assertEqual(state["healthState"], "WARN")
        self.assertFalse(state["startReady"])


if __name__ == "__main__":
    unittest.main()
