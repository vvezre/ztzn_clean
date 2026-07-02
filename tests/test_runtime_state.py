import io
import os
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAIN_PATH = os.path.join(ROOT, "main.py")


def read_main_source():
    with io.open(MAIN_PATH, "r", encoding="utf-8") as handle:
        return handle.read()


class RuntimeStateModelTest(unittest.TestCase):
    def test_initializing_state_blocks_start_until_setup_finishes(self):
        from runtime_state import build_runtime_state_snapshot

        state = build_runtime_state_snapshot(
            control_state="INITIALIZING",
            health_state="OK",
            fault_state="",
            mission="",
            parking=True,
            action="idle",
            start_ready=False,
            message="initializing runtime",
            detail={"initializing": True, "initializationPhase": "startup"},
            now=1234,
        )

        self.assertEqual(state["state"], "INITIALIZING")
        self.assertEqual(state["stateLabel"], "初始化中")
        self.assertFalse(state["effects"]["motionAllowed"])
        self.assertFalse(state["effects"]["taskActive"])
        self.assertFalse(state["effects"]["startAllowed"])
        self.assertFalse(state["effects"]["requiresAttention"])
        self.assertTrue(state["effects"]["initializing"])
        self.assertIn("初始化未完成", state["effectLabels"])

    def test_running_state_describes_lifecycle_action_and_effects(self):
        from runtime_state import build_runtime_state_snapshot

        state = build_runtime_state_snapshot(
            control_state="RUNNING",
            health_state="OK",
            fault_state="",
            mission="working",
            parking=False,
            action="auto_drive",
            task_name="002",
            task_index=3,
            start_ready=True,
            message="auto clean running",
            detail={"rtkFixState": "FIXED"},
            now=1234,
        )

        self.assertEqual(state["state"], "RUNNING")
        self.assertEqual(state["stateLabel"], "运行中")
        self.assertEqual(state["action"], "auto_drive")
        self.assertEqual(state["actionLabel"], "自动清扫")
        self.assertEqual(state["health"], "OK")
        self.assertEqual(state["healthLabel"], "正常")
        self.assertEqual(state["fault"], "")
        self.assertEqual(state["mission"], "working")
        self.assertFalse(state["parking"])
        self.assertEqual(state["task"]["name"], "002")
        self.assertEqual(state["task"]["index"], 3)
        self.assertTrue(state["startReady"])
        self.assertEqual(state["message"], "auto clean running")
        self.assertEqual(state["rtk"]["state"], "FIXED")
        self.assertTrue(state["effects"]["motionAllowed"])
        self.assertTrue(state["effects"]["taskActive"])
        self.assertFalse(state["effects"]["requiresAttention"])
        self.assertIn("允许运动", state["effectLabels"])
        self.assertIn("任务执行中", state["effectLabels"])

    def test_blocked_state_disables_motion_and_keeps_fault_reason(self):
        from runtime_state import build_runtime_state_snapshot

        state = build_runtime_state_snapshot(
            control_state="BLOCKED",
            health_state="WARN",
            fault_state="CURRENT_TASK_NOT_SET",
            mission="complete",
            parking=True,
            action="idle",
            start_ready=False,
            message="task is missing",
            now=1234,
        )

        self.assertEqual(state["state"], "BLOCKED")
        self.assertEqual(state["stateLabel"], "阻塞待处理")
        self.assertEqual(state["health"], "WARN")
        self.assertEqual(state["healthLabel"], "告警")
        self.assertEqual(state["fault"], "CURRENT_TASK_NOT_SET")
        self.assertEqual(state["faultLabel"], "当前任务未设置")
        self.assertFalse(state["effects"]["motionAllowed"])
        self.assertFalse(state["effects"]["startAllowed"])
        self.assertTrue(state["effects"]["requiresAttention"])
        self.assertIn("需要人工处理", state["effectLabels"])

    def test_complete_control_state_is_kept_as_lifecycle_state(self):
        from runtime_state import build_runtime_state_snapshot

        state = build_runtime_state_snapshot(
            control_state="COMPLETE",
            health_state="OK",
            mission="complete",
            parking=False,
            action="idle",
            start_ready=False,
            now=1234,
        )

        self.assertEqual(state["state"], "COMPLETE")
        self.assertEqual(state["stateLabel"], "任务完成")
        self.assertFalse(state["effects"]["motionAllowed"])
        self.assertFalse(state["effects"]["taskActive"])
        self.assertTrue(state["effects"]["startAllowed"])

    def test_main_runtime_state_writer_keeps_legacy_fields_and_publishes_snapshot(self):
        source = read_main_source()

        self.assertIn("build_runtime_state_snapshot", source)
        self.assertIn("RUNTIME_STATE_KEY", source)
        self.assertIn("_set_redis_value(RUNTIME_STATE_KEY", source)

    def test_main_has_explicit_initialization_runtime_transitions(self):
        source = read_main_source()

        self.assertIn("def _mark_runtime_initializing", source)
        self.assertIn("control_state='INITIALIZING'", source)
        self.assertIn("def _mark_runtime_initialized", source)
        self.assertIn("_mark_runtime_initializing(", source)
        self.assertIn("_mark_runtime_initialized(", source)
        self.assertIn("'INIT_FAILED'", source)

    def test_global_status_is_log_compatible_not_control_source(self):
        source = read_main_source()

        self.assertIn('logger.info("status={},voltage={}".format(global_status,voltage))', source)
        self.assertNotIn("if global_status == 'working'", source)
        self.assertNotIn("if global_status != 'working'", source)
        self.assertNotIn("if global_status != 'goCharging'", source)
        self.assertIn("def _is_runtime_task_active", source)
        self.assertIn("def _is_runtime_returning_to_charge", source)

    def test_main_runtime_state_is_connected_to_event_bus(self):
        source = read_main_source()

        self.assertIn("from robot_fsm import", source)
        self.assertIn("RUNTIME_EVENT_LOG_KEY", source)
        self.assertIn("robot_event_bus = RobotEventBus", source)
        self.assertIn("robot_lifecycle_fsm = RobotLifecycleFSM", source)
        self.assertIn("def _publish_runtime_event", source)
        self.assertIn("runtime_event_type_for_control_state", source)
        self.assertIn("redis_cli.lpush(RUNTIME_EVENT_LOG_KEY", source)
        self.assertIn("event_type=None", source)


if __name__ == "__main__":
    unittest.main()
