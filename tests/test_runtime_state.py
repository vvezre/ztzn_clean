import io
import os
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAIN_PATH = os.path.join(ROOT, "main.py")


def read_main_source():
    with io.open(MAIN_PATH, "r", encoding="utf-8") as handle:
        return handle.read()


def function_body(source, function_name):
    marker = "def {}(".format(function_name)
    start = source.index(marker)
    next_def = source.find("\ndef ", start + len(marker))
    if next_def == -1:
        return source[start:]
    return source[start:next_def]


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
            parking=True,
            action="idle",
            start_ready=False,
            now=1234,
        )

        self.assertEqual(state["state"], "COMPLETE")
        self.assertEqual(state["stateLabel"], "任务完成")
        self.assertFalse(state["effects"]["motionAllowed"])
        self.assertFalse(state["effects"]["taskActive"])
        self.assertTrue(state["effects"]["startAllowed"])

    def test_ready_state_is_start_ready_but_not_task_active(self):
        from runtime_state import build_runtime_state_snapshot

        state = build_runtime_state_snapshot(
            control_state="READY",
            health_state="OK",
            mission="complete",
            parking=True,
            action="idle",
            start_ready=True,
            now=1234,
        )

        self.assertEqual(state["state"], "READY")
        self.assertTrue(state["startReady"])
        self.assertFalse(state["effects"]["taskActive"])
        self.assertFalse(state["effects"]["startAllowed"])

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

    def test_main_dispatches_fsm_once_and_mirrors_legacy_redis_fields(self):
        source = read_main_source()
        set_body = function_body(source, "_set_runtime_state")
        dispatch_body = function_body(source, "dispatch_runtime_event")
        publish_body = function_body(source, "_publish_runtime_event")

        self.assertIn("legacy_fields_for_state", source)
        self.assertIn("def dispatch_runtime_event", source)
        self.assertIn("def _mirror_runtime_state_to_redis", source)
        self.assertIn("dispatch_runtime_event", set_body)
        self.assertNotIn("robot_lifecycle_fsm.apply_event", set_body)
        self.assertIn("robot_lifecycle_fsm.apply_event", dispatch_body)
        self.assertIn("_mirror_runtime_state_to_redis", dispatch_body)
        self.assertIn('transitionAccepted") is False', dispatch_body)
        self.assertLess(
            dispatch_body.index('transitionAccepted") is False'),
            dispatch_body.index("_mirror_runtime_state_to_redis"),
        )
        self.assertNotIn("robot_lifecycle_fsm.apply_event", publish_body)

    def test_task_start_threads_pass_action_to_fsm_instead_of_pre_writing_legacy_state(self):
        source = read_main_source()

        auto_body = function_body(source, "autoDriveByRTKThread")
        self.assertIn("'action': 'auto_drive'", auto_body)
        self.assertNotIn('redis_cli.set("mission", "working")', auto_body)
        self.assertNotIn('redis_cli.set("parking", "0")', auto_body)

        go_to_point_body = function_body(source, "goToPointThread")
        self.assertIn("'action': 'go_to_point'", go_to_point_body)
        self.assertNotIn('redis_cli.set("mission", "working")', go_to_point_body)
        self.assertNotIn('redis_cli.set("parking", "0")', go_to_point_body)

        multi_go_to_point_body = function_body(source, "multiGoToPointThread")
        self.assertIn("'action': 'multi_go_to_point'", multi_go_to_point_body)
        self.assertNotIn('redis_cli.set("mission", "working")', multi_go_to_point_body)
        self.assertNotIn('redis_cli.set("parking", "0")', multi_go_to_point_body)

        go_to_points_body = function_body(source, "goToPointsThread")
        self.assertIn("'action': 'multi_go_to_point'", go_to_points_body)
        self.assertNotIn('redis_cli.set("mission", "working")', go_to_points_body)
        self.assertNotIn('redis_cli.set("parking", "0")', go_to_points_body)

    def test_runtime_control_guards_read_fsm_state_not_legacy_redis_flags(self):
        source = read_main_source()

        active_body = function_body(source, "_is_runtime_task_active")
        self.assertIn("robot_lifecycle_fsm.get_state()", active_body)
        self.assertNotIn("redis_cli.get('mission')", active_body)
        self.assertNotIn("redis_cli.get('parking')", active_body)

        rtk_guard_body = function_body(source, "_is_rtk_guard_task_active")
        self.assertIn("robot_lifecycle_fsm.get_state()", rtk_guard_body)
        self.assertNotIn('redis_cli.get("mission")', rtk_guard_body)
        self.assertNotIn("redis_cli.get('parking')", rtk_guard_body)

    def test_parking_paths_use_fsm_dispatch_not_direct_legacy_state_writes(self):
        source = read_main_source()

        parking_body = function_body(source, "parking")
        self.assertIn("_request_runtime_stop('manual_parking'", parking_body)

        do_parking_body = function_body(source, "doParking")
        self.assertIn("update_runtime=True", do_parking_body)
        self.assertIn("if update_runtime:", do_parking_body)
        self.assertIn("_clear_runtime_task_state", do_parking_body)

        for body in (parking_body, do_parking_body):
            self.assertNotIn("set_current_action('parking')", body)
            self.assertNotIn('redis_cli.set("mission"', body)
            self.assertNotIn("redis_cli.set('mission'", body)
            self.assertNotIn('redis_cli.set("parking"', body)
            self.assertNotIn("redis_cli.set('parking'", body)

    def test_task_switching_uses_separate_stop_flags_and_active_token(self):
        source = read_main_source()

        self.assertIn("global_auto_clean_stop = 0", source)
        self.assertIn("global_waypoint_nav_stop = 0", source)
        self.assertIn("global_loop_auto_clean_stop = 0", source)
        self.assertIn("active_runtime_task_token = ''", source)
        self.assertIn("def _begin_runtime_task(", source)
        self.assertIn("def _is_current_runtime_task(", source)
        self.assertIn("def _request_runtime_stop(", source)

        starter_body = function_body(source, "_start_runtime_thread")
        self.assertIn("_begin_runtime_task(action)", starter_body)
        self.assertIn("args=(task_token,) + tuple(args or ())", starter_body)

        auto_body = function_body(source, "autoDriveByRTKThread")
        self.assertIn("task_token", auto_body)
        self.assertIn("_is_current_runtime_task(task_token)", auto_body)
        self.assertIn("global_auto_clean_stop", auto_body)

        waypoints_body = function_body(source, "goToPointsThread")
        self.assertIn("task_token", waypoints_body)
        self.assertIn("_is_current_runtime_task(task_token)", waypoints_body)
        self.assertIn("global_waypoint_nav_stop", waypoints_body)

    def test_waypoint_navigation_clears_auto_resume_before_starting(self):
        source = read_main_source()
        start_body = function_body(source, "goToPointsStart")

        self.assertIn("_request_runtime_stop('prepare_go_to_points'", start_body)
        self.assertIn("clear_auto_task=True", start_body)
        self.assertIn("_set_auto_resume_allowed(False", start_body)

    def test_voltage_listener_does_not_resume_auto_clean_from_tasklist_alone(self):
        source = read_main_source()
        listener_body = function_body(source, "listenerVoltage")

        self.assertIn("_is_auto_resume_allowed()", listener_body)
        self.assertIn("redis_cli.llen('taskList') != 0", listener_body)
        self.assertLess(
            listener_body.index("_is_auto_resume_allowed()"),
            listener_body.index("redis_cli.llen('taskList') != 0"),
        )

    def test_terminal_task_paths_brake_without_overwriting_terminal_lifecycle_state(self):
        source = read_main_source()

        for name in (
            "goToPointThread",
            "multiGoToPointThread",
            "goToPointsThread",
            "returnToPointThread",
            "returnToPointByRTKThread",
            "autoDriveByRTKThread",
            "goOnDoCleanByRTK",
        ):
            body = function_body(source, name)
            self.assertIn("_mark_runtime_complete", body)
            self.assertIn("doParking(update_runtime=False", body)

    def test_business_code_does_not_directly_write_legacy_runtime_state_fields(self):
        source = read_main_source()
        business_source = source[source.index("def set_garage_state"):]

        forbidden_patterns = (
            "redis_cli.set('mission'",
            'redis_cli.set("mission"',
            "redis_cli.set('parking'",
            'redis_cli.set("parking"',
            "redis_cli.set('currentAction'",
            'redis_cli.set("currentAction"',
            "redis_cli.set('controlState'",
            'redis_cli.set("controlState"',
            "redis_cli.set('healthState'",
            'redis_cli.set("healthState"',
            "redis_cli.set('faultState'",
            'redis_cli.set("faultState"',
            "set_current_action(",
        )
        for pattern in forbidden_patterns:
            self.assertNotIn(pattern, business_source, pattern)

    def test_control_code_does_not_read_legacy_runtime_state_fields(self):
        source = read_main_source()
        control_source = source[source.index("def _validate_auto_drive_request_legacy"):]

        forbidden_patterns = (
            "redis_cli.get('mission')",
            'redis_cli.get("mission")',
            "redis_cli.get('parking')",
            'redis_cli.get("parking")',
            "redis_cli.get('currentAction')",
            'redis_cli.get("currentAction")',
        )
        for pattern in forbidden_patterns:
            self.assertNotIn(pattern, control_source, pattern)

    def test_runtime_start_gate_disallows_ready_and_uses_locked_thread_starter(self):
        source = read_main_source()

        can_start_body = function_body(source, "_can_start_runtime_task")
        self.assertIn("return control_state in ('STOPPED', 'COMPLETE')", can_start_body)
        self.assertNotIn("'READY'", can_start_body)

        starter_body = function_body(source, "_start_runtime_thread")
        self.assertIn("with TASK_SWITCH_LOCK:", starter_body)
        self.assertIn("_can_start_runtime_task()", starter_body)
        self.assertIn("_mark_runtime_ready", starter_body)
        self.assertIn("thread.daemon = True", starter_body)
        self.assertIn("thread.start()", starter_body)

    def test_task_entrypoints_use_locked_thread_starter(self):
        source = read_main_source()

        for name in (
            "auto_driving",
            "autoDriveByRTK",
            "goToPoint",
            "multiGoToPoint",
            "returnToPoint",
            "goToPointsStart",
        ):
            body = function_body(source, name)
            self.assertIn("_start_runtime_thread", body, name)

    def test_loop_auto_drive_reuses_active_task_token_for_worker(self):
        source = read_main_source()
        loop_body = function_body(source, "loopAutoDriveThread")
        worker_body = function_body(source, "autoDriveByRTKThread")

        self.assertNotIn("_mark_runtime_running", loop_body)
        self.assertIn("_runtime_task_should_stop(task_token, 'loop_auto_drive')", loop_body)
        self.assertIn("autoDriveByRTKThread(task_token)", loop_body)
        self.assertLess(
            worker_body.index("_mark_runtime_running"),
            worker_body.index("goOutGarage(backLength)"),
        )

    def test_go_to_points_failure_cannot_be_reported_as_complete(self):
        source = read_main_source()
        body = function_body(source, "goToPointsThread")

        self.assertIn("failed = False", body)
        self.assertIn("failed = True", body)
        self.assertIn("_mark_runtime_blocked", body)
        self.assertIn("'WAYPOINT_INTERRUPTED'", body)
        self.assertIn("not failed", body)
        self.assertLess(
            body.index("not failed"),
            body.index("_mark_runtime_complete"),
        )

    def test_main_health_state_prefers_fsm_health_over_fault_fallback(self):
        source = read_main_source()
        body = function_body(source, "_derive_health_state")

        self.assertIn("if health_state:", body)
        self.assertIn("if fault_state", body)
        self.assertLess(body.index("health_state"), body.index("fault_state"))
        self.assertLess(body.index("if health_state:"), body.index("if fault_state"))


if __name__ == "__main__":
    unittest.main()
