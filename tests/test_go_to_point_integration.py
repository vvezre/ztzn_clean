import io
import os
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def read_main():
    with io.open(os.path.join(ROOT, "main.py"), "r", encoding="utf-8", errors="ignore") as fh:
        return fh.read()


def function_body(source, function_name):
    marker = "def {}(".format(function_name)
    start = source.index(marker)
    next_def = source.find("\ndef ", start + len(marker))
    if next_def == -1:
        return source[start:]
    return source[start:next_def]


class GoToPointIntegrationTest(unittest.TestCase):
    def test_rtk_turn_uses_left_right_commands_and_ignores_lower_finish(self):
        body = read_main()
        start = body.index("def _turn_to_heading_by_rtk(")
        end = body.index("\ndef turn(", start)
        helper = body[start:end]

        self.assertIn("choose_turn_direction", helper)
        self.assertIn("TURN_LEFT_PROTOCOL_VALUE", helper)
        self.assertIn("TURN_RIGHT_PROTOCOL_VALUE", helper)
        self.assertIn("_get_current_rtk_heading()", helper)
        self.assertIn("sendBraking()", helper)
        self.assertNotIn("getRotateArrive()", helper)

    def test_main_exposes_go_to_point_route(self):
        body = read_main()

        self.assertIn("from go_to_point import", body)
        self.assertIn('@app.route("/vehicle/goToPoint"', body)
        self.assertIn("def goToPoint():", body)
        self.assertIn("def goToPointThread(task_token=None, plan=None):", body)
        self.assertIn("build_go_to_point_plan(", body)
        self.assertIn("def pointToPointByRTKAutoHeading(", body)
        self.assertIn("util.get_distance_angle(current_start_lat, current_start_lon, endLat, endLon)", body)
        self.assertIn("return pointToPointByRTK(", body)
        self.assertIn("continuous_segments=continuous_segments", body)
        self.assertIn("pointToPointByRTKAutoHeading(", body)
        self.assertNotIn('data["heading"],\n            data["speed"],', body)
        self.assertIn("'action': 'go_to_point'", body)

    def test_go_to_point_route_rejects_when_runtime_cannot_start_task(self):
        body = read_main()
        start = body.index("def goToPoint():")
        end = body.index("\ndef _parse_waypoints_from_payload", start)
        route_body = body[start:end]

        self.assertIn("_start_runtime_thread", route_body)
        self.assertIn("RUNTIME_NOT_STARTABLE", body)
        self.assertNotIn("threading.Thread(target=goToPointThread", route_body)

    def test_auto_heading_turns_before_driving(self):
        body = read_main()

        self.assertIn("def pointToPointByRTKAutoHeading(", body)
        self.assertIn("turn_result = turn(", body)
        self.assertIn("heading * 10,", body)
        self.assertIn("source='point_to_point_auto_heading'", body)
        self.assertIn("if turn_result != 1:", body)
        self.assertIn("return pointToPointByRTK(", body)
        self.assertIn("continuous_segments=continuous_segments", body)

    def test_modeling_boundary_soft_turn_reuses_point_to_point_without_stationary_turn(self):
        body = read_main()
        segment_runner = function_body(body, "_run_task_segment_by_point_navigation")

        self.assertIn("segment.get('turnAtStart') is False", segment_runner)
        self.assertIn("go to continuous boundary point", segment_runner)
        self.assertIn("return pointToPointByRTK(", segment_runner)
        self.assertIn("return pointToPointByRTKAutoHeading(", segment_runner)

    def test_multi_waypoint_start_accepts_loop_options(self):
        body = read_main()

        self.assertIn("normalize_loop_options(", body)
        self.assertIn("iter_closed_loop_targets(", body)
        self.assertIn("'waypointLoopEnabled'", body)
        self.assertIn("'waypointLoopMode'", body)
        self.assertIn("'waypointLoopTarget'", body)
        self.assertIn("'waypointLoopCurrent'", body)

    def test_multi_waypoint_progress_exposes_loop_status(self):
        body = read_main()

        self.assertIn('"loopMode"', body)
        self.assertIn('"currentLoop"', body)
        self.assertIn('"targetLoop"', body)
        self.assertIn('"savedTotal"', body)
        self.assertIn("saved_total = len(_load_go_to_points())", body)
        self.assertIn("_coerce_bool(redis_cli.get(WAYPOINT_LOOP_ENABLED_KEY), False)", body)
        self.assertIn(
            "running = _is_runtime_task_active() and current_action == 'multi_go_to_point'",
            body,
        )

    def test_main_exposes_go_to_points_management_routes(self):
        body = read_main()

        for route in (
            '@app.route("/vehicle/goToPoints", methods=[\'GET\'])',
            '@app.route("/vehicle/goToPoints", methods=[\'POST\'])',
            '@app.route("/vehicle/goToPoints/add", methods=[\'POST\'])',
            '@app.route("/vehicle/goToPoints/remove", methods=[\'POST\'])',
            '@app.route("/vehicle/goToPoints/reorder", methods=[\'POST\'])',
            '@app.route("/vehicle/goToPoints/clear", methods=[\'POST\'])',
            '@app.route("/vehicle/goToPoints/start", methods=[\'POST\'])',
            '@app.route("/vehicle/goToPoints/stop", methods=[\'POST\'])',
            '@app.route("/vehicle/goToPoints/progress", methods=[\'GET\'])',
        ):
            self.assertIn(route, body)

        self.assertIn("def goToPointsThread(task_token=None):", body)
        self.assertIn("redis_cli.lrange('waypoints'", body)
        self.assertIn("redis_cli.set('waypointTotal'", body)
        self.assertIn("'action': 'multi_go_to_point'", body)
        self.assertIn("iter_closed_loop_targets(waypoints", body)

    def test_runtime_task_clear_removes_stale_waypoint_loop_state(self):
        body = read_main()
        clear_body = function_body(body, "_clear_runtime_task_state")
        delete_body = function_body(body, "delTaskList")
        thread_body = function_body(body, "goToPointsThread")
        stop_body = function_body(body, "goToPointsStop")

        self.assertIn("def _clear_runtime_task_state(", body)
        self.assertIn("redis_cli.delete('waypointLoopProgress')", clear_body)
        self.assertIn("_set_redis_value('runtimeDetail', {})", clear_body)
        self.assertIn("redis_cli.set('moveJudge', 'false')", clear_body)
        self.assertIn("redis_cli.set('reverse', 'false')", clear_body)
        self.assertIn("redis_cli.set('enterGarage', 'false')", clear_body)
        self.assertIn("redis_cli.set('exitGarage', 'false')", clear_body)
        self.assertIn("redis_cli.set(WAYPOINT_LOOP_ENABLED_KEY, '0')", clear_body)
        self.assertIn("redis_cli.set(WAYPOINT_LOOP_MODE_KEY, 'count')", clear_body)
        self.assertIn("redis_cli.set(WAYPOINT_LOOP_TARGET_KEY, 0)", clear_body)
        self.assertIn("redis_cli.set(WAYPOINT_LOOP_CURRENT_KEY, 0)", clear_body)
        self.assertIn("_clear_runtime_task_state('delTaskList'", delete_body)
        self.assertIn("_clear_runtime_task_state('goToPointsThread.finally'", thread_body)
        self.assertIn("_clear_runtime_task_state('goToPointsStop'", stop_body)

    def test_lower_machine_default_port_is_tty_acm0(self):
        body = read_main()
        port_body = function_body(body, "_resolve_lower_machine_port")

        self.assertIn('preferred = env_port or "/dev/ttyACM0"', port_body)
        self.assertIn('xwj_port = "/dev/ttyACM0"', body)
        self.assertNotIn('preferred = env_port or "/dev/ttyTHS1"', port_body)


if __name__ == "__main__":
    unittest.main()
