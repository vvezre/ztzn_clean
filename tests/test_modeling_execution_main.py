import re
import unittest


def read_main():
    with open("main.py", "r", encoding="utf-8") as handle:
        return handle.read()


def function_body(source, name):
    match = re.search(r"^def {}\([^\n]*\):\n".format(name), source, re.M)
    if not match:
        return ""
    start = match.end()
    next_match = re.search(r"^def [A-Za-z_][A-Za-z0-9_]*\([^\n]*\):\n", source[start:], re.M)
    end = start + next_match.start() if next_match else len(source)
    return source[start:end]


class ModelingExecutionMainTest(unittest.TestCase):
    def test_main_registers_modeling_task_execution_with_fsm_thread_starter(self):
        source = read_main()

        self.assertIn("from modeling_execution import", source)
        self.assertIn("task_execution_starter=_start_modeling_task_runtime", source)
        self.assertIn("task_progress_reader=_read_modeling_task_progress", source)
        self.assertIn("task_stop_handler=_stop_modeling_task_runtime", source)
        starter_body = function_body(source, "_start_modeling_task_runtime")
        self.assertIn("build_execution_plan", starter_body)
        self.assertIn("_start_runtime_thread", starter_body)
        self.assertIn("'modeling_task'", starter_body)
        self.assertIn("_validate_modeling_task_start", starter_body)

    def test_modeling_task_thread_reuses_rtk_segment_execution(self):
        source = read_main()

        runner_body = function_body(source, "_run_modeling_task_segment")
        shared_runner_body = function_body(source, "_run_task_segment_by_point_navigation")
        thread_body = function_body(source, "_modelingTaskThread")
        self.assertIn("_run_task_segment_by_point_navigation", runner_body)
        self.assertIn("pointToPointByRTKAutoHeading", shared_runner_body)
        self.assertIn("polyline_points=polyline_points", shared_runner_body)
        self.assertIn("global_cur_rtk_lat", shared_runner_body)
        self.assertIn("global_cur_rtk_lon", shared_runner_body)
        self.assertIn("_mark_runtime_running", thread_body)
        self.assertIn("_mark_runtime_complete", thread_body)
        self.assertIn("_mark_runtime_blocked", thread_body)
        self.assertIn("execute_modeling_plan", thread_body)

    def test_auto_drive_groups_soft_points_before_starting_navigation(self):
        source = read_main()
        body = function_body(source, "autoDriveByRTKThread")

        self.assertIn("collect_continuous_run(taskList, index)", body)
        self.assertIn("attach_continuations(continuous_run)", body)
        self.assertIn("for completed_offset in range(run_count)", body)

    def test_rtk_observer_tracks_entire_polyline_without_soft_point_restart(self):
        source = read_main()
        body = function_body(source, "observer_go_correct")

        self.assertIn("compute_polyline_guidance", body)
        self.assertIn("polylineProgressM", body)
        self.assertIn("polyline deviation correction", body)
        self.assertIn("polyline deviation unsafe", body)
        self.assertNotIn("_advance_continuous_target", source)
        self.assertNotIn("continuousTargets", source)

    def test_modeling_task_runtime_exposes_progress_stop_and_preflight(self):
        source = read_main()

        progress_body = function_body(source, "_read_modeling_task_progress")
        stop_body = function_body(source, "_stop_modeling_task_runtime")
        preflight_body = function_body(source, "_validate_modeling_task_start")

        self.assertIn("modelingTaskProgress", progress_body)
        self.assertIn("_decode_redis_value", progress_body)
        self.assertIn("_set_modeling_task_progress", stop_body)
        self.assertIn("doParking(update_runtime=True", stop_body)
        self.assertIn("_can_start_runtime_task", preflight_body)
        self.assertIn("_build_rtk_runtime_detail", preflight_body)
        self.assertIn("rtkFixAvailable", preflight_body)
        self.assertIn("validate_route_start", preflight_body)
        self.assertIn("TASK_ORIGIN_TOLERANCE_METERS", preflight_body)


if __name__ == "__main__":
    unittest.main()
