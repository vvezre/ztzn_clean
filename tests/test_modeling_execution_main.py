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
        thread_body = function_body(source, "_modelingTaskThread")
        self.assertIn("pointToPointByRTK", runner_body)
        self.assertIn("turn(", runner_body)
        self.assertIn("_mark_runtime_running", thread_body)
        self.assertIn("_mark_runtime_complete", thread_body)
        self.assertIn("_mark_runtime_blocked", thread_body)
        self.assertIn("execute_modeling_plan", thread_body)

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


if __name__ == "__main__":
    unittest.main()
