import unittest


class ModelingExecutionTest(unittest.TestCase):
    def test_build_execution_plan_requires_ready_task_plan_and_speed(self):
        from modeling_execution import ModelingExecutionError, build_execution_plan

        with self.assertRaises(ModelingExecutionError):
            build_execution_plan("m1", {"status": "empty"}, speed=300)

        with self.assertRaises(ModelingExecutionError):
            build_execution_plan("m1", {"status": "ready", "tasks": []}, speed=300)

        with self.assertRaises(ModelingExecutionError):
            build_execution_plan("m1", {"status": "ready", "tasks": [{"id": 1}]}, speed=None)

    def test_execute_modeling_plan_runs_segments_and_reports_progress(self):
        from modeling_execution import build_execution_plan, execute_modeling_plan

        task_plan = {
            "status": "ready",
            "summary": {"taskCount": 2},
            "tasks": [
                {
                    "id": 1,
                    "mode": 1,
                    "startLat": 32.0,
                    "startLon": 118.0,
                    "endLat": 32.0,
                    "endLon": 118.0001,
                    "heading": 90,
                    "length": 1000,
                },
                {
                    "id": 2,
                    "mode": 2,
                    "startLat": 32.0,
                    "startLon": 118.0001,
                    "endLat": 32.0001,
                    "endLon": 118.0001,
                    "heading": 0,
                    "length": 1000,
                },
            ],
        }
        plan = build_execution_plan("model-a", task_plan, speed=320, now=2000)
        calls = []
        progress = []

        result = execute_modeling_plan(
            plan,
            run_segment=lambda segment: calls.append(segment) or True,
            update_progress=lambda state: progress.append(dict(state)),
            should_stop=lambda: False,
            now=lambda: 2001,
        )

        self.assertEqual(plan["action"], "modeling_task")
        self.assertEqual(plan["speed"], 320)
        self.assertEqual(result["status"], "complete")
        self.assertEqual(result["completedCount"], 2)
        self.assertEqual([item["id"] for item in calls], [1, 2])
        self.assertEqual(calls[0]["speed"], 320)
        self.assertEqual(progress[-1]["status"], "complete")
        self.assertEqual(progress[-1]["currentIndex"], 2)

    def test_execute_modeling_plan_blocks_when_segment_fails(self):
        from modeling_execution import build_execution_plan, execute_modeling_plan

        plan = build_execution_plan("model-a", {
            "status": "ready",
            "tasks": [{
                "id": 1,
                "mode": 1,
                "startLat": 32.0,
                "startLon": 118.0,
                "endLat": 32.0,
                "endLon": 118.0001,
                "heading": 90,
                "length": 1000,
            }],
        }, speed=300)

        result = execute_modeling_plan(
            plan,
            run_segment=lambda segment: False,
            update_progress=lambda state: None,
            should_stop=lambda: False,
        )

        self.assertEqual(result["status"], "blocked")
        self.assertEqual(result["code"], "MODELING_SEGMENT_FAILED")

    def test_execute_modeling_plan_runs_soft_points_as_one_continuous_run(self):
        from continuous_route import CONTINUATION_KEY
        from modeling_execution import build_execution_plan, execute_modeling_plan

        tasks = []
        for index in range(3):
            tasks.append({
                "id": index + 1,
                "mode": 1,
                "startLat": 32.0,
                "startLon": 118.0 + index * 0.0001,
                "endLat": 32.0,
                "endLon": 118.0001 + index * 0.0001,
                "heading": 90,
                "length": 1000,
                "continuousPathId": "g1:boundary:1",
                "turnAtStart": index == 0,
                "stopAtEnd": index == 2,
            })
        plan = build_execution_plan("model-a", {"status": "ready", "tasks": tasks}, speed=300)
        calls = []

        result = execute_modeling_plan(
            plan,
            run_segment=lambda segment: calls.append(segment) or True,
        )

        self.assertEqual(result["status"], "complete")
        self.assertEqual(result["completedCount"], 3)
        self.assertEqual(len(calls), 1)
        self.assertEqual([item["id"] for item in calls[0][CONTINUATION_KEY]], [2, 3])
        self.assertTrue(calls[0]["stopAtEnd"])

    def test_execute_modeling_plan_reports_stopped_when_segment_is_interrupted(self):
        from modeling_execution import build_execution_plan, execute_modeling_plan

        plan = build_execution_plan("model-a", {
            "status": "ready",
            "tasks": [{
                "id": 1,
                "mode": 1,
                "startLat": 32.0,
                "startLon": 118.0,
                "endLat": 32.0,
                "endLon": 118.0001,
                "heading": 90,
                "length": 1000,
            }],
        }, speed=300)
        stop_state = {"requested": False}
        progress = []

        def run_segment(segment):
            stop_state["requested"] = True
            return False

        result = execute_modeling_plan(
            plan,
            run_segment=run_segment,
            update_progress=lambda state: progress.append(dict(state)),
            should_stop=lambda: stop_state["requested"],
            now=lambda: 3000,
        )

        self.assertEqual(result["status"], "stopped")
        self.assertEqual(result["code"], "MODELING_TASK_STOPPED")
        self.assertEqual(result["completedCount"], 0)
        self.assertEqual(progress[-1]["status"], "stopped")


if __name__ == "__main__":
    unittest.main()
