import unittest

from modeling_task_persistence import (
    ModelingTaskPersistenceError,
    build_named_task,
    is_same_named_task,
    normalize_task_name,
)


class ModelingTaskPersistenceTest(unittest.TestCase):
    def test_normalize_task_name_accepts_chinese_unicode_name(self):
        self.assertEqual(normalize_task_name(u"测试路线"), u"测试路线")

    def test_builds_named_robot_task_from_ready_modeling_plan(self):
        base_config = {
            "goBackLen": 10,
            "originHeading": 180,
            "taskList": [{"id": 99}],
        }
        first_task = {
            "id": 1,
            "startLat": 32.0364,
            "startLon": 118.1234,
            "endLat": 32.0365,
            "endLon": 118.1234,
            "heading": 0,
        }
        current_path = {
            "modelId": "model-1",
            "taskPlan": {
                "status": "ready",
                "tasks": [first_task],
            },
        }

        saved = build_named_task(base_config, current_path, u"厂区路线一")

        self.assertEqual(saved["taskName"], u"厂区路线一")
        self.assertEqual(saved["modelId"], "model-1")
        self.assertEqual(saved["taskList"], [first_task])
        self.assertEqual(saved["startLat"], 32.0364)
        self.assertEqual(saved["startLon"], 118.1234)
        self.assertEqual(saved["originHeading"], 0)
        self.assertEqual(saved["heading"], 0)
        self.assertEqual(saved["goBackLen"], 10)

    def test_rejects_unsafe_task_names_and_unready_paths(self):
        with self.assertRaises(ModelingTaskPersistenceError):
            normalize_task_name(None)
        with self.assertRaises(ModelingTaskPersistenceError):
            normalize_task_name("../bad")
        with self.assertRaises(ModelingTaskPersistenceError):
            normalize_task_name("CON")
        with self.assertRaises(ModelingTaskPersistenceError):
            build_named_task({}, {"taskPlan": {"status": "draft"}}, "route-1")

    def test_detects_idempotent_retry_for_same_modeling_route(self):
        current_path = {
            "modelId": "model-1",
            "taskPlan": {"tasks": [{"id": 1}]},
        }
        task_config = {
            "taskName": u"测试6",
            "modelId": "model-1",
            "taskList": [{"id": 1}],
        }

        self.assertTrue(is_same_named_task(task_config, current_path, u"测试6"))
        self.assertFalse(is_same_named_task(
            task_config,
            {"modelId": "model-2", "taskPlan": {"tasks": [{"id": 1}]}},
            u"测试6",
        ))
        self.assertFalse(is_same_named_task(task_config, current_path, u"测试7"))
        self.assertFalse(is_same_named_task(
            task_config,
            {"modelId": "model-1", "taskPlan": {"tasks": [{"id": 2}]}},
            u"测试6",
        ))


if __name__ == "__main__":
    unittest.main()
