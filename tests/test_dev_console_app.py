import json
import os
import tempfile
import unittest
from unittest import mock


class DevConsoleAppTest(unittest.TestCase):
    def test_index_serves_vue_page(self):
        from dev_console.app import create_app
        from tests.test_dev_console_correction_state import FakeRedis

        app = create_app(redis_client=FakeRedis())
        client = app.test_client()

        response = client.get("/")

        self.assertEqual(response.status_code, 200)
        self.assertIn(b'id="app"', response.data)
        self.assertIn(b"Vue", response.data)
        self.assertIn("运行总览".encode("utf-8"), response.data)
        self.assertIn("路径纠偏实时面板".encode("utf-8"), response.data)
        self.assertIn("任务路径查看器".encode("utf-8"), response.data)
        self.assertIn("Redis 状态查看".encode("utf-8"), response.data)
        self.assertIn("日志过滤查看".encode("utf-8"), response.data)
        self.assertIn("生命周期状态".encode("utf-8"), response.data)
        self.assertIn("参数来源".encode("utf-8"), response.data)
        self.assertIn("无".encode("utf-8"), response.data)
        self.assertIn("规划路径".encode("utf-8"), response.data)
        self.assertIn("当前段".encode("utf-8"), response.data)
        self.assertIn("实际轨迹".encode("utf-8"), response.data)
        self.assertIn("当前位置".encode("utf-8"), response.data)
        self.assertIn("航向误差".encode("utf-8"), response.data)
        self.assertIn("横向偏差距离".encode("utf-8"), response.data)
        self.assertIn("纠偏转速".encode("utf-8"), response.data)
        self.assertIn("到目标点距离".encode("utf-8"), response.data)
        self.assertIn("沿路径剩余距离".encode("utf-8"), response.data)
        self.assertIn("边缘到目标距离".encode("utf-8"), response.data)
        self.assertIn("当前车头方向".encode("utf-8"), response.data)
        self.assertIn("目标路径方向".encode("utf-8"), response.data)
        self.assertIn("航向误差增益".encode("utf-8"), response.data)
        self.assertIn("横向偏差增益".encode("utf-8"), response.data)

    def test_correction_state_endpoint_returns_json(self):
        from dev_console.app import create_app
        from tests.test_dev_console_correction_state import FakeRedis

        redis_client = FakeRedis()
        redis_client.hset("currentLocation", "lat", 0.0)
        redis_client.hset("currentLocation", "lon", 0.0)
        redis_client.hset("currentLocation", "heading", 90.0)
        app = create_app(redis_client=redis_client)
        client = app.test_client()

        response = client.get("/dev/correction/state")

        self.assertEqual(response.status_code, 200)
        payload = response.get_json()
        self.assertIn("vehicle", payload)
        self.assertIn("correction", payload)
        self.assertIn("trace", payload)

    def test_overview_endpoint_returns_runtime_summary(self):
        from dev_console.app import create_app
        from tests.test_dev_console_correction_state import FakeRedis

        redis_client = FakeRedis()
        redis_client.set("runtimeState", json.dumps({
            "state": "RUNNING",
            "action": "auto_drive",
            "health": "OK",
            "fault": "",
            "effects": {"motionAllowed": True},
        }))
        redis_client.set("mission", "working")
        redis_client.set("parking", "0")
        redis_client.set("currentAction", "auto_drive")
        redis_client.set("batteryPercent", "76")
        redis_client.set("garageState", "outside")
        redis_client.set("forwardSpeed", "350")
        redis_client.set("brushSpeed", "30")
        redis_client.set("runtimeDetail", json.dumps({"rtkFixAvailable": True}))
        app = create_app(redis_client=redis_client)
        client = app.test_client()

        response = client.get("/dev/overview/state")

        self.assertEqual(response.status_code, 200)
        payload = response.get_json()
        self.assertEqual(payload["mission"], "working")
        self.assertFalse(payload["parking"])
        self.assertEqual(payload["currentAction"], "auto_drive")
        self.assertEqual(payload["runtimeState"]["state"], "RUNNING")
        self.assertTrue(payload["runtimeState"]["effects"]["motionAllowed"])
        self.assertEqual(payload["batteryPercent"], 76.0)
        self.assertTrue(payload["rtkFixed"])

    def test_overview_does_not_replace_missing_runtime_values_with_defaults(self):
        from dev_console.app import create_app
        from tests.test_dev_console_correction_state import FakeRedis

        app = create_app(redis_client=FakeRedis())
        client = app.test_client()

        response = client.get("/dev/overview/state")

        self.assertEqual(response.status_code, 200)
        payload = response.get_json()
        self.assertIsNone(payload["forwardSpeed"])
        self.assertIsNone(payload["brushSpeed"])
        self.assertIsNone(payload["curTaskIndex"])

    def test_dev_console_redis_client_uses_short_timeouts(self):
        from local_redis import create_redis_client

        client = create_redis_client()
        kwargs = client.connection_pool.connection_kwargs

        self.assertLessEqual(kwargs.get("socket_connect_timeout"), 0.5)
        self.assertLessEqual(kwargs.get("socket_timeout"), 0.5)

    def test_dev_console_can_fallback_to_local_redis_when_redis_is_unreachable(self):
        from local_redis import LocalRedis, create_redis_client

        with mock.patch("local_redis._is_tcp_reachable", return_value=False):
            client = create_redis_client(fallback_if_unavailable=True)

        self.assertIsInstance(client, LocalRedis)

    def test_task_path_endpoint_returns_redis_tasks(self):
        from dev_console.app import create_app
        from tests.test_dev_console_correction_state import FakeRedis

        redis_client = FakeRedis()
        redis_client.set("currentTaskName", "demo")
        redis_client.set("curTaskIndex", "1")
        redis_client.rpush("taskList", json.dumps(
            {"id": 1, "startX": 0, "startY": 0, "endX": 0, "endY": 100, "heading": 90, "angle": 0, "mode": 1}
        ))
        redis_client.rpush("taskList", json.dumps(
            {"id": 2, "startX": 0, "startY": 100, "endX": 50, "endY": 100, "heading": 180, "angle": 90, "mode": 1}
        ))
        app = create_app(redis_client=redis_client)
        client = app.test_client()

        response = client.get("/dev/task-path")

        self.assertEqual(response.status_code, 200)
        payload = response.get_json()
        self.assertEqual(payload["taskName"], "demo")
        self.assertEqual(payload["source"], "redis")
        self.assertEqual(payload["currentIndex"], 1)
        self.assertEqual(len(payload["tasks"]), 2)
        self.assertTrue(payload["tasks"][1]["current"])

    def test_task_path_endpoint_does_not_return_config_tasks_without_redis_task_list(self):
        from dev_console.app import create_app
        from tests.test_dev_console_correction_state import FakeRedis

        config = {
            "taskName": "demo",
            "taskList": [
                {"id": 1, "startX": 0, "startY": 0, "endX": 0, "endY": 100, "heading": 90},
            ],
        }
        fd, path = tempfile.mkstemp(suffix=".json")
        os.close(fd)
        try:
            with open(path, "w") as fp:
                json.dump(config, fp)
            app = create_app(redis_client=FakeRedis(), config_path=path)
            client = app.test_client()

            response = client.get("/dev/task-path")
        finally:
            os.remove(path)

        self.assertEqual(response.status_code, 200)
        payload = response.get_json()
        self.assertEqual(payload["source"], "none")
        self.assertIsNone(payload["currentIndex"])
        self.assertEqual(payload["tasks"], [])

    def test_redis_state_endpoint_groups_known_keys(self):
        from dev_console.app import create_app
        from tests.test_dev_console_correction_state import FakeRedis

        redis_client = FakeRedis()
        redis_client.set("mission", "working")
        redis_client.set("parking", "0")
        redis_client.rpush("runtimeEvents", json.dumps({"type": "TASK_STARTED"}))
        redis_client.hset("taskParams", "originHeading", "183")
        redis_client.hset("currentLocation", "lat", "32.1")
        redis_client.hset("currentLocation", "lon", "118.1")
        app = create_app(redis_client=redis_client)
        client = app.test_client()

        response = client.get("/dev/redis/state")

        self.assertEqual(response.status_code, 200)
        payload = response.get_json()
        self.assertIn("vehicle", payload["groups"])
        self.assertEqual(payload["groups"]["vehicle"]["mission"], "working")
        self.assertEqual(payload["groups"]["vehicle"]["runtimeEvents"][0], json.dumps({"type": "TASK_STARTED"}))
        self.assertEqual(payload["groups"]["rtk"]["currentLocation"]["lat"], "32.1")
        self.assertEqual(payload["groups"]["task"]["taskParams"]["originHeading"], "183")

    def test_logs_endpoint_filters_by_query_and_limit(self):
        from dev_console.app import create_app
        from tests.test_dev_console_correction_state import FakeRedis

        fd, path = tempfile.mkstemp(suffix=".log")
        os.close(fd)
        try:
            with open(path, "w") as fp:
                fp.write("alpha\n")
                fp.write("RTK fixed\n")
                fp.write("linear correction cte=0.1\n")
                fp.write("RTK lost\n")
            app = create_app(redis_client=FakeRedis(), log_path=path)
            client = app.test_client()

            response = client.get("/dev/logs?query=RTK&limit=1")
        finally:
            os.remove(path)

        self.assertEqual(response.status_code, 200)
        payload = response.get_json()
        self.assertEqual(payload["query"], "RTK")
        self.assertEqual(payload["lines"], ["RTK lost"])


if __name__ == "__main__":
    unittest.main()
