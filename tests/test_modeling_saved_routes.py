import unittest

from modeling_saved_routes import build_saved_routes, discover_saved_task_names


class ModelingSavedRoutesTest(unittest.TestCase):
    def test_returns_every_saved_route_with_frontend_points(self):
        task_configs = {
            "route-a": {
                "modelId": "model-a",
                "areaOrder": [2, 1],
                "taskList": [{
                    "startX": 0,
                    "startY": 0,
                    "endX": 100,
                    "endY": 0,
                    "startLat": 32.0,
                    "startLon": 118.0,
                    "endLat": 32.0,
                    "endLon": 118.00001,
                }],
            },
            "route-b": {
                "modelId": "model-b",
                "taskList": [{
                    "startX": 200,
                    "startY": 0,
                    "endX": 200,
                    "endY": 100,
                }],
            },
        }
        models = {
            "model-a": {
                "captureSequence": [
                    {"pointType": "area", "pointId": "a1"},
                    {"pointType": "link", "pointId": "l1"},
                ],
                "groups": [{
                    "points": [{
                        "id": "a1",
                        "x": 0,
                        "y": 0,
                        "lat": 32.0,
                        "lon": 118.0,
                    }],
                }],
                "groupLinks": [{
                    "linkNumber": 3,
                    "points": [{
                        "id": "l1",
                        "x": 0,
                        "y": 50,
                        "lat": 32.00001,
                        "lon": 118.0,
                    }],
                }],
            },
            "model-b": {
                "groups": [],
                "groupLinks": [],
                "taskPlan": {"areaOrder": [1, 2]},
            },
        }

        result = build_saved_routes(
            {"route-b", "route-a"},
            "route-b",
            lambda task_name: task_configs[task_name],
            lambda model_id: models[model_id],
        )

        self.assertEqual(
            [route["taskName"] for route in result["routes"]],
            ["route-a", "route-b"],
        )
        self.assertEqual(result["currentTaskName"], "route-b")
        self.assertFalse(result["routes"][0]["current"])
        self.assertTrue(result["routes"][1]["current"])
        self.assertEqual(result["routes"][0]["modelId"], "model-a")
        self.assertEqual(result["routes"][0]["taskCount"], 1)
        self.assertEqual(result["routes"][0]["areaOrder"], [2, 1])
        self.assertEqual(result["routes"][1]["areaOrder"], [1, 2])
        self.assertEqual(result["routes"][0]["areaPoints"][0]["id"], "a1")
        self.assertEqual(result["routes"][0]["linkPoints"][0]["id"], "l1")
        self.assertEqual(result["routes"][0]["linkPoints"][0]["linkNumber"], 3)
        self.assertEqual(
            [point["id"] for point in result["routes"][0]["pathPoints"]],
            ["p1", "p2"],
        )

    def test_legacy_route_without_model_still_returns_path_points(self):
        result = build_saved_routes(
            ["legacy"],
            None,
            lambda task_name: {
                "taskList": [{
                    "startX": 0,
                    "startY": 0,
                    "endX": 0,
                    "endY": 100,
                }],
            },
            lambda model_id: {},
        )

        route = result["routes"][0]
        self.assertIsNone(route["modelId"])
        self.assertEqual(route["areaOrder"], [])
        self.assertEqual(route["areaPoints"], [])
        self.assertEqual(route["linkPoints"], [])
        self.assertEqual(len(route["pathPoints"]), 2)

    def test_discovers_valid_route_files_when_redis_index_is_missing(self):
        configs = {
            "test6.json": {
                "taskName": "test6",
                "modelId": "model-1",
                "taskList": [{"id": 1}],
            },
            u"测试6.json": {
                "taskName": u"测试6",
                "modelId": "model-1",
                "taskList": [{"id": 1}],
            },
            "legacy.json": {
                "taskName": "legacy",
                "taskList": [{"id": 1}],
            },
            "config.json": {
                "taskName": "test6",
                "modelId": "model-1",
                "taskList": [{"id": 1}],
            },
            "mqtt_config.json": {"mqtt": {}},
            "empty.json": {
                "taskName": "empty",
                "taskList": [],
            },
        }

        names = discover_saved_task_names(
            list(configs.keys()),
            lambda file_name: configs[file_name],
        )

        self.assertEqual(names, ["test6", u"测试6"])


if __name__ == "__main__":
    unittest.main()
