import unittest


class _FakeController(object):
    def __init__(self):
        self.model_id = None

    def get_modeling_path(self, model_id):
        self.model_id = model_id
        return {
            "success": True,
            "message": "ok",
            "data": {
                "modelId": model_id,
                "taskPlan": {"status": "ready", "tasks": []},
            },
        }


class _StubAdapter(object):
    def _call(self, path, params=None, json_data=None):
        self.path = path
        return {
            "success": True,
            "data": {
                "id": "model-1",
                "name": "area-a",
                "updatedAt": 123,
                "taskPlan": {
                    "status": "ready",
                    "taskName": "area-a",
                    "tasks": [{"startX": 0, "startY": 0, "endX": 100, "endY": 0}],
                },
            },
        }


class MqttModelingBridgeTest(unittest.TestCase):
    def test_realtime_position_report_contains_only_relative_coordinates(self):
        import sys
        import types
        from unittest.mock import patch

        fake_util = types.ModuleType("util")
        fake_mqtt_client = types.ModuleType("mqtt_client")
        fake_mqtt_client.MQTTClient = object
        fake_mqtt_client.get_mqtt_client = lambda: None

        sys.modules.pop("mqtt_integration", None)
        with patch.dict(sys.modules, {
            "util": fake_util,
            "mqtt_client": fake_mqtt_client,
        }):
            from mqtt_integration import MQTTIntegration

            integration = object.__new__(MQTTIntegration)
            integration._get_vehicle_position_from_redis = lambda: {
                "local_x": 123,
                "local_y": 456,
            }

            class _MqttClient(object):
                def publish_realtime(self, payload):
                    self.payload = payload

            integration.mqtt_client = _MqttClient()
            integration.publish_vehicle_position()

            self.assertEqual(integration.mqtt_client.payload, {
                "type": "vehicle_position",
                "data": {"local_x": 123, "local_y": 456},
            })

    def test_handler_routes_get_modeling_path_to_existing_adapter(self):
        from mqtt_handler import MQTTCommandHandler

        controller = _FakeController()
        handler = MQTTCommandHandler(controller)
        result = handler.handle({
            "command": "get_modeling_path",
            "params": {"modelId": "model-1"},
        })

        self.assertTrue(result["success"])
        self.assertEqual(controller.model_id, "model-1")
        self.assertEqual(result["data"]["modelId"], "model-1")

    def test_handler_requires_model_id(self):
        from mqtt_handler import MQTTCommandHandler

        result = MQTTCommandHandler(_FakeController()).handle({
            "command": "get_modeling_path",
            "params": {},
        })

        self.assertFalse(result["success"])
        self.assertIn("modelId", result["message"])

        invalid = MQTTCommandHandler(_FakeController()).handle({
            "command": "get_modeling_path",
            "params": {"modelId": "../bad"},
        })
        self.assertFalse(invalid["success"])

    def test_adapter_returns_existing_task_plan_without_changing_its_format(self):
        from mqtt_vehicle_adapter import VehicleControllerAdapter

        adapter = _StubAdapter()
        result = VehicleControllerAdapter.get_modeling_path(adapter, "model-1")

        self.assertTrue(result["success"])
        self.assertEqual(adapter.path, "/modeling/draft/model-1")
        self.assertEqual(result["data"]["taskPlan"]["status"], "ready")
        self.assertIn("tasks", result["data"]["taskPlan"])

    def test_modeling_live_position_uses_first_task_segment_as_coordinate_anchor(self):
        import math
        import sys
        import types
        from unittest.mock import patch

        earth_radius_m = 6371000.0
        fake_util = types.ModuleType("util")

        def local_to_lat_lon(lat0, lon0, x_m, y_m, heading):
            lat = lat0 + math.degrees(y_m / earth_radius_m)
            lon = lon0 + math.degrees(x_m / (earth_radius_m * math.cos(math.radians(lat0))))
            return lat, lon

        def lat_lon_to_local(lat0, lon0, lat, lon, heading):
            x_m = math.radians(lon - lon0) * earth_radius_m * math.cos(math.radians(lat0))
            y_m = math.radians(lat - lat0) * earth_radius_m
            return x_m, y_m

        fake_util.local_rotated_xy_to_latlon_precise = local_to_lat_lon
        fake_util.latlon_to_local_rotated_xy_precise = lat_lon_to_local
        fake_mqtt_client = types.ModuleType("mqtt_client")
        fake_mqtt_client.MQTTClient = object
        fake_mqtt_client.get_mqtt_client = lambda: None

        sys.modules.pop("mqtt_integration", None)
        with patch.dict(sys.modules, {
            "util": fake_util,
            "mqtt_client": fake_mqtt_client,
        }):
            from mqtt_integration import MQTTIntegration

            integration = object.__new__(MQTTIntegration)
            anchor_lat = 32.0
            anchor_lon = 118.0
            target_lat, target_lon = fake_util.local_rotated_xy_to_latlon_precise(
                anchor_lat, anchor_lon, 1.0, 0.0, 0.0
            )
            runtime_state = {
                "action": "modeling_task",
                "detail": {
                    "modelingTask": {
                        "segments": [{
                            "startLat": anchor_lat,
                            "startLon": anchor_lon,
                            "startX": 250,
                            "startY": 400,
                        }],
                    },
                },
            }

            local_x, local_y = integration._compute_local_xy_cm(
                target_lat, target_lon, runtime_state
            )

        self.assertAlmostEqual(local_x, 350, delta=1)
        self.assertAlmostEqual(local_y, 400, delta=1)


if __name__ == "__main__":
    unittest.main()
