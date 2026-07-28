import unittest


class _FakeController(object):
    def __init__(self):
        self.model_id = None
        self.group_id = None
        self.link_id = None

    def get_modeling_path(self, model_id=None):
        self.model_id = model_id
        return {
            "success": True,
            "message": "ok",
            "data": {
                "modelId": model_id or "active-model",
                "taskPlan": {"status": "ready", "tasks": []},
            },
        }

    def get_modeling_points(self, model_id=None):
        self.model_id = model_id
        return {
            "success": True,
            "message": "modeling points fetched",
            "data": {
                "points": [{
                    "id": "p1",
                    "name": "区域点1",
                    "sequence": 1,
                    "x": 0,
                    "y": 0,
                    "lat": 32.0364,
                    "lon": 118.1234,
                }],
            },
        }

    def get_modeling_link_points(self, model_id=None):
        self.model_id = model_id
        return {
            "success": True,
            "message": "modeling link points fetched",
            "data": {
                "points": [{
                    "id": "lp1",
                    "name": "连接点1",
                    "sequence": 1,
                    "x": 0,
                    "y": 100,
                    "lat": 32.0365,
                    "lon": 118.1235,
                }],
            },
        }

    def sample_modeling_point(self, model_id=None, group_id=None):
        self.model_id = model_id
        self.group_id = group_id
        return {
            "success": True,
            "message": "modeling point recorded",
            "data": {
                "modelId": model_id or "active-model",
                "groupId": group_id or "active-group",
                "point": {
                    "id": "p1",
                    "sequence": 1,
                    "lat": 32.0364,
                    "lon": 118.1234,
                },
            },
        }

    def sample_modeling_link_point(self, model_id=None, link_id=None):
        self.model_id = model_id
        self.link_id = link_id
        return {
            "success": True,
            "message": "modeling link point recorded",
            "data": {
                "modelId": model_id or "active-model",
                "linkId": link_id or "active-link",
                "point": {
                    "id": "p2",
                    "sequence": 2,
                    "lat": 32.0365,
                    "lon": 118.1235,
                    "role": "group_link_end",
                },
            },
        }

    def start_modeling(self, name=None, restart=False):
        return {"success": True, "data": {"status": "recording", "name": name}}

    def finish_modeling(self):
        return {"success": True, "data": {"modelId": "active-model", "taskPlan": {"status": "ready"}}}

    def get_modeling_state(self):
        return {"success": True, "data": {"status": "recording"}}

    def undo_modeling_point(self, point_type=None):
        return {"success": True, "data": {"pointType": point_type or "area"}}

    def delete_modeling_point(self, point_id):
        return {"success": True, "data": {"pointType": "area", "id": point_id}}

    def delete_modeling_link_point(self, point_id):
        return {"success": True, "data": {"pointType": "link", "id": point_id}}

    def clear_modeling_points(self, point_type=None):
        return {"success": True, "data": {"pointType": point_type or "area"}}

    def clear_all_modeling_points(self, point_type):
        return {
            "success": True,
            "data": {
                "pointType": point_type,
                "clearedPointCount": 2,
            },
        }


class _StubAdapter(object):
    def _normalize_modeling_response(self, response, default_message):
        from mqtt_vehicle_adapter import VehicleControllerAdapter
        return VehicleControllerAdapter._normalize_modeling_response(self, response, default_message)

    def _call(self, path, params=None, json_data=None):
        self.path = path
        self.json_data = json_data
        if path == "/modeling/session/current":
            return {
                "success": True,
                "data": {
                    "status": "recording",
                    "modelId": "model-1",
                },
            }
        if path.endswith("/sample-point"):
            return {
                "success": True,
                "data": {
                    "point": {
                        "id": "p1",
                        "sequence": 1,
                        "lat": 32.0364,
                        "lon": 118.1234,
                    },
                },
            }
        if path == "/modeling/session/delete-area-point":
            return {
                "success": True,
                "data": {
                    "pointType": "area",
                    "id": json_data["id"],
                },
            }
        if path == "/modeling/session/delete-link-point":
            return {
                "success": True,
                "data": {
                    "pointType": "link",
                    "id": json_data["id"],
                },
            }
        if path == "/modeling/session/clear-all":
            return {
                "success": True,
                "data": {
                    "pointType": json_data["pointType"],
                    "clearedPointCount": 2,
                },
            }
        return {
            "success": True,
            "data": {
                "id": "model-1",
                "name": "area-a",
                "updatedAt": 123,
                "captureSequence": [
                    {"sequence": 1, "pointType": "area", "pointId": "p1"},
                    {"sequence": 2, "pointType": "link", "pointId": "lp1"},
                    {"sequence": 3, "pointType": "area", "pointId": "p2"},
                ],
                "groups": [{
                    "id": "group-1",
                    "areaNumber": 1,
                    "name": "area-1",
                    "points": [{
                        "id": "p1",
                        "sequence": 1,
                        "lat": 32.0364,
                        "lon": 118.1234,
                        "x": 0,
                        "y": 0,
                    }],
                }, {
                    "id": "group-2",
                    "areaNumber": 2,
                    "name": "area-2",
                    "points": [{
                        "id": "p2",
                        "sequence": 1,
                        "lat": 32.0366,
                        "lon": 118.1236,
                        "x": 200,
                        "y": 100,
                    }],
                }],
                "groupLinks": [{
                    "id": "link-1",
                    "startGroupId": "group-1",
                    "endGroupId": "group-2",
                    "status": "draft",
                    "points": [{
                        "id": "lp1",
                        "sequence": 1,
                        "lat": 32.0365,
                        "lon": 118.1235,
                        "x": 0,
                        "y": 100,
                    }],
                }],
                "taskPlan": {
                    "status": "ready",
                    "taskName": "area-a",
                    "tasks": [{"startX": 0, "startY": 0, "endX": 100, "endY": 0}],
                },
                "taskPreview": {
                    "status": "ready",
                    "groups": [{
                        "areaNumber": 1,
                        "subAreas": [{
                            "polygon": [
                                {"id": "p1", "x": 0, "y": 0},
                                {"id": "p2", "x": 100, "y": 0},
                                {"id": "p3", "x": 100, "y": 100},
                                {"id": "p4", "x": 0, "y": 100},
                            ],
                        }],
                    }],
                    "groupLinks": [],
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

    def test_handler_uses_current_session_when_model_id_is_missing(self):
        from mqtt_handler import MQTTCommandHandler

        result = MQTTCommandHandler(_FakeController()).handle({
            "command": "get_modeling_path",
            "params": {},
        })

        self.assertTrue(result["success"])
        self.assertEqual(result["data"]["modelId"], "active-model")

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
        self.assertEqual(result["data"]["taskPreview"]["status"], "ready")
        self.assertEqual(
            len(result["data"]["taskPreview"]["groups"][0]["subAreas"][0]["polygon"]),
            4,
        )

    def test_handler_routes_get_modeling_points_to_existing_adapter(self):
        from mqtt_handler import MQTTCommandHandler

        controller = _FakeController()
        result = MQTTCommandHandler(controller).handle({
            "command": "get_modeling_points",
            "params": {"modelId": "model-1"},
        })

        self.assertTrue(result["success"])
        self.assertEqual(controller.model_id, "model-1")
        self.assertEqual(len(result["data"]["points"]), 1)
        self.assertEqual(result["data"]["points"][0]["id"], "p1")

        invalid = MQTTCommandHandler(controller).handle({
            "command": "get_modeling_points",
            "params": {"modelId": "../bad"},
        })
        self.assertFalse(invalid["success"])

    def test_adapter_returns_flat_frontend_area_point_list(self):
        from mqtt_vehicle_adapter import VehicleControllerAdapter

        adapter = _StubAdapter()
        result = VehicleControllerAdapter.get_modeling_points(adapter, "model-1")

        self.assertTrue(result["success"])
        self.assertEqual(adapter.path, "/modeling/draft/model-1")
        self.assertEqual(
            [point["id"] for point in result["data"]["points"]],
            ["p1", "p2"],
        )
        self.assertEqual(
            [point["name"] for point in result["data"]["points"]],
            ["区域点1", "区域点2"],
        )
        self.assertEqual(
            [point["sequence"] for point in result["data"]["points"]],
            [1, 2],
        )
        self.assertEqual(result["data"]["points"][1]["x"], 200)
        self.assertNotIn("groups", result["data"])
        self.assertNotIn("groupLinks", result["data"])

    def test_adapter_uses_current_modeling_session_when_model_id_is_missing(self):
        from mqtt_vehicle_adapter import VehicleControllerAdapter

        adapter = _StubAdapter()
        result = VehicleControllerAdapter.get_modeling_points(adapter)

        self.assertTrue(result["success"])
        self.assertEqual(adapter.path, "/modeling/draft/model-1")
        self.assertEqual(len(result["data"]["points"]), 2)

    def test_handler_and_adapter_return_flat_frontend_link_point_list(self):
        from mqtt_handler import MQTTCommandHandler
        from mqtt_vehicle_adapter import VehicleControllerAdapter

        controller = _FakeController()
        handled = MQTTCommandHandler(controller).handle({
            "command": "get_modeling_link_points",
            "params": {"modelId": "model-1"},
        })
        invalid = MQTTCommandHandler(controller).handle({
            "command": "get_modeling_link_points",
            "params": {"modelId": "../bad"},
        })
        adapter = _StubAdapter()
        result = VehicleControllerAdapter.get_modeling_link_points(adapter, "model-1")

        self.assertTrue(handled["success"])
        self.assertEqual(controller.model_id, "model-1")
        self.assertEqual(handled["data"]["points"][0]["id"], "lp1")
        self.assertFalse(invalid["success"])
        self.assertTrue(result["success"])
        self.assertEqual(adapter.path, "/modeling/draft/model-1")
        self.assertEqual(result["data"]["points"], [{
            "id": "lp1",
            "name": "连接点1",
            "sequence": 1,
            "x": 0,
            "y": 100,
            "lat": 32.0365,
            "lon": 118.1235,
        }])
        self.assertNotIn("groupLinks", result["data"])

    def test_handler_and_adapter_delete_requested_modeling_point_by_id(self):
        from mqtt_handler import MQTTCommandHandler
        from mqtt_vehicle_adapter import VehicleControllerAdapter

        controller = _FakeController()
        handled = MQTTCommandHandler(controller).handle({
            "command": "delete_modeling_point",
            "params": {"id": "p1"},
        })
        invalid = MQTTCommandHandler(controller).handle({
            "command": "delete_modeling_point",
            "params": {"id": "../bad"},
        })
        adapter = _StubAdapter()
        deleted = VehicleControllerAdapter.delete_modeling_point(adapter, "p1")

        self.assertTrue(handled["success"])
        self.assertEqual(handled["data"]["id"], "p1")
        self.assertFalse(invalid["success"])
        self.assertTrue(deleted["success"])
        self.assertEqual(adapter.path, "/modeling/session/delete-area-point")
        self.assertEqual(adapter.json_data, {"id": "p1"})
        self.assertEqual(deleted["data"]["id"], "p1")

    def test_handler_and_adapter_delete_requested_link_point_by_id(self):
        from mqtt_handler import MQTTCommandHandler
        from mqtt_vehicle_adapter import VehicleControllerAdapter

        controller = _FakeController()
        handled = MQTTCommandHandler(controller).handle({
            "command": "delete_modeling_link_point",
            "params": {"id": "lp1"},
        })
        invalid = MQTTCommandHandler(controller).handle({
            "command": "delete_modeling_link_point",
            "params": {"id": "../bad"},
        })
        adapter = _StubAdapter()
        deleted = VehicleControllerAdapter.delete_modeling_link_point(adapter, "lp1")

        self.assertTrue(handled["success"])
        self.assertEqual(handled["data"]["pointType"], "link")
        self.assertEqual(handled["data"]["id"], "lp1")
        self.assertFalse(invalid["success"])
        self.assertTrue(deleted["success"])
        self.assertEqual(adapter.path, "/modeling/session/delete-link-point")
        self.assertEqual(adapter.json_data, {"id": "lp1"})
        self.assertEqual(deleted["data"]["id"], "lp1")

    def test_handler_and_adapter_clear_all_area_and_link_points(self):
        from mqtt_handler import MQTTCommandHandler
        from mqtt_vehicle_adapter import VehicleControllerAdapter

        handler = MQTTCommandHandler(_FakeController())
        cleared_area = handler.handle({
            "command": "clear_modeling_area_points",
            "params": {},
        })
        cleared_link = handler.handle({
            "command": "clear_modeling_link_points",
            "params": {},
        })
        adapter = _StubAdapter()
        adapted = VehicleControllerAdapter.clear_all_modeling_points(adapter, "link")

        self.assertTrue(cleared_area["success"])
        self.assertEqual(cleared_area["data"]["pointType"], "area")
        self.assertTrue(cleared_link["success"])
        self.assertEqual(cleared_link["data"]["pointType"], "link")
        self.assertTrue(adapted["success"])
        self.assertEqual(adapter.path, "/modeling/session/clear-all")
        self.assertEqual(adapter.json_data, {"pointType": "link"})

    def test_handler_routes_sample_modeling_point_to_existing_adapter(self):
        from mqtt_handler import MQTTCommandHandler

        controller = _FakeController()
        result = MQTTCommandHandler(controller).handle({
            "command": "sample_modeling_point",
            "params": {"modelId": "model-1", "groupId": "group-1"},
        })

        self.assertTrue(result["success"])
        self.assertEqual(controller.model_id, "model-1")
        self.assertEqual(controller.group_id, "group-1")
        self.assertEqual(result["data"]["point"]["sequence"], 1)

    def test_handler_uses_current_session_or_requires_complete_identifier_pair(self):
        from mqtt_handler import MQTTCommandHandler

        handler = MQTTCommandHandler(_FakeController())
        current_session = handler.handle({
            "command": "sample_modeling_point",
            "params": {},
        })
        missing_group = handler.handle({
            "command": "sample_modeling_point",
            "params": {"modelId": "model-1"},
        })
        invalid_model = handler.handle({
            "command": "sample_modeling_point",
            "params": {"modelId": "../bad", "groupId": "group-1"},
        })

        self.assertTrue(current_session["success"])
        self.assertFalse(missing_group["success"])
        self.assertIn("groupId", missing_group["message"])
        self.assertFalse(invalid_model["success"])

    def test_adapter_calls_existing_sample_point_endpoint(self):
        from mqtt_vehicle_adapter import VehicleControllerAdapter

        adapter = _StubAdapter()
        result = VehicleControllerAdapter.sample_modeling_point(adapter, "model-1", "group-1")

        self.assertTrue(result["success"])
        self.assertEqual(adapter.path, "/modeling/groups/group-1/sample-point")
        self.assertEqual(adapter.json_data, {"modelId": "model-1"})
        self.assertEqual(result["data"]["point"]["id"], "p1")

    def test_adapter_uses_session_point_endpoint_when_ids_are_omitted(self):
        from mqtt_vehicle_adapter import VehicleControllerAdapter

        adapter = _StubAdapter()
        result = VehicleControllerAdapter.sample_modeling_point(adapter)

        self.assertTrue(result["success"])
        self.assertEqual(adapter.path, "/modeling/session/record-area-point")
        self.assertEqual(adapter.json_data, {})

    def test_handler_routes_sample_modeling_link_point_to_existing_adapter(self):
        from mqtt_handler import MQTTCommandHandler

        controller = _FakeController()
        result = MQTTCommandHandler(controller).handle({
            "command": "sample_modeling_link_point",
            "params": {"modelId": "model-1", "linkId": "link-1"},
        })

        self.assertTrue(result["success"])
        self.assertEqual(controller.model_id, "model-1")
        self.assertEqual(controller.link_id, "link-1")
        self.assertEqual(result["data"]["point"]["role"], "group_link_end")

    def test_handler_uses_current_session_or_requires_complete_link_identifier_pair(self):
        from mqtt_handler import MQTTCommandHandler

        handler = MQTTCommandHandler(_FakeController())
        current_session = handler.handle({
            "command": "sample_modeling_link_point",
            "params": {},
        })
        missing_link = handler.handle({
            "command": "sample_modeling_link_point",
            "params": {"modelId": "model-1"},
        })
        invalid_link = handler.handle({
            "command": "sample_modeling_link_point",
            "params": {"modelId": "model-1", "linkId": "../bad"},
        })

        self.assertTrue(current_session["success"])
        self.assertFalse(missing_link["success"])
        self.assertIn("linkId", missing_link["message"])
        self.assertFalse(invalid_link["success"])

    def test_adapter_calls_existing_sample_link_point_endpoint(self):
        from mqtt_vehicle_adapter import VehicleControllerAdapter

        adapter = _StubAdapter()
        result = VehicleControllerAdapter.sample_modeling_link_point(adapter, "model-1", "link-1")

        self.assertTrue(result["success"])
        self.assertEqual(adapter.path, "/modeling/group-links/link-1/sample-point")
        self.assertEqual(adapter.json_data, {"modelId": "model-1"})
        self.assertEqual(result["data"]["point"]["id"], "p1")

    def test_handler_routes_session_lifecycle_commands(self):
        from mqtt_handler import MQTTCommandHandler

        handler = MQTTCommandHandler(_FakeController())

        self.assertTrue(handler.handle({"command": "start_modeling", "params": {"name": "area-a"}})["success"])
        self.assertTrue(handler.handle({"command": "get_modeling_state", "params": {}})["success"])
        self.assertTrue(handler.handle({"command": "undo_modeling_point", "params": {"pointType": "link"}})["success"])
        self.assertTrue(handler.handle({"command": "clear_modeling_points", "params": {"pointType": "area"}})["success"])
        self.assertTrue(handler.handle({"command": "finish_modeling", "params": {}})["success"])

        invalid = handler.handle({"command": "undo_modeling_point", "params": {"pointType": "bad"}})
        self.assertFalse(invalid["success"])

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
