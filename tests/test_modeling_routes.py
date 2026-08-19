import json
import os
import shutil
import tempfile
import unittest

from flask import Flask


class ModelingRoutesTest(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        app = Flask(__name__)
        from modeling_routes import register_modeling_routes
        self.sample_provider = lambda: {
            "lat": 32.0364,
            "lon": 118.9244,
            "heading": 90.0,
            "rtkQuality": "4",
            "rtkGgaAgeSec": 0.1,
            "controlState": "READY",
            "moving": False,
            "source": "rtk_mean",
            "sample": {"count": 10},
        }
        register_modeling_routes(app, storage_dir=self.tmpdir, sample_point_provider=lambda: self.sample_provider())
        self.client = app.test_client()

    def tearDown(self):
        shutil.rmtree(self.tmpdir)

    def test_create_list_load_save_and_delete_model(self):
        created = self.client.post("/modeling/models", json={"name": "field-a"})
        self.assertEqual(created.status_code, 200)
        model = created.get_json()["data"]
        self.assertEqual(model["name"], "field-a")

        listed = self.client.get("/modeling/models")
        self.assertEqual(listed.status_code, 200)
        self.assertEqual(listed.get_json()["data"][0]["id"], model["id"])

        loaded = self.client.get("/modeling/models/" + model["id"])
        self.assertEqual(loaded.status_code, 200)
        self.assertEqual(loaded.get_json()["data"]["id"], model["id"])

        saved = self.client.post("/modeling/models/{}/save".format(model["id"]))
        self.assertEqual(saved.status_code, 200)
        self.assertEqual(saved.get_json()["data"]["status"], "saved")

        deleted = self.client.delete("/modeling/models/" + model["id"])
        self.assertEqual(deleted.status_code, 200)
        self.assertTrue(deleted.get_json()["success"])

    def test_save_and_read_draft(self):
        created = self.client.post("/modeling/models", json={"name": "draft-a"}).get_json()["data"]
        draft = dict(created)
        draft["groups"] = [{"id": "g1", "name": "group-1", "points": []}]

        saved = self.client.post("/modeling/draft", json={"modelId": created["id"], "draft": draft})
        self.assertEqual(saved.status_code, 200)
        self.assertEqual(saved.get_json()["data"]["groups"][0]["id"], "g1")

        loaded = self.client.get("/modeling/draft/" + created["id"])
        self.assertEqual(loaded.status_code, 200)
        self.assertEqual(loaded.get_json()["data"]["groups"][0]["name"], "group-1")

    def test_group_management_routes_update_model_draft(self):
        model = self.client.post("/modeling/models", json={"name": "groups-a"}).get_json()["data"]

        created = self.client.post("/modeling/groups", json={"modelId": model["id"], "name": "一区"})
        self.assertEqual(created.status_code, 200)
        group = created.get_json()["data"]["group"]
        self.assertEqual(group["name"], "一区")
        self.assertEqual(group["areaNumber"], 1)

        updated = self.client.patch("/modeling/groups/" + group["id"], json={
            "modelId": model["id"],
            "name": "一区东",
            "sweepDirection": "manual",
            "sweepAngle": 88,
        })
        self.assertEqual(updated.status_code, 200)
        self.assertEqual(updated.get_json()["data"]["group"]["sweepDirection"], "manual")

        cleared = self.client.post("/modeling/groups/{}/clear-points".format(group["id"]), json={"modelId": model["id"]})
        self.assertEqual(cleared.status_code, 200)
        self.assertEqual(cleared.get_json()["data"]["group"]["points"], [])

        deleted = self.client.delete("/modeling/groups/{}?modelId={}".format(group["id"], model["id"]))
        self.assertEqual(deleted.status_code, 200)
        self.assertEqual(deleted.get_json()["data"]["draft"]["groups"], [])

    def test_sample_point_route_appends_current_rtk_point_to_group(self):
        model = self.client.post("/modeling/models", json={"name": "sample-a"}).get_json()["data"]
        group = self.client.post("/modeling/groups", json={"modelId": model["id"], "name": "一区"}).get_json()["data"]["group"]

        sampled = self.client.post("/modeling/groups/{}/sample-point".format(group["id"]), json={"modelId": model["id"]})

        self.assertEqual(sampled.status_code, 200)
        point = sampled.get_json()["data"]["point"]
        self.assertEqual(point["lat"], 32.0364)
        self.assertEqual(point["sequence"], 1)
        self.assertEqual(sampled.get_json()["data"]["group"]["points"][0]["source"], "rtk_mean")

    def test_session_routes_manage_current_model_without_frontend_ids(self):
        started = self.client.post("/modeling/session/start", json={"name": "mini-program-area"})
        recorded = self.client.post("/modeling/session/record-area-point", json={})
        current = self.client.get("/modeling/session/current")

        self.assertEqual(started.status_code, 200)
        self.assertEqual(started.get_json()["data"]["status"], "recording")
        self.assertEqual(recorded.status_code, 200)
        self.assertEqual(recorded.get_json()["data"]["pointType"], "area")
        self.assertEqual(recorded.get_json()["data"]["pointNo"], 1)
        self.assertEqual(current.get_json()["data"]["areaPointCount"], 1)

    def test_session_new_link_route_selects_empty_bridge(self):
        self.client.post("/modeling/session/start", json={"name": "bridge-route"})
        for _ in range(4):
            self.client.post("/modeling/session/record-area-point", json={})

        created = self.client.post("/modeling/session/new-link", json={})

        self.assertEqual(created.status_code, 200)
        self.assertEqual(created.get_json()["data"]["linkNumber"], 1)
        self.assertEqual(created.get_json()["data"]["linkPointCount"], 0)
        self.assertEqual(created.get_json()["data"]["session"]["currentLinkNumber"], 1)

    def test_session_save_task_requires_ready_path_and_delegates_named_plan(self):
        from modeling_routes import register_modeling_routes

        calls = []
        app = Flask(__name__)

        def fake_save_handler(task_name, current_path):
            calls.append((task_name, current_path))
            return {
                "taskName": task_name,
                "taskCount": len(current_path["taskPlan"]["tasks"]),
            }

        register_modeling_routes(
            app,
            storage_dir=self.tmpdir,
            sample_point_provider=lambda: self.sample_provider(),
            task_save_handler=fake_save_handler,
        )
        client = app.test_client()
        started = client.post(
            "/modeling/session/start",
            json={"name": "route-preview"},
        ).get_json()["data"]

        not_ready = client.post(
            "/modeling/session/save-task",
            json={"taskName": "route-a"},
        )
        self.assertEqual(not_ready.status_code, 400)
        self.assertEqual(not_ready.get_json()["code"], "MODELING_PATH_NOT_READY")

        draft = client.get(
            "/modeling/draft/" + started["modelId"],
        ).get_json()["data"]
        draft["taskPlan"] = {
            "status": "ready",
            "tasks": [{"id": 1, "startX": 0, "startY": 0, "endX": 0, "endY": 100}],
        }
        client.post(
            "/modeling/draft",
            json={"modelId": started["modelId"], "draft": draft},
        )
        state_path = os.path.join(self.tmpdir, "active_session.json")
        with open(state_path, "r") as handle:
            state = json.load(handle)
        state["status"] = "ready"
        with open(state_path, "w") as handle:
            json.dump(state, handle)

        saved = client.post(
            "/modeling/session/save-task",
            json={"taskName": "route-a"},
        )

        self.assertEqual(saved.status_code, 200)
        self.assertEqual(saved.get_json()["data"]["taskName"], "route-a")
        self.assertEqual(saved.get_json()["data"]["taskCount"], 1)
        self.assertEqual(calls[0][0], "route-a")
        self.assertEqual(calls[0][1]["modelId"], started["modelId"])

    def test_session_delete_area_point_route_uses_only_point_id(self):
        self.client.post("/modeling/session/start", json={"name": "delete-area-point"})
        first = self.client.post(
            "/modeling/session/record-area-point",
            json={},
        ).get_json()["data"]["point"]
        self.client.post("/modeling/session/record-area-point", json={})

        deleted = self.client.post(
            "/modeling/session/delete-area-point",
            json={"id": first["id"]},
        )
        current = self.client.get("/modeling/session/current")

        self.assertEqual(deleted.status_code, 200)
        self.assertEqual(deleted.get_json()["data"]["id"], first["id"])
        self.assertEqual(current.get_json()["data"]["areaPointCount"], 1)

    def test_session_delete_link_point_route_uses_only_point_id(self):
        self.client.post("/modeling/session/start", json={"name": "delete-link-point"})
        for _ in range(4):
            self.client.post("/modeling/session/record-area-point", json={})
        first = self.client.post(
            "/modeling/session/record-link-point",
            json={},
        ).get_json()["data"]["point"]
        self.client.post("/modeling/session/record-link-point", json={})

        deleted = self.client.post(
            "/modeling/session/delete-link-point",
            json={"id": first["id"]},
        )
        current = self.client.get("/modeling/session/current")

        self.assertEqual(deleted.status_code, 200)
        self.assertEqual(deleted.get_json()["data"]["id"], first["id"])
        self.assertEqual(deleted.get_json()["data"]["pointType"], "link")
        self.assertEqual(current.get_json()["data"]["linkPointCount"], 1)

    def test_session_clear_all_route_clears_all_areas_and_connections(self):
        self.client.post("/modeling/session/start", json={"name": "clear-all-points"})
        for _ in range(4):
            self.client.post("/modeling/session/record-area-point", json={})
        self.client.post("/modeling/session/record-link-point", json={})
        self.client.post("/modeling/session/record-link-point", json={})
        created_area = self.client.post("/modeling/session/new-area", json={})
        self.assertEqual(created_area.status_code, 200)
        self.assertEqual(created_area.get_json()["data"], {"areaNumber": 2, "groupCount": 2})
        for _ in range(2):
            self.client.post("/modeling/session/record-area-point", json={})

        cleared_area = self.client.post(
            "/modeling/session/clear-all",
            json={"pointType": "area"},
        )
        cleared_link = self.client.post(
            "/modeling/session/clear-all",
            json={"pointType": "link"},
        )
        current = self.client.get("/modeling/session/current").get_json()["data"]

        self.assertEqual(cleared_area.status_code, 200)
        self.assertEqual(cleared_area.get_json()["data"]["clearedPointCount"], 6)
        self.assertEqual(cleared_area.get_json()["data"]["session"]["totalAreaPointCount"], 0)
        self.assertEqual(cleared_link.status_code, 200)
        self.assertEqual(cleared_link.get_json()["data"]["clearedPointCount"], 2)
        self.assertEqual(current["totalAreaPointCount"], 0)
        self.assertEqual(current["totalLinkPointCount"], 0)
        self.assertEqual(current["currentAreaNumber"], 1)
        self.assertEqual(current["groupCount"], 1)
        self.assertEqual(current["linkCount"], 0)

    def test_sample_status_route_reports_current_readiness(self):
        response = self.client.get("/modeling/sample-status")

        self.assertEqual(response.status_code, 200)
        data = response.get_json()["data"]
        self.assertTrue(data["ready"])
        self.assertEqual(data["code"], "READY")

    def test_delete_group_point_route_removes_one_point(self):
        model = self.client.post("/modeling/models", json={"name": "delete-point-a"}).get_json()["data"]
        group = self.client.post("/modeling/groups", json={"modelId": model["id"], "name": "一区"}).get_json()["data"]["group"]
        point = self.client.post("/modeling/groups/{}/sample-point".format(group["id"]), json={"modelId": model["id"]}).get_json()["data"]["point"]

        deleted = self.client.delete("/modeling/groups/{}/points/{}?modelId={}".format(group["id"], point["id"], model["id"]))

        self.assertEqual(deleted.status_code, 200)
        self.assertEqual(deleted.get_json()["data"]["group"]["points"], [])
        self.assertEqual(deleted.get_json()["data"]["pointId"], point["id"])

    def test_recognize_group_route_updates_group_draft(self):
        model = self.client.post("/modeling/models", json={"name": "recognize-a"}).get_json()["data"]
        draft = dict(model)
        draft["groups"] = [{
            "id": "g1",
            "name": "group-1",
            "areaNumber": 1,
            "points": [
                {"id": "p1", "sequence": 1, "lat": 32.0, "lon": 118.0, "x": 0, "y": 0},
                {"id": "p2", "sequence": 2, "lat": 32.0, "lon": 118.0, "x": 1000, "y": 0},
                {"id": "p3", "sequence": 3, "lat": 32.0, "lon": 118.0, "x": 1000, "y": 400},
                {"id": "p4", "sequence": 4, "lat": 32.0, "lon": 118.0, "x": 1800, "y": 400},
                {"id": "p5", "sequence": 5, "lat": 32.0, "lon": 118.0, "x": 1800, "y": 0},
                {"id": "p6", "sequence": 6, "lat": 32.0, "lon": 118.0, "x": 2600, "y": 0},
                {"id": "p7", "sequence": 7, "lat": 32.0, "lon": 118.0, "x": 2600, "y": 800},
                {"id": "p8", "sequence": 8, "lat": 32.0, "lon": 118.0, "x": 1800, "y": 800},
                {"id": "p9", "sequence": 9, "lat": 32.0, "lon": 118.0, "x": 1000, "y": 800},
                {"id": "p10", "sequence": 10, "lat": 32.0, "lon": 118.0, "x": 0, "y": 800},
            ],
        }]
        self.client.post("/modeling/draft", json={"modelId": model["id"], "draft": draft})

        response = self.client.post("/modeling/groups/g1/recognize", json={"modelId": model["id"]})

        self.assertEqual(response.status_code, 200)
        data = response.get_json()["data"]
        self.assertEqual(data["recognition"]["status"], "recognized")
        self.assertTrue(data["recognition"]["confirmed"])
        self.assertFalse(data["recognition"]["needsConfirmation"])
        self.assertEqual(data["group"]["connectors"][0]["pointIds"], ["p3", "p4"])
        self.assertEqual(len(data["group"]["subAreas"]), 2)

    def test_confirm_group_recognition_route_marks_result_confirmed(self):
        model = self.client.post("/modeling/models", json={"name": "confirm-recognition-a"}).get_json()["data"]
        draft = dict(model)
        draft["groups"] = [{
            "id": "g1",
            "name": "group-1",
            "areaNumber": 1,
            "points": [
                {"id": "p1", "sequence": 1, "lat": 32.0, "lon": 118.0, "x": 0, "y": 0},
                {"id": "p2", "sequence": 2, "lat": 32.0, "lon": 118.000106, "x": 1000, "y": 0},
                {"id": "p3", "sequence": 3, "lat": 32.000054, "lon": 118.000106, "x": 1000, "y": 600},
                {"id": "p4", "sequence": 4, "lat": 32.000054, "lon": 118.0, "x": 0, "y": 600},
            ],
        }]
        self.client.post("/modeling/draft", json={"modelId": model["id"], "draft": draft})
        self.client.post("/modeling/groups/g1/recognize", json={"modelId": model["id"]})

        response = self.client.post("/modeling/groups/g1/confirm-recognition", json={"modelId": model["id"]})

        self.assertEqual(response.status_code, 200)
        data = response.get_json()["data"]
        self.assertTrue(data["recognition"]["confirmed"])
        self.assertEqual(data["recognition"]["groupId"], "g1")

    def test_group_link_routes_create_sample_and_delete_link(self):
        model = self.client.post("/modeling/models", json={"name": "group-link-a"}).get_json()["data"]
        start_group = self.client.post("/modeling/groups", json={
            "modelId": model["id"],
            "name": "area-a",
        }).get_json()["data"]["group"]
        end_group = self.client.post("/modeling/groups", json={
            "modelId": model["id"],
            "name": "area-b",
        }).get_json()["data"]["group"]

        created = self.client.post("/modeling/group-links", json={
            "modelId": model["id"],
            "name": "area-a-to-area-b",
            "startGroupId": start_group["id"],
            "endGroupId": end_group["id"],
        })

        self.assertEqual(created.status_code, 200)
        link = created.get_json()["data"]["groupLink"]
        self.assertEqual(link["startGroupId"], start_group["id"])
        self.assertEqual(link["endGroupId"], end_group["id"])

        first = self.client.post("/modeling/group-links/{}/sample-point".format(link["id"]), json={"modelId": model["id"]})
        second = self.client.post("/modeling/group-links/{}/sample-point".format(link["id"]), json={"modelId": model["id"]})

        self.assertEqual(first.status_code, 200)
        self.assertEqual(first.get_json()["data"]["point"]["role"], "group_link_start")
        self.assertEqual(second.status_code, 200)
        self.assertEqual(second.get_json()["data"]["point"]["role"], "group_link_end")
        self.assertEqual(second.get_json()["data"]["groupLink"]["status"], "ready")

        deleted = self.client.delete("/modeling/group-links/{}?modelId={}".format(link["id"], model["id"]))

        self.assertEqual(deleted.status_code, 200)
        self.assertEqual(deleted.get_json()["data"]["groupLinkId"], link["id"])
        self.assertEqual(deleted.get_json()["data"]["draft"]["groupLinks"], [])

    def test_build_preview_route_returns_task_preview(self):
        model = self.client.post("/modeling/models", json={"name": "preview-a"}).get_json()["data"]
        draft = dict(model)
        draft["groups"] = [{
            "id": "g1",
            "name": "group-1",
            "areaNumber": 1,
            "sweepDirection": "auto",
            "points": [
                {"id": "p1", "sequence": 1, "lat": 32.0, "lon": 118.0, "x": 0, "y": 0},
                {"id": "p2", "sequence": 2, "lat": 32.0, "lon": 118.0, "x": 1000, "y": 0},
                {"id": "p3", "sequence": 3, "lat": 32.0, "lon": 118.0, "x": 1000, "y": 600},
                {"id": "p4", "sequence": 4, "lat": 32.0, "lon": 118.0, "x": 0, "y": 600},
            ],
            "subAreas": [{"id": "sa1", "name": "sub-a", "pointIds": ["p1", "p2", "p3", "p4"]}],
            "connectors": [],
        }]
        draft["recognition"] = {
            "confirmed": True,
            "groupId": "g1",
            "status": "recognized",
            "summary": {"pointCount": 4, "subAreaCount": 1, "connectorCount": 0, "assistPointCount": 0},
        }
        # 该测试直接构造的x/y已经处于统一模型坐标系，明确写出坐标系元数据，
        # 避免预览接口把近似经纬度再次迁移并引入与路线算法无关的测试误差。
        draft["coordinateFrame"] = {
            "type": "model_origin",
            "unit": "cm",
            "originPointId": "p1",
            "originGroupId": "g1",
            "originLat": 32.0,
            "originLon": 118.0,
        }
        self.client.post("/modeling/draft", json={"modelId": model["id"], "draft": draft})

        response = self.client.post("/modeling/preview", json={"modelId": model["id"]})

        self.assertEqual(response.status_code, 200)
        preview = response.get_json()["data"]["taskPreview"]
        self.assertEqual(preview["status"], "ready")
        self.assertGreater(preview["summary"]["laneCount"], 1)

    def test_generate_tasks_route_returns_legacy_task_plan(self):
        model = self.client.post("/modeling/models", json={"name": "task-plan-a"}).get_json()["data"]
        draft = dict(model)
        draft["groups"] = [{
            "id": "g1",
            "name": "group-1",
            "areaNumber": 1,
            "sweepDirection": "auto",
            "points": [
                {"id": "p1", "sequence": 1, "lat": 32.0, "lon": 118.0, "x": 0, "y": 0},
                {"id": "p2", "sequence": 2, "lat": 32.0, "lon": 118.000106, "x": 1000, "y": 0},
                {"id": "p3", "sequence": 3, "lat": 32.000054, "lon": 118.000106, "x": 1000, "y": 600},
                {"id": "p4", "sequence": 4, "lat": 32.000054, "lon": 118.0, "x": 0, "y": 600},
            ],
            "subAreas": [{"id": "sa1", "name": "sub-a", "pointIds": ["p1", "p2", "p3", "p4"]}],
            "connectors": [],
        }]
        draft["recognition"] = {
            "confirmed": True,
            "groupId": "g1",
            "status": "recognized",
            "summary": {"pointCount": 4, "subAreaCount": 1, "connectorCount": 0, "assistPointCount": 0},
        }
        self.client.post("/modeling/draft", json={"modelId": model["id"], "draft": draft})
        self.client.post("/modeling/preview", json={"modelId": model["id"]})

        response = self.client.post("/modeling/tasks/generate", json={"modelId": model["id"]})

        self.assertEqual(response.status_code, 200)
        task_plan = response.get_json()["data"]["taskPlan"]
        self.assertEqual(task_plan["status"], "ready")
        self.assertGreater(task_plan["summary"]["taskCount"], 1)
        self.assertIn("startLat", task_plan["tasks"][0])
        self.assertIn("heading", task_plan["tasks"][0])

    def test_start_modeling_tasks_route_uses_generated_task_plan(self):
        from modeling_routes import register_modeling_routes

        calls = []
        app = Flask(__name__)

        def fake_starter(model_id, draft, payload):
            calls.append({
                "model_id": model_id,
                "draft": draft,
                "payload": payload,
            })
            return {
                "status": "starting",
                "action": "modeling_task",
                "modelId": model_id,
                "taskCount": len(draft["taskPlan"]["tasks"]),
            }

        register_modeling_routes(
            app,
            storage_dir=self.tmpdir,
            sample_point_provider=lambda: self.sample_provider(),
            task_execution_starter=fake_starter,
        )
        client = app.test_client()
        model = client.post("/modeling/models", json={"name": "execute-a"}).get_json()["data"]
        draft = dict(model)
        draft["taskPlan"] = {
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
            "summary": {"taskCount": 1},
        }
        client.post("/modeling/draft", json={"modelId": model["id"], "draft": draft})

        response = client.post("/modeling/tasks/start", json={"modelId": model["id"], "speed": 320})

        self.assertEqual(response.status_code, 200)
        data = response.get_json()["data"]
        self.assertEqual(data["action"], "modeling_task")
        self.assertEqual(data["taskCount"], 1)
        self.assertEqual(calls[0]["model_id"], model["id"])
        self.assertEqual(calls[0]["draft"]["taskPlan"]["status"], "ready")
        self.assertEqual(calls[0]["payload"]["speed"], 320)

    def test_task_progress_and_stop_routes_delegate_to_runtime_handlers(self):
        from modeling_routes import register_modeling_routes

        calls = []
        app = Flask(__name__)

        def fake_progress_reader(payload):
            calls.append({"type": "progress", "payload": payload})
            return {
                "available": True,
                "progress": {
                    "status": "running",
                    "action": "modeling_task",
                    "modelId": payload.get("modelId"),
                    "currentIndex": 2,
                    "total": 8,
                },
            }

        def fake_stop_handler(payload):
            calls.append({"type": "stop", "payload": payload})
            return {
                "status": "stopping",
                "action": "modeling_task",
                "modelId": payload.get("modelId"),
            }

        register_modeling_routes(
            app,
            storage_dir=self.tmpdir,
            sample_point_provider=lambda: self.sample_provider(),
            task_progress_reader=fake_progress_reader,
            task_stop_handler=fake_stop_handler,
        )
        client = app.test_client()

        progress = client.get("/modeling/tasks/progress?modelId=model-a")
        stopped = client.post("/modeling/tasks/stop", json={"modelId": "model-a"})

        self.assertEqual(progress.status_code, 200)
        self.assertEqual(progress.get_json()["data"]["progress"]["currentIndex"], 2)
        self.assertEqual(stopped.status_code, 200)
        self.assertEqual(stopped.get_json()["data"]["status"], "stopping")
        self.assertEqual(calls[0]["type"], "progress")
        self.assertEqual(calls[0]["payload"]["modelId"], "model-a")
        self.assertEqual(calls[1]["type"], "stop")
        self.assertEqual(calls[1]["payload"]["modelId"], "model-a")

    def test_missing_model_returns_404_response(self):
        response = self.client.get("/modeling/models/missing")

        self.assertEqual(response.status_code, 404)
        self.assertFalse(response.get_json()["success"])
        self.assertEqual(response.get_json()["code"], "MODEL_NOT_FOUND")

    def test_invalid_payload_returns_400_response(self):
        response = self.client.post("/modeling/draft", json={"modelId": "", "draft": []})

        self.assertEqual(response.status_code, 400)
        self.assertFalse(response.get_json()["success"])


if __name__ == "__main__":
    unittest.main()
