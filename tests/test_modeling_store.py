import os
import shutil
import tempfile
import unittest


class ModelingStoreTest(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmpdir)

    def test_create_model_writes_formal_model_and_empty_draft(self):
        from modeling_store import ModelingStore

        store = ModelingStore(self.tmpdir, now=lambda: 1000)

        model = store.create_model("field-model-a")

        self.assertEqual(model["name"], "field-model-a")
        self.assertEqual(model["status"], "draft")
        self.assertEqual(model["version"], 1)
        self.assertEqual(model["groups"], [])
        self.assertTrue(os.path.exists(os.path.join(self.tmpdir, "models", model["id"] + ".json")))
        self.assertTrue(os.path.exists(os.path.join(self.tmpdir, "drafts", model["id"] + ".json")))

    def test_save_draft_updates_draft_without_marking_formal_model_saved(self):
        from modeling_store import ModelingStore

        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        model = store.create_model("draft-model")
        draft = dict(model)
        draft["groups"] = [{"id": "g1", "name": "group-1", "points": []}]

        saved = store.save_draft(model["id"], draft, now=1005)
        formal = store.get_model(model["id"])

        self.assertEqual(saved["groups"][0]["id"], "g1")
        self.assertEqual(saved["status"], "draft")
        self.assertEqual(saved["updatedAt"], 1005)
        self.assertEqual(formal["groups"], [])

    def test_create_update_clear_and_delete_group_updates_draft_only(self):
        from modeling_store import ModelingStore

        ticks = iter([1000, 1001, 1002, 1003, 1004])
        store = ModelingStore(self.tmpdir, now=lambda: next(ticks))
        model = store.create_model("group-model")

        created = store.create_group(model["id"], "北侧区域", now=1001)
        group = created["group"]

        self.assertEqual(group["name"], "北侧区域")
        self.assertEqual(group["areaNumber"], 1)
        self.assertEqual(group["sweepDirection"], "auto")
        self.assertEqual(group["points"], [])
        self.assertEqual(created["draft"]["groups"][0]["id"], group["id"])
        self.assertEqual(store.get_model(model["id"])["groups"], [])

        updated = store.update_group(model["id"], group["id"], {
            "name": "北侧区域A",
            "sweepDirection": "manual",
            "sweepAngle": 92.5,
        }, now=1002)
        self.assertEqual(updated["group"]["name"], "北侧区域A")
        self.assertEqual(updated["group"]["sweepDirection"], "manual")
        self.assertEqual(updated["group"]["sweepAngle"], 92.5)

        draft = updated["draft"]
        draft["groups"][0]["points"] = [{"id": "p1", "lat": 32.0, "lon": 118.0}]
        store.save_draft(model["id"], draft, now=1003)

        cleared = store.clear_group_points(model["id"], group["id"], now=1004)
        self.assertEqual(cleared["group"]["points"], [])

        deleted = store.delete_group(model["id"], group["id"], now=1005)
        self.assertEqual(deleted["draft"]["groups"], [])

    def test_append_group_point_saves_sample_to_draft_group(self):
        from modeling_store import ModelingStore

        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        model = store.create_model("point-model")
        created = store.create_group(model["id"], "一区", now=1001)
        group = created["group"]

        saved = store.append_group_point(model["id"], group["id"], {
            "lat": 32.0364,
            "lon": 118.9244,
            "heading": 91.2,
            "source": "rtk_mean",
        }, now=1002)

        points = saved["group"]["points"]
        self.assertEqual(len(points), 1)
        self.assertEqual(points[0]["sequence"], 1)
        self.assertEqual(points[0]["lat"], 32.0364)
        self.assertEqual(points[0]["source"], "rtk_mean")

    def test_delete_group_point_removes_one_point_and_resequences(self):
        from modeling_store import ModelingStore

        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        model = store.create_model("delete-point-model")
        group = store.create_group(model["id"], "一区", now=1001)["group"]
        first = store.append_group_point(model["id"], group["id"], {
            "id": "p1",
            "lat": 32.0,
            "lon": 118.0,
        }, now=1002)["point"]
        second = store.append_group_point(model["id"], group["id"], {
            "id": "p2",
            "lat": 32.1,
            "lon": 118.1,
        }, now=1003)["point"]

        saved = store.delete_group_point(model["id"], group["id"], first["id"], now=1004)

        points = saved["group"]["points"]
        self.assertEqual(len(points), 1)
        self.assertEqual(points[0]["id"], second["id"])
        self.assertEqual(points[0]["sequence"], 1)
        self.assertEqual(saved["pointId"], first["id"])

    def test_recognize_group_updates_draft_with_sub_areas_and_connectors(self):
        from modeling_store import ModelingStore

        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        model = store.create_model("recognize-model")
        group = store.create_group(model["id"], "area-a", now=1001)["group"]
        for index, xy in enumerate([
            (0, 0),
            (1000, 0),
            (1000, 400),
            (1800, 400),
            (1800, 0),
            (2600, 0),
            (2600, 800),
            (1800, 800),
            (1000, 800),
            (0, 800),
        ], start=1):
            store.append_group_point(model["id"], group["id"], {
                "id": "p{}".format(index),
                "lat": 32.0,
                "lon": 118.0,
                "x": xy[0],
                "y": xy[1],
            }, now=1001 + index)

        saved = store.recognize_group(model["id"], group["id"], now=1020)

        self.assertEqual(saved["recognition"]["status"], "recognized")
        self.assertTrue(saved["recognition"]["confirmed"])
        self.assertEqual(len(saved["group"]["connectors"]), 1)
        self.assertEqual(saved["group"]["connectors"][0]["pointIds"], ["p3", "p4"])
        self.assertEqual(len(saved["group"]["subAreas"]), 2)
        self.assertTrue(saved["draft"]["recognition"]["confirmed"])
        self.assertEqual(saved["draft"]["recognition"]["confirmedAt"], 1020)
        self.assertEqual(saved["draft"]["recognition"]["generatedAt"], 1020)

    def test_confirm_group_recognition_marks_current_result_confirmed(self):
        from modeling_store import ModelingStore

        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        model = store.create_model("confirm-recognition-model")
        group = store.create_group(model["id"], "area-a", now=1001)["group"]
        for index, xy in enumerate([(0, 0), (1000, 0), (1000, 600), (0, 600)], start=1):
            store.append_group_point(model["id"], group["id"], {
                "id": "p{}".format(index),
                "lat": 32.0,
                "lon": 118.0,
                "x": xy[0],
                "y": xy[1],
            }, now=1001 + index)
        store.recognize_group(model["id"], group["id"], now=1010)

        saved = store.confirm_group_recognition(model["id"], group["id"], now=1020)

        self.assertTrue(saved["recognition"]["confirmed"])
        self.assertEqual(saved["recognition"]["confirmedAt"], 1020)
        self.assertEqual(saved["draft"]["recognition"]["groupId"], group["id"])

    def test_confirm_group_recognition_requires_existing_result(self):
        from modeling_store import ModelingStore, InvalidModelPayloadError

        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        model = store.create_model("confirm-missing-model")
        group = store.create_group(model["id"], "area-a", now=1001)["group"]

        with self.assertRaises(InvalidModelPayloadError):
            store.confirm_group_recognition(model["id"], group["id"], now=1020)

    def test_group_link_create_sample_and_delete_updates_draft(self):
        from modeling_store import ModelingStore

        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        model = store.create_model("group-link-model")
        start_group = store.create_group(model["id"], "area-a", now=1001)["group"]
        end_group = store.create_group(model["id"], "area-b", now=1002)["group"]

        created = store.create_group_link(model["id"], {
            "name": "area-a-to-area-b",
            "startGroupId": start_group["id"],
            "endGroupId": end_group["id"],
        }, now=1003)

        link = created["groupLink"]
        self.assertEqual(link["type"], "group_connector")
        self.assertEqual(link["startGroupId"], start_group["id"])
        self.assertEqual(link["endGroupId"], end_group["id"])
        self.assertEqual(link["points"], [])

        first = store.append_group_link_point(model["id"], link["id"], {
            "id": "p1",
            "lat": 32.0,
            "lon": 118.0,
        }, now=1004)
        second = store.append_group_link_point(model["id"], link["id"], {
            "id": "p2",
            "lat": 32.1,
            "lon": 118.1,
        }, now=1005)

        self.assertEqual(first["point"]["role"], "group_link_start")
        self.assertEqual(second["point"]["role"], "group_link_end")
        self.assertEqual(second["groupLink"]["status"], "ready")
        self.assertEqual(len(second["groupLink"]["points"]), 2)

        deleted = store.delete_group_link(model["id"], link["id"], now=1006)
        self.assertEqual(deleted["groupLinkId"], link["id"])
        self.assertEqual(deleted["draft"]["groupLinks"], [])

    def test_group_link_rejects_same_group_and_accepts_polyline_points(self):
        from modeling_store import ModelingStore, InvalidModelPayloadError

        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        model = store.create_model("invalid-group-link-model")
        start_group = store.create_group(model["id"], "area-a", now=1001)["group"]
        end_group = store.create_group(model["id"], "area-b", now=1002)["group"]

        with self.assertRaises(InvalidModelPayloadError):
            store.create_group_link(model["id"], {
                "startGroupId": start_group["id"],
                "endGroupId": start_group["id"],
            }, now=1003)

        link = store.create_group_link(model["id"], {
            "startGroupId": start_group["id"],
            "endGroupId": end_group["id"],
        }, now=1004)["groupLink"]
        store.append_group_link_point(
            model["id"], link["id"], {"id": "l1", "lat": 32.0, "lon": 118.0}, now=1005
        )
        store.append_group_link_point(
            model["id"], link["id"], {"id": "l2", "lat": 32.1, "lon": 118.1}, now=1006
        )
        third = store.append_group_link_point(
            model["id"], link["id"], {"id": "l3", "lat": 32.2, "lon": 118.2}, now=1007
        )

        self.assertEqual(
            [point["role"] for point in third["groupLink"]["points"]],
            ["group_link_start", "group_link_waypoint", "group_link_end"],
        )
        self.assertEqual(third["groupLink"]["status"], "ready")

        deleted = store.delete_group_link_point(
            model["id"], link["id"], "l2", now=1008
        )
        self.assertEqual(
            [point["id"] for point in deleted["groupLink"]["points"]],
            ["l1", "l3"],
        )
        self.assertEqual(
            [point["role"] for point in deleted["groupLink"]["points"]],
            ["group_link_start", "group_link_end"],
        )
        self.assertEqual(deleted["groupLink"]["status"], "ready")

    def test_build_task_preview_requires_confirmed_recognition_and_saves_preview(self):
        from modeling_store import ModelingStore

        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        model = store.create_model("preview-model")
        group = store.create_group(model["id"], "area-a", now=1001)["group"]
        for index, xy in enumerate([(0, 0), (1000, 0), (1000, 600), (0, 600)], start=1):
            store.append_group_point(model["id"], group["id"], {
                "id": "p{}".format(index),
                "lat": 32.0,
                "lon": 118.0,
                "x": xy[0],
                "y": xy[1],
            }, now=1001 + index)
        store.recognize_group(model["id"], group["id"], now=1010)
        store.confirm_group_recognition(model["id"], group["id"], now=1011)

        saved = store.build_task_preview(model["id"], now=1020)

        self.assertEqual(saved["taskPreview"]["status"], "ready")
        self.assertGreater(saved["taskPreview"]["summary"]["laneCount"], 1)
        self.assertEqual(saved["draft"]["taskPreview"]["generatedAt"], 1020)

    def test_generate_task_plan_saves_legacy_task_segments_to_draft(self):
        from modeling_store import ModelingStore

        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        model = store.create_model("task-plan-model")
        group = store.create_group(model["id"], "area-a", now=1001)["group"]
        for index, xy in enumerate([(0, 0), (1000, 0), (1000, 600), (0, 600)], start=1):
            store.append_group_point(model["id"], group["id"], {
                "id": "p{}".format(index),
                "lat": 32.0 + xy[1] * 0.00000009,
                "lon": 118.0 + xy[0] * 0.000000106,
                "x": xy[0],
                "y": xy[1],
            }, now=1001 + index)
        store.recognize_group(model["id"], group["id"], now=1010)
        store.confirm_group_recognition(model["id"], group["id"], now=1011)
        store.build_task_preview(model["id"], now=1020)

        saved = store.generate_task_plan(model["id"], now=1030)

        self.assertEqual(saved["taskPlan"]["status"], "ready")
        self.assertGreater(saved["taskPlan"]["summary"]["taskCount"], 1)
        self.assertEqual(saved["draft"]["taskPlan"]["generatedAt"], 1030)
        first = saved["taskPlan"]["tasks"][0]
        self.assertEqual(first["mode"], 2)
        self.assertEqual((first["startX"], first["startY"]), (0, 0))
        self.assertEqual((first["endX"], first["endY"]), (1000, 0))
        self.assertIn("startLat", first)
        self.assertIn("endLon", first)

    def test_save_model_promotes_draft_to_formal_model(self):
        from modeling_store import ModelingStore

        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        model = store.create_model("saved-model")
        draft = dict(model)
        draft["groups"] = [{"id": "g1", "name": "group-1", "points": []}]
        store.save_draft(model["id"], draft, now=1005)

        saved = store.save_model(model["id"], now=1010)

        self.assertEqual(saved["status"], "saved")
        self.assertEqual(saved["savedAt"], 1010)
        self.assertEqual(store.get_model(model["id"])["groups"][0]["id"], "g1")

    def test_list_models_returns_saved_metadata_without_large_body(self):
        from modeling_store import ModelingStore

        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        model = store.create_model("list-model")
        store.save_model(model["id"], now=1010)

        items = store.list_models()

        self.assertEqual(len(items), 1)
        self.assertEqual(items[0]["id"], model["id"])
        self.assertEqual(items[0]["name"], "list-model")
        self.assertEqual(items[0]["status"], "saved")
        self.assertNotIn("groups", items[0])

    def test_delete_model_removes_model_and_draft_files(self):
        from modeling_store import ModelingStore, ModelNotFoundError

        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        model = store.create_model("delete-model")

        removed = store.delete_model(model["id"])

        self.assertTrue(removed)
        with self.assertRaises(ModelNotFoundError):
            store.get_model(model["id"])
        self.assertFalse(os.path.exists(os.path.join(self.tmpdir, "drafts", model["id"] + ".json")))

    def test_rejects_unsafe_model_id(self):
        from modeling_store import ModelingStore, InvalidModelIdError

        store = ModelingStore(self.tmpdir, now=lambda: 1000)

        with self.assertRaises(InvalidModelIdError):
            store.get_model("../bad")


if __name__ == "__main__":
    unittest.main()
