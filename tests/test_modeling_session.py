import shutil
import tempfile
import unittest


def _point(point_id, x, y):
    return {
        "id": point_id,
        "lat": 32.0 + y * 0.00000009,
        "lon": 118.0 + x * 0.000000106,
        "x": x,
        "y": y,
        "source": "rtk_mean",
    }


class _PointProvider(object):
    def __init__(self, points):
        self.points = iter(points)

    def __call__(self):
        return next(self.points)


class ModelingSessionTest(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmpdir)

    def test_start_record_finish_and_reload_current_path(self):
        from modeling_session import ModelingSession
        from modeling_store import ModelingStore

        provider = _PointProvider([
            _point("p1", 0, 0),
            _point("p2", 226, 0),
            _point("p3", 226, 113),
            _point("p4", 0, 113),
        ])
        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        session = ModelingSession(store, provider, now=lambda: 1000)

        started = session.start("panel-a")
        recorded = [session.record_area_point() for _ in range(4)]
        finished = session.finish()

        self.assertEqual(started["status"], "recording")
        self.assertEqual(recorded[-1]["pointNo"], 4)
        self.assertEqual(recorded[-1]["point"]["areaNumber"], 1)
        self.assertNotIn("groupId", recorded[-1]["point"])
        self.assertEqual(recorded[-1]["session"]["areaPointCount"], 4)
        self.assertEqual(finished["session"]["status"], "ready")
        self.assertEqual(finished["taskPreview"]["status"], "ready")
        self.assertEqual(len(finished["taskPreview"]["groups"][0]["subAreas"][0]["polygon"]), 4)
        self.assertEqual(finished["taskPlan"]["status"], "ready")
        self.assertGreaterEqual(finished["taskPlan"]["summary"]["cleanTaskCount"], 1)

        reloaded = ModelingSession(store, None, now=lambda: 1001)
        self.assertEqual(reloaded.current()["modelId"], started["modelId"])
        current_path = reloaded.current_path()
        self.assertEqual(current_path["taskPreview"]["status"], "ready")
        self.assertEqual(current_path["taskPlan"]["status"], "ready")

    def test_connection_points_wait_for_explicit_new_area_command(self):
        from modeling_session import ModelingSession
        from modeling_store import ModelingStore, InvalidModelPayloadError

        provider = _PointProvider([
            _point("a1", 0, 0),
            _point("a2", 400, 0),
            _point("a3", 400, 200),
            _point("a4", 0, 200),
            _point("l1", 400, 100),
            _point("l2", 600, 100),
            _point("unused", 700, 100),
            _point("b1", 600, 0),
            _point("b2", 1000, 0),
            _point("b3", 1000, 200),
            _point("b4", 600, 200),
        ])
        session = ModelingSession(
            ModelingStore(self.tmpdir, now=lambda: 1000),
            provider,
            now=lambda: 1000,
        )
        session.start("two-areas")
        for _ in range(4):
            session.record_area_point()

        first_link = session.record_link_point()
        second_link = session.record_link_point()

        self.assertEqual(first_link["point"]["role"], "group_link_start")
        self.assertEqual(second_link["point"]["role"], "group_link_end")
        self.assertEqual(second_link["session"]["linkPointCount"], 2)
        self.assertEqual(second_link["session"]["groupCount"], 1)

        with self.assertRaises(InvalidModelPayloadError):
            session.record_link_point()

        created_area = session.new_area()
        self.assertEqual(created_area, {"areaNumber": 2, "groupCount": 2})
        next_area = session.record_area_point()
        self.assertEqual(next_area["point"]["id"], "b1")
        self.assertEqual(next_area["point"]["areaNumber"], 2)
        self.assertEqual(next_area["session"]["currentAreaNumber"], 2)
        self.assertEqual(next_area["session"]["areaPointCount"], 1)

        for _ in range(3):
            session.record_area_point()
        finished = session.finish()
        self.assertEqual(finished["session"]["groupCount"], 2)
        self.assertEqual(finished["session"]["linkCount"], 1)
        self.assertEqual(finished["taskPlan"]["status"], "ready")
        self.assertGreater(finished["taskPlan"]["summary"]["transferTaskCount"], 0)

    def test_new_area_rejects_missing_or_incomplete_connection(self):
        from modeling_session import ModelingSession, ModelingSessionError
        from modeling_store import ModelingStore

        session = ModelingSession(
            ModelingStore(self.tmpdir, now=lambda: 1000),
            _PointProvider([
                _point("a1", 0, 0),
                _point("a2", 100, 0),
                _point("a3", 100, 100),
                _point("a4", 0, 100),
                _point("l1", 100, 50),
            ]),
            now=lambda: 1000,
        )
        session.start("new-area-validation")
        for _ in range(3):
            session.record_area_point()

        with self.assertRaises(ModelingSessionError) as incomplete_area:
            session.record_link_point()
        self.assertEqual(incomplete_area.exception.code, "MODELING_AREA_INCOMPLETE")
        session.record_area_point()

        with self.assertRaises(ModelingSessionError) as missing_link:
            session.new_area()
        self.assertEqual(missing_link.exception.code, "MODELING_LINK_MISSING")

        session.record_link_point()
        with self.assertRaises(ModelingSessionError) as incomplete_link:
            session.new_area()
        self.assertEqual(incomplete_link.exception.code, "MODELING_LINK_INCOMPLETE")

    def test_explicit_two_area_capture_generates_remote_first_round_trip(self):
        from modeling_session import ModelingSession
        from modeling_store import ModelingStore

        provider = _PointProvider([
            _point("p1", 0, 0),
            _point("p2", 0, 452),
            _point("p9", 565, 452),
            _point("p10", 565, 0),
            _point("p3", 0, 470),
            _point("p4", 0, 534),
            _point("p5", 0, 552),
            _point("p6", 0, 778),
            _point("p7", 452, 778),
            _point("p8", 452, 552),
        ])
        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        session = ModelingSession(store, provider, now=lambda: 1000)
        session.start("confirmed-route")

        for _ in range(4):
            session.record_area_point()
        session.record_link_point()
        session.record_link_point()
        session.new_area()
        for _ in range(4):
            session.record_area_point()
        finished = session.finish()

        draft = store.get_draft(finished["modelId"])
        self.assertEqual(
            [[point["id"] for point in group["points"]] for group in draft["groups"]],
            [["p1", "p2", "p9", "p10"], ["p5", "p6", "p7", "p8"]],
        )
        self.assertEqual(
            [group["subAreas"][0]["laneCount"] for group in draft["taskPreview"]["groups"]],
            [8, 4],
        )
        self.assertEqual(
            [
                [point["id"] for point in group["subAreas"][0]["polygon"]]
                for group in finished["taskPreview"]["groups"]
            ],
            [["p1", "p2", "p9", "p10"], ["p5", "p6", "p7", "p8"]],
        )
        self.assertEqual(len(finished["taskPreview"]["groupLinks"]), 1)
        task_plan = finished["taskPlan"]
        self.assertEqual(task_plan["routeType"], "bridge_round_trip")
        self.assertEqual(task_plan["summary"]["cleanTaskCount"], 12)
        self.assertEqual(
            (task_plan["tasks"][0]["startX"], task_plan["tasks"][0]["startY"]),
            (0, 0),
        )
        self.assertEqual(
            (task_plan["tasks"][0]["endX"], task_plan["tasks"][0]["endY"]),
            (0, 778),
        )
        self.assertGreaterEqual(task_plan["tasks"][0].get("mergedSegmentCount", 1), 2)
        clean_tasks = [task for task in task_plan["tasks"] if task["mode"] == 1]
        self.assertEqual([task["areaNumber"] for task in clean_tasks], [2] * 4 + [1] * 8)
        self.assertEqual(
            (clean_tasks[0]["startX"], clean_tasks[0]["startY"], clean_tasks[0]["endX"], clean_tasks[0]["endY"]),
            (0, 778, 452, 778),
        )
        self.assertEqual(
            (task_plan["tasks"][-1]["endX"], task_plan["tasks"][-1]["endY"]),
            (0, 0),
        )
        self.assertTrue(any(
            task.get("source") == "modeling_bridge_return"
            or "modeling_bridge_return" in (task.get("mergedSources") or [])
            for task in task_plan["tasks"]
        ))

    def test_explicit_area_capture_keeps_extra_boundary_points(self):
        from modeling_session import ModelingSession
        from modeling_store import ModelingStore

        provider = _PointProvider([
            _point("p1", 0, 0),
            _point("p2", 0, 452),
            _point("assist1", 10, 452),
            _point("assist2", 250, 452),
            _point("p9", 565, 452),
            _point("p10", 565, 0),
            _point("p3", 0, 470),
            _point("p4", 0, 534),
            _point("p5", 0, 552),
            _point("p6", 0, 778),
            _point("p7", 452, 778),
            _point("p8", 452, 552),
        ])
        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        session = ModelingSession(store, provider, now=lambda: 1000)
        session.start("extra-points")
        for _ in range(6):
            session.record_area_point()
        session.record_link_point()
        session.record_link_point()
        session.new_area()
        for _ in range(4):
            session.record_area_point()

        finished = session.finish()
        draft = store.get_draft(finished["modelId"])

        self.assertEqual(
            [point["id"] for point in draft["groups"][0]["points"]],
            ["p1", "p2", "assist1", "assist2", "p9", "p10"],
        )
        self.assertEqual(finished["taskPlan"]["summary"]["cleanTaskCount"], 12)

    def test_undo_and_clear_apply_to_selected_point_type(self):
        from modeling_session import ModelingSession
        from modeling_store import ModelingStore

        session = ModelingSession(
            ModelingStore(self.tmpdir, now=lambda: 1000),
            _PointProvider([
                _point("p1", 0, 0),
                _point("p2", 100, 0),
                _point("p3", 100, 100),
                _point("p4", 0, 100),
                _point("l1", 100, 50),
                _point("l2", 200, 50),
            ]),
            now=lambda: 1000,
        )
        session.start("undo-clear")
        for _ in range(4):
            session.record_area_point()
        session.record_link_point()
        session.record_link_point()

        undone = session.undo("link")
        cleared_link = session.clear("link")
        cleared_area = session.clear("area")

        self.assertEqual(undone["session"]["linkPointCount"], 1)
        self.assertEqual(cleared_link["session"]["linkPointCount"], 0)
        self.assertEqual(cleared_area["session"]["areaPointCount"], 0)

    def test_delete_area_point_by_id_removes_requested_point_and_resequences(self):
        from modeling_session import ModelingSession, ModelingSessionError
        from modeling_store import ModelingStore

        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        session = ModelingSession(
            store,
            _PointProvider([
                _point("p1", 0, 0),
                _point("p2", 100, 0),
                _point("p3", 100, 100),
            ]),
            now=lambda: 1000,
        )
        started = session.start("delete-by-id")
        for _ in range(3):
            session.record_area_point()

        deleted = session.delete_area_point("p2")
        draft = store.get_draft(started["modelId"])

        self.assertEqual(deleted["id"], "p2")
        self.assertEqual(
            [point["id"] for point in draft["groups"][0]["points"]],
            ["p1", "p3"],
        )
        self.assertEqual(
            [point["sequence"] for point in draft["groups"][0]["points"]],
            [1, 2],
        )
        self.assertEqual(
            [event["pointId"] for event in draft["captureSequence"]],
            ["p1", "p3"],
        )
        with self.assertRaises(ModelingSessionError):
            session.delete_area_point("missing")

    def test_delete_link_point_by_id_marks_connection_incomplete(self):
        from modeling_session import ModelingSession, ModelingSessionError
        from modeling_store import ModelingStore

        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        session = ModelingSession(
            store,
            _PointProvider([
                _point("p1", 0, 0),
                _point("p2", 100, 0),
                _point("p3", 100, 100),
                _point("p4", 0, 100),
                _point("l1", 100, 50),
                _point("l2", 200, 50),
            ]),
            now=lambda: 1000,
        )
        started = session.start("delete-link-by-id")
        for _ in range(4):
            session.record_area_point()
        session.record_link_point()
        session.record_link_point()

        deleted = session.delete_link_point("l1")
        draft = store.get_draft(started["modelId"])
        link = draft["groupLinks"][0]

        self.assertEqual(deleted["id"], "l1")
        self.assertEqual(deleted["pointType"], "link")
        self.assertEqual(link["status"], "draft")
        self.assertEqual([point["id"] for point in link["points"]], ["l2"])
        self.assertEqual(link["points"][0]["sequence"], 1)
        self.assertEqual(link["points"][0]["role"], "group_link_start")
        self.assertNotIn("l1", [
            event["pointId"] for event in draft["captureSequence"]
        ])
        with self.assertRaises(ModelingSessionError):
            session.delete_link_point("missing")

    def test_clear_all_removes_points_from_every_area_and_connection(self):
        from modeling_session import ModelingSession
        from modeling_store import ModelingStore

        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        session = ModelingSession(
            store,
            _PointProvider([
                _point("a1", 0, 0),
                _point("a2", 100, 0),
                _point("a3", 100, 100),
                _point("a4", 0, 100),
                _point("l1", 100, 50),
                _point("l2", 200, 50),
                _point("b1", 200, 0),
                _point("b2", 300, 0),
                _point("b3", 300, 100),
                _point("b4", 200, 100),
                _point("reset1", 0, 0),
            ]),
            now=lambda: 1000,
        )
        started = session.start("clear-all")
        for _ in range(4):
            session.record_area_point()
        session.record_link_point()
        session.record_link_point()
        session.new_area()
        for _ in range(4):
            session.record_area_point()

        cleared_area = session.clear_all("area")
        area_draft = store.get_draft(started["modelId"])

        self.assertEqual(cleared_area["clearedPointCount"], 8)
        self.assertEqual(cleared_area["session"]["totalAreaPointCount"], 0)
        self.assertEqual(cleared_area["session"]["totalLinkPointCount"], 2)
        self.assertEqual(
            [group["points"] for group in area_draft["groups"]],
            [[], []],
        )
        self.assertEqual(
            [event["pointId"] for event in area_draft["captureSequence"]],
            ["l1", "l2"],
        )

        cleared_link = session.clear_all("link")
        final_draft = store.get_draft(started["modelId"])

        self.assertEqual(cleared_link["clearedPointCount"], 2)
        self.assertEqual(cleared_link["session"]["totalLinkPointCount"], 0)
        self.assertTrue(cleared_link["resetToFirstArea"])
        self.assertEqual(cleared_link["session"]["currentAreaNumber"], 1)
        self.assertEqual(cleared_link["session"]["groupCount"], 1)
        self.assertEqual(cleared_link["session"]["linkCount"], 0)
        self.assertEqual(final_draft["groupLinks"], [])
        self.assertEqual(len(final_draft["groups"]), 1)
        self.assertEqual(final_draft["groups"][0]["areaNumber"], 1)
        self.assertEqual(final_draft["captureSequence"], [])

        recorded_again = session.record_area_point()
        self.assertEqual(recorded_again["point"]["id"], "reset1")
        self.assertEqual(recorded_again["point"]["areaNumber"], 1)

    def test_clear_all_resets_to_first_area_in_reverse_command_order(self):
        from modeling_session import ModelingSession
        from modeling_store import ModelingStore

        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        session = ModelingSession(
            store,
            _PointProvider([
                _point("a1", 0, 0),
                _point("a2", 100, 0),
                _point("a3", 100, 100),
                _point("a4", 0, 100),
                _point("l1", 100, 50),
                _point("l2", 200, 50),
                _point("b1", 200, 0),
                _point("again1", 0, 0),
            ]),
            now=lambda: 1000,
        )
        started = session.start("clear-all-reverse")
        for _ in range(4):
            session.record_area_point()
        session.record_link_point()
        session.record_link_point()
        session.new_area()
        session.record_area_point()

        cleared_link = session.clear_all("link")
        cleared_area = session.clear_all("area")
        current = session.current()
        draft = store.get_draft(started["modelId"])

        self.assertFalse(cleared_link["resetToFirstArea"])
        self.assertTrue(cleared_area["resetToFirstArea"])
        self.assertEqual(current["modelId"], started["modelId"])
        self.assertEqual(current["currentAreaNumber"], 1)
        self.assertEqual(current["groupCount"], 1)
        self.assertEqual(current["linkCount"], 0)
        self.assertEqual(draft["taskPreview"], None)
        self.assertEqual(draft["taskPlan"], None)

        recorded_again = session.record_area_point()
        self.assertEqual(recorded_again["point"]["id"], "again1")
        self.assertEqual(recorded_again["point"]["areaNumber"], 1)


if __name__ == "__main__":
    unittest.main()
