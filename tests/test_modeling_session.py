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

    def test_connection_clicks_keep_original_two_endpoint_rule_and_open_next_area(self):
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
        self.assertEqual(second_link["session"]["groupCount"], 2)

        with self.assertRaises(InvalidModelPayloadError):
            session.record_link_point()

        next_area = session.record_area_point()
        self.assertEqual(next_area["point"]["id"], "b1")
        self.assertEqual(next_area["session"]["currentAreaNumber"], 2)
        self.assertEqual(next_area["session"]["areaPointCount"], 1)

        for _ in range(3):
            session.record_area_point()
        finished = session.finish()
        self.assertEqual(finished["session"]["groupCount"], 2)
        self.assertEqual(finished["session"]["linkCount"], 1)
        self.assertEqual(finished["taskPlan"]["status"], "ready")
        self.assertGreater(finished["taskPlan"]["summary"]["transferTaskCount"], 0)

    def test_mixed_boundary_capture_generates_remote_first_round_trip(self):
        from modeling_session import ModelingSession
        from modeling_store import ModelingStore

        provider = _PointProvider([
            _point("p1", 0, 0),
            _point("p2", 0, 452),
            _point("p3", 0, 470),
            _point("p4", 0, 534),
            _point("p5", 0, 552),
            _point("p6", 0, 778),
            _point("p7", 452, 778),
            _point("p8", 452, 552),
            _point("p9", 565, 452),
            _point("p10", 565, 0),
        ])
        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        session = ModelingSession(store, provider, now=lambda: 1000)
        session.start("confirmed-route")

        session.record_area_point()
        session.record_area_point()
        session.record_link_point()
        session.record_link_point()
        for _ in range(6):
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
            (0, 452),
        )
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
        self.assertTrue(any(task["source"] == "modeling_bridge_return" for task in task_plan["tasks"]))

    def test_mixed_boundary_capture_keeps_extra_home_boundary_points(self):
        from modeling_session import ModelingSession
        from modeling_store import ModelingStore

        provider = _PointProvider([
            _point("p1", 0, 0),
            _point("p2", 0, 452),
            _point("p3", 0, 470),
            _point("p4", 0, 534),
            _point("p5", 0, 552),
            _point("p6", 0, 778),
            _point("p7", 452, 778),
            _point("p8", 452, 552),
            _point("assist1", 10, 452),
            _point("assist2", 250, 452),
            _point("p9", 565, 452),
            _point("p10", 565, 0),
        ])
        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        session = ModelingSession(store, provider, now=lambda: 1000)
        session.start("extra-points")
        session.record_area_point()
        session.record_area_point()
        session.record_link_point()
        session.record_link_point()
        for _ in range(8):
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


if __name__ == "__main__":
    unittest.main()
