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


def _rtk_point(point_id, lat, lon):
    """Match real vehicle samples: RTK is present and x/y has not been derived."""
    return {
        "id": point_id,
        "lat": lat,
        "lon": lon,
        "x": None,
        "y": None,
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
            _point("l3", 700, 200),
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
        third_link = session.record_link_point()

        self.assertEqual(first_link["point"]["role"], "group_link_start")
        self.assertEqual(second_link["point"]["role"], "group_link_end")
        self.assertEqual(third_link["point"]["role"], "group_link_end")
        self.assertEqual(third_link["session"]["linkPointCount"], 3)
        self.assertEqual(third_link["session"]["groupCount"], 1)
        draft = session.store.get_draft(session.current()["modelId"])
        self.assertEqual(
            [point["role"] for point in draft["groupLinks"][0]["points"]],
            ["group_link_start", "group_link_waypoint", "group_link_end"],
        )

        created_area = session.new_area()
        self.assertEqual(created_area, {
            "areaNumber": 2,
            "sourceAreaNumber": 1,
            "groupCount": 2,
        })
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

    def test_explicit_new_link_selects_bridge_without_sampling_a_point(self):
        from modeling_session import ModelingSession, ModelingSessionError
        from modeling_store import ModelingStore

        provider = _PointProvider([
            _point("a1", 0, 0),
            _point("a2", 0, 200),
            _point("a3", 400, 200),
            _point("a4", 400, 0),
            _point("l1", 0, 220),
            _point("l2", 0, 280),
            _point("b1", 0, 300),
            _point("b2", 0, 500),
            _point("b3", 300, 500),
            _point("b4", 300, 300),
        ])
        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        session = ModelingSession(store, provider, now=lambda: 1000)
        started = session.start("explicit-new-link")

        for _ in range(3):
            session.record_area_point()
        with self.assertRaises(ModelingSessionError) as incomplete_area:
            session.new_link()
        self.assertEqual(incomplete_area.exception.code, "MODELING_AREA_INCOMPLETE")

        session.record_area_point()
        created = session.new_link()
        created_again = session.new_link()
        draft_before_sampling = store.get_draft(started["modelId"])

        self.assertEqual(created["linkNumber"], 1)
        self.assertEqual(created["linkPointCount"], 0)
        self.assertEqual(created_again["linkNumber"], 1)
        self.assertEqual(draft_before_sampling["groupLinks"], [])
        with self.assertRaises(ModelingSessionError) as wrong_point_type:
            session.record_area_point()
        self.assertEqual(wrong_point_type.exception.code, "MODELING_LINK_POINT_REQUIRED")

        first = session.record_link_point()
        second = session.record_link_point()
        self.assertEqual(first["point"]["id"], "l1")
        self.assertEqual(first["linkNumber"], 1)
        self.assertEqual(second["linkNumber"], 1)

        session.new_area()
        for _ in range(4):
            session.record_area_point()
        second_bridge = session.new_link()
        self.assertEqual(second_bridge["linkNumber"], 2)
        self.assertEqual(second_bridge["session"]["currentLinkNumber"], 2)

    def test_connection_source_uses_nearest_area_for_branched_topology(self):
        """区域2完成后回到区域1记录连接点，应创建区域1到区域3的桥。"""
        from modeling_session import ModelingSession
        from modeling_store import ModelingStore

        provider = _PointProvider([
            # 区域1
            _point("a1", 0, 0),
            _point("a2", 0, 200),
            _point("a3", 400, 200),
            _point("a4", 400, 0),
            # 区域1 -> 区域2
            _point("l1", 0, 220),
            _point("l2", 0, 280),
            # 区域2
            _point("b1", 0, 300),
            _point("b2", 0, 500),
            _point("b3", 300, 500),
            _point("b4", 300, 300),
            # 当前区域虽为2，但这里已经驶回区域1右上角。
            _point("l3", 400, 200),
            _point("l4", 470, 230),
            # 区域3
            _point("c1", 500, 200),
            _point("c2", 500, 400),
            _point("c3", 800, 400),
            _point("c4", 800, 200),
        ])
        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        session = ModelingSession(store, provider, now=lambda: 1000)
        started = session.start("branched-three-areas")

        for _ in range(4):
            session.record_area_point()
        first_bridge_start = session.record_link_point()
        session.record_link_point()
        first_new_area = session.new_area()
        for _ in range(4):
            session.record_area_point()

        second_bridge_start = session.record_link_point()
        second_bridge_end = session.record_link_point()
        third_area = session.new_area()
        for _ in range(4):
            session.record_area_point()

        replanned = session.replan([1, 2, 3])
        saved = store.get_draft(started["modelId"])
        group_number_by_id = {
            group["id"]: group["areaNumber"]
            for group in saved["groups"]
        }
        topology = [
            (
                group_number_by_id[link["startGroupId"]],
                group_number_by_id[link["endGroupId"]],
            )
            for link in saved["groupLinks"]
        ]

        self.assertEqual(first_bridge_start["sourceAreaNumber"], 1)
        self.assertEqual(first_new_area["sourceAreaNumber"], 1)
        self.assertEqual(second_bridge_start["sourceAreaNumber"], 1)
        self.assertEqual(second_bridge_end["sourceAreaNumber"], 1)
        self.assertEqual(third_area["sourceAreaNumber"], 1)
        self.assertEqual(topology, [(1, 2), (1, 3)])
        self.assertEqual(replanned["areaOrder"], [1, 2, 3])
        self.assertEqual(replanned["taskPlan"]["status"], "ready")
        self.assertEqual(
            [selection["areaNumber"] for selection in replanned["taskPlan"]["routeSelections"]],
            [1, 2, 3],
        )

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

    def test_explicit_two_area_capture_defaults_to_area_one_then_area_two(self):
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
        self.assertEqual(task_plan["routeType"], "area_order")
        self.assertEqual(task_plan["areaOrder"], [1, 2])
        self.assertEqual(task_plan["summary"]["cleanTaskCount"], 12)
        self.assertEqual(
            (task_plan["tasks"][0]["startX"], task_plan["tasks"][0]["startY"]),
            (0, 0),
        )
        self.assertEqual(
            (task_plan["tasks"][0]["endX"], task_plan["tasks"][0]["endY"]),
            (565, 0),
        )
        clean_tasks = [task for task in task_plan["tasks"] if task["mode"] == 1]
        self.assertEqual([task["areaNumber"] for task in clean_tasks], [1] * 8 + [2] * 4)
        self.assertEqual(
            (clean_tasks[0]["startX"], clean_tasks[0]["startY"], clean_tasks[0]["endX"], clean_tasks[0]["endY"]),
            (0, 0, 565, 0),
        )
        self.assertEqual(
            (task_plan["tasks"][-1]["endX"], task_plan["tasks"][-1]["endY"]),
            (0, 0),
        )
        self.assertTrue(any(
            task.get("source") == "modeling_return_origin"
            or "modeling_return_origin" in (task.get("mergedSources") or [])
            for task in task_plan["tasks"]
        ))

    def test_ready_session_can_replan_using_frontend_area_order(self):
        from modeling_session import ModelingSession
        from modeling_store import ModelingStore

        provider = _PointProvider([
            _point("a1", 0, 0),
            _point("a2", 0, 200),
            _point("a3", 300, 200),
            _point("a4", 300, 0),
            _point("l1", 0, 200),
            _point("l2", 0, 300),
            _point("b1", 0, 300),
            _point("b2", 0, 500),
            _point("b3", 300, 500),
            _point("b4", 300, 300),
        ])
        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        session = ModelingSession(store, provider, now=lambda: 1000)
        session.start("reordered-route")
        for _ in range(4):
            session.record_area_point()
        session.record_link_point()
        session.record_link_point()
        session.new_area()
        for _ in range(4):
            session.record_area_point()

        finished = session.finish()
        replanned = session.replan([2, 1])
        saved = store.get_draft(finished["modelId"])

        self.assertEqual(finished["areaOrder"], [1, 2])
        self.assertEqual(replanned["areaOrder"], [2, 1])
        self.assertEqual(replanned["taskPlan"]["areaOrder"], [2, 1])
        self.assertEqual(replanned["taskPreview"]["areaOrder"], [2, 1])
        self.assertEqual(saved["routePolicy"], {
            "type": "area_order",
            "areaOrder": [2, 1],
        })
        clean_areas = [
            task["areaNumber"]
            for task in replanned["taskPlan"]["tasks"]
            if task["mode"] == 1
        ]
        first_area_one = clean_areas.index(1)
        self.assertTrue(all(area == 2 for area in clean_areas[:first_area_one]))
        self.assertTrue(all(area == 1 for area in clean_areas[first_area_one:]))

    def test_recording_session_can_replan_as_first_route_generation_command(self):
        """新前端确认区域顺序后，无需先调用 finish_modeling。"""
        from modeling_session import ModelingSession
        from modeling_store import ModelingStore

        provider = _PointProvider([
            _point("a1", 0, 0),
            _point("a2", 0, 200),
            _point("a3", 300, 200),
            _point("a4", 300, 0),
            _point("l1", 0, 200),
            _point("l2", 0, 300),
            _point("b1", 0, 300),
            _point("b2", 0, 500),
            _point("b3", 300, 500),
            _point("b4", 300, 300),
        ])
        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        session = ModelingSession(store, provider, now=lambda: 1000)
        started = session.start("first-replan")
        for _ in range(4):
            session.record_area_point()
        session.record_link_point()
        session.record_link_point()
        session.new_area()
        for _ in range(4):
            session.record_area_point()

        # 不调用 finish()，直接按前端确认的顺序完成首次区域识别和路线生成。
        replanned = session.replan([2, 1])
        saved = store.get_draft(started["modelId"])

        self.assertEqual(replanned["session"]["status"], "ready")
        self.assertEqual(replanned["areaOrder"], [2, 1])
        self.assertEqual(replanned["taskPreview"]["status"], "ready")
        self.assertEqual(replanned["taskPreview"]["areaOrder"], [2, 1])
        self.assertEqual(replanned["taskPlan"]["status"], "ready")
        self.assertEqual(replanned["taskPlan"]["areaOrder"], [2, 1])
        self.assertTrue(saved["recognition"]["confirmed"])
        self.assertTrue(all(group.get("subAreas") for group in saved["groups"]))

        clean_areas = [
            task["areaNumber"]
            for task in replanned["taskPlan"]["tasks"]
            if task["mode"] == 1
        ]
        first_area_one = clean_areas.index(1)
        self.assertTrue(all(area == 2 for area in clean_areas[:first_area_one]))
        self.assertTrue(all(area == 1 for area in clean_areas[first_area_one:]))

    def test_test12_rtk_points_share_one_model_coordinate_frame(self):
        """Regression: area two must stay above area one after route planning."""
        from modeling_session import ModelingSession
        from modeling_store import ModelingStore

        provider = _PointProvider([
            _rtk_point("a1", 32.036476871000005, 118.924488329),
            _rtk_point("a2", 32.03648831899999, 118.92448946500001),
            _rtk_point("a3", 32.036481553, 118.92458954699998),
            _rtk_point("a4", 32.036470035, 118.92458870099999),
            _rtk_point("l1", 32.036492853, 118.92449166799997),
            _rtk_point("l2", 32.036497688, 118.92449191199998),
            _rtk_point("a5", 32.03650237, 118.92449218099998),
            _rtk_point("a6", 32.036514266, 118.924492622),
            _rtk_point("a7", 32.036511214, 118.924531126),
            _rtk_point("a8", 32.036499099000004, 118.92452993699999),
        ])
        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        session = ModelingSession(store, provider, now=lambda: 1000)
        session.start("test12-coordinate-regression")
        for _ in range(4):
            session.record_area_point()
        first_link = session.record_link_point()
        second_link = session.record_link_point()
        self.assertGreater(first_link["point"]["y"], 150.0)
        self.assertGreater(second_link["point"]["y"], first_link["point"]["y"])
        session.new_area()
        for _ in range(4):
            session.record_area_point()

        finished = session.finish()
        draft = store.get_draft(finished["modelId"])
        self.assertEqual(draft["coordinateFrame"]["type"], "model_origin")
        self.assertEqual(draft["coordinateFrame"]["originPointId"], "a1")

        area_one = draft["groups"][0]["points"]
        area_two = draft["groups"][1]["points"]
        link = draft["groupLinks"][0]["points"]
        self.assertAlmostEqual(area_one[0]["x"], 0.0, places=3)
        self.assertAlmostEqual(area_one[0]["y"], 0.0, places=3)
        self.assertGreater(min(point["y"] for point in area_two), 240.0)
        self.assertGreater(min(point["y"] for point in link), 150.0)
        self.assertLess(max(point["y"] for point in link), min(point["y"] for point in area_two))

        clean_area_two = [
            task for task in finished["taskPlan"]["tasks"]
            if task["mode"] == 1 and task["areaNumber"] == 2
        ]
        self.assertTrue(clean_area_two)
        self.assertGreater(
            min(min(task["startY"], task["endY"]) for task in clean_area_two),
            240,
        )
        self.assertGreater(
            max(max(task["startY"], task["endY"]) for task in clean_area_two),
            380,
        )
        self.assertEqual(
            (finished["taskPlan"]["tasks"][-1]["endX"], finished["taskPlan"]["tasks"][-1]["endY"]),
            (0, 0),
        )

        # Simulate a draft saved by the previous implementation: area two has
        # its own local origin and carries no coordinate-frame marker.  The
        # next recognition/build cycle must migrate it without new RTK samples.
        legacy = store.get_draft(finished["modelId"])
        legacy.pop("coordinateFrame", None)
        remote_origin_x = legacy["groups"][1]["points"][0]["x"]
        remote_origin_y = legacy["groups"][1]["points"][0]["y"]
        for point in legacy["groups"][1]["points"]:
            point["x"] = round(point["x"] - remote_origin_x, 3)
            point["y"] = round(point["y"] - remote_origin_y, 3)
        store.save_draft(finished["modelId"], legacy)

        migrated = store.recognize_group(
            finished["modelId"],
            legacy["groups"][1]["id"],
        )["draft"]
        migrated_remote = migrated["groups"][1]["points"]
        self.assertGreater(migrated_remote[0]["x"], 30.0)
        self.assertGreater(migrated_remote[0]["y"], 280.0)

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
