# coding=utf-8
import unittest


class ModelingTaskGeneratorTest(unittest.TestCase):
    def test_route_cost_key_ignores_interpreter_level_float_noise(self):
        from modeling_task_generator import _stable_cost_key

        # Python 3 and the vehicle's Python 2.7 can produce these two values
        # for the same symmetric route.  They must remain a deterministic tie.
        self.assertEqual(
            _stable_cost_key(236.53595992744647),
            _stable_cost_key(236.53595992744644),
        )

    def test_round_int_uses_same_half_away_from_zero_rule_on_python2_and_python3(self):
        from modeling_task_generator import _round_int

        self.assertEqual(_round_int(320.5), 321)
        self.assertEqual(_round_int(320.49), 320)
        self.assertEqual(_round_int(-61.5), -62)
        self.assertEqual(_round_int(-61.49), -61)

    def test_area_transition_chooses_shorter_direction_around_recorded_boundary(self):
        from modeling_task_generator import _CoordinateMapper, _group_anchor_transition_points

        points = [
            {"id": "p1", "x": 0, "y": 0, "lat": 32.0, "lon": 118.0},
            {"id": "p2", "x": 0, "y": 120, "lat": 32.0, "lon": 118.0},
            {"id": "p3", "x": 340, "y": 90, "lat": 32.0, "lon": 118.0},
            {"id": "p4", "x": 340, "y": -20, "lat": 32.0, "lon": 118.0},
        ]
        draft = {"groups": [{"id": "g1", "points": points}]}
        mapper = _CoordinateMapper(draft)

        path = _group_anchor_transition_points(
            draft,
            "g1",
            (0, 0),
            (340, 90),
            mapper,
        )

        self.assertEqual(path, [(0.0, 0.0), (340.0, -20.0), (340.0, 90.0)])

    def test_connection_points_enter_boundary_through_the_nearest_recorded_anchors(self):
        """连接点不得为缩短距离而斜穿到同一条边的远端角点。"""
        from modeling_task_generator import (
            _CoordinateMapper,
            _compact_executable_tasks,
            _group_anchor_transition_points,
        )

        points = [
            {"id": "A1", "x": 0.0, "y": 0.0, "lat": 32.0, "lon": 118.0},
            {"id": "A2", "x": 10.708, "y": 127.296, "lat": 32.0, "lon": 118.0},
            {"id": "A3", "x": 954.092, "y": 52.061, "lat": 32.0, "lon": 118.0},
            {"id": "A4", "x": 946.118, "y": -76.013, "lat": 32.0, "lon": 118.0},
        ]
        draft = {"groups": [{"id": "area-1", "points": points}]}
        mapper = _CoordinateMapper(draft)

        outbound = _group_anchor_transition_points(
            draft,
            "area-1",
            (31.474, 177.712),  # L1，靠近 A2
            (957.0, 105.0),     # L3，靠近 A3
            mapper,
        )
        inbound = _group_anchor_transition_points(
            draft,
            "area-1",
            (957.0, 105.0),
            (31.474, 177.712),
            mapper,
        )

        self.assertEqual(outbound, [
            (31.474, 177.712),
            (10.708, 127.296),
            (954.092, 52.061),
            (957.0, 105.0),
        ])
        self.assertEqual(inbound, list(reversed(outbound)))

        # 后续普通移动点简化也必须保留 A2、A3 这两个明显转向点。
        raw_tasks = []
        for index in range(len(outbound) - 1):
            start = outbound[index]
            end = outbound[index + 1]
            raw_tasks.append({
                "id": index + 1,
                "mode": 2,
                "startX": start[0],
                "startY": start[1],
                "endX": end[0],
                "endY": end[1],
                "startLat": 32.0,
                "startLon": 118.0,
                "endLat": 32.0,
                "endLon": 118.0,
                "source": "modeling_transfer",
            })
        compacted = _compact_executable_tasks(raw_tasks)
        executable_points = {
            (task[prefix + "X"], task[prefix + "Y"])
            for task in compacted
            for prefix in ("start", "end")
        }
        self.assertIn((10.708, 127.296), executable_points)
        self.assertIn((954.092, 52.061), executable_points)

    def test_connection_bridge_at_edge_middle_uses_boundary_projection_before_corner(self):
        """桥头位于边中部时，必须先接上边界，再沿边界前往清扫入口。"""
        from modeling_task_generator import (
            _CoordinateMapper,
            _group_anchor_transition_points,
            _nearest_boundary_projection,
        )

        points = [
            {"id": "A9", "x": 774.092, "y": 212.061, "lat": 32.0, "lon": 118.0},
            {"id": "A10", "x": 778.249, "y": 344.338, "lat": 32.0, "lon": 118.0},
            {"id": "A11", "x": 1141.192, "y": 310.402, "lat": 32.0, "lon": 118.0},
            {"id": "A12", "x": 1129.985, "y": 175.689, "lat": 32.0, "lon": 118.0},
        ]
        draft = {"groups": [{"id": "area-3", "points": points}]}
        mapper = _CoordinateMapper(draft)
        anchors = [mapper.point_to_xy(point) for point in points]
        bridge = (965.0, 165.0)
        projection = _nearest_boundary_projection(bridge, anchors)["point"]

        outbound = _group_anchor_transition_points(
            draft,
            "area-3",
            bridge,
            anchors[2],  # A11：区域3第一条清扫线入口
            mapper,
        )
        inbound = _group_anchor_transition_points(
            draft,
            "area-3",
            anchors[3],  # A12：区域3最后一条清扫线出口
            bridge,
            mapper,
        )

        # 去程：L4 -> 边界投影Q -> A12 -> A11。
        self.assertEqual(len(outbound), 4)
        self.assertEqual(outbound[0], bridge)
        self.assertAlmostEqual(outbound[1][0], projection[0], places=6)
        self.assertAlmostEqual(outbound[1][1], projection[1], places=6)
        self.assertEqual(outbound[2:], [anchors[3], anchors[2]])

        # 返程：A12 -> 同一个投影Q -> L4，和去程使用相同边界接入点。
        self.assertEqual(len(inbound), 3)
        self.assertEqual(inbound[0], anchors[3])
        self.assertAlmostEqual(inbound[1][0], projection[0], places=6)
        self.assertAlmostEqual(inbound[1][1], projection[1], places=6)
        self.assertEqual(inbound[2], bridge)

    def test_bridge_projection_splits_edge_instead_of_visiting_corner_and_backtracking(self):
        """连接桥落在A2--A3边中部时，应从A2直达投影点，不能先到A3再折返。"""
        from modeling_task_generator import (
            _CoordinateMapper,
            _group_anchor_transition_points,
            _nearest_boundary_projection,
        )

        def point(point_id, x, y):
            return {
                "id": point_id,
                "x": x,
                "y": y,
                "lat": 32.0,
                "lon": 118.0,
            }

        # “三区域测试1”中区域1和区域3连接位置的实际坐标。L3在区域1
        # 的A2--A3边外侧，最近边界点Q应把这条边切开。
        anchors = [
            point("A1", 0.0, 0.0),
            point("A2", 9.869, 121.403),
            point("A3", 936.484, 44.456),
            point("A4", 924.522, -83.763),
        ]
        draft = {
            "groups": [{"id": "g1", "areaNumber": 1, "points": anchors}],
            "groupLinks": [],
        }
        mapper = _CoordinateMapper(draft)
        origin = mapper.point_to_xy(anchors[0])
        bridge = (823.503, 103.222)
        recorded_xy = [mapper.point_to_xy(item) for item in anchors]
        projection = _nearest_boundary_projection(bridge, recorded_xy)["point"]

        path = _group_anchor_transition_points(
            draft,
            "g1",
            origin,
            bridge,
            mapper,
        )

        self.assertEqual(path[0], origin)
        self.assertEqual(path[1], mapper.point_to_xy(anchors[1]))
        self.assertAlmostEqual(path[2][0], projection[0], places=6)
        self.assertAlmostEqual(path[2][1], projection[1], places=6)
        self.assertEqual(path[3], bridge)
        self.assertEqual(len(path), 4)
        self.assertNotIn(mapper.point_to_xy(anchors[2]), path)

    def test_collinear_same_mode_segments_are_compacted_without_crossing_turns_or_mode_changes(self):
        from modeling_task_generator import _compact_executable_tasks

        def task(task_id, mode, start, end):
            return {
                "id": task_id,
                "mode": mode,
                "startX": start[0],
                "startY": start[1],
                "endX": end[0],
                "endY": end[1],
                "startLat": 32.0,
                "startLon": 118.0,
                "endLat": 32.0,
                "endLon": 118.0,
                "source": "test",
            }

        compacted = _compact_executable_tasks([
            task(1, 2, (0, 0), (0, 100)),
            task(2, 2, (0, 100), (0, 200)),
            task(3, 2, (0, 200), (100, 200)),
            task(4, 1, (100, 200), (200, 200)),
        ])

        self.assertEqual(len(compacted), 3)
        self.assertEqual(
            (compacted[0]["startX"], compacted[0]["startY"], compacted[0]["endX"], compacted[0]["endY"]),
            (0, 0, 0, 200),
        )
        self.assertEqual(compacted[0]["mergedSegmentCount"], 2)
        self.assertEqual(
            (compacted[1]["startX"], compacted[1]["startY"], compacted[1]["endX"], compacted[1]["endY"]),
            (0, 200, 100, 200),
        )
        self.assertEqual(compacted[2]["mode"], 1)
        self.assertEqual([item["id"] for item in compacted], [1, 2, 3])

    def test_test12_nearly_straight_outbound_points_become_one_executable_segment(self):
        from modeling_task_generator import _compact_executable_tasks

        points = [
            (0, 0),
            (11, 127),
            (31, 178),
            (34, 231),
            (36, 284),
            (40, 416),
        ]
        tasks = []
        for index in range(len(points) - 1):
            tasks.append({
                "id": index + 1,
                "mode": 2,
                "startX": points[index][0],
                "startY": points[index][1],
                "endX": points[index + 1][0],
                "endY": points[index + 1][1],
                "startLat": 32.0,
                "startLon": 118.0,
                "endLat": 32.0,
                "endLon": 118.0,
                "source": "modeling_outbound" if index < 3 else "modeling_transfer",
            })

        compacted = _compact_executable_tasks(tasks)

        self.assertEqual(len(compacted), 1)
        self.assertEqual(
            (
                compacted[0]["startX"],
                compacted[0]["startY"],
                compacted[0]["endX"],
                compacted[0]["endY"],
            ),
            (0, 0, 40, 416),
        )
        self.assertEqual(compacted[0]["mergedSegmentCount"], 5)
        self.assertEqual(
            compacted[0]["mergedSources"],
            ["modeling_outbound", "modeling_transfer"],
        )

    def test_ninety_degree_transition_corner_is_kept_as_stop_and_turn_point(self):
        from modeling_task_generator import _compact_executable_tasks

        def task(task_id, start, end):
            return {
                "id": task_id,
                "mode": 2,
                "startX": start[0],
                "startY": start[1],
                "endX": end[0],
                "endY": end[1],
                "startLat": 32.0,
                "startLon": 118.0,
                "endLat": 32.0,
                "endLon": 118.0,
                "source": "modeling_outbound",
            }

        compacted = _compact_executable_tasks([
            task(1, (0, 0), (0, 100)),
            task(2, (0, 100), (100, 100)),
        ])

        self.assertEqual(len(compacted), 2)
        self.assertEqual((compacted[0]["endX"], compacted[0]["endY"]), (0, 100))
        self.assertEqual((compacted[1]["startX"], compacted[1]["startY"]), (0, 100))

    def test_one_centimeter_pseudo_transfer_is_absorbed_without_losing_next_turn(self):
        """亚厘米原始误差取整成1cm时，不得让小车额外移动并停车一次。"""
        from modeling_task_generator import _compact_executable_tasks

        tasks = [
            {
                "id": 1, "mode": 2, "areaNumber": 2,
                "startX": 0, "startY": 0, "endX": 40, "endY": 416,
                "startLat": 32.0, "startLon": 118.0,
                "endLat": 32.00001, "endLon": 118.00001,
                "heading": 5.5, "angle": 5.5, "length": 418,
                "source": "modeling_start_to_first_lane",
            },
            {
                "id": 2, "mode": 2, "areaNumber": 2,
                "startX": 40, "startY": 416, "endX": 41, "endY": 416,
                "startLat": 32.00001, "startLon": 118.00001,
                "endLat": 32.00001, "endLon": 118.00002,
                "heading": 90.0, "angle": 90.0, "length": 1,
                "source": "modeling_start_to_first_lane",
            },
            {
                "id": 3, "mode": 1, "areaNumber": 2,
                "startX": 41, "startY": 416, "endX": 403, "endY": 382,
                "startLat": 32.00001, "startLon": 118.00002,
                "endLat": 32.00002, "endLon": 118.00003,
                "heading": 95.4, "angle": 95.4, "length": 364,
                "source": "modeling_clean",
            },
        ]

        compacted = _compact_executable_tasks(tasks)

        self.assertEqual(len(compacted), 2)
        self.assertEqual((compacted[0]["endX"], compacted[0]["endY"]), (40, 416))
        self.assertEqual((compacted[1]["startX"], compacted[1]["startY"]), (40, 416))
        self.assertEqual((compacted[1]["endX"], compacted[1]["endY"]), (403, 382))
        self.assertEqual(compacted[1]["mode"], 1)
        self.assertAlmostEqual(compacted[1]["heading"], 95.4, places=1)

    def test_three_centimeter_transfer_and_explicit_short_stop_are_preserved(self):
        """达到3cm的真实移动和显式停车标记都不能被近点规则删除。"""
        from modeling_task_generator import _compact_executable_tasks

        def task(task_id, start, end, preserve=False):
            return {
                "id": task_id, "mode": 2, "areaNumber": 1,
                "startX": start[0], "startY": start[1],
                "endX": end[0], "endY": end[1],
                "startLat": 32.0, "startLon": 118.0,
                "endLat": 32.0, "endLon": 118.0,
                "heading": 90.0, "angle": 90.0,
                "length": abs(end[0] - start[0]),
                "source": "test",
                "preserveEndStop": preserve,
            }

        three_cm = _compact_executable_tasks([
            task(1, (0, 0), (3, 0)),
            task(2, (3, 0), (3, 100), preserve=True),
        ])
        explicit_one_cm = _compact_executable_tasks([
            task(1, (0, 0), (1, 0), preserve=True),
            task(2, (1, 0), (1, 100)),
        ])

        self.assertEqual(len(three_cm), 2)
        self.assertEqual((three_cm[0]["startX"], three_cm[0]["endX"]), (0, 3))
        self.assertEqual(len(explicit_one_cm), 2)
        self.assertTrue(explicit_one_cm[0]["preserveEndStop"])

    def test_explicit_stop_boundary_is_not_removed_by_transition_simplification(self):
        from modeling_task_generator import _compact_executable_tasks

        tasks = [
            {
                "id": 1,
                "mode": 2,
                "startX": 0,
                "startY": 0,
                "endX": 2,
                "endY": 100,
                "startLat": 32.0,
                "startLon": 118.0,
                "endLat": 32.0,
                "endLon": 118.0,
                "source": "test",
                "preserveEndStop": True,
            },
            {
                "id": 2,
                "mode": 2,
                "startX": 2,
                "startY": 100,
                "endX": 4,
                "endY": 200,
                "startLat": 32.0,
                "startLon": 118.0,
                "endLat": 32.0,
                "endLon": 118.0,
                "source": "test",
            },
        ]

        compacted = _compact_executable_tasks(tasks)

        self.assertEqual(len(compacted), 2)
        self.assertTrue(compacted[0]["preserveEndStop"])

    def test_test12_full_plan_simplifies_outbound_and_bridge_return_only(self):
        from modeling_preview import build_model_preview
        from modeling_task_generator import generate_task_plan

        def point(point_id, x, y):
            return {
                "id": point_id,
                "x": x,
                "y": y,
                "lat": 32.0,
                "lon": 118.0,
            }

        home_points = [
            point("A1", 0.0, 0.0),
            point("A2", 10.708, 127.296),
            point("A3", 954.092, 52.061),
            point("A4", 946.118, -76.013),
        ]
        remote_points = [
            point("A5", 36.309, 283.536),
            point("A6", 40.466, 415.813),
            point("A7", 403.409, 381.877),
            point("A8", 392.202, 247.164),
        ]
        link_points = [
            point("L1", 31.474, 177.712),
            point("L2", 33.774, 231.474),
        ]
        draft = {
            "id": "test12-general-turn",
            "recognition": {"confirmed": True},
            "groups": [
                {
                    "id": "home",
                    "areaNumber": 1,
                    "points": home_points,
                    "subAreas": [{"id": "home-area", "pointIds": [p["id"] for p in home_points]}],
                },
                {
                    "id": "remote",
                    "areaNumber": 2,
                    "points": remote_points,
                    "subAreas": [{"id": "remote-area", "pointIds": [p["id"] for p in remote_points]}],
                },
            ],
            "groupLinks": [{
                "id": "bridge",
                "startGroupId": "home",
                "endGroupId": "remote",
                "status": "ready",
                "points": link_points,
            }],
        }
        draft["taskPreview"] = build_model_preview(draft, now=1000)

        plan = generate_task_plan(draft, now=2000)
        tasks = plan["tasks"]

        self.assertEqual(plan["routeType"], "area_order")
        self.assertEqual(plan["areaOrder"], [1, 2])
        self.assertEqual(tasks[0]["mode"], 1)
        self.assertEqual((tasks[0]["startX"], tasks[0]["startY"]), (0, 0))

        remote_clean_indexes = [
            index for index, task in enumerate(tasks)
            if task["mode"] == 1 and task["areaNumber"] == 2
        ]
        bridge_return = tasks[remote_clean_indexes[-1] + 1:]
        self.assertTrue(bridge_return)
        self.assertTrue(all(task["mode"] == 2 for task in bridge_return))
        self.assertEqual(
            (bridge_return[0]["startX"], bridge_return[0]["startY"]),
            (36, 284),
        )
        self.assertEqual(
            (bridge_return[-1]["endX"], bridge_return[-1]["endY"]),
            (0, 0),
        )

        connector_coordinates = {(31, 178), (34, 231)}
        executable_endpoints = {
            (task["startX"], task["startY"]) for task in tasks
        } | {
            (task["endX"], task["endY"]) for task in tasks
        }
        # 返回原点必须严格经过两个人工连接点，不能从区域2斜切回区域1。
        self.assertTrue(connector_coordinates.issubset(executable_endpoints))

    def test_test12_area_order_two_then_one_has_no_one_centimeter_pseudo_task(self):
        """test12按[2,1]执行时，A6直接作为区域2第一条清扫线起点。"""
        from modeling_preview import build_model_preview
        from modeling_task_generator import generate_task_plan

        def point(point_id, x, y):
            return {
                "id": point_id,
                "x": x,
                "y": y,
                "lat": 32.0,
                "lon": 118.0,
            }

        home_points = [
            point("A1", 0.0, 0.0),
            point("A2", 10.708, 127.296),
            point("A3", 954.092, 52.061),
            point("A4", 946.118, -76.013),
        ]
        remote_points = [
            point("A5", 36.309, 283.536),
            point("A6", 40.466, 415.813),
            point("A7", 403.409, 381.877),
            point("A8", 392.202, 247.164),
        ]
        link_points = [
            point("L1", 31.474, 177.712),
            point("L2", 33.774, 231.474),
        ]
        draft = {
            "id": "test12-area-order-2-1",
            "recognition": {"confirmed": True},
            "groups": [
                {
                    "id": "home",
                    "areaNumber": 1,
                    "points": home_points,
                    "subAreas": [{"id": "home-area", "pointIds": [p["id"] for p in home_points]}],
                },
                {
                    "id": "remote",
                    "areaNumber": 2,
                    "points": remote_points,
                    "subAreas": [{"id": "remote-area", "pointIds": [p["id"] for p in remote_points]}],
                },
            ],
            "groupLinks": [{
                "id": "bridge",
                "startGroupId": "home",
                "endGroupId": "remote",
                "status": "ready",
                "points": link_points,
            }],
            "routePolicy": {"type": "area_order", "areaOrder": [2, 1]},
        }
        draft["taskPreview"] = build_model_preview(draft, now=1000)

        plan = generate_task_plan(draft, now=2000)
        tasks = plan["tasks"]

        self.assertEqual(plan["areaOrder"], [2, 1])
        self.assertEqual(plan["summary"]["cleanTaskCount"], 8)
        first_clean_index = next(
            index for index, task in enumerate(tasks)
            if int(task.get("mode") or 0) == 1
        )
        self.assertEqual(
            (tasks[0]["startX"], tasks[0]["startY"]),
            (0, 0),
        )
        # A6与浮点求交起点相差不足3cm时，清扫线直接吸附到A6，不生成1cm伪转场。
        self.assertEqual(first_clean_index, 5)
        self.assertEqual(
            (
                tasks[first_clean_index]["startX"],
                tasks[first_clean_index]["startY"],
                tasks[first_clean_index]["endX"],
                tasks[first_clean_index]["endY"],
            ),
            (40, 416, 403, 382),
        )
        self.assertFalse([
            task for task in tasks
            if int(task.get("mode") or 0) == 2 and int(task.get("length") or 0) < 3
        ])
        self.assertTrue(all(
            (left["endX"], left["endY"]) == (right["startX"], right["startY"])
            for left, right in zip(tasks, tasks[1:])
        ))

    def test_generate_task_plan_starts_and_ends_at_first_recorded_point(self):
        from modeling_task_generator import generate_task_plan

        preview = {
            "status": "ready",
            "groups": [{
                "groupId": "g1",
                "areaNumber": 1,
                "subAreas": [{
                    "id": "sa1",
                    "lanes": [
                        {"id": "lane-1", "heading": 90, "startX": 0, "startY": 100, "endX": 100, "endY": 100, "lengthCm": 100},
                        {"id": "lane-2", "heading": 90, "startX": 0, "startY": 0, "endX": 100, "endY": 0, "lengthCm": 100},
                    ],
                }],
            }],
        }
        draft = {
            "id": "m1",
            "name": "model-a",
            "taskPreview": preview,
            "groups": [{
                "id": "g1",
                "points": [
                    {"id": "p1", "x": 0, "y": 0, "lat": 32.0, "lon": 118.0},
                    {"id": "p2", "x": 0, "y": 100, "lat": 32.000009, "lon": 118.0},
                    {"id": "p3", "x": 100, "y": 100, "lat": 32.000009, "lon": 118.0000106},
                    {"id": "p4", "x": 100, "y": 0, "lat": 32.0, "lon": 118.0000106},
                ],
            }],
        }

        task_plan = generate_task_plan(draft, now=2000)

        self.assertEqual(task_plan["status"], "ready")
        self.assertEqual(task_plan["generatedAt"], 2000)
        self.assertEqual(task_plan["summary"]["cleanTaskCount"], 2)
        self.assertEqual(task_plan["summary"]["transferTaskCount"], 2)
        self.assertEqual(len(task_plan["tasks"]), 4)
        first = task_plan["tasks"][0]
        self.assertEqual(first["id"], 1)
        self.assertEqual(first["mode"], 2)
        self.assertEqual(first["areaNumber"], 1)
        self.assertEqual((first["startX"], first["startY"]), (0, 0))
        self.assertEqual((first["endX"], first["endY"]), (0, 100))
        self.assertEqual(first["heading"], 0.0)
        self.assertEqual(first["length"], 100)
        self.assertEqual(first["source"], "modeling_start_to_first_lane")
        self.assertIn("startLat", first)
        self.assertIn("endLon", first)
        first_clean = task_plan["tasks"][1]
        self.assertEqual(first_clean["mode"], 1)
        self.assertEqual((first_clean["startX"], first_clean["startY"]), (0, 100))
        self.assertEqual((first_clean["endX"], first_clean["endY"]), (100, 100))
        self.assertEqual(first_clean["heading"], 90.0)
        transfer = task_plan["tasks"][2]
        self.assertEqual(transfer["mode"], 2)
        self.assertEqual((transfer["startX"], transfer["startY"]), (100, 100))
        self.assertEqual((transfer["endX"], transfer["endY"]), (100, 0))
        second_clean = task_plan["tasks"][3]
        self.assertEqual(second_clean["mode"], 1)
        self.assertEqual((second_clean["startX"], second_clean["startY"]), (100, 0))
        self.assertEqual((second_clean["endX"], second_clean["endY"]), (0, 0))
        self.assertEqual(second_clean["heading"], 270.0)
        for index in range(1, len(task_plan["tasks"])):
            previous = task_plan["tasks"][index - 1]
            current = task_plan["tasks"][index]
            self.assertEqual(
                (previous["endX"], previous["endY"]),
                (current["startX"], current["startY"]),
            )

    def test_generated_four_lane_rectangle_follows_expected_up_then_serpentine_route(self):
        from modeling_preview import build_model_preview
        from modeling_task_generator import generate_task_plan

        points = [
            {"id": "p1", "x": 0, "y": 0, "lat": 32.0, "lon": 118.0},
            {"id": "p2", "x": 0, "y": 226, "lat": 32.0000203, "lon": 118.0},
            {"id": "p3", "x": 339, "y": 226, "lat": 32.0000203, "lon": 118.0000359},
            {"id": "p4", "x": 339, "y": 0, "lat": 32.0, "lon": 118.0000359},
        ]
        draft = {
            "id": "four-lane-route",
            "recognition": {"confirmed": True, "groupId": "g1"},
            "groups": [{
                "id": "g1",
                "areaNumber": 1,
                "sweepDirection": "auto",
                "points": points,
                "subAreas": [{
                    "id": "sa1",
                    "pointIds": [point["id"] for point in points],
                }],
                "connectors": [],
            }],
            "groupLinks": [],
        }
        draft["taskPreview"] = build_model_preview(draft, now=1000)

        task_plan = generate_task_plan(draft, now=2000)
        tasks = task_plan["tasks"]

        self.assertEqual(task_plan["summary"]["cleanTaskCount"], 4)
        self.assertEqual([task["mode"] for task in tasks], [2, 1, 2, 1, 2, 1, 2, 1])
        self.assertEqual((tasks[0]["startX"], tasks[0]["startY"]), (0, 0))
        self.assertEqual((tasks[0]["endX"], tasks[0]["endY"]), (0, 226))
        self.assertEqual((tasks[-1]["endX"], tasks[-1]["endY"]), (0, 0))
        for index in range(1, len(tasks)):
            self.assertEqual(
                (tasks[index - 1]["endX"], tasks[index - 1]["endY"]),
                (tasks[index]["startX"], tasks[index]["startY"]),
            )

    def test_first_lane_is_oriented_from_the_second_recorded_boundary_point(self):
        from modeling_preview import build_model_preview
        from modeling_task_generator import generate_task_plan

        points = [
            {"id": "p1", "x": 0, "y": 0, "lat": 32.0, "lon": 118.0},
            {"id": "p2", "x": 226, "y": 0, "lat": 32.0, "lon": 118.0000239},
            {"id": "p3", "x": 226, "y": 339, "lat": 32.0000305, "lon": 118.0000239},
            {"id": "p4", "x": 0, "y": 339, "lat": 32.0000305, "lon": 118.0},
        ]
        draft = {
            "id": "horizontal-first-edge",
            "recognition": {"confirmed": True, "groupId": "g1"},
            "groups": [{
                "id": "g1",
                "areaNumber": 1,
                "sweepDirection": "auto",
                "points": points,
                "subAreas": [{"id": "sa1", "pointIds": [point["id"] for point in points]}],
            }],
            "groupLinks": [],
        }
        draft["taskPreview"] = build_model_preview(draft, now=1000)

        tasks = generate_task_plan(draft, now=2000)["tasks"]

        self.assertEqual(
            (tasks[0]["startX"], tasks[0]["startY"], tasks[0]["endX"], tasks[0]["endY"]),
            (0, 0, 226, 0),
        )
        self.assertEqual(
            (tasks[1]["startX"], tasks[1]["startY"]),
            (226, 0),
        )
        self.assertEqual(
            (tasks[-1]["endX"], tasks[-1]["endY"]),
            (0, 0),
        )

    def test_multiple_areas_use_connection_bridge_chain_and_return_to_origin(self):
        from modeling_task_generator import generate_task_plan

        def preview_group(group_id, area_number, start_x, end_x):
            return {
                "groupId": group_id,
                "areaNumber": area_number,
                "subAreas": [{
                    "id": "sa-{}".format(group_id),
                    "lanes": [{
                        "id": "lane-{}".format(group_id),
                        "startX": start_x,
                        "startY": 0,
                        "endX": end_x,
                        "endY": 0,
                    }],
                }],
            }

        preview = {
            "status": "ready",
            "groups": [
                preview_group("g1", 1, 0, 10),
                preview_group("g2", 2, 20, 30),
                preview_group("g3", 3, 40, 50),
            ],
            "groupLinks": [
                {
                    "id": "l12",
                    "startGroupId": "g1",
                    "endGroupId": "g2",
                    "points": [
                        {"id": "l12-a", "x": 10, "y": 0},
                        {"id": "l12-b", "x": 20, "y": 0},
                    ],
                },
                {
                    "id": "l23",
                    "startGroupId": "g2",
                    "endGroupId": "g3",
                    "points": [
                        {"id": "l23-a", "x": 30, "y": 0},
                        {"id": "l23-mid", "x": 35, "y": 5},
                        {"id": "l23-b", "x": 40, "y": 0},
                    ],
                },
            ],
        }
        draft = {
            "id": "three-area-route",
            "groups": [
                {"id": "g1", "points": [{"id": "p1", "x": 0, "y": 0, "lat": 32.0, "lon": 118.0}]},
                {"id": "g2", "points": [{"id": "p2", "x": 20, "y": 0, "lat": 32.0, "lon": 118.0}]},
                {"id": "g3", "points": [{"id": "p3", "x": 40, "y": 0, "lat": 32.0, "lon": 118.0}]},
            ],
            "taskPreview": preview,
        }

        task_plan = generate_task_plan(draft, now=2000)
        tasks = task_plan["tasks"]

        self.assertEqual(task_plan["summary"]["cleanTaskCount"], 3)
        self.assertEqual((tasks[0]["startX"], tasks[0]["startY"]), (0, 0))
        self.assertEqual((tasks[-1]["endX"], tasks[-1]["endY"]), (0, 0))
        self.assertTrue(any((task["startX"], task["startY"]) == (35, 5) for task in tasks))
        for index in range(1, len(tasks)):
            self.assertEqual(
                (tasks[index - 1]["endX"], tasks[index - 1]["endY"]),
                (tasks[index]["startX"], tasks[index]["startY"]),
            )

    def test_two_connected_groups_default_to_area_one_then_area_two_and_return_to_origin(self):
        from modeling_task_generator import generate_task_plan

        preview = {
            "status": "ready",
            "groups": [
                {
                    "groupId": "home",
                    "areaNumber": 1,
                    "subAreas": [{"lanes": [
                        {"id": "home-top", "startX": 0, "startY": 100, "endX": 100, "endY": 100},
                        {"id": "home-bottom", "startX": 0, "startY": 0, "endX": 100, "endY": 0},
                    ]}],
                },
                {
                    "groupId": "remote",
                    "areaNumber": 2,
                    "subAreas": [{"lanes": [
                        {"id": "remote-top", "startX": 0, "startY": 300, "endX": 100, "endY": 300},
                        {"id": "remote-bottom", "startX": 0, "startY": 200, "endX": 100, "endY": 200},
                    ]}],
                },
            ],
            "groupLinks": [{
                "id": "bridge",
                "startGroupId": "home",
                "endGroupId": "remote",
                "points": [
                    {"id": "link-home", "x": 0, "y": 100},
                    {"id": "link-remote", "x": 0, "y": 200},
                ],
            }],
        }
        draft = {
            "id": "two-groups",
            "groups": [
                {"id": "home", "points": [
                    {"id": "origin", "x": 0, "y": 0, "lat": 32.0, "lon": 118.0},
                ]},
                {"id": "remote", "points": [
                    {"id": "remote-point", "x": 0, "y": 200, "lat": 32.000018, "lon": 118.0},
                ]},
            ],
            "groupLinks": preview["groupLinks"],
            "taskPreview": preview,
        }

        plan = generate_task_plan(draft, now=2000)
        tasks = plan["tasks"]
        clean_areas = [task["areaNumber"] for task in tasks if task["mode"] == 1]

        self.assertEqual(plan["routeType"], "area_order")
        self.assertEqual(plan["areaOrder"], [1, 2])
        self.assertEqual(clean_areas, [1, 1, 2, 2])
        self.assertEqual((tasks[0]["startX"], tasks[0]["startY"]), (0, 0))
        self.assertEqual((tasks[-1]["endX"], tasks[-1]["endY"]), (0, 0))
        # 同一直线普通点被合并，不会在连接桥端点额外停车。
        self.assertEqual((tasks[0]["endX"], tasks[0]["endY"]), (100, 0))
        for index in range(1, len(tasks)):
            self.assertEqual(
                (tasks[index - 1]["endX"], tasks[index - 1]["endY"]),
                (tasks[index]["startX"], tasks[index]["startY"]),
            )

    def test_disconnected_areas_do_not_generate_an_unsafe_straight_transfer(self):
        from modeling_task_generator import ModelingTaskGenerationError, generate_task_plan

        draft = {
            "id": "disconnected-areas",
            "groups": [{
                "id": "g1",
                "points": [{"id": "p1", "x": 0, "y": 0, "lat": 32.0, "lon": 118.0}],
            }],
            "taskPreview": {
                "status": "ready",
                "groups": [
                    {
                        "groupId": "g1",
                        "areaNumber": 1,
                        "subAreas": [{
                            "lanes": [{"id": "lane-1", "startX": 0, "startY": 0, "endX": 10, "endY": 0}],
                        }],
                    },
                    {
                        "groupId": "g2",
                        "areaNumber": 2,
                        "subAreas": [{
                            "lanes": [{"id": "lane-2", "startX": 100, "startY": 0, "endX": 110, "endY": 0}],
                        }],
                    },
                ],
                "groupLinks": [],
            },
        }

        with self.assertRaises(ModelingTaskGenerationError):
            generate_task_plan(draft, now=2000)

    def test_boundary_lane_keeps_every_point_but_only_stops_for_hard_turns(self):
        from modeling_frontend import frontend_path_points
        from modeling_task_generator import (
            _CoordinateMapper,
            _append_clean_segments,
            _compact_executable_tasks,
        )

        draft = {
            "groups": [{
                "id": "g1",
                "points": [{
                    "id": "origin",
                    "x": 0,
                    "y": 0,
                    "lat": 32.0,
                    "lon": 118.0,
                }],
            }],
        }
        mapper = _CoordinateMapper(draft)
        tasks = []
        current, next_task_id, clean_count = _append_clean_segments(
            tasks,
            [{
                "groupId": "g1",
                "areaNumber": 1,
                "sourceId": "lane-boundary",
                "laneType": "boundary",
                "start": (0, 0),
                "end": (200, 100),
                # 前两个拐角约11度，应连续经过；最后一个拐角约95度，应停车转向。
                "path": [(0, 0), (100, 10), (200, 0), (200, 100)],
            }],
            None,
            mapper,
            1,
            draft,
        )
        tasks = _compact_executable_tasks(tasks)

        self.assertEqual(clean_count, 1)
        self.assertEqual(next_task_id, 4)
        self.assertEqual(current, (200, 100))
        self.assertEqual(len(tasks), 3)
        self.assertEqual(
            [(task["startX"], task["startY"], task["endX"], task["endY"]) for task in tasks],
            [(0, 0, 100, 10), (100, 10, 200, 0), (200, 0, 200, 100)],
        )
        self.assertEqual([task["turnAtStart"] for task in tasks], [True, False, True])
        self.assertEqual([task["stopAtEnd"] for task in tasks], [False, True, True])
        self.assertEqual(len(set(task["continuousPathId"] for task in tasks)), 1)
        # 前端接口不需要增加字段；它按既有tasks顺序展开后仍能看到全部折线点。
        self.assertEqual(
            [(point["x"], point["y"]) for point in frontend_path_points({"tasks": tasks})],
            [(0, 0), (100, 10), (200, 0), (200, 100)],
        )

    def test_lane_changes_preserve_every_recorded_short_side_point(self):
        from modeling_task_generator import (
            _CoordinateMapper,
            _append_clean_segments,
            _compact_executable_tasks,
        )

        def point(point_id, x, y):
            return {
                "id": point_id,
                "x": x,
                "y": y,
                "lat": 32.0,
                "lon": 118.0,
            }

        # 两条短边均包含反复浮动点。四条清扫线之间的三次换行必须沿短边逐点走，
        # 不能再被直接连成三条竖直线，也不能被20cm转场简化删除。
        recorded = [
            point("A1", 0, 0),
            point("L1", 8, 20),
            point("L2", -10, 40),
            point("L3", 0, 60),
            point("L4", 7, 80),
            point("L5", -9, 100),
            point("L6", 0, 120),
            point("L7", 8, 140),
            point("L8", -10, 160),
            point("A2", 0, 180),
            point("A3", 300, 180),
            point("R1", 310, 160),
            point("R2", 292, 140),
            point("R3", 300, 120),
            point("R4", 307, 100),
            point("R5", 291, 80),
            point("R6", 300, 60),
            point("R7", 308, 40),
            point("R8", 294, 20),
            point("A4", 300, 0),
        ]
        draft = {"groups": [{"id": "g1", "areaNumber": 1, "points": recorded}]}
        mapper = _CoordinateMapper(draft)
        segments = [
            {"groupId": "g1", "areaNumber": 1, "sourceId": "lane-1", "start": (0, 0), "end": (300, 0)},
            {"groupId": "g1", "areaNumber": 1, "sourceId": "lane-2", "start": (300, 60), "end": (0, 60)},
            {"groupId": "g1", "areaNumber": 1, "sourceId": "lane-3", "start": (0, 120), "end": (300, 120)},
            {"groupId": "g1", "areaNumber": 1, "sourceId": "lane-4", "start": (300, 180), "end": (0, 180)},
        ]

        tasks = []
        _append_clean_segments(tasks, segments, None, mapper, 1, draft)
        tasks = _compact_executable_tasks(tasks)

        transfer_tasks = [task for task in tasks if int(task.get("mode") or 0) == 2]
        transfer_endpoints = {
            (task["endX"], task["endY"])
            for task in transfer_tasks
        }
        expected_side_points = {
            (294, 20), (308, 40), (300, 60),
            (7, 80), (-9, 100), (0, 120),
            (292, 140), (310, 160), (300, 180),
        }
        self.assertTrue(expected_side_points.issubset(transfer_endpoints))
        self.assertTrue(all(task.get("preserveRecordedPath") for task in transfer_tasks))
        self.assertTrue(all(task.get("continuousPathId") for task in transfer_tasks))

    def test_all_four_fluctuating_edges_generate_one_continuous_round_trip(self):
        from modeling_preview import build_model_preview
        from modeling_task_generator import generate_task_plan

        def point(point_id, x, y):
            return {
                "id": point_id,
                "x": x,
                "y": y,
                "lat": 32.0,
                "lon": 118.0,
            }

        # 四条边都包含人工记录的波动点：左右短边负责换行，上下长边本身就是
        # 两条边界清扫线。完整规划必须经过所有记录点，并且内部线仍是一段直线。
        points = [
            point("p1", 0, 0),
            point("p2", 25, 30),
            point("p3", -20, 70),
            point("p4", 0, 120),
            point("p5", 100, 128),
            point("p6", 200, 110),
            point("p7", 300, 120),
            point("p8", 320, 80),
            point("p9", 282, 40),
            point("p10", 300, 0),
            point("p11", 200, -10),
            point("p12", 100, 8),
        ]
        draft = {
            "id": "all-four-edges-fluctuate",
            "recognition": {"confirmed": True},
            "groups": [{
                "id": "g1",
                "areaNumber": 1,
                "sweepDirection": "auto",
                "points": points,
                "subAreas": [{
                    "id": "sa1",
                    "pointIds": [item["id"] for item in points],
                }],
            }],
            "groupLinks": [],
        }
        draft["taskPreview"] = build_model_preview(draft, now=1000)

        plan = generate_task_plan(draft, now=2000)
        tasks = plan["tasks"]
        visited = {
            (task[prefix + "X"], task[prefix + "Y"])
            for task in tasks
            for prefix in ("start", "end")
        }
        expected = {(int(item["x"]), int(item["y"])) for item in points}

        self.assertEqual(plan["status"], "ready")
        self.assertTrue(expected.issubset(visited))
        self.assertTrue(all(
            (left["endX"], left["endY"]) == (right["startX"], right["startY"])
            for left, right in zip(tasks, tasks[1:])
        ))
        self.assertEqual(
            (tasks[0]["startX"], tasks[0]["startY"]),
            (tasks[-1]["endX"], tasks[-1]["endY"]),
        )
        interior_lane_ids = [
            task.get("sourceLaneId")
            for task in tasks
            if int(task.get("mode") or 0) == 1 and task.get("laneType") == "interior"
        ]
        # 每条内部线只生成一个首尾任务段，说明它没有继承边界波动而变成折线。
        self.assertEqual(len(interior_lane_ids), len(set(interior_lane_ids)))

    def test_generate_task_plan_requires_ready_preview(self):
        from modeling_task_generator import ModelingTaskGenerationError, generate_task_plan

        with self.assertRaises(ModelingTaskGenerationError):
            generate_task_plan({"id": "m1", "taskPreview": {"status": "empty"}})


if __name__ == "__main__":
    unittest.main()
