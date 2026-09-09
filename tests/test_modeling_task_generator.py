# coding=utf-8
import unittest


class ModelingTaskGeneratorTest(unittest.TestCase):
    def test_group_recorded_anchors_merge_near_last_point_into_first(self):
        from modeling_task_generator import _CoordinateMapper, _group_recorded_xy

        points = [
            {"id": "p1", "x": 20.662, "y": 278.877, "lat": 32.0, "lon": 118.0},
            {"id": "p2", "x": 24.301, "y": 340.201, "lat": 32.0, "lon": 118.0},
            {"id": "p3", "x": 380.344, "y": 371.936, "lat": 32.0, "lon": 118.0},
            {"id": "p4", "x": 369.240, "y": 247.031, "lat": 32.0, "lon": 118.0},
            {"id": "p5", "x": 5.769, "y": 271.349, "lat": 32.0, "lon": 118.0},
        ]
        draft = {"groups": [{"id": "g1", "points": points}]}

        anchors = _group_recorded_xy(draft, "g1", _CoordinateMapper(draft))

        self.assertEqual(anchors, [
            (20.662, 278.877),
            (24.301, 340.201),
            (380.344, 371.936),
            (369.240, 247.031),
        ])
        # 规划只返回合并后的锚点，绝不能删除或改写原始记录。
        self.assertEqual(len(draft["groups"][0]["points"]), 5)
        self.assertEqual(draft["groups"][0]["points"][-1]["id"], "p5")

    def test_l_shaped_group_link_keeps_all_points_and_stops_at_corner(self):
        from modeling_task_generator import _CoordinateMapper, _append_transition_tasks

        draft = {
            "groups": [
                {"id": "g1", "points": [{
                    "id": "a1", "x": 0, "y": 0, "lat": 32.0, "lon": 118.0,
                }]},
                {"id": "g2", "points": [{
                    "id": "b1", "x": 100, "y": 100, "lat": 32.000009, "lon": 118.0000106,
                }]},
            ],
        }
        preview = {
            "groupLinks": [{
                "id": "l1",
                "startGroupId": "g1",
                "endGroupId": "g2",
                "points": [
                    {"id": "l1-start", "x": 0, "y": 0},
                    {"id": "l1-corner", "x": 100, "y": 0},
                    {"id": "l1-end", "x": 100, "y": 100},
                ],
            }],
        }
        mapper = _CoordinateMapper(draft)
        tasks = []

        next_task_id = _append_transition_tasks(
            tasks,
            (0, 0),
            (100, 100),
            2,
            mapper,
            1,
            preview,
            draft,
            "g1",
            "g2",
        )

        self.assertEqual(next_task_id, 3)
        self.assertEqual(
            [(task["startX"], task["startY"], task["endX"], task["endY"]) for task in tasks],
            [(0, 0, 100, 0), (100, 0, 100, 100)],
        )
        self.assertEqual([task["turnAtStart"] for task in tasks], [True, True])
        self.assertEqual([task["stopAtEnd"] for task in tasks], [True, True])

    def test_inside_bridge_endpoint_uses_polyline_boundary_intersection(self):
        """桥头在区域内时沿桥折线裁到真实边界交点，不使用最近垂足。"""
        from modeling_task_generator import (
            _CoordinateMapper,
            _route_edges_with_boundary_portals,
        )

        def point(point_id, x, y):
            return {
                "id": point_id, "x": x, "y": y,
                "lat": 32.0, "lon": 118.0,
            }

        draft = {
            "groups": [
                {"id": "g1", "points": [
                    point("a1", 0, 0), point("a2", 0, 100),
                    point("a3", 100, 100), point("a4", 100, 0),
                ]},
                {"id": "g2", "points": [
                    point("b1", 200, 0), point("b2", 200, 100),
                    point("b3", 300, 100), point("b4", 300, 0),
                ]},
            ],
        }
        edges = [{
            "fromGroupId": "g1",
            "toGroupId": "g2",
            # 首点和第二点仍在区域1内，第三点才从右边界穿出；这也验证多点/L形桥。
            "points": [(20.0, 20.0), (20.0, 80.0), (120.0, 80.0), (180.0, 80.0)],
        }]

        result = _route_edges_with_boundary_portals(
            edges, draft, _CoordinateMapper(draft),
        )

        self.assertTrue(result[0]["startUsesBoundaryPortal"])
        self.assertFalse(result[0]["endUsesBoundaryPortal"])
        self.assertEqual(result[0]["points"], [
            (100.0, 80.0), (120.0, 80.0), (180.0, 80.0),
        ])

        reverse_result = _route_edges_with_boundary_portals([{
            "fromGroupId": "g2", "toGroupId": "g1",
            "points": list(reversed(edges[0]["points"])),
        }], draft, _CoordinateMapper(draft))
        self.assertFalse(reverse_result[0]["startUsesBoundaryPortal"])
        self.assertTrue(reverse_result[0]["endUsesBoundaryPortal"])
        self.assertEqual(reverse_result[0]["points"], [
            (180.0, 80.0), (120.0, 80.0), (100.0, 80.0),
        ])

    def test_outside_bridge_endpoint_keeps_original_connection_point(self):
        """桥头在区域外时不延长桥线猜交点，继续交给原边界接入算法。"""
        from modeling_task_generator import (
            _CoordinateMapper,
            _route_edges_with_boundary_portals,
        )

        def point(point_id, x, y):
            return {
                "id": point_id, "x": x, "y": y,
                "lat": 32.0, "lon": 118.0,
            }

        draft = {"groups": [{"id": "g1", "points": [
            point("a1", 0, 0), point("a2", 0, 100),
            point("a3", 100, 100), point("a4", 100, 0),
        ]}, {"id": "g2", "points": [
            point("b1", 200, 0), point("b2", 200, 100),
            point("b3", 300, 100), point("b4", 300, 0),
        ]}]}
        original = [(120.0, 50.0), (180.0, 50.0)]

        result = _route_edges_with_boundary_portals([{
            "fromGroupId": "g1", "toGroupId": "g2", "points": original,
        }], draft, _CoordinateMapper(draft))

        self.assertEqual(result[0]["points"], original)
        self.assertFalse(result[0]["startUsesBoundaryPortal"])
        self.assertFalse(result[0]["endUsesBoundaryPortal"])

    def test_final_fifteen_cm_merge_prefers_true_bridge_point_and_refreshes_markers(self):
        """15cm短任务不输出，桥口优先保留，连续折线标记按新几何重算。"""
        from modeling_task_generator import (
            _merge_near_executable_tasks,
            _refresh_continuous_task_markers,
        )

        def task(task_id, start, end, preferred_start=False, preferred_end=False):
            item = {
                "id": task_id, "mode": 2,
                "startX": start[0], "startY": start[1],
                "endX": end[0], "endY": end[1],
                "startLat": 32.0, "startLon": 118.0,
                "endLat": 32.0, "endLon": 118.0,
                "source": "modeling_transfer",
                "continuousPathId": "bridge-path",
                "turnAtStart": True, "stopAtEnd": True,
            }
            if preferred_start:
                item["preferredStartPoint"] = True
            if preferred_end:
                item["preferredEndPoint"] = True
            return item

        merged = _merge_near_executable_tasks([
            task(1, (0, 0), (100, 0)),
            task(2, (100, 0), (106, 0), preferred_end=True),
            task(3, (106, 0), (200, 0), preferred_start=True),
        ])
        refreshed = _refresh_continuous_task_markers(merged)

        self.assertEqual([
            (item["startX"], item["startY"], item["endX"], item["endY"])
            for item in refreshed
        ], [(0, 0, 106, 0), (106, 0, 200, 0)])
        self.assertEqual([item["id"] for item in refreshed], [1, 2])
        self.assertEqual([item["continuousPathIndex"] for item in refreshed], [1, 2])
        self.assertEqual([item["continuousPathCount"] for item in refreshed], [2, 2])
        self.assertEqual([item["turnAtStart"] for item in refreshed], [True, False])
        self.assertEqual([item["stopAtEnd"] for item in refreshed], [False, True])
        self.assertNotIn("preferredEndPoint", refreshed[0])

    def test_transition_path_removes_local_out_and_back_loop(self):
        from modeling_task_generator import _simplify_transition_path_points

        # 对应真实日志中的“到桥头附近13cm -> 原路返回 -> 再继续”结构。
        simplified = _simplify_transition_path_points([
            (0.0, 0.0),
            (6.0, 11.0),
            (0.0, 0.0),
            (-13.0, -19.0),
            (-20.0, -80.0),
        ])

        self.assertEqual(simplified, [(0.0, 0.0), (-20.0, -80.0)])

    def test_transition_path_keeps_short_real_right_angle(self):
        from modeling_task_generator import _simplify_transition_path_points

        # 13cm 本身不能作为删除理由；形成90度真实拐角时必须留下停车转向点。
        simplified = _simplify_transition_path_points([
            (0.0, 0.0),
            (0.0, 13.0),
            (100.0, 13.0),
        ])

        self.assertEqual(simplified, [
            (0.0, 0.0),
            (0.0, 13.0),
            (100.0, 13.0),
        ])

    def test_transition_path_merges_short_nearly_straight_jitter(self):
        from modeling_task_generator import _simplify_transition_path_points

        simplified = _simplify_transition_path_points([
            (0.0, 0.0),
            (1.0, 15.0),
            (3.0, 100.0),
        ])

        self.assertEqual(simplified, [(0.0, 0.0), (3.0, 100.0)])

    def test_transition_near_pairs_keep_lower_cost_later_points(self):
        """真实近点对应删除A3-9/A2-10，而不是按扫描顺序固定删后点。"""
        from modeling_task_generator import _simplify_transition_path_points

        l2_2 = (822.80, 175.61)
        a3_9 = (836.19, 194.78)
        a3_1 = (829.93, 205.93)
        a3_2 = (838.99, 272.33)
        region_three = _simplify_transition_path_points([
            l2_2, a3_9, a3_1, a3_2,
        ])

        l1_2 = (18.17, 230.43)
        a2_10 = (5.77, 271.35)
        a2_1 = (20.66, 278.88)
        a2_2 = (24.30, 340.20)
        region_two = _simplify_transition_path_points([
            l1_2, a2_10, a2_1, a2_2,
        ])

        self.assertEqual(region_three, [l2_2, a3_1, a3_2])
        self.assertNotIn(a3_9, region_three)
        self.assertEqual(region_two, [l1_2, a2_1, a2_2])
        self.assertNotIn(a2_10, region_two)

    def test_transition_near_cluster_never_removes_explicit_bridge_points(self):
        """桥上两个点即使不足30cm也必须全部经过，不能被近点优化跨越。"""
        from modeling_task_generator import _simplify_transition_path_points

        start = (-100.0, 0.0)
        bridge_start = (0.0, 0.0)
        bridge_end = (0.0, 20.0)
        target = (100.0, 20.0)
        simplified = _simplify_transition_path_points(
            [start, bridge_start, bridge_end, target],
            protected_points=[bridge_start, bridge_end],
        )

        self.assertEqual(simplified, [
            start, bridge_start, bridge_end, target,
        ])

    def test_transition_loop_cleanup_cannot_cut_across_bridge_points(self):
        """局部回环命中起点附近时，也不能把已经经过的桥点整段截掉。"""
        from modeling_task_generator import _simplify_transition_path_points

        start = (0.0, 0.0)
        bridge_start = (0.0, 50.0)
        bridge_end = (0.0, 80.0)
        near_start_again = (2.0, 1.0)
        target = (100.0, 100.0)
        simplified = _simplify_transition_path_points(
            [start, bridge_start, bridge_end, near_start_again, target],
            protected_points=[bridge_start, bridge_end],
        )

        self.assertIn(bridge_start, simplified)
        self.assertIn(bridge_end, simplified)
        self.assertEqual(simplified[0], start)
        self.assertEqual(simplified[-1], target)

    def test_compaction_removes_short_clean_transfer_reverse_pair(self):
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
                "source": "modeling_clean" if mode == 1 else "modeling_transfer",
            }

        first = task(1, 1, (-100.0, 0.0), (0.0, 0.0))
        spur = task(2, 1, (0.0, 0.0), (6.0, 11.0))
        spur["continuousPathId"] = "boundary-lane"
        compacted = _compact_executable_tasks([
            first,
            spur,
            task(3, 2, (6.0, 11.0), (0.0, 0.0)),
            task(4, 2, (0.0, 0.0), (0.0, 100.0)),
        ])

        self.assertEqual(len(compacted), 2)
        self.assertEqual((compacted[0]["endX"], compacted[0]["endY"]), (0.0, 0.0))
        self.assertEqual((compacted[1]["startX"], compacted[1]["startY"]), (0.0, 0.0))
        self.assertEqual((compacted[1]["endX"], compacted[1]["endY"]), (0.0, 100.0))

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

    def test_connection_points_enter_boundary_through_nearest_projection(self):
        """连接点必须先投影到边界，不得斜穿到同一条边的远端角点。"""
        from modeling_task_generator import (
            _CoordinateMapper,
            _compact_executable_tasks,
            _group_anchor_transition_points,
            _nearest_boundary_projection,
        )

        points = [
            {"id": "A1", "x": 0.0, "y": 0.0, "lat": 32.0, "lon": 118.0},
            {"id": "A2", "x": 10.708, "y": 127.296, "lat": 32.0, "lon": 118.0},
            {"id": "A3", "x": 954.092, "y": 52.061, "lat": 32.0, "lon": 118.0},
            {"id": "A4", "x": 946.118, "y": -76.013, "lat": 32.0, "lon": 118.0},
        ]
        draft = {"groups": [{"id": "area-1", "points": points}]}
        mapper = _CoordinateMapper(draft)
        anchors = [mapper.point_to_xy(point) for point in points]
        l1_projection = _nearest_boundary_projection(
            (31.474, 177.712), anchors
        )["point"]

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
            l1_projection,
            (954.092, 52.061),
            (957.0, 105.0),
        ])
        self.assertEqual(inbound, list(reversed(outbound)))

        # 后续普通移动点简化也必须保留投影点和A3明显转向点。
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
        self.assertIn(l1_projection, executable_points)
        self.assertIn((954.092, 52.061), executable_points)

    def test_bridge_endpoint_within_ten_cm_replaces_boundary_projection(self):
        """桥头贴边且不穿区时，用人工桥头替代Q点，正反方向都不能生成短任务。"""
        from modeling_task_generator import _CoordinateMapper, _group_anchor_transition_points

        points = [
            {"id": "a1", "x": 0.0, "y": 0.0, "lat": 32.0, "lon": 118.0},
            {"id": "a2", "x": 0.0, "y": 100.0, "lat": 32.0, "lon": 118.0},
            {"id": "a3", "x": 200.0, "y": 100.0, "lat": 32.0, "lon": 118.0},
            {"id": "a4", "x": 200.0, "y": 0.0, "lat": 32.0, "lon": 118.0},
        ]
        draft = {"groups": [{"id": "g1", "points": points}]}
        mapper = _CoordinateMapper(draft)
        bridge = (100.0, -5.5)

        outbound = _group_anchor_transition_points(
            draft,
            "g1",
            (0.0, 0.0),
            bridge,
            mapper,
            target_is_bridge_endpoint=True,
        )
        inbound = _group_anchor_transition_points(
            draft,
            "g1",
            bridge,
            (0.0, 0.0),
            mapper,
            current_is_bridge_endpoint=True,
        )

        self.assertEqual(outbound, [(0.0, 0.0), bridge])
        self.assertEqual(inbound, list(reversed(outbound)))
        self.assertNotIn((100.0, 0.0), outbound)

    def test_bridge_endpoint_outside_tolerance_or_crossing_area_keeps_projection(self):
        """超过10cm或直连会穿过区域时，仍保留Q点，禁止为了合并而斜穿。"""
        from modeling_task_generator import _CoordinateMapper, _group_anchor_transition_points

        points = [
            {"id": "a1", "x": 0.0, "y": 0.0, "lat": 32.0, "lon": 118.0},
            {"id": "a2", "x": 0.0, "y": 100.0, "lat": 32.0, "lon": 118.0},
            {"id": "a3", "x": 200.0, "y": 100.0, "lat": 32.0, "lon": 118.0},
            {"id": "a4", "x": 200.0, "y": 0.0, "lat": 32.0, "lon": 118.0},
        ]
        draft = {"groups": [{"id": "g1", "points": points}]}
        mapper = _CoordinateMapper(draft)

        too_far = _group_anchor_transition_points(
            draft,
            "g1",
            (0.0, 0.0),
            (100.0, -10.1),
            mapper,
            target_is_bridge_endpoint=True,
        )
        crosses_area = _group_anchor_transition_points(
            draft,
            "g1",
            (0.0, 0.0),
            (100.0, 5.0),
            mapper,
            target_is_bridge_endpoint=True,
        )

        self.assertIn((100.0, 0.0), too_far)
        self.assertIn((100.0, 0.0), crosses_area)

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

    def test_same_edge_lane_change_does_not_overshoot_nearby_corner_and_reverse(self):
        """边界点测试1右侧换行应直达第二条线，不能越过17cm后掉头。"""
        from modeling_task_generator import _CoordinateMapper, _group_anchor_transition_points

        def point(point_id, x, y):
            return {
                "id": point_id,
                "x": x,
                "y": y,
                "lat": 32.0,
                "lon": 118.0,
            }

        # 边界点测试1区域2的真实记录坐标。第一条清扫线右端位于p6，
        # 第二条线右端(1194.8,265.0)位于p6--p7同一条边上，距p7约17cm。
        points = [
            point("p1", 836.869, 199.895),
            point("p2", 842.779, 278.232),
            point("p3", 847.106, 336.876),
            point("p4", 970.192, 324.422),
            point("p5", 1081.938, 315.983),
            point("p6", 1198.011, 303.840),
            point("p7", 1193.430, 247.987),
            point("p8", 1186.823, 179.580),
        ]
        draft = {"groups": [{"id": "area-2", "points": points}]}
        mapper = _CoordinateMapper(draft)
        current = (1198.0, 303.8)
        next_lane_start = (1194.8, 265.0)

        path = _group_anchor_transition_points(
            draft,
            "area-2",
            current,
            next_lane_start,
            mapper,
        )

        self.assertEqual(path[0], current)
        self.assertEqual(path[-1], next_lane_start)
        self.assertEqual(path, [current, next_lane_start])
        self.assertNotIn((1193.430, 247.987), path)
        # 沿同一边只允许单调接近目标，不能出现到p7后再反向17cm的回补段。
        distances_to_target = [
            ((item[0] - next_lane_start[0]) ** 2 + (item[1] - next_lane_start[1]) ** 2) ** 0.5
            for item in path
        ]
        self.assertTrue(all(
            following <= previous + 0.01
            for previous, following in zip(distances_to_target, distances_to_target[1:])
        ))

    def test_adjacent_edge_lane_change_does_not_snap_back_to_nearby_corner(self):
        """8.17真实坐标：相邻边换行必须经过共享点，不能先退回19cm外的原点。"""
        from modeling_task_generator import _CoordinateMapper, _group_anchor_transition_points

        def point(point_id, x, y):
            return {
                "id": point_id,
                "x": x,
                "y": y,
                "lat": 32.0,
                "lon": 118.0,
            }

        # 区域1原点短边的真实记录点。current 位于 A1--A1-02，target
        # 位于 A1-02--A1-03；A1-02 才是两条相邻边的共享换行点。
        points = [
            point("A1", 0.000, 0.000),
            point("A1-02", 4.581, 52.373),
            point("A1-03", 7.183, 116.121),
            point("A1-04", 232.844, 96.895),
            point("A1-05", 580.215, 68.885),
            point("A1-06", 934.486, 46.524),
            point("A1-07", 933.845, 40.508),
            point("A1-08", 926.879, -26.153),
            point("A1-09", 922.477, -87.066),
            point("A1-10", 790.804, -74.267),
            point("A1-11", 458.580, -50.060),
            point("A1-12", 106.817, -18.703),
            point("A1-13", -7.993, -11.075),
        ]
        draft = {"groups": [{"id": "area-1", "points": points}]}
        mapper = _CoordinateMapper(draft)
        current = (2.0, 19.0)
        next_lane_start = (5.0, 67.0)

        path = _group_anchor_transition_points(
            draft,
            "area-1",
            current,
            next_lane_start,
            mapper,
        )

        self.assertEqual(path[0], current)
        self.assertEqual(path[-1], next_lane_start)
        self.assertNotIn((0.0, 0.0), path)
        self.assertIn((4.581, 52.373), path)
        # 路径沿短边单调向上，任何一步都不能重新远离目标。
        distances_to_target = [
            ((item[0] - next_lane_start[0]) ** 2 + (item[1] - next_lane_start[1]) ** 2) ** 0.5
            for item in path
        ]
        self.assertTrue(all(
            following <= previous + 0.01
            for previous, following in zip(distances_to_target, distances_to_target[1:])
        ))

    def test_external_connector_takes_direct_route_only_when_it_stays_outside_area(self):
        """边界角点可直达区域外连接点，但禁止直线穿过清扫区域。"""
        from modeling_task_generator import _CoordinateMapper, _group_anchor_transition_points

        def point(point_id, x, y):
            return {
                "id": point_id,
                "x": x,
                "y": y,
                "lat": 32.0,
                "lon": 118.0,
            }

        draft = {"groups": [{
            "id": "area-1",
            "points": [
                point("p1", 0, 0),
                point("p2", 0, 100),
                point("p3", 100, 100),
                point("p4", 100, 0),
            ],
        }]}
        mapper = _CoordinateMapper(draft)

        outside_path = _group_anchor_transition_points(
            draft,
            "area-1",
            (0.0, 100.0),
            (20.0, 150.0),
            mapper,
        )
        self.assertEqual(outside_path, [(0.0, 100.0), (20.0, 150.0)])

        crossing_path = _group_anchor_transition_points(
            draft,
            "area-1",
            (0.0, 50.0),
            (120.0, 50.0),
            mapper,
        )
        self.assertNotEqual(crossing_path, [(0.0, 50.0), (120.0, 50.0)])
        self.assertTrue(any(point in crossing_path for point in ((0, 0), (0, 100))))

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
        # 统一15cm合并比较局部代价后保留(41,416)，不会再保留1cm独立任务。
        self.assertEqual((compacted[0]["endX"], compacted[0]["endY"]), (41, 416))
        self.assertEqual((compacted[1]["startX"], compacted[1]["startY"]), (41, 416))
        self.assertEqual((compacted[1]["endX"], compacted[1]["endY"]), (403, 382))
        self.assertEqual(compacted[1]["mode"], 1)
        self.assertAlmostEqual(compacted[1]["heading"], 95.4, places=1)

    def test_fifteen_cm_merge_has_no_point_type_or_stop_exemption(self):
        """15cm规则统一作用于执行点，旧3cm阈值和显式停车不再产生短任务。"""
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

        self.assertEqual(len(three_cm), 1)
        self.assertEqual(
            (three_cm[0]["startX"], three_cm[0]["startY"],
             three_cm[0]["endX"], three_cm[0]["endY"]),
            (0, 0, 3, 100),
        )
        self.assertEqual(len(explicit_one_cm), 1)
        self.assertEqual(
            (explicit_one_cm[0]["startX"], explicit_one_cm[0]["startY"],
             explicit_one_cm[0]["endX"], explicit_one_cm[0]["endY"]),
            (0, 0, 1, 100),
        )

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

    def test_long_axis_wins_even_when_first_recorded_edge_is_shorter_axis(self):
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

        # 区域纵向339cm、横向226cm，因此应沿纵向长轴清扫；第一条记录边
        # p1->p2是横向边，但不能再支配清扫方向。
        self.assertEqual(
            (tasks[0]["startX"], tasks[0]["startY"], tasks[0]["endX"], tasks[0]["endY"]),
            (0, 0, 0, 339),
        )
        self.assertEqual(
            (tasks[1]["startX"], tasks[1]["startY"]),
            (0, 339),
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
                preview_group("g1", 1, 0, 100),
                preview_group("g2", 2, 200, 300),
                preview_group("g3", 3, 400, 500),
            ],
            "groupLinks": [
                {
                    "id": "l12",
                    "startGroupId": "g1",
                    "endGroupId": "g2",
                    "points": [
                        {"id": "l12-a", "x": 100, "y": 0},
                        {"id": "l12-b", "x": 200, "y": 0},
                    ],
                },
                {
                    "id": "l23",
                    "startGroupId": "g2",
                    "endGroupId": "g3",
                    "points": [
                        {"id": "l23-a", "x": 300, "y": 0},
                        {"id": "l23-mid", "x": 350, "y": 50},
                        {"id": "l23-b", "x": 400, "y": 0},
                    ],
                },
            ],
        }
        draft = {
            "id": "three-area-route",
            "groups": [
                {"id": "g1", "points": [{"id": "p1", "x": 0, "y": 0, "lat": 32.0, "lon": 118.0}]},
                {"id": "g2", "points": [{"id": "p2", "x": 200, "y": 0, "lat": 32.0, "lon": 118.0}]},
                {"id": "g3", "points": [{"id": "p3", "x": 400, "y": 0, "lat": 32.0, "lon": 118.0}]},
            ],
            "taskPreview": preview,
        }

        task_plan = generate_task_plan(draft, now=2000)
        tasks = task_plan["tasks"]

        self.assertEqual(task_plan["summary"]["cleanTaskCount"], 3)
        self.assertEqual((tasks[0]["startX"], tasks[0]["startY"]), (0, 0))
        self.assertEqual((tasks[-1]["endX"], tasks[-1]["endY"]), (0, 0))
        self.assertTrue(any((task["startX"], task["startY"]) == (350, 50) for task in tasks))
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

    def test_turn_stop_thresholds_are_fifty_five_degrees(self):
        """清扫折线和跨区转场必须统一使用55度停车转向阈值。"""
        from modeling_task_generator import (
            CLEAN_PATH_HARD_TURN_DEG,
            TRANSFER_HARD_TURN_DEG,
        )

        self.assertEqual(TRANSFER_HARD_TURN_DEG, 55.0)
        self.assertEqual(CLEAN_PATH_HARD_TURN_DEG, 55.0)

    def test_continuous_path_passes_45_degree_turn_but_stops_for_60_degrees(self):
        """55度以下连续经过，55度以上仍停车并重新转向。"""
        from modeling_task_generator import _mark_continuous_path_tasks

        def tasks_for(end):
            return [
                {"startX": 0, "startY": 0, "endX": 100, "endY": 0},
                {"startX": 100, "startY": 0, "endX": end[0], "endY": end[1]},
            ]

        forty_five = tasks_for((200, 100))
        sixty = tasks_for((150, 86.6025403784))
        _mark_continuous_path_tasks(forty_five, "path-45")
        _mark_continuous_path_tasks(sixty, "path-60")

        self.assertFalse(forty_five[0]["stopAtEnd"])
        self.assertFalse(forty_five[1]["turnAtStart"])
        self.assertTrue(sixty[0]["stopAtEnd"])
        self.assertTrue(sixty[1]["turnAtStart"])

    def test_clean_path_collapses_same_position_points_but_keeps_real_short_corner(self):
        from modeling_task_generator import _simplify_clean_path_points

        # 0.75cm的A1-9/A1-8是同一物理位置，必须合并；13cm的
        # 90度短边是真实边界拐角，不能因为短就被抹掉。
        a1_9 = (940.5, 45.79)
        a1_8 = (940.2, 45.1)
        a1_7 = (675.5, 75.0)
        self.assertEqual(
            _simplify_clean_path_points([a1_9, a1_8, a1_7]),
            [a1_9, a1_7],
        )

        right_angle = [(0.0, 0.0), (0.0, 13.0), (100.0, 13.0)]
        self.assertEqual(_simplify_clean_path_points(right_angle), right_angle)

    def test_clean_path_removes_short_straight_tail_but_preserves_endpoint(self):
        from modeling_task_generator import _simplify_clean_path_points

        # 最后18cm与前一段只相5度，属于同一直行边的密集采样。
        # 删除中间点时必须保留原清扫终点，供下一段转场使用。
        start = (492.0, 88.0)
        dense = (31.1, 121.2)
        endpoint = (12.64, 123.938)
        self.assertEqual(
            _simplify_clean_path_points([start, dense, endpoint]),
            [start, endpoint],
        )

    def test_append_clean_segments_does_not_create_test2_one_centimeter_lane(self):
        from modeling_task_generator import _CoordinateMapper, _append_clean_segments

        draft = {
            "groups": [{
                "id": "g1",
                "points": [{
                    "id": "origin", "x": 0, "y": 0,
                    "lat": 32.0, "lon": 118.0,
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
                "sourceId": "lane-1",
                "laneType": "boundary",
                "start": (940.5, 45.79),
                "end": (675.475, 75.045),
                "path": [
                    (940.5, 45.79),
                    (940.217, 45.112),
                    (675.475, 75.045),
                ],
            }],
            (935.579, -26.965),
            mapper,
            1,
            draft,
        )

        clean_tasks = [task for task in tasks if task["mode"] == 1]
        self.assertEqual(clean_count, 1)
        self.assertEqual(next_task_id, 3)
        self.assertEqual(current, (675.475, 75.045))
        self.assertEqual(len(clean_tasks), 1)
        self.assertEqual(
            (
                clean_tasks[0]["startX"], clean_tasks[0]["startY"],
                clean_tasks[0]["endX"], clean_tasks[0]["endY"],
            ),
            (941, 46, 675, 75),
        )
        self.assertGreater(clean_tasks[0]["length"], 5)
        # 转场终点必须与清理后的清扫起点完全一致。
        self.assertEqual(
            (tasks[0]["endX"], tasks[0]["endY"]),
            (clean_tasks[0]["startX"], clean_tasks[0]["startY"]),
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

        # 四条边都包含人工记录的波动点：区域点定义真实形状，但不再要求每个
        # 记录点都成为机器人任务端点。两条外边界保留折线，内部线仍保持直线。
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
        self.assertEqual(plan["status"], "ready")
        self.assertEqual(
            [point["id"] for point in draft["taskPreview"]["groups"][0]["subAreas"][0]["polygon"]],
            [point["id"] for point in points],
        )
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

    def test_multi_point_bridge_recording_direction_does_not_change_route(self):
        from modeling_preview import build_model_preview
        from modeling_task_generator import generate_task_plan

        def point(point_id, x, y):
            return {
                "id": point_id,
                "x": x,
                "y": y,
                "lat": 32.0 + y / 11111100.0,
                "lon": 118.0 + x / 9420000.0,
            }

        group_one = [
            point("a1", 0, 0), point("a2", 0, 120),
            point("a3", 300, 120), point("a4", 300, 0),
        ]
        group_two = [
            point("b1", 420, 220), point("b2", 420, 340),
            point("b3", 720, 340), point("b4", 720, 220),
        ]
        forward_bridge = [
            point("l1", 300, 100),
            point("l2", 360, 100),
            point("l3", 360, 240),
            point("l4", 420, 240),
        ]

        def plan_for(bridge_points):
            draft = {
                "id": "bridge-direction-invariant",
                "recognition": {"confirmed": True},
                "groups": [
                    {
                        "id": "g1", "areaNumber": 1, "points": group_one,
                        "subAreas": [{"id": "sa1", "pointIds": [p["id"] for p in group_one]}],
                    },
                    {
                        "id": "g2", "areaNumber": 2, "points": group_two,
                        "subAreas": [{"id": "sa2", "pointIds": [p["id"] for p in group_two]}],
                    },
                ],
                "groupLinks": [{
                    "id": "link-1",
                    "startGroupId": "g1",
                    "endGroupId": "g2",
                    "points": bridge_points,
                }],
                "routePolicy": {"areaOrder": [1, 2]},
            }
            draft["taskPreview"] = build_model_preview(draft, now=1000)
            return generate_task_plan(draft, now=2000)

        forward = plan_for(forward_bridge)
        backward = plan_for(list(reversed(forward_bridge)))

        def signature(plan):
            return [
                (
                    task["mode"],
                    task["startX"], task["startY"],
                    task["endX"], task["endY"],
                    task.get("source"),
                )
                for task in plan["tasks"]
            ]

        self.assertEqual(signature(forward), signature(backward))
        # L形桥的四个点必须按自动确定的区域1->区域2方向依次出现在跨区路线中。
        visited_segments = [
            ((task["startX"], task["startY"]), (task["endX"], task["endY"]))
            for task in forward["tasks"]
            if task.get("source") == "modeling_transfer"
        ]
        self.assertIn(((300, 100), (360, 100)), visited_segments)
        self.assertIn(((360, 100), (360, 240)), visited_segments)
        self.assertIn(((360, 240), (420, 240)), visited_segments)

    def test_real_111_bridge_join_has_no_five_cm_projection_tasks(self):
        """真实111模型的Q<->L1短任务必须消失，同时保留四个桥点和完整往返。"""
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

        area_one = [
            point("a1", 0.0, 0.0),
            point("a2", 7.334, 123.771),
            point("a3", 12.480, 123.482),
            point("a4", 365.686, 106.002),
            point("a5", 931.441, 51.061),
            point("a6", 921.280, -73.578),
            point("a7", 594.147, -47.213),
        ]
        area_two = [
            point("b1", 824.398, 192.145),
            point("b2", 1189.528, 172.986),
            point("b3", 1195.900, 294.633),
            point("b4", 868.060, 340.668),
        ]
        bridge = [
            point("l1", 808.930, -69.986),
            point("l2", 812.135, 29.834),
            point("l3", 816.292, 106.703),
            point("l4", 823.833, 192.045),
        ]
        draft = {
            "id": "real-111-bridge-join",
            "recognition": {"confirmed": True},
            "groups": [
                {
                    "id": "g1", "areaNumber": 1, "points": area_one,
                    "subAreas": [{"id": "sa1", "pointIds": [p["id"] for p in area_one]}],
                },
                {
                    "id": "g2", "areaNumber": 2, "points": area_two,
                    "subAreas": [{"id": "sa2", "pointIds": [p["id"] for p in area_two]}],
                },
            ],
            "groupLinks": [{
                "id": "link-1",
                "startGroupId": "g1",
                "endGroupId": "g2",
                "points": bridge,
            }],
            "routePolicy": {"areaOrder": [2, 1]},
        }
        draft["taskPreview"] = build_model_preview(draft, now=1000)

        plan = generate_task_plan(draft, now=2000)
        tasks = plan["tasks"]
        endpoints = {
            (task[prefix + "X"], task[prefix + "Y"])
            for task in tasks
            for prefix in ("start", "end")
        }

        self.assertEqual(plan["status"], "ready")
        self.assertEqual(plan["summary"]["taskCount"], 25)
        self.assertNotIn((809, -65), endpoints)
        self.assertIn((809, -70), endpoints)
        self.assertIn((812, 30), endpoints)
        self.assertIn(
            (809, -70, 812, 30),
            {
                (task["startX"], task["startY"], task["endX"], task["endY"])
                for task in tasks
            },
        )

    def test_hhh123_uses_true_home_portal_and_has_no_fifteen_cm_task(self):
        """hhh123实数回归：区域内L1换成交点，区域外L2仍按原方法接入。"""
        from modeling_preview import build_model_preview
        from modeling_task_generator import generate_task_plan

        def point(point_id, x, y, lat, lon):
            return {
                "id": point_id, "x": x, "y": y,
                "lat": lat, "lon": lon,
            }

        area_one = [
            point("a1", 0.0, 0.0, 32.036478348, 118.924489846),
            point("a2", 9.85, 99.975, 32.036487339, 118.924490891),
            point("a3", 928.745, 20.471, 32.036480189, 118.924588375),
            point("a4", 929.584, -94.727, 32.036469829, 118.924588464),
        ]
        area_two = [
            point("b1", 37.968, 252.891, 32.036501091, 118.924493874),
            point("b2", 359.813, 244.562, 32.036500342, 118.924528018),
            point("b3", 373.283, 359.46, 32.036510675, 118.924529447),
            point("b4", 24.564, 397.355, 32.036514083, 118.924492452),
        ]
        bridge = [
            point("l1", 14.526, 73.155, 32.036484927, 118.924491387),
            point("l2", 26.025, 257.55, 32.036501510, 118.924492607),
        ]
        draft = {
            "id": "hhh123-regression",
            "recognition": {"confirmed": True},
            "groups": [
                {"id": "g1", "areaNumber": 1, "points": area_one,
                 "subAreas": [{"id": "sa1", "pointIds": [p["id"] for p in area_one]}]},
                {"id": "g2", "areaNumber": 2, "points": area_two,
                 "subAreas": [{"id": "sa2", "pointIds": [p["id"] for p in area_two]}]},
            ],
            "groupLinks": [{
                "id": "link", "startGroupId": "g1", "endGroupId": "g2",
                "status": "ready", "points": bridge,
            }],
            "routePolicy": {"type": "area_order", "areaOrder": [2, 1]},
        }
        draft["taskPreview"] = build_model_preview(draft, now=1000)

        plan = generate_task_plan(draft, now=2000, return_to_origin=True)
        endpoints = {
            (task[prefix + "X"], task[prefix + "Y"])
            for task in plan["tasks"] for prefix in ("start", "end")
        }

        self.assertEqual(plan["summary"]["taskCount"], 18)
        self.assertEqual(plan["summary"]["cleanTaskCount"], 8)
        self.assertIn((16, 99), endpoints)   # L1->L2方向与区域1边界的真实交点
        self.assertNotIn((15, 73), endpoints)  # 区域内部L1不再是执行目标
        self.assertIn((26, 258), endpoints)  # 区域外L2仍保留原接入语义
        self.assertFalse(any(task["length"] <= 15 for task in plan["tasks"]))

    def test_generate_task_plan_requires_ready_preview(self):
        from modeling_task_generator import ModelingTaskGenerationError, generate_task_plan

        with self.assertRaises(ModelingTaskGenerationError):
            generate_task_plan({"id": "m1", "taskPreview": {"status": "empty"}})


if __name__ == "__main__":
    unittest.main()
