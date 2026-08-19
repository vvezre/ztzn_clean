import unittest


def _point(point_id, x, y):
    return {
        "id": point_id,
        "sequence": int(point_id[1:]),
        "x": x,
        "y": y,
        "lat": 32.0,
        "lon": 118.0,
    }


class ModelingPreviewTest(unittest.TestCase):
    def test_build_preview_generates_cleaning_lanes_for_confirmed_sub_area(self):
        from modeling_preview import build_model_preview

        points = [
            _point("p1", 0, 0),
            _point("p2", 1000, 0),
            _point("p3", 1000, 600),
            _point("p4", 0, 600),
        ]
        draft = {
            "id": "m1",
            "recognition": {"confirmed": True, "groupId": "g1"},
            "groups": [{
                "id": "g1",
                "name": "area-a",
                "areaNumber": 1,
                "sweepDirection": "auto",
                "points": points,
                "subAreas": [{
                    "id": "sa1",
                    "name": "sub-a",
                    "pointIds": ["p1", "p2", "p3", "p4"],
                }],
                "connectors": [],
            }],
            "groupLinks": [],
        }

        preview = build_model_preview(draft, now=1000)

        self.assertEqual(preview["status"], "ready")
        self.assertEqual(preview["summary"]["groupCount"], 1)
        self.assertEqual(preview["summary"]["subAreaCount"], 1)
        self.assertGreater(preview["summary"]["laneCount"], 1)
        self.assertEqual(preview["config"]["brushWidthCm"], 116.0)
        self.assertEqual(preview["config"]["overlapCm"], 53.0)
        self.assertEqual(preview["config"]["laneSpacingCm"], 63.0)
        lanes = preview["groups"][0]["subAreas"][0]["lanes"]
        # 1000x600区域应沿1000cm长轴清扫，不能因为先记录了下方长边就竖着扫。
        self.assertEqual(lanes[0]["heading"], 90.0)
        self.assertGreater(lanes[0]["lengthCm"], 0)

    def test_standard_panel_length_offers_four_and_five_lane_candidates(self):
        from modeling_preview import build_model_preview

        points = [
            _point("p1", 0, 0),
            _point("p2", 0, 226),
            _point("p3", 339, 226),
            _point("p4", 339, 0),
        ]
        draft = {
            "id": "standard-panel-row",
            "recognition": {"confirmed": True, "groupId": "g1"},
            "groups": [{
                "id": "g1",
                "name": "panel-row",
                "areaNumber": 1,
                "sweepDirection": "auto",
                "points": points,
                "subAreas": [{
                    "id": "sa1",
                    "name": "panel-row-area",
                    "pointIds": ["p1", "p2", "p3", "p4"],
                }],
                "connectors": [],
            }],
            "groupLinks": [],
        }

        preview = build_model_preview(draft, now=1000)

        self.assertEqual(preview["config"]["laneSpacingCm"], 63.0)
        sub_area = preview["groups"][0]["subAreas"][0]
        self.assertEqual(sub_area["laneCount"], 5)
        self.assertEqual(
            [candidate["laneCount"] for candidate in sub_area["laneCandidates"]],
            [4, 5, 6],
        )
        self.assertTrue(all(
            candidate["actualOverlapCm"] >= 30.0
            for candidate in sub_area["laneCandidates"]
        ))

    def test_default_policy_does_not_force_even_lane_count(self):
        from modeling_preview import build_model_preview

        points = [
            _point("p1", 0, 0),
            _point("p2", 0, 139),
            _point("p3", 300, 139),
            _point("p4", 300, 0),
        ]
        draft = {
            "id": "default-even-route",
            "recognition": {"confirmed": True, "groupId": "g1"},
            "groups": [{
                "id": "g1",
                "areaNumber": 1,
                "sweepDirection": "auto",
                "points": points,
                "subAreas": [{"id": "sa1", "pointIds": [point["id"] for point in points]}],
                "connectors": [],
            }],
            "groupLinks": [],
        }

        preview = build_model_preview(draft, now=1000)

        self.assertFalse(preview["config"]["forceEvenLanes"])
        self.assertEqual(preview["groups"][0]["subAreas"][0]["laneCount"], 3)
        self.assertEqual(
            [candidate["laneCount"] for candidate in preview["groups"][0]["subAreas"][0]["laneCandidates"]],
            [3, 4, 5],
        )

    def test_round_trip_policy_chooses_nearest_even_lane_count_per_area(self):
        from modeling_preview import build_model_preview

        area_one = [
            _point("p1", 0, 0),
            _point("p2", 0, 452),
            _point("p3", 565, 452),
            _point("p4", 565, 0),
        ]
        area_two = [
            _point("p5", 0, 552),
            _point("p6", 0, 778),
            _point("p7", 452, 778),
            _point("p8", 452, 552),
        ]
        draft = {
            "id": "even-round-trip",
            "recognition": {"confirmed": True},
            "routePolicy": {"forceEvenLanes": True},
            "groups": [
                {
                    "id": "g1",
                    "areaNumber": 1,
                    "sweepDirection": "auto",
                    "points": area_one,
                    "subAreas": [{"id": "s1", "pointIds": [point["id"] for point in area_one]}],
                },
                {
                    "id": "g2",
                    "areaNumber": 2,
                    "sweepDirection": "auto",
                    "points": area_two,
                    "subAreas": [{"id": "s2", "pointIds": [point["id"] for point in area_two]}],
                },
            ],
            "groupLinks": [],
        }

        preview = build_model_preview(draft, now=1000)
        first = preview["groups"][0]["subAreas"][0]
        second = preview["groups"][1]["subAreas"][0]

        self.assertEqual(first["laneCount"], 8)
        self.assertEqual(second["laneCount"], 4)
        self.assertEqual(first["laneSpacingCm"], 64.6)
        self.assertEqual(second["laneSpacingCm"], 75.3)

    def test_actual_overlap_never_drops_below_thirty_centimeters(self):
        from modeling_preview import build_model_preview

        points = [
            _point("p1", 0, 0),
            _point("p2", 0, 90),
            _point("p3", 300, 90),
            _point("p4", 300, 0),
        ]
        draft = {
            "id": "minimum-overlap",
            "recognition": {"confirmed": True},
            "groups": [{
                "id": "g1",
                "areaNumber": 1,
                "sweepDirection": "auto",
                "points": points,
                "subAreas": [{"id": "sa1", "pointIds": [point["id"] for point in points]}],
            }],
            "groupLinks": [],
        }

        preview = build_model_preview(draft, now=1000, overlap_cm=0)
        lanes = preview["groups"][0]["subAreas"][0]["lanes"]

        self.assertEqual(preview["config"]["overlapCm"], 30.0)
        self.assertGreaterEqual(len(lanes), 2)
        self.assertTrue(all(116.0 - lane["laneSpacingCm"] >= 30.0 for lane in lanes))

    def test_tilted_quadrilateral_preserves_both_real_boundary_lanes(self):
        from modeling_preview import build_model_preview

        # 来自真实小车模型的区域1坐标：上下边界天然不平行，且包含厘米级定位误差。
        points = [
            _point("p1", 0.0, 0.0),
            _point("p2", 7.362, 117.944),
            _point("p3", 344.374, 86.365),
            _point("p4", 342.394, -61.646),
        ]
        draft = {
            "id": "tilted-real-boundary",
            "recognition": {"confirmed": True},
            "groups": [{
                "id": "g1",
                "areaNumber": 1,
                "sweepDirection": "auto",
                "points": points,
                "subAreas": [{"id": "sa1", "pointIds": [point["id"] for point in points]}],
                "connectors": [],
            }],
            "groupLinks": [],
        }

        preview = build_model_preview(draft, now=1000)
        lanes = preview["groups"][0]["subAreas"][0]["lanes"]

        self.assertGreaterEqual(len(lanes), 2)
        # 第一条精确保留 p2 -> p3 上边界。
        self.assertEqual(
            (lanes[0]["startX"], lanes[0]["startY"], lanes[0]["endX"], lanes[0]["endY"]),
            (7.4, 117.9, 344.4, 86.4),
        )
        # 最后一条精确保留 p1 -> p4 下边界。
        self.assertEqual(
            (lanes[-1]["startX"], lanes[-1]["startY"], lanes[-1]["endX"], lanes[-1]["endY"]),
            (0.0, 0.0, 342.4, -61.6),
        )
        # 两条真实边界可以有不同航向，不能再强迫它们绝对平行。
        self.assertNotEqual(lanes[0]["heading"], lanes[-1]["heading"])

    def test_multi_point_boundary_lanes_keep_recorded_folds_and_interior_lanes_are_straight(self):
        from modeling_preview import build_model_preview

        # 从左下角开始，先沿左侧向上记录；上下两条真实边界都包含人工记录的波动点。
        points = [
            _point("p1", 0, 0),
            _point("p2", 2, 50),
            _point("p3", 0, 100),
            _point("p4", 50, 108),
            _point("p5", 100, 92),
            _point("p6", 200, 100),
            _point("p7", 198, 50),
            _point("p8", 200, 0),
            _point("p9", 150, -5),
            _point("p10", 75, 8),
        ]
        draft = {
            "id": "folded-boundary",
            "recognition": {"confirmed": True},
            "groups": [{
                "id": "g1",
                "areaNumber": 1,
                "sweepDirection": "auto",
                "points": points,
                "subAreas": [{"id": "sa1", "pointIds": [point["id"] for point in points]}],
            }],
            "groupLinks": [],
        }

        preview = build_model_preview(draft, now=1000)
        lanes = preview["groups"][0]["subAreas"][0]["lanes"]

        self.assertEqual(lanes[0]["laneType"], "boundary")
        self.assertEqual(lanes[-1]["laneType"], "boundary")
        self.assertEqual(
            [(point["x"], point["y"]) for point in lanes[0]["pathPoints"]],
            [(0.0, 100.0), (50.0, 108.0), (100.0, 92.0), (200.0, 100.0)],
        )
        self.assertEqual(
            [(point["x"], point["y"]) for point in lanes[-1]["pathPoints"]],
            [(0.0, 0.0), (75.0, 8.0), (150.0, -5.0), (200.0, 0.0)],
        )
        self.assertTrue(all(
            lane["laneType"] == "interior" and len(lane["pathPoints"]) == 2
            for lane in lanes[1:-1]
        ))

    def test_near_last_recorded_point_merges_into_first_only_for_planning(self):
        from modeling_preview import build_model_preview

        # p6是绕区域一圈后回到p1附近的闭合重复点，二者相距约16.6cm。
        # 规划多边形应使用p1闭合，不能再生成p6->p1短边；原始数据必须保持不变。
        points = [
            _point("p1", 20.662, 278.877),
            _point("p2", 24.301, 397.400),
            _point("p3", 188.843, 387.125),
            _point("p4", 380.344, 371.936),
            _point("p5", 369.240, 247.031),
            _point("p6", 5.769, 271.349),
        ]
        draft = {
            "id": "near-boundary-closure",
            "recognition": {"confirmed": True},
            "groups": [{
                "id": "g1",
                "areaNumber": 1,
                "sweepDirection": "auto",
                "points": points,
                "subAreas": [{"id": "sa1", "pointIds": [point["id"] for point in points]}],
            }],
            "groupLinks": [],
        }

        preview = build_model_preview(draft, now=1000)
        sub_area = preview["groups"][0]["subAreas"][0]

        self.assertEqual(sub_area["pointIds"], ["p1", "p2", "p3", "p4", "p5"])
        self.assertEqual([point["id"] for point in draft["groups"][0]["points"]], [
            "p1", "p2", "p3", "p4", "p5", "p6",
        ])
        self.assertFalse(any(
            [(point["x"], point["y"]) for point in lane.get("pathPoints") or []][-2:]
            == [(5.8, 271.3), (20.7, 278.9)]
            for lane in sub_area["lanes"]
        ))

    def test_side_fluctuation_uses_the_whole_shape_for_sweep_direction(self):
        from modeling_preview import build_model_preview

        # 四边都存在真实波动。自动方向应由完整区域的长轴决定，不能被p1->p2
        # 这一小段带歪；整体仍应保持接近横向清扫。
        points = [
            _point("p1", 0, 0),
            _point("p2", 25, 30),
            _point("p3", -20, 70),
            _point("p4", 0, 120),
            _point("p5", 100, 128),
            _point("p6", 200, 110),
            _point("p7", 300, 120),
            _point("p8", 320, 80),
            _point("p9", 282, 40),
            _point("p10", 300, 0),
            _point("p11", 200, -10),
            _point("p12", 100, 8),
        ]
        draft = {
            "id": "all-edges-fluctuate",
            "recognition": {"confirmed": True},
            "groups": [{
                "id": "g1",
                "areaNumber": 1,
                "sweepDirection": "auto",
                "points": points,
                "subAreas": [{"id": "sa1", "pointIds": [point["id"] for point in points]}],
            }],
            "groupLinks": [],
        }

        preview = build_model_preview(draft, now=1000)
        sub_area = preview["groups"][0]["subAreas"][0]
        lanes = sub_area["lanes"]

        self.assertLess(abs(sub_area["sweepAngle"] - 90.0), 5.0)
        self.assertGreaterEqual(len(lanes), 2)
        self.assertEqual(lanes[0]["laneType"], "boundary")
        self.assertEqual(lanes[-1]["laneType"], "boundary")
        self.assertTrue(all(
            lane["laneType"] == "interior" and len(lane["pathPoints"]) == 2
            for lane in lanes[1:-1]
        ))

    def test_auto_sweep_is_invariant_to_start_point_and_recording_direction(self):
        from modeling_preview import build_model_preview

        base = [
            _point("p1", 0, 0),
            _point("p2", 0, 120),
            _point("p3", 400, 120),
            _point("p4", 400, 0),
        ]

        def preview_for(points):
            draft = {
                "id": "order-invariant",
                "recognition": {"confirmed": True},
                "groups": [{
                    "id": "g1",
                    "areaNumber": 1,
                    "sweepDirection": "auto",
                    "points": points,
                    "subAreas": [{
                        "id": "sa1",
                        "pointIds": [point["id"] for point in points],
                    }],
                }],
                "groupLinks": [],
            }
            return build_model_preview(draft, now=1000)["groups"][0]["subAreas"][0]

        variants = []
        for offset in range(len(base)):
            variants.append(base[offset:] + base[:offset])
        reversed_base = list(reversed(base))
        for offset in range(len(reversed_base)):
            variants.append(reversed_base[offset:] + reversed_base[:offset])

        results = [preview_for(points) for points in variants]
        self.assertEqual({result["sweepAngle"] for result in results}, {90.0})

        def lane_signature(result):
            return [
                (
                    tuple((point["x"], point["y"]) for point in lane.get("pathPoints") or []),
                    lane.get("laneType"),
                )
                for lane in result["lanes"]
            ]

        expected = lane_signature(results[0])
        self.assertTrue(all(lane_signature(result) == expected for result in results[1:]))

    def test_111_like_regions_both_choose_horizontal_long_axis(self):
        from modeling_preview import build_model_preview

        area_one = [
            _point("p1", 0.0, 0.0),
            _point("p2", 7.334, 123.771),
            _point("p3", 12.480, 123.482),
            _point("p4", 365.686, 106.002),
            _point("p5", 931.441, 51.061),
            _point("p6", 921.280, -73.578),
            _point("p7", 594.147, -47.213),
        ]
        # 区域二先记录了横向长边；旧算法会因此生成竖向短清扫线。
        area_two = [
            _point("q1", 824.398, 192.145),
            _point("q2", 1189.528, 172.986),
            _point("q3", 1195.900, 294.633),
            _point("q4", 868.060, 340.668),
        ]
        draft = {
            "id": "111-shape-directions",
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
            "groupLinks": [],
        }

        preview = build_model_preview(draft, now=1000)
        angles = [group["subAreas"][0]["sweepAngle"] for group in preview["groups"]]
        self.assertTrue(all(abs(angle - 90.0) < 15.0 for angle in angles))
        for group in preview["groups"]:
            lanes = group["subAreas"][0]["lanes"]
            self.assertTrue(all(abs(lane["endX"] - lane["startX"]) > abs(lane["endY"] - lane["startY"]) for lane in lanes))

    def test_build_preview_requires_confirmed_recognition(self):
        from modeling_preview import ModelingPreviewError, build_model_preview

        with self.assertRaises(ModelingPreviewError):
            build_model_preview({
                "id": "m1",
                "recognition": {"confirmed": False},
                "groups": [],
                "groupLinks": [],
            })

    def test_build_preview_includes_group_links(self):
        from modeling_preview import build_model_preview

        draft = {
            "id": "m1",
            "recognition": {"confirmed": True, "groupId": "g1"},
            "groups": [{
                "id": "g1",
                "name": "area-a",
                "areaNumber": 1,
                "sweepDirection": "manual",
                "sweepAngle": 90,
                "points": [_point("p1", 0, 0), _point("p2", 100, 0), _point("p3", 100, 100), _point("p4", 0, 100)],
                "subAreas": [{"id": "sa1", "name": "sub-a", "pointIds": ["p1", "p2", "p3", "p4"]}],
                "connectors": [],
            }],
            "groupLinks": [{
                "id": "l1",
                "type": "group_connector",
                "startGroupId": "g1",
                "endGroupId": "g2",
                "points": [
                    {"id": "lp1", "sequence": 1, "x": 10, "y": 10, "lat": 32.0, "lon": 118.0},
                    {"id": "lp2", "sequence": 2, "x": 100, "y": 10, "lat": 32.0, "lon": 118.0},
                    {"id": "lp3", "sequence": 3, "x": 100, "y": 100, "lat": 32.0, "lon": 118.0},
                ],
                "status": "ready",
            }],
        }

        preview = build_model_preview(draft, now=1000)

        self.assertEqual(len(preview["groupLinks"]), 1)
        self.assertEqual(preview["groupLinks"][0]["startPoint"]["id"], "lp1")
        self.assertEqual(preview["groupLinks"][0]["endPoint"]["id"], "lp3")
        self.assertEqual(
            [point["id"] for point in preview["groupLinks"][0]["points"]],
            ["lp1", "lp2", "lp3"],
        )
        self.assertEqual(preview["groupLinks"][0]["lengthCm"], 180.0)


if __name__ == "__main__":
    unittest.main()
