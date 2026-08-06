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
        self.assertEqual(lanes[0]["heading"], 180.0)
        self.assertGreater(lanes[0]["lengthCm"], 0)

    def test_standard_panel_length_generates_four_cleaning_lanes(self):
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
        self.assertEqual(preview["groups"][0]["subAreas"][0]["laneCount"], 4)

    def test_default_policy_uses_nearest_even_lane_count(self):
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

        self.assertTrue(preview["config"]["forceEvenLanes"])
        self.assertEqual(preview["groups"][0]["subAreas"][0]["laneCount"], 4)

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

        self.assertEqual(len(lanes), 4)
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
                    {"id": "lp2", "sequence": 2, "x": 200, "y": 10, "lat": 32.0, "lon": 118.0},
                ],
                "status": "ready",
            }],
        }

        preview = build_model_preview(draft, now=1000)

        self.assertEqual(len(preview["groupLinks"]), 1)
        self.assertEqual(preview["groupLinks"][0]["startPoint"]["id"], "lp1")
        self.assertEqual(preview["groupLinks"][0]["endPoint"]["id"], "lp2")
        self.assertEqual(
            [point["id"] for point in preview["groupLinks"][0]["points"]],
            ["lp1", "lp2"],
        )


if __name__ == "__main__":
    unittest.main()
