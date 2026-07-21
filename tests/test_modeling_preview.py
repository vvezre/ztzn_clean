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
        self.assertEqual(preview["config"]["overlapCm"], 40.7)
        self.assertEqual(preview["config"]["laneSpacingCm"], 75.3)
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

        self.assertEqual(preview["config"]["laneSpacingCm"], 75.3)
        self.assertEqual(preview["groups"][0]["subAreas"][0]["laneCount"], 4)

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


if __name__ == "__main__":
    unittest.main()
