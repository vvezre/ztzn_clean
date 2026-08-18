# coding=utf-8
import unittest


class ModelingCaptureTest(unittest.TestCase):
    def test_mixed_capture_preserves_all_connection_polyline_points(self):
        from modeling_capture import resolve_mixed_capture

        def point(point_id, x, y):
            return {"id": point_id, "x": x, "y": y}

        area_points = [
            point("h1", 0, 0),
            point("h2", 0, 100),
            point("r1", 300, 0),
            point("r2", 300, 100),
            point("h3", 100, 100),
            point("h4", 100, 0),
            point("r3", 400, 100),
            point("r4", 400, 0),
        ]
        link_points = [
            point("l1", 100, 50),
            point("l2", 200, 80),
            point("l3", 300, 50),
        ]
        events = [
            {"pointType": "area", "pointId": "h1"},
            {"pointType": "area", "pointId": "h2"},
            {"pointType": "link", "pointId": "l1"},
            {"pointType": "link", "pointId": "l2"},
            {"pointType": "link", "pointId": "l3"},
            {"pointType": "area", "pointId": "r1"},
            {"pointType": "area", "pointId": "r2"},
            {"pointType": "area", "pointId": "h3"},
            {"pointType": "area", "pointId": "h4"},
            {"pointType": "area", "pointId": "r3"},
            {"pointType": "area", "pointId": "r4"},
        ]
        draft = {
            "groups": [
                {"id": "g1", "points": area_points},
                {"id": "g2", "points": []},
            ],
            "groupLinks": [{
                "id": "bridge",
                "startGroupId": "g1",
                "endGroupId": "g2",
                "points": link_points,
            }],
            "captureSequence": events,
        }

        resolved = resolve_mixed_capture(draft)
        bridge = resolved["groupLinks"][0]

        self.assertEqual(
            [item["id"] for item in bridge["points"]],
            ["l1", "l2", "l3"],
        )
        self.assertEqual(
            [item["role"] for item in bridge["points"]],
            ["group_link_start", "group_link_waypoint", "group_link_end"],
        )
        self.assertEqual(
            resolved["routePolicy"]["outboundPointIds"][2:5],
            ["l1", "l2", "l3"],
        )
        self.assertEqual(
            resolved["routePolicy"]["returnPointIds"][1:4],
            ["l3", "l2", "l1"],
        )


if __name__ == "__main__":
    unittest.main()
