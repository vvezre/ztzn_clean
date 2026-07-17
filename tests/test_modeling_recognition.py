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


class ModelingRecognitionTest(unittest.TestCase):
    def test_rectangle_points_become_one_sub_area_without_connectors(self):
        from modeling_recognition import recognize_group_points

        points = [
            _point("p1", 0, 0),
            _point("p2", 1000, 0),
            _point("p3", 1000, 600),
            _point("p4", 0, 600),
        ]

        result = recognize_group_points({"id": "g1", "points": points})

        self.assertEqual(result["status"], "recognized")
        self.assertEqual(len(result["subAreas"]), 1)
        self.assertEqual(result["subAreas"][0]["pointIds"], ["p1", "p2", "p3", "p4"])
        self.assertEqual(result["connectors"], [])
        self.assertTrue(all(point["role"] == "boundary_corner" for point in result["points"]))

    def test_opposite_turn_segment_becomes_sub_area_connector(self):
        from modeling_recognition import recognize_group_points

        points = [
            _point("p1", 0, 0),
            _point("p2", 1000, 0),
            _point("p3", 1000, 400),
            _point("p4", 1800, 400),
            _point("p5", 1800, 0),
            _point("p6", 2600, 0),
            _point("p7", 2600, 800),
            _point("p8", 1800, 800),
            _point("p9", 1000, 800),
            _point("p10", 0, 800),
        ]

        result = recognize_group_points({"id": "g1", "points": points})

        self.assertEqual(result["status"], "recognized")
        self.assertFalse(result["needsConfirmation"])
        self.assertEqual(len(result["connectors"]), 1)
        self.assertEqual(result["connectors"][0]["pointIds"], ["p3", "p4"])
        self.assertEqual(result["connectors"][0]["type"], "sub_area_connector")
        self.assertFalse(result["connectors"][0]["needsConfirmation"])
        self.assertEqual(len(result["subAreas"]), 2)
        self.assertFalse(any(area["needsConfirmation"] for area in result["subAreas"]))
        self.assertEqual(result["points"][2]["role"], "connection_point")
        self.assertEqual(result["points"][3]["role"], "connection_point")

    def test_embedded_connector_points_in_clockwise_large_boundary_are_auto_sub_areas(self):
        from modeling_recognition import recognize_group_points

        points = [
            _point("p1", 0, 0),       # A1
            _point("p2", 1000, 0),    # B1
            _point("p3", 1000, 420),  # E1 on B1-C1
            _point("p4", 1800, 420),  # E2 on A2-D2
            _point("p5", 1800, 0),    # A2
            _point("p6", 2600, 0),    # B2
            _point("p7", 2600, 800),  # C2
            _point("p8", 1800, 800),  # D2
            _point("p9", 1000, 800),  # C1
            _point("p10", 0, 800),    # D1
        ]

        result = recognize_group_points({"id": "g1", "points": points})

        self.assertEqual(result["status"], "recognized")
        self.assertFalse(result["needsConfirmation"])
        self.assertEqual(result["connectors"][0]["pointIds"], ["p3", "p4"])
        self.assertEqual(result["summary"]["subAreaCount"], 2)
        self.assertEqual(result["summary"]["connectorCount"], 1)
        self.assertEqual(result["subAreas"][0]["pointIds"], ["p1", "p2", "p3", "p9", "p10"])
        self.assertEqual(result["subAreas"][1]["pointIds"], ["p4", "p5", "p6", "p7", "p8"])

    def test_curved_edge_helper_points_are_not_connectors(self):
        from modeling_recognition import recognize_group_points

        points = [
            _point("p1", 0, 0),
            _point("p2", 1000, 0),
            _point("p3", 1000, 250),
            _point("p4", 980, 500),
            _point("p5", 1000, 750),
            _point("p6", 1000, 1000),
            _point("p7", 0, 1000),
        ]

        result = recognize_group_points({"id": "g1", "points": points})

        self.assertEqual(result["connectors"], [])
        helper_roles = {
            point["id"]: point["role"]
            for point in result["points"]
        }
        self.assertEqual(helper_roles["p3"], "boundary_assist")
        self.assertEqual(helper_roles["p4"], "boundary_assist")
        self.assertEqual(helper_roles["p5"], "boundary_assist")

    def test_less_than_four_points_reports_insufficient_points(self):
        from modeling_recognition import recognize_group_points

        result = recognize_group_points({"id": "g1", "points": [_point("p1", 0, 0), _point("p2", 100, 0)]})

        self.assertEqual(result["status"], "insufficient_points")
        self.assertTrue(result["needsConfirmation"])
        self.assertEqual(result["message"], "至少需要4个点才能识别区域")


if __name__ == "__main__":
    unittest.main()
