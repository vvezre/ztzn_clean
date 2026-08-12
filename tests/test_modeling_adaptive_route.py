# coding=utf-8
import unittest


def _point(point_id, x, y):
    return {
        "id": point_id,
        "x": float(x),
        "y": float(y),
        "lat": 32.0,
        "lon": 118.0,
    }


def _two_area_draft(bridge_on_right=False):
    home = [
        _point("h1", 0, 0),
        _point("h2", 0, 128),
        _point("h3", 400, 128),
        _point("h4", 400, 0),
    ]
    remote = [
        _point("r1", 500, 0),
        _point("r2", 500, 128),
        _point("r3", 900, 128),
        _point("r4", 900, 0),
    ]
    link = [
        _point("l1", 400 if bridge_on_right else 0, 128),
        _point("l2", 500, 128),
    ]
    return {
        "id": "adaptive-route",
        "recognition": {"confirmed": True},
        "groups": [
            {
                "id": "g1",
                "areaNumber": 1,
                "points": home,
                "subAreas": [{"id": "s1", "pointIds": [item["id"] for item in home]}],
            },
            {
                "id": "g2",
                "areaNumber": 2,
                "points": remote,
                "subAreas": [{"id": "s2", "pointIds": [item["id"] for item in remote]}],
            },
        ],
        "groupLinks": [{
            "id": "bridge",
            "startGroupId": "g1",
            "endGroupId": "g2",
            "points": link,
        }],
    }


class ModelingAdaptiveRouteTest(unittest.TestCase):
    def _plan(self, draft):
        from modeling_preview import build_model_preview
        from modeling_task_generator import generate_task_plan

        draft["taskPreview"] = build_model_preview(draft, now=100)
        return draft["taskPreview"], generate_task_plan(draft, now=200)

    def test_lane_candidates_include_both_parities_and_keep_minimum_overlap(self):
        preview, _ = self._plan(_two_area_draft())
        self.assertFalse(preview["config"]["forceEvenLanes"])
        candidates = preview["groups"][0]["subAreas"][0]["laneCandidates"]
        self.assertIn(3, [item["laneCount"] for item in candidates])
        self.assertIn(4, [item["laneCount"] for item in candidates])
        self.assertTrue(all(item["actualOverlapCm"] >= 30.0 for item in candidates))

    def test_same_side_entry_and_exit_select_even_lanes(self):
        _, plan = self._plan(_two_area_draft(bridge_on_right=False))
        self.assertEqual(plan["areaOrder"], [1, 2])
        self.assertEqual(plan["routeSelections"][0]["laneCount"], 4)
        first_clean = next(task for task in plan["tasks"] if task["mode"] == 1)
        self.assertEqual(first_clean["areaNumber"], 1)

    def test_opposite_side_exit_can_select_odd_lanes(self):
        _, plan = self._plan(_two_area_draft(bridge_on_right=True))
        self.assertEqual(plan["routeSelections"][0]["laneCount"], 3)

    def test_frontend_area_order_changes_cleaning_order(self):
        draft = _two_area_draft()
        draft["routePolicy"] = {"type": "area_order", "areaOrder": [2, 1]}
        _, plan = self._plan(draft)
        self.assertEqual(plan["areaOrder"], [2, 1])
        first_clean = next(task for task in plan["tasks"] if task["mode"] == 1)
        self.assertEqual(first_clean["areaNumber"], 2)

    def test_area_order_must_be_complete_and_unique(self):
        from modeling_preview import build_model_preview
        from modeling_task_generator import ModelingTaskGenerationError, generate_task_plan

        draft = _two_area_draft()
        draft["routePolicy"] = {"type": "area_order", "areaOrder": [1]}
        draft["taskPreview"] = build_model_preview(draft, now=100)
        with self.assertRaises(ModelingTaskGenerationError):
            generate_task_plan(draft, now=200)

    def test_boundary_return_compares_forward_and_reverse(self):
        from modeling_task_generator import _CoordinateMapper, _group_anchor_transition_points

        draft = _two_area_draft()
        mapper = _CoordinateMapper(draft)
        path = _group_anchor_transition_points(
            draft,
            "g1",
            (0.0, 128.0),
            (0.0, 0.0),
            mapper,
        )
        self.assertEqual(path, [(0.0, 128.0), (0.0, 0.0)])

    def test_three_area_reorder_uses_bridge_graph_and_intermediate_boundary(self):
        area_one = [
            _point("a1", 0, 0), _point("a2", 0, 128),
            _point("a3", 400, 128), _point("a4", 400, 0),
        ]
        area_two = [
            _point("b1", 0, 228), _point("b2", 0, 356),
            _point("b3", 400, 356), _point("b4", 400, 228),
        ]
        area_three = [
            _point("c1", 500, 228), _point("c2", 500, 356),
            _point("c3", 900, 356), _point("c4", 900, 228),
        ]
        draft = {
            "id": "three-area-reorder",
            "recognition": {"confirmed": True},
            "routePolicy": {"type": "area_order", "areaOrder": [2, 1, 3]},
            "groups": [
                {"id": "g1", "areaNumber": 1, "points": area_one,
                 "subAreas": [{"id": "s1", "pointIds": [p["id"] for p in area_one]}]},
                {"id": "g2", "areaNumber": 2, "points": area_two,
                 "subAreas": [{"id": "s2", "pointIds": [p["id"] for p in area_two]}]},
                {"id": "g3", "areaNumber": 3, "points": area_three,
                 "subAreas": [{"id": "s3", "pointIds": [p["id"] for p in area_three]}]},
            ],
            "groupLinks": [
                {"id": "bridge-12", "startGroupId": "g1", "endGroupId": "g2",
                 "points": [_point("l1", 0, 128), _point("l2", 0, 228)]},
                {"id": "bridge-23", "startGroupId": "g2", "endGroupId": "g3",
                 "points": [_point("m1", 400, 300), _point("m2", 500, 300)]},
            ],
        }

        _, plan = self._plan(draft)
        tasks = plan["tasks"]
        clean_areas = [task["areaNumber"] for task in tasks if task["mode"] == 1]
        first_area_one = clean_areas.index(1)
        first_area_three = clean_areas.index(3)
        self.assertTrue(all(area == 2 for area in clean_areas[:first_area_one]))
        self.assertTrue(all(area == 1 for area in clean_areas[first_area_one:first_area_three]))
        self.assertTrue(all(area == 3 for area in clean_areas[first_area_three:]))

        last_area_one_task = max(
            index for index, task in enumerate(tasks)
            if task["mode"] == 1 and task["areaNumber"] == 1
        )
        first_area_three_task = min(
            index for index, task in enumerate(tasks)
            if task["mode"] == 1 and task["areaNumber"] == 3
        )
        transfer_points = {
            (task[prefix + "X"], task[prefix + "Y"])
            for task in tasks[last_area_one_task + 1:first_area_three_task]
            for prefix in ("start", "end")
        }
        # 区域2右侧桥头(400,300)位于右边界中部。投影点会把右边界切开，
        # 程序比较完整的上下两条边界路线后，应走更短的左下->右下->投影，
        # 而不是因投影略靠近右上角就绕行左上、右上再折返。
        self.assertIn((400, 228), transfer_points)
        self.assertIn((400, 300), transfer_points)
        self.assertIn((500, 300), transfer_points)
        self.assertNotIn((0, 356), transfer_points)
        self.assertNotIn((400, 356), transfer_points)

        self.assertEqual((tasks[0]["startX"], tasks[0]["startY"]), (0, 0))
        self.assertEqual((tasks[-1]["endX"], tasks[-1]["endY"]), (0, 0))
        for index in range(1, len(tasks)):
            self.assertEqual(
                (tasks[index - 1]["endX"], tasks[index - 1]["endY"]),
                (tasks[index]["startX"], tasks[index]["startY"]),
            )


if __name__ == "__main__":
    unittest.main()
