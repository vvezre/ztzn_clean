import math
import random
import unittest


def _rotate(x, y, angle_radians):
    cosine = math.cos(angle_radians)
    sine = math.sin(angle_radians)
    return (
        x * cosine - y * sine,
        x * sine + y * cosine,
    )


def _point(point_id, x, y):
    return {
        "id": point_id,
        "x": round(x, 6),
        "y": round(y, 6),
        "lat": 32.0,
        "lon": 118.0,
    }


def _assert_continuous_round_trip(test_case, tasks, origin=(0, 0)):
    test_case.assertTrue(tasks)
    test_case.assertEqual((tasks[0]["startX"], tasks[0]["startY"]), origin)
    test_case.assertEqual((tasks[-1]["endX"], tasks[-1]["endY"]), origin)
    for index, task in enumerate(tasks):
        test_case.assertGreater(task["length"], 0)
        test_case.assertIn(task["mode"], (1, 2))
        if index:
            previous = tasks[index - 1]
            test_case.assertEqual(
                (previous["endX"], previous["endY"]),
                (task["startX"], task["startY"]),
            )


class ModelingRoutePropertyTest(unittest.TestCase):
    def test_random_rotated_rectangles_keep_even_lanes_continuity_and_no_mergeable_stops(self):
        from modeling_preview import _nearest_even_lane_count, build_model_preview
        from modeling_task_generator import _same_direction_collinear, generate_task_plan

        randomizer = random.Random(20260803)
        for case_index in range(120):
            clean_span = randomizer.uniform(80.0, 1500.0)
            clean_length = randomizer.uniform(120.0, 3000.0)
            angle = randomizer.uniform(-math.pi, math.pi)
            raw_points = [
                (0.0, 0.0),
                (0.0, clean_span),
                (clean_length, clean_span),
                (clean_length, 0.0),
            ]
            rotated = [_rotate(x, y, angle) for x, y in raw_points]
            points = [_point("p{}".format(index + 1), x, y) for index, (x, y) in enumerate(rotated)]
            # 每三组增加一个首边中间采样点，确认普通边界点不会额外制造停车任务。
            if case_index % 3 == 0:
                middle = _rotate(0.0, clean_span / 2.0, angle)
                points.insert(1, _point("assist", middle[0], middle[1]))

            draft = {
                "id": "random-rectangle-{}".format(case_index),
                "recognition": {"confirmed": True},
                "groups": [{
                    "id": "g1",
                    "areaNumber": 1,
                    "points": points,
                    "subAreas": [{
                        "id": "sa1",
                        "pointIds": [point["id"] for point in points],
                    }],
                }],
                "groupLinks": [],
            }
            preview = build_model_preview(draft, now=1000)
            draft["taskPreview"] = preview
            plan = generate_task_plan(draft, now=2000)
            lanes = preview["groups"][0]["subAreas"][0]["lanes"]
            tasks = plan["tasks"]

            expected_lane_count = _nearest_even_lane_count(clean_span, 63.0)
            self.assertEqual(len(lanes), expected_lane_count, case_index)
            self.assertEqual(len(lanes) % 2, 0, case_index)
            self.assertEqual(plan["summary"]["cleanTaskCount"], len(lanes), case_index)
            _assert_continuous_round_trip(self, tasks)
            for index in range(1, len(tasks)):
                self.assertFalse(
                    _same_direction_collinear(tasks[index - 1], tasks[index]),
                    "case {} still contains a mergeable stop at task {}".format(case_index, index + 1),
                )

    def test_random_tilted_quadrilaterals_keep_real_outer_boundaries(self):
        from modeling_preview import _is_convex_quadrilateral, build_model_preview
        from modeling_task_generator import generate_task_plan

        randomizer = random.Random(20260805)
        tested = 0
        for case_index in range(100):
            width = randomizer.uniform(250.0, 1800.0)
            left_height = randomizer.uniform(100.0, 900.0)
            right_height = left_height + randomizer.uniform(-90.0, 90.0)
            left_skew = randomizer.uniform(-35.0, 35.0)
            right_skew = randomizer.uniform(-35.0, 35.0)
            bottom_right_y = randomizer.uniform(-35.0, 35.0)
            angle = randomizer.uniform(-math.pi, math.pi)
            raw_points = [
                (0.0, 0.0),
                (left_skew, left_height),
                (width + right_skew, right_height),
                (width, bottom_right_y),
            ]
            rotated = [_rotate(x, y, angle) for x, y in raw_points]
            if not _is_convex_quadrilateral(rotated):
                continue
            tested += 1
            points = [_point("p{}".format(index + 1), x, y) for index, (x, y) in enumerate(rotated)]
            draft = {
                "id": "tilted-quadrilateral-{}".format(case_index),
                "recognition": {"confirmed": True},
                "groups": [{
                    "id": "g1",
                    "areaNumber": 1,
                    "points": points,
                    "subAreas": [{"id": "sa1", "pointIds": [point["id"] for point in points]}],
                }],
                "groupLinks": [],
            }

            preview = build_model_preview(draft, now=1000)
            draft["taskPreview"] = preview
            plan = generate_task_plan(draft, now=2000)
            lanes = preview["groups"][0]["subAreas"][0]["lanes"]

            self.assertGreaterEqual(len(lanes), 2, case_index)
            self.assertEqual(len(lanes) % 2, 0, case_index)
            self.assertAlmostEqual(lanes[0]["startX"], points[1]["x"], places=1)
            self.assertAlmostEqual(lanes[0]["startY"], points[1]["y"], places=1)
            self.assertAlmostEqual(lanes[0]["endX"], points[2]["x"], places=1)
            self.assertAlmostEqual(lanes[0]["endY"], points[2]["y"], places=1)
            self.assertAlmostEqual(lanes[-1]["startX"], points[0]["x"], places=1)
            self.assertAlmostEqual(lanes[-1]["startY"], points[0]["y"], places=1)
            self.assertAlmostEqual(lanes[-1]["endX"], points[3]["x"], places=1)
            self.assertAlmostEqual(lanes[-1]["endY"], points[3]["y"], places=1)
            self.assertEqual(plan["summary"]["cleanTaskCount"], len(lanes), case_index)
            _assert_continuous_round_trip(self, plan["tasks"])

        self.assertGreaterEqual(tested, 90)

    def test_random_two_area_routes_clean_remote_first_and_preserve_bridge_turns(self):
        from modeling_preview import build_model_preview
        from modeling_task_generator import _same_direction_collinear, generate_task_plan

        randomizer = random.Random(20260804)
        for case_index in range(40):
            home_height = randomizer.uniform(180.0, 900.0)
            remote_height = randomizer.uniform(180.0, 700.0)
            gap = randomizer.uniform(40.0, 250.0)
            home_width = randomizer.uniform(300.0, 1800.0)
            remote_width = randomizer.uniform(200.0, 1200.0)
            remote_x = randomizer.uniform(-120.0, 120.0)
            remote_bottom = home_height + gap

            home_points = [
                _point("h1", 0, 0),
                _point("h2", 0, home_height),
                _point("h3", home_width, home_height),
                _point("h4", home_width, 0),
            ]
            remote_points = [
                _point("r1", remote_x, remote_bottom),
                _point("r2", remote_x, remote_bottom + remote_height),
                _point("r3", remote_x + remote_width, remote_bottom + remote_height),
                _point("r4", remote_x + remote_width, remote_bottom),
            ]
            link_points = [
                _point("l1", 0, home_height),
                _point("l2", remote_x, remote_bottom),
            ]
            draft = {
                "id": "two-area-{}".format(case_index),
                "recognition": {"confirmed": True},
                "groups": [
                    {
                        "id": "home",
                        "areaNumber": 1,
                        "points": home_points,
                        "subAreas": [{"id": "home-area", "pointIds": [point["id"] for point in home_points]}],
                    },
                    {
                        "id": "remote",
                        "areaNumber": 2,
                        "points": remote_points,
                        "subAreas": [{"id": "remote-area", "pointIds": [point["id"] for point in remote_points]}],
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
            preview = build_model_preview(draft, now=1000)
            draft["taskPreview"] = preview
            plan = generate_task_plan(draft, now=2000)
            tasks = plan["tasks"]
            clean_areas = [task["areaNumber"] for task in tasks if task["mode"] == 1]

            self.assertEqual(plan["routeType"], "bridge_round_trip", case_index)
            first_home_index = clean_areas.index(1)
            self.assertTrue(all(area == 2 for area in clean_areas[:first_home_index]), case_index)
            self.assertTrue(all(area == 1 for area in clean_areas[first_home_index:]), case_index)
            _assert_continuous_round_trip(self, tasks)
            for index in range(1, len(tasks)):
                self.assertFalse(_same_direction_collinear(tasks[index - 1], tasks[index]), case_index)

    def test_real_tilted_two_area_route_keeps_boundary_lanes_and_bridge_round_trip(self):
        from modeling_preview import build_model_preview
        from modeling_task_generator import _same_direction_collinear, generate_task_plan

        home_points = [
            _point("h1", 0.0, 0.0),
            _point("h2", 7.362, 117.944),
            _point("h3", 344.374, 86.365),
            _point("h4", 342.394, -61.646),
        ]
        remote_points = [
            _point("r1", 16.882, 260.83),
            _point("r2", 23.914, 399.779),
            _point("r3", 379.429, 363.04),
            _point("r4", 368.523, 235.322),
        ]
        link_points = [
            _point("l1", 9.502, 170.529),
            _point("l2", 10.576, 204.143),
        ]
        draft = {
            "id": "real-tilted-two-area",
            "recognition": {"confirmed": True},
            "groups": [
                {
                    "id": "home",
                    "areaNumber": 1,
                    "points": home_points,
                    "subAreas": [{"id": "home-area", "pointIds": [point["id"] for point in home_points]}],
                },
                {
                    "id": "remote",
                    "areaNumber": 2,
                    "points": remote_points,
                    "subAreas": [{"id": "remote-area", "pointIds": [point["id"] for point in remote_points]}],
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

        preview = build_model_preview(draft, now=1000)
        draft["taskPreview"] = preview
        plan = generate_task_plan(draft, now=2000)
        tasks = plan["tasks"]
        home_lanes = preview["groups"][0]["subAreas"][0]["lanes"]
        remote_lanes = preview["groups"][1]["subAreas"][0]["lanes"]

        self.assertEqual(len(home_lanes), 4)
        self.assertEqual(len(remote_lanes), 4)
        self.assertEqual(
            (home_lanes[0]["startX"], home_lanes[0]["startY"], home_lanes[0]["endX"], home_lanes[0]["endY"]),
            (7.4, 117.9, 344.4, 86.4),
        )
        self.assertEqual(
            (remote_lanes[0]["startX"], remote_lanes[0]["startY"], remote_lanes[0]["endX"], remote_lanes[0]["endY"]),
            (23.9, 399.8, 379.4, 363.0),
        )
        self.assertEqual(
            [task["areaNumber"] for task in tasks if task["mode"] == 1],
            [2, 2, 2, 2, 1, 1, 1, 1],
        )
        remote_clean = [task for task in tasks if task["mode"] == 1 and task["areaNumber"] == 2]
        home_clean = [task for task in tasks if task["mode"] == 1 and task["areaNumber"] == 1]
        self.assertEqual(
            (remote_clean[-1]["endX"], remote_clean[-1]["endY"]),
            (17, 261),
        )
        self.assertEqual(
            (home_clean[-1]["endX"], home_clean[-1]["endY"]),
            (0, 0),
        )
        self.assertFalse(any(task["source"] == "modeling_return_origin" for task in tasks))
        visited = {
            (task[prefix + "X"], task[prefix + "Y"])
            for task in tasks
            for prefix in ("start", "end")
        }
        expected_area_anchors = {
            (0, 0), (7, 118), (344, 86), (342, -62),
            (17, 261), (24, 400), (379, 363), (369, 235),
        }
        self.assertTrue(expected_area_anchors.issubset(visited))
        self.assertTrue({(10, 171), (11, 204)}.isdisjoint(visited))
        _assert_continuous_round_trip(self, tasks)
        for index in range(1, len(tasks)):
            self.assertFalse(_same_direction_collinear(tasks[index - 1], tasks[index]), index)

    def test_saved_task_frontend_points_and_execution_segments_are_identical(self):
        from modeling_frontend import frontend_path_points
        from modeling_preview import build_model_preview
        from modeling_task_generator import generate_task_plan
        from modeling_task_persistence import build_named_task

        points = [
            _point("p1", 0, 0),
            _point("p2", 0, 226),
            _point("p3", 700, 226),
            _point("p4", 700, 0),
        ]
        draft = {
            "id": "consistency-model",
            "recognition": {"confirmed": True},
            "groups": [{
                "id": "g1",
                "areaNumber": 1,
                "points": points,
                "subAreas": [{"id": "sa1", "pointIds": [point["id"] for point in points]}],
            }],
            "groupLinks": [],
        }
        draft["taskPreview"] = build_model_preview(draft, now=1000)
        plan = generate_task_plan(draft, now=2000)
        current_path = {"modelId": draft["id"], "taskPlan": plan}
        saved = build_named_task({}, current_path, "deep-test-route")
        path_points = frontend_path_points(plan)

        self.assertEqual(saved["taskList"], plan["tasks"])
        self.assertEqual(len(path_points), len(plan["tasks"]) + 1)
        for index, task in enumerate(plan["tasks"]):
            self.assertEqual((path_points[index]["x"], path_points[index]["y"]), (task["startX"], task["startY"]))
            self.assertEqual((path_points[index + 1]["x"], path_points[index + 1]["y"]), (task["endX"], task["endY"]))


if __name__ == "__main__":
    unittest.main()
