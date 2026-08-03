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
