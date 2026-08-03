import unittest


class ModelingTaskGeneratorTest(unittest.TestCase):
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

    def test_two_connected_groups_clean_remote_first_then_home_and_return_to_origin(self):
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

        self.assertEqual(plan["routeType"], "bridge_round_trip")
        self.assertEqual(clean_areas, [2, 2, 1, 1])
        self.assertEqual((tasks[0]["startX"], tasks[0]["startY"]), (0, 0))
        self.assertEqual((tasks[-1]["endX"], tasks[-1]["endY"]), (0, 0))
        # 出程上的同一直线普通点被合并，不会在连接桥端点额外停车。
        self.assertEqual((tasks[0]["endX"], tasks[0]["endY"]), (0, 300))
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

    def test_generate_task_plan_requires_ready_preview(self):
        from modeling_task_generator import ModelingTaskGenerationError, generate_task_plan

        with self.assertRaises(ModelingTaskGenerationError):
            generate_task_plan({"id": "m1", "taskPreview": {"status": "empty"}})


if __name__ == "__main__":
    unittest.main()
