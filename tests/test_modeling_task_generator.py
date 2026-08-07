import unittest


class ModelingTaskGeneratorTest(unittest.TestCase):
    def test_round_int_uses_same_half_away_from_zero_rule_on_python2_and_python3(self):
        from modeling_task_generator import _round_int

        self.assertEqual(_round_int(320.5), 321)
        self.assertEqual(_round_int(320.49), 320)
        self.assertEqual(_round_int(-61.5), -62)
        self.assertEqual(_round_int(-61.49), -61)

    def test_area_transition_follows_manual_recording_order_instead_of_shorter_reverse_side(self):
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

        self.assertEqual(path, [(0.0, 0.0), (0.0, 120.0), (340.0, 90.0)])

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

        self.assertEqual(plan["routeType"], "bridge_round_trip")
        self.assertEqual(tasks[0]["mode"], 2)
        self.assertEqual(
            (tasks[0]["startX"], tasks[0]["startY"], tasks[0]["endX"], tasks[0]["endY"]),
            (0, 0, 40, 416),
        )

        remote_clean_indexes = [
            index for index, task in enumerate(tasks)
            if task["mode"] == 1 and task["areaNumber"] == 2
        ]
        bridge_return = tasks[remote_clean_indexes[-1] + 1]
        self.assertEqual(bridge_return["mode"], 2)
        self.assertEqual(
            (
                bridge_return["startX"],
                bridge_return["startY"],
                bridge_return["endX"],
                bridge_return["endY"],
            ),
            (36, 284, 11, 127),
        )
        self.assertEqual(tasks[remote_clean_indexes[-1] + 2]["mode"], 1)

        connector_coordinates = {(31, 178), (34, 231)}
        executable_endpoints = {
            (task["startX"], task["startY"]) for task in tasks
        } | {
            (task["endX"], task["endY"]) for task in tasks
        }
        self.assertTrue(connector_coordinates.isdisjoint(executable_endpoints))

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
