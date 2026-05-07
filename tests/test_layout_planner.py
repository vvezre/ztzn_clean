import unittest
import math

from layout_planner import (
    _clean_line_point,
    build_panel_segments,
    connected_panel_components,
    create_task_by_panel_layout,
    expand_panel_cells,
    panel_point_xy,
)
from service import _default_rtk_origin_from_tasks, _layout_rtk_origin, _prepare_tasks_for_rtk_origin


class LayoutPlannerTest(unittest.TestCase):
    def test_expands_multiple_regions_and_holes_win(self):
        layout = {
            "areas": [
                {"rowStart": 0, "rowEnd": 1, "colStart": 0, "colEnd": 3},
                {"rowStart": 0, "rowEnd": 1, "colStart": 6, "colEnd": 7},
            ],
            "extras": [
                {"rowStart": -1, "rowEnd": -1, "colStart": 0, "colEnd": 1},
            ],
            "holes": [
                {"rowStart": 0, "rowEnd": 0, "colStart": 1, "colEnd": 2},
            ],
        }

        cells = expand_panel_cells(layout)
        self.assertIn((-1, 0), cells)
        self.assertIn((0, 0), cells)
        self.assertIn((0, 3), cells)
        self.assertIn((0, 6), cells)
        self.assertNotIn((0, 1), cells)
        self.assertNotIn((0, 2), cells)

        segments = build_panel_segments(cells)
        self.assertEqual(segments[-1], [(0, 1)])
        self.assertEqual(segments[0], [(0, 0), (3, 3), (6, 7)])
        self.assertEqual(segments[1], [(0, 3), (6, 7)])

    def test_col_connector_adds_horizontal_travel_distance(self):
        layout = {
            "areas": [
                {"rowStart": 0, "rowEnd": 0, "colStart": 0, "colEnd": 1},
                {"rowStart": 0, "rowEnd": 0, "colStart": 4, "colEnd": 5},
            ],
            "connectors": [
                {"type": "col", "rowStart": 0, "rowEnd": 0, "afterCol": 1, "length": 100},
            ],
        }

        tasks = create_task_by_panel_layout(
            layout,
            panelWidth=10,
            panelHeight=20,
            gapX=0,
            gapY=0,
            startX=0,
            startY=0,
            direction="left",
        )

        self.assertEqual(
            [(task["angle"], task["mode"], task["length"]) for task in tasks],
            [
                (90, 2, 5),
                (0, 2, 15),
                (90, 1, 10),
                (180, 2, 10),
                (270, 1, 10),
                (0, 2, 10),
                (90, 2, 10),
                (90, 2, 130),
                (90, 1, 10),
                (180, 2, 10),
                (270, 1, 10),
            ],
        )
        self.assertEqual(tasks[-1]["endX"], 145)
        self.assertEqual(tasks[-1]["endY"], 5)

    def test_col_connector_gap_shifts_all_columns_after_bridge(self):
        layout = {
            "connectors": [
                {"type": "col", "rowStart": 0, "rowEnd": 0, "afterCol": 1, "length": 100},
            ],
        }

        self.assertEqual(panel_point_xy(2, 4, 10, 20, layout["connectors"]), (140, 40))

    def test_row_connector_adds_vertical_distance_for_matching_columns(self):
        layout = {
            "areas": [
                {"rowStart": 0, "rowEnd": 0, "colStart": 0, "colEnd": 0},
                {"rowStart": 2, "rowEnd": 2, "colStart": 0, "colEnd": 0},
            ],
            "connectors": [
                {"type": "row", "colStart": 0, "colEnd": 0, "afterRow": 0, "length": 50},
            ],
        }

        self.assertEqual(
            panel_point_xy(2, 0, 10, 20, layout["connectors"]),
            (0, 90),
        )

        tasks = create_task_by_panel_layout(
            layout,
            panelWidth=10,
            panelHeight=20,
            gapX=0,
            gapY=0,
            startX=0,
            startY=0,
            direction="left",
        )

        bridge_task = tasks[4]
        self.assertEqual(bridge_task["angle"], 0)
        self.assertEqual(bridge_task["length"], 80)
        self.assertEqual(bridge_task["startX"], 5)
        self.assertEqual(bridge_task["endY"], 95)
        self.assertEqual(tasks[-1]["endX"], 5)
        self.assertEqual(tasks[-1]["endY"], 95)

    def test_row_connector_gap_shifts_all_rows_after_bridge(self):
        layout = {
            "connectors": [
                {"type": "row", "colStart": 0, "colEnd": 0, "afterRow": 0, "length": 45},
            ],
        }

        self.assertEqual(panel_point_xy(1, 3, 100, 100, layout["connectors"]), (300, 145))

    def test_panel_angle_projection_is_applied_to_layout_spacing(self):
        layout = {
            "areas": [
                {"rowStart": 0, "rowEnd": 0, "colStart": 0, "colEnd": 1},
            ],
        }

        tasks = create_task_by_panel_layout(
            layout,
            panelWidth=100,
            panelHeight=20,
            gapX=0,
            gapY=0,
            angle_radians_x=math.radians(60),
        )

        clean_tasks = [task for task in tasks if task["mode"] == 1]
        self.assertEqual(clean_tasks[0]["angle"], 90)
        self.assertEqual(clean_tasks[0]["length"], 50)
        self.assertEqual(clean_tasks[0]["startY"], 15)

    def test_panel_angle_projection_rounding_matches_preview(self):
        layout = {
            "areas": [
                {"rowStart": 0, "rowEnd": 0, "colStart": 0, "colEnd": 1},
            ],
        }

        tasks = create_task_by_panel_layout(
            layout,
            panelWidth=101,
            panelHeight=20,
            gapX=0,
            gapY=0,
            angle_radians_x=math.radians(60),
        )

        clean_tasks = [task for task in tasks if task["mode"] == 1]
        self.assertEqual(clean_tasks[0]["length"], 51)

    def test_disconnected_regions_are_visited_by_nearest_entry(self):
        layout = {
            "areas": [
                {"rowStart": 10, "rowEnd": 10, "colStart": 0, "colEnd": 1},
                {"rowStart": 0, "rowEnd": 0, "colStart": 2, "colEnd": 3},
            ],
        }

        tasks = create_task_by_panel_layout(
            layout,
            panelWidth=10,
            panelHeight=10,
            gapX=0,
            gapY=0,
            startX=0,
            startY=0,
            direction="left",
        )

        self.assertEqual(
            [(task["angle"], task["mode"], task["length"], task["endX"], task["endY"]) for task in tasks[:3]],
            [
                (0, 2, 8, 0, 8),
                (90, 2, 25, 25, 8),
                (90, 1, 10, 35, 8),
            ],
        )

    def test_connectors_bridge_matching_rows_into_one_component(self):
        layout = {
            "areas": [
                {"rowStart": 0, "rowEnd": 1, "colStart": 0, "colEnd": 0},
                {"rowStart": 0, "rowEnd": 1, "colStart": 4, "colEnd": 4},
            ],
            "connectors": [
                {"type": "col", "rowStart": 0, "rowEnd": 1, "afterCol": 0, "length": 100},
            ],
        }

        without_connector = connected_panel_components(expand_panel_cells(layout), [])
        with_connector = connected_panel_components(expand_panel_cells(layout), layout["connectors"])

        self.assertEqual(len(without_connector), 2)
        self.assertEqual(len(with_connector), 1)

    def test_connectors_do_not_merge_separate_layout_areas_for_cleaning(self):
        layout = {
            "areas": [
                {"rowStart": 0, "rowEnd": 0, "colStart": 0, "colEnd": 3},
                {"rowStart": 1, "rowEnd": 1, "colStart": 0, "colEnd": 3},
            ],
            "connectors": [
                {"type": "row", "colStart": 0, "colEnd": 0, "afterRow": 0, "length": 45},
            ],
        }

        components = connected_panel_components(expand_panel_cells(layout), layout["connectors"], layout)

        self.assertEqual(len(components), 2)

    def test_adjacent_layout_areas_do_not_auto_connect(self):
        layout = {
            "areas": [
                {"rowStart": 0, "rowEnd": 0, "colStart": 0, "colEnd": 3},
                {"rowStart": 1, "rowEnd": 1, "colStart": 0, "colEnd": 3},
            ],
        }

        components = connected_panel_components(expand_panel_cells(layout), [], layout)

        self.assertEqual(len(components), 2)

    def test_row_connector_does_not_create_diagonal_cleaning_tasks(self):
        layout = {
            "sweepDirection": "top_to_bottom",
            "areas": [
                {"rowStart": 0, "rowEnd": 0, "colStart": 0, "colEnd": 3},
                {"rowStart": 1, "rowEnd": 1, "colStart": 0, "colEnd": 3},
            ],
            "connectors": [
                {"type": "row", "colStart": 0, "colEnd": 0, "afterRow": 0, "length": 45},
            ],
        }

        tasks = create_task_by_panel_layout(
            layout,
            panelWidth=100,
            panelHeight=100,
            gapX=0,
            gapY=0,
            direction="left",
        )

        for task in tasks:
            self.assertTrue(task["startX"] == task["endX"] or task["startY"] == task["endY"])

    def test_task_path_crosses_separate_area_boundary_only_at_connector(self):
        layout = {
            "sweepDirection": "bottom_to_top",
            "areas": [
                {"rowStart": 0, "rowEnd": 0, "colStart": 0, "colEnd": 3},
                {"rowStart": 1, "rowEnd": 1, "colStart": 0, "colEnd": 3},
            ],
            "connectors": [
                {"type": "row", "colStart": 0, "colEnd": 0, "afterRow": 0, "length": 45},
            ],
        }

        tasks = create_task_by_panel_layout(
            layout,
            panelWidth=100,
            panelHeight=100,
            gapX=0,
            gapY=0,
            direction="left",
        )
        boundary_y = 100
        connector_x = 50

        crossing_tasks = [
            task for task in tasks
            if task["startX"] == task["endX"] and
            min(task["startY"], task["endY"]) < boundary_y < max(task["startY"], task["endY"])
        ]

        self.assertTrue(crossing_tasks)
        self.assertTrue(all(task["startX"] == connector_x for task in crossing_tasks))

    def test_return_to_origin_adds_explicit_return_segment(self):
        layout = {
            "areas": [
                {"rowStart": 0, "rowEnd": 0, "colStart": 0, "colEnd": 1},
            ],
        }

        tasks = create_task_by_panel_layout(
            layout,
            panelWidth=10,
            panelHeight=20,
            gapX=0,
            gapY=0,
            returnToOrigin=True,
        )

        clean_tasks = [task for task in tasks if task["mode"] == 1]
        self.assertEqual(clean_tasks[0]["endX"], 15)
        self.assertEqual(clean_tasks[0]["endY"], 15)
        self.assertEqual(tasks[-1]["action"], "return_origin")
        self.assertEqual(tasks[-1]["mode"], 2)
        self.assertEqual(tasks[-1]["endX"], 0)
        self.assertEqual(tasks[-1]["endY"], 0)

    def test_bottom_to_top_sweep_uses_lower_then_upper_quarter_clean_lanes(self):
        layout = {
            "sweepDirection": "bottom_to_top",
            "areas": [
                {"rowStart": 0, "rowEnd": 1, "colStart": 0, "colEnd": 1},
            ],
        }

        tasks = create_task_by_panel_layout(
            layout,
            panelWidth=10,
            panelHeight=20,
            gapX=0,
            gapY=0,
            direction="left",
        )

        clean_tasks = [task for task in tasks if task["mode"] == 1]
        self.assertEqual([(task["startY"], task["endY"]) for task in clean_tasks], [
            (5, 5),
            (15, 15),
            (25, 25),
            (35, 35),
        ])
        self.assertEqual([(task["startX"], task["endX"]) for task in clean_tasks], [
            (5, 15),
            (15, 5),
            (5, 15),
            (15, 5),
        ])

    def test_top_to_bottom_sweep_uses_upper_then_lower_quarter_clean_lanes(self):
        layout = {
            "sweepDirection": "top_to_bottom",
            "areas": [
                {"rowStart": 0, "rowEnd": 1, "colStart": 0, "colEnd": 1},
            ],
        }

        tasks = create_task_by_panel_layout(
            layout,
            panelWidth=10,
            panelHeight=20,
            gapX=0,
            gapY=0,
            direction="left",
        )

        clean_tasks = [task for task in tasks if task["mode"] == 1]
        self.assertEqual([(task["startY"], task["endY"]) for task in clean_tasks], [
            (35, 35),
            (25, 25),
            (15, 15),
            (5, 5),
        ])
        self.assertEqual([(task["startX"], task["endX"]) for task in clean_tasks], [
            (5, 15),
            (15, 5),
            (5, 15),
            (15, 5),
        ])

    def test_quarter_clean_line_rounding_matches_preview_for_default_panel_height(self):
        layout = {
            "sweepDirection": "bottom_to_top",
            "areas": [
                {"rowStart": 0, "rowEnd": 0, "colStart": 0, "colEnd": 1},
            ],
        }

        tasks = create_task_by_panel_layout(
            layout,
            panelWidth=113,
            panelHeight=226,
            gapX=0,
            gapY=0,
            direction="left",
        )

        clean_tasks = [task for task in tasks if task["mode"] == 1]
        self.assertEqual(clean_tasks[0]["startY"], 57)
        self.assertEqual(clean_tasks[0]["endY"], 57)
        self.assertEqual(clean_tasks[1]["startY"], 170)
        self.assertEqual(clean_tasks[1]["endY"], 170)

    def test_clean_line_point_uses_panel_center_x_and_quarter_y(self):
        self.assertEqual(
            _clean_line_point(0, 0, 10, 20, 10, 20, [], "bottom_to_top"),
            (5, 5),
        )

    def test_panel_coordinate_origin_stays_at_lower_left(self):
        layout = {
            "sweepDirection": "bottom_to_top",
            "areas": [
                {"rowStart": 0, "rowEnd": 0, "colStart": 0, "colEnd": 1},
            ],
        }

        tasks = create_task_by_panel_layout(
            layout,
            panelWidth=10,
            panelHeight=20,
            gapX=0,
            gapY=0,
            direction="left",
        )

        self.assertEqual((tasks[0]["startX"], tasks[0]["startY"]), (0, 0))
        self.assertEqual((tasks[0]["endX"], tasks[0]["endY"]), (5, 0))
        clean_tasks = [task for task in tasks if task["mode"] == 1]
        self.assertEqual((clean_tasks[0]["startX"], clean_tasks[0]["startY"]), (5, 5))
        self.assertEqual((clean_tasks[0]["endX"], clean_tasks[0]["endY"]), (15, 5))

    def test_layout_start_can_be_independent_rtk_origin(self):
        layout = {
            "sweepDirection": "bottom_to_top",
            "areas": [
                {"rowStart": 0, "rowEnd": 0, "colStart": 0, "colEnd": 1},
            ],
        }

        tasks = create_task_by_panel_layout(
            layout,
            startX=-20,
            startY=-30,
            panelWidth=10,
            panelHeight=20,
            gapX=0,
            gapY=0,
            direction="left",
        )

        self.assertEqual(
            [(task["mode"], task["startX"], task["startY"], task["endX"], task["endY"]) for task in tasks[:3]],
            [
                (2, -20, -30, -20, 5),
                (2, -20, 5, 5, 5),
                (1, 5, 5, 15, 5),
            ],
        )
        clean_tasks = [task for task in tasks if task["mode"] == 1]
        self.assertEqual((clean_tasks[0]["startX"], clean_tasks[0]["startY"]), (5, 5))

    def test_return_to_origin_targets_robot_origin_clean_lane(self):
        layout = {
            "returnToOrigin": True,
            "sweepDirection": "bottom_to_top",
            "areas": [
                {"rowStart": 0, "rowEnd": 0, "colStart": 0, "colEnd": 1},
            ],
        }

        tasks = create_task_by_panel_layout(
            layout,
            panelWidth=10,
            panelHeight=20,
            gapX=0,
            gapY=0,
            direction="left",
            returnToOrigin=True,
        )

        clean_tasks = [task for task in tasks if task["mode"] == 1]
        self.assertEqual((clean_tasks[0]["startX"], clean_tasks[0]["startY"]), (5, 5))
        self.assertEqual(tasks[-1]["action"], "return_origin")
        self.assertEqual((tasks[-1]["endX"], tasks[-1]["endY"]), (0, 0))

    def test_service_task_coordinates_keep_configured_rtk_origin(self):
        tasks = [
            {"id": 1, "mode": 2, "startX": -20, "startY": -30, "endX": 5, "endY": -30, "areaNumber": 1},
            {"id": 2, "mode": 2, "startX": 5, "startY": -30, "endX": 5, "endY": 5, "areaNumber": 1},
            {"id": 3, "mode": 1, "startX": 5, "startY": 5, "endX": 15, "endY": 5, "areaNumber": 1},
            {"id": 4, "mode": 2, "startX": 15, "startY": 5, "endX": 15, "endY": 15, "areaNumber": 1},
            {"id": 5, "mode": 1, "startX": 15, "startY": 15, "endX": 5, "endY": 15, "areaNumber": 1},
        ]

        normalized, origin = _prepare_tasks_for_rtk_origin(
            tasks,
            return_to_origin=True,
            turn_back_len=10,
            area_number=1,
            origin=(-20, -30),
        )

        self.assertEqual(origin, (-20, -30))
        self.assertEqual(
            [(task["id"], task["mode"], task["startX"], task["startY"], task["endX"], task["endY"]) for task in normalized],
            [
                (1, 2, 0, 0, 25, 0),
                (2, 2, 25, 0, 25, 35),
                (3, 1, 25, 35, 35, 35),
                (4, 2, 35, 35, 35, 45),
                (5, 1, 35, 45, 25, 45),
                (6, 2, 25, 45, 25, 0),
                (7, 2, 25, 0, 0, 0),
            ],
        )
        self.assertEqual(normalized[-1]["action"], "return_origin")

    def test_default_rtk_origin_uses_lower_left_clean_anchor(self):
        tasks = create_task_by_panel_layout(
            {
                "sweepDirection": "bottom_to_top",
                "areas": [
                    {"rowStart": 0, "rowEnd": 0, "colStart": 0, "colEnd": 1},
                ],
            },
            startX=0,
            startY=0,
            panelWidth=10,
            panelHeight=20,
            gapX=0,
            gapY=0,
            direction="left",
        )

        self.assertEqual(_default_rtk_origin_from_tasks(tasks), (5, 5))

    def test_default_rtk_origin_does_not_change_with_sweep_direction(self):
        tasks = create_task_by_panel_layout(
            {
                "sweepDirection": "top_to_bottom",
                "areas": [
                    {"rowStart": 0, "rowEnd": 0, "colStart": 0, "colEnd": 1},
                ],
            },
            startX=0,
            startY=0,
            panelWidth=10,
            panelHeight=20,
            gapX=0,
            gapY=0,
            direction="left",
        )

        self.assertEqual(_default_rtk_origin_from_tasks(tasks), (5, 5))

    def test_selected_rtk_anchor_cell_maps_to_clean_start_point(self):
        layout = {
            "rtkAnchorRow": 1,
            "rtkAnchorCol": 1,
            "areas": [
                {"rowStart": 0, "rowEnd": 1, "colStart": 0, "colEnd": 1},
            ],
        }

        self.assertEqual(
            _layout_rtk_origin(
                layout,
                panel_width=10,
                panel_height=20,
                gap_x=0,
                gap_y=0,
                angle_radians_x=0,
                angle_radians_y=0,
            ),
            (15, 25),
        )

    def test_default_rtk_anchor_ignores_negative_row_extra_regions(self):
        layout = {
            "rtkOriginX": 0,
            "rtkOriginY": 0,
            "sweepDirection": "top_to_bottom",
            "areas": [
                {"rowStart": 0, "rowEnd": 1, "colStart": 0, "colEnd": 2},
            ],
            "extras": [
                {"rowStart": -1, "rowEnd": 0, "colStart": 1, "colEnd": 2},
            ],
        }

        self.assertEqual(
            _layout_rtk_origin(
                layout,
                panel_width=113,
                panel_height=226,
                gap_x=2,
                gap_y=2,
                angle_radians_x=0,
                angle_radians_y=math.radians(5),
            ),
            (57, 56),
        )

    def test_task_coordinates_start_under_first_clean_lane_without_horizontal_entry(self):
        panel_tasks = create_task_by_panel_layout(
            {
                "sweepDirection": "bottom_to_top",
                "areas": [
                    {"rowStart": 0, "rowEnd": 0, "colStart": 0, "colEnd": 1},
                ],
            },
            startX=5,
            startY=5,
            panelWidth=10,
            panelHeight=20,
            gapX=0,
            gapY=0,
            direction="left",
        )

        normalized, origin = _prepare_tasks_for_rtk_origin(
            panel_tasks,
            return_to_origin=True,
            turn_back_len=10,
            area_number=1,
            origin=(5, 5),
        )

        self.assertEqual(origin, (5, 5))
        self.assertEqual(
            [(task["mode"], task["startX"], task["startY"], task["endX"], task["endY"]) for task in normalized[:3]],
            [
                (1, 0, 0, 10, 0),
                (2, 10, 0, 10, 10),
                (1, 10, 10, 0, 10),
            ],
        )
        self.assertEqual((normalized[-1]["endX"], normalized[-1]["endY"]), (0, 0))

    def test_transition_to_negative_row_extension_stays_on_panel_side(self):
        layout = {
            "sweepDirection": "top_to_bottom",
            "areas": [
                {"rowStart": 0, "rowEnd": 3, "colStart": 0, "colEnd": 13},
            ],
            "extras": [
                {"rowStart": -1, "rowEnd": 0, "colStart": 11, "colEnd": 13},
            ],
        }

        tasks = create_task_by_panel_layout(
            layout,
            panelWidth=113,
            panelHeight=226,
            gapX=2,
            gapY=2,
            direction="left",
        )

        self.assertEqual(
            [(task["mode"], task["startX"], task["startY"], task["endX"], task["endY"]) for task in tasks[17:20]],
            [
                (2, 57, 57, 1322, 57),
                (2, 1322, 57, 1322, -58),
                (1, 1322, -58, 1552, -58),
            ],
        )

    def test_bottom_to_top_start_transition_to_negative_row_extension_stays_inside_layout(self):
        layout = {
            "sweepDirection": "bottom_to_top",
            "areas": [
                {"rowStart": 0, "rowEnd": 3, "colStart": 0, "colEnd": 13},
            ],
            "extras": [
                {"rowStart": -1, "rowEnd": 0, "colStart": 11, "colEnd": 13},
            ],
        }

        tasks = create_task_by_panel_layout(
            layout,
            panelWidth=113,
            panelHeight=226,
            gapX=2,
            gapY=2,
            direction="left",
        )

        self.assertEqual(
            [(task["mode"], task["startX"], task["startY"], task["endX"], task["endY"]) for task in tasks[:3]],
            [
                (2, 0, 0, 1552, 0),
                (2, 1552, 0, 1552, -171),
                (1, 1552, -171, 1322, -171),
            ],
        )

    def test_same_row_transition_detours_around_hole(self):
        layout = {
            "sweepDirection": "bottom_to_top",
            "areas": [
                {"rowStart": 0, "rowEnd": 1, "colStart": 0, "colEnd": 5},
            ],
            "holes": [
                {"rowStart": 0, "rowEnd": 0, "colStart": 2, "colEnd": 3},
            ],
        }

        tasks = create_task_by_panel_layout(
            layout,
            panelWidth=10,
            panelHeight=20,
            gapX=0,
            gapY=0,
            direction="left",
        )

        self.assertEqual(
            [(task["mode"], task["startX"], task["startY"], task["endX"], task["endY"]) for task in tasks[:5]],
            [
                (2, 0, 0, 5, 0),
                (2, 5, 0, 5, 5),
                (1, 5, 5, 15, 5),
                (2, 15, 5, 15, 25),
                (2, 15, 25, 45, 25),
            ],
        )


if __name__ == "__main__":
    unittest.main()
