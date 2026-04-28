import unittest
import math

from layout_planner import (
    build_panel_segments,
    connected_panel_components,
    create_task_by_panel_layout,
    expand_panel_cells,
    panel_point_xy,
)


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
            [(90, 1, 10), (90, 2, 130), (90, 1, 10)],
        )
        self.assertEqual(tasks[-1]["endX"], 150)
        self.assertEqual(tasks[-1]["endY"], 0)

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

        self.assertEqual(tasks[0]["angle"], 0)
        self.assertEqual(tasks[0]["length"], 90)
        self.assertEqual(tasks[-1]["endX"], 0)
        self.assertEqual(tasks[-1]["endY"], 0)

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

        self.assertEqual(tasks[0]["angle"], 90)
        self.assertEqual(tasks[0]["length"], 50)

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
            [(task["angle"], task["mode"], task["length"], task["endX"], task["endY"]) for task in tasks[:2]],
            [
                (90, 2, 20, 20, 0),
                (90, 1, 10, 30, 0),
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

        self.assertEqual(tasks[0]["mode"], 1)
        self.assertEqual(tasks[0]["endX"], 10)
        self.assertEqual(tasks[0]["endY"], 0)
        self.assertEqual(tasks[-1]["action"], "return_origin")
        self.assertEqual(tasks[-1]["mode"], 2)
        self.assertEqual(tasks[-1]["endX"], 0)
        self.assertEqual(tasks[-1]["endY"], 0)


if __name__ == "__main__":
    unittest.main()
