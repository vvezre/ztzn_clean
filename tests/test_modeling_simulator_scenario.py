import tempfile
import unittest


class ModelingSimulatorScenarioTest(unittest.TestCase):
    def test_fixed_points_match_the_recent_tilted_two_area_model(self):
        from modeling_simulator import SCENARIO_POINTS

        self.assertEqual(
            [point["pointType"] for point in SCENARIO_POINTS],
            ["area"] * 4 + ["link"] * 2 + ["area"] * 4,
        )
        self.assertEqual(
            [(point["x"], point["y"]) for point in SCENARIO_POINTS],
            [
                (0.000, 0.000),
                (7.362, 117.944),
                (344.374, 86.365),
                (342.394, -61.646),
                (9.502, 170.529),
                (10.576, 204.143),
                (16.882, 260.830),
                (23.914, 399.779),
                (379.429, 363.040),
                (368.523, 235.322),
            ],
        )

    def test_preloaded_realistic_scenario_generates_four_lanes_per_area(self):
        from modeling_simulator import ModelingSimulatorController

        with tempfile.TemporaryDirectory() as directory:
            controller = ModelingSimulatorController(directory, now=lambda: 1000)
            result = controller.preload_scenario()

            self.assertTrue(result["success"])
            self.assertEqual(result["data"]["areaPointCount"], 8)
            self.assertEqual(result["data"]["linkPointCount"], 2)

            state = controller.session.current()
            draft = controller.store.get_draft(state["modelId"])
            preview = draft["taskPreview"]
            plan = draft["taskPlan"]
            lanes = [
                len(sub_area.get("lanes") or [])
                for group in preview.get("groups") or []
                for sub_area in group.get("subAreas") or []
            ]

            self.assertEqual(lanes, [4, 4])
            self.assertEqual(plan["summary"]["cleanTaskCount"], 8)
            self.assertEqual(
                [task["areaNumber"] for task in plan["tasks"] if task["mode"] == 1],
                [2, 2, 2, 2, 1, 1, 1, 1],
            )
            self.assertEqual(
                (plan["tasks"][0]["startX"], plan["tasks"][0]["startY"]),
                (0, 0),
            )
            self.assertEqual(
                (plan["tasks"][-1]["endX"], plan["tasks"][-1]["endY"]),
                (0, 0),
            )

    def test_clear_all_returns_fixed_scenario_to_first_area_and_first_point(self):
        from modeling_simulator import ModelingSimulatorController

        with tempfile.TemporaryDirectory() as directory:
            controller = ModelingSimulatorController(directory, now=lambda: 1000)
            controller.start_modeling(restart=True)
            for _ in range(4):
                self.assertTrue(controller.sample_modeling_point()["success"])
            for _ in range(2):
                self.assertTrue(controller.sample_modeling_link_point()["success"])
            self.assertTrue(controller.new_modeling_area()["success"])
            for _ in range(4):
                self.assertTrue(controller.sample_modeling_point()["success"])

            cleared_area = controller.clear_all_modeling_points("area")
            cleared_link = controller.clear_all_modeling_points("link")
            state = controller.get_modeling_state()["data"]

            self.assertFalse(cleared_area["data"]["resetToFirstArea"])
            self.assertTrue(cleared_link["data"]["resetToFirstArea"])
            self.assertEqual(state["currentAreaNumber"], 1)
            self.assertEqual(state["groupCount"], 1)
            self.assertEqual(state["linkCount"], 0)

            recorded_again = controller.sample_modeling_point()
            self.assertTrue(recorded_again["success"])
            self.assertEqual(recorded_again["data"]["point"]["id"], "sim_p01")
            self.assertEqual(recorded_again["data"]["point"]["areaNumber"], 1)
            self.assertEqual(recorded_again["data"]["simulator"]["scenarioIndex"], 1)


if __name__ == "__main__":
    unittest.main()
