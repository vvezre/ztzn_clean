import unittest


class ModelingTaskGeneratorTest(unittest.TestCase):
    def test_generate_task_plan_turns_preview_lanes_into_legacy_task_segments(self):
        from modeling_task_generator import generate_task_plan

        preview = {
            "status": "ready",
            "groups": [{
                "groupId": "g1",
                "areaNumber": 1,
                "subAreas": [{
                    "id": "sa1",
                    "lanes": [
                        {"id": "lane-1", "heading": 90, "startX": 0, "startY": 0, "endX": 100, "endY": 0, "lengthCm": 100},
                        {"id": "lane-2", "heading": 90, "startX": 0, "startY": 100, "endX": 100, "endY": 100, "lengthCm": 100},
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
                    {"id": "p2", "x": 100, "y": 0, "lat": 32.0, "lon": 118.0000106},
                    {"id": "p3", "x": 0, "y": 100, "lat": 32.000009, "lon": 118.0},
                ],
            }],
        }

        task_plan = generate_task_plan(draft, now=2000)

        self.assertEqual(task_plan["status"], "ready")
        self.assertEqual(task_plan["generatedAt"], 2000)
        self.assertEqual(task_plan["summary"]["cleanTaskCount"], 2)
        self.assertEqual(task_plan["summary"]["transferTaskCount"], 1)
        self.assertEqual(len(task_plan["tasks"]), 3)
        first = task_plan["tasks"][0]
        self.assertEqual(first["id"], 1)
        self.assertEqual(first["mode"], 1)
        self.assertEqual(first["areaNumber"], 1)
        self.assertEqual(first["startX"], 0)
        self.assertEqual(first["endX"], 100)
        self.assertEqual(first["heading"], 90.0)
        self.assertEqual(first["length"], 100)
        self.assertIn("startLat", first)
        self.assertIn("endLon", first)
        transfer = task_plan["tasks"][1]
        self.assertEqual(transfer["mode"], 2)
        self.assertEqual((transfer["startX"], transfer["startY"]), (100, 0))
        self.assertEqual((transfer["endX"], transfer["endY"]), (100, 100))
        second_clean = task_plan["tasks"][2]
        self.assertEqual(second_clean["mode"], 1)
        self.assertEqual((second_clean["startX"], second_clean["startY"]), (100, 100))
        self.assertEqual((second_clean["endX"], second_clean["endY"]), (0, 100))
        self.assertEqual(second_clean["heading"], 270.0)

    def test_generate_task_plan_requires_ready_preview(self):
        from modeling_task_generator import ModelingTaskGenerationError, generate_task_plan

        with self.assertRaises(ModelingTaskGenerationError):
            generate_task_plan({"id": "m1", "taskPreview": {"status": "empty"}})


if __name__ == "__main__":
    unittest.main()
