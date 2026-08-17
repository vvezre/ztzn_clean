import unittest


class ContinuousRouteTest(unittest.TestCase):
    def _segment(self, task_id, path_id="lane-a", turn=True, stop=True, mode=1):
        return {
            "id": task_id,
            "mode": mode,
            "continuousPathId": path_id,
            "turnAtStart": turn,
            "stopAtEnd": stop,
        }

    def test_soft_points_are_grouped_until_real_stop(self):
        from continuous_route import CONTINUATION_KEY, attach_continuations, collect_continuous_run

        segments = [
            self._segment(1, turn=True, stop=False),
            self._segment(2, turn=False, stop=False),
            self._segment(3, turn=False, stop=True),
            self._segment(4, path_id="lane-b", turn=True, stop=True),
        ]
        run = collect_continuous_run(segments, 0)
        runtime_segment = attach_continuations(run)

        self.assertEqual([item["id"] for item in run], [1, 2, 3])
        self.assertEqual([item["id"] for item in runtime_segment[CONTINUATION_KEY]], [2, 3])
        self.assertTrue(runtime_segment["stopAtEnd"])

    def test_missing_flags_and_mode_change_keep_safe_segment_boundaries(self):
        from continuous_route import collect_continuous_run

        missing_flags = [self._segment(1, stop=False), {"id": 2, "mode": 1, "continuousPathId": "lane-a"}]
        mode_change = [self._segment(1, stop=False), self._segment(2, turn=False, mode=2)]

        self.assertEqual(len(collect_continuous_run(missing_flags, 0)), 1)
        self.assertEqual(len(collect_continuous_run(mode_change, 0)), 1)


if __name__ == "__main__":
    unittest.main()
