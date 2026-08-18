# coding=utf-8
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


    def _polyline_points(self, coordinates):
        from continuous_route import _local_m_to_latlon

        origin_lat = 32.0
        origin_lon = 118.0
        result = []
        for index, coordinate in enumerate(coordinates):
            lat, lon = _local_m_to_latlon(origin_lat, origin_lon, coordinate[0], coordinate[1])
            result.append({"lat": lat, "lon": lon, "taskId": index})
        return result

    def test_short_first_segment_uses_stable_polyline_lookahead_heading(self):
        from continuous_route import compute_polyline_guidance, _local_m_to_latlon

        points = self._polyline_points([
            (0.0, 0.0),
            (0.02, 0.19),
            (0.11, 1.17),
            (0.13, 1.44),
            (0.19, 2.22),
            (0.23, 2.81),
        ])
        current_lat, current_lon = _local_m_to_latlon(32.0, 118.0, -0.20, 0.0)
        guidance = compute_polyline_guidance(points, current_lat, current_lon)

        # 路径近似向北，但该车RTK航向坐标比地理方位角顺时针偏移90度，
        # 因此运行时目标航向应接近90度，而不是接近0度。
        self.assertAlmostEqual(guidance["heading"], 90.0, delta=15.0)
        self.assertGreater(guidance["lookaheadM"], 0.50)
        self.assertFalse(guidance["complete"])

    def test_lookahead_cannot_cut_a_corner_outside_corridor(self):
        from continuous_route import compute_polyline_guidance

        points = self._polyline_points([(0.0, 0.0), (0.0, 1.0), (1.0, 1.0)])
        guidance = compute_polyline_guidance(
            points,
            points[0]["lat"],
            points[0]["lon"],
            lookahead_m=2.0,
            corridor_m=0.10,
        )

        self.assertLessEqual(guidance["chordDeviationM"], 0.100001)
        self.assertLess(guidance["lookaheadM"], 1.50)

    def test_final_point_requires_real_distance_and_corridor(self):
        from continuous_route import compute_polyline_guidance, _local_m_to_latlon

        points = self._polyline_points([(0.0, 0.0), (0.0, 1.0)])
        final_guidance = compute_polyline_guidance(
            points,
            points[-1]["lat"],
            points[-1]["lon"],
            previous_progress_m=0.95,
        )
        far_lat, far_lon = _local_m_to_latlon(32.0, 118.0, 0.30, 2.40)
        far_guidance = compute_polyline_guidance(
            points,
            far_lat,
            far_lon,
            previous_progress_m=0.95,
        )

        self.assertTrue(final_guidance["complete"])
        self.assertFalse(far_guidance["complete"])
        self.assertGreater(far_guidance["distanceToFinalM"], 1.0)

    def test_small_overshoot_is_completed_but_large_overshoot_fails_immediately(self):
        from continuous_route import compute_polyline_guidance, _local_m_to_latlon

        points = self._polyline_points([(0.0, 0.0), (0.0, 1.0)])
        near_lat, near_lon = _local_m_to_latlon(32.0, 118.0, 0.05, 1.12)
        far_lat, far_lon = _local_m_to_latlon(32.0, 118.0, 0.20, 1.12)

        near = compute_polyline_guidance(
            points,
            near_lat,
            near_lon,
            previous_progress_m=0.95,
        )
        far = compute_polyline_guidance(
            points,
            far_lat,
            far_lon,
            previous_progress_m=0.95,
        )

        self.assertTrue(near["passedFinal"])
        self.assertTrue(near["complete"])
        self.assertFalse(near["terminalMissed"])
        self.assertTrue(far["passedFinal"])
        self.assertFalse(far["complete"])
        self.assertTrue(far["terminalMissed"])
        self.assertAlmostEqual(far["overshootM"], 0.12, delta=0.01)


    def test_boundary_test_1_log_heading_uses_polyline_instead_of_short_target(self):
        import math
        from continuous_route import compute_polyline_guidance, _local_m_to_latlon

        # “边界点测试1”中2号停车点后的真实连续段，单位为米。
        coordinates = [
            (0.0, 0.0),
            (0.02, 0.19),
            (0.11, 1.17),
            (0.13, 1.44),
            (0.19, 2.22),
            (0.23, 2.81),
        ]
        points = self._polyline_points(coordinates)
        # 日志显示旧程序从实际位置直瞄23厘米外的L1，算出了70.659度。
        distance_m = 0.234
        old_heading_rad = math.radians(70.659)
        actual_x = coordinates[1][0] - distance_m * math.sin(old_heading_rad)
        actual_y = coordinates[1][1] - distance_m * math.cos(old_heading_rad)
        current_lat, current_lon = _local_m_to_latlon(32.0, 118.0, actual_x, actual_y)

        guidance = compute_polyline_guidance(points, current_lat, current_lon)

        # 真实地理方位约5.34度；转换到小车现有航向坐标后必须为95.34度。
        self.assertAlmostEqual(guidance["heading"], 95.34, delta=1.0)
        self.assertAlmostEqual(guidance["lookaheadM"], 0.80, delta=0.01)
        self.assertGreater(guidance["crossTrackM"], 0.20)
        self.assertFalse(guidance["complete"])

    def test_polyline_heading_matches_existing_point_to_point_vehicle_convention(self):
        from continuous_route import compute_polyline_guidance

        north_points = self._polyline_points([(0.0, 0.0), (0.0, 2.0)])
        east_points = self._polyline_points([(0.0, 0.0), (2.0, 0.0)])

        north = compute_polyline_guidance(
            north_points,
            north_points[0]["lat"],
            north_points[0]["lon"],
        )
        east = compute_polyline_guidance(
            east_points,
            east_points[0]["lat"],
            east_points[0]["lon"],
        )

        # 与util.get_distance_angle()现有约定一致：地理北向对应小车90度，
        # 地理东向对应小车180度，防止连续折线再次漏掉90度安装偏置。
        self.assertAlmostEqual(north["heading"], 90.0, delta=0.1)
        self.assertAlmostEqual(east["heading"], 180.0, delta=0.1)


if __name__ == "__main__":
    unittest.main()
