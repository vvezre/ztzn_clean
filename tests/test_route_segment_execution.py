import unittest


class RouteSegmentExecutionTest(unittest.TestCase):
    def _segment(self, mode=1):
        return {
            "id": 7,
            "mode": mode,
            "startLat": 10.0,
            "startLon": 20.0,
            "endLat": 32.1002,
            "endLon": 118.2003,
            # 故意放入完全错误的旧航向，执行策略不应读取它。
            "heading": 999.0,
            "angle": 999.0,
        }

    def test_clean_segment_turns_before_cleaning_and_stops_at_target(self):
        from route_segment_execution import run_route_segment

        events = []

        def read_position():
            events.append("read_live_rtk")
            return 32.0001, 118.0001

        def navigate(start_lat, start_lon, end_lat, end_lon, speed, before_drive):
            events.append(("navigate", start_lat, start_lon, end_lat, end_lon, speed))
            events.append("turn_complete")
            before_drive()
            events.append("drive")
            return 1

        def set_cleaning(enabled):
            events.append("clean_on" if enabled else "clean_off")

        result = run_route_segment(
            self._segment(mode=1),
            260,
            read_position,
            navigate,
            set_cleaning,
            lambda: events.append("brake"),
        )

        self.assertTrue(result)
        self.assertEqual(events[0], "read_live_rtk")
        self.assertEqual(events[1], "clean_off")
        self.assertEqual(events[2], ("navigate", 32.0001, 118.0001, 32.1002, 118.2003, 260))
        self.assertLess(events.index("turn_complete"), events.index("clean_on"))
        self.assertLess(events.index("clean_on"), events.index("drive"))
        self.assertEqual(events[-2:], ["brake", "clean_off"])
        self.assertNotIn(999.0, events[2])

    def test_transfer_segment_never_turns_cleaning_on(self):
        from route_segment_execution import run_route_segment

        events = []

        def navigate(start_lat, start_lon, end_lat, end_lon, speed, before_drive):
            events.append("turn_complete")
            before_drive()
            events.append("drive")
            return 1

        result = run_route_segment(
            self._segment(mode=2),
            200,
            lambda: (32.0, 118.0),
            navigate,
            lambda enabled: events.append("clean_on" if enabled else "clean_off"),
            lambda: events.append("brake"),
        )

        self.assertTrue(result)
        self.assertNotIn("clean_on", events)
        self.assertLess(events.index("turn_complete"), events.index("drive"))
        self.assertEqual(events[-2:], ["brake", "clean_off"])

    def test_soft_boundary_point_continues_without_braking_or_cleaning_toggle(self):
        from route_segment_execution import run_route_segment

        events = []
        segment = self._segment(mode=1)
        segment.update({
            "continuousPathId": "g1:lane-1:1",
            "turnAtStart": False,
            "stopAtEnd": False,
        })

        def navigate(start_lat, start_lon, end_lat, end_lon, speed, before_drive):
            events.append("navigate_continuous")
            before_drive()
            events.append("drive")
            return 1

        result = run_route_segment(
            segment,
            200,
            lambda: (32.0, 118.0),
            navigate,
            lambda enabled: events.append("clean_on" if enabled else "clean_off"),
            lambda: events.append("brake"),
        )

        self.assertTrue(result)
        self.assertEqual(events, ["navigate_continuous", "clean_on", "drive"])

    def test_missing_rtk_blocks_navigation_and_performs_safe_cleanup(self):
        from route_segment_execution import run_route_segment

        events = []
        errors = []
        result = run_route_segment(
            self._segment(mode=1),
            200,
            lambda: (None, None),
            lambda *args: events.append("navigate"),
            lambda enabled: events.append("clean_on" if enabled else "clean_off"),
            lambda: events.append("brake"),
            on_error=errors.append,
        )

        self.assertFalse(result)
        self.assertNotIn("navigate", events)
        self.assertNotIn("clean_on", events)
        self.assertEqual(events, ["brake", "clean_off"])
        self.assertEqual(len(errors), 1)

    def test_navigation_failure_and_exception_both_stop_and_clean_up(self):
        from route_segment_execution import run_route_segment

        for behavior in ("failure", "exception"):
            events = []
            errors = []

            def navigate(start_lat, start_lon, end_lat, end_lon, speed, before_drive):
                events.append("turn")
                if behavior == "exception":
                    raise RuntimeError("injected navigation error")
                return 0

            result = run_route_segment(
                self._segment(mode=1),
                200,
                lambda: (32.0, 118.0),
                navigate,
                lambda enabled: events.append("clean_on" if enabled else "clean_off"),
                lambda: events.append("brake"),
                on_error=errors.append,
            )

            self.assertFalse(result)
            self.assertNotIn("clean_on", events)
            self.assertEqual(events[-2:], ["brake", "clean_off"])
            self.assertEqual(len(errors), 1 if behavior == "exception" else 0)


if __name__ == "__main__":
    unittest.main()
