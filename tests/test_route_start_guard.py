import unittest


class RouteStartGuardTest(unittest.TestCase):
    def test_position_inside_twenty_centimeters_is_allowed(self):
        from route_start_guard import validate_route_start

        result = validate_route_start(
            32.0,
            118.0,
            32.000001,
            118.0,
            0.20,
            lambda *args: (0.11, 0.0),
        )

        self.assertTrue(result["isAtTaskOrigin"])
        self.assertEqual(result["distanceToTaskOriginM"], 0.11)
        self.assertEqual(result["taskOriginToleranceM"], 0.20)

    def test_position_outside_twenty_centimeters_is_rejected(self):
        from route_start_guard import RouteStartGuardError, validate_route_start

        with self.assertRaises(RouteStartGuardError) as raised:
            validate_route_start(
                32.0,
                118.0,
                32.00001,
                118.0,
                0.20,
                lambda *args: (0.21, 0.0),
            )

        self.assertEqual(raised.exception.code, "NOT_AT_TASK_ORIGIN")

    def test_missing_rtk_or_origin_is_rejected_before_distance_calculation(self):
        from route_start_guard import RouteStartGuardError, validate_route_start

        calls = []
        with self.assertRaises(RouteStartGuardError) as missing_rtk:
            validate_route_start(None, None, 32.0, 118.0, 0.20, lambda *args: calls.append(args))
        self.assertEqual(missing_rtk.exception.code, "RTK_NOT_READY")

        with self.assertRaises(RouteStartGuardError) as missing_origin:
            validate_route_start(32.0, 118.0, None, None, 0.20, lambda *args: calls.append(args))
        self.assertEqual(missing_origin.exception.code, "TASK_ORIGIN_UNKNOWN")
        self.assertEqual(calls, [])


if __name__ == "__main__":
    unittest.main()
