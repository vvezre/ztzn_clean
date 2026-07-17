import unittest

import util
from rtk_path_tracking import (
    RTKKalmanFilter2D,
    StraightLinePController,
    build_tracking_command,
)


class RTKPathTrackingTest(unittest.TestCase):
    def test_kalman_filter_smooths_position_jitter(self):
        filt = RTKKalmanFilter2D(process_noise=0.05, measurement_noise=4.0)
        samples = [
            (32.00000000, 118.00000000, 0.0),
            (32.00000018, 118.00000000, 0.1),
            (31.99999982, 118.00000000, 0.2),
        ]

        outputs = [filt.update(lat, lon, timestamp=ts) for lat, lon, ts in samples]

        raw_span = max(lat for lat, _, _ in samples) - min(lat for lat, _, _ in samples)
        filtered_span = max(point.lat for point in outputs) - min(point.lat for point in outputs)
        self.assertLess(filtered_span, raw_span * 0.75)
        self.assertTrue(outputs[-1].filtered)

    def test_straight_line_p_controller_uses_legacy_cross_track_sign(self):
        controller = StraightLinePController(heading_gain=10.0, cte_gain=1000.0)

        command = controller.compute(
            start_lat=32.0,
            start_lon=118.0,
            end_lat=32.0,
            end_lon=118.0001,
            current_lat=32.00001,
            current_lon=118.00002,
            vehicle_heading=90.0,
        )

        expected_cte = util.cross_track_error(
            32.0,
            118.0,
            32.0,
            118.0001,
            32.00001,
            118.00002,
        )
        self.assertAlmostEqual(command.cte_m, expected_cte, places=3)
        self.assertLess(command.cte_m, 0.0)
        expected = int(round(command.heading_error_deg * 10.0 - command.cte_m * 1000.0))
        self.assertEqual(command.z_speed, expected)
        self.assertGreater(command.z_speed, 0)
        self.assertEqual(command.source, "straight_line_p_control")

    def test_build_tracking_command_filters_raw_rtk_before_p_control(self):
        filt = RTKKalmanFilter2D(process_noise=0.05, measurement_noise=4.0)
        controller = StraightLinePController(heading_gain=10.0, cte_gain=1000.0)

        build_tracking_command(
            filt,
            controller,
            start_lat=32.0,
            start_lon=118.0,
            end_lat=32.0,
            end_lon=118.0001,
            current_lat=32.0,
            current_lon=118.00002,
            vehicle_heading=90.0,
            timestamp=0.0,
        )
        command = build_tracking_command(
            filt,
            controller,
            start_lat=32.0,
            start_lon=118.0,
            end_lat=32.0,
            end_lon=118.0001,
            current_lat=32.00002,
            current_lon=118.00002,
            vehicle_heading=90.0,
            timestamp=0.1,
        )

        self.assertNotEqual(command.raw_lat, command.filtered_lat)
        self.assertEqual(command.source, "kalman_p_control")


if __name__ == "__main__":
    unittest.main()
