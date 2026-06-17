import calendar
import unittest

import util


class RTKLatestNmeaTest(unittest.TestCase):
    def _utc_time(self, hour, minute, second):
        return calendar.timegm((2026, 5, 22, hour, minute, second, 0, 0, 0))

    def test_backlog_batch_returns_only_latest_current_sample(self):
        state = util._RtkLatestState(sync_threshold=0.5)
        now_time = self._utc_time(6, 20, 34) + 0.5
        lines = [
            "$GNHPR,062031.20,151.00,-09.53,000.00,4,39,0.00,0999*4F",
            "$GNGGA,062031.25,3202.19000000,N,11855.47000000,E,4,41,0.4,49.2734,M,2.7088,M,2.2,1058*59",
            "$GNHPR,062034.30,154.24,-08.58,000.00,4,39,0.00,0999*41",
            "$GNGGA,062034.35,3202.19056508,N,11855.47168638,E,4,41,0.4,49.2747,M,2.7088,M,2.3,1058*57",
        ]

        sample, stats = util._consume_rtk_nmea_lines(state, lines, now_time=now_time, port="/dev/ttyUSB0")

        self.assertEqual(stats["stale"], 2)
        self.assertIsNotNone(sample)
        self.assertAlmostEqual(sample[0], 32.03650942, places=8)
        self.assertAlmostEqual(sample[1], 118.92452811, places=8)
        self.assertAlmostEqual(sample[2], 154.24, places=2)

    def test_stale_lines_do_not_clear_last_heading(self):
        state = util._RtkLatestState(sync_threshold=0.5)
        state.last_heading = 155.5
        now_time = self._utc_time(6, 20, 34) + 0.5
        lines = [
            "$GNHPR,062031.20,151.00,-09.53,000.00,4,39,0.00,0999*4F",
            "$GNGGA,062034.35,3202.19056508,N,11855.47168638,E,4,41,0.4,49.2747,M,2.7088,M,2.3,1058*57",
        ]

        sample, stats = util._consume_rtk_nmea_lines(state, lines, now_time=now_time, port="/dev/ttyUSB0")

        self.assertEqual(stats["stale"], 1)
        self.assertIsNotNone(sample)
        self.assertAlmostEqual(sample[2], 155.5, places=2)


if __name__ == "__main__":
    unittest.main()
