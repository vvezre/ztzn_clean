import calendar
import unittest

import util


class RTKNmeaTimeTest(unittest.TestCase):
    def _utc_time(self, hour, minute, second):
        return calendar.timegm((2026, 5, 15, hour, minute, second, 0, 0, 0))

    def test_parse_nmea_utc_seconds(self):
        self.assertAlmostEqual(
            util._parse_nmea_utc_seconds("062423.80"),
            6 * 3600 + 24 * 60 + 23.8,
            places=2,
        )

    def test_nmea_age_uses_utc_seconds_of_day(self):
        age = util._nmea_utc_age_seconds("062423.80", self._utc_time(6, 25, 35))
        self.assertAlmostEqual(age, 71.2, places=1)

    def test_nmea_age_wraps_midnight(self):
        age = util._nmea_utc_age_seconds("235959.50", self._utc_time(0, 0, 1))
        self.assertAlmostEqual(age, 1.5, places=1)

    def test_future_time_is_not_stale(self):
        now_time = self._utc_time(6, 20, 0)
        self.assertFalse(util._is_stale_nmea_utc("062030.00", now_time, 2.0))

    def test_old_time_is_stale(self):
        now_time = self._utc_time(6, 25, 35)
        self.assertTrue(util._is_stale_nmea_utc("062423.80", now_time, 2.0))


if __name__ == "__main__":
    unittest.main()
