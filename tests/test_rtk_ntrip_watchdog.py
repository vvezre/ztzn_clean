import io
import os
import time
import unittest

from ntrip_runtime import NtripConfig, NtripCorrectionRuntime


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
MAIN_PATH = os.path.join(ROOT, "main.py")
MQTT_INTEGRATION_PATH = os.path.join(ROOT, "mqtt_integration.py")


class FakeLogger(object):
    def warning(self, *args, **kwargs):
        pass

    def error(self, *args, **kwargs):
        pass


class FakeSocket(object):
    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


class FakeSerial(object):
    def write(self, data):
        self.last_write = data


def read_file(path):
    with io.open(path, "r", encoding="utf-8", errors="ignore") as handle:
        return handle.read()


class NtripRuntimeStatusTest(unittest.TestCase):
    def test_rtcm_timeout_closes_connection_and_records_reason(self):
        runtime = NtripCorrectionRuntime(
            NtripConfig(
                enabled=True,
                host="caster.example",
                port=2101,
                mountpoint="RTCM33",
                username="user",
                password="pass",
                rtcm_timeout_seconds=0.01,
                reconnect_interval_seconds=999,
            ),
            FakeLogger(),
        )
        fake_socket = FakeSocket()
        runtime._sock = fake_socket
        runtime._connected_at = time.time() - 1.0

        runtime.step(FakeSerial())

        self.assertTrue(fake_socket.closed)
        self.assertFalse(runtime.is_connected())
        status = runtime.get_status()
        self.assertEqual(status["ntripLastDisconnectReason"], "rtcm_timeout")
        self.assertIsNotNone(status["rtcmLastTimeoutAt"])

    def test_observe_fixed_gga_updates_quality_and_fixed_status(self):
        runtime = NtripCorrectionRuntime(
            NtripConfig(enabled=False),
            FakeLogger(),
        )

        runtime.observe_gga("$GNGGA,123519,4807.038,N,01131.000,E,4,08,0.9,545.4,M,46.9,M,,*47")

        status = runtime.get_status()
        self.assertEqual(status["rtkQuality"], "4")
        self.assertTrue(status["rtkHasFixed"])
        self.assertIsNotNone(status["rtkLastFixedAt"])


class RTKWatchdogSourceTest(unittest.TestCase):
    def test_watchdog_stops_on_lost_fix_and_times_out_after_five_minutes(self):
        source = read_file(MAIN_PATH)

        self.assertIn("RTK_FIX_RECOVERY_TIMEOUT_SECONDS = 300.0", source)
        self.assertIn("RTK_FIXED_GGA_MAX_AGE_SECONDS = 2.0", source)
        self.assertIn("RTK_FIX_TIMEOUT", source)
        self.assertIn("sendBraking()", source)
        self.assertIn("_mark_runtime_rtk_recovering", source)
        self.assertIn("_resume_current_rtk_segment", source)

    def test_linear_correction_is_skipped_while_recovering(self):
        source = read_file(MAIN_PATH)

        self.assertIn("if global_rtk_recovering:", source)
        self.assertIn("RTK fixed recovery active; skip linear correction command", source)

    def test_mqtt_reports_rtk_and_ntrip_fields(self):
        source = read_file(MQTT_INTEGRATION_PATH)

        self.assertIn("RTK_STATUS_FIELDS", source)
        self.assertIn("rtkFixState", source)
        self.assertIn("rtcmAgeSec", source)
        self.assertIn("ntripConnected", source)
        self.assertIn("get_shared_runtime", source)
        self.assertIn("_build_live_rtk_runtime_detail", source)
        self.assertIn("status.update(self._build_rtk_status_fields(status.get('detail')))", source)


if __name__ == "__main__":
    unittest.main()
