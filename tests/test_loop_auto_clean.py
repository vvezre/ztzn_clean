import io
import os
import re
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAIN_PATH = os.path.join(ROOT, "main.py")


def read_main():
    with io.open(MAIN_PATH, "r", encoding="utf-8") as handle:
        return handle.read()


def function_body(source, name):
    match = re.search(r"^def %s\(" % re.escape(name), source, re.M)
    if not match:
        raise AssertionError("function %s not found" % name)
    signature_end = source.find(":\n", match.end())
    if signature_end < 0:
        raise AssertionError("function %s signature end not found" % name)
    start = signature_end + 2
    next_match = re.search(r"^def [A-Za-z_][A-Za-z0-9_]*\(.*?\):\n", source[start:], re.M)
    end = start + next_match.start() if next_match else len(source)
    return source[start:end]


class LoopAutoCleanTest(unittest.TestCase):
    def test_loop_routes_are_exposed(self):
        source = read_main()

        self.assertIn('/vehicle/startLoopAutoDrive', source)
        self.assertIn('/vehicle/stopLoopAutoDrive', source)
        self.assertIn('/vehicle/getLoopAutoDriveStatus', source)

    def test_loop_thread_repeats_auto_drive_until_low_battery(self):
        body = function_body(read_main(), "loopAutoDriveThread")

        self.assertIn("while _is_loop_auto_clean_enabled()", body)
        self.assertIn("isNeedReturnCharging()", body)
        self.assertIn("autoDriveByRTKThread()", body)
        self.assertIn("_disable_loop_auto_clean('low_battery_return')", body)

    def test_manual_parking_disables_loop_mode(self):
        body = function_body(read_main(), "parking")

        self.assertIn("_disable_loop_auto_clean('manual_parking')", body)

    def test_vehicle_status_contains_loop_state(self):
        body = function_body(read_main(), "_build_vehicle_status_payload")

        self.assertIn("'loop_auto_clean'", body)
        self.assertIn("_get_loop_auto_clean_status()", body)

    def test_low_battery_listener_stops_loop_and_current_clean_thread(self):
        body = function_body(read_main(), "listenerVoltage")

        self.assertIn("_disable_loop_auto_clean('low_battery_return')", body)
        self.assertIn("global_doCleanThreadStop = 1", body)


if __name__ == "__main__":
    unittest.main()
