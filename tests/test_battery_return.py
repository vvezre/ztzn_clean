import unittest
import os
import io

from battery_return import should_return_to_charge


ROOT = os.path.dirname(os.path.dirname(__file__))


def _read(name):
    with io.open(os.path.join(ROOT, name), "r", encoding="utf-8-sig") as handle:
        return handle.read()


class BatteryReturnTest(unittest.TestCase):
    def test_voltage_below_10_triggers_return(self):
        self.assertTrue(should_return_to_charge("9"))
        self.assertTrue(should_return_to_charge(0))

    def test_voltage_10_or_above_does_not_trigger_return(self):
        self.assertFalse(should_return_to_charge("10"))
        self.assertFalse(should_return_to_charge(90))

    def test_missing_voltage_does_not_trigger_return(self):
        self.assertFalse(should_return_to_charge(None))
        self.assertFalse(should_return_to_charge(""))

    def test_voltage_listener_thread_is_started(self):
        main_text = _read("main.py")

        self.assertIn("listenerVoltageThread.start()", main_text)
        self.assertNotIn("# listenerVoltageThread.start()", main_text)

    def test_voltage_listener_is_enabled_unless_explicitly_disabled(self):
        main_text = _read("main.py")

        self.assertIn("redis_cli.get('voltageListener') != '0'", main_text)
        self.assertNotIn("redis_cli.get('voltageListener') == '1'", main_text)


if __name__ == "__main__":
    unittest.main()
