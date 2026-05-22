import os
import re
import unittest


RUNTIME_ROOT = os.path.dirname(os.path.dirname(__file__))
MAIN_PATH = os.path.join(RUNTIME_ROOT, "main.py")


def _listener_rtk_body():
    with open(MAIN_PATH, "r", encoding="utf-8-sig") as handle:
        text = handle.read()
    match = re.search(r"def listenerRTK\(\):(.*?)(?:\ndef |\n# mqtt)", text, re.S)
    if not match:
        raise AssertionError("listenerRTK not found")
    return match.group(1)


class RTKObserverOrderTest(unittest.TestCase):
    def test_current_location_observer_runs_before_lower_machine_writer(self):
        body = _listener_rtk_body()
        update_index = body.find("register_observer(observer_go_correct)")
        lower_machine_index = body.find("register_observer(observer_rtk_data)")

        self.assertNotEqual(update_index, -1)
        self.assertNotEqual(lower_machine_index, -1)
        self.assertLess(update_index, lower_machine_index)


if __name__ == "__main__":
    unittest.main()
