import io
import os
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def read_main():
    with io.open(os.path.join(ROOT, "main.py"), "r", encoding="utf-8", errors="ignore") as fh:
        return fh.read()


class DevConsoleMainRoutesTest(unittest.TestCase):
    def test_main_service_exposes_dev_console_json_routes(self):
        source = read_main()

        self.assertIn("from dev_console.correction_state import build_correction_state", source)
        self.assertIn("from dev_console.state_readers import", source)
        self.assertIn('@app.route("/dev/overview/state"', source)
        self.assertIn('@app.route("/dev/correction/state"', source)
        self.assertIn('@app.route("/dev/task-path"', source)
        self.assertIn('@app.route("/dev/redis/state"', source)
        self.assertIn('@app.route("/dev/logs"', source)
        self.assertIn("read_log_lines('app.log'", source)


if __name__ == "__main__":
    unittest.main()
