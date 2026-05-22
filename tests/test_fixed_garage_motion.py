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


class FixedGarageMotionTest(unittest.TestCase):
    def test_manual_enter_uses_fixed_distance(self):
        body = function_body(read_main(), "enter_garage_task")

        self.assertIn("_run_fixed_enter_garage", body)
        self.assertNotIn("_run_visual_enter_garage", body)

    def test_manual_exit_uses_task_distance_only(self):
        body = function_body(read_main(), "exit_garage_task")

        self.assertIn("_run_exit_garage_by_back_length", body)
        self.assertNotIn("exit_uav", body)
        self.assertNotIn("moveDiatance", body)
        self.assertNotIn("turn(ser", body)

    def test_auto_enter_uses_fixed_distance(self):
        body = function_body(read_main(), "intoGarage")

        self.assertIn("_run_fixed_enter_garage(backLength)", body)
        self.assertNotIn("_run_visual_enter_garage", body)

    def test_fixed_enter_uses_distance_mode(self):
        body = function_body(read_main(), "_run_fixed_enter_garage")

        self.assertIn("setStatus(2)", body)
        self.assertNotIn("setStatus(1)", body)

    def test_auto_exit_uses_fixed_distance_helper(self):
        body = function_body(read_main(), "goOutGarage")

        self.assertIn("_run_exit_garage_by_back_length", body)


if __name__ == "__main__":
    unittest.main()
