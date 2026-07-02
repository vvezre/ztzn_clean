import io
import os
import re
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAIN_PATH = os.path.join(ROOT, "main.py")
MQTT_PATH = os.path.join(ROOT, "mqtt_integration.py")


def read_file(path):
    with io.open(path, "r", encoding="utf-8") as handle:
        return handle.read()


def function_body(source, name):
    match = re.search(r"^(?:    )?def %s\(" % re.escape(name), source, re.M)
    if not match:
        raise AssertionError("function %s not found" % name)
    signature_end = source.find(":\n", match.end())
    if signature_end < 0:
        raise AssertionError("function %s signature end not found" % name)
    start = signature_end + 2
    indent = re.match(r"^(\s*)def", source[match.start():match.end()]).group(1)
    next_pattern = r"^%sdef [A-Za-z_][A-Za-z0-9_]*\(.*?\):\n" % re.escape(indent)
    next_match = re.search(next_pattern, source[start:], re.M)
    end = start + next_match.start() if next_match else len(source)
    return source[start:end]


class IgnorePowerEnableStatusTest(unittest.TestCase):
    def test_main_status_derivation_ignores_power_on_state(self):
        source = read_file(MAIN_PATH)

        for name in ("_derive_control_state", "_derive_fault_state", "_derive_mission_state", "_derive_status"):
            body = function_body(source, name)
            self.assertNotIn("_get_power_on_state()", body, name)
            self.assertNotIn("LOWER_MACHINE_DISABLED", body, name)
            self.assertNotIn("LOWER_MACHINE_STATUS_UNKNOWN", body, name)

        self.assertNotIn("return 'disabled'", function_body(source, "_derive_status"))

    def test_auto_drive_validation_has_no_enable_warning_or_bypass(self):
        source = read_file(MAIN_PATH)

        for name in ("_validate_auto_drive_request_legacy", "_validate_auto_drive_request"):
            body = function_body(source, name)
            self.assertNotIn("_get_power_on_state()", body, name)
            self.assertNotIn("lowerMachineStatusWarning", body, name)
            self.assertNotIn("lowerMachineStartBypass", body, name)

    def test_runtime_detail_does_not_publish_enable_display_fields(self):
        body = function_body(read_file(MAIN_PATH), "_build_runtime_detail")

        self.assertNotIn("'powerOnState'", body)
        self.assertNotIn("'powerOnEnabled'", body)

    def test_mqtt_status_builder_ignores_power_on_state(self):
        source = read_file(MQTT_PATH)

        for name in ("_build_status", "_build_mission_state", "_build_control_state", "_build_fault_state"):
            body = function_body(source, name)
            self.assertNotIn("powerOnState", body, name)
            self.assertNotIn("LOWER_MACHINE_DISABLED", body, name)
            self.assertNotIn("LOWER_MACHINE_STATUS_UNKNOWN", body, name)

        self.assertNotIn("return 'disabled'", function_body(source, "_build_status"))
        self.assertNotIn("'powerOnState'", function_body(source, "_build_detail"))
        self.assertNotIn("'powerOnEnabled'", function_body(source, "_build_detail"))

    def test_enable_fault_warning_does_not_leak_into_health_state(self):
        main_health_body = function_body(read_file(MAIN_PATH), "_derive_health_state")
        mqtt_health_body = function_body(read_file(MQTT_PATH), "_build_health_state")

        self.assertIn("_is_ignored_enable_fault_state", main_health_body)
        self.assertIn("_is_ignored_enable_fault_state", mqtt_health_body)
        self.assertIn("healthState", main_health_body)
        self.assertIn("healthState", mqtt_health_body)
        self.assertIn("'OK'", main_health_body)
        self.assertIn("'OK'", mqtt_health_body)


if __name__ == "__main__":
    unittest.main()
