import io
import os
import re
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAIN_PATH = os.path.join(ROOT, "main.py")
MQTT_PATH = os.path.join(ROOT, "mqtt_integration.py")


def read_file(path):
    with io.open(path, "r", encoding="utf-8", errors="ignore") as handle:
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


class TaskOriginCheckTest(unittest.TestCase):
    def test_main_exposes_task_origin_status_and_endpoint(self):
        source = read_file(MAIN_PATH)

        self.assertIn("TASK_ORIGIN_TOLERANCE_METERS", source)
        self.assertIn("def _build_task_origin_check_result(", source)
        self.assertIn("def _build_task_origin_status_fields(", source)
        self.assertIn('@app.route("/vehicle/isAtTaskOrigin"', source)

        status_body = function_body(source, "_build_vehicle_status_payload")
        self.assertIn("_build_task_origin_status_fields(task_params)", status_body)
        self.assertIn("payload.update(task_origin_status)", status_body)

        vehicle_info_body = function_body(source, "getVehicleInfo")
        self.assertIn("metadata.update(_build_task_origin_status_fields())", vehicle_info_body)

    def test_auto_drive_by_rtk_entry_uses_start_validation(self):
        source = read_file(MAIN_PATH)
        body = function_body(source, "autoDriveByRTK")

        self.assertIn("validation = _validate_auto_drive_request()", body)
        self.assertIn("_mark_runtime_blocked(", body)
        self.assertIn("return jsonify(validation)", body)

    def test_auto_drive_validation_uses_task_origin_result(self):
        source = read_file(MAIN_PATH)

        for name in ("_validate_auto_drive_request_legacy", "_validate_auto_drive_request"):
            body = function_body(source, name)
            self.assertIn("_build_task_origin_check_result(task_params, detail)", body, name)
            self.assertIn("NOT_AT_TASK_ORIGIN", source)

        legacy_body = function_body(source, "_validate_auto_drive_request_legacy")
        self.assertNotIn("task_obj.get('taskList')", legacy_body)

    def test_runtime_detail_and_mqtt_publish_task_origin_fields(self):
        main_source = read_file(MAIN_PATH)
        mqtt_source = read_file(MQTT_PATH)

        runtime_body = function_body(main_source, "_build_runtime_detail")
        for field in (
            "distanceToTaskOriginM",
            "taskOriginToleranceM",
            "isAtTaskOrigin",
            "taskStartLat",
            "taskStartLon",
        ):
            self.assertIn(field, runtime_body)
            self.assertIn(field, mqtt_source)

        self.assertIn("def _build_task_origin_status_fields(", mqtt_source)
        self.assertIn("taskOrigin", mqtt_source)
        self.assertIn("currentLocation", mqtt_source)


if __name__ == "__main__":
    unittest.main()
