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


class VideoStreamDisabledTest(unittest.TestCase):
    def test_camera_stream_endpoint_does_not_read_or_stream_frames(self):
        body = function_body(read_main(), "cameraStream")

        self.assertIn("video stream disabled", body)
        self.assertNotIn("_camera_stream_generator", body)
        self.assertNotIn("_read_camera_frame_for_http", body)
        self.assertNotIn("stopThenStart", body)


if __name__ == "__main__":
    unittest.main()
