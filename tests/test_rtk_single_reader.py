import os
import re
import unittest
from io import open


RUNTIME_ROOT = os.path.dirname(os.path.dirname(__file__))
MAIN_PATH = os.path.join(RUNTIME_ROOT, "main.py")


def _main_text_without_comments():
    with open(MAIN_PATH, "r", encoding="utf-8-sig") as handle:
        lines = handle.readlines()
    return "\n".join(line for line in lines if not line.lstrip().startswith("#"))


class RTKSingleReaderTest(unittest.TestCase):
    def test_main_does_not_open_additional_rtk_readers(self):
        text = _main_text_without_comments()
        direct_reads = re.findall(r"\butil\.readRTK_v2\s*\(", text)
        self.assertEqual(direct_reads, [])

    def test_main_does_not_start_legacy_rtk_correction_threads(self):
        text = _main_text_without_comments()
        legacy_thread_starts = re.findall(
            r"threading\.Thread\s*\(\s*target\s*=\s*correctByRTK(?:Test)?\b",
            text,
        )
        self.assertEqual(legacy_thread_starts, [])


if __name__ == "__main__":
    unittest.main()
