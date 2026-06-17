import os
import re
import unittest


RUNTIME_ROOT = os.path.dirname(os.path.dirname(__file__))
FILES_TO_CHECK = ("GPSuse.py", "Ntrip2Uart3.py", "vision_line_detection.py")
ANNOTATION_RE = re.compile(r"^\s*def\s+\w+\s*\([^)]*:\s*[^)]*\)", re.MULTILINE)
FSTRING_RE = re.compile(r"(^|[^A-Za-z0-9_])([fF][rRbBuU]{0,2}|[rRbBuU]{0,2}[fF])(['\"])", re.MULTILINE)


def _iter_checked_files():
    for name in FILES_TO_CHECK:
        yield os.path.join(RUNTIME_ROOT, name)


def _active_lines(text):
    for lineno, line in enumerate(text.splitlines(), 1):
        stripped = line.lstrip()
        if stripped.startswith("#"):
            continue
        yield lineno, line


def find_python2_incompatibilities():
    issues = []
    for path in _iter_checked_files():
        basename = os.path.basename(path)
        with open(path, "r", encoding="utf-8-sig") as handle:
            text = handle.read()

        for lineno, line in _active_lines(text):
            if FSTRING_RE.search(line):
                issues.append("%s:%s uses f-string" % (basename, lineno))
            if ANNOTATION_RE.search(line):
                issues.append("%s:%s uses argument annotation" % (basename, lineno))
            if "daemon=True" in line:
                issues.append("%s:%s passes daemon= to threading.Thread" % (basename, lineno))
            if "ConnectionError(" in line:
                issues.append("%s:%s uses builtin ConnectionError" % (basename, lineno))

    return issues


class Python2CompatibilityTest(unittest.TestCase):
    def test_runtime_python_files_avoid_python3_only_constructs(self):
        self.assertEqual(find_python2_incompatibilities(), [])


if __name__ == "__main__":
    unittest.main()
