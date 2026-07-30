import os
import tempfile
import unittest

from dev_console.state_readers import read_log_lines


class DevConsoleStateReadersTest(unittest.TestCase):
    def test_read_log_lines_replaces_invalid_utf8_bytes(self):
        fd, path = tempfile.mkstemp(suffix=".log")
        os.close(fd)
        try:
            with open(path, "wb") as fp:
                fp.write(b"valid line\ninvalid: \xff\n")

            result = read_log_lines(path)
        finally:
            os.remove(path)

        self.assertEqual(result["lines"][0], "valid line")
        self.assertIn("invalid:", result["lines"][1])


if __name__ == "__main__":
    unittest.main()
