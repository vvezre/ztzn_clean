import io
import os
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAIN_PATH = os.path.join(ROOT, "main.py")


class ModelingMainRoutesTest(unittest.TestCase):
    def test_main_registers_modeling_routes(self):
        with io.open(MAIN_PATH, "r", encoding="utf-8") as handle:
            source = handle.read()

        self.assertIn("from modeling_routes import register_modeling_routes", source)
        self.assertIn("MODELING_STORE_DIR", source)
        self.assertIn("register_modeling_routes(app", source)


if __name__ == "__main__":
    unittest.main()
