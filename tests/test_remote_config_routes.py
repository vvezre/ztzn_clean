import io
import os
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
MAIN_PATH = os.path.join(ROOT, "main.py")
NTRIP_RUNTIME_PATH = os.path.join(ROOT, "ntrip_runtime.py")


def read_file(path):
    with io.open(path, "r", encoding="utf-8", errors="ignore") as handle:
        return handle.read()


class RemoteConfigRoutesTest(unittest.TestCase):
    def test_ntrip_config_routes_are_exposed_and_mask_passwords(self):
        source = read_file(MAIN_PATH)

        self.assertIn('/vehicle/getNtripConfig', source)
        self.assertIn('/vehicle/updateNtripConfig', source)
        self.assertIn('ntrip_config.json', source)
        self.assertIn('reset_shared_runtime', source)
        self.assertIn("payload['password'] = '******'", source)

    def test_device_config_routes_update_mqtt_identity(self):
        source = read_file(MAIN_PATH)

        self.assertIn('/vehicle/getDeviceConfig', source)
        self.assertIn('/vehicle/updateDeviceConfig', source)
        self.assertIn('mqtt_config.json', source)
        self.assertIn("RAILCAR/S/", source)
        self.assertIn("RAILCAR/R/", source)

    def test_ntrip_runtime_can_be_reset_after_config_update(self):
        source = read_file(NTRIP_RUNTIME_PATH)

        self.assertIn("def reset_shared_runtime", source)
        self.assertIn("_shared_runtime.close()", source)
        self.assertIn("_shared_runtime = None", source)


if __name__ == "__main__":
    unittest.main()
