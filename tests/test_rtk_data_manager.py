import unittest

import RTKDataManager as rtk_module


class FakeThread(object):
    instances = []

    def __init__(self, target=None, name=None):
        self.target = target
        self.name = name
        self.daemon = False
        self.started = False
        FakeThread.instances.append(self)

    def start(self):
        self.started = True


class RTKDataManagerStartTest(unittest.TestCase):
    def test_start_does_not_preopen_serial_before_reader_loop(self):
        original_thread = rtk_module.threading.Thread
        FakeThread.instances = []
        manager = rtk_module.RTKDataManager(port="/dev/ttyUSB0", baudrate=115200)
        open_calls = []

        def fake_open_serial():
            open_calls.append(manager.port)
            return True

        manager._open_serial = fake_open_serial

        try:
            rtk_module.threading.Thread = FakeThread

            manager.start()
        finally:
            rtk_module.threading.Thread = original_thread

        self.assertEqual(open_calls, [])
        self.assertTrue(manager.running)
        self.assertEqual(len(FakeThread.instances), 1)
        self.assertTrue(FakeThread.instances[0].started)
        self.assertTrue(FakeThread.instances[0].daemon)


if __name__ == "__main__":
    unittest.main()
