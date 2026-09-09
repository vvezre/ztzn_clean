# coding=utf-8
import json
import shutil
import tempfile
import time
import unittest
from cleaning_position import CleaningPositionHistory

from modeling_simulator import ModelingSimulatorController, build_simulator_lan_app


class CleaningSimulatorTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.mkdtemp()
        self.controller = ModelingSimulatorController(self.directory, playback_step_cm=1000, playback_interval=0.01)
        self.app = build_simulator_lan_app(self.controller)
        self.client = self.app.test_client()
        response = self.client.post('/auth/login', json={'username': 'admin', 'password': 'admin123'})
        self.client.environ_base['HTTP_AUTHORIZATION'] = 'Bearer ' + response.get_json()['data']['accessToken']

    def tearDown(self):
        self.controller.player.cancel()
        self.controller.manual_steering_controller.close()
        shutil.rmtree(self.directory)

    def prepare(self):
        self.assertTrue(self.controller.preload_scenario()['success'])
        self.assertTrue(self.controller.save_modeling_task('clean-test')['success'])
        self.assertTrue(self.controller.set_current_task('clean-test')['success'])

    def fetch(self, kind):
        response = self.client.get('/api/t-railcar/cleaning-' + kind + '/999999')
        self.assertEqual(200, response.status_code)
        return response.get_json()['data']

    def wait_for_point(self, previous_run=None):
        deadline = time.time() + 3
        while time.time() < deadline:
            history = self.fetch('position-history')
            if history['points'] and history['runId'] != previous_run:
                return history
            time.sleep(0.01)
        self.fail('no simulated cleaning position')

    def test_empty_before_start_and_failed_start_does_not_create_run(self):
        before = self.fetch('realtime-position')
        self.assertIsNone(before['runId'])
        self.assertEqual('IDLE', before['controlState'])
        self.assertIsNotNone(before['heading'])
        self.assertGreaterEqual(before['heading'], 0.0)
        self.assertLess(before['heading'], 360.0)
        self.assertFalse(self.controller.auto_drive()['success'])
        self.assertEqual([], self.fetch('position-history')['points'])
        self.assertEqual('START_FAILED', self.fetch('realtime-position')['controlState'])

    def test_saved_route_origin_works_after_new_empty_modeling_session(self):
        self.prepare()
        self.controller.start_modeling()
        self.controller.set_current_task('clean-test')
        self.assertTrue(self.controller.auto_drive()['success'])
        first = self.wait_for_point()
        self.assertEqual('clean-test', first['taskName'])
        self.assertTrue(first['coordinateReady'])
        self.assertEqual({'x': 0, 'y': 0}, first['points'][0])
        self.assertTrue(self.controller.player.wait(5))
        history = self.fetch('position-history')
        self.assertEqual(first['runId'], history['runId'])
        self.assertTrue(history['rtkFixAvailable'])
        self.assertEqual('COMPLETE', self.fetch('realtime-position')['controlState'])

    def test_duplicate_start_pause_resume_and_finished_history(self):
        self.prepare()
        self.controller.player.interval = 0.2
        self.controller.auto_drive()
        first = self.wait_for_point()
        for _ in range(5):
            self.assertTrue(self.controller.auto_drive()['success'])
        self.assertEqual(first['runId'], self.fetch('position-history')['runId'])
        self.controller.stop()
        paused = self.fetch('position-history')
        self.assertEqual(paused, self.fetch('position-history'))
        self.controller.go_on()
        self.assertEqual(first['runId'], self.fetch('position-history')['runId'])
        self.controller.parking()
        stopped = self.fetch('position-history')
        self.assertEqual(first['runId'], stopped['runId'])
        self.assertTrue(self.controller.set_current_task('clean-test', False)['success'])
        self.assertEqual(stopped['points'], self.fetch('position-history')['points'])
        self.controller.auto_drive()
        new = self.wait_for_point(previous_run=first['runId'])
        self.assertNotEqual(first['runId'], new['runId'])

    def test_reading_history_never_changes_saved_task_or_model(self):
        self.prepare()
        before = json.dumps(self.controller.get_saved_routes(), sort_keys=True)
        self.controller.auto_drive()
        self.wait_for_point()
        for _ in range(10):
            self.fetch('realtime-position')
            self.fetch('position-history')
        after = json.dumps(self.controller.get_saved_routes(), sort_keys=True)
        self.assertEqual(before, after)

    def test_completion_persists_tail_even_without_frontend_polling(self):
        self.prepare()
        self.controller.auto_drive()
        self.assertTrue(self.controller.player.wait(5))
        tracker = self.controller.cleaning_service.history_store
        restored = CleaningPositionHistory(tracker.path)
        expected = tracker.history()['points']
        self.assertEqual(expected, restored.history()['points'])
        self.assertTrue(expected)
        position = self.controller.current_position()
        self.assertLessEqual(abs(expected[-1]['x'] - position['local_x']), 1)
        self.assertLessEqual(abs(expected[-1]['y'] - position['local_y']), 1)
