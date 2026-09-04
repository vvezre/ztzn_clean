# coding=utf-8
import tempfile
import time
import unittest

from modeling_simulator import (
    ModelingSimulatorController,
    SimulatorPositionHistory,
    build_simulator_lan_app,
    SCENARIO_POINTS,
    SCENARIO_NEW_AREA_INDEXES,
    SCENARIO_NEW_LINK_INDEXES,
)
from position_history import ModelingPositionHistory


class ModelingSimulatorLanTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.clock = [100.0]
        self.controller = ModelingSimulatorController(self.directory.name, now=lambda: 1000)
        self.history = SimulatorPositionHistory(
            self.controller,
            now=lambda: self.clock[0],
        )
        self.app = build_simulator_lan_app(
            self.controller,
            position_history=self.history,
        )
        self.client = self.app.test_client()

    def tearDown(self):
        self.controller.manual_steering_controller.close()
        self.controller.player.cancel()
        self.directory.cleanup()

    def _authorized_client(self):
        response = self.client.post('/auth/login', json={
            'username': 'admin',
            'password': 'admin123',
        })
        self.assertEqual(200, response.status_code)
        payload = response.get_json()
        self.assertEqual(200, payload['code'])
        token = payload['data']['accessToken']
        self.client.environ_base['HTTP_AUTHORIZATION'] = 'Bearer {}'.format(token)
        return self.client

    def test_position_routes_require_login(self):
        realtime = self.client.get('/api/t-railcar/realtime-position/999999')
        history = self.client.get('/api/t-railcar/position-history/999999')
        self.assertEqual(401, realtime.status_code)
        self.assertEqual(401, history.status_code)

    def test_realtime_position_uses_simulator_coordinates(self):
        client = self._authorized_client()
        self.controller._on_playback_position({'local_x': 123, 'local_y': 456})
        self.history.observe(self.controller.current_position())
        response = client.get('/api/t-railcar/realtime-position/999999')
        self.assertEqual(200, response.status_code)
        self.assertEqual({
            'success': True,
            'data': {
                'x': 123,
                'y': 456,
                'coordinateReady': True,
                'rtkFixAvailable': True,
            },
        }, response.get_json())

    def test_history_records_simulator_playback_positions(self):
        client = self._authorized_client()
        self.assertEqual(1.0, self.history.history_interval)
        self.assertEqual(1500, self.history.max_points)
        self.controller._on_playback_position({'local_x': 5, 'local_y': 1})
        self.history.observe(self.controller.current_position())
        self.clock[0] += 1.0
        self.history.observe(self.controller.current_position())
        self.controller._on_playback_position({'local_x': 11, 'local_y': 2})
        self.clock[0] += 1.0
        self.history.observe(self.controller.current_position())
        response = client.get('/api/t-railcar/position-history/999999')
        self.assertEqual(200, response.status_code)
        self.assertEqual({
            'success': True,
            'data': {
                'points': [
                    {'x': 0, 'y': 0},
                    {'x': 5, 'y': 1},
                    {'x': 11, 'y': 2},
                ],
                'coordinateReady': True,
                'rtkFixAvailable': True,
            },
        }, response.get_json())

    def test_wrong_product_id_is_rejected(self):
        client = self._authorized_client()
        response = client.get('/api/t-railcar/realtime-position/250006')
        self.assertEqual(404, response.status_code)

    def test_full_three_area_flow_matches_robot_attribution_and_reset(self):
        client = self._authorized_client()
        # Real FSM data, same RTK samples and clock, two independent trackers.
        robot_history = ModelingPositionHistory(self.directory.name, now=lambda: self.clock[0])
        self.assertTrue(self.controller.start_modeling(restart=True)['success'])
        for index, spec in enumerate(SCENARIO_POINTS):
            if index in SCENARIO_NEW_LINK_INDEXES:
                self.assertTrue(self.controller.new_modeling_link()['success'])
            if index in SCENARIO_NEW_AREA_INDEXES:
                self.assertTrue(self.controller.new_modeling_area()['success'])
            if spec['pointType'] == 'area':
                result = self.controller.sample_modeling_point()
            else:
                result = self.controller.sample_modeling_link_point()
            self.assertTrue(result['success'], result)
            self.clock[0] += 1
            position = self.controller.current_position()
            self.history.observe(position)
            robot_history.update(position['lat'], position['lon'], True, force_context=True)

        before = self.history.history()
        self.assertEqual(16, len(before['points']))
        for area in (1, 2, 3):
            response = client.get('/api/t-railcar/position-history/999999?areaNumber={}'.format(area))
            self.assertEqual(200, response.status_code)
            data = response.get_json()['data']
            self.assertEqual(area, data['areaNumber'])
            self.assertEqual(4, len(data['points']))
            actual = robot_history.history(area)['points']
            self.assertEqual(len(actual), len(data['points']))
            # Simulator stores scenario x/y directly; robot converts rounded
            # simulated lat/lon. This can differ by at most 1 cm.
            for simulated, robot in zip(data['points'], actual):
                self.assertLessEqual(abs(simulated['x'] - robot['x']), 1)
                self.assertLessEqual(abs(simulated['y'] - robot['y']), 1)

        self.assertEqual([], self.history.history(4)['points'])
        old_model = self.controller.session.current()['modelId']
        # Read-only queries preserve a draft; start commands always replace it.
        self.assertTrue(self.controller.get_modeling_state()['success'])
        self.assertEqual(old_model, self.controller.session.current()['modelId'])
        self.assertEqual(before, self.history.history())
        self.assertTrue(self.controller.start_modeling(restart=False)['success'])
        self.assertNotEqual(old_model, self.controller.session.current()['modelId'])
        self.assertEqual([], self.history.history(2)['points'])
        self.assertEqual([], self.history.history(3)['points'])
        self.assertLessEqual(len(self.history.history()['points']), 1)

    def _command(self, command, params=None):
        response = self.client.post('/api/t-railcar/command', json={
            'productId': '999999', 'command': command, 'params': params or {},
        })
        self.assertEqual(200, response.status_code)
        command_id = response.get_json()['commandId']
        deadline = time.time() + 3
        while time.time() < deadline:
            status = self.client.get('/api/command-status/' + command_id).get_json()
            if status.get('terminal'):
                self.assertEqual('SUCCEEDED', status['status'], status)
                return
            time.sleep(0.01)
        self.fail('simulator command timed out')

    def test_lan_start_clears_previous_capture_and_history_without_restart_param(self):
        self._authorized_client()
        self._command('start_modeling')
        for params in ({}, {'restart': False}, {'restart': True}):
            self._command('sample_modeling_point')
            self._command('sample_modeling_point')
            self.clock[0] += 1
            old_points = self.history.history(1)['points']
            self.assertTrue(old_points)
            old_model = self.controller.session.current()['modelId']
            self._command('start_modeling', params)
            fresh = self.controller.session.current()
            self.assertNotEqual(old_model, fresh['modelId'])
            self.assertEqual(0, fresh['totalAreaPointCount'])
            self.assertEqual(0, fresh['totalLinkPointCount'])
            # Simulator starts at (0,0); none of the previous captured movement survives.
            for point in self.history.history(1)['points']:
                self.assertEqual({'x': 0, 'y': 0}, point)
            self.assertEqual([], self.history.history(2)['points'])

    def test_new_model_preserves_saved_route_variants(self):
        self.assertTrue(self.controller.preload_scenario()['success'])
        self.assertTrue(self.controller.save_modeling_task('saved-route')['success'])
        routes = self.controller.get_saved_routes()
        self.assertEqual(1, len(routes['data']['routes']))
        for kwargs in ({}, {'restart': False}):
            self.assertTrue(self.controller.start_modeling(**kwargs)['success'])
            self.assertEqual(routes, self.controller.get_saved_routes())
        self.assertTrue(self.controller.set_current_task('saved-route', return_to_origin=False)['success'])

    def test_filtered_history_before_modeling_is_empty(self):
        client = self._authorized_client()
        response = client.get('/api/t-railcar/position-history/999999?areaNumber=1')
        self.assertEqual(200, response.status_code)
        self.assertEqual([], response.get_json()['data']['points'])


if __name__ == '__main__':
    unittest.main()
