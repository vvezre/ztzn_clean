# coding=utf-8
import tempfile
import unittest

from modeling_simulator import (
    ModelingSimulatorController,
    SimulatorPositionHistory,
    build_simulator_lan_app,
)


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


if __name__ == '__main__':
    unittest.main()
