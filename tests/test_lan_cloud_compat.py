# coding=utf-8
import time
import unittest

from flask import Flask

from lan_local_auth import LocalAuthManager, create_password_record
from lan_cloud_compat import register_lan_cloud_compat_routes


class FakeCommandHandler(object):
    def __init__(self):
        self.calls = []
        self.results = {}

    def handle(self, message):
        self.calls.append(message)
        command = message.get('command')
        result = self.results.get(command)
        if callable(result):
            return result(message)
        if result is not None:
            return result
        return {
            'success': True,
            'message': '{} ok'.format(command),
            'data': {},
        }


class LanCloudCompatibilityRouteTests(unittest.TestCase):
    def setUp(self):
        self.handler = FakeCommandHandler()
        self.app = Flask(__name__)
        self.auth_manager = LocalAuthManager(config={
            'tokenSecret': 'test-secret-must-have-at-least-32-characters',
            'users': [{
                'userId': 1,
                'username': 'admin',
                'passwordHash': create_password_record('test-password'),
                'realName': '本地管理员',
                'roleId': 1,
                'roleName': 'admin',
                'permissions': [],
                'status': 'enable',
            }],
        })
        self.identity = {
            'id': 1,
            'productId': '250006',
            'productType': '-T01',
            'productModel': '-T01',
            'companyCode': 'ZTZN-PVC',
            'serialNumber': '-T01250006',
            'deviceId': '-T01250006',
        }
        self.bridge = register_lan_cloud_compat_routes(
            self.app,
            lambda: self.handler,
            auth_manager=self.auth_manager,
            device_identity_provider=lambda: self.identity,
        )
        self.client = self.app.test_client()
        login = self.client.post('/auth/login', json={
            'username': 'admin',
            'password': 'test-password',
        })
        self.assertEqual(200, login.status_code)
        login_payload = login.get_json()
        self.assertEqual(200, login_payload['code'])
        self.access_token = login_payload['data']['accessToken']
        self.refresh_token = login_payload['data']['refreshToken']
        self.client.environ_base['HTTP_AUTHORIZATION'] = 'Bearer {}'.format(
            self.access_token
        )

    def _wait_for_terminal(self, command_id):
        deadline = time.time() + 2.0
        while time.time() < deadline:
            response = self.client.get('/api/command-status/{}'.format(command_id))
            payload = response.get_json()
            if payload.get('terminal'):
                return payload
            time.sleep(0.01)
        self.fail('command did not become terminal')

    def test_command_uses_cloud_request_and_status_contract(self):
        self.handler.results['drive'] = {
            'success': True,
            'message': 'drive started',
            'data': {'motionState': 'forward'},
        }
        response = self.client.post('/api/t-railcar/command', json={
            'productId': '250006',
            'command': 'drive',
            'params': {'distance': 0},
        })
        self.assertEqual(200, response.status_code)
        dispatch = response.get_json()
        self.assertTrue(dispatch['success'])
        self.assertEqual('DISPATCHED', dispatch['commandStatus'])
        self.assertEqual('-T01250006', dispatch['deviceId'])
        self.assertTrue(dispatch['commandId'].startswith('cmd_'))

        snapshot = self._wait_for_terminal(dispatch['commandId'])
        self.assertEqual('SUCCEEDED', snapshot['status'])
        self.assertTrue(snapshot['terminal'])
        self.assertEqual('drive', snapshot['action'])
        self.assertEqual('drive started', snapshot['message'])
        self.assertEqual(
            'forward',
            snapshot['detail']['result']['data']['motionState'],
        )
        call = self.handler.calls[0]
        self.assertEqual('250006', call['product_id'])
        self.assertEqual({'distance': 0}, call['params'])
        self.assertEqual(dispatch['commandId'], call['command_id'])

    def test_login_matches_cloud_contract_and_rejects_bad_password(self):
        successful = self.client.post('/auth/login', json={
            'username': 'admin',
            'password': 'test-password',
        }).get_json()
        self.assertEqual(200, successful['code'])
        self.assertEqual('admin', successful['data']['user']['username'])
        self.assertEqual('admin', successful['data']['user']['roleName'])
        self.assertEqual(3600, successful['data']['expiresIn'])

        failed = self.client.post('/auth/login', json={
            'username': 'admin',
            'password': 'wrong-password',
        })
        self.assertEqual(200, failed.status_code)
        self.assertEqual(401, failed.get_json()['code'])

    def test_refresh_returns_new_access_token_and_keeps_refresh_token(self):
        self.client.environ_base['HTTP_AUTHORIZATION'] = 'Bearer {}'.format(
            self.refresh_token
        )
        refreshed = self.client.post('/auth/refresh')
        self.assertEqual(200, refreshed.status_code)
        payload = refreshed.get_json()
        self.assertEqual(200, payload['code'])
        self.assertEqual(self.refresh_token, payload['data']['refreshToken'])
        self.assertNotEqual(self.access_token, payload['data']['accessToken'])

    def test_lan_business_endpoint_requires_local_access_token(self):
        self.client.environ_base.pop('HTTP_AUTHORIZATION', None)
        response = self.client.get('/api/t-railcar/saved-routes/250006')
        self.assertEqual(401, response.status_code)
        self.assertEqual(401, response.get_json()['code'])
        self.assertEqual([], self.handler.calls)

    def test_logout_revokes_current_access_token(self):
        response = self.client.post('/auth/logout')
        self.assertEqual(200, response.status_code)
        self.assertEqual(200, response.get_json()['code'])
        rejected = self.client.get('/api/t-railcar/saved-routes/250006')
        self.assertEqual(401, rejected.status_code)

        self.client.environ_base['HTTP_AUTHORIZATION'] = 'Bearer {}'.format(
            self.refresh_token
        )
        refresh_rejected = self.client.post('/auth/refresh')
        self.assertEqual(401, refresh_rejected.get_json()['code'])

    def test_failed_command_is_reported_by_status_polling(self):
        self.handler.results['finish_modeling'] = {
            'success': False,
            'message': '区域点不足',
            'data': {'code': 'AREA_POINTS_INSUFFICIENT'},
        }
        dispatch = self.client.post('/api/t-railcar/command', json={
            'productId': '250006',
            'command': 'finish_modeling',
            'params': {},
        }).get_json()
        snapshot = self._wait_for_terminal(dispatch['commandId'])
        self.assertEqual('FAILED', snapshot['status'])
        self.assertEqual('区域点不足', snapshot['message'])
        self.assertTrue(snapshot['terminal'])

    def test_async_commands_keep_http_arrival_order(self):
        command_ids = []
        for sequence in (1, 2, 3):
            payload = self.client.post('/api/t-railcar/command', json={
                'productId': '250006',
                'command': 'manual_steering',
                'params': {
                    'action': 'keepalive',
                    'direction': 'right',
                    'controlId': 'press-1',
                    'sequence': sequence,
                },
            }).get_json()
            command_ids.append(payload['commandId'])

        self._wait_for_terminal(command_ids[-1])
        sequences = [call['params']['sequence'] for call in self.handler.calls]
        self.assertEqual([1, 2, 3], sequences)

    def test_missing_command_status_matches_cloud_shape(self):
        payload = self.client.get('/api/command-status/cmd_missing').get_json()
        self.assertFalse(payload['exists'])
        self.assertEqual('UNKNOWN', payload['status'])
        self.assertFalse(payload['terminal'])

    def test_saved_routes_adds_cloud_product_metadata(self):
        self.handler.results['get_saved_routes'] = {
            'success': True,
            'message': 'saved routes fetched',
            'data': {
                'currentTaskName': '路线一',
                'currentReturnToOrigin': False,
                'routes': [{
                    'taskName': '路线一',
                    'areaOrder': [2, 1],
                    'linkPoints': [{'linkNumber': 1}],
                }],
            },
        }
        response = self.client.get('/api/t-railcar/saved-routes/250006')
        self.assertEqual(200, response.status_code)
        payload = response.get_json()
        self.assertTrue(payload['success'])
        self.assertEqual('250006', payload['data']['productId'])
        self.assertEqual('-T01250006', payload['data']['serialNumber'])
        self.assertFalse(payload['data']['currentReturnToOrigin'])
        self.assertEqual([2, 1], payload['data']['routes'][0]['areaOrder'])

    def test_select_current_task_preserves_return_to_origin(self):
        self.handler.results['set_current_task'] = {
            'success': True,
            'message': 'selected',
            'data': {'taskCount': 25},
        }
        response = self.client.post('/api/t-railcar/tasks/current', json={
            'productId': '250006',
            'taskName': '测试路线',
            'returnToOrigin': False,
        })
        self.assertEqual(200, response.status_code)
        payload = response.get_json()
        self.assertTrue(payload['success'])
        self.assertFalse(payload['data']['returnToOrigin'])
        self.assertEqual(25, payload['data']['taskCount'])
        call = self.handler.calls[-1]
        self.assertEqual('set_current_task', call['command'])
        self.assertEqual('测试路线', call['params']['taskName'])
        self.assertFalse(call['params']['returnToOrigin'])

    def test_modeling_points_keeps_cloud_points_only_response(self):
        self.handler.results['get_modeling_points'] = {
            'success': True,
            'message': 'points fetched',
            'data': {'points': [{'id': 'p1', 'areaNumber': 1}]},
        }
        response = self.client.get('/api/t-railcar/modeling-points/250006')
        self.assertEqual({'points': [{'id': 'p1', 'areaNumber': 1}]}, response.get_json())

    def test_device_status_and_shadow_are_built_from_local_status(self):
        self.handler.results['get_status'] = {
            'success': True,
            'message': 'status ok',
            'data': {
                'status': 'active',
                'online_state': 'ONLINE',
                'mission_state': 'RUNNING',
                'control_state': 'RUNNING',
                'health_state': 'OK',
                'fault_state': '',
                'battery': 88.0,
                'lat': 32.0,
                'lon': 118.9,
                'heading': 90.0,
                'timestamp': 1788310800,
                'supported_actions': ['stop'],
                'supported_params': [],
                'supported_status_fields': ['control_state'],
            },
        }
        status = self.client.get('/api/device-status/-T01250006').get_json()
        self.assertTrue(status['exists'])
        self.assertEqual('running', status['status'])
        self.assertEqual(88.0, status['battery'])
        self.assertEqual(118.9, status['location']['lon'])

        shadow = self.client.get('/api/device-status/-T01250006/shadow').get_json()
        self.assertEqual('ONLINE', shadow['onlineState'])
        self.assertEqual('RUNNING', shadow['controlState'])
        self.assertEqual(90.0, shadow['currentLocation']['heading'])

    def test_login_landing_device_interfaces_return_the_local_robot(self):
        self.handler.results['get_status'] = {
            'success': True,
            'message': 'status ok',
            'data': {
                'online_state': 'ONLINE',
                'mission_state': 'IDLE',
                'control_state': 'READY',
                'health_state': 'NORMAL',
                'fault_state': 'NONE',
                'battery': 90.0,
                'lat': 32.0,
                'lon': 118.9,
                'heading': 45.0,
                'timestamp': 1788310800,
            },
        }
        mini_list = self.client.get('/api/mini-app/devices').get_json()
        self.assertEqual(200, mini_list['code'])
        self.assertEqual('250006', mini_list['data'][0]['productId'])
        self.assertTrue(mini_list['data'][0]['online'])
        self.assertEqual('idle', mini_list['data'][0]['status'])

        my_devices = self.client.get('/device/my-devices').get_json()
        self.assertEqual('-T01250006', my_devices['data'][0]['serialNumber'])
        self.assertTrue(my_devices['data'][0]['bound'])

        scanned = self.client.post('/device/scan', json={
            'companyCode': 'ZTZN-PVC',
            'productType': '-T01',
            'productId': '250006',
        }).get_json()
        self.assertEqual(200, scanned['code'])
        self.assertEqual('250006', scanned['data']['productId'])

    def test_invalid_product_id_is_rejected_without_running_command(self):
        response = self.client.post('/api/t-railcar/command', json={
            'productId': '6',
            'command': 'drive',
            'params': {},
        })
        self.assertEqual(400, response.status_code)
        self.assertEqual([], self.handler.calls)


if __name__ == '__main__':
    unittest.main()
