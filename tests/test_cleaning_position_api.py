# coding=utf-8
from __future__ import absolute_import
import json
import unittest
from flask import Flask
from lan_cloud_compat import register_lan_cloud_compat_routes
from lan_local_auth import LocalAuthManager, create_password_record


class CleaningPositionApiTests(unittest.TestCase):
    def setUp(self):
        self.app = Flask(__name__)
        self.calls = []
        self.live = {'taskName': u'清扫测试', 'runId': 'clean_test', 'x': 123, 'y': 456,
                     'heading': 92.6,
                     'coordinateReady': True, 'rtkFixAvailable': True,
                     'atTaskOrigin': False, 'cleaningCoordinateReady': True,
                     'controlState': 'RUNNING', 'isCleaning': True}
        self.history = dict(self.live, points=[{'x': 0, 'y': 0}, {'x': 5, 'y': 1},
                           {'x': 30, 'y': 40, 'breakBefore': True}], simplified=True)
        self.log_item = {
            'id': 'log_20260916_001', 'runId': 'clean_test', 'productId': '250006',
            'serialNumber': '-T01250006', 'taskName': u'清扫测试',
            'startTime': '2026-09-16T08:32:11+08:00',
            'endTime': '2026-09-16T09:14:29+08:00', 'durationSeconds': 2538,
            'distanceMeters': None, 'status': 'COMPLETED', 'endReason': u'路线正常完成',
        }
        self.log_detail = dict(self.log_item, trajectory={
            'coordinateSystem': 'LOCAL_XY', 'simplified': False,
            'points': [{'x': 0, 'y': 0, 'heading': 90, 'timestamp': 1789518731000}],
        }, plannedRoute={'areaPoints': [], 'linkPoints': [], 'pathPoints': []})
        auth = LocalAuthManager(config={
            'tokenSecret': 'test-secret-must-have-at-least-32-characters',
            'users': [{'userId': 1, 'username': 'admin', 'passwordHash': create_password_record('test'),
                       'roleId': 1, 'roleName': 'admin', 'status': 'enable', 'permissions': []}],
        })
        register_lan_cloud_compat_routes(self.app, lambda: self.calls.append('command'),
            auth_manager=auth, device_identity_provider=lambda: {'productId': '250006'},
            cleaning_realtime_provider=lambda: self.live, cleaning_history_provider=lambda: self.history,
            cleaning_logs_provider=lambda product_id, page, page_size: {
                'list': [self.log_item], 'total': 1, 'page': page, 'pageSize': page_size,
            },
            cleaning_log_detail_provider=lambda log_id, product_id: (
                self.log_detail if log_id == self.log_item['id'] and product_id == '250006' else None
            ))
        self.client = self.app.test_client()

    def get(self, kind, product='250006'):
        return self.client.get('/api/t-railcar/cleaning-' + kind + '/' + product)

    def payload(self, response):
        return json.loads(response.data.decode('utf-8'))

    def login(self):
        response = self.client.post('/auth/login', data=json.dumps({'username': 'admin', 'password': 'test'}),
                                    content_type='application/json')
        token = self.payload(response)['data']['accessToken']
        self.client.environ_base['HTTP_AUTHORIZATION'] = 'Bearer ' + token

    def test_both_interfaces_require_lan_token(self):
        for kind in ('realtime-position', 'position-history'):
            self.assertEqual(401, self.get(kind).status_code)
        self.assertEqual([], self.calls)

    def test_both_interfaces_reject_other_device(self):
        self.login()
        for kind in ('realtime-position', 'position-history'):
            self.assertEqual(404, self.get(kind, '999999').status_code)

    def test_realtime_contract_and_no_motion_commands(self):
        self.login()
        response = self.get('realtime-position')
        self.assertEqual(200, response.status_code)
        self.assertEqual({'success': True, 'data': {
            'taskName': u'清扫测试', 'runId': 'clean_test', 'x': 123, 'y': 456,
            'heading': 92.6,
            'coordinateReady': True, 'rtkFixAvailable': True,
            'controlState': 'RUNNING',
            'isCleaning': True,
        }}, self.payload(response))
        self.assertEqual([], self.calls)

    def test_cleaning_realtime_heading_is_normalized_and_requires_rtk_fix(self):
        self.login()
        self.live['heading'] = -10
        result = self.payload(self.get('realtime-position'))['data']
        self.assertEqual(350.0, result['heading'])

        self.live['rtkFixAvailable'] = False
        result = self.payload(self.get('realtime-position'))['data']
        self.assertIsNone(result['heading'])

    def test_at_origin_flag_does_not_gate_valid_xy(self):
        self.login()
        self.live['atTaskOrigin'] = True
        self.live['cleaningCoordinateReady'] = True
        result = self.payload(self.get('realtime-position'))['data']
        self.assertTrue(result['coordinateReady'])
        self.live['atTaskOrigin'] = False
        self.live['cleaningCoordinateReady'] = False
        result = self.payload(self.get('realtime-position'))['data']
        self.assertFalse(result['coordinateReady'])
        self.assertEqual((123, 456), (result['x'], result['y']))

    def test_realtime_exposes_only_supported_control_states(self):
        self.login()
        for state in ('IDLE', 'RUNNING', 'RETURNING', 'STOPPED', 'START_FAILED', 'COMPLETE'):
            self.live['controlState'] = state
            self.assertEqual(state, self.payload(self.get('realtime-position'))['data']['controlState'])
        self.live['controlState'] = 'FAULT'
        self.assertEqual('STOPPED', self.payload(self.get('realtime-position'))['data']['controlState'])

    def test_unfixed_position_nulls_coordinates_but_keeps_history(self):
        self.login()
        self.live['rtkFixAvailable'] = False
        self.history['rtkFixAvailable'] = False
        self.assertIsNone(self.payload(self.get('realtime-position'))['data']['x'])
        result = self.payload(self.get('position-history'))['data']
        self.assertEqual(self.history['points'], result['points'])
        self.assertFalse(result['rtkFixAvailable'])

    def test_no_origin_nulls_coordinates_even_if_at_origin(self):
        self.login()
        self.live.update({'x': 0, 'y': 0, 'coordinateReady': False})
        result = self.payload(self.get('realtime-position'))['data']
        self.assertIsNone(result['x'])
        self.assertFalse(result['coordinateReady'])

    def test_history_preserves_order_gaps_and_over_limit_warning(self):
        self.login()
        self.history['pointLimitExceeded'] = True
        result = self.payload(self.get('position-history'))['data']
        self.assertEqual(self.history['points'], result['points'])
        self.assertTrue(result['simplified'])
        self.assertTrue(result['pointLimitExceeded'])
        self.assertNotIn('areaNumber', result)
        self.assertEqual([], self.calls)

    def test_new_interfaces_do_not_change_original_modeling_contract(self):
        self.login()
        for path, extra in (
                ('realtime-position', {'x': None, 'y': None, 'heading': None}),
                ('position-history', {'points': []})):
            response = self.client.get('/api/t-railcar/' + path + '/250006')
            expected = {'coordinateReady': False, 'rtkFixAvailable': False}
            expected.update(extra)
            self.assertEqual({'success': True, 'data': expected}, self.payload(response))

    def test_cleaning_log_list_and_detail_contract(self):
        self.login()
        response = self.client.get(
            '/api/t-railcar/cleaning-logs?productId=250006&page=1&pageSize=20'
        )
        self.assertEqual(200, response.status_code)
        payload = self.payload(response)
        self.assertEqual(200, payload['code'])
        self.assertEqual('success', payload['message'])
        self.assertEqual(1, payload['data']['total'])
        self.assertIsNone(payload['data']['list'][0]['distanceMeters'])
        self.assertNotIn('trajectory', payload['data']['list'][0])

        detail = self.client.get('/api/t-railcar/cleaning-logs/log_20260916_001')
        self.assertEqual(200, detail.status_code)
        detail_data = self.payload(detail)['data']
        self.assertEqual('LOCAL_XY', detail_data['trajectory']['coordinateSystem'])
        self.assertIn('plannedRoute', detail_data)

    def test_cleaning_log_endpoints_validate_auth_device_and_paging(self):
        unauthenticated = Flask('unauthenticated-cleaning-logs')
        register_lan_cloud_compat_routes(
            unauthenticated, lambda: None,
            device_identity_provider=lambda: {'productId': '250006'},
        )
        self.assertEqual(
            401,
            unauthenticated.test_client().get(
                '/api/t-railcar/cleaning-logs?productId=250006'
            ).status_code,
        )

        self.login()
        self.assertEqual(404, self.client.get(
            '/api/t-railcar/cleaning-logs?productId=999999'
        ).status_code)
        self.assertEqual(400, self.client.get(
            '/api/t-railcar/cleaning-logs?productId=250006&page=0'
        ).status_code)
        self.assertEqual(400, self.client.get(
            '/api/t-railcar/cleaning-logs?productId=250006&pageSize=101'
        ).status_code)
        self.assertEqual(404, self.client.get(
            '/api/t-railcar/cleaning-logs/log_missing'
        ).status_code)


if __name__ == '__main__':
    unittest.main()
