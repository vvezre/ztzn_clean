# coding=utf-8
"""Cloud-compatible HTTP facade for direct LAN access to the robot.

The miniapp normally sends commands to the Java cloud service.  On a local
Wi-Fi connection it can send the same request paths and JSON bodies to the
robot's Flask service instead.  This module deliberately contains no route
planning or motion logic: every command is delegated to the same
``MQTTCommandHandler`` that processes cloud MQTT commands today.

The facade is Python 2.7 compatible because the deployed Jetson runtime still
uses Python 2 for the cleaner service.
"""

from __future__ import absolute_import

import datetime
import json
import os
import threading
import time
import uuid
from collections import OrderedDict

try:
    import Queue as queue_module
except ImportError:
    import queue as queue_module

from flask import g, jsonify, request

from lan_local_auth import (
    LocalAuthManager,
    auth_error_response,
    register_lan_auth_routes,
)


DEVICE_MODEL = '-T01'
DEVICE_TYPE = 'T_PYTHON'
COMPANY_CODE = 'ZTZN-PVC'
DEFAULT_TIMEOUT_MS = 30000


try:
    text_type = unicode
except NameError:
    text_type = str


def _text(value):
    if value is None:
        return ''
    if isinstance(value, text_type):
        return value.strip()
    try:
        return value.decode('utf-8').strip()
    except Exception:
        return text_type(value).strip()


def _now_ms():
    return int(time.time() * 1000)


def _new_id(prefix):
    return '{}_{}'.format(prefix, uuid.uuid4().hex[:12])


def _device_id(product_id):
    return '{}{}'.format(DEVICE_MODEL, product_id)


def _mqtt_topic(product_id):
    return 'RAILCAR/S/{}'.format(_device_id(product_id))


def _valid_product_id(product_id):
    return len(product_id) == 6 and product_id.isdigit()


def _parse_optional_bool(value, default=None):
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    normalized = _text(value).lower()
    if normalized in ('true', '1'):
        return True
    if normalized in ('false', '0'):
        return False
    return None


def _result_data(result):
    if not isinstance(result, dict):
        return None
    data = result.get('data')
    return data if isinstance(data, dict) else None


def _load_local_device_identity():
    """Read the robot identity already used by MQTT; never duplicate it."""
    config_path = os.environ.get('CLEANBOT_MQTT_CONFIG') or os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'mqtt_config.json',
    )
    try:
        with open(config_path, 'rb') as handle:
            config = json.loads(handle.read().decode('utf-8-sig'))
        mqtt_config = config.get('mqtt') if isinstance(config, dict) else {}
        product_id = _text(mqtt_config.get('product_id'))
        product_model = _text(mqtt_config.get('product_model')) or DEVICE_MODEL
        company_code = _text(mqtt_config.get('company_code')) or COMPANY_CODE
        if not _valid_product_id(product_id):
            raise ValueError('invalid product_id')
        return {
            'productId': product_id,
            'productType': product_model,
            'productModel': product_model,
            'companyCode': company_code,
            'serialNumber': '{}{}'.format(product_model, product_id),
            'deviceId': '{}{}'.format(product_model, product_id),
            'id': 1,
        }
    except Exception:
        return None


class LocalCommandStatusStore(object):
    """Bounded in-memory command status store matching the cloud snapshot."""

    def __init__(self, max_entries=500):
        self.max_entries = max(10, int(max_entries))
        self._items = OrderedDict()
        self._latest_by_device = {}
        self._lock = threading.RLock()

    def create(self, command_id, trace_id, product_id, action, params):
        now = _now_ms()
        device_id = _device_id(product_id)
        detail = {
            'deviceId': device_id,
            'deviceType': DEVICE_TYPE,
            'action': action,
            'mqttTopic': _mqtt_topic(product_id),
            'traceId': trace_id,
            'status': 'DISPATCHED',
        }
        if params:
            detail['params'] = params
        snapshot = {
            'exists': True,
            'commandId': command_id,
            'traceId': trace_id,
            'deviceId': device_id,
            'deviceType': DEVICE_TYPE,
            'action': action,
            'status': 'DISPATCHED',
            'message': 'T型号小车控制命令发送成功',
            'operator': 'lan',
            'timeoutMs': DEFAULT_TIMEOUT_MS,
            'createdAt': now,
            'updatedAt': now,
            'terminal': False,
            'detail': detail,
        }
        with self._lock:
            self._items[command_id] = snapshot
            self._latest_by_device[device_id] = command_id
            self._trim()
        return snapshot

    def complete(self, command_id, result):
        if not isinstance(result, dict):
            result = {
                'success': False,
                'message': 'robot command response is invalid',
            }
        succeeded = bool(result.get('success'))
        with self._lock:
            snapshot = self._items.get(command_id)
            if snapshot is None:
                return self.missing(command_id)
            snapshot['status'] = 'SUCCEEDED' if succeeded else 'FAILED'
            snapshot['message'] = (
                result.get('message') or
                ('命令执行成功' if succeeded else '命令执行失败')
            )
            snapshot['terminal'] = True
            snapshot['updatedAt'] = _now_ms()
            detail = dict(snapshot.get('detail') or {})
            detail['status'] = snapshot['status']
            detail['result'] = result
            snapshot['detail'] = detail
            # Python 2.7 OrderedDict has no move_to_end on some deployed
            # distributions, so refresh insertion order explicitly.
            self._items.pop(command_id, None)
            self._items[command_id] = snapshot
            return dict(snapshot)

    def get(self, command_id):
        with self._lock:
            snapshot = self._items.get(command_id)
            if snapshot is None:
                return self.missing(command_id)
            return dict(snapshot)

    def latest(self, device_id):
        with self._lock:
            command_id = self._latest_by_device.get(device_id)
            if not command_id:
                return self.missing(None)
            snapshot = self._items.get(command_id)
            return dict(snapshot) if snapshot else self.missing(None)

    def missing(self, command_id):
        return {
            'exists': False,
            'commandId': command_id,
            'status': 'UNKNOWN',
            'message': '命令状态不存在',
            'terminal': False,
            'detail': {},
        }

    def _trim(self):
        while len(self._items) > self.max_entries:
            old_command_id, old_snapshot = self._items.popitem(last=False)
            device_id = old_snapshot.get('deviceId')
            if self._latest_by_device.get(device_id) == old_command_id:
                self._latest_by_device.pop(device_id, None)


class LanCloudCompatibility(object):
    """Execute cloud-style HTTP operations through the local command handler."""

    def __init__(self, command_handler_provider, status_store=None):
        self.command_handler_provider = command_handler_provider
        self.status_store = status_store or LocalCommandStatusStore()
        # Cloud MQTT delivers commands on one callback thread.  Preserve that
        # ordering on LAN too, otherwise hold_start/keepalive/hold_stop could
        # race when several HTTP requests arrive close together.
        self._dispatch_queue = queue_module.Queue()
        self._dispatch_worker = threading.Thread(target=self._dispatch_loop)
        self._dispatch_worker.daemon = True
        self._dispatch_worker.start()

    def _handler(self):
        handler = self.command_handler_provider()
        if handler is None or not callable(getattr(handler, 'handle', None)):
            raise RuntimeError('local command handler is not ready')
        return handler

    def _prepare(self, product_id, command, params, command_id, trace_id):
        product_id = _text(product_id)
        command = _text(command)
        params = params if isinstance(params, dict) else {}
        command_id = _text(command_id) or _new_id('cmd')
        trace_id = _text(trace_id) or _new_id('trace')
        self.status_store.create(command_id, trace_id, product_id, command, params)
        return command_id, trace_id, {
            'company_code': COMPANY_CODE,
            'product_model': DEVICE_MODEL,
            'product_id': product_id,
            'timestamp': int(time.time()),
            'command_id': command_id,
            'trace_id': trace_id,
            'command': command,
            'params': params,
        }

    def _execute_prepared(self, command_id, message):
        try:
            result = self._handler().handle(message)
        except Exception as exc:
            result = {
                'success': False,
                'message': '命令处理异常: {}'.format(exc),
            }
        snapshot = self.status_store.complete(command_id, result)
        return result, snapshot

    def execute(self, product_id, command, params=None, command_id=None, trace_id=None):
        command_id, trace_id, message = self._prepare(
            product_id, command, params, command_id, trace_id
        )
        result, snapshot = self._execute_prepared(command_id, message)
        return command_id, trace_id, result, snapshot

    def dispatch(self, product_id, command, params=None, command_id=None, trace_id=None):
        """Start local handling in the background, like cloud MQTT dispatch."""
        command_id, trace_id, message = self._prepare(
            product_id, command, params, command_id, trace_id
        )
        self._dispatch_queue.put((command_id, message))
        return command_id, trace_id

    def dispatch_and_wait(self, product_id, command, params=None, timeout_seconds=30.0):
        """Queue a command in order and wait for its local terminal result."""
        command_id, trace_id = self.dispatch(product_id, command, params)
        deadline = time.time() + max(0.0, float(timeout_seconds))
        while time.time() <= deadline:
            snapshot = self.status_store.get(command_id)
            if snapshot.get('terminal'):
                detail = snapshot.get('detail') or {}
                result = detail.get('result')
                if not isinstance(result, dict):
                    result = {
                        'success': False,
                        'message': 'robot command response is invalid',
                    }
                return command_id, trace_id, result, snapshot
            time.sleep(0.01)
        return command_id, trace_id, {
            'success': False,
            'message': 'robot response timeout',
        }, self.status_store.get(command_id)

    def _dispatch_loop(self):
        while True:
            command_id, message = self._dispatch_queue.get()
            try:
                self._execute_prepared(command_id, message)
            finally:
                self._dispatch_queue.task_done()

    def execute_direct(self, product_id, command, params=None):
        """Run a read-only helper without changing the latest command status."""
        product_id = _text(product_id)
        command = _text(command)
        params = params if isinstance(params, dict) else {}
        message = {
            'company_code': COMPANY_CODE,
            'product_model': DEVICE_MODEL,
            'product_id': product_id,
            'timestamp': int(time.time()),
            'command_id': _new_id('local_query'),
            'trace_id': _new_id('trace'),
            'command': command,
            'params': params,
        }
        try:
            result = self._handler().handle(message)
        except Exception as exc:
            return {
                'success': False,
                'message': '命令处理异常: {}'.format(exc),
            }
        return result if isinstance(result, dict) else {
            'success': False,
            'message': 'robot command response is invalid',
        }

    def dispatch_response(self, product_id, command, command_id, trace_id):
        return {
            'success': True,
            'message': 'T型号小车控制命令发送成功',
            'deviceId': _device_id(product_id),
            'command': command,
            'mqttTopic': _mqtt_topic(product_id),
            'commandId': command_id,
            'traceId': trace_id,
            'commandStatus': 'DISPATCHED',
            'timestamp': datetime.datetime.now().isoformat(),
            'operationId': None,
        }


def register_lan_cloud_compat_routes(
        app, command_handler_provider, status_store=None, auth_manager=None,
        device_identity_provider=None):
    """Register cloud-compatible LAN routes on an existing Flask app."""

    bridge = LanCloudCompatibility(command_handler_provider, status_store=status_store)
    auth_manager = auth_manager or LocalAuthManager()
    device_identity_provider = device_identity_provider or _load_local_device_identity
    register_lan_auth_routes(app, auth_manager)

    @app.before_request
    def lan_cloud_auth_guard():
        # Only protect the cloud-compatible facade.  Existing /vehicle/* and
        # /modeling/* robot-internal routes keep their established behaviour.
        endpoint = _text(request.endpoint)
        if request.method == 'OPTIONS' or not endpoint.startswith('lan_cloud_'):
            return None
        claims, code, message = auth_manager.verify_authorization(
            request.headers.get('Authorization'),
            'access',
        )
        if claims is None:
            return auth_error_response(code, message)
        g.lan_auth_claims = claims
        return None

    def error_response(message, status_code, data=None):
        return jsonify({
            'success': False,
            'message': message,
            'data': data,
        }), status_code

    def validate_product(product_id):
        product_id = _text(product_id)
        return product_id if _valid_product_id(product_id) else None

    def result_response(code, message, data=None, http_status=200):
        payload = {
            'code': int(code),
            'message': message,
            'timestamp': datetime.datetime.now().isoformat(),
        }
        if data is not None:
            payload['data'] = data
        return jsonify(payload), http_status

    def local_identity():
        identity = device_identity_provider()
        if not isinstance(identity, dict):
            return None
        product_id = validate_product(identity.get('productId'))
        if not product_id:
            return None
        product_model = _text(identity.get('productType')) or DEVICE_MODEL
        normalized = dict(identity)
        normalized.update({
            'id': int(identity.get('id') or 1),
            'productId': product_id,
            'productType': product_model,
            'productModel': _text(identity.get('productModel')) or product_model,
            'companyCode': _text(identity.get('companyCode')) or COMPANY_CODE,
            'serialNumber': _text(identity.get('serialNumber')) or '{}{}'.format(product_model, product_id),
            'deviceId': _text(identity.get('deviceId')) or '{}{}'.format(product_model, product_id),
        })
        return normalized

    def run_command(product_id, command, params=None):
        return bridge.dispatch_and_wait(product_id, command, params or {})

    def terminal_data(product_id, command, params=None):
        command_id, trace_id, result, snapshot = run_command(product_id, command, params)
        if not result.get('success'):
            return None, error_response(result.get('message') or 'robot command failed', 502)
        data = _result_data(result)
        if data is None:
            data = {}
        return data, None

    @app.route('/api/t-railcar/command', methods=['POST'])
    def lan_cloud_send_command():
        payload = request.get_json(silent=True)
        if not isinstance(payload, dict):
            return error_response('request body must be a JSON object', 400)
        product_id = validate_product(payload.get('productId'))
        command = _text(payload.get('command'))
        params = payload.get('params')
        if not product_id:
            return error_response('产品ID不能为空或格式错误', 400)
        if not command:
            return error_response('command不能为空', 400)
        if params is not None and not isinstance(params, dict):
            return error_response('params必须是JSON对象', 400)
        command_id, trace_id = bridge.dispatch(
            product_id,
            command,
            params or {},
            command_id=payload.get('commandId'),
            trace_id=payload.get('traceId'),
        )
        return jsonify(bridge.dispatch_response(product_id, command, command_id, trace_id))

    @app.route('/api/t-railcar/move/<string:action>', methods=['POST'])
    def lan_cloud_quick_move(action):
        product_id = validate_product(request.args.get('productId'))
        if not product_id:
            return error_response('productId is required', 400)
        command_id, trace_id = bridge.dispatch(product_id, action)
        return jsonify(bridge.dispatch_response(product_id, action, command_id, trace_id))

    @app.route('/api/t-railcar/speed', methods=['POST'])
    def lan_cloud_speed():
        payload = request.get_json(silent=True) or {}
        product_id = validate_product(payload.get('productId'))
        if not product_id:
            return error_response('productId is required', 400)
        command_id, trace_id = bridge.dispatch(product_id, 'speed', payload)
        return jsonify(bridge.dispatch_response(product_id, 'speed', command_id, trace_id))

    @app.route('/api/t-railcar/advanced/<string:function_name>', methods=['POST'])
    def lan_cloud_advanced(function_name):
        product_id = validate_product(request.args.get('productId'))
        if not product_id:
            return error_response('productId is required', 400)
        command_id, trace_id = bridge.dispatch(product_id, function_name)
        return jsonify(bridge.dispatch_response(product_id, function_name, command_id, trace_id))

    @app.route('/api/command-status/device/<string:device_id>/latest', methods=['GET'])
    def lan_cloud_latest_command_status(device_id):
        return jsonify(bridge.status_store.latest(device_id))

    @app.route('/api/command-status/<string:command_id>', methods=['GET'])
    def lan_cloud_command_status(command_id):
        return jsonify(bridge.status_store.get(command_id))

    @app.route('/api/t-railcar/task-path/<string:product_id>', methods=['GET'])
    def lan_cloud_task_path(product_id):
        product_id = validate_product(product_id)
        if not product_id:
            return error_response('invalid productId', 400)
        data, error = terminal_data(product_id, 'get_task_path')
        if error:
            return error
        return jsonify({'success': True, 'message': '获取路径成功', 'data': data})

    @app.route('/api/t-railcar/modeling-path/<string:product_id>', methods=['GET'])
    def lan_cloud_modeling_path(product_id):
        product_id = validate_product(product_id)
        model_id = _text(request.args.get('modelId'))
        if not product_id or not model_id:
            return error_response('valid productId and modelId are required', 400)
        data, error = terminal_data(product_id, 'get_modeling_path', {'modelId': model_id})
        if error:
            return error
        return jsonify({'success': True, 'message': 'modeling path fetched', 'data': data})

    @app.route('/api/t-railcar/modeling-result/<string:product_id>', methods=['GET'])
    def lan_cloud_modeling_result(product_id):
        product_id = validate_product(product_id)
        if not product_id:
            return error_response('invalid productId', 400)
        data, error = terminal_data(product_id, 'get_modeling_result')
        if error:
            return error
        return jsonify({'success': True, 'message': '规划成功', 'data': data})

    def modeling_points_response(product_id, command):
        product_id = validate_product(product_id)
        if not product_id:
            return jsonify({'message': 'invalid productId'}), 400
        data, error = terminal_data(product_id, command)
        if error:
            return error
        points = data.get('points')
        return jsonify({'points': points if isinstance(points, list) else []})

    @app.route('/api/t-railcar/modeling-points/<string:product_id>', methods=['GET'])
    def lan_cloud_modeling_points(product_id):
        return modeling_points_response(product_id, 'get_modeling_points')

    @app.route('/api/t-railcar/modeling-link-points/<string:product_id>', methods=['GET'])
    def lan_cloud_modeling_link_points(product_id):
        return modeling_points_response(product_id, 'get_modeling_link_points')

    @app.route('/api/t-railcar/modeling-task/save', methods=['POST'])
    def lan_cloud_save_modeling_task():
        payload = request.get_json(silent=True) or {}
        product_id = validate_product(payload.get('productId'))
        task_name = _text(payload.get('taskName'))
        if not product_id or not task_name:
            return error_response('productId and taskName are required', 400)
        data, error = terminal_data(product_id, 'save_modeling_task', {'taskName': task_name})
        if error:
            return error
        response_data = {
            'taskName': data.get('taskName'),
            'taskCount': data.get('taskCount'),
        }
        if data.get('modelId') is not None:
            response_data['modelId'] = data.get('modelId')
        return jsonify({'success': True, 'message': '路线保存成功', 'data': response_data})

    @app.route('/api/t-railcar/tasks/<string:product_id>', methods=['GET'])
    def lan_cloud_tasks(product_id):
        product_id = validate_product(product_id)
        if not product_id:
            return error_response('invalid productId', 400)
        data, error = terminal_data(product_id, 'get_task_names')
        if error:
            return error
        return jsonify({
            'success': True,
            'message': '获取路线列表成功',
            'data': {
                'taskNames': data.get('taskNames') or [],
                'currentTaskName': data.get('currentTaskName'),
            },
        })

    @app.route('/api/t-railcar/saved-routes/<string:product_id>', methods=['GET'])
    def lan_cloud_saved_routes(product_id):
        product_id = validate_product(product_id)
        if not product_id:
            return error_response('invalid productId', 400)
        data, error = terminal_data(product_id, 'get_saved_routes')
        if error:
            return error
        routes = data.get('routes')
        if not isinstance(routes, list):
            return error_response('robot saved routes response is invalid', 502)
        return jsonify({
            'success': True,
            'message': '获取已保存路线成功',
            'data': {
                'productId': product_id,
                'serialNumber': _device_id(product_id),
                'currentTaskName': data.get('currentTaskName'),
                'currentReturnToOrigin': data.get('currentReturnToOrigin'),
                'routes': routes,
            },
        })

    @app.route('/api/t-railcar/tasks/current', methods=['POST'])
    def lan_cloud_select_task():
        payload = request.get_json(silent=True) or {}
        product_id = validate_product(payload.get('productId'))
        task_name = _text(payload.get('taskName'))
        return_value = payload.get('returnToOrigin')
        return_to_origin = _parse_optional_bool(return_value, True)
        if not product_id or not task_name:
            return error_response('productId and taskName are required', 400)
        if return_value is not None and return_to_origin is None:
            return error_response('returnToOrigin must be true or false', 400)
        data, error = terminal_data(product_id, 'set_current_task', {
            'taskName': task_name,
            'returnToOrigin': return_to_origin,
        })
        if error:
            return error
        response_data = {
            'taskName': task_name,
            'returnToOrigin': return_to_origin,
        }
        if data.get('taskCount') is not None:
            response_data['taskCount'] = data.get('taskCount')
        return jsonify({'success': True, 'message': '路线选择成功', 'data': response_data})

    @app.route('/api/t-railcar/tasks/generate', methods=['POST'])
    def lan_cloud_generate_task():
        payload = request.get_json(silent=True) or {}
        product_id = validate_product(payload.get('productId'))
        task_name = _text(payload.get('taskName'))
        area_list = payload.get('areaList')
        if not product_id or not task_name:
            return error_response('产品ID和任务名称不能为空', 400)
        if not isinstance(area_list, list):
            return error_response('areaList不能为空，且必须是数组', 400)
        command_id, trace_id = bridge.dispatch(product_id, 'create_task', {
            'taskName': task_name,
            'areaList': area_list,
        })
        response_data = {
            'taskName': task_name,
            'areaCount': len(area_list),
            'layoutV2AreaCount': len([
                area for area in area_list
                if isinstance(area, dict) and _text(area.get('layoutVersion')) == '2'
            ]),
            'commandId': command_id,
            'traceId': trace_id,
            'commandStatus': 'DISPATCHED',
        }
        return jsonify({'success': True, 'message': '任务生成命令已发送', 'data': response_data})

    def get_status_payload(device_id):
        product_id = device_id[-6:] if len(device_id) >= 6 else ''
        if not _valid_product_id(product_id):
            return None
        result = bridge.execute_direct(product_id, 'get_status')
        if not result.get('success'):
            return None
        return _result_data(result) or {}

    def legacy_device_status(device_id):
        status_data = get_status_payload(device_id)
        if status_data is None:
            return {
                'deviceId': device_id,
                'exists': False,
            }
        control_state = _text(status_data.get('control_state')).upper()
        mission_state = _text(status_data.get('mission_state')).upper()
        online_state = _text(status_data.get('online_state')).upper()
        if online_state != 'ONLINE':
            status = 'offline'
        elif mission_state == 'CHARGING':
            status = 'charging'
        elif mission_state == 'RUNNING' or control_state == 'RUNNING':
            status = 'running'
        else:
            status = 'idle'
        return {
            'deviceId': device_id,
            'exists': True,
            'battery': status_data.get('battery'),
            'status': status,
            'operationMode': status_data.get('control_state'),
            'location': {
                'lon': status_data.get('lon'),
                'lat': status_data.get('lat'),
            },
            'lastUpdateTime': int(status_data.get('timestamp') or time.time()) * 1000,
        }

    def shadow_device_status(device_id):
        status_data = get_status_payload(device_id)
        if status_data is None:
            return {'exists': False, 'deviceId': device_id}
        product_id = device_id[-6:]
        return {
            'exists': True,
            'deviceId': device_id,
            'serialNumber': device_id,
            'deviceType': DEVICE_TYPE,
            'productType': DEVICE_MODEL,
            'productId': product_id,
            'companyCode': COMPANY_CODE,
            'onlineState': status_data.get('online_state'),
            'missionState': status_data.get('mission_state'),
            'controlState': status_data.get('control_state'),
            'healthState': status_data.get('health_state'),
            'faultState': status_data.get('fault_state'),
            'rawStatus': status_data.get('status'),
            'battery': status_data.get('battery'),
            'voltage': status_data.get('voltage'),
            'angle': status_data.get('heading'),
            'updatedAt': int(status_data.get('timestamp') or time.time()) * 1000,
            'location': {
                'lon': status_data.get('lon'),
                'lat': status_data.get('lat'),
            },
            'currentLocation': {
                'lon': status_data.get('lon'),
                'lat': status_data.get('lat'),
                'heading': status_data.get('heading'),
            },
            'distanceToTaskOriginM': status_data.get('distanceToTaskOriginM'),
            'taskOriginToleranceM': status_data.get('taskOriginToleranceM'),
            'isAtTaskOrigin': status_data.get('isAtTaskOrigin'),
            'supportedActions': status_data.get('supported_actions') or [],
            'supportedParams': status_data.get('supported_params') or [],
            'supportedStatusFields': status_data.get('supported_status_fields') or [],
            'detail': status_data,
        }

    def local_mini_app_device():
        identity = local_identity()
        if identity is None:
            return None
        shadow = shadow_device_status(identity.get('deviceId'))
        detail = shadow.get('detail') if isinstance(shadow.get('detail'), dict) else {}
        online = shadow.get('onlineState') == 'ONLINE'
        mission_state = _text(shadow.get('missionState')).upper()
        control_state = _text(shadow.get('controlState')).upper()
        if not online:
            status = 'offline'
        elif mission_state == 'CHARGING':
            status = 'charging'
        elif mission_state == 'RUNNING' or control_state == 'RUNNING':
            status = 'running'
        else:
            status = 'idle'
        return {
            'id': identity.get('id'),
            'serialNumber': identity.get('serialNumber'),
            'name': '清扫机器人{}'.format(identity.get('productId')),
            'companyCode': identity.get('companyCode'),
            'productType': identity.get('productType'),
            'productModel': identity.get('productModel'),
            'productId': identity.get('productId'),
            'deviceId': identity.get('deviceId'),
            'vehicleType': 'railcar',
            'deviceType': DEVICE_TYPE,
            'online': online,
            'status': status,
            'onlineState': shadow.get('onlineState'),
            'missionState': shadow.get('missionState'),
            'controlState': shadow.get('controlState'),
            'healthState': shadow.get('healthState'),
            'faultState': shadow.get('faultState'),
            'battery': shadow.get('battery'),
            'updatedAt': shadow.get('updatedAt'),
            'voltage': shadow.get('voltage'),
            'angle': shadow.get('angle'),
            'lat': (shadow.get('currentLocation') or {}).get('lat'),
            'lon': (shadow.get('currentLocation') or {}).get('lon'),
            'heading': (shadow.get('currentLocation') or {}).get('heading'),
            'taskName': detail.get('task_name') or detail.get('taskName'),
            'curTaskIndex': detail.get('cur_task_index') or detail.get('curTaskIndex'),
            'taskCount': detail.get('clean_task_count') or detail.get('cleanTaskCount'),
            'timestamp': detail.get('timestamp'),
            'taskOrigin': detail.get('taskOrigin'),
            'currentLocation': shadow.get('currentLocation'),
            'distanceToTaskOriginM': shadow.get('distanceToTaskOriginM'),
            'taskOriginToleranceM': shadow.get('taskOriginToleranceM'),
            'isAtTaskOrigin': shadow.get('isAtTaskOrigin'),
            'supportedActions': shadow.get('supportedActions') or [],
            'supportedParams': shadow.get('supportedParams') or [],
            'supportedStatusFields': shadow.get('supportedStatusFields') or [],
            'shadowDetail': detail,
        }

    def local_device_info():
        mini_device = local_mini_app_device()
        if mini_device is None:
            return None
        claims = getattr(g, 'lan_auth_claims', {}) or {}
        return {
            'deviceId': mini_device.get('id'),
            'companyCode': mini_device.get('companyCode'),
            'productType': mini_device.get('productType'),
            'productId': mini_device.get('productId'),
            'serialNumber': mini_device.get('serialNumber'),
            'status': mini_device.get('status'),
            'bound': True,
            'boundUsername': claims.get('sub'),
            'userId': claims.get('userId'),
        }

    @app.route('/api/mini-app/devices', methods=['GET'])
    def lan_cloud_mini_app_devices():
        device = local_mini_app_device()
        if device is None:
            return result_response(500, '本机设备身份配置无效', None, 500)
        return result_response(200, '成功', [device])

    @app.route('/api/mini-app/devices/<int:device_id>', methods=['GET'])
    def lan_cloud_mini_app_device_detail(device_id):
        device = local_mini_app_device()
        if device is None or device_id != device.get('id'):
            return result_response(404, '设备不存在')
        return result_response(200, '成功', device)

    @app.route('/api/mini-app/devices/<int:device_id>/shadow', methods=['GET'])
    def lan_cloud_mini_app_device_shadow(device_id):
        identity = local_identity()
        if identity is None or device_id != identity.get('id'):
            return result_response(404, '设备不存在')
        return result_response(200, '成功', shadow_device_status(identity.get('deviceId')))

    @app.route('/device/my-devices', methods=['GET'])
    def lan_cloud_my_devices():
        device = local_device_info()
        if device is None:
            return result_response(500, '本机设备身份配置无效', None, 500)
        return result_response(200, '成功', [device])

    @app.route('/device/info/<string:product_id>', methods=['GET'])
    def lan_cloud_device_info(product_id):
        device = local_device_info()
        if device is None or product_id != device.get('productId'):
            return result_response(404, '设备不存在或未绑定')
        return result_response(200, '成功', device)

    @app.route('/device/scan', methods=['POST'])
    def lan_cloud_scan_device():
        payload = request.get_json(silent=True) or {}
        device = local_device_info()
        if device is None:
            return result_response(500, '本机设备身份配置无效', None, 500)
        requested = (
            _text(payload.get('companyCode')),
            _text(payload.get('productType')),
            _text(payload.get('productId')),
        )
        actual = (
            device.get('companyCode'),
            device.get('productType'),
            device.get('productId'),
        )
        if requested != actual:
            return result_response(404, '二维码不是当前局域网小车')
        return result_response(200, '成功', device)

    @app.route('/api/device-status/<string:device_id>/shadow', methods=['GET'])
    def lan_cloud_device_shadow(device_id):
        return jsonify(shadow_device_status(device_id))

    @app.route('/api/device-status/<string:device_id>', methods=['GET'])
    def lan_cloud_device_status(device_id):
        return jsonify(legacy_device_status(device_id))

    @app.route('/api/device-status/batch', methods=['POST'])
    def lan_cloud_device_status_batch():
        payload = request.get_json(silent=True) or {}
        device_ids = payload.get('deviceIds') or []
        return jsonify(dict((item, legacy_device_status(item)) for item in device_ids))

    @app.route('/api/device-status/shadow/batch', methods=['POST'])
    def lan_cloud_device_shadow_batch():
        payload = request.get_json(silent=True) or {}
        device_ids = payload.get('deviceIds') or []
        return jsonify(dict((item, shadow_device_status(item)) for item in device_ids))

    bridge.auth_manager = auth_manager
    return bridge
