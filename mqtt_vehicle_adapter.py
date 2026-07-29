# coding=utf-8
"""
?????????? MQTT ??????? Flask HTTP ???
????? requests??????? Python 2 ???????????
"""
import json
import os

try:
    from urllib import quote, urlencode
    from urllib2 import Request, URLError, urlopen
except ImportError:
    from urllib.error import URLError
    from urllib.parse import quote, urlencode
    from urllib.request import Request, urlopen

from AppLogger import logger
from modeling_frontend import (
    frontend_area_points,
    frontend_link_points,
    frontend_path_points,
)

# Keep the original helper names for the local-only modeling simulator.
_frontend_area_points = frontend_area_points
_frontend_link_points = frontend_link_points
_frontend_path_points = frontend_path_points


class VehicleControllerAdapter(object):
    def __init__(self, base_url=None, timeout=10):
        self.base_url = (base_url or os.environ.get('CLEANER_HTTP_BASE_URL') or 'http://127.0.0.1:7899').rstrip('/')
        self.timeout = timeout
        logger.info('MQTT????????????base_url={}'.format(self.base_url))

    def _decode_body(self, body):
        if body is None:
            return None
        if isinstance(body, bytes):
            try:
                body = body.decode('utf-8')
            except Exception:
                body = body.decode('utf-8', 'replace')
        return body

    def _parse_response(self, response):
        body = self._decode_body(response.read())
        if body is None:
            return None
        try:
            return json.loads(body)
        except Exception:
            return body

    def _call(self, path, params=None, json_data=None):
        url = self.base_url + path
        if params:
            url = url + '?' + urlencode(params)
        headers = {}
        payload = None
        if json_data is not None:
            payload = json.dumps(json_data).encode('utf-8')
            headers['Content-Type'] = 'application/json'
        logger.warning('MQTT?????????: {} params={} json={}'.format(url, params, json_data))
        try:
            request = Request(url, data=payload, headers=headers)
            response = urlopen(request, timeout=self.timeout)
            return self._parse_response(response)
        except URLError as exc:
            raise RuntimeError('MQTT adapter request failed: {}'.format(exc))

    def drive(self, distance=0, speed=None):
        if speed is not None:
            self.adjust_speed(speed)
        if distance:
            resolved_speed = int(speed) if speed is not None else 100
            return self._call('/vehicle/moveDistance/{}/{}'.format(int(distance), resolved_speed))
        return self._call('/vehicle/drive')

    def back(self, distance=0, speed=None):
        if distance:
            resolved_speed = int(speed) if speed is not None else 100
            return self._call('/vehicle/moveBackDistance/{}/{}'.format(int(distance), resolved_speed))
        return self._call('/vehicle/back')

    def turn_left(self, angle=90):
        if angle == 90:
            return self._call('/vehicle/turnLeft90')
        return self._call('/vehicle/turnLeft')

    def turn_right(self, angle=90):
        if angle == 90:
            return self._call('/vehicle/turnRight90')
        if angle == 180:
            return self._call('/vehicle/turnRight180')
        return self._call('/vehicle/turnRight')

    def stop(self):
        return self._call('/vehicle/parking')

    def parking(self):
        return self._call('/vehicle/parking')

    def joystick_move(self, distance, dir_x, dir_y):
        return self._call('/vehicle/joystickMove/{}/{}/{}'.format(distance, dir_x, dir_y))

    def auto_drive(self):
        return self._call('/vehicle/autoDrive')

    def go_on(self):
        return self._call('/vehicle/goOn')

    def resume_after_battery_swap(self):
        return self._call('/vehicle/resumeAfterBatterySwap')

    def get_battery_swap_status(self):
        return self._call('/vehicle/getBatterySwapStatus')

    def clear_battery_swap_checkpoint(self):
        return self._call('/vehicle/clearBatterySwapCheckpoint')

    def return_to_point(self):
        return self._call('/vehicle/returnToPoint')

    def enter_garage(self):
        return self._call('/vehicle/enterGarage')

    def exit_garage(self):
        return self._call('/vehicle/exitGarage')

    def adjust_speed(self, speed):
        return self._call('/vehicle/adjustSpeed/{}'.format(int(speed)))

    def adjust_brush_speed(self, speed):
        return self._call('/vehicle/adjustBrushSpeed/{}'.format(int(speed)))

    def toggle_tracking(self, enabled):
        tracking = '0' if enabled else '1'
        return self._call('/vehicle/toggleTracking/{}'.format(tracking))

    def toggle_path_planning(self, path):
        return self._call('/vehicle/togglePathPlanning/{}'.format(path))

    def create_task(self, params):
        return self._call('/vehicle/createTask', json_data=params or {})

    def select_task(self, task_name):
        return self._call('/vehicle/selectTaskByName', params={'taskName': task_name})

    def save_task(self, task_name):
        return self.set_current_task(task_name)

    def set_current_task(self, task_name):
        return self._call('/vehicle/setCurrentTask', params={'taskName': task_name})

    def save_modeling_task(self, task_name):
        return self._normalize_modeling_response(
            self._call(
                '/modeling/session/save-task',
                json_data={'taskName': str(task_name)},
            ),
            'modeling task saved',
        )

    def get_task_names(self):
        response = self._call('/vehicle/selectTaskName')
        if not isinstance(response, dict) or response.get('success') is False:
            return {
                'success': False,
                'message': (
                    response.get('msg') or response.get('message')
                    if isinstance(response, dict)
                    else 'task list response is invalid'
                ),
            }
        data = response.get('data') if isinstance(response.get('data'), dict) else {}
        return {
            'success': True,
            'message': 'task names fetched',
            'data': {
                'taskNames': data.get('taskNames') or [],
                'currentTaskName': data.get('currentTaskName'),
            },
        }

    def get_saved_routes(self):
        response = self._call('/vehicle/selectSavedRoutes')
        if not isinstance(response, dict) or response.get('success') is False:
            return {
                'success': False,
                'message': (
                    response.get('msg') or response.get('message')
                    if isinstance(response, dict)
                    else 'saved routes response is invalid'
                ),
            }
        data = response.get('data') if isinstance(response.get('data'), dict) else {}
        return {
            'success': True,
            'message': 'saved routes fetched',
            'data': {
                'routes': data.get('routes') or [],
                'currentTaskName': data.get('currentTaskName'),
            },
        }

    def save_params(self, params):
        return self._call('/vehicle/saveParams', json_data=params or {})

    def set_garage_entry(self, lat, lon):
        return self._call('/vehicle/setGarageEntryInfo', params={'lat': lat, 'lon': lon})

    def get_status(self):
        return self._call('/vehicle/getInfo')

    def get_task_path(self):
        return self._call('/vehicle/getTaskPath')

    def _normalize_modeling_response(self, response, default_message):
        if not isinstance(response, dict):
            return {
                'success': False,
                'message': '{} response is invalid'.format(default_message),
            }
        if response.get('success') is False:
            data = response.get('data') if isinstance(response.get('data'), dict) else {}
            if response.get('code'):
                data = dict(data)
                data['code'] = response.get('code')
            result = {
                'success': False,
                'message': response.get('msg') or response.get('message') or '{} failed'.format(default_message),
            }
            if data:
                result['data'] = data
            return result
        return {
            'success': True,
            'message': default_message,
            'data': response.get('data') or {},
        }

    def start_modeling(self, name=None, restart=False):
        payload = {'restart': bool(restart)}
        if name:
            payload['name'] = str(name)
        return self._normalize_modeling_response(
            self._call('/modeling/session/start', json_data=payload),
            'modeling started',
        )

    def get_modeling_state(self):
        return self._normalize_modeling_response(
            self._call('/modeling/session/current'),
            'modeling state fetched',
        )

    def undo_modeling_point(self, point_type=None):
        payload = {}
        if point_type:
            payload['pointType'] = str(point_type)
        return self._normalize_modeling_response(
            self._call('/modeling/session/undo', json_data=payload),
            'modeling point undone',
        )

    def delete_modeling_point(self, point_id):
        return self._normalize_modeling_response(
            self._call(
                '/modeling/session/delete-area-point',
                json_data={'id': str(point_id)},
            ),
            'modeling point deleted',
        )

    def delete_modeling_link_point(self, point_id):
        return self._normalize_modeling_response(
            self._call(
                '/modeling/session/delete-link-point',
                json_data={'id': str(point_id)},
            ),
            'modeling link point deleted',
        )

    def clear_modeling_points(self, point_type=None):
        payload = {}
        if point_type:
            payload['pointType'] = str(point_type)
        return self._normalize_modeling_response(
            self._call('/modeling/session/clear', json_data=payload),
            'modeling points cleared',
        )

    def clear_all_modeling_points(self, point_type):
        return self._normalize_modeling_response(
            self._call(
                '/modeling/session/clear-all',
                json_data={'pointType': str(point_type)},
            ),
            'all modeling points cleared',
        )

    def finish_modeling(self):
        return self._normalize_modeling_response(
            self._call('/modeling/session/finish', json_data={}),
            'modeling path generated',
        )

    def sample_modeling_point(self, model_id=None, group_id=None):
        if model_id is None and group_id is None:
            return self._normalize_modeling_response(
                self._call('/modeling/session/record-area-point', json_data={}),
                'modeling point recorded',
            )
        encoded_group_id = quote(str(group_id), safe='')
        response = self._call(
            '/modeling/groups/{}/sample-point'.format(encoded_group_id),
            json_data={'modelId': str(model_id)},
        )
        if not isinstance(response, dict):
            return {
                'success': False,
                'message': 'modeling point response is invalid',
            }
        if response.get('success') is False:
            error_data = {
                'modelId': str(model_id),
                'groupId': str(group_id),
            }
            if response.get('code'):
                error_data['code'] = response.get('code')
            return {
                'success': False,
                'message': response.get('msg') or response.get('message') or 'modeling point recording failed',
                'data': error_data,
            }

        response_data = response.get('data') or {}
        point = response_data.get('point') if isinstance(response_data, dict) else None
        if not isinstance(point, dict):
            return {
                'success': False,
                'message': 'modeling point is missing from response',
            }

        return {
            'success': True,
            'message': 'modeling point recorded',
            'data': {
                'modelId': str(model_id),
                'groupId': str(group_id),
                'point': point,
            },
        }

    def sample_modeling_link_point(self, model_id=None, link_id=None):
        if model_id is None and link_id is None:
            return self._normalize_modeling_response(
                self._call('/modeling/session/record-link-point', json_data={}),
                'modeling link point recorded',
            )
        encoded_link_id = quote(str(link_id), safe='')
        response = self._call(
            '/modeling/group-links/{}/sample-point'.format(encoded_link_id),
            json_data={'modelId': str(model_id)},
        )
        if not isinstance(response, dict):
            return {
                'success': False,
                'message': 'modeling link point response is invalid',
            }
        if response.get('success') is False:
            error_data = {
                'modelId': str(model_id),
                'linkId': str(link_id),
            }
            if response.get('code'):
                error_data['code'] = response.get('code')
            return {
                'success': False,
                'message': response.get('msg') or response.get('message') or 'modeling link point recording failed',
                'data': error_data,
            }

        response_data = response.get('data') or {}
        point = response_data.get('point') if isinstance(response_data, dict) else None
        if not isinstance(point, dict):
            return {
                'success': False,
                'message': 'modeling link point is missing from response',
            }

        return {
            'success': True,
            'message': 'modeling link point recorded',
            'data': {
                'modelId': str(model_id),
                'linkId': str(link_id),
                'point': point,
            },
        }

    def get_modeling_points(self, model_id=None):
        if model_id is None:
            current_response = self._call('/modeling/session/current')
            if not isinstance(current_response, dict):
                return {
                    'success': False,
                    'message': 'modeling state response is invalid',
                }
            if current_response.get('success') is False:
                return {
                    'success': False,
                    'message': current_response.get('msg') or current_response.get('message') or 'modeling state fetch failed',
                }
            current_data = current_response.get('data') or {}
            model_id = current_data.get('modelId') if isinstance(current_data, dict) else None
            if not model_id:
                return {
                    'success': False,
                    'message': 'modeling session is not active',
                }

        encoded_model_id = quote(str(model_id), safe='')
        response = self._call('/modeling/draft/{}'.format(encoded_model_id))
        if not isinstance(response, dict):
            return {
                'success': False,
                'message': 'modeling draft response is invalid',
            }
        if response.get('success') is False:
            return {
                'success': False,
                'message': response.get('msg') or response.get('message') or 'modeling points fetch failed',
            }

        draft = response.get('data') or {}
        if not isinstance(draft, dict):
            return {
                'success': False,
                'message': 'modeling draft data is invalid',
            }

        return {
            'success': True,
            'message': 'modeling points fetched',
            'data': {
                'points': frontend_area_points(draft),
            },
        }

    def get_modeling_link_points(self, model_id=None):
        if model_id is None:
            current_response = self._call('/modeling/session/current')
            if not isinstance(current_response, dict):
                return {
                    'success': False,
                    'message': 'modeling state response is invalid',
                }
            if current_response.get('success') is False:
                return {
                    'success': False,
                    'message': current_response.get('msg') or current_response.get('message') or 'modeling state fetch failed',
                }
            current_data = current_response.get('data') or {}
            model_id = current_data.get('modelId') if isinstance(current_data, dict) else None
            if not model_id:
                return {
                    'success': False,
                    'message': 'modeling session is not active',
                }

        encoded_model_id = quote(str(model_id), safe='')
        response = self._call('/modeling/draft/{}'.format(encoded_model_id))
        if not isinstance(response, dict):
            return {
                'success': False,
                'message': 'modeling draft response is invalid',
            }
        if response.get('success') is False:
            return {
                'success': False,
                'message': response.get('msg') or response.get('message') or 'modeling link points fetch failed',
            }

        draft = response.get('data') or {}
        if not isinstance(draft, dict):
            return {
                'success': False,
                'message': 'modeling draft data is invalid',
            }

        return {
            'success': True,
            'message': 'modeling link points fetched',
            'data': {
                'points': frontend_link_points(draft),
            },
        }

    def get_modeling_path(self, model_id=None):
        if model_id is None:
            return self._normalize_modeling_response(
                self._call('/modeling/session/path'),
                'modeling path fetched',
            )
        encoded_model_id = quote(str(model_id), safe='')
        response = self._call('/modeling/draft/{}'.format(encoded_model_id))
        if not isinstance(response, dict):
            return {
                'success': False,
                'message': 'modeling draft response is invalid',
            }
        if response.get('success') is False:
            return {
                'success': False,
                'message': response.get('msg') or response.get('message') or 'modeling path fetch failed',
            }

        draft = response.get('data') or {}
        task_plan = draft.get('taskPlan') if isinstance(draft, dict) else None
        if not isinstance(task_plan, dict) or task_plan.get('status') != 'ready':
            return {
                'success': False,
                'message': 'modeling task plan is not ready',
            }

        return {
            'success': True,
            'message': 'modeling path fetched',
            'data': {
                'modelId': draft.get('id') or str(model_id),
                'taskName': task_plan.get('taskName') or draft.get('name') or '',
                'updatedAt': draft.get('updatedAt') or task_plan.get('generatedAt'),
                'taskPreview': draft.get('taskPreview'),
                'taskPlan': task_plan,
            },
        }

    def get_modeling_result(self, model_id=None):
        if model_id is None:
            current_response = self._call('/modeling/session/current')
            if not isinstance(current_response, dict):
                return {
                    'success': False,
                    'message': 'modeling state response is invalid',
                }
            if current_response.get('success') is False:
                return {
                    'success': False,
                    'message': current_response.get('msg') or current_response.get('message') or 'modeling state fetch failed',
                }
            current_data = current_response.get('data') or {}
            model_id = current_data.get('modelId') if isinstance(current_data, dict) else None
            if not model_id:
                return {
                    'success': False,
                    'message': 'modeling session is not active',
                }

        encoded_model_id = quote(str(model_id), safe='')
        response = self._call('/modeling/draft/{}'.format(encoded_model_id))
        if not isinstance(response, dict):
            return {
                'success': False,
                'message': 'modeling draft response is invalid',
            }
        if response.get('success') is False:
            return {
                'success': False,
                'message': response.get('msg') or response.get('message') or 'modeling result fetch failed',
            }

        draft = response.get('data') or {}
        task_plan = draft.get('taskPlan') if isinstance(draft, dict) else None
        if not isinstance(task_plan, dict) or task_plan.get('status') != 'ready':
            return {
                'success': False,
                'message': 'modeling task plan is not ready',
            }

        return {
            'success': True,
            'message': 'modeling result fetched',
            'data': {
                'areaPoints': frontend_area_points(draft),
                'linkPoints': frontend_link_points(draft),
                'pathPoints': frontend_path_points(task_plan),
            },
        }
