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

    def save_params(self, params):
        return self._call('/vehicle/saveParams', json_data=params or {})

    def set_garage_entry(self, lat, lon):
        return self._call('/vehicle/setGarageEntryInfo', params={'lat': lat, 'lon': lon})

    def get_status(self):
        return self._call('/vehicle/getInfo')

    def get_task_path(self):
        return self._call('/vehicle/getTaskPath')

    def sample_modeling_point(self, model_id, group_id):
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

    def sample_modeling_link_point(self, model_id, link_id):
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

    def get_modeling_path(self, model_id):
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
                'taskPlan': task_plan,
            },
        }
