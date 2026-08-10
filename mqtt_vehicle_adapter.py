# coding=utf-8
"""
MQTT 命令处理器与小车本地 Flask HTTP 接口之间的适配层。

该文件不计算清扫路径，只把 MQTT 命令转成本地接口调用，再将结果整理为 MQTT 回复。
实现仅使用标准库 HTTP 客户端，保持与小车 Python 2.7 环境兼容。
"""
import json
import os

try:
    from urllib import quote, urlencode
    from urllib2 import HTTPError, Request, URLError, urlopen
except ImportError:
    from urllib.error import HTTPError, URLError
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
    """封装小车 127.0.0.1:7899 本地接口，供 MQTTCommandHandler 统一调用。"""
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
        """
        调用小车本机 Flask 接口。

        - path：本地接口路径，默认基址为 http://127.0.0.1:7899。
        - params：编码到 URL query string。
        - json_data：编码成 UTF-8 JSON 请求体。

        HTTP 4xx/5xx 如果已包含业务 JSON，则尽量保留原错误码和错误数据；
        连接失败则抛出 RuntimeError，由上层 MQTT 处理器转成失败回复。
        """
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
        except HTTPError as exc:
            try:
                response = self._parse_response(exc)
            except Exception:
                response = None
            if isinstance(response, dict):
                if 'success' not in response:
                    response['success'] = False
                return response
            return {
                'success': False,
                'message': 'HTTP {} {}'.format(exc.code, exc.reason),
                'data': {'httpStatus': exc.code},
            }
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
        """将一条已保存路线设为当前任务；只有完成此操作，auto_drive 才会执行该路线。"""
        return self._call('/vehicle/setCurrentTask', params={'taskName': task_name})

    def save_modeling_task(self, task_name):
        """把当前已规划路径以 taskName 命名保存。"""
        return self._normalize_modeling_response(
            self._call(
                '/modeling/session/save-task',
                json_data={'taskName': task_name},
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
        """
        查询小车中已命名保存的所有路线。

        routes 中每一项包含路线名称以及 areaPoints/linkPoints/pathPoints，
        currentTaskName 表示当前真正被小车选中的路线。
        """
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
        """
        统一小车建模 HTTP 接口的返回格式。

        本地 Flask 接口可能使用 msg 或 message，该方法统一转为
        success/message/data，同时保留 code，供云平台和前端判断具体错误。
        """
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
        """
        启动小车本地建模会话。

        name 可以作为建模阶段的临时名称；
        restart 明确表示是否丢弃现有未完成会话并重新开始。
        """
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

    def new_modeling_area(self):
        """Create the next area and bind the connection points just recorded."""
        return self._normalize_modeling_response(
            self._call('/modeling/session/new-area', json_data={}),
            'new modeling area created',
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
        """调用本地 finish 流程，触发区域识别、清扫线计算和 taskPlan 生成。"""
        return self._normalize_modeling_response(
            self._call('/modeling/session/finish', json_data={}),
            'modeling path generated',
        )

    def replan_modeling_route(self, area_order):
        """按前端给出的区域编号顺序重新规划，不重新采点。"""
        return self._normalize_modeling_response(
            self._call(
                '/modeling/session/replan',
                json_data={'areaOrder': list(area_order or [])},
            ),
            'modeling route replanned',
        )

    def sample_modeling_point(self, model_id=None, group_id=None):
        """
        采样一个区域边界点。

        正常小程序流程不传 ID，直接将小车当前 RTK 位置写入活动会话。
        显式传入 model_id/group_id 时，可将点写入指定建模和区域，主要用于精确调试。
        成功返回的 point 包含唯一 id、记录顺序、x/y 和 lat/lon。
        """
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
        """
        采样一个跨区域连接点。

        该点与区域点分开存储，同一个 linkId 可以连续追加多个过渡点，
        路径生成时会按记录顺序拆成多段 mode=2 连接桥任务。
        """
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
        """
        读取指定或当前建模草稿，只返回前端需要的区域点列表。

        frontend_area_points 负责将内部 group/subArea 结构展平为 points[]，
        每个点保留 id/name/sequence/x/y/lat/lon，供前端列表展示和按 id 删除。
        """
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
        """
        读取指定或当前建模草稿，只返回前端需要的连接点列表。

        返回 points[] 的字段与区域点保持一致，但数据来源是 groupLinks，
        前端可以使用独立页签展示和删除。
        """
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
        """
        返回 FSM 生成的完整 taskPreview 和 taskPlan，不在适配层重新规划。

        taskPreview 是区域、连接桥和清扫线的预览结构；
        taskPlan.tasks 是机器人真正执行的有序分段列表。
        """
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
                'areaOrder': task_plan.get('areaOrder') or [],
                'taskName': task_plan.get('taskName') or draft.get('name') or '',
                'updatedAt': draft.get('updatedAt') or task_plan.get('generatedAt'),
                'taskPreview': draft.get('taskPreview'),
                'taskPlan': task_plan,
            },
        }

    def get_modeling_result(self, model_id=None):
        """
        把建模草稿整理为前端统一绘图数据：

        - areaPoints：用户记录的全部区域边界点。
        - linkPoints：用户记录的全部跨区域连接点。
        - pathPoints：从 taskPlan.tasks 中按执行顺序展开的路径点。

        前端只需要按 sequence 连接 pathPoints，无需理解内部 lane 和 task 结构。
        """
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
                'areaOrder': task_plan.get('areaOrder') or [],
                'areaPoints': frontend_area_points(draft),
                'linkPoints': frontend_link_points(draft),
                'pathPoints': frontend_path_points(task_plan),
            },
        }
