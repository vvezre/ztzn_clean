# coding=utf-8
import io
import json
import os
import time

from AppLogger import logger


class MQTTCommandHandler(object):
    def __init__(self, vehicle_controller):
        self.vehicle_controller = vehicle_controller
        self.command_map = self._init_command_map()
        self.command_map.update({
            'turnLeft': self._handle_turn_left,
            'turnRight': self._handle_turn_right,
            'joystickMove': self._handle_joystick_move,
            'autoDrive': self._handle_auto_drive,
            'goOn': self._handle_go_on,
            'returnToPoint': self._handle_return_to_point,
            'enterGarage': self._handle_enter_garage,
            'exitGarage': self._handle_exit_garage,
            'adjustSpeed': self._handle_adjust_speed,
            'adjustBrushSpeed': self._handle_adjust_brush_speed,
            'toggleTracking': self._handle_toggle_tracking,
            'togglePathPlanning': self._handle_toggle_path_planning,
            'createTask': self._handle_create_task,
            'selectTask': self._handle_select_task,
            'saveTask': self._handle_save_task,
            'setCurrentTask': self._handle_set_current_task,
            'saveParams': self._handle_save_params,
            'setGarageEntry': self._handle_set_garage_entry,
            'getStatus': self._handle_get_status,
            'getTaskPath': self._handle_get_task_path,
            'get_task_path': self._handle_get_task_path,
        })
        logger.info("MQTT command handler initialized")

    def _init_command_map(self):
        return {
            'drive': self._handle_drive,
            'back': self._handle_back,
            'turn_left': self._handle_turn_left,
            'turn_right': self._handle_turn_right,
            'stop': self._handle_stop,
            'parking': self._handle_parking,
            'joystick_move': self._handle_joystick_move,
            'auto_drive': self._handle_auto_drive,
            'go_on': self._handle_go_on,
            'return_to_point': self._handle_return_to_point,
            'enter_garage': self._handle_enter_garage,
            'exit_garage': self._handle_exit_garage,
            'adjust_speed': self._handle_adjust_speed,
            'adjust_brush_speed': self._handle_adjust_brush_speed,
            'toggle_tracking': self._handle_toggle_tracking,
            'toggle_path_planning': self._handle_toggle_path_planning,
            'create_task': self._handle_create_task,
            'select_task': self._handle_select_task,
            'save_task': self._handle_save_task,
            'set_current_task': self._handle_set_current_task,
            'save_params': self._handle_save_params,
            'set_garage_entry': self._handle_set_garage_entry,
            'get_status': self._handle_get_status,
        }

    def handle(self, message_data):
        try:
            command = message_data.get('command')
            params = message_data.get('params')
            if params is None:
                params = message_data.get('parameters', {})

            if not command:
                return {'success': False, 'message': '消息格式错误：缺少 command 字段'}

            logger.info("Processing MQTT command: {}, params: {}".format(command, params))
            handler = self.command_map.get(command)
            if not handler:
                logger.warning("Unknown MQTT command: {}".format(command))
                return {'success': False, 'message': '未知命令: {}'.format(command)}

            result = handler(params or {})
            logger.info("MQTT command result: {}".format(result))
            return result
        except Exception as exc:
            logger.error("MQTT command handling failed: {}".format(exc), exc_info=True)
            return {'success': False, 'message': '命令处理异常: {}'.format(exc)}

    def _normalize_controller_result(self, result, default_message):
        if isinstance(result, dict):
            success = result.get('success')
            message = result.get('message') or default_message
            response = {
                'success': False if success is False else True,
                'message': message,
            }
            if 'data' in result:
                response['data'] = result.get('data')
            return response

        response = {
            'success': True,
            'message': default_message,
        }
        if result not in (None, '', '1'):
            response['data'] = result
        return response

    def _call_controller(self, method_name, default_message, *args):
        if not hasattr(self.vehicle_controller, method_name):
            return {'success': False, 'message': '车辆控制器不支持 {} 方法'.format(method_name)}
        result = getattr(self.vehicle_controller, method_name)(*args)
        return self._normalize_controller_result(result, default_message)

    def _extract_task_name(self, params):
        if not isinstance(params, dict):
            return ''
        task_name = params.get('taskName')
        if task_name is None:
            return ''
        return str(task_name).strip()

    def _handle_drive(self, params):
        return self._call_controller('drive', '前进命令已执行', params.get('distance', 0), params.get('speed'))

    def _handle_back(self, params):
        return self._call_controller('back', '后退命令已执行', params.get('distance', 0), params.get('speed'))

    def _handle_turn_left(self, params):
        return self._call_controller('turn_left', '左转命令已执行', params.get('angle', 90))

    def _handle_turn_right(self, params):
        return self._call_controller('turn_right', '右转命令已执行', params.get('angle', 90))

    def _handle_stop(self, params):
        return self._call_controller('stop', '停止命令已执行')

    def _handle_parking(self, params):
        return self._call_controller('parking', '停车命令已执行')

    def _handle_joystick_move(self, params):
        return self._call_controller(
            'joystick_move',
            '摇杆控制命令已执行',
            params.get('distance', 50),
            params.get('dirX', 0),
            params.get('dirY', 0)
        )

    def _handle_auto_drive(self, params):
        return self._call_controller('auto_drive', '自动清扫已启动')

    def _handle_go_on(self, params):
        return self._call_controller('go_on', '继续清扫命令已执行')

    def _handle_return_to_point(self, params):
        return self._call_controller('return_to_point', '返回原点命令已执行')

    def _handle_enter_garage(self, params):
        return self._call_controller('enter_garage', '入库命令已执行')

    def _handle_exit_garage(self, params):
        return self._call_controller('exit_garage', '出库命令已执行')

    def _handle_adjust_speed(self, params):
        return self._call_controller('adjust_speed', '移动速度已调整', params.get('speed', 50))

    def _handle_adjust_brush_speed(self, params):
        return self._call_controller('adjust_brush_speed', '滚刷速度已调整', params.get('speed', 50))

    def _handle_toggle_tracking(self, params):
        return self._call_controller('toggle_tracking', '循迹状态已切换', params.get('tracking', True))

    def _handle_toggle_path_planning(self, params):
        return self._call_controller('toggle_path_planning', '路径规划模式已切换', params.get('path', 'left'))

    def _handle_save_params(self, params):
        return self._call_controller('save_params', '参数已保存', params)

    def _handle_set_garage_entry(self, params):
        if params.get('lat') is None or params.get('lon') is None:
            return {'success': False, 'message': 'set_garage_entry命令需要参数：lat, lon'}
        return self._call_controller(
            'set_garage_entry',
            '入舱点已设置',
            params.get('lat'),
            params.get('lon')
        )

    def _handle_get_status(self, params):
        return self._call_controller('get_status', '状态获取成功')

    # Override task-related handlers to enforce non-empty task name
    # and make current-task switch single-step.
    def _handle_create_task(self, params):
        task_name = self._extract_task_name(params)
        if not task_name:
            return {'success': False, 'message': 'taskName不能为空'}
        if not isinstance(params, dict):
            return {'success': False, 'message': 'create_task参数格式错误'}
        area_list = params.get('areaList')
        if not isinstance(area_list, list) or len(area_list) == 0:
            return {'success': False, 'message': 'areaList不能为空'}
        return self._call_controller('create_task', '任务创建成功', params)

    def _handle_select_task(self, params):
        task_name = self._extract_task_name(params)
        if not task_name:
            return {'success': False, 'message': 'taskName不能为空'}
        return self._call_controller('select_task', '任务已选择', task_name)

    def _handle_save_task(self, params):
        return self._handle_set_current_task(params)

    def _handle_set_current_task(self, params):
        task_name = self._extract_task_name(params)
        if not task_name:
            return {'success': False, 'message': 'taskName不能为空'}
        return self._call_controller('set_current_task', '任务已设置为当前任务', task_name)

    def _fallback_task_path(self, params):
        config_path = 'config.json'
        if not os.path.exists(config_path):
            return {'success': False, 'message': '未找到任务配置文件'}

        with io.open(config_path, 'r', encoding='utf-8') as fp:
            config = json.load(fp)

        task_list = config.get('taskList') or []
        if not isinstance(task_list, list) or len(task_list) == 0:
            return {'success': False, 'message': '任务列表为空'}

        segments = []
        for item in task_list:
            if not isinstance(item, dict):
                continue
            segments.append({
                'id': item.get('id'),
                'startX': item.get('startX', 0),
                'startY': item.get('startY', 0),
                'endX': item.get('endX', 0),
                'endY': item.get('endY', 0),
                'mode': item.get('mode'),
                'angle': item.get('angle'),
                'heading': item.get('heading'),
                'areaNumber': item.get('areaNumber'),
            })

        task_name = config.get('taskName') or ''
        return {
            'success': True,
            'message': '任务路径获取成功',
            'data': {
                'taskId': params.get('taskId') if isinstance(params, dict) else task_name or 'current',
                'taskName': task_name,
                'originLat': config.get('startLat'),
                'originLon': config.get('startLon'),
                'yAxisBearing': config.get('originHeading'),
                'updatedAt': int(time.time() * 1000),
                'segments': segments,
            }
        }

    def _handle_get_task_path(self, params):
        try:
            if hasattr(self.vehicle_controller, 'get_task_path'):
                return self._normalize_controller_result(
                    self.vehicle_controller.get_task_path(),
                    '任务路径获取成功'
                )
            return self._fallback_task_path(params)
        except Exception as exc:
            logger.error("Fetch task path failed: {}".format(exc), exc_info=True)
            return {'success': False, 'message': '获取任务路径失败: {}'.format(exc)}
