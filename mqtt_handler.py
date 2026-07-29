# coding=utf-8
import io
import json
import os
import re
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
            'resumeAfterBatterySwap': self._handle_resume_after_battery_swap,
            'getBatterySwapStatus': self._handle_get_battery_swap_status,
            'clearBatterySwapCheckpoint': self._handle_clear_battery_swap_checkpoint,
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
            'saveModelingTask': self._handle_save_modeling_task,
            'save_modeling_task': self._handle_save_modeling_task,
            'getTaskNames': self._handle_get_task_names,
            'get_task_names': self._handle_get_task_names,
            'saveParams': self._handle_save_params,
            'setGarageEntry': self._handle_set_garage_entry,
            'getStatus': self._handle_get_status,
            'getTaskPath': self._handle_get_task_path,
            'get_task_path': self._handle_get_task_path,
            'getModelingPath': self._handle_get_modeling_path,
            'get_modeling_path': self._handle_get_modeling_path,
            'getModelingPoints': self._handle_get_modeling_points,
            'get_modeling_points': self._handle_get_modeling_points,
            'getModelingLinkPoints': self._handle_get_modeling_link_points,
            'get_modeling_link_points': self._handle_get_modeling_link_points,
            'getModelingResult': self._handle_get_modeling_result,
            'get_modeling_result': self._handle_get_modeling_result,
            'sampleModelingPoint': self._handle_sample_modeling_point,
            'sample_modeling_point': self._handle_sample_modeling_point,
            'sampleModelingLinkPoint': self._handle_sample_modeling_link_point,
            'sample_modeling_link_point': self._handle_sample_modeling_link_point,
            'startModeling': self._handle_start_modeling,
            'start_modeling': self._handle_start_modeling,
            'finishModeling': self._handle_finish_modeling,
            'finish_modeling': self._handle_finish_modeling,
            'getModelingState': self._handle_get_modeling_state,
            'get_modeling_state': self._handle_get_modeling_state,
            'undoModelingPoint': self._handle_undo_modeling_point,
            'undo_modeling_point': self._handle_undo_modeling_point,
            'deleteModelingPoint': self._handle_delete_modeling_point,
            'delete_modeling_point': self._handle_delete_modeling_point,
            'deleteModelingLinkPoint': self._handle_delete_modeling_link_point,
            'delete_modeling_link_point': self._handle_delete_modeling_link_point,
            'clearModelingPoints': self._handle_clear_modeling_points,
            'clear_modeling_points': self._handle_clear_modeling_points,
            'clearModelingAreaPoints': self._handle_clear_modeling_area_points,
            'clear_modeling_area_points': self._handle_clear_modeling_area_points,
            'clearModelingLinkPoints': self._handle_clear_modeling_link_points,
            'clear_modeling_link_points': self._handle_clear_modeling_link_points,
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
            'resume_after_battery_swap': self._handle_resume_after_battery_swap,
            'get_battery_swap_status': self._handle_get_battery_swap_status,
            'clear_battery_swap_checkpoint': self._handle_clear_battery_swap_checkpoint,
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
            'save_modeling_task': self._handle_save_modeling_task,
            'get_task_names': self._handle_get_task_names,
            'save_params': self._handle_save_params,
            'set_garage_entry': self._handle_set_garage_entry,
            'get_status': self._handle_get_status,
            'get_modeling_path': self._handle_get_modeling_path,
            'get_modeling_points': self._handle_get_modeling_points,
            'get_modeling_link_points': self._handle_get_modeling_link_points,
            'get_modeling_result': self._handle_get_modeling_result,
            'sample_modeling_point': self._handle_sample_modeling_point,
            'sample_modeling_link_point': self._handle_sample_modeling_link_point,
            'start_modeling': self._handle_start_modeling,
            'finish_modeling': self._handle_finish_modeling,
            'get_modeling_state': self._handle_get_modeling_state,
            'undo_modeling_point': self._handle_undo_modeling_point,
            'delete_modeling_point': self._handle_delete_modeling_point,
            'delete_modeling_link_point': self._handle_delete_modeling_link_point,
            'clear_modeling_points': self._handle_clear_modeling_points,
            'clear_modeling_area_points': self._handle_clear_modeling_area_points,
            'clear_modeling_link_points': self._handle_clear_modeling_link_points,
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

    def _extract_model_id(self, params):
        if not isinstance(params, dict):
            return ''
        model_id = params.get('modelId')
        if model_id is None:
            return ''
        return str(model_id).strip()

    def _extract_group_id(self, params):
        if not isinstance(params, dict):
            return ''
        group_id = params.get('groupId')
        if group_id is None:
            return ''
        return str(group_id).strip()

    def _extract_link_id(self, params):
        if not isinstance(params, dict):
            return ''
        link_id = params.get('linkId')
        if link_id is None:
            return ''
        return str(link_id).strip()

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

    def _handle_resume_after_battery_swap(self, params):
        return self._call_controller('resume_after_battery_swap', 'resume after battery swap command executed')

    def _handle_get_battery_swap_status(self, params):
        return self._call_controller('get_battery_swap_status', 'battery swap status fetched')

    def _handle_clear_battery_swap_checkpoint(self, params):
        return self._call_controller('clear_battery_swap_checkpoint', 'battery swap checkpoint cleared')

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

    def _handle_save_modeling_task(self, params):
        task_name = self._extract_task_name(params)
        if not task_name:
            return {'success': False, 'message': 'taskName is required'}
        return self._call_controller(
            'save_modeling_task',
            'modeling task saved',
            task_name,
        )

    def _handle_get_task_names(self, params):
        return self._call_controller(
            'get_task_names',
            'task names fetched',
        )

    def _handle_sample_modeling_point(self, params):
        model_id = self._extract_model_id(params)
        group_id = self._extract_group_id(params)
        try:
            if not model_id and not group_id:
                return self._call_controller(
                    'sample_modeling_point',
                    'modeling point recorded',
                )
            if not model_id or re.match(r'^[A-Za-z0-9_-]+$', model_id) is None:
                return {'success': False, 'message': 'valid modelId is required when identifiers are provided'}
            if not group_id or re.match(r'^[A-Za-z0-9_-]+$', group_id) is None:
                return {'success': False, 'message': 'valid groupId is required when identifiers are provided'}
            return self._call_controller(
                'sample_modeling_point',
                'modeling point recorded',
                model_id,
                group_id,
            )
        except Exception as exc:
            logger.error("Record modeling point failed: {}".format(exc), exc_info=True)
            return {'success': False, 'message': 'modeling point recording failed: {}'.format(exc)}

    def _handle_sample_modeling_link_point(self, params):
        model_id = self._extract_model_id(params)
        link_id = self._extract_link_id(params)
        try:
            if not model_id and not link_id:
                return self._call_controller(
                    'sample_modeling_link_point',
                    'modeling link point recorded',
                )
            if not model_id or re.match(r'^[A-Za-z0-9_-]+$', model_id) is None:
                return {'success': False, 'message': 'valid modelId is required when identifiers are provided'}
            if not link_id or re.match(r'^[A-Za-z0-9_-]+$', link_id) is None:
                return {'success': False, 'message': 'valid linkId is required when identifiers are provided'}
            return self._call_controller(
                'sample_modeling_link_point',
                'modeling link point recorded',
                model_id,
                link_id,
            )
        except Exception as exc:
            logger.error("Record modeling link point failed: {}".format(exc), exc_info=True)
            return {'success': False, 'message': 'modeling link point recording failed: {}'.format(exc)}

    def _handle_get_modeling_path(self, params):
        model_id = self._extract_model_id(params)
        try:
            if not model_id:
                return self._call_controller(
                    'get_modeling_path',
                    'modeling path fetched',
                )
            if re.match(r'^[A-Za-z0-9_-]+$', model_id) is None:
                return {'success': False, 'message': 'valid modelId is required when provided'}
            return self._call_controller(
                'get_modeling_path',
                'modeling path fetched',
                model_id,
            )
        except Exception as exc:
            logger.error("Fetch modeling path failed: {}".format(exc), exc_info=True)
            return {'success': False, 'message': 'modeling path fetch failed: {}'.format(exc)}

    def _handle_get_modeling_result(self, params):
        model_id = self._extract_model_id(params)
        try:
            if not model_id:
                return self._call_controller(
                    'get_modeling_result',
                    'modeling result fetched',
                )
            if re.match(r'^[A-Za-z0-9_-]+$', model_id) is None:
                return {'success': False, 'message': 'valid modelId is required when provided'}
            return self._call_controller(
                'get_modeling_result',
                'modeling result fetched',
                model_id,
            )
        except Exception as exc:
            logger.error("Fetch modeling result failed: {}".format(exc), exc_info=True)
            return {'success': False, 'message': 'modeling result fetch failed: {}'.format(exc)}

    def _handle_get_modeling_points(self, params):
        model_id = self._extract_model_id(params)
        try:
            if not model_id:
                return self._call_controller(
                    'get_modeling_points',
                    'modeling points fetched',
                )
            if re.match(r'^[A-Za-z0-9_-]+$', model_id) is None:
                return {'success': False, 'message': 'valid modelId is required when provided'}
            return self._call_controller(
                'get_modeling_points',
                'modeling points fetched',
                model_id,
            )
        except Exception as exc:
            logger.error("Fetch modeling points failed: {}".format(exc), exc_info=True)
            return {'success': False, 'message': 'modeling points fetch failed: {}'.format(exc)}

    def _handle_get_modeling_link_points(self, params):
        model_id = self._extract_model_id(params)
        try:
            if not model_id:
                return self._call_controller(
                    'get_modeling_link_points',
                    'modeling link points fetched',
                )
            if re.match(r'^[A-Za-z0-9_-]+$', model_id) is None:
                return {'success': False, 'message': 'valid modelId is required when provided'}
            return self._call_controller(
                'get_modeling_link_points',
                'modeling link points fetched',
                model_id,
            )
        except Exception as exc:
            logger.error("Fetch modeling link points failed: {}".format(exc), exc_info=True)
            return {'success': False, 'message': 'modeling link points fetch failed: {}'.format(exc)}

    def _handle_start_modeling(self, params):
        name = str(params.get('name') or '').strip() if isinstance(params, dict) else ''
        restart = bool(params.get('restart', False)) if isinstance(params, dict) else False
        return self._call_controller('start_modeling', 'modeling started', name or None, restart)

    def _handle_finish_modeling(self, params):
        return self._call_controller('finish_modeling', 'modeling path generated')

    def _handle_get_modeling_state(self, params):
        return self._call_controller('get_modeling_state', 'modeling state fetched')

    def _extract_point_type(self, params):
        point_type = str(params.get('pointType') or '').strip().lower() if isinstance(params, dict) else ''
        if point_type and point_type not in ('area', 'link'):
            return None, {'success': False, 'message': 'pointType must be area or link'}
        return point_type or None, None

    def _handle_undo_modeling_point(self, params):
        point_type, error = self._extract_point_type(params)
        if error:
            return error
        return self._call_controller('undo_modeling_point', 'modeling point undone', point_type)

    def _handle_delete_modeling_point(self, params):
        point_id = str(params.get('id') or '').strip() if isinstance(params, dict) else ''
        if not point_id or re.match(r'^[A-Za-z0-9_-]+$', point_id) is None:
            return {'success': False, 'message': 'valid point id is required'}
        return self._call_controller(
            'delete_modeling_point',
            'modeling point deleted',
            point_id,
        )

    def _handle_delete_modeling_link_point(self, params):
        point_id = str(params.get('id') or '').strip() if isinstance(params, dict) else ''
        if not point_id or re.match(r'^[A-Za-z0-9_-]+$', point_id) is None:
            return {'success': False, 'message': 'valid connection point id is required'}
        return self._call_controller(
            'delete_modeling_link_point',
            'modeling link point deleted',
            point_id,
        )

    def _handle_clear_modeling_points(self, params):
        point_type, error = self._extract_point_type(params)
        if error:
            return error
        return self._call_controller('clear_modeling_points', 'modeling points cleared', point_type)

    def _handle_clear_modeling_area_points(self, params):
        return self._call_controller(
            'clear_all_modeling_points',
            'all modeling area points cleared',
            'area',
        )

    def _handle_clear_modeling_link_points(self, params):
        return self._call_controller(
            'clear_all_modeling_points',
            'all modeling link points cleared',
            'link',
        )

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
