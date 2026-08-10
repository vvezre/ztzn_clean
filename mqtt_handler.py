# coding=utf-8
import io
import json
import os
import re
import time

from AppLogger import logger


try:
    text_type = unicode
    binary_type = str
except NameError:
    text_type = str
    binary_type = bytes


def _as_text(value):
    if value is None:
        return ''
    if isinstance(value, text_type):
        return value.strip()
    if isinstance(value, binary_type):
        try:
            return value.decode('utf-8').strip()
        except Exception:
            return value.decode('utf-8', 'replace').strip()
    return text_type(value).strip()


class MQTTCommandHandler(object):
    """
    MQTT 命令入口。

    云平台发来的 command 在这里映射到小车控制器方法，处理结果再交由 MQTT 客户端回传。

    典型请求数据：
        {
            "command": "sample_modeling_point",
            "params": {}
        }

    统一处理结果：
        {
            "success": true,
            "message": "modeling point recorded",
            "data": {...}
        }

    该层只负责命令路由、基础参数校验和返回格式统一，
    不在这里采集 RTK，也不在这里计算清扫路径。
    """
    def __init__(self, vehicle_controller):
        # command_map 将云平台命令名与对应处理函数建立一对一关系。
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
            'getSavedRoutes': self._handle_get_saved_routes,
            'get_saved_routes': self._handle_get_saved_routes,
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
            'newModelingArea': self._handle_new_modeling_area,
            'new_modeling_area': self._handle_new_modeling_area,
            'finishModeling': self._handle_finish_modeling,
            'finish_modeling': self._handle_finish_modeling,
            'replanModelingRoute': self._handle_replan_modeling_route,
            'replan_modeling_route': self._handle_replan_modeling_route,
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
            'get_saved_routes': self._handle_get_saved_routes,
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
            'new_modeling_area': self._handle_new_modeling_area,
            'finish_modeling': self._handle_finish_modeling,
            'replan_modeling_route': self._handle_replan_modeling_route,
            'get_modeling_state': self._handle_get_modeling_state,
            'undo_modeling_point': self._handle_undo_modeling_point,
            'delete_modeling_point': self._handle_delete_modeling_point,
            'delete_modeling_link_point': self._handle_delete_modeling_link_point,
            'clear_modeling_points': self._handle_clear_modeling_points,
            'clear_modeling_area_points': self._handle_clear_modeling_area_points,
            'clear_modeling_link_points': self._handle_clear_modeling_link_points,
        }

    def handle(self, message_data):
        """
        MQTT 命令的统一处理流程。

        1. 读取 command 和 params。
        2. 根据 command_map 定位处理函数。
        3. 调用适配器，进入小车本地 HTTP/FSM 功能。
        4. 统一返回 success/message/data。
        5. 任何未捕获异常转为 success=false，避免 MQTT 回调线程退出。
        """
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
        """
        将小车不同本地接口的返回值整理为统一 MQTT 结果。

        - dict 且含 success 时，保留其 success/message/data。
        - 普通数据时认为成功，并放入 data。
        - None、空字符串或 '1' 视为只有成功状态，使用 default_message。
        """
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
        """
        调用 VehicleControllerAdapter 中的指定方法。

        调用前先检查适配器是否真正实现该方法，
        防止云平台发来一个小车当前版本不支持的命令时直接崩溃。
        """
        if not hasattr(self.vehicle_controller, method_name):
            return {'success': False, 'message': '车辆控制器不支持 {} 方法'.format(method_name)}
        result = getattr(self.vehicle_controller, method_name)(*args)
        return self._normalize_controller_result(result, default_message)

    def _extract_task_name(self, params):
        if not isinstance(params, dict):
            return ''
        return _as_text(params.get('taskName'))

    def _extract_model_id(self, params):
        if not isinstance(params, dict):
            return ''
        return _as_text(params.get('modelId'))

    def _extract_group_id(self, params):
        if not isinstance(params, dict):
            return ''
        return _as_text(params.get('groupId'))

    def _extract_link_id(self, params):
        if not isinstance(params, dict):
            return ''
        return _as_text(params.get('linkId'))

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
        # 仅在 finish_modeling 已生成可执行路径后，才能用 taskName 将其保存。
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

    def _handle_get_saved_routes(self, params):
        # 返回的不是单纯名称列表，而是每条已保存路线的区域点、连接点和规划点。
        return self._call_controller(
            'get_saved_routes',
            'saved routes fetched',
        )

    def _handle_sample_modeling_point(self, params):
        # “记录区域点”按钮对应该命令，实际坐标由小车当前 RTK 位置采样得到。
        # params 为空时写入当前活动建模会话；显式传入标识时，modelId 和 groupId 必须成对出现。
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
        # “记录连接点”与区域点分开存储，后续用于生成跨区域移动段。
        # 一条连接桥可连续记录多个点，每次点击只追加一个点，这里不强制“两次点击即完成”。
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
        # 一次返回 areaPoints、linkPoints 和 pathPoints，供前端按顺序直接绘图。
        # 该命令只查询 FSM 已生成的结果，不会再次计算或改变任务。
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
        # 创建新的建模会话，后续的打点命令都写入该会话。
        # restart=false 时若已有活动会话则保护现有数据；restart=true 才明确重新开始。
        name = str(params.get('name') or '').strip() if isinstance(params, dict) else ''
        restart = bool(params.get('restart', False)) if isinstance(params, dict) else False
        return self._call_controller('start_modeling', 'modeling started', name or None, restart)

    def _handle_new_modeling_area(self, params):
        # 连接点记录完成后显式创建下一区域；区域编号由 FSM 自动递增。
        return self._call_controller('new_modeling_area', 'new modeling area created')

    def _handle_finish_modeling(self, params):
        # 完成打点后依次执行：区域识别 -> 清扫线预览 -> 机器人 taskPlan 生成。
        # 返回成功只表示规划完成；此时还没有路线名，必须再调用 save_modeling_task 才会保存。
        return self._call_controller('finish_modeling', 'modeling path generated')

    def _handle_replan_modeling_route(self, params):
        area_order = params.get('areaOrder') if isinstance(params, dict) else None
        if not isinstance(area_order, (list, tuple)) or not area_order:
            return {'success': False, 'message': 'areaOrder must be a non-empty array'}
        normalized = []
        for value in area_order:
            try:
                area_number = int(value)
            except (TypeError, ValueError):
                return {'success': False, 'message': 'areaOrder must contain area numbers'}
            if area_number <= 0 or area_number in normalized:
                return {'success': False, 'message': 'areaOrder must contain unique positive area numbers'}
            normalized.append(area_number)
        return self._call_controller(
            'replan_modeling_route',
            'modeling route replanned',
            normalized,
        )

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
        # id 是点位的唯一标识，删除后下次查询区域点列表将不再包含该点。
        point_id = str(params.get('id') or '').strip() if isinstance(params, dict) else ''
        if not point_id or re.match(r'^[A-Za-z0-9_-]+$', point_id) is None:
            return {'success': False, 'message': 'valid point id is required'}
        return self._call_controller(
            'delete_modeling_point',
            'modeling point deleted',
            point_id,
        )

    def _handle_delete_modeling_link_point(self, params):
        # 连接点使用独立命令删除，避免与区域点列表混淆。
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
        # 只清空当前建模会话的区域点，不删除连接点。
        return self._call_controller(
            'clear_all_modeling_points',
            'all modeling area points cleared',
            'area',
        )

    def _handle_clear_modeling_link_points(self, params):
        # 只清空当前建模会话的连接点，不删除区域点。
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
