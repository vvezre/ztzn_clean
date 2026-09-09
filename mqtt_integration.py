#!/usr/bin/env python
# coding=utf-8
"""
MQTT 集成模块
将 MQTT 功能集成到现有系统，提供车辆状态上报和远程控制功能。
"""

import json
import threading
import time

import util
from AppLogger import logger
from mqtt_client import MQTTClient, get_mqtt_client
from mqtt_handler import MQTTCommandHandler
from motion_state import derive_manual_motion_state, manual_steering_allowed
from status_values import (
    live_heading_from_location,
    live_value_from_report,
    visible_task_field,
)
try:
    from ntrip_runtime import get_shared_runtime
except Exception:
    get_shared_runtime = None

DEFAULT_TASK_ORIGIN_TOLERANCE_METERS = 0.20
RTK_FIXED_QUALITY = '4'
RTK_FIXED_GGA_MAX_AGE_SECONDS = 2.0
RTK_FIX_RECOVERY_TIMEOUT_SECONDS = 300.0
RTK_STATUS_FIELDS = (
    'rtkQuality',
    'rtkFixAvailable',
    'rtkFixState',
    'rtkRecovering',
    'rtkLostSeconds',
    'rtkRecoveryTimeoutSec',
    'rtkGgaAgeSec',
    'rtkLastGgaAt',
    'rtkLastFixedAt',
    'rtkFixedAgeSec',
    'rtcmAgeSec',
    'rtcmTimeoutSec',
    'rtcmTimedOut',
    'rtcmLastReceivedAt',
    'ntripEnabled',
    'ntripConfigured',
    'ntripConnected',
    'ntripLastDisconnectReason',
)


class MQTTIntegration:
    """MQTT 集成类"""

    def __init__(self, config, vehicle_controller, redis_client=None):
        self.config = config
        self.vehicle_controller = vehicle_controller
        self.redis_client = redis_client

        self.mqtt_client = MQTTClient(config)
        self.command_handler = MQTTCommandHandler(vehicle_controller)
        self.mqtt_client.set_message_callback(self._on_mqtt_message)

        self.status_thread = None
        self.position_thread = None
        self.heartbeat_thread = None
        self.running = False
        self.status_interval = config.get('mqtt', {}).get('status_interval', 5)
        self.position_interval = max(
            float(config.get('mqtt', {}).get('position_interval', 1)),
            0.1
        )
        self.heartbeat_interval = max(
            float(config.get('mqtt', {}).get('heartbeat_interval', max(self.status_interval * 3, 15))),
            1.0
        )
        self.heartbeat_check_interval = min(max(int(self.heartbeat_interval / 3), 1), 5)

        logger.info("MQTT集成模块初始化完成")

    def _on_mqtt_message(self, message_data):
        try:
            self._publish_command_ack(message_data)
            result = self.command_handler.handle(message_data)
            self._publish_command_result(message_data, result)

            if result.get('success'):
                self.publish_vehicle_status()
        except Exception as e:
            self._publish_command_result(message_data, {
                'success': False,
                'message': 'command handler exception: {}'.format(str(e))
            })
            logger.error("处理MQTT消息异常: {}".format(str(e)), exc_info=True)

    def _publish_command_ack(self, message_data):
        try:
            command = message_data.get('command')
            if not command:
                return
            self.mqtt_client.publish_status({
                'type': 'ack',
                'command_id': message_data.get('command_id'),
                'trace_id': message_data.get('trace_id'),
                'command': command,
                'status': 'accepted',
                'timestamp': int(time.time())
            })
        except Exception as e:
            logger.error("发布命令ACK失败: {}".format(str(e)), exc_info=True)

    def _publish_command_result(self, message_data, result):
        try:
            self.mqtt_client.publish_status({
                'type': 'command_result',
                'command_id': message_data.get('command_id'),
                'trace_id': message_data.get('trace_id'),
                'command': message_data.get('command'),
                'result': result,
                'timestamp': int(time.time())
            })
        except Exception as e:
            logger.error("发布命令结果失败: {}".format(str(e)), exc_info=True)

    def _get_vehicle_status_from_redis(self):
        """
        从 Redis 汇总云平台需要的完整设备状态。

        数据来源分为四类：
        1. currentLocation：RTK 经纬度和当前航向。
        2. taskParams/config.json：当前路线名、起点、起始航向和任务段数量。
        3. runtimeState：当前动作、运行阶段、停止原因和故障状态。
        4. 下位机实时上报：行走速度、滚刷速度、电压和电量。

        重要字段：
        - speed/brush_speed：最新硬件实测值，过期时不冒充实时值。
        - command_speed/command_brush_speed：下发的目标值。
        - lat/lon/heading：绝对 RTK 位置和航向。
        - local_x/local_y：相对当前路线原点的厘米坐标。
        - task_name/cur_task_index/task_count：当前路线执行进度。
        - control_state/health_state/fault_state：前端按钮和故障提示的判断依据。
        """
        if not self.redis_client:
            return {}

        try:
            current_location = self._get_redis_hash('currentLocation')
            task_params = self._get_redis_hash('taskParams')
            runtime_state = self._read_runtime_state()
            current_action = runtime_state.get('action') or self._get_redis_value('currentAction', str, '')
            control_state = self._build_control_state(runtime_state)
            hardware_report_at = self._get_optional_int('hardwareReportAt')
            clean_task_count = self._build_task_count()
            waypoint_count = self._build_waypoint_count(current_action == 'multi_go_to_point')
            lat = self._coerce_value(current_location.get('lat'), float, None)
            lon = self._coerce_value(current_location.get('lon'), float, None)
            heading = live_heading_from_location(
                lat,
                lon,
                self._coerce_value(current_location.get('heading'), float, None)
            )
            local_x, local_y = self._compute_local_xy_cm(lat, lon, runtime_state, current_action)
            task_origin_fields = self._build_task_origin_status_fields(task_params, current_location)
            active_task_name = visible_task_field(self._build_task_name(), current_action, control_state)
            active_task_index = visible_task_field(
                self._get_redis_value('curTaskIndex', int, None),
                current_action,
                control_state,
            )
            active_task_count = visible_task_field(
                clean_task_count if clean_task_count > 0 else None,
                current_action,
                control_state,
            )
            live_speed = live_value_from_report(
                self._get_redis_value('xSpeed', int, None),
                hardware_report_at,
            )
            manual_mode = self._get_redis_value('manualSteeringMode', str, 'none') or 'none'
            fault_state = self._build_fault_state(runtime_state)
            motion_state = derive_manual_motion_state(
                live_speed,
                lower_status=self._get_redis_value('lowerMachineStatus', int, None),
                control_state=control_state,
                fault_state=fault_state,
                manual_mode=manual_mode,
                stop_requested=current_action == 'parking',
                z_speed=self._get_redis_value('zSpeed', int, None),
                power_on=self._get_redis_value('powerOnState', int, None),
                report_at=hardware_report_at,
            )
            status = {
                'speed': live_speed,
                'xSpeed': live_speed,
                'brush_speed': live_value_from_report(self._get_redis_value('brushSpeedActual', int, None), hardware_report_at),
                'command_speed': self._get_redis_value('forwardSpeed', int, None),
                'command_brush_speed': self._get_redis_value('brushSpeed', int, None),
                'voltage': self._get_redis_value('packVoltage', float, None),
                'lat': lat,
                'lon': lon,
                'heading': heading,
                # local_x/local_y 是相对当前建模原点的厘米坐标，前端用它绘制机器人在路线中的实时位置。
                'local_x': local_x,
                'local_y': local_y,
                'taskOrigin': task_origin_fields.get('taskOrigin'),
                'currentLocation': task_origin_fields.get('currentLocation'),
                'distanceToTaskOriginM': task_origin_fields.get('distanceToTaskOriginM'),
                'taskOriginToleranceM': task_origin_fields.get('taskOriginToleranceM'),
                'isAtTaskOrigin': task_origin_fields.get('isAtTaskOrigin'),
                'status': self._build_status(runtime_state),
                'action': self._build_action(runtime_state),
                'task_name': active_task_name,
                'cur_task_index': active_task_index,
                'task_count': active_task_count,
                'clean_task_count': active_task_count,
                'waypoint_index': self._get_redis_value('waypointIndex', int, 0),
                'waypoint_count': waypoint_count,
                'battery': self._get_redis_value('batteryPercent', float, None),
                'battery_percent': self._get_redis_value('batteryPercent', float, None),
                'battery_raw': self._get_redis_value('batteryPercentRaw', float, None),
                'battery_percent_raw': self._get_redis_value('batteryPercentRaw', float, None),
                'pack_voltage': self._get_redis_value('packVoltage', float, None),
                'online_state': 'ONLINE',
                'mission_state': self._build_mission_state(runtime_state),
                'control_state': control_state,
                'health_state': self._build_health_state(runtime_state),
                'fault_state': fault_state,
                'motionState': motion_state,
                'manualSteeringAllowed': manual_steering_allowed(motion_state),
                'manualSteeringMode': manual_mode,
                'manualSteeringDirection': self._get_redis_value('manualSteeringDirection', str, '') or None,
                'manualCorrectionValue': self._get_redis_value('manualCorrectionValue', int, 0),
                'manualCorrectionLevel': self._get_redis_value('manualCorrectionLevel', int, 0),
                'tracking': self._get_redis_value('correct', self._bool_value, False),
                'path_planning': self._get_redis_value('pathPlanning', str, ''),
                'move_judge': self._get_redis_value('moveJudge', self._bool_value, False),
                'detect_qrcode': self._get_redis_value('detectQrcode', self._bool_value, False),
                'enter_garage': self._get_redis_value('enterGarage', self._bool_value, False),
                'supported_actions': [
                    'auto_drive', 'go_on', 'stop', 'parking', 'manual_steering', 'return_to_point', 'go_to_point', 'multi_go_to_point', 'get_status', 'get_task_path', 'get_modeling_path', 'get_modeling_points', 'new_modeling_link', 'new_modeling_area', 'replan_modeling_route'
                ],
                'supported_params': ['taskName', 'modelId', 'speed', 'tracking', 'path'],
                'supported_status_fields': [
                    'control_state', 'health_state', 'fault_state', 'mission_state',
                    'local_x', 'local_y', 'detail', 'rtk', 'xSpeed', 'motionState',
                    'manualSteeringAllowed', 'manualSteeringMode',
                    'manualSteeringDirection', 'manualCorrectionValue',
                    'manualCorrectionLevel'
                ],
                'detail': self._build_detail(
                    task_params=task_params,
                    current_location=current_location,
                    runtime_state=runtime_state,
                ),
                'timestamp': int(time.time())
            }
            status.update(self._build_rtk_status_fields(status.get('detail')))

            return status
        except Exception as e:
            logger.error("从Redis获取车辆状态失败: {}".format(str(e)), exc_info=True)
            return {}

    def _modeling_local_xy_cm(self, lat, lon, runtime_state, current_action=None):
        """
        计算“建模任务执行期间”的实时相对位置。

        建模任务的第一段同时保存了 startLat/startLon 和 startX/startY。
        程序以该点作为锚点：
        1. 将当前 RTK lat/lon 换算为相对锚点的米制 x/y。
        2. 转为厘米后加上锚点本身的 startX/startY。

        这样前端收到的实时位置与 pathPoints 使用同一套相对坐标系。
        非 modeling_task 运行状态时返回 None，交给通用路线原点算法处理。
        """
        runtime_state = runtime_state if isinstance(runtime_state, dict) else {}
        action = runtime_state.get('action') or current_action
        if action != 'modeling_task':
            return None

        detail = runtime_state.get('detail')
        if not isinstance(detail, dict):
            detail = self._read_runtime_detail()
        modeling_task = detail.get('modelingTask') if isinstance(detail, dict) else None
        segments = modeling_task.get('segments') if isinstance(modeling_task, dict) else None
        if not isinstance(segments, list) or not segments:
            return None, None

        anchor = segments[0] if isinstance(segments[0], dict) else {}
        anchor_lat = self._coerce_value(anchor.get('startLat'), float, None)
        anchor_lon = self._coerce_value(anchor.get('startLon'), float, None)
        anchor_x = self._coerce_value(anchor.get('startX'), float, None)
        anchor_y = self._coerce_value(anchor.get('startY'), float, None)
        if None in (anchor_lat, anchor_lon, anchor_x, anchor_y):
            return None, None

        x_m, y_m = util.latlon_to_local_rotated_xy_precise(
            anchor_lat, anchor_lon, lat, lon, 0.0
        )
        return (
            int(round(anchor_x + x_m * 100.0)),
            int(round(anchor_y + y_m * 100.0)),
        )

    def _compute_local_xy_cm(self, lat, lon, runtime_state=None, current_action=None):
        """
        统一计算前端使用的 local_x/local_y，单位厘米。

        优先级：
        1. 正在执行建模任务时，使用 modelingTask 第一段作为坐标锚点。
        2. 执行已保存路线时，使用 taskParams.startLat/startLon/originHeading 建立路线坐标系。

        latlon_to_local_rotated_xy_precise 先计算米制东北位移，再根据 originHeading 旋转到路线坐标系。
        缺少 RTK 位置或路线原点参数时返回 (None, None)，不向前端上报伪造的 0,0。
        """
        if lat is None or lon is None:
            return None, None
        try:
            modeling_xy = self._modeling_local_xy_cm(lat, lon, runtime_state, current_action)
            if modeling_xy is not None:
                return modeling_xy

            task_params = self._get_redis_hash('taskParams')
            origin_lat = self._coerce_value(task_params.get('startLat'), float, None)
            origin_lon = self._coerce_value(task_params.get('startLon'), float, None)
            origin_heading = self._coerce_value(task_params.get('originHeading'), float, None)
            if origin_lat is None or origin_lon is None or origin_heading is None:
                return None, None
            x_m, y_m = util.latlon_to_local_rotated_xy_precise(
                origin_lat, origin_lon, lat, lon, origin_heading
            )
            return int(round(x_m * 100)), int(round(y_m * 100))
        except Exception as e:
            logger.debug("计算本地坐标失败: {}".format(str(e)))
            return None, None

    def _get_redis_hash(self, key):
        try:
            value = self.redis_client.hgetall(key)
            if not value:
                return {}

            result = {}
            for item_key, item_value in value.items():
                result[self._decode_value(item_key)] = self._decode_value(item_value)
            return result
        except Exception:
            return {}

    def _decode_value(self, value):
        try:
            if isinstance(value, bytes):
                return value.decode('utf-8')
        except Exception:
            pass
        return value

    def _coerce_value(self, value, value_type, default):
        if value is None:
            return default
        try:
            return value_type(self._decode_value(value))
        except Exception:
            return default

    def _get_redis_value(self, key, value_type, default):
        try:
            value = self.redis_client.get(key)
            if value is None:
                return default
            return value_type(self._decode_value(value))
        except Exception:
            return default

    def _get_optional_int(self, key):
        try:
            value = self.redis_client.get(key)
            if value is None:
                return None
            return int(self._decode_value(value))
        except Exception:
            return None

    def _hardware_report_age_sec(self):
        report_at = self._get_optional_int('hardwareReportAt')
        if report_at is None:
            return None
        try:
            return max(0, int(time.time()) - int(report_at))
        except Exception:
            return None

    def _get_task_origin_tolerance_m(self, detail=None):
        detail = detail or self._read_runtime_detail()
        tolerance = detail.get('taskOriginToleranceM')
        if tolerance is None:
            tolerance = detail.get('startToleranceM')
        return self._coerce_value(tolerance, float, DEFAULT_TASK_ORIGIN_TOLERANCE_METERS)

    def _distance_to_task_origin(self, task_params=None, current_location=None):
        task_params = task_params or self._get_redis_hash('taskParams')
        current_location = current_location or self._get_redis_hash('currentLocation')
        start_lat = self._coerce_value(task_params.get('startLat'), float, None)
        start_lon = self._coerce_value(task_params.get('startLon'), float, None)
        current_lat = self._coerce_value(current_location.get('lat'), float, None)
        current_lon = self._coerce_value(current_location.get('lon'), float, None)
        if start_lat is None or start_lon is None or current_lat is None or current_lon is None:
            return None
        try:
            distance, _ = util.get_distance_angle(current_lat, current_lon, start_lat, start_lon)
            return round(float(distance), 3)
        except Exception:
            return None

    def _build_task_origin_status_fields(self, task_params=None, current_location=None, tolerance_m=None):
        task_params = task_params or self._get_redis_hash('taskParams')
        current_location = current_location or self._get_redis_hash('currentLocation')
        start_lat = self._coerce_value(task_params.get('startLat'), float, None)
        start_lon = self._coerce_value(task_params.get('startLon'), float, None)
        current_lat = self._coerce_value(current_location.get('lat'), float, None)
        current_lon = self._coerce_value(current_location.get('lon'), float, None)
        current_heading = self._coerce_value(current_location.get('heading'), float, None)
        tolerance = tolerance_m if tolerance_m is not None else self._get_task_origin_tolerance_m()
        distance_to_start = self._distance_to_task_origin(task_params, current_location)
        at_origin = None
        if distance_to_start is not None:
            at_origin = distance_to_start <= tolerance
        return {
            'taskOrigin': {
                'lat': start_lat,
                'lon': start_lon,
            },
            'currentLocation': {
                'lat': current_lat,
                'lon': current_lon,
                'heading': live_heading_from_location(current_lat, current_lon, current_heading),
            },
            'distanceToTaskOriginM': distance_to_start,
            'taskOriginToleranceM': tolerance,
            'isAtTaskOrigin': at_origin,
        }

    def _bool_value(self, value):
        value = self._decode_value(value)
        if isinstance(value, bool):
            return value
        if value is None:
            return False
        text = str(value).strip().lower()
        if text in ('1', 'true', 'yes', 'on'):
            return True
        if text in ('0', 'false', 'no', 'off', 'none', ''):
            return False
        try:
            return int(text) != 0
        except Exception:
            return False

    def _read_runtime_detail(self):
        raw = self._get_redis_value('runtimeDetail', str, '')
        if not raw:
            return {}
        try:
            detail = json.loads(raw)
            return detail if isinstance(detail, dict) else {}
        except Exception:
            return {}

    def _read_runtime_state(self):
        raw = self._get_redis_value('runtimeState', str, '')
        if not raw:
            return {}
        try:
            state = json.loads(raw)
            return state if isinstance(state, dict) else {}
        except Exception:
            return {}

    def _build_rtk_status_fields(self, detail=None):
        detail = detail or self._read_runtime_detail()
        return dict((key, detail.get(key)) for key in RTK_STATUS_FIELDS if key in detail)

    def _build_live_rtk_runtime_detail(self, detail=None):
        detail = dict(detail or {})
        status = {}
        try:
            if get_shared_runtime is not None:
                runtime = get_shared_runtime(logger)
                if runtime is not None and hasattr(runtime, 'get_status'):
                    status = runtime.get_status()
        except Exception as e:
            logger.debug("get live RTK runtime status failed: {}".format(str(e)))

        if not status:
            return {}

        quality = status.get('rtkQuality')
        try:
            gga_age = float(status.get('rtkGgaAgeSec'))
        except (TypeError, ValueError):
            gga_age = None

        fixed_available = (
            quality is not None
            and str(quality) == RTK_FIXED_QUALITY
            and gga_age is not None
            and gga_age <= RTK_FIXED_GGA_MAX_AGE_SECONDS
        )
        if gga_age is None:
            fix_state = 'NO_RTK_GGA'
        elif gga_age > RTK_FIXED_GGA_MAX_AGE_SECONDS:
            fix_state = 'RTK_GGA_TIMEOUT'
        elif quality is None or str(quality) != RTK_FIXED_QUALITY:
            fix_state = 'RTK_NOT_FIXED'
        else:
            fix_state = 'FIXED'

        if detail.get('rtkRecovering') or detail.get('rtkFixState') == 'RTK_FIX_TIMEOUT':
            fix_state = detail.get('rtkFixState') or fix_state

        status.update({
            'rtkFixAvailable': fixed_available,
            'rtkFixState': fix_state,
            'rtkFixedQuality': RTK_FIXED_QUALITY,
            'rtkGgaMaxAgeSec': RTK_FIXED_GGA_MAX_AGE_SECONDS,
            'rtkRecovering': bool(detail.get('rtkRecovering', False)),
            'rtkLostAt': detail.get('rtkLostAt'),
            'rtkLostSeconds': detail.get('rtkLostSeconds'),
            'rtkRecoveryTimeoutSec': detail.get('rtkRecoveryTimeoutSec', RTK_FIX_RECOVERY_TIMEOUT_SECONDS),
        })
        return status

    def _build_status(self, runtime_state=None):
        runtime_state = runtime_state or {}
        runtime_lifecycle = str(runtime_state.get('state') or runtime_state.get('controlState') or '').upper()
        runtime_action = str(runtime_state.get('action') or '')
        if runtime_action == 'return_to_point':
            return 'returning'
        if runtime_lifecycle in ('RUNNING', 'PAUSED', 'STOPPING'):
            return 'working'
        if runtime_lifecycle == 'BLOCKED':
            return 'blocked'
        if runtime_lifecycle in ('STOPPED', 'COMPLETE', 'DISABLED'):
            return 'idle'
        if runtime_lifecycle == 'UNKNOWN':
            return 'unknown'

        mission = self._get_redis_value('mission', str, '')
        parking = self._get_redis_value('parking', str, '0')
        current_action = self._get_redis_value('currentAction', str, '')
        if current_action == 'return_to_point':
            return 'returning'
        if mission == 'working':
            return 'working'
        if parking == '1':
            return 'idle'
        if mission == 'complete':
            return 'idle'
        return 'active'

    def _build_action(self, runtime_state=None):
        runtime_state = runtime_state or {}
        runtime_action = str(runtime_state.get('action') or '')
        if runtime_action:
            return runtime_action
        current_action = self._get_redis_value('currentAction', str, '')
        if current_action:
            return current_action
        if self._get_redis_value('mission', str, '') == 'working':
            return 'auto_drive'
        if self._get_redis_value('parking', str, '0') == '1':
            return 'parking'
        return 'idle'

    def _build_mission_state(self, runtime_state=None):
        runtime_state = runtime_state or {}
        runtime_action = str(runtime_state.get('action') or '')
        runtime_lifecycle = str(runtime_state.get('state') or runtime_state.get('controlState') or '').upper()
        if runtime_action == 'return_to_point':
            return 'RETURNING'
        if runtime_lifecycle in ('INITIALIZING', 'READY', 'RUNNING', 'PAUSED', 'STOPPING', 'STOPPED', 'COMPLETE', 'BLOCKED', 'FAULT', 'DISABLED', 'UNKNOWN'):
            return 'RUNNING' if runtime_lifecycle in ('PAUSED', 'STOPPING') else runtime_lifecycle

        current_action = self._get_redis_value('currentAction', str, '')
        if current_action == 'return_to_point':
            return 'RETURNING'

        mission = self._get_redis_value('mission', str, '')
        if mission == 'working':
            return 'RUNNING'
        if self._get_redis_value('parking', str, '0') == '1':
            return 'STOPPED'
        if mission == 'complete':
            return 'COMPLETE'
        return 'IDLE'

    def _build_control_state(self, runtime_state=None):
        runtime_state = runtime_state or {}
        runtime_control = str(runtime_state.get('controlState') or runtime_state.get('state') or '').upper()
        if runtime_control:
            return runtime_control
        control_state = self._get_redis_value('controlState', str, '')
        if control_state and control_state not in ('DISABLED', 'UNKNOWN'):
            return control_state
        if self._get_redis_value('mission', str, '') == 'working':
            return 'RUNNING'
        if self._get_redis_value('parking', str, '0') == '1':
            return 'STOPPED'
        return 'IDLE'

    def _is_ignored_enable_fault_state(self, fault_state):
        return fault_state in ('LOWER_MACHINE_DISABLED', 'LOWER_MACHINE_STATUS_UNKNOWN')

    def _build_fault_state(self, runtime_state=None):
        runtime_state = runtime_state or {}
        runtime_fault = str(runtime_state.get('fault') or runtime_state.get('faultState') or '')
        if runtime_fault and not self._is_ignored_enable_fault_state(runtime_fault):
            return runtime_fault
        fault_state = self._get_redis_value('faultState', str, '')
        if fault_state and not self._is_ignored_enable_fault_state(fault_state):
            return fault_state
        return ''

    def _build_health_state(self, runtime_state=None):
        runtime_state = runtime_state or {}
        runtime_health = str(runtime_state.get('health') or runtime_state.get('healthState') or '')
        runtime_fault = str(runtime_state.get('fault') or runtime_state.get('faultState') or '')
        if runtime_health:
            return runtime_health
        if runtime_fault and not self._is_ignored_enable_fault_state(runtime_fault):
            return 'WARN'
        raw_fault_state = self._get_redis_value('faultState', str, '')
        health_state = self._get_redis_value('healthState', str, '')
        if self._is_ignored_enable_fault_state(raw_fault_state):
            return 'OK'
        if health_state:
            return health_state
        if self._build_fault_state():
            return 'WARN'
        return 'OK'

    def _build_detail(self, task_params=None, current_location=None, runtime_state=None):
        detail = self._read_runtime_detail()
        runtime_state = runtime_state or {}
        runtime_detail = runtime_state.get('detail')
        if isinstance(runtime_detail, dict):
            detail.update(runtime_detail)
        task_params = task_params or self._get_redis_hash('taskParams')
        current_location = current_location or self._get_redis_hash('currentLocation')
        current_lat = self._coerce_value(current_location.get('lat'), float, None)
        current_lon = self._coerce_value(current_location.get('lon'), float, None)
        current_heading = self._coerce_value(current_location.get('heading'), float, None)
        task_origin_tolerance = self._get_task_origin_tolerance_m(detail)
        distance_to_start = self._distance_to_task_origin(task_params, current_location)
        battery_percent = self._get_redis_value('batteryPercent', float, None)
        detail['lastCommandMessage'] = self._get_redis_value('lastCommandMessage', str, '')
        detail['startCheckReady'] = self._get_redis_value('startCheckReady', self._bool_value, False)
        detail['startCheckReason'] = self._get_redis_value('startCheckReason', str, '')
        detail['batteryPercent'] = battery_percent
        detail['batteryPercentRaw'] = self._get_redis_value('batteryPercentRaw', float, None)
        detail['batteryReportAt'] = self._get_optional_int('batteryReportAt')
        detail['packVoltage'] = self._get_redis_value('packVoltage', float, None)
        detail['packVoltageReportAt'] = self._get_optional_int('packVoltageReportAt')
        detail['hardwareState'] = self._get_optional_int('hardwareState')
        detail['hardwareReportAt'] = self._get_optional_int('hardwareReportAt')
        detail['hardwareReportAgeSec'] = self._hardware_report_age_sec()
        detail['distanceToStartM'] = distance_to_start
        detail['distanceToTaskOriginM'] = distance_to_start
        detail['startToleranceM'] = task_origin_tolerance
        detail['taskOriginToleranceM'] = task_origin_tolerance
        runtime_action = str(runtime_state.get('action') or self._get_redis_value('currentAction', str, ''))
        control_state = self._build_control_state(runtime_state)
        configured_task_count = self._build_task_count()
        detail['cleanTaskCount'] = visible_task_field(
            configured_task_count if configured_task_count > 0 else None,
            runtime_action,
            control_state,
        )
        detail['configuredTaskName'] = self._build_task_name()
        detail['configuredCleanTaskCount'] = configured_task_count if configured_task_count > 0 else None
        detail['commandSpeed'] = self._get_redis_value('forwardSpeed', int, None)
        detail['commandBrushSpeed'] = self._get_redis_value('brushSpeed', int, None)
        detail['waypointCount'] = self._build_waypoint_count(
            runtime_action == 'multi_go_to_point'
        )
        detail['waypointIndex'] = self._get_redis_value('waypointIndex', int, 0)
        detail['currentLat'] = current_lat
        detail['currentLon'] = current_lon
        detail['currentHeading'] = live_heading_from_location(current_lat, current_lon, current_heading)
        detail['taskStartLat'] = self._coerce_value(task_params.get('startLat'), float, None)
        detail['taskStartLon'] = self._coerce_value(task_params.get('startLon'), float, None)
        detail['originHeading'] = self._coerce_value(task_params.get('originHeading'), float, None)
        detail.update(self._build_live_rtk_runtime_detail(detail))
        return detail

    def _build_task_name(self):
        task_name = self._get_redis_value('currentTaskName', str, '')
        if task_name:
            return task_name
        try:
            task_config = self._load_task_config()
            return task_config.get('taskName', '')
        except Exception:
            return ''

    def _build_task_count(self):
        try:
            task_config = self._load_task_config()
            task_list = task_config.get('taskList', [])
            if isinstance(task_list, list) and len(task_list) > 0:
                return len(task_list)
        except Exception:
            pass

        try:
            return self.redis_client.llen('taskList')
        except Exception:
            return 0

    def _build_waypoint_count(self, prefer_runtime_total=False):
        if prefer_runtime_total:
            runtime_total = self._get_redis_value('waypointTotal', int, 0)
            if runtime_total > 0:
                return runtime_total
        try:
            count = int(self.redis_client.llen('waypoints'))
            if count > 0:
                return count
        except Exception:
            pass
        return self._get_redis_value('waypointTotal', int, 0)

    def _load_task_config(self):
        with open('config.json', 'r') as fp:
            return json.load(fp)

    def publish_vehicle_status(self):
        """
        周期性发布完整设备状态。

        MQTT 消息 type=vehicle_status，data 为 _get_vehicle_status_from_redis 组装的完整字段。
        云平台用它更新设备在线状态、电量、速度、当前任务和故障信息。
        该上报包含字段多，因此保持原有较低频率。
        """
        try:
            status = self._get_vehicle_status_from_redis()

            if status:
                self.mqtt_client.publish_status({
                    'type': 'vehicle_status',
                    'data': status
                })
                logger.debug(
                    "车辆状态已发布: task={task}, index={index}, lat={lat}, lon={lon}".format(
                        task=status.get('task_name'),
                        index=status.get('cur_task_index'),
                        lat=status.get('lat'),
                        lon=status.get('lon')
                    )
                )
        except Exception as e:
            logger.error("发布车辆状态失败: {}".format(str(e)), exc_info=True)

    def _get_vehicle_position_from_redis(self):
        """
        读取最新 RTK 位置，并换算为前端路线图使用的相对坐标。

        这是高频实时位置消息的最小数据集，只包含 local_x/local_y，
        不重复上报电量、任务列表等大量低频字段。
        """
        if not self.redis_client:
            return {}

        try:
            current_location = self._get_redis_hash('currentLocation')
            lat = self._coerce_value(current_location.get('lat'), float, None)
            lon = self._coerce_value(current_location.get('lon'), float, None)
            if lat is None or lon is None:
                return {}

            runtime_state = self._read_runtime_state()
            current_action = runtime_state.get('action') or self._get_redis_value('currentAction', str, '')
            local_x, local_y = self._compute_local_xy_cm(lat, lon, runtime_state, current_action)
            if local_x is None or local_y is None:
                return {}

            return {
                'local_x': local_x,
                'local_y': local_y,
            }
        except Exception as e:
            logger.error("读取车辆实时相对位置失败: {}".format(str(e)), exc_info=True)
            return {}

    def publish_vehicle_position(self):
        """
        独立高频发布小车相对位置，不改变原有完整状态上报频率。

        MQTT 消息 type=vehicle_position，data 中只有 local_x/local_y。
        前端每次收到后更新路线图上的机器人图标，实现清扫进度动画。
        """
        try:
            position = self._get_vehicle_position_from_redis()
            if position:
                self.mqtt_client.publish_realtime({
                    'type': 'vehicle_position',
                    'data': position,
                })
        except Exception as e:
            logger.error("发布车辆实时相对位置失败: {}".format(str(e)), exc_info=True)

    def _status_publish_loop(self):
        logger.info("状态上报线程已启动，间隔 {} 秒".format(self.status_interval))

        while self.running:
            try:
                if not self.mqtt_client.ensure_connected():
                    time.sleep(1)
                    continue

                self.publish_vehicle_status()
                time.sleep(self.status_interval)
            except Exception as e:
                logger.error("状态上报循环异常: {}".format(str(e)), exc_info=True)
                time.sleep(1)

    def _position_publish_loop(self):
        """实时位置独立线程：断线时尝试重连，连接正常时按 position_interval 循环上报。"""
        logger.info("实时相对位置上报线程已启动，间隔 {} 秒".format(self.position_interval))

        while self.running:
            try:
                if not self.mqtt_client.ensure_connected():
                    time.sleep(1)
                    continue

                self.publish_vehicle_position()
                time.sleep(self.position_interval)
            except Exception as e:
                logger.error("实时相对位置上报循环异常: {}".format(str(e)), exc_info=True)
                time.sleep(1)

    def _heartbeat_loop(self):
        logger.info("MQTT心跳监测线程已启动，间隔 {} 秒".format(self.heartbeat_interval))

        while self.running:
            try:
                if not self.mqtt_client.is_connected():
                    self.mqtt_client.ensure_connected()
                    time.sleep(1)
                    continue

                last_publish_at = self.mqtt_client.get_last_publish_at()
                now = time.time()

                if last_publish_at <= 0 or (now - last_publish_at) >= self.heartbeat_interval:
                    logger.info("MQTT心跳触发，补发车辆状态")
                    self.publish_vehicle_status()

                time.sleep(self.heartbeat_check_interval)
            except Exception as e:
                logger.error("MQTT心跳循环异常: {}".format(str(e)), exc_info=True)
                time.sleep(1)

    def start(self):
        try:
            self.running = True
            initial_connected = self.mqtt_client.connect()
            if not initial_connected:
                logger.warning("MQTT初始连接失败，已进入后台重连模式")

            self.status_thread = threading.Thread(target=self._status_publish_loop)
            self.status_thread.daemon = True
            self.status_thread.start()

            self.position_thread = threading.Thread(target=self._position_publish_loop)
            self.position_thread.daemon = True
            self.position_thread.start()

            self.heartbeat_thread = threading.Thread(target=self._heartbeat_loop)
            self.heartbeat_thread.daemon = True
            self.heartbeat_thread.start()

            logger.info("MQTT集成已启动")
            return True
        except Exception as e:
            logger.error("启动MQTT集成失败: {}".format(str(e)), exc_info=True)
            return False

    def stop(self):
        try:
            self.running = False
            if self.status_thread:
                self.status_thread.join(timeout=5)
            if self.position_thread:
                self.position_thread.join(timeout=5)
            if self.heartbeat_thread:
                self.heartbeat_thread.join(timeout=5)

            self.mqtt_client.disconnect()

            logger.info("MQTT集成已停止")
        except Exception as e:
            logger.error("停止MQTT集成失败: {}".format(str(e)), exc_info=True)

    def is_connected(self):
        return self.mqtt_client.is_connected()

    def publish_event(self, event_type, event_data):
        return self.mqtt_client.publish_status({
            'type': 'event',
            'event_type': event_type,
            'data': event_data,
            'timestamp': int(time.time())
        })


_mqtt_integration = None


def init_mqtt(config, vehicle_controller, redis_client=None):
    global _mqtt_integration

    if _mqtt_integration is None:
        _mqtt_integration = MQTTIntegration(config, vehicle_controller, redis_client)
        _mqtt_integration.start()

    return _mqtt_integration


def get_mqtt_integration():
    return _mqtt_integration
