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
        self.heartbeat_thread = None
        self.running = False
        self.status_interval = config.get('mqtt', {}).get('status_interval', 5)
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
            logger.error("鍙戝竷鍛戒护ACK澶辫触: {}".format(str(e)), exc_info=True)

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
        if not self.redis_client:
            return {}

        try:
            current_location = self._get_redis_hash('currentLocation')
            lat = self._coerce_value(current_location.get('lat'), float, None)
            lon = self._coerce_value(current_location.get('lon'), float, None)
            local_x, local_y = self._compute_local_xy_cm(lat, lon)
            status = {
                'speed': self._get_redis_value('forwardSpeed', int, 0),
                'brush_speed': self._get_redis_value('brushSpeed', int, 0),
                'voltage': self._get_redis_value('packVoltage', float, None),
                'lat': lat,
                'lon': lon,
                'heading': self._coerce_value(current_location.get('heading'), float, None),
                'local_x': local_x,
                'local_y': local_y,
                'status': self._build_status(),
                'action': self._build_action(),
                'task_name': self._build_task_name(),
                'cur_task_index': self._get_redis_value('curTaskIndex', int, 0),
                'task_count': self._build_task_count(),
                'battery': self._get_redis_value('batteryPercent', float, None),
                'battery_percent': self._get_redis_value('batteryPercent', float, None),
                'battery_raw': self._get_redis_value('batteryPercentRaw', float, None),
                'battery_percent_raw': self._get_redis_value('batteryPercentRaw', float, None),
                'pack_voltage': self._get_redis_value('packVoltage', float, None),
                'online_state': 'ONLINE',
                'mission_state': self._build_mission_state(),
                'control_state': self._build_control_state(),
                'health_state': self._build_health_state(),
                'fault_state': self._build_fault_state(),
                'tracking': self._get_redis_value('correct', self._bool_value, False),
                'path_planning': self._get_redis_value('pathPlanning', str, ''),
                'move_judge': self._get_redis_value('moveJudge', self._bool_value, False),
                'detect_qrcode': self._get_redis_value('detectQrcode', self._bool_value, False),
                'enter_garage': self._get_redis_value('enterGarage', self._bool_value, False),
                'supported_actions': [
                    'auto_drive', 'go_on', 'stop', 'parking', 'return_to_point', 'get_status', 'get_task_path'
                ],
                'supported_params': ['taskName', 'speed', 'tracking', 'path'],
                'supported_status_fields': [
                    'control_state', 'health_state', 'fault_state', 'mission_state', 'detail'
                ],
                'detail': self._build_detail(),
                'timestamp': int(time.time())
            }

            return status
        except Exception as e:
            logger.error("从Redis获取车辆状态失败: {}".format(str(e)), exc_info=True)
            return {}

    def _compute_local_xy_cm(self, lat, lon):
        if lat is None or lon is None:
            return None, None
        try:
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
            return json.loads(raw)
        except Exception:
            return {}

    def _build_status(self):
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

    def _build_action(self):
        current_action = self._get_redis_value('currentAction', str, '')
        if current_action:
            return current_action
        if self._get_redis_value('mission', str, '') == 'working':
            return 'auto_drive'
        if self._get_redis_value('parking', str, '0') == '1':
            return 'parking'
        return 'idle'

    def _build_mission_state(self):
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

    def _build_control_state(self):
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

    def _build_fault_state(self):
        fault_state = self._get_redis_value('faultState', str, '')
        if fault_state and not self._is_ignored_enable_fault_state(fault_state):
            return fault_state
        return ''

    def _build_health_state(self):
        raw_fault_state = self._get_redis_value('faultState', str, '')
        if self._build_fault_state():
            return 'WARN'
        health_state = self._get_redis_value('healthState', str, '')
        if self._is_ignored_enable_fault_state(raw_fault_state):
            return 'OK'
        if health_state:
            return health_state
        return 'OK'

    def _build_detail(self):
        detail = self._read_runtime_detail()
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

    def _load_task_config(self):
        with open('config.json', 'r') as fp:
            return json.load(fp)

    def publish_vehicle_status(self):
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
