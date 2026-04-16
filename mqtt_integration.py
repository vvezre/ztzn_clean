#!/usr/bin/env python
# coding=utf-8
"""
MQTT 集成模块
将 MQTT 功能集成到现有系统，提供车辆状态上报和远程控制功能。
"""

import json
import threading
import time

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
        self.running = False
        self.status_interval = config.get('mqtt', {}).get('status_interval', 5)

        logger.info("MQTT集成模块初始化完成")

    def _on_mqtt_message(self, message_data):
        try:
            result = self.command_handler.handle(message_data)
            self._publish_command_result(message_data.get('command'), result)

            if result.get('success'):
                self.publish_vehicle_status()
        except Exception as e:
            logger.error("处理MQTT消息异常: {}".format(str(e)), exc_info=True)

    def _publish_command_result(self, command, result):
        try:
            self.mqtt_client.publish_status({
                'type': 'command_result',
                'command': command,
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
            status = {
                'speed': self._get_redis_value('forwardSpeed', int, 0),
                'brush_speed': self._get_redis_value('brushSpeed', int, 0),
                'voltage': self._get_redis_value('voltage', float, 0.0),
                'lat': self._coerce_value(current_location.get('lat'), float, None),
                'lon': self._coerce_value(current_location.get('lon'), float, None),
                'heading': self._coerce_value(current_location.get('heading'), float, None),
                'status': self._build_status(),
                'action': self._build_action(),
                'task_name': self._build_task_name(),
                'cur_task_index': self._get_redis_value('curTaskIndex', int, 0),
                'task_count': self._build_task_count(),
                'timestamp': int(time.time())
            }

            return status
        except Exception as e:
            logger.error("从Redis获取车辆状态失败: {}".format(str(e)), exc_info=True)
            return {}

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
                self.publish_vehicle_status()
                time.sleep(self.status_interval)
            except Exception as e:
                logger.error("状态上报循环异常: {}".format(str(e)), exc_info=True)
                time.sleep(1)

    def start(self):
        try:
            if not self.mqtt_client.connect():
                logger.error("MQTT连接失败，集成启动失败")
                return False

            self.running = True
            self.status_thread = threading.Thread(target=self._status_publish_loop)
            self.status_thread.start()

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
