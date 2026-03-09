#!/usr/bin/env python
# coding=utf-8
"""
MQTT集成模块
将MQTT功能集成到现有系统，提供车辆状态上报和远程控制功能
"""

import json
import time
import threading
from mqtt_client import MQTTClient, get_mqtt_client
from mqtt_handler import MQTTCommandHandler
from AppLogger import logger


class MQTTIntegration:
    """MQTT集成类"""

    def __init__(self, config, vehicle_controller, redis_client=None):
        """
        初始化MQTT集成

        Args:
            config: MQTT配置字典
            vehicle_controller: 车辆控制器对象
            redis_client: Redis客户端（可选，用于获取车辆状态）
        """
        self.config = config
        self.vehicle_controller = vehicle_controller
        self.redis_client = redis_client

        # 创建MQTT客户端
        self.mqtt_client = MQTTClient(config)

        # 创建命令处理器
        self.command_handler = MQTTCommandHandler(vehicle_controller)

        # 设置消息回调
        self.mqtt_client.set_message_callback(self._on_mqtt_message)

        # 状态上报线程
        self.status_thread = None
        self.running = False

        # 状态上报间隔（秒）
        self.status_interval = config.get('status_interval', 5)

        logger.info("MQTT集成模块初始化完成")

    def _on_mqtt_message(self, message_data):
        """
        MQTT消息接收回调

        Args:
            message_data: 接收到的消息数据
        """
        try:
            # 使用命令处理器处理消息
            result = self.command_handler.handle_message(message_data)

            # 发布命令执行结果
            self._publish_command_result(message_data.get('command'), result)

            # 立即上报一次状态
            if result.get('success'):
                self.publish_vehicle_status()

        except Exception as e:
            logger.error("处理MQTT消息异常: {}".format(str(e)), exc_info=True)

    def _publish_command_result(self, command, result):
        """
        发布命令执行结果

        Args:
            command: 命令名称
            result: 执行结果
        """
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
        """
        从Redis获取车辆状态

        Returns:
            dict: 车辆状态数据
        """
        if not self.redis_client:
            return {}

        try:
            status = {
                # 基础状态
                'speed': self._get_redis_value('forwardSpeed', int, 0),
                'brush_speed': self._get_redis_value('brushSpeed', int, 0),
                'tracking': self._get_redis_value('correct', str, 'false') == 'true',
                'path_planning': self._get_redis_value('pathPlanning', str, 'left'),

                # 传感器数据
                'voltage': self._get_redis_value('voltage', float, 0.0),
                'battery_percent': self._get_redis_value('battery_percent', float, 0.0),
                'angle': self._get_redis_value('angle', float, 0.0),

                # 边缘检测
                'left_edge': self._get_redis_value('leftEdge', int, 0),
                'right_edge': self._get_redis_value('rightEdge', int, 0),

                # 状态标志
                'move_judge': self._get_redis_value('moveJudge', str, 'false') == 'true',
                'detect_qrcode': self._get_redis_value('detectQrcode', str, 'false') == 'true',
                'enter_garage': self._get_redis_value('enterGarage', str, 'false') == 'true',

                # 时间戳
                'timestamp': int(time.time())
            }

            return status

        except Exception as e:
            logger.error("从Redis获取状态失败: {}".format(str(e)), exc_info=True)
            return {}

    def _get_redis_value(self, key, value_type, default):
        """
        从Redis获取值并转换类型

        Args:
            key: Redis键
            value_type: 目标类型（int, float, str）
            default: 默认值

        Returns:
            转换后的值或默认值
        """
        try:
            value = self.redis_client.get(key)
            if value is None:
                return default

            if isinstance(value, bytes):
                value = value.decode('utf-8')

            return value_type(value)
        except Exception:
            return default

    def publish_vehicle_status(self):
        """发布车辆状态"""
        try:
            # 获取车辆状态
            status = self._get_vehicle_status_from_redis()

            if status:
                # 发布状态
                self.mqtt_client.publish_status({
                    'type': 'vehicle_status',
                    'data': status
                })
                logger.debug("车辆状态已发布: battery={}%".format(status.get('battery_percent')))

        except Exception as e:
            logger.error("发布车辆状态失败: {}".format(str(e)), exc_info=True)

    def _status_publish_loop(self):
        """状态上报循环（在后台线程中运行）"""
        logger.info("状态上报线程已启动，间隔: {}秒".format(self.status_interval))

        while self.running:
            try:
                self.publish_vehicle_status()
                time.sleep(self.status_interval)
            except Exception as e:
                logger.error("状态上报循环异常: {}".format(str(e)), exc_info=True)
                time.sleep(1)

    def start(self):
        """启动MQTT集成"""
        try:
            # 连接MQTT
            if not self.mqtt_client.connect():
                logger.error("MQTT连接失败，集成启动失败")
                return False

            # 启动状态上报线程
            self.running = True
            self.status_thread = threading.Thread(target=self._status_publish_loop)
            self.status_thread.start()

            logger.info("MQTT集成已启动")
            return True

        except Exception as e:
            logger.error("启动MQTT集成失败: {}".format(str(e)), exc_info=True)
            return False

    def stop(self):
        """停止MQTT集成"""
        try:
            # 停止状态上报线程
            self.running = False
            if self.status_thread:
                self.status_thread.join(timeout=5)

            # 断开MQTT连接
            self.mqtt_client.disconnect()

            logger.info("MQTT集成已停止")

        except Exception as e:
            logger.error("停止MQTT集成失败: {}".format(str(e)), exc_info=True)

    def is_connected(self):
        """检查MQTT是否已连接"""
        return self.mqtt_client.is_connected()

    def publish_event(self, event_type, event_data):
        """
        发布事件消息

        Args:
            event_type: 事件类型
            event_data: 事件数据

        Returns:
            bool: 发布是否成功
        """
        return self.mqtt_client.publish_status({
            'type': 'event',
            'event_type': event_type,
            'data': event_data,
            'timestamp': int(time.time())
        })


# 全局MQTT集成实例
_mqtt_integration = None


def init_mqtt(config, vehicle_controller, redis_client=None):
    """
    初始化MQTT集成（单例模式）

    Args:
        config: MQTT配置
        vehicle_controller: 车辆控制器
        redis_client: Redis客户端

    Returns:
        MQTTIntegration: MQTT集成实例
    """
    global _mqtt_integration

    if _mqtt_integration is None:
        _mqtt_integration = MQTTIntegration(config, vehicle_controller, redis_client)
        _mqtt_integration.start()

    return _mqtt_integration


def get_mqtt_integration():
    """
    获取MQTT集成实例

    Returns:
        MQTTIntegration: MQTT集成实例
    """
    return _mqtt_integration