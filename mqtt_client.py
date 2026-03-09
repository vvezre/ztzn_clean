#!/usr/bin/env python
# coding=utf-8
"""
MQTT客户端模块
用于与MQTT平台通信，接收控制指令并上报车辆状态
"""

import json
import time
import threading
import paho.mqtt.client as mqtt
from AppLogger import logger


class MQTTClient:
    """MQTT客户端类"""

    def __init__(self, config):
        """
        初始化MQTT客户端

        Args:
            config: MQTT配置字典，包含以下字段：
                - broker: MQTT服务器地址
                - port: MQTT服务器端口
                - username: 用户名
                - password: 密码
                - company_code: 公司代号 (8字节)
                - product_model: 产品型号 (4字节)
                - product_id: 产品编号 (6字节)
                - keepalive: 心跳时间间隔
                - qos: 消息质量等级 (0, 1, 2)
        """
        self.config = config
        self.client = None
        self.connected = False
        self.message_callback = None

        # 构建主题
        topic_suffix = "{}{}".format(config['mqtt']['product_model'],config['mqtt']['product_id'])
        self.subscribe_topic = "RAILCAR/S/{}".format(topic_suffix)  # 后端订阅主题（接收控制指令）
        self.publish_topic = "RAILCAR/R/{}".format(topic_suffix)  # 后端发布主题（上报状态）

        logger.info("MQTT主题配置 - 订阅: {}, 发布: {}".format(self.subscribe_topic,self.publish_topic))

        # 初始化MQTT客户端
        self._init_client()

    def _init_client(self):
        """初始化MQTT客户端"""
        client_id = self.config['mqtt']['product_id']
        self.client = mqtt.Client(client_id=client_id)

        # 设置用户名密码
        if self.config.get('mqtt').get('username') and self.config.get('mqtt').get('password'):
            self.client.username_pw_set(
                self.config['mqtt']['username'],
                self.config['mqtt']['password']
            )

        # 设置回调函数
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message

        # 设置遗嘱消息（连接异常断开时发送）
        will_payload = self._build_message({
            'status': 'offline',
            'timestamp': int(time.time())
        })
        self.client.will_set(
            self.publish_topic,
            will_payload,
            qos=self.config.get('qos', 1),
            retain=True
        )

    def _on_connect(self, client, userdata, flags, rc):
        """连接回调"""
        if rc == 0:
            self.connected = True
            logger.info("MQTT连接成功 - Broker: {}:{}".format(self.config['mqtt']['broker'],self.config['mqtt']['port']))

            # 订阅控制主题
            result, mid = client.subscribe(self.subscribe_topic, qos=self.config.get('mqtt').get('qos', 1))
            if result == mqtt.MQTT_ERR_SUCCESS:
                logger.info("订阅主题成功: {}".format(self.subscribe_topic))
            else:
                logger.error("订阅主题失败: {}, 错误码: {}".format(self.subscribe_topic,result))

            # 发布上线消息
            self.publish_status({
                'status': 'online',
                'timestamp': int(time.time())
            })
        else:
            self.connected = False
            error_msg = {
                1: "协议版本错误",
                2: "无效的客户端ID",
                3: "服务器不可用",
                4: "用户名或密码错误",
                5: "未授权"
            }
            logger.error("MQTT连接失败 - 错误码: {}, 原因: {}".format(rc,error_msg.get(rc, '未知错误')))

    def _on_disconnect(self, client, userdata, rc):
        """断开连接回调"""
        self.connected = False
        if rc != 0:
            logger.warning("MQTT意外断开 - 错误码: {}, 将尝试重连...".format(rc))
        else:
            logger.info("MQTT正常断开连接")

    def _on_message(self, client, userdata, msg):
        """消息接收回调"""
        try:
            payload = msg.payload.decode('utf-8')
            logger.info("收到MQTT消息 - Topic: {}, Payload: {}".format(msg.topic,payload))

            # 解析消息
            message_data = self._parse_message(payload)

            # 调用外部回调函数处理消息
            if self.message_callback:
                self.message_callback(message_data)
            else:
                logger.warning("未设置消息处理回调函数")

        except Exception as e:
            logger.error("处理MQTT消息失败: {}".format(str(e)), exc_info=True)

    def _build_message(self, data):
        """
        构建MQTT消息

        Args:
            data: 消息数据字典

        Returns:
            bytes: 消息负载（JSON格式）
        """
        message = {
            'company_code': self.config['mqtt']['company_code'],
            'product_model': self.config['mqtt']['product_model'],
            'product_id': self.config['mqtt']['product_id'],
            'timestamp': int(time.time()),
            'data': data
        }
        return json.dumps(message, ensure_ascii=False).encode('utf-8')

    def _parse_message(self, payload):
        """
        解析接收到的MQTT消息

        Args:
            payload: 消息负载字符串

        Returns:
            dict: 解析后的消息数据
        """
        try:
            message = json.loads(payload)
            return message.get('data', message)
        except json.JSONDecodeError:
            logger.warning("消息不是有效的JSON格式，原样返回: {}".format(payload))
            return {'raw': payload}

    def connect(self):
        """连接到MQTT服务器"""
        try:
            logger.info("正在连接MQTT服务器: {}:{}".format(self.config['mqtt']['broker'],self.config['mqtt']['port']))
            self.client.connect(
                self.config['mqtt']['broker'],
                self.config['mqtt']['port'],
                keepalive=self.config.get('mqtt').get('keepalive', 60)
            )

            # 在后台线程中运行网络循环
            self.client.loop_start()

            # 等待连接建立
            retry_count = 0
            while not self.connected and retry_count < 10:
                time.sleep(0.5)
                retry_count += 1

            if self.connected:
                logger.info("MQTT客户端已启动")
                return True
            else:
                logger.error("MQTT连接超时")
                return False

        except Exception as e:
            logger.error("MQTT连接异常: {}".format(str(e)), exc_info=True)
            return False

    def disconnect(self):
        """断开MQTT连接"""
        try:
            # 发布下线消息
            self.publish_status({
                'status': 'offline',
                'timestamp': int(time.time())
            })

            time.sleep(0.5)  # 等待消息发送完成

            self.client.loop_stop()
            self.client.disconnect()
            logger.info("MQTT客户端已断开")
        except Exception as e:
            logger.error("断开MQTT连接失败: {}".format(str(e)), exc_info=True)

    def publish(self, topic, data, qos=None, retain=False):
        """
        发布消息到指定主题

        Args:
            topic: 主题
            data: 消息数据（字典）
            qos: 消息质量等级
            retain: 是否保留消息

        Returns:
            bool: 发布是否成功
        """
        if not self.connected:
            logger.warning("MQTT未连接，无法发布消息")
            return False

        try:
            payload = self._build_message(data)
            qos = qos if qos is not None else self.config.get('qos', 1)

            result = self.client.publish(topic, payload, qos=qos, retain=retain)

            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                logger.debug("消息发布成功 - Topic: {}".format(topic))
                return True
            else:
                logger.error("消息发布失败 - Topic: {}, 错误码: {}".format(topic,result.rc))
                return False

        except Exception as e:
            logger.error("发布消息异常: {}".format(str(e)), exc_info=True)
            return False

    def publish_status(self, status_data):
        """
        发布车辆状态到发布主题

        Args:
            status_data: 状态数据字典

        Returns:
            bool: 发布是否成功
        """
        return self.publish(self.publish_topic, status_data, retain=True)

    def set_message_callback(self, callback):
        """
        设置消息处理回调函数

        Args:
            callback: 回调函数，接收一个参数（消息数据字典）
        """
        self.message_callback = callback
        logger.info("消息回调函数已设置")

    def is_connected(self):
        """检查是否已连接"""
        return self.connected


# 单例模式
_mqtt_client_instance = None


def get_mqtt_client(config=None):
    """
    获取MQTT客户端单例

    Args:
        config: MQTT配置（仅在首次调用时需要）

    Returns:
        MQTTClient: MQTT客户端实例
    """
    global _mqtt_client_instance

    if _mqtt_client_instance is None:
        if config is None:
            raise ValueError("首次调用需要提供配置参数")
        _mqtt_client_instance = MQTTClient(config)

    return _mqtt_client_instance