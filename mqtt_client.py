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

try:
    text_type = unicode
    binary_type = str
except NameError:
    text_type = str
    binary_type = bytes



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
        self.loop_started = False
        self.stop_event = threading.Event()
        self.connect_lock = threading.Lock()
        self.reconnect_lock = threading.Lock()
        self.reconnect_thread = None
        self._intentional_disconnect = False

        mqtt_config = config.get('mqtt', {})
        self.keepalive = mqtt_config.get('keepalive', 60)
        self.auto_reconnect = mqtt_config.get('auto_reconnect', True)
        self.reconnect_delay = max(1, int(mqtt_config.get('reconnect_delay', 5)))
        self.max_reconnect_delay = max(
            self.reconnect_delay,
            int(mqtt_config.get('max_reconnect_delay', 60))
        )
        self.connect_wait_timeout = max(1.0, float(mqtt_config.get('connect_wait_timeout', 5)))

        now = time.time()
        self.last_activity_at = now
        self.last_publish_at = 0
        self.last_receive_at = 0
        self.last_connect_at = 0
        self.last_connect_attempt_at = 0

        # 构建主题，优先使用显式配置，回退到 product_model + product_id
        topics = config.get('topics', {})
        topic_suffix = "{}{}".format(
            config['mqtt'].get('product_model', ''),
            config['mqtt'].get('product_id', '')
        )
        self.subscribe_topic = topics.get('subscribe') or "RAILCAR/S/{}".format(topic_suffix)  # 后端订阅主题（接收控制指令）
        self.publish_topic = topics.get('publish') or "RAILCAR/R/{}".format(topic_suffix)  # 后端发布主题（上报状态）

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
        will_payload = self._build_message(self._build_presence_data('offline'))
        self.client.will_set(
            self.publish_topic,
            will_payload,
            qos=self.config.get('mqtt', {}).get('qos', 1),
            retain=True
        )

    def _build_presence_data(self, state):
        """构建在线状态消息体。"""
        timestamp = int(time.time())
        return {
            'type': state,
            'status': state,
            'timestamp': timestamp
        }

    def _mark_activity(self, publish=False, receive=False, connected=False):
        now = time.time()
        self.last_activity_at = now
        if publish:
            self.last_publish_at = now
        if receive:
            self.last_receive_at = now
        if connected:
            self.last_connect_at = now

    def _start_network_loop(self):
        if not self.loop_started:
            self.client.loop_start()
            self.loop_started = True

    def _wait_for_connection(self, timeout=None):
        deadline = time.time() + (timeout if timeout is not None else self.connect_wait_timeout)
        while not self.connected and time.time() < deadline and not self.stop_event.is_set():
            time.sleep(0.2)
        return self.connected

    def _connect_once(self):
        if self.connected:
            return True

        with self.connect_lock:
            if self.connected:
                return True

            self.last_connect_attempt_at = time.time()
            try:
                logger.info(
                    "正在连接MQTT服务器: {}:{}".format(
                        self.config['mqtt']['broker'],
                        self.config['mqtt']['port']
                    )
                )
                self._intentional_disconnect = False
                if self.loop_started and self.last_connect_at > 0:
                    self.client.reconnect()
                else:
                    self.client.connect(
                        self.config['mqtt']['broker'],
                        self.config['mqtt']['port'],
                        keepalive=self.keepalive
                    )
                self._start_network_loop()
            except Exception as e:
                logger.error("MQTT连接异常: {}".format(str(e)), exc_info=True)
                return False

        if self._wait_for_connection():
            logger.info("MQTT客户端已启动")
            return True

        logger.error("MQTT连接超时")
        return False

    def _schedule_reconnect(self, immediate=False):
        if not self.auto_reconnect or self.stop_event.is_set() or self._intentional_disconnect:
            return False

        with self.reconnect_lock:
            if self.reconnect_thread and self.reconnect_thread.is_alive():
                return False

            self.reconnect_thread = threading.Thread(
                target=self._reconnect_loop,
                args=(immediate,)
            )
            self.reconnect_thread.daemon = True
            self.reconnect_thread.start()
            return True

    def _reconnect_loop(self, immediate=False):
        delay = 0 if immediate else self.reconnect_delay

        while not self.stop_event.is_set():
            if self.connected:
                return

            if delay > 0:
                logger.info("MQTT将在 {} 秒后重连".format(delay))
                if self.stop_event.wait(delay):
                    return

            if self._connect_once():
                logger.info("MQTT自动重连成功")
                return

            delay = self.reconnect_delay if delay <= 0 else min(delay * 2, self.max_reconnect_delay)
            logger.warning("MQTT重连失败，继续重试")

    def _on_connect(self, client, userdata, flags, rc):
        """连接回调"""
        if rc == 0:
            self.connected = True
            self._mark_activity(connected=True)
            logger.info("MQTT连接成功 - Broker: {}:{}".format(self.config['mqtt']['broker'],self.config['mqtt']['port']))

            # 订阅控制主题
            result, mid = client.subscribe(self.subscribe_topic, qos=self.config.get('mqtt').get('qos', 1))
            if result == mqtt.MQTT_ERR_SUCCESS:
                logger.info("订阅主题成功: {}".format(self.subscribe_topic))
            else:
                logger.error("订阅主题失败: {}, 错误码: {}".format(self.subscribe_topic,result))

            # 发布上线消息
            self.publish_status(self._build_presence_data('online'))
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
            self._schedule_reconnect(immediate=True)
        else:
            logger.info("MQTT正常断开连接")

    def _on_message(self, client, userdata, msg):
        """消息接收回调"""
        try:
            payload = msg.payload.decode('utf-8')
            self._mark_activity(receive=True)
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
        normalized = self._normalize_json_value(message)
        return json.dumps(normalized, ensure_ascii=False).encode('utf-8')

    def _normalize_json_value(self, value):
        if isinstance(value, dict):
            result = {}
            for key, item in value.items():
                result[self._normalize_json_value(key)] = self._normalize_json_value(item)
            return result
        if isinstance(value, list):
            return [self._normalize_json_value(item) for item in value]
        if isinstance(value, tuple):
            return [self._normalize_json_value(item) for item in value]
        if isinstance(value, binary_type) and not isinstance(value, text_type):
            try:
                return value.decode('utf-8')
            except Exception:
                return value.decode('utf-8', 'replace')
        return value


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
        except (ValueError, TypeError):
            logger.warning("消息不是有效的JSON格式，原样返回: {}".format(payload))
            return {'raw': payload}

    def connect(self):
        """连接到MQTT服务器"""
        self.stop_event.clear()
        success = self._connect_once()
        if not success:
            self._schedule_reconnect(immediate=False)
        return success

    def ensure_connected(self):
        """确保连接存在；若已断开则进入后台重连。"""
        if self.connected:
            return True
        self._schedule_reconnect(immediate=True)
        return False

    def disconnect(self):
        """断开MQTT连接"""
        try:
            self._intentional_disconnect = True
            self.stop_event.set()

            if self.connected:
                # 发布下线消息
                self.publish_status(self._build_presence_data('offline'))
                time.sleep(0.5)  # 等待消息发送完成

            if self.loop_started:
                self.client.loop_stop()
                self.loop_started = False

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
            self.ensure_connected()
            return False

        try:
            payload = self._build_message(data)
            qos = qos if qos is not None else self.config.get('mqtt', {}).get('qos', 1)

            result = self.client.publish(topic, payload, qos=qos, retain=retain)

            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                self._mark_activity(publish=True)
                logger.debug("消息发布成功 - Topic: {}".format(topic))
                return True
            else:
                logger.error("消息发布失败 - Topic: {}, 错误码: {}".format(topic,result.rc))
                if result.rc == mqtt.MQTT_ERR_NO_CONN:
                    self.connected = False
                    self.ensure_connected()
                return False

        except Exception as e:
            logger.error("发布消息异常: {}".format(str(e)), exc_info=True)
            self.connected = False
            self.ensure_connected()
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

    def get_last_publish_at(self):
        return self.last_publish_at

    def get_last_activity_at(self):
        return self.last_activity_at


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
