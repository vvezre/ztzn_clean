# coding=utf-8
import paho.mqtt.client as mqtt
import time

from AppLogger import logger


class MqttClient:
    def __init__(
            self,
            broker,
            port=1883,
            client_id="",
            username=None,
            password=None,
            keepalive=60,
            on_message_callback=None,
    ):
        """
        初始化 MQTT 客户端

        :param broker: MQTT 服务器地址
        :param port: 端口，默认 1883
        :param client_id: 客户端 ID（可为空，由库自动生成）
        :param username: 用户名（可选）
        :param password: 密码（可选）
        :param keepalive: 保活时间（秒）
        :param on_message_callback: 自定义消息处理函数，签名为 func(topic: str, payload: str)
        """
        self.broker = broker
        self.port = port
        self.keepalive = keepalive
        self.on_message_callback = on_message_callback or self._default_on_message

        # 创建客户端
        self.client = mqtt.Client(client_id=client_id)
        if username and password:
            self.client.username_pw_set(username, password)

        # 绑定回调
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message

        self.connected = False

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            logger.info("✅ 成功连接到 MQTT Broker: {}:{}".format(self.broker,self.port))
            self.connected = True
        else:
            logger.error("❌ 连接失败，错误码: {}".format(rc))

    def _on_disconnect(self, client, userdata, rc):
        self.connected = False
        logger.warning("🔌 MQTT 连接已断开")
        if rc != 0:
            logger.info("🔄 尝试自动重连...")
            self.reconnect()

    def _on_message(self, client, userdata, msg):
        try:
            topic = msg.topic
            payload = msg.payload.decode()
            self.on_message_callback(topic, payload)
        except Exception as e:
            logger.error("处理消息时出错: {}".format(e))

    def _default_on_message(self, topic, payload):
        logger.info("📬 收到消息 - 主题: {} | 内容: {}".format(topic,payload))

    def connect(self):
        """连接到 MQTT Broker"""
        try:
            self.client.connect(self.broker, self.port, self.keepalive)
            self.client.loop_start()
            # 等待连接完成（最多 5 秒）
            for _ in range(10):
                if self.connected:
                    break
                time.sleep(0.5)
            else:
                logger.error("⚠️ 连接超时")
        except Exception as e:
            logger.error("连接异常: {}".format(e))

    def reconnect(self):
        """重新连接"""
        try:
            self.client.reconnect()
            self.client.loop_start()
        except Exception as e:
            logger.error("重连失败: {}".format(e))

    def subscribe(self, topic, qos=0):
        """订阅主题"""
        if self.connected:
            self.client.subscribe(topic, qos=0)
            logger.info("📡 已订阅主题: {} (QoS={})".format(topic,qos))
        else:
            logger.warning("⚠️ 未连接，无法订阅主题")

    def publish(self, topic, payload, qos=0, retain = False):
        """发布消息"""
        if self.connected:
            result = self.client.publish(topic, payload, qos=qos, retain=retain)
            status = "成功" if result.rc == mqtt.MQTT_ERR_SUCCESS else "失败"
            logger.info("📤 发布 {} - 主题: {} | 内容: {}".format(status,topic,payload))
            return result
        else:
            logger.warning("⚠️ 未连接，无法发布消息")
            return None

    def disconnect(self):
        """断开连接"""
        self.client.loop_stop()
        self.client.disconnect()
        self.connected = False
        logger.info("🛑 MQTT 客户端已断开")

    def is_connected(self):
        """检查是否已连接"""
        return self.connected