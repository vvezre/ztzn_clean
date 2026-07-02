# coding=utf-8
"""
启动门禁模块
程序启动时向云平台 MQTT 读取 disabled 字段：
  - disabled == 1  → 退出进程
  - disabled == 0  → 继续启动
  - 超时/异常     → 退出进程
"""

import json
import sys
import threading
import time

import paho.mqtt.client as mqtt

from AppLogger import logger

# 默认等待时间（秒）
DEFAULT_STARTUP_TIMEOUT = 10
# 门禁字段名
GATE_FIELD = "disabled"


def _read_config(config_path):
    """读取 MQTT 配置文件"""
    with open(config_path, 'r') as f:
        return json.load(f)


def _build_startup_topic(config):
    """构建门禁 topic，优先用配置，回退为自动拼接"""
    topics = config.get('topics', {})
    startup_topic = topics.get('startup_topic')
    if startup_topic:
        return startup_topic

    mqtt_cfg = config.get('mqtt', {})
    product_model = mqtt_cfg.get('product_model', '')
    product_id = mqtt_cfg.get('product_id', '')
    return "RAILCAR/S/{}{}/startup".format(product_model, product_id)


def _is_disabled(message_data):
    """
    判断消息中的 disabled 字段是否为"禁用"。
    兼容字符串 "1"、数字 1、布尔 true。
    """
    value = message_data.get(GATE_FIELD)
    if value is None:
        logger.warning("门禁消息中缺少 '{}' 字段，视为允许启动".format(GATE_FIELD))
        return False

    # 布尔值
    if isinstance(value, bool):
        return value

    # 数字
    if isinstance(value, (int, float)):
        return int(value) != 0

    # 字符串
    if isinstance(value, str):
        if hasattr(value, 'decode'):
            value = value.decode('utf-8', errors='ignore')
        text = value.strip().lower()
        if text in ('1', 'true', 'yes', 'on'):
            return True
        if text in ('0', 'false', 'no', 'off', ''):
            return False
        try:
            return int(text) != 0
        except (ValueError, TypeError):
            pass

    logger.warning("无法解析门禁字段 '{}': {}，视为允许启动".format(GATE_FIELD, repr(value)))
    return False


def check_startup_gate(config_path='mqtt_config.json', timeout=DEFAULT_STARTUP_TIMEOUT):
    """
    启动门禁检查。
    连接 MQTT 云平台，读取 startup topic 的 retained 消息中的 disabled 字段。
    若 disabled 为真或超时/异常，直接退出进程。
    """
    try:
        config = _read_config(config_path)
    except Exception as e:
        logger.error("读取MQTT配置文件失败: {}".format(str(e)))
        sys.exit(0)

    mqtt_cfg = config.get('mqtt', {})
    broker = mqtt_cfg.get('broker')
    port = mqtt_cfg.get('port', 1883)
    username = mqtt_cfg.get('username')
    password = mqtt_cfg.get('password')
    product_id = mqtt_cfg.get('product_id', 'unknown')
    keepalive = int(mqtt_cfg.get('keepalive', 60))

    startup_topic = _build_startup_topic(config)
    logger.info("启动门禁: broker={}:{}, topic={}, timeout={}s".format(
        broker, port, startup_topic, timeout))

    # 同步信号：收到消息或超时
    received_event = threading.Event()
    received_data = [None]  # 用列表包装，方便在回调里修改
    connect_error = [None]

    client = mqtt.Client(client_id="{}-startup".format(product_id))

    if username and password:
        client.username_pw_set(username, password)

    # ---- 回调 ----
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            logger.info("门禁MQTT连接成功，订阅 {}".format(startup_topic))
            client.subscribe(startup_topic, qos=1)
        else:
            error_map = {
                1: "协议版本错误",
                2: "无效的客户端ID",
                3: "服务器不可用",
                4: "用户名或密码错误",
                5: "未授权",
            }
            err_msg = error_map.get(rc, "未知错误({})".format(rc))
            connect_error[0] = err_msg
            logger.error("门禁MQTT连接失败: {}".format(err_msg))
            received_event.set()

    def on_message(client, userdata, msg):
        try:
            payload = msg.payload
            if isinstance(payload, bytes):
                payload = payload.decode('utf-8', errors='ignore')
            logger.info("收到门禁消息: topic={}, payload={}".format(msg.topic, payload))
            message_data = json.loads(payload)
            received_data[0] = message_data
        except Exception as e:
            logger.error("解析门禁消息失败: {}".format(str(e)))
        finally:
            received_event.set()

    client.on_connect = on_connect
    client.on_message = on_message

    # ---- 连接 ----
    try:
        client.connect(broker, port, keepalive)
    except Exception as e:
        logger.error("门禁MQTT连接异常: {}".format(str(e)))
        sys.exit(0)

    client.loop_start()

    # ---- 等待结果 ----
    if not received_event.wait(timeout):
        logger.error("门禁检查超时 ({}s)，未收到云平台消息，退出进程".format(timeout))
        client.loop_stop()
        client.disconnect()
        sys.exit(0)

    client.loop_stop()
    client.disconnect()

    # ---- 判断 ----
    if connect_error[0] is not None:
        logger.error("门禁MQTT连接失败: {}，退出进程".format(connect_error[0]))
        sys.exit(0)

    if received_data[0] is None:
        logger.error("门禁检查未收到有效消息，退出进程")
        sys.exit(0)

    if _is_disabled(received_data[0]):
        logger.warning("云平台已禁用本机启动 (disabled=1)，退出进程")
        sys.exit(0)

    logger.info("启动门禁检查通过 (disabled=0)，继续启动")
