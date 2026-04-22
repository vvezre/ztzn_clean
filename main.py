#!/usr/bin/env python
# coding=utf-8

# coding: utf-8

import base64

import binascii
import codecs

import hashlib

import json

import logging
import math

import os

import platform

import socket

import sys

import threading

import time

import cv2

import numpy as np

import redis


import serial

from flask import Flask, make_response, request, jsonify, Response

from AppLogger import logger

import util
import traceback

import service
from DynamicCronScheduler import DynamicCronScheduler
from FixedPositiveChecker import FixedPositiveChecker

from flask_cors import CORS

from MqttClient import MqttClient
from ntrip_runtime import get_shared_runtime
from RTKDataManager import RTKDataManager
# from pid import PID

from mqtt_integration import MQTTIntegration
from mqtt_vehicle_adapter import VehicleControllerAdapter

app = Flask(__name__)
CORS(app)

canStart = 1

global clients

clients = {}

CMD_LEN = 23

PATH_PLANNING_KEY = "pathPlanning"

LEFT_PATH_PLANNING = "left_path_planning"

RIGHT_PATH_PLANNING = "right_path_planning"

high_speed = 350

high_brush_speed = 30

ip = "218.2.130.246"

id = "30f4b4af-c1a8-f3f6-072d-3807918c0dc0"

redis_cli = redis.Redis(host='localhost', port=6379, db=0)

redis_cli.set("forwardSpeed", high_speed)
redis_cli.set("brushSpeed", high_brush_speed)
redis_cli.set('correct', 'false')
redis_cli.set('moveJudge', 'false')
redis_cli.set(PATH_PLANNING_KEY, LEFT_PATH_PLANNING)
redis_cli.set('detectQrcode', 'false')
redis_cli.set('enterGarage', 'false')
redis_cli.set('currentAction', 'idle')
redis_cli.set('curTaskIndex', 0)
redis_cli.set('mission', 'complete')
redis_cli.set('parking', '1')
redis_cli.set('action', 'false')
redis_cli.set('correct', 'false')
redis_cli.set('moveJudge', 'false')
redis_cli.set('reverse', 'false')
redis_cli.set('controlState', 'IDLE')
redis_cli.set('healthState', 'OK')
redis_cli.set('faultState', '')
redis_cli.set('startCheckReady', 'false')
redis_cli.set('startCheckReason', '')
redis_cli.set('runtimeDetail', '{}')
redis_cli.delete('battery')
redis_cli.delete('batteryPercent')
redis_cli.delete('batteryRaw')
redis_cli.delete('batteryPercentRaw')
redis_cli.delete('batteryReportAt')
redis_cli.delete('voltage')
redis_cli.delete('packVoltage')
redis_cli.delete('packVoltageReportAt')
redis_cli.set('bootSafeStopAt', int(time.time()))

# 状态，0刹车，1速度模式，2距离速度模式，3旋转模式
global_get_status = 0

global_get_powerOn = 0  # 使能状态，0断电，1通电

global_get_HWstatus = 0  # 硬件功能状态，0异常，1正常？

global_get_XSpeed = 0  # X速度乘以一千存进去

global_get_ZSpeed = 0  # Z速度乘以一千存进去

global_get_brushSpeed = 0  # 滚刷速度

global_get_edge = 0  # 边缘，即超声波 1能走，0到边

global_get_voltage = 0  # 电池电压

global_get_air = 0  # 气压

global_get_moveFinish = 0  # 距离运动到位

global_get_rotateFinish = 0  # 旋转运动到位

# 小车状态 active- 空闲;
# working- 工作中;
# charging- 充电中;
# disabled- 维护;
# goCharging-返回充电中
global_status = "active"

ser_rtk_params = {'port': '/dev/ttyUSB0', 'baudRate': 115200, 'timeout': 1}
# 全局变量是否是直行,0:不是直行，1：直行
global_go = 0
# 当前任务的开始点经纬度和结束点经纬度
global_cur_taskPoint = {}
global_cur_taskPointTest = {}
# 自动清扫线程是否结束，0：未结束，1：结束
global_doCleanThreadStop = 0
# 当前任务下标标记
global_cur_task_index = 0
# 是否偏差过大，如果视觉纠偏过大，则启用RTK纠偏,0:表示不需要RTK纠偏，1：表示需要
global_is_need_rtk = 0
# RTK纠偏是否打开成功,0:表示失败，1:表示成功
global_open_rtk = 1
global_start_angle_rtk = 350
# 全局实时的经纬度和航向角
global_cur_rtk_lat = None
global_cur_rtk_lon = None
global_cur_rtk_heading = 0.0
# 上一次距离目标距离，用于是否停止
global_last_distance_to_target = 100000
# 每个任务的时间间隔
global_interval = 0
# 原点经纬度和每个任务的开始经纬度
global_originLat=34.35228117
global_originLon=117.93049352
global_startLat=34.35228117
global_startLon=117.93049352
# 起始点到充电桩的距离
startToChargingPilePointLength=0
# 配置文件中的数据
taskList = []
# 配置固定点位所在区域
global_area = 2
# 固定点经纬度
global_point_lat = 32.03647652
global_point_lon = 118.92454171
# 点对点执行任务是否被打断标识,0:表示没有被打断，1：表示打断
global_pointToPoint_flag = 0
checker = FixedPositiveChecker(window_size=3)
# 小车id
vehicleId = '0001'
# 小车类型
vehicleType = 'tracklayer'
# 自动清扫线程
drive_thread = None
global_last_cte = 0.0
START_POSITION_TOLERANCE_METERS = 2.0
global_power_on_guard_sent = False
BATTERY_SMOOTH_ALPHA = 0.18
BATTERY_MAX_DROP_PER_SAMPLE = 0.3
BATTERY_MAX_RISE_PER_SAMPLE = 0.6
BATTERY_SMOOTH_RESET_AFTER_SEC = 60


def set_current_action(action_name):
    try:
        redis_cli.set('currentAction', action_name)
    except Exception as e:
        logger.warning("设置currentAction失败: {}".format(str(e)))


def sync_current_location(lat, lon, heading=None):
    try:
        redis_cli.hset('currentLocation', 'lat', lat)
        redis_cli.hset('currentLocation', 'lon', lon)
        if heading is not None:
            redis_cli.hset('currentLocation', 'heading', heading)
    except Exception as e:
        logger.warning("同步currentLocation失败: {}".format(str(e)))


def _decode_redis_value(value):
    if value is None:
        return None
    try:
        if isinstance(value, bytes):
            return value.decode('utf-8')
    except Exception:
        pass
    return value


def _coerce_float(value, default=None):
    value = _decode_redis_value(value)
    if value in (None, ''):
        return default
    try:
        return float(value)
    except Exception:
        return default


def _coerce_int(value, default=None):
    value = _decode_redis_value(value)
    if value in (None, ''):
        return default
    try:
        return int(float(value))
    except Exception:
        return default


def _coerce_bool(value, default=False):
    value = _decode_redis_value(value)
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ('1', 'true', 'yes', 'on'):
        return True
    if text in ('0', 'false', 'no', 'off', 'none', ''):
        return False
    try:
        return int(text) != 0
    except Exception:
        return default


def _set_redis_value(key, value):
    if value is None:
        redis_cli.delete(key)
        return
    if isinstance(value, (dict, list)):
        redis_cli.set(key, json.dumps(value))
        return
    if isinstance(value, bool):
        redis_cli.set(key, 'true' if value else 'false')
        return
    redis_cli.set(key, value)


def _load_runtime_detail():
    raw = _decode_redis_value(redis_cli.get('runtimeDetail'))
    if not raw:
        return {}
    try:
        return json.loads(raw)
    except Exception:
        return {}


def _get_power_on_state():
    return _coerce_int(redis_cli.get('powerOnState'), None)


def _get_hardware_report_at():
    return _coerce_int(redis_cli.get('hardwareReportAt'), None)


def _get_hardware_report_age_sec():
    report_at = _get_hardware_report_at()
    if report_at is None:
        return None
    try:
        return max(0, int(time.time()) - int(report_at))
    except Exception:
        return None


def _clamp_percent(value):
    value = _coerce_float(value, None)
    if value is None:
        return None
    return max(0.0, min(100.0, value))


def _smooth_battery_percent(raw_percent, report_at=None):
    raw_percent = _clamp_percent(raw_percent)
    if raw_percent is None:
        return None

    previous = _coerce_float(redis_cli.get('batteryPercent'), None)
    previous_report_at = _coerce_int(redis_cli.get('batteryReportAt'), None)
    if previous is None:
        return round(raw_percent, 1)
    if report_at is None:
        report_at = int(time.time())
    if previous_report_at is None or report_at - previous_report_at > BATTERY_SMOOTH_RESET_AFTER_SEC:
        return round(raw_percent, 1)

    delta = raw_percent - previous
    if abs(delta) < 0.05:
        return round(raw_percent, 1)
    if delta < 0:
        step = max(delta * BATTERY_SMOOTH_ALPHA, -BATTERY_MAX_DROP_PER_SAMPLE)
    else:
        step = min(delta * BATTERY_SMOOTH_ALPHA, BATTERY_MAX_RISE_PER_SAMPLE)
    return round(_clamp_percent(previous + step), 1)


def _cache_battery_percent(raw_percent, report_at=None):
    raw_percent = _clamp_percent(raw_percent)
    if raw_percent is None:
        return None
    if report_at is None:
        report_at = int(time.time())

    smoothed_percent = _smooth_battery_percent(raw_percent, report_at)
    redis_cli.set("batteryRaw", raw_percent)
    redis_cli.set("batteryPercentRaw", raw_percent)
    redis_cli.set("battery", smoothed_percent)
    redis_cli.set("batteryPercent", smoothed_percent)
    redis_cli.set("batteryReportAt", report_at)
    # Legacy low-battery logic reads "voltage" as percentage.
    redis_cli.set("voltage", smoothed_percent)
    return smoothed_percent


def _set_runtime_state(control_state=None, health_state=None, fault_state=None,
                       start_ready=None, start_reason=None, detail=None):
    if control_state is not None:
        _set_redis_value('controlState', control_state)
    if health_state is not None:
        _set_redis_value('healthState', health_state)
    if fault_state is not None:
        _set_redis_value('faultState', fault_state)
    if start_ready is not None:
        _set_redis_value('startCheckReady', start_ready)
    if start_reason is not None:
        _set_redis_value('startCheckReason', start_reason)
    if detail is not None:
        _set_redis_value('runtimeDetail', detail)


def _load_task_params_snapshot():
    raw = redis_cli.hgetall('taskParams') or {}
    result = {}
    for key, value in raw.items():
        result[_decode_redis_value(key)] = _decode_redis_value(value)
    return result


def _compute_local_xy_cm(lat, lon, task_params):
    if lat is None or lon is None:
        return None, None

    origin_lat = _coerce_float(task_params.get('startLat'), None)
    origin_lon = _coerce_float(task_params.get('startLon'), None)
    origin_heading = _coerce_float(task_params.get('originHeading'), None)
    if origin_lat is None or origin_lon is None or origin_heading is None:
        return None, None

    try:
        x_m, y_m = util.latlon_to_local_rotated_xy_precise(
            origin_lat, origin_lon, lat, lon, origin_heading
        )
        return int(round(x_m * 100)), int(round(y_m * 100))
    except Exception:
        return None, None


def _distance_to_task_start(task_params):
    start_lat = _coerce_float(task_params.get('startLat'), None)
    start_lon = _coerce_float(task_params.get('startLon'), None)
    if start_lat is None or start_lon is None:
        return None
    if global_cur_rtk_lat is None or global_cur_rtk_lon is None:
        return None
    try:
        distance, _ = util.get_distance_angle(global_cur_rtk_lat, global_cur_rtk_lon, start_lat, start_lon)
        return round(float(distance), 3)
    except Exception:
        return None


def _build_runtime_detail(extra=None):
    task_params = _load_task_params_snapshot()
    detail = _load_runtime_detail()
    power_on_state = _get_power_on_state()
    detail.update({
        'batteryReportAt': _coerce_int(redis_cli.get('batteryReportAt'), None),
        'batteryPercent': _coerce_float(redis_cli.get('batteryPercent'), None),
        'batteryPercentRaw': _coerce_float(redis_cli.get('batteryPercentRaw'), None),
        'packVoltage': _coerce_float(redis_cli.get('packVoltage'), None),
        'packVoltageReportAt': _coerce_int(redis_cli.get('packVoltageReportAt'), None),
        'powerOnState': power_on_state,
        'powerOnEnabled': power_on_state == 1,
        'hardwareState': _coerce_int(redis_cli.get('hardwareState'), None),
        'hardwareReportAt': _get_hardware_report_at(),
        'hardwareReportAgeSec': _get_hardware_report_age_sec(),
        'distanceToStartM': _distance_to_task_start(task_params),
        'startToleranceM': START_POSITION_TOLERANCE_METERS,
        'currentLat': global_cur_rtk_lat,
        'currentLon': global_cur_rtk_lon,
        'currentHeading': global_cur_rtk_heading,
        'taskStartLat': _coerce_float(task_params.get('startLat'), None),
        'taskStartLon': _coerce_float(task_params.get('startLon'), None),
        'originHeading': _coerce_float(task_params.get('originHeading'), None),
    })
    if extra:
        detail.update(extra)
    return detail


def _mark_runtime_ready(message, extra=None):
    _set_runtime_state(
        control_state='READY',
        health_state='OK',
        fault_state='',
        start_ready=True,
        start_reason=message,
        detail=_build_runtime_detail(extra)
    )


def _mark_runtime_running(message, extra=None):
    _set_runtime_state(
        control_state='RUNNING',
        health_state='OK',
        fault_state='',
        start_ready=True,
        start_reason=message,
        detail=_build_runtime_detail(extra)
    )


def _mark_runtime_idle(message, extra=None):
    _set_runtime_state(
        control_state='STOPPED',
        health_state='OK',
        fault_state='',
        start_ready=False,
        start_reason=message,
        detail=_build_runtime_detail(extra)
    )


def _mark_runtime_blocked(fault_state, message, extra=None):
    detail = _build_runtime_detail(extra)
    detail['blocked'] = True
    _set_runtime_state(
        control_state='BLOCKED',
        health_state='WARN',
        fault_state=fault_state,
        start_ready=False,
        start_reason=message,
        detail=detail
    )


def _maybe_brake_on_power_enable(previous_power_on_state, current_power_on_state):
    global global_power_on_guard_sent
    if current_power_on_state != 1 or previous_power_on_state == 1:
        return
    if global_power_on_guard_sent:
        return

    mission = _decode_redis_value(redis_cli.get('mission'))
    current_action = _decode_redis_value(redis_cli.get('currentAction'))
    if mission == 'working' and current_action in ('auto_drive', 'go_on', 'return_to_point'):
        return

    global_power_on_guard_sent = True
    redis_cli.set('parking', '1')
    redis_cli.set('mission', 'complete')
    set_current_action('parking')
    _mark_runtime_idle('下位机刚使能，已自动补发安全停车')
    try:
        logger.warning("下位机使能从 {} 切换为 1，当前非任务执行态，补发安全停车".format(previous_power_on_state))
        sendBraking()
    except Exception as exc:
        logger.error("下位机使能安全停车失败: {}".format(exc), exc_info=True)


def _frame_byte_to_int(byte_value):
    try:
        if isinstance(byte_value, int):
            return byte_value
        return int(binascii.b2a_hex(byte_value), 16)
    except Exception:
        return None


def _frame_u16_to_int(data, index):
    high = _frame_byte_to_int(data[index])
    low = _frame_byte_to_int(data[index + 1])
    if high is None or low is None:
        return None
    return (high << 8) + low


def _cache_hardware_status_frame(data):
    if not data or len(data) < CMD_LEN:
        return False

    try:
        report_at = int(time.time())
        redis_cli.set("hardwareReportAt", report_at)

        previous_power_on_state = _get_power_on_state()
        power_on_state = _frame_byte_to_int(data[2])
        if power_on_state is not None:
            redis_cli.set("powerOnState", power_on_state)
            _maybe_brake_on_power_enable(previous_power_on_state, power_on_state)

        hardware_state = _frame_byte_to_int(data[3])
        if hardware_state is not None:
            redis_cli.set("hardwareState", hardware_state)

        _cache_battery_percent(_frame_byte_to_int(data[10]), report_at)

        pack_voltage_raw = _frame_u16_to_int(data, 14)
        if pack_voltage_raw is not None:
            redis_cli.set("packVoltage", round(pack_voltage_raw * 0.01, 1))
            redis_cli.set("packVoltageReportAt", report_at)

        return True
    except Exception as exc:
        logger.warning("cache hardware status frame failed: {}".format(exc), exc_info=True)
        return False


def _load_task_items_for_preview():
    cached_items = redis_cli.lrange('taskList', 0, -1)
    items = []
    for raw in cached_items or []:
        try:
            items.append(json.loads(_decode_redis_value(raw)))
        except Exception:
            continue
    if items:
        return items

    try:
        task_obj = util.readConfig("config.json")
        task_list = task_obj.get('taskList', [])
        if isinstance(task_list, list):
            return task_list
    except Exception as e:
        logger.warning("读取任务预览失败: {}".format(str(e)))
    return []


def _build_task_path_payload():
    task_params = _load_task_params_snapshot()
    task_items = _load_task_items_for_preview()
    segments = []
    for item in task_items:
        if not isinstance(item, dict):
            continue
        segments.append({
            'id': item.get('id'),
            'startX': _coerce_int(item.get('startX'), 0),
            'startY': _coerce_int(item.get('startY'), 0),
            'endX': _coerce_int(item.get('endX'), 0),
            'endY': _coerce_int(item.get('endY'), 0),
            'mode': _coerce_int(item.get('mode'), None),
            'angle': _coerce_float(item.get('angle'), None),
            'heading': _coerce_float(item.get('heading'), None),
            'areaNumber': _coerce_int(item.get('areaNumber'), None),
        })

    return {
        'taskId': _decode_redis_value(redis_cli.get('currentTaskName')) or 'current',
        'taskName': _decode_redis_value(redis_cli.get('currentTaskName')) or '',
        'originLat': _coerce_float(task_params.get('startLat'), None),
        'originLon': _coerce_float(task_params.get('startLon'), None),
        'yAxisBearing': _coerce_float(task_params.get('originHeading'), None),
        'updatedAt': int(time.time() * 1000),
        'segments': segments,
    }


def _derive_control_state():
    power_on_state = _get_power_on_state()
    if power_on_state == 0:
        return 'DISABLED'
    control_state = _decode_redis_value(redis_cli.get('controlState'))
    if control_state:
        return control_state
    if power_on_state is None:
        return 'UNKNOWN'
    if _decode_redis_value(redis_cli.get('mission')) == 'working':
        return 'RUNNING'
    if _coerce_bool(redis_cli.get('parking'), False):
        return 'STOPPED'
    return 'IDLE'


def _derive_health_state():
    if _derive_fault_state():
        return 'WARN'
    health_state = _decode_redis_value(redis_cli.get('healthState'))
    if health_state:
        return health_state
    return 'OK'


def _derive_fault_state():
    power_on_state = _get_power_on_state()
    if power_on_state == 0:
        return 'LOWER_MACHINE_DISABLED'
    fault_state = _decode_redis_value(redis_cli.get('faultState'))
    if fault_state:
        return fault_state
    if power_on_state is None:
        return 'LOWER_MACHINE_STATUS_UNKNOWN'
    return ''


def _derive_mission_state(control_state):
    if control_state in ('BLOCKED', 'DISABLED', 'UNKNOWN'):
        return control_state
    current_action = _decode_redis_value(redis_cli.get('currentAction'))
    if current_action == 'return_to_point':
        return 'RETURNING'
    mission = _decode_redis_value(redis_cli.get('mission'))
    if mission == 'working':
        return 'RUNNING'
    if _coerce_bool(redis_cli.get('parking'), False):
        return 'STOPPED'
    if mission == 'complete':
        return 'COMPLETE'
    return 'IDLE'


def _derive_status(control_state, mission_state):
    if control_state == 'UNKNOWN':
        return 'unknown'
    if control_state == 'BLOCKED' or control_state == 'DISABLED':
        return 'disabled'
    if mission_state == 'RUNNING':
        return 'working'
    if mission_state == 'RETURNING':
        return 'returning'
    if _coerce_bool(redis_cli.get('parking'), False):
        return 'idle'
    return 'active'


def _build_vehicle_status_payload():
    task_params = _load_task_params_snapshot()
    lat = global_cur_rtk_lat
    lon = global_cur_rtk_lon
    heading = global_cur_rtk_heading if global_cur_rtk_heading is not None else None
    local_x, local_y = _compute_local_xy_cm(lat, lon, task_params)
    control_state = _derive_control_state()
    mission_state = _derive_mission_state(control_state)
    fault_state = _derive_fault_state()
    current_action = _decode_redis_value(redis_cli.get('currentAction')) or ('parking' if _coerce_bool(redis_cli.get('parking'), False) else 'idle')
    battery_percent = _clamp_percent(redis_cli.get('batteryPercent'))
    battery_percent_raw = _clamp_percent(redis_cli.get('batteryPercentRaw'))

    detail = _build_runtime_detail({
        'lastCommandMessage': _decode_redis_value(redis_cli.get('lastCommandMessage')) or '',
        'startCheckReady': _coerce_bool(redis_cli.get('startCheckReady'), False),
        'startCheckReason': _decode_redis_value(redis_cli.get('startCheckReason')) or '',
    })

    return {
        'status': _derive_status(control_state, mission_state),
        'battery': battery_percent,
        'battery_percent': battery_percent,
        'battery_raw': battery_percent_raw,
        'battery_percent_raw': battery_percent_raw,
        'action': current_action,
        'task_name': _decode_redis_value(redis_cli.get('currentTaskName')) or '',
        'cur_task_index': _coerce_int(redis_cli.get('curTaskIndex'), 0),
        'task_count': len(_load_task_items_for_preview()),
        'online_state': 'ONLINE',
        'mission_state': mission_state,
        'control_state': control_state,
        'health_state': _derive_health_state(),
        'fault_state': fault_state,
        'speed': _coerce_int(redis_cli.get('forwardSpeed'), 0),
        'brush_speed': _coerce_int(redis_cli.get('brushSpeed'), 0),
        'voltage': _coerce_float(redis_cli.get('packVoltage'), None),
        'lat': lat,
        'lon': lon,
        'heading': heading,
        'local_x': local_x,
        'local_y': local_y,
        'tracking': _coerce_bool(redis_cli.get('correct'), False),
        'path_planning': _decode_redis_value(redis_cli.get(PATH_PLANNING_KEY)) or '',
        'move_judge': _coerce_bool(redis_cli.get('moveJudge'), False),
        'detect_qrcode': _coerce_bool(redis_cli.get('detectQrcode'), False),
        'enter_garage': _coerce_bool(redis_cli.get('enterGarage'), False),
        'supported_actions': ['auto_drive', 'go_on', 'stop', 'parking', 'return_to_point', 'get_status', 'get_task_path'],
        'supported_params': ['taskName', 'speed', 'tracking', 'path'],
        'supported_status_fields': ['control_state', 'health_state', 'fault_state', 'detail', 'mission_state'],
        'detail': detail,
        'timestamp': int(time.time()),
    }


def _validate_auto_drive_request():
    task_params = _load_task_params_snapshot()
    start_lat = _coerce_float(task_params.get('startLat'), None)
    start_lon = _coerce_float(task_params.get('startLon'), None)
    origin_heading = _coerce_float(task_params.get('originHeading'), None)
    detail = _build_runtime_detail()

    if _decode_redis_value(redis_cli.get('mission')) == 'working':
        return {
            'success': False,
            'faultState': 'ALREADY_RUNNING',
            'message': '小车当前正在执行任务，请勿重复启动',
            'data': detail,
        }

    power_on_state = _get_power_on_state()
    if power_on_state is None:
        detail['lowerMachineStatusWarning'] = '尚未收到下位机使能状态上报，已按用户指令继续启动校验'

    elif power_on_state != 1:
        return {
            'success': False,
            'faultState': 'LOWER_MACHINE_DISABLED',
            'message': '下位机未使能，拒绝启动自动清扫',
            'data': detail,
        }

    if start_lat is None or start_lon is None or origin_heading is None:
        return {
            'success': False,
            'faultState': 'TASK_PARAMS_MISSING',
            'message': '任务起点或航向参数未配置完整，无法启动',
            'data': detail,
        }

    if global_cur_rtk_lat is None or global_cur_rtk_lon is None:
        return {
            'success': False,
            'faultState': 'RTK_NOT_READY',
            'message': 'RTK 定位未就绪，无法校验任务起点',
            'data': detail,
        }

    distance_to_start = _distance_to_task_start(task_params)
    if distance_to_start is None:
        return {
            'success': False,
            'faultState': 'START_POSITION_UNKNOWN',
            'message': '无法计算当前位置与任务起点距离，拒绝启动',
            'data': detail,
        }

    detail['distanceToStartM'] = distance_to_start
    if distance_to_start > START_POSITION_TOLERANCE_METERS:
        return {
            'success': False,
            'faultState': 'NOT_AT_TASK_START',
            'message': '当前位置距离任务起点 {:.2f} 米，超过允许范围 {:.2f} 米'.format(
                distance_to_start, START_POSITION_TOLERANCE_METERS
            ),
            'data': detail,
        }

    if len(_load_task_items_for_preview()) == 0:
        return {
            'success': False,
            'faultState': 'TASK_PATH_EMPTY',
            'message': '当前没有可执行任务路径，拒绝启动',
            'data': detail,
        }

    return {
        'success': True,
        'message': '启动条件通过',
        'data': detail,
    }



# =============== 可调整参数 ===============
MIN_LINE_LENGTH = 100  # 最小线段长度
MAX_ANGLE = 45  # 最大垂直偏差角度
CANNY_THRESHOLD1 = 50  # Canny边缘检测低阈值
CANNY_THRESHOLD2 = 150  # Canny边缘检测高阈值
# =======================================
CONSENSUS_THRESHOLD = 7  # 共识阈值（需要多少个相同的角度值）
# =======================================
# 全局变量存储角度样本和最终结果
angle_samples = []
final_angle = None

# 下位机端口
xwj_port = "/dev/ttyACM0"
# rtk端口
rtk_port = util.findPort("$GN")
logger.warn(rtk_port)
if rtk_port == "/dev/ttyACM4":
    xwj_port = "/dev/ttyACM0"
elif rtk_port == "/dev/ttyUSB0":
    xwj_port = "/dev/ttyACM0"
elif rtk_port != None:
    xwj_port = "/dev/ttyACM4"
logger.warn("下位机串口：{}".format(xwj_port))

def globalDataSet(data):
    i = 0
    global global_get_status
    global global_get_powerOn
    global global_get_HWstatus
    global global_get_XSpeed
    global global_get_ZSpeed
    global global_get_brushSpeed
    global global_get_edge
    global global_get_voltage
    global global_get_air
    global global_get_moveFinish

    redis_cli.set("hardwareReportAt", int(time.time()))

    for ch in data:
        if i == 1:
            global_get_status = int(binascii.b2a_hex(data[i]), 16)
        elif i == 2:
            previous_power_on_state = _get_power_on_state()
            global_get_powerOn = int(binascii.b2a_hex(data[i]), 16)
            redis_cli.set("powerOnState", global_get_powerOn)
            _maybe_brake_on_power_enable(previous_power_on_state, global_get_powerOn)
        elif i == 3:
            global_get_HWstatus = int(binascii.b2a_hex(data[i]), 16)
            redis_cli.set("hardwareState", global_get_HWstatus)
        elif i == 4:
            global_get_XSpeed = int(binascii.b2a_hex(data[i] + data[i + 1]), 16)
        elif i == 6:
            global_get_ZSpeed = int(binascii.b2a_hex(data[i] + data[i + 1]), 16)
        elif i == 8:
            global_get_brushSpeed = int(binascii.b2a_hex(data[i]), 16)
        elif i == 9:
            if binascii.b2a_hex(data[i]) == '00':
                global_get_edge = 1
            elif binascii.b2a_hex(data[i]) == 'ff':
                logger.info("返回到边指令")
                redis_cli.set("ultraSonic", "true")
                global_get_edge = 0
            else:
                global_get_edge = 0
        elif i == 10:
            global_get_voltage = int(binascii.b2a_hex(data[i]), 16)
            _cache_battery_percent(global_get_voltage, int(time.time()))
        elif i == 11:
            global_get_air = int(binascii.b2a_hex(data[i]), 16)
        elif i == 12:
            if binascii.b2a_hex(data[i]) == '00':
                global_get_moveFinish = 0
            elif binascii.b2a_hex(data[i]) == 'bb':
                global_get_moveFinish = 1
            else:
                global_get_moveFinish = 0

        elif i == 13:
            global global_get_rotateFinish
            print(binascii.b2a_hex(data[i]))
            if binascii.b2a_hex(data[i]) == '00':
                global_get_rotateFinish = 0
            elif binascii.b2a_hex(data[i]) == 'bb':
                global_get_rotateFinish = 1
            else:
                global_get_rotateFinish = 0
        i = i + 1
    info = binascii.b2a_hex(data[11])
    logger.info(int(info, 16))
    str1 = binascii.b2a_hex(data[14])
    str2 = binascii.b2a_hex(data[15])
    voltage = int(str1 + str2, 16)
    redis_cli.set("packVoltage", round(voltage * 0.01, 1))
    redis_cli.set("packVoltageReportAt", int(time.time()))
    # if voltage > 0:
    #     logger.info('获取到电压值: %d', voltage)
    #     roundVoltage = round(voltage * 0.01, 1)
    #     roundVoltage = round((roundVoltage - 23) / (28 - 23)) * 100
    #     # redis_cli.set('voltage', roundVoltage)

    str1 = binascii.b2a_hex(data[16])
    str2 = binascii.b2a_hex(data[17])
    angle = int(str1 + str2, 16)
    redis_cli.set("angle", angle)

    str1 = binascii.b2a_hex(data[18])
    str2 = binascii.b2a_hex(data[19])
    odometer = int(str1 + str2, 16)

    redis_cli.set("odometer", odometer)


def init():
    pass


# 通知客户端
def notify(message):
    for conn_id in clients.keys():
        try:
            connection = clients.get(conn_id)
            connection.send('%c%c%s' % (0x81, len(message), message))
        except Exception as e:
            logger.error('ws报错')
            clients.pop(conn_id)


#获取任务

def getTask(file_path):
    """

    从指定的 TXT 文件中读取任务点列表。

    每一行格式为: 纬度,经度,执行方式（用逗号分隔）

    返回值: List[Tuple[float, float, int]]

    """

    task_points = []

    with open(file_path, "r") as f:

        for line_num, line in enumerate(f, 1):

            line = line.strip()

            if not line or line.startswith("#"):
                continue  # 跳过空行或注释行

            try:

                angle_str, mode_str, length_str, back_len_str = line.split(",")

                angle = int(angle_str)

                mode = int(mode_str)

                length = float(length_str)

                back_len = int(back_len_str)

                task_points.append((angle, mode, length, back_len))

            except ValueError:

                print("[警告] 第{line_num}行格式错误: {line}")

    return task_points


# 客户端处理线程
class websocket_thread(threading.Thread):
    def __init__(self, connection, username):
        super(websocket_thread, self).__init__()
        self.connection = connection
        self.username = username

    def run(self):
        print('new websocket client joined!')
        data = self.connection.recv(1024)
        headers = self.parse_headers(data)
        token = self.generate_token(headers['Sec-WebSocket-Key'])
        self.connection.send(
            'HTTP/1.1 101 WebSocket Protocol Hybi-10 Upgrade: WebSocket Connection: Upgrade Sec-WebSocket-Accept: %s' % token)
        while True:
            try:
                data = self.connection.recv(1024)
            except socket.error as e:
                print("unexpected error: ", e)
                clients.pop(self.username)
                break
            try:
                data = self.parse_data(data)
            except:
                pass
            if len(data) == 0 or data.startswith('\03'):
                continue
            message = self.username + ": " + data
            notify(message)

    def parse_data(self, msg):

        v = ord(msg[1]) & 0x7f

        if v == 0x7e:

            p = 4

        elif v == 0x7f:

            p = 10

        else:

            p = 2

        mask = msg[p:p + 4]

        data = msg[p + 4:]

        return ''.join([chr(ord(v) ^ ord(mask[k % 4])) for k, v in enumerate(data)])

    def parse_headers(self, msg):

        headers = {}

        header, data = msg.split('\r\n\r\n', 1)

        for line in header.split('\r\n')[1:]:
            key, value = line.split(': ', 1)

            headers[key] = value

        headers['data'] = data

        return headers

    def generate_token(self, msg):

        key = msg + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'

        ser_key = hashlib.sha1(key).digest()

        return base64.b64encode(ser_key)


# 服务端
class websocket_server(threading.Thread):

    def __init__(self, port):
        super(websocket_server, self).__init__()
        self.port = port

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        sock.bind(('0.0.0.0', self.port))
        sock.listen(5)
        print('websocket server started!')
        while True:
            connection, address = sock.accept()
            try:
                username = "ID" + str(address[1])
                thread = websocket_thread(connection, username)
                thread.start()
                clients[username] = connection
            except socket.timeout:
                print('websocket connection timeout!')


# 获取是否到边，1：表示在板子上，0：表示不在板子上
def getEdge():
    global global_status
    ser = serial.Serial(xwj_port, 115200, timeout=0.5)
    if ser.is_open:

        redis_cli.set('moveJudge', 'true')
        while redis_cli.get("mission") == "working":
            try:
                data = ser.read(CMD_LEN * 2)
                hex_data = binascii.b2a_hex(data).decode('utf-8')
                logger.info(hex_data)
                # 数据长度不够不要
                if len(data) < CMD_LEN:
                    logger.error("getEdge() data not full")
                    # global_status = "data not full"
                    hex_data = binascii.b2a_hex(data).decode('utf-8')
                    print(hex_data)
                    continue

                i = 0
                for q in data:
                    if binascii.b2a_hex(q) == '7b':
                        break
                    else:
                        i = i + 1
                        continue
                data = data[i:]
                globalDataSet(data)
                break
            except serial.serialutil.SerialException:

                try:

                    ser.close()

                    ser = serial.Serial(xwj_port, 115200, timeout=0.5)
                    # ser = serial.Serial('COM3', 115200, timeout=0.5)
                except serial.serialutil.SerialException:
                    logger.error("fail open COM")
                    global_status = "fail open COM"
                    time.sleep(0.5)
        redis_cli.set('moveJudge', 'false')
    else:
        global_status = "fail open COM"
        logger.error("fail open COM")
    return str(global_get_edge)


# 获取是否到达距离，0：未到达；1：到达
def getDistanceArrive():
    global global_status
    ser = serial.Serial(xwj_port, 115200, timeout=0.5)
    # ser = serial.Serial('COM3', 115200, timeout=0.5)
    if ser.is_open:
        redis_cli.set('moveJudge', 'true')
        while redis_cli.get("mission") == "working":
            try:
                data = ser.read(CMD_LEN * 2)
                logger.info('位数：{}'.format(len(data)))
                if len(data) < CMD_LEN:
                    # global_status = "data not full"
                    logger.error("getDistanceArrive() data not full")
                    continue
                i = 0
                for q in data:
                    if binascii.b2a_hex(q) == '7b':
                        break
                    else:
                        i = i + 1
                        continue
                data = data[i:]
                globalDataSet(data)
                break
            except serial.serialutil.SerialException:
                try:
                    ser.close()
                    ser = serial.Serial(xwj_port, 115200, timeout=0.5)
                    # ser = serial.Serial('COM3', 115200, timeout=0.5)
                except serial.serialutil.SerialException:
                    print("fail open COM")
                    global_status = "fail open COM"
                    time.sleep(0.5)
        redis_cli.set('moveJudge', 'false')

    else:
        print("fail open COM")
        global_status = "fail open COM"
    return str(global_get_moveFinish)


# 获取转圈是否完成，0：未完成；1：完成
def getRotateArrive():
    global global_status
    ser = serial.Serial(xwj_port, 115200, timeout=0.5)
    # ser = serial.Serial('COM3', 115200, timeout=0.5)
    if ser.is_open:
        logger.info("success open COM")
        # global_status = "success open COM"
        redis_cli.set('moveJudge', 'true')
        while redis_cli.get("mission") == "working":
            logger.info('goon rotate')
            try:
                data = ser.read(CMD_LEN * 2)
                # 数据长度不够不要
                if len(data) < CMD_LEN:
                    logger.warn("getRotateArrive() data not full")
                    # global_status = "data not full"
                    continue
                i = 0
                for q in data:
                    if binascii.b2a_hex(q) == '7b':
                        break
                    else:
                        i = i + 1
                        continue
                data = data[i:]

                globalDataSet(data)
                break
            except serial.serialutil.SerialException:
                try:
                    ser.close()
                    ser = serial.Serial(xwj_port, 115200, timeout=0.5)
                    # ser = serial.Serial('COM3', 115200, timeout=0.5)

                except serial.serialutil.SerialException:
                    print("fail open COM")
                    # global_status = "fail open COM"
                    time.sleep(0.5)
        redis_cli.set('moveJudge', 'false')
    else:
        print("fail open COM")
        # global_status = "fail open COM"
    return str(global_get_rotateFinish)


cap = cv2.VideoCapture(0)
drivingUp = False
# command = bytearray(17)
command = bytearray(19)

if sys.platform.startswith('win'):
    # ser = serial.Serial('COM3', 115200, timeout=0.5)
    ser = None
else:
    try:
        ser = serial.Serial(xwj_port, 115200, timeout=0.5)
    except serial.serialutil.SerialException as e:
        logger.warning('???????????? %s: %s', xwj_port, e)
        ser = None
    # ser = None


def encrypt_password(password):
    md5 = hashlib.md5()
    md5.update(password.encode('utf-8'))
    return md5.hexdigest()


@app.route("/vehicle/login", methods=['POST'])
def login():
    payload = request.get_json(silent=True) or request.form.to_dict() or {}
    username = payload.get('username', '')
    password = payload.get('password', '')

    if username == 'admin' and password:
        password_hash = encrypt_password(password)
        if password == 'njzt888' or password_hash == encrypt_password('njzt888'):
            res = {}
            res['msg'] = '????'
            res['code'] = 200
            res['token'] = "eyJhbGciOiJIUzUxMiJ9.eyJsb2dpbl91c2VyX2tleSI6Ijc3NjZjZDQyLWNlYWYtNDk1NC1hNjNjLWRhNmRiYTJlMzllZiJ9.JFVqfw5rTiKhpn0v_kiRyH5tw6XYx3R2Ru_sAePljTCCQbVB9aDZyS0k2WjHcw4UWAcMr9wMJ6oC2YmwRzi7vQ"
            response = make_response(json.dumps(res))
            response.headers['Content-Type'] = 'application/json'
            return response

    response = make_response(json.dumps({'msg': '????????', 'code': 401}))
    response.status_code = 401
    response.headers['Content-Type'] = 'application/json'
    return response

def _build_legacy_web_user_info():
    dept = {
        'deptId': 103,
        'parentId': 101,
        'deptName': 'dev',
        'status': '0',
    }
    role = {
        'roleId': 1,
        'roleName': 'admin',
        'roleKey': 'admin',
        'roleSort': 1,
        'dataScope': '1',
        'menuCheckStrictly': False,
        'deptCheckStrictly': False,
        'status': '0',
        'flag': False,
        'admin': True,
    }
    return {
        'createBy': 'admin',
        'createTime': '2024-06-30 11:27:11',
        'remark': 'admin',
        'userId': 1,
        'deptId': 103,
        'userName': 'admin',
        'nickName': 'admin',
        'email': 'zt@163.com',
        'phonenumber': '15888888888',
        'sex': '1',
        'status': '0',
        'delFlag': '0',
        'dept': dept,
        'roles': role,
        'admin': True,
    }


@app.route("/vehicle/getInfo", methods=['GET'])
def getInfo():
    # Keep the legacy web login contract while exposing the vehicle status
    # fields consumed by MQTT/cloud and the miniapp.
    return jsonify({
        'success': True,
        'message': 'status ok',
        'data': _build_vehicle_status_payload(),
        'msg': 'operation success',
        'code': 200,
        'permissions': '*:*:*',
        'roles': 'admin',
        'user': _build_legacy_web_user_info(),
    })


@app.route("/vehicle/getTaskPath", methods=['GET'])
def getTaskPath():
    payload = _build_task_path_payload()
    if not payload.get('segments'):
        return jsonify({
            'success': False,
            'message': '当前没有可用任务路径',
            'data': payload,
        })

    return jsonify({
        'success': True,
        'message': '任务路径获取成功',
        'data': payload,
    })


@app.route("/vehicle/enterGarage", methods=['GET'])
def enterGarage():
    task = threading.Thread(target=enter_garage_task)

    task.start()

    response = make_response("1")

    return response


# 车库
def enter_garage_task():
    redis_cli.set("enterGarage", "true")

    redis_cli.set("detectQrcode", "false")

    redis_cli.set("correct", "true")

    redis_cli.set('action', 'true')

    reSetStatus(ser)

    setBrushSpeed(0)

    setStatus(1)

    setPowerOn(1)

    setHWstatus(0, 0, 0, 0, 0)

    setXSpeed(90)

    command[17] = tem_listener(command, 17)

    duplicateWriteCmd(ser, command)

    while redis_cli.get('enterGarage') == 'true' and redis_cli.get("detectQrcode") == "false":
        time.sleep(0.25)

    sendBraking()

    redis_cli.set("enterGarage", "false")

    redis_cli.set("correct", "false")

    redis_cli.set('action', 'false')


def writeCmd(ser, command):
    if ser is None:
        logger.warning('??????????????')
        return
    if ser.is_open:
        pass
    else:
        ser = serial.Serial(xwj_port, 115200, timeout=0.5)
    ser.write(command)


def duplicateWriteCmd(ser, command):
    if ser is None:
        logger.warning('????????????????')
        return
    for i in range(5):
        writeCmd(ser, command)
    ser.flushOutput()


@app.route("/vehicle/exitGarage", methods=['GET'])
def exitGarage():
    redis_cli.set("reverse", "false")
    task = threading.Thread(target=exit_garage_task)

    task.start()

    response = make_response("1")

    return response


def exit_garage_task():
    redis_cli.set("reverse", "false")

    redis_cli.set("correct", "true")

    redis_cli.set('action', 'true')

    redis_cli.set('mission', 'working')

    exit_uav(ser)

    justMove(ser)

    reSetStatus(ser)

    moveBack(ser)

    turn(ser, 180 * 10)

    moveDiatance(ser, 110)

    moveBack(ser)

    turn(ser, 0)

    path = redis_cli.get(PATH_PLANNING_KEY)

    if LEFT_PATH_PLANNING == path:

        redis_cli.set(PATH_PLANNING_KEY, RIGHT_PATH_PLANNING)

    else:

        redis_cli.set(PATH_PLANNING_KEY, LEFT_PATH_PLANNING)

    redis_cli.set('action', 'false')

    redis_cli.set("correct", "false")

    redis_cli.set('mission', 'complete')


@app.route("/vehicle/getVehicleInfo", methods=['GET'])
def getVehicleInfo():
    metadata = {}

    metadata[PATH_PLANNING_KEY] = redis_cli.get(PATH_PLANNING_KEY)

    metadata['correct'] = redis_cli.get('correct')

    metadata['forwardSpeed'] = redis_cli.get('forwardSpeed')

    metadata['brushSpeed'] = redis_cli.get('brushSpeed')

    voltage = redis_cli.get('voltage')

    if voltage != None:
        metadata['voltage'] = voltage

    return json.dumps(metadata)


@app.route("/vehicle/togglePathPlanning/<string:path>")
def togglePathPlanning(path):
    redis_cli.set(PATH_PLANNING_KEY, path)
    response = make_response("1")
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response


# 是否纠偏
@app.route("/vehicle/toggleTracking/<string:tracking>")
def toggleTracking(tracking):
    if "0" == tracking:
        redis_cli.set("correct", "true")
    else:
        redis_cli.set("correct", "false")
    response = make_response("1")
    response.headers.add('Access-Control-Allow-Origin', '*')

    return response


# 修改速度
@app.route("/vehicle/adjustSpeed/<int:speed>", methods=['GET'])
def adjust_speed(speed):
    redis_cli.set("forwardSpeed", speed)
    preBuildCommand()
    if redis_cli.get('reverse') == 'true':
        setXSpeed(-speed)
    else:
        setXSpeed(speed)
    command[8] = 0x00
    command[9] = 0x00
    command[10] = int(redis_cli.get("brushSpeed"))
    duplicateWriteCmd(ser, command)
    response = make_response("1")
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response


# 发送调整速度命令
def sendCommandSetXSpeed(speed):
    setXSpeed(speed)
    duplicateWriteCmd(ser,command)

# 修改滚刷速度
@app.route("/vehicle/adjustBrushSpeed/<int:speed>", methods=['GET'])
def adjust_brush(speed):
    redis_cli.set("brushSpeed", speed)
    preBuildCommand()
    setBrushSpeed(speed)
    duplicateWriteCmd(ser, command)
    response = make_response("1")
    return response


def goCommand(speed=300):
    reSetStatus(ser)
    setStatus(1)
    setPowerOn(1)
    setHWstatus(0, 0, 0, 0, 0)
    setXSpeed(speed)
    command[17] = tem_listener(command, 17)
    logger.warn(' '.join(format(x, '02x') for x in command))
    duplicateWriteCmd(ser, command)

def correctByRTKTest():
    logger.warn("开启RTK纠偏线程")
    global global_cur_taskPointTest
    global global_open_rtk
    global global_cur_rtk_lat
    global global_cur_rtk_lon
    global global_cur_rtk_heading

    rtk_generator = util.readRTK_v2(ser_rtk_params)
    try:
        # —— 1. 主循环
        for lat, lon, heading_deg in rtk_generator:
            global_cur_rtk_lat = lat
            global_cur_rtk_lon = lon
            global_cur_rtk_heading = heading_deg
            # 是否启动RTK纠偏,0:表示没有开启RTK纠偏，1表示开启
            if global_open_rtk == 0:
                time.sleep(0.01)
                continue
            logger.info("lat={},lon={},heading={}".format(lat, lon, heading_deg))
            if lat is None or lon is None:
                logger.error("RTK数据获取有问题，请检查精度！")
                continue
            # 如果值为空，则跳过下面步骤，直接到下一个循环
            if not global_cur_taskPointTest:
                time.sleep(0.01)
                continue
            else:
                start_lat = global_cur_taskPointTest['startLat']
                start_lon = global_cur_taskPointTest['startLon']
                target_lat = global_cur_taskPointTest['endLat']
                target_lon = global_cur_taskPointTest['endLon']
            const_target_h = float(global_cur_taskPointTest['target_heading'])
            # 走到这一步说明RTK打开成功
            global_open_rtk = 1
            # 计算期望航向角（从当前位置指向目标点）
            distance_to_target, target_heading = util.get_distance_angle(lat, lon, target_lat, target_lon)
            heading_error = float(const_target_h) - float(heading_deg)
            # 计算最短偏差
            heading_error = (heading_error + 180) % 360 - 180
            # 只有直行，才发送纠偏指令
            cte = util.cross_track_error(start_lat, start_lon, target_lat, target_lon, lat, lon)
            stree_output = heading_error * 10 - int(1000 * cte)
            # 发送电机控制指令
            # 正数左轮快，向右偏，负数右轮快，向左偏
            setZSpeed(stree_output)
            duplicateWriteCmd(ser, command)
            # 打印状态
            logger.warn("航向角:{:.2f} | 当前航向角:{:.2f} | heading_error:{:.2f}横向偏差:{:.2f}距离目标:{:.2f}m | 转向输出: {:.2f}"
                        .format(const_target_h, heading_deg, heading_error, cte, distance_to_target, stree_output))
            # 更新路径进度
            if distance_to_target <= 0.05:
                global_cur_taskPointTest = {}
                sendBraking()
            # 防止cup资源占满
            time.sleep(0.01)
    except Exception as e:
        logger.error(e)
        time.sleep(0.5)


# 通过RTK实现直行
@app.route("/vehicle/goByRTK", methods=['POST'])
def goByRTK():
    data = request.get_json()
    distance = float(data['dis'])
    head_target = float(data['heading'])
    reset_odometer(ser)
    # 直行
    justMoveByRTK(distance, head_target)
    # endLat, endLon = util.get_B_GPS(global_cur_rtk_lat, global_cur_rtk_lon, distance, head_target)
    # moveByRTK(endLat,endLon)

    return jsonify(global_cur_taskPoint)


# 根据当前任务使用rtk纠偏并直行
def justMoveByRTK(distance, head_target):
    global global_cur_taskPoint
    global global_go,global_interval
    endLat, endLon = util.get_B_GPS(global_cur_rtk_lat, global_cur_rtk_lon, distance, head_target)
    # 获取当前任务开始点和结束点的经纬度
    global_cur_taskPoint = {"startLat": global_cur_rtk_lat, "startLon": global_cur_rtk_lon, "endLat": endLat,
                                "endLon": endLon, "heading": head_target}

    goCommand(100)
    # 开启RTK纠偏
    global_go = 1
    global_interval = 0
    # 如果global_go==0，则说明直行结束
    startTime = time.time()
    # 如果global_go = 1，则说明直行未结束
    while global_go == 1:
        # 表示到边了
        if getEdge() == 0:
            break
        endTime = time.time()
        # 获取时间间隔
        global_interval = endTime - startTime
        time.sleep(0.1)
    sendBraking()


def autoToPointByRTK():
    # 直行
    # justMoveByRTK(3, 273)
    # 左转
    turn(ser, 90)
    # 直行
    # justMoveByRTK(1, 4)

# 通过RTK返回充电桩
@app.route("/vehicle/returnToPointByRTK", methods=['GET'])
def returnToPointByRTK():
    sendBraking()
    thread = threading.Thread(target=returnToPointByRTKThread)
    thread.start()
    response = make_response("开启自动执行任务")
    return response
def returnToPointByRTKThread():
    global global_go,global_status
    set_current_action('return_to_point')
    redis_cli.set('curTaskIndex', 0)
    charginPileLat = float(redis_cli.hget('taskParams','chargingPileLat'))
    chargingPileLon = float(redis_cli.hget('taskParams','chargingPileLon'))
    # 原点航向角，用于起始点转正
    originHeading = float(redis_cli.hget('taskParams','originHeading'))

    global_go = 0
    taskParams = redis_cli.hgetall("taskParams")
    # 初始航向角
    startHeading = float(taskParams.get('heading'))
    originLat = float(taskParams.get('startLat'))
    originLon = float(taskParams.get('startLon'))
    start_angle_rtk = float(taskParams.get('heading'))
    backLength = int(taskParams.get('startToChargingPilePointLength'))
    # 返回路线任务
    routes = []
    # 判断当前到那个任务了
    json_item = redis_cli.lindex('taskList', 0)
    json_next_item = redis_cli.lindex('taskList', 1)
    next_item = json.loads(json_next_item)
    if json_item:
        item = json.loads(json_item)
        areaNumber = item['areaNumber']

        if areaNumber == 1:
            if item['angle'] == 90:
                startLat, startLon = item['startLat'], item['startLon']
                item['startLat'] = item['endLat']
                item['startLon'] = item['endLon']
                item['endLat'] = startLat
                item['endLon'] = startLon
                item['angle'] = 270
                item['heading'] = (item['heading'] + 180)%360

                routes.append(item)
                route = {'startLat':item['endLat'], 'startLon':item['endLon'], 'endLat':originLat,
                         'endLon':originLon,'angle':180,'heading':(180+start_angle_rtk)%360}
                routes.append(route)

            elif item['angle'] == 180 and next_item['angle'] == 270:
                routes.append(item)
                routes.append(next_item)
                route = {'startLat': next_item['endLat'], 'startLon': next_item['endLon'], 'endLat': originLat,
                         'endLon': originLon, 'angle': 180,'heading':(180+start_angle_rtk)%360}
                routes.append(route)
                # 要删除3个任务
                for _ in range(3):
                    redis_cli.lpop("taskList")
            elif item['angle'] == 180 and next_item['angle'] == 90:
                route = {'startLat': item['endLat'], 'startLon': item['endLon'], 'endLat': originLat,
                         'endLon': originLon, 'angle': 180,'heading':(180+start_angle_rtk)%360}
                routes.append(route)
                # 要删除1个任务
                redis_cli.lpop("taskList")
            elif item['angle'] == 270:
                routes.append(item)
                route = {'startLat': item['endLat'], 'startLon': item['endLon'], 'endLat': originLat,
                         'endLon': originLon, 'angle': 180,'heading':(180+start_angle_rtk)%360}
                routes.append(route)
                # 要删除2个任务
                for _ in range(2):
                    redis_cli.lpop("taskList")
        else:
            if item['angle'] == 90:
                startX,startY = item['startX'], item['startY']

                startLat, startLon = item['startLat'], item['startLon']
                item['startLat'] = item['endLat']
                item['startLon'] = item['endLon']
                item['endLat'] = startLat
                item['endLon'] = startLon
                item['angle'] = 270
                item['heading'] = (item['heading'] + 180) % 360
                routes.append(item)

                # 构建(0,startX),将其转化为经纬度
                endLat, endLon = util.local_rotated_xy_to_latlon_precise(originLat, originLon, startX / 100.0,0, startHeading)
                route = {'startLat': item['endLat'], 'startLon': item['endLon'], 'endLat': endLat,
                         'endLon': endLon, 'angle': 180, 'heading': (180 + start_angle_rtk) % 360}
                routes.append(route)

                route2 = {'startLat': endLat, 'startLon': endLon, 'endLat': originLat,
                         'endLon': originLon, 'angle': 270, 'heading': (270 + start_angle_rtk) % 360}
                routes.append(route2)
            elif item['angle'] == 180 and next_item['angle'] == 270:
                routes.append(item)
                routes.append(next_item)
                x,y = next_item['endX'], next_item['endY']
                # 构建(0,startX),将其转化为经纬度
                endLat, endLon = util.local_rotated_xy_to_latlon_precise(originLat, originLon, x / 100.0, 0,
                                                                         startHeading)
                route = {'startLat': next_item['endLat'], 'startLon': next_item['endLon'], 'endLat': endLat,
                         'endLon': endLon, 'angle': 180, 'heading': (180 + start_angle_rtk) % 360}
                routes.append(route)

                route2 = {'startLat': endLat, 'startLon': endLon, 'endLat': originLat,
                         'endLon': originLon, 'angle': 270, 'heading': (270 + start_angle_rtk) % 360}
                routes.append(route2)
                # 要删除3个任务
                for _ in range(3):
                    redis_cli.lpop("taskList")
            elif item['angle'] == 180 and next_item['angle'] == 90:
                route = {'startLat': item['endLat'], 'startLon': item['endLon'], 'endLat': originLat,
                         'endLon': originLon, 'angle': 180,'heading':(180+start_angle_rtk)%360}
                routes.append(route)
                x,y = item['endX'], item['endY']
                endLat, endLon = util.local_rotated_xy_to_latlon_precise(originLat, originLon, x / 100.0, 0,
                                                                         startHeading)
                route2 = {'startLat': item['endLat'], 'startLon': item['endLon'], 'endLat': endLat,
                         'endLon': endLon, 'angle': 270, 'heading': (270 + start_angle_rtk) % 360}
                routes.append(route2)
                # 要删除1个任务
                redis_cli.lpop("taskList")
            elif item['angle'] == 270:
                routes.append(item)
                x,y = item['endX'], item['endY']
                endLat, endLon = util.local_rotated_xy_to_latlon_precise(originLat, originLon, x / 100.0, 0,
                                                                         startHeading)
                route = {'startLat': item['endLat'], 'startLon': item['endLon'], 'endLat': endLat,
                         'endLon': endLon, 'angle': 180,'heading':(180+start_angle_rtk)%360}
                routes.append(route)

                route2 = {'startLat': endLat, 'startLon': endLon, 'endLat': originLat,
                          'endLon': originLon, 'angle': 270, 'heading': (270 + start_angle_rtk) % 360}
                routes.append(route2)
                # 要删除2个任务
                for _ in range(2):
                    redis_cli.lpop("taskList")
    logger.warn(routes)
    goByRoutes(routes)
    doParking()
    # 进充电桩
    intoGarage(backLength)
# 根据任务路线行走
def goByRoutes(routes):
    global global_go
    redis_cli.set("mission", "working")
    redis_cli.set("parking", "0")
    # 执行任务
    for index, task in enumerate(routes):

        angle = task['angle']
        startLat, startLon = task['startLat'], task['startLon']

        heading = task['heading']
        endLat = task['endLat']
        endLon = task['endLon']
        # logger.warn("转向：{}".format(angle))
        turn(ser, angle * 10)
        speed = 350
        if angle == 180:
            speed = 200
        result = pointToPointByRTK(startLat, startLon, endLat, endLon, heading,speed)
        if result == 0:
            global_go = 0
            break


# 将任务中的x,y轴坐标转为以起始点为原点的经纬度坐标
def converterXY(task):
    global global_startLat,global_startLon,global_originLat,global_originLon

    startLat,startLon = util.local_rotated_xy_to_latlon_precise(global_originLat, global_originLon, task['startX'] / 100.0,task['startY'] / 100.0, global_start_angle_rtk)
    endLat, endLon = util.local_rotated_xy_to_latlon_precise(global_originLat, global_originLon, task['endX'] / 100.0,task['endY'] / 100.0, global_start_angle_rtk)

    task['startLat'] = startLat
    task['startLon'] = startLon
    task['endLat'] = endLat
    task['endLon'] = endLon

# 通过RTK自动清扫
@app.route("/vehicle/autoDriveByRTK", methods=['GET'])
def autoDriveByRTK():
    thread = threading.Thread(target=autoDriveByRTKThread)
    thread.start()
    response = make_response("开启自动执行任务")
    return response

# 根据当前未执行的任务，构建任务
def buildTask(taskParams):
    resultTask = []
    # 初始航向角
    start_angle_rtk = float(taskParams.get('heading'))
    originLat = float(taskParams.get('startLat'))
    originLon = float(taskParams.get('startLon'))
    # 判断当前到那个任务了,需要将没有清扫的任务继续执行，清扫过的地区就不用清扫了
    json_item = redis_cli.lindex('taskList', 0)
    if json_item:
        item = json.loads(json_item)
        # 如果id=1，就不需要任何处理,直接从文件中读取文件就可以了
        if item['areaNumber'] == 1 and item['id'] == 1:
            return resultTask
        # 读取缓存中的全部任务
        unDoTaskList_str = redis_cli.lrange('taskList', 0, -1)
        unDoTaskList  = []
        for unDoTask in unDoTaskList_str:
            task = json.loads(unDoTask)
            unDoTaskList.append(task)
        areaNumber = item['areaNumber']
        startX, startY = item['startX'], item['startY']
        startLat,startLon = item['startLat'], item['startLon']

        if areaNumber != 1:
            # 构建路径，(0,startX),(startX,startY)
            # 计算经纬度
            endLat, endLon = util.local_rotated_xy_to_latlon_precise(originLat, originLon, startX / 100.0,
                                                                     0, start_angle_rtk)
            route = {'startLat': originLat, 'startLon': originLon, 'endLat': endLat,
                     'endLon': endLon, 'angle': 270, 'heading': (270 + start_angle_rtk) % 360,
                     'startX': 0, 'startY': 0, 'endX': startX, 'endY': 0, 'length': startX,'areaNumber': areaNumber,'mode':1,'turn_back_len':5,'back_len':5}
            resultTask.append(route)
            endLat2, endLon2 = util.local_rotated_xy_to_latlon_precise(originLat, originLon, startX / 100.0,
                                                                       startY, start_angle_rtk)
            route2 = {'startLat': endLat, 'startLon': endLon, 'endLat': endLat2,
                      'endLon': endLon2, 'angle': 0, 'heading': (0 + start_angle_rtk) % 360,
                      'startX': startX, 'startY': 0, 'endX': startX, 'endY': startY, 'length': startY,'areaNumber': areaNumber,'mode':1,'turn_back_len':5,'back_len':5}
            resultTask.append(route2)
        else:
            route = {'startLat': originLat, 'startLon': originLon, 'endLat': startLat,
                     'endLon': startLon, 'angle': 0, 'heading': (0 + start_angle_rtk) % 360,
                     'startX': 0, 'startY': 0, 'endX': 0, 'endY': startY, 'length': startY,'areaNumber': areaNumber,'mode':1,'turn_back_len':5,'back_len':5}
            resultTask.append(route)

        resultTask = resultTask + unDoTaskList

    return resultTask

# 根据当前位置获取任务名称
def getTaskNameByLoc():
    # 遍历所有任务名称
    taskNameSet = redis_cli.smembers('taskNameSet')
    taskNameList = list(taskNameSet)
    for taskName in taskNameList:
        start_lat_lon_str = redis_cli.hget('loc_start_lat_lon',taskName)
        if start_lat_lon_str:
            start_lat_lon = json.loads(start_lat_lon_str)
            dis,heading = util.get_distance_angle(global_cur_rtk_lat,global_cur_rtk_lon,start_lat_lon['startLat'],start_lat_lon['startLon'])
            # 如果当前坐标距离该任务的起始点坐标小于2m，则获取该任务
            if dis <= 2:
                return taskName
    return None

def autoDriveByRTKThread():
    global global_status
    global taskList  # 申明使用全局变量
    global global_pointToPoint_flag,global_doCleanThreadStop,global_go,global_originLat,global_originLon
    set_current_action('auto_drive')
    redis_cli.set('curTaskIndex', 0)
    taskParams = redis_cli.hgetall("taskParams")
    # 获取当前坐标点，判断当前点位是否在充电桩中
    chargingPileLat = float(taskParams.get('chargingPileLat'))
    chargingPileLon = float(taskParams.get('chargingPileLon'))
    backLength = int(taskParams.get('startToChargingPilePointLength'))
    lastTaskBackLength = int(taskParams.get('lastTaskBackLength'))
    # 起始点经纬度
    global_originLat = float(taskParams.get('startLat'))
    global_originLon = float(taskParams.get('startLon'))
    # 初始航向角
    # initHeading = taskObj['heading']
    # 起始点航向角,用于位置转正
    originHeading = float(taskParams.get('originHeading'))
    # 如果不等于0，原点和起始点不是同一点
    if backLength != 0:
        if global_status == 'goCharging':
            goOutGarage(backLength)
        else:
            if isGarage(chargingPileLat, chargingPileLon):
                goOutGarage(backLength)
    # 获取是否开启定点找寻任务功能
    if redis_cli.get("isOpenFindTaskName") == '1':
        taskName = getTaskNameByLoc()
        logger.warn("搜索当前任务....")
        if taskName:
            logger.warn("搜索到当前任务，任务名称为:{}".format(taskName))
        else:
            logger.warn("未搜索到当前任务")
            _mark_runtime_blocked('TASK_NAME_NOT_FOUND', '当前位置没有匹配到可执行任务')
            return
        # 然后将taskName.json文件中的内容复制到config.json中
        fileName = taskName + '.json'
        # 将文件内容复制到执行任务的文件中
        with open(fileName, 'r') as src, open('config.json', 'w') as dst:
            for line in src:
                dst.write(line)
        redis_cli.set('currentTaskName', taskName)
        redis_cli.set('curTaskIndex', 0)
        # 将当前任务文件中的参数信息同步到redis中
        syncCurTaskFileToRedis()
    # 如果小车已经在工作了，就没有办法再启动工作
    if global_status == 'working':
        logger.warn("小车已经在工作了，无法再开启工作")
        _mark_runtime_blocked('ALREADY_WORKING', '小车当前已经在执行任务，请勿重复启动')
        return 0
    global_status = 'working'
    redis_cli.set("mission", "working")
    redis_cli.set("parking", "0")
    global_doCleanThreadStop = 0
    _mark_runtime_running('自动清扫启动成功，任务执行中')

    # 根据缓存中是否存在任务，来构建新的任务
    resultTask = buildTask(taskParams)
    # 如果缓存中没有任务，则读取当前任务中的数据
    if len(resultTask) == 0:
        taskObj = util.readConfig("config.json")
        taskList = taskObj['taskList']
    else:
        taskList = resultTask
    # 清除当前航向角记录
    reset_clean_mode(ser)
    time.sleep(0.02)
    # 如果启动自动清扫任务，那redis中的任务列表就要被清除，然后再初始化
    redis_cli.delete('taskList')
    logger.warn(taskList)
    for item in taskList:
        # 将字典转为JSON字符串存储
        redis_cli.rpush('taskList', json.dumps(item))


    # # 位置校验
    if turnCheckPoint(originHeading) == 0:
        _mark_runtime_blocked('START_HEADING_CHECK_FAILED', '起始姿态校验失败，自动清扫未启动')
        doParking()
        return
    time.sleep(0.02)
    # 重置陀螺仪(记录当前航向角)
    switch_on_clean_mode(ser)
    # 点到点直线行走是否停止标识
    global_pointToPoint_flag = 0
    # 执行任务
    for index,task in enumerate(taskList):
        logger.warn("执行任务{}".format(index + 1))
        turn_back_len = task['turn_back_len']
        back_len = task['back_len']
        angle = task['angle']
        startLat, startLon = task['startLat'], task['startLon']
        heading = task['heading']
        endLat = task['endLat']
        endLon = task['endLon']
        mode = task['mode']

        if index != 0:
            turn(ser, angle*10)
            if redis_cli.get('parking') == '1':
                global_doCleanThreadStop = 1
                break
            if mode == 1:
                # moveBack(ser, turn_back_len)
                if redis_cli.get('parking') == '1':
                    global_doCleanThreadStop = 1
                    break
        speed = 350
        if angle == 180:
            speed = 200
        result = pointToPointByRTK(startLat,startLon,endLat,endLon,heading,speed)
        if redis_cli.get('parking') == '1':
            global_go = 0
            # 表示自动清扫线程停止
            global_doCleanThreadStop = 1
            break
        else:
            if mode == 1 and index != len(taskList)-1:
                # moveBack(ser,back_len)
                if redis_cli.get('parking') == '1':
                    global_doCleanThreadStop = 1
                    break
            if mode == 1 and index == len(taskList)-1:
                if lastTaskBackLength != '0':
                    moveBack(ser, lastTaskBackLength)
            # 在redis中设置是否是最后一个任务，如果是则设置为1，不是则设置为0
            if index == len(taskList) - 2:
                redis_cli.set("lastTask", 1)
            else:
                redis_cli.set("lastTask",0)
            logger.warn("删除任务{}".format(index + 1))
            redis_cli.lpop("taskList")
    logger.warn("自动行驶结束")
    redis_cli.incr("doTaskCounter")
    doParking()
    # 如果自动清扫被停止，则不继续运行
    if global_doCleanThreadStop == 0:
        if backLength != 0:
            # intoGarage(chargingPileLat,chargingPileLon,originHeading)
            intoGarage(backLength)




# 根据充电桩的lat,lon和当前的lat,lon距离是否大于1.3m,如果小于1.3m,则表明小车在车库中，返回1，否则返回0
def isGarage(lat,lon):
    # 如果当前没有rtk信号，则说明小车在充电桩中，也可以直接出仓
    logger.info("当前经纬度：{}".format(lat))
    if global_cur_rtk_lat is None:
        return 1
    dis,angle = util.get_distance_angle(global_cur_rtk_lat,global_cur_rtk_lon,lat,lon)
    logger.warn(dis)
    if dis < 1:
        return 1
    else:
        return 0
# 进入充电桩
def intoGarage(backLength):
    global global_status
    logger.warn('进充电桩')
    reset_odometer(ser)
    turn(ser, 180 * 10)
    # justMoveByRTK(1.20,186)
    goByLength(ser, backLength, 100)
    # 关闭视觉纠偏
    redis_cli.set("correct", "false")
    # 充电命令
    reset_odometer(ser)
    setStatus(5)
    logger.warn(' '.join(format(x, '02x') for x in command))
    duplicateWriteCmd(ser, command)

    global_status = 'goCharging'
    # moveByRTK(chargingPileLat, chargingPileLon, (originHeading + 180) % 360)
# 出充电桩
def goOutGarage(backLength):
    global global_status
    global_status = 'move back'
    reset_odometer(ser)
    redis_cli.set("mission", "working")
    moveBack(ser, backLength)
    # 如果后退没有到达，则不往下执行
    if getDistanceArrive() == 0:
        return

@app.route("/vehicle/intoGarage", methods=['GET'])
def intoGarage_api():
    # taskParams = redis_cli.hgetall("taskParams")
    # chargingPileLat = float(taskParams.get('chargingPileLat'))
    # chargingPileLon = float(taskParams.get('chargingPileLon'))
    # originHeading = int(taskParams.get('originHeading'))
    # redis_cli.set("mission","working")
    # reset_odometer(ser)
    # moveByRTK(chargingPileLat, chargingPileLon, (originHeading + 180) % 360)
    # 开启视觉纠偏
    # redis_cli.set("correct","true")
    # redis_cli.set("mission", "working")
    reset_odometer(ser)
    goByLength(ser,100,100)
    # 关闭视觉纠偏
    redis_cli.set("correct","false")
    # 充电命令
    setStatus(5)
    logger.warn(' '.join(format(x, '02x') for x in command))
    duplicateWriteCmd(ser, command)
    # moveDiatance(ser,160,100)
    # justMove(ser)
    response = make_response("启动进仓线程")
    return response

@app.route("/vehicle/setStatus", methods=['GET'])
def setStatus_api():
    reSetStatus(ser)
    setStatus(5)
    logger.warn(' '.join(format(x, '02x') for x in command))
    duplicateWriteCmd(ser, command)
    response = make_response("启动充电")
    return response

# 自动清扫
@app.route("/vehicle/autoDrive", methods=['GET'])
def auto_driving():
    redis_cli.set("reverse", "false")
    global global_doCleanThreadStop
    validation = _validate_auto_drive_request()
    if not validation.get('success'):
        _mark_runtime_blocked(
            validation.get('faultState', 'AUTO_DRIVE_BLOCKED'),
            validation.get('message', '启动条件未通过'),
            validation.get('data')
        )
        return jsonify(validation)

    global_doCleanThreadStop = 0
    _mark_runtime_ready('启动条件通过，正在创建自动清扫线程', validation.get('data'))

    thread = threading.Thread(target=autoDriveByRTKThread)
    thread.start()

    return jsonify({
        'success': True,
        'message': '启动自动清扫任务线程',
        'data': validation.get('data'),
    })


# 根据RTK获取航向角偏差值
def getAngleByRTK():
    heading = float(redis_cli.hget('taskParams','heading'))
    logger.warn("初始航向角：{}".format(heading))
    logger.warn("heading={},target_heading={}".format(global_cur_rtk_heading, heading))
    angle = global_cur_rtk_heading - heading
    return util.normalize_angle(angle)

# 校验初始航向角
@app.route("/vehicle/checkInitHeading", methods=['GET'])
def checkInitHeading():
    # 计算第二个点到第三个点的航向角
    second_lat = float(redis_cli.hget('secondPoint','lat'))
    second_lon = float(redis_cli.hget('secondPoint','lon'))

    third_lat = float(redis_cli.hget('thirdPoint', 'lat'))
    third_lon = float(redis_cli.hget('thirdPoint', 'lon'))

    dis, angle = util.get_distance_angle(second_lat, second_lon, third_lat, third_lon)

    result = {"success": True, "msg": "获取校验数据", "initHeading": global_start_angle_rtk,"checkHeading": angle}
    return jsonify(result)
# 设置点位置信息
@app.route("/vehicle/setPoint", methods=['GET'])
def setPoint():
    lat = float(request.args.get('lat'))
    lon = float(request.args.get('lon'))
    flag = request.args.get('flag')
    if flag == 'secondPoint':
        redis_cli.hset('secondPoint','lat',lat)
        redis_cli.hset('secondPoint','lon',lon)
    elif flag == 'thirdPoint':
        redis_cli.hset('thirdPoint', 'lat', lat)
        redis_cli.hset('thirdPoint', 'lon', lon)
    elif flag == 'fourPoint':
        redis_cli.hset('fourPoint', 'lat', lat)
        redis_cli.hset('fourPoint', 'lon', lon)
    return make_response("设置成功")

# 获取当前位置
@app.route("/vehicle/getCurLocation", methods=['GET'])
def getCurLocation():
    result = {"lat":global_cur_rtk_lat,"lon":global_cur_rtk_lon,"heading":global_cur_rtk_heading}
    return jsonify(result)
# 设置充电桩信息
@app.route("/vehicle/setCharginPileInfo", methods=['GET'])
def setCharginPileInfo():
    global global_originLat,global_originLon,startToChargingPilePointLength
    lat = float(request.args.get('lat'))
    lon = float(request.args.get('lon'))
    # 起始点到充电桩的距离，如果为0，则没有充电桩
    # if request.args.get('startToChargingPilePointLength') != '':
    #     startToChargingPilePointLength = int(request.args.get('startToChargingPilePointLength'))
    redis_cli.hset('taskParams','chargingPileLat',lat)
    redis_cli.hset('taskParams','chargingPileLon',lon)
    # redis_cli.hset('taskParams','startToChargingPilePointLength',startToChargingPilePointLength)
    result = {"success":True,"msg":"设置成功","chargingPileLat":lat,"chargingPileLon":lon,'startToChargingPilePointLength':startToChargingPilePointLength}
    return jsonify(result)
# 设置原点，也就是起始点
@app.route("/vehicle/setOrigin", methods=['GET'])
def setOrigin():
    global global_originLat,global_originLon,startToChargingPilePointLength
    global_originLat = float(request.args.get('originLat'))
    global_originLon = float(request.args.get('originLon'))

    redis_cli.hset('taskParams','startLat',global_originLat)
    redis_cli.hset('taskParams','startLon',global_originLon)
    result = {"success":True,"msg":"设置成功","global_originLat":global_originLat,"global_originLon":global_originLon}
    return jsonify(result)
# 设置目标航向角，根据当前位置和原点设置目标航向角
@app.route("/vehicle/setHeading", methods=['GET'])
def setHeading():
    global global_start_angle_rtk
    lat = float(request.args.get('lat'))
    lon = float(request.args.get('lon'))
    dis,angle = util.get_distance_angle(global_originLat,global_originLon,lat,lon)
    # 如果距离小于2m,则设置失败，规定设置初始航向角时，两个点的距离必须不得小于2m
    if dis < 1:
        result = {"success":0,"msg": "设置失败，距离原点小于1m"}
    else:
        global_start_angle_rtk = round(angle,2)
        redis_cli.hset('taskParams', 'heading', global_start_angle_rtk)
        result = {"success":1,"msg":"设置成功","global_start_angle_rtk":global_start_angle_rtk}
    return jsonify(result)

def correctByRTK():
    logger.warn("开启RTK纠偏线程")
    global global_cur_taskPoint
    global global_is_need_rtk
    global global_open_rtk
    global global_cur_rtk_lat
    global global_cur_rtk_lon
    global global_cur_rtk_heading

    rtk_generator = util.readRTK_v2(ser_rtk_params)
    try:
        # —— 1. 主循环
        for lat, lon, heading_deg in rtk_generator:
            # 表示清扫线程结束，停止当前纠偏线程
            if global_doCleanThreadStop:
                # 将当前任务的开始和结束点的经纬度设置为空
                global_cur_taskPoint = {}
                logger.warn("关闭RTK纠偏线程")
                util.closeSerRtk()
                break
            global_cur_rtk_lat = lat
            global_cur_rtk_lon = lon
            global_cur_rtk_heading = heading_deg
            # 是否启动RTK纠偏,0:表示没有开启RTK纠偏，1表示开启
            if global_open_rtk == 0:
                time.sleep(0.01)
                continue
            logger.info("lat={},lon={},heading={}".format(lat, lon, heading_deg))
            if lat is None or lon is None:
                logger.error("RTK数据获取有问题，请检查精度！")
                # global_open_rtk = 0
                continue
            # 如果值为空，则跳过下面步骤，直接到下一个循环
            if not global_cur_taskPoint:
                # logger.warn("当前任务为空")
                # global_open_rtk = 0
                time.sleep(0.01)
                continue
            else:
                start_lat = global_cur_taskPoint['startLat']
                start_lon = global_cur_taskPoint['startLon']
                target_lat = global_cur_taskPoint['endLat']
                target_lon = global_cur_taskPoint['endLon']
                # dis_total, target_heading = util.get_distance_angle(start_lat, start_lon, target_lat, target_lon)
            target_heading = float(global_cur_taskPoint['heading'])
            # 走到这一步说明RTK打开成功
            global_open_rtk = 1
            # 计算期望航向角（从当前位置指向目标点）
            distance_to_target, target_heading_cur = util.get_distance_angle(lat, lon, target_lat, target_lon)
            heading_error = float(target_heading) - float(heading_deg)
            # 计算最短偏差
            heading_error = (heading_error + 180) % 360 - 180
            # 只有直行，才发送纠偏指令
            cte = util.cross_track_error(start_lat, start_lon, target_lat, target_lon, lat, lon)
            stree_output = heading_error * 10 - int(1000 * cte)
            if global_go == 1:
                if abs(heading_error) > 1 or abs(cte) > 0.02:
                    # logger.warning("视觉纠偏关闭，RTK纠偏开启")
                    redis_cli.set("correct", "false")
                    # 发送电机控制指令
                    # 正数左轮快，向右偏，负数右轮快，向左偏
                    setZSpeed(stree_output)
                    duplicateWriteCmd(ser, command)
                    # 打印状态
                    logger.warn("航向角:{:.2f} | 当前航向角:{:.2f} | heading_error:{:.2f}横向偏差:{:.2f}距离目标:{:.2f}m | 转向输出: {:.2f}"
                                .format(target_heading, heading_deg, heading_error, cte, distance_to_target,
                                        stree_output))
                else:
                    redis_cli.set("correct", "true")
                    pass
            # else:
            #     redis_cli.set("correct", "true")
            # 更新路径进度
            if distance_to_target < 0.03:
                global_cur_taskPoint = {}
                logger.warn("路径直行结束")
                # redis_cli.set("correct","true")
            # 防止cup资源占满
            time.sleep(0.01)
    except Exception as e:
        logger.error(e)
        time.sleep(0.5)
        # RTK纠偏发生错误，使用视觉纠偏
        redis_cli.set("correct", "true")


# 急停
@app.route("/vehicle/parking", methods=['GET'])
def parking():
    global global_pointToPoint_flag,global_go,global_status
    set_current_action('parking')
    redis_cli.set("ultraSonic", "false")
    redis_cli.set("mission", "complete")
    redis_cli.set('action', 'false')
    redis_cli.set("correct", "false")
    redis_cli.set('moveJudge', 'false')
    redis_cli.set('reverse', 'false')
    redis_cli.set('enterGarage', 'false')
    redis_cli.set('exitGarage', 'false')
    # 设置暂停
    redis_cli.set('parking', 1)
    # 停止清扫线程,打断点到点执行任务
    global_pointToPoint_flag = 1
    global_go = 0
    sendBraking()
    global_status = 'active'
    redis_cli.set('curTaskIndex', 0)
    _mark_runtime_idle('已执行停车指令')
    response = make_response("1")

    return response


def doParking():
    global global_pointToPoint_flag, global_go
    set_current_action('parking')
    redis_cli.set('parking', 1)

    redis_cli.set("ultraSonic", "false")
    redis_cli.set("mission", "complete")
    redis_cli.set('action', 'false')
    redis_cli.set("correct", "false")
    redis_cli.set('moveJudge', 'false')
    redis_cli.set('reverse', 'false')
    redis_cli.set('enterGarage', 'false')
    redis_cli.set('exitGarage', 'false')
    # 停止清扫线程,打断点到点执行任务
    global_pointToPoint_flag = 1
    global_go = 0
    sendBraking()
    redis_cli.set('curTaskIndex', 0)
    _mark_runtime_idle('任务已停止并进入停车状态')

# 删除缓存任务
@app.route('/vehicle/delTaskList', methods=['GET'])
def delTaskList():
    redis_cli.delete('taskList')
    result = {"success": True, "msg": "删除成功"}
    return jsonify(result)

# 查看参数接口
@app.route('/vehicle/selectParams', methods=['GET'])
def selectParams():
    data = redis_cli.hgetall('taskParams')
    result = {"success": True, "msg": "获取成功"}
    if data == None:
        result['data'] = []
    else:
        result['data'] = data
    return jsonify(result)


# 保存参数接口
@app.route('/vehicle/saveParams', methods=['POST'])
def saveParams():
    data = request.get_json()
    redis_cli.hset('taskParams', "goBackLen", data['goBackLen'])
    redis_cli.hset('taskParams', "goLeftOrRightBackLen", data['goLeftOrRightBackLen'])
    redis_cli.hset('taskParams', "turnBackLen", data['turnBackLen'])
    redis_cli.hset('taskParams', "panelWidth", data['panelWidth'])
    redis_cli.hset('taskParams', "panelHeight", data['panelHeight'])
    # redis_cli.hset('taskParams', "upOrDownBridgeLen", data['upOrDownBridgeLen'])
    redis_cli.hset('taskParams', "leftOrRightBridgeLen", data['leftOrRightBridgeLen'])
    redis_cli.hset('taskParams', "voltageWarn", data['voltageWarn'])
    # 初始航向角
    redis_cli.hset('taskParams', "heading", data['heading'])
    # 起始点经纬度
    redis_cli.hset('taskParams', "startLat", data['startLat'])
    redis_cli.hset('taskParams', "startLon", data['startLon'])
    # 充电桩经纬度
    redis_cli.hset('taskParams', "chargingPileLat", data['chargingPileLat'])
    redis_cli.hset('taskParams', "chargingPileLon", data['chargingPileLon'])
    # 起始点到充电桩位置
    redis_cli.hset('taskParams', "startToChargingPilePointLength", data['startToChargingPilePointLength'])
    # 最后一个任务结束后的后退距离
    redis_cli.hset('taskParams', "lastTaskBackLength", data['lastTaskBackLength'])

    redis_cli.hset('taskParams', "panelAngle", data['panelAngle'])
    redis_cli.hset('taskParams', "panelAngleX", data['panelAngleX'])
    redis_cli.hset('taskParams', "gap", data['gap'])
    redis_cli.hset('taskParams', "gapX", data.get('gapX', data['gap']))
    redis_cli.hset('taskParams', "gapY", data.get('gapY', data['gap']))
    # 原点航向角，用于起始点转正
    redis_cli.hset('taskParams', "originHeading", data['originHeading'])
    result = {"success": True, "msg": "保存成功"}

    return jsonify(result)


# 生成任务列表接口
@app.route("/vehicle/createTask", methods=['POST'])
def createTask():
    global taskList
    data = request.get_json()
    taskName = data['taskName']
    # 将任务名称放在一个set集合中
    redis_cli.sadd('taskNameSet', taskName)
    taskParams = redis_cli.hgetall("taskParams")
    startLat = float(taskParams.get('startLat'))
    startLon = float(taskParams.get('startLon'))
    point = {'startLat': startLat, 'startLon': startLon}
    redis_cli.hset('loc_start_lat_lon',taskName,json.dumps(point))
    areaList = data['areaList']
    result = service.createTask(taskName, areaList)
    # 需要将原本来的任务列表置空，让其重新加载任务列表文件
    taskList = []
    return jsonify(result)


# 获取当前任务信息
@app.route("/vehicle/selectTask", methods=['GET'])
def selectTask():
    global taskList
    if len(taskList) == 0:
        taskList = util.readConfig("config.json")
    result = {"success": True, "msg": "获取数据成功"}
    result['data'] = taskList
    return jsonify(result)


# 获取所有任务名称
@app.route("/vehicle/selectTaskName", methods=['GET'])
def selectTaskName():
    # 获取所有任务
    taskNames = redis_cli.smembers('taskNameSet')
    # 获取当前任务
    currentTaskName = redis_cli.get('currentTaskName')
    data = {'taskNames': list(taskNames), 'currentTaskName': currentTaskName}

    if len(taskNames) == 0:
        result = {"success": True, "msg": "数据为空"}
        return jsonify(result)
    else:
        result = {"success": True, "msg": "获取数据成功"}
        result['data'] = data
        return jsonify(result)


# 根据任务名称获取任务信息
@app.route("/vehicle/selectTaskByName", methods=['GET'])
def selectTaskByName():
    global taskList
    taskName = request.args.get('taskName')
    fileName = taskName + '.json'
    try:
        taskList = util.readConfig(fileName)
        result = {"success": True, "msg": "获取数据成功", 'data': taskList}
    except Exception as e:
        logger.error(u"文件不存在：{}".format(taskName))
        result = {"success": False, "msg": u"文件不存在:{}".format(fileName)}
    return jsonify(result)


# 保存当前任务
@app.route("/vehicle/saveCurrentTaskName", methods=['GET'])
def saveCurrentTaskName():
    global taskList
    taskName = request.args.get('taskName')
    fileName = taskName + '.json'
    # 将文件内容复制到执行任务的文件中
    with open(fileName, 'r') as src, open('config.json', 'w') as dst:
        for line in src:
            dst.write(line)
    redis_cli.set('currentTaskName', taskName)
    # 删除redis中taskList任务
    redis_cli.delete('taskList')
    syncCurTaskFileToRedis()
    # 把当前任务列表置空
    taskList = []
    result = {"success": True, "msg": "保存数据成功"}

    return jsonify(result)

# 将当前任务中的参数信息同步到redis中
def syncCurTaskFileToRedis():
    taskObj = util.readConfig("config.json")
    # 将文件里的参数设置到redis缓存中，taskParams
    redis_cli.hset('taskParams', "goBackLen", taskObj['goBackLen'])
    redis_cli.hset('taskParams', "goLeftOrRightBackLen", taskObj['goLeftOrRightBackLen'])
    redis_cli.hset('taskParams', "turnBackLen", taskObj['turnBackLen'])
    redis_cli.hset('taskParams', "panelWidth", taskObj['panelWidth'])
    redis_cli.hset('taskParams', "panelHeight", taskObj['panelHeight'])
    # redis_cli.hset('taskParams', "upOrDownBridgeLen", data['upOrDownBridgeLen'])
    redis_cli.hset('taskParams', "leftOrRightBridgeLen", taskObj['leftOrRightBridgeLen'])
    redis_cli.hset('taskParams', "voltageWarn", taskObj['voltageWarn'])
    # 初始航向角
    redis_cli.hset('taskParams', "heading", taskObj['heading'])
    # 起始点经纬度
    redis_cli.hset('taskParams', "startLat", taskObj['startLat'])
    redis_cli.hset('taskParams', "startLon", taskObj['startLon'])
    # 充电桩经纬度
    redis_cli.hset('taskParams', "chargingPileLat", taskObj['chargingPileLat'])
    redis_cli.hset('taskParams', "chargingPileLon", taskObj['chargingPileLon'])
    # 起始点到充电桩位置
    redis_cli.hset('taskParams', "startToChargingPilePointLength", taskObj['startToChargingPilePointLength'])

    redis_cli.hset('taskParams', "panelAngle", taskObj['panelAngle'])
    redis_cli.hset('taskParams', "panelAngleX", taskObj['panelAngleX'])
    redis_cli.hset('taskParams', "gap", taskObj['gap'])
    redis_cli.hset('taskParams', "gapX", taskObj.get('gapX', taskObj['gap']))
    redis_cli.hset('taskParams', "gapY", taskObj.get('gapY', taskObj['gap']))
    # 原点航向角，用于起始点转正
    redis_cli.hset('taskParams', "originHeading", taskObj['originHeading'])

# 获取电池电量
@app.route("/vehicle/getVoltage", methods=['GET'])
def getVoltage():
    voltage = redis_cli.get('voltage')
    result = {"success": True, "msg": "保存数据成功", "data": voltage}
    return jsonify(result)


# 返回到固定点
@app.route("/vehicle/returnToPoint", methods=['GET'])
def returnToPoint():
    # if redis_cli.get("doCleanThreadStop") == '0':
    #     return make_response("请先点击急停,然后再点击返回原点")
    thread = threading.Thread(target=returnToPointThread)
    logger.warn("启动返回固定点线程")
    thread.start()
    response = make_response("1")
    return response


def returnToPointThread():
    try:
        logger.warn("启动返回固定点")
        set_current_action('return_to_point')
        redis_cli.set('curTaskIndex', 0)
        correctThread = threading.Thread(target=correctByRTKTest)
        correctThread.start()
        # 判断当前到那个任务了
        json_item = redis_cli.lindex('taskList', 0)
        # 下一个任务
        json_next_item = redis_cli.lindex('taskList', 1)

        redis_cli.set("mission", "working")
        redis_cli.set("correct", "true")
        redis_cli.set('action', 'true')
        set_current_action('return_to_point')
        # 使其每一次接口调用，都重新开始
        redis_cli.set('parking', 0)
        reset_odometer(ser)

        if json_item:
            item = json.loads(json_item)
            next_item = json.loads(json_next_item)
            goByBackRoute(item, next_item)
        # 关闭工作模式
        redis_cli.set("mission", "complete")
        # 开启纠偏
        redis_cli.set("correct", "false")
        set_current_action('idle')
        redis_cli.set('curTaskIndex', 0)
        # 初始化
        redis_cli.set("doCleanThreadStop", 0)
        logger.warn("返回固定点结束")
        # 停止一切
        doParking()
    except Exception as e:
        doParking()
        redis_cli.set("doCleanThreadStop", 0)
        logger.error(traceback.format_exc())


# 根据当前任务和下一个任务判断小车的位置
def goByBackRoute(curItem, nextItem):
    curArea = int(curItem['areaNumber'])
    curAngle = int(curItem['angle'])
    nextAngle = int(nextItem['angle'])
    if curArea == global_area:
        if curAngle != 0:
            if curAngle == 180 and nextAngle == 90:
                pass
            else:
                turn(ser, 10 * 270)
                goUp(10)
                turn(ser, 10 * 180)
        # 关闭视觉纠偏
        redis_cli.set("correct", "false")
        # 获取当前经纬度，获取固定点经纬度，计算航向角和距离，然后根据RTK走到固定点
        dis, heading = util.get_distance_angle(global_cur_rtk_lat, global_cur_rtk_lon, global_point_lat,
                                               global_point_lon)
        justMoveByRTK(dis, heading)
    elif curArea < global_area:
        if curAngle != 0:
            if curAngle == 180 and nextAngle == 90:
                pass
            else:
                turn(ser, 10 * 270)
                goUp(10)
                turn(ser, 10 * 180)
        else:
            turn(ser, 10 * 180)
        goUp(10)
        turn(ser, 10 * 90)
        # 关闭视觉纠偏
        redis_cli.set("correct", "false")
        # 获取当前经纬度，获取固定点经纬度，计算航向角和距离，然后根据RTK走到固定点
        dis, heading = util.get_distance_angle(global_cur_rtk_lat, global_cur_rtk_lon, global_point_lat,
                                               global_point_lon)
        justMoveByRTK(dis, heading)
    else:
        if curAngle != 0:
            if curAngle == 180 and nextAngle == 90:
                pass
            else:
                turn(ser, 10 * 270)
                goUp(10)
                turn(ser, 10 * 180)
        else:
            turn(ser, 10 * 180)
        goUp(10)
        turn(ser, 10 * 270)
        # 关闭视觉纠偏
        redis_cli.set("correct", "false")
        # 获取当前经纬度，获取固定点经纬度，计算航向角和距离，然后根据RTK走到固定点
        dis, heading = util.get_distance_angle(global_cur_rtk_lat, global_cur_rtk_lon, global_point_lat,
                                               global_point_lon)
        justMoveByRTK(dis, heading)

    turn(ser, 10 * 180)

def moveByRTK(endLat, endLon,heading=0):
    global global_cur_taskPoint
    global global_go,global_interval
    if redis_cli.get("mission") == "complete":
        return
    # 获取当前任务开始点和结束点的经纬度
    # dis,heading = util.get_distance_angle(global_cur_rtk_lat,global_cur_rtk_lon,endLat,endLon)
    global_cur_taskPoint = {"heading":heading,"startLat": global_cur_rtk_lat, "startLon": global_cur_rtk_lon,
                            "endLat": endLat,"endLon": endLon}
    # 开启RTK纠偏
    global_go = 1
    goCommand(100)
    # 开启RTK纠偏
    global_go = 1
    global_interval = 0
    # 如果global_go==0，则说明直行结束
    startTime = time.time()
    # 如果global_go = 1，则说明直行未结束
    while global_go == 1:
        # 表示到边了
        if getEdge() == 0:
            break
        # 如果时间过长也需要停止
        if global_interval > 30:
            break
        endTime = time.time()
        # 获取时间间隔
        global_interval = endTime - startTime
        logger.warning(global_interval)
        time.sleep(0.1)
    sendBraking()
    global_go = 0
# 通过RTK实现从一个点直线移动到另一个点，返回结果0：表示该任务未执行完成，1：表示执行完成
def pointToPointByRTK(startLat, startLon, endLat, endLon,heading,speed=200):
    global global_cur_taskPoint,global_interval
    global global_go

    # 返回结果，0：不成功，1：任务执行成功
    result = 1
    global_interval = 0
    global_cur_taskPoint = {"heading": heading, "startLat": startLat, "startLon": startLon,"endLat": endLat, "endLon": endLon}
    # 开启RTK纠偏
    global_go = 1
    # 设置滚刷
    brush_speed = int(redis_cli.get("brushSpeed"))
    setBrushSpeed(brush_speed)
    goCommand(speed)
    startTime = time.time()
    # 如果global_go = 1，则说明直行未结束
    while global_go == 1:
        if redis_cli.get('parking') == '1':
            result = 0
            break
        # 表示到边了
        if getEdge() == '0':
            global_go = 0
            break
        endTime = time.time()
        # 获取时间间隔
        global_interval = endTime-startTime
        time.sleep(0.1)
    return result

# 实时获取当前位置，计算当前位置到充电的距离
def observer_to_chargingPile(data):
    lat = data.lat
    lon = data.lon

def observer_rtk_data(data):
    """观察者用于获取rtk数据，将航向角发送给下位机，将经纬度发送给服务端"""
    lat = data.lat
    lon = data.lon
    if data.heading is None:
        if redis_cli.get('openLog') == '1':
            logger.warning("RTK宸叉洿鏂扮粡绾害锛屼絾褰撳墠鏃犳湁鏁堣埅鍚戣锛岃烦杩囪埅鍚戝悓姝? lat={}, lon={}".format(lat, lon))
        return
    heading = float(data.heading)
    # preBuildCommand()
    setHeadingToVehicle(heading)
    duplicateWriteCmd(ser, command)
    time.sleep(0.01)

# zSpeed_pid = PID(Kp=2,Ki=0.5,Kd=1,setpoint=0)
# 4. 参数整定 (PID 参数)
    # 如果车左右震荡：减小 K_P_CTE，增大 K_D_CTE
    # 如果车反应迟钝/越走越偏：增大 K_P_CTE，检查符号
K_HEAD = 3.0
K_CTE_P = 50.0   # 原 450 太大，建议从 50-100 开始试
K_CTE_D = 150.0  # 微分项，抑制震荡

def observer_go_correct(data):
    """观察者用于直行纠偏"""
    global global_last_distance_to_target,global_last_cte
    if redis_cli.get('openLog') == '1':
        logger.info(data)
    global global_cur_rtk_lat, global_cur_rtk_lon, global_cur_rtk_heading,global_go
    global_cur_rtk_lat = data.lat
    global_cur_rtk_lon = data.lon
    if data.heading is not None:
        global_cur_rtk_heading = data.heading
    sync_current_location(data.lat, data.lon, data.heading)
    # global_go=1表示直行启动
    if global_go == 1:
        if data.heading is None:
            logger.warning("RTK缁忕含搴﹀凡鏇存柊锛屼絾褰撳墠鏃犳湁鏁堣埅鍚戣锛屾殏涓嶆墽琛岀洿琛岀籂鍋?...")
            return
        start_lat = global_cur_taskPoint['startLat']
        start_lon = global_cur_taskPoint['startLon']
        target_lat = global_cur_taskPoint['endLat']
        target_lon = global_cur_taskPoint['endLon']
        target_heading = float(global_cur_taskPoint['heading'])

        # 计算期望航向角（从当前位置指向目标点）
        distance_to_target, target_heading_cur = util.get_distance_angle(data.lat, data.lon, target_lat, target_lon)
        heading_error = float(target_heading) - float(data.heading)
        # 计算最短偏差
        heading_error = (heading_error + 180) % 360 - 180
        if distance_to_target < 2:
            heading_error = max(-5,min(heading_error,5))
        # 只有直行，才发送纠偏指令
        cte = util.cross_track_error(start_lat, start_lon, target_lat, target_lon, data.lat, data.lon)

        # cte_dot = (cte - global_last_cte) / 0.01
        # raw_output = (heading_error * K_HEAD) - (cte * K_CTE_P) - (cte_dot * K_CTE_D)
        # raw_output = - (cte * K_CTE_P) - (cte_dot * K_CTE_D)

        # stree_output = int(raw_output)
        # stree_output = heading_error*10 - int(450 * cte)
        stree_output = heading_error*10 - int(800 * cte)
        # if abs(heading_error) > 1 or abs(cte) > 0.02:
            # logger.warning("视觉纠偏关闭，RTK纠偏开启")
            # redis_cli.set("correct", "false")
        # 发送电机控制指令
        # 正数左轮快，向右偏，负数右轮快，向左偏
        setZSpeed(-stree_output)
        duplicateWriteCmd(ser, command)

        # 打印状态
        logger.info("航向角:{:.2f} | 当前航向角:{:.2f} | heading_error:{:.2f}横向偏差:{:.2f}上次横向偏差:{:.2f}距离目标:{:.2f}m | 转向输出: {:.2f}"
                    .format(target_heading, data.heading, heading_error, cte, global_last_cte,distance_to_target,
                            stree_output))
        global_last_cte = cte
        # else:
        #     redis_cli.set("correct", "true")
        #     pass
        result = distance_to_target - global_last_distance_to_target
        # 更新路径进度,如果上一次距离目标距离小于当前距离目标距离，说明已经到达目标位置
        # 每个直行任务，必须从开始时间，3秒之后才可以判断当前目标距离和上一个目标距离的大小
        checker.add_number(round(result,2))
        if global_interval > 2 and checker.are_all_positive() or distance_to_target <= 0.05:
            global_go = 0
            redis_cli.set("correct", "false")
            logger.warn("路径直行结束")
        if distance_to_target <= 2 and redis_cli.get('lastTask') == '1':
            sendCommandSetXSpeed(200)
        global_last_distance_to_target = distance_to_target
        # 防止cup资源占满
        time.sleep(0.01)


def goOnDoClean():
    redis_cli.set('parking', 0)
    try:
        # 未完成的任务列表
        previousTaskList = [json.loads(item) for item in redis_cli.lrange('taskList', 0, -1)]

        if doClean(previousTaskList, True) == 0:
            logger.warn("清扫工作未完成")
        else:
            logger.warn("清扫工作完成")
        sendBraking()
        logger.warn("任务执行结束")
        redis_cli.set("mission", "complete")
        redis_cli.set("correct", "false")
        redis_cli.set('action', 'false')
        # 表示自动清扫线程停止
        redis_cli.set("doCleanThreadStop", 1)
    except Exception as e:
        logger.error(e.message)

def goOnDoCleanByRTK():
    global global_go,global_doCleanThreadStop
    redis_cli.set("mission", "working")
    redis_cli.set("parking", "0")
    # 未完成的任务列表
    previousTaskList = [json.loads(item) for item in redis_cli.lrange('taskList', 0, -1)]
    # 执行任务
    for index, task in enumerate(previousTaskList):
        logger.warn("执行任务{}".format(task['id']))
        turn_back_len = task['turn_back_len']
        back_len = task['back_len']
        angle = task['angle']
        startLat, startLon = task['startLat'], task['startLon']
        heading = task['heading']
        endLat = task['endLat']
        endLon = task['endLon']
        mode = task['mode']

        if index != 0:
            turn(ser, angle * 10)
            if redis_cli.get('parking') == '1':
                global_doCleanThreadStop = 1
                break
            if mode == 1:
                # moveBack(ser, turn_back_len)
                if redis_cli.get('parking') == '1':
                    global_doCleanThreadStop = 1
                    break
        speed = 350
        if angle == 180:
            speed = 200
        result = pointToPointByRTK(startLat, startLon, endLat, endLon, heading, speed)
        if result == 0:
            global_go = 0
            # 表示自动清扫线程停止
            global_doCleanThreadStop = 1
            break
        else:
            if mode == 1:
                moveBack(ser, back_len)
                if redis_cli.get('parking') == '1':
                    global_doCleanThreadStop = 1
                    break
        if redis_cli.get('parking') == '0':
            logger.warn("删除任务{}".format(task['id']))
            redis_cli.lpop("taskList")
    logger.warn("继续清扫结束")
    doParking()

# 继续清扫
@app.route("/vehicle/goOn", methods=['GET'])
def goOn():
    logger.warn('继续清扫任务')
    redis_cli.set("reverse", "false")
    # thread = threading.Thread(target=goOnDoClean)
    # thread.start()
    thread = threading.Thread(target=goOnDoCleanByRTK)
    thread.start()
    response = make_response("1")
    return response


# 获取角度
@app.route("/vehicle/getAngle", methods=['GET'])
def getAngle():
    # 开启纠偏
    redis_cli.set("correct", "true")
    # 让其旋转
    turn_left()
    angle = redis_cli.get("angle")
    response = make_response("angle={}".format(angle))
    return response


@app.route("/vehicle/moveDistance/<int:length>/<int:speed>", methods=['GET'])
def moveDistance(length, speed):
    logger.warn("移动距离={}cm;速度={}".format(length, speed))
    reset_odometer(ser)
    goByLength(ser, length, speed)
    response = make_response("1")
    return response


@app.route("/vehicle/moveBackDistance/<int:length>/<int:speed>", methods=['GET'])
def moveBackDistance(length, speed):
    logger.warn("倒退移动距离={}cm;速度={}".format(length, speed))
    reset_odometer(ser)
    goBackByLength(ser, length, -speed)
    response = make_response("1")
    return response


# 向左转向90
@app.route("/vehicle/turnLeft90", methods=['GET'])
def turnLeft90():
    turnByAngle(270)
    response = make_response("1")
    return response


# 向右转向90
@app.route("/vehicle/turnRight90", methods=['GET'])
def turnRight90():
    turnByAngle(90)
    response = make_response("1")
    return response


# 向右转向180
@app.route("/vehicle/turnRight180", methods=['GET'])
def turnRight180():
    turnByAngle(180)
    response = make_response("1")
    return response


# 通过转向，找寻向上位置
@app.route("/vehicle/turnCheckPosition", methods=['GET'])
def turnCheckPosition():
    turnCheckPoint()
    response = make_response("1")
    return response


def turnByAngle(angle):
    redis_cli.set("reverse", "false")
    sendBraking()
    preBuildCommand()
    redis_cli.set("correct", "true")
    redis_cli.set('action', 'true')

    command[1] = 0x03
    command[2] = 0x01
    command[4] = 0x00
    command[5] = 0x00
    command[8] = 0x00
    command[9] = 0x00
    command[10] = 0x00

    setRotateTo(angle * 10)
    duplicateWriteCmd(ser, command)


@app.errorhandler(Exception)  # 捕获所有未处理异常
def handle_global_exception(e):
    error_msg = {
        "error_type": type(e).__name__,
        "message": str(e),
        "traceback": traceback.format_exc()  # 记录堆栈
    }
    logger.error(error_msg)  # 输出到控制台
    return jsonify({"error": error_msg}), 500


def preBuildCommand():
    command[0] = 123
    command[3] = 0
    command[6] = 0
    command[7] = 0
    command[11] = 0
    command[12] = 0
    command[13] = 0
    command[14] = 0
    command[15] = 0
    command[16] = 0
    command[17] = 0
    command[18] = 125


def calculate_angle(x0, y0, x1, y1):
    # 固定顺序，确保 y1 > y0
    if y0 > y1:
        x0, y0, x1, y1 = x1, y1, x0, y0

    dx = x1 - x0
    dy = y1 - y0

    if abs(dy) < 1e-5:  # 避免除零
        return 90.0 if dx > 0 else -90.0

    angle_rad = math.atan2(dx, dy)  # dx 放前
    angle_deg = math.degrees(angle_rad)

    return angle_deg


def is_vertical_line(x0, y0, x1, y1, max_angle=MAX_ANGLE):
    """检查线段是否为竖向（与垂直方向的夹角小于max_angle度）"""
    # 计算线段方向向量
    dx = x1 - x0
    dy = y1 - y0

    # 避免除以零
    if abs(dx) < 1e-5:
        return True

    # 计算与垂直方向的夹角
    # 计算角度（弧度）
    angle_rad = math.atan2(abs(dx), abs(dy))
    # 转换为角度
    angle_deg = math.degrees(angle_rad)

    # 检查是否在允许的角度范围内
    return angle_deg <= max_angle


def check_angle_consensus(angle):
    """检查角度是否达成共识"""
    global angle_samples, final_angle

    # 四舍五入取整数
    rounded_angle = round(angle)

    # 添加到样本列表
    angle_samples.append(rounded_angle)

    # 如果样本数量足够
    if len(angle_samples) >= 10:
        # 检查是否有足够数量的相同角度
        from collections import Counter
        angle_count = Counter(angle_samples)
        most_common = angle_count.most_common(1)

        if most_common[0][1] >= CONSENSUS_THRESHOLD:
            final_angle = most_common[0][0]
            return True

        # 移除最早的样本，保持样本数量为SAMPLE_SIZE
        angle_samples.pop(0)
    return False


def getAngleByVision():
    global cap  # 申明使用全局变量
    center_x = 0
    center_y = 0

    while True:
        ret, image = cap.read()
        if not ret:
            logger.error('无法读取视频流或文件结束')
            stopThenStart()
            continue
        else:
            # 获取视频属性
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            center_x = width // 2
            center_y = height // 2
            break
    cap.release()
    time.sleep(1)
    stopThenStart()

    # 处理每一帧
    while True:
        ret, frame = cap.read()
        if not ret:
            # 如果视频结束，重置到开始
            logger.error('无法读取视频流')
            stopThenStart()
            continue

        # 灰度处理
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 高斯模糊减少噪声
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # 边缘检测
        edges = cv2.Canny(blurred, CANNY_THRESHOLD1, CANNY_THRESHOLD2, apertureSize=3)

        # 使用霍夫变换检测直线
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=math.pi / 180,
            threshold=50,
            minLineLength=MIN_LINE_LENGTH,  # 最小线段长度
            maxLineGap=10  # 最大线段间隙
        )

        min_offset = float('inf')  # 初始化为无穷大
        closest_line = None  # 存储最接近中心的线段
        vertical_lines = []  # 存储所有竖直线

        if lines is not None:
            for line in lines:
                x0, y0, x1, y1 = line[0]
                # 只保留竖向的线（与垂直方向夹角在±max_angle度以内）
                if is_vertical_line(x0, y0, x1, y1, MAX_ANGLE):
                    # 计算线段角度
                    angle = calculate_angle(x0, y0, x1, y1)
                    vertical_lines.append(line[0])
                    # 计算线段长度
                    length = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
                    # 只考虑长度大于min_length的线段
                    if length > MIN_LINE_LENGTH:
                        # 计算中点
                        xmid = (x0 + x1) / 2
                        offset = xmid - center_x
                        # 更新最小偏移量
                        if abs(offset) < abs(min_offset):
                            min_offset = offset
                            closest_line = (x0, y0, x1, y1)
                            closest_angle = angle  # 保存最近线段的夹角

        # 绘制最接近中心的竖直线（蓝色）
        if closest_line is not None:
            # 检查角度共识
            if check_angle_consensus(closest_angle):
                break
    return final_angle


# 方向校正
# 0:未完成，1：完成
def turnCheckPoint(originHeading):
    # 开启校正
    try:
        reset_odometer(ser)
        redis_cli.set("mission", "working")
        # 获取原点航向角
        # heading = int(redis_cli.hget('taskParams', 'originHeading'))
        logger.warn("获取原点航向角：{}".format(originHeading))
        turn(ser, originHeading * 10)
        if getRotateArrive() == '1':
            return 1
        else:
            return 0
    except Exception as e:
        logger.error(e)


# 前进
@app.route("/vehicle/drive", methods=['GET'])
def driving():
    redis_cli.set("correct", "false")
    redis_cli.set("reverse", "false")
    forward_speed = int(redis_cli.get("forwardSpeed"))
    redis_cli.set('action', 'true')

    preBuildCommand()
    command[0] = 0x7B
    command[17] = 0x7D
    command[1] = 1
    command[2] = 1
    setXSpeed(forward_speed)
    command[8] = 0
    command[9] = 0
    command[10] = 0
    setHWstatus(0, 0, 0, 0, 0)
    command[17] = tem_listener(command, 17)
    duplicateWriteCmd(ser, command)
    response = make_response("1")
    return response

# 前进
def drive():
    redis_cli.set("reverse", "false")
    forward_speed = int(redis_cli.get("forwardSpeed"))
    redis_cli.set('action', 'true')

    preBuildCommand()
    command[0] = 0x7B
    command[17] = 0x7D
    command[1] = 1
    command[2] = 1
    setXSpeed(forward_speed)
    command[8] = 0
    command[9] = 0
    command[10] = 0
    setHWstatus(0, 0, 0, 0, 0)
    command[17] = tem_listener(command, 17)
    duplicateWriteCmd(ser, command)


# 后退
@app.route("/vehicle/back", methods=['GET'])
def reverse():
    redis_cli.set("correct", "false")
    redis_cli.set('reverse', 'true')
    redis_cli.set('action', 'true')
    # 使其失能
    reSetStatus(ser)
    # 初始化
    preBuildCommand()
    command[1] = 0x01
    command[2] = 0x01
    setXSpeed(-100)
    command[8] = 0x00
    command[9] = 0x00
    command[10] = 0x00

    setPowerOn(1)
    setHWstatus(0, 0, 0, 0, 0)
    setXSpeed(-100)
    setDistance(1000)
    command[17] = tem_listener(command, 17)

    duplicateWriteCmd(ser, command)
    response = make_response("1")

    return response


def sendBraking():
    preBuildCommand()
    logger.warn("停止命令")
    command[1] = 0x00
    command[2] = 0x01
    command[4] = 0x00
    command[5] = 0x00
    command[7] = 0x00
    command[8] = 0x00
    command[8] = 0x00
    command[9] = 0x00
    command[10] = 0x00
    duplicateWriteCmd(ser, command)
    time.sleep(0.2)


@app.route("/vehicle/turnLeft", methods=['GET'])
def turn_left():
    redis_cli.set("reverse", "false")
    sendBraking()
    preBuildCommand()

    redis_cli.set("correct", "false")
    redis_cli.set('action', 'true')

    command[1] = 0x03
    command[2] = 0x01
    command[4] = 0x00
    command[5] = 0x00
    command[8] = 0x00
    command[9] = 0x00
    command[10] = 0x00

    setRotateTo(666 * 10)
    duplicateWriteCmd(ser, command)
    response = make_response("1")

    return response


@app.route("/vehicle/turnRight", methods=['GET'])
def turn_right():
    redis_cli.set("reverse", "false")
    sendBraking()
    preBuildCommand()
    redis_cli.set("correct", "false")
    redis_cli.set('action', 'true')

    command[1] = 0x03
    command[2] = 0x01
    command[4] = 0x00
    command[5] = 0x00
    command[8] = 0x00
    command[9] = 0x00
    command[10] = 0x00
    setRotateTo(555 * 10)

    print(command[13])
    print(command[14])

    duplicateWriteCmd(ser, command)

    response = make_response("1")

    return response


def sendDrivingNorth(forwardSpeed):
    preBuildCommand()

    command[1] = 0x04

    command[2] = 0x01

    byteArr = getCmdBytes(forwardSpeed)

    command[4] = byteArr[0]

    command[5] = byteArr[1]

    command[8] = 0x00

    command[9] = 0x00

    # command[10] = 0x00

    command[11] = 0x00

    command[12] = 0x00

    writeCmd(ser, command)


def sendDrivingSouth(forwardSpeed):
    preBuildCommand()

    command[1] = 0x04

    command[2] = 0x01

    byteArr = getCmdBytes(-forwardSpeed)

    command[4] = byteArr[0]

    command[5] = byteArr[1]

    command[8] = 0x00

    command[9] = 0x00

    # command[10] = 0x00

    command[11] = 0x00

    command[12] = 0x00

    setPowerOn(1)

    setHWstatus(0, 0, 0, 0, 0)

    #setXSpeed(-100)

    setDistance(1000)

    command[17] = tem_listener(command, 17)

    writeCmd(ser, command)


def getCmdBytes(param):
    tmp = bytearray(2)

    param = param & 0xFFFF

    tmp[0] = param >> 8 & 0xFF

    tmp[1] = param & 0xFF

    return tmp


def sendDrivingWest(forwardSpeed, offset):
    preBuildCommand()

    command[1] = 0x04

    command[2] = 0x01

    # byteArr = getCmdBytes(forwardSpeed)

    command[4] = 0x00

    command[5] = 0x00

    command[8] = 0x00

    command[9] = 0x00

    # command[10] = 0x00

    byteArr = getCmdBytes(offset)

    command[11] = byteArr[0]

    command[12] = byteArr[1]

    writeCmd(ser, command)


def sendDrivingEast(forwardSpeed, offset):
    preBuildCommand()

    command[1] = 0x04

    command[2] = 0x01

    # byteArr = getCmdBytes(forwardSpeed)

    command[4] = 0x00

    command[5] = 0x00

    command[8] = 0x00

    command[9] = 0x00

    # command[10] = 0x00

    byteArr = getCmdBytes(offset)

    command[11] = byteArr[0]

    command[12] = byteArr[1]

    writeCmd(ser, command)


def sendDrivingNorthWest(forwardSpeed, offset):
    preBuildCommand()

    command[1] = 0x04

    command[2] = 0x01

    byteArr = getCmdBytes(forwardSpeed)

    command[4] = byteArr[0]

    command[5] = byteArr[1]

    command[8] = 0x00

    command[9] = 0x00

    # command[10] = 0x00

    byteArr = getCmdBytes(offset)

    command[11] = byteArr[0]

    command[12] = byteArr[1]

    writeCmd(ser, command)


def sendDrivingNorthEast(forwardSpeed, offset):
    preBuildCommand()

    command[1] = 0x04

    command[2] = 0x01

    byteArr = getCmdBytes(forwardSpeed)

    command[4] = byteArr[0]

    command[5] = byteArr[1]

    command[8] = 0x00

    command[9] = 0x00

    # command[10] = 0x00

    byteArr = getCmdBytes(-offset)

    command[11] = byteArr[0]

    command[12] = byteArr[1]

    writeCmd(ser, command)


def sendDrivingSouthWest(forwardSpeed, offset):
    preBuildCommand()

    command[1] = 0x04

    command[2] = 0x01

    byteArr = getCmdBytes(-forwardSpeed)

    print(-forwardSpeed)

    command[4] = byteArr[0]

    command[5] = byteArr[1]

    command[8] = 0x00

    command[9] = 0x00

    # command[10] = 0x00

    byteArr = getCmdBytes(-offset)

    command[11] = byteArr[0]

    command[12] = byteArr[1]

    writeCmd(ser, command)


def sendDrivingSouthEast(forwardSpeed, offset):
    preBuildCommand()

    command[1] = 0x04

    command[2] = 0x01

    byteArr = getCmdBytes(-forwardSpeed)

    print(-forwardSpeed)

    command[4] = byteArr[0]

    command[5] = byteArr[1]

    command[8] = 0x00

    command[9] = 0x00

    # command[10] = 0x00

    byteArr = getCmdBytes(offset)

    command[11] = byteArr[0]

    command[12] = byteArr[1]

    writeCmd(ser, command)


@app.route("/vehicle/joystickMove/<string:distance>/<string:dirX>/<string:dirY>", methods=['GET'])
def joystick_move(distance, dirX, dirY):
    # redis_cli.set("reverse", "false")
    # redis_cli.set("correct", "false")
    intDistance = float(distance)
    floatX = float(dirX)
    floatY = float(dirY)
    if (abs(floatX) <= 0.2 and abs(floatY) <= 0.2):
        parking()
        response = make_response("1")
        return response

    forwardSpeed = int(intDistance / 50 * 250)
    brushSpeed = int(redis_cli.get('brushSpeed'))
    setBrushSpeed(brushSpeed)

    if (abs(floatX) <= 0.2 and floatY > 0.2):

        # currentStatus = NORTH

        sendDrivingNorth(forwardSpeed)

    elif (abs(floatX) <= 0.2 and floatY < -0.2):

        # currentStatus = SOUTH

        sendDrivingSouth(forwardSpeed)

    elif (abs(floatY) <= 0.2 and floatX < -0.2):

        # currentStatus = WEST

        offset = 1000

        sendDrivingWest(forwardSpeed, offset)

    elif (abs(floatY) <= 0.2 and floatX > 0.2):

        # currentStatus = EAST

        offset = -1000

        sendDrivingEast(forwardSpeed, offset)

    elif (floatX < -0.2 and floatY > 0.2):

        # currentStatus = NORTH_WEST

        offset = int((4.5 - abs(floatY / floatX)) * 200)

        sendDrivingNorthWest(forwardSpeed, offset)

    elif (floatX > 0.2 and floatY > 0.2):

        # currentStatus = NORTH_EAST

        offset = int((4.5 - abs(floatY / floatX)) * 200)

        sendDrivingNorthEast(forwardSpeed, offset)

    elif (floatX < -0.2 and floatY < -0.2):

        # currentStatus = SOUTH_WEST

        offset = int((4.5 - abs(floatY / floatX)) * 200)

        sendDrivingSouthWest(forwardSpeed, offset)

    elif floatX > 0.2 and floatY < -0.2:

        # currentStatus = SOUTH_EAST

        offset = int((4.5 - abs(floatY / floatX)) * 200)
        sendDrivingSouthEast(forwardSpeed, offset)

    response = make_response("1")
    return response


def tem_listener(data, data_length):  # 数组 数组长度
    output = 0
    for num in range(0, data_length + 1):
        output = output ^ data[num]
    return output


def setStatus(status):
    command[1] = status


def setPowerOn(status):
    command[2] = status


def setHWstatus(use, sth, air, gate, duo):
    hex_number = ""

    if use == 1:

        hex_number = "f"

    elif use == 0:
        hex_number == "0"
    otherControl = str(sth) + str(air) + str(gate) + str(duo)

    hex_otherControl = hex(int(otherControl, 2))[2:]

    hex_number = hex_number + hex_otherControl

    command[3] = int(hex_number, 16)


def setXSpeed(status):  # 速度待定
    if status > 32767:
        status = 32767
    if status < -32768:
        status = -32768

    if int(status) < 0:

        binary_num = bin(int(status) & 0xffff)  # 将负数转换为二进制

        hex_num = hex(int(binary_num, 2))[2:]  # 将二进制转换为十六进制

        hex_1 = hex_num[0] + hex_num[1]

        command[4] = int(hex_1, 16)

        hex_2 = hex_num[2] + hex_num[3]

        command[5] = int(hex_2, 16)

    elif int(status) <= 255:

        hex_string = hex(int(status))[2:]

        hex_1 = "00"

        command[4] = int(hex_1, 16)

        hex_2 = hex_string

        command[5] = int(hex_2, 16)

    elif int(status) <= 4095:

        hex_string = hex(int(status))[2:]

        hex_1 = "0" + hex_string[0]

        command[4] = int(hex_1, 16)

        hex_2 = hex_string[1] + hex_string[2]

        command[5] = int(hex_2, 16)

    elif int(status) <= 65535:

        hex_string = hex(int(status))[2:]

        hex_1 = hex_string[0] + hex_string[1]

        command[4] = int(hex_1, 16)

        hex_2 = hex_string[2] + hex_string[3]

        command[5] = int(hex_2, 16)


def setZSpeed(status):
    if status > 32767:
        status = 32767

    if status < -32768:
        status = -32768

    if int(status) < 0:

        binary_num = bin(int(status) & 0xffff)  # 将负数转换为二进制

        hex_num = hex(int(binary_num, 2))[2:]  # 将二进制转换为十六进制

        hex_1 = hex_num[0] + hex_num[1]

        command[6] = int(hex_1, 16)

        hex_2 = hex_num[2] + hex_num[3]

        command[7] = int(hex_2, 16)

    elif int(status) <= 255:

        hex_string = hex(int(status))[2:]

        hex_1 = "00"

        command[6] = int(hex_1, 16)

        hex_2 = hex_string

        command[7] = int(hex_2, 16)

    elif int(status) <= 4095:

        hex_string = hex(int(status))[2:]

        hex_1 = "0" + hex_string[0]

        command[6] = int(hex_1, 16)

        hex_2 = hex_string[1] + hex_string[2]

        command[7] = int(hex_2, 16)

    elif int(status) <= 65535:

        hex_string = hex(int(status))[2:]

        hex_1 = hex_string[0] + hex_string[1]

        command[6] = int(hex_1, 16)

        hex_2 = hex_string[2] + hex_string[3]

        command[7] = int(hex_2, 16)


def setDistance(status):
    if status > 50000:  # 设置范围为0-50000cm

        hex_string = "FF"

        command[8] = int(hex_string, 16)

        command[9] = int(hex_string, 16)

    elif int(status) <= 255:

        hex_string = hex(int(status))[2:]

        hex_1 = "00"

        command[8] = int(hex_1, 16)

        hex_2 = hex_string

        command[9] = int(hex_2, 16)

    elif int(status) <= 4095:

        hex_string = hex(int(status))[2:]

        hex_1 = "0" + hex_string[0]

        command[8] = int(hex_1, 16)

        hex_2 = hex_string[1] + hex_string[2]

        command[9] = int(hex_2, 16)

    elif int(status) <= 32767:

        hex_string = hex(int(status))[2:]

        hex_1 = hex_string[0] + hex_string[1]

        command[8] = int(hex_1, 16)

        hex_2 = hex_string[2] + hex_string[3]

        command[9] = int(hex_2, 16)


def setBrushSpeed(status):
    if status > 100:
        status = 100
    if status < -100:
        status = -100
    if int(status) < 0:
        binary_num = bin(status & 0xff)  # 将负数转换为二进制
        hex_num = hex(int(binary_num, 2))[2:]  # 将二进制转换为十六进制
        command[10] = int(hex_num, 16)
    elif int(status) >= 0:
        hex_string = hex(int(status))[2:]
        command[10] = int(hex_string, 16)

def setHeadingToVehicle(heading):
    normalized_heading = float(heading) % 360.0
    heading_centidegrees = int(round(normalized_heading * 100.0)) % 36000
    byteArr = getCmdBytes(heading_centidegrees)
    command[15] = byteArr[0]
    command[16] = byteArr[1]


def setRotate(status):
    if int(status) <= 255:

        hex_string = hex(int(status))[2:]

        hex_1 = "00"

        command[11] = int(hex_1, 16)

        hex_2 = hex_string

        command[12] = int(hex_2, 16)

    elif int(status) <= 4095:

        hex_string = hex(int(status))[2:]

        hex_1 = "0" + hex_string[0]

        command[11] = int(hex_1, 16)

        hex_2 = hex_string[1] + hex_string[2]

        command[12] = int(hex_2, 16)

    elif int(status) <= 32767:

        hex_string = hex(int(status))[2:]

        hex_1 = hex_string[0] + hex_string[1]

        command[8] = int(hex_1, 16)

        hex_2 = hex_string[2] + hex_string[3]

        command[9] = int(hex_2, 16)


def setRotateTo(status):
    status = int(round(float(status))) & 0xffff
    command[13] = status >> 8 & 0xff
    command[14] = status & 0xff


def reSetStatus(ser):
    setStatus(0)
    command[17] = tem_listener(command, 17)
    logger.warn(' '.join(format(x, '02x') for x in command))
    duplicateWriteCmd(ser, command)
    time.sleep(0.5)


def stopThenStart():
    global cap
    global global_status
    if cap is not None:
        for _ in range(20):
            try:
                cap.release()
                time.sleep(0.1)
                if cap is None:
                    print("成功退出视频")
                    global_status = "成功退出视频"
                    break
            except Exception as e:
                print("释放失败，重试")
                global_status = "释放失败，重试"
    activeIndex = 0
    for i in range(3):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            logger.warn("Camera index {} is available".format(i))
            activeIndex = i
            cap.release()
            break
    cap = cv2.VideoCapture(activeIndex)


def justMove(ser, arriveable=False):
    if redis_cli.get("mission") == "complete":
        return
    global global_status
    forward_speed = int(redis_cli.get("forwardSpeed"))
    reSetStatus(ser)
    brush_speed = int(redis_cli.get("brushSpeed"))
    setBrushSpeed(brush_speed)
    setStatus(1)
    setPowerOn(1)
    setHWstatus(0, 0, 0, 0, 0)
    setXSpeed(forward_speed)
    command[17] = tem_listener(command, 17)
    logger.warn(' '.join(format(x, '02x') for x in command))
    duplicateWriteCmd(ser, command)
    # 这里开始判断路径和到边,1能走，0不行
    while (getEdge() == "1" and redis_cli.get("mission") == "working") or (
            arriveable and redis_cli.get("odometer_arrive") == "true"):
        print("keep walking")
        global_status = "keep walking"
        if global_go == 0:
            break
        time.sleep(0.1)
    # time.sleep(0.5)


def moveBack(ser, distance=33):
    if redis_cli.get("mission") == "complete":
        return
    logger.warn('向后退了{}cm'.format(distance))
    global global_status
    reSetStatus(ser)
    setStatus(2)
    setPowerOn(1)
    setHWstatus(0, 0, 0, 0, 0)
    setXSpeed(-100)
    setDistance(distance)
    command[17] = tem_listener(command, 17)
    logger.warn(' '.join(format(x, '02x') for x in command))
    duplicateWriteCmd(ser, command)

    while getDistanceArrive() == "0" and redis_cli.get("mission") == "working":
        print("wait Distance finish")
        # global_status = "wait Distance finish"
    # 退到传感器给1了
    # while True:
    #     if getEdge() == "1":
    #         reSetStatus(ser)
    #         setStatus(2)
    #         setPowerOn(1)
    #         setHWstatus(0, 0, 0, 0, 0)
    #         setXSpeed(-100)
    #         setDistance(distance)
    #         command[17] = tem_listener(command, 17)
    #         duplicateWriteCmd(ser, command)
    #         while getDistanceArrive() == "0" and redis_cli.get("mission") == "working":
    #             print("wait Distance finish")
    #             global_status = "wait Distance finish"
    #         break
    #     time.sleep(0.3)


def goBackByLength(ser, length, speed=-100):
    # 开启纠偏
    redis_cli.set("correct", "true")
    global global_status
    reSetStatus(ser)

    setStatus(2)
    setPowerOn(1)
    setHWstatus(0, 0, 0, 0, 0)
    setXSpeed(speed)
    setDistance(length)
    command[17] = tem_listener(command, 17)
    duplicateWriteCmd(ser, command)
    # 这里开始判断路径和到边,1能走，0不行
    while getEdge() == "1" and getDistanceArrive() == "0":
        if redis_cli.get("mission") == "complete":
            return 0
        print("keep walking")
        # global_status = "keep walking"
        time.sleep(0.25)
    time.sleep(0.5)
    if getDistanceArrive() == "1":
        # global_status = "到达"
        print("到达")
        return 1
    else:
        print("未到达，结束")
        # global_status = "未到达，结束"
        return 0


def goByLength(ser, length, speed=100):
    global global_status
    # 开启纠偏
    redis_cli.set("correct", "true")
    reSetStatus(ser)
    setStatus(2)
    setPowerOn(1)
    setHWstatus(0, 0, 0, 0, 0)
    setXSpeed(speed)
    setDistance(length)
    command[17] = tem_listener(command, 17)
    duplicateWriteCmd(ser, command)
    startTime = time.time()
    # 这里开始判断路径和到边,1能走，0不行
    # getEdge() == "1" and
    while getDistanceArrive() == "0":
        if redis_cli.get("mission") == "complete":
            return 0
        print("keep walking")
        global_status = "keep walking"
        time.sleep(0.25)
        endTime = time.time()
        interval_time = endTime - startTime
        if interval_time > 8:
            # 开启纠偏
            redis_cli.set("correct", "true")
    time.sleep(0.5)

    if getDistanceArrive() == "1":
        # global_status = "到达"
        print("到达")
        return 1
    else:
        print("未到达，结束")
        # global_status = "未到达，结束"
        return 0


def moveDiatance(ser, length, speed=100):
    logger.warn("向前移动{}cm".format(length))
    logger.warn(redis_cli.get("mission"))
    if redis_cli.get("mission") == "complete":
        return
    global global_status
    reSetStatus(ser)

    setStatus(2)
    setPowerOn(1)
    setHWstatus(0, 0, 0, 0, 0)

    setXSpeed(speed)
    setDistance(length)

    # setDistance(110)
    command[17] = tem_listener(command, 17)
    logger.warn(' '.join(format(x, '02x') for x in command))
    duplicateWriteCmd(ser, command)

    # 这里开始判断路径和到边,1能走，0不行
    while getEdge() == "1" and getDistanceArrive() == "0" and redis_cli.get("mission") == "working":
        if global_go == 0:
            break
        if redis_cli.get("mission") == "complete":
            return 0
        print("keep walking")
        # global_status = "keep walking"
        time.sleep(0.25)
    time.sleep(0.5)
    if getDistanceArrive() == "1":
        global_status = "到达"
        print("到达")
        return 1
    else:
        print("未到达，结束")
        # global_status = "未到达，结束"
        return 0


def turn(ser, roundTo):
    logger.warn('转向:{}'.format(roundTo))
    redis_cli.set("ultraSonic", "false")
    redis_cli.set("mission","working")
    # logger.warn(redis_cli.get("mission"))
    if redis_cli.get("mission") == "complete":
        return
    global global_status
    reSetStatus(ser)
    setStatus(3)
    setRotateTo(roundTo)
    setZSpeed(0.6)
    command[17] = tem_listener(command, 17)
    logger.warn(' '.join(format(x, '02x') for x in command))
    duplicateWriteCmd(ser, command)

    # 如果未完成，则继续阻塞，等待旋转完成 and redis_cli.get("mission") == "working"
    while getRotateArrive() == "0" and redis_cli.get("mission") == "working":
        # if redis_cli.get("ultraSonic") == "true":
        #     break
        logger.info("wait rotate finish")
        # global_status = "wait rotate finish"

    time.sleep(0.5)

    redis_cli.set("carStatus", "go")
    # 传感器报警了，到边了
    # if redis_cli.get("ultraSonic") == "true":
    #     moveBack(ser, distance=9)
    #     turn(ser, roundTo)


def exit_uav(ser):
    preBuildCommand()
    command[1] = 0xFB
    command[2] = 0x01
    command[4] = 0x00
    command[5] = 0x00
    command[8] = 0x00
    command[9] = 0x00
    duplicateWriteCmd(ser, command)
    time.sleep(0.1)


def switch_off_clean_mode(ser):
    preBuildCommand()

    command[1] = 0xEA

    command[2] = 0x01

    command[4] = 0x00

    command[5] = 0x00

    command[8] = 0x00

    command[9] = 0x00

    command[10] = 0x00

    duplicateWriteCmd(ser, command)

    time.sleep(0.1)


def switch_on_clean_mode(ser):
    preBuildCommand()
    command[1] = 0xFA
    command[2] = 0x01
    command[4] = 0x00
    command[5] = 0x00
    command[8] = 0x00
    command[9] = 0x00
    logger.warn('记录当前航向角')
    logger.warn(' '.join(format(x, '02x') for x in command))
    duplicateWriteCmd(ser, command)
    time.sleep(0.1)
def reset_clean_mode(ser):
    preBuildCommand()
    command[1] = 0xFB
    command[2] = 0x01
    command[4] = 0x00
    command[5] = 0x00
    command[8] = 0x00
    command[9] = 0x00
    logger.warn('清除当前航向角')
    logger.warn(' '.join(format(x, '02x') for x in command))
    duplicateWriteCmd(ser, command)
    time.sleep(0.1)

def sendInitHeading(ser,heading):
    preBuildCommand()
    normalized_heading = float(heading) % 360.0
    heading_centidegrees = int(round(normalized_heading * 100.0)) % 36000
    byteArr = getCmdBytes(heading_centidegrees)
    command[17] = byteArr[0]
    command[18] = byteArr[1]
    logger.warn('发送初始航向角给下位机')
    logger.warn(' '.join(format(x, '02x') for x in command))
    duplicateWriteCmd(ser, command)

def reset_odometer(ser):
    logger.warn('清除里程计')
    preBuildCommand()
    command[1] = 0xE0
    command[2] = 0x01
    command[4] = 0x00
    command[5] = 0x00
    command[8] = 0x00
    command[9] = 0x00

    logger.warn(' '.join(format(x, '02x') for x in command))
    duplicateWriteCmd(ser, command)
    time.sleep(0.1)


def goUp(back_len=33):
    logger.warn("moving up...")
    global drivingUp
    global global_go
    redis_cli.set("forwardSpeed", high_speed)
    drivingUp = True
    if redis_cli.get("mission") == "complete":
        return
    global global_status
    global cap
    cap.release()
    time.sleep(0.5)
    command[0] = 123
    command[17] = 125
    reSetStatus(ser)
    global_go = 1
    justMove(ser)
    global_go = 0
    logger.warn("move up end.")
    logger.warn("moving back...")
    reSetStatus(ser)
    # global_status = "back"
    if redis_cli.get("mission") == "complete":
        return
    moveBack(ser, back_len)
    logger.warn("move back end.")
    drivingUp = False
    redis_cli.set("forwardSpeed", high_speed)


def goStop():
    setStatus(0)
    command[17] = tem_listener(command, 17)
    duplicateWriteCmd(ser, command)


def goRound():
    if redis_cli.get("mission") == "complete":
        return

    pathPlanning = redis_cli.get(PATH_PLANNING_KEY)

    while redis_cli.get("mission") == "working":

        command[0] = 123

        command[17] = 125

        if pathPlanning == LEFT_PATH_PLANNING:

            turn(ser, 90 * 10)

        else:

            turn(ser, 270 * 10)

        reset_odometer(ser)

        justMove(ser)

        logger.info("前进结束")

        moveBack(ser)

        turn(ser, 180 * 10)

        reset_odometer(ser)

        if not moveDiatance(ser, 100) or redis_cli.get("mission") == "complete":
            break

        if pathPlanning == LEFT_PATH_PLANNING:

            turn(ser, 270 * 10)

        else:

            turn(ser, 90 * 10)

        reset_odometer(ser)

        justMove(ser)

        logger.info("前进结束")

        moveBack(ser)

        turn(ser, 180 * 10)

        reset_odometer(ser)

        if not moveDiatance(ser, 100) or redis_cli.get("mission") == "complete":
            break

    moveBack(ser)

    turn(ser, 0)

    switch_off_clean_mode(ser)


def horizontal_env_auto_clean():
    while True:

        drive_up()

        while True:
            cell_board_loop()

            cross_brige()

        drive_right()

        cross_brige()

    back_to_base()

    while True:
        drive_left()

        cross_brige()


def init_status():
    redis_cli.set("mission", "working")

    redis_cli.set("correct", "true")

    redis_cli.set('action', 'true')


def drive_up():
    init_status()

    redis_cli.set("driveUp", "true")

    switch_on_clean_mode(ser)

    if redis_cli.get("mission") == "complete":
        return

    forward_speed = 280

    brush_speed = int(redis_cli.get("brushSpeed"))

    reSetStatus(ser)

    setStatus(1)

    setPowerOn(1)

    setHWstatus(0, 0, 0, 0, 0)

    setXSpeed(forward_speed)

    setBrushSpeed(brush_speed)

    command[17] = tem_listener(command, 17)

    duplicateWriteCmd(ser, command)

    # 这里开始判断路径和到边,1能走，0不行

    while getEdge() == "1" and redis_cli.get("mission") == "working":
        print("keep walking")

        time.sleep(0.25)

    time.sleep(0.5)

    reSetStatus(ser)

    moveBack(ser)

    redis_cli.set("driveUp", "false")


def doCleanThreadByRTK():
    global taskList  # 申明使用全局变量
    global global_doCleanThreadStop
    if len(taskList) == 0:
        taskList = util.readConfig("config_rtk.json")

    # 重置陀螺仪
    switch_on_clean_mode(ser)

    # 如果启动自动清扫任务，那redis中的任务列表就要被清除，然后再初始化
    redis_cli.delete('taskList')
    # logger.warn(taskList)
    for item in taskList:
        # 将字典转为JSON字符串存储
        redis_cli.rpush('taskList', json.dumps(item))
    if doCleanByRTK(taskList) == 0:
        logger.warn("清扫工作未完成")
    else:
        logger.warn("清扫工作完成")
    sendBraking()
    logger.warn("任务执行结束")
    redis_cli.set("mission", "complete")
    redis_cli.set("correct", "false")
    redis_cli.set('action', 'false')
    # 表示自动清扫线程停止
    global_doCleanThreadStop = 1


def doCleanThread():
    global taskList  # 申明使用全局变量
    global global_doCleanThreadStop

    taskList = util.readConfig("config_rtk.json")
    for task in taskList:
        converterXY(task)
    # 获取当前坐标点，判断当前点位是否在充电桩中
    if isGarage(32.03646721, 118.92448852):
        # 后退出充电桩
        reset_odometer(ser)
        redis_cli.set("mission", "working")
        moveBack(ser, 150)
        if redis_cli.get('parking') == '1':
            global_doCleanThreadStop = 1
        # 如果后退没有到达，则不往下执行
        if getDistanceArrive() == 0:
            return
    # 位置校验
    if turnCheckPoint() == 0:
        return
    # 重置陀螺仪
    switch_on_clean_mode(ser)
    # 如果启动自动清扫任务，那redis中的任务列表就要被清除，然后再初始化
    redis_cli.delete('taskList')
    # logger.warn(taskList)
    for item in taskList:
        # 将字典转为JSON字符串存储
        redis_cli.rpush('taskList', json.dumps(item))
    if doClean(taskList) == 0:
        logger.warn("清扫工作未完成")
    else:
        logger.warn("清扫工作完成")
    sendBraking()
    logger.warn("任务执行结束")
    redis_cli.set("mission", "complete")
    redis_cli.set("correct", "false")
    redis_cli.set('action', 'false')

    # 如果自动清扫被停止，则不继续运行
    if redis_cli.get('parking') == '0':
        logger.warn('进充电桩')
        reset_odometer(ser)
        turn(ser, 180 * 10)
        # justMoveByRTK(1.20,186)
        moveByRTK(32.03646721, 118.92448852, 186)
    # 表示自动清扫线程停止
    # global_doCleanThreadStop = 1



# 自动清扫,goon默认为false,如果传入True，则表示是继续清扫
# 如果返回0，表示被中断了，返回1表示执行结束
def doClean(tmpTaskList, goon=False):
    global drivingUp
    global global_cur_task_index
    global global_cur_taskPoint
    global global_go
    global global_is_need_rtk

    try:
        redis_cli.set("mission", "working")
        redis_cli.set("correct", "true")
        redis_cli.set('action', 'true')
        # 使其每一次接口调用，都重新开始
        redis_cli.set('parking', 0)
        reset_odometer(ser)

        logger.warn('任务获取成功！开始执行任务')
        for index, item in enumerate(tmpTaskList):
            # 通过重redis中获取一个值，看是否是暂停,0为false,1为true
            if redis_cli.get('parking') == '0':
                id = item['id']
                angle = item['angle']
                mode = item['mode']
                length = item['length']
                back_len = item['back_len']
                turn_back_len = item['turn_back_len']

                global_cur_task_index = id - 1
                redis_cli.set('curTaskIndex', global_cur_task_index)
                try:
                    startLat = item['startLat']
                    startLon = item['startLon']
                    endLat = item['endLat']
                    endLon = item['endLon']
                    heading = item['heading']
                    # 获取当前任务开始点和结束点的经纬度
                    global_cur_taskPoint = {"startLat": startLat, "startLon": startLon, "endLat": endLat,
                                            "endLon": endLon, "heading": heading}
                except (KeyError, TypeError) as e:
                    logger.error(e)
                    pass

                logger.warn(
                    '执行任务{}；angle={},mode={},length={},turn_back_len={},back_len={}'.format(id, angle, mode, length,
                                                                                                turn_back_len,
                                                                                                back_len))
                # 在这里根据 action 值执行不同操作，1无限走，2有限
                if mode == 1 and angle == 0:
                    if index != 0:
                        turn(ser, angle * 10)
                    reset_odometer(ser)
                    goUp(back_len)
                    drivingUp = False
                    redis_cli.set("forwardSpeed", high_speed)
                elif mode == 1 and angle != 0:
                    if goon == True and index == 0:
                        pass
                    else:
                        turn(ser, angle * 10)
                        reset_odometer(ser)
                    if angle == 180:
                        pass
                    else:
                        if goon == True and index == 0:
                            pass
                        else:
                            # 转向以后，左右直行前的后退
                            moveBack(ser, turn_back_len)
                    # justMove(ser)
                    # 直行之后的后退，需要区分是往上直行的任务，还是左右直行的任务
                    goUp(back_len)
                    logger.warn("前进结束")
                    # moveBack(ser, back_len)
                elif mode == 2:
                    # 先判断电量，如果电量少于10，则跑完这个任务，然后停止
                    voltageStr = redis_cli.get("voltage")
                    voltage = 10 if voltageStr is None else int(voltageStr)
                    # voltage = int(redis_cli.get("voltage"))
                    voltageWarn = int(redis_cli.hget('taskParam', 'voltageWarn')) if redis_cli.hget('taskParam',
                                                                                                    'voltageWarn') else 0
                    # 如果是继续清扫，第一个任务就不需要转弯了
                    if goon == True and index == 0:
                        # 如果是继续运行，并且是第一个任务，那么就不需要转弯了
                        pass
                    else:
                        turn(ser, angle * 10)
                        reset_odometer(ser)
                    if angle == 180 or angle == 90:
                        pass
                    else:
                        # 转向以后，左右直行前的后退
                        moveBack(ser, turn_back_len)
                    # 这个标记，表示是否向下位机发送纠偏指令的信号，1：表示正在直行，需要纠偏；0：表示不是直行，不需要纠偏
                    if angle == 90:
                        global_go = 1
                    # redis_cli.set("correct","true")
                    global_go = 1
                    moveDiatance(ser, length, 250)
                    global_go = 0
                    if voltage <= voltageWarn:
                        parking()
                elif mode == 3:
                    turn(ser, angle * 10)
                    moveBack(ser, 26)
                    redis_cli.set("odometer_arrive", "false")
                    reset_odometer(ser)
                    justMove(ser)
                if redis_cli.get("parking") != "1":
                    # 任务完成，从redis list中清除该任务
                    logger.warn("删除任务")
                    redis_cli.lpop("taskList")
            else:
                logger.warn('外部设置停止自动清扫，任务未执行完！')
                return 0
        global_cur_taskPoint = {}
    except Exception as e:
        doParking()
        logger.error(traceback.format_exc())


def doCleanByRTK(tmpTaskList, goon=False):
    global drivingUp
    global global_cur_task_index
    global global_cur_taskPoint
    global global_go
    global global_is_need_rtk

    try:
        redis_cli.set("mission", "working")
        # redis_cli.set("correct", "true")
        # redis_cli.set('action', 'true')
        set_current_action('auto_drive')
        # 使其每一次接口调用，都重新开始
        redis_cli.set('parking', 0)
        reset_odometer(ser)

        logger.warn('任务获取成功！开始执行任务')
        for index, item in enumerate(tmpTaskList):
            # 通过重redis中获取一个值，看是否是暂停,0为false,1为true
            if redis_cli.get('parking') == '0':
                id = item['id']
                angle = item['angle']
                mode = item['mode']
                length = item['length']
                back_len = item['back_len']
                turn_back_len = item['turn_back_len']

                global_cur_task_index = id - 1
                redis_cli.set('curTaskIndex', global_cur_task_index)
                try:
                    startLat = item['startLat']
                    startLon = item['startLon']
                    heading = item['heading']
                    endLat, endLon = util.get_B_GPS(startLat, startLon, length, heading)
                    # 获取当前任务开始点和结束点的经纬度
                    global_cur_taskPoint = {"startLat": startLat, "startLon": startLon, "endLat": endLat,
                                            "endLon": endLon}
                except (KeyError, TypeError) as e:
                    logger.error(e)
                    pass

                logger.warn(
                    '执行任务{}；angle={},mode={},length={},turn_back_len={},back_len={}'.format(id, angle, mode, length,
                                                                                                turn_back_len,
                                                                                                back_len))
                # 在这里根据 action 值执行不同操作，1无限走，2有限
                if mode == 1 and angle == 0:
                    turn(ser, angle * 10)
                    reset_odometer(ser)
                    goUp(back_len)
                    drivingUp = False
                    redis_cli.set("forwardSpeed", high_speed)
                elif mode == 1 and angle != 0:
                    if goon == True and index == 0:
                        pass
                    else:
                        turn(ser, angle * 10)
                        reset_odometer(ser)
                    if angle == 180:
                        pass
                    else:
                        if goon == True and index == 0:
                            pass
                        else:
                            # 转向以后，左右直行前的后退
                            moveBack(ser, turn_back_len)
                    # justMove(ser)
                    # 直行之后的后退，需要区分是往上直行的任务，还是左右直行的任务
                    goUp(back_len)
                    logger.warn("前进结束")
                    # moveBack(ser, back_len)
                elif mode == 2:
                    # 先判断电量，如果电量少于10，则跑完这个任务，然后停止
                    voltageStr = redis_cli.get("voltage")
                    voltage = 10 if voltageStr is None else int(voltageStr)
                    # voltage = int(redis_cli.get("voltage"))
                    voltageWarn = int(redis_cli.hget('taskParam', 'voltageWarn')) if redis_cli.hget('taskParam',
                                                                                                    'voltageWarn') else 0
                    # 如果是继续清扫，第一个任务就不需要转弯了
                    if goon == True and index == 0:
                        # 如果是继续运行，并且是第一个任务，那么就不需要转弯了
                        pass
                    else:
                        turn(ser, angle * 10)
                        reset_odometer(ser)
                    if angle == 180 or angle == 90:
                        pass
                    else:
                        # 转向以后，左右直行前的后退
                        moveBack(ser, turn_back_len)
                    # 这个标记，表示是否向下位机发送纠偏指令的信号，1：表示正在直行，需要纠偏；0：表示不是直行，不需要纠偏
                    # global_go = 1
                    moveDiatance(ser, length, 250)
                    # global_go = 0
                    if voltage <= voltageWarn:
                        parking()
                elif mode == 3:
                    turn(ser, angle * 10)
                    moveBack(ser, 26)
                    redis_cli.set("odometer_arrive", "false")
                    reset_odometer(ser)
                    justMove(ser)
                if redis_cli.get("parking") != "1":
                    # 任务完成，从redis list中清除该任务
                    logger.warn("删除任务")
                    redis_cli.lpop("taskList")
            else:
                logger.warn('外部设置停止自动清扫，任务未执行完！')
                return 0
        global_cur_taskPoint = {}
    except Exception as e:
        doParking()
        logger.error(traceback.format_exc())


def starttt():
    global drivingUp
    global counter
    global taskList  # 申明使用全局变量
    global global_go

    redis_cli.set("mission", "working")
    redis_cli.set("correct", "true")
    redis_cli.set('action', 'true')

    # 使其每一次接口调用，都重新开始
    redis_cli.set('parking', 0)
    reset_odometer(ser)

    taskList = util.readConfig("config_view.json")

    # 如果启动自动清扫任务，那redis中的任务列表就要被清除，然后再初始化
    redis_cli.delete('taskList')
    # 将任务存储到Redis列表
    for item in taskList:
        # 将字典转为JSON字符串存储
        redis_cli.rpush('taskList', json.dumps(item))
    # while redis_cli.get('parking') == '0':
    logger.warn('任务获取成功！开始执行任务')
    for item in taskList:
        global_go = 1
        # 通过重redis中获取一个值，看是否是暂停,0为false,1为true
        if redis_cli.get('parking') == '0':
            id = item['id']
            angle = item['angle']
            mode = item['mode']
            length = item['length']
            back_len = item['back_len']
            logger.warn('执行任务{}；angle={},mode={},length={},back_len={}'.format(id, angle, mode, length, back_len))
            # 在这里根据 action 值执行不同操作，1无限走，2有限
            if mode == 1 and angle == 0:
                # 打开清扫
                switch_on_clean_mode(ser)
                turn(ser, angle * 10)
                reset_odometer(ser)
                goUp(back_len)
                drivingUp = False
                redis_cli.set("forwardSpeed", high_speed)
            elif mode == 1 and angle != 0:
                turn(ser, angle * 10)
                reset_odometer(ser)
                if angle == 180:
                    pass
                else:
                    moveBack(ser, back_len)
                justMove(ser)
                logger.info("前进结束")
                moveBack(ser, back_len)
            elif mode == 2:
                turn(ser, angle * 10)
                reset_odometer(ser)
                if angle == 180:
                    pass
                else:
                    moveBack(ser, 26)
                moveDiatance(ser, length)
            elif mode == 3:
                turn(ser, angle * 10)
                moveBack(ser, 26)
                redis_cli.set("odometer_arrive", "false")
                reset_odometer(ser)
                justMove(ser)
            # 任务完成，从redis list中清除该任务
            redis_cli.lpop("taskList")
        else:
            logger.warn('外部设置停止自动清扫，任务未执行完！')
            break
    sendBraking()
    logger.warn("任务执行结束")
    redis_cli.set("mission", "complete")
    redis_cli.set("correct", "false")
    redis_cli.set('action', 'false')
    set_current_action('idle')
    redis_cli.set('curTaskIndex', 0)

    return "1"


def startGPS():
    pass
    # while True:

    #     try:

    #         ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

    #         time.sleep(60)  # 等待串口准备就绪（可选）

    #         line = ser.readline()  # 读取一行数据，直到遇到换行符'\n'

    #         try:

    #             requests.get('http://'+ip+':7899/sendGPS?id='+id+'&gps=' + line.decode().strip())

    #         except requests.exceptions.ConnectionError as e:

    #             print("链接失败")

    #     except serial.serialutil.SerialException as eee:

    #         print("串口链接失败")

    #     finally:

    #         ser.close()

    #         time.sleep(10)


def testOpencv():
    '''

    global cap
    fourcc = cv2.VideoWriter_fourcc(*'MP4V')

    # out = cv2.VideoWriter('testwrite.mp4', fourcc, 20.0, (1920, 1080), True)

    out = cv2.VideoWriter('testwrite.mp4', fourcc, 20.0, (640, 480), True)

    while (cap.isOpened()):
        ret, frame = cap.read()
        if ret == True:
            cv2.imshow('frame', frame)
            out.write(frame)
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break
        else:
            break



    cap.release()
    out.release()
    cv2.destroyAllWindows()
    '''

    center_x = 0
    center_y = 0

    # file_name = 'D://testPics//WIN_20250417_15_25_29_Pro - Trim - frame at 0m46s.jpg'

    # file_name = 'D://testPics//WIN_20250417_15_25_29_Pro - frame at 0m29s.jpg'

    # file_name = 'D://testPics//Snipaste_2025-04-03_17-16-49.png'

    # file_name = 'D://testPics//Snipaste_2025-04-03_17-16-49copy.png'

    file_name = 'D://testPics//qrcode-origin.png'

    # if True:

    #     image = cv2.imread(file_name)

    #     image = image[200:880, 500:1420]

    #     height, width = image.shape[:2]

    #     center_x = width // 2

    #     center_y = height // 2

    #     logger.info('一帧x坐标: %d', center_x)

    #     logger.info('一帧y坐标: %d', center_y)

    # time.sleep(1)

    image = cv2.imread(file_name)
    # cv2.imshow('origin', image)
    # image = image[200:880, 500:1420]
    # cv2.imshow('trim', image)

    image = cv2.blur(image, (5, 5))

    # cv2.imshow('blur', image)

    # image = cv2.Canny(image, 10, 20)

    # cv2.imshow('edge', image)

    # image = cv2.GaussianBlur(image, (3, 3), 1, 2)

    # cv2.imshow('gauss', image)

    height, width = image.shape[:2]
    center_x = width // 2
    center_y = height // 2

    # logger.info('一帧x坐标: %d', center_x)

    # logger.info('一帧y坐标: %d', center_y)

    '''

    save_file = './img_save.png'

    cv2.imwrite(save_file, image)

    '''

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    lsd = cv2.createLineSegmentDetector(0)
    dlines = lsd.detect(gray)
    line = None
    f_x0 = 0
    f_y0 = 0
    f_x1 = 0
    f_y1 = 0
    if not (dlines[0] is None):
        logging.info('直线条数：%d', len(dlines[0]))
        for dline in dlines[0]:
            x0 = int(round(dline[0][0]))
            y0 = int(round(dline[0][1]))
            x1 = int(round(dline[0][2]))
            y1 = int(round(dline[0][3]))

            # todo 是否需要abs
            if y1 - y0 <= 70:
                cv2.line(image, (x0, y0), (x1, y1), (0, 255, 0), 1, cv2.LINE_AA)
                continue
            else:
                xmid = (x1 + x0) / 2
                offset = xmid - center_x
                if line is None:
                    line = offset
                if abs(offset) < abs(line):
                    line = offset
                    f_x0 = x0
                    f_y0 = y0
                    f_x1 = x1
                    f_y1 = y1
                cv2.line(image, (x0, y0), (x1, y1), (0, 255, 255), 1, cv2.LINE_AA)

    if line is None:
        pass

    logger.info('x0: %d, y0: %d, x1: %d, y1: %d', f_x0, f_y0, f_x1, f_y1)
    cv2.line(image, (f_x0, f_y0), (f_x1, f_y1), (0, 0, 255), 1, cv2.LINE_AA)
    cv2.circle(image, (center_x, center_y), 5, (0, 0, 255), -1)

    k = -(f_y1 - f_y0) / (f_x1 - f_x0)
    angle = int(np.arctan(k) * 57.29577)
    cv2.putText(image, 'offset: ' + str(line) + ', angle: ' + str(angle), (0, 50), cv2.FONT_HERSHEY_COMPLEX, 1,
                (0, 0, 255), 2)
    # cv2.imwrite(save_file, image)
    #logger.info('位置偏移: %d', line)

    cv2.imshow('final', image)
    cv2.waitKey(0)
    if line == 0:
        logger.error('=====================>位置偏移为0!')
    time.sleep(1)
    return 0


'''

def cmdConsumer():

    while True:

        try:

            cmd_msg = cmd_redis_cli.brpop("cmd_qu")

            if cmd_msg is None:

                time.sleep(1)

                continue

            logger.info(">>>msg:%s", cmd_msg)

            msg = cmd_msg[1]

            if msg.startswith("enterGarage"):

                enterGarage()

            elif msg.startswith("exitGarage"):

                exitGarage()

            elif msg.startswith("togglePathPlanning"):

                togglePathPlanning(msg.split("/")[1])

            elif msg.startswith("toggleTracking"):

                toggleTracking(msg.split("/")[1])

            elif msg.startswith("adjustSpeed"):

                adjust_speed(int(msg.split("/")[1]))

            elif msg.startswith("adjustBrushSpeed"):

                adjust_brush(int(msg.split("/")[1]))

            elif msg.startswith("autoDrive"):

                auto_driving()

            elif msg.startswith("parking"):

                parking()

            elif msg.startswith("drive"):

                driving()

            elif msg.startswith("back"):

                reverse()

            elif msg.startswith("turnLeft"):

                turn_left()

            elif msg.startswith("turnRight"):

                turn_right()

            elif msg.startswith("joystickMove"):

                distance = msg.split("/")[1]

                dirX = msg.split("/")[2]

                dirY = msg.split("/")[3]

                joystick_move(distance, dirX, dirY)

        except:

            pass

'''


def listenerSlavePort():
    cur_sn_signal = 0
    previous_sn_signal = 0
    while True:
        if ser != None and ser.is_open:
            if redis_cli.get("moveJudge") == "true":
                time.sleep(2)
                continue
            try:
                data = ser.read(CMD_LEN * 2)
                # logger.info(data)
                # logger.info(' '.join(x.encode('hex') for x in data))
                # 数据长度不够不要
                if len(data) < CMD_LEN:
                    time.sleep(2)
                    continue
                start_index = None
                for idx, q in enumerate(data):
                    if binascii.b2a_hex(q) == '7b':
                        start_index = idx
                        break
                if start_index is None:
                    time.sleep(2)
                    continue
                data = data[start_index:]
                if len(data) < CMD_LEN:
                    time.sleep(2)
                    continue
                _cache_hardware_status_frame(data)
                # 激光传感器值
                jgValue = int(binascii.b2a_hex(data[11]), 16)
                # logging.info('获取到激光传感器值: %d', jgValue)
                # redis_cli.set("jgValue", jgValue)
                # 电池电压值
                voltage = int(binascii.b2a_hex(data[10]), 16)
                # logging.info('获取到电压值: %d', voltage)
                # redis_cli.set('voltage', voltage)
                # str1 = binascii.b2a_hex(data[14])
                # str2 = binascii.b2a_hex(data[15])
                # voltage = int(str1 + str2, 16)
                # if voltage > 0:
                #     logging.info('获取到电压值: %d', voltage)
                #     roundVoltage = round(voltage * 0.01, 1)
                #     roundVoltage = round((roundVoltage - 23) / (28 - 23)) * 100
                #     redis_cli.set('voltage', roundVoltage)
                # 陀螺仪值
                # str1 = binascii.b2a_hex(data[16])
                # str2 = binascii.b2a_hex(data[17])
                # tlyValue = int(str1 + str2, 16)
                # logger.warn('获取到陀螺仪值: %d', int(tlyValue/10))
                # redis_cli.set('tlyValue', int(tlyValue / 10))
                # 使能信号
                # snStr = binascii.b2a_hex(data[2])
                # cur_sn_signal = int(snStr, 16)
                # # 信号从0变为1时才执行操作
                # if previous_sn_signal == 0 and cur_sn_signal == 1:
                #     thread = threading.Thread(target=autoDriveByRTKThread)
                #     thread.start()
                #     logger.warn("启动清扫线程成功")
                # previous_sn_signal = cur_sn_signal
            except Exception as e:
                logging.info('监听报错:{}'.format(e))
                time.sleep(2)
            time.sleep(0.1)


# def get_cpu_temperature():

#     try:

#         temperatures = psutil.sensors_temperatures()

#         if 'coretemp' in temperatures:

#             for entry in temperatures['coretemp']:

#                 if entry.label == 'Package id 0':

#                     return entry.current

#     except Exception as e:

#         print("Error getting CPU temperature: ", e)

#     return None


def pushMetadata():
    metadata = {}
    while True:
        voltage = redis_cli.get('voltage')
        if voltage != None:
            metadata['voltage'] = voltage

        # airPressure = redis_cli.get('airPressure')
        # if airPressure != None:
        #     metadata['airPressure'] = airPressure
        metadata['forwardSpeed'] = redis_cli.get('forwardSpeed')
        metadata['brushSpeed'] = redis_cli.get('brushSpeed')
        metadata['correct'] = redis_cli.get('correct')
        metadata[PATH_PLANNING_KEY] = redis_cli.get(PATH_PLANNING_KEY)
        # notify(json.dumps(metadata))
        time.sleep(1)


def startOpencv():
    global drivingUp
    global cap

    center_x = 0
    center_y = 0

    while True:
        ret, image = cap.read()
        if not ret:
            logger.error('无法读取视频流或文件结束')
            return
        else:
            height, width = image.shape[:2]
            logger.info('图片高度: %d, 宽度: %d', height, width)
            image = image[100:380, 125:515]
            height, width = image.shape[:2]
            center_x = width // 2
            center_y = height // 2
            logger.info('一帧x坐标: %d', center_x)
            logger.info('一帧y坐标: %d', center_y)
            break
    cap.release()
    time.sleep(1)
    stopThenStart()

    lower1 = np.array([9, 250, 250])
    upper1 = np.array([10, 255, 255])
    lower2 = np.array([170, 140, 140])
    upper2 = np.array([180, 255, 255])
    while True:
        try:
            if "false" == redis_cli.get("correct"):
                time.sleep(0.1)
                continue

            ret, image = cap.read()
            if not ret:
                logger.error('无法读取视频流')
                stopThenStart()
                continue

            image = image[100:380, 125:515]
            image = cv2.blur(image, (5, 5))
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            lsd = cv2.createLineSegmentDetector(0)
            dlines = lsd.detect(gray)
            line = None
            f_x0 = 0
            f_y0 = 0
            f_x1 = 0
            f_y1 = 0
            selected_length = 0.0
            line_angle = 0  # 夹角
            if not (dlines[0] is None):
                for dline in dlines[0]:
                    x0 = int(round(dline[0][0]))
                    y0 = int(round(dline[0][1]))
                    x1 = int(round(dline[0][2]))
                    y1 = int(round(dline[0][3]))

                    vertical_span = abs(y1 - y0)
                    if vertical_span <= 70:
                        continue
                    dx = x1 - x0
                    dy = y1 - y0
                    length = math.hypot(dx, dy)
                    if length < 100:
                        continue
                    vertical_angle = math.degrees(math.atan2(abs(dx), abs(dy))) if abs(dy) >= 1e-5 else 90
                    if vertical_angle > 35:
                        continue

                    xmid = (x1 + x0) / 2
                    offset = xmid - center_x
                    if line is None or abs(offset) < abs(line) or (abs(offset) == abs(line) and length > selected_length):
                        line = offset
                        f_x0 = x0
                        f_y0 = y0
                        f_x1 = x1
                        f_y1 = y1
                        selected_length = length
                        line_angle = math.degrees(math.atan2(dx, dy))
            if line is None:
                logger.info("没有找到线")
                line = 0
            if line == 0:
                logger.info('=====================>位置偏移为0!, 角度: %d', line_angle)
            else:
                logger.info('位置偏移: %d, 角度: %d', line, line_angle)
                # 存储角度
                redis_cli.set("angle", int(line_angle))
            # 小车直行时，角度大于10度，就操作下面逻辑
            if line_angle > 10:
                line = -line_angle * 2
            setZSpeed(line)
            duplicateWriteCmd(ser, command)

            if redis_cli.get("enterGarage") == "true" and redis_cli.get("detectQrcode") == "false":
                hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                mask1 = cv2.inRange(hsv_img, lower1, upper1)
                mask2 = cv2.inRange(hsv_img, lower2, upper2)
                mask = mask1 + mask2

                if np.sum(mask) > 0:
                    redis_cli.set("detectQrcode", "true")
                    redis_cli.set('enterGarage', 'false')
                else:
                    continue

            # if redis_cli.get('video') == 'finish':

            #     break
        except Exception as e:
            logger.error('纠偏出错: {}'.format(e))
            pass

    cap.release()

    # out.release()

    time.sleep(1)

    return 0


def is_network_reachable(host="8.8.8.8"):
    param = '-n' if platform.system().lower() == 'windows' else '-c'

    command = ['ping', param, '1', host]

    return os.system(' '.join(command)) == 0


def varifyGps():
    gps_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    if (not is_network_reachable("www.baidu.com")
            and not is_network_reachable("www.bing.com")
            and not is_network_reachable()):
        logger.error("网络连接失败")
        return False

    lot = 0.0
    lat = 0.0
    while True:
        try:
            gps_info = gps_port.readline()
            infos = gps_info.split(',')
            if infos[2] and infos[4]:
                latList = list(str(infos[2]).replace('.', ''))
                latList.insert(2, '.')
                lat = float("".join(latList))
                lotList = list(str(infos[4]).replace('.', ''))
                lotList.insert(3, '.')
                lot = float("".join(lotList))
                break
            else:
                time.sleep(0.2)
        except:
            pass

    try:
        response = requests.get(
            'http://218.2.130.246:7899/canStart?id=30f4b4af-c1a8-f3f6-072d-3807918c0dc0&lot=' + lot + '&lat=' + lat,
            timeout=10)
        if response.content.decode("utf-8") != '1':
            logger.info("超出限定区域范围")
            return False
        else:
            return True
    except:
        pass
# 判断是否需要回充电桩
def isNeedReturnCharging():
    length = 0
    if None == global_cur_rtk_lat:
        return False

    # 判断当前到那个任务了
    json_item = redis_cli.lindex('taskList', 0)
    json_next_item = redis_cli.lindex('taskList', 1)

    if json_item == None or json_next_item == None:
        return False
    next_item = json.loads(json_next_item)
    item = json.loads(json_item)

    if item['angle'] == 90:
        startX = item['startX']
        startY = item['startY']
        dis,heading = util.get_distance_angle(global_cur_rtk_lat,global_cur_rtk_lon,item['startLat'],item['startLon'])
        length = dis + startX + startY
    elif item['angle'] == 180 and next_item['angle'] == 270:
        endX = item['endX']
        endY = item['endY']
        length = item['length'] + endX + endY
    elif item['angle'] == 180 and next_item['angle'] == 90:
        startX = item['startX']
        startY = item['startY']
        length = startX + startY
    elif item['angle'] == 270:
        dis, heading = util.get_distance_angle(global_cur_rtk_lat, global_cur_rtk_lon, item['endLat'], item['endLon'])
        startX = item['startX']
        startY = item['startY']
        length = dis + startX + startY
    # 获取电量值
    voltage = int(redis_cli.get("voltage"))
    length = int(length)
    # 假设1个电量可以跑1m
    # 剩余电量可以跑
    voltage_dis = voltage * 50
    logger.info("电量距离：{},到充电桩距离:{}".format(voltage_dis,length))
    # 如果返回距离大于等于电池剩余电量距离，则需要返回充电桩
    if length >= voltage_dis:
        return True
    return False


def listenerVoltage():
    global global_status,drive_thread

    while True:
        # 如果voltageLisener为1表示开启电池监听
        if redis_cli.get('voltageListener') == '1':
            voltage = int(redis_cli.get("voltage"))
            logger.info("status={},voltage={}".format(global_status,voltage))
            # 当小车状态不是返回充电桩时并且需要返回充电桩充电时,就立即停止小车，并返回充电桩充电
            if global_status != 'goCharging' and isNeedReturnCharging() and redis_cli.llen('taskList') != 0:
                # global_status = 'goCharging'
                doParking()
                # if global_status == 'goCharging':
                if global_doCleanThreadStop == 1:
                    logger.warning("返回充电桩")
                    thread = threading.Thread(target=returnToPointByRTKThread)
                    thread.start()

            # 当电量大于93时并且有未完成的任务，则继续清扫未完成的任务
            if voltage >= 90 and redis_cli.llen('taskList') != 0:
                # 状态等于工作状态或者不等于从充电桩后退的状态，就可以启动
                if global_status != 'working':
                    if drive_thread is None or not drive_thread.is_alive():
                        drive_thread = threading.Thread(target=autoDriveByRTKThread)
                        drive_thread.start()
        # 防止cpu过载
        time.sleep(1)



def listenerRTK():
    if rtk_port != None:
        # 在RTK读线程启动前先建立NTRIP差分链路,失败不阻塞后续读取
        try:
            if get_shared_runtime(logger).prepare():
                logger.info("NTRIP差分链路已预连接")
        except Exception as exc:
            logger.error("NTRIP预连接异常: {}".format(exc))

        rtk_manager = RTKDataManager(rtk_port, baudrate=115200)
        # 注册多个观察者
        rtk_manager.register_observer(observer_rtk_data)
        rtk_manager.register_observer(observer_go_correct)
        # 实时计算到充电桩位置
        # rtk_manager.register_observer(observer_to_chargingPile)
        # 启动
        rtk_manager.start()
    else:
        logger.warn("rtk串口为空")

# mqtt模块
global_mqtt_integration = None
def init_mqtt(redis_client=None):
    """
    初始化MQTT集成（单例模式）
    Args:
        config: MQTT配置
        vehicle_controller: 车辆控制器
        redis_client: Redis客户端

    Returns:
        MQTTIntegration: MQTT集成实例
    """
    global global_mqtt_integration

    # 构建config
    config = util.readConfig('mqtt_config.json')
    # 创建小车控制器
    vehicle_controller = VehicleControllerAdapter()

    if global_mqtt_integration is None:
        global_mqtt_integration = MQTTIntegration(config, vehicle_controller, redis_client)
        global_mqtt_integration.start()

    return global_mqtt_integration

# def my_message_handler(topic, payload):
#     logger.warn("[自定义处理] 主题: {}, 消息: {}".format(topic,payload))
#     if topic == controllerTopic:
#         json_data = json.loads(payload)
#         command = json_data['command']
#         logger.info("执行命令：{}".format(command))
#     elif topic == setCronTopic:
#         redis_cli.set('taskCron', payload)
#         dyn_scheduler.update_cron("autoDrive", payload)
#
# # 创建客户端实例（使用公共测试服务器）
# mqtt_client = MqttClient(
#     broker="218.2.130.246",
#     port=1883,
#     client_id=vehicleType + '-' + vehicleId + "-client",
#     on_message_callback=my_message_handler  # 可选自定义回调
# )
#
# # 连接并订阅
# mqtt_client.connect()
#
# registerInfo = {
#   "deviceId": vehicleId,
#   "timestamp": int(time.time()),
#   "data": {
#     "name": "履带式"+vehicleId,
#     "serialNumber": "v483hf934hf",
#     "brand": "大疆",
#     "model": "M300",
#     "vehicleType": "tracklayer",
#     "batteryCapacity": "100",
#     "weight": 500,
#     "cleaningWidth": 2.5,
#     "location": {
#       "lat": global_cur_rtk_lat,
#       "lon": global_cur_rtk_lon
#     }
#   }
# }
# registerTopic = "vehicle/" + vehicleId + "/register"
# # 功能订阅主题
# controllerTopic = "vehicle/" + vehicleId + "/controller"
# mqtt_client.subscribe(controllerTopic)
# # 设置cron主题
# setCronTopic = "vehicle/" + vehicleId + "/setCron"
# mqtt_client.subscribe(setCronTopic)
# # 发布消息
# mqtt_client.publish(registerTopic, json.dumps(registerInfo))
#
# def sendHeartbeat():
#     hearBeatTopic = "vehicle/" + vehicleId + "/heartbeat"
#     while True:
#         hearbeatInfo = {
#             "deviceId": vehicleId,
#             "timestamp": int(time.time()),
#             "data": {
#                 "status": global_status,
#                 "battery": 85,
#                 "location": {
#                     "lat": global_cur_rtk_lat,
#                     "lon": global_cur_rtk_lon
#                 },
#             }}
#         mqtt_client.publish(hearBeatTopic, json.dumps(hearbeatInfo))
#         time.sleep(5)

# 启动定时任务
def task_hello():
    logger.info("hello")

# 创建调度器实例
dyn_scheduler = DynamicCronScheduler()
cron = redis_cli.get('taskCron')
if cron == None:
    cron = "0 10 * * *"
# 添加任务：每分钟执行
# dyn_scheduler.add_job("myjob", task_hello, cron="*/10 * * * *")
dyn_scheduler.add_job("autoDrive", autoDriveByRTKThread, cron=cron)

@app.route("/vehicle/updateCron", methods=['GET'])
def updateCron():
    cron = request.args.get('cron')
    redis_cli.set('taskCron', cron)
    dyn_scheduler.update_cron("autoDrive", cron)
    return make_response("设置成功")

def main():
    # if not varifyGps():
    #     return
    thread = threading.Thread(target=startOpencv)
    thread.start()

    listener_thread = threading.Thread(target=listenerSlavePort)
    listener_thread.start()

    # # 监听RTK模块线程
    listener_rtk_thread = threading.Thread(target=listenerRTK)
    listener_rtk_thread.start()

    # 向服务器发送心跳值
    # sendHearbeatThread = threading.Thread(target=sendHeartbeat)
    # sendHearbeatThread.start()

    # 电池管理，监控电池电量，获取已经跑了的里程数
    listenerVoltageThread = threading.Thread(target=listenerVoltage)
    # listenerVoltageThread.start()

    # 调用Ntrip2Uart2主要方法
    # Ntrip2Uart2.main()
    '''
    cmd_thread = threading.Thread(target=cmdConsumer)
    cmd_thread.start()
    '''

    # 启动websocket服务
    # server = websocket_server(9000)
    # server.start()
    # ws_thread = threading.Thread(target=pushMetadata)
    # ws_thread.start()

    # 启动mqtt
    init_mqtt(redis_cli)

    # 启动flask后台服务
    app.run(host='0.0.0.0', port=7899)


if __name__ == '__main__':
    main()
