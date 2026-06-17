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
from garage_state import (
    EXIT_DECISION_ALLOW,
    EXIT_DECISION_BLOCKED,
    EXIT_DECISION_CONFIRM,
    GARAGE_STATE_DOCKED_BY_COMMAND,
    GARAGE_STATE_DOCKED_MANUAL_CONFIRMED,
    GARAGE_STATE_ENTERING,
    GARAGE_STATE_EXITING,
    GARAGE_STATE_KEY,
    GARAGE_STATE_OUTSIDE,
    GARAGE_STATE_REASON_KEY,
    GARAGE_STATE_UNKNOWN,
    GARAGE_STATE_UPDATED_AT_KEY,
    decide_auto_exit_garage,
    normalize_garage_state,
)
from rtk_correction import compute_linear_steering
from battery_return import LOW_BATTERY_RETURN_THRESHOLD, should_return_to_charge
from edge_target_guard import (
    DEFAULT_EDGE_TARGET_TOLERANCE_M,
    should_accept_edge_stop,
    should_recover_from_edge_stop,
)
from DynamicCronScheduler import DynamicCronScheduler
from FixedPositiveChecker import FixedPositiveChecker

from flask_cors import CORS

from MqttClient import MqttClient
from ntrip_runtime import get_shared_runtime, reset_shared_runtime
from RTKDataManager import RTKDataManager
# from pid import PID

from mqtt_integration import MQTTIntegration
from mqtt_vehicle_adapter import VehicleControllerAdapter
from vision_line_detection import GuidanceBandTracker, find_vertical_bright_band, resolve_guidance_command
from local_redis import create_redis_client

app = Flask(__name__)
CORS(app)

LOCAL_MODE = os.getenv("CLEAN_LOCAL_MODE") == "1"

canStart = 1

global clients

clients = {}

CMD_LEN = 23

PATH_PLANNING_KEY = "pathPlanning"

LEFT_PATH_PLANNING = "left_path_planning"

RIGHT_PATH_PLANNING = "right_path_planning"

LOOP_AUTO_CLEAN_ENABLED_KEY = "loopAutoCleanEnabled"
LOOP_AUTO_CLEAN_RUNNING_KEY = "loopAutoCleanRunning"
LOOP_AUTO_CLEAN_CYCLE_KEY = "loopAutoCleanCycle"
LOOP_AUTO_CLEAN_STOP_REASON_KEY = "loopAutoCleanStopReason"
LOOP_AUTO_CLEAN_UPDATED_AT_KEY = "loopAutoCleanUpdatedAt"
LOOP_AUTO_CLEAN_SLEEP_SECONDS = 2.0

high_speed = 350

high_brush_speed = 30

ip = "218.2.130.246"

id = "30f4b4af-c1a8-f3f6-072d-3807918c0dc0"

redis_cli = create_redis_client(host='localhost', port=6379, db=0, decode_responses=True, local_mode=LOCAL_MODE)

redis_cli.set("forwardSpeed", high_speed)
redis_cli.set("brushSpeed", high_brush_speed)
redis_cli.set('correct', 'false')
redis_cli.set('moveJudge', 'false')
redis_cli.set(PATH_PLANNING_KEY, LEFT_PATH_PLANNING)
redis_cli.set('detectQrcode', 'false')
redis_cli.set('enterGarage', 'false')
redis_cli.set('currentAction', 'idle')
if redis_cli.get(GARAGE_STATE_KEY) is None:
    redis_cli.set(GARAGE_STATE_KEY, GARAGE_STATE_UNKNOWN)
    redis_cli.set(GARAGE_STATE_REASON_KEY, 'startup_unset')
    redis_cli.set(GARAGE_STATE_UPDATED_AT_KEY, int(time.time()))
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
redis_cli.set(LOOP_AUTO_CLEAN_ENABLED_KEY, 'false')
redis_cli.set(LOOP_AUTO_CLEAN_RUNNING_KEY, 'false')
redis_cli.set(LOOP_AUTO_CLEAN_CYCLE_KEY, 0)
redis_cli.set(LOOP_AUTO_CLEAN_STOP_REASON_KEY, '')
redis_cli.set(LOOP_AUTO_CLEAN_UPDATED_AT_KEY, int(time.time()))
redis_cli.delete('battery')
redis_cli.delete('batteryPercent')
redis_cli.delete('batteryRaw')
redis_cli.delete('batteryPercentRaw')
redis_cli.delete('batteryReportAt')
redis_cli.delete('voltage')
redis_cli.delete('packVoltage')
redis_cli.delete('packVoltageReportAt')
redis_cli.set('bootSafeStopAt', int(time.time()))

TASK_SWITCH_LOCK = threading.RLock()
CONFIG_FILE_LOCK = threading.RLock()

LOWER_MACHINE_OPEN_LOCK = threading.RLock()
LOWER_MACHINE_READ_LOCK = threading.RLock()
LOWER_MACHINE_WRITE_LOCK = threading.RLock()
LOWER_MACHINE_RX_LOCK = threading.RLock()
LOWER_MACHINE_RX_BUFFER = bytearray()
LOWER_MACHINE_FRAME_START = 0x7b
LOWER_MACHINE_FRAME_END = 0x7d
LOWER_MACHINE_SHORT_STATUS_LEN = 14
LOWER_MACHINE_RX_BUFFER_LIMIT = 512

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
global_cur_rtk_heading_at = 0
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
loop_auto_clean_thread = None
LOOP_AUTO_CLEAN_LOCK = threading.RLock()
global_last_cte = 0.0
TASK_ORIGIN_TOLERANCE_METERS = 0.02
START_POSITION_TOLERANCE_METERS = TASK_ORIGIN_TOLERANCE_METERS
EDGE_TARGET_TOLERANCE_M = DEFAULT_EDGE_TARGET_TOLERANCE_M
EDGE_RECOVERY_BACK_CM = 5
EDGE_RECOVERY_MAX_ATTEMPTS = 3
EDGE_STOP_ACTION_TARGET = 'target'
EDGE_STOP_ACTION_RECOVER = 'recover'
EDGE_STOP_ACTION_ABORT = 'abort'
global_power_on_guard_sent = False
BATTERY_SMOOTH_ALPHA = 0.18
BATTERY_MAX_DROP_PER_SAMPLE = 0.3
BATTERY_MAX_RISE_PER_SAMPLE = 0.6
BATTERY_SMOOTH_RESET_AFTER_SEC = 60
TURN_RTK_FALLBACK_TOLERANCE_DEG = 2.0
TURN_RTK_FALLBACK_STABLE_COUNT = 1
TURN_RTK_FALLBACK_MIN_WAIT_SEC = 1.0
TURN_RTK_HEADING_MAX_AGE_SEC = 2.0
TURN_RTK_CROSSING_WINDOW_DEG = 3.0


def set_current_action(action_name):
    try:
        redis_cli.set('currentAction', action_name)
    except Exception as e:
        logger.warning("设置currentAction失败: {}".format(str(e)))


def set_garage_state(state, reason):
    state = normalize_garage_state(state)
    try:
        redis_cli.set(GARAGE_STATE_KEY, state)
        redis_cli.set(GARAGE_STATE_REASON_KEY, reason or '')
        redis_cli.set(GARAGE_STATE_UPDATED_AT_KEY, int(time.time()))
    except Exception as e:
        logger.warning("设置garageState失败: {}".format(str(e)))


def get_garage_state():
    try:
        return normalize_garage_state(redis_cli.get(GARAGE_STATE_KEY))
    except Exception:
        return GARAGE_STATE_UNKNOWN


def _garage_state_payload():
    return {
        'garage_state': get_garage_state(),
        'garage_state_reason': _decode_redis_value(redis_cli.get(GARAGE_STATE_REASON_KEY)) or '',
        'garage_state_updated_at': _coerce_int(redis_cli.get(GARAGE_STATE_UPDATED_AT_KEY), 0),
    }


def sync_current_location(lat, lon, heading=None):
    try:
        redis_cli.hset('currentLocation', 'lat', lat)
        redis_cli.hset('currentLocation', 'lon', lon)
        if heading is not None:
            redis_cli.hset('currentLocation', 'heading', heading)
            redis_cli.hset('currentLocation', 'headingAt', time.time())
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


def _normalize_task_name(task_name):
    value = _decode_redis_value(task_name)
    if value is None:
        return ''
    return str(value).strip()


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


def _update_json_file_field(file_name, field_name, field_value):
    if not file_name or not os.path.exists(file_name):
        return False
    try:
        task_obj = util.readConfig(file_name)
        task_obj[field_name] = field_value
        with open(file_name, 'w') as f:
            f.write(json.dumps(task_obj, indent=2))
        return True
    except Exception as e:
        logger.warning("update json field failed: file={}, field={}, error={}".format(file_name, field_name, e))
        return False


def _load_json_config(file_name, default_value=None):
    if default_value is None:
        default_value = {}
    if not os.path.exists(file_name):
        return dict(default_value)
    try:
        with open(file_name, 'r') as fp:
            data = json.load(fp)
        if isinstance(data, dict):
            return data
    except Exception as e:
        logger.warning("load json config failed: file={}, error={}".format(file_name, e))
    return dict(default_value)


def _write_json_config(file_name, data):
    tmp_name = file_name + '.tmp'
    with open(tmp_name, 'w') as fp:
        fp.write(json.dumps(data, indent=2, sort_keys=True))
    if os.path.exists(file_name):
        backup_name = file_name + '.bak_' + time.strftime('%Y%m%d_%H%M%S')
        try:
            os.rename(file_name, backup_name)
        except Exception as e:
            logger.warning("backup json config failed: file={}, error={}".format(file_name, e))
    os.rename(tmp_name, file_name)


def _request_payload():
    return request.get_json(silent=True) or request.form.to_dict() or request.args.to_dict()


def _mask_secret(value):
    if value in (None, ''):
        return ''
    return '******'


def _masked_ntrip_config(data):
    payload = dict(data or {})
    payload['password'] = '******' if payload.get('password') else ''
    return payload


def _normalize_product_model(value):
    value = str(value or '').strip()
    if value and not value.startswith('-'):
        value = '-' + value
    return value


def _normalize_product_id(value):
    return str(value or '').strip()


def _device_no(product_model, product_id):
    return _normalize_product_model(product_model) + _normalize_product_id(product_id)


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


def _set_loop_auto_clean_state(enabled=None, running=None, stop_reason=None, cycle=None):
    try:
        if enabled is not None:
            redis_cli.set(LOOP_AUTO_CLEAN_ENABLED_KEY, 'true' if enabled else 'false')
        if running is not None:
            redis_cli.set(LOOP_AUTO_CLEAN_RUNNING_KEY, 'true' if running else 'false')
        if stop_reason is not None:
            redis_cli.set(LOOP_AUTO_CLEAN_STOP_REASON_KEY, stop_reason)
        if cycle is not None:
            redis_cli.set(LOOP_AUTO_CLEAN_CYCLE_KEY, cycle)
        redis_cli.set(LOOP_AUTO_CLEAN_UPDATED_AT_KEY, int(time.time()))
    except Exception as e:
        logger.warning("设置循环清扫状态失败: {}".format(str(e)))


def _is_loop_auto_clean_enabled():
    return _coerce_bool(redis_cli.get(LOOP_AUTO_CLEAN_ENABLED_KEY), False)


def _disable_loop_auto_clean(reason):
    _set_loop_auto_clean_state(enabled=False, stop_reason=reason or 'stopped')


def _is_loop_low_battery():
    voltage = redis_cli.get("voltage")
    need_return = should_return_to_charge(voltage)
    if need_return:
        logger.warning("循环清扫低电停止: voltage={}, threshold={}".format(
            voltage,
            LOW_BATTERY_RETURN_THRESHOLD
        ))
    return need_return


def _get_loop_auto_clean_status():
    voltage = redis_cli.get("voltage")
    return {
        'enabled': _is_loop_auto_clean_enabled(),
        'running': _coerce_bool(redis_cli.get(LOOP_AUTO_CLEAN_RUNNING_KEY), False),
        'cycle': _coerce_int(redis_cli.get(LOOP_AUTO_CLEAN_CYCLE_KEY), 0),
        'stop_reason': _decode_redis_value(redis_cli.get(LOOP_AUTO_CLEAN_STOP_REASON_KEY)) or '',
        'updated_at': _coerce_int(redis_cli.get(LOOP_AUTO_CLEAN_UPDATED_AT_KEY), 0),
        'voltage': _coerce_float(voltage, None),
        'threshold': LOW_BATTERY_RETURN_THRESHOLD,
        'need_return': should_return_to_charge(voltage),
    }


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


def _normalize_heading_delta(current_heading, target_heading):
    if current_heading is None or target_heading is None:
        return None
    try:
        delta = float(current_heading) - float(target_heading)
    except Exception:
        return None
    return (delta + 180.0) % 360.0 - 180.0


def _heading_delta_crossed_target(previous_delta, current_delta):
    if previous_delta is None or current_delta is None:
        return False
    try:
        previous_delta = float(previous_delta)
        current_delta = float(current_delta)
    except Exception:
        return False
    if abs(previous_delta) <= TURN_RTK_FALLBACK_TOLERANCE_DEG or abs(current_delta) <= TURN_RTK_FALLBACK_TOLERANCE_DEG:
        return True
    if previous_delta * current_delta >= 0:
        return False
    return abs(previous_delta) <= TURN_RTK_CROSSING_WINDOW_DEG and abs(current_delta) <= TURN_RTK_CROSSING_WINDOW_DEG


def _get_current_rtk_heading():
    now_at = time.time()
    redis_heading = _coerce_float(redis_cli.hget('currentLocation', 'heading'), None)
    redis_heading_at = _coerce_float(redis_cli.hget('currentLocation', 'headingAt'), None)
    if redis_heading is not None and redis_heading_at is not None and now_at - redis_heading_at <= TURN_RTK_HEADING_MAX_AGE_SEC:
        return redis_heading

    global_heading = _coerce_float(global_cur_rtk_heading, None)
    if global_heading is not None and global_cur_rtk_heading_at and now_at - global_cur_rtk_heading_at <= TURN_RTK_HEADING_MAX_AGE_SEC:
        return global_heading
    return None


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


def _is_at_task_origin(task_params, tolerance_m=TASK_ORIGIN_TOLERANCE_METERS):
    distance_to_start = _distance_to_task_start(task_params)
    if distance_to_start is None:
        return None, None
    return distance_to_start <= tolerance_m, distance_to_start


def _build_task_origin_check_result(task_params, detail=None, tolerance_m=TASK_ORIGIN_TOLERANCE_METERS):
    detail = dict(detail or {})
    start_lat = _coerce_float(task_params.get('startLat'), None)
    start_lon = _coerce_float(task_params.get('startLon'), None)
    origin_heading = _coerce_float(task_params.get('originHeading'), None)

    detail['taskStartLat'] = start_lat
    detail['taskStartLon'] = start_lon
    detail['taskOriginToleranceM'] = tolerance_m

    if start_lat is None or start_lon is None or origin_heading is None:
        return {
            'success': False,
            'faultState': 'TASK_PARAMS_MISSING',
            'message': '任务原点或航向参数未配置完整，无法校验是否在任务原点',
            'data': detail,
        }

    if global_cur_rtk_lat is None or global_cur_rtk_lon is None:
        return {
            'success': False,
            'faultState': 'RTK_NOT_READY',
            'message': 'RTK 定位未就绪，无法校验是否在任务原点',
            'data': detail,
        }

    at_origin, distance_to_start = _is_at_task_origin(task_params, tolerance_m)
    if distance_to_start is None:
        return {
            'success': False,
            'faultState': 'TASK_ORIGIN_UNKNOWN',
            'message': '无法计算当前位置与任务原点距离，拒绝启动自动清扫',
            'data': detail,
        }

    detail['distanceToTaskOriginM'] = distance_to_start
    detail['distanceToStartM'] = distance_to_start
    detail['isAtTaskOrigin'] = bool(at_origin)
    if not at_origin:
        return {
            'success': False,
            'faultState': 'NOT_AT_TASK_ORIGIN',
            'message': '当前位置距离任务原点 {:.3f} 米，超过允许范围 {:.3f} 米'.format(
                distance_to_start, tolerance_m
            ),
            'data': detail,
        }

    return {
        'success': True,
        'message': '当前位置在任务原点允许范围内',
        'data': detail,
    }


def _build_task_origin_status_fields(task_params=None, tolerance_m=TASK_ORIGIN_TOLERANCE_METERS):
    task_params = task_params or _load_task_params_snapshot()
    start_lat = _coerce_float(task_params.get('startLat'), None)
    start_lon = _coerce_float(task_params.get('startLon'), None)
    distance_to_start = _distance_to_task_start(task_params)
    at_origin = None
    if distance_to_start is not None:
        at_origin = distance_to_start <= tolerance_m

    return {
        'taskOrigin': {
            'lat': start_lat,
            'lon': start_lon,
        },
        'currentLocation': {
            'lat': global_cur_rtk_lat,
            'lon': global_cur_rtk_lon,
            'heading': global_cur_rtk_heading,
        },
        'distanceToTaskOriginM': distance_to_start,
        'taskOriginToleranceM': tolerance_m,
        'isAtTaskOrigin': at_origin,
    }


def _distance_to_current_task_target():
    if not global_cur_taskPoint:
        return None
    target_lat = _coerce_float(global_cur_taskPoint.get('endLat'), None)
    target_lon = _coerce_float(global_cur_taskPoint.get('endLon'), None)
    if target_lat is None or target_lon is None:
        return None
    if global_cur_rtk_lat is None or global_cur_rtk_lon is None:
        return None
    try:
        distance, _ = util.get_distance_angle(global_cur_rtk_lat, global_cur_rtk_lon, target_lat, target_lon)
        return round(float(distance), 3)
    except Exception:
        return None


def _handle_edge_stop_for_current_task(source):
    distance_to_target = _distance_to_current_task_target()
    try:
        redis_cli.set('edgeDistanceToTargetM', '' if distance_to_target is None else distance_to_target)
    except Exception:
        pass

    if should_accept_edge_stop(distance_to_target, EDGE_TARGET_TOLERANCE_M):
        logger.warn(
            "edge accepted near task target: source={}, distanceToTarget={}m, tolerance={}m".format(
                source,
                distance_to_target,
                EDGE_TARGET_TOLERANCE_M
            )
        )
        return EDGE_STOP_ACTION_TARGET

    if should_recover_from_edge_stop(distance_to_target, EDGE_TARGET_TOLERANCE_M):
        logger.warn(
            "edge abnormal but target is still far; recover and continue: source={}, distanceToTarget={}m, tolerance={}m".format(
                source,
                distance_to_target,
                EDGE_TARGET_TOLERANCE_M
            )
        )
        try:
            redis_cli.set('edgeStopReason', 'far_from_task_target_recover')
        except Exception:
            pass
        return EDGE_STOP_ACTION_RECOVER

    reason = 'target_distance_unavailable' if distance_to_target is None else 'far_from_task_target'
    logger.warn(
        "edge rejected; stop current task: source={}, reason={}, distanceToTarget={}m, tolerance={}m".format(
            source,
            reason,
            distance_to_target,
            EDGE_TARGET_TOLERANCE_M
        )
    )
    try:
        redis_cli.set('edgeStopReason', reason)
    except Exception:
        pass
    doParking()
    return EDGE_STOP_ACTION_ABORT


def _recover_from_abnormal_edge(source):
    logger.warn("edge recovery: source={}, brake and back {}cm".format(source, EDGE_RECOVERY_BACK_CM))
    sendBraking()
    moveBack(ser, EDGE_RECOVERY_BACK_CM)
    reset_odometer(ser)
    return True


def _build_runtime_detail(extra=None):
    task_params = _load_task_params_snapshot()
    detail = _load_runtime_detail()
    # Runtime detail builder should always return a plain detail object.
    detail.update({
        'batteryReportAt': _coerce_int(redis_cli.get('batteryReportAt'), None),
        'batteryPercent': _coerce_float(redis_cli.get('batteryPercent'), None),
        'batteryPercentRaw': _coerce_float(redis_cli.get('batteryPercentRaw'), None),
        'packVoltage': _coerce_float(redis_cli.get('packVoltage'), None),
        'packVoltageReportAt': _coerce_int(redis_cli.get('packVoltageReportAt'), None),
        'hardwareState': _coerce_int(redis_cli.get('hardwareState'), None),
        'hardwareReportAt': _get_hardware_report_at(),
        'hardwareReportAgeSec': _get_hardware_report_age_sec(),
        'distanceToStartM': _distance_to_task_start(task_params),
        'distanceToTaskOriginM': _distance_to_task_start(task_params),
        'startToleranceM': TASK_ORIGIN_TOLERANCE_METERS,
        'taskOriginToleranceM': TASK_ORIGIN_TOLERANCE_METERS,
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

    current_task_name = _normalize_task_name(redis_cli.get('currentTaskName'))
    if not current_task_name:
        return {
            'success': False,
            'faultState': 'CURRENT_TASK_NOT_SET',
            'message': '未设置当前任务，请先选择任务并设为当前任务',
            'data': detail,
        }

    try:
        task_obj = util.readConfig("config.json")
    except Exception as e:
        logger.error("读取config.json失败: {}".format(str(e)))
        return {
            'success': False,
            'faultState': 'CURRENT_TASK_CONFIG_MISSING',
            'message': '当前任务配置不存在或不可读',
            'data': detail,
        }

    config_task_name = _normalize_task_name(task_obj.get('taskName'))
    if config_task_name != current_task_name:
        detail['currentTaskName'] = current_task_name
        detail['configTaskName'] = config_task_name
        return {
            'success': False,
            'faultState': 'CURRENT_TASK_MISMATCH',
            'message': '当前任务与执行配置不一致，请重新设置当前任务',
            'data': detail,
        }

    detail.update({
        'batteryReportAt': _coerce_int(redis_cli.get('batteryReportAt'), None),
        'batteryPercent': _coerce_float(redis_cli.get('batteryPercent'), None),
        'batteryPercentRaw': _coerce_float(redis_cli.get('batteryPercentRaw'), None),
        'packVoltage': _coerce_float(redis_cli.get('packVoltage'), None),
        'packVoltageReportAt': _coerce_int(redis_cli.get('packVoltageReportAt'), None),
        'hardwareState': _coerce_int(redis_cli.get('hardwareState'), None),
        'hardwareReportAt': _get_hardware_report_at(),
        'hardwareReportAgeSec': _get_hardware_report_age_sec(),
        'distanceToStartM': _distance_to_task_start(task_params),
        'distanceToTaskOriginM': _distance_to_task_start(task_params),
        'startToleranceM': TASK_ORIGIN_TOLERANCE_METERS,
        'taskOriginToleranceM': TASK_ORIGIN_TOLERANCE_METERS,
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


def _frame_hex(data):
    if data is None:
        return ''
    try:
        return binascii.b2a_hex(data)
    except Exception:
        parts = []
        for item in data:
            value = _frame_byte_to_int(item)
            if value is None:
                value = 0
            parts.append("{:02x}".format(value))
        return ''.join(parts)


def _serial_is_open(port):
    if port is None:
        return False
    state = getattr(port, 'is_open', None)
    if state is not None:
        return bool(state)
    if hasattr(port, 'isOpen'):
        try:
            return bool(port.isOpen())
        except Exception:
            return False
    return False


def _serial_in_waiting(port):
    try:
        waiting = getattr(port, 'in_waiting', None)
        if waiting is not None:
            return int(waiting)
        if hasattr(port, 'inWaiting'):
            return int(port.inWaiting())
    except Exception:
        return 0
    return 0


def _reset_lower_machine_serial(reason=None):
    global ser
    with LOWER_MACHINE_OPEN_LOCK:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        ser = None
    if reason is not None:
        logger.warning("reset lower-machine serial: {}".format(reason), exc_info=True)


def _get_lower_machine_serial():
    global ser
    global global_status
    if sys.platform.startswith('win'):
        return ser
    with LOWER_MACHINE_OPEN_LOCK:
        try:
            if not _serial_is_open(ser):
                if ser is not None:
                    try:
                        ser.close()
                    except Exception:
                        pass
                ser = serial.Serial(xwj_port, 115200, timeout=0.05)
            try:
                ser.timeout = 0.05
            except Exception:
                pass
            return ser
        except serial.serialutil.SerialException as exc:
            global_status = "fail open COM"
            ser = None
            logger.error("fail open lower-machine serial {}: {}".format(xwj_port, exc), exc_info=True)
            return None


def _find_byte_in_rx_buffer(value, start_index=0):
    index = start_index
    with LOWER_MACHINE_RX_LOCK:
        while index < len(LOWER_MACHINE_RX_BUFFER):
            if LOWER_MACHINE_RX_BUFFER[index] == value:
                return index
            index += 1
    return -1


def _trim_lower_machine_rx_buffer_locked():
    if len(LOWER_MACHINE_RX_BUFFER) <= LOWER_MACHINE_RX_BUFFER_LIMIT:
        return
    keep_start = -1
    index = len(LOWER_MACHINE_RX_BUFFER) - 1
    while index >= 0:
        if LOWER_MACHINE_RX_BUFFER[index] == LOWER_MACHINE_FRAME_START:
            keep_start = index
            break
        index -= 1
    if keep_start > 0:
        del LOWER_MACHINE_RX_BUFFER[:keep_start]
    if len(LOWER_MACHINE_RX_BUFFER) > LOWER_MACHINE_RX_BUFFER_LIMIT:
        del LOWER_MACHINE_RX_BUFFER[:-LOWER_MACHINE_RX_BUFFER_LIMIT]


def _append_lower_machine_rx_data(data):
    if not data:
        return
    with LOWER_MACHINE_RX_LOCK:
        LOWER_MACHINE_RX_BUFFER.extend(bytearray(data))
        _trim_lower_machine_rx_buffer_locked()


def _pop_lower_machine_rx_frame_locked():
    while LOWER_MACHINE_RX_BUFFER and LOWER_MACHINE_RX_BUFFER[0] != LOWER_MACHINE_FRAME_START:
        del LOWER_MACHINE_RX_BUFFER[0]

    if not LOWER_MACHINE_RX_BUFFER:
        return None

    next_start_index = -1
    end_index = -1
    index = 1
    while index < len(LOWER_MACHINE_RX_BUFFER):
        byte_value = LOWER_MACHINE_RX_BUFFER[index]
        if byte_value == LOWER_MACHINE_FRAME_START and next_start_index < 0:
            next_start_index = index
        if byte_value == LOWER_MACHINE_FRAME_END:
            end_index = index
            break
        index += 1

    if end_index >= 0 and (next_start_index < 0 or end_index < next_start_index):
        frame = bytearray(LOWER_MACHINE_RX_BUFFER[:end_index + 1])
        del LOWER_MACHINE_RX_BUFFER[:end_index + 1]
        return frame

    if next_start_index > 0 and next_start_index < LOWER_MACHINE_SHORT_STATUS_LEN:
        frame = bytearray(LOWER_MACHINE_RX_BUFFER[:next_start_index])
        del LOWER_MACHINE_RX_BUFFER[:next_start_index]
        return frame

    if len(LOWER_MACHINE_RX_BUFFER) >= LOWER_MACHINE_SHORT_STATUS_LEN:
        frame_len = LOWER_MACHINE_SHORT_STATUS_LEN
        if next_start_index > 0:
            frame_len = min(frame_len, next_start_index)
        frame = bytearray(LOWER_MACHINE_RX_BUFFER[:frame_len])
        del LOWER_MACHINE_RX_BUFFER[:frame_len]
        return frame

    return None


def _apply_lower_machine_status_frame(data, source):
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
    global global_get_rotateFinish

    if not data:
        return False

    first_byte = _frame_byte_to_int(data[0])
    if first_byte != LOWER_MACHINE_FRAME_START:
        logger.warn("{} ignore unsynced lower-machine frame, raw={}".format(source, _frame_hex(data)))
        return False

    frame_len = len(data)
    report_at = int(time.time())

    def byte_at(index):
        if frame_len <= index:
            return None
        return _frame_byte_to_int(data[index])

    if frame_len < 14:
        logger.info("{} ignore short lower-machine frame, len={}, raw={}".format(source, frame_len, _frame_hex(data)))
        return False

    if frame_len < 20:
        move_finish = byte_at(12)
        rotate_finish = byte_at(13)
        if move_finish == 0xbb:
            global_get_moveFinish = 1
        if rotate_finish == 0xbb:
            global_get_rotateFinish = 1
        if move_finish == 0xbb or rotate_finish == 0xbb:
            logger.warn(
                "{} parsed short finish frame, len={}, raw={}, moveFinish={}, rotateFinish={}".format(
                    source,
                    frame_len,
                    _frame_hex(data),
                    global_get_moveFinish,
                    global_get_rotateFinish
                )
            )
            return True
        logger.info("{} ignore short non-finish frame, len={}, raw={}".format(source, frame_len, _frame_hex(data)))
        return False

    redis_cli.set("hardwareReportAt", report_at)

    status = byte_at(1)
    if status is not None:
        global_get_status = status
        
        current_g_state = get_garage_state()
        # 1. 硬件强装充电
        if status == 5 and current_g_state != GARAGE_STATE_DOCKED_BY_COMMAND:
            logger.warn("嗅探到下位机硬件处于充电状态(5)，自动恢复 garageState 为 docked_by_command")
            set_garage_state(GARAGE_STATE_DOCKED_BY_COMMAND, 'auto_recovered_by_hardware_status')
            
        # 2. 如果当前状态处于未知
        elif current_g_state == GARAGE_STATE_UNKNOWN:
            if global_cur_rtk_lat is None:
                # 按照业务确认：没有信号默认视为在舱内（光伏板遮挡）。如果误判，可通过人工页面强制修改
                logger.warn("无 RTK 信号，推断为在光伏舱内，恢复为 docked_by_command (若误判请人工确认)")
                set_garage_state(GARAGE_STATE_DOCKED_BY_COMMAND, 'auto_recovered_by_no_signal')
            else:
                logger.warn("嗅探到有 RTK 固定解，推断为舱外 outside")
                set_garage_state(GARAGE_STATE_OUTSIDE, 'auto_recovered_by_rtk_fix')
                
        # 3. 如果已经被判定为舱内(或未判定)，但突然有了 RTK 信号，强制转为舱外 (解决开机搜星慢导致的误判)
        elif current_g_state == GARAGE_STATE_DOCKED_BY_COMMAND and global_cur_rtk_lat is not None:
            logger.warn("在判定为舱内的状态下获取到了 RTK 信号，自动修正推断为舱外 outside")
            set_garage_state(GARAGE_STATE_OUTSIDE, 'auto_corrected_by_rtk_fix')

    power_on = byte_at(2)
    if power_on is not None:
        previous_power_on_state = _get_power_on_state()
        global_get_powerOn = power_on
        redis_cli.set("powerOnState", global_get_powerOn)
        _maybe_brake_on_power_enable(previous_power_on_state, global_get_powerOn)

    hardware_state = byte_at(3)
    if hardware_state is not None:
        global_get_HWstatus = hardware_state
        redis_cli.set("hardwareState", global_get_HWstatus)

    if frame_len > 5:
        x_speed = _frame_u16_to_int(data, 4)
        if x_speed is not None:
            global_get_XSpeed = x_speed

    if frame_len > 7:
        z_speed = _frame_u16_to_int(data, 6)
        if z_speed is not None:
            global_get_ZSpeed = z_speed

    brush_speed = byte_at(8)
    if brush_speed is not None:
        global_get_brushSpeed = brush_speed

    edge_status = byte_at(9)
    if edge_status is not None:
        if edge_status == 0:
            global_get_edge = 1
        elif edge_status == 0xff:
            logger.warn("lower-machine edge alarm, edge=0 raw={}".format(_frame_hex(data)))
            redis_cli.set("ultraSonic", "true")
            global_get_edge = 0
        else:
            global_get_edge = 0

    voltage = byte_at(10)
    if voltage is not None:
        global_get_voltage = voltage
        _cache_battery_percent(global_get_voltage, report_at)

    air = byte_at(11)
    if air is not None:
        global_get_air = air

    move_finish = byte_at(12)
    if move_finish is not None:
        global_get_moveFinish = 1 if move_finish == 0xbb else 0

    rotate_finish = byte_at(13)
    if rotate_finish is not None:
        global_get_rotateFinish = 1 if rotate_finish == 0xbb else 0

    if frame_len > 15:
        pack_voltage_raw = _frame_u16_to_int(data, 14)
        if pack_voltage_raw is not None:
            redis_cli.set("packVoltage", round(pack_voltage_raw * 0.01, 1))
            redis_cli.set("packVoltageReportAt", report_at)

    if frame_len > 17:
        angle = _frame_u16_to_int(data, 16)
        if angle is not None:
            redis_cli.set("angle", angle)

    if frame_len > 19:
        odometer = _frame_u16_to_int(data, 18)
        if odometer is not None:
            redis_cli.set("odometer", odometer)

    if move_finish == 0xbb or rotate_finish == 0xbb:
        logger.warn(
            "{} parsed finish frame, len={}, raw={}, moveFinish={}, rotateFinish={}".format(
                source,
                frame_len,
                _frame_hex(data),
                global_get_moveFinish,
                global_get_rotateFinish
            )
        )

    return True


def _drain_lower_machine_rx_buffer(source):
    parsed = False
    while True:
        with LOWER_MACHINE_RX_LOCK:
            frame = _pop_lower_machine_rx_frame_locked()
        if frame is None:
            break
        if _apply_lower_machine_status_frame(frame, source):
            parsed = True
    return parsed


def _read_lower_machine_status_frame(source, wait_seconds=0.25):
    if _drain_lower_machine_rx_buffer(source):
        return True

    deadline = time.time() + wait_seconds
    while time.time() < deadline:
        port = _get_lower_machine_serial()
        if port is None:
            time.sleep(0.05)
            return False

        try:
            with LOWER_MACHINE_READ_LOCK:
                waiting = _serial_in_waiting(port)
                read_len = waiting if waiting > 0 else 1
                read_len = min(max(read_len, 1), LOWER_MACHINE_RX_BUFFER_LIMIT)
                data = port.read(read_len)
        except serial.serialutil.SerialException as exc:
            _reset_lower_machine_serial(exc)
            time.sleep(0.05)
            continue
        except Exception as exc:
            logger.warning("{} read lower-machine serial failed: {}".format(source, exc), exc_info=True)
            time.sleep(0.02)
            continue

        if data:
            _append_lower_machine_rx_data(data)
            if _drain_lower_machine_rx_buffer(source):
                return True
        else:
            time.sleep(0.02)

    return False


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
    control_state = _decode_redis_value(redis_cli.get('controlState'))
    if control_state and control_state not in ('DISABLED', 'UNKNOWN'):
        return control_state
    if _decode_redis_value(redis_cli.get('mission')) == 'working':
        return 'RUNNING'
    if _coerce_bool(redis_cli.get('parking'), False):
        return 'STOPPED'
    return 'IDLE'


def _derive_health_state():
    raw_fault_state = _decode_redis_value(redis_cli.get('faultState'))
    if _derive_fault_state():
        return 'WARN'
    health_state = _decode_redis_value(redis_cli.get('healthState'))
    if _is_ignored_enable_fault_state(raw_fault_state):
        return 'OK'
    if health_state:
        return health_state
    return 'OK'


def _is_ignored_enable_fault_state(fault_state):
    return (fault_state or '').strip().upper() in ('LOWER_MACHINE_DISABLED', 'LOWER_MACHINE_STATUS_UNKNOWN')


def _derive_fault_state():
    fault_state = _decode_redis_value(redis_cli.get('faultState'))
    if fault_state and not _is_ignored_enable_fault_state(fault_state):
        return fault_state
    return ''


def _derive_mission_state(control_state):
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
    task_origin_status = _build_task_origin_status_fields(task_params)
    local_x, local_y = _compute_local_xy_cm(lat, lon, task_params)
    control_state = _derive_control_state()
    mission_state = _derive_mission_state(control_state)
    fault_state = _derive_fault_state()
    current_action = _decode_redis_value(redis_cli.get('currentAction')) or ('parking' if _coerce_bool(redis_cli.get('parking'), False) else 'idle')
    garage_state = get_garage_state()
    battery_percent = _clamp_percent(redis_cli.get('batteryPercent'))
    battery_percent_raw = _clamp_percent(redis_cli.get('batteryPercentRaw'))
    loop_auto_clean = _get_loop_auto_clean_status()

    detail = _build_runtime_detail({
        'lastCommandMessage': _decode_redis_value(redis_cli.get('lastCommandMessage')) or '',
        'startCheckReady': _coerce_bool(redis_cli.get('startCheckReady'), False),
        'startCheckReason': _decode_redis_value(redis_cli.get('startCheckReason')) or '',
        'garageStateReason': _decode_redis_value(redis_cli.get(GARAGE_STATE_REASON_KEY)) or '',
    })

    payload = {
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
        'garage_state': garage_state,
        'garage_state_updated_at': _coerce_int(redis_cli.get(GARAGE_STATE_UPDATED_AT_KEY), 0),
        'loop_auto_clean': loop_auto_clean,
        'loopAutoClean': loop_auto_clean,
        'supported_actions': ['auto_drive', 'go_on', 'stop', 'parking', 'return_to_point', 'get_status', 'get_task_path'],
        'supported_params': ['taskName', 'speed', 'tracking', 'path'],
        'supported_status_fields': ['control_state', 'health_state', 'fault_state', 'detail', 'mission_state', 'garage_state', 'loop_auto_clean'],
        'detail': detail,
        'timestamp': int(time.time()),
    }
    payload.update(task_origin_status)
    return payload


def _validate_auto_drive_request_legacy():
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

    task_items = task_obj.get('taskList') if isinstance(task_obj, dict) else None
    if not isinstance(task_items, list) or len(task_items) == 0:
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

    current_task_name = _normalize_task_name(redis_cli.get('currentTaskName'))
    if not current_task_name:
        return {
            'success': False,
            'faultState': 'CURRENT_TASK_NOT_SET',
            'message': '未设置当前任务，请先设置当前任务',
            'data': detail,
        }

    try:
        task_obj = util.readConfig("config.json")
    except Exception as e:
        logger.error("读取config.json失败: {}".format(str(e)))
        return {
            'success': False,
            'faultState': 'CURRENT_TASK_CONFIG_MISSING',
            'message': '当前任务配置不存在或不可读',
            'data': detail,
        }

    config_task_name = _normalize_task_name(task_obj.get('taskName'))
    if config_task_name != current_task_name:
        detail['currentTaskName'] = current_task_name
        detail['configTaskName'] = config_task_name
        return {
            'success': False,
            'faultState': 'CURRENT_TASK_MISMATCH',
            'message': '当前任务与执行配置不一致，请重新设置当前任务',
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

    task_items = task_obj.get('taskList') if isinstance(task_obj, dict) else None
    if not isinstance(task_items, list) or len(task_items) == 0:
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

def _validate_auto_drive_request_legacy():
    task_params = _load_task_params_snapshot()
    detail = _build_runtime_detail()

    if _decode_redis_value(redis_cli.get('mission')) == 'working':
        return {
            'success': False,
            'faultState': 'ALREADY_RUNNING',
            'message': '小车当前正在执行任务，请勿重复启动',
            'data': detail,
        }

    return _build_task_origin_check_result(task_params, detail)


def _validate_auto_drive_request():
    task_params = _load_task_params_snapshot()
    detail = _build_runtime_detail()

    if _decode_redis_value(redis_cli.get('mission')) == 'working':
        return {
            'success': False,
            'faultState': 'ALREADY_RUNNING',
            'message': '小车当前正在执行任务，请勿重复启动',
            'data': detail,
        }

    current_task_name = _normalize_task_name(redis_cli.get('currentTaskName'))
    if not current_task_name:
        return {
            'success': False,
            'faultState': 'CURRENT_TASK_NOT_SET',
            'message': '未设置当前任务，请先设置当前任务',
            'data': detail,
        }

    try:
        task_obj = util.readConfig("config.json")
    except Exception as e:
        logger.error("读取config.json失败: {}".format(str(e)))
        return {
            'success': False,
            'faultState': 'CURRENT_TASK_CONFIG_MISSING',
            'message': '当前任务配置不存在或不可读',
            'data': detail,
        }

    config_task_name = _normalize_task_name(task_obj.get('taskName'))
    if config_task_name != current_task_name:
        detail['currentTaskName'] = current_task_name
        detail['configTaskName'] = config_task_name
        return {
            'success': False,
            'faultState': 'CURRENT_TASK_MISMATCH',
            'message': '当前任务与执行配置不一致，请重新设置当前任务',
            'data': detail,
        }

    origin_check = _build_task_origin_check_result(task_params, detail)
    if not origin_check.get('success'):
        return origin_check
    detail = origin_check.get('data', detail)

    task_items = task_obj.get('taskList') if isinstance(task_obj, dict) else None
    if not isinstance(task_items, list) or len(task_items) == 0:
        return {
            'success': False,
            'faultState': 'TASK_PATH_EMPTY',
            'message': '当前任务没有可执行路径，请先生成并设置任务',
            'data': detail,
        }

    return {
        'success': True,
        'message': '启动条件通过',
        'data': detail,
    }


def _can_access_serial_port(port):
    return bool(port) and os.path.exists(port) and os.access(port, os.R_OK | os.W_OK)


def _resolve_lower_machine_port(_rtk_port):
    env_port = os.getenv("CLEANER_LOWER_MACHINE_PORT")
    preferred = env_port or "/dev/ttyTHS1"

    if _can_access_serial_port(preferred):
        logger.warn("下位机串口已固定使用：{}".format(preferred))
        return preferred

    if os.path.exists(preferred):
        logger.error("下位机串口固定为 {}，但当前进程无读写权限".format(preferred))
    else:
        logger.error("下位机串口固定为 {}，但设备节点不存在".format(preferred))

    # Keep returning the fixed port; do not fallback to ttyACM0/ttyACM4.
    return preferred


# 下位机端口（固定 ttyTHS1）
xwj_port = "/dev/ttyTHS1"
# rtk端口
rtk_port = util.findPort("$GN")
logger.warn(rtk_port)
xwj_port = _resolve_lower_machine_port(rtk_port)
logger.warn("下位机串口：{}".format(xwj_port))

def globalDataSet(data):
    return _apply_lower_machine_status_frame(data, "globalDataSet")
    if not data or len(data) < 20:
        logger.warn("globalDataSet() ignore short frame, len=%s raw=%s", len(data) if data else 0, binascii.b2a_hex(data or ''))
        return
    if binascii.b2a_hex(data[0]) != '7b':
        logger.warn("globalDataSet() ignore unsynced frame, raw=%s", binascii.b2a_hex(data))
        return
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
                logger.warn("收到边缘传感器报警帧，edge=0 raw=%s", binascii.b2a_hex(data))
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
    redis_cli.set('moveJudge', 'true')
    try:
        _read_lower_machine_status_frame("getEdge", 0.25)
    finally:
        redis_cli.set('moveJudge', 'false')
    return str(global_get_edge)
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
                if len(data) < 20:
                    logger.warn("getEdge() ignore short synced frame, len=%s raw=%s", len(data), binascii.b2a_hex(data))
                    continue
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
    redis_cli.set('moveJudge', 'true')
    try:
        _read_lower_machine_status_frame("getDistanceArrive", 0.25)
    finally:
        redis_cli.set('moveJudge', 'false')
    return str(global_get_moveFinish)
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
                if len(data) < 20:
                    logger.warn("getDistanceArrive() ignore short synced frame, len=%s raw=%s", len(data), binascii.b2a_hex(data))
                    continue
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
    redis_cli.set('moveJudge', 'true')
    try:
        _read_lower_machine_status_frame("getRotateArrive", 0.05)
    finally:
        redis_cli.set('moveJudge', 'false')
    return str(global_get_rotateFinish)
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
                if len(data) < 20:
                    logger.warn("getRotateArrive() ignore short synced frame, len=%s raw=%s", len(data), binascii.b2a_hex(data))
                    continue

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


CAMERA_SOURCES = ['/dev/video0', '/dev/video1', '/dev/video2', 0, 1, 2]


def _open_camera_capture():
    for source in CAMERA_SOURCES:
        if isinstance(source, str) and not os.path.exists(source):
            continue
        candidate = cv2.VideoCapture(source)
        if candidate.isOpened():
            logger.warn("Camera source {} is available".format(source))
            return candidate
        try:
            candidate.release()
        except Exception:
            pass
    logger.warning("no camera source is available")
    return None


cap = None if LOCAL_MODE else _open_camera_capture()
camera_http_lock = threading.RLock()
latest_camera_frame = None
latest_camera_frame_at = 0.0
latest_camera_frame_lock = threading.Lock()
last_camera_open_attempt_at = 0.0
GUIDANCE_CROP_TOP = 70
GUIDANCE_CROP_BOTTOM = 430
GUIDANCE_CROP_LEFT = 60
GUIDANCE_CROP_RIGHT = 580
ENTER_GARAGE_FIXED_SPEED = 90
ENTER_GARAGE_MAX_WAIT_SECONDS = 45
VISUAL_ENTER_GARAGE_SPEED = ENTER_GARAGE_FIXED_SPEED
VISUAL_ENTER_GARAGE_FINAL_LENGTH = 80
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


@app.route("/vehicle/getNtripConfig", methods=['GET'])
def getNtripConfig():
    with CONFIG_FILE_LOCK:
        config = _load_json_config('ntrip_config.json')
    return jsonify({
        'success': True,
        'code': 200,
        'data': _masked_ntrip_config(config),
    })


@app.route("/vehicle/updateNtripConfig", methods=['POST'])
def updateNtripConfig():
    payload = _request_payload()
    allowed_fields = (
        'enabled',
        'host',
        'port',
        'mountpoint',
        'username',
        'password',
        'gga_interval_seconds',
        'connect_timeout_seconds',
        'reconnect_interval_seconds',
    )
    with CONFIG_FILE_LOCK:
        config = _load_json_config('ntrip_config.json')
        for field in allowed_fields:
            if field in payload:
                if field == 'enabled':
                    config[field] = _coerce_bool(payload.get(field), False)
                elif field == 'port':
                    config[field] = _coerce_int(payload.get(field), 0)
                elif field in ('gga_interval_seconds', 'connect_timeout_seconds', 'reconnect_interval_seconds'):
                    config[field] = _coerce_float(payload.get(field), 5.0)
                else:
                    config[field] = str(payload.get(field) or '').strip()
        _write_json_config('ntrip_config.json', config)
    reset_shared_runtime(logger)
    return jsonify({
        'success': True,
        'code': 200,
        'message': 'ntrip config updated',
        'data': _masked_ntrip_config(config),
    })


@app.route("/vehicle/getDeviceConfig", methods=['GET'])
def getDeviceConfig():
    with CONFIG_FILE_LOCK:
        config = _load_json_config('mqtt_config.json')
    mqtt_config = config.get('mqtt', {})
    topics = config.get('topics', {})
    product_model = mqtt_config.get('product_model', '')
    product_id = mqtt_config.get('product_id', '')
    return jsonify({
        'success': True,
        'code': 200,
        'data': {
            'product_model': product_model,
            'product_id': product_id,
            'device_no': _device_no(product_model, product_id),
            'subscribe': topics.get('subscribe', ''),
            'publish': topics.get('publish', ''),
        },
    })


@app.route("/vehicle/updateDeviceConfig", methods=['POST'])
def updateDeviceConfig():
    payload = _request_payload()
    with CONFIG_FILE_LOCK:
        config = _load_json_config('mqtt_config.json')
        mqtt_config = config.setdefault('mqtt', {})
        product_model = _normalize_product_model(payload.get('product_model', mqtt_config.get('product_model', '')))
        product_id = _normalize_product_id(payload.get('product_id', mqtt_config.get('product_id', '')))
        mqtt_config['product_model'] = product_model
        mqtt_config['product_id'] = product_id
        device_no = _device_no(product_model, product_id)
        topics = config.setdefault('topics', {})
        topics['subscribe'] = 'RAILCAR/S/' + device_no
        topics['publish'] = 'RAILCAR/R/' + device_no
        _write_json_config('mqtt_config.json', config)
    return jsonify({
        'success': True,
        'code': 200,
        'message': 'device config updated',
        'restart_required': True,
        'data': {
            'product_model': product_model,
            'product_id': product_id,
            'device_no': device_no,
            'subscribe': topics.get('subscribe', ''),
            'publish': topics.get('publish', ''),
        },
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
def _get_task_enter_garage_length():
    taskParams = redis_cli.hgetall("taskParams")
    try:
        return int(float(taskParams.get('startToChargingPilePointLength') or 0))
    except (TypeError, ValueError):
        return 0


def _run_fixed_enter_garage(travel_length_cm, max_wait_seconds=ENTER_GARAGE_MAX_WAIT_SECONDS,
                            forward_speed=ENTER_GARAGE_FIXED_SPEED):
    global global_get_moveFinish
    try:
        travel_length_cm = int(float(travel_length_cm))
    except (TypeError, ValueError):
        travel_length_cm = 0

    if travel_length_cm <= 0:
        logger.warn("入舱点到充电桩距离未配置，无法固定距离进舱: {}".format(travel_length_cm))
        return False

    redis_cli.set("enterGarage", "false")
    redis_cli.set("detectQrcode", "false")
    redis_cli.set("correct", "false")
    redis_cli.set('action', 'true')
    reset_odometer(ser)
    global_get_moveFinish = 0

    preBuildCommand()
    reSetStatus(ser)
    setBrushSpeed(0)
    setStatus(2)
    setPowerOn(1)
    setHWstatus(0, 0, 0, 0, 0)
    setXSpeed(forward_speed)
    setZSpeed(0)
    setDistance(travel_length_cm)
    command[17] = tem_listener(command, 17)
    logger.warn("固定距离低速进舱: distance={}cm, speed={}".format(travel_length_cm, forward_speed))
    duplicateWriteCmd(ser, command)

    wait_start_time = time.time()
    distance_arrived = False
    while redis_cli.get('action') == 'true':
        if getDistanceArrive() == "1":
            distance_arrived = True
            logger.warn("固定距离进舱达到 {}cm".format(travel_length_cm))
            break
        if max_wait_seconds and (time.time() - wait_start_time) >= max_wait_seconds:
            logger.warn("固定距离进舱等待超时({}s)，主动结束".format(max_wait_seconds))
            break
        time.sleep(0.25)

    sendBraking()
    redis_cli.set("enterGarage", "false")
    redis_cli.set("correct", "false")
    redis_cli.set('action', 'false')
    return distance_arrived


def _run_visual_enter_garage(max_wait_seconds=None, travel_length_cm=None, forward_speed=VISUAL_ENTER_GARAGE_SPEED):
    global global_get_moveFinish
    redis_cli.set("enterGarage", "true")

    redis_cli.set("detectQrcode", "false")

    redis_cli.set("correct", "true")

    redis_cli.set('action', 'true')

    if travel_length_cm is not None:
        reset_odometer(ser)
        global_get_moveFinish = 0

    preBuildCommand()

    reSetStatus(ser)

    setBrushSpeed(0)

    setStatus(1)

    setPowerOn(1)

    setHWstatus(0, 0, 0, 0, 0)

    setXSpeed(forward_speed)
    setZSpeed(0)
    if travel_length_cm is None:
        setDistance(0)
    else:
        travel_length_cm = max(int(travel_length_cm), 0)
        setDistance(travel_length_cm)
        logger.warn("视觉入舱直行距离限制 {}cm，速度={}".format(travel_length_cm, forward_speed))

    command[17] = tem_listener(command, 17)

    duplicateWriteCmd(ser, command)

    distance_arrived = False
    wait_start_time = time.time()
    while redis_cli.get('enterGarage') == 'true':
        if travel_length_cm is not None and getDistanceArrive() == "1":
            distance_arrived = True
            logger.warn("视觉入舱直行达到 {}cm，结束本次入舱".format(travel_length_cm))
            break
        if max_wait_seconds and (time.time() - wait_start_time) >= max_wait_seconds:
            logger.warn("视觉入舱等待超时({}s)，主动结束本次入舱".format(max_wait_seconds))
            break
        time.sleep(0.25)

    sendBraking()

    redis_cli.set("enterGarage", "false")

    redis_cli.set("correct", "false")

    redis_cli.set('action', 'false')

    return redis_cli.get("detectQrcode") == "true"


def enter_garage_task():
    redis_cli.set('mission', 'working')
    travel_length_cm = _get_task_enter_garage_length()
    arrived = _run_fixed_enter_garage(travel_length_cm)
    redis_cli.set('mission', 'complete')
    logger.warn("手动固定距离进舱结束，distance={}cm, arrived={}".format(
        travel_length_cm,
        "true" if arrived else "false"
    ))


def writeCmd(ser, command):
    port = _get_lower_machine_serial()
    if port is None:
        logger.warning('lower-machine serial is not ready')
        return
    try:
        with LOWER_MACHINE_WRITE_LOCK:
            port.write(command)
    except serial.serialutil.SerialException as exc:
        _reset_lower_machine_serial(exc)
    except Exception as exc:
        logger.warning("write lower-machine command failed: {}".format(exc), exc_info=True)
    return
    if ser is None:
        logger.warning('??????????????')
        return
    if ser.is_open:
        pass
    else:
        ser = serial.Serial(xwj_port, 115200, timeout=0.5)
    ser.write(command)


def duplicateWriteCmd(ser, command):
    for i in range(5):
        writeCmd(ser, command)
    port = _get_lower_machine_serial()
    if port is not None:
        try:
            with LOWER_MACHINE_WRITE_LOCK:
                port.flushOutput()
        except Exception as exc:
            logger.warning("flush lower-machine serial failed: {}".format(exc), exc_info=True)
    return
    if ser is None:
        logger.warning('????????????????')
        return
    for i in range(5):
        writeCmd(ser, command)
    ser.flushOutput()


def _remember_camera_frame(frame):
    global latest_camera_frame
    global latest_camera_frame_at
    if frame is None:
        return
    try:
        with latest_camera_frame_lock:
            latest_camera_frame = frame.copy()
            latest_camera_frame_at = time.time()
    except Exception as exc:
        logger.warning("cache camera frame failed: {}".format(exc))


def _get_recent_camera_frame(max_age_seconds=2.0):
    with latest_camera_frame_lock:
        if latest_camera_frame is None:
            return None
        if time.time() - latest_camera_frame_at > max_age_seconds:
            return None
        return latest_camera_frame.copy()


def _read_camera_frame_for_http():
    global cap
    frame = _get_recent_camera_frame()
    if frame is not None:
        return frame

    with camera_http_lock:
        try:
            if cap is None or not cap.isOpened():
                stopThenStart()
                if cap is None or not cap.isOpened():
                    return None
            for _ in range(3):
                ret, frame = cap.read()
                if ret and frame is not None:
                    _remember_camera_frame(frame)
                    return frame
                time.sleep(0.05)
            stopThenStart()
        except Exception as exc:
            logger.warning("read camera frame failed: {}".format(exc), exc_info=True)
    return None


def _encode_camera_frame(frame):
    ok, encoded = cv2.imencode('.jpg', frame)
    if not ok:
        return None
    return encoded.tostring()


def _crop_guidance_region(image):
    if image is None:
        return image

    height, width = image.shape[:2]
    top = max(0, min(GUIDANCE_CROP_TOP, height - 1))
    bottom = max(top + 1, min(GUIDANCE_CROP_BOTTOM, height))
    left = max(0, min(GUIDANCE_CROP_LEFT, width - 1))
    right = max(left + 1, min(GUIDANCE_CROP_RIGHT, width))
    return image[top:bottom, left:right]


@app.route("/vehicle/cameraSnapshot", methods=['GET'])
def cameraSnapshot():
    frame = _read_camera_frame_for_http()
    if frame is None:
        return jsonify({'success': False, 'message': 'camera frame unavailable'}), 503
    payload = _encode_camera_frame(frame)
    if payload is None:
        return jsonify({'success': False, 'message': 'camera frame encode failed'}), 500
    return Response(payload, mimetype='image/jpeg')


@app.route("/vehicle/cameraStream", methods=['GET'])
def cameraStream():
    return jsonify({
        'success': False,
        'message': 'video stream disabled',
    }), 410


def _get_task_exit_back_length():
    taskParams = redis_cli.hgetall("taskParams")
    try:
        return int(float(taskParams.get('startToChargingPilePointLength') or 0))
    except (TypeError, ValueError):
        return 0


def _run_exit_garage_by_back_length(backLength, reason_prefix):
    global global_status
    try:
        backLength = int(float(backLength))
    except (TypeError, ValueError):
        backLength = 0

    if backLength <= 0:
        logger.warn("出库距离未配置，无法按任务距离出库: {}".format(backLength))
        set_garage_state(GARAGE_STATE_UNKNOWN, reason_prefix + '_back_length_not_configured')
        return False

    set_garage_state(GARAGE_STATE_EXITING, reason_prefix + '_started')
    global_status = 'move back'
    redis_cli.set("correct", "false")
    redis_cli.set("enterGarage", "false")
    reset_odometer(ser)
    redis_cli.set("mission", "working")
    moveBack(ser, backLength)
    if str(getDistanceArrive()) == "0":
        set_garage_state(GARAGE_STATE_UNKNOWN, reason_prefix + '_not_arrived')
        return False
    set_garage_state(GARAGE_STATE_OUTSIDE, reason_prefix + '_completed')
    return True


@app.route("/vehicle/exitGarage", methods=['GET'])
def exitGarage():
    redis_cli.set("reverse", "false")
    task = threading.Thread(target=exit_garage_task)

    task.start()

    response = make_response("1")

    return response


def exit_garage_task():
    set_current_action('exit_garage')
    redis_cli.set("reverse", "false")
    redis_cli.set("correct", "false")
    redis_cli.set('action', 'true')
    redis_cli.set('mission', 'working')
    backLength = _get_task_exit_back_length()
    logger.warn("手动出库按任务距离后退: {}cm".format(backLength))
    _run_exit_garage_by_back_length(backLength, 'manual_exit_garage')
    redis_cli.set('action', 'false')
    redis_cli.set("correct", "false")
    redis_cli.set('mission', 'complete')
    set_current_action('idle')


@app.route("/vehicle/getVehicleInfo", methods=['GET'])
def getVehicleInfo():
    metadata = {}
    metadata.update(_build_task_origin_status_fields())

    metadata[PATH_PLANNING_KEY] = redis_cli.get(PATH_PLANNING_KEY)

    metadata['correct'] = redis_cli.get('correct')

    metadata['forwardSpeed'] = redis_cli.get('forwardSpeed')

    metadata['brushSpeed'] = redis_cli.get('brushSpeed')

    voltage = redis_cli.get('voltage')

    if voltage != None:
        metadata['voltage'] = voltage

    loop_status = _get_loop_auto_clean_status()
    metadata['loopAutoClean'] = loop_status
    metadata['loop_auto_clean'] = loop_status
    metadata['loopAutoCleanEnabled'] = loop_status.get('enabled')
    metadata['loopAutoCleanRunning'] = loop_status.get('running')
    metadata['loopAutoCleanCycle'] = loop_status.get('cycle')
    metadata['loopAutoCleanStopReason'] = loop_status.get('stop_reason')

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
    global global_cur_rtk_heading_at

    logger.warn("legacy correctByRTKTest disabled; use listenerRTK shared reader")
    return
    # rtk_generator = util.readRTK_v2(ser_rtk_params)
    try:
        # —— 1. 主循环
        for lat, lon, heading_deg in rtk_generator:
            global_cur_rtk_lat = lat
            global_cur_rtk_lon = lon
            global_cur_rtk_heading = heading_deg
            global_cur_rtk_heading_at = time.time()
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
            stree_output = compute_linear_steering(heading_error, cte)
            # 发送电机控制指令
            # 正数左轮快，向右偏，负数右轮快，向左偏
            setZSpeed(stree_output)
            duplicateWriteCmd(ser, command)
            logger.warn(
                "linear correction target={:.2f} current={:.2f} heading_error={:.2f} cte={:.2f} distance={:.2f}m z={}".format(
                    const_target_h, heading_deg, heading_error, cte, distance_to_target, stree_output
                )
            )
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
                                "endLon": endLon, "heading": head_target, "speed": 100}

    goCommand(100)
    # 开启RTK纠偏
    global_go = 1
    global_interval = 0
    # 如果global_go==0，则说明直行结束
    startTime = time.time()
    # 如果global_go = 1，则说明直行未结束
    while global_go == 1:
        # 表示到边了
        if getEdge() == "0":
            edge_action = _handle_edge_stop_for_current_task('justMoveByRTK')
            if edge_action == EDGE_STOP_ACTION_TARGET:
                global_go = 0
                break
            if edge_action == EDGE_STOP_ACTION_RECOVER:
                _recover_from_abnormal_edge('justMoveByRTK')
                goCommand(100)
                continue
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
    # 原点航向角，用于起始点转正
    originHeading = float(redis_cli.hget('taskParams','originHeading'))

    global_go = 0
    taskParams = redis_cli.hgetall("taskParams")
    # 初始航向角
    startHeading = float(taskParams.get('heading'))
    originLat = float(taskParams.get('startLat'))
    originLon = float(taskParams.get('startLon'))
    garageEntryLat = float(taskParams.get('garageEntryLat') or originLat)
    garageEntryLon = float(taskParams.get('garageEntryLon') or originLon)
    start_angle_rtk = float(taskParams.get('heading'))
    backLength = _coerce_int(taskParams.get('startToChargingPilePointLength'), 0)
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
                route = {'startLat':item['endLat'], 'startLon':item['endLon'], 'endLat':garageEntryLat,
                         'endLon':garageEntryLon,'angle':180,'heading':(180+start_angle_rtk)%360}
                routes.append(route)

            elif item['angle'] == 180 and next_item['angle'] == 270:
                routes.append(item)
                routes.append(next_item)
                route = {'startLat': next_item['endLat'], 'startLon': next_item['endLon'], 'endLat': garageEntryLat,
                         'endLon': garageEntryLon, 'angle': 180,'heading':(180+start_angle_rtk)%360}
                routes.append(route)
                # 要删除3个任务
                for _ in range(3):
                    redis_cli.lpop("taskList")
            elif item['angle'] == 180 and next_item['angle'] == 90:
                route = {'startLat': item['endLat'], 'startLon': item['endLon'], 'endLat': garageEntryLat,
                         'endLon': garageEntryLon, 'angle': 180,'heading':(180+start_angle_rtk)%360}
                routes.append(route)
                # 要删除1个任务
                redis_cli.lpop("taskList")
            elif item['angle'] == 270:
                routes.append(item)
                route = {'startLat': item['endLat'], 'startLon': item['endLon'], 'endLat': garageEntryLat,
                         'endLon': garageEntryLon, 'angle': 180,'heading':(180+start_angle_rtk)%360}
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

                route2 = {'startLat': endLat, 'startLon': endLon, 'endLat': garageEntryLat,
                         'endLon': garageEntryLon, 'angle': 270, 'heading': (270 + start_angle_rtk) % 360}
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

                route2 = {'startLat': endLat, 'startLon': endLon, 'endLat': garageEntryLat,
                         'endLon': garageEntryLon, 'angle': 270, 'heading': (270 + start_angle_rtk) % 360}
                routes.append(route2)
                # 要删除3个任务
                for _ in range(3):
                    redis_cli.lpop("taskList")
            elif item['angle'] == 180 and next_item['angle'] == 90:
                route = {'startLat': item['endLat'], 'startLon': item['endLon'], 'endLat': garageEntryLat,
                         'endLon': garageEntryLon, 'angle': 180,'heading':(180+start_angle_rtk)%360}
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

                route2 = {'startLat': endLat, 'startLon': endLon, 'endLat': garageEntryLat,
                          'endLon': garageEntryLon, 'angle': 270, 'heading': (270 + start_angle_rtk) % 360}
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
    validation = _validate_auto_drive_request()
    if not validation.get('success'):
        _mark_runtime_blocked(
            validation.get('faultState', 'AUTO_DRIVE_BLOCKED'),
            validation.get('message', '启动条件未通过'),
            validation.get('data')
        )
        return jsonify(validation)
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

def log_task_turn_command(task, index, source):
    task_id = task.get('id', index + 1)
    angle = round(float(task.get('angle', 0)), 2)
    heading = round(float(task.get('heading', 0)), 2)
    logger.warn(
        u"[{}] 已发送第{}段转向命令: taskId={}, 目标角度={}°, 目标航向={}°".format(
            source,
            index + 1,
            task_id,
            angle,
            heading,
        )
    )

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
    backLength = _coerce_int(taskParams.get('startToChargingPilePointLength'), 0)
    lastTaskBackLength = _coerce_int(taskParams.get('lastTaskBackLength'), 0)
    # 起始点经纬度
    global_originLat = float(taskParams.get('startLat'))
    global_originLon = float(taskParams.get('startLon'))
    # 初始航向角
    # initHeading = taskObj['heading']
    # 起始点航向角,用于位置转正
    originHeading = float(taskParams.get('originHeading'))
    exit_decision = decide_auto_exit_garage(get_garage_state(), backLength)
    logger.warn("自动清扫前出舱判定: {}".format(exit_decision))
    if exit_decision.get('decision') == EXIT_DECISION_ALLOW:
        goOutGarage(backLength)
        if get_garage_state() != GARAGE_STATE_OUTSIDE:
            _mark_runtime_blocked(
                'GARAGE_EXIT_NOT_CONFIRMED',
                '出舱未确认完成，自动清扫未启动',
                {
                    'garage_state': get_garage_state(),
                    'garage_state_reason': _decode_redis_value(redis_cli.get(GARAGE_STATE_REASON_KEY)) or '',
                }
            )
            return
    elif exit_decision.get('decision') == EXIT_DECISION_CONFIRM:
        _mark_runtime_blocked(
            'GARAGE_STATE_UNKNOWN',
            '无法确认车辆是否在舱内，请人工确认后再启动自动清扫',
            exit_decision
        )
        return
    elif exit_decision.get('decision') == EXIT_DECISION_BLOCKED:
        _mark_runtime_blocked(
            'GARAGE_STATE_BUSY',
            '车辆正在进舱或出舱，自动清扫未启动',
            exit_decision
        )
        return
    # 获取是否开启定点找寻任务功能
    if False and redis_cli.get("isOpenFindTaskName") == '1':
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
    current_task_name = _normalize_task_name(redis_cli.get('currentTaskName'))
    if not current_task_name:
        _mark_runtime_blocked('CURRENT_TASK_NOT_SET', '未设置当前任务，请先设置当前任务后再启动')
        return

    try:
        taskObj = util.readConfig("config.json")
    except Exception as e:
        logger.error("读取config.json失败: {}".format(str(e)))
        _mark_runtime_blocked('CURRENT_TASK_CONFIG_MISSING', '当前任务配置不存在或不可读')
        return

    config_task_name = _normalize_task_name(taskObj.get('taskName'))
    if config_task_name != current_task_name:
        _mark_runtime_blocked('CURRENT_TASK_MISMATCH', '当前任务与执行配置不一致，请重新设置当前任务')
        return

    taskList = taskObj.get('taskList')
    if not isinstance(taskList, list) or len(taskList) == 0:
        _mark_runtime_blocked('TASK_PATH_EMPTY', '当前任务没有可执行路径，请先生成并设置任务')
        return

    global_status = 'working'
    redis_cli.set("mission", "working")
    redis_cli.set("parking", "0")
    global_doCleanThreadStop = 0
    _mark_runtime_running('自动清扫启动成功，任务执行中')

    # 根据缓存中是否存在任务，来构建新的任务
    resultTask = []
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

        if index == 0:
            logger.warn(
                u"[auto_drive] 第{}段为起始段，不发送转向命令: taskId={}, 目标角度={}°, 目标航向={}°".format(
                    index + 1,
                    task.get('id', index + 1),
                    round(float(angle), 2),
                    round(float(heading), 2),
                )
            )
        else:
            log_task_turn_command(task, index, 'auto_drive')
            turn_result = turn(ser, angle * 10, target_heading=heading, source='auto_drive', segment_index=index + 1, task_id=task.get('id', index + 1))
            if turn_result != 1:
                logger.warn("[auto_drive] 第{}段转向未确认完成，停止自动清扫，避免航向错误后继续直行".format(index + 1))
                sendBraking()
                global_doCleanThreadStop = 1
                break
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
                if lastTaskBackLength != 0:
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
    # RTK unavailable means position is unknown, not that the vehicle is docked.
    logger.info("当前经纬度：{}".format(lat))
    if global_cur_rtk_lat is None:
        logger.warn("RTK信号不可用，无法通过距离确认是否在车库中")
        return 0
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
    set_garage_state(GARAGE_STATE_ENTERING, 'into_garage_started')
    reset_odometer(ser)
    turn(ser, 180 * 10)
    arrived = _run_fixed_enter_garage(backLength)
    logger.warn("自动固定距离进舱结束，distance={}cm, arrived={}".format(
        backLength,
        "true" if arrived else "false"
    ))
    redis_cli.set("correct", "false")
    # 充电命令
    reset_odometer(ser)
    setStatus(5)
    logger.warn(' '.join(format(x, '02x') for x in command))
    duplicateWriteCmd(ser, command)

    global_status = 'goCharging'
    set_garage_state(GARAGE_STATE_DOCKED_BY_COMMAND, 'into_garage_charge_command_sent')
    # moveByRTK(chargingPileLat, chargingPileLon, (originHeading + 180) % 360)
# 出充电桩
def goOutGarage(backLength):
    return _run_exit_garage_by_back_length(backLength, 'auto_drive_exit_garage')

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


def loopAutoDriveThread():
    global loop_auto_clean_thread, global_doCleanThreadStop
    logger.warn("循环自动清扫线程启动")
    _set_loop_auto_clean_state(running=True, stop_reason='running')
    set_current_action('loop_auto_drive')
    try:
        while _is_loop_auto_clean_enabled():
            if _is_loop_low_battery() or isNeedReturnCharging():
                _disable_loop_auto_clean('low_battery_return')
                break

            validation = _validate_auto_drive_request()
            if not validation.get('success'):
                _mark_runtime_blocked(
                    validation.get('faultState', 'LOOP_AUTO_DRIVE_BLOCKED'),
                    validation.get('message', '循环清扫启动条件未通过'),
                    validation.get('data')
                )
                _disable_loop_auto_clean(validation.get('faultState', 'start_blocked'))
                break

            cycle = redis_cli.incr(LOOP_AUTO_CLEAN_CYCLE_KEY)
            redis_cli.set(LOOP_AUTO_CLEAN_UPDATED_AT_KEY, int(time.time()))
            logger.warn("循环自动清扫第{}轮开始".format(cycle))
            global_doCleanThreadStop = 0
            autoDriveByRTKThread()

            if _is_loop_low_battery() or isNeedReturnCharging():
                _disable_loop_auto_clean('low_battery_return')
                break
            if global_doCleanThreadStop != 0:
                _disable_loop_auto_clean('task_interrupted')
                break
            if not _is_loop_auto_clean_enabled():
                break

            logger.warn("循环自动清扫第{}轮结束，等待下一轮".format(cycle))
            slept = 0.0
            while slept < LOOP_AUTO_CLEAN_SLEEP_SECONDS and _is_loop_auto_clean_enabled():
                if _is_loop_low_battery():
                    _disable_loop_auto_clean('low_battery_return')
                    break
                time.sleep(0.5)
                slept += 0.5
    except Exception as e:
        logger.error("循环自动清扫异常: {}".format(str(e)))
        logger.error(traceback.format_exc())
        _mark_runtime_blocked('LOOP_AUTO_DRIVE_ERROR', '循环自动清扫异常: {}'.format(str(e)))
        _disable_loop_auto_clean('error')
    finally:
        _set_loop_auto_clean_state(running=False)
        with LOOP_AUTO_CLEAN_LOCK:
            if loop_auto_clean_thread is threading.current_thread():
                loop_auto_clean_thread = None
        logger.warn("循环自动清扫线程结束: {}".format(
            _decode_redis_value(redis_cli.get(LOOP_AUTO_CLEAN_STOP_REASON_KEY)) or ''
        ))


@app.route("/vehicle/startLoopAutoDrive", methods=['GET'])
def start_loop_auto_drive():
    global loop_auto_clean_thread, global_doCleanThreadStop
    redis_cli.set("reverse", "false")
    with LOOP_AUTO_CLEAN_LOCK:
        if loop_auto_clean_thread is not None and loop_auto_clean_thread.is_alive():
            return jsonify({
                'success': True,
                'message': '循环自动清扫已在运行',
                'data': _get_loop_auto_clean_status(),
            })

        validation = _validate_auto_drive_request()
        if not validation.get('success'):
            _mark_runtime_blocked(
                validation.get('faultState', 'LOOP_AUTO_DRIVE_BLOCKED'),
                validation.get('message', '循环清扫启动条件未通过'),
                validation.get('data')
            )
            return jsonify(validation)

        global_doCleanThreadStop = 0
        _set_loop_auto_clean_state(enabled=True, running=False, stop_reason='', cycle=0)
        _mark_runtime_ready('循环自动清扫启动条件通过，正在创建循环线程', validation.get('data'))
        loop_auto_clean_thread = threading.Thread(target=loopAutoDriveThread)
        loop_auto_clean_thread.daemon = True
        loop_auto_clean_thread.start()

    return jsonify({
        'success': True,
        'message': '循环自动清扫已启动，将持续执行到低电回充',
        'data': _get_loop_auto_clean_status(),
    })


@app.route("/vehicle/stopLoopAutoDrive", methods=['GET'])
def stop_loop_auto_drive():
    global global_status
    _disable_loop_auto_clean('manual_stop')
    doParking()
    global_status = 'active'
    return jsonify({
        'success': True,
        'message': '循环自动清扫已停止',
        'data': _get_loop_auto_clean_status(),
    })


@app.route("/vehicle/getLoopAutoDriveStatus", methods=['GET'])
def get_loop_auto_drive_status():
    return jsonify({
        'success': True,
        'data': _get_loop_auto_clean_status(),
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
@app.route("/vehicle/isAtTaskOrigin", methods=['GET'])
def isAtTaskOrigin():
    task_params = _load_task_params_snapshot()
    result = _build_task_origin_check_result(task_params, _build_runtime_detail())
    return jsonify(result)


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


def _set_current_task(task_name):
    global taskList
    taskName = _normalize_task_name(task_name)
    if not taskName:
        return {"success": False, "msg": "taskName不能为空"}

    fileName = taskName + '.json'
    if not os.path.exists(fileName):
        return {"success": False, "msg": u"文件不存在 {}".format(fileName)}

    with TASK_SWITCH_LOCK:
        with open(fileName, 'r') as src, open('config.json', 'w') as dst:
            for line in src:
                dst.write(line)
        redis_cli.set('currentTaskName', taskName)
        redis_cli.set('curTaskIndex', 0)
        redis_cli.delete('taskList')
        syncCurTaskFileToRedis()
        taskList = []

    return {"success": True, "msg": "淇濆瓨鏁版嵁鎴愬姛", "data": {"taskName": taskName}}
# 设置入舱点，入舱点是小车进入充电桩前的入口位置，不等同于充电桩位置
@app.route("/vehicle/setGarageEntryInfo", methods=['GET'])
def setGarageEntryInfo():
    lat = float(request.args.get('lat'))
    lon = float(request.args.get('lon'))
    redis_cli.hset('taskParams','garageEntryLat',lat)
    redis_cli.hset('taskParams','garageEntryLon',lon)
    result = {"success":True,"msg":"设置成功","garageEntryLat":lat,"garageEntryLon":lon}
    return jsonify(result)


@app.route("/vehicle/confirmInGarage", methods=['GET'])
def confirmInGarage():
    set_garage_state(GARAGE_STATE_DOCKED_MANUAL_CONFIRMED, 'manual_confirm_in_garage')
    return jsonify({"success": True, "msg": "已人工确认车辆在舱内", "data": _garage_state_payload()})


@app.route("/vehicle/confirmOutGarage", methods=['GET'])
def confirmOutGarage():
    set_garage_state(GARAGE_STATE_OUTSIDE, 'manual_confirm_out_garage')
    return jsonify({"success": True, "msg": "已人工确认车辆在舱外", "data": _garage_state_payload()})


@app.route("/vehicle/resetGarageState", methods=['GET'])
def resetGarageState():
    set_garage_state(GARAGE_STATE_UNKNOWN, 'manual_reset_garage_state')
    return jsonify({"success": True, "msg": "已重置舱位状态为未知", "data": _garage_state_payload()})
# 设置原点，也就是起始点
@app.route("/vehicle/setOrigin", methods=['GET'])
def setOrigin():
    global global_originLat,global_originLon,startToChargingPilePointLength
    global_originLat = float(request.args.get('originLat'))
    global_originLon = float(request.args.get('originLon'))

    redis_cli.hset('taskParams','startLat',global_originLat)
    redis_cli.hset('taskParams','startLon',global_originLon)
    if not redis_cli.hget('taskParams','garageEntryLat') or not redis_cli.hget('taskParams','garageEntryLon'):
        redis_cli.hset('taskParams','garageEntryLat',global_originLat)
        redis_cli.hset('taskParams','garageEntryLon',global_originLon)
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
    global global_cur_rtk_heading_at

    logger.warn("legacy correctByRTK disabled; use listenerRTK shared reader")
    return
    # rtk_generator = util.readRTK_v2(ser_rtk_params)
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
            global_cur_rtk_heading_at = time.time()
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
            stree_output = compute_linear_steering(heading_error, cte)
            if global_go == 1:
                if abs(heading_error) > 1 or abs(cte) > 0.02:
                    # logger.warning("视觉纠偏关闭，RTK纠偏开启")
                    redis_cli.set("correct", "false")
                    # 发送电机控制指令
                    # 正数左轮快，向右偏，负数右轮快，向左偏
                    setZSpeed(stree_output)
                    duplicateWriteCmd(ser, command)
                    logger.warn(
                        "linear correction target={:.2f} current={:.2f} heading_error={:.2f} cte={:.2f} distance={:.2f}m z={}".format(
                            target_heading, heading_deg, heading_error, cte, distance_to_target, stree_output
                        )
                    )
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
    _disable_loop_auto_clean('manual_parking')
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
    data = request.get_json() or {}
    garageEntryLat = data.get('garageEntryLat') or data['startLat']
    garageEntryLon = data.get('garageEntryLon') or data['startLon']
    last_task_back_length = _coerce_int(
        data.get('lastTaskBackLength', redis_cli.hget('taskParams', 'lastTaskBackLength')),
        0,
    )
    start_to_charging_pile_point_length = _coerce_int(
        data.get('startToChargingPilePointLength', redis_cli.hget('taskParams', 'startToChargingPilePointLength')),
        0,
    )
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
    # 入舱点经纬度，未设置时默认等于起始点
    redis_cli.hset('taskParams', "garageEntryLat", garageEntryLat)
    redis_cli.hset('taskParams', "garageEntryLon", garageEntryLon)
    # 充电桩经纬度
    redis_cli.hset('taskParams', "chargingPileLat", data['chargingPileLat'])
    redis_cli.hset('taskParams', "chargingPileLon", data['chargingPileLon'])
    # 入舱点到充电桩距离
    redis_cli.hset('taskParams', "startToChargingPilePointLength", start_to_charging_pile_point_length)
    # 最后一个任务结束后的后退距离
    redis_cli.hset('taskParams', "lastTaskBackLength", last_task_back_length)
    current_task_name = _normalize_task_name(redis_cli.get('currentTaskName'))
    if current_task_name:
        _update_json_file_field(current_task_name + '.json', 'lastTaskBackLength', last_task_back_length)
    _update_json_file_field('config.json', 'lastTaskBackLength', last_task_back_length)

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
    data = request.get_json() or {}
    taskName = _normalize_task_name(data.get('taskName'))
    if not taskName:
        return jsonify({"success": False, "msg": "taskName不能为空"})
    # 将任务名称放在一个set集合中
    redis_cli.sadd('taskNameSet', taskName)
    taskParams = redis_cli.hgetall("taskParams")
    startLat = float(taskParams.get('startLat'))
    startLon = float(taskParams.get('startLon'))
    point = {'startLat': startLat, 'startLon': startLon}
    redis_cli.hset('loc_start_lat_lon',taskName,json.dumps(point))
    areaList = data.get('areaList')
    if not isinstance(areaList, list) or len(areaList) == 0:
        return jsonify({"success": False, "msg": "areaList不能为空"})
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
    taskName = _normalize_task_name(request.args.get('taskName'))
    if not taskName:
        return jsonify({"success": False, "msg": "taskName不能为空"})
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
    return jsonify(_set_current_task(request.args.get('taskName')))


@app.route("/vehicle/setCurrentTask", methods=['GET'])
def setCurrentTask():
    return jsonify(_set_current_task(request.args.get('taskName')))

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
    # 入舱点经纬度，兼容旧任务文件：没有入舱点则默认使用起始点
    redis_cli.hset('taskParams', "garageEntryLat", taskObj.get('garageEntryLat', taskObj['startLat']))
    redis_cli.hset('taskParams', "garageEntryLon", taskObj.get('garageEntryLon', taskObj['startLon']))
    # 充电桩经纬度
    redis_cli.hset('taskParams', "chargingPileLat", taskObj['chargingPileLat'])
    redis_cli.hset('taskParams', "chargingPileLon", taskObj['chargingPileLon'])
    # 入舱点到充电桩距离
    redis_cli.hset('taskParams', "startToChargingPilePointLength", taskObj['startToChargingPilePointLength'])
    existing_last_task_back_length = _coerce_int(redis_cli.hget('taskParams', 'lastTaskBackLength'), 0)
    last_task_back_length = taskObj.get('lastTaskBackLength', existing_last_task_back_length)
    redis_cli.hset('taskParams', "lastTaskBackLength", last_task_back_length)

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
        logger.warn("return_to_point uses shared RTK reader")
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
                            "endLat": endLat,"endLon": endLon, "speed": 100}
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
        if getEdge() == "0":
            edge_action = _handle_edge_stop_for_current_task('moveByRTK')
            if edge_action == EDGE_STOP_ACTION_TARGET:
                global_go = 0
                break
            if edge_action == EDGE_STOP_ACTION_RECOVER:
                _recover_from_abnormal_edge('moveByRTK')
                goCommand(100)
                continue
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
    global_cur_taskPoint = {"heading": heading, "startLat": startLat, "startLon": startLon,
                            "endLat": endLat, "endLon": endLon, "speed": speed}
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
            edge_action = _handle_edge_stop_for_current_task('pointToPointByRTK')
            if edge_action == EDGE_STOP_ACTION_TARGET:
                global_go = 0
                break
            if edge_action == EDGE_STOP_ACTION_RECOVER:
                _recover_from_abnormal_edge('pointToPointByRTK')
                goCommand(speed)
                continue
            result = 0
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
    write_start = time.time()
    duplicateWriteCmd(ser, command)
    write_ms = (time.time() - write_start) * 1000.0
    now = time.time()
    last_diag_at = getattr(observer_rtk_data, '_last_diag_at', 0.0)
    # RTK_DIAG lower_machine_heading log disabled.
    if False and (now - last_diag_at >= 1.0 or write_ms > 100.0):
        logger.warning("RTK_DIAG lower_machine_heading heading={} write_ms={:.1f}".format(heading, write_ms))
        observer_rtk_data._last_diag_at = now
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
    global global_cur_rtk_lat, global_cur_rtk_lon, global_cur_rtk_heading, global_cur_rtk_heading_at, global_go
    global_cur_rtk_lat = data.lat
    global_cur_rtk_lon = data.lon
    if data.heading is not None:
        global_cur_rtk_heading = data.heading
        global_cur_rtk_heading_at = time.time()
    sync_start = time.time()
    sync_current_location(data.lat, data.lon, data.heading)
    sync_ms = (time.time() - sync_start) * 1000.0
    now = time.time()
    last_diag_at = getattr(observer_go_correct, '_last_diag_at', 0.0)
    # RTK_DIAG redis_update log disabled.
    if False and (now - last_diag_at >= 1.0 or sync_ms > 100.0):
        logger.warning(
            "RTK_DIAG redis_update lat={} lon={} heading={} heading_at={} sync_ms={:.1f} global_go={}".format(
                data.lat, data.lon, data.heading, global_cur_rtk_heading_at, sync_ms, global_go
            )
        )
        observer_go_correct._last_diag_at = now
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
        stree_output = compute_linear_steering(heading_error, cte, cte_gain=800)
        # if abs(heading_error) > 1 or abs(cte) > 0.02:
            # logger.warning("视觉纠偏关闭，RTK纠偏开启")
            # redis_cli.set("correct", "false")
        # 发送电机控制指令
        # 正数左轮快，向右偏，负数右轮快，向左偏
        setZSpeed(-stree_output)
        duplicateWriteCmd(ser, command)
        logger.info(
            "linear correction target={:.2f} current={:.2f} heading_error={:.2f} cte={:.2f} last_cte={:.2f} distance={:.2f}m cte_gain=800 z={}".format(
                target_heading, data.heading, heading_error, cte, global_last_cte, distance_to_target,
                -stree_output
            )
        )

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

        if index == 0:
            logger.warn(
                u"[go_on] 第{}段为续跑列表的首段，不发送转向命令: taskId={}, 目标角度={}°, 目标航向={}°".format(
                    index + 1,
                    task.get('id', index + 1),
                    round(float(angle), 2),
                    round(float(heading), 2),
                )
            )
        else:
            log_task_turn_command(task, index, 'go_on')
            turn_result = turn(ser, angle * 10, target_heading=heading, source='go_on', segment_index=index + 1, task_id=task.get('id', index + 1))
            if turn_result != 1:
                logger.warn("[go_on] 第{}段转向未确认完成，停止续跑，避免航向错误后继续直行".format(index + 1))
                sendBraking()
                global_doCleanThreadStop = 1
                break
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

    return normalize_visual_line_angle(angle_deg)


def normalize_visual_line_angle(angle_deg):
    try:
        angle = float(angle_deg)
    except Exception:
        return 0.0
    while angle > 90.0:
        angle -= 180.0
    while angle <= -90.0:
        angle += 180.0
    return angle


def _detect_guidance_line(image, center_x, allow_lsd=True):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    bright_band = find_vertical_bright_band(
        gray,
        center_x,
        min_band_width=14,
        min_band_height=max(120, int(image.shape[0] * 0.45)),
        min_column_ratio=0.28,
        max_vertical_angle=45.0
    )
    if bright_band is not None:
        return {
            'offset': bright_band['center_x'] - center_x,
            'angle': bright_band['angle'],
            'mode': 'bright_band',
            'width': bright_band['width'],
        }

    if not allow_lsd:
        return None

    lsd = cv2.createLineSegmentDetector(0)
    dlines = lsd.detect(gray)
    line = None
    selected_length = 0.0
    line_angle = 0.0

    if not (dlines[0] is None):
        for dline in dlines[0]:
            x0 = int(round(dline[0][0]))
            y0 = int(round(dline[0][1]))
            x1 = int(round(dline[0][2]))
            y1 = int(round(dline[0][3]))

            vertical_span = abs(y1 - y0)
            if vertical_span <= 90:
                continue

            dx = x1 - x0
            dy = y1 - y0
            length = math.hypot(dx, dy)
            if length < 130:
                continue

            vertical_angle = math.degrees(math.atan2(abs(dx), abs(dy))) if abs(dy) >= 1e-5 else 90
            if vertical_angle > 25:
                continue

            xmid = (x1 + x0) / 2.0
            offset = xmid - center_x
            if line is None or abs(offset) < abs(line) or (abs(offset) == abs(line) and length > selected_length):
                line = offset
                selected_length = length
                line_angle = calculate_angle(x0, y0, x1, y1)

    if line is None:
        return None

    return {
        'offset': line,
        'angle': line_angle,
        'mode': 'lsd',
        'width': 0,
    }


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
        turn_result = turn(ser, originHeading * 10, target_heading=originHeading, source='start_heading_check')
        if turn_result == 1 or getRotateArrive() == '1':
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
    global last_camera_open_attempt_at
    with camera_http_lock:
        now = time.time()
        if now - last_camera_open_attempt_at < 3.0:
            return
        last_camera_open_attempt_at = now
        if cap is not None:
            try:
                cap.release()
                global_status = "成功退出视频"
            except Exception:
                global_status = "释放失败，重试"
        cap = _open_camera_capture()
        if cap is None:
            global_status = "no camera source is available"


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


def _send_distance_move_command(length, speed):
    reSetStatus(ser)
    setStatus(2)
    setPowerOn(1)
    setHWstatus(0, 0, 0, 0, 0)
    setXSpeed(speed)
    setDistance(length)
    command[17] = tem_listener(command, 17)
    logger.warn(' '.join(format(x, '02x') for x in command))
    duplicateWriteCmd(ser, command)


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
    edge_accepted = False
    edge_recovery_attempts = 0
    while getDistanceArrive() == "0" and redis_cli.get("mission") == "working":
        if getEdge() != "1":
            edge_action = _handle_edge_stop_for_current_task('moveDiatance')
            if edge_action == EDGE_STOP_ACTION_TARGET:
                edge_accepted = True
                break
            if edge_action == EDGE_STOP_ACTION_RECOVER and edge_recovery_attempts < EDGE_RECOVERY_MAX_ATTEMPTS:
                edge_recovery_attempts += 1
                _recover_from_abnormal_edge('moveDiatance')
                _send_distance_move_command(length, speed)
                continue
            if edge_action == EDGE_STOP_ACTION_RECOVER:
                logger.warn("edge recovery exceeded max attempts: source=moveDiatance, attempts={}".format(
                    edge_recovery_attempts
                ))
                doParking()
                return 0
            return 0
        if global_go == 0:
            break
        if redis_cli.get("mission") == "complete":
            return 0
        print("keep walking")
        # global_status = "keep walking"
        time.sleep(0.25)
    time.sleep(0.5)
    if edge_accepted or getDistanceArrive() == "1":
        global_status = "到达"
        print("到达")
        return 1
    else:
        print("未到达，结束")
        # global_status = "未到达，结束"
        return 0


def turn(ser, roundTo, target_heading=None, source='turn', segment_index=None, task_id=None):
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

    target_heading = _coerce_float(target_heading, None)
    turn_start_at = time.time()
    stable_count = 0
    last_fallback_log_at = 0
    turn_result = 0
    previous_delta = None
    logger.warn(
        "[turn_start] source={}, segment={}, taskId={}, roundTo={}, targetHeading={}".format(
            source, segment_index, task_id, roundTo, target_heading
        )
    )

    # 如果未完成，则继续阻塞，等待旋转完成 and redis_cli.get("mission") == "working"
    while redis_cli.get("mission") == "working":
        rotate_arrive = getRotateArrive()
        if rotate_arrive != "0":
            logger.warn(
                "[turn_finish] lower-machine finish source={}, segment={}, taskId={}, elapsed={:.2f}s".format(
                    source, segment_index, task_id, time.time() - turn_start_at
                )
            )
            turn_result = 1
            break
        # if redis_cli.get("ultraSonic") == "true":
        #     break
        logger.info("wait rotate finish")
        # global_status = "wait rotate finish"
        if target_heading is None:
            continue

        elapsed = time.time() - turn_start_at
        current_heading = _get_current_rtk_heading()
        delta = _normalize_heading_delta(current_heading, target_heading)
        crossed_target = False
        if delta is not None and elapsed >= TURN_RTK_FALLBACK_MIN_WAIT_SEC:
            crossed_target = _heading_delta_crossed_target(previous_delta, delta)
            if abs(delta) <= TURN_RTK_FALLBACK_TOLERANCE_DEG:
                stable_count += 1
            else:
                stable_count = 0
            if stable_count >= TURN_RTK_FALLBACK_STABLE_COUNT or crossed_target:
                finish_reason = 'crossed_target' if crossed_target else 'within_tolerance'
                logger.warn(
                    "[turn_rtk_fallback] source={}, segment={}, taskId={}, reason={}, target={:.2f}, current={:.2f}, delta={:.2f}, previousDelta={}, elapsed={:.2f}s, stableCount={}; sending brake".format(
                        source,
                        segment_index,
                        task_id,
                        finish_reason,
                        target_heading,
                        current_heading,
                        delta,
                        previous_delta,
                        elapsed,
                        stable_count,
                    )
                )
                sendBraking()
                turn_result = 1
                break
        else:
            stable_count = 0

        if delta is not None:
            previous_delta = delta

        if elapsed - last_fallback_log_at >= 2.0:
            logger.info(
                "[turn_wait] source={}, segment={}, taskId={}, target={}, current={}, delta={}, previousDelta={}, elapsed={:.2f}s, stableCount={}".format(
                    source,
                    segment_index,
                    task_id,
                    target_heading,
                    current_heading,
                    delta,
                    previous_delta,
                    elapsed,
                    stable_count,
                )
            )
            last_fallback_log_at = elapsed

    time.sleep(0.5)

    redis_cli.set("carStatus", "go")
    # 传感器报警了，到边了
    # if redis_cli.get("ultraSonic") == "true":
    #     moveBack(ser, distance=9)
    #     turn(ser, roundTo)
    return turn_result


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
                                            "endLon": endLon, "heading": heading, "speed": 250}
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
                    if not moveDiatance(ser, length, 250):
                        logger.warn("mode2 moveDiatance failed; stop current task without deleting taskList")
                        return 0
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
                                            "endLon": endLon, "heading": heading, "speed": 250}
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
                    if not moveDiatance(ser, length, 250):
                        logger.warn("mode2 moveDiatance failed; stop current task without deleting taskList")
                        return 0
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
                if not moveDiatance(ser, length):
                    logger.warn("mode2 moveDiatance failed; stop current task without deleting taskList")
                    break
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
    while True:
        try:
            if _coerce_bool(redis_cli.get("moveJudge"), False):
                time.sleep(0.05)
                continue
            _read_lower_machine_status_frame("listenerSlavePort", 0.25)
        except Exception as e:
            logging.info('鐩戝惉鎶ラ敊:{}'.format(e))
            time.sleep(0.5)
        time.sleep(0.1)

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
        with camera_http_lock:
            if cap is None or not cap.isOpened():
                ret = False
                image = None
            else:
                ret, image = cap.read()
        if not ret:
            logger.error('无法读取视频流或文件结束')
            stopThenStart()
            time.sleep(2)
            continue
        else:
            _remember_camera_frame(image)
            height, width = image.shape[:2]
            logger.info('图片高度: %d, 宽度: %d', height, width)
            image = _crop_guidance_region(image)
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
    guidance_tracking = False
    guidance_band_tracker = GuidanceBandTracker(
        max_abs_offset=None,
        max_offset_jump=25,
        max_width_change=8,
        min_stable_frames=3,
    )
    while True:
        try:
            if "false" == redis_cli.get("correct"):
                guidance_tracking = False
                guidance_band_tracker.reset()
                time.sleep(0.1)
                continue

            ret, image = cap.read()
            if not ret:
                logger.error('无法读取视频流')
                stopThenStart()
                continue

            _remember_camera_frame(image)
            image = _crop_guidance_region(image)
            image = cv2.blur(image, (5, 5))
            enter_garage_mode = redis_cli.get("enterGarage") == "true"
            guidance_line = _detect_guidance_line(image, center_x, allow_lsd=False)
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
                    if vertical_span <= 90:
                        continue
                    dx = x1 - x0
                    dy = y1 - y0
                    length = math.hypot(dx, dy)
                    if length < 130:
                        continue
                    vertical_angle = math.degrees(math.atan2(abs(dx), abs(dy))) if abs(dy) >= 1e-5 else 90
                    if vertical_angle > 25:
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
                        line_angle = calculate_angle(x0, y0, x1, y1)
            if guidance_line is not None:
                raw_guidance_line = guidance_line
                guidance_line = guidance_band_tracker.update(raw_guidance_line)
                if guidance_line is None:
                    logger.info(
                        'bright_band rejected: offset=%.1f, width=%s',
                        raw_guidance_line.get('offset'),
                        raw_guidance_line.get('width')
                    )
                    line = None
                    line_angle = 0
                if guidance_line is not None:
                    line = guidance_line['offset']
                    line_angle = guidance_line['angle']
                    logger.info(
                        'visual guidance using %s correction: offset=%.1f, angle=%.1f, width=%s',
                        guidance_line.get('mode'),
                        line,
                        line_angle,
                        guidance_line.get('width')
                    )
            else:
                guidance_band_tracker.reset()
                line = None
                line_angle = 0
            guidance_decision = resolve_guidance_command(line, line_angle, guidance_tracking)
            guidance_tracking = guidance_decision['tracking']
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
            if guidance_decision['mode'] == 'no_detection':
                logger.info('visual guidance no detection; keep straight without z command')
            elif guidance_decision['mode'] == 'tracking_lost':
                logger.info('visual guidance tracking lost; send one zero z command')
            if abs(line_angle) > 10:
                line = -line_angle * 2
            if guidance_decision['send']:
                setZSpeed(guidance_decision['z_speed'])
                duplicateWriteCmd(ser, command)

            if redis_cli.get("enterGarage") == "true":
                hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                mask1 = cv2.inRange(hsv_img, lower1, upper1)
                mask2 = cv2.inRange(hsv_img, lower2, upper2)
                mask = mask1 + mask2

                if np.sum(mask) > 0:
                    redis_cli.set("detectQrcode", "true")
                    redis_cli.set('enterGarage', 'false')
                    logger.warn("视觉入舱识别成功，准备停车")

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
    voltage = redis_cli.get("voltage")
    need_return = should_return_to_charge(voltage)
    logger.info("low battery return check: voltage={}, threshold=10, need_return={}".format(voltage, need_return))
    if not need_return:
        return False

    if None == global_cur_rtk_lat:
        logger.warning("low battery return skipped: RTK position unavailable")
        return False

    # 判断当前到那个任务了
    json_item = redis_cli.lindex('taskList', 0)
    json_next_item = redis_cli.lindex('taskList', 1)

    if json_item == None or json_next_item == None:
        logger.warning("low battery return skipped: taskList needs current and next task")
        return False
    return True


def listenerVoltage():
    global global_status,drive_thread,global_doCleanThreadStop

    while True:
        # 如果voltageLisener为1表示开启电池监听
        if redis_cli.get('voltageListener') != '0':
            voltage = redis_cli.get("voltage")
            logger.info("status={},voltage={}".format(global_status,voltage))
            # 当小车状态不是返回充电桩时并且需要返回充电桩充电时,就立即停止小车，并返回充电桩充电
            if global_status != 'goCharging' and isNeedReturnCharging() and redis_cli.llen('taskList') != 0:
                # global_status = 'goCharging'
                _disable_loop_auto_clean('low_battery_return')
                global_doCleanThreadStop = 1
                doParking()
                # if global_status == 'goCharging':
                if global_doCleanThreadStop == 1:
                    logger.warning("返回充电桩")
                    thread = threading.Thread(target=returnToPointByRTKThread)
                    thread.start()

            # 当电量大于93时并且有未完成的任务，则继续清扫未完成的任务
            if _coerce_int(voltage, 0) >= 90 and redis_cli.llen('taskList') != 0:
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
        rtk_manager.register_observer(observer_go_correct)
        rtk_manager.register_observer(observer_rtk_data)
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
    if LOCAL_MODE:
        logger.warning("CLEAN_LOCAL_MODE=1: skip camera, serial, RTK and MQTT background workers")
        app.run(host='0.0.0.0', port=7899)
        return

    # if not varifyGps():
    #     return
    if cap is not None and cap.isOpened():
        thread = threading.Thread(target=startOpencv)
        thread.start()
    else:
        logger.warning("camera unavailable; skip startOpencv thread")

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
    listenerVoltageThread.start()

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
