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
from rtk_path_tracking import (
    RTKKalmanFilter2D,
    StraightLinePController,
    build_tracking_command,
)
from battery_return import LOW_BATTERY_RETURN_THRESHOLD, should_return_to_charge
from edge_target_guard import (
    DEFAULT_EDGE_TARGET_TOLERANCE_M,
    EdgeTriggerLatch,
    should_accept_edge_stop,
    should_recover_from_edge_stop,
)
from FixedPositiveChecker import FixedPositiveChecker

from flask_cors import CORS

from MqttClient import MqttClient
from ntrip_runtime import get_shared_runtime, reset_shared_runtime
from RTKDataManager import RTKDataManager
# from pid import PID

from mqtt_integration import MQTTIntegration
from mqtt_vehicle_adapter import VehicleControllerAdapter
from manual_steering import ManualSteeringController, ManualSteeringError
from motion_state import derive_manual_motion_state
from vision_line_detection import GuidanceBandTracker, find_vertical_bright_band, resolve_guidance_command
from go_to_point import build_go_to_point_plan
from turn_heading_control import (
    TURN_LEFT_PROTOCOL_VALUE,
    TURN_RIGHT_PROTOCOL_VALUE,
    choose_turn_direction,
)
from waypoint_loop import iter_closed_loop_targets, normalize_loop_options
from runtime_state import RUNTIME_STATE_KEY, build_runtime_state_snapshot
from status_values import (
    live_heading_from_location,
    live_value_from_report,
    visible_task_field,
)
from robot_fsm import (
    RUNTIME_EVENT_LOG_KEY,
    RobotEventBus,
    RobotLifecycleFSM,
    legacy_fields_for_state,
    runtime_event_type_for_control_state,
)
from dev_console.correction_state import build_correction_state
from dev_console.state_readers import (
    build_overview_state,
    build_redis_state,
    build_task_path_state,
    read_log_lines,
)
from modeling_routes import register_modeling_routes
from lan_cloud_compat import register_lan_cloud_compat_routes
from position_history import ModelingPositionHistory
from cleaning_position import CleaningPositionHistory, CleaningPositionService
from modeling_sampler import sample_current_point
from modeling_task_persistence import (
    ModelingTaskPersistenceError,
    build_named_task,
    is_same_named_task,
    normalize_task_name,
    select_named_task_variant,
)
from modeling_saved_routes import build_saved_routes, discover_saved_task_names
from modeling_task_generator import generate_task_plan as generate_modeling_task_plan
from modeling_execution import (
    ModelingExecutionError,
    build_execution_plan,
    execute_modeling_plan,
)
from route_segment_execution import run_route_segment
from continuous_route import (
    CONTINUATION_KEY,
    DEFAULT_CORRIDOR_M,
    DEFAULT_LOOKAHEAD_M,
    attach_continuations,
    build_continuous_polyline,
    collect_continuous_run,
    compute_polyline_guidance,
)
from route_start_guard import RouteStartGuardError, validate_route_start

app = Flask(__name__)
CORS(app)
robot_event_bus = RobotEventBus(max_events=200)
robot_lifecycle_fsm = RobotLifecycleFSM()
dev_console_trace = []

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
AUTO_RESUME_ALLOWED_KEY = "autoResumeAllowed"

WAYPOINT_LOOP_ENABLED_KEY = 'waypointLoopEnabled'
WAYPOINT_LOOP_MODE_KEY = 'waypointLoopMode'
WAYPOINT_LOOP_TARGET_KEY = 'waypointLoopTarget'
WAYPOINT_LOOP_CURRENT_KEY = 'waypointLoopCurrent'

high_speed = 350

high_brush_speed = 30

ip = "218.2.130.246"

id = "30f4b4af-c1a8-f3f6-072d-3807918c0dc0"

redis_cli = redis.Redis(host='localhost', port=6379, db=0, decode_responses=True)

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
redis_cli.set('controlState', 'INITIALIZING')
redis_cli.set('healthState', 'OK')
redis_cli.set('faultState', '')
redis_cli.set('startCheckReady', 'false')
redis_cli.set('startCheckReason', '系统启动，正在初始化配置和运行环境')
startup_runtime_detail = {
    'initializing': True,
    'initializationPhase': 'redis_defaults',
}
redis_cli.set('runtimeDetail', json.dumps(startup_runtime_detail))
redis_cli.set(LOOP_AUTO_CLEAN_ENABLED_KEY, 'false')
redis_cli.set(LOOP_AUTO_CLEAN_RUNNING_KEY, 'false')
redis_cli.set(LOOP_AUTO_CLEAN_CYCLE_KEY, 0)
redis_cli.set(LOOP_AUTO_CLEAN_STOP_REASON_KEY, '')
redis_cli.set(LOOP_AUTO_CLEAN_UPDATED_AT_KEY, int(time.time()))
redis_cli.set(AUTO_RESUME_ALLOWED_KEY, 'false')
redis_cli.delete('battery')
redis_cli.delete('batteryPercent')
redis_cli.delete('batteryRaw')
redis_cli.delete('batteryPercentRaw')
redis_cli.delete('batteryReportAt')
redis_cli.delete('voltage')
redis_cli.delete('packVoltage')
redis_cli.delete('packVoltageReportAt')
redis_cli.set('bootSafeStopAt', int(time.time()))
redis_cli.set(RUNTIME_STATE_KEY, json.dumps(build_runtime_state_snapshot(
    control_state='INITIALIZING',
    health_state='OK',
    fault_state='',
    mission='complete',
    parking=True,
    action='idle',
    task_index=0,
    start_ready=False,
    message='系统启动，正在初始化配置和运行环境',
    detail=startup_runtime_detail,
    now=time.time(),
), ensure_ascii=False))

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
MANUAL_STEERING_COMMAND_LOCK = threading.RLock()
manual_steering_controller = None
MANUAL_STEERING_CONTROLLER_LOCK = threading.RLock()

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
# RTK 固定解可信度较高时，适当提高过程噪声、降低测量噪声：
# - process_noise 变大：少依赖“车辆按上一帧速度连续运动”的预测；
# - measurement_noise 变小：更多采纳当前 RTK 测量坐标。
global_rtk_tracking_filter = RTKKalmanFilter2D(process_noise=0.2, measurement_noise=1.0)
global_straight_line_controller = StraightLinePController(
    heading_gain=10.0,
    cte_gain=1000.0,
    short_range_heading_limit_deg=5.0,
    max_z_speed=15000,
)
# 自动清扫线程是否结束，0：未结束，1：结束
global_doCleanThreadStop = 0
global_auto_clean_stop = 0
global_waypoint_nav_stop = 0
global_loop_auto_clean_stop = 0
active_runtime_task_token = ''
active_runtime_task_action = ''
global_runtime_task_sequence = 0
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
global_rtk_fix_lost_time = None
global_rtk_recovering = False
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
TASK_ORIGIN_TOLERANCE_METERS = 0.20
START_POSITION_TOLERANCE_METERS = TASK_ORIGIN_TOLERANCE_METERS
EDGE_TARGET_TOLERANCE_M = DEFAULT_EDGE_TARGET_TOLERANCE_M
EDGE_RECOVERY_BACK_CM = 5
EDGE_RECOVERY_MAX_ATTEMPTS = 3
EDGE_STOP_ACTION_TARGET = 'target'
EDGE_STOP_ACTION_RECOVER = 'recover'
EDGE_STOP_ACTION_ABORT = 'abort'
global_edge_trigger_latch = EdgeTriggerLatch()
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
TURN_RTK_MAX_DURATION_SEC = 30.0
RTK_FIXED_QUALITY = '4'
RTK_FIXED_GGA_MAX_AGE_SECONDS = 2.0
RTK_FIX_RECOVERY_TIMEOUT_SECONDS = 300.0

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


def _publish_global_go(value):
    try:
        redis_cli.set('globalGo', 1 if int(value) == 1 else 0)
    except Exception as e:
        logger.warning("同步globalGo失败: {}".format(str(e)))


def _publish_correction_debug(heading_error, cte, z_speed, distance_to_target,
                              signed_remaining, target_heading, current_heading,
                              extra=None):
    try:
        redis_cli.hset('correctionDebug', 'headingError', round(float(heading_error), 6))
        redis_cli.hset('correctionDebug', 'cte', round(float(cte), 6))
        redis_cli.hset('correctionDebug', 'zSpeed', int(round(float(z_speed))))
        redis_cli.hset('correctionDebug', 'distanceToTarget', round(float(distance_to_target), 6))
        redis_cli.hset('correctionDebug', 'signedRemaining', round(float(signed_remaining), 6))
        redis_cli.hset('correctionDebug', 'targetHeading', round(float(target_heading), 6))
        redis_cli.hset('correctionDebug', 'currentHeading', round(float(current_heading), 6))
        redis_cli.hset('correctionDebug', 'headingGain', 10.0)
        redis_cli.hset('correctionDebug', 'cteGain', 1000.0)
        redis_cli.hset('correctionDebug', 'cteDotGain', 0.0)
        if isinstance(extra, dict):
            for key, value in extra.items():
                if value is None:
                    continue
                if isinstance(value, float):
                    value = round(value, 8)
                redis_cli.hset('correctionDebug', key, value)
        redis_cli.hset('correctionDebug', 'updatedAt', time.time())
    except Exception as e:
        logger.warning("同步correctionDebug失败: {}".format(str(e)))


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
    try:
        return normalize_task_name(_decode_redis_value(task_name))
    except ModelingTaskPersistenceError:
        return ''


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


def _set_auto_resume_allowed(allowed, reason=''):
    try:
        redis_cli.set(AUTO_RESUME_ALLOWED_KEY, 'true' if allowed else 'false')
        redis_cli.set('autoResumeReason', reason or '')
        redis_cli.set('autoResumeUpdatedAt', int(time.time()))
    except Exception as e:
        logger.warning("set auto resume flag failed: {}".format(str(e)))


def _is_auto_resume_allowed():
    return _coerce_bool(redis_cli.get(AUTO_RESUME_ALLOWED_KEY), False)


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


def _begin_runtime_task(action):
    global active_runtime_task_token, active_runtime_task_action
    global global_runtime_task_sequence
    global global_auto_clean_stop, global_waypoint_nav_stop, global_loop_auto_clean_stop
    global global_doCleanThreadStop

    global_runtime_task_sequence += 1
    action_name = str(action or 'task')
    active_runtime_task_token = '{}:{}:{}'.format(action_name, int(time.time() * 1000), global_runtime_task_sequence)
    active_runtime_task_action = action_name

    if action_name == 'auto_drive':
        global_auto_clean_stop = 0
        global_doCleanThreadStop = 0
    elif action_name == 'multi_go_to_point':
        global_waypoint_nav_stop = 0
        global_doCleanThreadStop = 0
    elif action_name == 'loop_auto_drive':
        global_loop_auto_clean_stop = 0
        global_auto_clean_stop = 0
        global_doCleanThreadStop = 0
    return active_runtime_task_token


def _is_current_runtime_task(task_token):
    if not task_token:
        return not active_runtime_task_token
    return str(task_token) == str(active_runtime_task_token)


def _runtime_task_should_stop(task_token, task_type=None):
    if not _is_current_runtime_task(task_token):
        return True
    if task_type == 'auto_drive' and global_auto_clean_stop:
        return True
    if task_type == 'multi_go_to_point' and global_waypoint_nav_stop:
        return True
    if task_type == 'loop_auto_drive' and global_loop_auto_clean_stop:
        return True
    return _is_runtime_stop_requested()


def _is_runtime_task_active():
    fsm_state = robot_lifecycle_fsm.get_state()
    control_state = str(fsm_state.get('controlState') or '').upper()
    return control_state in ('RUNNING', 'PAUSED', 'STOPPING')


def _is_runtime_stop_requested():
    fsm_state = robot_lifecycle_fsm.get_state()
    control_state = str(fsm_state.get('controlState') or '').upper()
    return control_state in ('STOPPED', 'COMPLETE', 'BLOCKED', 'FAULT', 'DISABLED', 'UNKNOWN')


def _can_start_runtime_task():
    fsm_state = robot_lifecycle_fsm.get_state()
    control_state = str(fsm_state.get('controlState') or '').upper()
    return control_state in ('STOPPED', 'COMPLETE')


def _runtime_not_startable_payload():
    fsm_state = robot_lifecycle_fsm.get_state()
    control_state = str(fsm_state.get('controlState') or '').upper()
    return {
        'success': False,
        'code': 'RUNTIME_NOT_STARTABLE',
        'msg': '当前状态不允许启动任务',
        'data': {
            'controlState': control_state,
            'action': fsm_state.get('action') or '',
            'faultState': fsm_state.get('faultState') or '',
            'message': fsm_state.get('message') or '',
        },
    }


def _start_runtime_thread(action, target, args=(), ready_message='正在创建任务线程', detail=None):
    with TASK_SWITCH_LOCK:
        if not _can_start_runtime_task():
            return None, _runtime_not_startable_payload()

        # 自动清扫、返航和点位导航等任务开始前，统一结束手动转向会话。
        # 这里只清除控制器中的长按/点按状态并使延迟定时器失效，不向下位机
        # 额外发送直行或停车命令；随后启动的运行任务会自行下发它需要的指令。
        # 这样可以避免用户刚松开方向键时，旧的 300ms 点按恢复动作干扰新任务。
        _reset_manual_steering(send_hardware=False)

        task_token = _begin_runtime_task(action)
        ready_detail = dict(detail or {})
        if action:
            ready_detail['action'] = action
        ready_detail['taskToken'] = task_token
        ready_detail['starting'] = True
        _mark_runtime_ready(ready_message, ready_detail)

        thread = threading.Thread(target=target, args=(task_token,) + tuple(args or ()))
        thread.daemon = True
        thread.start()
        return thread, None


def _runtime_action():
    fsm_state = robot_lifecycle_fsm.get_state()
    return str(fsm_state.get('action') or '')


def _is_runtime_returning_to_charge():
    current_action = _runtime_action()
    if current_action in ('return_to_point', 'charging', 'into_garage'):
        return True
    garage_state = get_garage_state()
    return garage_state in (
        GARAGE_STATE_ENTERING,
        GARAGE_STATE_DOCKED_BY_COMMAND,
        GARAGE_STATE_DOCKED_MANUAL_CONFIRMED,
    )


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


def _publish_runtime_event(event_type, message='', payload=None, source='runtime_state', fsm_state=None):
    if not event_type:
        return None
    payload = dict(payload or {})
    payload['fsm'] = fsm_state if isinstance(fsm_state, dict) else robot_lifecycle_fsm.get_state()
    event = robot_event_bus.publish(
        event_type,
        source=source,
        message=message,
        payload=payload,
    )
    try:
        redis_cli.lpush(RUNTIME_EVENT_LOG_KEY, json.dumps(event, ensure_ascii=False))
        try:
            redis_cli.ltrim(RUNTIME_EVENT_LOG_KEY, 0, 199)
        except Exception:
            pass
    except Exception as e:
        logger.warning("同步runtime event失败: {}".format(str(e)))
    return event


def _mirror_runtime_state_to_redis(fsm_state, legacy_fields):
    fsm_state = fsm_state if isinstance(fsm_state, dict) else {}
    legacy_fields = legacy_fields if isinstance(legacy_fields, dict) else {}
    _set_redis_value('controlState', fsm_state.get('controlState'))
    _set_redis_value('healthState', fsm_state.get('healthState'))
    _set_redis_value('faultState', fsm_state.get('faultState'))
    _set_redis_value('startCheckReady', fsm_state.get('startReady'))
    _set_redis_value('startCheckReason', fsm_state.get('message') or '')
    _set_redis_value('mission', legacy_fields.get('mission'))
    _set_redis_value('parking', legacy_fields.get('parking'))
    _set_redis_value('currentAction', legacy_fields.get('currentAction'))


def dispatch_runtime_event(event_type, message='', payload=None, detail=None):
    fsm_payload = dict(payload or {})
    if isinstance(detail, dict):
        fsm_payload.update(detail)
    elif detail is not None:
        fsm_payload['detail'] = detail

    fsm_state = robot_lifecycle_fsm.apply_event(
        event_type or 'STATE_UPDATED',
        message or '',
        fsm_payload,
    )
    if fsm_state.get("transitionAccepted") is False:
        legacy_fields = legacy_fields_for_state(fsm_state)
        runtime_detail = fsm_state.get('detail')
        if not isinstance(runtime_detail, dict):
            runtime_detail = {}
        runtime_state = build_runtime_state_snapshot(
            control_state=fsm_state.get('controlState'),
            health_state=fsm_state.get('healthState'),
            fault_state=fsm_state.get('faultState'),
            mission=legacy_fields.get('mission'),
            parking=legacy_fields.get('parking'),
            action=legacy_fields.get('currentAction'),
            task_name=_decode_redis_value(redis_cli.get('currentTaskName')),
            task_index=_coerce_int(redis_cli.get('curTaskIndex'), None),
            start_ready=fsm_state.get('startReady'),
            message=fsm_state.get('message') or message or '',
            detail=runtime_detail,
            now=time.time(),
        )
        _publish_runtime_event(
            'FSM_TRANSITION_REJECTED',
            runtime_state.get('message') or 'FSM transition rejected',
            {
                'rejectedEvent': fsm_state.get('rejectedEvent'),
                'requestedControlState': fsm_state.get('requestedControlState'),
                'previousControlState': fsm_state.get('previousControlState'),
                'runtimeState': runtime_state,
            },
            fsm_state=fsm_state,
        )
        return runtime_state

    legacy_fields = legacy_fields_for_state(fsm_state)
    _mirror_runtime_state_to_redis(fsm_state, legacy_fields)
    # UI telemetry receives accepted state transitions only. Enqueueing does
    # not compress/write files or send a hardware command on this thread.
    position_observer = globals().get('_notify_cleaning_position_state')
    if callable(position_observer):
        position_observer(fsm_state.get('controlState'))

    runtime_detail = fsm_state.get('detail')
    if not isinstance(runtime_detail, dict):
        runtime_detail = {}
    _set_redis_value('runtimeDetail', runtime_detail)
    runtime_state = build_runtime_state_snapshot(
        control_state=fsm_state.get('controlState'),
        health_state=fsm_state.get('healthState'),
        fault_state=fsm_state.get('faultState'),
        mission=legacy_fields.get('mission'),
        parking=legacy_fields.get('parking'),
        action=legacy_fields.get('currentAction'),
        task_name=_decode_redis_value(redis_cli.get('currentTaskName')),
        task_index=_coerce_int(redis_cli.get('curTaskIndex'), None),
        start_ready=fsm_state.get('startReady'),
        message=fsm_state.get('message') or message or '',
        detail=runtime_detail,
        now=time.time(),
    )
    _set_redis_value(RUNTIME_STATE_KEY, runtime_state)

    event_payload = dict(fsm_payload)
    event_payload['runtimeState'] = runtime_state
    _publish_runtime_event(
        event_type or 'STATE_UPDATED',
        runtime_state.get('message') or '',
        event_payload,
        fsm_state=fsm_state,
    )
    return runtime_state


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
                       start_ready=None, start_reason=None, detail=None,
                       event_type=None, event_payload=None):
    payload = dict(event_payload or {})
    if fault_state is not None:
        payload['faultState'] = fault_state
    if health_state is not None:
        payload['healthState'] = health_state
    if start_ready is not None:
        payload['startReady'] = start_ready
    if event_type is None and control_state is not None:
        event_type = runtime_event_type_for_control_state(
            control_state,
            fault_state,
        )
    return dispatch_runtime_event(
        event_type or 'STATE_UPDATED',
        start_reason or '',
        payload,
        detail=detail,
    )


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


def _build_task_origin_status_fields(task_params):
    start_lat = _coerce_float(task_params.get('startLat'), None)
    start_lon = _coerce_float(task_params.get('startLon'), None)
    distance_to_task_origin = _distance_to_task_start(task_params)
    is_at_task_origin = None
    if distance_to_task_origin is not None:
        is_at_task_origin = distance_to_task_origin <= TASK_ORIGIN_TOLERANCE_METERS
    return {
        'taskOrigin': {
            'lat': start_lat,
            'lon': start_lon,
        },
        'currentLocation': {
            'lat': global_cur_rtk_lat,
            'lon': global_cur_rtk_lon,
            'heading': live_heading_from_location(global_cur_rtk_lat, global_cur_rtk_lon, global_cur_rtk_heading),
        },
        'distanceToTaskOriginM': distance_to_task_origin,
        'taskOriginToleranceM': TASK_ORIGIN_TOLERANCE_METERS,
        'isAtTaskOrigin': is_at_task_origin,
    }


def _build_task_origin_check_result(task_params, detail):
    task_origin_status = _build_task_origin_status_fields(task_params)
    detail.update(task_origin_status)
    distance_to_task_origin = task_origin_status.get('distanceToTaskOriginM')
    if distance_to_task_origin is None:
        return {
            'success': False,
            'faultState': 'TASK_ORIGIN_UNKNOWN',
            'message': '无法计算当前位置与任务起点距离，拒绝启动',
            'data': detail,
        }
    if not task_origin_status.get('isAtTaskOrigin'):
        return {
            'success': False,
            'faultState': 'NOT_AT_TASK_ORIGIN',
            'message': '当前位置距离任务起点 {:.2f} 米，超过允许范围 {:.2f} 米'.format(
                distance_to_task_origin, TASK_ORIGIN_TOLERANCE_METERS
            ),
            'data': detail,
        }
    return {
        'success': True,
        'message': '当前位置已在任务起点范围内',
        'data': detail,
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


def _get_rtk_runtime_status():
    try:
        runtime = get_shared_runtime(logger)
        if runtime is not None and hasattr(runtime, 'get_status'):
            return runtime.get_status()
    except Exception as e:
        logger.warning("get RTK runtime status failed: {}".format(e))
    return {}


def _is_rtk_fixed_status(status):
    quality = status.get('rtkQuality')
    gga_age = _coerce_float(status.get('rtkGgaAgeSec'), None)
    if quality is None or str(quality) != RTK_FIXED_QUALITY:
        return False
    return gga_age is not None and gga_age <= RTK_FIXED_GGA_MAX_AGE_SECONDS


def _rtk_fix_problem_reason(status):
    gga_age = _coerce_float(status.get('rtkGgaAgeSec'), None)
    quality = status.get('rtkQuality')
    if gga_age is None:
        return 'NO_RTK_GGA'
    if gga_age > RTK_FIXED_GGA_MAX_AGE_SECONDS:
        return 'RTK_GGA_TIMEOUT'
    if quality is None or str(quality) != RTK_FIXED_QUALITY:
        return 'RTK_NOT_FIXED'
    return ''


def _build_rtk_runtime_detail(status=None):
    status = dict(status or _get_rtk_runtime_status())
    fixed_available = _is_rtk_fixed_status(status)
    reason = '' if fixed_available else _rtk_fix_problem_reason(status)
    lost_seconds = None
    if global_rtk_fix_lost_time:
        lost_seconds = round(max(0.0, time.time() - global_rtk_fix_lost_time), 1)
    status.update({
        'rtkFixAvailable': fixed_available,
        'rtkFixState': 'FIXED' if fixed_available else reason,
        'rtkFixedQuality': RTK_FIXED_QUALITY,
        'rtkGgaMaxAgeSec': RTK_FIXED_GGA_MAX_AGE_SECONDS,
        'rtkRecovering': bool(global_rtk_recovering),
        'rtkLostAt': int(global_rtk_fix_lost_time) if global_rtk_fix_lost_time else None,
        'rtkLostSeconds': lost_seconds,
        'rtkRecoveryTimeoutSec': RTK_FIX_RECOVERY_TIMEOUT_SECONDS,
    })
    return status


def _build_runtime_detail(extra=None):
    task_params = _load_task_params_snapshot()
    task_origin_status = _build_task_origin_status_fields(task_params)
    detail = _load_runtime_detail()
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
        'startToleranceM': START_POSITION_TOLERANCE_METERS,
        'currentLat': global_cur_rtk_lat,
        'currentLon': global_cur_rtk_lon,
        'currentHeading': live_heading_from_location(global_cur_rtk_lat, global_cur_rtk_lon, global_cur_rtk_heading),
        'taskStartLat': _coerce_float(task_params.get('startLat'), None),
        'taskStartLon': _coerce_float(task_params.get('startLon'), None),
        'originHeading': _coerce_float(task_params.get('originHeading'), None),
        'distanceToTaskOriginM': task_origin_status.get('distanceToTaskOriginM'),
        'taskOriginToleranceM': task_origin_status.get('taskOriginToleranceM'),
        'isAtTaskOrigin': task_origin_status.get('isAtTaskOrigin'),
    })
    detail.update(task_origin_status)
    detail.update(_build_rtk_runtime_detail())
    if extra:
        detail.update(extra)
    return detail

def _initialization_detail(phase, initializing=True, extra=None):
    detail = {
        'initializing': bool(initializing),
        'initializationPhase': phase,
    }
    if extra:
        detail.update(extra)
    return detail


def _mark_runtime_initializing(message, phase='startup', extra=None):
    _set_runtime_state(
        control_state='INITIALIZING',
        health_state='OK',
        fault_state='',
        start_ready=False,
        start_reason=message,
        detail=_initialization_detail(phase, True, extra)
    )


def _mark_runtime_initialized(message, extra=None):
    _set_runtime_state(
        control_state='STOPPED',
        health_state='OK',
        fault_state='',
        start_ready=False,
        start_reason=message,
        detail=_build_runtime_detail(_initialization_detail('complete', False, extra))
    )


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


def _mark_runtime_complete(message, extra=None):
    _set_runtime_state(
        control_state='COMPLETE',
        health_state='OK',
        fault_state='',
        start_ready=False,
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


def _mark_runtime_rtk_recovering(message, extra=None):
    _set_runtime_state(
        control_state='PAUSED',
        health_state='WARN',
        fault_state='RTK_FIX_LOST',
        start_ready=False,
        start_reason=message,
        detail=_build_runtime_detail(extra)
    )


def _clear_runtime_task_state(reason, clear_auto_task=False, clear_waypoints=False,
                              update_runtime=False, message=None):
    if clear_auto_task:
        redis_cli.delete('taskList')
    if clear_waypoints:
        redis_cli.delete('waypoints')

    redis_cli.set('action', 'false')
    redis_cli.set('correct', 'false')
    redis_cli.set('moveJudge', 'false')
    redis_cli.set('reverse', 'false')
    redis_cli.set('enterGarage', 'false')
    redis_cli.set('exitGarage', 'false')
    redis_cli.set('waypointIndex', 0)
    redis_cli.set('waypointTotal', 0)
    redis_cli.set(WAYPOINT_LOOP_ENABLED_KEY, '0')
    redis_cli.set(WAYPOINT_LOOP_MODE_KEY, 'count')
    redis_cli.set(WAYPOINT_LOOP_TARGET_KEY, 0)
    redis_cli.set(WAYPOINT_LOOP_CURRENT_KEY, 0)
    redis_cli.delete('waypointLoopProgress')
    _set_redis_value('runtimeDetail', {})

    detail = {
        'runtimeTaskType': 'none',
        'lastRuntimeClearReason': reason or '',
    }
    clear_message = message or 'runtime task state cleared'
    if update_runtime:
        _mark_runtime_idle(clear_message, detail)
    else:
        _set_runtime_state(
            event_type='STATE_UPDATED',
            start_reason=clear_message,
            detail=_build_runtime_detail(detail)
        )


def _request_runtime_stop(reason, clear_auto_task=False, clear_waypoints=False,
                          update_runtime=True, message=None):
    global global_auto_clean_stop, global_waypoint_nav_stop, global_loop_auto_clean_stop
    global global_doCleanThreadStop, global_pointToPoint_flag, global_go, global_status
    global active_runtime_task_token, active_runtime_task_action

    # Every explicit runtime stop cancels old tap/hold timers and records a
    # stopped intent. No forward/reverse frame is emitted during takeover.
    _reset_manual_steering(send_hardware=False, commanded_motion='stopped')
    global_auto_clean_stop = 1
    global_waypoint_nav_stop = 1
    global_loop_auto_clean_stop = 1
    global_doCleanThreadStop = 1
    global_pointToPoint_flag = 1
    global_go = 0
    active_runtime_task_token = ''
    active_runtime_task_action = ''
    _disable_loop_auto_clean(reason or 'runtime_stop')
    _set_auto_resume_allowed(False, reason or 'runtime_stop')
    _publish_global_go(global_go)
    try:
        sendBraking()
    except Exception as exc:
        logger.error("runtime stop brake failed: {}".format(exc), exc_info=True)
    global_status = 'active'
    redis_cli.set('curTaskIndex', 0)
    _clear_runtime_task_state(
        reason,
        clear_auto_task=clear_auto_task,
        clear_waypoints=clear_waypoints,
        update_runtime=update_runtime,
        message=message or 'runtime task stopped'
    )


def _maybe_brake_on_power_enable(previous_power_on_state, current_power_on_state):
    global global_power_on_guard_sent
    if current_power_on_state != 1 or previous_power_on_state == 1:
        return
    if global_power_on_guard_sent:
        return

    current_action = _runtime_action()
    if _is_runtime_task_active() and current_action in ('auto_drive', 'go_on', 'return_to_point'):
        return

    global_power_on_guard_sent = True
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


def _frame_i16_to_int(data, index):
    """Decode the signed speed fields returned by the lower machine."""
    value = _frame_u16_to_int(data, index)
    if value is None:
        return None
    return value - 0x10000 if value >= 0x8000 else value


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
        redis_cli.set("lowerMachineStatus", status)
        
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
        x_speed = _frame_i16_to_int(data, 4)
        if x_speed is not None:
            global_get_XSpeed = x_speed
            redis_cli.set("xSpeed", x_speed)

    if frame_len > 7:
        z_speed = _frame_i16_to_int(data, 6)
        if z_speed is not None:
            global_get_ZSpeed = z_speed
            redis_cli.set("zSpeed", z_speed)

    brush_speed = byte_at(8)
    if brush_speed is not None:
        global_get_brushSpeed = brush_speed
        redis_cli.set("brushSpeedActual", brush_speed)

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
    # 先消费已经缓存的串口数据；如果缓存里已经拼出完整状态帧，就不用再读串口。
    if _drain_lower_machine_rx_buffer(source):
        return True

    # 本次读取最多等待 wait_seconds 秒，避免业务线程长时间阻塞。
    deadline = time.time() + wait_seconds
    while time.time() < deadline:
        # 获取下位机串口对象；串口未打开时直接返回失败，由外层循环下次再尝试。
        port = _get_lower_machine_serial()
        if port is None:
            time.sleep(0.05)
            return False

        try:
            # 串口读取加锁，防止 listenerSlavePort/getEdge/getRotateArrive 等多个线程同时读串口导致帧被拆乱。
            with LOWER_MACHINE_READ_LOCK:
                # 优先读取串口缓冲区里已有的全部字节；没有可读字节时至少读 1 个字节。
                waiting = _serial_in_waiting(port)
                read_len = waiting if waiting > 0 else 1
                # 限制单次读取长度，避免异常数据把接收缓存撑得过大。
                read_len = min(max(read_len, 1), LOWER_MACHINE_RX_BUFFER_LIMIT)
                # 真正从下位机串口读取原始字节数据。
                data = port.read(read_len)
        except serial.serialutil.SerialException as exc:
            # 串口异常通常表示设备断开或句柄失效，重置串口后继续等待下一轮读取。
            _reset_lower_machine_serial(exc)
            time.sleep(0.05)
            continue
        except Exception as exc:
            # 其他异常只记录日志，不让监听线程退出。
            logger.warning("{} read lower-machine serial failed: {}".format(source, exc), exc_info=True)
            time.sleep(0.02)
            continue

        if data:
            # 串口可能一次只返回半帧数据，所以先追加到接收缓存。
            _append_lower_machine_rx_data(data)
            # 追加后尝试从缓存中切出完整状态帧，并交给 _apply_lower_machine_status_frame() 解析。
            if _drain_lower_machine_rx_buffer(source):
                return True
        else:
            # 没读到数据时短暂休眠，避免空循环占满 CPU。
            time.sleep(0.02)

    # 超过等待时间仍没有解析出有效下位机状态帧，返回失败。
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


def _get_waypoint_count(prefer_runtime_total=False):
    if prefer_runtime_total:
        runtime_total = _coerce_int(redis_cli.get('waypointTotal'), 0)
        if runtime_total > 0:
            return runtime_total
    try:
        count = int(redis_cli.llen('waypoints'))
        if count > 0:
            return count
    except Exception:
        pass
    return _coerce_int(redis_cli.get('waypointTotal'), 0)


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
    fsm_state = robot_lifecycle_fsm.get_state()
    return str(fsm_state.get('controlState') or 'UNKNOWN').upper()


def _is_ignored_enable_fault_state(fault_state):
    return fault_state in ('LOWER_MACHINE_DISABLED', 'LOWER_MACHINE_STATUS_UNKNOWN')


def _derive_health_state():
    fsm_state = robot_lifecycle_fsm.get_state()
    health_state = str(fsm_state.get('healthState') or '').upper()
    fault_state = str(fsm_state.get('faultState') or '')
    if health_state:
        return health_state
    if fault_state and not _is_ignored_enable_fault_state(fault_state):
        return 'WARN'
    return 'OK'


def _derive_fault_state():
    fsm_state = robot_lifecycle_fsm.get_state()
    fault_state = str(fsm_state.get('faultState') or '')
    if fault_state and not _is_ignored_enable_fault_state(fault_state):
        return fault_state
    return ''


def _derive_mission_state(control_state):
    if control_state in ('BLOCKED', 'DISABLED', 'UNKNOWN'):
        return control_state
    fsm_state = robot_lifecycle_fsm.get_state()
    current_action = str(fsm_state.get('action') or '')
    if current_action == 'return_to_point':
        return 'RETURNING'
    if control_state in ('RUNNING', 'PAUSED', 'STOPPING'):
        return 'RUNNING'
    if control_state == 'STOPPED':
        return 'STOPPED'
    if control_state == 'COMPLETE':
        return 'COMPLETE'
    return 'IDLE'


def _derive_status(control_state, mission_state):
    if control_state == 'UNKNOWN':
        return 'unknown'
    if control_state == 'BLOCKED':
        return 'blocked'
    if control_state == 'DISABLED':
        return 'idle'
    if mission_state == 'RUNNING':
        return 'working'
    if mission_state == 'RETURNING':
        return 'returning'
    if control_state in ('STOPPED', 'COMPLETE'):
        return 'idle'
    return 'active'


def _build_vehicle_status_payload():
    task_params = _load_task_params_snapshot()
    lat = global_cur_rtk_lat
    lon = global_cur_rtk_lon
    heading = live_heading_from_location(lat, lon, global_cur_rtk_heading)
    local_x, local_y = _compute_local_xy_cm(lat, lon, task_params)
    control_state = _derive_control_state()
    mission_state = _derive_mission_state(control_state)
    fault_state = _derive_fault_state()
    current_action = _runtime_action() or 'idle'
    garage_state = get_garage_state()
    hardware_report_at = _get_hardware_report_at()
    battery_percent = _clamp_percent(redis_cli.get('batteryPercent'))
    battery_percent_raw = _clamp_percent(redis_cli.get('batteryPercentRaw'))
    loop_auto_clean = _get_loop_auto_clean_status()
    task_origin_status = _build_task_origin_status_fields(task_params)
    clean_task_count = len(_load_task_items_for_preview())
    waypoint_count = _get_waypoint_count(current_action == 'multi_go_to_point')
    configured_task_name = _decode_redis_value(redis_cli.get('currentTaskName')) or ''
    active_task_name = visible_task_field(configured_task_name, current_action, control_state)
    active_task_index = visible_task_field(_coerce_int(redis_cli.get('curTaskIndex'), None), current_action, control_state)
    active_task_count = visible_task_field(clean_task_count if clean_task_count > 0 else None, current_action, control_state)
    command_speed = _coerce_int(redis_cli.get('forwardSpeed'), None)
    command_brush_speed = _coerce_int(redis_cli.get('brushSpeed'), None)
    live_speed = live_value_from_report(_coerce_int(global_get_XSpeed, None), hardware_report_at)
    live_brush_speed = live_value_from_report(_coerce_int(global_get_brushSpeed, None), hardware_report_at)
    manual_status = _manual_steering_status_snapshot(live_speed, control_state, fault_state)

    detail = _build_runtime_detail({
        'lastCommandMessage': _decode_redis_value(redis_cli.get('lastCommandMessage')) or '',
        'startCheckReady': _coerce_bool(redis_cli.get('startCheckReady'), False),
        'startCheckReason': _decode_redis_value(redis_cli.get('startCheckReason')) or '',
        'garageStateReason': _decode_redis_value(redis_cli.get(GARAGE_STATE_REASON_KEY)) or '',
        'cleanTaskCount': active_task_count,
        'configuredTaskName': configured_task_name,
        'configuredCleanTaskCount': clean_task_count if clean_task_count > 0 else None,
        'commandSpeed': command_speed,
        'commandBrushSpeed': command_brush_speed,
        'waypointCount': waypoint_count,
        'waypointIndex': _coerce_int(redis_cli.get('waypointIndex'), 0),
    })

    payload = {
        'status': _derive_status(control_state, mission_state),
        'battery': battery_percent,
        'battery_percent': battery_percent,
        'battery_raw': battery_percent_raw,
        'battery_percent_raw': battery_percent_raw,
        'action': current_action,
        'task_name': active_task_name,
        'cur_task_index': active_task_index,
        'task_count': active_task_count,
        'clean_task_count': active_task_count,
        'waypoint_index': _coerce_int(redis_cli.get('waypointIndex'), 0),
        'waypoint_count': waypoint_count,
        'online_state': 'ONLINE',
        'mission_state': mission_state,
        'control_state': control_state,
        'health_state': _derive_health_state(),
        'fault_state': fault_state,
        'speed': live_speed,
        'xSpeed': live_speed,
        'brush_speed': live_brush_speed,
        'command_speed': command_speed,
        'command_brush_speed': command_brush_speed,
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
        'waypointLoopEnabled': _coerce_bool(redis_cli.get('waypointLoopEnabled'), False),
        'waypointLoopMode': _decode_redis_value(redis_cli.get('waypointLoopMode')) or 'count',
        'waypointLoopTarget': _coerce_int(redis_cli.get('waypointLoopTarget'), 0),
        'waypointLoopCurrent': _coerce_int(redis_cli.get('waypointLoopCurrent'), 0),
        'waypointLoopProgress': {
            "loopMode": _decode_redis_value(redis_cli.get('waypointLoopMode')) or 'count',
            "currentLoop": _coerce_int(redis_cli.get('waypointLoopCurrent'), 0),
            "targetLoop": _coerce_int(redis_cli.get('waypointLoopTarget'), 0),
        },
        'supported_actions': ['auto_drive', 'go_on', 'stop', 'parking', 'manual_steering', 'return_to_point', 'go_to_point', 'multi_go_to_point', 'get_status', 'get_task_path'],
        'supported_params': ['taskName', 'speed', 'tracking', 'path'],
        'supported_status_fields': [
            'control_state', 'health_state', 'fault_state', 'detail',
            'mission_state', 'garage_state', 'loop_auto_clean', 'taskOrigin',
            'xSpeed', 'motionState', 'manualSteeringAllowed',
            'manualSteeringMode', 'manualSteeringDirection',
            'manualCorrectionValue', 'manualCorrectionLevel',
        ],
        'detail': detail,
        'timestamp': int(time.time()),
    }
    payload.update(task_origin_status)
    payload.update(manual_status)
    return payload


def _validate_auto_drive_request_legacy():
    task_params = _load_task_params_snapshot()
    start_lat = _coerce_float(task_params.get('startLat'), None)
    start_lon = _coerce_float(task_params.get('startLon'), None)
    origin_heading = _coerce_float(task_params.get('originHeading'), None)
    detail = _build_runtime_detail()

    if _is_runtime_task_active():
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

    task_origin_check = _build_task_origin_check_result(task_params, detail)
    if not task_origin_check.get('success'):
        return task_origin_check

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

    task_items = [True]
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

    if _is_runtime_task_active():
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

    task_origin_check = _build_task_origin_check_result(task_params, detail)
    if not task_origin_check.get('success'):
        return task_origin_check

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

def _can_access_serial_port(port):
    return bool(port) and os.path.exists(port) and os.access(port, os.R_OK | os.W_OK)


def _resolve_lower_machine_port(_rtk_port):
    env_port = os.getenv("CLEANER_LOWER_MACHINE_PORT")
    preferred = env_port or "/dev/ttyACM0"

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
xwj_port = "/dev/ttyACM0"
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
            redis_cli.set("xSpeed", global_get_XSpeed)
        elif i == 6:
            global_get_ZSpeed = int(binascii.b2a_hex(data[i] + data[i + 1]), 16)
        elif i == 8:
            global_get_brushSpeed = int(binascii.b2a_hex(data[i]), 16)
            redis_cli.set("brushSpeedActual", global_get_brushSpeed)
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
        while _is_runtime_task_active():
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
        while _is_runtime_task_active():
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
        while _is_runtime_task_active():
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


cap = _open_camera_capture()
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


@app.route("/dev/overview/state", methods=['GET'])
def dev_overview_state():
    return jsonify(build_overview_state(redis_cli))


@app.route("/dev/correction/state", methods=['GET'])
def dev_correction_state():
    return jsonify(build_correction_state(
        redis_cli,
        config_path="config.json",
        trace=dev_console_trace,
    ))


@app.route("/dev/task-path", methods=['GET'])
def dev_task_path_state():
    return jsonify(build_task_path_state(redis_cli, "config.json"))


@app.route("/dev/redis/state", methods=['GET'])
def dev_redis_state():
    return jsonify(build_redis_state(redis_cli))


@app.route("/dev/logs", methods=['GET'])
def dev_logs_state():
    query = request.args.get("query", "")
    limit = request.args.get("limit", "200")
    return jsonify(read_log_lines('app.log', query=query, limit=limit))


def _read_modeling_rtk_snapshot():
    rtk_detail = _build_rtk_runtime_detail()
    fsm_state = robot_lifecycle_fsm.get_state()
    hardware_report_at = _get_hardware_report_at()
    live_speed = live_value_from_report(_coerce_int(global_get_XSpeed, None), hardware_report_at)
    return {
        "lat": global_cur_rtk_lat,
        "lon": global_cur_rtk_lon,
        "heading": live_heading_from_location(global_cur_rtk_lat, global_cur_rtk_lon, global_cur_rtk_heading),
        "rtkQuality": rtk_detail.get("rtkQuality"),
        "rtkGgaAgeSec": rtk_detail.get("rtkGgaAgeSec"),
        "rtkFixAvailable": rtk_detail.get("rtkFixAvailable"),
        "rtkFixState": rtk_detail.get("rtkFixState"),
        "controlState": fsm_state.get("controlState"),
        "action": fsm_state.get("action"),
        "xSpeed": live_speed,
        "moving": live_speed is not None and abs(live_speed) > 0.01,
    }


def _sample_modeling_current_point():
    sample_count = _coerce_int(os.environ.get("MODELING_SAMPLE_COUNT"), 10)
    max_radius_m = _coerce_float(os.environ.get("MODELING_SAMPLE_MAX_RADIUS_M"), 0.05)
    sleep_seconds = _coerce_float(os.environ.get("MODELING_SAMPLE_INTERVAL_SEC"), 0.05)
    return sample_current_point(
        _read_modeling_rtk_snapshot,
        sample_count=sample_count,
        max_radius_m=max_radius_m,
        sleep_seconds=sleep_seconds,
    )


def _set_modeling_task_progress(state):
    state = dict(state or {})
    _set_redis_value('modelingTaskProgress', state)
    _set_redis_value('modelingTaskStatus', state.get('status'))
    _set_redis_value('modelingTaskIndex', state.get('currentIndex'))
    _set_redis_value('modelingTaskTotal', state.get('total'))
    _set_redis_value('runtimeDetail', _build_runtime_detail({'modelingTaskProgress': state}))


def _read_modeling_task_progress(payload=None):
    payload = payload if isinstance(payload, dict) else {}
    model_id = payload.get('modelId')
    raw = _decode_redis_value(redis_cli.get('modelingTaskProgress'))
    progress = None
    if raw:
        try:
            progress = json.loads(raw) if isinstance(raw, str) else raw
        except Exception:
            progress = None
    if not isinstance(progress, dict):
        return {
            'available': False,
            'modelId': model_id,
            'progress': None,
        }
    if model_id and str(progress.get('modelId') or '') != str(model_id):
        return {
            'available': False,
            'modelId': model_id,
            'progress': None,
        }
    return {
        'available': True,
        'modelId': progress.get('modelId'),
        'progress': progress,
    }


def _validate_modeling_task_start(execution_plan):
    if not _can_start_runtime_task():
        fsm_state = robot_lifecycle_fsm.get_state()
        raise ModelingExecutionError(
            'runtime is not startable: {}'.format(fsm_state.get('controlState') or '')
        )
    rtk_detail = _build_rtk_runtime_detail()
    if not rtk_detail.get('rtkFixAvailable'):
        raise ModelingExecutionError(
            'RTK fixed solution is required before starting modeling task: {}'.format(
                rtk_detail.get('rtkFixState') or 'RTK_NOT_READY'
            )
        )
    segments = execution_plan.get('segments') if isinstance(execution_plan, dict) else None
    if not isinstance(segments, list) or not segments:
        raise ModelingExecutionError('modeling execution plan has no segments')
    first_segment = segments[0]
    try:
        origin_check = validate_route_start(
            global_cur_rtk_lat,
            global_cur_rtk_lon,
            first_segment.get('startLat'),
            first_segment.get('startLon'),
            TASK_ORIGIN_TOLERANCE_METERS,
            util.get_distance_angle,
        )
    except RouteStartGuardError as error:
        raise ModelingExecutionError('{}: {}'.format(error.code, error.message))
    result = {
        'rtkFixAvailable': True,
        'rtkFixState': rtk_detail.get('rtkFixState'),
        'rtkQuality': rtk_detail.get('rtkQuality'),
        'rtkGgaAgeSec': rtk_detail.get('rtkGgaAgeSec'),
    }
    result.update(origin_check)
    return result


def _modeling_execution_speed(payload):
    payload = payload if isinstance(payload, dict) else {}
    speed = _coerce_int(payload.get('speed'), None)
    if speed is None:
        speed = _coerce_int(redis_cli.get("forwardSpeed"), None)
    if speed is None or speed <= 0:
        raise ModelingExecutionError("valid forwardSpeed is required before starting modeling task")
    return speed


def _start_modeling_task_runtime(model_id, draft, payload):
    task_plan = draft.get('taskPlan') if isinstance(draft, dict) else None
    speed = _modeling_execution_speed(payload)
    execution_plan = build_execution_plan(model_id, task_plan, speed=speed, now=time.time())
    preflight = _validate_modeling_task_start(execution_plan)
    _set_modeling_task_progress({
        'status': 'starting',
        'action': 'modeling_task',
        'modelId': execution_plan.get('modelId'),
        'currentIndex': 0,
        'total': execution_plan.get('taskCount'),
        'message': 'modeling task thread starting',
        'preflight': preflight,
    })
    thread, error_payload = _start_runtime_thread(
        'modeling_task',
        _modelingTaskThread,
        args=(execution_plan,),
        ready_message='正在创建建模任务执行线程',
        detail={'modelingTask': execution_plan, 'modelingTaskPreflight': preflight},
    )
    if error_payload:
        raise ModelingExecutionError(error_payload.get('msg') or error_payload.get('code') or 'runtime not startable')
    return {
        'status': 'starting',
        'action': 'modeling_task',
        'modelId': execution_plan.get('modelId'),
        'taskCount': execution_plan.get('taskCount'),
        'speed': execution_plan.get('speed'),
        'preflight': preflight,
    }


def _stop_modeling_task_runtime(payload=None):
    payload = payload if isinstance(payload, dict) else {}
    fsm_state = robot_lifecycle_fsm.get_state()
    action = str(fsm_state.get('action') or '')
    if action != 'modeling_task' or not _is_runtime_task_active():
        raise ModelingExecutionError('modeling task is not running')
    progress_state = _read_modeling_task_progress(payload).get('progress') or {}
    stop_state = dict(progress_state)
    stop_state.update({
        'status': 'stopping',
        'action': 'modeling_task',
        'modelId': payload.get('modelId') or progress_state.get('modelId'),
        'message': 'modeling task stop requested',
        'requestedStopAt': int(time.time()),
    })
    _set_modeling_task_progress(stop_state)
    doParking(update_runtime=True, message='已请求停止建模任务')
    return stop_state


def _run_task_segment_by_point_navigation(segment, speed, source, segment_index):
    """
    使用原有点位导航执行一段规划路线。

    保存任务里的 heading 只作为展示数据，不直接用于控制转向。每一段都读取此刻的
    RTK 位置，再由 pointToPointByRTKAutoHeading 计算到终点的真实方向。转向阶段关闭
    清扫，到达正确朝向后才根据 mode 决定是否开启清扫；到点后停车并关闭清扫。
    """
    def read_position():
        # 读取RTK线程持续更新的全局实时位置，作为当前任务段的真实起点。
        return global_cur_rtk_lat, global_cur_rtk_lon

    def navigate(current_lat, current_lon, end_lat, end_lon, runtime_speed, before_drive):
        continuous_segments = list(segment.get(CONTINUATION_KEY) or [])
        polyline_points = build_continuous_polyline(segment, continuous_segments)
        # 相同continuousPathId的多个记录点作为一整条折线执行。普通中间点仍完整
        # 保留在polyline_points里，但不再逐点停止、重置滤波器或直接瞄准十几厘米外
        # 的短目标；observer_go_correct会沿折线投影并选择受走廊约束的前视点。
        if continuous_segments and len(polyline_points) >= 2:
            final_point = polyline_points[-1]
            if segment.get('turnAtStart') is False:
                guidance = compute_polyline_guidance(
                    polyline_points,
                    current_lat,
                    current_lon,
                )
                if not guidance:
                    return 0
                logger.warn(
                    "start continuous polyline without stop-turn: taskId={}, points={}, heading={:.3f}, lookahead={:.3f}".format(
                        segment.get('id'),
                        len(polyline_points),
                        float(guidance['heading']),
                        float(guidance['lookaheadM']),
                    )
                )
                if callable(before_drive):
                    before_drive()
                return pointToPointByRTK(
                    current_lat,
                    current_lon,
                    final_point['lat'],
                    final_point['lon'],
                    guidance['heading'],
                    runtime_speed,
                    polyline_points=polyline_points,
                )
            return pointToPointByRTKAutoHeading(
                current_lat,
                current_lon,
                final_point['lat'],
                final_point['lon'],
                runtime_speed,
                before_drive=before_drive,
                source=source,
                segment_index=segment_index,
                task_id=segment.get('id'),
                polyline_points=polyline_points,
            )

        # 非连续任务保持原有安全流程。历史任务若显式标记turnAtStart=False，仍不
        # 原地转向，但这种单段任务的终点较远，不涉及短点位放大航向的问题。
        if segment.get('turnAtStart') is False:
            distance, heading = util.get_distance_angle(
                current_lat,
                current_lon,
                end_lat,
                end_lon,
            )
            logger.warn(
                "go to continuous boundary point: taskId={}, distance={:.3f}, heading={:.3f}".format(
                    segment.get('id'),
                    float(distance),
                    float(heading),
                )
            )
            if callable(before_drive):
                # 连续清扫子段会再次确认mode=1清扫状态，但不会关闭后重开滚刷。
                before_drive()
            # 直接更新点到点终点并继续直线控制，不调用turn()、不主动刹车。
            return pointToPointByRTK(
                current_lat,
                current_lon,
                end_lat,
                end_lon,
                heading,
                runtime_speed,
            )
        # 普通任务、边界首段、30°以上硬拐点和折线终点走完整安全流程：
        # 实时算航向 -> 原地转向 -> before_drive开/关滚刷 -> 点到点直行。
        return pointToPointByRTKAutoHeading(
            current_lat,
            current_lon,
            end_lat,
            end_lon,
            runtime_speed,
            before_drive=before_drive,
            source=source,
            segment_index=segment_index,
            task_id=segment.get('id'),
        )

    def set_cleaning(enabled):
        if enabled:
            switch_on_clean_mode(ser)
        else:
            switch_off_clean_mode(ser)

    def stop_vehicle():
        sendBraking()

    def report_error(error):
        logger.warn('[{}] task segment failed: taskId={}, error={}'.format(
            source,
            segment.get('id'),
            error,
        ))

    return run_route_segment(
        segment,
        speed,
        read_position,
        navigate,
        set_cleaning,
        stop_vehicle,
        on_error=report_error,
    )


def _run_modeling_task_segment(segment):
    return _run_task_segment_by_point_navigation(
        segment,
        int(segment.get('speed')),
        'modeling_task',
        int(segment.get('index') or 0) + 1,
    )


def _modelingTaskThread(task_token=None, execution_plan=None):
    if execution_plan is None and isinstance(task_token, dict):
        execution_plan = task_token
        task_token = None
    global global_status, global_go, global_doCleanThreadStop, global_pointToPoint_flag
    global_status = 'working'
    global_auto_clean_stop = 0
    global_doCleanThreadStop = 0
    global_pointToPoint_flag = 0
    redis_cli.set("correct", "true")
    _begin_cleaning_position_run({
        'taskName': None, 'modelId': execution_plan.get('modelId'),
        'taskList': execution_plan.get('segments') or [],
    }, task_token)
    _mark_runtime_running('建模任务执行启动', {
        'action': 'modeling_task',
        'modelingTask': execution_plan,
    })
    try:
        result = execute_modeling_plan(
            execution_plan,
            run_segment=_run_modeling_task_segment,
            update_progress=_set_modeling_task_progress,
            should_stop=_is_runtime_stop_requested,
            now=time.time,
        )
        if result.get('status') == 'complete':
            _mark_runtime_complete('建模任务执行完成', {
                'action': 'modeling_task',
                'modelingTaskProgress': result,
            })
        elif result.get('status') == 'stopped':
            logger.warn('建模任务收到停止信号: {}'.format(result))
        else:
            _mark_runtime_blocked(
                result.get('code') or 'MODELING_TASK_BLOCKED',
                result.get('message') or '建模任务执行中断',
                {'modelingTaskProgress': result}
            )
    except Exception as e:
        logger.error("modeling task thread error: {}".format(traceback.format_exc()))
        _mark_runtime_blocked('MODELING_TASK_ERROR', '建模任务执行异常: {}'.format(str(e)))
    finally:
        global_go = 0
        global_doCleanThreadStop = 0
        _publish_global_go(global_go)
        redis_cli.set("correct", "false")
        switch_off_clean_mode(ser)
        doParking(update_runtime=False)
        global_status = 'active'


MODELING_STORE_DIR = os.environ.get("MODELING_STORE_DIR", os.path.join(os.getcwd(), "modeling_models"))


def _save_modeling_task(task_name, current_path):
    global taskList
    task_name = normalize_task_name(task_name)
    file_name = task_name + ".json"

    def register_task(task_config):
        redis_cli.sadd("taskNameSet", task_name)
        redis_cli.hset(
            "loc_start_lat_lon",
            task_name,
            json.dumps({
                "startLat": task_config.get("startLat"),
                "startLon": task_config.get("startLon"),
            }),
        )

    with TASK_SWITCH_LOCK:
        if os.path.exists(file_name):
            existing_config = _load_json_config(file_name)
            if is_same_named_task(existing_config, current_path, task_name):
                register_task(existing_config)
                taskList = []
                return {
                    "taskName": task_name,
                    "taskCount": len(existing_config.get("taskList") or []),
                    "modelId": existing_config.get("modelId"),
                    "recovered": True,
                }
            raise ModelingTaskPersistenceError(
                "TASK_NAME_EXISTS",
                "taskName already exists",
            )

        model_id = current_path.get("modelId")
        draft = modeling_store.get_draft(model_id)
        no_return_task_plan = generate_modeling_task_plan(
            draft,
            return_to_origin=False,
        )
        task_config = build_named_task(
            _load_json_config("config.json"),
            current_path,
            task_name,
            no_return_task_plan=no_return_task_plan,
        )
        _write_json_config(file_name, task_config)
        try:
            register_task(task_config)
        except Exception as error:
            logger.error(
                "register saved modeling task failed: taskName={}, error={}".format(
                    repr(task_name),
                    error,
                ),
                exc_info=True,
            )
            try:
                redis_cli.srem("taskNameSet", task_name)
                redis_cli.hdel("loc_start_lat_lon", task_name)
            except Exception:
                pass
            try:
                os.remove(file_name)
            except Exception as cleanup_error:
                logger.error(
                    "remove incomplete modeling task failed: file={}, error={}".format(
                        repr(file_name),
                        cleanup_error,
                    ),
                    exc_info=True,
                )
            raise
        taskList = []

    return {
        "taskName": task_name,
        "taskCount": len(task_config.get("taskList") or []),
        "modelId": current_path.get("modelId"),
    }


modeling_store = register_modeling_routes(app,
    storage_dir=MODELING_STORE_DIR,
    sample_point_provider=_sample_modeling_current_point,
    task_execution_starter=_start_modeling_task_runtime,
    task_progress_reader=_read_modeling_task_progress,
    task_stop_handler=_stop_modeling_task_runtime,
    task_save_handler=_save_modeling_task,
)
modeling_position_history = ModelingPositionHistory(MODELING_STORE_DIR)
cleaning_position_service = CleaningPositionService(
    CleaningPositionHistory(os.path.join(MODELING_STORE_DIR, 'cleaning_history', 'latest.json')),
    model_loader=modeling_store.get_model,
    on_error=lambda error: logger.warning('cleaning position telemetry: {}'.format(error)),
)


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


@app.route("/vehicle/isAtTaskOrigin", methods=['GET'])
def isAtTaskOrigin():
    task_params = _load_task_params_snapshot()
    detail = _build_runtime_detail()
    result = _build_task_origin_check_result(task_params, detail)
    return jsonify({
        'success': result.get('success', False),
        'code': 200 if result.get('success') else 409,
        'faultState': result.get('faultState', ''),
        'message': result.get('message', ''),
        'data': result.get('data', detail),
    })


@app.route("/vehicle/enterGarage", methods=['GET'])
def enterGarage():
    task, error_payload = _start_runtime_thread(
        'enter_garage',
        enter_garage_task,
        ready_message='正在创建进舱线程',
    )
    if error_payload:
        return jsonify(error_payload)

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


def enter_garage_task(task_token=None):
    _mark_runtime_running('手动固定距离进舱启动', {'action': 'enter_garage'})
    travel_length_cm = _get_task_enter_garage_length()
    arrived = _run_fixed_enter_garage(travel_length_cm)
    _mark_runtime_complete('手动固定距离进舱结束', {
        'action': 'enter_garage',
        'travelLengthCm': travel_length_cm,
        'arrived': arrived,
    })
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
    moveBack(ser, backLength)
    if str(getDistanceArrive()) == "0":
        set_garage_state(GARAGE_STATE_UNKNOWN, reason_prefix + '_not_arrived')
        return False
    set_garage_state(GARAGE_STATE_OUTSIDE, reason_prefix + '_completed')
    return True


@app.route("/vehicle/exitGarage", methods=['GET'])
def exitGarage():
    redis_cli.set("reverse", "false")
    task, error_payload = _start_runtime_thread(
        'exit_garage',
        exit_garage_task,
        ready_message='正在创建出舱线程',
    )
    if error_payload:
        return jsonify(error_payload)

    response = make_response("1")

    return response


def exit_garage_task(task_token=None):
    _mark_runtime_running('手动出舱启动', {'action': 'exit_garage'})
    redis_cli.set("reverse", "false")
    redis_cli.set("correct", "false")
    redis_cli.set('action', 'true')
    backLength = _get_task_exit_back_length()
    logger.warn("手动出库按任务距离后退: {}cm".format(backLength))
    _run_exit_garage_by_back_length(backLength, 'manual_exit_garage')
    redis_cli.set('action', 'false')
    redis_cli.set("correct", "false")
    _mark_runtime_complete('手动出舱结束', {'action': 'exit_garage'})


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

    logger.warn("legacy correctByRTKTest uses shared RTKDataManager observer stream")
    rtk_generator = ()
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
        if global_edge_trigger_latch.consume(getEdge()):
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
    thread, error_payload = _start_runtime_thread(
        'return_to_point',
        returnToPointByRTKThread,
        ready_message='正在创建 RTK 返回充电桩线程',
    )
    if error_payload:
        return jsonify(error_payload)
    response = make_response("开启自动执行任务")
    return response
def returnToPointByRTKThread(task_token=None):
    global global_go,global_status
    _mark_runtime_running('RTK返回充电桩启动', {'action': 'return_to_point'})
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
    if _is_runtime_stop_requested():
        doParking()
        return
    doParking(update_runtime=False)
    # 进充电桩
    intoGarage(backLength)
    _mark_runtime_complete('RTK返回充电桩结束', {'action': 'return_to_point'})
# 根据任务路线行走
def goByRoutes(routes):
    global global_go
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
    global global_doCleanThreadStop
    if not _can_start_runtime_task():
        return jsonify(_runtime_not_startable_payload())

    validation = _validate_auto_drive_request()
    if not validation.get('success'):
        _mark_runtime_blocked(
            validation.get('faultState', 'AUTO_DRIVE_BLOCKED'),
            validation.get('message', '启动条件未通过'),
            validation.get('data')
        )
        _notify_cleaning_position_start_failed()
        return jsonify(validation)

    global_doCleanThreadStop = 0
    thread, error_payload = _start_runtime_thread(
        'auto_drive',
        autoDriveByRTKThread,
        ready_message='启动条件通过，正在创建 RTK 自动清扫线程',
        detail=validation.get('data'),
    )
    if error_payload:
        return jsonify(error_payload)
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

def autoDriveByRTKThread(task_token=None):
    """
    执行前端 auto_drive 命令对应的 RTK 自动清扫流程。

    任务数据来源：
    - currentTaskName：Redis 中当前真正选中的路线名。
    - config.json：当前选中路线的完整配置。
    - taskParams：从 config.json 同步到 Redis 的起点、起始航向、车库等参数。
    - taskList：按顺序执行的分段路径，mode=1 为清扫段，mode=2 为移动/换行段。

    执行顺序：
    1. 登记运行任务 token，拒绝重复启动或已过期线程。
    2. 判断车辆是否在车库，必要时先执行出库。
    3. 校验 currentTaskName 与 config.json.taskName 完全一致。
    4. 读取 taskList，将待执行任务写入 Redis，供续扫和前端进度展示。
    5. 每一段都读取实时 RTK 位置，并复用点位导航自动计算目标方向。
    6. 按 taskList 顺序执行“自动转向 -> RTK 点到点直行 -> 到点停车”。
    7. 每完成一段从 Redis 弹出一段；全部完成后停车并根据配置回库。

    任何阶段收到停止信号、转向失败或配置不一致都会停止执行，不继续直行。
    """
    global global_status
    global taskList  # 申明使用全局变量
    global global_pointToPoint_flag,global_doCleanThreadStop,global_go,global_originLat,global_originLon
    global global_auto_clean_stop
    # 任务线程可能由接口启动，也可能由循环自动清扫复用；没有 token 时先登记为新的自动清扫任务。
    if task_token is None:
        # Normal manual/MQTT starts pass through _start_runtime_thread, which
        # checks the lifecycle under TASK_SWITCH_LOCK before creating a token.
        # Reject direct background calls while a task is active so they cannot
        # replace the active token and make the running route stop as stale.
        with TASK_SWITCH_LOCK:
            if not _can_start_runtime_task():
                logger.warning(
                    "direct autoDriveByRTKThread call rejected: runtime task is already active"
                )
                return 0
            task_token = _begin_runtime_task('auto_drive')
    # 如果当前线程已经不是最新任务，或已收到停止信号，直接退出，避免旧线程继续控车。
    if _runtime_task_should_stop(task_token, 'auto_drive'):
        logger.warn("auto clean task token is no longer active; exit")
        return 0
    # 防止重复启动：状态机认为已有任务运行时，不允许再启动新的自动清扫。
    if _is_runtime_task_active():
        logger.warn("小车已经在工作了，无法再开启工作")
        _mark_runtime_blocked('ALREADY_WORKING', '小车当前已经在执行任务，请勿重复启动')
        return 0
    _mark_runtime_running('自动清扫准备中', {'action': 'auto_drive', 'phase': 'prepare_exit_garage'})
    redis_cli.set('curTaskIndex', 0)
    # 读取任务基础参数：起点、充电桩、出舱距离、起始姿态等都保存在 taskParams。
    taskParams = redis_cli.hgetall("taskParams")
    # 获取当前坐标点，判断当前点位是否在充电桩中
    chargingPileLat = float(taskParams.get('chargingPileLat'))
    chargingPileLon = float(taskParams.get('chargingPileLon'))
    backLength = _coerce_int(taskParams.get('startToChargingPilePointLength'), 0)
    # 起始点经纬度
    global_originLat = float(taskParams.get('startLat'))
    global_originLon = float(taskParams.get('startLon'))

    # 舱内/舱外状态判断：决定自动清扫前是否需要先执行出舱。
    exit_decision = decide_auto_exit_garage(get_garage_state(), backLength)
    logger.warn("自动清扫前出舱判定: {}".format(exit_decision))
    if _runtime_task_should_stop(task_token, 'auto_drive'):
        return 0
    if exit_decision.get('decision') == EXIT_DECISION_ALLOW:
        # 车辆被判定在舱内且允许出舱，先后退出舱，再确认状态已经变为舱外。
        goOutGarage(backLength)
        if _runtime_task_should_stop(task_token, 'auto_drive'):
            return 0
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
    # 自动清扫必须先选择当前任务，currentTaskName 用来和 config.json 中的 taskName 做一致性校验。
    # 页面高亮不等于小车已选中；只有 /tasks/current 成功后 Redis 才存在 currentTaskName。
    current_task_name = _normalize_task_name(redis_cli.get('currentTaskName'))
    if not current_task_name:
        _mark_runtime_blocked('CURRENT_TASK_NOT_SET', '未设置当前任务，请先设置当前任务后再启动')
        return

    try:
        # config.json 是当前要执行的任务文件，里面包含 taskList 路径段。
        taskObj = util.readConfig("config.json")
    except Exception as e:
        logger.error("读取config.json失败: {}".format(str(e)))
        _mark_runtime_blocked('CURRENT_TASK_CONFIG_MISSING', '当前任务配置不存在或不可读')
        return

    # 防止 Redis 里选中的任务和实际执行文件不一致。
    # 双重校验避免“前端选中 A，但小车 config.json 仍是 B”时误启动其他路线。
    config_task_name = _normalize_task_name(taskObj.get('taskName'))
    if config_task_name != current_task_name:
        _mark_runtime_blocked('CURRENT_TASK_MISMATCH', '当前任务与执行配置不一致，请重新设置当前任务')
        return

    # taskList 是自动清扫真正执行的分段路径，每段包含起点、终点、角度、模式等信息。
    taskList = taskObj.get('taskList')
    if not isinstance(taskList, list) or len(taskList) == 0:
        _mark_runtime_blocked('TASK_PATH_EMPTY', '当前任务没有可执行路径，请先生成并设置任务')
        return

    global_status = 'working'
    global_doCleanThreadStop = 0
    _begin_cleaning_position_run(taskObj, task_token)
    _mark_runtime_running('自动清扫启动成功，任务执行中', {'action': 'auto_drive'})

    # 根据缓存中是否存在任务，来构建新的任务
    resultTask = []
    # 如果缓存中没有任务，则读取当前任务中的数据
    if len(resultTask) == 0:
        taskObj = util.readConfig("config.json")
        taskList = taskObj['taskList']
    else:
        taskList = resultTask
    # 清除下位机当前记录的清扫航向；每一段都会根据实时位置重新计算目标方向。
    reset_clean_mode(ser)
    time.sleep(0.02)
    # 将当前任务段写入 Redis taskList，便于中断续扫、低电回充、前端进度展示。
    redis_cli.delete('taskList')
    logger.warn(taskList)
    for item in taskList:
        # 将字典转为JSON字符串存储
        redis_cli.rpush('taskList', json.dumps(item))


    # 点到点直线行走是否停止标识
    global_pointToPoint_flag = 0
    runtime_speed = _coerce_int(redis_cli.get('forwardSpeed'), 350)
    if runtime_speed is None or runtime_speed <= 0:
        runtime_speed = 350
    # 执行任务。相同continuousPathId中的普通浮动点会合并成一次连续直行；
    # 路径文件仍保留原始分段，前端绘图数据和保存格式均不改变。
    index = 0
    while index < len(taskList):
        continuous_run = collect_continuous_run(taskList, index)
        if not continuous_run:
            continuous_run = [taskList[index]]
        task = attach_continuations(continuous_run)
        run_count = len(continuous_run)
        # 每段开始前都检查一次停止信号，保证急停或任务切换能尽快生效。
        if _runtime_task_should_stop(task_token, 'auto_drive'):
            global_doCleanThreadStop = 1
            break
        logger.warn("执行任务{}".format(index + 1))
        # 第一段和后续各段使用完全相同的点位导航：实时位置 -> 自动转向 -> 直行到终点。
        segment_speed = _coerce_int(task.get('speed'), runtime_speed)
        result = _run_task_segment_by_point_navigation(
            task,
            segment_speed,
            'auto_drive',
            index + 1,
        )
        if _runtime_task_should_stop(task_token, 'auto_drive'):
            global_go = 0
            # 表示自动清扫线程停止
            global_doCleanThreadStop = 1
            break
        if not result:
            global_doCleanThreadStop = 1
            _mark_runtime_blocked(
                'AUTO_DRIVE_SEGMENT_FAILED',
                '第{}段点位导航失败，自动清扫已停止'.format(index + 1),
                {'taskId': task.get('id', index + 1), 'segmentIndex': index + 1},
            )
            break

        # 当前连续折线之后只剩最后一段时，RTK纠偏回调会据此在接近终点时降速。
        if index + run_count == len(taskList) - 1:
            redis_cli.set("lastTask", 1)
        else:
            redis_cli.set("lastTask", 0)
        # 整条连续折线真正成功后才一次性弹出其中所有原始任务段；执行中断时全部保留，
        # 避免续跑从浮动点后的错误位置开始。
        for completed_offset in range(run_count):
            logger.warn("删除任务{}".format(index + completed_offset + 1))
            redis_cli.lpop("taskList")
        index += run_count
    logger.warn("自动行驶结束")
    redis_cli.incr("doTaskCounter")
    if not _is_current_runtime_task(task_token):
        logger.warn("auto clean task token is stale; skip final runtime writes")
        return 0
    completed_normally = global_doCleanThreadStop == 0 and not _runtime_task_should_stop(task_token, 'auto_drive')
    # 如果自动清扫被停止，则不继续运行
    if completed_normally:
        # 正常完成后停车、清空缓存任务，并按配置执行回舱。
        doParking(update_runtime=False)
        redis_cli.delete('taskList')
        _set_auto_resume_allowed(False, 'auto_drive_completed')
        if backLength != 0:
            intoGarage(backLength)
        _mark_runtime_complete('RTK自动清扫任务结束', {'action': 'auto_drive'})
    else:
        if _is_current_runtime_task(task_token):
            doParking()




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
    # legacy mission write removed; runtime state goes through FSM.
    # reset_odometer(ser)
    # moveByRTK(chargingPileLat, chargingPileLon, (originHeading + 180) % 360)
    # 开启视觉纠偏
    # redis_cli.set("correct","true")
    # legacy mission write removed; runtime state goes through FSM.
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
    _reset_manual_steering(send_hardware=False)
    redis_cli.set("reverse", "false")
    global global_doCleanThreadStop
    if not _can_start_runtime_task():
        return jsonify(_runtime_not_startable_payload())

    validation = _validate_auto_drive_request()
    if not validation.get('success'):
        _mark_runtime_blocked(
            validation.get('faultState', 'AUTO_DRIVE_BLOCKED'),
            validation.get('message', '启动条件未通过'),
            validation.get('data')
        )
        _notify_cleaning_position_start_failed()
        return jsonify(validation)

    global_doCleanThreadStop = 0
    thread, error_payload = _start_runtime_thread(
        'auto_drive',
        autoDriveByRTKThread,
        ready_message='启动条件通过，正在创建自动清扫线程',
        detail=validation.get('data'),
    )
    if error_payload:
        return jsonify(error_payload)

    return jsonify({
        'success': True,
        'message': '启动自动清扫任务线程',
        'data': validation.get('data'),
    })


def loopAutoDriveThread(task_token=None):
    global loop_auto_clean_thread, global_doCleanThreadStop, global_loop_auto_clean_stop
    if task_token is None:
        task_token = _begin_runtime_task('loop_auto_drive')
    logger.warn("循环自动清扫线程启动")
    _set_loop_auto_clean_state(running=True, stop_reason='running')
    try:
        while _is_loop_auto_clean_enabled():
            if _runtime_task_should_stop(task_token, 'loop_auto_drive'):
                _disable_loop_auto_clean('task_interrupted')
                break
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
            autoDriveByRTKThread(task_token)

            if _runtime_task_should_stop(task_token, 'loop_auto_drive'):
                _disable_loop_auto_clean('task_interrupted')
                break
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
                if _runtime_task_should_stop(task_token, 'loop_auto_drive'):
                    _disable_loop_auto_clean('task_interrupted')
                    break
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
    global loop_auto_clean_thread, global_doCleanThreadStop, global_loop_auto_clean_stop
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
        global_loop_auto_clean_stop = 0
        _set_loop_auto_clean_state(enabled=True, running=False, stop_reason='', cycle=0)
        loop_auto_clean_thread, error_payload = _start_runtime_thread(
            'loop_auto_drive',
            loopAutoDriveThread,
            ready_message='循环自动清扫启动条件通过，正在创建循环线程',
            detail=validation.get('data'),
        )
        if error_payload:
            _set_loop_auto_clean_state(enabled=False, running=False, stop_reason='start_rejected')
            return jsonify(error_payload)

    return jsonify({
        'success': True,
        'message': '循环自动清扫已启动，将持续执行到低电回充',
        'data': _get_loop_auto_clean_status(),
    })


@app.route("/vehicle/stopLoopAutoDrive", methods=['GET'])
def stop_loop_auto_drive():
    global global_status
    _request_runtime_stop('manual_stop_loop_auto_drive', clear_auto_task=True,
                          update_runtime=True,
                          message='循环自动清扫已停止')
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


def _set_current_task(task_name, return_to_origin=True):
    global taskList
    try:
        taskName = normalize_task_name(task_name)
    except ModelingTaskPersistenceError as error:
        return {"success": False, "msg": error.message, "data": {"code": error.code}}
    if not taskName:
        return {"success": False, "msg": "taskName不能为空"}

    fileName = taskName + '.json'
    if not os.path.exists(fileName):
        return {"success": False, "msg": u"文件不存在 {}".format(fileName)}

    if _is_runtime_task_active():
        return {
            "success": False,
            "msg": "自动任务运行中，不能切换路线",
            "data": {
                "code": "TASK_SWITCH_WHILE_RUNNING",
                "action": _runtime_action(),
            },
        }

    return_to_origin = bool(return_to_origin)
    task_config = _load_json_config(fileName)
    try:
        selected_config = select_named_task_variant(
            task_config,
            return_to_origin=return_to_origin,
        )
    except ModelingTaskPersistenceError as error:
        return {
            "success": False,
            "msg": "该路线没有所选的返回方式，请重新建模并保存",
            "data": {"code": error.code},
        }
    selected_tasks = selected_config.get('taskList') or []

    with TASK_SWITCH_LOCK:
        _write_json_config('config.json', selected_config)
        redis_cli.set('currentTaskName', taskName)
        redis_cli.set('currentTaskReturnToOrigin', 'true' if return_to_origin else 'false')
        redis_cli.set('curTaskIndex', 0)
        redis_cli.delete('taskList')
        syncCurTaskFileToRedis()
        taskList = []

    return {
        "success": True,
        "msg": "路线选择成功",
        "data": {
            "taskName": taskName,
            "returnToOrigin": return_to_origin,
            "taskCount": len(selected_tasks),
        },
    }
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

    logger.warn("legacy correctByRTK uses shared RTKDataManager observer stream")
    rtk_generator = ()
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
    _request_runtime_stop('manual_parking', clear_auto_task=True, update_runtime=True,
                          message='已执行停车指令')
    response = make_response("1")

    return response


def doParking(update_runtime=True, message='任务已停止并进入停车状态'):
    global global_pointToPoint_flag, global_go
    _reset_manual_steering(send_hardware=False, commanded_motion='stopped')
    redis_cli.set("ultraSonic", "false")
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
    if update_runtime:
        _clear_runtime_task_state('doParking', update_runtime=True, message=message)

# 删除缓存任务
@app.route('/vehicle/delTaskList', methods=['GET'])
def delTaskList():
    _clear_runtime_task_state('delTaskList', clear_auto_task=True, update_runtime=True,
                              message='cached task state cleared')
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
def _reconcile_saved_task_index():
    discovered_names = discover_saved_task_names(
        os.listdir(os.getcwd()),
        lambda file_name: _load_json_config(file_name),
    )
    discovered_set = set(discovered_names)
    indexed_names = set(
        _normalize_task_name(task_name)
        for task_name in redis_cli.smembers('taskNameSet')
        if _normalize_task_name(task_name)
    )

    for task_name in sorted(discovered_set - indexed_names):
        task_config = _load_json_config(task_name + '.json')
        redis_cli.sadd('taskNameSet', task_name)
        redis_cli.hset(
            'loc_start_lat_lon',
            task_name,
            json.dumps({
                'startLat': task_config.get('startLat'),
                'startLon': task_config.get('startLon'),
            }),
        )

    return sorted(discovered_set | indexed_names)


@app.route("/vehicle/selectTaskName", methods=['GET'])
def selectTaskName():
    # 获取所有任务
    taskNames = _reconcile_saved_task_index()
    # 获取当前任务
    currentTaskName = redis_cli.get('currentTaskName')
    data = {
        'taskNames': list(taskNames),
        'currentTaskName': _normalize_task_name(currentTaskName) or None,
    }

    if len(taskNames) == 0:
        result = {"success": True, "msg": "数据为空"}
        return jsonify(result)
    else:
        result = {"success": True, "msg": "获取数据成功"}
        result['data'] = data
        return jsonify(result)


@app.route("/vehicle/selectSavedRoutes", methods=['GET'])
def selectSavedRoutes():
    task_names = _reconcile_saved_task_index()
    current_task_name = redis_cli.get('currentTaskName')
    current_return_to_origin = _coerce_bool(
        redis_cli.get('currentTaskReturnToOrigin'),
        True,
    )

    def load_task_config(task_name):
        return _load_json_config(task_name + '.json')

    def load_model(model_id):
        return modeling_store.get_model(model_id)

    data = build_saved_routes(
        task_names,
        current_task_name,
        load_task_config,
        load_model,
        current_return_to_origin=current_return_to_origin,
    )
    return jsonify({
        "success": True,
        "msg": "获取已保存路线成功",
        "data": data,
    })


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
    return jsonify(_set_current_task(
        request.args.get('taskName'),
        _coerce_bool(request.args.get('returnToOrigin'), True),
    ))


@app.route("/vehicle/setCurrentTask", methods=['GET'])
def setCurrentTask():
    return jsonify(_set_current_task(
        request.args.get('taskName'),
        _coerce_bool(request.args.get('returnToOrigin'), True),
    ))

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
    logger.warn("启动返回固定点线程")
    thread, error_payload = _start_runtime_thread(
        'return_to_point',
        returnToPointThread,
        ready_message='正在创建返回固定点线程',
    )
    if error_payload:
        return jsonify(error_payload)
    response = make_response("1")
    return response


def returnToPointThread(task_token=None):
    try:
        logger.warn("启动返回固定点")
        _mark_runtime_running('返回固定点启动', {'action': 'return_to_point'})
        redis_cli.set('curTaskIndex', 0)
        logger.warn("returnToPoint uses shared RTKDataManager observer stream")
        # 判断当前到那个任务了
        json_item = redis_cli.lindex('taskList', 0)
        # 下一个任务
        json_next_item = redis_cli.lindex('taskList', 1)

        redis_cli.set("correct", "true")
        redis_cli.set('action', 'true')
        reset_odometer(ser)

        if json_item:
            item = json.loads(json_item)
            next_item = json.loads(json_next_item)
            goByBackRoute(item, next_item)
        # 开启纠偏
        redis_cli.set("correct", "false")
        redis_cli.set('curTaskIndex', 0)
        # 初始化
        redis_cli.set("doCleanThreadStop", 0)
        logger.warn("返回固定点结束")
        _mark_runtime_complete('返回固定点结束', {'action': 'return_to_point'})
        # 停止一切
        doParking(update_runtime=False)
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
    if _is_runtime_stop_requested():
        return
    # 获取当前任务开始点和结束点的经纬度
    # dis,heading = util.get_distance_angle(global_cur_rtk_lat,global_cur_rtk_lon,endLat,endLon)
    global_cur_taskPoint = {"heading":heading,"startLat": global_cur_rtk_lat, "startLon": global_cur_rtk_lon,
                            "endLat": endLat,"endLon": endLon, "speed": 100}
    global_rtk_tracking_filter.reset()
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
        if global_edge_trigger_latch.consume(getEdge()):
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
def pointToPointByRTK(startLat, startLon, endLat, endLon, heading, speed=200,
                      polyline_points=None):
    global global_cur_taskPoint,global_interval
    global global_go

    # 返回结果，0：不成功，1：任务执行成功
    result = 1
    global_interval = 0
    runtime_polyline = [dict(item) for item in list(polyline_points or []) if isinstance(item, dict)]
    if runtime_polyline:
        # 碰边判断和最终完成判断必须指向整条折线的最后一个停车点，不能指向前视
        # 虚拟目标或中间软点，否则会再次出现“离最终点很远却提前完成”的问题。
        endLat = float(runtime_polyline[-1]['lat'])
        endLon = float(runtime_polyline[-1]['lon'])
    global_cur_taskPoint = {"heading": heading, "startLat": startLat, "startLon": startLon,
                            "endLat": endLat, "endLon": endLon, "speed": speed,
                            "polylinePoints": runtime_polyline,
                            "polylineProgressM": 0.0,
                            "polylineDeviationStartedAt": None,
                            "polylineFailed": False}
    global_rtk_tracking_filter.reset()
    # 开启RTK纠偏
    global_go = 1
    _publish_global_go(global_go)
    # 设置滚刷
    brush_speed = int(redis_cli.get("brushSpeed"))
    setBrushSpeed(brush_speed)
    goCommand(speed)
    startTime = time.time()
    # 如果global_go = 1，则说明直行未结束
    while global_go == 1:
        if _is_runtime_stop_requested():
            result = 0
            break
        # 表示到边了
        if global_edge_trigger_latch.consume(getEdge()):
            edge_action = _handle_edge_stop_for_current_task('pointToPointByRTK')
            if edge_action == EDGE_STOP_ACTION_TARGET:
                # 此时endLat/endLon始终是整条折线的最终停车点，因此只有真正接近
                # 最终目标时，碰边才可以作为本次连续路径完成。
                global_go = 0
                _publish_global_go(global_go)
                break
            if edge_action == EDGE_STOP_ACTION_RECOVER:
                _recover_from_abnormal_edge('pointToPointByRTK')
                goCommand(speed)
                continue
            result = 0
            global_go = 0
            _publish_global_go(global_go)
            break
        endTime = time.time()
        # 获取时间间隔
        global_interval = endTime-startTime
        time.sleep(0.1)
    if global_cur_taskPoint.get('polylineFailed'):
        result = 0
    _publish_global_go(global_go)
    return result


def pointToPointByRTKAutoHeading(
        current_start_lat,
        current_start_lon,
        endLat,
        endLon,
        speed=200,
        before_drive=None,
        source='point_to_point_auto_heading',
        segment_index=None,
        task_id=None,
        polyline_points=None):
    """按实时起点计算目标方向，完成转向后再执行点到点直行。"""
    guidance = None
    if polyline_points:
        guidance = compute_polyline_guidance(
            polyline_points,
            current_start_lat,
            current_start_lon,
        )
    if guidance:
        # 连续折线的初始转向使用沿折线前方的前视方向，而不是直接瞄准最近的短点。
        # 真实停车位置即使偏离理论起点十几厘米，也不会把几度方向放大成七十多度。
        distance = guidance['remainingM']
        heading = guidance['heading']
        logger.warn(
            "go to polyline auto heading: remaining={:.3f}, heading={:.3f}, crossTrack={:.3f}, lookahead={:.3f}".format(
                float(distance),
                float(heading),
                float(guidance['crossTrackM']),
                float(guidance['lookaheadM']),
            )
        )
    else:
        # 普通单段任务继续按实时位置直接计算目标航向。
        distance, heading = util.get_distance_angle(current_start_lat, current_start_lon, endLat, endLon)
        logger.warn("go to point auto heading: distance={:.3f}, heading={:.3f}".format(float(distance), float(heading)))
    # 下位机转向协议使用0.1°单位，所以把heading乘10；target_heading保留度数供闭环判断。
    turn_result = turn(
        ser,
        heading * 10,
        target_heading=heading,
        source=source,
        segment_index=segment_index,
        task_id=task_id,
    )
    if turn_result != 1:
        # 转向失败不能开始直行或开启滚刷，立即制动并把失败返回上层。
        sendBraking()
        return 0
    if callable(before_drive):
        # 只有确认朝向正确后才根据mode设置滚刷，避免原地转向时执行清扫。
        before_drive()
    # 使用相同的实时起点、终点和刚计算的heading进入RTK直线纠偏控制。
    return pointToPointByRTK(
        current_start_lat,
        current_start_lon,
        endLat,
        endLon,
        heading,
        speed,
        polyline_points=polyline_points,
    )


def goToPointThread(task_token=None, plan=None):
    if plan is None and isinstance(task_token, dict):
        plan = task_token
        task_token = None
    global global_pointToPoint_flag, global_go
    redis_cli.set("correct", "true")
    _mark_runtime_running('点对点导航启动', {'goToPointPlan': plan, 'action': 'go_to_point'})
    try:
        result = pointToPointByRTKAutoHeading(
            plan.get("startLat"),
            plan.get("startLon"),
            plan.get("targetLat"),
            plan.get("targetLon"),
            plan.get("speed"),
        )
        if result == 1:
            _mark_runtime_complete('点对点导航完成', {'goToPointPlan': plan})
        else:
            _mark_runtime_blocked('GO_TO_POINT_INTERRUPTED', '点对点导航被中断', {'goToPointPlan': plan})
    except Exception as e:
        logger.error("goToPointThread error: {}".format(traceback.format_exc()))
        _mark_runtime_blocked('GO_TO_POINT_ERROR', '点对点导航异常: {}'.format(str(e)), {'goToPointPlan': plan})
    finally:
        global_go = 0
        global_pointToPoint_flag = 0
        _publish_global_go(global_go)
        redis_cli.set("correct", "false")
        doParking(update_runtime=False)


@app.route("/vehicle/goToPoint", methods=['POST', 'GET'])
def goToPoint():
    payload = _request_payload()
    target_lat = payload.get('targetLat', payload.get('lat'))
    target_lon = payload.get('targetLon', payload.get('lon'))
    speed = payload.get('speed', redis_cli.get("forwardSpeed"))
    plan_result = build_go_to_point_plan(
        global_cur_rtk_lat,
        global_cur_rtk_lon,
        target_lat,
        target_lon,
        speed,
    )
    if not plan_result.get("success"):
        return jsonify(plan_result)

    plan = plan_result.get("data") or {}
    thread, error_payload = _start_runtime_thread(
        'go_to_point',
        goToPointThread,
        args=(plan,),
        ready_message='正在创建点对点导航线程',
        detail={'goToPointPlan': plan},
    )
    if error_payload:
        return jsonify(error_payload)
    return jsonify({
        'success': True,
        'code': 200,
        'msg': plan_result.get('msg', 'go to point started'),
        'data': plan,
    })


def _parse_waypoints_from_payload(payload):
    waypoints = payload.get('waypoints') or payload.get('points') or []
    if isinstance(waypoints, str):
        try:
            waypoints = json.loads(waypoints)
        except Exception:
            waypoints = []
    if not isinstance(waypoints, list):
        return []
    result = []
    for item in waypoints:
        if not isinstance(item, dict):
            continue
        lat = _coerce_float(item.get('lat', item.get('targetLat')), None)
        lon = _coerce_float(item.get('lon', item.get('targetLon')), None)
        if lat is None or lon is None:
            continue
        result.append({
            'lat': lat,
            'lon': lon,
            'speed': _coerce_int(item.get('speed'), _coerce_int(payload.get('speed'), 200)),
        })
    return result


def _set_waypoint_loop_progress(loop_options, current_loop=0, waypoint_index=0):
    redis_cli.set('waypointLoopEnabled', '1' if loop_options.get('loop') else '0')
    redis_cli.set('waypointLoopMode', loop_options.get('loopMode') or 'count')
    redis_cli.set('waypointLoopTarget', loop_options.get('loopCount', 1))
    redis_cli.set('waypointLoopCurrent', current_loop)
    redis_cli.set('waypointIndex', waypoint_index)
    redis_cli.hset('waypointLoopProgress', 'waypointIndex', waypoint_index)
    redis_cli.hset('waypointLoopProgress', 'loopMode', loop_options.get('loopMode') or 'count')
    redis_cli.hset('waypointLoopProgress', 'currentLoop', current_loop)
    redis_cli.hset('waypointLoopProgress', 'targetLoop', loop_options.get('loopCount', 1))


def multiGoToPointThread(task_token=None, waypoints=None, loop_options=None):
    # 兼容旧调用方式：以前可能直接把 waypoints 作为第一个参数传进来。
    # 新流程里 task_token 由 _start_runtime_thread 管理，waypoints 和 loop_options 作为 args 传入。
    if loop_options is None and isinstance(task_token, list):
        loop_options = waypoints
        waypoints = task_token
        task_token = None
    # 标记当前进入纠偏/导航流程；真正的 RTK 纠偏会在 pointToPointByRTK() 打开 global_go 后开始。
    redis_cli.set("correct", "true")
    # 写入 FSM/运行态，告诉前端和云端当前动作是 multi_go_to_point。
    _mark_runtime_running('多路点导航启动', {
        'waypoints': waypoints,
        'loopOptions': loop_options,
        'action': 'multi_go_to_point',
    })
    try:
        # 根据闭环配置生成目标点迭代器：
        # - loop=true 且 loopMode=continuous 时，loop_count=None 表示持续循环；
        # - loop=true 且 loopMode=count 时，按 loopCount 圈数闭环；
        # - loop=false 时，只按路点顺序执行一遍。
        if loop_options.get('loop'):
            loop_count = None if loop_options.get('loopMode') == 'continuous' else loop_options.get('loopCount')
            target_iter = iter_closed_loop_targets(waypoints, loop_count)
        else:
            target_iter = (
                {'index': index, 'waypoint': waypoint, 'completed_loop': 0}
                for index, waypoint in enumerate(waypoints)
            )
        for target in target_iter:
            # 每个路点开始前检查停止状态，收到停止/急停/任务切换后不再继续执行后续路点。
            if _is_runtime_stop_requested():
                break
            waypoint = target.get('waypoint') or {}
            current_loop = target.get('completed_loop', 0)
            # 同步当前路点索引和闭环圈数，供状态接口/页面展示进度。
            _set_waypoint_loop_progress(loop_options, current_loop, target.get('index', 0))
            loop_progress = {
                "loopMode": loop_options.get('loopMode'),
                "currentLoop": current_loop,
                "targetLoop": loop_options.get('loopCount'),
            }
            _set_redis_value('runtimeDetail', _build_runtime_detail({'waypointLoopProgress': loop_progress}))
            # 用“当前实时 RTK 位置 -> 目标路点”生成单点导航计划，计算起点、终点、距离、航向和速度。
            plan_result = build_go_to_point_plan(
                global_cur_rtk_lat,
                global_cur_rtk_lon,
                waypoint.get('lat'),
                waypoint.get('lon'),
                waypoint.get('speed'),
            )
            if not plan_result.get('success'):
                # 当前 RTK 不可用、目标点无效或速度无效时，阻塞任务并停止继续遍历。
                _mark_runtime_blocked(plan_result.get('code', 'WAYPOINT_INVALID'), plan_result.get('msg', '路点无效'), plan_result)
                break
            plan = plan_result.get('data') or {}
            # 执行单个路点：
            # pointToPointByRTKAutoHeading() 会先 turn() 转到目标航向，
            # 再进入 pointToPointByRTK()，由 observer_go_correct() 按 RTK 数据持续纠偏。
            result = pointToPointByRTKAutoHeading(
                plan.get("startLat"),
                plan.get("startLon"),
                plan.get("targetLat"),
                plan.get("targetLon"),
                plan.get("speed"),
            )
            if result != 1:
                # 单个路点未完成时，不再执行后续路点，避免路径状态不连续。
                _mark_runtime_blocked('WAYPOINT_INTERRUPTED', '多路点导航被中断', {'waypoint': waypoint})
                break
        else:
            # for 循环没有被 break 打断，说明所有路点/闭环目标都正常完成。
            _mark_runtime_complete('多路点导航完成', {'waypoints': waypoints, 'loopOptions': loop_options})
    except Exception as e:
        logger.error("multiGoToPointThread error: {}".format(traceback.format_exc()))
        _mark_runtime_blocked('MULTI_GO_TO_POINT_ERROR', '多路点导航异常: {}'.format(str(e)))
    finally:
        # 无论正常完成、被打断还是异常，都关闭纠偏标记并停车收尾。
        redis_cli.set("correct", "false")
        doParking(update_runtime=False)


@app.route("/vehicle/multiGoToPoint", methods=['POST'])
def multiGoToPoint():
    payload = _request_payload()
    waypoints = _parse_waypoints_from_payload(payload)
    if not waypoints:
        return jsonify({'success': False, 'code': 'WAYPOINTS_EMPTY', 'msg': '路点为空'})
    try:
        loop_options = normalize_loop_options(payload, len(waypoints))
    except ValueError as e:
        return jsonify({'success': False, 'code': 'WAYPOINT_LOOP_INVALID', 'msg': str(e)})
    with TASK_SWITCH_LOCK:
        if not _can_start_runtime_task():
            return jsonify(_runtime_not_startable_payload())
        redis_cli.set('waypointTotal', len(waypoints))
        redis_cli.set('waypointIndex', 0)
        _set_waypoint_loop_progress(loop_options, 0, 0)
        thread, error_payload = _start_runtime_thread(
            'multi_go_to_point',
            multiGoToPointThread,
            args=(waypoints, loop_options),
            ready_message='正在创建多路点导航线程',
            detail={'waypoints': waypoints, 'loopOptions': loop_options},
        )
        if error_payload:
            return jsonify(error_payload)
    return jsonify({
        'success': True,
        'code': 200,
        'msg': 'multi go to point started',
        'data': {
            'waypoints': waypoints,
            'loopOptions': loop_options,
        },
    })

# 实时获取当前位置，计算当前位置到充电的距离
def _load_go_to_points():
    raw_list = redis_cli.lrange('waypoints', 0, -1) or []
    waypoints = []
    for item in raw_list:
        try:
            waypoint = json.loads(_decode_redis_value(item))
        except Exception:
            continue
        if isinstance(waypoint, dict):
            waypoints.append(waypoint)
    return waypoints


def _save_go_to_points(waypoints):
    redis_cli.delete('waypoints')
    for waypoint in waypoints:
        redis_cli.rpush('waypoints', json.dumps(waypoint, ensure_ascii=False))
    redis_cli.set('waypointTotal', len(waypoints))


def _normalize_go_to_point_item(item, default_speed=200):
    if not isinstance(item, dict):
        return None
    lat = _coerce_float(item.get('lat', item.get('targetLat')), None)
    lon = _coerce_float(item.get('lon', item.get('targetLon')), None)
    if lat is None or lon is None:
        return None
    return {
        'lat': lat,
        'lon': lon,
        'speed': _coerce_int(item.get('speed'), default_speed),
    }


def _execute_go_to_points_target(target):
    waypoint = target.get('waypoint') or target
    index = _coerce_int(target.get('index'), 0)
    redis_cli.set('waypointIndex', index)
    redis_cli.set('curTaskIndex', index)
    plan_result = build_go_to_point_plan(
        global_cur_rtk_lat,
        global_cur_rtk_lon,
        waypoint.get('lat'),
        waypoint.get('lon'),
        waypoint.get('speed'),
    )
    if not plan_result.get('success'):
        logger.warn("goToPoints waypoint invalid: {}".format(plan_result))
        _mark_runtime_blocked(plan_result.get('code', 'WAYPOINT_INVALID'), plan_result.get('msg', '路点无效'), plan_result)
        return False
    data = plan_result.get('data') or {}
    result = pointToPointByRTKAutoHeading(
        data['startLat'], data['startLon'],
        data['targetLat'], data['targetLon'],
        data['speed'],
    )
    if result == 0:
        logger.warn("多点路点导航: 路点#{} 未完成，中断后续路点".format(index + 1))
        return False
    return True


@app.route("/vehicle/goToPoints", methods=['GET'])
def goToPointsList():
    try:
        waypoints = _load_go_to_points()
        return jsonify({
            "success": True,
            "msg": "获取路点列表成功",
            "data": waypoints,
            "total": len(waypoints),
        })
    except Exception as e:
        logger.error("获取路点列表失败: {}".format(e), exc_info=True)
        return jsonify({"success": False, "msg": "获取路点列表失败", "data": [], "total": 0})


@app.route("/vehicle/goToPoints", methods=['POST'])
def goToPointsBatchSet():
    payload = _request_payload()
    items = payload.get('waypoints') if isinstance(payload, dict) else None
    if isinstance(items, str):
        try:
            items = json.loads(items)
        except Exception:
            items = []
    if not isinstance(items, list):
        return jsonify({"success": False, "code": "INVALID_WAYPOINTS", "msg": "路点参数无效"})
    waypoints = []
    for item in items:
        waypoint = _normalize_go_to_point_item(item, _coerce_int(payload.get('speed'), 200))
        if waypoint is None:
            return jsonify({"success": False, "code": "INVALID_WAYPOINT", "msg": "路点经纬度无效"})
        waypoints.append(waypoint)
    try:
        _save_go_to_points(waypoints)
        return jsonify({"success": True, "msg": "路点已保存", "data": waypoints, "total": len(waypoints)})
    except Exception as e:
        logger.error("批量设置路点失败: {}".format(e), exc_info=True)
        return jsonify({"success": False, "msg": "批量设置路点失败"})


@app.route("/vehicle/goToPoints/add", methods=['POST'])
def goToPointsAdd():
    payload = _request_payload()
    waypoint = _normalize_go_to_point_item(payload)
    if waypoint is None:
        return jsonify({"success": False, "code": "INVALID_WAYPOINT", "msg": "路点经纬度无效"})
    try:
        redis_cli.rpush('waypoints', json.dumps(waypoint, ensure_ascii=False))
        redis_cli.set('waypointTotal', redis_cli.llen('waypoints'))
        return jsonify({"success": True, "msg": "路点已添加", "data": waypoint})
    except Exception as e:
        logger.error("添加路点失败: {}".format(e), exc_info=True)
        return jsonify({"success": False, "msg": "添加路点失败"})


@app.route("/vehicle/goToPoints/remove", methods=['POST'])
def goToPointsRemove():
    payload = _request_payload()
    index = _coerce_int(payload.get('index'), -1)
    waypoints = _load_go_to_points()
    if index < 0 or index >= len(waypoints):
        return jsonify({"success": False, "code": "INVALID_INDEX", "msg": "索引无效"})
    removed = waypoints.pop(index)
    try:
        _save_go_to_points(waypoints)
        return jsonify({"success": True, "msg": "路点已删除", "data": removed, "total": len(waypoints)})
    except Exception as e:
        logger.error("删除路点失败: {}".format(e), exc_info=True)
        return jsonify({"success": False, "msg": "删除路点失败"})


@app.route("/vehicle/goToPoints/reorder", methods=['POST'])
def goToPointsReorder():
    payload = _request_payload()
    order = payload.get('order') if isinstance(payload, dict) else None
    if not isinstance(order, list):
        return jsonify({"success": False, "code": "INVALID_ORDER", "msg": "排序参数无效"})
    waypoints = _load_go_to_points()
    try:
        indexes = [int(item) for item in order]
    except Exception:
        return jsonify({"success": False, "code": "INVALID_ORDER", "msg": "排序参数无效"})
    if sorted(indexes) != list(range(len(waypoints))):
        return jsonify({"success": False, "code": "INVALID_ORDER", "msg": "排序索引不完整"})
    reordered = [waypoints[index] for index in indexes]
    try:
        _save_go_to_points(reordered)
        return jsonify({"success": True, "msg": "路点顺序已更新", "data": reordered})
    except Exception as e:
        logger.error("调整路点顺序失败: {}".format(e), exc_info=True)
        return jsonify({"success": False, "msg": "调整路点顺序失败"})


@app.route("/vehicle/goToPoints/clear", methods=['POST'])
def goToPointsClear():
    try:
        _clear_runtime_task_state('goToPointsClear', clear_waypoints=True, update_runtime=True,
                                  message='waypoints cleared')
        return jsonify({"success": True, "msg": "路点已清空", "data": [], "total": 0})
    except Exception as e:
        logger.error("清空路点失败: {}".format(e), exc_info=True)
        return jsonify({"success": False, "msg": "清空路点失败"})


def goToPointsThread(task_token=None):
    global global_status, global_go, global_doCleanThreadStop, global_waypoint_nav_stop
    if task_token is None:
        task_token = _begin_runtime_task('multi_go_to_point')
    if _runtime_task_should_stop(task_token, 'multi_go_to_point'):
        logger.warn("goToPointsThread: task token is no longer active, exit")
        return
    try:
        waypoints = _load_go_to_points()
        if not waypoints:
            logger.warn("goToPointsThread: 路点列表为空，退出")
            return

        loop_enabled = redis_cli.get(WAYPOINT_LOOP_ENABLED_KEY) == '1'
        loop_mode = redis_cli.get(WAYPOINT_LOOP_MODE_KEY) or 'count'
        loop_target = _coerce_int(redis_cli.get(WAYPOINT_LOOP_TARGET_KEY), 0)
        loop_options = {
            'loop': loop_enabled,
            'loopMode': loop_mode,
            'loopCount': loop_target,
        }
        logger.warn("启动多点路点导航: 共{}个路点, 闭环={}, 模式={}, 圈数={}".format(
            len(waypoints), loop_enabled, loop_mode, loop_target))

        if len(waypoints) == 1:
            wp = waypoints[0]
            plan = build_go_to_point_plan(
                global_cur_rtk_lat,
                global_cur_rtk_lon,
                wp.get('lat'),
                wp.get('lon'),
                wp.get('speed', 200),
            )
            if plan.get('success'):
                logger.warn("单路点委托给 goToPointThread")
                goToPointThread(plan.get('data') or {})
            else:
                logger.warn("单路点校验失败: {}".format(plan.get('msg')))
            return

        global_status = 'working'
        redis_cli.set("correct", "true")
        redis_cli.set('waypointTotal', len(waypoints))
        _mark_runtime_running('多点路点导航启动', {
            'waypoints': waypoints,
            'loopOptions': loop_options,
            'action': 'multi_go_to_point',
        })

        if loop_enabled:
            loop_count = None if loop_mode == 'continuous' else loop_target
            target_iter = iter_closed_loop_targets(waypoints, loop_count)
        else:
            target_iter = (
                {'index': index, 'waypoint': waypoint, 'completed_loop': 0}
                for index, waypoint in enumerate(waypoints)
            )

        completed_loop = 0
        completed_normally = False
        failed = False
        for target in target_iter:
            if _runtime_task_should_stop(task_token, 'multi_go_to_point') or global_doCleanThreadStop:
                failed = True
                break
            if not _execute_go_to_points_target(target):
                failed = True
                _mark_runtime_blocked(
                    'WAYPOINT_INTERRUPTED',
                    '多点路点导航未完成',
                    {'target': target}
                )
                break
            if target.get('completed_loop', 0) > completed_loop:
                completed_loop = target['completed_loop']
                redis_cli.set(WAYPOINT_LOOP_CURRENT_KEY, completed_loop)
                logger.warn("多点闭环导航: 第{}圈完成".format(completed_loop))
        completed_normally = (
            not failed
            and not _runtime_task_should_stop(task_token, 'multi_go_to_point')
            and not global_doCleanThreadStop
        )
        if completed_normally:
            _mark_runtime_complete('多点路点导航完成', {'waypoints': waypoints, 'loopOptions': loop_options})
        logger.warn("多点路点导航结束")
    except Exception as e:
        logger.error("多点路点导航异常: {}".format(e), exc_info=True)
        _mark_runtime_blocked('GO_TO_POINTS_ERROR', '多点路点导航异常: {}'.format(str(e)))
    finally:
        if _is_current_runtime_task(task_token):
            global_go = 0
            _publish_global_go(global_go)
            _clear_runtime_task_state('goToPointsThread.finally', update_runtime=False,
                                      message='go to points thread finished')
            global_status = 'active'
            doParking(update_runtime=False)
        else:
            logger.warn("goToPointsThread: stale task token, skip final runtime writes")


@app.route("/vehicle/goToPoints/start", methods=['POST'])
def goToPointsStart():
    global global_doCleanThreadStop, global_waypoint_nav_stop
    try:
        waypoints = _load_go_to_points()
        if not waypoints:
            return jsonify({"success": False, "code": "WAYPOINTS_EMPTY", "msg": "路点为空"})
        payload = _request_payload()
        loop_options = normalize_loop_options(payload, len(waypoints))
        with TASK_SWITCH_LOCK:
            if not _can_start_runtime_task():
                return jsonify(_runtime_not_startable_payload())
            _set_auto_resume_allowed(False, 'prepare_go_to_points')
            _request_runtime_stop('prepare_go_to_points', clear_auto_task=True,
                                  update_runtime=False,
                                  message='prepare go to points task')
            redis_cli.set(WAYPOINT_LOOP_ENABLED_KEY, '1' if loop_options['loop'] else '0')
            redis_cli.set(WAYPOINT_LOOP_MODE_KEY, loop_options['loopMode'])
            redis_cli.set(WAYPOINT_LOOP_TARGET_KEY, loop_options['loopCount'])
            redis_cli.set(WAYPOINT_LOOP_CURRENT_KEY, 0)
            redis_cli.set('waypointTotal', len(waypoints))
            redis_cli.set('waypointIndex', 0)
            global_doCleanThreadStop = 0
            global_waypoint_nav_stop = 0
            thread, error_payload = _start_runtime_thread(
                'multi_go_to_point',
                goToPointsThread,
                ready_message='正在创建多点路点导航线程',
                detail={'waypoints': waypoints, 'loopOptions': loop_options},
            )
            if error_payload:
                return jsonify(error_payload)
        return jsonify({
            "success": True,
            "code": "MULTI_GO_TO_POINT_STARTED",
            "msg": "多点路点导航已启动({}个路点)".format(len(waypoints)),
            "data": {
                "total": len(waypoints),
                "loopOptions": loop_options,
            },
        })
    except ValueError as e:
        return jsonify({"success": False, "code": "WAYPOINT_LOOP_INVALID", "msg": str(e)})
    except Exception as e:
        logger.error("启动多点路点导航失败: {}".format(e), exc_info=True)
        return jsonify({"success": False, "msg": "启动多点路点导航失败"})


@app.route("/vehicle/goToPoints/stop", methods=['POST'])
def goToPointsStop():
    global global_doCleanThreadStop, global_waypoint_nav_stop
    try:
        global_doCleanThreadStop = 1
        global_waypoint_nav_stop = 1
        _request_runtime_stop('goToPointsStop', update_runtime=False,
                              message='go to points stopped')
        _clear_runtime_task_state('goToPointsStop', update_runtime=True,
                                  message='go to points stopped')
        logger.warn("多点路点导航: 收到停止命令")
        return jsonify({"success": True, "msg": "多点路点导航已停止"})
    except Exception as e:
        logger.error("停止多点路点导航失败: {}".format(e), exc_info=True)
        return jsonify({"success": False, "msg": "停止多点路点导航失败"})


@app.route("/vehicle/goToPoints/progress", methods=['GET'])
def goToPointsProgress():
    try:
        current_action = _runtime_action()
        running = _is_runtime_task_active() and current_action == 'multi_go_to_point'
        saved_total = len(_load_go_to_points())
        current_index = _coerce_int(redis_cli.get('waypointIndex'), 0) if running else 0
        total = _coerce_int(redis_cli.get('waypointTotal'), saved_total) if running else saved_total
        return jsonify({
            "success": True,
            "data": {
                "running": running,
                "currentIndex": current_index,
                "total": total,
                "savedTotal": saved_total,
                "loop": _coerce_bool(redis_cli.get(WAYPOINT_LOOP_ENABLED_KEY), False),
                "loopMode": redis_cli.get(WAYPOINT_LOOP_MODE_KEY) or 'count',
                "currentLoop": _coerce_int(redis_cli.get(WAYPOINT_LOOP_CURRENT_KEY), 0),
                "targetLoop": _coerce_int(redis_cli.get(WAYPOINT_LOOP_TARGET_KEY), 0),
            },
        })
    except Exception as e:
        logger.error("查询路点导航进度失败: {}".format(e), exc_info=True)
        return jsonify({"success": False, "msg": "查询进度失败"})


def observer_to_chargingPile(data):
    lat = data.lat
    lon = data.lon

def observer_rtk_data(data):
    """观察者用于获取rtk数据，将航向角发送给下位机，将经纬度发送给服务端"""
    lat = data.lat
    lon = data.lon
    if data.heading is None:
        if redis_cli.get('openLog') == '1':
            logger.warning("RTK已更新经纬度，但当前无有效航向角，跳过航向同步 lat={}, lon={}".format(lat, lon))
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
    global global_cur_taskPoint
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
    _publish_global_go(global_go)
    if global_go == 1:
        if global_rtk_recovering:
            last_recovering_log_at = getattr(observer_go_correct, '_last_recovering_log_at', 0.0)
            if now - last_recovering_log_at >= 1.0:
                logger.warning("RTK fixed recovery active; skip linear correction command")
                observer_go_correct._last_recovering_log_at = now
            return
        if data.heading is None:
            logger.warning("RTK经纬度已更新，但当前无有效航向角，暂不执行直行纠偏...")
            return
        polyline_guidance = None
        polyline_points = list(global_cur_taskPoint.get('polylinePoints') or [])
        if polyline_points:
            polyline_guidance = compute_polyline_guidance(
                polyline_points,
                data.lat,
                data.lon,
                previous_progress_m=global_cur_taskPoint.get('polylineProgressM', 0.0),
                lookahead_m=DEFAULT_LOOKAHEAD_M,
                corridor_m=DEFAULT_CORRIDOR_M,
            )
            if not polyline_guidance:
                logger.error("continuous polyline guidance unavailable; stop route")
                global_cur_taskPoint['polylineFailed'] = True
                global_go = 0
                _publish_global_go(global_go)
                redis_cli.set("correct", "false")
                sendBraking()
                return
            global_cur_taskPoint['polylineProgressM'] = polyline_guidance['progressM']
            global_cur_taskPoint['heading'] = polyline_guidance['heading']
            start_lat = polyline_guidance['referenceLat']
            start_lon = polyline_guidance['referenceLon']
            target_lat = polyline_guidance['targetLat']
            target_lon = polyline_guidance['targetLon']
            target_heading = float(polyline_guidance['heading'])
        else:
            start_lat = global_cur_taskPoint['startLat']
            start_lon = global_cur_taskPoint['startLon']
            target_lat = global_cur_taskPoint['endLat']
            target_lon = global_cur_taskPoint['endLon']
            target_heading = float(global_cur_taskPoint['heading'])

        # 连续折线模式把“当前位置投影点->受走廊约束的前视点”作为本帧纠偏线；
        # 普通单段任务仍使用保存的起点和终点。两者共用原有RTK直线控制器。
        tracking_command = build_tracking_command(
            global_rtk_tracking_filter,
            global_straight_line_controller,
            start_lat=start_lat,
            start_lon=start_lon,
            end_lat=target_lat,
            end_lon=target_lon,
            current_lat=data.lat,
            current_lon=data.lon,
            vehicle_heading=data.heading,
            target_heading=target_heading,
            timestamp=now,
            # 连续折线的前视点固定在0.8m以内，不能沿用普通短距离任务的±5度
            # 航向限制，否则全过程都无法充分纠偏。20度仍保留限幅，避免猛打方向。
            short_range_heading_limit_deg=20.0 if polyline_guidance else None,
        )
        steering_distance = tracking_command.distance_to_target_m
        heading_error = tracking_command.heading_error_deg
        cte = tracking_command.cte_m
        z_speed = tracking_command.z_speed
        if polyline_guidance:
            distance_to_target = polyline_guidance['distanceToFinalM']
            signed_remaining = polyline_guidance['remainingM']
            route_cross_track = polyline_guidance['crossTrackM']
        else:
            distance_to_target = steering_distance
            signed_remaining = tracking_command.signed_remaining_m
            route_cross_track = abs(cte)

        if polyline_guidance and polyline_guidance.get('terminalMissed'):
            # 已经越过最终点且超出15cm可接受制动范围时，继续向前只会把误差扩大，
            # 最终触发30cm硬限位。这里立即失败停车，由上层明确报告本段未完成。
            logger.error(
                "continuous polyline passed final target; stop route: overshoot={:.3f}, distance={:.3f}, cte={:.3f}".format(
                    polyline_guidance.get('overshootM') or 0.0,
                    distance_to_target,
                    route_cross_track,
                )
            )
            global_cur_taskPoint['polylineFailed'] = True
            global_go = 0
            _publish_global_go(global_go)
            redis_cli.set("correct", "false")
            sendBraking()
            return

        setZSpeed(z_speed)
        duplicateWriteCmd(ser, command)
        logger.info(
            "kalman p correction target={:.2f} current={:.2f} heading_error={:.2f} cte={:.2f} routeCte={:.2f} distance={:.2f}m z={}".format(
                target_heading, data.heading, heading_error, cte, route_cross_track,
                distance_to_target, z_speed
            )
        )
        global_last_cte = cte
        if distance_to_target <= 2 and redis_cli.get('lastTask') == '1':
            sendCommandSetXSpeed(200)

        _publish_correction_debug(
            heading_error,
            cte,
            z_speed,
            distance_to_target,
            signed_remaining,
            target_heading,
            data.heading,
            {
                'rawLat': tracking_command.raw_lat,
                'rawLon': tracking_command.raw_lon,
                'filteredLat': tracking_command.filtered_lat,
                'filteredLon': tracking_command.filtered_lon,
                'controlSource': 'polyline_lookahead' if polyline_guidance else tracking_command.source,
                'routeCrossTrack': route_cross_track,
                'lookahead': polyline_guidance.get('lookaheadM') if polyline_guidance else None,
                'routeProgress': polyline_guidance.get('progressM') if polyline_guidance else None,
            }
        )

        if polyline_guidance:
            # 10厘米走廊用于限制前视线抹平真实折线。超过后控制器继续主动纠偏；
            # 偏差达到30厘米，或20厘米以上持续5秒仍未恢复，才安全停车并报告失败。
            if route_cross_track > DEFAULT_CORRIDOR_M:
                deviation_started_at = global_cur_taskPoint.get('polylineDeviationStartedAt')
                if deviation_started_at is None:
                    deviation_started_at = now
                    global_cur_taskPoint['polylineDeviationStartedAt'] = now
                last_log_at = global_cur_taskPoint.get('polylineLastDeviationLogAt') or 0.0
                if now - last_log_at >= 1.0:
                    logger.warn(
                        "polyline deviation correction: cte={:.3f}, corridor={:.3f}, progress={:.3f}, remaining={:.3f}".format(
                            route_cross_track,
                            DEFAULT_CORRIDOR_M,
                            polyline_guidance['progressM'],
                            polyline_guidance['remainingM'],
                        )
                    )
                    global_cur_taskPoint['polylineLastDeviationLogAt'] = now
                hard_deviation = route_cross_track >= 0.30
                persistent_deviation = (
                    route_cross_track >= 0.20 and
                    now - deviation_started_at >= 5.0
                )
                if hard_deviation or persistent_deviation:
                    reason = 'hard_limit' if hard_deviation else 'persistent'
                    logger.error(
                        "polyline deviation unsafe; stop route: reason={}, cte={:.3f}, elapsed={:.2f}".format(
                            reason,
                            route_cross_track,
                            now - deviation_started_at,
                        )
                    )
                    global_cur_taskPoint['polylineFailed'] = True
                    global_go = 0
                    _publish_global_go(global_go)
                    redis_cli.set("correct", "false")
                    sendBraking()
                    return
            else:
                global_cur_taskPoint['polylineDeviationStartedAt'] = None

            finished = bool(polyline_guidance['complete'])
        else:
            finished = util.should_finish_point_to_point(
                distance_to_target,
                signed_remaining,
                cte,
            )

        if finished:
            global_go = 0
            _publish_global_go(global_go)
            redis_cli.set("correct", "false")
            logger.warn(
                "路径直行结束 distance={:.3f} remaining={:.3f} cte={:.3f}".format(
                    distance_to_target,
                    signed_remaining,
                    route_cross_track,
                )
            )
        global_last_distance_to_target = distance_to_target
        # 防止cup资源占满
        time.sleep(0.01)


def goOnDoClean():
    _mark_runtime_running('继续清扫启动', {'action': 'go_on'})
    try:
        # 未完成的任务列表
        previousTaskList = [json.loads(item) for item in redis_cli.lrange('taskList', 0, -1)]

        if doClean(previousTaskList, True) == 0:
            logger.warn("清扫工作未完成")
        else:
            logger.warn("清扫工作完成")
        sendBraking()
        logger.warn("任务执行结束")
        _mark_runtime_complete('继续清扫结束', {'action': 'go_on'})
        redis_cli.set("correct", "false")
        redis_cli.set('action', 'false')
        # 表示自动清扫线程停止
        redis_cli.set("doCleanThreadStop", 1)
    except Exception as e:
        logger.error(e.message)

def goOnDoCleanByRTK(task_token=None):
    global global_go,global_doCleanThreadStop
    _mark_runtime_running('RTK继续清扫启动', {'action': 'go_on'})
    # 未完成的任务列表
    previousTaskList = [json.loads(item) for item in redis_cli.lrange('taskList', 0, -1)]
    if previousTaskList:
        try:
            _begin_cleaning_position_run(util.readConfig('config.json'), task_token, resume=True)
        except Exception as error:
            logger.warning('resume cleaning telemetry: {}'.format(error))
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
            if _is_runtime_stop_requested():
                global_doCleanThreadStop = 1
                break
            if mode == 1:
                # moveBack(ser, turn_back_len)
                if _is_runtime_stop_requested():
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
                if _is_runtime_stop_requested():
                    global_doCleanThreadStop = 1
                    break
        if not _is_runtime_stop_requested():
            logger.warn("删除任务{}".format(task['id']))
            redis_cli.lpop("taskList")
    logger.warn("继续清扫结束")
    completed_normally = global_doCleanThreadStop == 0 and not _is_runtime_stop_requested()
    if completed_normally:
        _mark_runtime_complete('RTK继续清扫结束', {'action': 'go_on'})
        doParking(update_runtime=False)
    else:
        doParking()

# 继续清扫
@app.route("/vehicle/goOn", methods=['GET'])
def goOn():
    logger.warn('继续清扫任务')
    redis_cli.set("reverse", "false")
    # thread = threading.Thread(target=goOnDoClean)
    # thread.start()
    thread, error_payload = _start_runtime_thread(
        'go_on',
        goOnDoCleanByRTK,
        ready_message='正在创建继续清扫线程',
    )
    if error_payload:
        return jsonify(error_payload)
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
    _reset_manual_steering(send_hardware=False, commanded_motion='forward')
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
    _reset_manual_steering(send_hardware=False, commanded_motion='reverse')
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

    # The joystick is another explicit source of manual travel intent.  Keep
    # the steering state machine synchronized without relying on transient
    # wheel-speed feedback from the lower machine.
    if floatY > 0.2:
        joystick_motion = 'forward'
    elif floatY < -0.2:
        joystick_motion = 'reverse'
    else:
        joystick_motion = 'stopped'
    _reset_manual_steering(send_hardware=False, commanded_motion=joystick_motion)

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


def _manual_steering_base_motion():
    """Derive motion from live lower-machine data, never from frontend input."""
    hardware_report_at = _get_hardware_report_at()
    live_speed = live_value_from_report(_coerce_int(global_get_XSpeed, None), hardware_report_at)
    return derive_manual_motion_state(
        live_speed,
        lower_status=global_get_status,
        control_state=_derive_control_state(),
        fault_state=_derive_fault_state(),
        stop_requested=_runtime_action() == 'parking',
        z_speed=global_get_ZSpeed,
        power_on=_get_power_on_state(),
        report_at=hardware_report_at,
    )


def _persist_manual_steering_state(snapshot):
    """Expose the manual-button state to MQTT/WebSocket through Redis."""
    values = {
        'manualSteeringMode': snapshot.get('manualSteeringMode') or 'none',
        'manualSteeringDirection': snapshot.get('manualSteeringDirection') or '',
        'manualCorrectionValue': int(snapshot.get('manualCorrectionValue') or 0),
        'manualCorrectionLevel': int(snapshot.get('manualCorrectionLevel') or 0),
        'manualSteeringControlId': snapshot.get('manualSteeringControlId') or '',
    }
    for key, value in values.items():
        redis_cli.set(key, value)


def _manual_steering_target_speed(motion_state):
    """Return the configured manual speed, not decelerated wheel feedback."""
    if motion_state == 'forward':
        return max(1, abs(_coerce_int(redis_cli.get('forwardSpeed'), high_speed)))
    if motion_state == 'reverse':
        # /vehicle/back uses a fixed -100 longitudinal command.
        return 100
    raise RuntimeError('unsupported moving state: {}'.format(motion_state))


def _send_manual_trim(correction_value, motion_state, travel_speed):
    """Reuse the local joystick's fixed 45-degree directions while moving.

    ``correction_value`` is now a direction marker, not an accumulating RTK
    correction: negative means the left joystick diagonal, positive means the
    right diagonal, and zero restores straight travel.  ``travel_speed`` is
    captured from the original manual command and remains unchanged throughout
    the gesture; live wheel feedback is deliberately not reused as a command.
    """
    value = int(correction_value)
    travel_speed = max(1, abs(int(travel_speed)))
    steering_value = abs(value)
    logger.info(
        'manual joystick frame: motion={}, travelSpeed={}, steering={}'.format(
            motion_state, travel_speed, value,
        )
    )
    with MANUAL_STEERING_COMMAND_LOCK:
        if motion_state == 'forward':
            if value < 0:
                sendDrivingNorthWest(travel_speed, steering_value)
            elif value > 0:
                sendDrivingNorthEast(travel_speed, steering_value)
            else:
                sendDrivingNorth(travel_speed)
        elif motion_state == 'reverse':
            if value < 0:
                sendDrivingSouthWest(travel_speed, steering_value)
            elif value > 0:
                sendDrivingSouthEast(travel_speed, steering_value)
            else:
                sendDrivingSouth(travel_speed)
        else:
            raise RuntimeError('unsupported moving state: {}'.format(motion_state))


def _start_manual_rotation(direction):
    """Reuse the exact pure-left/pure-right joystick protocol (mode 4)."""
    with MANUAL_STEERING_COMMAND_LOCK:
        if direction == 'left':
            sendDrivingWest(0, 1000)
        else:
            sendDrivingEast(0, -1000)


def _stop_manual_rotation():
    with MANUAL_STEERING_COMMAND_LOCK:
        sendBraking()


def _get_manual_steering_controller():
    global manual_steering_controller
    with MANUAL_STEERING_CONTROLLER_LOCK:
        if manual_steering_controller is None:
            manual_steering_controller = ManualSteeringController(
                motion_provider=_manual_steering_base_motion,
                apply_trim=_send_manual_trim,
                start_rotation=_start_manual_rotation,
                stop_rotation=_stop_manual_rotation,
                travel_speed_provider=_manual_steering_target_speed,
                state_callback=_persist_manual_steering_state,
                moving_value=700,
                tap_duration=0.3,
                keepalive_timeout=1.5,
            )
        return manual_steering_controller


def _reset_manual_steering(send_hardware=False, commanded_motion=None):
    controller = manual_steering_controller
    if commanded_motion is not None and controller is None:
        controller = _get_manual_steering_controller()
    if controller is not None:
        controller.reset(send_hardware=send_hardware)
        if commanded_motion is not None:
            controller.set_commanded_motion(commanded_motion)


def _manual_steering_status_snapshot(live_speed=None, control_state=None, fault_state=None):
    return _get_manual_steering_controller().snapshot()


@app.route('/vehicle/manualSteering', methods=['POST'])
def manual_steering_api():
    """Local business endpoint used by MQTT for tap/hold left-right buttons."""
    params = request.get_json(silent=True) or {}
    try:
        return jsonify(_get_manual_steering_controller().handle(params))
    except ManualSteeringError as error:
        logger.warning('manual steering rejected: code={}, lowerStatus={}, '
                       'xSpeed={}, zSpeed={}, controlState={}, reportAge={}'.format(
                           error.code, global_get_status, global_get_XSpeed,
                           global_get_ZSpeed, _derive_control_state(),
                           _get_hardware_report_age_sec()))
        return jsonify({
            'success': False,
            'message': error.message,
            'data': {'code': error.code},
        })


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
    if _is_runtime_stop_requested():
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
    while (getEdge() == "1" and _is_runtime_task_active()) or (
            arriveable and redis_cli.get("odometer_arrive") == "true"):
        print("keep walking")
        global_status = "keep walking"
        if global_go == 0:
            break
        time.sleep(0.1)
    # time.sleep(0.5)


def moveBack(ser, distance=33):
    if _is_runtime_stop_requested():
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

    while getDistanceArrive() == "0" and _is_runtime_task_active():
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
    #         while getDistanceArrive() == "0" and _is_runtime_task_active():
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
        if _is_runtime_stop_requested():
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
        if _is_runtime_stop_requested():
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
    logger.warn(robot_lifecycle_fsm.get_state().get('controlState'))
    if _is_runtime_stop_requested():
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
    while getDistanceArrive() == "0" and _is_runtime_task_active():
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
        if _is_runtime_stop_requested():
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


def _turn_to_heading_by_rtk(ser, target_heading, source='turn', segment_index=None, task_id=None):
    target_heading = float(target_heading) % 360.0
    current_heading = _get_current_rtk_heading()
    if current_heading is None:
        logger.error("[turn_rtk_abort] no fresh RTK heading; target={}".format(target_heading))
        sendBraking()
        return 0

    direction, relative_angle = choose_turn_direction(current_heading, target_heading)
    initial_delta = _normalize_heading_delta(current_heading, target_heading)
    if direction == 'none' or abs(initial_delta) <= TURN_RTK_FALLBACK_TOLERANCE_DEG:
        logger.warn(
            "[turn_rtk_done] already at target: target={:.2f}, current={:.2f}, delta={:.2f}".format(
                target_heading, current_heading, initial_delta
            )
        )
        sendBraking()
        return 1

    protocol_value = TURN_RIGHT_PROTOCOL_VALUE if direction == 'right' else TURN_LEFT_PROTOCOL_VALUE
    sendBraking()
    preBuildCommand()
    setStatus(3)
    setPowerOn(1)
    setXSpeed(0)
    setZSpeed(0)
    setRotateTo(protocol_value)
    command[17] = tem_listener(command, 17)
    logger.warn(
        "[turn_rtk_start] source={}, segment={}, taskId={}, direction={}, relativeAngle={:.2f}, target={:.2f}, current={:.2f}, protocolValue={}, frame={}".format(
            source,
            segment_index,
            task_id,
            direction,
            relative_angle,
            target_heading,
            current_heading,
            protocol_value,
            ' '.join(format(x, '02x') for x in command),
        )
    )
    duplicateWriteCmd(ser, command)

    turn_start_at = time.time()
    stable_count = 0
    previous_delta = initial_delta
    last_log_at = 0.0
    while _is_runtime_task_active():
        elapsed = time.time() - turn_start_at
        if elapsed >= TURN_RTK_MAX_DURATION_SEC:
            logger.error(
                "[turn_rtk_timeout] direction={}, target={:.2f}, elapsed={:.2f}s".format(
                    direction, target_heading, elapsed
                )
            )
            sendBraking()
            return 0

        current_heading = _get_current_rtk_heading()
        delta = _normalize_heading_delta(current_heading, target_heading)
        if delta is not None:
            crossed_target = _heading_delta_crossed_target(previous_delta, delta)
            if abs(delta) <= TURN_RTK_FALLBACK_TOLERANCE_DEG:
                stable_count += 1
            else:
                stable_count = 0
            if stable_count >= TURN_RTK_FALLBACK_STABLE_COUNT or crossed_target:
                logger.warn(
                    "[turn_rtk_finish] direction={}, target={:.2f}, current={:.2f}, delta={:.2f}, elapsed={:.2f}s".format(
                        direction, target_heading, current_heading, delta, elapsed
                    )
                )
                sendBraking()
                return 1
            previous_delta = delta

        if elapsed - last_log_at >= 1.0:
            logger.info(
                "[turn_rtk_wait] direction={}, target={:.2f}, current={}, delta={}, elapsed={:.2f}s".format(
                    direction, target_heading, current_heading, delta, elapsed
                )
            )
            last_log_at = elapsed
        time.sleep(0.05)

    sendBraking()
    return 0


def turn(ser, roundTo, target_heading=None, source='turn', segment_index=None, task_id=None):
    normalized_target_heading = _coerce_float(target_heading, None)
    if normalized_target_heading is not None:
        return _turn_to_heading_by_rtk(
            ser,
            normalized_target_heading,
            source=source,
            segment_index=segment_index,
            task_id=task_id,
        )

    # 转向控制入口：给下位机下发旋转命令，并等待旋转完成。
    # roundTo 是下位机协议使用的角度值，调用方通常会传入 angle * 10。
    logger.warn('转向:{}'.format(roundTo))
    # 每次转向前先清掉超声波报警标记，避免上一次报警状态影响本次转向流程。
    redis_cli.set("ultraSonic", "false")
    # 如果运行时已经收到停止请求，直接退出，不再给下位机发送新的转向命令。
    if _is_runtime_stop_requested():
        return
    global global_status
    # 重新组装下位机命令：状态 3 表示旋转，RotateTo 是目标旋转角度，ZSpeed 是旋转速度。
    reSetStatus(ser)
    setStatus(3)
    setRotateTo(roundTo)
    setZSpeed(0.6)
    # 重新计算校验位后，通过串口把转向命令发送给下位机。
    command[17] = tem_listener(command, 17)
    logger.warn(' '.join(format(x, '02x') for x in command))
    duplicateWriteCmd(ser, command)

    # target_heading 用于 RTK 兜底判断：下位机没有及时回完成时，用实时航向判断是否已转到位。
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

    # 阻塞等待旋转完成：只要当前运行任务还有效，就持续检查下位机和 RTK 航向。
    while _is_runtime_task_active():
        # 第一优先级：以下位机返回的旋转完成标志为准。
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
        # 没有传目标 RTK 航向时，只能继续等待下位机完成标志。
        if target_heading is None:
            continue

        # 第二优先级：RTK 兜底判断。读取当前航向，计算当前航向与目标航向的最短角度差。
        elapsed = time.time() - turn_start_at
        current_heading = _get_current_rtk_heading()
        delta = _normalize_heading_delta(current_heading, target_heading)
        crossed_target = False
        # 给下位机留出最小等待时间，超过后才启用 RTK 兜底，避免刚开始转向就误判完成。
        if delta is not None and elapsed >= TURN_RTK_FALLBACK_MIN_WAIT_SEC:
            # 如果角度差从正到负或从负到正，说明车头已经越过目标航向，也可以认为转向到位。
            crossed_target = _heading_delta_crossed_target(previous_delta, delta)
            # 连续多次进入允许误差范围，才认为航向稳定到位，减少 RTK 瞬时抖动导致的误判。
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
                # RTK 判断已经到位后，主动刹车停止旋转，并把本次转向视为成功。
                sendBraking()
                turn_result = 1
                break
        else:
            stable_count = 0

        # 保存上一次航向偏差，下一轮用于判断是否已经跨过目标航向。
        if delta is not None:
            previous_delta = delta

        # 每 2 秒打印一次等待日志，方便排查转向卡住时的目标航向、当前航向和偏差。
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

    # 转向流程结束后，把车辆状态恢复为可行走状态；调用方会根据返回值决定是否进入直行。
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
    if _is_runtime_stop_requested():
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
    if _is_runtime_stop_requested():
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
    if _is_runtime_stop_requested():
        return

    pathPlanning = redis_cli.get(PATH_PLANNING_KEY)

    while _is_runtime_task_active():

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

        if not moveDiatance(ser, 100) or _is_runtime_stop_requested():
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

        if not moveDiatance(ser, 100) or _is_runtime_stop_requested():
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
    _mark_runtime_running('视觉清扫启动', {'action': 'auto_drive'})

    redis_cli.set("correct", "true")

    redis_cli.set('action', 'true')


def drive_up():
    init_status()

    redis_cli.set("driveUp", "true")

    switch_on_clean_mode(ser)

    if _is_runtime_stop_requested():
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

    while getEdge() == "1" and _is_runtime_task_active():
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
    _mark_runtime_complete('RTK清扫任务结束', {'action': 'auto_drive'})
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
        moveBack(ser, 150)
        if _is_runtime_stop_requested():
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
    _mark_runtime_complete('清扫任务结束', {'action': 'auto_drive'})
    redis_cli.set("correct", "false")
    redis_cli.set('action', 'false')

    # 如果自动清扫被停止，则不继续运行
    if not _is_runtime_stop_requested():
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
        _mark_runtime_running('清扫任务执行中', {'action': 'go_on' if goon else 'auto_drive'})
        redis_cli.set("correct", "true")
        redis_cli.set('action', 'true')
        reset_odometer(ser)

        logger.warn('任务获取成功！开始执行任务')
        for index, item in enumerate(tmpTaskList):
            # 通过重redis中获取一个值，看是否是暂停,0为false,1为true
            if not _is_runtime_stop_requested():
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
                if not _is_runtime_stop_requested():
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
        # redis_cli.set("correct", "true")
        # redis_cli.set('action', 'true')
        _mark_runtime_running('RTK清扫任务执行中', {'action': 'auto_drive'})
        reset_odometer(ser)

        logger.warn('任务获取成功！开始执行任务')
        for index, item in enumerate(tmpTaskList):
            # 通过重redis中获取一个值，看是否是暂停,0为false,1为true
            if not _is_runtime_stop_requested():
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
                if not _is_runtime_stop_requested():
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

    _mark_runtime_running('视觉清扫任务启动', {'action': 'auto_drive'})
    redis_cli.set("correct", "true")
    redis_cli.set('action', 'true')

    reset_odometer(ser)

    taskList = util.readConfig("config_view.json")

    # 如果启动自动清扫任务，那redis中的任务列表就要被清除，然后再初始化
    redis_cli.delete('taskList')
    # 将任务存储到Redis列表
    for item in taskList:
        # 将字典转为JSON字符串存储
        redis_cli.rpush('taskList', json.dumps(item))
    # while not _is_runtime_stop_requested():
    logger.warn('任务获取成功！开始执行任务')
    for item in taskList:
        global_go = 1
        # 通过重redis中获取一个值，看是否是暂停,0为false,1为true
        if not _is_runtime_stop_requested():
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
    _mark_runtime_complete('视觉清扫任务结束', {'action': 'auto_drive'})
    redis_cli.set("correct", "false")
    redis_cli.set('action', 'false')
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
            logging.info('监听报错:{}'.format(e))
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
    global global_status,drive_thread,global_doCleanThreadStop, global_auto_clean_stop

    while True:
        # 如果voltageLisener为1表示开启电池监听
        if redis_cli.get('voltageListener') != '0':
            voltage = redis_cli.get("voltage")
            logger.info("status={},voltage={}".format(global_status,voltage))
            # 当小车状态不是返回充电桩时并且需要返回充电桩充电时,就立即停止小车，并返回充电桩充电
            if not _is_runtime_returning_to_charge() and isNeedReturnCharging():
                _disable_loop_auto_clean('low_battery_return')
                _set_auto_resume_allowed(True, 'low_battery_return')
                global_doCleanThreadStop = 1
                global_auto_clean_stop = 1
                doParking()
                if global_doCleanThreadStop == 1:
                    logger.warning("返回充电桩")
                    thread = threading.Thread(target=returnToPointByRTKThread)
                    thread.start()

            # 当电量大于93时并且有未完成的任务，则继续清扫未完成的任务
            if _is_auto_resume_allowed() and _coerce_int(voltage, 0) >= 90 and redis_cli.llen('taskList') != 0:
                # 状态等于工作状态或者不等于从充电桩后退的状态，就可以启动
                if not _is_runtime_task_active():
                    if drive_thread is None or not drive_thread.is_alive():
                        drive_thread, error_payload = _start_runtime_thread(
                            'auto_drive',
                            autoDriveByRTKThread,
                            ready_message='电量恢复，继续未完成自动清扫任务'
                        )
                        if error_payload:
                            logger.warning("auto resume rejected: {}".format(error_payload))
        # 防止cpu过载
        time.sleep(1)



def _is_rtk_guard_task_active():
    fsm_state = robot_lifecycle_fsm.get_state()
    control_state = str(fsm_state.get('controlState') or '').upper()
    return control_state in ('RUNNING', 'PAUSED')


def _resume_current_rtk_segment():
    if global_go != 1 or not global_cur_taskPoint:
        return
    speed = _coerce_int(global_cur_taskPoint.get('speed'), _coerce_int(redis_cli.get("forwardSpeed"), high_speed))
    logger.warning("RTK fixed recovered; resume current segment with speed={}".format(speed))
    goCommand(speed)


def _stop_task_after_rtk_timeout(detail):
    global global_doCleanThreadStop, global_pointToPoint_flag, global_go, global_status
    logger.error("RTK fixed recovery timeout; stop current task")
    timeout_detail = dict(detail or {})
    timeout_detail.update({
        'rtkFixState': 'RTK_FIX_TIMEOUT',
        'rtkRecovering': False,
        'rtkLostSeconds': round(max(0.0, time.time() - global_rtk_fix_lost_time), 1) if global_rtk_fix_lost_time else None,
    })
    _disable_loop_auto_clean('rtk_fix_timeout')
    global_doCleanThreadStop = 1
    global_pointToPoint_flag = 1
    global_go = 0
    doParking(update_runtime=False)
    _mark_runtime_blocked(
        'RTK_FIX_TIMEOUT',
        'RTK fixed solution did not recover within 5 minutes; task stopped',
        timeout_detail
    )
    global_status = 'active'


def rtk_watchdog_thread():
    global global_rtk_fix_lost_time, global_rtk_recovering
    while True:
        try:
            if not _is_rtk_guard_task_active():
                if global_rtk_recovering:
                    global_rtk_fix_lost_time = None
                    global_rtk_recovering = False
                time.sleep(1)
                continue

            status = _get_rtk_runtime_status()
            if _is_rtk_fixed_status(status):
                if global_rtk_recovering:
                    global_rtk_fix_lost_time = None
                    global_rtk_recovering = False
                    detail = _build_rtk_runtime_detail(status)
                    logger.warning("RTK fixed solution recovered; continue task")
                    _mark_runtime_running('RTK fixed solution recovered; task resumed', detail)
                    _resume_current_rtk_segment()
                else:
                    _set_redis_value('runtimeDetail', _build_runtime_detail(_build_rtk_runtime_detail(status)))
                time.sleep(1)
                continue

            now = time.time()
            reason = _rtk_fix_problem_reason(status)
            if not global_rtk_recovering:
                global_rtk_fix_lost_time = now
                global_rtk_recovering = True
                logger.warning("RTK fixed solution lost; reason={}, stop and wait for recovery".format(reason))

            detail = _build_rtk_runtime_detail(status)
            lost_seconds = 0.0 if global_rtk_fix_lost_time is None else now - global_rtk_fix_lost_time
            if lost_seconds >= RTK_FIX_RECOVERY_TIMEOUT_SECONDS:
                _stop_task_after_rtk_timeout(detail)
                global_rtk_fix_lost_time = None
                global_rtk_recovering = False
                time.sleep(1)
                continue

            sendBraking()
            _mark_runtime_rtk_recovering(
                'RTK fixed solution lost; vehicle stopped and waiting for recovery',
                detail
            )
        except Exception as e:
            logger.error("rtk_watchdog_thread error: {}".format(e))
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


def _cleaning_position_sample():
    return (global_cur_rtk_lat, global_cur_rtk_lon,
            bool(_build_rtk_runtime_detail().get('rtkFixAvailable')))


def _begin_cleaning_position_run(task_config, task_token, resume=False):
    """Called only after a real cleaning start passed task validation."""
    try:
        service = globals().get('cleaning_position_service')
        if service is not None:
            service.begin(task_config, task_token, resume=resume, sample=_cleaning_position_sample())
    except Exception as error:
        logger.warning('begin cleaning telemetry: {}'.format(error))


def _notify_cleaning_position_state(control_state):
    try:
        service = globals().get('cleaning_position_service')
        if service is not None:
            service.state(control_state, active_runtime_task_token, _cleaning_position_sample())
    except Exception as error:
        logger.warning('cleaning telemetry state: {}'.format(error))


def _notify_cleaning_position_start_failed():
    """Publish a rejected start without creating or clearing route history."""
    try:
        service = globals().get('cleaning_position_service')
        if service is not None:
            service.start_failed()
    except Exception as error:
        logger.warning('cleaning telemetry start failure: {}'.format(error))


def _cleaning_position_history_loop():
    """Separate UI worker: never blocks the RTK steering observer."""
    while True:
        try:
            state = robot_lifecycle_fsm.get_state().get('controlState')
            cleaning_position_service.poll(_cleaning_position_sample(), state, active_runtime_task_token)
        except Exception as error:
            logger.warning('cleaning position sampler: {}'.format(error))
        time.sleep(0.2)


def _update_modeling_position_history(lat=None, lon=None, force_context=False):
    """Feed the read-only LAN position cache from the latest RTK snapshot."""
    if lat is None:
        lat = global_cur_rtk_lat
    if lon is None:
        lon = global_cur_rtk_lon
    rtk_detail = _build_rtk_runtime_detail()
    return modeling_position_history.update(
        lat,
        lon,
        bool(rtk_detail.get('rtkFixAvailable')),
        force_context=force_context,
    )


def _get_lan_realtime_position():
    raw = dict(_update_modeling_position_history(force_context=True) or {})
    raw['heading'] = (
        _get_current_rtk_heading()
        if raw.get('rtkFixAvailable') else None
    )
    return raw


def _get_lan_position_history(area_number=None):
    _update_modeling_position_history(force_context=True)
    return modeling_position_history.history(area_number=area_number)


def _get_lan_cleaning_realtime_position():
    """Add current task-origin presence without changing x/y availability."""
    raw = cleaning_position_service.realtime()
    raw = dict(raw or {})
    raw['heading'] = (
        _get_current_rtk_heading()
        if raw.get('rtkFixAvailable') else None
    )
    try:
        origin_status = _build_task_origin_status_fields(_load_task_params_snapshot())
        raw['atTaskOrigin'] = bool(
            raw.get('rtkFixAvailable') and
            origin_status.get('isAtTaskOrigin') is True
        )
    except Exception:
        raw['atTaskOrigin'] = False
    return raw


def _modeling_position_history_loop():
    """Sample RTK for the UI without blocking the RTK correction callback."""
    last_error_at = 0.0
    while True:
        try:
            _update_modeling_position_history()
        except Exception as e:
            now = time.time()
            if now - last_error_at >= 10.0:
                logger.warning("更新建模实时位置失败: {}".format(str(e)))
                last_error_at = now
        time.sleep(0.2)


def _get_lan_cloud_command_handler():
    """Reuse the exact MQTT command router for direct LAN HTTP requests."""
    integration = global_mqtt_integration
    return integration.command_handler if integration is not None else None


# The LAN facade only translates cloud-compatible HTTP requests.  All motion,
# modeling and route commands still pass through MQTTCommandHandler and the
# existing local vehicle/modeling endpoints.
lan_cloud_compatibility = register_lan_cloud_compat_routes(
    app,
    _get_lan_cloud_command_handler,
    realtime_position_provider=_get_lan_realtime_position,
    position_history_provider=_get_lan_position_history,
    cleaning_realtime_provider=_get_lan_cleaning_realtime_position,
    cleaning_history_provider=cleaning_position_service.history,
)


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

def main():
    _mark_runtime_initializing('系统启动，正在初始化配置和运行环境', 'main_start')
    try:
        # if not varifyGps():
        #     return
        # 视觉线程Opencv
        if cap is not None and cap.isOpened():
            thread = threading.Thread(target=startOpencv)
            thread.start()
        else:
            logger.warning("camera unavailable; skip startOpencv thread")
        # 下位机线程
        listener_thread = threading.Thread(target=listenerSlavePort)
        listener_thread.start()

        # # 监听RTK模块线程
        watchdog_thread = threading.Thread(target=rtk_watchdog_thread)
        watchdog_thread.start()
        listener_rtk_thread = threading.Thread(target=listenerRTK)
        listener_rtk_thread.start()

        # UI-only position sampling is isolated from the RTK observer so file
        # persistence and frontend reads cannot delay steering correction.
        position_history_thread = threading.Thread(target=_modeling_position_history_loop)
        position_history_thread.daemon = True
        position_history_thread.start()

        cleaning_history_thread = threading.Thread(target=_cleaning_position_history_loop)
        cleaning_history_thread.daemon = True
        cleaning_history_thread.start()

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

        _mark_runtime_initialized('初始化完成，已停车待命')

        # 启动flask后台服务
        # LAN cloud-compatible query endpoints reuse the proven local HTTP
        # adapter.  Keep Flask threaded so a facade request can call the
        # existing /vehicle or /modeling endpoint without self-deadlocking.
        # Modern Flask already defaults to threaded mode; the explicit option
        # also protects the older Python 2 Flask version used on the Jetson.
        app.run(host='0.0.0.0', port=7899, threaded=True)
    except Exception as exc:
        _mark_runtime_blocked(
            'INIT_FAILED',
            '初始化失败: {}'.format(str(exc)),
            _initialization_detail('failed', False, {'initializationError': str(exc)})
        )
        raise


if __name__ == '__main__':
    main()
