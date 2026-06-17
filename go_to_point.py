# coding=utf-8
from __future__ import division

import math

import util


DEFAULT_GO_TO_POINT_SPEED = 200
DEFAULT_MIN_DISTANCE_M = 0.3


def _to_float(value):
    if value is None or value == "":
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(result) or math.isinf(result):
        return None
    return result


def _to_int(value, default_value):
    try:
        return int(value)
    except (TypeError, ValueError):
        return default_value


def _response(success, code, msg, data=None):
    result = {
        "success": success,
        "code": code,
        "msg": msg,
    }
    if data is not None:
        result["data"] = data
    return result


def build_go_to_point_plan(
    current_lat,
    current_lon,
    target_lat,
    target_lon,
    speed=DEFAULT_GO_TO_POINT_SPEED,
    min_distance_m=DEFAULT_MIN_DISTANCE_M,
):
    current_lat = _to_float(current_lat)
    current_lon = _to_float(current_lon)
    target_lat = _to_float(target_lat)
    target_lon = _to_float(target_lon)
    speed = _to_int(speed, DEFAULT_GO_TO_POINT_SPEED)

    if target_lat is None or target_lon is None:
        return _response(False, "INVALID_TARGET", "目标经纬度无效")

    if current_lat is None or current_lon is None:
        return _response(False, "CURRENT_RTK_UNAVAILABLE", "当前RTK不可用，无法前往目标点")

    distance, heading = util.get_distance_angle(current_lat, current_lon, target_lat, target_lon)
    distance = round(float(distance), 3)
    heading = float(heading) % 360

    data = {
        "startLat": current_lat,
        "startLon": current_lon,
        "targetLat": target_lat,
        "targetLon": target_lon,
        "distance": distance,
        "heading": round(heading, 3),
        "speed": speed,
    }

    if distance < float(min_distance_m):
        return _response(False, "TARGET_TOO_CLOSE", "目标点距离过近，不执行行驶", data)

    return _response(True, "GO_TO_POINT_READY", "前往目标点可以启动", data)
