# coding=utf-8

import json
import os
import time

from dev_console.correction_state import (
    _decode,
    _get_hash,
    _get_value,
    _parse_json,
    _rtk_fixed,
    _to_bool,
    _to_float,
    _to_int,
)


REDIS_GROUPS = {
    "vehicle": [
        "mission",
        "parking",
        "runtimeState",
        "runtimeEvents",
        "currentAction",
        "controlState",
        "healthState",
        "faultState",
        "forwardSpeed",
        "brushSpeed",
        "batteryPercent",
        "packVoltage",
        "voltage",
        "garageState",
    ],
    "task": [
        "currentTaskName",
        "curTaskIndex",
        "taskParams",
        "lastTask",
        "runtimeDetail",
        "startCheckReady",
        "startCheckReason",
        "correctionDebug",
    ],
    "rtk": [
        "currentLocation",
        "edgeDistanceToTargetM",
    ],
    "loop": [
        "loopAutoCleanEnabled",
        "loopAutoCleanRunning",
        "loopAutoCleanCycle",
        "loopAutoCleanStopReason",
    ],
}


def build_overview_state(redis_client):
    detail = _parse_json(_get_value(redis_client, "runtimeDetail", "{}"), {})
    runtime_state = _parse_json(_get_value(redis_client, "runtimeState", "{}"), {})
    if not isinstance(runtime_state, dict):
        runtime_state = {}
    return {
        "runtimeState": runtime_state,
        "mission": _get_value(redis_client, "mission", ""),
        "parking": _to_bool(_get_value(redis_client, "parking"), False),
        "currentAction": _get_value(redis_client, "currentAction", ""),
        "controlState": _get_value(redis_client, "controlState", ""),
        "healthState": _get_value(redis_client, "healthState", ""),
        "faultState": _get_value(redis_client, "faultState", ""),
        "batteryPercent": _to_float(_get_value(redis_client, "batteryPercent")),
        "packVoltage": _to_float(_get_value(redis_client, "packVoltage")),
        "garageState": _get_value(redis_client, "garageState", ""),
        "forwardSpeed": _to_int(_get_value(redis_client, "forwardSpeed"), None),
        "brushSpeed": _to_int(_get_value(redis_client, "brushSpeed"), None),
        "rtkFixed": _rtk_fixed(detail if isinstance(detail, dict) else {}),
        "currentLocation": _get_hash(redis_client, "currentLocation"),
        "currentTaskName": _get_value(redis_client, "currentTaskName", ""),
        "curTaskIndex": _to_int(_get_value(redis_client, "curTaskIndex"), None),
        "updatedAt": int(time.time()),
    }


def _redis_task_list(redis_client):
    try:
        raw_items = redis_client.lrange("taskList", 0, -1)
    except Exception:
        raw_items = []
    tasks = []
    for item in raw_items or []:
        parsed = _parse_json(item, None)
        if isinstance(parsed, dict):
            tasks.append(parsed)
    return tasks


def _task_summary(item, index, current_index):
    return {
        "index": index,
        "id": item.get("id"),
        "current": current_index is not None and index == current_index,
        "mode": item.get("mode"),
        "angle": item.get("angle"),
        "heading": item.get("heading"),
        "length": item.get("length"),
        "startX": item.get("startX"),
        "startY": item.get("startY"),
        "endX": item.get("endX"),
        "endY": item.get("endY"),
        "startLat": item.get("startLat"),
        "startLon": item.get("startLon"),
        "endLat": item.get("endLat"),
        "endLon": item.get("endLon"),
        "back_len": item.get("back_len"),
        "turn_back_len": item.get("turn_back_len"),
        "areaNumber": item.get("areaNumber"),
    }


def build_task_path_state(redis_client, config_path="config.json"):
    task_name = _get_value(redis_client, "currentTaskName", "")
    current_index = _to_int(_get_value(redis_client, "curTaskIndex"), None)
    tasks = _redis_task_list(redis_client)
    source = "redis" if tasks else "none"
    return {
        "taskName": task_name,
        "source": source,
        "currentIndex": current_index,
        "count": len(tasks),
        "tasks": [_task_summary(item, index, current_index) for index, item in enumerate(tasks)],
        "updatedAt": int(time.time()),
    }


def _read_redis_key(redis_client, key):
    hash_value = _get_hash(redis_client, key)
    if hash_value:
        return hash_value
    try:
        list_value = redis_client.lrange(key, 0, -1)
    except Exception:
        list_value = None
    if list_value:
        return [_decode(item) for item in list_value]
    return _get_value(redis_client, key, "")


def build_redis_state(redis_client):
    groups = {}
    for group_name, keys in REDIS_GROUPS.items():
        groups[group_name] = {}
        for key in keys:
            groups[group_name][key] = _read_redis_key(redis_client, key)

    try:
        all_keys = [_decode(key) for key in redis_client.keys("*")]
    except Exception:
        all_keys = []

    return {
        "groups": groups,
        "allKeys": sorted([key for key in all_keys if key]),
        "updatedAt": int(time.time()),
    }


def read_log_lines(log_path="app.log", query="", limit=200):
    query = query or ""
    limit = max(1, min(int(limit or 200), 1000))
    if not log_path or not os.path.exists(log_path):
        return {
            "path": log_path,
            "query": query,
            "lines": [],
            "exists": False,
        }

    with open(log_path, "r", errors="replace") as fp:
        lines = [line.rstrip("\r\n") for line in fp.readlines()]
    if query:
        needle = query.lower()
        lines = [line for line in lines if needle in line.lower()]
    return {
        "path": log_path,
        "query": query,
        "lines": lines[-limit:],
        "exists": True,
    }
