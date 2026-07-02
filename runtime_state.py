# coding=utf-8

import time


RUNTIME_STATE_KEY = "runtimeState"


STATE_LABELS = {
    "INITIALIZING": "初始化中",
    "IDLE": "空闲待命",
    "READY": "启动就绪",
    "RUNNING": "运行中",
    "PAUSED": "暂停等待",
    "STOPPING": "停止中",
    "STOPPED": "已停车",
    "COMPLETE": "任务完成",
    "BLOCKED": "阻塞待处理",
    "FAULT": "故障",
    "DISABLED": "禁用",
    "UNKNOWN": "状态未知",
}


ACTION_LABELS = {
    "idle": "空闲",
    "parking": "停车",
    "auto_drive": "自动清扫",
    "go_on": "继续任务",
    "return_to_point": "返回充电点",
    "enter_garage": "入舱",
    "exit_garage": "出舱",
    "turn_left": "左转",
    "turn_right": "右转",
    "reverse": "后退",
}


HEALTH_LABELS = {
    "OK": "正常",
    "WARN": "告警",
    "ERROR": "异常",
    "UNKNOWN": "未知",
}


FAULT_LABELS = {
    "INIT_FAILED": "初始化失败",
    "CURRENT_TASK_NOT_SET": "当前任务未设置",
    "TASK_ORIGIN_TOO_FAR": "当前位置距离任务起点过远",
    "RTK_FIX_LOST": "RTK 固定解丢失",
    "RTK_FIX_TIMEOUT": "RTK 固定解恢复超时",
    "NO_RTK_GGA": "未收到 RTK GGA 数据",
    "RTK_GGA_TIMEOUT": "RTK 数据超时",
    "RTK_NOT_FIXED": "RTK 未达到固定解",
    "LOWER_MACHINE_DISABLED": "下位机未使能",
    "LOWER_MACHINE_STATUS_UNKNOWN": "下位机状态未知",
}


EFFECT_LABELS = {
    "motionAllowed": "允许运动",
    "taskActive": "任务执行中",
    "startAllowed": "允许启动",
    "requiresAttention": "需要人工处理",
    "initializing": "初始化未完成",
}


def _text(value):
    if value is None:
        return ""
    return str(value).strip()


def _upper(value):
    return _text(value).upper()


def _bool(value):
    if isinstance(value, bool):
        return value
    text = _text(value).lower()
    if text in ("1", "true", "yes", "on", "working"):
        return True
    if text in ("0", "false", "no", "off", "none", ""):
        return False
    try:
        return int(float(text)) != 0
    except Exception:
        return False


def _int_or_none(value):
    if value in (None, ""):
        return None
    try:
        return int(float(value))
    except Exception:
        return None


def _rtk_state(detail):
    detail = detail if isinstance(detail, dict) else {}
    if detail.get("rtkFixState"):
        return _text(detail.get("rtkFixState"))
    if detail.get("rtkFixAvailable") is True:
        return "FIXED"
    if detail.get("rtkFixAvailable") is False:
        return "NOT_FIXED"
    return ""


def _lifecycle_state(control_state, mission, parking, action, fault_state):
    control = _upper(control_state)
    mission_value = _text(mission).lower()
    is_parking = _bool(parking)
    action_value = _text(action)

    if control in ("INITIALIZING", "READY", "RUNNING", "PAUSED", "STOPPING", "STOPPED", "BLOCKED", "FAULT", "DISABLED", "UNKNOWN"):
        return control
    if control == "IDLE":
        if is_parking:
            return "STOPPED"
        if mission_value == "complete":
            return "COMPLETE"
        return "IDLE"
    if fault_state:
        return "BLOCKED"
    if action_value == "return_to_point":
        return "RUNNING"
    if mission_value == "working":
        return "RUNNING"
    if is_parking:
        return "STOPPED"
    if mission_value == "complete":
        return "COMPLETE"
    return "IDLE"


def _effects(state, health, fault):
    state = _upper(state)
    health = _upper(health)
    has_fault = bool(_text(fault))
    requires_attention = state in ("BLOCKED", "FAULT", "DISABLED", "UNKNOWN") or health in ("WARN", "ERROR") or has_fault
    return {
        "motionAllowed": state == "RUNNING",
        "taskActive": state in ("READY", "RUNNING", "PAUSED", "STOPPING"),
        "startAllowed": state in ("IDLE", "READY", "STOPPED", "COMPLETE") and not requires_attention,
        "requiresAttention": requires_attention,
        "initializing": state == "INITIALIZING",
    }


def _label(mapping, value):
    text = _text(value)
    if not text:
        return ""
    return mapping.get(text, mapping.get(text.upper(), text))


def _effect_labels(effects):
    return [label for key, label in EFFECT_LABELS.items() if effects.get(key)]


def build_runtime_state_snapshot(control_state=None, health_state=None, fault_state=None,
                                 mission=None, parking=None, action=None,
                                 task_name=None, task_index=None, start_ready=None,
                                 message=None, detail=None, now=None):
    detail = detail if isinstance(detail, dict) else {}
    health = _upper(health_state) or ("WARN" if fault_state else "OK")
    fault = _text(fault_state)
    state = _lifecycle_state(control_state, mission, parking, action, fault)
    effects = _effects(state, health, fault)
    timestamp = int(time.time() if now is None else now)
    return {
        "schemaVersion": 1,
        "state": state,
        "stateLabel": _label(STATE_LABELS, state),
        "controlState": _upper(control_state),
        "action": _text(action),
        "actionLabel": _label(ACTION_LABELS, action),
        "health": health,
        "healthLabel": _label(HEALTH_LABELS, health),
        "fault": fault,
        "faultLabel": _label(FAULT_LABELS, fault),
        "mission": _text(mission),
        "parking": _bool(parking),
        "task": {
            "name": _text(task_name),
            "index": _int_or_none(task_index),
        },
        "rtk": {
            "state": _rtk_state(detail),
        },
        "startReady": _bool(start_ready),
        "message": _text(message),
        "detail": detail,
        "effects": effects,
        "effectLabels": _effect_labels(effects),
        "updatedAt": timestamp,
    }
