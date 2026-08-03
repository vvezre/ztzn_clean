# coding=utf-8

"""把机器人内部的多组运行字段整理成统一的前端状态快照。

旧程序同时存在 controlState、mission、parking、currentAction 等字段，不同页面各自判断，
容易出现同一时刻显示不一致。本模块集中完成兼容、状态推导、中文名称和行为能力计算，
最终生成 runtimeState。它只解释状态，不发送控制命令，也不改变小车运行状态。
"""

import time


try:
    # 同时兼容小车 Python 2 与开发机 Python 3，避免中文状态标签被错误地按 ASCII 转换。
    text_type = unicode
    binary_type = str
except NameError:
    text_type = str
    binary_type = bytes


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
    """安全地把 str/unicode/bytes 转成 Unicode 文本，供全部状态字段统一使用。"""
    if value is None:
        return ""
    if isinstance(value, text_type):
        return value.strip()
    if isinstance(value, binary_type):
        try:
            return value.decode("utf-8").strip()
        except Exception:
            return value.decode("utf-8", "replace").strip()
    return text_type(value).strip()


def _upper(value):
    """标准化状态码；状态枚举统一用大写比较。"""
    return _text(value).upper()


def _bool(value):
    """解析 Redis/JSON 中常见的布尔表达，包括 0/1、true/false 和 working。"""
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
    """把任务序号等数值转换为整数；无效值返回 None，避免状态接口异常。"""
    if value in (None, ""):
        return None
    try:
        return int(float(value))
    except Exception:
        return None


def _rtk_state(detail):
    """从运行 detail 中提取 RTK 状态，优先使用明确状态码，再兼容旧布尔字段。"""
    detail = detail if isinstance(detail, dict) else {}
    if detail.get("rtkFixState"):
        return _text(detail.get("rtkFixState"))
    if detail.get("rtkFixAvailable") is True:
        return "FIXED"
    if detail.get("rtkFixAvailable") is False:
        return "NOT_FIXED"
    return ""


def _lifecycle_state(control_state, mission, parking, action, fault_state):
    """确定前端最终展示的生命周期状态。

    新版 controlState 优先级最高；只有它缺失或是旧的 IDLE 时，才结合 mission、parking、
    action 和 faultState 推导。这保证新状态机上线后不会被旧字段覆盖，同时仍能展示旧版本
    小车的状态。
    """
    control = _upper(control_state)
    mission_value = _text(mission).lower()
    is_parking = _bool(parking)
    action_value = _text(action)

    # 新状态机已经给出明确值时直接采用，不再用旧字段二次猜测。
    if control in ("INITIALIZING", "READY", "RUNNING", "PAUSED", "STOPPING", "STOPPED", "COMPLETE", "BLOCKED", "FAULT", "DISABLED", "UNKNOWN"):
        return control
    # IDLE 是旧程序的模糊状态，需要结合是否停车、任务是否完成进一步区分。
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
    """根据最终状态计算页面和控制逻辑可以直接使用的行为能力标记。

    ``startAllowed`` 仅表示状态层面允许显示/尝试启动；真正启动时仍需通过任务选择、RTK、
    下位机使能、起点距离等启动检查，不能把这里的 True 当作电机一定会启动。
    """
    state = _upper(state)
    health = _upper(health)
    has_fault = bool(_text(fault))
    requires_attention = state in ("BLOCKED", "FAULT", "DISABLED", "UNKNOWN") or health in ("WARN", "ERROR") or has_fault
    return {
        "motionAllowed": state == "RUNNING",
        "taskActive": state in ("RUNNING", "PAUSED", "STOPPING"),
        "startAllowed": state in ("IDLE", "STOPPED", "COMPLETE") and not requires_attention,
        "requiresAttention": requires_attention,
        "initializing": state == "INITIALIZING",
    }


def _label(mapping, value):
    """把状态码映射为中文；未知值保留原文，便于发现新增状态。"""
    text = _text(value)
    if not text:
        return ""
    return _text(mapping.get(text, mapping.get(text.upper(), text)))


def _effect_labels(effects):
    """把所有为 True 的行为能力转换成中文列表，便于页面和日志展示。"""
    return [_text(label) for key, label in EFFECT_LABELS.items() if effects.get(key)]


def build_runtime_state_snapshot(control_state=None, health_state=None, fault_state=None,
                                 mission=None, parking=None, action=None,
                                 task_name=None, task_index=None, start_ready=None,
                                 message=None, detail=None, now=None):
    """生成一次完整、可序列化的运行状态快照。

    输出同时包含机器字段（state/action/health/fault）、中文字段（各类 Label）、当前任务、
    RTK 摘要、启动就绪标志、行为能力 effects 和更新时间。云平台及前端读取这一份结构即可，
    不需要各自重复推断旧字段。
    """
    detail = detail if isinstance(detail, dict) else {}
    health = _upper(health_state) or ("WARN" if fault_state else "OK")
    fault = _text(fault_state)
    state = _lifecycle_state(control_state, mission, parking, action, fault)
    effects = _effects(state, health, fault)
    timestamp = int(time.time() if now is None else now)
    # schemaVersion 为以后扩展字段预留；现有前端可按版本决定兼容策略。
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
