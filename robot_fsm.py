# coding=utf-8

"""机器人运行生命周期状态机。

本文件只管理“机器人现在处于什么状态、某个状态是否允许切换到下一状态”，不负责生成
清扫路线，也不直接控制电机。主程序把初始化、启动、暂停、RTK 丢失、完成、故障等事件
交给状态机，再把得到的统一状态通过 HTTP/MQTT 上报给云平台。

状态机的价值是让本地控制页、MQTT 命令和自动清扫线程使用同一套状态含义，避免一处显示
RUNNING、另一处仍认为 STOPPED。
"""

import threading
import time


try:
    # 小车现场程序仍可能使用 Python 2，而开发机使用 Python 3：
    # Python 2 的 unicode/str 与 Python 3 的 str/bytes 含义不同，因此先统一类型别名。
    text_type = unicode
    binary_type = str
except NameError:
    text_type = str
    binary_type = bytes


RUNTIME_EVENT_LOG_KEY = "runtimeEvents"


EVENT_INIT_STARTED = "INIT_STARTED"
EVENT_INIT_SUCCEEDED = "INIT_SUCCEEDED"
EVENT_INIT_FAILED = "INIT_FAILED"
EVENT_START_CHECK_PASSED = "START_CHECK_PASSED"
EVENT_TASK_STARTED = "TASK_STARTED"
EVENT_TASK_PAUSED = "TASK_PAUSED"
EVENT_TASK_STOPPING = "TASK_STOPPING"
EVENT_TASK_STOPPED = "TASK_STOPPED"
EVENT_TASK_FINISHED = "TASK_FINISHED"
EVENT_TASK_BLOCKED = "TASK_BLOCKED"
EVENT_FAULT_OCCURRED = "FAULT_OCCURRED"
EVENT_DISABLED = "DISABLED"
EVENT_RTK_LOST = "RTK_LOST"
EVENT_RTK_RECOVERED = "RTK_RECOVERED"
EVENT_STATE_UPDATED = "STATE_UPDATED"


DEFAULT_RUNTIME_STATE = {
    "controlState": "STOPPED",
    "healthState": "OK",
    "faultState": "",
    "startReady": False,
    "action": "idle",
    "message": "",
    "detail": {},
    "eventType": "",
}


def _text(value):
    """把 Redis、MQTT 和 Python 内部的文本统一为去除首尾空白的 Unicode 文本。

    MQTT/Redis 在不同 Python 版本中可能返回 str、unicode 或 bytes。如果直接使用 str(value)，
    Python 2 遇到中文时可能触发隐式 ASCII 编码错误；这里显式按 UTF-8 解码，损坏字节则用
    replacement 字符代替，保证状态上报不会因为一条中文消息中断。
    """
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


def _payload_bool(value):
    """兼容布尔值、数字和字符串形式的开关量，供 startReady 等状态字段使用。"""
    if isinstance(value, bool):
        return value
    text = _text(value).lower()
    if text in ("1", "true", "yes", "on", "ready"):
        return True
    if text in ("0", "false", "no", "off", "none", ""):
        return False
    return bool(value)


class RobotEventBus(object):
    """线程安全的轻量事件总线，同时保留最近事件用于现场排查。

    发布事件时先在锁内记录快照，再在锁外调用订阅函数，避免某个处理器耗时或再次发布事件
    时阻塞整个状态机。``*`` 订阅者可以接收全部事件。
    """
    def __init__(self, max_events=200):
        self.max_events = int(max_events or 200)
        self._events = []
        self._handlers = {}
        self._sequence = 0
        self._lock = threading.RLock()

    def subscribe(self, event_type, handler):
        """为指定事件注册处理函数；event_type='*' 表示监听所有事件。"""
        event_type = _text(event_type)
        with self._lock:
            self._handlers.setdefault(event_type, []).append(handler)

    def publish(self, event_type, source="", message="", payload=None, now=None):
        """记录并分发一个运行事件，返回写入事件历史的标准化事件对象。"""
        with self._lock:
            self._sequence += 1
            event = {
                "sequence": self._sequence,
                "type": _text(event_type),
                "source": _text(source),
                "message": _text(message),
                "payload": payload if isinstance(payload, dict) else {},
                "createdAt": int(time.time() if now is None else now),
            }
            self._events.append(event)
            # 只保留最近 max_events 条，避免长期运行时事件历史无限占用内存。
            if len(self._events) > self.max_events:
                self._events = self._events[-self.max_events:]
            handlers = list(self._handlers.get(event["type"], [])) + list(self._handlers.get("*", []))

        # 回调放在锁外执行，防止业务处理器造成事件总线死锁。
        for handler in handlers:
            handler(event)
        return event

    def recent_events(self, limit=None):
        with self._lock:
            events = list(self._events)
        if limit is None:
            return events
        return events[-int(limit):]


class RobotLifecycleFSM(object):
    """根据运行事件维护机器人生命周期，并拒绝不合法的状态跳转。

    ``VALID_TRANSITIONS`` 定义“当前状态可以去哪里”；``EVENT_TRANSITIONS`` 定义“收到某类
    事件后希望变成什么状态”。二者分开后，事件即使给出目标状态，也必须经过合法性检查。
    例如 RUNNING 可以进入 PAUSED，但 STOPPED 不会因误发暂停事件直接进入 PAUSED。
    """

    # 表中列出每个 controlState 允许到达的下一状态；同状态重复事件也允许通过。
    VALID_TRANSITIONS = {
        "STOPPED": ("INITIALIZING", "READY", "RUNNING", "STOPPED", "COMPLETE", "BLOCKED", "DISABLED"),
        "INITIALIZING": ("INITIALIZING", "STOPPED", "BLOCKED", "FAULT", "DISABLED"),
        "READY": ("READY", "RUNNING", "STOPPED", "BLOCKED", "FAULT", "DISABLED"),
        "RUNNING": ("RUNNING", "PAUSED", "STOPPING", "COMPLETE", "STOPPED", "BLOCKED", "FAULT", "DISABLED"),
        "PAUSED": ("PAUSED", "RUNNING", "STOPPING", "STOPPED", "BLOCKED", "FAULT", "DISABLED"),
        "STOPPING": ("STOPPING", "STOPPED", "COMPLETE", "BLOCKED", "FAULT", "DISABLED"),
        "COMPLETE": ("READY", "RUNNING", "STOPPED", "COMPLETE", "BLOCKED", "DISABLED"),
        "BLOCKED": ("STOPPED", "READY", "INITIALIZING", "BLOCKED", "FAULT", "DISABLED"),
        "FAULT": ("STOPPED", "BLOCKED", "FAULT", "DISABLED"),
        "DISABLED": ("INITIALIZING", "STOPPED", "DISABLED"),
        "UNKNOWN": ("INITIALIZING", "STOPPED", "BLOCKED", "UNKNOWN"),
    }

    # 每个业务事件对应的标准状态修改。payload 只覆盖允许动态补充的字段，不能绕过此表。
    EVENT_TRANSITIONS = {
        EVENT_INIT_STARTED: {
            "controlState": "INITIALIZING",
            "healthState": "OK",
            "faultState": "",
            "startReady": False,
            "action": "idle",
        },
        EVENT_INIT_SUCCEEDED: {
            "controlState": "STOPPED",
            "healthState": "OK",
            "faultState": "",
            "startReady": False,
            "action": "idle",
        },
        EVENT_START_CHECK_PASSED: {
            "controlState": "READY",
            "healthState": "OK",
            "faultState": "",
            "startReady": True,
        },
        EVENT_TASK_STARTED: {
            "controlState": "RUNNING",
            "healthState": "OK",
            "faultState": "",
            "startReady": True,
        },
        EVENT_TASK_PAUSED: {
            "controlState": "PAUSED",
            "healthState": "OK",
            "faultState": "",
            "startReady": False,
        },
        EVENT_TASK_STOPPING: {
            "controlState": "STOPPING",
            "healthState": "OK",
            "faultState": "",
            "startReady": False,
        },
        EVENT_TASK_STOPPED: {
            "controlState": "STOPPED",
            "healthState": "OK",
            "faultState": "",
            "startReady": False,
            "action": "parking",
        },
        EVENT_TASK_FINISHED: {
            "controlState": "COMPLETE",
            "healthState": "OK",
            "faultState": "",
            "startReady": False,
            "action": "idle",
        },
        EVENT_TASK_BLOCKED: {
            "controlState": "BLOCKED",
            "healthState": "WARN",
            "faultState": "TASK_BLOCKED",
            "startReady": False,
            "action": "idle",
        },
        EVENT_FAULT_OCCURRED: {
            "controlState": "FAULT",
            "healthState": "ERROR",
            "faultState": "FAULT",
            "startReady": False,
            "action": "idle",
        },
        EVENT_DISABLED: {
            "controlState": "DISABLED",
            "healthState": "WARN",
            "faultState": "DISABLED",
            "startReady": False,
            "action": "idle",
        },
        EVENT_RTK_LOST: {
            "controlState": "PAUSED",
            "healthState": "WARN",
            "faultState": "RTK_FIX_LOST",
            "startReady": False,
        },
        EVENT_RTK_RECOVERED: {
            "controlState": "RUNNING",
            "healthState": "OK",
            "faultState": "",
            "startReady": True,
        },
        EVENT_INIT_FAILED: {
            "controlState": "BLOCKED",
            "healthState": "WARN",
            "faultState": "INIT_FAILED",
            "startReady": False,
            "action": "idle",
        },
        EVENT_STATE_UPDATED: {},
    }

    def __init__(self, initial_state=None):
        state = dict(DEFAULT_RUNTIME_STATE)
        if isinstance(initial_state, dict):
            state.update(initial_state)
        self._state = state
        self._lock = threading.RLock()

    def get_state(self):
        """返回当前状态副本，防止调用方直接修改状态机内部字典。"""
        with self._lock:
            state = dict(self._state)
            detail = state.get("detail")
            state["detail"] = dict(detail) if isinstance(detail, dict) else {}
            return state

    def _is_transition_allowed(self, current_state, next_state):
        """按照 VALID_TRANSITIONS 判断本次生命周期跳转是否合法。"""
        current_state = _text(current_state or "UNKNOWN").upper()
        next_state = _text(next_state or "UNKNOWN").upper()
        if current_state == next_state:
            return True
        return next_state in self.VALID_TRANSITIONS.get(current_state, ())

    def apply_event(self, event_type, message="", payload=None):
        """将一个业务事件应用到当前状态，并返回应用后的完整状态。

        未知事件会尝试进入 UNKNOWN；非法跳转不会修改内部状态，而是在返回值中写入
        transitionAccepted=False、前一状态和请求状态，供日志及前端定位问题。
        """
        payload = payload if isinstance(payload, dict) else {}
        with self._lock:
            state = dict(self._state)
            previous_control_state = _text(state.get("controlState") or "UNKNOWN").upper()
            # 先从固定事件表取得目标状态；未知事件进入可诊断的 UNKNOWN 状态分支。
            if event_type in self.EVENT_TRANSITIONS:
                transition = dict(self.EVENT_TRANSITIONS.get(event_type, {}))
            else:
                transition = {
                    "controlState": "UNKNOWN",
                    "healthState": "WARN",
                    "faultState": "UNKNOWN_EVENT",
                    "startReady": False,
                    "action": "idle",
                }
            next_control_state = _text(
                transition.get("controlState") or previous_control_state
            ).upper()
            # 拒绝非法跳转时只构造返回快照，self._state 保持不变。
            if not self._is_transition_allowed(previous_control_state, next_control_state):
                rejected = self.get_state()
                rejected["transitionAccepted"] = False
                rejected["rejectedEvent"] = _text(event_type)
                rejected["requestedControlState"] = next_control_state
                rejected["previousControlState"] = previous_control_state
                rejected["message"] = _text(message)
                return rejected

            # 固定转换先应用，再允许 payload 补充故障、健康和动作等运行细节。
            state.update(transition)
            if payload.get("faultState") is not None:
                state["faultState"] = _text(payload.get("faultState"))
            if payload.get("healthState") is not None:
                state["healthState"] = _text(payload.get("healthState"))
            if payload.get("startReady") is not None:
                state["startReady"] = _payload_bool(payload.get("startReady"))
            if payload.get("action") is not None and "action" not in transition:
                state["action"] = _text(payload.get("action"))
            state["message"] = _text(message)
            state["detail"] = payload
            state["eventType"] = _text(event_type)
            state["transitionAccepted"] = True
            state["previousControlState"] = previous_control_state
            self._state = state
            return self.get_state()


def legacy_fields_for_state(state):
    """把新状态机字段转换为旧接口仍在使用的 mission/parking/currentAction。

    这是兼容旧控制页和既有 Redis 字段的输出适配，不会反向改变状态机，也不会影响任务
    路径。待所有调用方迁移到 runtimeState 后才可移除此兼容层。
    """
    state = state if isinstance(state, dict) else {}
    control_state = _text(state.get("controlState")).upper()
    action = _text(state.get("action") or "idle")
    if not action:
        action = "idle"

    if control_state in ("RUNNING", "PAUSED"):
        return {
            "mission": "working",
            "parking": "0",
            "currentAction": action,
        }
    if control_state == "STOPPING":
        return {
            "mission": "working",
            "parking": "1",
            "currentAction": action,
        }
    if control_state == "STOPPED":
        return {
            "mission": "complete",
            "parking": "1",
            "currentAction": "parking" if action == "parking" else "idle",
        }
    return {
        "mission": "complete",
        "parking": "1",
        "currentAction": "idle",
    }


def runtime_event_type_for_control_state(control_state, fault_state=""):
    """把外部读取到的状态快照反向映射成最接近的标准事件类型。

    BLOCKED 需要结合 faultState 区分初始化失败和普通任务阻塞；PAUSED 也要区分人为暂停与
    RTK 固定解丢失。无法识别时返回 STATE_UPDATED，避免伪造具体业务原因。
    """
    control_state = _text(control_state).upper()
    fault_state = _text(fault_state)
    if control_state == "INITIALIZING":
        return EVENT_INIT_STARTED
    if control_state == "READY":
        return EVENT_START_CHECK_PASSED
    if control_state == "RUNNING":
        return EVENT_TASK_STARTED
    if control_state == "PAUSED" and fault_state == "RTK_FIX_LOST":
        return EVENT_RTK_LOST
    if control_state == "PAUSED":
        return EVENT_TASK_PAUSED
    if control_state == "STOPPING":
        return EVENT_TASK_STOPPING
    if control_state == "STOPPED":
        return EVENT_TASK_STOPPED
    if control_state == "COMPLETE":
        return EVENT_TASK_FINISHED
    if control_state == "FAULT":
        return EVENT_FAULT_OCCURRED
    if control_state == "DISABLED":
        return EVENT_DISABLED
    if control_state == "BLOCKED" and fault_state == "INIT_FAILED":
        return EVENT_INIT_FAILED
    if control_state == "BLOCKED":
        return EVENT_TASK_BLOCKED
    return "STATE_UPDATED"
