# coding=utf-8

import threading
import time


RUNTIME_EVENT_LOG_KEY = "runtimeEvents"


EVENT_INIT_STARTED = "INIT_STARTED"
EVENT_INIT_SUCCEEDED = "INIT_SUCCEEDED"
EVENT_INIT_FAILED = "INIT_FAILED"
EVENT_START_CHECK_PASSED = "START_CHECK_PASSED"
EVENT_TASK_STARTED = "TASK_STARTED"
EVENT_TASK_STOPPED = "TASK_STOPPED"
EVENT_TASK_FINISHED = "TASK_FINISHED"
EVENT_TASK_BLOCKED = "TASK_BLOCKED"
EVENT_RTK_LOST = "RTK_LOST"
EVENT_RTK_RECOVERED = "RTK_RECOVERED"


class RobotEventBus(object):
    def __init__(self, max_events=200):
        self.max_events = int(max_events or 200)
        self._events = []
        self._handlers = {}
        self._sequence = 0
        self._lock = threading.RLock()

    def subscribe(self, event_type, handler):
        event_type = str(event_type or "")
        with self._lock:
            self._handlers.setdefault(event_type, []).append(handler)

    def publish(self, event_type, source="", message="", payload=None, now=None):
        with self._lock:
            self._sequence += 1
            event = {
                "sequence": self._sequence,
                "type": str(event_type or ""),
                "source": str(source or ""),
                "message": str(message or ""),
                "payload": payload if isinstance(payload, dict) else {},
                "createdAt": int(time.time() if now is None else now),
            }
            self._events.append(event)
            if len(self._events) > self.max_events:
                self._events = self._events[-self.max_events:]
            handlers = list(self._handlers.get(event["type"], [])) + list(self._handlers.get("*", []))

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
    EVENT_TRANSITIONS = {
        EVENT_INIT_STARTED: {
            "controlState": "INITIALIZING",
            "healthState": "OK",
            "faultState": "",
            "startReady": False,
        },
        EVENT_INIT_SUCCEEDED: {
            "controlState": "STOPPED",
            "healthState": "OK",
            "faultState": "",
            "startReady": False,
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
        EVENT_TASK_STOPPED: {
            "controlState": "STOPPED",
            "healthState": "OK",
            "faultState": "",
            "startReady": False,
        },
        EVENT_TASK_FINISHED: {
            "controlState": "COMPLETE",
            "healthState": "OK",
            "faultState": "",
            "startReady": False,
        },
        EVENT_TASK_BLOCKED: {
            "controlState": "BLOCKED",
            "healthState": "WARN",
            "faultState": "TASK_BLOCKED",
            "startReady": False,
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
        },
    }

    def apply_event(self, event_type, message="", payload=None):
        payload = payload if isinstance(payload, dict) else {}
        state = dict(self.EVENT_TRANSITIONS.get(event_type, {}))
        if not state:
            state = {
                "controlState": "UNKNOWN",
                "healthState": "WARN",
                "faultState": "UNKNOWN_EVENT",
                "startReady": False,
            }
        if payload.get("faultState") is not None:
            state["faultState"] = str(payload.get("faultState") or "")
        state["message"] = str(message or "")
        state["detail"] = payload
        state["eventType"] = str(event_type or "")
        return state


def runtime_event_type_for_control_state(control_state, fault_state=""):
    control_state = str(control_state or "").upper()
    fault_state = str(fault_state or "")
    if control_state == "INITIALIZING":
        return EVENT_INIT_STARTED
    if control_state == "READY":
        return EVENT_START_CHECK_PASSED
    if control_state == "RUNNING":
        return EVENT_TASK_STARTED
    if control_state == "PAUSED" and fault_state == "RTK_FIX_LOST":
        return EVENT_RTK_LOST
    if control_state == "PAUSED":
        return "TASK_PAUSED"
    if control_state == "STOPPED":
        return EVENT_TASK_STOPPED
    if control_state == "COMPLETE":
        return EVENT_TASK_FINISHED
    if control_state == "BLOCKED" and fault_state == "INIT_FAILED":
        return EVENT_INIT_FAILED
    if control_state == "BLOCKED":
        return EVENT_TASK_BLOCKED
    return "STATE_UPDATED"
