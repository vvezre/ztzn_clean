# coding=utf-8

import threading
import time


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


def _payload_bool(value):
    if isinstance(value, bool):
        return value
    text = str(value or "").strip().lower()
    if text in ("1", "true", "yes", "on", "ready"):
        return True
    if text in ("0", "false", "no", "off", "none", ""):
        return False
    return bool(value)


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
        with self._lock:
            state = dict(self._state)
            detail = state.get("detail")
            state["detail"] = dict(detail) if isinstance(detail, dict) else {}
            return state

    def _is_transition_allowed(self, current_state, next_state):
        current_state = str(current_state or "UNKNOWN").upper()
        next_state = str(next_state or "UNKNOWN").upper()
        if current_state == next_state:
            return True
        return next_state in self.VALID_TRANSITIONS.get(current_state, ())

    def apply_event(self, event_type, message="", payload=None):
        payload = payload if isinstance(payload, dict) else {}
        with self._lock:
            state = dict(self._state)
            previous_control_state = str(state.get("controlState") or "UNKNOWN").upper()
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
            next_control_state = str(
                transition.get("controlState") or previous_control_state
            ).upper()
            if not self._is_transition_allowed(previous_control_state, next_control_state):
                rejected = self.get_state()
                rejected["transitionAccepted"] = False
                rejected["rejectedEvent"] = str(event_type or "")
                rejected["requestedControlState"] = next_control_state
                rejected["previousControlState"] = previous_control_state
                rejected["message"] = str(message or "")
                return rejected

            state.update(transition)
            if payload.get("faultState") is not None:
                state["faultState"] = str(payload.get("faultState") or "")
            if payload.get("healthState") is not None:
                state["healthState"] = str(payload.get("healthState") or "")
            if payload.get("startReady") is not None:
                state["startReady"] = _payload_bool(payload.get("startReady"))
            if payload.get("action") is not None and "action" not in transition:
                state["action"] = str(payload.get("action") or "")
            state["message"] = str(message or "")
            state["detail"] = payload
            state["eventType"] = str(event_type or "")
            state["transitionAccepted"] = True
            state["previousControlState"] = previous_control_state
            self._state = state
            return self.get_state()


def legacy_fields_for_state(state):
    state = state if isinstance(state, dict) else {}
    control_state = str(state.get("controlState") or "").upper()
    action = str(state.get("action") or "idle")
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
