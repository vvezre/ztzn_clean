# coding=utf-8

"""Garage occupancy state and auto-exit decision helpers."""

GARAGE_STATE_KEY = "garageState"
GARAGE_STATE_REASON_KEY = "garageStateReason"
GARAGE_STATE_UPDATED_AT_KEY = "garageStateUpdatedAt"

GARAGE_STATE_UNKNOWN = "unknown"
GARAGE_STATE_OUTSIDE = "outside"
GARAGE_STATE_ENTERING = "entering"
GARAGE_STATE_DOCKED_BY_COMMAND = "docked_by_command"
GARAGE_STATE_DOCKED_MANUAL_CONFIRMED = "docked_manual_confirmed"
GARAGE_STATE_EXITING = "exiting"

EXIT_DECISION_ALLOW = "allow_exit"
EXIT_DECISION_SKIP = "skip_exit"
EXIT_DECISION_CONFIRM = "require_confirm"
EXIT_DECISION_BLOCKED = "blocked"

_KNOWN_GARAGE_STATES = set([
    GARAGE_STATE_UNKNOWN,
    GARAGE_STATE_OUTSIDE,
    GARAGE_STATE_ENTERING,
    GARAGE_STATE_DOCKED_BY_COMMAND,
    GARAGE_STATE_DOCKED_MANUAL_CONFIRMED,
    GARAGE_STATE_EXITING,
])


def normalize_garage_state(value):
    if value is None:
        return GARAGE_STATE_UNKNOWN
    if isinstance(value, bytes):
        try:
            value = value.decode("utf-8")
        except Exception:
            return GARAGE_STATE_UNKNOWN
    value = str(value).strip().lower()
    if value in _KNOWN_GARAGE_STATES:
        return value
    return GARAGE_STATE_UNKNOWN


def _coerce_back_length(back_length):
    try:
        return int(float(back_length))
    except (TypeError, ValueError):
        return 0


def _decision(decision, state, reason):
    return {
        "decision": decision,
        "state": state,
        "reason": reason,
    }


def decide_auto_exit_garage(garage_state, back_length):
    state = normalize_garage_state(garage_state)
    back_length = _coerce_back_length(back_length)

    if back_length <= 0:
        return _decision(EXIT_DECISION_SKIP, state, "garage_back_length_not_configured")

    if state in (GARAGE_STATE_DOCKED_BY_COMMAND, GARAGE_STATE_DOCKED_MANUAL_CONFIRMED):
        return _decision(EXIT_DECISION_ALLOW, state, "garage_state_docked")

    if state == GARAGE_STATE_OUTSIDE:
        return _decision(EXIT_DECISION_SKIP, state, "garage_state_outside")

    if state in (GARAGE_STATE_ENTERING, GARAGE_STATE_EXITING):
        return _decision(EXIT_DECISION_BLOCKED, state, "garage_state_busy")

    return _decision(EXIT_DECISION_CONFIRM, GARAGE_STATE_UNKNOWN, "garage_state_unknown")
