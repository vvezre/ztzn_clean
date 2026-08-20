# coding=utf-8
"""Shared vehicle motion-state derivation for real and simulated devices.

The lower machine reports a control mode and a signed longitudinal speed, but
the mini-program needs a small, stable set of business states.  Keep that
translation in one place so MQTT status and local status cannot disagree.
"""


def derive_motion_state(x_speed, lower_status=None, control_state=None,
                        fault_state=None, manual_mode=None):
    """Return forward/reverse/stopped/turning/auto/fault/unknown.

    ``manual_mode == in_place_rotate`` deliberately wins over the lower
    machine mode.  Joystick-style in-place rotation uses lower mode 4 rather
    than the lower machine's fixed-angle rotation mode 3.
    """
    if fault_state:
        return 'fault'

    normalized_control = str(control_state or '').upper()
    if normalized_control == 'FAULT':
        return 'fault'
    if normalized_control in ('RUNNING', 'PAUSED', 'STOPPING'):
        return 'auto'
    if normalized_control in ('BLOCKED', 'DISABLED', 'UNKNOWN', 'INITIALIZING'):
        return 'unknown'

    if manual_mode == 'in_place_rotate':
        return 'turning'

    try:
        status_value = int(lower_status) if lower_status is not None else None
    except (TypeError, ValueError):
        status_value = None
    if status_value == 3:
        return 'turning'
    if status_value == 2:
        return 'auto'

    try:
        speed_value = int(x_speed) if x_speed is not None else None
    except (TypeError, ValueError):
        speed_value = None
    if speed_value is None:
        return 'unknown'
    if speed_value > 0:
        return 'forward'
    if speed_value < 0:
        return 'reverse'
    if status_value in (0, 1, 4):
        return 'stopped'
    return 'unknown'


def manual_steering_allowed(motion_state):
    """Whether a new manual steering gesture may start in this state."""
    return motion_state in ('forward', 'reverse', 'stopped')
