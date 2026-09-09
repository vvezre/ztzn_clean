# coding=utf-8
"""Shared vehicle motion-state derivation for real and simulated devices.

The lower machine reports a control mode and a signed longitudinal speed, but
the mini-program needs a small, stable set of business states.  Keep that
translation in one place so MQTT status and local status cannot disagree.
"""

import time


MANUAL_REPORT_MAX_AGE_SECONDS = 2.0


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


def derive_manual_motion_state(x_speed, lower_status=None, control_state=None,
                               fault_state=None, manual_mode=None,
                               stop_requested=False, z_speed=None, power_on=None,
                               report_at=None, now=None):
    """Resolve manual takeover without treating every unknown mode as parked.

    After an explicit stop the lower machine may keep a non-motion command
    code in its mode byte. The old classifier returned unknown even with a
    fresh zero-speed report, which prevented the first joystick rotation.
    Only this stopped/confirmed case is recovered here. Missing/stale reports,
    faults, charging, non-zero motion and active automatic tasks stay blocked.
    This function classifies telemetry only; it never sends a drive command.
    """
    motion = derive_motion_state(x_speed, lower_status, control_state,
                                 fault_state, manual_mode)
    if motion == 'fault':
        return motion
    control = str(control_state or '').upper()
    # READY also belongs to automatic control: its worker is starting.
    if control in ('READY', 'RUNNING', 'PAUSED', 'STOPPING'):
        return 'auto'
    try:
        age = float(time.time() if now is None else now) - float(report_at)
    except (TypeError, ValueError, OverflowError):
        return 'unknown'
    # This comparison also rejects NaN, infinity and future timestamps.
    if not 0 <= age <= MANUAL_REPORT_MAX_AGE_SECONDS:
        return 'unknown'
    # An active joystick rotation is deliberate manual motion, not stale lower
    # machine state left by the automatic worker.
    if manual_mode == 'in_place_rotate':
        return motion
    try:
        mode = int(lower_status)
        stationary = float(x_speed) == 0 and float(z_speed) == 0
    except (TypeError, ValueError, OverflowError):
        return 'unknown'
    # Once the upper runtime has completed an explicit stop and a fresh lower
    # report confirms both speeds are zero, its STOPPED/parking latch is more
    # authoritative than a residual mode byte.  In practice mode 2 can remain
    # briefly after automatic cleaning, and braking can legitimately report
    # power_on=0.  Neither means the vehicle is still moving.  Charging mode 5
    # remains blocked, as do malformed telemetry and any non-zero speed.
    if (control == 'STOPPED' and stop_requested and 0 <= mode <= 255
            and mode != 5 and stationary):
        return 'stopped'
    return motion
