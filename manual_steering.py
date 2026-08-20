# coding=utf-8
"""Safe state machine for cloud manual left/right button gestures.

This module contains no Flask, MQTT, serial-port or robot-specific imports.
The real FSM supplies hardware callbacks, while the modeling simulator uses
no-op callbacks.  Both therefore expose exactly the same frontend behavior.
"""

import threading
import time


VALID_ACTIONS = frozenset(('tap', 'hold_start', 'keepalive', 'hold_stop', 'reset'))
VALID_DIRECTIONS = frozenset(('left', 'right'))


class ManualSteeringError(Exception):
    def __init__(self, code, message):
        Exception.__init__(self, message)
        self.code = code
        self.message = message


class ManualSteeringController(object):
    """Manage tap/hold steering without trusting frontend motion state."""

    def __init__(self, motion_provider, apply_trim, start_rotation,
                 stop_rotation, state_callback=None, now=None,
                 step=50, max_value=500, keepalive_timeout=1.5):
        self.motion_provider = motion_provider
        self.apply_trim = apply_trim
        self.start_rotation = start_rotation
        self.stop_rotation = stop_rotation
        self.state_callback = state_callback
        self.now = now or time.time
        self.step = max(1, int(step))
        self.max_value = max(self.step, int(max_value))
        self.max_level = max(1, int(self.max_value / self.step))
        self.keepalive_timeout = max(0.2, float(keepalive_timeout))
        self._lock = threading.RLock()
        self._correction_level = 0
        self._correction_value = 0
        self._correction_motion = None
        self._mode = 'none'
        self._direction = None
        self._control_id = None
        self._last_sequence = -1
        self._last_keepalive = 0.0
        self._closed = False
        self._watchdog = threading.Thread(target=self._watchdog_loop)
        self._watchdog.daemon = True
        self._watchdog.start()
        self._notify_state()

    @staticmethod
    def _text(value):
        if value is None:
            return ''
        try:
            return unicode(value).strip()
        except NameError:
            return str(value).strip()

    def _motion(self):
        try:
            value = self.motion_provider()
        except Exception:
            return 'unknown'
        return self._text(value).lower() or 'unknown'

    def _snapshot_locked(self, motion_state=None):
        motion = motion_state or self._motion()
        if self._mode == 'in_place_rotate':
            motion = 'turning'
        return {
            'motionState': motion,
            'manualSteeringAllowed': motion in ('forward', 'reverse', 'stopped'),
            'manualSteeringMode': self._mode,
            'manualSteeringDirection': self._direction,
            'manualCorrectionValue': self._correction_value,
            'manualCorrectionLevel': abs(self._correction_level),
            'manualSteeringControlId': self._control_id,
        }

    def snapshot(self):
        with self._lock:
            return self._snapshot_locked()

    def _notify_state(self):
        if not callable(self.state_callback):
            return
        try:
            self.state_callback(self.snapshot())
        except Exception:
            pass

    def _response(self, message, motion_state=None):
        with self._lock:
            data = self._snapshot_locked(motion_state)
        return {'success': True, 'message': message, 'data': data}

    def _validate_common(self, action, direction, control_id, sequence):
        if action not in VALID_ACTIONS:
            raise ManualSteeringError('INVALID_ACTION', 'unsupported manual steering action')
        if action != 'reset' and direction not in VALID_DIRECTIONS:
            raise ManualSteeringError('INVALID_DIRECTION', 'direction must be left or right')
        if not control_id:
            raise ManualSteeringError('INVALID_CONTROL_ID', 'controlId is required')
        if len(control_id) > 80:
            raise ManualSteeringError('INVALID_CONTROL_ID', 'controlId is too long')
        try:
            sequence_value = int(sequence)
        except (TypeError, ValueError):
            raise ManualSteeringError('INVALID_SEQUENCE', 'sequence must be a non-negative integer')
        if sequence_value < 0:
            raise ManualSteeringError('INVALID_SEQUENCE', 'sequence must be a non-negative integer')
        return sequence_value

    def _raw_value(self, level, motion_state):
        semantic_value = int(level) * self.step
        if motion_state == 'reverse':
            semantic_value = -semantic_value
        return semantic_value

    def _step_trim_locked(self, direction, motion_state):
        if self._correction_motion != motion_state:
            self._correction_level = 0
            self._correction_value = 0
            self._correction_motion = motion_state
        delta = -1 if direction == 'left' else 1
        next_level = max(-self.max_level, min(self.max_level, self._correction_level + delta))
        next_value = self._raw_value(next_level, motion_state)
        self.apply_trim(next_value, motion_state)
        self._correction_level = next_level
        self._correction_value = next_value

    def _clear_session_locked(self):
        self._mode = 'none'
        self._direction = None
        self._control_id = None
        self._last_sequence = -1
        self._last_keepalive = 0.0

    def handle(self, params):
        params = params or {}
        action = self._text(params.get('action')).lower()
        direction = self._text(params.get('direction')).lower()
        control_id = self._text(params.get('controlId'))
        sequence = self._validate_common(action, direction, control_id, params.get('sequence'))

        try:
            if action == 'tap':
                return self._tap(direction)
            if action == 'hold_start':
                return self._hold_start(direction, control_id, sequence)
            if action == 'keepalive':
                return self._keepalive(direction, control_id, sequence)
            if action == 'hold_stop':
                return self._hold_stop(control_id)
            return self.reset(send_hardware=True, message='manual steering reset')
        except ManualSteeringError:
            raise
        except Exception as error:
            raise ManualSteeringError('ACTUATOR_ERROR', 'manual steering actuator failed: {}'.format(error))

    def _tap(self, direction):
        motion = self._motion()
        if motion == 'stopped':
            raise ManualSteeringError(
                'TAP_REQUIRES_MOVEMENT',
                'tap is ignored while stopped; hold the button for in-place rotation',
            )
        if motion not in ('forward', 'reverse'):
            raise ManualSteeringError('MANUAL_STEERING_BLOCKED', 'manual steering is not allowed while {}'.format(motion))
        with self._lock:
            if self._mode != 'none':
                raise ManualSteeringError('CONTROL_SESSION_ACTIVE', 'another manual steering hold is active')
            self._step_trim_locked(direction, motion)
        self._notify_state()
        return self._response('{} trim step applied'.format(motion), motion)

    def _hold_start(self, direction, control_id, sequence):
        motion = self._motion()
        if motion not in ('forward', 'reverse', 'stopped'):
            raise ManualSteeringError('MANUAL_STEERING_BLOCKED', 'manual steering is not allowed while {}'.format(motion))
        with self._lock:
            if self._mode != 'none':
                if self._control_id == control_id and sequence <= self._last_sequence:
                    return self._response('duplicate hold_start ignored')
                raise ManualSteeringError('CONTROL_SESSION_ACTIVE', 'another manual steering hold is active')
            if motion == 'stopped':
                self.start_rotation(direction)
                self._mode = 'in_place_rotate'
            else:
                self._step_trim_locked(direction, motion)
                self._mode = '{}_trim'.format(motion)
            self._direction = direction
            self._control_id = control_id
            self._last_sequence = sequence
            self._last_keepalive = self.now()
        self._notify_state()
        if motion == 'stopped':
            return self._response('in-place {} rotation started'.format(direction))
        return self._response('{} continuous trim started'.format(motion), motion)

    def _keepalive(self, direction, control_id, sequence):
        with self._lock:
            if self._mode == 'none' or self._control_id != control_id:
                raise ManualSteeringError('CONTROL_SESSION_NOT_FOUND', 'manual steering hold session was not found')
            if direction != self._direction:
                raise ManualSteeringError('CONTROL_DIRECTION_MISMATCH', 'direction does not match the active hold')
            if sequence <= self._last_sequence:
                return self._response('duplicate keepalive ignored')
            base_motion = self._motion()
            expected_motion = None
            if self._mode == 'forward_trim':
                expected_motion = 'forward'
            elif self._mode == 'reverse_trim':
                expected_motion = 'reverse'
            elif self._mode == 'in_place_rotate':
                expected_motion = 'stopped'
            if base_motion != expected_motion:
                rotate = self._mode == 'in_place_rotate'
                self._clear_session_locked()
                if rotate:
                    self.stop_rotation()
                raise ManualSteeringError('MOTION_STATE_CHANGED', 'vehicle motion state changed during hold')
            if self._mode in ('forward_trim', 'reverse_trim'):
                self._step_trim_locked(direction, base_motion)
            self._last_sequence = sequence
            self._last_keepalive = self.now()
        self._notify_state()
        return self._response('manual steering keepalive accepted')

    def _hold_stop(self, control_id):
        with self._lock:
            if self._mode == 'none':
                return self._response('manual steering hold already stopped')
            if self._control_id != control_id:
                raise ManualSteeringError('CONTROL_SESSION_NOT_FOUND', 'manual steering hold session was not found')
            rotate = self._mode == 'in_place_rotate'
            self._clear_session_locked()
        if rotate:
            self.stop_rotation()
        self._notify_state()
        return self._response('manual steering hold stopped')

    def reset(self, send_hardware=True, message='manual steering reset'):
        with self._lock:
            rotate = self._mode == 'in_place_rotate'
            motion = self._motion()
            had_trim = self._correction_value != 0
            self._clear_session_locked()
            self._correction_level = 0
            self._correction_value = 0
            self._correction_motion = None
        if send_hardware:
            if rotate:
                self.stop_rotation()
            elif had_trim and motion in ('forward', 'reverse'):
                self.apply_trim(0, motion)
        self._notify_state()
        return self._response(message)

    def _watchdog_loop(self):
        while not self._closed:
            time.sleep(0.1)
            timed_out = False
            rotate = False
            with self._lock:
                if self._mode != 'none' and self._last_keepalive > 0:
                    timed_out = self.now() - self._last_keepalive > self.keepalive_timeout
                    if timed_out:
                        rotate = self._mode == 'in_place_rotate'
                        self._clear_session_locked()
            if timed_out:
                if rotate:
                    try:
                        self.stop_rotation()
                    except Exception:
                        pass
                self._notify_state()

    def close(self):
        self._closed = True
        self.reset(send_hardware=False, message='manual steering controller closed')
