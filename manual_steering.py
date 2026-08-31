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
                 stop_rotation, travel_speed_provider=None,
                 state_callback=None, now=None,
                 moving_value=700, tap_duration=0.3,
                 keepalive_timeout=1.5):
        self.motion_provider = motion_provider
        self.apply_trim = apply_trim
        self.start_rotation = start_rotation
        self.stop_rotation = stop_rotation
        self.travel_speed_provider = travel_speed_provider
        self.state_callback = state_callback
        self.now = now or time.time
        # The moving left/right buttons reuse the existing joystick's fixed
        # 45-degree command.  They do not accumulate a correction value.
        self.moving_value = max(1, abs(int(moving_value)))
        self.tap_duration = max(0.05, float(tap_duration))
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
        # Moving gestures must retain the original commanded travel speed.
        # Reading live wheel feedback on every frame makes the speed ratchet
        # down because a diagonal command naturally reports a lower X speed.
        self._travel_speed = None
        # A frontend/network retry may deliver keepalive after hold_stop.  Keep
        # a small bounded tombstone set so such frames can never revive a
        # released gesture.  A new explicit hold_start removes its tombstone.
        self._closed_control_ids = []
        # Incrementing this invalidates a delayed tap restore.  A stale timer
        # can therefore never overwrite a later drive/stop/automatic command.
        self._session_generation = 0
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

    def _moving_value(self, direction):
        return -self.moving_value if direction == 'left' else self.moving_value

    def _target_speed(self, motion_state):
        if not callable(self.travel_speed_provider):
            raise RuntimeError('manual steering travel speed provider is unavailable')
        speed = abs(int(self.travel_speed_provider(motion_state)))
        if speed <= 0:
            raise RuntimeError('manual steering travel speed must be positive')
        return speed

    def _apply_moving_direction_locked(self, direction, motion_state, travel_speed=None):
        """Apply one fixed joystick direction while preserving travel speed."""
        if travel_speed is None:
            travel_speed = self._target_speed(motion_state)
        value = self._moving_value(direction)
        self.apply_trim(value, motion_state, travel_speed)
        self._correction_level = -1 if direction == 'left' else 1
        self._correction_value = value
        self._correction_motion = motion_state
        self._travel_speed = travel_speed

    def _clear_correction_locked(self):
        self._correction_level = 0
        self._correction_value = 0
        self._correction_motion = None

    def _clear_session_locked(self):
        self._session_generation += 1
        self._mode = 'none'
        self._direction = None
        self._control_id = None
        self._last_sequence = -1
        self._last_keepalive = 0.0
        self._travel_speed = None

    def _remember_closed_control_locked(self, control_id):
        if not control_id:
            return
        try:
            self._closed_control_ids.remove(control_id)
        except ValueError:
            pass
        self._closed_control_ids.append(control_id)
        if len(self._closed_control_ids) > 32:
            del self._closed_control_ids[:-32]

    def _forget_closed_control_locked(self, control_id):
        try:
            self._closed_control_ids.remove(control_id)
        except ValueError:
            pass

    def _restore_straight(self, motion_state, travel_speed):
        if motion_state in ('forward', 'reverse') and travel_speed:
            self.apply_trim(0, motion_state, travel_speed)

    def handle(self, params):
        params = params or {}
        action = self._text(params.get('action')).lower()
        direction = self._text(params.get('direction')).lower()
        control_id = self._text(params.get('controlId'))
        sequence = self._validate_common(action, direction, control_id, params.get('sequence'))

        try:
            if action == 'tap':
                return self._tap(direction, control_id, sequence)
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

    def _schedule_tap_restore(self, generation, motion_state, travel_speed):
        def restore_after_delay():
            time.sleep(self.tap_duration)
            with self._lock:
                expected_mode = '{}_tap'.format(motion_state)
                if self._session_generation != generation or self._mode != expected_mode:
                    return
                self._clear_session_locked()
                self._clear_correction_locked()
            try:
                self._restore_straight(motion_state, travel_speed)
            except Exception:
                # A later stop/fault can make the lower machine unavailable.
                # The session has already been invalidated, so never retry a
                # stale steering command from this background thread.
                pass
            self._notify_state()

        worker = threading.Thread(target=restore_after_delay)
        worker.daemon = True
        worker.start()

    def _tap(self, direction, control_id, sequence):
        motion = self._motion()
        if motion == 'stopped':
            raise ManualSteeringError(
                'TAP_REQUIRES_MOVEMENT',
                'tap is ignored while stopped; hold the button for in-place rotation',
            )
        if motion not in ('forward', 'reverse'):
            raise ManualSteeringError('MANUAL_STEERING_BLOCKED', 'manual steering is not allowed while {}'.format(motion))
        with self._lock:
            # A second tap may replace the previous 300 ms pulse.  A long-hold
            # session still owns the controls and cannot be interrupted by an
            # unrelated tap.
            if self._mode in ('forward_tap', 'reverse_tap'):
                if (self._control_id == control_id and
                        self._direction == direction and
                        sequence == self._last_sequence):
                    return self._response('duplicate tap ignored', motion)
                self._clear_session_locked()
                self._clear_correction_locked()
            elif self._mode in ('forward_trim', 'reverse_trim'):
                # Some frontend gesture implementations emit a tap event while
                # the long-press session is already active.  The hold owns the
                # controls; accepting the tap's different controlId would make
                # the next valid keepalive look stale and break the session.
                return self._response('tap ignored while hold is active', self._correction_motion)
            elif self._mode != 'none':
                raise ManualSteeringError('CONTROL_SESSION_ACTIVE', 'another manual steering hold is active')
            self._apply_moving_direction_locked(direction, motion)
            self._mode = '{}_tap'.format(motion)
            self._direction = direction
            self._control_id = control_id
            self._last_sequence = sequence
            generation = self._session_generation
            travel_speed = self._travel_speed
        self._schedule_tap_restore(generation, motion, travel_speed)
        self._notify_state()
        return self._response('{} joystick tap applied'.format(motion), motion)

    def _hold_start(self, direction, control_id, sequence):
        # Check the active session before reading the lower-machine motion.
        # MQTT QoS 1 may deliver the same hold_start more than once.  During
        # joystick rotation the lower machine can report forward/turning even
        # though this controller still owns the same in-place rotation.  The
        # duplicate must therefore be treated as idempotent instead of being
        # rejected or reinterpreted as a new forward-trim gesture.
        with self._lock:
            self._forget_closed_control_locked(control_id)
            if self._mode in ('forward_tap', 'reverse_tap'):
                # A deliberate hold supersedes an unfinished tap without an
                # intermediate straight frame that would cause a visible jerk.
                self._clear_session_locked()
                self._clear_correction_locked()
            if self._mode != 'none':
                if (self._control_id == control_id and
                        self._direction == direction and
                        sequence <= self._last_sequence):
                    self._last_keepalive = self.now()
                    return self._response('duplicate hold_start ignored')
                # A new left/right press supersedes the prior manual hold.  It
                # keeps the same captured travel speed, so changing direction
                # never introduces a stop frame or speed step.
                if self._mode in ('forward_trim', 'reverse_trim'):
                    motion = self._correction_motion
                    travel_speed = self._travel_speed
                    self._apply_moving_direction_locked(direction, motion, travel_speed)
                    self._direction = direction
                    self._control_id = control_id
                    self._last_sequence = sequence
                    self._last_keepalive = self.now()
                    self._notify_state()
                    return self._response('{} fixed joystick steering switched'.format(motion), motion)
                if self._mode == 'in_place_rotate':
                    self.stop_rotation()
                    self.start_rotation(direction)
                    self._direction = direction
                    self._control_id = control_id
                    self._last_sequence = sequence
                    self._last_keepalive = self.now()
                    self._notify_state()
                    return self._response('in-place rotation direction switched')
                raise ManualSteeringError('CONTROL_SESSION_ACTIVE', 'another manual steering hold is active')

            motion = self._motion()
            if motion not in ('forward', 'reverse', 'stopped'):
                raise ManualSteeringError(
                    'MANUAL_STEERING_BLOCKED',
                    'manual steering is not allowed while {}'.format(motion),
                )
            if motion == 'stopped':
                self.start_rotation(direction)
                self._mode = 'in_place_rotate'
            else:
                self._apply_moving_direction_locked(direction, motion)
                self._mode = '{}_trim'.format(motion)
            self._direction = direction
            self._control_id = control_id
            self._last_sequence = sequence
            self._last_keepalive = self.now()
        self._notify_state()
        if motion == 'stopped':
            return self._response('in-place {} rotation started'.format(direction))
        return self._response('{} fixed joystick steering started'.format(motion), motion)

    def _keepalive(self, direction, control_id, sequence):
        with self._lock:
            if self._mode == 'none':
                if control_id in self._closed_control_ids:
                    return self._response('stale keepalive ignored')
                # Be tolerant of a lost/delayed hold_start: the first orphan
                # keepalive can establish the hold, but never after a known
                # hold_stop for the same controlId.
                motion = self._motion()
                if motion not in ('forward', 'reverse', 'stopped'):
                    raise ManualSteeringError(
                        'MANUAL_STEERING_BLOCKED',
                        'manual steering is not allowed while {}'.format(motion),
                    )
                if motion == 'stopped':
                    self.start_rotation(direction)
                    self._mode = 'in_place_rotate'
                else:
                    self._apply_moving_direction_locked(direction, motion)
                    self._mode = '{}_trim'.format(motion)
                self._direction = direction
                self._control_id = control_id
                self._last_sequence = sequence
                self._last_keepalive = self.now()
                self._notify_state()
                return self._response('orphan keepalive promoted to hold', motion)

            if self._mode in ('forward_tap', 'reverse_tap'):
                if direction != self._direction:
                    raise ManualSteeringError('CONTROL_DIRECTION_MISMATCH', 'direction does not match the active tap')
                motion = self._correction_motion
                travel_speed = self._travel_speed
                self._apply_moving_direction_locked(direction, motion, travel_speed)
                self._mode = '{}_trim'.format(motion)
                self._control_id = control_id
                self._last_sequence = sequence
                self._last_keepalive = self.now()
                self._notify_state()
                return self._response('tap promoted to active hold', motion)

            if self._control_id != control_id:
                raise ManualSteeringError('CONTROL_SESSION_NOT_FOUND', 'manual steering hold session was not found')
            if direction != self._direction:
                raise ManualSteeringError('CONTROL_DIRECTION_MISMATCH', 'direction does not match the active hold')
            base_motion = self._motion()
            motion_matches_session = False
            if self._mode == 'forward_trim':
                motion_matches_session = base_motion == 'forward'
            elif self._mode == 'reverse_trim':
                motion_matches_session = base_motion == 'reverse'
            elif self._mode == 'in_place_rotate':
                # The pure-left/pure-right joystick command uses lower mode 4.
                # Real hardware may expose a signed X speed while rotating, so
                # requiring "stopped" here incorrectly brakes on the first
                # valid keepalive.  Auto/fault/unknown still terminate the hold
                # immediately; manual lower-machine states are accepted until
                # hold_stop or the watchdog timeout performs the safe stop.
                motion_matches_session = base_motion in ('forward', 'reverse', 'stopped', 'turning')
            if not motion_matches_session:
                rotate = self._mode == 'in_place_rotate'
                self._clear_session_locked()
                if rotate:
                    self.stop_rotation()
                else:
                    self._clear_correction_locked()
                raise ManualSteeringError('MOTION_STATE_CHANGED', 'vehicle motion state changed during hold')

            current_time = self.now()
            # Re-send the exact same fixed 45-degree frame.  This mirrors the
            # local joystick page, which streams its current direction while
            # held.  The saved target speed and steering value never increase.
            if self._mode in ('forward_trim', 'reverse_trim'):
                self._apply_moving_direction_locked(
                    self._direction,
                    self._correction_motion,
                    self._travel_speed,
                )
            self._last_sequence = max(self._last_sequence, sequence)
            self._last_keepalive = current_time
        self._notify_state()
        return self._response('manual steering keepalive accepted')

    def _hold_stop(self, control_id):
        with self._lock:
            if self._mode == 'none':
                self._remember_closed_control_locked(control_id)
                return self._response('manual steering hold already stopped')
            if self._control_id != control_id:
                self._remember_closed_control_locked(control_id)
                return self._response('stale hold stop ignored')
            rotate = self._mode == 'in_place_rotate'
            trim_motion = None
            travel_speed = self._travel_speed
            if self._mode in ('forward_trim', 'forward_tap'):
                trim_motion = 'forward'
            elif self._mode in ('reverse_trim', 'reverse_tap'):
                trim_motion = 'reverse'
            self._remember_closed_control_locked(control_id)
            self._clear_session_locked()
            if trim_motion is not None:
                # A moving long-press is temporary steering assistance.  Do
                # not carry its last correction into the next left/right
                # gesture: releasing either button restores straight travel.
                self._clear_correction_locked()
        if rotate:
            self.stop_rotation()
        elif trim_motion is not None:
            # This is intentionally a single reset frame.  The lower-machine
            # command also contains the current longitudinal speed, so it must
            # not be transmitted continuously after the button is released.
            self._restore_straight(trim_motion, travel_speed)
        self._notify_state()
        return self._response('manual steering hold stopped')

    def reset(self, send_hardware=True, message='manual steering reset'):
        with self._lock:
            rotate = self._mode == 'in_place_rotate'
            moving_motion = self._correction_motion
            travel_speed = self._travel_speed
            self._remember_closed_control_locked(self._control_id)
            had_moving_direction = self._correction_value != 0 and moving_motion in ('forward', 'reverse')
            self._clear_session_locked()
            self._clear_correction_locked()
        if send_hardware:
            if rotate:
                self.stop_rotation()
            elif had_moving_direction:
                self._restore_straight(moving_motion, travel_speed)
        self._notify_state()
        return self._response(message)

    def _watchdog_loop(self):
        while not self._closed:
            time.sleep(0.1)
            timed_out = False
            rotate = False
            moving_motion = None
            travel_speed = None
            with self._lock:
                if self._mode in ('forward_trim', 'reverse_trim', 'in_place_rotate') and self._last_keepalive > 0:
                    timed_out = self.now() - self._last_keepalive > self.keepalive_timeout
                    if timed_out:
                        rotate = self._mode == 'in_place_rotate'
                        moving_motion = self._correction_motion
                        travel_speed = self._travel_speed
                        self._remember_closed_control_locked(self._control_id)
                        self._clear_session_locked()
                        self._clear_correction_locked()
            if timed_out:
                if rotate:
                    try:
                        self.stop_rotation()
                    except Exception:
                        pass
                elif moving_motion in ('forward', 'reverse'):
                    try:
                        # A lost hold_stop must not leave the robot following a
                        # permanent diagonal command.  Resume its prior straight
                        # manual movement instead of braking it.
                        self._restore_straight(moving_motion, travel_speed)
                    except Exception:
                        pass
                self._notify_state()

    def close(self):
        self._closed = True
        self.reset(send_hardware=False, message='manual steering controller closed')
