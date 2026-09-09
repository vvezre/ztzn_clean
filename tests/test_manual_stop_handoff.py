# coding=utf-8
"""Offline stop -> manual regression: no imports of main or serial hardware."""
import ast
import copy
import io
import os
import time
import unittest

from manual_steering import ManualSteeringController, ManualSteeringError
from motion_state import derive_motion_state, derive_manual_motion_state


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load_main_functions(names, namespace):
    with io.open(os.path.join(ROOT, 'main.py'), encoding='utf-8') as handle:
        tree = ast.parse(handle.read().encode('utf-8'))
    module = ast.parse('')
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name in names:
            node = copy.deepcopy(node)
            node.decorator_list = []
            module.body.append(node)
    # eval of an exec code object works on both Python 2.7 and Python 3.
    eval(compile(module, 'isolated-main-functions', 'exec'), namespace)


def stopped_report(**changes):
    values = dict(x_speed=0, lower_status=250, control_state='STOPPED',
                  fault_state='', stop_requested=True, z_speed=0,
                  power_on=1, report_at=100, now=100.5)
    values.update(changes)
    return values


class ManualStopClassificationTest(unittest.TestCase):
    def test_reproduces_old_gap_and_recovers_only_confirmed_stop(self):
        # The failure log has STOPPED, speed=0, fresh hardware and unknown
        # motion. Its exact mode byte was not logged: use representative
        # unclassified bytes, not an invented claim about that hardware log.
        for code in (6, 7, 250, 255):
            self.assertEqual('unknown', derive_motion_state(0, code, 'STOPPED'))
            self.assertEqual('stopped', derive_manual_motion_state(**stopped_report(lower_status=code)))

        # Actual stop reports may retain automatic mode 2 while braking has
        # already disabled the motor.  This was the path missed by the first
        # implementation because derive_motion_state returned auto too early.
        self.assertEqual('auto', derive_motion_state(0, 2, 'STOPPED'))
        self.assertEqual('stopped', derive_manual_motion_state(
            **stopped_report(lower_status=2, power_on=0)))

    def test_no_blanket_unknown_bypass_without_explicit_parking(self):
        self.assertEqual('unknown', derive_manual_motion_state(**stopped_report(stop_requested=False)))
        for state in ('COMPLETE', 'IDLE', 'UNKNOWN', 'DISABLED', 'BLOCKED', 'INITIALIZING'):
            self.assertEqual('unknown', derive_manual_motion_state(**stopped_report(control_state=state)))

    def test_auto_start_run_pause_and_stopping_remain_blocked(self):
        for state in ('READY', 'RUNNING', 'PAUSED', 'STOPPING'):
            self.assertEqual('auto', derive_manual_motion_state(**stopped_report(control_state=state)))

    def test_fault_and_charging_are_not_reclassified_but_residual_motion_modes_are(self):
        self.assertEqual('fault', derive_manual_motion_state(**stopped_report(fault_state='SERIAL_FAULT')))
        self.assertEqual('fault', derive_manual_motion_state(**stopped_report(control_state='FAULT')))
        for code, expected in ((2, 'stopped'), (3, 'stopped'), (5, 'unknown')):
            self.assertEqual(expected, derive_manual_motion_state(**stopped_report(lower_status=code)))

    def test_no_report_stale_future_and_nonfinite_timestamps_block(self):
        for timestamp in (None, 0, 97.9, 101, 'invalid', float('nan'), float('inf')):
            for code in (0, 1, 4, 250):
                self.assertEqual('unknown', derive_manual_motion_state(**stopped_report(report_at=timestamp, lower_status=code)))
        self.assertEqual('stopped', derive_manual_motion_state(**stopped_report(report_at=98.5)))

    def test_missing_or_invalid_speed_and_mode_cannot_recover_unknown(self):
        for key, values in (('x_speed', (None, 'bad', float('nan'))),
                            ('z_speed', (None, 1, -1, float('nan'))),
                            ('lower_status', (None, 'bad', -1, 256))):
            for value in values:
                self.assertEqual('unknown', derive_manual_motion_state(**stopped_report(**{key: value})))

    def test_power_enable_is_not_motion_evidence_after_confirmed_stop(self):
        for value in (None, 0, 1, 2, 'bad'):
            self.assertEqual('stopped', derive_manual_motion_state(
                **stopped_report(lower_status=2, power_on=value)))

    def test_moving_feedback_is_not_new_stationary_confirmation(self):
        self.assertEqual('forward', derive_manual_motion_state(**stopped_report(x_speed=100)))
        self.assertEqual('reverse', derive_manual_motion_state(**stopped_report(x_speed=-100)))

    def test_existing_motion_states_and_rotation_display_preserved(self):
        for code in (0, 1, 4):
            self.assertEqual('stopped', derive_manual_motion_state(**stopped_report(lower_status=code)))
        self.assertEqual('turning', derive_manual_motion_state(**stopped_report(lower_status=4, manual_mode='in_place_rotate')))


class FakeRedis(object):
    def __init__(self):
        self.values = {}

    def set(self, key, value):
        self.values[key] = value


class ManualStopMainIntegrationTest(unittest.TestCase):
    def setUp(self):
        self.events = []
        self.state = dict(control='RUNNING', action='auto_drive', report=100,
                          power=1, fault='')
        self.ns = dict(global_get_XSpeed=0, global_get_ZSpeed=0, global_get_status=250,
                       redis_cli=FakeRedis(),
                       _get_hardware_report_at=lambda: self.state['report'],
                       _get_hardware_report_age_sec=lambda: 100.5 - self.state['report'],
                       _get_power_on_state=lambda: self.state['power'],
                       _derive_control_state=lambda: self.state['control'],
                       _derive_fault_state=lambda: self.state['fault'],
                       _runtime_action=lambda: self.state['action'],
                       _coerce_int=lambda value, default: default if value is None else int(value),
                       live_value_from_report=lambda value, report: value if report is not None else None,
                       derive_manual_motion_state=self.classify)
        load_main_functions(('_manual_steering_base_motion', '_reset_manual_steering',
                             '_request_runtime_stop', 'parking'), self.ns)
        self.controller = ManualSteeringController(
            self.ns['_manual_steering_base_motion'],
            lambda value, mode, speed: self.events.append(('trim', value, mode, speed)),
            lambda direction: self.events.append(('rotate', direction)),
            lambda: self.events.append(('brake',)),
            travel_speed_provider=lambda mode: 350 if mode == 'forward' else 100,
            tap_duration=0.05)
        self.ns.update(manual_steering_controller=self.controller,
                       _get_manual_steering_controller=lambda: self.controller,
                       _disable_loop_auto_clean=lambda reason: None,
                       _set_auto_resume_allowed=lambda *args: None,
                       _publish_global_go=lambda value: None,
                       sendBraking=lambda: self.events.append(('brake',)),
                       _clear_runtime_task_state=self.clear_runtime,
                       make_response=lambda value: value)

    def classify(self, *args, **kwargs):
        kwargs['now'] = 100.5
        return derive_manual_motion_state(*args, **kwargs)

    def clear_runtime(self, *args, **kwargs):
        self.state.update(control='STOPPED', action='parking')

    def tearDown(self):
        self.controller.close()

    def gesture(self, action, direction='right', control_id='button', sequence=1):
        return self.controller.handle(dict(action=action, direction=direction,
                                           controlId=control_id, sequence=sequence))

    def test_actual_parking_then_long_press_without_forward_command(self):
        self.ns['parking']()
        self.assertEqual('stopped', self.controller.snapshot()['motionState'])
        self.assertTrue(self.gesture('hold_start')['success'])
        for _ in range(5):
            self.assertTrue(self.gesture('keepalive', sequence=2)['success'])
        self.gesture('hold_stop', sequence=6)
        self.assertEqual([('brake',), ('rotate', 'right'), ('brake',)], self.events)
        self.assertEqual('', self.ns['active_runtime_task_token'])
        self.assertEqual(1, self.ns['global_auto_clean_stop'])

    def test_twenty_rapid_presses_each_direction_never_issue_translation(self):
        self.ns['parking']()
        for direction in ('right', 'left'):
            for index in range(20):
                self.gesture('hold_start', direction)
                self.ns['global_get_status'] = 4
                self.ns['global_get_XSpeed'] = 40 if index % 2 else -40
                self.gesture('keepalive', direction, sequence=2)
                self.gesture('hold_stop', direction, sequence=6)
        self.assertEqual(40, sum(event[0] == 'rotate' for event in self.events))
        self.assertFalse(any(event[0] == 'trim' for event in self.events))

    def test_stop_cancels_old_moving_tap_restore(self):
        self.state.update(control='STOPPED', action='parking')
        self.ns['global_get_status'] = 1
        self.ns['global_get_XSpeed'] = 50
        self.controller.set_commanded_motion('forward')
        self.gesture('tap')
        self.ns['_request_runtime_stop']('stop_auto')
        self.ns['global_get_status'] = 250
        self.ns['global_get_XSpeed'] = 0
        self.gesture('hold_start', control_id='new')
        time.sleep(0.09)
        self.gesture('hold_stop', control_id='new', sequence=6)
        self.assertEqual([('trim', 700, 'forward', 350), ('brake',),
                          ('rotate', 'right'), ('brake',)], self.events)

    def test_unknown_stale_running_and_fault_stop_latch_do_not_bypass_safety(self):
        self.ns['parking']()
        for change in ({'report': 95}, {'control': 'READY'}, {'control': 'PAUSED'},
                       {'control': 'RUNNING'}, {'fault': 'FAULT'}):
            original = dict(self.state)
            self.state.update(change)
            with self.assertRaises(ManualSteeringError) as raised:
                self.gesture('hold_start')
            self.assertEqual('MANUAL_STEERING_BLOCKED', raised.exception.code)
            self.state = original
        self.assertEqual([('brake',)], self.events)

    def test_actual_residual_mode_two_and_power_off_allow_first_rotation(self):
        self.ns['parking']()
        self.ns['global_get_status'] = 2
        self.state['power'] = 0
        self.assertTrue(self.gesture('hold_start')['success'])
        self.assertEqual(('rotate', 'right'), self.events[-1])
        self.gesture('hold_stop', sequence=6)

    def test_stale_report_interrupts_active_rotation_keepalive(self):
        self.ns['parking']()
        self.gesture('hold_start')
        self.state['report'] = 95
        with self.assertRaises(ManualSteeringError):
            self.gesture('keepalive', sequence=2)
        self.assertEqual('none', self.controller.snapshot()['manualSteeringMode'])
        self.assertEqual([('brake',), ('rotate', 'right'), ('brake',)], self.events)

    def test_explicit_drive_and_reverse_retain_original_steering_behavior(self):
        self.ns['parking']()
        for mode, speed in (('forward', 350), ('reverse', 100)):
            self.ns['global_get_status'] = 1
            self.ns['global_get_XSpeed'] = 20 if mode == 'forward' else -20
            self.controller.set_commanded_motion(mode)
            self.gesture('hold_start', 'left', mode)
            self.gesture('hold_stop', 'left', mode, 6)
            self.assertEqual(('trim', 0, mode, speed), self.events[-1])

    def test_mqtt_status_uses_same_confirmed_stop_inputs(self):
        with io.open(os.path.join(ROOT, 'mqtt_integration.py'), encoding='utf-8') as handle:
            tree = ast.parse(handle.read().encode('utf-8'))
        calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
                 and isinstance(node.func, ast.Name) and node.func.id == 'derive_manual_motion_state']
        self.assertEqual(1, len(calls))
        data = dict(lowerMachineStatus=250, zSpeed=0, powerOnState=1)

        class StatusInput(object):
            def _get_redis_value(self, key, converter, default):
                return data.get(key, default)

        expression = ast.Expression(body=calls[0])
        result = eval(compile(expression, 'isolated-mqtt-classification', 'eval'),
                      dict(self=StatusInput(), live_speed=0, control_state='STOPPED',
                           fault_state='', manual_mode='none', current_action='parking',
                           hardware_report_at=100, derive_manual_motion_state=self.classify))
        self.assertEqual('stopped', result)


if __name__ == '__main__':
    unittest.main()
