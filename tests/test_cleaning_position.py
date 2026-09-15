# coding=utf-8
"""Portable offline telemetry tests; also runnable under robot Python 2.7."""
from __future__ import absolute_import
import copy
import io
import json
import math
import os
import shutil
import tempfile
import unittest

from cleaning_position import (CleaningPositionHistory, CleaningPositionService,
    cleaning_origin, simplify_history, _distance_to_segment,
    modeling_position_context_is_current, resolve_cleaning_coordinate_ready,
    resolve_live_cleaning_control_state,
    resolve_position_task_name, route_position_snapshot)
from modeling_coordinates import EARTH_RADIUS_M


ORIGIN = {'lat': 32.0, 'lon': 118.0}


def gps(x, y=0, origin=None):
    origin = origin or ORIGIN
    lat = origin['lat'] + math.degrees(y / 100.0 / EARTH_RADIUS_M)
    lon = origin['lon'] + math.degrees(x / 100.0 / EARTH_RADIUS_M /
                                     math.cos(math.radians((lat + origin['lat']) / 2.0)))
    return lat, lon


def task(name=u'中文清扫任务', origin=None):
    origin = origin or ORIGIN
    return {'taskName': name, 'modelId': 'saved_model', 'returnToOrigin': True,
            'coordinateFrame': {'type': 'model_origin', 'originLat': origin['lat'], 'originLon': origin['lon']},
            'taskList': [{'startX': 0, 'startY': 0, 'endX': 100, 'endY': 0}]}


class CleaningHistoryTests(unittest.TestCase):
    def setUp(self):
        self.clock = [100.0]
        self.tracker = CleaningPositionHistory(now=lambda: self.clock[0])

    def begin(self, name=u'中文清扫任务', token='token1', resume=False, origin=None):
        self.tracker.begin(task(name, origin), origin or ORIGIN, token, resume)

    def sample(self, x, y=0, state='RUNNING', token='token1', fixed=True, advance=1.0):
        self.clock[0] += advance
        self.tracker.observe(*(gps(x, y) + (fixed, state, token, self.clock[0])))

    def test_without_started_run_returns_no_coordinate_not_fake_zero(self):
        self.sample(10)
        self.assertEqual({'x': None, 'y': None, 'taskName': None, 'runId': None,
                          'coordinateReady': False, 'rtkFixAvailable': True,
                          'atTaskOrigin': False, 'controlState': 'IDLE'},
                         self.tracker.realtime())
        self.assertEqual([], self.tracker.history()['points'])

    def test_running_away_from_origin_has_realtime_position(self):
        self.begin()
        self.sample(123, 456)
        data = self.tracker.realtime()
        self.assertEqual((123, 456), (data['x'], data['y']))
        self.assertTrue(data['coordinateReady'])
        self.assertFalse(data['atTaskOrigin'])
        self.assertEqual('RUNNING', data['controlState'])
        self.assertEqual(u'中文清扫任务', data['taskName'])

    def test_origin_presence_uses_twenty_centimeter_tolerance(self):
        self.begin()
        self.sample(12, 16)
        self.assertTrue(self.tracker.realtime()['atTaskOrigin'])
        self.sample(21, 0)
        self.assertFalse(self.tracker.realtime()['atTaskOrigin'])

    def test_invalid_origin_does_not_fallback_to_current_car_position(self):
        self.tracker.begin(task(), None, 'token1')
        self.sample(100)
        self.assertFalse(self.tracker.realtime()['coordinateReady'])
        self.assertIsNone(self.tracker.realtime()['x'])

    def test_start_origin_and_variant_are_frozen(self):
        config = task()
        origin = dict(ORIGIN)
        self.tracker.begin(config, origin, 'token1')
        config['taskName'] = 'other'
        origin['lat'] += 1
        self.sample(9, 12)
        self.assertEqual((9, 12), (self.tracker.realtime()['x'], self.tracker.realtime()['y']))

    def test_duplicate_start_keeps_same_run_and_points(self):
        self.begin()
        self.sample(0)
        self.sample(30)
        before = self.tracker.history()
        self.begin()
        self.assertEqual(before['runId'], self.tracker.history()['runId'])
        self.assertEqual(before['points'], self.tracker.history()['points'])

    def test_new_successful_start_same_route_creates_new_run(self):
        self.begin()
        self.sample(0)
        self.sample(30, state='COMPLETE')
        previous = self.tracker.history()['runId']
        self.begin(token='token2')
        self.assertNotEqual(previous, self.tracker.history()['runId'])
        self.assertEqual([], self.tracker.history()['points'])

    def test_pause_and_resume_preserve_run_no_paused_points(self):
        self.begin()
        self.sample(0)
        self.sample(5, state='PAUSED', advance=0.1)
        previous = self.tracker.history()
        self.sample(10, state='PAUSED')
        self.sample(20, state='PAUSED')
        self.assertEqual(previous['points'], self.tracker.history()['points'])
        self.sample(30)
        self.assertEqual(previous['runId'], self.tracker.history()['runId'])
        self.assertTrue(self.tracker.history()['points'][-1]['breakBefore'])

    def test_explicit_go_on_reuses_run_with_new_runtime_token(self):
        self.begin()
        self.sample(0)
        self.sample(8, state='STOPPED', token='')
        previous = self.tracker.history()
        self.begin(token='token2', resume=True)
        self.sample(50, token='token2')
        self.assertEqual(previous['runId'], self.tracker.history()['runId'])
        self.assertEqual(previous['points'], self.tracker.history()['points'][:-1])

    def test_resume_other_route_or_variant_does_not_mix(self):
        self.begin()
        self.sample(0)
        previous = self.tracker.history()['runId']
        changed = task()
        changed['returnToOrigin'] = False
        self.tracker.begin(changed, ORIGIN, 'token2', resume=True)
        self.assertNotEqual(previous, self.tracker.history()['runId'])
        self.assertEqual([], self.tracker.history()['points'])

    def test_new_runtime_token_cannot_append_manual_or_other_mission(self):
        self.begin()
        self.sample(0)
        self.sample(5, token='manual')
        previous = self.tracker.history()['points']
        self.sample(200, token='manual')
        self.assertEqual(previous, self.tracker.history()['points'])

    def test_complete_or_stop_retains_final_point_no_later_manual_trail(self):
        for terminal in ('COMPLETE', 'STOPPED', 'FAULT', 'BLOCKED'):
            self.begin(token=terminal)
            self.sample(0, token=terminal)
            self.sample(8, token=terminal, state=terminal, advance=0.1)
            before = self.tracker.history()['points']
            self.assertEqual({'x': 8, 'y': 0}, before[-1])
            self.sample(90, token=terminal, state=terminal)
            self.assertEqual(before, self.tracker.history()['points'])

    def test_public_control_state_contract(self):
        self.assertEqual('IDLE', self.tracker.realtime()['controlState'])
        self.begin()
        self.assertEqual('RUNNING', self.tracker.realtime()['controlState'])
        self.sample(5, state='PAUSED')
        self.assertEqual('STOPPED', self.tracker.realtime()['controlState'])
        self.sample(6, state='RUNNING')
        self.sample(8, state='COMPLETE')
        self.assertEqual('COMPLETE', self.tracker.realtime()['controlState'])

    def test_rejected_start_is_distinct_and_does_not_clear_previous_run(self):
        self.begin()
        self.sample(0)
        previous = self.tracker.history()
        self.tracker.start_failed()
        self.assertEqual('START_FAILED', self.tracker.realtime()['controlState'])
        self.assertEqual(previous['runId'], self.tracker.history()['runId'])
        self.assertEqual(previous['points'], self.tracker.history()['points'])
        self.begin(token='token2')
        self.assertEqual('RUNNING', self.tracker.realtime()['controlState'])

    def test_rtk_loss_nulls_live_and_preserves_history_with_gap(self):
        self.begin()
        self.sample(0)
        previous = self.tracker.history()['points']
        self.sample(50, fixed=False)
        self.assertEqual(previous, self.tracker.history()['points'])
        self.assertIsNone(self.tracker.realtime()['x'])
        self.assertTrue(self.tracker.realtime()['coordinateReady'])
        self.assertFalse(self.tracker.realtime()['rtkFixAvailable'])
        self.sample(60)
        self.assertTrue(self.tracker.history()['points'][-1]['breakBefore'])

    def test_invalid_lat_lon_is_not_recorded(self):
        self.begin()
        for lat, lon in ((None, 118), (32, None), (float('nan'), 118), (32, float('inf'))):
            self.tracker.observe(lat, lon, True, 'RUNNING', 'token1')
            self.assertIsNone(self.tracker.realtime()['x'])
        self.assertEqual([], self.tracker.history()['points'])

    def test_sampling_one_second_three_cm_dedup_realtime_is_faster(self):
        self.begin()
        self.sample(0)
        self.sample(20, advance=0.2)
        self.assertEqual(20, self.tracker.realtime()['x'])
        self.assertEqual(1, len(self.tracker.history()['points']))
        self.sample(2, advance=0.9)
        self.assertEqual(1, len(self.tracker.history()['points']))
        self.sample(40)
        self.assertEqual(2, len(self.tracker.history()['points']))

    def test_over_1500_keeps_first_and_latest_instead_of_fifo(self):
        self.begin()
        for index in range(4000):
            self.sample(index * 5)
        data = self.tracker.history()
        self.assertEqual({'x': 0, 'y': 0}, data['points'][0])
        self.assertEqual({'x': 19995, 'y': 0}, data['points'][-1])
        self.assertLessEqual(len(data['points']), 1500)
        self.assertTrue(data['simplified'])
        self.assertFalse(data['pointLimitExceeded'])
        self.assertTrue(all(set(p) == {'x', 'y'} for p in data['points']))

    def test_complex_keypoints_over_limit_are_not_silently_deleted(self):
        self.tracker.max_points = 4
        self.tracker._next_compact = 5
        self.begin()
        for index in range(20):
            self.sample(index * 30, (index % 2) * 100)
        data = self.tracker.history()
        self.assertEqual(20, len(data['points']))
        self.assertTrue(data['pointLimitExceeded'])

    def test_persist_chinese_task_and_reload_without_resuming_capture(self):
        directory = tempfile.mkdtemp()
        try:
            path = os.path.join(directory, 'latest.json')
            self.tracker.path = path
            self.begin()
            self.sample(0)
            self.sample(10)
            self.tracker.flush(force=True)
            before = self.tracker.history()
            fresh = CleaningPositionHistory(path)
            fresh.observe(*(gps(90) + (True, 'RUNNING', 'token1')))
            self.assertEqual(before['points'], fresh.history()['points'])
            self.assertEqual(before['runId'], fresh.history()['runId'])
            self.assertEqual(before['taskName'], fresh.realtime()['taskName'])
        finally:
            shutil.rmtree(directory)

    def test_bad_telemetry_cache_cannot_break_program_startup(self):
        directory = tempfile.mkdtemp()
        try:
            path = os.path.join(directory, 'latest.json')
            for data in ([], {'schemaVersion': 1, 'run': []},
                         {'schemaVersion': 1, 'run': {'origin': 5}},
                         {'schemaVersion': 1, 'points': ['bad']},
                         {'schemaVersion': 1, 'points': [{'x': 'bad', 'y': 0}]}):
                with io.open(path, 'w', encoding='utf-8') as handle:
                    handle.write(json.dumps(data).decode('ascii') if isinstance(json.dumps(data), bytes)
                                 else json.dumps(data))
                fresh = CleaningPositionHistory(path)
                self.assertEqual([], fresh.history()['points'])
                self.assertIsNone(fresh.realtime()['runId'])
        finally:
            shutil.rmtree(directory)


class LiveCleaningControlStateTests(unittest.TestCase):
    def test_live_auto_clean_running_overrides_stale_stopped_cache(self):
        cached = {'runId': 'clean_old', 'controlState': 'STOPPED'}
        live = {'controlState': 'RUNNING', 'action': 'auto_drive'}
        self.assertEqual('RUNNING', resolve_live_cleaning_control_state(cached, live))

    def test_auto_clean_ready_pause_and_stopping_remain_running_to_ui(self):
        cached = {'runId': 'clean_current', 'controlState': 'STOPPED'}
        for state in ('READY', 'PAUSED', 'STOPPING'):
            live = {'controlState': state, 'action': 'auto_drive'}
            self.assertEqual('RUNNING', resolve_live_cleaning_control_state(cached, live))

    def test_stopped_runtime_clears_stale_running_cache(self):
        cached = {'runId': 'clean_current', 'controlState': 'RUNNING'}
        live = {'controlState': 'STOPPED', 'action': 'idle'}
        self.assertEqual('STOPPED', resolve_live_cleaning_control_state(cached, live))

    def test_manual_motion_is_not_reported_as_automatic_cleaning(self):
        cached = {'runId': 'clean_old', 'controlState': 'RUNNING'}
        live = {'controlState': 'RUNNING', 'action': 'manual_steering'}
        self.assertEqual('STOPPED', resolve_live_cleaning_control_state(cached, live))

    def test_return_and_completed_cleaning_use_live_runtime(self):
        cached = {'runId': 'clean_current', 'controlState': 'STOPPED'}
        returning = {'controlState': 'RUNNING', 'action': 'return_to_point'}
        complete = {'controlState': 'COMPLETE', 'action': 'idle',
                    'detail': {'action': 'auto_drive'}}
        self.assertEqual('RETURNING', resolve_live_cleaning_control_state(cached, returning))
        self.assertEqual('COMPLETE', resolve_live_cleaning_control_state(cached, complete))

    def test_start_failure_and_never_started_idle_are_preserved(self):
        failed = {'runId': 'clean_old', 'controlState': 'START_FAILED'}
        self.assertEqual('START_FAILED', resolve_live_cleaning_control_state(
            failed, {'controlState': 'BLOCKED', 'action': 'idle'}))
        self.assertEqual('IDLE', resolve_live_cleaning_control_state(
            {'runId': None, 'controlState': 'IDLE'},
            {'controlState': 'STOPPED', 'action': 'idle'}))

    def test_coordinate_ready_is_latched_only_during_accepted_cleaning_run(self):
        for state in ('READY', 'RUNNING', 'PAUSED', 'STOPPING'):
            self.assertTrue(resolve_cleaning_coordinate_ready(False, {
                'controlState': state,
                'action': 'auto_drive',
            }))
        self.assertFalse(resolve_cleaning_coordinate_ready(False, {
            'controlState': 'COMPLETE',
            'action': 'idle',
            'detail': {'action': 'auto_drive'},
        }))
        self.assertFalse(resolve_cleaning_coordinate_ready(False, {
            'controlState': 'BLOCKED',
            'action': 'auto_drive',
        }))
        self.assertTrue(resolve_cleaning_coordinate_ready(True, {
            'controlState': 'STOPPED',
            'action': 'idle',
        }))


class CompressionTests(unittest.TestCase):
    def points(self, coords):
        return [dict(x=x, y=y, index=i) for i, (x, y) in enumerate(coords)]

    def test_no_mutation_or_prefix_removal(self):
        points = self.points([(i, 0) for i in range(100)])
        original = copy.deepcopy(points)
        result = simplify_history(points, 5)
        self.assertEqual(original, points)
        self.assertEqual([0, 99], [result[0]['index'], result[-1]['index']])
        self.assertEqual(5, len(result))

    def test_square_corners_and_out_back_are_preserved(self):
        points = self.points([(0, 0), (20, 0), (40, 0), (40, 30), (40, 60),
                              (40, 30), (40, 0), (20, 0), (0, 0)])
        result = simplify_history(points, 2)
        indices = [p['index'] for p in result]
        for corner in (0, 2, 4, 6, 8):
            self.assertIn(corner, indices)

    def test_gap_endpoints_never_joined(self):
        points = self.points([(i * 10, 0) for i in range(12)])
        points[6]['breakBefore'] = True
        result = simplify_history(points, 2)
        self.assertEqual([0, 5, 6, 11], [p['index'] for p in result])
        self.assertTrue(result[2]['breakBefore'])

    def test_online_passes_bound_error_against_original_ordered_samples(self):
        original = self.points([(i * 2, int(round(35 * math.sin(i / 60.0)))) for i in range(3000)])
        retained = []
        for point in original:
            retained.append(dict(point))
            if len(retained) > 80:
                retained = simplify_history(retained, 65, 5)
        for left, right in zip(retained, retained[1:]):
            for raw in original[left['index']:right['index'] + 1]:
                self.assertLessEqual(_distance_to_segment(raw, left, right), 5.000001)
        self.assertEqual(0, retained[0]['index'])
        self.assertEqual(2999, retained[-1]['index'])
        self.assertGreater(len(retained), 2)


class OriginAndServiceTests(unittest.TestCase):
    def test_saved_route_snapshot_uses_its_model_origin(self):
        config = task()
        lat, lon = gps(25)
        snapshot = route_position_snapshot(config, lat, lon, True)
        self.assertEqual((25, 0), (snapshot['x'], snapshot['y']))
        self.assertTrue(snapshot['coordinateReady'])
        self.assertTrue(snapshot['rtkFixAvailable'])
        self.assertFalse(snapshot['atTaskOrigin'])

    def test_saved_route_snapshot_without_fixed_rtk_keeps_frame_but_nulls_xy(self):
        lat, lon = gps(25)
        snapshot = route_position_snapshot(task(), lat, lon, False)
        self.assertIsNone(snapshot['x'])
        self.assertIsNone(snapshot['y'])
        self.assertTrue(snapshot['coordinateReady'])
        self.assertFalse(snapshot['rtkFixAvailable'])
        self.assertFalse(snapshot['atTaskOrigin'])

    def test_saved_route_snapshot_can_load_origin_from_saved_model(self):
        seen = []
        config = {'taskName': 'selected', 'modelId': 'model-1', 'taskList': []}
        lat, lon = gps(8)
        snapshot = route_position_snapshot(
            config,
            lat,
            lon,
            True,
            lambda model_id: (
                seen.append(model_id) or {'groups': [{'points': [dict(ORIGIN)]}]}
            ),
        )
        self.assertEqual(['model-1'], seen)
        self.assertEqual((8, 0), (snapshot['x'], snapshot['y']))
        self.assertTrue(snapshot['atTaskOrigin'])

    def test_selected_route_wins_over_stale_stopped_cleaning_cache(self):
        cached = {'taskName': 'old', 'controlState': 'STOPPED'}
        fsm = {'action': 'idle', 'controlState': 'STOPPED'}
        self.assertEqual('selected', resolve_position_task_name('selected', cached, fsm))

    def test_active_cleaning_and_return_route_win_over_selected_route(self):
        running = {'taskName': 'running', 'controlState': 'RUNNING'}
        cleaning_fsm = {'action': 'auto_drive', 'controlState': 'RUNNING'}
        self.assertEqual('running', resolve_position_task_name('selected', running, cleaning_fsm))
        return_fsm = {
            'action': 'return_to_point',
            'controlState': 'RUNNING',
            'detail': {'taskName': 'returning'},
        }
        self.assertEqual('returning', resolve_position_task_name('selected', running, return_fsm))

    def test_later_route_selection_overrides_abandoned_recording_session(self):
        self.assertFalse(modeling_position_context_is_current(
            'recording', 100.0, 101.0, 'selected'))
        self.assertTrue(modeling_position_context_is_current(
            'recording', 102.0, 101.0, 'selected'))
        self.assertFalse(modeling_position_context_is_current(
            'ready', 102.0, 101.0, 'selected'))

    def test_pre_timestamp_migration_prefers_existing_selected_route(self):
        self.assertFalse(modeling_position_context_is_current(
            'recording', 100.0, None, 'selected'))
        self.assertTrue(modeling_position_context_is_current(
            'recording', 100.0, None, None))

    def test_model_loader_is_given_saved_model_id(self):
        seen = []
        def loader(model_id):
            seen.append(model_id)
            return {'groups': [{'points': [dict(ORIGIN)]}]}
        self.assertEqual(ORIGIN, cleaning_origin({'modelId': 'saved', 'startLat': 9, 'startLon': 8}, loader))
        self.assertEqual(['saved'], seen)

    def test_missing_model_fallback_requires_explicit_xy_zero(self):
        config = {'modelId': 'gone', 'startLat': 32, 'startLon': 118,
                  'taskList': [{'startX': 9, 'startY': 0, 'startLat': 32, 'startLon': 118}]}
        self.assertIsNone(cleaning_origin(config))
        config['taskList'][0]['startX'] = 0
        self.assertEqual(ORIGIN, cleaning_origin(config))
        config.pop('modelId')
        self.assertIsNone(cleaning_origin(config))

    def test_metadata_origin_range_is_validated(self):
        config = task()
        config['coordinateFrame']['originLat'] = 300
        self.assertIsNone(cleaning_origin(config))

    def test_control_notification_does_no_io_until_poll(self):
        calls = []
        tracker = CleaningPositionHistory()
        service = CleaningPositionService(tracker, lambda model_id: calls.append(model_id))
        config = {'modelId': 'saved', 'taskName': 'test', 'taskList': []}
        service.begin(config, 't1')
        self.assertEqual([], calls)
        self.assertIsNone(service.realtime()['runId'])
        service.poll(gps(0) + (True,), 'RUNNING', 't1')
        self.assertEqual(['saved'], calls)
        self.assertIsNotNone(service.realtime()['runId'])

    def test_queued_start_failure_is_visible_until_next_successful_begin(self):
        tracker = CleaningPositionHistory()
        service = CleaningPositionService(tracker)
        service.start_failed()
        service.poll(gps(0) + (True,), 'BLOCKED', '')
        self.assertEqual('START_FAILED', service.realtime()['controlState'])
        service.begin(task(), 't1', sample=gps(0) + (True,))
        service.poll(gps(0) + (True,), 'RUNNING', 't1')
        self.assertEqual('RUNNING', service.realtime()['controlState'])

    def test_queued_begin_wins_over_old_ready_snapshot(self):
        tracker = CleaningPositionHistory()
        service = CleaningPositionService(tracker)
        service.begin(task(), 't1', sample=gps(0) + (True,))
        service.poll(gps(5) + (True,), 'READY', 't1')
        self.assertFalse(tracker._ended)
        self.assertEqual({'x': 0, 'y': 0}, service.history()['points'][0])

    def test_end_event_retains_actual_terminal_position_not_later_manual_position(self):
        clock = [100.0]
        tracker = CleaningPositionHistory(now=lambda: clock[0])
        service = CleaningPositionService(tracker)
        service.begin(task(), 't1', sample=gps(0) + (True,))
        service.poll(gps(0) + (True,), 'RUNNING', 't1')
        clock[0] += 0.1
        service.state('STOPPED', '', gps(7) + (True,))
        service.poll(gps(800) + (True,), 'STOPPED', '')
        self.assertEqual({'x': 7, 'y': 0}, service.history()['points'][-1])

    def test_worker_disk_failure_does_not_raise_to_caller(self):
        errors = []
        tracker = CleaningPositionHistory()
        def fail(*args, **kwargs):
            raise IOError('disk error')
        tracker.flush = fail
        service = CleaningPositionService(tracker, on_error=errors.append)
        service.begin(task(), 't1')
        service.poll(gps(0) + (True,), 'RUNNING', 't1')
        self.assertEqual(1, len(errors))
        self.assertEqual(0, service.realtime()['x'])


if __name__ == '__main__':
    unittest.main()
