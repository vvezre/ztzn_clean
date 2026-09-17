# coding=utf-8
"""Persistent automatic-cleaning logs for the LAN frontend.

The current-run polyline and the historical log are intentionally separate.
Only after a terminal log has been written atomically may the caller clear the
current-run polyline.  This module is Python 2.7 compatible and contains no
Flask, Redis, serial-port or motor-control dependency.
"""
from __future__ import absolute_import

import copy
import datetime
import io
import json
import math
import os
import threading
import time
import uuid

from cleaning_position import simplify_history


TERMINAL_STATUSES = frozenset(('COMPLETED', 'STOPPED', 'FAILED'))


def _number(value):
    try:
        value = float(value)
        return value if not math.isnan(value) and not math.isinf(value) else None
    except (TypeError, ValueError, OverflowError):
        return None


def _atomic_json_write(path, payload):
    parent = os.path.dirname(os.path.abspath(path))
    if not os.path.isdir(parent):
        os.makedirs(parent)
    serialized = json.dumps(payload, ensure_ascii=True, separators=(',', ':'))
    if not isinstance(serialized, type(u'')):
        serialized = serialized.decode('ascii')
    temporary = path + '.tmp'
    with io.open(temporary, 'w', encoding='utf-8') as handle:
        handle.write(serialized)
        handle.flush()
        try:
            os.fsync(handle.fileno())
        except (AttributeError, OSError):
            pass
    getattr(os, 'replace', os.rename)(temporary, path)


def _iso_local(timestamp):
    """Return an ISO-8601 local timestamp including its UTC offset."""
    timestamp = float(timestamp)
    local = datetime.datetime.fromtimestamp(timestamp)
    local_clock = time.localtime(timestamp)
    offset_seconds = -(
        time.altzone if local_clock.tm_isdst > 0 and time.daylight else time.timezone
    )
    sign = '+' if offset_seconds >= 0 else '-'
    offset_seconds = abs(offset_seconds)
    hours, remainder = divmod(offset_seconds, 3600)
    minutes = remainder // 60
    return '{}{}{:02d}:{:02d}'.format(
        local.strftime('%Y-%m-%dT%H:%M:%S'), sign, hours, minutes,
    )


def _public_point(point):
    result = {
        'x': point.get('x'),
        'y': point.get('y'),
        'heading': point.get('heading'),
        'timestamp': point.get('timestamp'),
    }
    if point.get('breakBefore'):
        result['breakBefore'] = True
    return result


class CleaningLogStore(object):
    """Store one file per run plus a lightweight, time-sorted index."""

    def __init__(self, storage_dir, now=None, max_points=1500,
                 history_interval=1.0, tolerance_cm=5.0):
        self.storage_dir = os.path.abspath(storage_dir)
        self.records_dir = os.path.join(self.storage_dir, 'records')
        self.index_path = os.path.join(self.storage_dir, 'index.json')
        self.now = now or time.time
        self.max_points = max(2, int(max_points))
        self.history_interval = max(1.0, float(history_interval))
        self.tolerance_cm = max(0.0, float(tolerance_cm))
        self._lock = threading.RLock()
        self._index = []
        self._active_id = None
        self._active_record = None
        self._last_at = None
        self._gap = False
        self._dirty = False
        self._last_flush = 0.0
        self._load()
        self._recover_interrupted_runs()

    def _record_path(self, log_id):
        return os.path.join(self.records_dir, '{}.json'.format(log_id))

    def _load(self):
        if not os.path.isfile(self.index_path):
            return
        try:
            with io.open(self.index_path, 'r', encoding='utf-8') as handle:
                payload = json.load(handle)
            items = payload.get('list') if isinstance(payload, dict) else None
            if not isinstance(items, list):
                return
            self._index = [item for item in items if isinstance(item, dict) and item.get('id')]
        except (IOError, OSError, ValueError, TypeError):
            self._index = []

    def _read_record(self, log_id):
        try:
            with io.open(self._record_path(log_id), 'r', encoding='utf-8') as handle:
                payload = json.load(handle)
            return payload if isinstance(payload, dict) else None
        except (IOError, OSError, ValueError, TypeError):
            return None

    def _write_index(self):
        _atomic_json_write(self.index_path, {
            'schemaVersion': 1,
            'list': self._index,
        })

    def _metadata(self, record):
        return {
            key: copy.deepcopy(record.get(key))
            for key in (
                'id', 'runId', 'productId', 'serialNumber', 'taskName',
                'startTime', 'endTime', 'durationSeconds', 'distanceMeters',
                'status', 'endReason',
            )
        }

    def _replace_metadata(self, record):
        metadata = self._metadata(record)
        replaced = False
        for index, item in enumerate(self._index):
            if item.get('id') == record.get('id'):
                self._index[index] = metadata
                replaced = True
                break
        if not replaced:
            self._index.append(metadata)
        self._index.sort(key=lambda item: item.get('startTime') or '', reverse=True)

    def _write_record_and_index(self, record):
        # The detail is written first.  If the index write fails, the caller
        # receives an exception and must not clear the current trajectory.
        _atomic_json_write(self._record_path(record['id']), record)
        self._replace_metadata(record)
        self._write_index()

    def _recover_interrupted_runs(self):
        """A process restart cannot leave a historical item RUNNING forever."""
        with self._lock:
            changed = False
            ended_at = float(self.now())
            for item in list(self._index):
                if item.get('status') != 'RUNNING':
                    continue
                record = self._read_record(item.get('id'))
                if not record:
                    continue
                started_at = _number(record.get('_startTimestamp')) or ended_at
                record.update({
                    'endTime': _iso_local(ended_at),
                    'durationSeconds': max(0, int(round(ended_at - started_at))),
                    'distanceMeters': None,
                    'status': 'FAILED',
                    'endReason': u'上位机重启，任务异常中断',
                    '_endTimestamp': ended_at,
                })
                self._write_record_and_index(record)
                changed = True
            if changed:
                self._write_index()

    def begin(self, run_id, identity, task_name, planned_route, at=None):
        """Create an idempotent RUNNING record after auto-drive really starts."""
        at = float(self.now() if at is None else at)
        identity = identity if isinstance(identity, dict) else {}
        with self._lock:
            current = self._active_record
            if current and current.get('runId') == run_id and current.get('status') == 'RUNNING':
                return copy.deepcopy(current)
            if current and current.get('status') == 'RUNNING':
                self._finalize_locked('FAILED', u'新的清扫任务启动，上一任务未正常结束', at)

            log_id = 'log_{}_{}'.format(
                time.strftime('%Y%m%d_%H%M%S', time.localtime(at)),
                uuid.uuid4().hex[:8],
            )
            record = {
                'id': log_id,
                'runId': run_id,
                'productId': identity.get('productId'),
                'serialNumber': identity.get('serialNumber'),
                'taskName': task_name,
                'startTime': _iso_local(at),
                'endTime': None,
                'durationSeconds': None,
                # The lower-machine odometer is not yet part of this feature.
                # Keep the contract stable and fill it in later.
                'distanceMeters': None,
                'status': 'RUNNING',
                'endReason': None,
                'trajectory': {
                    'coordinateSystem': 'LOCAL_XY',
                    'simplified': False,
                    'points': [],
                },
                'plannedRoute': copy.deepcopy(planned_route or {
                    'areaPoints': [], 'linkPoints': [], 'pathPoints': [],
                }),
                '_startTimestamp': at,
                '_endTimestamp': None,
            }
            self._write_record_and_index(record)
            self._active_id = log_id
            self._active_record = record
            self._last_at = None
            self._gap = False
            self._dirty = False
            self._last_flush = at
            return copy.deepcopy(record)

    def _append_locked(self, record, x, y, heading, at, force=False):
        x, y = _number(x), _number(y)
        if x is None or y is None:
            self._gap = True
            return False
        if not force and self._last_at is not None and at - self._last_at < self.history_interval:
            return False
        heading = _number(heading)
        point = {
            'x': int(round(x)),
            'y': int(round(y)),
            'heading': None if heading is None else round(heading % 360.0, 3),
            'timestamp': int(round(float(at) * 1000.0)),
        }
        points = record['trajectory']['points']
        if points:
            previous = points[-1]
            distance = math.hypot(point['x'] - previous['x'], point['y'] - previous['y'])
            if not self._gap and distance < (0.5 if force else 3.0):
                return False
            if self._gap:
                point['breakBefore'] = True
        points.append(point)
        self._gap = False
        self._last_at = float(at)
        if len(points) > self.max_points:
            before = len(points)
            points = simplify_history(points, int(self.max_points * 0.8), self.tolerance_cm)
            record['trajectory']['points'] = points
            if len(points) < before:
                record['trajectory']['simplified'] = True
        self._dirty = True
        return True

    def observe(self, x, y, heading, available=True, at=None, force=False):
        at = float(self.now() if at is None else at)
        with self._lock:
            if not self._active_id:
                return False
            record = self._active_record
            if not record or record.get('status') != 'RUNNING':
                return False
            if not available:
                self._gap = True
                return False
            appended = self._append_locked(record, x, y, heading, at, force=force)
            if appended and (force or at - self._last_flush >= 2.0):
                self._write_record_and_index(record)
                self._dirty = False
                self._last_flush = at
            return appended

    def _finalize_locked(self, status, end_reason, at):
        if status not in TERMINAL_STATUSES or not self._active_id:
            return None
        record = self._active_record or self._read_record(self._active_id)
        if not record:
            return None
        started_at = _number(record.get('_startTimestamp')) or float(at)
        record.update({
            'endTime': _iso_local(at),
            'durationSeconds': max(0, int(round(float(at) - started_at))),
            'distanceMeters': None,
            'status': status,
            'endReason': end_reason or '',
            '_endTimestamp': float(at),
        })
        self._write_record_and_index(record)
        self._active_id = None
        self._active_record = None
        self._last_at = None
        self._gap = False
        self._dirty = False
        self._last_flush = float(at)
        return copy.deepcopy(record)

    def finalize(self, status, end_reason, at=None):
        at = float(self.now() if at is None else at)
        with self._lock:
            return self._finalize_locked(status, end_reason, at)

    def active(self):
        with self._lock:
            record = self._active_record
            return copy.deepcopy(record) if record else None

    def list_logs(self, product_id, page=1, page_size=20):
        with self._lock:
            matches = [
                copy.deepcopy(item) for item in self._index
                if str(item.get('productId') or '') == str(product_id or '')
            ]
        page, page_size = int(page), int(page_size)
        start = (page - 1) * page_size
        return {
            'list': matches[start:start + page_size],
            'total': len(matches),
            'page': page,
            'pageSize': page_size,
        }

    def get_log(self, log_id, product_id=None):
        with self._lock:
            record = self._read_record(log_id)
            if not record:
                return None
            if product_id is not None and str(record.get('productId') or '') != str(product_id):
                return None
            public = {
                key: copy.deepcopy(value)
                for key, value in record.items()
                if not str(key).startswith('_')
            }
            trajectory = public.get('trajectory') or {}
            trajectory['points'] = [_public_point(point) for point in trajectory.get('points') or []]
            public['trajectory'] = trajectory
            return public
