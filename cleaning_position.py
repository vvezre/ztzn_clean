# coding=utf-8
"""Read-only automatic-cleaning telemetry, independent of modeling capture.

Python 2.7 compatible. No serial, Redis, Flask or motion imports. Control
threads only enqueue lifecycle notifications; conversion, compression and
disk writes happen in the telemetry worker, never in the steering callback.
"""
from __future__ import absolute_import

import copy
import hashlib
import heapq
import io
import json
import math
import os
import threading
import time
import uuid
try:
    from Queue import Queue, Empty
except ImportError:
    from queue import Queue, Empty

from modeling_coordinates import find_model_origin, lat_lon_to_model_xy_cm


# The robot already uses a 0.20 m task-origin start tolerance.  Keep the
# read-only cleaning-position view consistent without importing hardware main.
TASK_ORIGIN_TOLERANCE_CM = 20.0
PUBLIC_CONTROL_STATES = frozenset((
    'IDLE', 'RUNNING', 'STOPPED', 'START_FAILED', 'COMPLETE',
))
ACTIVE_RUNTIME_STATES = frozenset((
    'READY', 'RUNNING', 'PAUSED', 'STOPPING',
))
CLEANING_RUNTIME_ACTIONS = frozenset((
    'auto_drive', 'go_on', 'loop_auto_drive',
))
POSITION_ROUTE_RUNTIME_ACTIONS = frozenset(tuple(CLEANING_RUNTIME_ACTIONS) + (
    'return_to_point',
))


try:
    text_type = unicode
except NameError:
    text_type = str


def _state_text(value):
    if value is None:
        return ''
    try:
        return text_type(value).strip()
    except Exception:
        return ''


def resolve_live_cleaning_control_state(cached, fsm_state):
    """Resolve the cleaning-page state from the live runtime FSM.

    Position history deliberately has its own asynchronous worker so RTK
    callbacks never wait for disk or UI work.  Its cached public state can
    therefore lag a freshly accepted runtime transition.  The cleaning
    realtime endpoint must not report STOPPED while an automatic cleaning
    task is actually RUNNING, nor keep reporting RUNNING after that task has
    stopped.

    Only the public status value is resolved here.  Coordinates, history,
    motion commands and route execution remain completely untouched.
    """
    cached = cached if isinstance(cached, dict) else {}
    fsm_state = fsm_state if isinstance(fsm_state, dict) else {}
    runtime_state = _state_text(fsm_state.get('controlState')).upper()
    action = _state_text(fsm_state.get('action')).lower()
    detail = fsm_state.get('detail')
    detail = detail if isinstance(detail, dict) else {}
    detail_action = _state_text(detail.get('action')).lower()
    effective_action = action if action and action != 'idle' else detail_action

    if effective_action == 'return_to_point' and runtime_state in ACTIVE_RUNTIME_STATES:
        return 'RETURNING'

    if effective_action in CLEANING_RUNTIME_ACTIONS:
        if runtime_state in ACTIVE_RUNTIME_STATES:
            return 'RUNNING'
        if runtime_state == 'COMPLETE':
            return 'COMPLETE'
        if runtime_state == 'BLOCKED' and _state_text(cached.get('controlState')).upper() == 'START_FAILED':
            return 'START_FAILED'
        return 'STOPPED' if cached.get('runId') else 'IDLE'

    cached_state = _state_text(cached.get('controlState')).upper()
    if runtime_state == 'BLOCKED' and cached_state == 'START_FAILED':
        return 'START_FAILED'
    if runtime_state == 'COMPLETE' and cached_state == 'COMPLETE':
        return 'COMPLETE'
    if cached_state == 'IDLE' and not cached.get('runId'):
        return 'IDLE'
    return 'STOPPED'


def resolve_cleaning_coordinate_ready(at_task_origin, fsm_state):
    """Latch the public origin-ready flag while one cleaning run is active.

    Before and after a run the flag retains its original meaning: whether the
    vehicle is currently inside the selected task's origin tolerance.  Once a
    cleaning action has passed its start guard, READY/RUNNING/PAUSED/STOPPING
    all belong to the same accepted run, so leaving the origin must not make
    the frontend lose the task coordinate context mid-run.
    """
    fsm_state = fsm_state if isinstance(fsm_state, dict) else {}
    runtime_state = _state_text(fsm_state.get('controlState')).upper()
    action = _state_text(fsm_state.get('action')).lower()
    detail = fsm_state.get('detail')
    detail = detail if isinstance(detail, dict) else {}
    detail_action = _state_text(detail.get('action')).lower()
    effective_action = action if action and action != 'idle' else detail_action
    if (
            effective_action in CLEANING_RUNTIME_ACTIONS and
            runtime_state in ACTIVE_RUNTIME_STATES):
        return True
    return bool(at_task_origin)


def number(value):
    try:
        value = float(value)
        return value if not math.isnan(value) and not math.isinf(value) else None
    except (TypeError, ValueError, OverflowError):
        return None


def valid_origin(lat, lon):
    lat, lon = number(lat), number(lon)
    if lat is None or lon is None or not (-90 <= lat <= 90 and -180 <= lon <= 180):
        return None
    return {'lat': lat, 'lon': lon}


def cleaning_origin(task, model_loader=None):
    """Use this saved route's model frame, never active_session.json.

    startLat/startLon alone are NOT proof of the plotted coordinate origin.
    A fallback is safe only for a model-based task with a geographic point
    explicitly corresponding to x=y=0. Legacy rotated frames are not guessed.
    """
    task = task or {}
    frame = task.get('coordinateFrame') or {}
    if frame.get('type') == 'model_origin':
        origin = valid_origin(frame.get('originLat'), frame.get('originLon'))
        if origin:
            return origin
    if task.get('modelId') and callable(model_loader):
        try:
            model = model_loader(task['modelId'])
            origin = find_model_origin(model)
            if origin:
                return valid_origin(origin.get('lat'), origin.get('lon'))
        except Exception:
            pass
    if task.get('modelId'):
        for item in task.get('taskList') or []:
            for prefix in ('start', 'end'):
                if number(item.get(prefix + 'X')) == 0 and number(item.get(prefix + 'Y')) == 0:
                    origin = valid_origin(item.get(prefix + 'Lat'), item.get(prefix + 'Lon'))
                    if origin:
                        return origin
    return None


def route_position_snapshot(task, lat, lon, fixed, model_loader=None):
    """Project one RTK sample into a saved route's own model frame.

    This helper is read-only. It does not start a run, append history or
    modify the selected task. Both LAN realtime endpoints use it when a
    saved route is selected, so an idle vehicle no longer depends on an
    unrelated active-modeling session or the previous cleaning-run cache.
    """
    origin = cleaning_origin(task, model_loader)
    ready = origin is not None
    fixed = bool(fixed)
    lat, lon = number(lat), number(lon)
    xy = (
        lat_lon_to_model_xy_cm(origin, lat, lon)
        if ready and fixed and lat is not None and lon is not None
        else None
    )
    x = int(round(xy[0])) if xy is not None else None
    y = int(round(xy[1])) if xy is not None else None
    return {
        'x': x,
        'y': y,
        'coordinateReady': ready,
        'rtkFixAvailable': fixed,
        'atTaskOrigin': bool(
            ready and fixed and x is not None and y is not None and
            math.hypot(float(x), float(y)) <= TASK_ORIGIN_TOLERANCE_CM
        ),
    }


def resolve_position_task_name(selected_task_name, cached, fsm_state):
    """Choose the saved route whose origin owns the current position view.

    An active cleaning/return action wins over a later UI selection. Outside
    those actions, the explicitly selected route is authoritative; a stale
    previous-run cache is never silently presented as the current route.
    """
    cached = cached if isinstance(cached, dict) else {}
    fsm_state = fsm_state if isinstance(fsm_state, dict) else {}
    runtime_state = _state_text(fsm_state.get('controlState')).upper()
    action = _state_text(fsm_state.get('action')).lower()
    detail = fsm_state.get('detail')
    detail = detail if isinstance(detail, dict) else {}
    detail_action = _state_text(detail.get('action')).lower()
    effective_action = action if action and action != 'idle' else detail_action

    if runtime_state in ACTIVE_RUNTIME_STATES and effective_action in POSITION_ROUTE_RUNTIME_ACTIONS:
        detail_task_name = _state_text(detail.get('taskName'))
        if detail_task_name:
            return detail_task_name
        cached_state = _state_text(cached.get('controlState')).upper()
        cached_task_name = _state_text(cached.get('taskName'))
        if cached_state == 'RUNNING' and cached_task_name:
            return cached_task_name
        selected_task_name = _state_text(selected_task_name)
        return selected_task_name or cached_task_name or None

    selected_task_name = _state_text(selected_task_name)
    return selected_task_name or None


def modeling_position_context_is_current(status, modeling_updated_at,
                                         route_selected_at, selected_task_name):
    """Resolve an unfinished modeling session against a later route choice.

    A persisted ``recording`` state can survive an abandoned modeling page and
    a process restart.  Its file timestamp therefore wins only when it is
    newer than the last explicit saved-route selection.  On installations
    created before the timestamp key existed, an existing selected route is
    the safe migration default; the next start-modeling write immediately
    makes the modeling context newer.
    """
    if _state_text(status).lower() != 'recording':
        return False
    modeling_updated_at = number(modeling_updated_at)
    route_selected_at = number(route_selected_at)
    if route_selected_at is None:
        return not bool(_state_text(selected_task_name))
    return bool(
        modeling_updated_at is not None and
        modeling_updated_at > route_selected_at
    )


def _corner(a, b, c):
    """Protect >=30 degree sampled corners, including real reversals.

    This is DISPLAY compression only, unrelated to the 55 degree motor-turn
    threshold. Small bends also remain subject to the cumulative error bound.
    """
    ux, uy = b['x'] - a['x'], b['y'] - a['y']
    vx, vy = c['x'] - b['x'], c['y'] - b['y']
    length = math.hypot(ux, uy) * math.hypot(vx, vy)
    return length > 0 and (ux * vx + uy * vy) <= length * math.cos(math.pi / 6)


def _distance_to_segment(point, start, end):
    dx, dy = end['x'] - start['x'], end['y'] - start['y']
    length2 = float(dx * dx + dy * dy)
    if not length2:
        return math.hypot(point['x'] - start['x'], point['y'] - start['y'])
    ratio = ((point['x'] - start['x']) * dx + (point['y'] - start['y']) * dy) / length2
    ratio = max(0.0, min(1.0, ratio))
    return math.hypot(point['x'] - start['x'] - ratio * dx,
                      point['y'] - start['y'] - ratio * dy)


def simplify_history(points, target, tolerance_cm=5.0):
    """Remove least significant INTERIOR points; never FIFO/drop a prefix.

    Each surviving edge carries a conservative error certificate (_error).
    When A-B-C becomes A-C, previous error + distance(B,A-C) is an upper bound
    for all original samples on both edges. Repeated online passes therefore
    cannot silently accumulate unbounded drift. Gap endpoints and corners
    cannot be removed. If target is impossible safely, keep extra points.
    """
    result = [dict(point) for point in points]
    size = len(result)
    if size <= target:
        return result
    left, right = list(range(-1, size - 1)), list(range(1, size + 1))
    alive = [True] * size
    heap = []
    for index in range(1, size - 1):
        if (not result[index].get('breakBefore') and not result[index + 1].get('breakBefore')
                and _corner(result[index - 1], result[index], result[index + 1])):
            result[index]['_corner'] = True

    def offer(index):
        if index <= 0 or index >= size - 1 or not alive[index]:
            return
        a, c = left[index], right[index]
        b = result[index]
        if b.get('_corner') or b.get('breakBefore') or result[c].get('breakBefore'):
            return
        # Preserve traversal order: no shortcut across an out-and-back.
        ab = (b['x'] - result[a]['x'], b['y'] - result[a]['y'])
        bc = (result[c]['x'] - b['x'], result[c]['y'] - b['y'])
        if ab[0] * bc[0] + ab[1] * bc[1] < 0:
            return
        bound = max(float(b.get('_error', 0)), float(result[c].get('_error', 0)))
        bound += _distance_to_segment(b, result[a], result[c])
        if bound <= tolerance_cm:
            heapq.heappush(heap, (bound, index, a, c))

    for index in range(1, size - 1):
        offer(index)
    remaining = size
    while heap and remaining > max(2, target):
        bound, index, a, c = heapq.heappop(heap)
        if not alive[index] or left[index] != a or right[index] != c:
            continue
        # A neighbor's edge certificate may have changed without its index.
        current = max(float(result[index].get('_error', 0)), float(result[c].get('_error', 0)))
        current += _distance_to_segment(result[index], result[a], result[c])
        if abs(current - bound) > 1e-9:
            offer(index)
            continue
        alive[index] = False
        right[a], left[c] = c, a
        result[c]['_error'] = bound
        remaining -= 1
        offer(a)
        offer(c)
    return [point for index, point in enumerate(result) if alive[index]]


class CleaningPositionHistory(object):
    """One current/most recent run; selecting routes does not reset it."""
    def __init__(self, storage_path=None, now=None, max_points=1500,
                 history_interval=1.0, tolerance_cm=5.0):
        self.path = storage_path
        self.now = now or time.time
        self.max_points = max(2, int(max_points))
        self.history_interval = max(1.0, float(history_interval))
        self.tolerance_cm = max(0.0, float(tolerance_cm))
        self._lock = threading.RLock()
        self._run = {}
        self._points = []
        self._token = None
        self._state = 'STOPPED'
        self._public_state = 'IDLE'
        self._ended = True
        self._last_at = None
        self._gap = False
        self._dirty = False
        self._last_flush = 0
        self._simplified = False
        self._next_compact = self.max_points + 1
        self._latest = {'x': None, 'y': None, 'rtkFixAvailable': False}
        if self.path and os.path.isfile(self.path):
            try:
                with io.open(self.path, 'r', encoding='utf-8') as handle:
                    saved = json.load(handle)
                if isinstance(saved, dict) and saved.get('schemaVersion') == 1:
                    run, points = saved.get('run') or {}, saved.get('points') or []
                    if not isinstance(run, dict) or not isinstance(points, list):
                        raise ValueError('invalid cleaning cache')
                    origin = run.get('origin')
                    if origin is not None and (not isinstance(origin, dict) or
                            valid_origin(origin.get('lat'), origin.get('lon')) is None):
                        raise ValueError('invalid cleaning cache origin')
                    for point in points:
                        if not isinstance(point, dict) or number(point.get('x')) is None or number(point.get('y')) is None:
                            raise ValueError('invalid cleaning cache point')
                        if number(point.get('_error', 0)) is None or float(point.get('_error', 0)) < 0:
                            raise ValueError('invalid cleaning compression certificate')
                        point['x'], point['y'] = int(round(float(point['x']))), int(round(float(point['y'])))
                    self._run, self._points = run, points
                    self._simplified = bool(saved.get('simplified'))
                    if self._run.get('runId'):
                        self._public_state = 'STOPPED'
                    # Restart never resumes motion/history sampling by itself.
                    self._gap = bool(self._points)
                    self._next_compact = max(self.max_points + 1, len(self._points) + 100)
            except (IOError, OSError, ValueError, TypeError):
                pass

    def begin(self, task, origin, token, resume=False):
        with self._lock:
            if token and token == self._token and not self._ended:
                return  # duplicate accepted notification must not clear history
            identity = {key: task.get(key) for key in ('taskName', 'modelId', 'returnToOrigin')}
            identity['origin'] = copy.deepcopy(origin)
            identity['planHash'] = hashlib.sha256(json.dumps(task.get('taskList') or [],
                sort_keys=True, ensure_ascii=True).encode('utf-8')).hexdigest()
            if not (resume and self._run.get('identity') == identity):
                self._run = {'taskName': task.get('taskName'), 'origin': copy.deepcopy(origin),
                             'runId': 'clean_' + uuid.uuid4().hex, 'identity': identity}
                self._points = []
                self._last_at = None
                self._simplified = False
                self._next_compact = self.max_points + 1
            self._token = token
            self._ended = False
            self._state = 'RUNNING'
            self._public_state = 'RUNNING'
            self._gap = bool(self._points)
            self._latest = {'x': None, 'y': None, 'rtkFixAvailable': False}
            self._dirty = True

    def _append(self, x, y, at, force=False):
        if not force and self._last_at is not None and at - self._last_at < self.history_interval:
            return
        point = {'x': int(x), 'y': int(y)}
        if self._points:
            previous = self._points[-1]
            distance = math.hypot(x - previous['x'], y - previous['y'])
            if not self._gap and distance < (0.5 if force else 3.0):
                return
            if self._gap:
                point['breakBefore'] = True
            elif len(self._points) > 1 and not previous.get('breakBefore'):
                if _corner(self._points[-2], previous, point):
                    previous['_corner'] = True
        self._points.append(point)
        self._gap = False
        self._last_at = at
        if len(self._points) >= self._next_compact:
            before = len(self._points)
            self._points = simplify_history(self._points, int(self.max_points * 0.8), self.tolerance_cm)
            self._simplified = self._simplified or len(self._points) < before
            self._next_compact = max(self.max_points + 1, len(self._points) + 100)
        self._dirty = True

    def observe(self, lat, lon, fixed, state, token, at=None):
        at = float(self.now() if at is None else at)
        with self._lock:
            origin = self._run.get('origin')
            lat, lon = number(lat), number(lon)
            xy = lat_lon_to_model_xy_cm(origin, lat, lon) if fixed and lat is not None and lon is not None else None
            self._latest = {'x': int(round(xy[0])) if xy else None,
                            'y': int(round(xy[1])) if xy else None,
                            'rtkFixAvailable': bool(fixed)}
            matching = bool(token and token == self._token)
            previous_running = self._state == 'RUNNING' and not self._ended
            next_running = matching and state == 'RUNNING' and not self._ended
            if previous_running and not next_running and xy:
                self._append(self._latest['x'], self._latest['y'], at, force=True)
            if next_running and xy:
                self._append(self._latest['x'], self._latest['y'], at)
            if not xy or not next_running:
                self._gap = bool(self._points)
            if not matching or state not in ('RUNNING', 'PAUSED'):
                self._ended = True
            self._state = state if matching else 'STOPPED'
            # Expose a deliberately small UI state contract.  START_FAILED is
            # sticky until the next successful begin so a following BLOCKED or
            # STOPPED polling snapshot cannot erase the failure before the
            # frontend observes it.
            if matching and state == 'RUNNING':
                self._public_state = 'RUNNING'
            elif matching and state == 'COMPLETE':
                self._public_state = 'COMPLETE'
            elif state == 'BLOCKED' and token and not matching:
                self._public_state = 'START_FAILED'
            elif self._public_state != 'START_FAILED':
                self._public_state = 'STOPPED' if self._run.get('runId') else 'IDLE'

    def start_failed(self):
        """Remember a rejected start without creating or clearing a run."""
        with self._lock:
            self._public_state = 'START_FAILED'
            self._ended = True
            self._gap = bool(self._points)

    def flush(self, force=False):
        with self._lock:
            at = float(self.now())
            if not self.path or not self._dirty or (not force and at - self._last_flush < 2.0):
                return
            parent = os.path.dirname(os.path.abspath(self.path))
            if not os.path.isdir(parent):
                os.makedirs(parent)
            data = {'schemaVersion': 1, 'run': self._run, 'points': self._points,
                    'simplified': self._simplified}
            serialized = json.dumps(data, ensure_ascii=True)
            if not isinstance(serialized, type(u'')):
                serialized = serialized.decode('ascii')
            temporary = self.path + '.tmp'
            with io.open(temporary, 'w', encoding='utf-8') as handle:
                handle.write(serialized)
            getattr(os, 'replace', os.rename)(temporary, self.path)
            self._dirty = False
            self._last_flush = at

    def realtime(self):
        with self._lock:
            result = dict(self._latest)
            frame_ready = bool(self._run.get('origin'))
            x, y = result.get('x'), result.get('y')
            at_origin = bool(
                frame_ready and result.get('rtkFixAvailable') and
                x is not None and y is not None and
                math.hypot(float(x), float(y)) <= TASK_ORIGIN_TOLERANCE_CM
            )
            result.update({'taskName': self._run.get('taskName'), 'runId': self._run.get('runId'),
                           # Internal meaning retained for safe x/y gating in
                           # the API adapter.  atTaskOrigin becomes the public
                           # coordinateReady value on the cleaning endpoint.
                           'coordinateReady': frame_ready,
                           'atTaskOrigin': at_origin,
                           'controlState': self._public_state})
            return result

    def history(self):
        with self._lock:
            result = self.realtime()
            result.pop('x')
            result.pop('y')
            # The history endpoint contract is unchanged by the realtime-only
            # status addition.
            result.pop('atTaskOrigin', None)
            result.pop('controlState', None)
            result['points'] = [{key: point[key] for key in ('x', 'y', 'breakBefore') if key in point}
                                for point in self._points]
            result['simplified'] = self._simplified
            result['pointLimitExceeded'] = len(self._points) > self.max_points
            return result


class CleaningPositionService(object):
    """Non-blocking control-side notifications, single worker-side consumer."""
    def __init__(self, history, model_loader=None, on_error=None):
        self.history_store = history
        self.model_loader = model_loader
        self.on_error = on_error
        self.events = Queue()
        self._poll_lock = threading.Lock()

    def begin(self, task, token, resume=False, sample=None):
        # Only copy immutable task metadata needed later by the UI worker.
        self.events.put_nowait(('begin', copy.deepcopy(task), token, resume,
                               sample, self.history_store.now()))

    def state(self, state, token, sample):
        self.events.put_nowait(('state', state, token, tuple(sample), self.history_store.now()))

    def start_failed(self):
        self.events.put_nowait(('start_failed', self.history_store.now()))

    def poll(self, sample, state, token):
        if not self._poll_lock.acquire(False):
            return
        try:
            had_events = False
            while True:
                try:
                    event = self.events.get_nowait()
                except Empty:
                    break
                had_events = True
                if event[0] == 'begin':
                    task = event[1]
                    self.history_store.begin(task, cleaning_origin(task, self.model_loader), event[2], event[3])
                    if event[4] is not None:
                        self.history_store.observe(*(tuple(event[4]) + ('RUNNING', event[2], event[5])))
                elif event[0] == 'state':
                    self.history_store.observe(*(event[3] + (event[1], event[2], event[4])))
                else:
                    self.history_store.start_failed()
            # The caller may have read READY just before a successful begin
            # was queued. A newer lifecycle event wins over that old snapshot.
            if had_events:
                state, token = self.history_store._state, self.history_store._token
            self.history_store.observe(*(tuple(sample) + (state, token)))
            self.history_store.flush(force=self.history_store._state != 'RUNNING')
        except Exception as error:
            if callable(self.on_error):
                self.on_error(error)
        finally:
            self._poll_lock.release()

    def realtime(self):
        return self.history_store.realtime()

    def history(self):
        return self.history_store.history()
