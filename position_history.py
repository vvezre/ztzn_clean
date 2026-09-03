# coding=utf-8
"""Realtime modeling position and bounded history storage.

The mini-program draws modeling points in a local centimetre coordinate frame
whose origin is the first valid point in area one.  This module watches the
active modeling draft, converts each fixed RTK sample into that same frame and
keeps a small persisted history for the LAN read-only APIs.

The implementation intentionally has no Flask, MQTT or motion dependency and
remains compatible with the robot's Python 2.7 runtime.
"""
from __future__ import absolute_import

import io
import json
import math
import os
import re
import threading
import time

from modeling_coordinates import find_model_origin, lat_lon_to_model_xy_cm


MODEL_ID_PATTERN = re.compile(r"^[A-Za-z0-9_-]+$")
HISTORY_SCHEMA_VERSION = 1


def _finite_number(value):
    if value is None or value == "":
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if not math.isnan(number) and not math.isinf(number) else None


def _read_json(path):
    with io.open(path, "r", encoding="utf-8") as handle:
        value = json.load(handle)
    return value if isinstance(value, dict) else None


def _file_signature(path):
    try:
        stat = os.stat(path)
    except OSError:
        return None
    return (
        getattr(stat, "st_mtime", None),
        getattr(stat, "st_size", None),
        getattr(stat, "st_ino", None),
    )


class ModelingPositionHistory(object):
    """Track the active modeling session in its own x/y coordinate frame."""

    def __init__(self, modeling_root, now=None, max_points=10000,
                 history_interval=0.2, minimum_distance_cm=3.0,
                 flush_interval=2.0):
        self.modeling_root = os.path.abspath(modeling_root)
        self.state_path = os.path.join(self.modeling_root, "active_session.json")
        self.history_dir = os.path.join(self.modeling_root, "position_history")
        self._now = now or time.time
        self.max_points = max(1, int(max_points))
        self.history_interval = max(0.0, float(history_interval))
        self.minimum_distance_cm = max(0.0, float(minimum_distance_cm))
        self.flush_interval = max(0.0, float(flush_interval))
        self._lock = threading.RLock()

        self._state_signature = None
        self._draft_signature = None
        self._cached_state = None
        self._cached_context = None
        self._last_context_check_at = 0.0

        self._model_id = None
        self._frame = None
        self._points = []
        self._latest = {
            "x": None,
            "y": None,
            "coordinateReady": False,
            "rtkFixAvailable": False,
        }
        self._last_history_at = None
        self._dirty = False
        self._last_flush_at = 0.0

    def _ensure_history_dir(self):
        if not os.path.exists(self.history_dir):
            try:
                os.makedirs(self.history_dir)
            except OSError:
                if not os.path.isdir(self.history_dir):
                    raise

    def _history_path(self, model_id):
        if not model_id or not MODEL_ID_PATTERN.match(str(model_id)):
            return None
        return os.path.join(self.history_dir, "{}.json".format(model_id))

    def _draft_path(self, model_id):
        if not model_id or not MODEL_ID_PATTERN.match(str(model_id)):
            return None
        return os.path.join(self.modeling_root, "drafts", "{}.json".format(model_id))

    def _frame_from_origin(self, origin):
        if not isinstance(origin, dict):
            return None
        lat = _finite_number(origin.get("lat"))
        lon = _finite_number(origin.get("lon"))
        if lat is None or lon is None:
            return None
        return {
            "pointId": origin.get("pointId"),
            "lat": round(lat, 10),
            "lon": round(lon, 10),
        }

    def _same_frame(self, left, right):
        if not isinstance(left, dict) or not isinstance(right, dict):
            return left is None and right is None
        return (
            str(left.get("pointId") or "") == str(right.get("pointId") or "")
            and abs(float(left.get("lat")) - float(right.get("lat"))) <= 1e-10
            and abs(float(left.get("lon")) - float(right.get("lon"))) <= 1e-10
        )

    def _refresh_context(self, now, force=False):
        # RTK may arrive at 10 Hz.  File signatures are checked at most 5 Hz;
        # the cached origin is still used for every live position update.
        if (
            not force
            and self._cached_context is not None
            and now - self._last_context_check_at < 0.2
        ):
            return self._cached_context
        self._last_context_check_at = now

        state_signature = _file_signature(self.state_path)
        if state_signature is None:
            self._state_signature = None
            self._draft_signature = None
            self._cached_state = None
            self._cached_context = {
                "modelId": None,
                "origin": None,
                "frame": None,
                "coordinateReady": False,
            }
            return self._cached_context

        if state_signature != self._state_signature or self._cached_state is None:
            try:
                self._cached_state = _read_json(self.state_path)
                self._state_signature = state_signature
                self._draft_signature = None
            except (IOError, OSError, ValueError):
                return self._cached_context

        state = self._cached_state if isinstance(self._cached_state, dict) else {}
        model_id = str(state.get("modelId") or "")
        draft_path = self._draft_path(model_id)
        if draft_path is None:
            context = {
                "modelId": None,
                "origin": None,
                "frame": None,
                "coordinateReady": False,
            }
            self._cached_context = context
            return context

        draft_signature = _file_signature(draft_path)
        if (
            self._cached_context is None
            or self._cached_context.get("modelId") != model_id
            or draft_signature != self._draft_signature
        ):
            try:
                draft = _read_json(draft_path)
            except (IOError, OSError, ValueError):
                return self._cached_context
            origin = find_model_origin(draft)
            frame = self._frame_from_origin(origin)
            self._draft_signature = draft_signature
            self._cached_context = {
                "modelId": model_id,
                "origin": origin,
                "frame": frame,
                "coordinateReady": frame is not None,
            }
        return self._cached_context

    def _normalize_points(self, points):
        normalized = []
        for raw in points or []:
            if not isinstance(raw, dict):
                continue
            x = _finite_number(raw.get("x"))
            y = _finite_number(raw.get("y"))
            if x is None or y is None:
                continue
            normalized.append({"x": int(round(x)), "y": int(round(y))})
        return normalized[-self.max_points:]

    def _load_history(self, model_id, frame):
        path = self._history_path(model_id)
        payload = None
        if path and os.path.exists(path):
            try:
                payload = _read_json(path)
            except (IOError, OSError, ValueError):
                payload = None
        if not isinstance(payload, dict) or not self._same_frame(payload.get("frame"), frame):
            return []
        return self._normalize_points(payload.get("points"))

    def _select_context(self, context, now):
        context = context if isinstance(context, dict) else {}
        model_id = context.get("modelId")
        frame = context.get("frame")
        changed_model = model_id != self._model_id
        changed_frame = not self._same_frame(frame, self._frame)
        if not changed_model and not changed_frame:
            return

        self._flush(now, force=True)
        self._model_id = model_id
        self._frame = frame
        self._last_history_at = None
        if model_id and frame:
            self._points = self._load_history(model_id, frame)
        else:
            self._points = []
        # A deleted/replaced first area point changes the complete coordinate
        # frame.  Persist the now-empty history so stale coordinates cannot
        # reappear after a process restart.
        self._dirty = bool(model_id)
        self._last_flush_at = now

    def _write_history(self):
        path = self._history_path(self._model_id)
        if path is None:
            return
        self._ensure_history_dir()
        payload = {
            "version": HISTORY_SCHEMA_VERSION,
            "modelId": self._model_id,
            "frame": self._frame,
            "points": self._points[-self.max_points:],
        }
        temporary_path = path + ".tmp"
        serialized = json.dumps(payload, ensure_ascii=True, separators=(",", ":"))
        if not isinstance(serialized, type(u"")):
            serialized = serialized.decode("utf-8")
        with io.open(temporary_path, "w", encoding="utf-8") as handle:
            handle.write(serialized)
        getattr(os, "replace", os.rename)(temporary_path, path)

    def _flush(self, now, force=False):
        if not self._dirty or not self._model_id:
            return
        if not force and now - self._last_flush_at < self.flush_interval:
            return
        self._write_history()
        self._dirty = False
        self._last_flush_at = now

    def _append_history(self, x, y, now):
        if self._last_history_at is not None and now - self._last_history_at < self.history_interval:
            return
        current = {"x": int(x), "y": int(y)}
        if self._points:
            previous = self._points[-1]
            distance = math.hypot(current["x"] - previous["x"], current["y"] - previous["y"])
            if distance < self.minimum_distance_cm:
                return
        self._points.append(current)
        if len(self._points) > self.max_points:
            self._points = self._points[-self.max_points:]
        self._last_history_at = now
        self._dirty = True

    def update(self, lat, lon, rtk_fix_available, force_context=False):
        """Consume one RTK snapshot and return the strict realtime contract."""
        now = float(self._now())
        with self._lock:
            context = self._refresh_context(now, force=force_context)
            self._select_context(context, now)
            ready = bool(context and context.get("coordinateReady"))
            fixed = bool(rtk_fix_available)
            x = None
            y = None
            lat = _finite_number(lat)
            lon = _finite_number(lon)
            if ready and fixed and lat is not None and lon is not None:
                xy = lat_lon_to_model_xy_cm(context.get("origin"), lat, lon)
                if xy is not None:
                    x = int(round(xy[0]))
                    y = int(round(xy[1]))
                    self._append_history(x, y, now)
            self._latest = {
                "x": x,
                "y": y,
                "coordinateReady": ready,
                "rtkFixAvailable": fixed,
            }
            self._flush(now)
            return dict(self._latest)

    def realtime(self):
        with self._lock:
            return dict(self._latest)

    def history(self):
        with self._lock:
            self._flush(float(self._now()), force=True)
            return {
                "points": [dict(point) for point in self._points],
                "coordinateReady": bool(self._latest.get("coordinateReady")),
                "rtkFixAvailable": bool(self._latest.get("rtkFixAvailable")),
            }

