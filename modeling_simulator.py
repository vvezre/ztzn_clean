# coding=utf-8
"""MQTT modeling simulator for mini-program integration testing.

This module intentionally does not import ``main.py`` or any hardware driver.
It acts as the dedicated test device ``-T01999999`` and reuses the production
modeling session/path algorithm.
"""

from __future__ import print_function

import argparse
import copy
import json
import math
import os
import signal
import sys
import threading
import time

from AppLogger import logger
from modeling_session import ModelingSession, ModelingSessionError
from modeling_store import ModelingStore
from modeling_task_persistence import normalize_task_name
from mqtt_handler import MQTTCommandHandler
from mqtt_vehicle_adapter import (
    _frontend_area_points,
    _frontend_link_points,
    _frontend_path_points,
)


SIMULATOR_COMPANY_CODE = "ZTZN-PVC"
SIMULATOR_PRODUCT_MODEL = "-T01"
SIMULATOR_PRODUCT_ID = "999999"
SIMULATOR_DEVICE_ID = SIMULATOR_PRODUCT_MODEL + SIMULATOR_PRODUCT_ID
SIMULATOR_ORIGIN_LAT = 32.03647857
SIMULATOR_ORIGIN_LON = 118.92448993
EARTH_RADIUS_M = 6371000.0


# 固定场景使用最近一次真实双区域建模的相对尺寸和倾斜程度。
#
# 前端记录顺序仍严格保持为：
#   区域1四点 -> 两个连接点 -> new_modeling_area -> 区域2四点。
#
# 原始真实记录中区域1的右侧两点是在区域2之后补录的；显式分区流程要求先完整
# 记录区域1，所以这里只调整录入顺序，不改变各点坐标和最终区域轮廓。
SCENARIO_POINTS = (
    {"pointType": "area", "name": "home_lower_left", "x": 0.000, "y": 0.000},
    {"pointType": "area", "name": "home_upper_left", "x": 7.362, "y": 117.944},
    {"pointType": "area", "name": "home_upper_right", "x": 344.374, "y": 86.365},
    {"pointType": "area", "name": "home_lower_right", "x": 342.394, "y": -61.646},
    {"pointType": "link", "name": "bridge_1_start", "x": 9.502, "y": 170.529},
    {"pointType": "link", "name": "bridge_1_end", "x": 10.576, "y": 204.143},
    {"pointType": "area", "name": "upper_lower_left", "x": 16.882, "y": 260.830},
    {"pointType": "area", "name": "upper_upper_left", "x": 23.914, "y": 399.779},
    {"pointType": "area", "name": "upper_upper_right", "x": 379.429, "y": 363.040},
    {"pointType": "area", "name": "upper_lower_right", "x": 368.523, "y": 235.322},
    # Bridge 2 joins the midpoint of area two's right edge to the midpoint of
    # area three's left edge.  Area three keeps the same tilted footprint as
    # area two and is translated to its right.
    {"pointType": "link", "name": "bridge_2_start", "x": 373.976, "y": 299.181},
    {"pointType": "link", "name": "bridge_2_end", "x": 433.516, "y": 299.182},
    {"pointType": "area", "name": "right_lower_left", "x": 430.000, "y": 229.707},
    {"pointType": "area", "name": "right_upper_left", "x": 437.032, "y": 368.656},
    {"pointType": "area", "name": "right_upper_right", "x": 792.547, "y": 331.917},
    {"pointType": "area", "name": "right_lower_right", "x": 781.641, "y": 204.199},
)

# A new area must be created immediately before recording scenario points 7
# and 13 (zero-based indexes 6 and 12).  Keeping these boundaries explicit
# makes the manual MQTT flow and the automatic preload flow identical.
SCENARIO_NEW_AREA_INDEXES = frozenset((6, 12))


class ModelingSimulatorError(Exception):
    def __init__(self, code, message):
        super(ModelingSimulatorError, self).__init__(message)
        self.code = code
        self.message = message


def xy_to_lat_lon(x, y, origin_lat=SIMULATOR_ORIGIN_LAT, origin_lon=SIMULATOR_ORIGIN_LON):
    """Convert simulator-local centimeters into an RTK-like coordinate."""
    dy_m = float(y) / 100.0
    dx_m = float(x) / 100.0
    lat = float(origin_lat) + math.degrees(dy_m / EARTH_RADIUS_M)
    mean_lat = math.radians((float(origin_lat) + lat) / 2.0)
    lon = float(origin_lon) + math.degrees(dx_m / (EARTH_RADIUS_M * math.cos(mean_lat)))
    return round(lat, 8), round(lon, 8)


def scenario_point(index):
    spec = SCENARIO_POINTS[index]
    lat, lon = xy_to_lat_lon(spec["x"], spec["y"])
    return {
        "id": "sim_p{:02d}".format(index + 1),
        "role": "unknown",
        "roles": [],
        "lat": lat,
        "lon": lon,
        "x": spec["x"],
        "y": spec["y"],
        "heading": 0,
        "source": "modeling_simulator",
        "sample": {
            "count": 1,
            "radiusM": 0,
            "quality": "4",
            "ggaAgeSec": 0,
            "scenarioIndex": index + 1,
            "scenarioName": spec["name"],
        },
    }


def build_simulator_mqtt_config(base_config):
    """Force a base MQTT config onto the one permitted simulator identity."""
    config = copy.deepcopy(base_config or {})
    mqtt = config.setdefault("mqtt", {})
    mqtt["company_code"] = SIMULATOR_COMPANY_CODE
    mqtt["product_model"] = SIMULATOR_PRODUCT_MODEL
    mqtt["product_id"] = SIMULATOR_PRODUCT_ID
    mqtt["status_interval"] = max(float(mqtt.get("status_interval", 2)), 0.5)
    mqtt["position_interval"] = max(float(mqtt.get("position_interval", 0.2)), 0.1)
    config["topics"] = {
        "subscribe": "RAILCAR/S/{}".format(SIMULATOR_DEVICE_ID),
        "publish": "RAILCAR/R/{}".format(SIMULATOR_DEVICE_ID),
        "startup_topic": "RAILCAR/S/{}/startup".format(SIMULATOR_DEVICE_ID),
    }
    return config


def build_playback_samples(task_plan, step_cm=25.0):
    """Expand task segments into local positions for WebSocket animation."""
    if not isinstance(task_plan, dict):
        raise ModelingSimulatorError("SIMULATOR_PATH_MISSING", "modeling path is missing")
    tasks = task_plan.get("tasks") or []
    if not tasks:
        raise ModelingSimulatorError("SIMULATOR_PATH_EMPTY", "modeling path has no tasks")
    step_cm = max(float(step_cm), 1.0)
    samples = []
    for task_index, task in enumerate(tasks):
        try:
            start_x = float(task.get("startX"))
            start_y = float(task.get("startY"))
            end_x = float(task.get("endX"))
            end_y = float(task.get("endY"))
        except (TypeError, ValueError):
            raise ModelingSimulatorError(
                "SIMULATOR_PATH_INVALID",
                "task {} has invalid local coordinates".format(task_index + 1),
            )
        length = math.hypot(end_x - start_x, end_y - start_y)
        step_count = max(1, int(math.ceil(length / step_cm)))
        first_step = 0 if task_index == 0 else 1
        for step_index in range(first_step, step_count + 1):
            ratio = float(step_index) / float(step_count)
            local_x = int(round(start_x + (end_x - start_x) * ratio))
            local_y = int(round(start_y + (end_y - start_y) * ratio))
            lat, lon = xy_to_lat_lon(local_x, local_y)
            samples.append({
                "local_x": local_x,
                "local_y": local_y,
                "lat": lat,
                "lon": lon,
                "taskId": task.get("id"),
                "taskIndex": task_index + 1,
                "taskTotal": len(tasks),
                "mode": int(task.get("mode") or 2),
                "areaNumber": int(task.get("areaNumber") or 1),
                "segmentProgress": round(ratio, 4),
                "source": task.get("source"),
            })
    return samples


class ModelingPathPlayer(object):
    def __init__(self, on_position=None, step_cm=25.0, interval=0.2):
        self.on_position = on_position
        self.step_cm = max(float(step_cm), 1.0)
        self.interval = max(float(interval), 0.01)
        self._lock = threading.RLock()
        self._samples = []
        self._index = 0
        self._state = "idle"
        self._thread = None
        self._cancel_event = threading.Event()
        self._resume_event = threading.Event()
        self._resume_event.set()

    def load(self, task_plan):
        self.cancel()
        samples = build_playback_samples(task_plan, self.step_cm)
        with self._lock:
            self._samples = samples
            self._index = 0
            self._state = "ready"
        return len(samples)

    def start(self):
        with self._lock:
            if not self._samples:
                raise ModelingSimulatorError("SIMULATOR_PATH_EMPTY", "generate a modeling path first")
            if self._thread and self._thread.is_alive():
                if self._state == "paused":
                    self._state = "running"
                    self._resume_event.set()
                    return {"state": self._state, "positionCount": len(self._samples)}
                return {"state": self._state, "positionCount": len(self._samples)}
            self._index = 0
            self._state = "running"
            self._cancel_event = threading.Event()
            self._resume_event.set()
            self._thread = threading.Thread(target=self._run)
            self._thread.daemon = True
            self._thread.start()
            return {"state": self._state, "positionCount": len(self._samples)}

    def _run(self):
        while not self._cancel_event.is_set():
            if not self._resume_event.wait(0.1):
                continue
            with self._lock:
                if self._index >= len(self._samples):
                    self._state = "complete"
                    return
                sample = dict(self._samples[self._index])
                self._index += 1
            if callable(self.on_position):
                self.on_position(sample)
            if self._cancel_event.wait(self.interval):
                return
        with self._lock:
            if self._state not in ("ready", "complete"):
                self._state = "stopped"

    def pause(self):
        with self._lock:
            if self._state != "running":
                return {"state": self._state, "positionIndex": self._index}
            self._state = "paused"
            self._resume_event.clear()
            return {"state": self._state, "positionIndex": self._index}

    def resume(self):
        with self._lock:
            if self._state != "paused":
                return self.start()
            self._state = "running"
            self._resume_event.set()
            return {"state": self._state, "positionIndex": self._index}

    def cancel(self):
        with self._lock:
            thread = self._thread
            self._cancel_event.set()
            self._resume_event.set()
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        with self._lock:
            self._thread = None
            self._index = 0
            self._state = "ready" if self._samples else "idle"

    def wait(self, timeout=None):
        with self._lock:
            thread = self._thread
        if thread:
            thread.join(timeout=timeout)
        return not (thread and thread.is_alive())

    def snapshot(self):
        with self._lock:
            return {
                "state": self._state,
                "positionIndex": self._index,
                "positionCount": len(self._samples),
            }


class ScenarioPointProvider(object):
    def __init__(self):
        self._lock = threading.Lock()
        self._pending = None

    def prepare(self, point):
        with self._lock:
            self._pending = dict(point)

    def clear(self):
        with self._lock:
            self._pending = None

    def __call__(self):
        with self._lock:
            if self._pending is None:
                raise ModelingSimulatorError(
                    "SIMULATOR_POINT_NOT_PREPARED",
                    "simulator point was not prepared",
                )
            point = dict(self._pending)
            self._pending = None
            return point


class ModelingSimulatorController(object):
    def __init__(self, data_dir, playback_step_cm=25.0, playback_interval=0.2, now=None):
        self.store = ModelingStore(data_dir, now=now)
        self.point_provider = ScenarioPointProvider()
        self.session = ModelingSession(self.store, self.point_provider, now=now)
        self.position_callback = None
        self._position_lock = threading.RLock()
        self._tasks_lock = threading.RLock()
        self._saved_tasks = {}
        self._current_task_name = None
        self._current_position = dict(scenario_point(0), local_x=0, local_y=0, mode=2)
        self.player = ModelingPathPlayer(
            on_position=self._on_playback_position,
            step_cm=playback_step_cm,
            interval=playback_interval,
        )

    def set_position_callback(self, callback):
        self.position_callback = callback

    def _success(self, message, data=None):
        response = {"success": True, "message": message}
        if data is not None:
            response["data"] = data
        return response

    def _failure(self, error):
        code = getattr(error, "code", "SIMULATOR_ERROR")
        message = getattr(error, "message", None) or str(error)
        return {
            "success": False,
            "message": message,
            "data": {"code": code, "simulator": True},
        }

    def _call(self, message, callback):
        try:
            return self._success(message, callback())
        except Exception as error:
            logger.warning("Simulator command failed: {}".format(str(error)))
            return self._failure(error)

    def _capture_events(self):
        state = self.session.current()
        model_id = state.get("modelId")
        if not model_id:
            return []
        draft = self.store.get_draft(model_id)
        return list(draft.get("captureSequence") or [])

    def _next_scenario_point(self, point_type):
        events = self._capture_events()
        for index, event in enumerate(events):
            if index >= len(SCENARIO_POINTS) or event.get("pointType") != SCENARIO_POINTS[index]["pointType"]:
                raise ModelingSimulatorError(
                    "SIMULATOR_SCENARIO_ORDER_INVALID",
                    "recorded points no longer match the fixed simulation order; restart modeling",
                )
        index = len(events)
        if index >= len(SCENARIO_POINTS):
            raise ModelingSimulatorError(
                "SIMULATOR_SCENARIO_COMPLETE",
                "all ten simulation points have already been recorded",
            )
        expected_type = SCENARIO_POINTS[index]["pointType"]
        if point_type != expected_type:
            raise ModelingSimulatorError(
                "SIMULATOR_WRONG_BUTTON",
                "point {} requires the {} button".format(index + 1, expected_type),
            )
        return index, scenario_point(index)

    def _record(self, point_type, record_callback):
        index, point = self._next_scenario_point(point_type)
        self.point_provider.prepare(point)
        try:
            result = record_callback()
        finally:
            self.point_provider.clear()
        with self._position_lock:
            self._current_position = dict(point, local_x=point["x"], local_y=point["y"], mode=2)
        result["simulator"] = {
            "deviceId": SIMULATOR_DEVICE_ID,
            "scenarioIndex": index + 1,
            "scenarioTotal": len(SCENARIO_POINTS),
            "scenarioName": SCENARIO_POINTS[index]["name"],
        }
        return result

    def _on_playback_position(self, sample):
        with self._position_lock:
            self._current_position = dict(sample)
        if callable(self.position_callback):
            self.position_callback(dict(sample))

    def start_modeling(self, name=None, restart=False):
        def action():
            self.player.cancel()
            result = self.session.start(name or "simulator-two-areas", restart=restart)
            events = self._capture_events()
            index = max(0, min(len(events) - 1, len(SCENARIO_POINTS) - 1))
            point = scenario_point(index)
            with self._position_lock:
                self._current_position = dict(point, local_x=point["x"], local_y=point["y"], mode=2)
            result["simulator"] = {
                "deviceId": SIMULATOR_DEVICE_ID,
                "scenarioPointCount": len(SCENARIO_POINTS),
            }
            return result
        return self._call("modeling simulator started", action)

    def get_modeling_state(self):
        return self._call("modeling simulator state fetched", self.session.current)

    def new_modeling_area(self):
        return self._call("simulation modeling area created", self.session.new_area)

    def preload_scenario(self):
        """Create the complete fixed two-area scenario before MQTT starts."""
        def action():
            self.player.cancel()
            started = self.session.start("simulator-two-areas", restart=True)
            for index, spec in enumerate(SCENARIO_POINTS):
                if index in SCENARIO_NEW_AREA_INDEXES:
                    self.session.new_area()
                if spec["pointType"] == "area":
                    self._record("area", self.session.record_area_point)
                else:
                    self._record("link", self.session.record_link_point)
            finished = self.session.finish()
            task_plan = finished.get("taskPlan") or {}
            self.player.load(task_plan)
            return {
                "modelId": started.get("modelId"),
                "scenarioPointCount": len(SCENARIO_POINTS),
                "areaPointCount": sum(
                    1 for point in SCENARIO_POINTS if point["pointType"] == "area"
                ),
                "linkPointCount": sum(
                    1 for point in SCENARIO_POINTS if point["pointType"] == "link"
                ),
                "pathTaskCount": len(task_plan.get("tasks") or []),
            }
        return self._call("modeling simulator scenario preloaded", action)

    def get_modeling_points(self, model_id=None):
        def action():
            current_model_id = model_id or self.session.current().get("modelId")
            if not current_model_id:
                raise ModelingSimulatorError(
                    "MODELING_SESSION_NOT_STARTED",
                    "modeling has not been started",
                )
            draft = self.store.get_draft(current_model_id)
            return {
                "points": _frontend_area_points(draft),
            }
        return self._call("modeling points fetched", action)

    def get_modeling_link_points(self, model_id=None):
        def action():
            current_model_id = model_id or self.session.current().get("modelId")
            if not current_model_id:
                raise ModelingSimulatorError(
                    "MODELING_SESSION_NOT_STARTED",
                    "modeling has not been started",
                )
            draft = self.store.get_draft(current_model_id)
            return {
                "points": _frontend_link_points(draft),
            }
        return self._call("modeling link points fetched", action)

    def sample_modeling_point(self, model_id=None, group_id=None):
        return self._call(
            "simulation area point recorded",
            lambda: self._record("area", self.session.record_area_point),
        )

    def sample_modeling_link_point(self, model_id=None, link_id=None):
        return self._call(
            "simulation link point recorded",
            lambda: self._record("link", self.session.record_link_point),
        )

    def undo_modeling_point(self, point_type=None):
        return self._call("simulation point undone", lambda: self.session.undo(point_type))

    def delete_modeling_point(self, point_id):
        return self._call(
            "simulation area point deleted",
            lambda: self.session.delete_area_point(point_id),
        )

    def delete_modeling_link_point(self, point_id):
        return self._call(
            "simulation connection point deleted",
            lambda: self.session.delete_link_point(point_id),
        )

    def clear_modeling_points(self, point_type=None):
        return self._call("simulation points cleared", lambda: self.session.clear(point_type))

    def clear_all_modeling_points(self, point_type):
        result = self._call(
            "all simulation {} points cleared".format(point_type),
            lambda: self.session.clear_all(point_type),
        )
        data = result.get("data") if isinstance(result, dict) else None
        if isinstance(data, dict) and data.get("resetToFirstArea"):
            # The simulator advances through a fixed list of ten realistic
            # samples.  When the shared FSM collapses an emptied draft back to
            # area one, align the simulated live position with the first sample
            # as well so the next record command starts at scenario point one.
            point = scenario_point(0)
            with self._position_lock:
                self._current_position = dict(
                    point,
                    local_x=point["x"],
                    local_y=point["y"],
                    mode=2,
                )
        return result

    def finish_modeling(self):
        def action():
            result = self.session.finish()
            self.player.load(result.get("taskPlan"))
            result["simulator"] = {
                "deviceId": SIMULATOR_DEVICE_ID,
                "playbackCommand": "auto_drive",
            }
            return result
        return self._call("simulation modeling path generated", action)

    def replan_modeling_route(self, area_order):
        def action():
            result = self.session.replan(area_order)
            self.player.load(result.get("taskPlan"))
            return result
        return self._call("simulation modeling route replanned", action)

    def get_modeling_path(self, model_id=None):
        def action():
            result = self.session.current_path()
            self.player.load(result.get("taskPlan"))
            return result
        return self._call("simulation modeling path fetched", action)

    def get_modeling_result(self, model_id=None):
        def action():
            current_model_id = model_id or self.session.current().get("modelId")
            if not current_model_id:
                raise ModelingSimulatorError(
                    "MODELING_SESSION_NOT_STARTED",
                    "modeling has not been started",
                )
            draft = self.store.get_draft(current_model_id)
            task_plan = draft.get("taskPlan")
            if not isinstance(task_plan, dict) or task_plan.get("status") != "ready":
                raise ModelingSimulatorError(
                    "MODELING_PATH_NOT_READY",
                    "modeling task plan is not ready",
                )
            self.player.load(task_plan)
            return {
                "areaOrder": task_plan.get("areaOrder") or [],
                "areaPoints": _frontend_area_points(draft),
                "linkPoints": _frontend_link_points(draft),
                "pathPoints": _frontend_path_points(task_plan),
            }
        return self._call("simulation modeling result fetched", action)

    def save_modeling_task(self, task_name):
        def action():
            name = normalize_task_name(task_name)
            current_path = self.session.current_path()
            task_plan = current_path.get("taskPlan") or {}
            tasks = task_plan.get("tasks") or []
            if not tasks:
                raise ModelingSimulatorError(
                    "MODELING_PATH_EMPTY",
                    "modeling path contains no executable tasks",
                )
            with self._tasks_lock:
                if name in self._saved_tasks:
                    raise ModelingSimulatorError(
                        "TASK_NAME_EXISTS",
                        "taskName already exists",
                    )
                model_id = current_path.get("modelId")
                self._saved_tasks[name] = {
                    "modelId": model_id,
                    "taskPlan": copy.deepcopy(task_plan),
                    "model": copy.deepcopy(self.store.get_model(model_id)),
                }
            return {
                "taskName": name,
                "taskCount": len(tasks),
                "modelId": current_path.get("modelId"),
            }
        return self._call("simulation modeling task saved", action)

    def get_task_names(self):
        def action():
            with self._tasks_lock:
                return {
                    "taskNames": sorted(self._saved_tasks.keys()),
                    "currentTaskName": self._current_task_name,
                }
        return self._call("simulation task names fetched", action)

    def get_saved_routes(self):
        def action():
            with self._tasks_lock:
                saved_tasks = copy.deepcopy(self._saved_tasks)
                current_task_name = self._current_task_name
            routes = []
            for name in sorted(saved_tasks.keys()):
                saved = saved_tasks[name]
                task_plan = saved.get("taskPlan") or {}
                model = saved.get("model") or {}
                routes.append({
                    "taskName": name,
                    "modelId": saved.get("modelId"),
                    "current": name == current_task_name,
                    "taskCount": len(task_plan.get("tasks") or []),
                    # Match the real FSM saved-routes response: every named
                    # route carries the area order used to generate its path.
                    "areaOrder": copy.deepcopy(task_plan.get("areaOrder") or []),
                    "areaPoints": _frontend_area_points(model),
                    "linkPoints": _frontend_link_points(model),
                    "pathPoints": _frontend_path_points(task_plan),
                })
            return {
                "routes": routes,
                "currentTaskName": current_task_name,
            }
        return self._call("simulation saved routes fetched", action)

    def set_current_task(self, task_name):
        def action():
            name = normalize_task_name(task_name)
            with self._tasks_lock:
                saved = self._saved_tasks.get(name)
                if saved is None:
                    raise ModelingSimulatorError(
                        "TASK_NOT_FOUND",
                        "saved task does not exist",
                    )
                self._current_task_name = name
                selected_plan = copy.deepcopy(saved.get("taskPlan") or {})
            self.player.load(selected_plan)
            return {"taskName": name}
        return self._call("simulation task selected", action)

    def auto_drive(self):
        return self._call("simulation path playback started", self.player.start)

    def stop(self):
        return self._call("simulation path playback paused", self.player.pause)

    def go_on(self):
        return self._call("simulation path playback resumed", self.player.resume)

    def parking(self):
        def action():
            self.player.cancel()
            return self.player.snapshot()
        return self._call("simulation path playback stopped", action)

    def get_status(self):
        return self._success("simulation status fetched", self.status_snapshot())

    def current_position(self):
        with self._position_lock:
            position = dict(self._current_position)
        position.setdefault("local_x", int(position.get("x") or 0))
        position.setdefault("local_y", int(position.get("y") or 0))
        return position

    def status_snapshot(self):
        playback = self.player.snapshot()
        position = self.current_position()
        running = playback["state"] == "running"
        paused = playback["state"] == "paused"
        complete = playback["state"] == "complete"
        return {
            "status": "working" if running or paused else "active",
            "online_state": "ONLINE",
            "speed": 100 if running else 0,
            "brush_speed": 100 if running and int(position.get("mode") or 2) == 1 else 0,
            "battery_percent": 88,
            "lat": position.get("lat"),
            "lon": position.get("lon"),
            "local_x": int(position.get("local_x") or 0),
            "local_y": int(position.get("local_y") or 0),
            "action": "modeling_task" if running or paused else "idle",
            "mission_state": "RUNNING" if running or paused else ("COMPLETE" if complete else "IDLE"),
            "control_state": "RUNNING" if running else ("PAUSED" if paused else "IDLE"),
            "simulation": True,
            "simulation_device_id": SIMULATOR_DEVICE_ID,
            "simulation_playback": playback,
        }


class ModelingSimulatorMQTTService(object):
    def __init__(self, config, controller, mqtt_client=None):
        self.config = build_simulator_mqtt_config(config)
        self.controller = controller
        if mqtt_client is None:
            try:
                from mqtt_client import MQTTClient
            except ImportError as error:
                raise ModelingSimulatorError(
                    "SIMULATOR_MQTT_DEPENDENCY_MISSING",
                    "install paho-mqtt before starting the MQTT simulator: {}".format(str(error)),
                )
            mqtt_client = MQTTClient(self.config)
        self.mqtt_client = mqtt_client
        self.command_handler = MQTTCommandHandler(controller)
        self.mqtt_client.set_message_callback(self._on_message)
        self.controller.set_position_callback(self.publish_position)
        self.status_interval = float(self.config.get("mqtt", {}).get("status_interval", 2))
        self.running = False
        self.status_thread = None

    def _publish_ack(self, message):
        self.mqtt_client.publish_status({
            "type": "ack",
            "command_id": message.get("command_id"),
            "trace_id": message.get("trace_id"),
            "command": message.get("command"),
            "status": "accepted",
            "timestamp": int(time.time()),
        })

    def _publish_result(self, message, result):
        self.mqtt_client.publish_status({
            "type": "command_result",
            "command_id": message.get("command_id"),
            "trace_id": message.get("trace_id"),
            "command": message.get("command"),
            "result": result,
            "timestamp": int(time.time()),
        })

    def _on_message(self, message):
        self._publish_ack(message)
        try:
            result = self.command_handler.handle(message)
        except Exception as error:
            result = {
                "success": False,
                "message": "simulator command exception: {}".format(str(error)),
                "data": {"code": "SIMULATOR_COMMAND_ERROR", "simulator": True},
            }
        self._publish_result(message, result)
        if result.get("success"):
            self.publish_position(self.controller.current_position())
        self.publish_status()

    def publish_status(self):
        return self.mqtt_client.publish_status({
            "type": "vehicle_status",
            "data": self.controller.status_snapshot(),
        })

    def publish_position(self, position):
        data = {
            "local_x": int(position.get("local_x") or 0),
            "local_y": int(position.get("local_y") or 0),
            "simulation": True,
        }
        for field in ("taskId", "taskIndex", "taskTotal", "mode", "areaNumber", "segmentProgress", "source"):
            if field in position:
                data[field] = position[field]
        return self.mqtt_client.publish_realtime({
            "type": "vehicle_position",
            "data": data,
        })

    def _status_loop(self):
        while self.running:
            if self.mqtt_client.ensure_connected():
                self.publish_status()
            time.sleep(self.status_interval)

    def start(self):
        self.running = True
        connected = self.mqtt_client.connect()
        self.status_thread = threading.Thread(target=self._status_loop)
        self.status_thread.daemon = True
        self.status_thread.start()
        return connected

    def stop(self):
        self.running = False
        self.controller.player.cancel()
        if self.status_thread and self.status_thread.is_alive():
            self.status_thread.join(timeout=2.0)
        self.mqtt_client.disconnect()


def load_json(path):
    with open(path, "r", encoding="utf-8") as handle:
        payload = json.load(handle)
    if not isinstance(payload, dict):
        raise ValueError("MQTT config must be a JSON object")
    return payload


def _default_run_dir(base_dir):
    name = "run_{}_{}".format(int(time.time()), os.getpid())
    return os.path.join(os.path.abspath(base_dir), name)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Run the isolated MQTT modeling simulator")
    parser.add_argument("--config", default="mqtt_config.json", help="base MQTT config file")
    parser.add_argument("--data-dir", default="modeling_simulator_data", help="simulator-only data root")
    parser.add_argument("--step-cm", type=float, default=25.0, help="playback position spacing in cm")
    parser.add_argument("--interval", type=float, default=0.2, help="playback publish interval in seconds")
    parser.add_argument(
        "--preload",
        action="store_true",
        help="preload the complete fixed scenario for query-only frontend tests",
    )
    args = parser.parse_args(argv)

    config = build_simulator_mqtt_config(load_json(args.config))
    run_dir = _default_run_dir(args.data_dir)
    controller = ModelingSimulatorController(
        run_dir,
        playback_step_cm=args.step_cm,
        playback_interval=args.interval,
    )
    if args.preload:
        preload_result = controller.preload_scenario()
        if not preload_result.get("success"):
            raise ModelingSimulatorError(
                "SIMULATOR_PRELOAD_FAILED",
                preload_result.get("message") or "simulator scenario preload failed",
            )
    service = ModelingSimulatorMQTTService(config, controller)

    print("=" * 68)
    print("MODELING SIMULATOR ONLY - NO HARDWARE CONTROL")
    print("device: {}".format(SIMULATOR_DEVICE_ID))
    print("subscribe: {}".format(config["topics"]["subscribe"]))
    print("publish: {}".format(config["topics"]["publish"]))
    print("data: {}".format(run_dir))
    print("preloaded: {}".format(bool(args.preload)))
    print("Press Ctrl+C to stop")
    print("=" * 68)

    stopped = threading.Event()

    def request_stop(signum=None, frame=None):
        stopped.set()

    try:
        signal.signal(signal.SIGINT, request_stop)
        if hasattr(signal, "SIGTERM"):
            signal.signal(signal.SIGTERM, request_stop)
    except ValueError:
        pass

    service.start()
    try:
        while not stopped.wait(0.5):
            pass
    except KeyboardInterrupt:
        pass
    finally:
        service.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
