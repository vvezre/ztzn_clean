# coding=utf-8

import json
import math
import time


DEFAULT_TRACE_LIMIT = 120
CURRENT_LOCATION_MAX_AGE_SECONDS = 3.0


def _decode(value):
    if isinstance(value, bytes):
        try:
            return value.decode("utf-8")
        except Exception:
            return value.decode("utf-8", "replace")
    return value


def _to_float(value, default=None):
    value = _decode(value)
    if value in (None, ""):
        return default
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    if math.isnan(result) or math.isinf(result):
        return default
    return result


def _to_int(value, default=0):
    value = _decode(value)
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


def _to_bool(value, default=False):
    value = _decode(value)
    if value in (None, ""):
        return default
    return str(value).strip().lower() in ("1", "true", "yes", "on", "working")


def _get_value(redis_client, key, default=None):
    try:
        value = redis_client.get(key)
    except Exception:
        return default
    return default if value is None else _decode(value)


def _get_hash(redis_client, key):
    try:
        raw = redis_client.hgetall(key)
    except Exception:
        return {}
    result = {}
    for item_key, item_value in (raw or {}).items():
        result[_decode(item_key)] = _decode(item_value)
    return result


def _parse_json(value, default=None):
    value = _decode(value)
    if not value:
        return default
    try:
        return json.loads(value)
    except Exception:
        return default


def _list_first(redis_client, key):
    try:
        value = redis_client.lindex(key, 0)
    except Exception:
        return None
    return _parse_json(value, None)


def _active_segment(redis_client, config_path):
    task = _list_first(redis_client, "taskList")
    source = "redis"
    if not isinstance(task, dict):
        return None, None

    start_lat = _to_float(task.get("startLat"))
    start_lon = _to_float(task.get("startLon"))
    end_lat = _to_float(task.get("endLat"))
    end_lon = _to_float(task.get("endLon"))
    heading = _to_float(task.get("heading"))
    if None in (start_lat, start_lon, end_lat, end_lon, heading):
        return None, None

    return {
        "id": task.get("id"),
        "startLat": start_lat,
        "startLon": start_lon,
        "endLat": end_lat,
        "endLon": end_lon,
        "targetHeading": heading,
        "source": source,
    }, source


def _empty_vehicle():
    return {
        "lat": None,
        "lon": None,
        "heading": None,
        "headingAt": None,
    }


def _current_vehicle(redis_client, now=None):
    current = _get_hash(redis_client, "currentLocation")
    heading_at = _to_float(current.get("headingAt"))
    if heading_at is None:
        return _empty_vehicle()
    if now is not None and now - heading_at > CURRENT_LOCATION_MAX_AGE_SECONDS:
        return _empty_vehicle()
    return {
        "lat": _to_float(current.get("lat")),
        "lon": _to_float(current.get("lon")),
        "heading": _to_float(current.get("heading")),
        "headingAt": heading_at,
    }


def _runtime_detail(redis_client):
    detail = _parse_json(_get_value(redis_client, "runtimeDetail", "{}"), {})
    return detail if isinstance(detail, dict) else {}


def _rtk_fixed(detail):
    if detail.get("rtkFixAvailable") is not None:
        return bool(detail.get("rtkFixAvailable"))
    state = detail.get("rtkFixState")
    if state:
        return str(state).upper() == "FIXED"
    quality = detail.get("rtkQuality")
    return str(quality) == "4" if quality is not None else None


def _empty_correction(global_go):
    return {
        "headingError": None,
        "cte": None,
        "zSpeed": None,
        "distanceToTarget": None,
        "signedRemaining": None,
        "globalGo": global_go,
        "source": "unreported",
    }


def _runtime_correction_debug(redis_client, global_go):
    debug = _get_hash(redis_client, "correctionDebug")
    if not debug:
        return None
    return {
        "headingError": _to_float(debug.get("headingError")),
        "cte": _to_float(debug.get("cte")),
        "zSpeed": _to_int(debug.get("zSpeed"), None),
        "distanceToTarget": _to_float(debug.get("distanceToTarget")),
        "signedRemaining": _to_float(debug.get("signedRemaining")),
        "globalGo": global_go,
        "source": "runtime",
    }


def _runtime_parameters(redis_client):
    debug = _get_hash(redis_client, "correctionDebug")
    if not debug:
        return {
            "headingGain": None,
            "cteGain": None,
            "cteDotGain": None,
            "source": "unreported",
        }
    return {
        "headingGain": _to_float(debug.get("headingGain")),
        "cteGain": _to_float(debug.get("cteGain")),
        "cteDotGain": _to_float(debug.get("cteDotGain")),
        "source": "runtime",
    }


def _append_trace(trace, vehicle, now, trace_limit):
    if trace is None:
        trace = []
    if vehicle.get("lat") is None or vehicle.get("lon") is None:
        return trace[-trace_limit:]
    trace.append({
        "lat": vehicle.get("lat"),
        "lon": vehicle.get("lon"),
        "heading": vehicle.get("heading"),
        "timestamp": now,
    })
    del trace[:-trace_limit]
    return list(trace)


def _build_correction(redis_client, segment, vehicle, global_go, parameters):
    runtime_debug = _runtime_correction_debug(redis_client, global_go)
    if runtime_debug is not None:
        return runtime_debug
    return _empty_correction(global_go)


def build_correction_state(redis_client, config_path=None, trace=None,
                           trace_limit=DEFAULT_TRACE_LIMIT, now=None):
    now = time.time() if now is None else now
    trace_limit = max(1, int(trace_limit or DEFAULT_TRACE_LIMIT))
    segment, segment_source = _active_segment(redis_client, config_path)
    vehicle = _current_vehicle(redis_client, now=now)
    mission = _get_value(redis_client, "mission", "")
    parking = _to_bool(_get_value(redis_client, "parking"), False)
    global_go_raw = _get_value(redis_client, "globalGo", None)
    global_go = None if global_go_raw is None else _to_bool(global_go_raw)
    detail = _runtime_detail(redis_client)
    parameters = _runtime_parameters(redis_client)

    status = {
        "mission": mission,
        "parking": parking,
        "rtkFixed": _rtk_fixed(detail),
        "edgeDistanceToTargetM": _to_float(_get_value(redis_client, "edgeDistanceToTargetM")),
        "reason": None if segment else "no_active_segment",
        "segmentSource": segment_source,
        "globalGoSource": "redis" if global_go_raw is not None else "not_reported",
    }

    public_segment = None
    if segment:
        public_segment = dict(segment)
        public_segment.pop("source", None)

    return {
        "segment": public_segment,
        "vehicle": vehicle,
        "correction": _build_correction(redis_client, segment, vehicle, global_go, parameters),
        "parameters": parameters,
        "status": status,
        "trace": _append_trace(trace, vehicle, now, trace_limit),
        "timestamp": now,
    }
