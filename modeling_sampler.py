# coding=utf-8
import math
import time
import uuid


class ModelingSampleError(Exception):
    def __init__(self, code, message):
        super(ModelingSampleError, self).__init__(message)
        self.code = code
        self.message = message


def _timestamp(now=None):
    value = now() if callable(now) else (now if now is not None else time.time())
    return int(value)


def _float_or_none(value):
    if value is None or value == "":
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(number) or math.isinf(number):
        return None
    return number


def _is_rtk_fixed(snapshot):
    if snapshot.get("rtkFixAvailable") is True:
        return True
    quality = snapshot.get("rtkQuality")
    gga_age = _float_or_none(snapshot.get("rtkGgaAgeSec"))
    return str(quality) == "4" and gga_age is not None and gga_age <= 2.0


def _sample_error_message(code):
    messages = {
        "READY": "可以记录当前点",
        "RTK_SAMPLE_PROVIDER_MISSING": "采样来源未接入",
        "RTK_SAMPLE_EMPTY": "未读取到RTK状态",
        "RTK_LOCATION_MISSING": "无RTK坐标，无法记录当前点",
        "RTK_NOT_FIXED": "RTK未固定，暂不能采样",
        "VEHICLE_NOT_STATIC": "车辆未静止，禁止采样",
    }
    return messages.get(code, code)


def _assert_vehicle_static(snapshot):
    control_state = str(snapshot.get("controlState") or "").upper()
    action = str(snapshot.get("action") or "").lower()
    if control_state in ("RUNNING", "STOPPING"):
        raise ModelingSampleError("VEHICLE_NOT_STATIC", "vehicle is running")
    if snapshot.get("moving") is True:
        raise ModelingSampleError("VEHICLE_NOT_STATIC", "vehicle is moving")
    if action in ("auto_drive", "modeling_task", "point_to_point", "cleaning"):
        raise ModelingSampleError("VEHICLE_NOT_STATIC", "vehicle action is active")
    for key in ("speed", "xSpeed", "forwardSpeed"):
        speed = _float_or_none(snapshot.get(key))
        if speed is not None and abs(speed) > 0.01:
            raise ModelingSampleError("VEHICLE_NOT_STATIC", "vehicle speed is not zero")


def _distance_m(lat_a, lon_a, lat_b, lon_b):
    radius = 6371000.0
    lat1 = math.radians(lat_a)
    lat2 = math.radians(lat_b)
    d_lat = math.radians(lat_b - lat_a)
    d_lon = math.radians(lon_b - lon_a)
    a = (
        math.sin(d_lat / 2.0) * math.sin(d_lat / 2.0)
        + math.cos(lat1) * math.cos(lat2) * math.sin(d_lon / 2.0) * math.sin(d_lon / 2.0)
    )
    return radius * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


def _average_heading(headings):
    values = [_float_or_none(item) for item in headings]
    values = [item for item in values if item is not None]
    if not values:
        return None
    sin_sum = sum(math.sin(math.radians(item)) for item in values)
    cos_sum = sum(math.cos(math.radians(item)) for item in values)
    if abs(sin_sum) < 1e-12 and abs(cos_sum) < 1e-12:
        return values[-1] % 360.0
    return math.degrees(math.atan2(sin_sum, cos_sum)) % 360.0


def inspect_sample_readiness(snapshot_reader):
    try:
        if snapshot_reader is None:
            raise ModelingSampleError("RTK_SAMPLE_PROVIDER_MISSING", "sample provider is missing")
        snapshot = snapshot_reader()
        if not isinstance(snapshot, dict):
            raise ModelingSampleError("RTK_SAMPLE_EMPTY", "sample is empty")
        _assert_vehicle_static(snapshot)
        lat = _float_or_none(snapshot.get("lat"))
        lon = _float_or_none(snapshot.get("lon"))
        if lat is None or lon is None:
            raise ModelingSampleError("RTK_LOCATION_MISSING", "rtk location is missing")
        if not _is_rtk_fixed(snapshot):
            raise ModelingSampleError("RTK_NOT_FIXED", "rtk is not fixed")
        return {
            "ready": True,
            "code": "READY",
            "message": _sample_error_message("READY"),
            "lat": lat,
            "lon": lon,
            "heading": _float_or_none(snapshot.get("heading")),
            "rtkQuality": snapshot.get("rtkQuality"),
            "rtkGgaAgeSec": _float_or_none(snapshot.get("rtkGgaAgeSec")),
            "controlState": snapshot.get("controlState"),
            "action": snapshot.get("action"),
        }
    except ModelingSampleError as error:
        return {
            "ready": False,
            "code": error.code,
            "message": _sample_error_message(error.code),
        }


def sample_current_point(snapshot_reader, sample_count=10, max_radius_m=0.05, now=None, sleep_seconds=0.0):
    if snapshot_reader is None:
        raise ModelingSampleError("RTK_SAMPLE_PROVIDER_MISSING", "sample provider is missing")
    sample_total = int(sample_count or 1)
    if sample_total <= 0:
        raise ModelingSampleError("INVALID_SAMPLE_COUNT", "sample count must be positive")

    started_at = _timestamp(now)
    samples = []
    for index in range(sample_total):
        snapshot = snapshot_reader()
        if not isinstance(snapshot, dict):
            raise ModelingSampleError("RTK_SAMPLE_EMPTY", "sample is empty")
        _assert_vehicle_static(snapshot)
        lat = _float_or_none(snapshot.get("lat"))
        lon = _float_or_none(snapshot.get("lon"))
        if lat is None or lon is None:
            raise ModelingSampleError("RTK_LOCATION_MISSING", "rtk location is missing")
        if not _is_rtk_fixed(snapshot):
            raise ModelingSampleError("RTK_NOT_FIXED", "rtk is not fixed")
        samples.append(dict(snapshot, lat=lat, lon=lon))
        if sleep_seconds and index < sample_total - 1:
            time.sleep(float(sleep_seconds))

    mean_lat = sum(item["lat"] for item in samples) / len(samples)
    mean_lon = sum(item["lon"] for item in samples) / len(samples)
    radius_m = max(_distance_m(mean_lat, mean_lon, item["lat"], item["lon"]) for item in samples)
    if radius_m > float(max_radius_m):
        raise ModelingSampleError("RTK_SAMPLE_UNSTABLE", "rtk sample is unstable")

    last = samples[-1]
    return {
        "id": "p" + uuid.uuid4().hex[:11],
        "role": "unknown",
        "roles": [],
        "lat": mean_lat,
        "lon": mean_lon,
        "x": None,
        "y": None,
        "heading": _average_heading([item.get("heading") for item in samples]),
        "source": "rtk_mean",
        "sample": {
            "count": len(samples),
            "startedAt": started_at,
            "endedAt": _timestamp(now),
            "radiusM": round(radius_m, 4),
            "quality": last.get("rtkQuality"),
            "ggaAgeSec": _float_or_none(last.get("rtkGgaAgeSec")),
        },
    }
