# coding=utf-8
"""与硬件无关的路线起点距离校验。"""


class RouteStartGuardError(Exception):
    def __init__(self, code, message):
        super(RouteStartGuardError, self).__init__(message)
        self.code = code
        self.message = message


def _number(value):
    if value is None or value == "":
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def validate_route_start(
        current_lat,
        current_lon,
        origin_lat,
        origin_lon,
        tolerance_m,
        calculate_distance):
    """校验实时位置是否位于路线第一段起点允许范围内。"""
    current_lat = _number(current_lat)
    current_lon = _number(current_lon)
    origin_lat = _number(origin_lat)
    origin_lon = _number(origin_lon)
    tolerance_m = _number(tolerance_m)
    if current_lat is None or current_lon is None:
        raise RouteStartGuardError("RTK_NOT_READY", "current RTK position is unavailable")
    if origin_lat is None or origin_lon is None:
        raise RouteStartGuardError("TASK_ORIGIN_UNKNOWN", "route origin is unavailable")
    if tolerance_m is None or tolerance_m < 0:
        raise RouteStartGuardError("TASK_ORIGIN_TOLERANCE_INVALID", "route origin tolerance is invalid")
    if not callable(calculate_distance):
        raise RouteStartGuardError("DISTANCE_CALCULATOR_MISSING", "distance calculator is required")

    measured = calculate_distance(current_lat, current_lon, origin_lat, origin_lon)
    distance_m = measured[0] if isinstance(measured, (list, tuple)) else measured
    distance_m = _number(distance_m)
    if distance_m is None:
        raise RouteStartGuardError("TASK_ORIGIN_UNKNOWN", "route origin distance is unavailable")
    if distance_m > tolerance_m:
        raise RouteStartGuardError(
            "NOT_AT_TASK_ORIGIN",
            "current position is {:.3f}m from route origin, tolerance is {:.3f}m".format(
                distance_m,
                tolerance_m,
            ),
        )
    return {
        "distanceToTaskOriginM": round(distance_m, 3),
        "taskOriginToleranceM": tolerance_m,
        "isAtTaskOrigin": True,
    }
