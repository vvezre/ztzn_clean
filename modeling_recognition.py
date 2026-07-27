# coding=utf-8
import math


ASSIST_TURN_DEG = 20.0
CONNECTOR_MIN_TURN_DEG = 45.0
CONNECTOR_MAX_TURN_DEG = 135.0
CONNECTOR_MIN_LENGTH_CM = 50.0


def _number(value):
    if value is None or value == "":
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if not math.isnan(number) and not math.isinf(number) else None


def _lat_lon_to_xy_cm(origin_lat, origin_lon, lat, lon):
    mean_lat = math.radians((origin_lat + lat) / 2.0)
    x_m = math.radians(lon - origin_lon) * 6371000.0 * math.cos(mean_lat)
    y_m = math.radians(lat - origin_lat) * 6371000.0
    return x_m * 100.0, y_m * 100.0


def _points_with_xy(points):
    origin = None
    for point in points:
        lat = _number(point.get("lat"))
        lon = _number(point.get("lon"))
        if lat is not None and lon is not None:
            origin = (lat, lon)
            break

    normalized = []
    for index, point in enumerate(points, start=1):
        item = dict(point)
        item["sequence"] = int(item.get("sequence") or index)
        x = _number(item.get("x"))
        y = _number(item.get("y"))
        if (x is None or y is None) and origin is not None:
            lat = _number(item.get("lat"))
            lon = _number(item.get("lon"))
            if lat is not None and lon is not None:
                x, y = _lat_lon_to_xy_cm(origin[0], origin[1], lat, lon)
        item["x"] = round(x, 3) if x is not None else None
        item["y"] = round(y, 3) if y is not None else None
        normalized.append(item)
    return normalized


def _vector(start, end):
    if start.get("x") is None or start.get("y") is None or end.get("x") is None or end.get("y") is None:
        return None
    return end["x"] - start["x"], end["y"] - start["y"]


def _length(vector):
    if vector is None:
        return None
    return math.hypot(vector[0], vector[1])


def _signed_turn(prev_point, point, next_point):
    incoming = _vector(prev_point, point)
    outgoing = _vector(point, next_point)
    if _length(incoming) in (None, 0) or _length(outgoing) in (None, 0):
        return None
    cross = incoming[0] * outgoing[1] - incoming[1] * outgoing[0]
    dot = incoming[0] * outgoing[0] + incoming[1] * outgoing[1]
    return math.degrees(math.atan2(cross, dot))


def _turn_sign(angle):
    if angle is None or abs(angle) < 1e-6:
        return 0
    return 1 if angle > 0 else -1


def _dominant_turn_sign(turns):
    signed = [
        _turn_sign(turn)
        for turn in turns
        if turn is not None and abs(turn) >= ASSIST_TURN_DEG
    ]
    total = sum(signed)
    if total == 0:
        return 0
    return 1 if total > 0 else -1


def _connector_candidate(points, turns, dominant_sign, index):
    next_index = index + 1
    if next_index >= len(points):
        return False
    left = turns[index]
    right = turns[next_index]
    if left is None or right is None:
        return False
    if dominant_sign == 0:
        return False
    if _turn_sign(left) != -dominant_sign or _turn_sign(right) != -dominant_sign:
        return False
    if not (CONNECTOR_MIN_TURN_DEG <= abs(left) <= CONNECTOR_MAX_TURN_DEG):
        return False
    if not (CONNECTOR_MIN_TURN_DEG <= abs(right) <= CONNECTOR_MAX_TURN_DEG):
        return False
    segment_length = _length(_vector(points[index], points[next_index]))
    return segment_length is not None and segment_length >= CONNECTOR_MIN_LENGTH_CM


def _make_connector(points, index, connector_index):
    start = points[index]
    end = points[index + 1]
    return {
        "id": "c{}".format(connector_index),
        "type": "sub_area_connector",
        "label": "子区域连接段",
        "pointIds": [start.get("id"), end.get("id")],
        "startPointId": start.get("id"),
        "endPointId": end.get("id"),
        "lengthCm": round(_length(_vector(start, end)) or 0.0, 1),
        "needsConfirmation": True,
    }


def _single_sub_area(group_id, points, needs_confirmation=False):
    return [{
        "id": "{}-sa1".format(group_id),
        "name": "子区域1",
        "pointIds": [point.get("id") for point in points],
        "needsConfirmation": needs_confirmation,
    }]


def _split_by_connector_axis(group_id, points, connector):
    start_id, end_id = connector["pointIds"]
    start = next((point for point in points if point.get("id") == start_id), None)
    end = next((point for point in points if point.get("id") == end_id), None)
    axis = _vector(start, end) if start and end else None
    axis_len = _length(axis)
    if not axis or not axis_len:
        return []

    unit = (axis[0] / axis_len, axis[1] / axis_len)
    midpoint = ((start["x"] + end["x"]) / 2.0, (start["y"] + end["y"]) / 2.0)
    midpoint_projection = midpoint[0] * unit[0] + midpoint[1] * unit[1]
    left_ids = []
    right_ids = []
    for point in points:
        if point.get("id") == start_id:
            left_ids.append(point.get("id"))
            continue
        if point.get("id") == end_id:
            right_ids.append(point.get("id"))
            continue
        if point.get("x") is None or point.get("y") is None:
            continue
        projection = point["x"] * unit[0] + point["y"] * unit[1]
        if projection <= midpoint_projection:
            left_ids.append(point.get("id"))
        else:
            right_ids.append(point.get("id"))

    sub_areas = []
    for index, point_ids in enumerate((left_ids, right_ids), start=1):
        if point_ids:
            sub_areas.append({
                "id": "{}-sa{}".format(group_id, index),
                "name": "子区域{}".format(index),
                "pointIds": point_ids,
                "needsConfirmation": True,
            })
    return sub_areas


def recognize_group_points(group):
    group_id = str(group.get("id") or "group")
    points = _points_with_xy(list(group.get("points") or []))
    point_count = len(points)
    if point_count < 4:
        return {
            "groupId": group_id,
            "status": "insufficient_points",
            "message": "至少需要4个点才能识别区域",
            "needsConfirmation": True,
            "points": points,
            "subAreas": [],
            "connectors": [],
            "warnings": ["INSUFFICIENT_POINTS"],
            "summary": {
                "pointCount": point_count,
                "subAreaCount": 0,
                "connectorCount": 0,
                "assistPointCount": 0,
            },
        }

    turns = [
        _signed_turn(points[(index - 1) % point_count], points[index], points[(index + 1) % point_count])
        for index in range(point_count)
    ]
    dominant_sign = _dominant_turn_sign(turns)
    connector_indexes = [
        index for index in range(point_count - 1)
        if _connector_candidate(points, turns, dominant_sign, index)
    ]
    connectors = [
        _make_connector(points, index, connector_number)
        for connector_number, index in enumerate(connector_indexes, start=1)
    ]
    connector_point_ids = set()
    for connector in connectors:
        connector_point_ids.update(connector["pointIds"])

    assist_count = 0
    for index, point in enumerate(points):
        roles = []
        if point.get("id") in connector_point_ids:
            role = "connection_point"
            roles.append("sub_area_connector")
        elif turns[index] is not None and abs(turns[index]) < ASSIST_TURN_DEG:
            role = "boundary_assist"
            assist_count += 1
        else:
            role = "boundary_corner"
        point["role"] = role
        point["roles"] = roles
        point["turnDeg"] = round(turns[index], 1) if turns[index] is not None else None

    warnings = []
    if len(connectors) == 0:
        sub_areas = _single_sub_area(group_id, points)
        status = "recognized"
        needs_confirmation = False
        message = "已识别为单个子区域"
    elif len(connectors) == 1:
        sub_areas = _split_by_connector_axis(group_id, points, connectors[0])
        confident_split = len(sub_areas) == 2 and all(len(item["pointIds"]) >= 4 for item in sub_areas)
        if confident_split:
            for connector in connectors:
                connector["needsConfirmation"] = False
            for sub_area in sub_areas:
                sub_area["needsConfirmation"] = False
        if len(sub_areas) != 2 or any(len(item["pointIds"]) < 3 for item in sub_areas):
            warnings.append("CONNECTOR_SPLIT_NEEDS_MANUAL_CONFIRMATION")
        if confident_split:
            status = "recognized"
            needs_confirmation = False
        else:
            status = "needs_confirmation"
            needs_confirmation = True
        message = "已识别到子区域连接段，请确认子区域划分"
    else:
        sub_areas = []
        warnings.append("MULTIPLE_CONNECTORS_NEED_MANUAL_CONFIRMATION")
        status = "needs_confirmation"
        needs_confirmation = True
        message = "识别到多个连接段，请人工确认子区域"

    if len(connectors) == 1 and not needs_confirmation:
        message = "auto recognized sub areas and connector"

    return {
        "groupId": group_id,
        "status": status,
        "message": message,
        "needsConfirmation": needs_confirmation,
        "points": points,
        "subAreas": sub_areas,
        "connectors": connectors,
        "warnings": warnings,
        "summary": {
            "pointCount": point_count,
            "subAreaCount": len(sub_areas),
            "connectorCount": len(connectors),
            "assistPointCount": assist_count,
        },
    }
