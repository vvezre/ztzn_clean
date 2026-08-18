# coding=utf-8
import math


EARTH_RADIUS_M = 6371000.0
EPSILON_CM = 1e-6


class MixedCaptureError(Exception):
    def __init__(self, code, message):
        super(MixedCaptureError, self).__init__(message)
        self.code = code
        self.message = message


def _number(value):
    if value is None or value == "":
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if not math.isnan(result) and not math.isinf(result) else None


def _local_xy(point, origin):
    x = _number(point.get("x"))
    y = _number(point.get("y"))
    origin_x = _number(origin.get("x"))
    origin_y = _number(origin.get("y"))
    if x is not None and y is not None and origin_x is not None and origin_y is not None:
        return x - origin_x, y - origin_y

    lat = _number(point.get("lat"))
    lon = _number(point.get("lon"))
    origin_lat = _number(origin.get("lat"))
    origin_lon = _number(origin.get("lon"))
    if lat is None or lon is None or origin_lat is None or origin_lon is None:
        raise MixedCaptureError(
            "MODELING_POINT_COORDINATE_MISSING",
            "recorded point is missing usable coordinates",
        )
    mean_lat = math.radians((origin_lat + lat) / 2.0)
    x_m = math.radians(lon - origin_lon) * EARTH_RADIUS_M * math.cos(mean_lat)
    y_m = math.radians(lat - origin_lat) * EARTH_RADIUS_M
    return x_m * 100.0, y_m * 100.0


def _point_map(draft):
    result = {}
    for group in draft.get("groups") or []:
        for point in group.get("points") or []:
            if point.get("id"):
                result[point["id"]] = dict(point)
    for link in draft.get("groupLinks") or []:
        for point in link.get("points") or []:
            if point.get("id"):
                result[point["id"]] = dict(point)
    return result


def _ordered_event_ids(events, point_type=None):
    return [
        event.get("pointId")
        for event in events
        if event.get("pointId") and (point_type is None or event.get("pointType") == point_type)
    ]


def _set_group_points(group, point_ids, points):
    normalized = []
    for sequence, point_id in enumerate(point_ids, start=1):
        point = dict(points[point_id])
        point["sequence"] = sequence
        point.setdefault("role", "unknown")
        point.setdefault("roles", [])
        normalized.append(point)
    group = dict(group)
    group["points"] = normalized
    group["subAreas"] = []
    group["connectors"] = []
    return group


def resolve_mixed_capture(draft):
    """Turn one chronological mixed capture into two normal groups and one link."""
    if not isinstance(draft, dict):
        raise MixedCaptureError("MODELING_CAPTURE_INVALID", "model draft is required")

    events = list(draft.get("captureSequence") or [])
    area_ids = _ordered_event_ids(events, "area")
    link_event_ids = _ordered_event_ids(events, "link")
    links = list(draft.get("groupLinks") or [])
    groups = list(draft.get("groups") or [])
    if len(links) != 1 or len(groups) < 2:
        raise MixedCaptureError(
            "MODELING_LINK_INVALID",
            "mixed modeling requires exactly one completed connection",
        )
    link = dict(links[0])
    link_points = list(link.get("points") or [])
    if len(link_points) < 2 or len(link_event_ids) != len(link_points):
        raise MixedCaptureError(
            "MODELING_LINK_INCOMPLETE",
            "the connection requires at least its original start and end points",
        )
    if len(area_ids) < 8:
        raise MixedCaptureError(
            "MODELING_AREA_INCOMPLETE",
            "the mixed capture does not contain enough area points",
        )

    points = _point_map(draft)
    missing_ids = [point_id for point_id in area_ids + link_event_ids if point_id not in points]
    if missing_ids:
        raise MixedCaptureError(
            "MODELING_CAPTURE_INVALID",
            "recorded point references are incomplete",
        )

    origin = points[area_ids[0]]
    for point_id in area_ids + link_event_ids:
        x, y = _local_xy(points[point_id], origin)
        points[point_id]["x"] = round(x, 3)
        points[point_id]["y"] = round(y, 3)

    link_start = points[link_event_ids[0]]
    link_end = points[link_event_ids[-1]]
    dx = link_end["x"] - link_start["x"]
    dy = link_end["y"] - link_start["y"]
    length = math.hypot(dx, dy)
    if length <= EPSILON_CM:
        raise MixedCaptureError(
            "MODELING_LINK_INVALID",
            "connection start and end points must be different",
        )
    unit_x = dx / length
    unit_y = dy / length
    midpoint_projection = (
        ((link_start["x"] + link_end["x"]) / 2.0) * unit_x
        + ((link_start["y"] + link_end["y"]) / 2.0) * unit_y
    )

    def side(point_id):
        point = points[point_id]
        projection = point["x"] * unit_x + point["y"] * unit_y
        return projection - midpoint_projection

    origin_side = side(area_ids[0])
    if abs(origin_side) <= EPSILON_CM:
        raise MixedCaptureError(
            "MODELING_AREA_SPLIT_FAILED",
            "origin cannot lie on the connection split axis",
        )
    home_ids = []
    remote_ids = []
    for point_id in area_ids:
        if side(point_id) * origin_side >= 0:
            home_ids.append(point_id)
        else:
            remote_ids.append(point_id)
    if len(home_ids) < 4 or len(remote_ids) < 4:
        raise MixedCaptureError(
            "MODELING_AREA_INCOMPLETE",
            "each resolved modeling area requires at least four points",
        )

    start_group_id = link.get("startGroupId")
    end_group_id = link.get("endGroupId")
    group_by_id = {group.get("id"): dict(group) for group in groups}
    if start_group_id not in group_by_id or end_group_id not in group_by_id:
        raise MixedCaptureError(
            "MODELING_LINK_INVALID",
            "connection group references are invalid",
        )
    home_group = _set_group_points(group_by_id[start_group_id], home_ids, points)
    remote_group = _set_group_points(group_by_id[end_group_id], remote_ids, points)

    link["points"] = [dict(points[point_id]) for point_id in link_event_ids]
    point_count = len(link["points"])
    for sequence, point in enumerate(link["points"], start=1):
        point["sequence"] = sequence
        if sequence == 1:
            point["role"] = "group_link_start"
        elif sequence == point_count:
            point["role"] = "group_link_end"
        else:
            point["role"] = "group_link_waypoint"
        point["roles"] = ["group_connector"]
    link["status"] = "ready"

    first_link_index = next(
        index for index, event in enumerate(events) if event.get("pointType") == "link"
    )
    last_link_index = max(
        index for index, event in enumerate(events) if event.get("pointType") == "link"
    )
    before_link_home_ids = [
        event.get("pointId")
        for event in events[:first_link_index]
        if event.get("pointType") == "area" and event.get("pointId") in home_ids
    ]
    after_link_remote_ids = [
        event.get("pointId")
        for event in events[last_link_index + 1:]
        if event.get("pointType") == "area" and event.get("pointId") in remote_ids
    ]
    if len(before_link_home_ids) < 2 or len(after_link_remote_ids) < 2:
        raise MixedCaptureError(
            "MODELING_CAPTURE_ORDER_INVALID",
            "record the home left edge, connection, and remote left edge in order",
        )

    origin_id = before_link_home_ids[0]
    home_entry_id = before_link_home_ids[-1]
    remote_exit_id = after_link_remote_ids[0]
    remote_entry_id = after_link_remote_ids[1]
    outbound_ids = before_link_home_ids + link_event_ids + [remote_exit_id, remote_entry_id]
    return_ids = [remote_exit_id] + list(reversed(link_event_ids)) + [home_entry_id]

    next_groups = []
    for group in groups:
        if group.get("id") == start_group_id:
            next_groups.append(home_group)
        elif group.get("id") == end_group_id:
            next_groups.append(remote_group)
        else:
            next_groups.append(group)

    resolved = dict(draft)
    resolved["groups"] = next_groups
    resolved["groupLinks"] = [link]
    resolved["routePolicy"] = {
        "type": "bridge_round_trip",
        "forceEvenLanes": True,
        "originPointId": origin_id,
        "homeGroupId": start_group_id,
        "remoteGroupId": end_group_id,
        "linkId": link.get("id"),
        "outboundPointIds": outbound_ids,
        "returnPointIds": return_ids,
        "groupAnchors": {
            end_group_id: {
                "entryPointId": remote_entry_id,
                "exitPointId": remote_exit_id,
            },
            start_group_id: {
                "entryPointId": home_entry_id,
                "exitPointId": origin_id,
            },
        },
    }
    resolved["captureResolved"] = True
    resolved["recognition"] = {"confirmed": False, "items": []}
    resolved["taskPreview"] = None
    resolved["taskPlan"] = None
    return resolved
