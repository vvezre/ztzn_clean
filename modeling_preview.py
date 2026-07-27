# coding=utf-8
import math
import time


BRUSH_WIDTH_CM = 116.0
DEFAULT_OVERLAP_CM = 53.0
MIN_OVERLAP_CM = 10.0
EPSILON = 1e-6


class ModelingPreviewError(Exception):
    pass


def _number(value):
    if value is None or value == "":
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if not math.isnan(number) and not math.isinf(number) else None


def _round(value, digits=3):
    number = _number(value)
    return round(number, digits) if number is not None else None


def _normalize_heading(angle):
    number = _number(angle)
    if number is None:
        return None
    return round(number % 360.0, 1)


def _point_xy(point):
    x = _number(point.get("x"))
    y = _number(point.get("y"))
    if x is None or y is None:
        return None
    return x, y


def _distance(start, end):
    return math.hypot(end[0] - start[0], end[1] - start[1])


def _heading_from_points(start, end):
    start_xy = _point_xy(start)
    end_xy = _point_xy(end)
    if start_xy is None or end_xy is None:
        return None
    dx = end_xy[0] - start_xy[0]
    dy = end_xy[1] - start_xy[1]
    if abs(dx) < EPSILON and abs(dy) < EPSILON:
        return None
    return _normalize_heading(math.degrees(math.atan2(dx, dy)))


def _default_sweep_angle(points):
    if len(points) < 2:
        return None
    first_edge = _heading_from_points(points[0], points[1])
    if first_edge is None:
        return None
    return _normalize_heading(first_edge + 90.0)


def _group_sweep_angle(group, polygon):
    direction = str(group.get("sweepDirection") or "auto")
    manual_angle = _normalize_heading(group.get("sweepAngle"))
    if direction == "manual" and manual_angle is not None:
        return manual_angle
    return _default_sweep_angle(polygon)


def _preview_point(point):
    return {
        "id": point.get("id"),
        "sequence": point.get("sequence"),
        "x": _round(point.get("x")),
        "y": _round(point.get("y")),
        "lat": _round(point.get("lat"), 8),
        "lon": _round(point.get("lon"), 8),
        "role": point.get("role"),
        "roles": list(point.get("roles") or []),
    }


def _clean_polygon_points(points):
    cleaned = []
    for point in points:
        xy = _point_xy(point)
        if xy is None:
            continue
        cleaned.append(point)
    return cleaned


def _unique_intersections(items):
    unique = []
    for item in items:
        if any(abs(item[0] - existing[0]) < EPSILON and abs(item[1] - existing[1]) < EPSILON for existing in unique):
            continue
        unique.append(item)
    return unique


def _line_polygon_intersections(polygon_xy, normal, offset):
    intersections = []
    count = len(polygon_xy)
    for index in range(count):
        start = polygon_xy[index]
        end = polygon_xy[(index + 1) % count]
        start_value = normal[0] * start[0] + normal[1] * start[1] - offset
        end_value = normal[0] * end[0] + normal[1] * end[1] - offset
        if abs(start_value) < EPSILON:
            intersections.append(start)
        denominator = start_value - end_value
        if abs(denominator) < EPSILON:
            continue
        ratio = start_value / denominator
        if -EPSILON <= ratio <= 1.0 + EPSILON:
            x = start[0] + (end[0] - start[0]) * ratio
            y = start[1] + (end[1] - start[1]) * ratio
            intersections.append((x, y))
    return _unique_intersections(intersections)


def _nearest_even_lane_count(span, target_spacing):
    if span <= EPSILON:
        return 0
    ideal = span / max(float(target_spacing), 1.0) + 1.0
    lower = max(2, int(math.floor(ideal / 2.0)) * 2)
    candidates = sorted(set([max(2, lower - 2), lower, lower + 2, lower + 4]))
    return min(
        candidates,
        key=lambda count: (
            abs(span / float(count - 1) - target_spacing),
            abs(count - ideal),
            count,
        ),
    )


def _generate_lanes(polygon, sweep_angle, lane_spacing_cm, force_even=False):
    polygon_xy = [_point_xy(point) for point in polygon]
    polygon_xy = [xy for xy in polygon_xy if xy is not None]
    if len(polygon_xy) < 3 or sweep_angle is None:
        return []

    radians = math.radians(sweep_angle)
    direction = (math.sin(radians), math.cos(radians))
    normal = (math.cos(radians), -math.sin(radians))
    offsets = [normal[0] * x + normal[1] * y for x, y in polygon_xy]
    min_offset = min(offsets)
    max_offset = max(offsets)
    spacing = max(float(lane_spacing_cm), 1.0)

    span = max_offset - min_offset
    if force_even and span > EPSILON:
        lane_count = _nearest_even_lane_count(span, spacing)
        actual_spacing = span / float(lane_count - 1)
        lane_offsets = [min_offset + actual_spacing * index for index in range(lane_count)]
    else:
        actual_spacing = spacing
        lane_offsets = []
        offset = min_offset
        while offset <= max_offset + EPSILON:
            lane_offsets.append(offset)
            offset += spacing

    lanes = []
    index = 1
    for offset in lane_offsets:
        intersections = _line_polygon_intersections(polygon_xy, normal, offset)
        if len(intersections) >= 2:
            ordered = sorted(intersections, key=lambda xy: direction[0] * xy[0] + direction[1] * xy[1])
            start = ordered[0]
            end = ordered[-1]
            if _distance(start, end) > EPSILON:
                lanes.append({
                    "id": "lane-{}".format(index),
                    "heading": sweep_angle,
                    "startX": round(start[0], 1),
                    "startY": round(start[1], 1),
                    "endX": round(end[0], 1),
                    "endY": round(end[1], 1),
                    "lengthCm": round(_distance(start, end), 1),
                    "laneSpacingCm": round(actual_spacing, 1),
                })
                index += 1

    if lanes:
        return lanes

    ordered = sorted(polygon_xy, key=lambda xy: direction[0] * xy[0] + direction[1] * xy[1])
    start = ordered[0]
    end = ordered[-1]
    if _distance(start, end) <= EPSILON:
        return []
    return [{
        "id": "lane-1",
        "heading": sweep_angle,
        "startX": round(start[0], 1),
        "startY": round(start[1], 1),
        "endX": round(end[0], 1),
        "endY": round(end[1], 1),
        "lengthCm": round(_distance(start, end), 1),
    }]


def _build_group_link_preview(link):
    points = list(link.get("points") or [])
    start_point = _preview_point(points[0]) if len(points) >= 1 else None
    end_point = _preview_point(points[1]) if len(points) >= 2 else None
    length_cm = None
    if len(points) >= 2 and _point_xy(points[0]) is not None and _point_xy(points[1]) is not None:
        length_cm = round(_distance(_point_xy(points[0]), _point_xy(points[1])), 1)
    return {
        "id": link.get("id"),
        "name": link.get("name"),
        "type": link.get("type") or "group_connector",
        "startGroupId": link.get("startGroupId"),
        "endGroupId": link.get("endGroupId"),
        "status": link.get("status") or "draft",
        "startPoint": start_point,
        "endPoint": end_point,
        "lengthCm": length_cm,
    }


def build_model_preview(draft, now=None, brush_width_cm=BRUSH_WIDTH_CM, overlap_cm=DEFAULT_OVERLAP_CM):
    if not isinstance(draft, dict):
        raise ModelingPreviewError("model draft is required")
    recognition = draft.get("recognition") or {}
    if not isinstance(recognition, dict) or not recognition.get("confirmed"):
        raise ModelingPreviewError("区域识别结果未确认，不能生成路径预览")

    brush_width = float(brush_width_cm)
    overlap = max(float(overlap_cm), MIN_OVERLAP_CM)
    lane_spacing = max(brush_width - overlap, 1.0)

    group_previews = []
    sub_area_count = 0
    connector_count = 0
    lane_count = 0
    warnings = []
    route_policy = draft.get("routePolicy") or {}
    force_even_lanes = bool(route_policy.get("forceEvenLanes"))

    for group in draft.get("groups") or []:
        points = list(group.get("points") or [])
        points_by_id = {point.get("id"): point for point in points}
        sub_area_previews = []
        for sub_area in group.get("subAreas") or []:
            polygon = [
                points_by_id[point_id]
                for point_id in (sub_area.get("pointIds") or [])
                if point_id in points_by_id
            ]
            polygon = _clean_polygon_points(polygon)
            sweep_angle = _group_sweep_angle(group, polygon)
            lanes = _generate_lanes(
                polygon,
                sweep_angle,
                lane_spacing,
                force_even=force_even_lanes,
            )
            sub_area_count += 1
            lane_count += len(lanes)
            if len(polygon) < 3:
                warnings.append("SUB_AREA_POLYGON_INCOMPLETE:{}".format(sub_area.get("id")))
            sub_area_previews.append({
                "id": sub_area.get("id"),
                "name": sub_area.get("name"),
                "pointIds": [point.get("id") for point in polygon],
                "sweepAngle": sweep_angle,
                "polygon": [_preview_point(point) for point in polygon],
                "lanes": lanes,
                "laneCount": len(lanes),
                "laneSpacingCm": lanes[0].get("laneSpacingCm") if lanes else None,
            })
        connectors = list(group.get("connectors") or [])
        connector_count += len(connectors)
        group_previews.append({
            "groupId": group.get("id"),
            "groupName": group.get("name"),
            "areaNumber": group.get("areaNumber"),
            "sweepDirection": group.get("sweepDirection") or "auto",
            "sweepAngle": _normalize_heading(group.get("sweepAngle")),
            "subAreas": sub_area_previews,
            "connectors": connectors,
        })

    group_links = [
        _build_group_link_preview(link)
        for link in (draft.get("groupLinks") or [])
    ]

    status = "ready" if sub_area_count > 0 and lane_count > 0 else "empty"
    return {
        "status": status,
        "generatedAt": int(now if now is not None else time.time()),
        "config": {
            "brushWidthCm": round(brush_width, 1),
            "overlapCm": round(overlap, 1),
            "laneSpacingCm": round(lane_spacing, 1),
        },
        "summary": {
            "groupCount": len(draft.get("groups") or []),
            "subAreaCount": sub_area_count,
            "connectorCount": connector_count,
            "groupLinkCount": len(group_links),
            "laneCount": lane_count,
        },
        "groups": group_previews,
        "groupLinks": group_links,
        "warnings": warnings,
    }
