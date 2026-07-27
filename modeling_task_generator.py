# coding=utf-8
import math
import time


EARTH_RADIUS_M = 6371000.0
EPSILON_CM = 1e-6


class ModelingTaskGenerationError(Exception):
    pass


def _number(value):
    if value is None or value == "":
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if not math.isnan(number) and not math.isinf(number) else None


def _round_int(value):
    return int(round(float(value)))


def _normalize_heading(value):
    number = _number(value)
    if number is None:
        return None
    return round(number % 360.0, 1)


def _heading_from_xy(start, end):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    if abs(dx) < EPSILON_CM and abs(dy) < EPSILON_CM:
        return None
    return _normalize_heading(math.degrees(math.atan2(dx, dy)))


def _length_cm(start, end):
    return math.hypot(end[0] - start[0], end[1] - start[1])


class _CoordinateMapper(object):
    def __init__(self, draft):
        self.origin = self._find_origin(draft)
        if self.origin is None:
            raise ModelingTaskGenerationError("缺少带经纬度和相对坐标的建模点，不能生成执行任务")

    def _find_origin(self, draft):
        for group in draft.get("groups") or []:
            for point in group.get("points") or []:
                x = _number(point.get("x"))
                y = _number(point.get("y"))
                lat = _number(point.get("lat"))
                lon = _number(point.get("lon"))
                if x is not None and y is not None and lat is not None and lon is not None:
                    return {
                        "x": x,
                        "y": y,
                        "lat": lat,
                        "lon": lon,
                    }
        return None

    def xy_to_lat_lon(self, x, y):
        origin = self.origin
        dx_m = (float(x) - origin["x"]) / 100.0
        dy_m = (float(y) - origin["y"]) / 100.0
        lat = origin["lat"] + math.degrees(dy_m / EARTH_RADIUS_M)
        mean_lat = math.radians((origin["lat"] + lat) / 2.0)
        cos_lat = math.cos(mean_lat)
        if abs(cos_lat) < 1e-12:
            raise ModelingTaskGenerationError("当前纬度无法换算经度")
        lon = origin["lon"] + math.degrees(dx_m / (EARTH_RADIUS_M * cos_lat))
        return round(lat, 8), round(lon, 8)

    def point_to_xy(self, point):
        if not isinstance(point, dict):
            return None
        x = _number(point.get("x"))
        y = _number(point.get("y"))
        if x is not None and y is not None:
            return x, y
        lat = _number(point.get("lat"))
        lon = _number(point.get("lon"))
        if lat is None or lon is None:
            return None
        origin = self.origin
        mean_lat = math.radians((origin["lat"] + lat) / 2.0)
        x_m = math.radians(lon - origin["lon"]) * EARTH_RADIUS_M * math.cos(mean_lat)
        y_m = math.radians(lat - origin["lat"]) * EARTH_RADIUS_M
        return origin["x"] + x_m * 100.0, origin["y"] + y_m * 100.0


def _is_same_point(left, right):
    return abs(left[0] - right[0]) < EPSILON_CM and abs(left[1] - right[1]) < EPSILON_CM


def _segment_task(start, end, mode, area_number, task_id, mapper, source):
    length = _length_cm(start, end)
    if length <= EPSILON_CM:
        return None
    heading = _heading_from_xy(start, end)
    start_lat, start_lon = mapper.xy_to_lat_lon(start[0], start[1])
    end_lat, end_lon = mapper.xy_to_lat_lon(end[0], end[1])
    return {
        "id": task_id,
        "mode": int(mode),
        "areaNumber": int(area_number or 1),
        "startX": _round_int(start[0]),
        "startY": _round_int(start[1]),
        "endX": _round_int(end[0]),
        "endY": _round_int(end[1]),
        "startLat": start_lat,
        "startLon": start_lon,
        "endLat": end_lat,
        "endLon": end_lon,
        "heading": heading,
        "angle": heading,
        "length": _round_int(length),
        "turn_back_len": 0,
        "back_len": 0,
        "source": source,
    }


def _lane_points(lane, reverse=False):
    start = (_number(lane.get("startX")), _number(lane.get("startY")))
    end = (_number(lane.get("endX")), _number(lane.get("endY")))
    if None in start or None in end:
        return None
    return (end, start) if reverse else (start, end)


def _iter_clean_segments(preview):
    for group in preview.get("groups") or []:
        group_id = group.get("groupId")
        area_number = group.get("areaNumber") or 1
        lane_index = 0
        for sub_area in group.get("subAreas") or []:
            for lane in sub_area.get("lanes") or []:
                points = _lane_points(lane, reverse=lane_index % 2 == 1)
                lane_index += 1
                if points is None:
                    continue
                yield {
                    "groupId": group_id,
                    "areaNumber": area_number,
                    "start": points[0],
                    "end": points[1],
                    "sourceId": lane.get("id"),
                }


def _link_points_between(preview, from_group_id, to_group_id, mapper):
    if not from_group_id or not to_group_id or from_group_id == to_group_id:
        return None
    for link in preview.get("groupLinks") or []:
        start_group_id = link.get("startGroupId")
        end_group_id = link.get("endGroupId")
        start_point = mapper.point_to_xy(link.get("startPoint"))
        end_point = mapper.point_to_xy(link.get("endPoint"))
        if start_point is None or end_point is None:
            continue
        if start_group_id == from_group_id and end_group_id == to_group_id:
            return [start_point, end_point]
        if start_group_id == to_group_id and end_group_id == from_group_id:
            return [end_point, start_point]
    return None


def _append_transition_tasks(tasks, current, target, area_number, mapper, task_id, preview, from_group_id, to_group_id):
    route_points = _link_points_between(preview, from_group_id, to_group_id, mapper) or []
    path_points = [current] + route_points + [target]
    for index in range(len(path_points) - 1):
        start = path_points[index]
        end = path_points[index + 1]
        task = _segment_task(start, end, 2, area_number, task_id, mapper, "modeling_transfer")
        if task is not None:
            tasks.append(task)
            task_id += 1
    return task_id


def _preview_group(preview, group_id):
    return next(
        (group for group in (preview.get("groups") or []) if group.get("groupId") == group_id),
        None,
    )


def _draft_point_map(draft):
    result = {}
    for group in draft.get("groups") or []:
        for point in group.get("points") or []:
            if point.get("id"):
                result[point["id"]] = point
    for link in draft.get("groupLinks") or []:
        for point in link.get("points") or []:
            if point.get("id"):
                result[point["id"]] = point
    return result


def _group_lane_segments(preview, group_id, entry, exit_point):
    group = _preview_group(preview, group_id)
    if group is None:
        raise ModelingTaskGenerationError("route policy references an unknown modeling group")
    lanes = []
    for sub_area in group.get("subAreas") or []:
        lanes.extend(sub_area.get("lanes") or [])
    if not lanes:
        raise ModelingTaskGenerationError("route policy group has no cleaning lanes")

    candidates = []
    for reverse_order in (False, True):
        ordered = list(reversed(lanes)) if reverse_order else list(lanes)
        for reverse_first in (False, True):
            segments = []
            for index, lane in enumerate(ordered):
                points = _lane_points(lane, reverse=bool(index % 2) ^ reverse_first)
                if points is None:
                    continue
                segments.append({
                    "groupId": group_id,
                    "areaNumber": group.get("areaNumber") or 1,
                    "start": points[0],
                    "end": points[1],
                    "sourceId": lane.get("id"),
                })
            if not segments:
                continue
            score = _length_cm(entry, segments[0]["start"])
            score += _length_cm(segments[-1]["end"], exit_point)
            for index in range(len(segments) - 1):
                score += _length_cm(segments[index]["end"], segments[index + 1]["start"])
            candidates.append((score, reverse_order, reverse_first, segments))
    if not candidates:
        raise ModelingTaskGenerationError("route policy group has no usable cleaning lanes")
    return min(candidates, key=lambda item: (item[0], item[1], item[2]))[3]


def _append_xy_path(tasks, path_points, area_number, mapper, task_id, source):
    for index in range(len(path_points) - 1):
        task = _segment_task(
            path_points[index],
            path_points[index + 1],
            2,
            area_number,
            task_id,
            mapper,
            source,
        )
        if task is not None:
            tasks.append(task)
            task_id += 1
    return task_id


def _point_ids_to_xy(point_ids, point_map, mapper):
    result = []
    for point_id in point_ids or []:
        point = point_map.get(point_id)
        xy = mapper.point_to_xy(point)
        if xy is None:
            raise ModelingTaskGenerationError("route policy point coordinates are missing")
        if not result or not _is_same_point(result[-1], xy):
            result.append(xy)
    return result


def _append_clean_segments(tasks, segments, current, mapper, task_id):
    clean_count = 0
    for segment in segments:
        if current is not None and not _is_same_point(current, segment["start"]):
            task_id = _append_xy_path(
                tasks,
                [current, segment["start"]],
                segment["areaNumber"],
                mapper,
                task_id,
                "modeling_transfer",
            )
        task = _segment_task(
            segment["start"],
            segment["end"],
            1,
            segment["areaNumber"],
            task_id,
            mapper,
            "modeling_clean",
        )
        if task is not None:
            task["sourceLaneId"] = segment.get("sourceId")
            tasks.append(task)
            task_id += 1
            clean_count += 1
            current = segment["end"]
    return current, task_id, clean_count


def _generate_bridge_round_trip_plan(draft, preview, mapper, route_policy, now=None):
    point_map = _draft_point_map(draft)
    anchors = route_policy.get("groupAnchors") or {}
    remote_group_id = route_policy.get("remoteGroupId")
    home_group_id = route_policy.get("homeGroupId")
    remote_group = _preview_group(preview, remote_group_id) or {}
    home_group = _preview_group(preview, home_group_id) or {}
    remote_anchor = anchors.get(remote_group_id) or {}
    home_anchor = anchors.get(home_group_id) or {}

    remote_entry = mapper.point_to_xy(point_map.get(remote_anchor.get("entryPointId")))
    remote_exit = mapper.point_to_xy(point_map.get(remote_anchor.get("exitPointId")))
    home_entry = mapper.point_to_xy(point_map.get(home_anchor.get("entryPointId")))
    home_exit = mapper.point_to_xy(point_map.get(home_anchor.get("exitPointId")))
    if None in (remote_entry, remote_exit, home_entry, home_exit):
        raise ModelingTaskGenerationError("route policy anchors are incomplete")

    tasks = []
    task_id = 1
    clean_count = 0
    outbound = _point_ids_to_xy(route_policy.get("outboundPointIds"), point_map, mapper)
    if len(outbound) < 2:
        raise ModelingTaskGenerationError("route policy outbound path is incomplete")
    task_id = _append_xy_path(
        tasks,
        outbound,
        remote_group.get("areaNumber") or 2,
        mapper,
        task_id,
        "modeling_outbound",
    )
    current = outbound[-1]

    remote_segments = _group_lane_segments(
        preview,
        remote_group_id,
        remote_entry,
        remote_exit,
    )
    current, task_id, added = _append_clean_segments(
        tasks, remote_segments, current, mapper, task_id
    )
    clean_count += added
    if not _is_same_point(current, remote_exit):
        task_id = _append_xy_path(
            tasks,
            [current, remote_exit],
            remote_group.get("areaNumber") or 2,
            mapper,
            task_id,
            "modeling_transfer",
        )
        current = remote_exit

    return_path = _point_ids_to_xy(route_policy.get("returnPointIds"), point_map, mapper)
    if not return_path:
        raise ModelingTaskGenerationError("route policy return path is incomplete")
    if not _is_same_point(current, return_path[0]):
        return_path.insert(0, current)
    task_id = _append_xy_path(
        tasks,
        return_path,
        home_group.get("areaNumber") or 1,
        mapper,
        task_id,
        "modeling_bridge_return",
    )
    current = return_path[-1]

    home_segments = _group_lane_segments(
        preview,
        home_group_id,
        home_entry,
        home_exit,
    )
    current, task_id, added = _append_clean_segments(
        tasks, home_segments, current, mapper, task_id
    )
    clean_count += added
    if not _is_same_point(current, home_exit):
        task_id = _append_xy_path(
            tasks,
            [current, home_exit],
            home_group.get("areaNumber") or 1,
            mapper,
            task_id,
            "modeling_return_origin",
        )

    if not tasks or clean_count == 0:
        raise ModelingTaskGenerationError("route policy produced no cleaning tasks")
    total_length = sum(int(task.get("length") or 0) for task in tasks)
    return {
        "status": "ready",
        "generatedAt": int(now if now is not None else time.time()),
        "taskName": draft.get("name") or draft.get("id") or "",
        "routeType": "bridge_round_trip",
        "summary": {
            "taskCount": len(tasks),
            "cleanTaskCount": clean_count,
            "transferTaskCount": len(tasks) - clean_count,
            "totalLengthCm": total_length,
        },
        "tasks": tasks,
    }


def generate_task_plan(draft, now=None):
    if not isinstance(draft, dict):
        raise ModelingTaskGenerationError("model draft is required")
    preview = draft.get("taskPreview") or {}
    if not isinstance(preview, dict) or preview.get("status") != "ready":
        raise ModelingTaskGenerationError("路径预览未生成或不可用，不能生成执行任务")

    mapper = _CoordinateMapper(draft)
    route_policy = draft.get("routePolicy") or {}
    if route_policy.get("type") == "bridge_round_trip":
        return _generate_bridge_round_trip_plan(
            draft,
            preview,
            mapper,
            route_policy,
            now=now,
        )
    tasks = []
    task_id = 1
    previous = None
    clean_count = 0

    for segment in _iter_clean_segments(preview):
        if previous is not None and not _is_same_point(previous["end"], segment["start"]):
            task_id = _append_transition_tasks(
                tasks,
                previous["end"],
                segment["start"],
                segment["areaNumber"],
                mapper,
                task_id,
                preview,
                previous.get("groupId"),
                segment.get("groupId"),
            )
        clean_task = _segment_task(
            segment["start"],
            segment["end"],
            1,
            segment["areaNumber"],
            task_id,
            mapper,
            "modeling_clean",
        )
        if clean_task is not None:
            clean_task["sourceLaneId"] = segment.get("sourceId")
            tasks.append(clean_task)
            task_id += 1
            clean_count += 1
            previous = segment

    if not tasks or clean_count == 0:
        raise ModelingTaskGenerationError("路径预览中没有可生成的清扫线")

    total_length = sum(int(task.get("length") or 0) for task in tasks)
    return {
        "status": "ready",
        "generatedAt": int(now if now is not None else time.time()),
        "taskName": draft.get("name") or draft.get("id") or "",
        "summary": {
            "taskCount": len(tasks),
            "cleanTaskCount": clean_count,
            "transferTaskCount": len(tasks) - clean_count,
            "totalLengthCm": total_length,
        },
        "tasks": tasks,
    }
