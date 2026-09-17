# coding=utf-8
"""Build a safe return-to-origin route from an already saved task.

During automatic cleaning, this module treats every saved executable segment
(including bridge segments) as an undirected graph and finds the shortest path
back to the task origin.  Before cleaning, it may use a direct line only when
the whole line is inside one recorded area polygon; otherwise it falls back to
the same saved-route graph.  A return route therefore never cuts across an
unknown gap or jumps directly between separate areas.

This module is deliberately independent from Flask, Redis and vehicle I/O so
the same result can be tested on Python 2 and Python 3.
"""

from __future__ import absolute_import

import heapq
import math


EARTH_RADIUS_M = 6371000.0
HARD_TURN_DEG = 55.0
MIN_SEGMENT_CM = 15.0
GRAPH_SPLIT_TOLERANCE_CM = 15.0
DIRECT_ROUTE_TOLERANCE_CM = 15.0
DIRECT_ROUTE_SAMPLE_CM = 5.0
GEOMETRY_EPSILON = 1e-6


class ReturnOriginRouteError(Exception):
    def __init__(self, code, message):
        super(ReturnOriginRouteError, self).__init__(message)
        self.code = code
        self.message = message


def _number(value):
    if value is None or value == "":
        return None
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if not math.isnan(result) and not math.isinf(result) else None


def _point(task, prefix):
    x = _number(task.get(prefix + "X"))
    y = _number(task.get(prefix + "Y"))
    lat = _number(task.get(prefix + "Lat"))
    lon = _number(task.get(prefix + "Lon"))
    if lat is None or lon is None:
        return None
    return {
        "x": int(round(x)) if x is not None else None,
        "y": int(round(y)) if y is not None else None,
        "lat": lat,
        "lon": lon,
    }


def _key(point):
    if point.get("x") is not None and point.get("y") is not None:
        return ("xy", int(point["x"]), int(point["y"]))
    return ("ll", round(float(point["lat"]), 8), round(float(point["lon"]), 8))


def _distance_cm(left, right):
    lat1 = math.radians(float(left["lat"]))
    lat2 = math.radians(float(right["lat"]))
    d_lat = lat2 - lat1
    d_lon = math.radians(float(right["lon"]) - float(left["lon"]))
    a = (
        math.sin(d_lat / 2.0) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(d_lon / 2.0) ** 2
    )
    a = max(0.0, min(1.0, a))
    return EARTH_RADIUS_M * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a)) * 100.0


def _xy(point):
    x = _number(point.get("x"))
    y = _number(point.get("y"))
    return None if x is None or y is None else (x, y)


def _xy_distance(left, right):
    return math.hypot(float(right[0]) - float(left[0]), float(right[1]) - float(left[1]))


def _project_to_segment(point, start, end):
    delta_x = float(end[0]) - float(start[0])
    delta_y = float(end[1]) - float(start[1])
    denominator = delta_x * delta_x + delta_y * delta_y
    if denominator <= GEOMETRY_EPSILON:
        return 0.0, start, _xy_distance(point, start)
    ratio = (
        (float(point[0]) - float(start[0])) * delta_x
        + (float(point[1]) - float(start[1])) * delta_y
    ) / denominator
    ratio = max(0.0, min(1.0, ratio))
    projection = (
        float(start[0]) + delta_x * ratio,
        float(start[1]) + delta_y * ratio,
    )
    return ratio, projection, _xy_distance(point, projection)


def _local_xy(origin_lat, origin_lon, lat, lon):
    mean_lat = math.radians((float(origin_lat) + float(lat)) / 2.0)
    return (
        math.radians(float(lon) - float(origin_lon))
        * EARTH_RADIUS_M * math.cos(mean_lat) * 100.0,
        math.radians(float(lat) - float(origin_lat)) * EARTH_RADIUS_M * 100.0,
    )


def _bearing(left, right):
    lat1 = math.radians(float(left["lat"]))
    lat2 = math.radians(float(right["lat"]))
    d_lon = math.radians(float(right["lon"]) - float(left["lon"]))
    east = math.sin(d_lon) * math.cos(lat2)
    north = (
        math.cos(lat1) * math.sin(lat2)
        - math.sin(lat1) * math.cos(lat2) * math.cos(d_lon)
    )
    # Keep the same vehicle-heading convention used by util.get_distance_angle.
    return (math.degrees(math.atan2(east, north)) + 90.0) % 360.0


def _turn_delta(left_heading, right_heading):
    return abs((float(right_heading) - float(left_heading) + 180.0) % 360.0 - 180.0)


def _variant_tasks(task_config):
    result = []
    seen = set()
    variants = task_config.get("routeVariants") or {}
    sources = [
        (variants.get("return") or {}).get("tasks"),
        (variants.get("noReturn") or {}).get("tasks"),
        task_config.get("taskList"),
    ]
    for tasks in sources:
        for task in tasks or []:
            if not isinstance(task, dict):
                continue
            start = _point(task, "start")
            end = _point(task, "end")
            if start is None or end is None:
                continue
            signature = (_key(start), _key(end))
            reverse_signature = (signature[1], signature[0])
            if signature in seen or reverse_signature in seen:
                continue
            seen.add(signature)
            result.append(dict(task))
    return result


def _build_graph(tasks):
    nodes = {}
    edges = []
    graph = {}
    for task in tasks:
        start = _point(task, "start")
        end = _point(task, "end")
        if start is None or end is None:
            continue
        start_key = _key(start)
        end_key = _key(end)
        if start_key == end_key:
            continue
        nodes.setdefault(start_key, start)
        nodes.setdefault(end_key, end)
        edges.append((start_key, end_key, task))

    # Different saved variants can put a waypoint in the middle of another
    # variant's longer segment.  Treat that waypoint as part of the segment,
    # otherwise Dijkstra can only reach it by travelling to the far endpoint
    # and reversing over the same line.
    node_items = list(nodes.items())
    for start_key, end_key, task in edges:
        start = nodes[start_key]
        end = nodes[end_key]
        start_xy = _xy(start)
        end_xy = _xy(end)
        split_nodes = [(0.0, start_key), (1.0, end_key)]
        if start_xy is not None and end_xy is not None:
            for node_key, point in node_items:
                if node_key in (start_key, end_key):
                    continue
                point_xy = _xy(point)
                if point_xy is None:
                    continue
                ratio, unused_projection, offset = _project_to_segment(
                    point_xy, start_xy, end_xy,
                )
                if (
                        ratio > GEOMETRY_EPSILON
                        and ratio < 1.0 - GEOMETRY_EPSILON
                        and offset <= GRAPH_SPLIT_TOLERANCE_CM):
                    split_nodes.append((ratio, node_key))
        split_nodes.sort(key=lambda item: (item[0], item[1]))
        for index in range(len(split_nodes) - 1):
            left_key = split_nodes[index][1]
            right_key = split_nodes[index + 1][1]
            if left_key == right_key:
                continue
            distance = _distance_cm(nodes[left_key], nodes[right_key])
            graph.setdefault(left_key, []).append((right_key, distance, task))
            graph.setdefault(right_key, []).append((left_key, distance, task))
    return nodes, graph


def _model_polygons(model):
    result = []
    if not isinstance(model, dict):
        return result
    for group in model.get("groups") or []:
        polygon = []
        for point in group.get("points") or []:
            point_xy = _xy(point)
            if point_xy is not None:
                polygon.append(point_xy)
        if len(polygon) > 3 and _xy_distance(polygon[0], polygon[-1]) <= GEOMETRY_EPSILON:
            polygon.pop()
        if len(polygon) >= 3:
            result.append(polygon)
    return result


def _point_on_polygon_boundary(point, polygon, tolerance):
    for index, start in enumerate(polygon):
        end = polygon[(index + 1) % len(polygon)]
        unused_ratio, unused_projection, distance = _project_to_segment(
            point, start, end,
        )
        if distance <= tolerance:
            return True
    return False


def _point_in_polygon(point, polygon, tolerance=DIRECT_ROUTE_TOLERANCE_CM):
    if _point_on_polygon_boundary(point, polygon, tolerance):
        return True
    inside = False
    x, y = point
    previous = polygon[-1]
    for current in polygon:
        current_x, current_y = current
        previous_x, previous_y = previous
        crosses = ((current_y > y) != (previous_y > y))
        if crosses:
            intersection_x = (
                (previous_x - current_x) * (y - current_y)
                / (previous_y - current_y)
                + current_x
            )
            if x < intersection_x:
                inside = not inside
        previous = current
    return inside


def _segment_inside_polygon(start, end, polygon):
    distance = _xy_distance(start, end)
    sample_count = max(1, int(math.ceil(distance / DIRECT_ROUTE_SAMPLE_CM)))
    for index in range(sample_count + 1):
        ratio = float(index) / float(sample_count)
        point = (
            start[0] + (end[0] - start[0]) * ratio,
            start[1] + (end[1] - start[1]) * ratio,
        )
        if not _point_in_polygon(point, polygon):
            return False
    return True


def _direct_return_is_safe(current, origin, model):
    current_xy = _xy(current)
    origin_xy = _xy(origin)
    if current_xy is None or origin_xy is None:
        return False
    return any(
        _segment_inside_polygon(current_xy, origin_xy, polygon)
        for polygon in _model_polygons(model)
    )


def _shortest_paths(graph, origin_key):
    distances = {origin_key: 0.0}
    following = {}
    queue = [(0.0, origin_key)]
    while queue:
        distance, node_key = heapq.heappop(queue)
        if distance != distances.get(node_key):
            continue
        for neighbor_key, edge_distance, task in graph.get(node_key, []):
            candidate = distance + edge_distance
            if candidate >= distances.get(neighbor_key, float("inf")):
                continue
            distances[neighbor_key] = candidate
            # Starting at neighbor, travel to node_key to get closer to origin.
            following[neighbor_key] = (node_key, task)
            heapq.heappush(queue, (candidate, neighbor_key))
    return distances, following


def _origin_key(task_config, tasks, nodes):
    origin = {
        "lat": _number(task_config.get("startLat")),
        "lon": _number(task_config.get("startLon")),
        "x": 0,
        "y": 0,
    }
    if origin["lat"] is None or origin["lon"] is None:
        raise ReturnOriginRouteError("RETURN_ORIGIN_MISSING", "当前路线缺少原点坐标")

    nearest_key = None
    nearest_distance = float("inf")
    for node_key, point in nodes.items():
        distance = _distance_cm(origin, point)
        if distance < nearest_distance:
            nearest_key = node_key
            nearest_distance = distance
    if nearest_key is None or nearest_distance > MIN_SEGMENT_CM:
        raise ReturnOriginRouteError("RETURN_ORIGIN_NOT_ON_ROUTE", "当前路线原点不在可执行路径上")
    return nearest_key, nodes[nearest_key]


def _active_candidates(active_segment, nodes, distances):
    if not isinstance(active_segment, dict):
        return []
    result = []
    for prefix in ("start", "end"):
        point = _point(active_segment, prefix)
        if point is None:
            continue
        point_key = _key(point)
        if point_key in nodes and point_key in distances:
            result.append(point_key)
    return result


def _interpolate_point(start, end, ratio, projection_xy):
    """Create a route point at ``ratio`` along an existing saved segment."""
    return {
        "x": int(round(projection_xy[0])),
        "y": int(round(projection_xy[1])),
        "lat": float(start["lat"]) + (
            float(end["lat"]) - float(start["lat"])
        ) * float(ratio),
        "lon": float(start["lon"]) + (
            float(end["lon"]) - float(start["lon"])
        ) * float(ratio),
    }


def _nearest_route_join(current, nodes, graph, distances):
    """Return the nearest point on the saved route and its homeward endpoint.

    The previous implementation compared the live position only with route
    *nodes*.  A vehicle in the middle of a long segment could therefore be
    rejected as more than one metre away even though it was exactly on the
    route.  Work with the actual graph edges here and project the live position
    onto each edge instead.
    """
    current_xy = _xy(current)
    best = None
    seen_edges = set()
    if current_xy is not None:
        for start_key, neighbors in graph.items():
            if start_key not in distances:
                continue
            for end_key, edge_distance, unused_task in neighbors:
                if end_key not in distances:
                    continue
                edge_key = frozenset((start_key, end_key))
                if edge_key in seen_edges:
                    continue
                seen_edges.add(edge_key)
                start = nodes[start_key]
                end = nodes[end_key]
                start_xy = _xy(start)
                end_xy = _xy(end)
                if start_xy is None or end_xy is None:
                    continue
                ratio, projection_xy, offset = _project_to_segment(
                    current_xy, start_xy, end_xy,
                )
                through_start = float(ratio) * edge_distance + distances[start_key]
                through_end = (1.0 - float(ratio)) * edge_distance + distances[end_key]
                if through_start <= through_end:
                    homeward_key = start_key
                    homeward_cost = through_start
                else:
                    homeward_key = end_key
                    homeward_cost = through_end
                candidate = (
                    float(offset),
                    float(homeward_cost),
                    _interpolate_point(start, end, ratio, projection_xy),
                    homeward_key,
                )
                if (
                        best is None
                        or candidate[0] < best[0] - GEOMETRY_EPSILON
                        or (
                            abs(candidate[0] - best[0]) <= GEOMETRY_EPSILON
                            and candidate[1] < best[1]
                        )):
                    best = candidate
    if best is not None:
        return best[2], best[3]

    # Old saved data may lack local x/y values.  It cannot be projected in the
    # local plane, so keep a coordinate-compatible fallback to the nearest
    # reachable node.  There is deliberately no maximum-distance rejection.
    candidates = list(distances.keys())
    if not candidates:
        return None, None
    chosen = min(
        candidates,
        key=lambda node_key: _distance_cm(current, nodes[node_key]),
    )
    return nodes[chosen], chosen


def _mark_continuous(tasks):
    if not tasks:
        return tasks
    headings = [float(task.get("heading") or 0.0) for task in tasks]
    hard_turns = [
        _turn_delta(headings[index], headings[index + 1]) >= HARD_TURN_DEG
        for index in range(len(headings) - 1)
    ]
    path_id = "return_to_origin"
    for index, task in enumerate(tasks):
        task["continuousPathId"] = path_id
        task["continuousPathIndex"] = index + 1
        task["continuousPathCount"] = len(tasks)
        task["turnAtStart"] = bool(index == 0 or hard_turns[index - 1])
        task["stopAtEnd"] = bool(index == len(tasks) - 1 or hard_turns[index])
    return tasks


def _segment(start, end, task_id, area_number=None):
    distance = _distance_cm(start, end)
    if distance <= MIN_SEGMENT_CM:
        return None
    return {
        "id": int(task_id),
        "mode": 2,
        "areaNumber": int(area_number or 1),
        "startX": start.get("x"),
        "startY": start.get("y"),
        "endX": end.get("x"),
        "endY": end.get("y"),
        "startLat": float(start["lat"]),
        "startLon": float(start["lon"]),
        "endLat": float(end["lat"]),
        "endLon": float(end["lon"]),
        "heading": round(_bearing(start, end), 1),
        "angle": round(_bearing(start, end), 1),
        "length": int(round(distance)),
        "turn_back_len": 0,
        "back_len": 0,
        "source": "return_to_origin",
    }


def build_return_to_origin_tasks(task_config, current_lat, current_lon,
                                 active_segment=None, max_idle_join_cm=None,
                                 model=None):
    """Return mode=2 tasks from the live position to the selected task origin.

    When automatic cleaning is active, ``active_segment`` anchors the live
    position to the segment that the robot is actually executing.  Before
    cleaning, a direct segment is accepted only if ``model`` proves that its
    complete geometry lies inside one recorded area.  Otherwise the vehicle
    joins the nearest point on a saved route segment and follows the route
    graph home.  ``max_idle_join_cm`` is retained only for call compatibility;
    node distance is no longer used as a safety gate.
    """
    if not isinstance(task_config, dict):
        raise ReturnOriginRouteError("RETURN_TASK_INVALID", "当前路线配置无效")
    current = {
        "lat": _number(current_lat),
        "lon": _number(current_lon),
        "x": None,
        "y": None,
    }
    if current["lat"] is None or current["lon"] is None:
        raise ReturnOriginRouteError("RETURN_POSITION_UNAVAILABLE", "当前RTK位置不可用")

    tasks = _variant_tasks(task_config)
    if not tasks:
        raise ReturnOriginRouteError("RETURN_ROUTE_EMPTY", "当前路线没有可用于返航的路径")
    nodes, graph = _build_graph(tasks)
    origin_key, origin = _origin_key(task_config, tasks, nodes)
    if _distance_cm(current, origin) <= MIN_SEGMENT_CM:
        return []

    origin_lat = _number(task_config.get("startLat"))
    origin_lon = _number(task_config.get("startLon"))
    if origin_lat is not None and origin_lon is not None:
        current_x, current_y = _local_xy(
            origin_lat, origin_lon, current["lat"], current["lon"],
        )
        current["x"] = int(round(current_x))
        current["y"] = int(round(current_y))

    # Before cleaning, a direct return is allowed only when the complete line
    # is covered by one recorded cleaning-area polygon.  This removes the
    # unnecessary perimeter detour inside an area without allowing a diagonal
    # shortcut across an unknown gap or between separate areas.
    if active_segment is None and _direct_return_is_safe(current, origin, model):
        direct_task = _segment(current, origin, 1)
        return _mark_continuous([direct_task] if direct_task is not None else [])

    distances, following = _shortest_paths(graph, origin_key)
    candidates = _active_candidates(active_segment, nodes, distances)
    join_point = None
    if candidates:
        chosen = min(
            candidates,
            key=lambda node_key: (
                _distance_cm(current, nodes[node_key]) + distances[node_key]
            ),
        )
    else:
        join_point, chosen = _nearest_route_join(current, nodes, graph, distances)
    if chosen is None:
        raise ReturnOriginRouteError("RETURN_ROUTE_DISCONNECTED", "当前路线无法连通到原点")

    node_path = [chosen]
    cursor = chosen
    while cursor != origin_key:
        step = following.get(cursor)
        if step is None:
            raise ReturnOriginRouteError("RETURN_ROUTE_DISCONNECTED", "当前路线无法连通到原点")
        cursor = step[0]
        node_path.append(cursor)

    route_points = [current]
    if join_point is not None:
        route_points.append(join_point)
    route_points.extend(nodes[node_key] for node_key in node_path)
    result = []
    for index in range(len(route_points) - 1):
        task = _segment(route_points[index], route_points[index + 1], len(result) + 1)
        if task is not None:
            result.append(task)
    if result and _distance_cm(_point(result[-1], "end"), origin) > MIN_SEGMENT_CM:
        raise ReturnOriginRouteError("RETURN_ROUTE_NOT_AT_ORIGIN", "生成的返航路线没有到达原点")
    return _mark_continuous(result)
