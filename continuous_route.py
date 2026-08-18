# coding=utf-8
"""Helpers for executing recorded boundary points as one continuous polyline.

The route planner still stores one task per recorded sub-segment so the
frontend can draw every real boundary point.  The executor, however, must not
stop and restart the lower machine at every soft point.  These helpers group
adjacent tasks that belong to the same continuous path and attach the
remaining sub-segments to the first task as runtime-only look-ahead data.
"""

import math


CONTINUATION_KEY = "_continuousSegments"

# 连续折线跟踪的默认控制参数。这里的单位都是米：
# * 前视0.8米可以避免车辆直接瞄准只有十几厘米远的短点位；
# * 允许折线被平滑的最大距离为0.10米，超过后会自动缩短前视距离；
# * 最终停车点必须进入0.10米范围，不能只因为越过终点横截线就提前完成。
DEFAULT_LOOKAHEAD_M = 0.80
DEFAULT_CORRIDOR_M = 0.10
DEFAULT_FINAL_TOLERANCE_M = 0.10
# 车辆已经越过最终投影时不应继续前冲到30cm安全阈值。因制动距离和RTK滤波存在
# 延迟，越过终点后15cm内仍可作为完成；超过该范围立即报告终点越过失败并停车。
DEFAULT_PASSED_FINAL_TOLERANCE_M = 0.15
DEFAULT_TERMINAL_OVERSHOOT_M = 0.02
EARTH_RADIUS_M = 6371000.0

# 这台车的双天线RTK航向基准与地理方位角相差90度。项目原有的
# util.get_distance_angle()一直使用“地理方位角+90度”作为下位机转向目标。
# 连续折线跟踪必须采用同一约定，否则它算出的路径方向虽然在地图上正确，
# 发给小车后却会让车头少转90度。
VEHICLE_HEADING_OFFSET_DEG = 90.0


def _number(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(result) or math.isinf(result):
        return None
    return result


def _latlon_to_local_m(origin_lat, origin_lon, lat, lon):
    """把小范围经纬度转换成以origin为原点的局部米制坐标。"""
    lat0_rad = math.radians(float(origin_lat))
    x = math.radians(float(lon) - float(origin_lon)) * EARTH_RADIUS_M * math.cos(lat0_rad)
    y = math.radians(float(lat) - float(origin_lat)) * EARTH_RADIUS_M
    return x, y


def _local_m_to_latlon(origin_lat, origin_lon, x, y):
    """把局部米制坐标还原为经纬度。"""
    lat = float(origin_lat) + math.degrees(float(y) / EARTH_RADIUS_M)
    lat0_rad = math.radians(float(origin_lat))
    lon = float(origin_lon) + math.degrees(float(x) / (EARTH_RADIUS_M * math.cos(lat0_rad)))
    return lat, lon


def _distance_to_segment(point, start, end):
    """返回point到局部平面线段start->end的距离及投影比例。"""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        return math.hypot(point[0] - start[0], point[1] - start[1]), 0.0
    ratio = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_sq
    ratio = max(0.0, min(1.0, ratio))
    projection = (start[0] + ratio * dx, start[1] + ratio * dy)
    return math.hypot(point[0] - projection[0], point[1] - projection[1]), ratio


def _point_at_progress(local_points, cumulative, progress_m):
    """按沿折线累计距离取一个插值点。"""
    if not local_points:
        return (0.0, 0.0)
    progress_m = max(0.0, min(float(progress_m), cumulative[-1]))
    for index in range(len(local_points) - 1):
        start_s = cumulative[index]
        end_s = cumulative[index + 1]
        if progress_m <= end_s or index == len(local_points) - 2:
            segment_length = end_s - start_s
            if segment_length <= 1e-9:
                return local_points[index + 1]
            ratio = (progress_m - start_s) / segment_length
            start = local_points[index]
            end = local_points[index + 1]
            return (
                start[0] + (end[0] - start[0]) * ratio,
                start[1] + (end[1] - start[1]) * ratio,
            )
    return local_points[-1]


def _polyline_chord_deviation(local_points, cumulative, start_s, end_s):
    """计算折线在start_s到end_s之间相对直连弦线的最大偏离。"""
    chord_start = _point_at_progress(local_points, cumulative, start_s)
    chord_end = _point_at_progress(local_points, cumulative, end_s)
    candidates = [chord_start, chord_end]
    for index, distance_m in enumerate(cumulative):
        if start_s < distance_m < end_s:
            candidates.append(local_points[index])
    maximum = 0.0
    for point in candidates:
        distance_m, _ = _distance_to_segment(point, chord_start, chord_end)
        maximum = max(maximum, distance_m)
    return maximum


def build_continuous_polyline(root_segment, continuation_segments=None):
    """从保存任务的首段和后续软段构造完整经纬度折线。

    返回的点不会改变保存任务，也不会删除任何真实记录点。相邻重复点会被忽略，
    防止零长度段让航向计算失去意义。
    """
    if not isinstance(root_segment, dict):
        return []
    segments = [root_segment]
    segments.extend(item for item in list(continuation_segments or []) if isinstance(item, dict))

    points = []

    def append_point(lat, lon, task_id=None):
        lat = _number(lat)
        lon = _number(lon)
        if lat is None or lon is None:
            return
        if points:
            previous = points[-1]
            if abs(previous['lat'] - lat) <= 1e-12 and abs(previous['lon'] - lon) <= 1e-12:
                if task_id is not None:
                    previous['taskId'] = task_id
                return
        points.append({'lat': lat, 'lon': lon, 'taskId': task_id})

    append_point(root_segment.get('startLat'), root_segment.get('startLon'))
    for segment in segments:
        append_point(segment.get('endLat'), segment.get('endLon'), segment.get('id'))
    return points


def compute_polyline_guidance(points, current_lat, current_lon, previous_progress_m=0.0,
                              lookahead_m=DEFAULT_LOOKAHEAD_M,
                              corridor_m=DEFAULT_CORRIDOR_M,
                              final_tolerance_m=DEFAULT_FINAL_TOLERANCE_M,
                              passed_final_tolerance_m=DEFAULT_PASSED_FINAL_TOLERANCE_M):
    """根据当前位置计算连续折线的投影、前视目标和最终到点状态。

    算法先把车辆投影到原始折线上，再沿折线向前选择目标。前视弦线如果会把原始
    折线抹平超过corridor_m，就逐步缩短目标距离。因此车辆可以连续经过普通记录点，
    同时不会从直角或明显波动处大幅抄近路。
    """
    current_lat = _number(current_lat)
    current_lon = _number(current_lon)
    lookahead_m = max(0.20, _number(lookahead_m) or DEFAULT_LOOKAHEAD_M)
    corridor_m = max(0.01, _number(corridor_m) or DEFAULT_CORRIDOR_M)
    final_tolerance_m = max(0.01, _number(final_tolerance_m) or DEFAULT_FINAL_TOLERANCE_M)
    passed_final_tolerance_m = max(
        final_tolerance_m,
        _number(passed_final_tolerance_m) or DEFAULT_PASSED_FINAL_TOLERANCE_M,
    )
    previous_progress_m = max(0.0, _number(previous_progress_m) or 0.0)
    if current_lat is None or current_lon is None or not isinstance(points, (list, tuple)):
        return None

    normalized = []
    for item in points:
        if not isinstance(item, dict):
            continue
        lat = _number(item.get('lat'))
        lon = _number(item.get('lon'))
        if lat is None or lon is None:
            continue
        if normalized and abs(normalized[-1]['lat'] - lat) <= 1e-12 and abs(normalized[-1]['lon'] - lon) <= 1e-12:
            continue
        normalized.append({'lat': lat, 'lon': lon, 'taskId': item.get('taskId')})
    if len(normalized) < 2:
        return None

    origin_lat = normalized[0]['lat']
    origin_lon = normalized[0]['lon']
    local_points = [
        _latlon_to_local_m(origin_lat, origin_lon, item['lat'], item['lon'])
        for item in normalized
    ]
    cumulative = [0.0]
    for index in range(len(local_points) - 1):
        cumulative.append(cumulative[-1] + math.hypot(
            local_points[index + 1][0] - local_points[index][0],
            local_points[index + 1][1] - local_points[index][1],
        ))
    total_m = cumulative[-1]
    if total_m <= 1e-6:
        return None

    current_xy = _latlon_to_local_m(origin_lat, origin_lon, current_lat, current_lon)
    # 只允许在上一次进度附近向前寻找投影，避免自交折线或相邻清扫线使车辆跳到
    # 很远的后续段。RTK短时抖动最多允许回看0.25米，但最终进度保持单调不后退。
    search_min = max(0.0, previous_progress_m - 0.25)
    search_max = min(total_m, previous_progress_m + max(1.50, lookahead_m * 3.0))
    best = None
    for index in range(len(local_points) - 1):
        segment_start_s = cumulative[index]
        segment_end_s = cumulative[index + 1]
        if segment_end_s < search_min or segment_start_s > search_max:
            continue
        distance_m, ratio = _distance_to_segment(current_xy, local_points[index], local_points[index + 1])
        segment_length = segment_end_s - segment_start_s
        progress_m = segment_start_s + ratio * segment_length
        if progress_m < search_min - 1e-6 or progress_m > search_max + 1e-6:
            continue
        candidate = (distance_m, -progress_m, progress_m)
        if best is None or candidate < best:
            best = candidate
    if best is None:
        projected_progress_m = min(previous_progress_m, total_m)
        projected_xy = _point_at_progress(local_points, cumulative, projected_progress_m)
        cross_track_m = math.hypot(current_xy[0] - projected_xy[0], current_xy[1] - projected_xy[1])
    else:
        projected_progress_m = best[2]
        cross_track_m = best[0]
    progress_m = max(previous_progress_m, projected_progress_m)
    progress_m = min(progress_m, total_m)
    projected_xy = _point_at_progress(local_points, cumulative, progress_m)
    # 进度保持单调后，横向偏差也必须相对“当前有效进度点”重新计算；否则RTK短暂
    # 回跳时可能沿用后方投影的较小距离，低估车辆已经偏离当前路径的位置。
    cross_track_m = math.hypot(current_xy[0] - projected_xy[0], current_xy[1] - projected_xy[1])

    desired_target_s = min(total_m, progress_m + lookahead_m)
    target_s = desired_target_s
    # 从最长前视开始尝试，每次缩短10厘米，选择仍处于折线允许通道内的最远目标。
    while target_s - progress_m > 0.20:
        if _polyline_chord_deviation(local_points, cumulative, progress_m, target_s) <= corridor_m:
            break
        target_s = max(progress_m + 0.20, target_s - 0.10)
    chord_deviation_m = _polyline_chord_deviation(local_points, cumulative, progress_m, target_s)
    target_xy = _point_at_progress(local_points, cumulative, target_s)
    reference_xy = projected_xy
    dx = target_xy[0] - reference_xy[0]
    dy = target_xy[1] - reference_xy[1]
    if math.hypot(dx, dy) <= 0.05:
        # 车辆已经越过最终投影但尚未真正进入终点容差时，继续使用最后一段折线
        # 的方向逼近终点。不能构造“终点->终点”的零长度控制线，否则控制器会误以
        # 为距离为0而失去纠偏能力。
        reference_s = max(0.0, total_m - min(lookahead_m, 0.50))
        reference_xy = _point_at_progress(local_points, cumulative, reference_s)
        target_xy = local_points[-1]
        dx = target_xy[0] - reference_xy[0]
        dy = target_xy[1] - reference_xy[1]
    # atan2(东向位移, 北向位移)得到标准地理方位角：北0°、东90°。
    # 小车现有RTK/下位机控制链路使用相对它顺时针偏移90°的航向坐标，
    # 这里必须与util.get_distance_angle()保持一致，再把结果归一化到[0, 360)。
    geographic_bearing = math.degrees(math.atan2(dx, dy)) % 360.0
    heading = (geographic_bearing + VEHICLE_HEADING_OFFSET_DEG) % 360.0

    reference_lat, reference_lon = _local_m_to_latlon(
        origin_lat, origin_lon, reference_xy[0], reference_xy[1]
    )
    target_lat, target_lon = _local_m_to_latlon(
        origin_lat, origin_lon, target_xy[0], target_xy[1]
    )
    final_xy = local_points[-1]
    distance_to_final_m = math.hypot(current_xy[0] - final_xy[0], current_xy[1] - final_xy[1])
    remaining_m = max(0.0, total_m - progress_m)
    normal_complete = (
        remaining_m <= final_tolerance_m and
        distance_to_final_m <= final_tolerance_m and
        cross_track_m <= corridor_m
    )

    # progressM 被设计为单调且最多等于 totalM，仅看它无法区分“真正到点”和
    # “沿最后一段方向越过终点”。这里用最后一段单位向量计算车辆相对终点的
    # 有符号纵向位置：正值表示已经越过最终垂足，负值表示还在终点之前。
    last_start = local_points[-2]
    last_dx = final_xy[0] - last_start[0]
    last_dy = final_xy[1] - last_start[1]
    last_length = math.hypot(last_dx, last_dy)
    overshoot_m = 0.0
    if last_length > 1e-6:
        overshoot_m = (
            (current_xy[0] - final_xy[0]) * last_dx +
            (current_xy[1] - final_xy[1]) * last_dy
        ) / last_length
    passed_final = overshoot_m > DEFAULT_TERMINAL_OVERSHOOT_M
    passed_complete = (
        passed_final and
        distance_to_final_m <= passed_final_tolerance_m and
        cross_track_m <= passed_final_tolerance_m
    )
    complete = normal_complete or passed_complete
    terminal_missed = bool(passed_final and not passed_complete)
    return {
        'referenceLat': reference_lat,
        'referenceLon': reference_lon,
        'targetLat': target_lat,
        'targetLon': target_lon,
        'heading': heading,
        'progressM': progress_m,
        'remainingM': remaining_m,
        'totalM': total_m,
        'crossTrackM': cross_track_m,
        'distanceToFinalM': distance_to_final_m,
        'overshootM': max(0.0, overshoot_m),
        'passedFinal': passed_final,
        'terminalMissed': terminal_missed,
        'lookaheadM': max(0.0, target_s - progress_m),
        'chordDeviationM': chord_deviation_m,
        'complete': complete,
    }


def collect_continuous_run(segments, start_index):
    """Return the executable run beginning at ``start_index``.

    A following task is part of the same uninterrupted run only when:

    * both tasks have the same non-empty ``continuousPathId``;
    * the preceding task explicitly says not to stop at its end;
    * the following task explicitly says not to turn at its start; and
    * both tasks use the same movement mode.

    Missing flags keep their historical safe behaviour (stop and turn), so
    old task files are never silently merged.
    """
    if not isinstance(segments, (list, tuple)):
        return []
    if start_index < 0 or start_index >= len(segments):
        return []

    first = segments[start_index]
    if not isinstance(first, dict):
        return [first]

    run = [first]
    path_id = first.get("continuousPathId")
    mode = first.get("mode")
    if not path_id:
        return run

    cursor = start_index + 1
    while cursor < len(segments):
        previous = run[-1]
        candidate = segments[cursor]
        if not isinstance(candidate, dict):
            break
        if previous.get("stopAtEnd") is not False:
            break
        if candidate.get("turnAtStart") is not False:
            break
        if candidate.get("continuousPathId") != path_id:
            break
        if candidate.get("mode") != mode:
            break
        run.append(candidate)
        cursor += 1
    return run


def attach_continuations(run):
    """Copy a run into one runtime segment with look-ahead targets attached."""
    if not run:
        return None
    root = dict(run[0])
    root[CONTINUATION_KEY] = [dict(segment) for segment in run[1:]]
    # Cleanup after the combined run must follow the last sub-segment, not the
    # first one.  This keeps the vehicle moving through intermediate points
    # and still brakes at a real corner or at the end of the polyline.
    root["stopAtEnd"] = run[-1].get("stopAtEnd") is not False
    return root
