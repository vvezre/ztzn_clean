# coding=utf-8
import math
import time


# 机器人一次通过能覆盖的有效滚刷宽度。
BRUSH_WIDTH_CM = 116.0
# 相邻两条清扫线的目标重叠宽度，不是清扫线间距。
DEFAULT_OVERLAP_CM = 53.0
MIN_OVERLAP_CM = 10.0
EPSILON = 1e-6


class ModelingPreviewError(Exception):
    pass


def _number(value):
    """
    把 JSON 中的数字或数字字符串安全转为 float。

    None、空字符串、NaN 和无穷大都视为无效坐标，避免它们进入几何计算。
    """
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
    """将任意角度归一到 [0, 360)，保留 0.1 度。"""
    number = _number(angle)
    if number is None:
        return None
    return round(number % 360.0, 1)


def _point_xy(point):
    """从建模点中取出 (x, y)；两个坐标的单位都是厘米。"""
    x = _number(point.get("x"))
    y = _number(point.get("y"))
    if x is None or y is None:
        return None
    return x, y


def _distance(start, end):
    """计算两个相对坐标点之间的平面直线距离，返回单位为厘米。"""
    return math.hypot(end[0] - start[0], end[1] - start[1])


def _heading_from_points(start, end):
    """
    根据起点和终点计算小车航向。

    本项目坐标约定：+y 为 0°，+x 为 90°，-y 为 180°，-x 为 270°。
    因此使用 atan2(dx, dy)，而不是常见数学坐标的 atan2(dy, dx)。
    """
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
    """
    自动确定清扫主方向。

    当前约定用户打的前两个区域点表示希望的首段方向，
    后续生成的所有清扫线都与这个方向平行。
    """
    if len(points) < 2:
        return None
    start_xy = _point_xy(points[0])
    end_xy = _point_xy(points[1])
    if start_xy is None or end_xy is None:
        return None
    dx = end_xy[0] - start_xy[0]
    dy = end_xy[1] - start_xy[1]
    if abs(dx) < EPSILON and abs(dy) < EPSILON:
        return None
    # 几何求交必须保留完整精度；提前四舍五入到0.1°会让旋转矩形的最外侧扫描线
    # 与边界产生轻微夹角，最终漏掉首尾两条清扫线。展示时再统一保留0.1°。
    first_edge = math.degrees(math.atan2(dx, dy)) % 360.0
    return (first_edge + 90.0) % 360.0


def _group_sweep_angle(group, polygon):
    """
    获取一个区域的清扫方向。

    sweepDirection=manual 时使用人工指定的 sweepAngle；
    其他情况根据该区域前两个有效点自动计算。
    """
    direction = str(group.get("sweepDirection") or "auto")
    manual_angle = _normalize_heading(group.get("sweepAngle"))
    if direction == "manual" and manual_angle is not None:
        return manual_angle
    return _default_sweep_angle(polygon)


def _preview_point(point):
    """仅保留前端绘图和路径计算需要的点位字段。"""
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
    """
    整理多边形点序列。

    删除无效坐标、相邻重复点，并去掉“末点等于首点”的重复闭合点。
    几何求交时会自动连接最后一点和第一点，因此无需在数组中重复保存首点。
    """
    cleaned = []
    for point in points:
        xy = _point_xy(point)
        if xy is None:
            continue
        cleaned.append(point)
    return cleaned


def _unique_intersections(items):
    """合并误差 EPSILON 范围内的交点，避免扫描线穿过多边形顶点时重复计数。"""
    unique = []
    for item in items:
        if any(abs(item[0] - existing[0]) < EPSILON and abs(item[1] - existing[1]) < EPSILON for existing in unique):
            continue
        unique.append(item)
    return unique


def _line_polygon_intersections(polygon_xy, normal, offset):
    """
    计算一条无限长扫描线与多边形每条边的交点。

    扫描线使用法向量方程 normal·point=offset 表示。
    对每条多边形边判断两个端点是否分布在扫描线两侧，若是则线性插值求交点。
    """
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
    """
    在 2、4、6... 中选择实际间距最接近目标间距的清扫线数。

    如果生成 N 条线，第一条和最后一条分别放在区域两侧，
    那么实际间距是 span/(N-1)。程序比较相邻几个偶数候选值，
    优先选择与 target_spacing 差值最小的数量，差值相同时选更接近理论条数的方案。
    """
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
    """
    使用一组平行扫描线与区域多边形求交，得到每条真正位于区域内的清扫线段。

    输入：
    - polygon：按记录顺序排列的区域边界点。
    - sweep_angle：清扫线航向，0° 沿 +y，90° 沿 +x。
    - lane_spacing_cm：目标线间距，单位厘米。
    - force_even：是否将清扫线数限定为偶数。

    输出的每条 lane 包含 startX/startY、endX/endY、heading、lengthCm 和实际间距。
    force_even=True 时保证清扫线为偶数，便于生成往返式 S 形路线。
    """
    polygon_xy = [_point_xy(point) for point in polygon]
    polygon_xy = [xy for xy in polygon_xy if xy is not None]
    if len(polygon_xy) < 3 or sweep_angle is None:
        return []

    radians = math.radians(sweep_angle)
    # direction 是小车沿清扫线行驶的方向，normal 用于沿扫宽方向平移清扫线。
    direction = (math.sin(radians), math.cos(radians))
    normal = (math.cos(radians), -math.sin(radians))
    # 把每个多边形顶点投影到法向量上，最小/最大投影差就是扫宽方向的区域跨度。
    offsets = [normal[0] * x + normal[1] * y for x, y in polygon_xy]
    min_offset = min(offsets)
    max_offset = max(offsets)
    spacing = max(float(lane_spacing_cm), 1.0)

    span = max_offset - min_offset
    if force_even and span > EPSILON:
        # 偶数条模式下重新均分区域跨度，所以 actual_spacing 可能与目标值略有差异。
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
        # 一条扫描线可能与凹多边形产生多个交点；当前使用沿行驶方向最前和最后的交点作为线段端点。
        intersections = _line_polygon_intersections(polygon_xy, normal, offset)
        if len(intersections) >= 2:
            ordered = sorted(intersections, key=lambda xy: direction[0] * xy[0] + direction[1] * xy[1])
            start = ordered[0]
            end = ordered[-1]
            if _distance(start, end) > EPSILON:
                lanes.append({
                    "id": "lane-{}".format(index),
                    "heading": _normalize_heading(sweep_angle),
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
        "heading": _normalize_heading(sweep_angle),
        "startX": round(start[0], 1),
        "startY": round(start[1], 1),
        "endX": round(end[0], 1),
        "endY": round(end[1], 1),
        "lengthCm": round(_distance(start, end), 1),
    }]


def _build_group_link_preview(link):
    """
    生成跨区域连接桥的预览数据。

    连接桥可以包含两个或更多记录点，这里保留全部点及其顺序，
    后续任务生成器会按这些点分段通过连接桥，不强制只允许两个点。
    """
    points = list(link.get("points") or [])
    start_point = _preview_point(points[0]) if len(points) >= 1 else None
    end_point = _preview_point(points[-1]) if len(points) >= 2 else None
    length_cm = 0.0
    valid_segment_count = 0
    for index in range(len(points) - 1):
        start_xy = _point_xy(points[index])
        end_xy = _point_xy(points[index + 1])
        if start_xy is None or end_xy is None:
            continue
        length_cm += _distance(start_xy, end_xy)
        valid_segment_count += 1
    return {
        "id": link.get("id"),
        "name": link.get("name"),
        "type": link.get("type") or "group_connector",
        "startGroupId": link.get("startGroupId"),
        "endGroupId": link.get("endGroupId"),
        "status": link.get("status") or "draft",
        "startPoint": start_point,
        "endPoint": end_point,
        "points": [_preview_point(point) for point in points],
        "lengthCm": round(length_cm, 1) if valid_segment_count else None,
    }


def build_model_preview(draft, now=None, brush_width_cm=BRUSH_WIDTH_CM, overlap_cm=DEFAULT_OVERLAP_CM):
    """
    根据已确认的建模区域生成路径预览。

    draft 中的关键数据：
    - recognition.confirmed：区域识别是否已确认。
    - groups[].points：用户记录的区域点。
    - groups[].subAreas[].pointIds：每个子区域使用哪些点构成多边形。
    - groupLinks：区域之间的连接点序列。

    返回值中保留区域多边形、连接桥和全部清扫线，
    后续的任务生成和前端绘图都使用这份数据。
    """
    if not isinstance(draft, dict):
        raise ModelingPreviewError("model draft is required")
    recognition = draft.get("recognition") or {}
    if not isinstance(recognition, dict) or not recognition.get("confirmed"):
        raise ModelingPreviewError("区域识别结果未确认，不能生成路径预览")

    brush_width = float(brush_width_cm)
    overlap = max(float(overlap_cm), MIN_OVERLAP_CM)
    # 清扫线间距 = 滚刷宽度 - 重叠宽度。当前默认为 116 - 53 = 63cm。
    lane_spacing = max(brush_width - overlap, 1.0)

    group_previews = []
    sub_area_count = 0
    connector_count = 0
    lane_count = 0
    warnings = []
    route_policy = draft.get("routePolicy") or {}
    force_even_lanes = bool(route_policy.get("forceEvenLanes", True))

    for group in draft.get("groups") or []:
        # 每个 group 是一个独立清扫区域；一个 group 内仍可以识别出多个 subArea。
        points = list(group.get("points") or [])
        points_by_id = {point.get("id"): point for point in points}
        sub_area_previews = []
        for sub_area in group.get("subAreas") or []:
            # pointIds 只保存引用关系，这里按 ID 还原出真正的多边形点列表。
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
                "sweepAngle": _normalize_heading(sweep_angle),
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

    # 至少有一个可用子区域且已生成清扫线，路径预览才能进入 ready。
    status = "ready" if sub_area_count > 0 and lane_count > 0 else "empty"
    return {
        "status": status,
        "generatedAt": int(now if now is not None else time.time()),
        "config": {
            "brushWidthCm": round(brush_width, 1),
            "overlapCm": round(overlap, 1),
            "laneSpacingCm": round(lane_spacing, 1),
            "forceEvenLanes": force_even_lanes,
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
