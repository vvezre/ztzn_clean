# coding=utf-8
"""根据已经确认的区域点生成“可绘制、可执行”的清扫线预览。

本文件只负责几何规划，不直接驱动小车。评审时可以按下面四步阅读：

1. 用区域前两个记录点确定一条边的方向，再旋转90度得到清扫主方向。
2. 用“滚刷宽度 - 重叠宽度”得到目标线间距，并生成满足最低重叠的奇偶候选。
3. 按记录顺序把区域边界分为两条侧边和两条边界清扫线；边界清扫线保留全部记录点。
4. 两条边界线之间生成平行直线，并输出给任务生成器排成 S 形。

坐标约定：x 向东为正，y 向北为正，单位厘米；航向0度沿+y，90度沿+x。
"""
import math
import time


# 机器人一次通过能覆盖的有效滚刷宽度。
BRUSH_WIDTH_CM = 116.0
# 相邻两条清扫线的目标重叠宽度，不是清扫线间距。
DEFAULT_OVERLAP_CM = 53.0
# 实际路线允许偏离目标重叠，但任何方案都不能低于30厘米重叠。
MIN_OVERLAP_CM = 30.0
# 浮点几何判断误差，避免把几乎相等的坐标误判成不同点。
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


def _path_length(points):
    """计算折线路径总长度，单位厘米。"""
    return sum(
        _distance(points[index], points[index + 1])
        for index in range(len(points) - 1)
    )


def _lane_path_payload(points):
    """把内部浮点坐标整理成可保存、可传给任务生成器的折线点。"""
    return [
        {
            "x": round(point[0], 1),
            "y": round(point[1], 1),
        }
        for point in points
    ]


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
    # 没有两个点就无法得到边界方向。
    if len(points) < 2:
        return None
    # 记录顺序有业务含义：points[0] -> points[1] 就是用户先记录的区域侧边。
    start_xy = _point_xy(points[0])
    end_xy = _point_xy(points[1])
    if start_xy is None or end_xy is None:
        return None
    # 边界方向向量 v=(dx,dy)。
    dx = end_xy[0] - start_xy[0]
    dy = end_xy[1] - start_xy[1]
    if abs(dx) < EPSILON and abs(dy) < EPSILON:
        return None
    # 几何求交必须保留完整精度；提前四舍五入到0.1°会让旋转矩形的最外侧扫描线
    # 与边界产生轻微夹角，最终漏掉首尾两条清扫线。展示时再统一保留0.1°。
    # 本项目0度沿+y，所以航向角使用 atan2(dx,dy)。
    first_edge = math.degrees(math.atan2(dx, dy)) % 360.0
    # 清扫线与该侧边垂直，因此统一沿顺时针方向旋转90度。
    # 旋转后的反方向仍属于同一组平行线；真正从哪一端出发由 S 形排序阶段决定。
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
    # 扫描线方程：normal.x*x + normal.y*y = offset。
    intersections = []
    count = len(polygon_xy)
    for index in range(count):
        start = polygon_xy[index]
        end = polygon_xy[(index + 1) % count]
        # start_value/end_value 是边的两个端点到扫描线的带符号投影差。
        start_value = normal[0] * start[0] + normal[1] * start[1] - offset
        end_value = normal[0] * end[0] + normal[1] * end[1] - offset
        if abs(start_value) < EPSILON:
            intersections.append(start)
        # 两个值相等说明该边与扫描线平行，不能用下面的比例式求唯一交点。
        denominator = start_value - end_value
        if abs(denominator) < EPSILON:
            continue
        # 在线段 start -> end 上线性插值：Q=start+ratio*(end-start)。
        ratio = start_value / denominator
        if -EPSILON <= ratio <= 1.0 + EPSILON:
            x = start[0] + (end[0] - start[0]) * ratio
            y = start[1] + (end[1] - start[1]) * ratio
            intersections.append((x, y))
    return _unique_intersections(intersections)


def _minimum_lane_count(span, max_spacing):
    """Return the smallest lane count whose actual spacing does not exceed max_spacing."""
    if span <= EPSILON:
        return 0
    # 至少保留1厘米的正间距，避免配置错误造成除零。
    spacing_limit = max(float(max_spacing), 1.0)
    # N条线有N-1个间隔，因此 N >= ceil(span/max_spacing)+1。
    return max(2, int(math.ceil(span / spacing_limit)) + 1)


def _nearest_even_lane_count(span, target_spacing, max_spacing=None):
    """
    在 2、4、6... 中选择实际间距最接近目标间距的清扫线数。

    如果生成 N 条线，第一条和最后一条分别放在区域两侧，
    那么实际间距是 span/(N-1)。程序比较相邻几个偶数候选值，
    优先选择与 target_spacing 差值最小的数量，差值相同时选更接近理论条数的方案。
    """
    if span <= EPSILON:
        return 0
    # 如果不限制整数和奇偶，理论条数 N*=span/target_spacing+1。
    ideal = span / max(float(target_spacing), 1.0) + 1.0
    # 找到理论条数附近较小的偶数，再向两侧扩展少量偶数候选。
    lower = max(2, int(math.floor(ideal / 2.0)) * 2)
    candidates = sorted(set([max(2, lower - 2), lower, lower + 2, lower + 4]))
    if max_spacing is not None:
        # 先根据“最低重叠30厘米”得到绝不能少于的条数，再向上取偶数。
        minimum = _minimum_lane_count(span, max_spacing)
        minimum_even = minimum if minimum % 2 == 0 else minimum + 1
        candidates.extend([minimum_even, minimum_even + 2])
        # 删除实际间距超过最大允许值的方案，保证实际重叠不低于下限。
        candidates = sorted(set(
            count for count in candidates
            if span / float(count - 1) <= float(max_spacing) + EPSILON
        ))
    # 排序优先级：实际间距最接近目标值；其次条数最接近理论值；最后取更少条数。
    return min(
        candidates,
        key=lambda count: (
            abs(span / float(count - 1) - target_spacing),
            abs(count - ideal),
            count,
        ),
    )


def _lane_count_candidates(span, target_spacing, max_spacing=None):
    """返回同时包含奇数和偶数的有限清扫线数量候选。

    N 条清扫线共有 N-1 个间隔。最低重叠决定最少需要多少条线，目标重叠
    决定理论最优条数。候选范围从满足最低重叠的最少条数开始，到理论条数
    上方一个整数为止；这样既包含目标值附近的奇偶方案，也不会为了改变出口
    一直增加没有必要的清扫线。
    """
    if span <= EPSILON:
        return []
    spacing = max(float(target_spacing), 1.0)
    ideal = span / spacing + 1.0
    minimum = 2
    if max_spacing is not None:
        minimum = _minimum_lane_count(span, max_spacing)
    maximum = max(minimum, int(math.ceil(ideal)) + 1)
    candidates = []
    for count in range(minimum, maximum + 1):
        actual_spacing = span / float(count - 1)
        if max_spacing is not None and actual_spacing > float(max_spacing) + EPSILON:
            continue
        candidates.append(count)
    return candidates


def _nearest_lane_count(span, target_spacing, max_spacing=None):
    """在不限制奇偶的候选中选择实际间距最接近目标值的条数。"""
    candidates = _lane_count_candidates(span, target_spacing, max_spacing=max_spacing)
    if not candidates:
        return 0
    ideal = span / max(float(target_spacing), 1.0) + 1.0
    return min(
        candidates,
        key=lambda count: (
            abs(span / float(count - 1) - target_spacing),
            abs(count - ideal),
            count,
        ),
    )


def _angle_difference(left, right):
    """返回两个航向之间的最小夹角，单位为度。"""
    return abs((float(left) - float(right) + 180.0) % 360.0 - 180.0)


def _is_convex_quadrilateral(points):
    """
    判断四个按边界顺序排列的点是否构成凸四边形。

    凹四边形或存在连续三点共线时继续使用原有多边形扫描算法，避免使用两侧插值后
    生成越出区域的清扫线。
    """
    if len(points) != 4:
        return False
    signs = []
    for index in range(4):
        previous = points[index]
        current = points[(index + 1) % 4]
        following = points[(index + 2) % 4]
        first = (current[0] - previous[0], current[1] - previous[1])
        second = (following[0] - current[0], following[1] - current[1])
        cross = first[0] * second[1] - first[1] * second[0]
        if abs(cross) <= EPSILON:
            return False
        signs.append(cross > 0)
    return all(sign == signs[0] for sign in signs[1:])


def _recorded_boundary_paths(polygon_xy, direction, normal):
    """
    按人工记录顺序识别两条真正需要清扫的外边界折线。

    区域点按一圈边界依次记录，第一个点位于第一条侧边的起点；自动清扫方向又与
    第一段边界垂直。因此闭合边界的边序列应当依次呈现：

        第一侧边 -> 第一外边界 -> 第二侧边 -> 第二外边界

    这里不根据某个固定矩形坐标找四个角，而是枚举三个分割位置。每条边分别计算
    在清扫方向 direction 和跨行方向 normal 上的投影长度，选择与上述四段方向最
    匹配的分割。这样一条外边界中即使包含上下波动的多个记录点，也只影响折线形状，
    不会被当成需要平滑或删除的误点。

    返回值按 normal 投影从小到大排列。每条路径均调整为沿 direction 正向行驶，
    后续 S 形反向由任务生成器处理。
    """
    count = len(polygon_xy)
    if count < 4:
        return None

    edges = []
    for index in range(count):
        start = polygon_xy[index]
        end = polygon_xy[(index + 1) % count]
        vector = (end[0] - start[0], end[1] - start[1])
        length = math.hypot(vector[0], vector[1])
        if length <= EPSILON:
            # 重复点仍保留在原始列表中，但零长度边不应左右分段结果。
            sweep_component = 0.0
            side_component = 0.0
        else:
            sweep_component = abs(direction[0] * vector[0] + direction[1] * vector[1])
            side_component = abs(normal[0] * vector[0] + normal[1] * vector[1])
        edges.append((sweep_component, side_component))

    # 前缀和让每一种四段切分都能用常数时间计算方向匹配分数。
    sweep_prefix = [0.0]
    side_prefix = [0.0]
    for sweep_component, side_component in edges:
        sweep_prefix.append(sweep_prefix[-1] + sweep_component)
        side_prefix.append(side_prefix[-1] + side_component)

    def score(prefix, start, end):
        return prefix[end] - prefix[start]

    best = None
    # 至少为四段各保留一条边：0:a侧边，a:b外边界，b:c侧边，c:n外边界。
    for first_split in range(1, count - 2):
        for second_split in range(first_split + 1, count - 1):
            for third_split in range(second_split + 1, count):
                matched = (
                    score(side_prefix, 0, first_split)
                    + score(sweep_prefix, first_split, second_split)
                    + score(side_prefix, second_split, third_split)
                    + score(sweep_prefix, third_split, count)
                )
                mismatched = (
                    score(sweep_prefix, 0, first_split)
                    + score(side_prefix, first_split, second_split)
                    + score(sweep_prefix, second_split, third_split)
                    + score(side_prefix, third_split, count)
                )
                candidate = (
                    matched - mismatched,
                    matched,
                    -first_split,
                    -second_split,
                    -third_split,
                    first_split,
                    second_split,
                    third_split,
                )
                if best is None or candidate[:5] > best[:5]:
                    best = candidate

    if best is None:
        return None
    first_split, second_split, third_split = best[5], best[6], best[7]

    first_boundary = list(polygon_xy[first_split:second_split + 1])
    second_boundary = list(polygon_xy[third_split:]) + [polygon_xy[0]]
    if len(first_boundary) < 2 or len(second_boundary) < 2:
        return None

    def orient(points):
        start_projection = direction[0] * points[0][0] + direction[1] * points[0][1]
        end_projection = direction[0] * points[-1][0] + direction[1] * points[-1][1]
        return list(reversed(points)) if end_projection < start_projection else points

    first_boundary = orient(first_boundary)
    second_boundary = orient(second_boundary)
    if _path_length(first_boundary) <= EPSILON or _path_length(second_boundary) <= EPSILON:
        return None

    def average_normal_offset(points):
        return sum(
            normal[0] * point[0] + normal[1] * point[1]
            for point in points
        ) / float(len(points))

    paths = [first_boundary, second_boundary]
    paths.sort(key=average_normal_offset)
    return paths


def _generate_boundary_interpolated_quadrilateral_lanes(
        polygon, sweep_angle, lane_spacing_cm, force_even, max_spacing_cm=None,
        lane_count_override=None):
    """
    为按顺序记录的区域生成“外边界折线 + 内部直线”清扫线。

    建模约定前两个点给出一条侧边方向，清扫线与它垂直。程序按记录顺序识别
    两条侧边和两条外边界：第一条、最后一条清扫线完整经过对应外边界的全部
    记录点，中间清扫线保持直线。

    这样既兼容原有四点梯形，也支持一条边上记录多个点、边界上下波动的区域。

    只有 sweep_angle 与“前两个点自动确定的方向”一致时才启用该方法；手工指定
    其他清扫方向时仍走原有通用扫描线算法。
    """
    polygon_xy = [_point_xy(point) for point in polygon]
    if any(point is None for point in polygon_xy):
        return None
    if len(polygon_xy) < 4:
        return None

    automatic_angle = _default_sweep_angle(polygon)
    if automatic_angle is None or _angle_difference(sweep_angle, automatic_angle) > 1e-4:
        return None

    # 航向角转换为单位方向向量和与它垂直的法向量。
    radians = math.radians(sweep_angle)
    direction = (math.sin(radians), math.cos(radians))
    normal = (math.cos(radians), -math.sin(radians))
    boundary_paths = _recorded_boundary_paths(polygon_xy, direction, normal)
    if boundary_paths is None:
        return None
    first_boundary, last_boundary = boundary_paths

    offsets = [normal[0] * point[0] + normal[1] * point[1] for point in polygon_xy]
    min_offset = min(offsets)
    max_offset = max(offsets)
    # 四点区域继续使用左右侧边投影均值，保持已验证路线与Python 2/3结果不变；
    # 多点不规则区域使用完整投影跨度，保证最外侧波动也处于滚刷覆盖范围内。
    if len(polygon_xy) == 4 and _is_convex_quadrilateral(polygon_xy):
        first, second, third, fourth = polygon_xy
        left_span = abs(
            normal[0] * (first[0] - second[0])
            + normal[1] * (first[1] - second[1])
        )
        right_span = abs(
            normal[0] * (fourth[0] - third[0])
            + normal[1] * (fourth[1] - third[1])
        )
        span = (left_span + right_span) / 2.0
    else:
        span = max_offset - min_offset
    if span <= EPSILON:
        return None

    # spacing 是目标值63厘米；lane_count_override 用于生成指定奇偶候选。
    spacing = max(float(lane_spacing_cm), 1.0)
    if lane_count_override is not None:
        lane_count = max(2, int(lane_count_override))
    elif force_even:
        lane_count = _nearest_even_lane_count(span, spacing, max_spacing=max_spacing_cm)
    else:
        lane_count = _nearest_lane_count(span, spacing, max_spacing=max_spacing_cm)
    # N条线把整个区域跨度分成N-1份，实际间距=D/(N-1)。
    actual_spacing = span / float(lane_count - 1)

    lanes = []
    for index in range(lane_count):
        # ratio从0均匀变化到1，对应从第一条真实边界移动到第二条真实边界。
        ratio = index / float(lane_count - 1)
        if index == 0:
            lane_path = list(first_boundary)
            lane_type = "boundary"
            boundary_side = "min_offset"
        elif index == lane_count - 1:
            lane_path = list(last_boundary)
            lane_type = "boundary"
            boundary_side = "max_offset"
        else:
            # 内部线使用固定法向偏移与多边形求交，因此不跟随外边界波动。
            offset = min_offset + (max_offset - min_offset) * ratio
            intersections = _line_polygon_intersections(polygon_xy, normal, offset)
            ordered = sorted(
                intersections,
                key=lambda xy: direction[0] * xy[0] + direction[1] * xy[1],
            )
            if len(ordered) < 2:
                # 极端凹形或退化数据无法得到两个交点时，退回两侧端点线性插值。
                start = (
                    first_boundary[0][0] + (last_boundary[0][0] - first_boundary[0][0]) * ratio,
                    first_boundary[0][1] + (last_boundary[0][1] - first_boundary[0][1]) * ratio,
                )
                end = (
                    first_boundary[-1][0] + (last_boundary[-1][0] - first_boundary[-1][0]) * ratio,
                    first_boundary[-1][1] + (last_boundary[-1][1] - first_boundary[-1][1]) * ratio,
                )
            else:
                start, end = ordered[0], ordered[-1]
            lane_path = [start, end]
            lane_type = "interior"
            boundary_side = None

        start = lane_path[0]
        end = lane_path[-1]
        # 保持每条预览线的 start -> end 与统一清扫方向同向，S形反向由任务生成器处理。
        projection = direction[0] * (end[0] - start[0]) + direction[1] * (end[1] - start[1])
        if projection < 0:
            lane_path = list(reversed(lane_path))
            start, end = lane_path[0], lane_path[-1]
        if _path_length(lane_path) <= EPSILON:
            continue
        heading = _normalize_heading(
            math.degrees(math.atan2(end[0] - start[0], end[1] - start[1]))
        )
        lanes.append({
            "id": "lane-{}".format(index + 1),
            "heading": heading,
            "startX": round(start[0], 1),
            "startY": round(start[1], 1),
            "endX": round(end[0], 1),
            "endY": round(end[1], 1),
            "lengthCm": round(_path_length(lane_path), 1),
            "laneSpacingCm": round(actual_spacing, 1),
            "laneType": lane_type,
            "boundarySide": boundary_side,
            "pathPoints": _lane_path_payload(lane_path),
        })
    return lanes


def _generate_lanes(
        polygon, sweep_angle, lane_spacing_cm, force_even=False, max_spacing_cm=None,
        lane_count_override=None):
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

    # 优先使用能完整保留四条真实边界的四点插值算法。
    quadrilateral_lanes = _generate_boundary_interpolated_quadrilateral_lanes(
        polygon,
        sweep_angle,
        lane_spacing_cm,
        force_even,
        max_spacing_cm=max_spacing_cm,
        lane_count_override=lane_count_override,
    )
    if quadrilateral_lanes is not None:
        return quadrilateral_lanes

    # 四点专用算法不适用时，建立通用平行扫描线坐标系。
    radians = math.radians(sweep_angle)
    # direction 是小车沿清扫线行驶的方向，normal 用于沿扫宽方向平移清扫线。
    direction = (math.sin(radians), math.cos(radians))
    normal = (math.cos(radians), -math.sin(radians))
    # 把每个多边形顶点投影到法向量上，最小/最大投影差就是扫宽方向的区域跨度。
    # q=n·P：把每个顶点投影到扫宽方向；最大值减最小值就是区域扫宽跨度D。
    offsets = [normal[0] * x + normal[1] * y for x, y in polygon_xy]
    min_offset = min(offsets)
    max_offset = max(offsets)
    spacing = max(float(lane_spacing_cm), 1.0)

    span = max_offset - min_offset
    if span > EPSILON and (lane_count_override is not None or force_even):
        # 指定条数和旧偶数兼容模式都重新均分完整区域跨度。
        if lane_count_override is not None:
            lane_count = max(2, int(lane_count_override))
        else:
            lane_count = _nearest_even_lane_count(span, spacing, max_spacing=max_spacing_cm)
        actual_spacing = span / float(lane_count - 1)
        # 每个offset对应一条方程 n·P=offset 的无限长平行扫描线。
        lane_offsets = [min_offset + actual_spacing * index for index in range(lane_count)]
    else:
        actual_spacing = spacing
        if max_spacing_cm is not None:
            minimum_count = _minimum_lane_count(span, max_spacing_cm)
            if minimum_count > 1 and span > EPSILON:
                actual_spacing = min(spacing, span / float(minimum_count - 1))
        lane_offsets = []
        offset = min_offset
        while offset <= max_offset + EPSILON:
            lane_offsets.append(offset)
            offset += actual_spacing

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


def _generate_lane_candidates(
        polygon, sweep_angle, lane_spacing_cm, brush_width_cm,
        max_spacing_cm=None):
    """为一个子区域生成目标条数附近的全部合法奇偶清扫线方案。"""
    polygon_xy = [_point_xy(point) for point in polygon]
    polygon_xy = [xy for xy in polygon_xy if xy is not None]
    if len(polygon_xy) < 3 or sweep_angle is None:
        return []

    radians = math.radians(sweep_angle)
    normal = (math.cos(radians), -math.sin(radians))
    if len(polygon_xy) == 4 and _is_convex_quadrilateral(polygon_xy):
        first, second, third, fourth = polygon_xy
        left_span = abs(normal[0] * (first[0] - second[0]) + normal[1] * (first[1] - second[1]))
        right_span = abs(normal[0] * (fourth[0] - third[0]) + normal[1] * (fourth[1] - third[1]))
        span = (left_span + right_span) / 2.0
    else:
        offsets = [normal[0] * x + normal[1] * y for x, y in polygon_xy]
        span = max(offsets) - min(offsets)

    result = []
    for lane_count in _lane_count_candidates(
            span, lane_spacing_cm, max_spacing=max_spacing_cm):
        lanes = _generate_lanes(
            polygon,
            sweep_angle,
            lane_spacing_cm,
            force_even=False,
            max_spacing_cm=max_spacing_cm,
            lane_count_override=lane_count,
        )
        if len(lanes) != lane_count:
            continue
        actual_spacing = span / float(lane_count - 1)
        result.append({
            "laneCount": lane_count,
            "parity": "even" if lane_count % 2 == 0 else "odd",
            "laneSpacingCm": round(actual_spacing, 1),
            "actualOverlapCm": round(float(brush_width_cm) - actual_spacing, 1),
            "lanes": lanes,
        })
    return result


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

    # 有效滚刷宽度W=116厘米。
    brush_width = float(brush_width_cm)
    # 目标重叠O默认53厘米，并强制不能配置到30厘米以下。
    overlap = max(float(overlap_cm), MIN_OVERLAP_CM)
    # 清扫线间距 = 滚刷宽度 - 重叠宽度。当前默认为 116 - 53 = 63cm。
    lane_spacing = max(brush_width - overlap, 1.0)
    # 最大允许间距=116-30=86厘米，用于保证实际重叠始终不少于30厘米。
    max_lane_spacing = max(brush_width - MIN_OVERLAP_CM, 1.0)

    group_previews = []
    sub_area_count = 0
    connector_count = 0
    lane_count = 0
    warnings = []
    route_policy = draft.get("routePolicy") or {}
    # 新规划默认不限制奇偶；显式 forceEvenLanes 只保留给历史策略兼容。
    force_even_lanes = bool(route_policy.get("forceEvenLanes", False))

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
            lane_candidates = _generate_lane_candidates(
                polygon,
                sweep_angle,
                lane_spacing,
                brush_width,
                max_spacing_cm=max_lane_spacing,
            )
            if force_even_lanes:
                compatible = [
                    candidate for candidate in lane_candidates
                    if candidate.get("parity") == "even"
                ]
            else:
                compatible = list(lane_candidates)
            selected_candidate = min(
                compatible or lane_candidates,
                key=lambda candidate: (
                    abs(float(candidate.get("laneSpacingCm") or 0.0) - lane_spacing),
                    candidate.get("laneCount") or 0,
                ),
            ) if lane_candidates else None
            lanes = list((selected_candidate or {}).get("lanes") or [])
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
                "laneCandidates": lane_candidates,
                "laneCount": len(lanes),
                "laneSpacingCm": lanes[0].get("laneSpacingCm") if lanes else None,
                "actualOverlapCm": (
                    selected_candidate.get("actualOverlapCm")
                    if selected_candidate else None
                ),
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
            "minimumOverlapCm": round(MIN_OVERLAP_CM, 1),
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
        "areaOrder": route_policy.get("areaOrder") or [
            group.get("areaNumber")
            for group in (draft.get("groups") or [])
        ],
        "warnings": warnings,
    }
