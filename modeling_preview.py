# coding=utf-8
"""根据已经确认的区域点生成“可绘制、可执行”的清扫线预览。

本文件只负责几何规划，不直接驱动小车。评审时可以按下面四步阅读：

1. 用完整区域的凸包和最小外接矩形确定长轴方向，不依赖前两个点或记录方向。
2. 用“滚刷宽度 - 重叠宽度”得到目标线间距，并生成满足最低重叠的奇偶候选。
3. 在闭合边界的所有循环起点和正反方向中识别两条外边界，保留真实边界折线。
4. 两条边界折线之间生成沿区域长轴的平行直线，并输出给任务生成器排成 S 形。

坐标约定：x 向东为正，y 向北为正，单位厘米；航向0度沿+y，90度沿+x。
"""
import math
import time


# 机器人一次直线通过时，滚刷在“垂直于行驶方向”上能够覆盖的有效宽度 W。
# 单位统一使用厘米；当前设备实测/业务配置为 W=116cm。
BRUSH_WIDTH_CM = 116.0
# 相邻两次滚刷覆盖区域希望重复覆盖的目标宽度 O_target。
# 注意：53cm 是“重叠宽度”，不是两条清扫中心线之间的距离。
DEFAULT_OVERLAP_CM = 53.0
# 最低允许重叠 O_min。整数条清扫线无法保证实际重叠恰好等于目标值，
# 但所有候选都必须满足 O_actual>=30cm，这是路线覆盖完整性的硬约束。
MIN_OVERLAP_CM = 30.0
# 浮点几何判断误差，避免把几乎相等的坐标误判成不同点。
EPSILON = 1e-6
# 人工沿区域边界记录一圈后，最后一点经常会落在第一个点附近。二者距离
# 不超过30cm时，规划几何把最后一点并入第一个点，避免闭合边界生成一条
# 无意义的十几厘米清扫短线。这里只整理规划副本，不修改原始记录点数据。
BOUNDARY_CLOSURE_MERGE_CM = 30.0


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


def _convex_hull(points):
    """返回与记录起点、记录方向和重复采样数量无关的二维凸包。"""
    unique = sorted(set(
        (float(point[0]), float(point[1]))
        for point in points
        if point is not None
    ))
    if len(unique) <= 1:
        return unique

    def cross(origin, left, right):
        return (
            (left[0] - origin[0]) * (right[1] - origin[1])
            - (left[1] - origin[1]) * (right[0] - origin[0])
        )

    lower = []
    for point in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= EPSILON:
            lower.pop()
        lower.append(point)
    upper = []
    for point in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= EPSILON:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def _canonical_axis_heading(direction):
    """把一条无向轴统一表示成[0,180)航向，消除正反记录造成的180度差。"""
    heading = math.degrees(math.atan2(direction[0], direction[1])) % 180.0
    if abs(heading - 180.0) <= 1e-9:
        return 0.0
    return heading


def _default_sweep_angle(points):
    """
    根据完整区域形状自动确定清扫主方向。

    区域点只描述闭合边界，不再用第一个点、第二个点或第一条记录边决定清扫
    方向。程序先计算所有边界点的凸包，再枚举凸包边对应的外接矩形，选择面积
    最小且跨行宽度最小的稳定方向，最后沿外接矩形长轴生成清扫线。这样同一
    区域无论从哪个角开始、顺时针还是逆时针记录，都会得到同一组平行清扫线。
    """
    polygon_xy = [_point_xy(point) for point in points]
    polygon_xy = [point for point in polygon_xy if point is not None]
    hull = _convex_hull(polygon_xy)
    if len(hull) < 2:
        return None

    candidates = []
    for index, start in enumerate(hull):
        end = hull[(index + 1) % len(hull)]
        edge = (end[0] - start[0], end[1] - start[1])
        edge_length = math.hypot(edge[0], edge[1])
        if edge_length <= EPSILON:
            continue
        edge_axis = (edge[0] / edge_length, edge[1] / edge_length)
        perpendicular_axis = (edge_axis[1], -edge_axis[0])
        edge_offsets = [
            edge_axis[0] * point[0] + edge_axis[1] * point[1]
            for point in hull
        ]
        perpendicular_offsets = [
            perpendicular_axis[0] * point[0] + perpendicular_axis[1] * point[1]
            for point in hull
        ]
        edge_span = max(edge_offsets) - min(edge_offsets)
        perpendicular_span = max(perpendicular_offsets) - min(perpendicular_offsets)
        if edge_span <= EPSILON and perpendicular_span <= EPSILON:
            continue

        # 清扫线沿外接矩形长轴；较短轴是需要布置多条清扫线的跨行宽度。
        if edge_span >= perpendicular_span:
            sweep_axis = edge_axis
            sweep_span = edge_span
            cross_span = perpendicular_span
        else:
            sweep_axis = perpendicular_axis
            sweep_span = perpendicular_span
            cross_span = edge_span
        heading = _canonical_axis_heading(sweep_axis)
        # 主排序使用最小外接面积；面积相同时优先跨行宽度更小、清扫线更长的方向。
        # 最后用规范化航向消除正方形等对称区域的方向歧义，保证结果可复现。
        candidates.append((
            edge_span * perpendicular_span,
            cross_span,
            -sweep_span,
            heading,
        ))

    if not candidates:
        return None
    return min(candidates)[3]


def _group_sweep_angle(group, polygon):
    """
    获取一个区域的清扫方向。

    sweepDirection=manual 时使用人工指定的 sweepAngle；
    其他情况根据完整区域形状自动计算，与记录起点和记录方向无关。
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

    删除无效坐标；当末点回到首点30cm范围内时，把末点并入首点。
    几何求交会自动连接最后一个有效点和第一点，因此无需保留这段很短的
    人工闭合尾巴。传入的原始点列表不会被修改，前端点位查询仍返回全部记录点。
    """
    cleaned = []
    for point in points:
        xy = _point_xy(point)
        if xy is None:
            continue
        cleaned.append(point)

    # 至少保留三个多边形顶点。只处理“记录序列的末点回到首点附近”这一种
    # 明确的闭合重复，不改变区域中间点的近点、拐点和波动边界处理规则。
    if len(cleaned) > 3:
        first_xy = _point_xy(cleaned[0])
        last_xy = _point_xy(cleaned[-1])
        if _distance(first_xy, last_xy) <= BOUNDARY_CLOSURE_MERGE_CM:
            cleaned.pop()
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
    """计算满足最大线间距约束的最少清扫线条数。

    设区域跨行宽度为 D=span，清扫线数量为 N。第一条和最后一条分别落在
    区域两侧，因此 N 条线只有 N-1 个间隔：S_actual=D/(N-1)。要求
    S_actual<=S_max，移项得到 N>=D/S_max+1；N 必须为整数，所以向上取整。
    """
    if span <= EPSILON:
        return 0
    # 将配置转成浮点数，同时把异常的0或负值钳制为1cm，避免后面除零。
    spacing_limit = max(float(max_spacing), 1.0)
    # 公式：N_min=ceil(D/S_max)+1；max(2, ...)保证至少能形成一个有效间隔。
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
    # S_target=W-O_target；当前默认是116-53=63cm。
    spacing = max(float(target_spacing), 1.0)
    # 连续数学条件下的理论条数 N*=D/S_target+1，通常不是整数。
    ideal = span / spacing + 1.0
    # 没有最低重叠限制时至少需要两条线；配置max_spacing后会重新计算下限。
    minimum = 2
    if max_spacing is not None:
        # max_spacing=W-O_min；当前为116-30=86cm。
        # 从 N_min 开始枚举，保证每个候选的实际重叠都不会低于30cm。
        minimum = _minimum_lane_count(span, max_spacing)
    # 理论条数上方再保留一个整数候选，使入口/出口需要相反奇偶性时仍有选择，
    # 但不无限增加清扫线，避免只为改变出口方向而产生过密路线。
    maximum = max(minimum, int(math.ceil(ideal)) + 1)
    candidates = []
    for count in range(minimum, maximum + 1):
        # 对当前整数条数重新均分完整跨度，得到真实中心线间距。
        actual_spacing = span / float(count - 1)
        # 再做一次硬约束检查，防止浮点或边界条件把不合格候选带入后续评分。
        if max_spacing is not None and actual_spacing > float(max_spacing) + EPSILON:
            continue
        # 这里不筛奇偶，因此结果同时包含奇数和偶数方案。
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


def _recorded_boundary_sections_from_start(polygon_xy, direction, normal):
    """
    按人工记录顺序识别“侧边、外边界、侧边、外边界”四段折线。

    区域点按一圈边界依次记录，第一个点位于第一条侧边的起点；自动清扫方向又与
    第一段边界垂直。因此闭合边界的边序列应当依次呈现：

        第一侧边 -> 第一外边界 -> 第二侧边 -> 第二外边界

    这里不根据某个固定矩形坐标找四个角，而是枚举三个分割位置。每条边分别计算
    在清扫方向 direction 和跨行方向 normal 上的投影长度，选择与上述四段方向最
    匹配的分割。这样一条外边界中即使包含上下波动的多个记录点，也只影响折线形状，
    不会被当成需要平滑或删除的误点。

    返回四段原始点列，同时提供按 normal 投影排序、沿 direction 正向排列的两条
    外边界。第一侧边的完整首尾方向也用于抵消第一个浮动点对清扫角度的影响。
    """
    # count 是沿区域边界依次记录的有效点数；闭环的最后一条边由末点连回首点。
    count = len(polygon_xy)
    if count < 4:
        # 少于四点无法可靠分成“侧边/外边界/侧边/外边界”四个非空部分。
        return None

    # edges[i]=(沿清扫方向投影长度, 沿跨行方向投影长度)。
    # 它只描述第i条边更像“横向清扫边”还是“纵向侧边”，不会改变原始点位。
    edges = []
    edge_lengths = []
    for index in range(count):
        # 区域按闭环处理，所以最后一个点的下一点重新取 polygon_xy[0]。
        start = polygon_xy[index]
        end = polygon_xy[(index + 1) % count]
        # 当前记录边向量 e=P(i+1)-Pi。
        vector = (end[0] - start[0], end[1] - start[1])
        # |e|只用于识别重复点/零长度边，不参与后面的方向评分。
        length = math.hypot(vector[0], vector[1])
        if length <= EPSILON:
            # 重复点仍保留在原始列表中，但零长度边不应左右分段结果。
            sweep_component = 0.0
            side_component = 0.0
        else:
            # a_i=|d·e_i|：边在清扫方向d上的绝对投影长度。
            sweep_component = abs(direction[0] * vector[0] + direction[1] * vector[1])
            # b_i=|n·e_i|：边在跨行法向n上的绝对投影长度。
            side_component = abs(normal[0] * vector[0] + normal[1] * vector[1])
        edges.append((sweep_component, side_component))
        edge_lengths.append(length)

    # 前缀和让每一种四段切分都能用常数时间计算方向匹配分数。
    sweep_prefix = [0.0]
    side_prefix = [0.0]
    for sweep_component, side_component in edges:
        sweep_prefix.append(sweep_prefix[-1] + sweep_component)
        side_prefix.append(side_prefix[-1] + side_component)
    length_prefix = [0.0]
    for length in edge_lengths:
        length_prefix.append(length_prefix[-1] + length)

    def score(prefix, start, end):
        # prefix[end]-prefix[start] 等于半开区间[start,end)内全部投影分量之和。
        return prefix[end] - prefix[start]

    best = None
    # 至少为四段各保留一条边：0:a侧边，a:b外边界，b:c侧边，c:n外边界。
    for first_split in range(1, count - 2):
        for second_split in range(first_split + 1, count - 1):
            for third_split in range(second_split + 1, count):
                # 先用四段首尾弦判断“对边平行、邻边近似垂直”，这一步完全不依赖
                # 第一个短线段的方向，因此第一侧边有较大浮动时仍能找到真正角点。
                chord_vectors = (
                    (
                        polygon_xy[first_split][0] - polygon_xy[0][0],
                        polygon_xy[first_split][1] - polygon_xy[0][1],
                    ),
                    (
                        polygon_xy[second_split][0] - polygon_xy[first_split][0],
                        polygon_xy[second_split][1] - polygon_xy[first_split][1],
                    ),
                    (
                        polygon_xy[third_split][0] - polygon_xy[second_split][0],
                        polygon_xy[third_split][1] - polygon_xy[second_split][1],
                    ),
                    (
                        polygon_xy[0][0] - polygon_xy[third_split][0],
                        polygon_xy[0][1] - polygon_xy[third_split][1],
                    ),
                )
                chord_lengths = [math.hypot(vector[0], vector[1]) for vector in chord_vectors]
                if any(length <= EPSILON for length in chord_lengths):
                    continue
                unit_vectors = [
                    (vector[0] / length, vector[1] / length)
                    for vector, length in zip(chord_vectors, chord_lengths)
                ]

                def absolute_dot(left, right):
                    return abs(left[0] * right[0] + left[1] * right[1])

                def absolute_cross(left, right):
                    return abs(left[0] * right[1] - left[1] * right[0])

                # 对边同轴得分最大为2；四个相邻角接近90度得分最大为4。
                axis_score = (
                    absolute_dot(unit_vectors[0], unit_vectors[2])
                    + absolute_dot(unit_vectors[1], unit_vectors[3])
                    + sum(
                        absolute_cross(unit_vectors[index], unit_vectors[(index + 1) % 4])
                        for index in range(4)
                    )
                )
                section_path_lengths = (
                    length_prefix[first_split] - length_prefix[0],
                    length_prefix[second_split] - length_prefix[first_split],
                    length_prefix[third_split] - length_prefix[second_split],
                    length_prefix[count] - length_prefix[third_split],
                )
                # chord/path越接近1，说明切分没有把一个真实拐角错误包进同一段。
                straightness_score = sum(
                    chord_lengths[index] / max(section_path_lengths[index], EPSILON)
                    for index in range(4)
                )
                geometry_score = axis_score + straightness_score
                # 正确匹配分数：第一/第三段应沿normal，第二/第四段应沿direction。
                matched = (
                    score(side_prefix, 0, first_split)
                    + score(sweep_prefix, first_split, second_split)
                    + score(side_prefix, second_split, third_split)
                    + score(sweep_prefix, third_split, count)
                )
                # 反向分量是“侧边沿清扫方向、外边界沿跨行方向”的总长度，属于惩罚项。
                mismatched = (
                    score(sweep_prefix, 0, first_split)
                    + score(side_prefix, first_split, second_split)
                    + score(sweep_prefix, second_split, third_split)
                    + score(side_prefix, third_split, count)
                )
                # 主评分=matched-mismatched；相同时再比较matched以及更靠前的稳定切分位置。
                # 最后三个正整数只用于在选中候选后取回实际分割下标。
                candidate = (
                    geometry_score,
                    axis_score,
                    straightness_score,
                    matched - mismatched,
                    matched,
                    -first_split,
                    -second_split,
                    -third_split,
                    first_split,
                    second_split,
                    third_split,
                )
                if best is None or candidate[:8] > best[:8]:
                    best = candidate

    if best is None:
        return None
    first_split, second_split, third_split = best[8], best[9], best[10]

    # 第一段和第三段是两条短边；它们同样保留全部人工记录点，供换行路线使用。
    first_side = list(polygon_xy[:first_split + 1])
    second_side = list(polygon_xy[second_split:third_split + 1])
    # 第二段即第一条外边界，包含两个分割端点，保留其中全部人工记录点。
    first_boundary = list(polygon_xy[first_split:second_split + 1])
    # 第四段跨过闭环尾部，所以需要显式把首点补回，形成完整的另一条外边界。
    second_boundary = list(polygon_xy[third_split:]) + [polygon_xy[0]]
    if len(first_boundary) < 2 or len(second_boundary) < 2:
        return None

    def orient(points):
        # 比较折线首尾在清扫方向d上的投影，统一让预览路径沿+d排列。
        start_projection = direction[0] * points[0][0] + direction[1] * points[0][1]
        end_projection = direction[0] * points[-1][0] + direction[1] * points[-1][1]
        # 这里只统一数据方向；执行时是否反向由S形候选算法决定。
        return list(reversed(points)) if end_projection < start_projection else points

    first_boundary = orient(first_boundary)
    second_boundary = orient(second_boundary)
    if _path_length(first_boundary) <= EPSILON or _path_length(second_boundary) <= EPSILON:
        return None

    def average_normal_offset(points):
        # 用整条折线所有记录点在n方向上的平均投影判断它位于区域哪一侧。
        return sum(
            normal[0] * point[0] + normal[1] * point[1]
            for point in points
        ) / float(len(points))

    # 固定按跨行投影从小到大排列，确保lane-1/lane-N的顺序稳定且可复现。
    paths = [first_boundary, second_boundary]
    paths.sort(key=average_normal_offset)
    return {
        "firstSide": first_side,
        "firstBoundary": first_boundary,
        "secondSide": second_side,
        "secondBoundary": second_boundary,
        "boundaryPaths": paths,
        "splits": (first_split, second_split, third_split),
        "_quality": best[:5],
    }


def _recorded_boundary_sections(polygon_xy, direction, normal):
    """识别两条真实外边界，结果不依赖从哪个点开始或按哪个方向记录。"""
    points = list(polygon_xy or [])
    if len(points) < 4:
        return None

    best_result = None
    best_quality = None
    best_signature = None
    # 同时枚举原方向和反方向；每个方向再枚举所有循环起点。记录点只描述
    # 边界形状，因此同一闭环的循环移位和整体反转必须得到相同的外边界。
    for oriented in (points, list(reversed(points))):
        for start_index in range(len(oriented)):
            rotated = oriented[start_index:] + oriented[:start_index]
            result = _recorded_boundary_sections_from_start(rotated, direction, normal)
            if result is None:
                continue
            # 循环移位后，同一几何切分会因浮点加法顺序产生约1e-15的差异。
            # 先统一到稳定精度，避免这类无意义差异压过后续的方向匹配分数。
            quality = tuple(round(value, 9) for value in (result.get("_quality") or ()))
            signature = tuple(
                tuple((round(point[0], 8), round(point[1], 8)) for point in path)
                for path in (result.get("boundaryPaths") or [])
            )
            if (
                    best_result is None
                    or quality > best_quality
                    or (quality == best_quality and signature < best_signature)):
                best_result = result
                best_quality = quality
                best_signature = signature

    if best_result is not None:
        best_result.pop("_quality", None)
    return best_result


def _recorded_boundary_paths(polygon_xy, direction, normal):
    """兼容原调用方：只返回按跨行方向排序后的两条真实外边界折线。"""
    sections = _recorded_boundary_sections(polygon_xy, direction, normal)
    if sections is None:
        return None
    return sections.get("boundaryPaths")


def _generate_boundary_interpolated_quadrilateral_lanes(
        polygon, sweep_angle, lane_spacing_cm, force_even, max_spacing_cm=None,
        lane_count_override=None):
    """
    为按顺序记录的区域生成“外边界折线 + 内部直线”清扫线。

    程序根据完整区域形状得到长轴清扫方向，再从闭合记录边界中识别两条外边界：
    第一条、最后一条清扫线完整经过对应外边界的全部记录点，中间清扫线保持直线。

    这样既兼容原有四点梯形，也支持一条边上记录多个点、边界上下波动的区域。

    只有 sweep_angle 与“完整区域形状自动确定的方向”一致时才启用该方法；手工指定
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

    # 将清扫航向角θ转换为两个正交单位向量：
    # d=(sinθ,cosθ)沿小车清扫行驶方向；n=(cosθ,-sinθ)沿相邻清扫线的跨行方向。
    radians = math.radians(sweep_angle)
    direction = (math.sin(radians), math.cos(radians))
    normal = (math.cos(radians), -math.sin(radians))
    # 根据人工记录顺序识别两条外边界折线。识别失败时返回None，让调用方使用通用扫描算法。
    boundary_paths = _recorded_boundary_paths(polygon_xy, direction, normal)
    if boundary_paths is None:
        return None
    first_boundary, last_boundary = boundary_paths

    # q_i=n·P_i。每个q_i表示顶点Pi在跨行轴上的位置。
    offsets = [normal[0] * point[0] + normal[1] * point[1] for point in polygon_xy]
    # q_min/q_max给出区域在跨行轴上的两侧边界。
    min_offset = min(offsets)
    max_offset = max(offsets)
    # D=q_max-q_min。不能再假定第一个点和第二个点位于某条短边，否则同一四边形
    # 仅改变记录起点就会算出0宽度。使用完整投影跨度既与记录顺序无关，也能保证
    # 多点不规则区域最外侧的真实记录位置仍处于滚刷覆盖范围内。
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
    # N条线把整个区域跨度D分成N-1份，实际间距S_actual=D/(N-1)。
    actual_spacing = span / float(lane_count - 1)

    lanes = []
    for index in range(lane_count):
        # ratio=i/(N-1)，从0均匀变化到1；它表示第i条线处于整个跨行宽度的比例位置。
        ratio = index / float(lane_count - 1)
        if index == 0:
            # 第一条线不做直线化，直接使用第一条真实外边界折线及其全部记录点。
            lane_path = list(first_boundary)
            lane_type = "boundary"
            boundary_side = "min_offset"
        elif index == lane_count - 1:
            # 最后一条线同样保留另一侧真实外边界折线。
            lane_path = list(last_boundary)
            lane_type = "boundary"
            boundary_side = "max_offset"
        else:
            # 内部线使用固定法向偏移与多边形求交，因此不跟随外边界波动。
            # 内部第i条无限扫描线方程：n·P=q_i，q_i=q_min+(q_max-q_min)*ratio。
            offset = min_offset + (max_offset - min_offset) * ratio
            # 求该无限扫描线与闭合区域每条边的交点。
            intersections = _line_polygon_intersections(polygon_xy, normal, offset)
            # 交点沿清扫方向d排序，最前和最后交点构成区域内部的可清扫直线段。
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
        # dot(d,end-start)>0表示当前点列沿+d；若为负则反转完整点列。
        # 这里只统一预览数据方向，S形候选阶段仍可再次整体反转。
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

    # 与实际生成清扫线使用相同的跨行法向量，确保候选条数计算的D与几何结果一致。
    radians = math.radians(sweep_angle)
    normal = (math.cos(radians), -math.sin(radians))
    # 使用所有顶点在跨行轴上的完整跨度，禁止依赖“前两点恰好属于短边”的旧约定。
    offsets = [normal[0] * x + normal[1] * y for x, y in polygon_xy]
    span = max(offsets) - min(offsets)

    # 对每个满足最低重叠的整数N都真正生成一次几何线路；几何生成不足N条时丢弃候选。
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
        # S_actual=D/(N-1)。实际重叠由滚刷宽度减去实际中心线间距得到。
        actual_spacing = span / float(lane_count - 1)
        result.append({
            "laneCount": lane_count,
            "parity": "even" if lane_count % 2 == 0 else "odd",
            "laneSpacingCm": round(actual_spacing, 1),
            # O_actual=W-S_actual；候选生成阶段已确保O_actual>=O_min。
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

    # 有效滚刷宽度W=116厘米，是一次通过实际能够覆盖的横向宽度。
    brush_width = float(brush_width_cm)
    # 目标重叠O默认53厘米，并强制不能配置到30厘米以下。
    overlap = max(float(overlap_cm), MIN_OVERLAP_CM)
    # 目标中心线间距 S_target=W-O_target，当前为116-53=63cm。
    lane_spacing = max(brush_width - overlap, 1.0)
    # 最大允许间距 S_max=W-O_min=116-30=86cm；候选若超过它就违反最低重叠硬约束。
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
            # 这里得到的不只是一个预览结果，而是目标条数附近全部合法奇偶方案；
            # 任务生成器稍后还会结合当前区域的入口桥头和出口桥头做最终选择。
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
            # 预览阶段先展示最接近63cm目标间距的候选；最终执行路线允许在
            # laneCandidates中选择另一个奇偶方案，以便更靠近连接桥并减少无效转场。
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
