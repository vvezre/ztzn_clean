# coding=utf-8
"""把清扫线预览转换为小车按顺序执行的点到点任务。

本文件承担路线规划的“排序与落地”部分：

1. 为每个区域比较奇数/偶数清扫线、首线方向和整组正反顺序，生成 S 形候选。
2. 默认按区域编号1、2、3依次清扫，也可按前端 areaOrder 重新排序。
3. 把 groupLinks 当作双向图，通过连接点寻找任意区域之间的可达路径。
4. 区域内沿人工记录边界比较正反两个方向，不允许用斜线穿过区域。
5. 每两个连续路径点生成一条任务：mode=1 是清扫，mode=2 是对接、换行或连接桥移动。
6. 合并近似直行的普通移动点；清扫折线仅合并5cm内同位点和30cm内明确直行/折返近点。
7. 输出前强制检查起点、终点和每一段连续性，防止小车执行断裂路线。

坐标单位为厘米；航向角约定0度=+y、90度=+x、180度=-y、270度=-x。
"""
import itertools
import math
import time


# 经纬度与局部厘米坐标互相换算时使用的地球平均半径，单位米。
EARTH_RADIUS_M = 6371000.0
# 几何计算的近零阈值。
EPSILON_CM = 1e-6
# 区域边界和连接桥由人工遥控打点，RTK 抖动会让本来接近直线的点左右偏几厘米。
# 只有普通移动 mode=2 使用这条20厘米走廊；清扫线 mode=1 不使用容差合并。
TRANSFER_MAX_LATERAL_DEVIATION_CM = 20.0
# 边界投影点距离记录角点不超过20cm时直接吸附到角点，避免让小车为RTK抖动
# 产生的很短边界段额外停车；超过该距离时必须保留投影点，禁止斜切到角点。
BOUNDARY_CORNER_SNAP_CM = 20.0
# 只有当前点和目标点本身都贴在边界上，才允许使用“同边直达”。连接桥可能
# 位于区域外侧，即使两个最近投影碰巧落在同一边，也仍须经过人工边界锚点。
SAME_EDGE_DIRECT_MAX_OFFSET_CM = 2.0
# 清扫线求交端点距真实记录点5cm以内时直接使用记录点。这样既保留人工点，
# 又不会为了回到只差几厘米的角点额外生成停车掉头任务。
LANE_ENDPOINT_ANCHOR_SNAP_CM = 5.0

# 跨区域转场会把“区域边界路径、连接桥点、下一片区域边界路径”拼成一条折线。
# 不同几何步骤可能把同一个物理位置算成相差几厘米的两个点；如果原样落成任务，
# 小车会为这些 RTK/投影误差停车、掉头，甚至形成 A->B->A 的短距离往返。
# 5cm 内视为同一个物理连接点。相邻的两个普通中间点若相距不超过30cm，
# 会组成一个近点簇，并比较保留簇内哪个点能让前后局部路线最短；原点、目标点
# 和人工连接桥点属于保护点，永远不参加近点删除。孤立的短直角仍按转角规则保留。
TRANSITION_DUPLICATE_POINT_CM = 5.0
TRANSITION_NEAR_POINT_CM = 30.0
TRANSITION_BACKTRACK_DEG = 150.0

# 局部方向变化达到30度就认为是真实转弯，必须保留为任务端点，让小车停车重新转向。
# test12 中16～18度的采样摆动可以连续直行，而实际90度连接桥转角一定会被保留。
TRANSFER_HARD_TURN_DEG = 30.0
# 同一条边界清扫折线也使用30度作为“必须停车重新转向”的阈值。小于该角度时，
# 相邻点仍作为真实路径点保留，但执行器连续驶过，不关闭滚刷、不执行原地转向。
CLEAN_PATH_HARD_TURN_DEG = 30.0

# 人工记录的边界清扫折线中，两点不超过5cm时视为同一物理位置，
# 禁止拆成“停车转向 -> 行走1cm -> 再停车”的独立清扫任务。5～30cm
# 只有当其中一点在上下文中属于近似直行或明显原路折返时才删除；
# 真实的90度短边仍然保留。这些阈值只作用于边界 laneType=boundary。
CLEAN_DUPLICATE_POINT_CM = 5.0
CLEAN_NEAR_POINT_CM = 30.0
CLEAN_BACKTRACK_DEG = 150.0

# 小于3厘米的普通转场低于当前RTK点到点导航的有效执行尺度，不应单独形成
# “移动几乎看不见、但停车并重新下发下一任务”的伪任务。这里只处理mode=2
# 普通转场；清扫线本身以及带preserveStartStop/preserveEndStop的显式停车点
# 永远保留。被吸收后，下一任务的起点会回填到上一任务终点，并重新计算
# 航向与长度，因此真正的90度转向仍然存在，只是不再多停一次。
MIN_EXECUTABLE_TRANSFER_CM = 3.0

# 入口和出口首先决定清扫线奇偶及S形方向；在端点代价相近时，再用目标重叠
# 偏差区分候选。该权重只参与同一端点方案内的排序，不是覆盖硬限制。
OVERLAP_DEVIATION_WEIGHT = 2.0


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
    """
    将厘米坐标稳定地四舍五入为整数，并保证 Python 2/3 结果一致。

    Python 2 和 Python 3 对正好位于 ``.5`` 的数采用不同取整规则：前者通常
    远离零取整，后者采用“银行家舍入”。建模插值点出现半厘米时，这会让电脑
    预览与小车实际任务相差 1 cm，并进一步改变相邻转移段的角度和总长度。

    这里显式采用工程上常用的“四舍五入，半数远离零”，不再依赖解释器内置
    ``round`` 的版本差异。
    """
    number = float(value)
    if number >= 0:
        return int(math.floor(number + 0.5))
    return int(math.ceil(number - 0.5))


def _stable_cost_key(value):
    """Return a Python-2/3-stable integer key for route-cost comparison.

    Route candidates are scored with Euclidean floating-point distances.  Two
    geometrically symmetric candidates can therefore differ only in the last
    binary floating-point bits on Python 2 and Python 3.  That meaningless
    difference must not override the explicit deterministic tie breakers
    (lane order and first-lane direction) below.

    Costs use centimetres, so quantising to 0.001 cm keeps far more precision
    than the robot can physically execute while producing the same ordering on
    both interpreters.
    """
    return _round_int(float(value) * 1000.0)


def _normalize_heading(value):
    number = _number(value)
    if number is None:
        return None
    return round(number % 360.0, 1)


def _heading_from_xy(start, end):
    """
    使用相对坐标计算从 start 到 end 的航向角。

    坐标约定与 RTK 航向保持一致：0°=+y，90°=+x，180°=-y，270°=-x。
    start 和 end 重合时没有有效方向，返回 None。
    """
    # 先构造“任务起点 -> 任务终点”的方向向量。
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    if abs(dx) < EPSILON_CM and abs(dy) < EPSILON_CM:
        return None
    # 因0度沿+y，必须用atan2(dx,dy)；结果统一归一化到[0,360)。
    return _normalize_heading(math.degrees(math.atan2(dx, dy)))


def _length_cm(start, end):
    """使用勾股定理计算两个厘米坐标点之间的直线距离。"""
    return math.hypot(end[0] - start[0], end[1] - start[1])


def _polyline_length_cm(points):
    """计算有序点列的折线总长度；不足两个点时长度为0。"""
    return sum(
        _length_cm(points[index], points[index + 1])
        for index in range(len(points) - 1)
    )


class _CoordinateMapper(object):
    """
    以第一个有效建模点为原点，在相对坐标（cm）和 RTK 经纬度之间转换。

    为什么同时保留两套坐标：
    - x/y 用于路径规划、距离计算和前端绘图，单位是厘米。
    - lat/lon 用于小车 RTK 实际导航。

    原点从 groups 按顺序查找，第一个同时含有 x/y/lat/lon 的点即为建模原点。
    """
    def __init__(self, draft):
        self.origin_group_id = None
        self.origin_point_id = None
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
                    self.origin_group_id = group.get("id")
                    self.origin_point_id = point.get("id")
                    return {
                        "x": x,
                        "y": y,
                        "lat": lat,
                        "lon": lon,
                    }
        return None

    def xy_to_lat_lon(self, x, y):
        """
        把相对厘米坐标转换为经纬度。

        小范围建模采用局部平面近似：dy 换算纬度差，dx 根据当前纬度下的经度缩放换算经度差。
        """
        origin = self.origin
        # x/y是厘米，先减去模型原点并换成米。
        dx_m = (float(x) - origin["x"]) / 100.0
        dy_m = (float(y) - origin["y"]) / 100.0
        # 纬度方向直接使用“南北距离/地球半径”得到弧度差。
        lat = origin["lat"] + math.degrees(dy_m / EARTH_RADIUS_M)
        # 经度方向还要除以平均纬度的cos值，修正经线间距。
        mean_lat = math.radians((origin["lat"] + lat) / 2.0)
        cos_lat = math.cos(mean_lat)
        if abs(cos_lat) < 1e-12:
            raise ModelingTaskGenerationError("当前纬度无法换算经度")
        lon = origin["lon"] + math.degrees(dx_m / (EARTH_RADIUS_M * cos_lat))
        return round(lat, 8), round(lon, 8)

    def point_to_xy(self, point):
        """
        获取一个点的相对坐标。

        点已有 x/y 时直接使用；只有 lat/lon 时，再以建模原点反算局部厘米坐标。
        """
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
    """
    把两个相对坐标点转为一个机器人可执行的任务段。

    mode=1 表示开启清扫的作业段，mode=2 表示起点对接、换行或连接桥移动段。
    heading 统一由当前段起点指向终点计算。

    返回字段含义：
    - startX/startY/endX/endY：相对厘米坐标。
    - startLat/startLon/endLat/endLon：对应的 RTK 经纬度。
    - heading/angle：该段目标航向，单位度。
    - length：该段距离，单位厘米。
    - source：该段的来源，用于区分清扫、换行、连接桥和回原点。
    """
    # 小车任务和前端接口都使用整数厘米，因此先落到实际执行精度再判断长度。
    # 这样可以避免原始浮点坐标不同、但取整后起终点相同的0厘米伪任务。
    # 小车实际协议使用整数厘米；先统一取整，保证电脑和车载Python版本结果一致。
    # P_start=(x1,y1)、P_end=(x2,y2)统一按“半数远离零”取整到协议实际执行精度。
    start = (_round_int(start[0]), _round_int(start[1]))
    end = (_round_int(end[0]), _round_int(end[1]))
    # L=sqrt((x2-x1)^2+(y2-y1)^2)。
    # 段长L=sqrt((x2-x1)^2+(y2-y1)^2)，单位厘米。
    length = _length_cm(start, end)
    if length <= EPSILON_CM:
        # 取整后起终点重合的任务不可执行，也没有业务意义，直接不生成。
        return None
    # heading完全由本任务段起终点计算，不沿用上一段旧方向。
    # 规划航向heading=atan2(dx,dy)，0°沿+y、90°沿+x。
    heading = _heading_from_xy(start, end)
    # 同时生成RTK经纬度：x/y供规划与绘图，经纬度供真实导航和日志核对。
    # 相对坐标用于几何和前端绘图；经纬度用于真车RTK点到点导航。
    start_lat, start_lon = mapper.xy_to_lat_lon(start[0], start[1])
    end_lat, end_lon = mapper.xy_to_lat_lon(end[0], end[1])
    return {
        "id": task_id,
        "mode": int(mode),
        "areaNumber": int(area_number or 1),
        "startX": start[0],
        "startY": start[1],
        "endX": end[0],
        "endY": end[1],
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


def _refresh_task_geometry(task):
    """根据任务当前的整数厘米起终点，重新计算航向和长度。"""
    start = (_number(task.get("startX")), _number(task.get("startY")))
    end = (_number(task.get("endX")), _number(task.get("endY")))
    if None in start or None in end:
        return task
    length = _length_cm(start, end)
    heading = _heading_from_xy(start, end)
    task["heading"] = heading
    task["angle"] = heading
    task["length"] = _round_int(length)
    return task


def _reanchor_task_start(task, source_task):
    """把task起点吸附到source_task起点，并同步坐标、经纬度和任务几何。"""
    task = dict(task)
    task["startX"] = source_task.get("startX")
    task["startY"] = source_task.get("startY")
    task["startLat"] = source_task.get("startLat")
    task["startLon"] = source_task.get("startLon")
    return _refresh_task_geometry(task)


def _reanchor_task_end(task, source_task):
    """把task终点吸附到source_task终点，并同步坐标、经纬度和任务几何。"""
    task = dict(task)
    task["endX"] = source_task.get("endX")
    task["endY"] = source_task.get("endY")
    task["endLat"] = source_task.get("endLat")
    task["endLon"] = source_task.get("endLon")
    return _refresh_task_geometry(task)


def _remove_short_transfer_tasks(tasks):
    """
    吸收低于导航有效尺度的普通转场，同时保持整条任务链首尾连续。

    典型情况是记录点A6与浮点计算得到的首条清扫线起点实际只差零点几
    毫米，却因分别取整落在40cm和41cm，形成1cm独立任务。中间短转场
    被删除时，下一任务起点改为短转场起点；末尾短转场则把上一任务终点
    延伸到短转场终点。这样不会跳点，也不会删除下一段自身的目标方向。
    """
    compacted = [dict(task) for task in (tasks or [])]
    index = 0
    while index < len(compacted):
        task = compacted[index]
        start = (_number(task.get("startX")), _number(task.get("startY")))
        end = (_number(task.get("endX")), _number(task.get("endY")))
        length = None if None in start or None in end else _length_cm(start, end)
        removable = (
            int(task.get("mode") or 0) == 2
            and length is not None
            and length < MIN_EXECUTABLE_TRANSFER_CM
            and not task.get("preserveStartStop")
            and not task.get("preserveEndStop")
            and not task.get("preserveRecordedPath")
        )
        if not removable:
            index += 1
            continue

        previous = compacted[index - 1] if index > 0 else None
        following = compacted[index + 1] if index + 1 < len(compacted) else None

        # 中间或开头的短转场：下一任务从短转场起点直接出发。前一任务
        # 仍会正常停车；下一任务仍按自己的终点重新计算航向。
        if following is not None:
            compacted[index + 1] = _reanchor_task_start(following, task)
            del compacted[index]
            continue

        # 末尾短转场：上一任务直接落到原短转场终点，保证闭环终点不变。
        if previous is not None:
            compacted[index - 1] = _reanchor_task_end(previous, task)
            del compacted[index]
            index = max(0, index - 1)
            continue

        del compacted[index]
    return compacted


def _remove_short_task_backtracks(tasks):
    """删除跨任务类型边界形成的短距离 A->B->A 原路往返。

    一条边界清扫折线可能以 A->B 结束，而紧接着的自动转场又从 B 返回 A。
    这种折返无法在单独构建转场点列时发现，因为第一段属于 mode=1、第二段属于
    mode=2。只有两段都不超过30cm、首尾回到5cm内，并且属于“两个自动转场”
    或“连续边界清扫末段+自动转场”时才删除；独立清扫线、真实90度转角和较长
    连接桥不会命中。
    """
    compacted = [dict(task) for task in (tasks or [])]
    index = 0
    while index + 1 < len(compacted):
        outbound = compacted[index]
        inbound = compacted[index + 1]
        outbound_start = (_number(outbound.get("startX")), _number(outbound.get("startY")))
        outbound_end = (_number(outbound.get("endX")), _number(outbound.get("endY")))
        inbound_start = (_number(inbound.get("startX")), _number(inbound.get("startY")))
        inbound_end = (_number(inbound.get("endX")), _number(inbound.get("endY")))
        if None in outbound_start or None in outbound_end or None in inbound_start or None in inbound_end:
            index += 1
            continue
        contiguous = _is_same_point(outbound_end, inbound_start)
        returns_to_start = _length_cm(outbound_start, inbound_end) <= TRANSITION_DUPLICATE_POINT_CM
        outbound_length = _length_cm(outbound_start, outbound_end)
        inbound_length = _length_cm(inbound_start, inbound_end)
        short_pair = max(outbound_length, inbound_length) <= TRANSITION_NEAR_POINT_CM
        outbound_mode = int(outbound.get("mode") or 0)
        inbound_mode = int(inbound.get("mode") or 0)
        removable_pair = (
            (outbound_mode == 2 and inbound_mode == 2) or
            (
                outbound_mode == 1 and
                inbound_mode == 2 and
                bool(outbound.get("continuousPathId"))
            )
        )
        if not (contiguous and returns_to_start and short_pair and removable_pair):
            index += 1
            continue

        # 下一任务从折返前的A点继续，前一任务（若存在）本来就结束在A点。
        following = compacted[index + 2] if index + 2 < len(compacted) else None
        if following is not None:
            compacted[index + 2] = _reanchor_task_start(following, outbound)
        del compacted[index:index + 2]
        if index > 0:
            index -= 1
    return compacted


def _same_direction_collinear(left, right):
    """判断两个首尾相接的任务段能否作为一条直线连续执行。"""
    if int(left.get("mode") or 0) != int(right.get("mode") or 0):
        return False
    if left.get("preserveEndStop") or right.get("preserveStartStop"):
        return False
    if left.get("preserveRecordedPath") or right.get("preserveRecordedPath"):
        return False
    # 边界折线中的每个记录点都属于前端和小车共用的真实路径。即使三个点恰好
    # 共线，也不能在任务压缩阶段删掉；是否停车由 turnAtStart/stopAtEnd 决定。
    if left.get("continuousPathId") or right.get("continuousPathId"):
        return False

    left_end = (_number(left.get("endX")), _number(left.get("endY")))
    right_start = (_number(right.get("startX")), _number(right.get("startY")))
    if None in left_end or None in right_start or not _is_same_point(left_end, right_start):
        return False

    left_start = (_number(left.get("startX")), _number(left.get("startY")))
    right_end = (_number(right.get("endX")), _number(right.get("endY")))
    if None in left_start or None in right_end:
        return False
    left_vector = (left_end[0] - left_start[0], left_end[1] - left_start[1])
    right_vector = (right_end[0] - right_start[0], right_end[1] - right_start[1])
    left_length = math.hypot(left_vector[0], left_vector[1])
    right_length = math.hypot(right_vector[0], right_vector[1])
    if left_length <= EPSILON_CM or right_length <= EPSILON_CM:
        return False

    # 叉积为0表示两段共线；点积大于0表示方向相同，而不是到中间点后掉头。
    cross = left_vector[0] * right_vector[1] - left_vector[1] * right_vector[0]
    dot = left_vector[0] * right_vector[0] + left_vector[1] * right_vector[1]
    return abs(cross) <= EPSILON_CM * left_length * right_length and dot > 0


def _turn_angle_degrees(start, middle, end):
    """Return the smaller heading change at *middle* in the range 0..180."""
    # θ_in是start->middle航向，θ_out是middle->end航向。
    incoming = _heading_from_xy(start, middle)
    outgoing = _heading_from_xy(middle, end)
    if incoming is None or outgoing is None:
        return 0.0
    # 例如350度到10度的真实变化是20度，不是340度，因此取圆周上的较小夹角。
    difference = abs(incoming - outgoing) % 360.0
    return min(difference, 360.0 - difference)


def _point_to_segment_distance(point, start, end):
    """Shortest distance from *point* to the finite start/end segment."""
    # 线段方向向量AB。
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared <= EPSILON_CM:
        return _length_cm(point, start)
    # 投影比例 t=((P-A)·(B-A))/|B-A|^2。
    projection = (
        (point[0] - start[0]) * dx + (point[1] - start[1]) * dy
    ) / float(length_squared)
    # 限制到有限线段[0,1]，防止掉头点投影到线段延长线后被错误合并。
    projection = max(0.0, min(1.0, projection))
    # Q=A+t(B-A)是线段上离P最近的投影点，|P-Q|就是横向偏差。
    nearest = (start[0] + projection * dx, start[1] + projection * dy)
    return _length_cm(point, nearest)


def _rdp_point_indexes(points, start_index, end_index, tolerance_cm):
    """
    Return the indexes retained by a Ramer-Douglas-Peucker simplification.

    The distance is measured against a finite segment, so a point that doubles
    back beyond either endpoint cannot be silently merged into a straight run.
    """
    if end_index <= start_index + 1:
        return [start_index, end_index]

    # 先假设这一段可以由首点直接连到尾点。
    start = points[start_index]
    end = points[end_index]
    maximum_distance = -1.0
    maximum_index = None
    # 找出所有中间点中偏离首尾线段最远的一个。
    for index in range(start_index + 1, end_index):
        distance = _point_to_segment_distance(points[index], start, end)
        if distance > maximum_distance:
            maximum_distance = distance
            maximum_index = index

    # 最大偏差超过20厘米时必须保留该点，并递归检查它左右两段。
    if maximum_index is not None and maximum_distance > tolerance_cm:
        left = _rdp_point_indexes(points, start_index, maximum_index, tolerance_cm)
        right = _rdp_point_indexes(points, maximum_index, end_index, tolerance_cm)
        return left[:-1] + right
    # 所有中间点都在容差走廊内，执行时只保留首尾点即可。
    return [start_index, end_index]


def _merge_task_span(tasks, start_index, end_index):
    """Merge tasks[start_index:end_index] while preserving endpoint metadata."""
    span = tasks[start_index:end_index]
    merged = dict(span[0])
    last = span[-1]
    merged["endX"] = last.get("endX")
    merged["endY"] = last.get("endY")
    merged["endLat"] = last.get("endLat")
    merged["endLon"] = last.get("endLon")

    start = (_number(merged.get("startX")), _number(merged.get("startY")))
    end = (_number(merged.get("endX")), _number(merged.get("endY")))
    if None not in start and None not in end:
        merged["length"] = _round_int(_length_cm(start, end))
        heading = _heading_from_xy(start, end)
        merged["heading"] = heading
        merged["angle"] = heading

    if len(span) > 1:
        sources = []
        segment_count = 0
        for task in span:
            segment_count += int(task.get("mergedSegmentCount") or 1)
            task_sources = task.get("mergedSources") or [task.get("source")]
            for source in task_sources:
                if source and source not in sources:
                    sources.append(source)
        merged["mergedSources"] = sources
        merged["mergedSegmentCount"] = segment_count

    if last.get("preserveEndStop"):
        merged["preserveEndStop"] = True
    else:
        merged.pop("preserveEndStop", None)
    return merged


def _can_extend_transfer_run(previous, current):
    """Whether two neighbouring mode=2 tasks may be simplified as one chain."""
    if int(previous.get("mode") or 0) != 2 or int(current.get("mode") or 0) != 2:
        return False
    if previous.get("preserveEndStop") or current.get("preserveStartStop"):
        return False
    # 人工记录边界形成的换行折线必须逐点保留，不能进入20cm走廊简化。
    if previous.get("preserveRecordedPath") or current.get("preserveRecordedPath"):
        return False
    previous_end = (_number(previous.get("endX")), _number(previous.get("endY")))
    current_start = (_number(current.get("startX")), _number(current.get("startY")))
    if None in previous_end or None in current_start:
        return False
    return _is_same_point(previous_end, current_start)


def _simplify_transfer_run(tasks):
    """
    Simplify one continuous mode=2 run without removing real corners.

    First preserve every local turn of at least TRANSFER_HARD_TURN_DEG.  Then
    simplify the points between those hard corners using the 20 cm corridor.
    The resulting task endpoints are exactly the places where the existing
    point-to-point executor will stop and calculate a new heading.
    """
    if len(tasks) < 2:
        return list(tasks)

    first_start = (_number(tasks[0].get("startX")), _number(tasks[0].get("startY")))
    if None in first_start:
        return list(tasks)
    points = [first_start]
    for task in tasks:
        endpoint = (_number(task.get("endX")), _number(task.get("endY")))
        if None in endpoint:
            return list(tasks)
        points.append(endpoint)

    # 首点和尾点永远保留，中间先检查是否存在30度以上的真实转角。
    hard_indexes = [0]
    for index in range(1, len(points) - 1):
        if _turn_angle_degrees(points[index - 1], points[index], points[index + 1]) >= TRANSFER_HARD_TURN_DEG:
            hard_indexes.append(index)
    hard_indexes.append(len(points) - 1)

    # 每两个真实转角之间再执行20厘米RDP直线简化。
    retained_indexes = []
    for index in range(len(hard_indexes) - 1):
        section = _rdp_point_indexes(
            points,
            hard_indexes[index],
            hard_indexes[index + 1],
            TRANSFER_MAX_LATERAL_DEVIATION_CM,
        )
        if retained_indexes:
            section = section[1:]
        retained_indexes.extend(section)

    simplified = []
    for index in range(len(retained_indexes) - 1):
        start_point_index = retained_indexes[index]
        end_point_index = retained_indexes[index + 1]
        simplified.append(_merge_task_span(tasks, start_point_index, end_point_index))
    return simplified


def _compact_executable_tasks(tasks):
    """
    合并同模式、同方向、同一直线上的普通中间任务段。

    区域边界采样点、前端绘图点或连接桥中间点只有在真正形成拐角时才会留下；
    清扫模式发生变化或显式标记为必须停车的位置永远不会被跨越合并。
    """
    # 先删除跨“清扫末段/转场首段”边界的短距离原路折返，再吸收低于导航
    # 有效尺度的普通短转场；随后执行原有的共线合并和转角简化。
    tasks = _remove_short_task_backtracks(tasks)
    tasks = _remove_short_transfer_tasks(tasks)

    # Preserve the old exact-collinear behaviour for every task mode first.
    # This also keeps compatibility with existing saved plans and tests.
    exactly_compacted = []
    for raw_task in tasks or []:
        task = dict(raw_task)
        if exactly_compacted and _same_direction_collinear(exactly_compacted[-1], task):
            exactly_compacted[-1] = _merge_task_span(
                [exactly_compacted[-1], task],
                0,
                2,
            )
            continue
        exactly_compacted.append(task)

    # Only ordinary movement runs receive tolerance-based simplification.
    # Clean lines (mode=1) and boundaries carrying preserve*Stop remain exact.
    compacted = []
    index = 0
    while index < len(exactly_compacted):
        task = exactly_compacted[index]
        if int(task.get("mode") or 0) != 2:
            compacted.append(task)
            index += 1
            continue

        run = [task]
        next_index = index + 1
        while (
                next_index < len(exactly_compacted)
                and _can_extend_transfer_run(run[-1], exactly_compacted[next_index])):
            run.append(exactly_compacted[next_index])
            next_index += 1
        compacted.extend(_simplify_transfer_run(run))
        index = next_index

    for index, task in enumerate(compacted, start=1):
        task["id"] = index
    return compacted


def _lane_path_points(lane, reverse=False):
    """
    取出一条清扫线的完整路径。

    新的边界 lane 使用 pathPoints 保存人工记录的折线；内部 lane 以及历史任务仍然
    只有 start/end。reverse=True 时反转整个点列，而不只是交换首尾点，这样 S 形
    反向经过边界时仍按相反顺序逐点行驶。
    """
    points = []
    for point in lane.get("pathPoints") or []:
        if not isinstance(point, dict):
            continue
        xy = (_number(point.get("x")), _number(point.get("y")))
        if None in xy:
            continue
        if not points or not _is_same_point(points[-1], xy):
            points.append(xy)

    if len(points) < 2:
        start = (_number(lane.get("startX")), _number(lane.get("startY")))
        end = (_number(lane.get("endX")), _number(lane.get("endY")))
        if None in start or None in end:
            return None
        points = [start, end]
    return list(reversed(points)) if reverse else points


def _lane_points(lane, reverse=False):
    """兼容旧调用方：返回完整清扫路径的首点和尾点。"""
    points = _lane_path_points(lane, reverse=reverse)
    if not points:
        return None
    return points[0], points[-1]


def _draft_group_entry(draft, group_id, mapper):
    """
    取得区域的首次进入参考点。

    当前程序把用户记录的第二个区域点作为优先参考，
    用它决定第一条清扫线从哪一端开始；只有一个点时才退回使用第一点。
    """
    for group in draft.get("groups") or []:
        if group.get("id") != group_id:
            continue
        points = list(group.get("points") or [])
        if len(points) >= 2:
            return mapper.point_to_xy(points[1])
        if points:
            return mapper.point_to_xy(points[0])
    return None


def _iter_clean_segments(preview, draft, mapper):
    """
    按区域和清扫线顺序交替方向，生成 S 形清扫段。

    先比较区域进入参考点到第一条清扫线两端的距离，选较近的一端作为首次行驶方向。
    之后按 lane_index 奇偶交替反转起终点，形成“去一趟、回一趟”的蛇形路线。
    """
    for group in preview.get("groups") or []:
        group_id = group.get("groupId")
        area_number = group.get("areaNumber") or 1
        lanes = []
        for sub_area in group.get("subAreas") or []:
            lanes.extend(sub_area.get("lanes") or [])
        entry = _draft_group_entry(draft, group_id, mapper)
        reverse_first = False
        if lanes and entry is not None:
            first_points = _lane_points(lanes[0])
            if first_points is not None:
                reverse_first = (
                    _length_cm(entry, first_points[1])
                    < _length_cm(entry, first_points[0])
                )
        lane_index = 0
        for lane in lanes:
            points = _lane_points(
                lane,
                reverse=bool(lane_index % 2) ^ reverse_first,
            )
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


def _link_xy_points(link, mapper):
    """
    把一条连接桥中按顺序记录的全部点转为相对坐标。

    相邻重复点会被忽略，但中间过渡点会完整保留，因此连接桥不限于两个点。
    """
    # 优先使用按人工记录顺序保存的连接点数组。
    raw_points = list(link.get("points") or [])
    if not raw_points:
        raw_points = [link.get("startPoint"), link.get("endPoint")]
    # 全部连接点转换到模型统一坐标系，并仅删除相邻重复位置。
    points = []
    for point in raw_points:
        xy = mapper.point_to_xy(point)
        if xy is not None and (not points or not _is_same_point(points[-1], xy)):
            points.append(xy)
    return points


def _link_route_between(preview, from_group_id, to_group_id, mapper):
    """返回区域图中的连接桥边序列，每条边保留方向化后的全部桥点。"""
    # 同一区域不需要经过连接桥；缺少任一groupId也无法构图。
    if not from_group_id or not to_group_id or from_group_id == to_group_id:
        return []

    # graph[groupId]保存从该区域可以直接到达的全部“有向桥边”。
    # 原始groupLink是无向业务连接，但为了保留不同通行方向下的点位顺序，
    # 这里把每条桥拆成正向和反向两条有向边。
    graph = {}
    for link in preview.get("groupLinks") or []:
        # start/endGroupId定义桥连接的两个区域，不表示只能单向行驶。
        start_group_id = link.get("startGroupId")
        end_group_id = link.get("endGroupId")
        # link_points严格保持人工记录顺序，可包含两个以上的桥内中间点。
        link_points = _link_xy_points(link, mapper)
        if not start_group_id or not end_group_id or len(link_points) < 2:
            # 不完整连接不参与路径搜索，避免生成到一半中断的跨区路线。
            continue
        # 正向：startGroup -> endGroup，连接点使用原顺序。
        graph.setdefault(start_group_id, []).append({
            "fromGroupId": start_group_id,
            "toGroupId": end_group_id,
            "linkId": link.get("id"),
            "points": link_points,
        })
        # 反向：endGroup -> startGroup，连接点必须整体反转。
        graph.setdefault(end_group_id, []).append({
            "fromGroupId": end_group_id,
            "toGroupId": start_group_id,
            "linkId": link.get("id"),
            "points": list(reversed(link_points)),
        })

    # 使用广度优先搜索BFS：队列项=(当前区域, 从起点走到这里的桥边列表)。
    # 这样优先找到经过桥数量最少的可达路径，而不是允许区域间直接斜线连接。
    queue = [(from_group_id, [])]
    # visited防止区域图存在环时反复搜索同一节点。
    visited = set([from_group_id])
    while queue:
        # FIFO弹出保证按桥数量从少到多搜索。
        group_id, route_edges = queue.pop(0)
        for edge in graph.get(group_id, []):
            next_group_id = edge["toGroupId"]
            if next_group_id in visited:
                continue
            # 复制既有桥链并追加当前桥，不能原地修改其他队列分支的route_edges。
            next_route = route_edges + [edge]
            if next_group_id == to_group_id:
                # 第一次到达目标区域时即得到桥数量最少的连通路径。
                return next_route
            # 未到目标则标记并继续向下一层区域扩展。
            visited.add(next_group_id)
            queue.append((next_group_id, next_route))
    # 返回None而不是[]，用于区分“同一区域无需桥”和“两个区域不连通”。
    return None


def _link_points_between(preview, from_group_id, to_group_id, mapper):
    """
    在区域连接图中查找 from_group_id 到 to_group_id 的连接点序列。

    groupLinks 被构建为双向图，使用广度优先搜索寻找可达路径。
    反向通过同一条连接桥时，会自动反转连接点顺序。
    """
    route_edges = _link_route_between(preview, from_group_id, to_group_id, mapper)
    if route_edges is None:
        return None
    points = []
    for edge in route_edges:
        points.extend(edge.get("points") or [])
    return _dedupe_xy_path(points)


def _append_transition_tasks(
        tasks,
        current,
        target,
        area_number,
        mapper,
        task_id,
        preview,
        draft,
        from_group_id,
        to_group_id,
        source="modeling_transfer"):
    """
    在两段清扫线之间补充 mode=2 移动段。

    同一区域内直接从 current 连接到 target。
    跨区域时必须先查找用户记录的连接桥，然后按“当前点 -> 连接点... -> 目标点”拆成多个连续任务段。
    若两个区域没有可达的连接桥，直接报错，不允许机器人跨空直线行驶。
    """
    route_edges = _link_route_between(preview, from_group_id, to_group_id, mapper)
    # 连接桥点是人工明确记录的跨区通道，不能被30cm近点规则删掉。首尾任务点
    # 也会在简化函数内部自动保护；这里收集路线经过的全部桥点作为额外保护点。
    protected_transition_points = []
    for edge in route_edges or []:
        protected_transition_points.extend(edge.get("points") or [])
    if from_group_id != to_group_id and route_edges is None:
        raise ModelingTaskGenerationError(
            "modeling groups {} and {} are not connected".format(from_group_id, to_group_id)
        )
    if from_group_id == to_group_id and source in (
            "modeling_start_to_first_lane", "modeling_return_origin"):
        path_points = _group_anchor_transition_points(
            draft, from_group_id, current, target, mapper
        )
    elif from_group_id == to_group_id:
        path_points = [current, target]
    elif route_edges:
        first_points = route_edges[0].get("points") or []
        path_points = _group_anchor_transition_points(
            draft, from_group_id, current, first_points[0], mapper
        )
        for edge_index, edge in enumerate(route_edges):
            edge_points = edge.get("points") or []
            path_points = _dedupe_xy_path(path_points + edge_points)
            if edge_index + 1 < len(route_edges):
                next_points = route_edges[edge_index + 1].get("points") or []
                intermediate_group_id = edge.get("toGroupId")
                boundary = _group_anchor_transition_points(
                    draft,
                    intermediate_group_id,
                    edge_points[-1],
                    next_points[0],
                    mapper,
                )
                path_points = _dedupe_xy_path(path_points + boundary[1:])
        arrival = _group_anchor_transition_points(
            draft, to_group_id, path_points[-1], target, mapper
        )
        path_points = _dedupe_xy_path(path_points + arrival[1:])
    else:
        path_points = [current, target]
    # 跨模块拼接完成后再统一清理，才能识别“区域边界末点 -> 桥头 -> 同一边界末点”
    # 这种单个模块内部看不出的局部折返。清理后 _append_xy_path 会重新计算每段
    # 航向、长度以及 turnAtStart/stopAtEnd，不沿用旧任务的停车标记。
    path_points = _simplify_transition_path_points(
        path_points,
        protected_points=protected_transition_points,
    )
    return _append_xy_path(
        tasks,
        path_points,
        area_number,
        mapper,
        task_id,
        source,
        preserve_recorded_path=len(path_points) > 2,
    )


def _preview_group(preview, group_id):
    """根据 groupId 从路径预览中查找对应区域，找不到时返回 None。"""
    return next(
        (group for group in (preview.get("groups") or []) if group.get("groupId") == group_id),
        None,
    )


def _draft_point_map(draft):
    """将所有区域点和连接点建立为 id -> point 索引，便于路线策略按 ID 引用。"""
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


def _dedupe_xy_path(points):
    """Keep an ordered XY path while removing only consecutive duplicate positions."""
    result = []
    for point in points or []:
        if point is None:
            continue
        xy = (_number(point[0]), _number(point[1]))
        if None in xy:
            continue
        if not result or not _is_same_point(result[-1], xy):
            result.append(xy)
    return result


def _is_protected_transition_point(point, protected_points):
    """判断一个转场坐标是否属于原点、目标点或人工连接桥保护点。"""
    return any(_is_same_point(point, protected) for protected in protected_points or [])


def _collapse_near_transition_clusters(points, protected_points):
    """把30cm内的普通近点簇压缩为局部路线代价最低的一个代表点。

    对于 ``P -> A -> B -> N`` 且 ``|A-B| <= 30cm`` 的情况，同时比较：

    - 删除A、保留B：``|P-B| + |B-N|``；
    - 保留A、删除B：``|P-A| + |A-N|``。

    选择局部总长更小的方案。若连续三个以上普通点都两两相近，则把它们作为
    一个近点簇统一比较，避免结果依赖从左到右的扫描顺序。首点、尾点和人工
    连接桥点会把近点簇截断，因此真实入口、出口和桥头不会被删除。
    """
    if len(points) < 4:
        return list(points)

    result = [points[0]]
    index = 1
    final_index = len(points) - 1
    while index < final_index:
        point = points[index]
        if _is_protected_transition_point(point, protected_points):
            result.append(point)
            index += 1
            continue

        cluster_end = index
        while cluster_end + 1 < final_index:
            next_point = points[cluster_end + 1]
            if _is_protected_transition_point(next_point, protected_points):
                break
            if _length_cm(points[cluster_end], next_point) > TRANSITION_NEAR_POINT_CM:
                break
            cluster_end += 1

        if cluster_end == index:
            result.append(point)
            index += 1
            continue

        previous_point = result[-1]
        next_point = points[cluster_end + 1]
        candidates = []
        for candidate_index in range(index, cluster_end + 1):
            candidate = points[candidate_index]
            local_cost = (
                _length_cm(previous_point, candidate)
                + _length_cm(candidate, next_point)
            )
            # 成本按0.001cm量化，保证小车Python2和电脑Python3选择一致；
            # 完全同成本时优先保留记录顺序更靠后的点，删除前方多余短任务。
            candidates.append((
                _stable_cost_key(local_cost),
                -candidate_index,
                candidate,
            ))
        result.append(min(candidates, key=lambda item: item[:2])[2])
        index = cluster_end + 1

    result.append(points[-1])
    return _dedupe_xy_path(result)


def _simplify_transition_path_points(points, protected_points=None):
    """清理跨区域转场里的重复点、局部折返和短距离直线冗余点。

    该函数只用于规划器自动拼接出来的 mode=2 转场，不处理用户记录的区域清扫
    边界，也不处理最终回原点之外的整条任务序列。因此它不会把真实边界波动抹平。

    处理顺序：

    1. 精确重复点先由 ``_dedupe_xy_path`` 删除；
    2. 新点回到最近几个历史点 5cm 内时，删除中间局部环路，例如 A->B->A；
    3. 30cm内、连续出现的普通中间点按局部路线总代价选择一个代表点；
    4. 其余孤立短段只在小于30度的近似直行或大于150度的明显折返时合并；
    5. 始终把首尾重新固定为原始 current/target，保证任务接口坐标不漂移。
    """
    normalized = _dedupe_xy_path(points)
    if len(normalized) < 3:
        return normalized

    # 原始首尾和所有人工连接桥点都属于不可删除点。首尾自动加入保护集合，
    # 即使调用方没有传protected_points，也不会因距离近而丢失任务真实端点。
    protected = list(protected_points or [])
    protected.extend((normalized[0], normalized[-1]))

    # 先做局部环路消除。只回看最近8个点，避免把较长、合法的闭环返回路线误判
    # 为局部重复；连接桥附近的 A->B->A 和 A->B->C->B 都会被消除。但若
    # 中间已经经过人工连接桥保护点，则禁止截断该段，避免把真实桥路线删掉。
    loop_free = []
    for point in normalized:
        match_index = None
        if not _is_protected_transition_point(point, protected):
            search_start = max(0, len(loop_free) - 8)
            for index in range(len(loop_free) - 1, search_start - 1, -1):
                crosses_protected_point = any(
                    _is_protected_transition_point(item, protected)
                    for item in loop_free[index + 1:]
                )
                if crosses_protected_point:
                    continue
                if _length_cm(loop_free[index], point) <= TRANSITION_DUPLICATE_POINT_CM:
                    match_index = index
                    break
        if match_index is not None:
            loop_free = loop_free[:match_index + 1]
            continue
        loop_free.append(point)

    loop_free = _collapse_near_transition_clusters(loop_free, protected)

    if len(loop_free) < 3:
        result = loop_free
    else:
        result = list(loop_free)
        changed = True
        while changed and len(result) >= 3:
            changed = False
            index = 1
            while index < len(result) - 1:
                previous_point = result[index - 1]
                current_point = result[index]
                next_point = result[index + 1]
                if _is_protected_transition_point(current_point, protected):
                    index += 1
                    continue
                incoming_length = _length_cm(previous_point, current_point)
                outgoing_length = _length_cm(current_point, next_point)
                turn_angle = _turn_angle_degrees(previous_point, current_point, next_point)

                # 小于5cm是同一个物理点；5～30cm只有结合上下文才能删除。
                duplicate = min(incoming_length, outgoing_length) <= TRANSITION_DUPLICATE_POINT_CM
                near_segment = min(incoming_length, outgoing_length) <= TRANSITION_NEAR_POINT_CM
                nearly_straight = turn_angle < TRANSFER_HARD_TURN_DEG
                clear_backtrack = turn_angle >= TRANSITION_BACKTRACK_DEG
                if duplicate or (near_segment and (nearly_straight or clear_backtrack)):
                    del result[index]
                    changed = True
                    if index > 1:
                        index -= 1
                    continue
                index += 1

    # 环路消除可能用“距离原点不足5cm”的历史点代替真实终点。重新固定首尾，
    # 使前端预览、保存坐标和小车最终停车目标仍与本次规划请求完全一致。
    if not result:
        return [normalized[0], normalized[-1]]
    result[0] = normalized[0]
    if len(result) == 1:
        if not _is_same_point(result[0], normalized[-1]):
            result.append(normalized[-1])
    else:
        result[-1] = normalized[-1]
    return _dedupe_xy_path(result)


def _pin_path_endpoints(path, current, target):
    """把几何投影路线的首尾重新钉到真实任务端点。

    边界投影点与清扫线端点在浮点坐标中可能只相差十亿分之一厘米，几何上
    应视为同一点，但它们若刚好分处 0.5cm 的两侧，独立转换为整数协议坐标
    后可能分别变成 335cm 和 336cm。这里在生成任务前统一替换首尾点，确保
    前一段的终点与后一段的起点使用完全相同的原始数值，杜绝 1cm 假断点。
    """
    pinned = list(path or [])
    if not pinned:
        return _dedupe_xy_path([current, target])
    pinned[0] = current
    pinned[-1] = target
    return _dedupe_xy_path(pinned)


def _group_recorded_xy(draft, group_id, mapper):
    """Return the area's recorded route anchors in the same order as manual capture."""
    for group in draft.get("groups") or []:
        if group.get("id") != group_id:
            continue
        return _dedupe_xy_path([
            mapper.point_to_xy(point)
            for point in (group.get("points") or [])
        ])
    return []


def _cyclic_anchor_indexes(start_index, end_index, count, step):
    indexes = [start_index]
    current = start_index
    while current != end_index:
        current = (current + step) % count
        indexes.append(current)
    return indexes


def _nearest_boundary_projection(point, anchors):
    """把任意位置投影到记录区域的闭合边界，返回最近投影点及所在边。"""
    # best按(桥头到投影距离, 边序号, 边内比例)排序，保证结果稳定可复现。
    best = None
    for index in range(len(anchors)):
        # 每条记录边均参与投影，末点到首点同样作为闭环边。
        start = anchors[index]
        end = anchors[(index + 1) % len(anchors)]
        delta_x = end[0] - start[0]
        delta_y = end[1] - start[1]
        denominator = delta_x * delta_x + delta_y * delta_y
        if denominator <= EPSILON_CM:
            ratio = 0.0
        else:
            # t=((P-A)·(B-A))/|B-A|²，是P在线段AB方向上的投影比例。
            ratio = (
                (point[0] - start[0]) * delta_x
                + (point[1] - start[1]) * delta_y
            ) / denominator
            ratio = max(0.0, min(1.0, ratio))
        # Q=A+t(B-A)，Q是当前有限边上距离桥头point最近的位置。
        projection = (
            start[0] + delta_x * ratio,
            start[1] + delta_y * ratio,
        )
        candidate = (
            _length_cm(point, projection),
            index,
            ratio,
            projection,
        )
        if best is None or candidate[:3] < best[:3]:
            best = candidate
    return {
        "edgeIndex": best[1],
        "ratio": best[2],
        "point": best[3],
    }


def _boundary_anchor_candidates(projection, anchors):
    """
    返回投影所在边的两个端点，并把较近端点放在前面。

    投影点相当于把原边界边切成了两段。例如连接桥投影 Q 位于 A2--A3
    中间时，从 A1 前往 Q 既可以沿 A1--A2--Q，也可以沿另一侧边界到
    A3--Q。旧逻辑只返回离 Q 最近的 A3，会错误地排除 A2 方向，导致
    小车先到 A3 再折返 Q。两个端点都作为候选后，由完整边界路径长度
    自动选择较短方向，因此同一规则适用于任意边、任意倾斜区域和返程。
    """
    for index, anchor in enumerate(anchors):
        if _is_same_point(anchor, projection["point"]):
            return [index]
    edge_index = projection["edgeIndex"]
    next_index = (edge_index + 1) % len(anchors)

    # 投影已经落在角点吸附范围内时仍按该角点处理，避免桥头在角点附近
    # 因几厘米测量误差绕过人工记录的明确拐点。只有真正位于边中部时才
    # 把边切开，并同时比较经两个端点到达投影点的完整路径。
    edge_distance = _length_cm(projection["point"], anchors[edge_index])
    next_distance = _length_cm(projection["point"], anchors[next_index])
    if edge_distance <= BOUNDARY_CORNER_SNAP_CM:
        return [edge_index]
    if next_distance <= BOUNDARY_CORNER_SNAP_CM:
        return [next_index]
    if projection["ratio"] <= 0.5:
        return [edge_index, next_index]
    return [next_index, edge_index]


def _boundary_projection_waypoint(projection, anchor):
    """投影离角点较远时返回投影点，靠近角点时允许直接吸附。"""
    point = projection["point"]
    if _length_cm(point, anchor) > BOUNDARY_CORNER_SNAP_CM:
        return point
    return None


def _group_anchor_transition_points(draft, group_id, current, target, mapper):
    """
    Connect two positions through the area's manually recorded anchors.

    记录点是允许行驶的边界锚点，因此不能从 current 斜穿区域到 target。
    边界本身按闭环处理，同时比较人工记录正向和反向两条路径，选择总长度更短
    的合法方向。后续直线简化只删除近似共线中间点，不会改变所选边界方向。
    """
    if current is None or target is None:
        return []
    if _is_same_point(current, target):
        return [current]
    # anchors是该区域人工记录的边界闭环；顺序不能打乱，因为正/反向路线依赖它。
    anchors = _group_recorded_xy(draft, group_id, mapper)
    if len(anchors) < 2:
        return [current, target]

    # 清扫线端点或连接点可能位于两个人工角点之间。先找到最近边界和最近
    # 角点，再比较沿记录边界正向、反向行驶的总长度。投影离角点超过容差
    # 时必须加入路径，形成“桥头 -> 投影点 -> 角点”，禁止斜切到角点。
    # 将当前点和目标点分别落到最近的记录边界上，支持桥位于边中部而不是角点。
    current_projection = _nearest_boundary_projection(current, anchors)
    target_projection = _nearest_boundary_projection(target, anchors)
    # candidates保存所有“从哪个边端进入/离开、沿闭环正向还是反向”的合法路线。
    candidates = []
    start_indexes = _boundary_anchor_candidates(current_projection, anchors)
    end_indexes = _boundary_anchor_candidates(target_projection, anchors)
    # 两点落在同一条记录边上时，最短且不会反向的合法路径就是沿这条边直接
    # 前往目标投影。这里不能再使用“距离角点20cm就吸附”的候选结果决定是否
    # 绕角点：清扫线起点可能恰好距角点17cm，若先吸附到角点再回到起点，就会
    # 生成“越过目标 -> 原地掉头 -> 返回目标”的错误换行任务。即使两点本身正好
    # 是该边两端的角点，直接连接也仍然完整地沿着这条真实边界行驶。
    current_boundary_offset = _length_cm(current, current_projection["point"])
    target_boundary_offset = _length_cm(target, target_projection["point"])
    current_edge_index = current_projection["edgeIndex"]
    target_edge_index = target_projection["edgeIndex"]
    # 角点同时属于前后两条相邻边。最近投影为保证稳定会选择编号较小的边，
    # 例如右上角可能归到“上边”，而紧接着的换行目标归到“右边”。只比较边号
    # 会误判成不同边并绕到右下角；若一个投影点正好是另一条边的端点，也应
    # 视为可以从这个共享角点直接进入该边。
    current_is_target_edge_endpoint = any(
        _is_same_point(current_projection["point"], anchors[index])
        for index in (
            target_edge_index,
            (target_edge_index + 1) % len(anchors),
        )
    )
    target_is_current_edge_endpoint = any(
        _is_same_point(target_projection["point"], anchors[index])
        for index in (
            current_edge_index,
            (current_edge_index + 1) % len(anchors),
        )
    )
    same_or_shared_edge = (
        current_edge_index == target_edge_index
        or current_is_target_edge_endpoint
        or target_is_current_edge_endpoint
    )
    if (
            same_or_shared_edge
            and current_boundary_offset <= SAME_EDGE_DIRECT_MAX_OFFSET_CM
            and target_boundary_offset <= SAME_EDGE_DIRECT_MAX_OFFSET_CM):
        direct_path = _dedupe_xy_path([
            current,
            current_projection["point"],
            target_projection["point"],
            target,
        ])
        return _pin_path_endpoints(direct_path, current, target)
    for start_priority, start_index in enumerate(start_indexes):
        for end_priority, end_index in enumerate(end_indexes):
            # 沿记录点正序(+1)和逆序(-1)各生成一条闭环边界候选，避免固定绕行方向。
            forward_indexes = _cyclic_anchor_indexes(start_index, end_index, len(anchors), 1)
            reverse_indexes = _cyclic_anchor_indexes(start_index, end_index, len(anchors), -1)
            for direction_priority, indexes in enumerate((forward_indexes, reverse_indexes)):
                start_projection_point = _boundary_projection_waypoint(
                    current_projection, anchors[start_index]
                )
                end_projection_point = _boundary_projection_waypoint(
                    target_projection, anchors[end_index]
                )
                # 路线必须从真实current开始，随后经过投影点、人工锚点，最终到真实target。
                path = [current]
                if start_projection_point is not None:
                    path.append(start_projection_point)
                path.extend(anchors[index] for index in indexes)
                if end_projection_point is not None:
                    path.append(end_projection_point)
                path.append(target)
                path = _dedupe_xy_path(path)
                # L_path=Σ|P(i+1)-Pi|，用完整折线长度比较，而不是只看首尾直线距离。
                length = sum(
                    _length_cm(path[index], path[index + 1])
                    for index in range(len(path) - 1)
                )
                candidates.append((
                    length,
                    start_priority,
                    end_priority,
                    direction_priority,
                    path,
                ))
    # 首先选择总长度最短的合法边界路线；完全同长时使用候选优先级稳定决策。
    selected = min(candidates, key=lambda item: item[:4])[4]
    return _pin_path_endpoints(selected, current, target)


def _group_lane_candidate_sets(group):
    """组合一个区域内各子区域的合法清扫线数量候选。"""
    option_sets = []
    for sub_area in group.get("subAreas") or []:
        options = list(sub_area.get("laneCandidates") or [])
        if not options and sub_area.get("lanes"):
            spacing = _number(sub_area.get("laneSpacingCm"))
            options = [{
                "laneCount": len(sub_area.get("lanes") or []),
                "laneSpacingCm": spacing,
                "actualOverlapCm": sub_area.get("actualOverlapCm"),
                "lanes": list(sub_area.get("lanes") or []),
            }]
        if options:
            option_sets.append((sub_area.get("id"), options))
    if not option_sets:
        return []

    combinations = []
    for selected_options in itertools.product(*[item[1] for item in option_sets]):
        lanes = []
        selections = []
        for (sub_area_id, _), option in zip(option_sets, selected_options):
            option_lanes = list(option.get("lanes") or [])
            lanes.extend(option_lanes)
            selections.append({
                "subAreaId": sub_area_id,
                "laneCount": len(option_lanes),
                "laneSpacingCm": option.get("laneSpacingCm"),
                "actualOverlapCm": option.get("actualOverlapCm"),
            })
        if lanes:
            combinations.append({
                "lanes": lanes,
                "subAreas": selections,
            })
    return combinations


def _select_group_lane_segments(preview, group_id, entry, exit_point):
    """
    为指定区域选择一组最适合入口和出口的 S 形清扫顺序。

    对一组清扫线尝试四种组合：
    1. 按原线序，第一条正向。
    2. 按原线序，第一条反向。
    3. 整组清扫线逆序，第一条正向。
    4. 整组清扫线逆序，第一条反向。

    评分为“入口到首线起点距离 + 末线终点到出口距离”，选择总距离最小的组合。
    """
    # 先取得目标区域的预览数据，其中包含每个子区域的全部合法奇偶清扫线候选。
    group = _preview_group(preview, group_id)
    if group is None:
        raise ModelingTaskGenerationError("route policy references an unknown modeling group")
    # 一个区域可能含多个subArea；笛卡尔积组合出它们的候选条数组合。
    lane_sets = _group_lane_candidate_sets(group)
    if not lane_sets:
        raise ModelingTaskGenerationError("route policy group has no cleaning lanes")

    # O_target用于计算“实际重叠偏离53cm多少”，它是软优化目标，不替代30cm硬限制。
    target_overlap = _number((preview.get("config") or {}).get("overlapCm")) or 0.0
    # candidates将保存：线数方案、整组正反顺序、首线方向，共同形成的所有S形方案。
    candidates = []
    for lane_set in lane_sets:
        # lanes已经按跨行方向排序；每个lane可能是两点直线，也可能是多点边界折线。
        lanes = lane_set["lanes"]
        for reverse_order in (False, True):
            # reverse_order决定先扫区域哪一侧，等价于整组线序正序/倒序。
            ordered = list(reversed(lanes)) if reverse_order else list(lanes)
            for reverse_first in (False, True):
                # reverse_first决定第一条线从哪端出发；后续线按奇偶自动交替方向。
                segments = []
                for index, lane in enumerate(ordered):
                    # 第i条是否反向：reverse=(i mod 2) XOR reverse_first。
                    # 所以相邻清扫线方向必然相反，天然形成S形。
                    path = _lane_path_points(
                        lane,
                        reverse=bool(index % 2) ^ reverse_first,
                    )
                    if not path:
                        continue
                    # start/end参与入口、出口和换行代价计算；path保留整条边界折线。
                    segments.append({
                        "groupId": group_id,
                        "areaNumber": group.get("areaNumber") or 1,
                        "start": path[0],
                        "end": path[-1],
                        "path": path,
                        "sourceId": lane.get("id"),
                        "laneType": lane.get("laneType") or "interior",
                    })
                if not segments:
                    continue
                # d_entry=|入口桥头(或原点)-首线起点|。
                entry_distance = _length_cm(entry, segments[0]["start"])
                # d_exit=|末线终点-出口桥头(或回程参考点)|。
                exit_distance = _length_cm(segments[-1]["end"], exit_point)
                # 区域内部相邻清扫线之间的换行总距离。
                transfer_distance = sum(
                    _length_cm(segments[index]["end"], segments[index + 1]["start"])
                    for index in range(len(segments) - 1)
                )
                # 全部清扫线长度；边界折线按每个相邻点的长度求和，而不是首尾直线。
                clean_distance = sum(
                    _polyline_length_cm(
                        segment.get("path") or [segment["start"], segment["end"]]
                    )
                    for segment in segments
                )
                # P=Σ|O_actual-O_target|；用于端点代价相同或非常接近时偏向目标重叠。
                overlap_deviation = sum(
                    abs((_number(item.get("actualOverlapCm")) or target_overlap) - target_overlap)
                    for item in lane_set.get("subAreas") or []
                )
                candidates.append({
                    # E=d_entry+d_exit。桥头位置主要通过E决定奇偶、先扫哪侧和首线方向。
                    "endpointCost": entry_distance + exit_distance,
                    # R=E+换行距离+清扫距离，用于进一步避免明显绕行。
                    "routeCost": entry_distance + exit_distance + transfer_distance + clean_distance,
                    "overlapDeviation": overlap_deviation,
                    "entryDistance": entry_distance,
                    "exitDistance": exit_distance,
                    "reverseOrder": reverse_order,
                    "reverseFirst": reverse_first,
                    "segments": segments,
                    "subAreas": lane_set.get("subAreas") or [],
                })
    if not candidates:
        raise ModelingTaskGenerationError("route policy group has no usable cleaning lanes")
    # 排序采用元组字典序：前一个指标完全相同/量化后相同，才比较下一个指标。
    # 因此优先级明确为：桥头端点代价 -> 重叠偏差 -> 总路线长度 -> 更少线数 -> 稳定方向。
    selected = min(
        candidates,
        key=lambda candidate: (
            # 1) 首先让区域路线尽量从入口附近开始、在出口附近结束。
            _stable_cost_key(candidate["endpointCost"]),
            _stable_cost_key(
                # 2) 再偏向实际重叠接近53cm的方案；权重2只改变该项量纲。
                candidate["overlapDeviation"] * OVERLAP_DEVIATION_WEIGHT
            ),
            # 3) 再比较包含清扫和换行的区域内总距离。
            _stable_cost_key(candidate["routeCost"]),
            # 4) 完全同价时用更少清扫线，避免无意义增加路线。
            len(candidate["segments"]),
            # 5) 最后两个布尔值只负责跨Python版本获得稳定、可复现结果。
            candidate["reverseOrder"],
            candidate["reverseFirst"],
        ),
    )
    return {
        "segments": selected["segments"],
        "selection": {
            "groupId": group_id,
            "areaNumber": group.get("areaNumber") or 1,
            "laneCount": len(selected["segments"]),
            "entryDistanceCm": round(selected["entryDistance"], 1),
            "exitDistanceCm": round(selected["exitDistance"], 1),
            "reverseOrder": selected["reverseOrder"],
            "reverseFirst": selected["reverseFirst"],
            "subAreas": selected["subAreas"],
        },
    }


def _group_lane_segments(preview, group_id, entry, exit_point):
    """兼容旧调用方，只返回选中的S形清扫段。"""
    return _select_group_lane_segments(preview, group_id, entry, exit_point)["segments"]


def _clean_turn_can_remove_point(previous_point, current_point, next_point):
    """判断一个5～30cm近点能否在不破坏真实边界的前提下删除。

    ``previous -> current -> next`` 的方向变化小于30度，说明current
    只是近似直线上的密集采样点；方向变化大于等于150度，说明它
    形成了很短的原路折返。两种情况都可以删除。30～150度之间视为
    真实边界拐点，即使相邻段很短也不能抹掉。
    """
    turn_angle = _turn_angle_degrees(previous_point, current_point, next_point)
    return (
        turn_angle < CLEAN_PATH_HARD_TURN_DEG
        or turn_angle >= CLEAN_BACKTRACK_DEG
    )


def _collapse_duplicate_clean_point_clusters(points):
    """将5cm内连续记录点簇压缩成一个代表点。

    清扫线首点和尾点同时是区域进出口，所以点簇位于首端时保留
    原首点，位于尾端时保留原尾点，防止清扫线与前后转场脱节。
    中间点簇则比较“前一点 -> 候选点 -> 后一点”的局部总距离，
    保留路线代价最小的候选点。
    """
    if len(points) < 2:
        return list(points)

    collapsed = []
    index = 0
    final_index = len(points) - 1
    while index <= final_index:
        cluster_end = index
        while (
                cluster_end < final_index
                and _length_cm(points[cluster_end], points[cluster_end + 1])
                <= CLEAN_DUPLICATE_POINT_CM):
            cluster_end += 1

        if cluster_end == index:
            collapsed.append(points[index])
            index += 1
            continue

        if index == 0:
            # 首端点簇保留已被区域顺序算法选中的原清扫起点。
            representative = points[index]
        elif cluster_end == final_index:
            # 尾端点簇保留原清扫终点，让下一段转场从真实出口出发。
            representative = points[cluster_end]
        else:
            previous_point = collapsed[-1]
            next_point = points[cluster_end + 1]
            candidates = []
            for candidate_index in range(index, cluster_end + 1):
                candidate = points[candidate_index]
                local_cost = (
                    _length_cm(previous_point, candidate)
                    + _length_cm(candidate, next_point)
                )
                candidates.append((
                    _stable_cost_key(local_cost),
                    -candidate_index,
                    candidate,
                ))
            representative = min(candidates, key=lambda item: item[:2])[2]

        collapsed.append(representative)
        index = cluster_end + 1
    return _dedupe_xy_path(collapsed)


def _simplify_clean_path_points(points):
    """在拆分mode=1任务前，清理边界清扫折线里的无意义近点。

    处理顺序：

    1. 删除坐标完全相同的相邻点；
    2. 将相邻距离不超过5cm的点簇视为同一物理位置；
    3. 对5～30cm短段，只删除近似直行中间点或明显折返点；
    4. 每次删点后重新检查相邻关系，直到结果稳定。

    首尾点不参与5～30cm的上下文删除，真实短直角也会完整保留。
    """
    result = _collapse_duplicate_clean_point_clusters(_dedupe_xy_path(points))
    if len(result) < 3:
        return result

    changed = True
    while changed and len(result) >= 3:
        changed = False
        pair_index = 0
        while pair_index < len(result) - 1:
            left_point = result[pair_index]
            right_point = result[pair_index + 1]
            distance = _length_cm(left_point, right_point)
            if distance > CLEAN_NEAR_POINT_CM:
                pair_index += 1
                continue

            removable_indexes = []
            if (
                    pair_index > 0
                    and _clean_turn_can_remove_point(
                        result[pair_index - 1], left_point, right_point
                    )):
                removable_indexes.append(pair_index)
            if (
                    pair_index + 2 < len(result)
                    and _clean_turn_can_remove_point(
                        left_point, right_point, result[pair_index + 2]
                    )):
                removable_indexes.append(pair_index + 1)

            if not removable_indexes:
                pair_index += 1
                continue

            # 两点都可删时，对比删除后整条折线长度；代价相同时
            # 优先删记录顺序靠前的点，使 Python 2/3 结果完全一致。
            choices = []
            for remove_index in removable_indexes:
                candidate = result[:remove_index] + result[remove_index + 1:]
                choices.append((
                    _stable_cost_key(_polyline_length_cm(candidate)),
                    remove_index,
                    candidate,
                ))
            result = min(choices, key=lambda item: item[:2])[2]
            changed = True
            pair_index = max(0, pair_index - 1)

    return _dedupe_xy_path(result)


def _simplify_clean_segments(segments):
    """清理所有边界清扫线，并把线段首尾同步到清理后的点列。"""
    simplified_segments = []
    for original in segments or []:
        segment = dict(original)
        path = list(segment.get("path") or [segment.get("start"), segment.get("end")])
        path = _dedupe_xy_path(path)
        if segment.get("laneType") == "boundary" and len(path) >= 3:
            path = _simplify_clean_path_points(path)
        if len(path) < 2:
            raise ModelingTaskGenerationError("cleaning lane contains fewer than two usable points")
        segment["path"] = path
        segment["start"] = path[0]
        segment["end"] = path[-1]
        simplified_segments.append(segment)
    return simplified_segments


def _mark_continuous_path_tasks(path_tasks, continuous_path_id):
    """保留一条人工折线的全部点，仅在30度以上硬拐点停车重新转向。"""
    if not path_tasks:
        return
    for task in path_tasks:
        task["preserveRecordedPath"] = True
    if len(path_tasks) == 1:
        return

    hard_turns = []
    for index in range(len(path_tasks) - 1):
        start = (
            _number(path_tasks[index].get("startX")),
            _number(path_tasks[index].get("startY")),
        )
        middle = (
            _number(path_tasks[index].get("endX")),
            _number(path_tasks[index].get("endY")),
        )
        end = (
            _number(path_tasks[index + 1].get("endX")),
            _number(path_tasks[index + 1].get("endY")),
        )
        hard_turns.append(_turn_angle_degrees(start, middle, end) >= CLEAN_PATH_HARD_TURN_DEG)

    for index, task in enumerate(path_tasks):
        task["continuousPathId"] = continuous_path_id
        task["continuousPathIndex"] = index + 1
        task["continuousPathCount"] = len(path_tasks)
        task["turnAtStart"] = bool(index == 0 or hard_turns[index - 1])
        task["stopAtEnd"] = bool(index == len(path_tasks) - 1 or hard_turns[index])
        if index > 0:
            task["turnAngleAtStart"] = round(
                _turn_angle_degrees(
                    (
                        _number(path_tasks[index - 1].get("startX")),
                        _number(path_tasks[index - 1].get("startY")),
                    ),
                    (
                        _number(task.get("startX")),
                        _number(task.get("startY")),
                    ),
                    (
                        _number(task.get("endX")),
                        _number(task.get("endY")),
                    ),
                ),
                1,
            )


def _append_xy_path(
        tasks,
        path_points,
        area_number,
        mapper,
        task_id,
        source,
        preserve_recorded_path=False):
    """把相对坐标折线拆成连续mode=2任务，并可强制保留全部人工记录点。"""
    path_tasks = []
    for index in range(len(path_points) - 1):
        task = _segment_task(
            path_points[index],
            path_points[index + 1],
            2,
            area_number,
            task_id + len(path_tasks),
            mapper,
            source,
        )
        if task is not None:
            path_tasks.append(task)
    if preserve_recorded_path and path_tasks:
        _mark_continuous_path_tasks(
            path_tasks,
            "transfer:{}:{}".format(source or "path", task_id),
        )
    tasks.extend(path_tasks)
    return task_id + len(path_tasks)


def _point_ids_to_xy(point_ids, point_map, mapper):
    """按 point_ids 指定的业务顺序取点，转为相对坐标并删除相邻重复点。"""
    result = []
    for point_id in point_ids or []:
        point = point_map.get(point_id)
        xy = mapper.point_to_xy(point)
        if xy is None:
            raise ModelingTaskGenerationError("route policy point coordinates are missing")
        if not result or not _is_same_point(result[-1], xy):
            result.append(xy)
    return result


def _append_clean_segments(tasks, segments, current, mapper, task_id, draft):
    """
    把已排好顺序的清扫线加入任务列表。

    如果 current 不在下一条清扫线起点，先插入 mode=2 换行段；
    然后再插入 mode=1 清扫段。返回最终位置、下一个任务 ID 和新增清扫段数量。
    """
    # 拆分mode=1任务前先清理边界近点。这里再做一次是为了保护
    # 历史调用方；新版区域顺序规划在首段转场前已经做过同样处理。
    segments = _simplify_clean_segments(segments)
    # clean_count既用于汇总，也用于判断“进入第一条线”是否需要沿人工锚点走。
    clean_count = 0
    # segments已经按照选中的S形方案排序，不能在这里重新改变顺序或方向。
    for segment in segments:
        if current is not None and not _is_same_point(current, segment["start"]):
            # 首条线入口和相邻清扫线换行都沿人工记录边界寻找最短合法路线。
            # 这样A1--A2类短边上的全部浮动点都会成为真实换行路径点，而不是
            # 从上一条线终点直接斜连下一条线起点。
            transition_points = _group_anchor_transition_points(
                draft,
                segment.get("groupId"),
                current,
                segment["start"],
                mapper,
            )
            task_id = _append_xy_path(
                tasks,
                transition_points,
                segment["areaNumber"],
                mapper,
                task_id,
                "modeling_transfer",
                preserve_recorded_path=True,
            )
        # 内部清扫线只有首尾两个点，仍生成一个mode=1任务。边界清扫线的path则按
        # 每两个相邻记录点拆段，但所有子段共享continuousPathId，执行时按转角决定
        # 是连续经过还是停车转向。
        # 内部线path=[start,end]；外边界线path包含全部人工记录点。
        path = list(segment.get("path") or [segment["start"], segment["end"]])
        # lane_tasks用于保存同一条业务清扫线拆出的一个或多个可执行mode=1子段。
        lane_tasks = []
        for path_index in range(len(path) - 1):
            # 每两个相邻路径点形成一个执行段，确保真车能够沿折线逐点行驶。
            task = _segment_task(
                path[path_index],
                path[path_index + 1],
                1,
                segment["areaNumber"],
                task_id + len(lane_tasks),
                mapper,
                "modeling_clean",
            )
            if task is not None:
                lane_tasks.append(task)

        if lane_tasks:
            if len(lane_tasks) > 1:
                # 同一continuousPathId表示这些子段属于同一条边界清扫线，
                # 便于执行器在普通中间点保持车辆和滚刷连续运行。
                continuous_path_id = "{}:{}:{}".format(
                    segment.get("groupId") or "group",
                    segment.get("sourceId") or "lane",
                    task_id,
                )
                # hard_turns[i]描述第i段终点处是否达到30°停车转向阈值。
                hard_turns = []
                for index in range(len(lane_tasks) - 1):
                    start = (
                        _number(lane_tasks[index].get("startX")),
                        _number(lane_tasks[index].get("startY")),
                    )
                    middle = (
                        _number(lane_tasks[index].get("endX")),
                        _number(lane_tasks[index].get("endY")),
                    )
                    end = (
                        _number(lane_tasks[index + 1].get("endX")),
                        _number(lane_tasks[index + 1].get("endY")),
                    )
                    # θ_turn=min(|θ_in-θ_out|,360-|θ_in-θ_out|)，范围0..180°。
                    turn_angle = _turn_angle_degrees(start, middle, end)
                    # 达到30°是真实硬拐点；小于30°仍保留目标点，但连续通过。
                    hard_turns.append(turn_angle >= CLEAN_PATH_HARD_TURN_DEG)

                for index, task in enumerate(lane_tasks):
                    task["continuousPathId"] = continuous_path_id
                    task["continuousPathIndex"] = index + 1
                    task["continuousPathCount"] = len(lane_tasks)
                    # 第一子段或上一连接点是硬拐点时，当前段开始前必须原地重新定向。
                    task["turnAtStart"] = bool(index == 0 or hard_turns[index - 1])
                    # 最后一子段或当前终点是硬拐点时，到点必须刹车并关闭滚刷。
                    task["stopAtEnd"] = bool(index == len(lane_tasks) - 1 or hard_turns[index])
                    if index > 0:
                        task["turnAngleAtStart"] = round(
                            _turn_angle_degrees(
                                (
                                    _number(lane_tasks[index - 1].get("startX")),
                                    _number(lane_tasks[index - 1].get("startY")),
                                ),
                                (
                                    _number(task.get("startX")),
                                    _number(task.get("startY")),
                                ),
                                (
                                    _number(task.get("endX")),
                                    _number(task.get("endY")),
                                ),
                            ),
                            1,
                        )

            # 添加所有子段；clean_count稍后只加1，保持“一条边界折线=一条业务清扫线”。
            for task in lane_tasks:
                task["sourceLaneId"] = segment.get("sourceId")
                task["laneType"] = segment.get("laneType") or "interior"
                tasks.append(task)
            task_id += len(lane_tasks)
            clean_count += 1
            # 必须使用清理后的真实终点，下一条线的转场才不会又回到已删除的近点。
            current = path[-1]
    return current, task_id, clean_count


def _snap_lane_endpoints_to_recorded_anchors(segments, draft, mapper):
    """将5cm内的清扫线端点吸附到同一区域的真实记录点。

    扫描线与边界求交会产生浮点坐标。例如真实角点A6为40.466cm，求交结果
    可能是40.501cm；两者几何上几乎重合，但协议取整后会得到40cm和41cm，
    从而生成没有实际意义的短距离停车任务。只有当端点确实位于人工记录点5cm
    范围内时才吸附，区域内部正常清扫线和较远的真实边界折线均保持不变。
    """
    snapped_segments = []
    anchors_by_group = {}
    for original in segments or []:
        segment = dict(original)
        group_id = segment.get("groupId")
        if group_id not in anchors_by_group:
            anchors_by_group[group_id] = _group_recorded_xy(draft, group_id, mapper)
        anchors = anchors_by_group[group_id]
        path = list(segment.get("path") or [segment.get("start"), segment.get("end")])

        for key, path_index in (("start", 0), ("end", -1)):
            endpoint = segment.get(key)
            if endpoint is None or not anchors:
                continue
            nearest = min(anchors, key=lambda anchor: _length_cm(endpoint, anchor))
            distance = _length_cm(endpoint, nearest)
            if distance < LANE_ENDPOINT_ANCHOR_SNAP_CM:
                segment[key] = nearest
                if path:
                    path[path_index] = nearest

        segment["path"] = _dedupe_xy_path(path)
        snapped_segments.append(segment)
    return snapped_segments


def _task_xy(task, prefix):
    """从任务段中取出 startX/startY 或 endX/endY，并按实际执行整数厘米精度比较。"""
    return (
        _round_int(task.get(prefix + "X")),
        _round_int(task.get(prefix + "Y")),
    )


def _validate_continuous_round_trip(tasks, origin):
    """
    在保存和执行前校验路线闭环性。

    依次检查：
    1. 任务列表不能为空。
    2. 第一段起点必须等于建模原点。
    3. 每一段终点必须等于下一段起点。
    4. 最后一段终点必须回到建模原点。

    任何一项不满足都拒绝生成 taskPlan，避免小车执行断裂或跳点路径。
    """
    if not tasks:
        raise ModelingTaskGenerationError("generated task path is empty")

    # 最终防线：边界近点应在拆任务前已被合并。如果这里仍出现
    # 5cm以内的mode=1清扫段，说明某条新生成链路绕过了统一清理；
    # 此时宁可拒绝保存，也不把“转向-走1cm-再转向”任务交给实车。
    for task in tasks:
        if int(task.get("mode") or 0) != 1 or task.get("source") != "modeling_clean":
            continue
        clean_start = _task_xy(task, "start")
        clean_end = _task_xy(task, "end")
        if _length_cm(clean_start, clean_end) <= CLEAN_DUPLICATE_POINT_CM:
            raise ModelingTaskGenerationError(
                "generated cleaning task {} is not longer than {}cm".format(
                    task.get("id"),
                    int(CLEAN_DUPLICATE_POINT_CM),
                )
            )

    rounded_origin = (_round_int(origin[0]), _round_int(origin[1]))
    if _task_xy(tasks[0], "start") != rounded_origin:
        raise ModelingTaskGenerationError("generated task path does not start at the modeling origin")

    for index in range(1, len(tasks)):
        previous_end = _task_xy(tasks[index - 1], "end")
        current_start = _task_xy(tasks[index], "start")
        if previous_end != current_start:
            raise ModelingTaskGenerationError(
                "generated task path is discontinuous between task {} and {}".format(
                    tasks[index - 1].get("id"),
                    tasks[index].get("id"),
                )
            )

    if _task_xy(tasks[-1], "end") != rounded_origin:
        raise ModelingTaskGenerationError("generated task path does not return to the modeling origin")


def _generate_bridge_round_trip_plan(draft, preview, mapper, route_policy, now=None):
    """
    生成“从区域1出发、经连接桥到远端区域、再返回区域1”的闭环路线。

    routePolicy 中需要提供：
    - homeGroupId / remoteGroupId：起始区域和远端区域。
    - groupAnchors：两个区域的进入点和离开点。
    - outboundPointIds：从原点到远端区域的出程点序列。
    - returnPointIds：从远端区域回到起始区域的返程点序列。

    生成顺序：出程 -> 远端区域 S 形清扫 -> 返程连接桥 -> 起始区域 S 形清扫 -> 回原点。
    """
    # 先建立pointId索引，再从策略中读取桥头、区域入口和区域出口。
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

    # tasks始终保持首尾连续；task_id按最终执行顺序递增。
    tasks = []
    task_id = 1
    clean_count = 0
    # 第一阶段：按业务指定的出程点序列到达远端区域入口。
    # 出程初始点序列是“模型原点 -> 连接桥点...”。
    outbound = _point_ids_to_xy(route_policy.get("outboundPointIds"), point_map, mapper)
    if len(outbound) < 2:
        raise ModelingTaskGenerationError("route policy outbound path is incomplete")
    home_departure = _group_anchor_transition_points(
        draft,
        home_group_id,
        outbound[0],
        outbound[1],
        mapper,
    )
    outbound = _dedupe_xy_path(home_departure + outbound[2:])
    task_id = _append_xy_path(
        tasks,
        outbound,
        remote_group.get("areaNumber") or 2,
        mapper,
        task_id,
        "modeling_outbound",
    )
    current = outbound[-1]

    # 第二阶段：选择与远端区域入口/出口最匹配的 S 形清扫顺序。
    # 从四种S形候选中选择最匹配远端入口和远端桥侧出口的一种。
    remote_segments = _group_lane_segments(
        preview,
        remote_group_id,
        remote_entry,
        remote_exit,
    )
    current, task_id, added = _append_clean_segments(
        tasks, remote_segments, current, mapper, task_id, draft
    )
    clean_count += added
    if not _is_same_point(current, remote_exit):
        remote_exit_path = _group_anchor_transition_points(
            draft,
            remote_group_id,
            current,
            remote_exit,
            mapper,
        )
        task_id = _append_xy_path(
            tasks,
            remote_exit_path,
            remote_group.get("areaNumber") or 2,
            mapper,
            task_id,
            "modeling_transfer",
        )
        current = remote_exit

    # 第三阶段：按用户记录的连接点返回起始区域。
    # 原桥返回，点序列已在策略阶段反转为“远端桥头 -> 起始区域桥头”。
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

    # 第四阶段：清扫起始区域，并将最后一段对齐回原点方向。
    # 起始区域把桥头作为入口，把模型原点作为出口，再选择对应S形方向。
    home_segments = _group_lane_segments(
        preview,
        home_group_id,
        home_entry,
        home_exit,
    )
    current, task_id, added = _append_clean_segments(
        tasks, home_segments, current, mapper, task_id, draft
    )
    clean_count += added
    if not _is_same_point(current, home_exit):
        home_exit_path = _group_anchor_transition_points(
            draft,
            home_group_id,
            current,
            home_exit,
            mapper,
        )
        task_id = _append_xy_path(
            tasks,
            home_exit_path,
            home_group.get("areaNumber") or 1,
            mapper,
            task_id,
            "modeling_return_origin",
        )

    if not tasks or clean_count == 0:
        raise ModelingTaskGenerationError("route policy produced no cleaning tasks")
    # 全部几何段生成后再合并近似直行的mode=2中间点，降低无意义停车次数。
    tasks = _compact_executable_tasks(tasks)
    # clean_count保持“清扫线数量”的历史含义；一条边界折线虽然会拆成多个连续
    # mode=1执行子段，但对业务和前端仍然只算一条清扫线。
    clean_segment_count = sum(1 for task in tasks if int(task.get("mode") or 0) == 1)
    transfer_segment_count = sum(1 for task in tasks if int(task.get("mode") or 0) == 2)
    _validate_continuous_round_trip(
        tasks,
        (mapper.origin["x"], mapper.origin["y"]),
    )
    total_length = sum(int(task.get("length") or 0) for task in tasks)
    return {
        "status": "ready",
        "generatedAt": int(now if now is not None else time.time()),
        "taskName": draft.get("name") or draft.get("id") or "",
        "routeType": "bridge_round_trip",
        "summary": {
            "taskCount": len(tasks),
            "cleanTaskCount": clean_count,
            "cleanSegmentTaskCount": clean_segment_count,
            "transferTaskCount": transfer_segment_count,
            "totalLengthCm": total_length,
        },
        "tasks": tasks,
    }


def _resolve_area_order(preview, route_policy):
    """把前端区域编号顺序转换为完整、无重复的 groupId 顺序。"""
    groups = list(preview.get("groups") or [])
    if not groups:
        raise ModelingTaskGenerationError("modeling preview contains no areas")
    default_groups = sorted(
        groups,
        key=lambda group: (
            int(group.get("areaNumber") or 0),
            str(group.get("groupId") or ""),
        ),
    )
    requested = (route_policy or {}).get("areaOrder")
    if requested is None:
        requested = [group.get("areaNumber") for group in default_groups]
    if not isinstance(requested, (list, tuple)):
        raise ModelingTaskGenerationError("areaOrder must be an array")

    by_id = {str(group.get("groupId")): group for group in groups if group.get("groupId")}
    by_number = {
        int(group.get("areaNumber")): group
        for group in groups
        if group.get("areaNumber") is not None
    }
    ordered_groups = []
    seen = set()
    for value in requested:
        group = by_id.get(str(value))
        if group is None:
            try:
                group = by_number.get(int(value))
            except (TypeError, ValueError):
                group = None
        group_id = group.get("groupId") if group else None
        if not group_id:
            raise ModelingTaskGenerationError("areaOrder contains an unknown area: {}".format(value))
        if group_id in seen:
            raise ModelingTaskGenerationError("areaOrder contains duplicate areas")
        seen.add(group_id)
        ordered_groups.append(group)
    expected = set(group.get("groupId") for group in groups)
    if seen != expected:
        raise ModelingTaskGenerationError("areaOrder must contain every modeling area exactly once")
    return ordered_groups


def _transition_entry_reference(preview, from_group_id, to_group_id, mapper, fallback):
    """返回跨区路线到达目标区域一侧的桥头，供S形入口评分使用。"""
    if from_group_id == to_group_id:
        return fallback
    # 搜索从当前区域到目标区域的有向桥链。
    route_edges = _link_route_between(preview, from_group_id, to_group_id, mapper)
    if not route_edges:
        raise ModelingTaskGenerationError(
            "modeling groups {} and {} are not connected".format(from_group_id, to_group_id)
        )
    # 最后一座桥的最后一个点位于目标区域一侧，是目标区域的入口评分参考点。
    return route_edges[-1]["points"][-1]


def _transition_exit_reference(preview, from_group_id, to_group_id, mapper, fallback):
    """返回离开当前区域时应靠近的第一座桥头，供S形出口评分使用。"""
    if from_group_id == to_group_id:
        return fallback
    # 搜索当前区域离开后到下一目标区域的有向桥链。
    route_edges = _link_route_between(preview, from_group_id, to_group_id, mapper)
    if not route_edges:
        raise ModelingTaskGenerationError(
            "modeling groups {} and {} are not connected".format(from_group_id, to_group_id)
        )
    # 第一座桥的第一个点位于当前区域一侧，是当前区域的出口评分参考点。
    return route_edges[0]["points"][0]


def _generate_area_order_plan(draft, preview, mapper, route_policy, now=None):
    """按默认或前端指定区域顺序，统一规划单区域和任意多区域闭环路线。"""
    # 把前端areaOrder（区域编号或groupId）解析为不遗漏、不重复的区域对象列表。
    ordered_groups = _resolve_area_order(preview, route_policy)
    # 第一个有效建模点既是统一坐标原点，也是整条闭环路线的起点/终点。
    origin = (mapper.origin["x"], mapper.origin["y"])
    origin_group_id = mapper.origin_group_id
    origin_group = _preview_group(preview, origin_group_id) or {}

    tasks = []
    selections = []
    task_id = 1
    clean_count = 0
    current = origin
    current_group_id = origin_group_id

    # 按areaOrder逐个规划区域；每完成一个区域，current更新为该区域末线终点。
    for index, group in enumerate(ordered_groups):
        group_id = group.get("groupId")
        # 当前区域不是最后一个时，出口指向areaOrder中的下一区域；
        # 最后一个区域的下一目标是原点所属区域，用于规划闭环返程。
        next_group_id = (
            ordered_groups[index + 1].get("groupId")
            if index + 1 < len(ordered_groups)
            else origin_group_id
        )
        # 入口参考：同区域时用当前位置，跨区时用到达目标区域的最后一个桥头。
        entry_reference = _transition_entry_reference(
            preview,
            current_group_id,
            group_id,
            mapper,
            current,
        )
        # 出口参考：跨区时用离开当前区域的第一个桥头，最后区域则指向返程桥/原点。
        exit_reference = _transition_exit_reference(
            preview,
            group_id,
            next_group_id,
            mapper,
            origin,
        )
        # 在满足最低重叠的奇偶候选中，结合入口/出口选出当前区域唯一S形方案。
        selected = _select_group_lane_segments(
            preview,
            group_id,
            entry_reference,
            exit_reference,
        )
        # 求交得到的线端点若距离人工角点不足3cm，统一吸附到该真实记录点。
        # 这样跨区转场和随后清扫任务共用同一整数厘米坐标，不会产生1cm伪任务。
        segments = _snap_lane_endpoints_to_recorded_anchors(
            selected["segments"], draft, mapper
        )
        # 首条清扫线的起点同时是下方转场的目标点。因此近点清理
        # 必须在生成首段转场之前完成，不能等mode=1拆任务时才修改。
        segments = _simplify_clean_segments(segments)
        selection = dict(selected["selection"])
        selection["order"] = index + 1
        selections.append(selection)

        # 若当前真实位置不等于首条清扫线起点，先生成mode=2合法转场；
        # 跨区域转场必须经过连接桥，同区域转场会沿人工边界而不是斜穿区域。
        if not _is_same_point(current, segments[0]["start"]):
            task_id = _append_transition_tasks(
                tasks,
                current,
                segments[0]["start"],
                group.get("areaNumber") or index + 1,
                mapper,
                task_id,
                preview,
                draft,
                current_group_id,
                group_id,
                source="modeling_start_to_first_lane" if clean_count == 0 else "modeling_transfer",
            )
        # 追加本区域全部mode=1清扫线及相邻线之间的mode=2换行段。
        current, task_id, added = _append_clean_segments(
            tasks,
            segments,
            segments[0]["start"],
            mapper,
            task_id,
            draft,
        )
        clean_count += added
        current_group_id = group_id

    if not tasks or clean_count == 0:
        raise ModelingTaskGenerationError("路径预览中没有可生成的清扫线")

    # 清扫完最后区域后，如果尚未回到原点，按连接图和边界锚点生成闭环返程。
    if not _is_same_point(current, origin):
        task_id = _append_transition_tasks(
            tasks,
            current,
            origin,
            origin_group.get("areaNumber") or 1,
            mapper,
            task_id,
            preview,
            draft,
            current_group_id,
            origin_group_id,
            source="modeling_return_origin",
        )

    # 删除小于3cm伪转场、合并普通近似直行转场，同时保留30°以上真实拐点。
    tasks = _compact_executable_tasks(tasks)
    # 边界折线的多个mode=1子段属于同一条清扫线，不能改变cleanTaskCount的历史含义。
    clean_segment_count = sum(1 for task in tasks if int(task.get("mode") or 0) == 1)
    transfer_segment_count = sum(1 for task in tasks if int(task.get("mode") or 0) == 2)
    # 保存前执行最终安全校验：首段从原点开始、段段连续、末段回到原点。
    _validate_continuous_round_trip(tasks, origin)
    total_length = sum(int(task.get("length") or 0) for task in tasks)
    return {
        "status": "ready",
        "generatedAt": int(now if now is not None else time.time()),
        "taskName": draft.get("name") or draft.get("id") or "",
        "routeType": "area_order",
        "areaOrder": [group.get("areaNumber") for group in ordered_groups],
        "routeSelections": selections,
        "summary": {
            "taskCount": len(tasks),
            "cleanTaskCount": clean_count,
            "cleanSegmentTaskCount": clean_segment_count,
            "transferTaskCount": transfer_segment_count,
            "totalLengthCm": total_length,
        },
        "tasks": tasks,
    }


def generate_task_plan(draft, now=None):
    """
    把路径预览转换为小车真正执行的 taskPlan。

    输入 draft 必须已经包含 status=ready 的 taskPreview。
    输出的 tasks 是唯一有序任务段列表，前一段的终点必须等于后一段的起点。

    普通多区域流程：
    1. 从第一个建模点（origin）出发。
    2. 如果不在第一条清扫线起点，先生成 mode=2 对接段。
    3. 逐条加入 mode=1 清扫段，并在两条线之间插入 mode=2 换行段。
    4. 跨区域时按 groupLinks 中的连接点通过。
    5. 最后从末端返回 origin，并执行闭环校验。

    summary.cleanTaskCount 统计业务清扫线数量；边界折线拆成多个连续mode=1子段时仍只算一条线。
    summary.cleanSegmentTaskCount 统计实际mode=1执行子段数量。
    summary.transferTaskCount 统计起点对接、换行、连接桥和回原点等 mode=2 移动段。
    """
    if not isinstance(draft, dict):
        raise ModelingTaskGenerationError("model draft is required")
    preview = draft.get("taskPreview") or {}
    if not isinstance(preview, dict) or preview.get("status") != "ready":
        raise ModelingTaskGenerationError("路径预览未生成或不可用，不能生成执行任务")

    # 所有任务段都必须同时生成相对坐标和 RTK 经纬度，因此先建立统一坐标转换器。
    # 建立统一坐标转换器，保证任务中的x/y与lat/lon互相对应。
    mapper = _CoordinateMapper(draft)
    # 显式历史策略继续兼容；所有新建模型统一按区域顺序走通用候选规划。
    route_policy = draft.get("routePolicy") or {}
    if route_policy.get("type") == "bridge_round_trip":
        # 业务明确指定先清扫远端区域、再返回起始区域时，使用专用闭环策略。
        return _generate_bridge_round_trip_plan(
            draft,
            preview,
            mapper,
            route_policy,
            now=now,
        )
    return _generate_area_order_plan(
        draft,
        preview,
        mapper,
        route_policy,
        now=now,
    )
