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
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    if abs(dx) < EPSILON_CM and abs(dy) < EPSILON_CM:
        return None
    return _normalize_heading(math.degrees(math.atan2(dx, dy)))


def _length_cm(start, end):
    return math.hypot(end[0] - start[0], end[1] - start[1])


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
    start = (_round_int(start[0]), _round_int(start[1]))
    end = (_round_int(end[0]), _round_int(end[1]))
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


def _same_direction_collinear(left, right):
    """判断两个首尾相接的任务段能否作为一条直线连续执行。"""
    if int(left.get("mode") or 0) != int(right.get("mode") or 0):
        return False
    if left.get("preserveEndStop") or right.get("preserveStartStop"):
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

    # 叉积判断是否在同一直线上，点积保证不是到中间点后掉头。
    cross = left_vector[0] * right_vector[1] - left_vector[1] * right_vector[0]
    dot = left_vector[0] * right_vector[0] + left_vector[1] * right_vector[1]
    return abs(cross) <= EPSILON_CM * left_length * right_length and dot > 0


def _compact_executable_tasks(tasks):
    """
    合并同模式、同方向、同一直线上的普通中间任务段。

    区域边界采样点、前端绘图点或连接桥中间点只有在真正形成拐角时才会留下；
    清扫模式发生变化或显式标记为必须停车的位置永远不会被跨越合并。
    """
    compacted = []
    for raw_task in tasks or []:
        task = dict(raw_task)
        if compacted and _same_direction_collinear(compacted[-1], task):
            merged = compacted[-1]
            merged["endX"] = task.get("endX")
            merged["endY"] = task.get("endY")
            merged["endLat"] = task.get("endLat")
            merged["endLon"] = task.get("endLon")
            start = (_number(merged.get("startX")), _number(merged.get("startY")))
            end = (_number(merged.get("endX")), _number(merged.get("endY")))
            merged["length"] = _round_int(_length_cm(start, end))
            heading = _heading_from_xy(start, end)
            merged["heading"] = heading
            merged["angle"] = heading
            sources = list(merged.get("mergedSources") or [merged.get("source")])
            if task.get("source") not in sources:
                sources.append(task.get("source"))
            merged["mergedSources"] = [source for source in sources if source]
            merged["mergedSegmentCount"] = int(merged.get("mergedSegmentCount") or 1) + 1
            continue
        compacted.append(task)

    for index, task in enumerate(compacted, start=1):
        task["id"] = index
    return compacted


def _lane_points(lane, reverse=False):
    """取出一条清扫线的起终点；reverse=True 时交换起终点，用于生成 S 形往返。"""
    start = (_number(lane.get("startX")), _number(lane.get("startY")))
    end = (_number(lane.get("endX")), _number(lane.get("endY")))
    if None in start or None in end:
        return None
    return (end, start) if reverse else (start, end)


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
    raw_points = list(link.get("points") or [])
    if not raw_points:
        raw_points = [link.get("startPoint"), link.get("endPoint")]
    points = []
    for point in raw_points:
        xy = mapper.point_to_xy(point)
        if xy is not None and (not points or not _is_same_point(points[-1], xy)):
            points.append(xy)
    return points


def _link_points_between(preview, from_group_id, to_group_id, mapper):
    """
    在区域连接图中查找 from_group_id 到 to_group_id 的连接点序列。

    groupLinks 被构建为双向图，使用广度优先搜索寻找可达路径。
    反向通过同一条连接桥时，会自动反转连接点顺序。
    """
    if not from_group_id or not to_group_id or from_group_id == to_group_id:
        return []

    graph = {}
    for link in preview.get("groupLinks") or []:
        start_group_id = link.get("startGroupId")
        end_group_id = link.get("endGroupId")
        link_points = _link_xy_points(link, mapper)
        if not start_group_id or not end_group_id or len(link_points) < 2:
            continue
        graph.setdefault(start_group_id, []).append((end_group_id, link_points))
        graph.setdefault(end_group_id, []).append((start_group_id, list(reversed(link_points))))

    queue = [(from_group_id, [])]
    visited = set([from_group_id])
    while queue:
        group_id, route_points = queue.pop(0)
        for next_group_id, link_points in graph.get(group_id, []):
            if next_group_id in visited:
                continue
            next_route = list(route_points)
            for point in link_points:
                if not next_route or not _is_same_point(next_route[-1], point):
                    next_route.append(point)
            if next_group_id == to_group_id:
                return next_route
            visited.add(next_group_id)
            queue.append((next_group_id, next_route))
    return None


def _append_transition_tasks(
        tasks,
        current,
        target,
        area_number,
        mapper,
        task_id,
        preview,
        from_group_id,
        to_group_id,
        source="modeling_transfer"):
    """
    在两段清扫线之间补充 mode=2 移动段。

    同一区域内直接从 current 连接到 target。
    跨区域时必须先查找用户记录的连接桥，然后按“当前点 -> 连接点... -> 目标点”拆成多个连续任务段。
    若两个区域没有可达的连接桥，直接报错，不允许机器人跨空直线行驶。
    """
    route_points = _link_points_between(preview, from_group_id, to_group_id, mapper)
    if from_group_id != to_group_id and route_points is None:
        raise ModelingTaskGenerationError(
            "modeling groups {} and {} are not connected".format(from_group_id, to_group_id)
        )
    route_points = route_points or []
    path_points = [current] + route_points + [target]
    for index in range(len(path_points) - 1):
        start = path_points[index]
        end = path_points[index + 1]
        task = _segment_task(start, end, 2, area_number, task_id, mapper, source)
        if task is not None:
            tasks.append(task)
            task_id += 1
    return task_id


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


def _derive_two_group_round_trip_policy(draft, preview, mapper):
    """
    为常规“起始区域 -> 连接桥 -> 远端区域”记录流程自动生成往返策略。

    记录顺序只决定哪个区域是起始区域；执行顺序固定为先经连接桥到远端区域，
    清扫远端后沿原桥返回，再清扫起始区域并回到第一个记录点。
    """
    groups = list(preview.get("groups") or [])
    if len(groups) != 2 or not mapper.origin_group_id or not mapper.origin_point_id:
        return None
    group_ids = [group.get("groupId") for group in groups]
    if mapper.origin_group_id not in group_ids:
        return None
    remote_group_id = next(
        (group_id for group_id in group_ids if group_id != mapper.origin_group_id),
        None,
    )
    if not remote_group_id:
        return None

    for link in draft.get("groupLinks") or []:
        start_group_id = link.get("startGroupId")
        end_group_id = link.get("endGroupId")
        if set([start_group_id, end_group_id]) != set([mapper.origin_group_id, remote_group_id]):
            continue
        point_ids = [
            point.get("id")
            for point in (link.get("points") or [])
            if point.get("id")
        ]
        if len(point_ids) < 2:
            continue
        if start_group_id != mapper.origin_group_id:
            point_ids.reverse()

        outbound_ids = [mapper.origin_point_id]
        for point_id in point_ids:
            if outbound_ids[-1] != point_id:
                outbound_ids.append(point_id)
        return_ids = list(reversed(point_ids))
        return {
            "type": "bridge_round_trip",
            "derived": True,
            "forceEvenLanes": True,
            "originPointId": mapper.origin_point_id,
            "homeGroupId": mapper.origin_group_id,
            "remoteGroupId": remote_group_id,
            "linkId": link.get("id"),
            "outboundPointIds": outbound_ids,
            "returnPointIds": return_ids,
            "groupAnchors": {
                remote_group_id: {
                    "entryPointId": point_ids[-1],
                    "exitPointId": point_ids[-1],
                },
                mapper.origin_group_id: {
                    "entryPointId": point_ids[0],
                    "exitPointId": mapper.origin_point_id,
                },
            },
        }
    return None


def _group_lane_segments(preview, group_id, entry, exit_point):
    """
    为指定区域选择一组最适合入口和出口的 S 形清扫顺序。

    对一组清扫线尝试四种组合：
    1. 按原线序，第一条正向。
    2. 按原线序，第一条反向。
    3. 整组清扫线逆序，第一条正向。
    4. 整组清扫线逆序，第一条反向。

    评分为“入口到首线起点距离 + 末线终点到出口距离”，选择总距离最小的组合。
    """
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
    """把一串相对坐标点拆成首尾连续的 mode=2 移动任务段。"""
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


def _append_clean_segments(tasks, segments, current, mapper, task_id):
    """
    把已排好顺序的清扫线加入任务列表。

    如果 current 不在下一条清扫线起点，先插入 mode=2 换行段；
    然后再插入 mode=1 清扫段。返回最终位置、下一个任务 ID 和新增清扫段数量。
    """
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
    # 第一阶段：按业务指定的出程点序列到达远端区域入口。
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

    # 第二阶段：选择与远端区域入口/出口最匹配的 S 形清扫顺序。
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

    # 第三阶段：按用户记录的连接点返回起始区域。
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
    tasks = _compact_executable_tasks(tasks)
    clean_count = sum(1 for task in tasks if int(task.get("mode") or 0) == 1)
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
            "transferTaskCount": len(tasks) - clean_count,
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

    summary.cleanTaskCount 只统计 mode=1 清扫段；
    summary.transferTaskCount 统计起点对接、换行、连接桥和回原点等 mode=2 移动段。
    """
    if not isinstance(draft, dict):
        raise ModelingTaskGenerationError("model draft is required")
    preview = draft.get("taskPreview") or {}
    if not isinstance(preview, dict) or preview.get("status") != "ready":
        raise ModelingTaskGenerationError("路径预览未生成或不可用，不能生成执行任务")

    # 所有任务段都必须同时生成相对坐标和 RTK 经纬度，因此先建立统一坐标转换器。
    mapper = _CoordinateMapper(draft)
    route_policy = draft.get("routePolicy") or {}
    if not route_policy.get("type"):
        route_policy = _derive_two_group_round_trip_policy(draft, preview, mapper) or route_policy
    if route_policy.get("type") == "bridge_round_trip":
        # 业务明确指定先清扫远端区域、再返回起始区域时，使用专用闭环策略。
        return _generate_bridge_round_trip_plan(
            draft,
            preview,
            mapper,
            route_policy,
            now=now,
        )
    tasks = []
    task_id = 1
    clean_count = 0
    # 路线原点固定为用户记录的第一个有效建模点。
    origin = (mapper.origin["x"], mapper.origin["y"])
    origin_group_id = mapper.origin_group_id
    origin_group = _preview_group(preview, origin_group_id) or {}
    current = origin
    current_group_id = origin_group_id

    for segment in _iter_clean_segments(preview, draft, mapper):
        if not _is_same_point(current, segment["start"]):
            # 第一次对接标记为 modeling_start_to_first_lane，后续换行标记为 modeling_transfer。
            task_id = _append_transition_tasks(
                tasks,
                current,
                segment["start"],
                segment["areaNumber"],
                mapper,
                task_id,
                preview,
                current_group_id,
                segment.get("groupId"),
                source="modeling_start_to_first_lane" if clean_count == 0 else "modeling_transfer",
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
            # sourceLaneId 将可执行任务段与预览中的清扫线关联，便于日志排查和前端对照。
            clean_task["sourceLaneId"] = segment.get("sourceId")
            tasks.append(clean_task)
            task_id += 1
            clean_count += 1
            current = segment["end"]
            current_group_id = segment.get("groupId")

    if not tasks or clean_count == 0:
        raise ModelingTaskGenerationError("路径预览中没有可生成的清扫线")

    if not _is_same_point(current, origin):
        # 清扫完成后生成返回原点的移动段，使整条路线形成闭环。
        task_id = _append_transition_tasks(
            tasks,
            current,
            origin,
            origin_group.get("areaNumber") or 1,
            mapper,
            task_id,
            preview,
            current_group_id,
            origin_group_id,
            source="modeling_return_origin",
        )

    # 同一直线上的普通中间点只用于建模/绘图，不让它们变成额外停车点。
    tasks = _compact_executable_tasks(tasks)
    clean_count = sum(1 for task in tasks if int(task.get("mode") or 0) == 1)
    # 任务一旦保存后会直接驱动小车，所以在输出前强制校验首尾和每一段连续性。
    _validate_continuous_round_trip(tasks, origin)
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
