# coding=utf-8
"""区域点几何识别模块。

当前小程序的新建模流程会把“区域点”和“跨区域连接点”分别保存到 groups 与
groupLinks；这种显式连接关系不需要本文件猜测。本文件主要负责单个 group 内部
的兼容识别：当旧数据把多个子区域及其窄连接段连续记录在同一组点中时，根据
相邻边的转角和连接段长度，尝试识别子区域连接段并拆分子区域。

核心公式：

    入射向量 a = 当前点 - 前一点
    出射向量 b = 后一点 - 当前点
    转角 alpha = atan2(a x b, a · b)

叉积决定左转/右转符号，点积参与计算夹角大小。
"""
import math


# 小于20度的轻微变化视为边界辅助点，而不是明显拐角。
ASSIST_TURN_DEG = 20.0
# 旧模型自动识别连接段时，连接段两端的转角必须位于45～135度。
CONNECTOR_MIN_TURN_DEG = 45.0
CONNECTOR_MAX_TURN_DEG = 135.0
# 过短的定位抖动不能当作连接段，候选连接段至少长50厘米。
CONNECTOR_MIN_LENGTH_CM = 50.0


def _number(value):
    if value is None or value == "":
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if not math.isnan(number) and not math.isinf(number) else None


def _lat_lon_to_xy_cm(origin_lat, origin_lon, lat, lon):
    """以组内第一个有效 RTK 点为临时原点，计算该组内部厘米坐标。"""
    # 平均纬度用于修正不同纬度下每度经度对应的实际距离。
    mean_lat = math.radians((origin_lat + lat) / 2.0)
    # x 是东西距离，y 是南北距离；先算米，再统一转成厘米。
    x_m = math.radians(lon - origin_lon) * 6371000.0 * math.cos(mean_lat)
    y_m = math.radians(lat - origin_lat) * 6371000.0
    return x_m * 100.0, y_m * 100.0


def _points_with_xy(points):
    """保持点位记录顺序，补齐识别算法需要的 sequence、x、y。"""
    # 先查找第一个有效经纬度，只有缺失 x/y 时才用它做组内临时换算。
    origin = None
    for point in points:
        lat = _number(point.get("lat"))
        lon = _number(point.get("lon"))
        if lat is not None and lon is not None:
            origin = (lat, lon)
            break

    # 逐点复制，不能在识别阶段打乱人工记录顺序。
    normalized = []
    for index, point in enumerate(points, start=1):
        item = dict(point)
        item["sequence"] = int(item.get("sequence") or index)
        x = _number(item.get("x"))
        y = _number(item.get("y"))
        if (x is None or y is None) and origin is not None:
            lat = _number(item.get("lat"))
            lon = _number(item.get("lon"))
            if lat is not None and lon is not None:
                x, y = _lat_lon_to_xy_cm(origin[0], origin[1], lat, lon)
        item["x"] = round(x, 3) if x is not None else None
        item["y"] = round(y, 3) if y is not None else None
        normalized.append(item)
    return normalized


def _vector(start, end):
    """返回 start 指向 end 的二维向量 (dx, dy)。"""
    if start.get("x") is None or start.get("y") is None or end.get("x") is None or end.get("y") is None:
        return None
    return end["x"] - start["x"], end["y"] - start["y"]


def _length(vector):
    """用勾股定理计算向量长度。"""
    if vector is None:
        return None
    return math.hypot(vector[0], vector[1])


def _signed_turn(prev_point, point, next_point):
    """计算在 point 处从入射边转到出射边的带符号角度。"""
    # incoming 是“前一点 -> 当前点”，outgoing 是“当前点 -> 后一点”。
    incoming = _vector(prev_point, point)
    outgoing = _vector(point, next_point)
    if _length(incoming) in (None, 0) or _length(outgoing) in (None, 0):
        return None
    # 二维叉积给出转向符号，点积给出两向量夹角关系。
    cross = incoming[0] * outgoing[1] - incoming[1] * outgoing[0]
    dot = incoming[0] * outgoing[0] + incoming[1] * outgoing[1]
    # atan2(叉积, 点积) 可同时得到角度大小和左右转符号。
    return math.degrees(math.atan2(cross, dot))


def _turn_sign(angle):
    if angle is None or abs(angle) < 1e-6:
        return 0
    return 1 if angle > 0 else -1


def _dominant_turn_sign(turns):
    """统计区域轮廓主要是顺时针还是逆时针记录。"""
    # 小于20度的轻微抖动不参与主转向统计。
    signed = [
        _turn_sign(turn)
        for turn in turns
        if turn is not None and abs(turn) >= ASSIST_TURN_DEG
    ]
    total = sum(signed)
    if total == 0:
        return 0
    return 1 if total > 0 else -1


def _connector_candidate(points, turns, dominant_sign, index):
    """判断 points[index] -> points[index+1] 是否像旧模型中的窄连接段。"""
    next_index = index + 1
    if next_index >= len(points):
        return False
    left = turns[index]
    right = turns[next_index]
    if left is None or right is None:
        return False
    if dominant_sign == 0:
        return False
    # 连接段两端应当相对主轮廓产生连续反向拐折。
    if _turn_sign(left) != -dominant_sign or _turn_sign(right) != -dominant_sign:
        return False
    if not (CONNECTOR_MIN_TURN_DEG <= abs(left) <= CONNECTOR_MAX_TURN_DEG):
        return False
    if not (CONNECTOR_MIN_TURN_DEG <= abs(right) <= CONNECTOR_MAX_TURN_DEG):
        return False
    # 最后检查该段是否达到50厘米，排除 RTK 抖动形成的短小假连接。
    segment_length = _length(_vector(points[index], points[next_index]))
    return segment_length is not None and segment_length >= CONNECTOR_MIN_LENGTH_CM


def _make_connector(points, index, connector_index):
    start = points[index]
    end = points[index + 1]
    return {
        "id": "c{}".format(connector_index),
        "type": "sub_area_connector",
        "label": "子区域连接段",
        "pointIds": [start.get("id"), end.get("id")],
        "startPointId": start.get("id"),
        "endPointId": end.get("id"),
        "lengthCm": round(_length(_vector(start, end)) or 0.0, 1),
        "needsConfirmation": True,
    }


def _single_sub_area(group_id, points, needs_confirmation=False):
    return [{
        "id": "{}-sa1".format(group_id),
        "name": "子区域1",
        "pointIds": [point.get("id") for point in points],
        "needsConfirmation": needs_confirmation,
    }]


def _split_by_connector_axis(group_id, points, connector):
    """沿连接段轴线的中点投影，把旧式混合点列尝试拆成两个子区域。"""
    start_id, end_id = connector["pointIds"]
    start = next((point for point in points if point.get("id") == start_id), None)
    end = next((point for point in points if point.get("id") == end_id), None)
    axis = _vector(start, end) if start and end else None
    axis_len = _length(axis)
    if not axis or not axis_len:
        return []

    # 把连接段向量单位化，后面所有点都投影到这条轴线上比较位置。
    unit = (axis[0] / axis_len, axis[1] / axis_len)
    midpoint = ((start["x"] + end["x"]) / 2.0, (start["y"] + end["y"]) / 2.0)
    midpoint_projection = midpoint[0] * unit[0] + midpoint[1] * unit[1]
    left_ids = []
    right_ids = []
    for point in points:
        if point.get("id") == start_id:
            left_ids.append(point.get("id"))
            continue
        if point.get("id") == end_id:
            right_ids.append(point.get("id"))
            continue
        if point.get("x") is None or point.get("y") is None:
            continue
        # 点积就是该点沿连接轴方向的一维投影位置。
        projection = point["x"] * unit[0] + point["y"] * unit[1]
        if projection <= midpoint_projection:
            left_ids.append(point.get("id"))
        else:
            right_ids.append(point.get("id"))

    sub_areas = []
    for index, point_ids in enumerate((left_ids, right_ids), start=1):
        if point_ids:
            sub_areas.append({
                "id": "{}-sa{}".format(group_id, index),
                "name": "子区域{}".format(index),
                "pointIds": point_ids,
                "needsConfirmation": True,
            })
    return sub_areas


def recognize_group_points(group):
    """识别一组按记录顺序排列的区域点，并返回点角色、子区域和候选连接段。"""
    # groupId 既用于结果关联，也用于生成稳定的子区域编号。
    group_id = str(group.get("id") or "group")
    # 识别算法只使用同一坐标系中的有序点列。
    points = _points_with_xy(list(group.get("points") or []))
    point_count = len(points)
    # 当前业务规定至少四个区域点才允许完成区域识别。
    if point_count < 4:
        return {
            "groupId": group_id,
            "status": "insufficient_points",
            "message": "至少需要4个点才能识别区域",
            "needsConfirmation": True,
            "points": points,
            "subAreas": [],
            "connectors": [],
            "warnings": ["INSUFFICIENT_POINTS"],
            "summary": {
                "pointCount": point_count,
                "subAreaCount": 0,
                "connectorCount": 0,
                "assistPointCount": 0,
            },
        }

    # 首尾闭合：第一个点的前一点是最后一个点，最后一个点的后一点是第一个点。
    turns = [
        _signed_turn(points[(index - 1) % point_count], points[index], points[(index + 1) % point_count])
        for index in range(point_count)
    ]
    # 主转向符号代表这圈边界整体的记录方向。
    dominant_sign = _dominant_turn_sign(turns)
    # 只检查相邻点形成的线段；满足角度、方向和长度条件的才是连接段候选。
    connector_indexes = [
        index for index in range(point_count - 1)
        if _connector_candidate(points, turns, dominant_sign, index)
    ]
    connectors = [
        _make_connector(points, index, connector_number)
        for connector_number, index in enumerate(connector_indexes, start=1)
    ]
    # 建立连接点 ID 集合，便于给原始点标注 connection_point 角色。
    connector_point_ids = set()
    for connector in connectors:
        connector_point_ids.update(connector["pointIds"])

    # 根据转角把点分为连接点、边界辅助点和边界拐角点。
    assist_count = 0
    for index, point in enumerate(points):
        roles = []
        if point.get("id") in connector_point_ids:
            role = "connection_point"
            roles.append("sub_area_connector")
        elif turns[index] is not None and abs(turns[index]) < ASSIST_TURN_DEG:
            role = "boundary_assist"
            assist_count += 1
        else:
            role = "boundary_corner"
        point["role"] = role
        point["roles"] = roles
        point["turnDeg"] = round(turns[index], 1) if turns[index] is not None else None

    warnings = []
    # 没有候选连接段：整组点就是一个子区域。
    if len(connectors) == 0:
        sub_areas = _single_sub_area(group_id, points)
        status = "recognized"
        needs_confirmation = False
        message = "已识别为单个子区域"
    # 一个候选连接段：尝试自动拆成两个子区域，并检查两边点数是否足够。
    elif len(connectors) == 1:
        sub_areas = _split_by_connector_axis(group_id, points, connectors[0])
        confident_split = len(sub_areas) == 2 and all(len(item["pointIds"]) >= 4 for item in sub_areas)
        if confident_split:
            for connector in connectors:
                connector["needsConfirmation"] = False
            for sub_area in sub_areas:
                sub_area["needsConfirmation"] = False
        if len(sub_areas) != 2 or any(len(item["pointIds"]) < 3 for item in sub_areas):
            warnings.append("CONNECTOR_SPLIT_NEEDS_MANUAL_CONFIRMATION")
        if confident_split:
            status = "recognized"
            needs_confirmation = False
        else:
            status = "needs_confirmation"
            needs_confirmation = True
        message = "已识别到子区域连接段，请确认子区域划分"
    # 多个候选连接段存在歧义，不能擅自拆分，必须由业务层人工确认。
    else:
        sub_areas = []
        warnings.append("MULTIPLE_CONNECTORS_NEED_MANUAL_CONFIRMATION")
        status = "needs_confirmation"
        needs_confirmation = True
        message = "识别到多个连接段，请人工确认子区域"

    if len(connectors) == 1 and not needs_confirmation:
        message = "auto recognized sub areas and connector"

    return {
        "groupId": group_id,
        "status": status,
        "message": message,
        "needsConfirmation": needs_confirmation,
        "points": points,
        "subAreas": sub_areas,
        "connectors": connectors,
        "warnings": warnings,
        "summary": {
            "pointCount": point_count,
            "subAreaCount": len(sub_areas),
            "connectorCount": len(connectors),
            "assistPointCount": assist_count,
        },
    }
