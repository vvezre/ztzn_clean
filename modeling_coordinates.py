# coding=utf-8
"""建模坐标统一模块。

领导评审时可以把本文件理解为“经纬度 -> 建模平面坐标”的第一道处理：

1. 从区域列表中找到区域1的第一个有效 RTK 点，并把它定义为整个模型的原点。
2. 区域点和连接桥点全部使用同一个原点换算，不能让区域2、区域3重新从 (0, 0) 开始。
3. 局部建模距离较短，所以采用地球表面局部平面近似：

       x = R * cos(平均纬度) * 经度差(弧度) * 100
       y = R * 纬度差(弧度) * 100

   x 表示东西方向，y 表示南北方向，最终单位都是厘米。
4. 本模块只改变坐标和坐标系说明，不改变点位 ID、记录顺序、区域归属和连接关系。
"""
import copy
import math


# 地球平均半径，单位米；经纬度差换算为局部平面距离时使用。
EARTH_RADIUS_M = 6371000.0


def _number(value):
    """把接口中的数字或数字字符串安全转换为有限浮点数。"""
    # None 和空字符串表示没有坐标，不能参与几何计算。
    if value is None or value == "":
        return None
    # 前端、JSON 文件可能把数值保存成字符串，因此统一调用 float。
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    # NaN 和无穷大会污染后续距离计算，同样视为无效坐标。
    return number if not math.isnan(number) and not math.isinf(number) else None


def find_model_origin(draft):
    """按“区域顺序 -> 点位记录顺序”查找第一个有效 RTK 点作为全局原点。"""
    # draft 必须是完整建模字典，否则无法查找区域和点位。
    if not isinstance(draft, dict):
        return None
    # groups 的顺序就是区域创建顺序，因此最先遍历到的是区域1。
    for group in draft.get("groups") or []:
        # points 保持人工打点顺序，因此最先命中的有效点就是区域1第一点。
        for point in group.get("points") or []:
            # 原点必须同时具有有效纬度和经度。
            lat = _number(point.get("lat"))
            lon = _number(point.get("lon"))
            if lat is None or lon is None:
                continue
            # 原点的经纬度保留真实 RTK 值，相对坐标强制定义为 (0, 0)。
            return {
                "pointId": point.get("id"),
                "groupId": group.get("id"),
                "lat": lat,
                "lon": lon,
                "x": 0.0,
                "y": 0.0,
            }
    return None


def lat_lon_to_model_xy_cm(origin, lat, lon):
    """把一个 RTK 点投影到以 origin 为原点的东西/南北厘米坐标系。"""
    # 先验证待转换点的经纬度。
    lat = _number(lat)
    lon = _number(lon)
    if not isinstance(origin, dict) or lat is None or lon is None:
        return None
    # 再验证模型原点的经纬度。
    origin_lat = _number(origin.get("lat"))
    origin_lon = _number(origin.get("lon"))
    if origin_lat is None or origin_lon is None:
        return None
    # 经度方向的实际距离会随纬度变化，因此使用原点和目标点的平均纬度修正。
    mean_lat = math.radians((origin_lat + lat) / 2.0)
    # 经度差、纬度差先由“度”转换为“弧度”，再乘地球半径得到米。
    x_m = math.radians(lon - origin_lon) * EARTH_RADIUS_M * math.cos(mean_lat)
    y_m = math.radians(lat - origin_lat) * EARTH_RADIUS_M
    # 路线规划和下位机任务统一使用厘米，所以最后乘100。
    return x_m * 100.0, y_m * 100.0


def _normalize_point(point, origin, force=False):
    """在不改变原始点位其他字段的前提下，补齐或重算该点的统一 x/y。"""
    # 复制数据，避免坐标标准化过程直接修改调用方持有的对象。
    item = dict(point or {})
    # 只要经纬度有效，就能从统一原点重新得到 x/y。
    xy = lat_lon_to_model_xy_cm(origin, item.get("lat"), item.get("lon"))
    # 旧数据可能已经保存过 x/y；force 决定是否以 RTK 为准强制覆盖。
    has_xy = _number(item.get("x")) is not None and _number(item.get("y")) is not None
    if xy is not None and (force or not has_xy):
        # force=True 可修复旧模型中“每个区域都把自己的第一点写成(0,0)”的问题。
        item["x"] = round(xy[0], 3)
        item["y"] = round(xy[1], 3)
    return item


def normalize_draft_coordinates(draft, force=False):
    """让一份模型中的全部区域点和连接桥点共用同一个模型原点。

    点位 ID、记录顺序、识别角色、区域归属和连接拓扑都保持不变；这里只更新
    x/y 和 coordinateFrame，因此可以同时用于新模型与旧模型坐标迁移。
    """
    # 非字典输入原样返回，保持该工具函数对异常输入的兼容性。
    if not isinstance(draft, dict):
        return draft
    # 深复制保证标准化过程没有外部副作用。
    normalized = copy.deepcopy(draft)
    # 整个模型只能有一个原点。
    origin = find_model_origin(normalized)
    if origin is None:
        return normalized

    # 第一轮处理所有区域点。
    groups = []
    for group in normalized.get("groups") or []:
        item = dict(group)
        item["points"] = [
            _normalize_point(point, origin, force=force)
            for point in (group.get("points") or [])
        ]
        groups.append(item)
    normalized["groups"] = groups

    # 第二轮用完全相同的原点处理所有跨区域连接桥点。
    links = []
    for link in normalized.get("groupLinks") or []:
        item = dict(link)
        item["points"] = [
            _normalize_point(point, origin, force=force)
            for point in (link.get("points") or [])
        ]
        links.append(item)
    normalized["groupLinks"] = links

    # 保存坐标系元数据，便于前端绘图、日志排查和后续旧模型迁移。
    normalized["coordinateFrame"] = {
        "type": "model_origin",
        "unit": "cm",
        "originPointId": origin.get("pointId"),
        "originGroupId": origin.get("groupId"),
        "originLat": round(origin["lat"], 10),
        "originLon": round(origin["lon"], 10),
    }
    return normalized
