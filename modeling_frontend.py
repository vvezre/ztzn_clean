# coding=utf-8

"""把 FSM 内部建模数据整理成前端可直接使用的三个点位列表。

FSM 内部为了执行路线，会保留区域分组、连接桥、任务线段等结构化数据；这些结构适合
机器人计算，但前端画图只需要按记录/行驶顺序排列的扁平点数组。本模块只负责格式转换，
不会重新规划路径，也不会改变小车实际执行的任务。

三个公开函数分别产生：
    areaPoints：用户建模时记录的区域边界点；
    linkPoints：不同区域之间记录的连接桥点；
    pathPoints：任务线段首尾点展开后的有序路径点。

所有列表中的 x/y 都沿用 FSM 的局部坐标（厘米），lat/lon 沿用 RTK 经纬度。
"""


def frontend_area_points(draft):
    """返回前端需要的全部区域点，并保持用户实际记录顺序。

    参数 draft 是当前建模草稿。区域点实际存放在 ``groups[*].points`` 中，而用户跨区域
    记录点的先后顺序存放在 ``captureSequence`` 中，所以这里分两步处理：

    1. 先从所有区域组收集点对象，并用 id 去重；
    2. 再按 captureSequence 还原全局记录顺序；旧草稿若没有完整顺序，则把遗漏点补到末尾。

    返回的每个元素固定包含 id、name、sequence、x、y、lat、lon，前端可直接列表展示、
    标号或连线。id 是删除点位时使用的唯一标识，sequence 只是当前列表展示顺序。
    """
    # points_by_id 保存唯一点对象；fallback_point_ids 保存草稿中的自然遍历顺序，
    # 用于兼容早期没有 captureSequence 字段的建模数据。
    points_by_id = {}
    area_number_by_point_id = {}
    fallback_point_ids = []

    # 同一个 id 只收录一次，防止异常草稿让前端出现重复点。
    for group in draft.get('groups') or []:
        if not isinstance(group, dict):
            continue
        for point in group.get('points') or []:
            if not isinstance(point, dict):
                continue
            point_id = point.get('id')
            if not point_id or point_id in points_by_id:
                continue
            points_by_id[point_id] = point
            area_number_by_point_id[point_id] = group.get('areaNumber')
            fallback_point_ids.append(point_id)

    # captureSequence 同时包含 area/link 两种事件；本函数只提取区域点事件。
    ordered_point_ids = []
    seen_point_ids = set()
    for event in draft.get('captureSequence') or []:
        if not isinstance(event, dict) or event.get('pointType') != 'area':
            continue
        point_id = event.get('pointId')
        if point_id not in points_by_id or point_id in seen_point_ids:
            continue
        ordered_point_ids.append(point_id)
        seen_point_ids.add(point_id)

    # 对没有写入 captureSequence 的旧点位进行兜底，确保“查询全部区域点”不会漏数据。
    for point_id in fallback_point_ids:
        if point_id not in seen_point_ids:
            ordered_point_ids.append(point_id)
            seen_point_ids.add(point_id)

    # sequence 从 1 开始，name 仅用于前端显示；真正删除和定位仍以 id 为准。
    result = []
    for sequence, point_id in enumerate(ordered_point_ids, start=1):
        point = points_by_id[point_id]
        result.append({
            'id': point_id,
            'name': u'\u533a\u57df\u70b9{}'.format(sequence),
            'sequence': sequence,
            'areaNumber': area_number_by_point_id.get(point_id),
            'x': point.get('x'),
            'y': point.get('y'),
            'lat': point.get('lat'),
            'lon': point.get('lon'),
        })
    return result


def frontend_link_points(draft):
    """返回前端需要的全部连接点，并保持用户实际记录顺序。

    连接桥在 FSM 内部按 ``groupLinks`` 分组，每条桥可以包含两个或更多点；这里不会强制
    把连接桥限制为两个点，而是把所有桥点展开为一个列表。这样既保留现有多点连接能力，
    也符合前端“查询全部连接点、按 id 删除”的使用方式。

    排序和兼容策略与 :func:`frontend_area_points` 一致：优先使用 captureSequence，
    没有顺序记录的历史点追加到末尾。
    """
    # 先建立 id -> 点对象索引，保证列表里的 id 唯一。
    points_by_id = {}
    link_number_by_point_id = {}
    fallback_point_ids = []

    for fallback_link_number, link in enumerate(draft.get('groupLinks') or [], start=1):
        if not isinstance(link, dict):
            continue
        link_number = link.get('linkNumber') or fallback_link_number
        for point in link.get('points') or []:
            if not isinstance(point, dict):
                continue
            point_id = point.get('id')
            if not point_id or point_id in points_by_id:
                continue
            points_by_id[point_id] = point
            link_number_by_point_id[point_id] = link_number
            fallback_point_ids.append(point_id)

    # 只读取 pointType=link 的记录事件，区域点不会混入连接点接口。
    ordered_point_ids = []
    seen_point_ids = set()
    for event in draft.get('captureSequence') or []:
        if not isinstance(event, dict) or event.get('pointType') != 'link':
            continue
        point_id = event.get('pointId')
        if point_id not in points_by_id or point_id in seen_point_ids:
            continue
        ordered_point_ids.append(point_id)
        seen_point_ids.add(point_id)

    # 兼容历史草稿：即使缺少 captureSequence，也返回 groupLinks 中实际存在的点。
    for point_id in fallback_point_ids:
        if point_id not in seen_point_ids:
            ordered_point_ids.append(point_id)
            seen_point_ids.add(point_id)

    # name/sequence 是展示字段；id 是前端删除连接点时必须回传的唯一键。
    result = []
    for sequence, point_id in enumerate(ordered_point_ids, start=1):
        point = points_by_id[point_id]
        result.append({
            'id': point_id,
            'name': u'\u8fde\u63a5\u70b9{}'.format(sequence),
            'sequence': sequence,
            'linkNumber': link_number_by_point_id.get(point_id),
            'x': point.get('x'),
            'y': point.get('y'),
            'lat': point.get('lat'),
            'lon': point.get('lon'),
        })
    return result


def frontend_path_points(task_plan):
    """把机器人可执行线段展开为前端可连续连线的路径点。

    FSM 的 ``taskPlan.tasks`` 不是“一个点数组”，而是一组按执行顺序排列的线段。每段包含
    startX/startY 和 endX/endY（以及对应经纬度）。本函数按任务顺序依次取“起点、终点”，
    得到前端要求的 pathPoints。

    相邻任务通常满足“上一段终点 = 下一段起点”。若不去重，同一转折点会连续出现两次，
    所以这里只删除相邻且 x/y 完全相同的重复点。它不会改变任务顺序，不会平滑、旋转或
    重新计算路线；前端依次连接返回点，画出的就是 FSM 当前生成的线段顺序。
    """
    result = []
    # task 的先后顺序就是机器人执行顺序，不能在这里按坐标重新排序。
    for task in task_plan.get('tasks') or []:
        if not isinstance(task, dict):
            continue
        # 同一套代码分别读取 startX/startY/startLat/startLon 和
        # endX/endY/endLat/endLon，避免两个端点的字段处理不一致。
        for prefix in ('start', 'end'):
            x = task.get(prefix + 'X')
            y = task.get(prefix + 'Y')
            if x is None or y is None:
                continue
            # 仅压缩连续重复端点；非连续地再次经过同一点仍必须保留，才能画出回程路线。
            if result and result[-1].get('x') == x and result[-1].get('y') == y:
                continue
            sequence = len(result) + 1
            result.append({
                'id': 'p{}'.format(sequence),
                'name': u'\u8def\u5f84\u70b9{}'.format(sequence),
                'sequence': sequence,
                'x': x,
                'y': y,
                'lat': task.get(prefix + 'Lat'),
                'lon': task.get(prefix + 'Lon'),
            })
    return result
