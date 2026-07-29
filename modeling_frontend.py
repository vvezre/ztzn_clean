# coding=utf-8


def frontend_area_points(draft):
    """Build the flat area-point list expected by the mini-program."""
    points_by_id = {}
    fallback_point_ids = []

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
            fallback_point_ids.append(point_id)

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

    for point_id in fallback_point_ids:
        if point_id not in seen_point_ids:
            ordered_point_ids.append(point_id)
            seen_point_ids.add(point_id)

    result = []
    for sequence, point_id in enumerate(ordered_point_ids, start=1):
        point = points_by_id[point_id]
        result.append({
            'id': point_id,
            'name': u'\u533a\u57df\u70b9{}'.format(sequence),
            'sequence': sequence,
            'x': point.get('x'),
            'y': point.get('y'),
            'lat': point.get('lat'),
            'lon': point.get('lon'),
        })
    return result


def frontend_link_points(draft):
    """Build the flat connection-point list expected by the mini-program."""
    points_by_id = {}
    fallback_point_ids = []

    for link in draft.get('groupLinks') or []:
        if not isinstance(link, dict):
            continue
        for point in link.get('points') or []:
            if not isinstance(point, dict):
                continue
            point_id = point.get('id')
            if not point_id or point_id in points_by_id:
                continue
            points_by_id[point_id] = point
            fallback_point_ids.append(point_id)

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

    for point_id in fallback_point_ids:
        if point_id not in seen_point_ids:
            ordered_point_ids.append(point_id)
            seen_point_ids.add(point_id)

    result = []
    for sequence, point_id in enumerate(ordered_point_ids, start=1):
        point = points_by_id[point_id]
        result.append({
            'id': point_id,
            'name': u'\u8fde\u63a5\u70b9{}'.format(sequence),
            'sequence': sequence,
            'x': point.get('x'),
            'y': point.get('y'),
            'lat': point.get('lat'),
            'lon': point.get('lon'),
        })
    return result


def frontend_path_points(task_plan):
    """Build ordered route points from the robot's executable task segments."""
    result = []
    for task in task_plan.get('tasks') or []:
        if not isinstance(task, dict):
            continue
        for prefix in ('start', 'end'):
            x = task.get(prefix + 'X')
            y = task.get(prefix + 'Y')
            if x is None or y is None:
                continue
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
