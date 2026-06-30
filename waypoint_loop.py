def _as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value or '').strip().lower() in ('1', 'true', 'yes', 'on')


def normalize_loop_options(payload, waypoint_count):
    payload = payload if isinstance(payload, dict) else {}
    loop_enabled = _as_bool(payload.get('loop'))
    loop_mode = str(payload.get('loopMode') or 'count').strip().lower()

    if not loop_enabled:
        return {
            'loop': False,
            'loopMode': 'count',
            'loopCount': 1,
        }

    if waypoint_count < 2:
        raise ValueError('closed-loop mode requires at least two waypoints')
    if loop_mode not in ('count', 'continuous'):
        raise ValueError('invalid loop mode')
    if loop_mode == 'continuous':
        return {
            'loop': True,
            'loopMode': 'continuous',
            'loopCount': 0,
        }

    try:
        loop_count = int(payload.get('loopCount'))
    except (TypeError, ValueError):
        loop_count = 0
    if loop_count <= 0:
        raise ValueError('loop count must be greater than 0')

    return {
        'loop': True,
        'loopMode': 'count',
        'loopCount': loop_count,
    }


def iter_closed_loop_targets(waypoints, loop_count=None):
    if len(waypoints) < 2:
        raise ValueError('closed-loop mode requires at least two waypoints')
    if loop_count is not None and loop_count <= 0:
        raise ValueError('loop count must be greater than 0')

    yield {
        'index': 0,
        'waypoint': waypoints[0],
        'completed_loop': 0,
    }
    cycle = 0
    while loop_count is None or cycle < loop_count:
        for index in range(1, len(waypoints)):
            yield {
                'index': index,
                'waypoint': waypoints[index],
                'completed_loop': cycle,
            }
        cycle += 1
        yield {
            'index': 0,
            'waypoint': waypoints[0],
            'completed_loop': cycle,
        }


def build_closed_loop_targets(waypoints, loop_count):
    return list(iter_closed_loop_targets(waypoints, loop_count))
