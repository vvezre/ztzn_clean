# coding=utf-8
"""与硬件无关的单段路线执行顺序。

生产环境通过回调接入实时 RTK、点位导航、清扫开关和刹车；测试环境使用假的回调，
因此可以验证真实生产调用顺序，而不需要连接串口、RTK 或小车。
"""


class RouteSegmentExecutionError(Exception):
    pass


def _number(value):
    if value is None or value == "":
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _positive_int(value):
    try:
        result = int(value)
    except (TypeError, ValueError):
        return None
    return result if result > 0 else None


def _position_pair(value):
    if isinstance(value, dict):
        lat = _number(value.get("lat"))
        lon = _number(value.get("lon"))
    elif isinstance(value, (list, tuple)) and len(value) >= 2:
        lat = _number(value[0])
        lon = _number(value[1])
    else:
        return None
    return (lat, lon) if lat is not None and lon is not None else None


def run_route_segment(
        segment,
        speed,
        read_position,
        navigate,
        set_cleaning,
        stop_vehicle,
        on_error=None):
    """
    按“关清扫 -> 实时点位导航 -> 到点停车 -> 关清扫”执行一段任务。

    navigate 接收六个参数：当前纬度、当前经度、目标纬度、目标经度、速度、
    before_drive 回调。点位导航应先完成转向，再调用 before_drive，最后开始直行。
    segment 中预存的 heading/angle 不参与控制，避免旧航向或坐标约定影响真车转向。
    """
    if not isinstance(segment, dict):
        raise RouteSegmentExecutionError("route segment must be object")
    for callback, name in (
            (read_position, "read_position"),
            (navigate, "navigate"),
            (set_cleaning, "set_cleaning"),
            (stop_vehicle, "stop_vehicle")):
        if not callable(callback):
            raise RouteSegmentExecutionError("{} callback is required".format(name))

    position = _position_pair(read_position())
    end_lat = _number(segment.get("endLat"))
    end_lon = _number(segment.get("endLon"))
    runtime_speed = _positive_int(speed)
    mode = _positive_int(segment.get("mode")) or 2

    def report_error(error):
        if callable(on_error):
            on_error(error)

    # 即使输入无效也执行安全收尾，保证测试和真车行为一致。
    if position is None or end_lat is None or end_lon is None or runtime_speed is None:
        report_error(RouteSegmentExecutionError("RTK position, endpoint or speed is invalid"))
        stop_vehicle()
        set_cleaning(False)
        return False

    set_cleaning(False)

    def before_drive():
        set_cleaning(mode == 1)

    try:
        result = navigate(
            position[0],
            position[1],
            end_lat,
            end_lon,
            runtime_speed,
            before_drive,
        )
        return result is True or result == 1
    except Exception as error:
        report_error(error)
        return False
    finally:
        stop_vehicle()
        set_cleaning(False)
