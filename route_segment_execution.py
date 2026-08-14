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
    默认按“关清扫 -> 实时点位导航 -> 到点停车 -> 关清扫”执行一段任务。

    边界折线的普通中间点会额外携带 turnAtStart=False、stopAtEnd=False：上一段到点
    后不刹车、不关滚刷，下一段也不做原地转向，而是直接更新点到点目标继续行驶。
    30度以上的明显拐点以及折线终点仍使用默认安全流程停车、转向。

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
    turn_at_start = segment.get("turnAtStart") is not False
    stop_at_end = segment.get("stopAtEnd") is not False

    def report_error(error):
        if callable(on_error):
            on_error(error)

    # 即使输入无效也执行安全收尾，保证测试和真车行为一致。
    if position is None or end_lat is None or end_lon is None or runtime_speed is None:
        report_error(RouteSegmentExecutionError("RTK position, endpoint or speed is invalid"))
        stop_vehicle()
        set_cleaning(False)
        return False

    # 只有需要停车转向的段才先关闭滚刷。连续折线子段保持上一段的清扫状态，
    # 避免每经过一个普通边界采样点都反复开关滚刷。
    if turn_at_start:
        set_cleaning(False)

    def before_drive():
        set_cleaning(mode == 1)

    succeeded = False
    try:
        result = navigate(
            position[0],
            position[1],
            end_lat,
            end_lon,
            runtime_speed,
            before_drive,
        )
        succeeded = result is True or result == 1
        return succeeded
    except Exception as error:
        report_error(error)
        return False
    finally:
        # 失败时无条件安全停车。成功经过普通折线中间点时保持车辆和滚刷运行；
        # 明显转角、折线终点以及所有历史普通任务仍按原逻辑停车并关闭清扫。
        if not succeeded or stop_at_end:
            stop_vehicle()
            set_cleaning(False)
