# coding=utf-8

import math


# 地球平均半径，单位米。这里用于把小范围经纬度近似转换成局部平面米制坐标。
EARTH_RADIUS_M = 6371000.0


class FilteredRTKPoint(object):
    """保存一次 RTK 滤波结果。

    raw_lat/raw_lon 是原始 RTK 坐标；lat/lon 是滤波后的坐标。
    第一帧还没有历史状态，filtered=False；后续帧经过 Kalman 更新后 filtered=True。
    """

    def __init__(self, raw_lat, raw_lon, lat, lon, timestamp, filtered):
        self.raw_lat = raw_lat
        self.raw_lon = raw_lon
        self.lat = lat
        self.lon = lon
        self.timestamp = timestamp
        self.filtered = filtered


class TrackingCommand(object):
    """直线纠偏计算的输出结果。

    observer_go_correct() 会读取这里的 z_speed 下发给下位机，
    同时读取 distance/cte/heading_error 判断是否到达终点并发布调试状态。
    """

    def __init__(self, raw_lat, raw_lon, filtered_lat, filtered_lon,
                 distance_to_target_m, signed_remaining_m, cte_m,
                 heading_error_deg, z_speed, source):
        self.raw_lat = raw_lat
        self.raw_lon = raw_lon
        self.filtered_lat = filtered_lat
        self.filtered_lon = filtered_lon
        self.distance_to_target_m = distance_to_target_m
        self.signed_remaining_m = signed_remaining_m
        self.cte_m = cte_m
        self.heading_error_deg = heading_error_deg
        self.z_speed = z_speed
        self.source = source


def _normalize_heading_delta(target_heading, current_heading):
    # 把“目标航向 - 当前航向”归一化到 [-180, 180)，方便判断最短转向方向。
    # 例如目标 1°、当前 359°，差值不是 -358°，而是 +2°。
    return (float(target_heading) - float(current_heading) + 180.0) % 360.0 - 180.0


def _latlon_to_local_m(origin_lat, origin_lon, lat, lon):
    # 以 origin_lat/origin_lon 为局部原点，把经纬度换算成米。
    # x 表示东西方向偏移，y 表示南北方向偏移。清扫路径范围较小，所以用平面近似足够。
    lat0_rad = math.radians(float(origin_lat))
    x = math.radians(float(lon) - float(origin_lon)) * EARTH_RADIUS_M * math.cos(lat0_rad)
    y = math.radians(float(lat) - float(origin_lat)) * EARTH_RADIUS_M
    return x, y


def _local_m_to_latlon(origin_lat, origin_lon, x, y):
    # 与 _latlon_to_local_m 相反，把局部米制坐标还原成经纬度。
    # Kalman 滤波内部用 x/y 计算，最终仍需要返回经纬度给外部调试展示。
    lat = float(origin_lat) + math.degrees(float(y) / EARTH_RADIUS_M)
    lat0_rad = math.radians(float(origin_lat))
    lon = float(origin_lon) + math.degrees(float(x) / (EARTH_RADIUS_M * math.cos(lat0_rad)))
    return lat, lon


def _matmul(a, b):
    # 简单矩阵乘法，供 Kalman 滤波使用，避免额外依赖 numpy。
    rows = len(a)
    cols = len(b[0])
    inner = len(b)
    return [
        [sum(a[r][i] * b[i][c] for i in range(inner)) for c in range(cols)]
        for r in range(rows)
    ]


def _transpose(a):
    # 矩阵转置。
    return [list(row) for row in zip(*a)]


def _matadd(a, b):
    # 矩阵加法。
    return [
        [a[r][c] + b[r][c] for c in range(len(a[0]))]
        for r in range(len(a))
    ]


def _matsub(a, b):
    # 矩阵减法。
    return [
        [a[r][c] - b[r][c] for c in range(len(a[0]))]
        for r in range(len(a))
    ]


def _identity(size):
    # 生成单位矩阵。
    return [[1.0 if r == c else 0.0 for c in range(size)] for r in range(size)]


def _inv2(m):
    # 2x2 矩阵求逆，Kalman 更新里的 S 矩阵需要用到。
    # det 太小时做一个极小值保护，避免除 0。
    det = m[0][0] * m[1][1] - m[0][1] * m[1][0]
    if abs(det) < 1e-9:
        det = 1e-9 if det >= 0 else -1e-9
    return [
        [m[1][1] / det, -m[0][1] / det],
        [-m[1][0] / det, m[0][0] / det],
    ]


class RTKKalmanFilter2D(object):
    """二维 RTK Kalman 滤波器。

    状态向量是 [x, y, vx, vy]：
    x/y 表示车辆在局部坐标系中的位置，vx/vy 表示速度。
    作用是平滑 RTK 经纬度跳动，让后面的横向偏差和转向输出更稳定。
    """

    def __init__(self, process_noise=0.2, measurement_noise=2.0, max_dt=1.0):
        # process_noise 越大，越相信车辆运动模型会变化；measurement_noise 越大，越不相信 RTK 测量值。
        self.process_noise = float(process_noise)
        self.measurement_noise = float(measurement_noise)
        # 限制两帧之间最大时间差，避免串口卡顿后一次预测跨度过大。
        self.max_dt = float(max_dt)
        self.origin_lat = None
        self.origin_lon = None
        self.state = None
        self.covariance = None
        self.last_timestamp = None

    def reset(self):
        # 每条新路径段开始时调用，清空上一段的滤波历史。
        self.origin_lat = None
        self.origin_lon = None
        self.state = None
        self.covariance = None
        self.last_timestamp = None

    def update(self, lat, lon, timestamp=None):
        timestamp = 0.0 if timestamp is None else float(timestamp)
        lat = float(lat)
        lon = float(lon)
        if self.state is None:
            # 第一帧作为局部坐标原点，还没有历史速度可用，所以直接返回原始点。
            self.origin_lat = lat
            self.origin_lon = lon
            # 状态向量：[x, y, vx, vy]，第一帧位置为原点，速度先置 0。
            self.state = [[0.0], [0.0], [0.0], [0.0]]
            self.covariance = [
                [self.measurement_noise, 0.0, 0.0, 0.0],
                [0.0, self.measurement_noise, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
            self.last_timestamp = timestamp
            return FilteredRTKPoint(lat, lon, lat, lon, timestamp, False)

        dt = timestamp - self.last_timestamp
        if dt <= 0.0:
            # 时间戳异常或重复时给一个很小的默认步长，保证矩阵计算还能继续。
            dt = 0.05
        dt = min(dt, self.max_dt)
        self.last_timestamp = timestamp

        # 当前 RTK 测量值先转换成局部米制坐标，Kalman 内部只处理 x/y。
        measurement_x, measurement_y = _latlon_to_local_m(
            self.origin_lat,
            self.origin_lon,
            lat,
            lon,
        )
        # F 是状态转移矩阵：x = x + vx * dt，y = y + vy * dt。
        f = [
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
        # H 是观测矩阵：RTK 只能直接测到位置 x/y，不能直接测到速度 vx/vy。
        h = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
        ]
        # Q 是过程噪声，表示“模型预测本身的不确定性”。
        q = [
            [self.process_noise * dt * dt, 0.0, 0.0, 0.0],
            [0.0, self.process_noise * dt * dt, 0.0, 0.0],
            [0.0, 0.0, self.process_noise * dt, 0.0],
            [0.0, 0.0, 0.0, self.process_noise * dt],
        ]
        # R 是测量噪声，表示“RTK 坐标测量值的不确定性”。
        r = [
            [self.measurement_noise, 0.0],
            [0.0, self.measurement_noise],
        ]

        # 预测步骤：根据上一帧状态和速度预测当前状态。
        self.state = _matmul(f, self.state)
        self.covariance = _matadd(_matmul(_matmul(f, self.covariance), _transpose(f)), q)

        # 更新步骤：用当前 RTK 测量值修正预测值。
        z = [[measurement_x], [measurement_y]]
        # innovation 是“实际测量值 - 预测测量值”，也就是这次 RTK 对预测的纠偏量。
        innovation = _matsub(z, _matmul(h, self.state))
        s = _matadd(_matmul(_matmul(h, self.covariance), _transpose(h)), r)
        # K 是 Kalman 增益，决定这次更相信预测还是更相信 RTK 测量。
        k = _matmul(_matmul(self.covariance, _transpose(h)), _inv2(s))
        self.state = _matadd(self.state, _matmul(k, innovation))
        self.covariance = _matmul(_matsub(_identity(4), _matmul(k, h)), self.covariance)

        # 把滤波后的 x/y 位置转回经纬度，供后续路径控制和调试输出使用。
        filtered_lat, filtered_lon = _local_m_to_latlon(
            self.origin_lat,
            self.origin_lon,
            self.state[0][0],
            self.state[1][0],
        )
        return FilteredRTKPoint(lat, lon, filtered_lat, filtered_lon, timestamp, True)


class StraightLinePController(object):
    """直线路径 P 控制器。

    输入当前点、目标线段和车辆航向，输出 z_speed。
    z_speed 后续会写入 command 的 Z 速度字段，让车一边前进一边修正方向。
    """

    def __init__(self, heading_gain=10.0, cte_gain=1000.0,
                 short_range_heading_limit_deg=5.0, max_z_speed=15000):
        # 航向误差增益：航向偏差每多 1 度，转向输出增加 heading_gain。
        self.heading_gain = float(heading_gain)
        # 横向偏差增益：偏离目标直线越多，转向修正越强。
        self.cte_gain = float(cte_gain)
        # 接近终点时限制航向误差，防止最后阶段大幅摆动。
        self.short_range_heading_limit_deg = float(short_range_heading_limit_deg)
        # 限制最终 z_speed 的绝对值，避免转向命令过大。
        self.max_z_speed = int(max_z_speed)

    def compute(self, start_lat, start_lon, end_lat, end_lon, current_lat, current_lon,
                vehicle_heading, target_heading=None, raw_lat=None, raw_lon=None):
        # 以路径起点为局部坐标原点，把终点和当前点都转换成米制坐标。
        end_x, end_y = _latlon_to_local_m(start_lat, start_lon, end_lat, end_lon)
        cur_x, cur_y = _latlon_to_local_m(start_lat, start_lon, current_lat, current_lon)
        path_length = math.hypot(end_x, end_y)
        if path_length <= 1e-6:
            # 起点和终点几乎重合，路径没有方向，直接返回 0 修正。
            return TrackingCommand(
                raw_lat=float(raw_lat if raw_lat is not None else current_lat),
                raw_lon=float(raw_lon if raw_lon is not None else current_lon),
                filtered_lat=float(current_lat),
                filtered_lon=float(current_lon),
                distance_to_target_m=0.0,
                signed_remaining_m=0.0,
                cte_m=0.0,
                heading_error_deg=0.0,
                z_speed=0,
                source="straight_line_p_degenerate_path",
            )

        # 目标路径单位向量，表示“从起点指向终点”的方向。
        unit_x = end_x / path_length
        unit_y = end_y / path_length
        # 当前点投影到目标路径上的距离，表示车辆沿路径方向已经走了多少米。
        projection_m = cur_x * unit_x + cur_y * unit_y
        # cte 是横向偏差，单位米。它表示车辆偏离“起点 -> 终点”目标直线的距离。
        # 正负号代表偏在目标线的哪一侧，后面会影响 z_speed 的修正方向。
        cte = unit_y * cur_x - unit_x * cur_y
        # signed_remaining 是沿路径方向还剩多少米；小于等于 0 表示已经到达或越过终点投影。
        signed_remaining = path_length - projection_m
        # 当前点到终点的真实直线距离，用于靠近终点降速和完成判断。
        distance_to_target = math.hypot(end_x - cur_x, end_y - cur_y)
        # 优先使用任务生成时给出的目标航向；没有传入时，根据起点到终点方向临时计算。
        desired_heading = float(target_heading) if target_heading is not None else math.degrees(math.atan2(end_x, end_y)) % 360.0
        # heading_error 是目标航向和当前航向的最短角度差，范围 [-180, 180)。
        heading_error = _normalize_heading_delta(desired_heading, vehicle_heading)
        if distance_to_target < 1.0:
            # 接近终点时限制航向误差，避免为追求角度而在终点附近剧烈摆动。
            limit = abs(self.short_range_heading_limit_deg)
            heading_error = max(-limit, min(limit, heading_error))
        # Matches the server backup active observer: compute_linear_steering(...)
        # was negated before being sent to setZSpeed.
        # P 控制输出：航向误差负责“车头朝向”，横向偏差负责“车身位置”。
        # cte 前面是减号，表示偏到某一侧时要往反方向修正。
        z_speed = int(round(float(heading_error) * self.heading_gain - float(cte) * self.cte_gain))
        # 对转向输出做限幅，保护下位机和车辆动作稳定性。
        z_speed = max(-self.max_z_speed, min(self.max_z_speed, z_speed))

        return TrackingCommand(
            raw_lat=float(raw_lat if raw_lat is not None else current_lat),
            raw_lon=float(raw_lon if raw_lon is not None else current_lon),
            filtered_lat=float(current_lat),
            filtered_lon=float(current_lon),
            distance_to_target_m=distance_to_target,
            signed_remaining_m=signed_remaining,
            cte_m=cte,
            heading_error_deg=heading_error,
            z_speed=z_speed,
            source="straight_line_p_control",
        )


def build_tracking_command(rtk_filter, tracker, start_lat, start_lon, end_lat, end_lon,
                           current_lat, current_lon, vehicle_heading, target_heading=None,
                           timestamp=None):
    """直行纠偏总入口。

    observer_go_correct() 每收到一帧 RTK 数据都会调用这里：
    1. 先对当前 RTK 坐标做 Kalman 滤波；
    2. 再按“起点 -> 终点”目标直线计算横向偏差、航向误差和 z_speed；
    3. 返回 TrackingCommand，外层再把 z_speed 下发给下位机。
    """

    # 第一步：滤波当前 RTK 坐标，减少原始经纬度跳动对控制输出的影响。
    filtered = rtk_filter.update(current_lat, current_lon, timestamp=timestamp)
    # 第二步：使用滤波后的坐标计算直线 P 控制命令。
    command = tracker.compute(
        start_lat=start_lat,
        start_lon=start_lon,
        end_lat=end_lat,
        end_lon=end_lon,
        current_lat=filtered.lat,
        current_lon=filtered.lon,
        vehicle_heading=vehicle_heading,
        target_heading=target_heading,
        raw_lat=current_lat,
        raw_lon=current_lon,
    )
    command.source = "kalman_p_control"
    return command
