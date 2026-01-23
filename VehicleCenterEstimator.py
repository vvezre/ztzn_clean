# coding=utf-8
import numpy as np
from pyproj import Proj

import util


class VehicleCenterEstimator:
    """
    从 RTK 后置天线的经纬度 + 航向角，实时估算车辆几何中心位置。

    假设：
      - 主天线（RTK 输出）位于车辆后部
      - 车辆中心在前后天线连线中点（可自定义偏移）
      - 使用 WGS84 椭球模型
    """

    def __init__(self, baseline_length=1.2, center_offset_from_rear=None):
        """
        初始化估计器

        参数:
            baseline_length (float): 前后天线距离（米），默认 1.2m
            center_offset_from_rear (float or None):
                从后天线到车辆中心的距离（米）。若为 None，则自动设为 baseline_length / 2
        """
        self.baseline_length = baseline_length
        if center_offset_from_rear is None:
            self.center_offset = baseline_length / 2.0
        else:
            self.center_offset = center_offset_from_rear

        # 缓存 UTM 投影对象，避免重复创建
        self._utm_proj = None
        self._utm_zone = None

    def _get_utm_proj(self, lon):
        """根据经度动态获取 UTM 投影（自动判断带号）"""
        zone = int((lon + 180) // 6) + 1
        if self._utm_proj is None or self._utm_zone != zone:
            self._utm_zone = zone
            self._utm_proj = Proj(proj='utm', zone=zone, ellps='WGS84', datum='WGS84')
        return self._utm_proj, self._utm_zone

    def latlon_to_utm(self, lat, lon):
        """经纬度 → UTM (Easting, Northing)"""
        utm_proj, zone = self._get_utm_proj(lon)
        easting, northing = utm_proj(lon, lat)
        return np.array([easting, northing]), zone

    def utm_to_latlon(self, easting, northing):
        """UTM → 经纬度"""
        if self._utm_proj is None:
            raise ValueError("UTM 投影未初始化，请先调用 latlon_to_utm")
        lon, lat = self._utm_proj(easting, northing, inverse=True)
        return lat, lon

    def estimate_center_from_rear_antenna(self, rear_lat, rear_lon, heading_deg):
        """
        核心方法：从后天线 RTK 位置 + 航向，计算车辆中心经纬度

        参数:
            rear_lat (float): 后天线纬度（度）
            rear_lon (float): 后天线经度（度）
            heading_deg (float): 航向角（0°=北，顺时针）

        返回:
            dict: {
                'center_lat': ...,
                'center_lon': ...,
                'center_utm': [E, N],
                'rear_utm': [E, N],
                'utm_zone': ...
            }
        """
        # 1. 转 UTM
        rear_utm, zone = self.latlon_to_utm(rear_lat, rear_lon)

        # 2. 补偿到中心（车体坐标系偏移）
        offset_body = np.array([self.center_offset, 0.0])  # [前, 右]
        theta = np.radians(90 - heading_deg)  # 航向 → 数学角
        R = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
        ])
        # offset_geo = R @ offset_body
        offset_geo = np.dot(R, offset_body)
        center_utm = rear_utm + offset_geo

        # 3. 转回经纬度
        center_lat, center_lon = self.utm_to_latlon(center_utm[0], center_utm[1])

        return {
            'center_lat': float(center_lat),
            'center_lon': float(center_lon),
            'center_utm': center_utm,
            'rear_utm': rear_utm,
            'utm_zone': zone
        }


# ==============================
# 示例使用
# ==============================

if __name__ == "__main__":
    # 创建估计器：基线 0.2m，中心在后天线前 0.1m
    # estimator = VehicleCenterEstimator(baseline_length=0.23)
    estimator = VehicleCenterEstimator(baseline_length=0.23)

    # 模拟 RTK 数据
    lat1 = 32.03656709
    lon1 = 118.92460540
    heading1 = 275.07

    # 估算中心
    result = estimator.estimate_center_from_rear_antenna(lat1, lon1, heading1)

    print("✅ 车辆中心位置:")
    print("   纬度: {:.8f}°".format(result['center_lat']))
    print("   经度: {:.8f}°".format(result['center_lon']))
    print("   UTM: E={:.3f}, N={:.3f} (Zone {}N)".format(result['center_utm'][0],result['center_utm'][1],result['utm_zone']))


    lat2 = 32.03656860
    lon2 = 118.92460220
    heading2 = 2.56

    # 估算中心
    result2 = estimator.estimate_center_from_rear_antenna(lat2, lon2, heading2)
    r = util.get_distance_angle(result['center_lat'], result['center_lon'], result2['center_lat'],
                                     result2['center_lon'])
    print(r)

    lat3 = 32.03657101
    lon3 = 118.92460375
    heading3 = 95.72

    # 估算中心
    result3 = estimator.estimate_center_from_rear_antenna(lat3, lon3, heading3)
    r = util.get_distance_angle(result['center_lat'], result['center_lon'], result3['center_lat'],
                                result3['center_lon'])
    print(r)

    lat4 = 32.03657001
    lon4 = 118.92460675
    heading4 = 185.70

    # 估算中心
    result3 = estimator.estimate_center_from_rear_antenna(lat4, lon4, heading4)
    r = util.get_distance_angle(result['center_lat'], result['center_lon'], result3['center_lat'],
                                result3['center_lon'])
    print(r)