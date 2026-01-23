from geographiclib.geodesic import Geodesic
import serial
import math
from serial.serialutil import SerialException
import time


# pip install geographiclib


port = 'COM9'
baudrate = 115200

# 已知起点，角度，距离，计算终点GPS
#使用示例：
# lat3, lon3 = get_B_GPS(lat1, lon1, distance_m, azimuth_deg)
# print(f"终点GPS: lat3={lat3:.8f}, lon3={lon3:.8f}")
def get_B_GPS(lat1, lon1, distance_m, azimuth_deg):
    geod = Geodesic.WGS84
    result = geod.Direct(lat1, lon1, azimuth_deg, distance_m)
    return result['lat2'], result['lon2']

# 已知起点，终点 计算 方向角、距离
#s12距离，azi1角度
# 使用示例：
# distance, angle = get_distance_angle(lat1, lon1, lat2, lon2)
# print(f"实际距离: {distance:.4f} m")
# print(f"起点→终点方向角: {angle:.4f}°")
def get_distance_angle(lat1, lon1, lat2, lon2):
    geod = Geodesic.WGS84
    result = geod.Inverse(lat1, lon1, lat2, lon2)
    return result['s12'], result['azi1']

#分析gngga数据
def parse_gngga(line):
    # line = "$GNGGA,070622.00,3202.20138810,N,11855.48034587,E,4,30,0.6,40.6076,M,2.7092,M,1.0,569*60"
    if not line.startswith("$GNGGA"):
        return None
    parts = line.strip().split(',')
    if len(parts) < 7:
        return None

    # 检查 E,4 条件：E方向且Fix类型为4
    if parts[5] != 'E' or parts[6] != '4':
        return None

    try:
        lat_raw = float(parts[2])
        lon_raw = float(parts[4])
        lat = int(lat_raw / 100) + (lat_raw % 100) / 60
        lon = int(lon_raw / 100) + (lon_raw % 100) / 60
        return (round(lat, 8), round(lon, 8))
    except:
        return None

#分析uniheadinga数据
def parse_uniheadinga(line):
    if not line.startswith("#UNIHEADINGA"):
        return None
    parts = line.strip().split(',')
    if len(parts) < 13:
        return None
    try:
        heading = float(parts[12])  # 第13个字段是角度
        return heading
    except:
        return None

#分析GPTHS数据
def parse_GPTHS(line):
    if not line.startswith("$GPTHS"):
        return None
    parts = line.strip().split(',')
    if len(parts) < 3:
        return None
    try:
        heading = float(parts[1])  # 第1个字段是角度
        return heading
    except:
        return None

#根据斜边和夹角求临边。如斜边10cm，夹角60度
#cos(10,60)
def cos(length, angle_degrees ):
    """
    根据夹角（角度）和斜边长计算邻边长度
    参数:
        angle_degrees (float): 夹角的角度值（单位：度）
        hypotenuse (float): 斜边的长度
    返回:
        float: 邻边的长度
    """
    # 将角度转换为弧度
    angle_radians = math.radians(angle_degrees)

    # 使用余弦公式计算邻边：邻边 = 斜边 * cos(夹角)
    adjacent = length * math.cos(angle_radians)

    return adjacent

#重连
def try_connect(port, baudrate):
    try:
        return serial.Serial(port, baudrate, timeout=1)
    except SerialException as e:
        print(f"[连接失败] 无法打开串口 {port}，原因：{e}")
        return None

def main():
    ser = None

    while True:
        if ser is None or not ser.is_open:
            print("[尝试连接]...")
            ser = try_connect(port, baudrate)
            if ser is None:
                time.sleep(1)  # 失败后等待一段时间再重试
                continue
            print("[连接成功] 串口已打开")
        try:
            line = ser.readline().decode(errors='ignore').strip()
            if not line:
                continue

            if line.startswith("$GNGGA"):
                gps = parse_gngga(line)
                if gps:
                    print(f"[GNGGA] Lat: {gps[0]}°, Lon: {gps[1]}°")

            elif line.startswith("#UNIHEADINGA"):
                heading = parse_uniheadinga(line)
                if heading is not None:
                    print(f"[UNIHEADINGA] Heading: {heading}°")

            elif line.startswith("$GPTHS"):
                heading = parse_GPTHS(line)
                if heading is not None:
                    print(f"[GPTHS] Heading: {heading}°")
        except KeyboardInterrupt:
            print("Stopped by user.")
            break

        except SerialException as e:
            print(f"[串口异常] {e}")
            if ser:
                ser.close()
            ser = None
            time.sleep(1)  # 等待后重新连接

        except Exception as e:
            print(f"Error: {e}")



if __name__ == "__main__":
    main()
