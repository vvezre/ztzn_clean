# coding=utf-8
import time

import serial

import util
from AppLogger import logger

ser_rtk = None

def readRTK(ser_rtk_params):
    global ser_rtk
    raw_buffer = b''

    while True:
        try:
            if ser_rtk is None or not ser_rtk.is_open:
                ser_rtk = serial.Serial(ser_rtk_params['port'], ser_rtk_params['baudRate'], rtscts=True, timeout=0.01)

            raw_buffer += ser_rtk.read(ser_rtk.in_waiting)
            # 按帧分割处理
            while b'\n' in raw_buffer:
                line, raw_buffer = raw_buffer.split(b'\n', 1)
                if line:
                    print(line)
        except serial.SerialException as e:
            logger.error("RTK串口异常：{}".format(e))
            if ser_rtk:
                ser_rtk.close()
            ser_rtk = None
        except Exception as e:
            logger.error("RTK处理过程未知错误：{}".format(e))
            if ser_rtk:
                ser_rtk.close()
            ser_rtk = None

        time.sleep(0.01)


def rtk_thread(ser_rtk_params,sync_threshold = 0.5):
    global ser_rtk
    buffer = {}  # 缓存最近的 GGA 和 THS 数据

    while True:
        try:
            rtk_port = util.findPort('$GN')
            if rtk_port:
                # 使用 with 语句，离开代码块时会自动 close，即使发生异常
                with serial.Serial(rtk_port, ser_rtk_params['baudRate'], timeout=1) as ser_rtk:
                    print("串口已打开")
                    while ser_rtk.is_open:
                        try:
                            if ser_rtk.in_waiting:
                                line = ser_rtk.readline()
                                # # 处理数据...
                                if line:
                                    timestamp = time.time()  # 记录当前时间戳
                                    # logger.info(line)
                                    if line.startswith(b"$GNGGA"):
                                        gps = parse_gngga(line.decode('utf-8', errors='ignore').strip())
                                        if gps:
                                            buffer['gga'] = {
                                                'timestamp': timestamp,
                                                'lat': gps[0],
                                                'lon': gps[1]
                                            }
                                            # yield gps[0], gps[1]
                                    elif line.startswith(b"$GNTHS"):
                                        heading = parse_GPTHS(line.decode('utf-8', errors='ignore').strip())
                                        buffer['ths'] = {
                                            'timestamp': timestamp,
                                            'heading': heading
                                        }

                                    # 同步 GGA 和 VTG 数据
                                    if 'gga' in buffer and 'ths' in buffer:
                                        gga_ts = buffer['gga']['timestamp']
                                        ths_ts = buffer['ths']['timestamp']

                                        if abs(gga_ts - ths_ts) < sync_threshold:  # 时间差阈值为 0.5 秒
                                            lat = buffer['gga']['lat']
                                            lon = buffer['gga']['lon']
                                            heading_deg = buffer['ths']['heading']

                                            # 清空缓冲区，准备接收下一次数据
                                            del buffer['gga']
                                            del buffer['ths']

                                            yield lat, lon, heading_deg
                        except Exception as e:
                            logger.error("读取数据异常: {}".format(e))
                            break  # 跳出内层循环，重新打开串口
                # 离开 with 块，串口自动关闭
        except serial.SerialException as e:
            logger.error("打开串口失败: {}".format(e))
        except Exception as e:
            logger.error("其他错误: {}".format(e))

        time.sleep(1)  # 确保休眠在所有情况（包括失败）下都执行，防止死循环

def parse_gngga(line):
    """
    解析 $GNGGA 语句，仅返回 RTK Fixed (quality=4) 且有效坐标的 (lat, lon)

    Args:
        line (str): NMEA GGA 语句

    Returns:
        tuple: (lat, lon) in decimal degrees, or None if invalid
    """
    if not line or not line.startswith("$GNGGA"):
        return None

    parts = line.strip().split(',')
    if len(parts) < 10:
        return None

    # 检查定位质量：4 = RTK Fixed
    if parts[6] != '4':
        return None

    # 检查经纬度字段是否为空
    lat_str = parts[2]
    lon_str = parts[4]
    if not lat_str or not lon_str:
        return None

    try:
        # 解析纬度：ddmm.mmmm -> 度
        lat_deg = int(float(lat_str) / 100)
        lat_min = float(lat_str) - lat_deg * 100
        lat = lat_deg + lat_min / 60.0
        if parts[3] == 'S':  # 南纬为负
            lat = -lat

        # 解析经度：dddmm.mmmm -> 度
        lon_deg = int(float(lon_str) / 100)
        lon_min = float(lon_str) - lon_deg * 100
        lon = lon_deg + lon_min / 60.0
        if parts[5] == 'W':  # 西经为负
            lon = -lon

        return (round(lat, 8), round(lon, 8))

    except (ValueError, IndexError, TypeError) as e:
        # 可选：记录日志用于调试
        # logger.debug(f"Failed to parse GNGGA: {line}, error: {e}")
        return None


def parse_GPTHS(line):
    if not line.startswith("$GNTHS"):
        return None
    parts = line.strip().split(',')
    if len(parts) < 3:
        return None
    try:
        heading = float(parts[1])  # 第1个字段是角度
        return heading
    except:
        return None

def process_line(line,buffer,timestamp):
    try:
        # 安全解码：忽略非法字符
        line_str = line.decode('ascii', errors='ignore').strip()

        # 跳过校验和无效行（可选增强）
        if not line_str or '*' not in line_str:
            return

        # 根据语句头分发处理
        if line_str.startswith('$GNGGA'):
            result = parse_gngga(line_str)
            if result:
                buffer['gga'] = {
                    'timestamp': timestamp,
                    'lat': result[0],
                    'lon': result[1]
                }
        elif line_str.startswith('$GNTHS'):  # 自定义航向语句（如有）
            heading = parse_GPTHS(line_str)  # ← 保留兼容性
            if heading is not None:
                buffer['ths'] = {
                    'timestamp': timestamp,
                    'heading': heading
                }

        # 可扩展：添加 $GNRMC, $GPGSV 等其他语句

    except Exception as e:
        # 不应崩溃，仅记录异常
        logger.debug("解析 NMEA 行失败: {}, error: {}".format(line_str, e))

def readRTK2(ser_rtk_params, sync_threshold=0.5, timeout=5):
    """
    持续读取 RTK 串口数据，同步 GGA 和 THS 帧后 yield (lat, lon, heading)

    Args:
        ser_rtk_params: dict, 包含 'port' 和 'baudRate'
        sync_threshold: float, GGA 与 THS 时间戳最大允许差值（秒）
        timeout: float, 多久未收到任何有效数据视为超时（秒）
    """
    buffer = {}
    ser = None
    last_valid_time = time.time()

    while True:
        try:
            # --- 1. 确保串口打开 ---
            if ser is None or not ser.is_open:
                ser = serial.Serial(
                    port=ser_rtk_params['port'],
                    baudrate=ser_rtk_params['baudRate'],
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS,
                    rtscts=False,  # 关闭硬件流控（更通用）
                    timeout=0.1  # 读超时，避免永久阻塞
                )
                last_valid_time = time.time()  # 重置超时计时器

            # --- 2. 读取数据 ---
            raw_buffer += ser_rtk.read(ser_rtk.in_waiting)
            # 按帧分割处理
            while b'\n' in raw_buffer:
                line, raw_buffer = raw_buffer.split(b'\n', 1)
                if line:
                    print(line)
                    current_time = time.time()
                    process_line(line.strip(), buffer, current_time)


            # --- 3. 检查是否可同步输出 ---
            if 'gga' in buffer and 'ths' in buffer:
                gga_ts = buffer['gga']['timestamp']
                ths_ts = buffer['ths']['timestamp']
                if abs(gga_ts - ths_ts) < sync_threshold:
                    lat = buffer['gga']['lat']
                    lon = buffer['gga']['lon']
                    heading = buffer['ths']['heading']
                    # 清空已使用的数据
                    del buffer['gga']
                    del buffer['ths']
                    yield lat, lon, heading

            # --- 4. 超时检测 ---
            if current_time - last_valid_time > timeout:
                logger.warning("RTK 超时 {} 秒未收到有效数据，尝试重连...".format(timeout))
                ser.close()
                ser = None
                time.sleep(1)
                continue

            # --- 5. 防止 CPU 空转 ---
            # if not raw_data:
            #     time.sleep(0.01)

        except (serial.SerialException, OSError) as e:
            logger.error("RTK 串口异常: {}".format(e))
            if ser:
                ser.close()
            ser = None
            time.sleep(2)  # 等待设备恢复
            continue  # 不 raise，而是重试

        except Exception as e:
            logger.exception("RTK 数据处理未知错误: {}".format(e))
            if ser:
                ser.close()
            ser = None
            time.sleep(2)
            continue

if __name__ == '__main__':
    ser_rtk_params = {'port': '/dev/ttyACM4', 'baudRate': 115200, 'timeout': 1}
    rtk_generator = util.readRTK(ser_rtk_params)
    # rtk_generator = rtk_thread(ser_rtk_params)
    for lat, lon, heading_deg in rtk_generator:
        print(lat, lon, heading_deg)
        time.sleep(0.01)
    # rtk_thread(ser_rtk_params)