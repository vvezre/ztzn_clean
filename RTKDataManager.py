# coding=utf-8
import base64
import socket
import threading
import time
from datetime import datetime
import datetime

import serial

import util
from AppLogger import logger
from NMEABuffer import NMEABuffer
from VehicleCenterEstimator import VehicleCenterEstimator


# ==================== 数据结构 ====================
class RTKData:
    """
    RTK 定位数据类（Python 2.7 兼容）

    Attributes:
        latitude (float): 纬度，单位：度
        longitude (float): 经度，单位：度
        altitude (float): 高度，单位：米
        speed (float): 速度，单位：米/秒
        fix_type (int): 定位类型：1=无定位, 2=2D, 3=3D, 4=RTK固定解, 5=RTK浮点解
        timestamp (datetime.datetime): 数据时间戳
    """

    def __init__(self):
        self.lat = 0.0  # float
        self.lon = 0.0  # float
        self.heading = None  # float
        self.timestamp = datetime.datetime.now()  # datetime.datetime

    def __str__(self):
        # Python 2.7 不支持 f-string，改用 .format() 或 % 格式化
        return ("Lat: {lat:.8f}, Lon: {lon:.8f}, "
                "heading: {heading:.2f}, Time: {time}").format(
            lat=self.lat,
            lon=self.lon,
            heading=self.heading if self.heading is not None else 0.0,
            time=self.timestamp.strftime('%H:%M:%S.%f')[:-3]
        )

    def __repr__(self):
        return ("RTKData(lat={lat:.8f}, lon={lon:.8f}, heading={heading:.2f}, "
                "time={time})").format(
            lat=self.lat,
            lon=self.lon,
            heading=self.heading if self.heading is not None else 0.0,
            time=repr(self.timestamp)
        )
# ==================== 观察者模式核心类 ====================
# ==================== RTKDataManager 类（Python 2.7 兼容） ====================
class RTKDataManager:
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_port = None      # serial.Serial or None
        self.running = False
        self.observers = []          # 存储回调函数的列表
        self.lock = threading.RLock()  # 线程安全锁
        self.current_data = None     # RTKData or None

    def register_observer(self, callback):
        """
        注册一个观察者（回调函数）
        callback: 函数，接受一个 RTKData 参数
        """
        with self.lock:
            self.observers.append(callback)
            logger.info("Observer registered. Total observers: %d" % len(self.observers))

    def _notify_observers(self, data):
        """
        通知所有观察者（线程安全）
        """
        with self.lock:
            # 复制列表，避免回调中修改原列表
            observers_copy = self.observers[:]
            self.current_data = data

        # 在锁外调用，防止阻塞
        for observer in observers_copy:
            try:
                observer(data)
            except Exception as e:
                logger.error("Observer callback error: %s" % str(e))

    def _open_serial(self):
        """打开串口"""
        try:
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=1)
            logger.info("Serial port %s opened successfully." % self.port)
            return True
        except Exception as e:
            logger.error("Failed to open serial port %s: %s" % (self.port, str(e)))
            return False

    def _close_serial(self):
        """关闭串口"""
        if self.serial_port and self.serial_port.isOpen():
            self.serial_port.close()
            logger.info("Serial port closed.")

    def start(self):
        """启动串口读取线程"""
        if self.running:
            logger.warning("RTKDataManager is already running.")
            return

        if not self._open_serial():
            return

        self.running = True
        thread = threading.Thread(target=self._serial_reader_loop, name="RTKReader")
        # thread = threading.Thread(target=self._serial_reader_loop2, name="RTKReader")
        thread.daemon = True
        thread.start()
        logger.info("RTK data manager started.")

    def stop(self):
        """停止读取"""
        self.running = False
        self._close_serial()
        logger.info("RTK data manager stopped.")

    def _serial_reader_loop(self):
        """串口读取主循环（运行在独立线程）"""
        # 创建估计器：基线 0.2m，中心在后天线前 0.1m
        estimator = VehicleCenterEstimator(baseline_length=0.23)
        ser_rtk_params = {'port': self.port, 'baudRate': self.baudrate, 'timeout': 1}
        rtk_generator = util.readRTK_v2(ser_rtk_params)
        for lat, lon, heading_deg in rtk_generator:
            # logger.info('lat={},lon={},heading={}'.format(lat, lon, heading_deg))
            rtk_data = RTKData()
            # 估算中心
            # result = estimator.estimate_center_from_rear_antenna(lat, lon, heading_deg)
            # c_lat,c_lon = util.compute_center_from_master(lat, lon, heading_deg, 0.17, 0.11)
            if heading_deg is None:
                c_lat, c_lon = lat, lon
            else:
                c_lat,c_lon = util.compute_center_from_master(lat, lon, heading_deg, 0.18, 0.10)
            # c_lat,c_lon = util.calculate_center_offset(lat,lon,heading_deg,0.1,0.2)
            # c_lat, c_lon = util.calculate_center_gps(lat,lon,heading_deg,0.1,0.2)
            # rtk_data.lat = round(result['center_lat'], 8)
            # rtk_data.lon = round(result['center_lon'], 8)
            rtk_data.lat = c_lat
            rtk_data.lon = c_lon
            # rtk_data.lat = lat
            # rtk_data.lon = lon
            rtk_data.heading = heading_deg
            self._notify_observers(rtk_data)

    def _serial_reader_loop2(self):
        # ==================== 配置 ====================
        COM = 'COM7'
        BPS = 115200
        NtripIP = '120.253.239.161'
        NtripPort = 8001
        NtripPoint = 'RTCM33_GRCE'
        NtripUser = 'csha42914'
        NtripPwd = 'ac956382'

        global RTK, ntrip
        stop_event = threading.Event()

        # 打开串口
        RTK = serial.Serial(COM, BPS, timeout=0.1)
        print("等待RTK模块输出有效GNGGA...")

        strGNGGA = None
        while not stop_event.is_set():
            try:
                data = RTK.readline()
                if data:
                    try:
                        strNMEA = data.decode('ascii', errors='ignore')
                    except:
                        continue
                    seg = strNMEA.split(',')
                    if seg[0] == "$GNGGA" and len(seg) > 6 and seg[6] not in ('', '0'):
                        strGNGGA = strNMEA + "\r\n\r\n"
                        print("[INFO] 定位有效，GNGGA: {}".format(strGNGGA.strip()))
                        break
            except Exception as e:
                print("[ERROR] 读取串口异常: {}".format(e))
                time.sleep(1)

        if not strGNGGA:
            print("[ERROR] 未能获取有效GNGGA，退出。")
            return

        # 连接 NTRIP
        try:
            ntrip = socket.socket()
            # ntrip.settimeout(10)
            ntrip.connect((NtripIP, NtripPort))

            user_pwd = base64.b64encode(bytes(NtripUser + ':' + NtripPwd)).decode("utf-8")
            httpHead = "GET /" + NtripPoint + " HTTP/1.0\r\nUser-Agent: NTRIP GNSSInternetRadio/1.4.10\r\nAccept: */*\r\nConnection: close\r\nAuthorization: Basic " + user_pwd + "\r\n\r\n"
            ntrip.send(httpHead.encode())
            data = ntrip.recv(1024)
            print(data)
            # 发送 GGA（用于 VRS）
            ntrip.send(strGNGGA.encode())
            print("[INFO] 已发送 GGA 到 NTRIP 服务器")

        except Exception as e:
            print("[ERROR] NTRIP 连接失败: {}".format(e))
            return

        # ========== 双向转发：使用两个线程 ==========
        def forward_serial_to_stdout():
            """从 RTK 读取数据（如 NMEA 日志）并打印"""
            nmea_buffer = NMEABuffer()
            while not stop_event.is_set():
                try:
                    if RTK.in_waiting:
                        raw = RTK.read(RTK.in_waiting)
                        complete_lines = nmea_buffer.feed(raw)
                        for line in complete_lines:
                            try:
                                text = line.decode('ascii', errors='ignore')
                                # 现在 text 一定是完整的一行（以 \r\n 结尾）
                                if text.startswith('$'):
                                    logger.info(text)
                            except Exception as e:
                                logger.debug("NMEA decode error: %s", e)
                    else:
                        time.sleep(0.01)
                except Exception as e:
                    if not stop_event.is_set():
                        print("[Serial Read Error] {}".format(e))
                    break

        def forward_ntrip_to_serial():
            """从 NTRIP 接收 RTCM 并写入串口"""
            while not stop_event.is_set():
                try:
                    data = ntrip.recv(10240)  # RTCM 是二进制
                    if not data:
                        print("[NTRIP] 连接断开")
                        break
                    RTK.write(data)
                except socket.timeout:
                    continue
                except Exception as e:
                    if not stop_event.is_set():
                        print("[NTRIP Recv Error] {}".format(e))
                    break

        # 启动线程
        t1 = threading.Thread(target=forward_serial_to_stdout)
        t2 = threading.Thread(target=forward_ntrip_to_serial)
        t1.start()
        t2.start()

        print("[RUNNING] NTRIP ↔ RTK 转发中... 按 Ctrl+C 退出")

        try:
            while t1.is_alive() or t2.is_alive():
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\n[INFO] 正在退出...")
        finally:
            stop_event.set()
            try:
                ntrip.close()
                RTK.close()
            except:
                pass
# ==================== 使用示例 ====================
def observer_a(data):
    """观察者 A：比如用于地图显示"""
    logger.info("[Observer A] Map Update: {}".format(data))


def observer_b(data):
    """观察者 B：比如用于导航算法"""
    logger.info("[Observer B] ✅ RTK Fixed! Navigating... Position: {:.8f}, {:.8f}".format(data.lat, data.lon))



# ==================== 主程序 ====================
def main():
    # 创建 RTK 数据管理器（假设串口是 /dev/ttyUSB0 或 COM3）
    rtk_manager = RTKDataManager(port="/dev/ttyUSB0", baudrate=115200)  # Linux
    # rtk_manager = RTKDataManager(port="COM3", baudrate=115200)      # Windows

    # 注册多个观察者
    rtk_manager.register_observer(observer_a)
    rtk_manager.register_observer(observer_b)

    # 启动
    rtk_manager.start()

    try:
        # 主线程可以做其他事，比如监控
        while True:
            time.sleep(5)
            # 可选：获取当前最新数据（线程安全）
            latest_data = rtk_manager.current_data
            if latest_data:
                logger.info("[Main] Latest data: {}".format(latest_data))
    except KeyboardInterrupt:
        logger.info("Shutting down...")
        rtk_manager.stop()


if __name__ == "__main__":
    main()
