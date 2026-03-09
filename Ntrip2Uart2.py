# coding=utf-8

import serial
import socket
import base64
import time
import threading

import util
from AppLogger import logger
from NMEABuffer import NMEABuffer

# ==================== 配置 ====================
COM = 'COM7'
BPS = 115200
NtripIP = '120.253.239.161'
NtripPort = 8001
NtripPoint = 'RTCM33_GRCE'
NtripUser = 'csha42914'
NtripPwd = 'ac956382'

# ==============================================

def main():
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

    def connect_ntrip():
        # 连接 NTRIP
        try:
            ntrip = socket.socket()
            ntrip.settimeout(10)
            ntrip.connect((NtripIP, NtripPort))

            user_pwd = base64.b64encode(bytes(NtripUser + ':' + NtripPwd)).decode("utf-8")
            httpHead = "GET /" + NtripPoint + " HTTP/1.0\r\nUser-Agent: NTRIP GNSSInternetRadio/1.4.10\r\nAccept: */*\r\nConnection: close\r\nAuthorization: Basic " + user_pwd + "\r\n\r\n"
            ntrip.send(httpHead.encode())
            data = ntrip.recv(1024)
            print(data)

            return ntrip
        except Exception as e:
            print("[ERROR] NTRIP 连接失败: {}".format(e))
            return None

    # 定期发送 GGA（维持会话）
    def sendGGA():
        global RTK, ntrip

        while True:
            # 打开串口
            if RTK is None or not RTK.is_open:
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
            ntripScoket = connect_ntrip()
            # 发送 GGA（用于 VRS）
            ntripScoket.send(strGNGGA.encode())
            print("[INFO] 已发送 GGA 到 NTRIP 服务器")
            # try:
            #     ntrip = socket.socket()
            #     ntrip.settimeout(10)
            #     ntrip.connect((NtripIP, NtripPort))
            #
            #     user_pwd = base64.b64encode(bytes(NtripUser + ':' + NtripPwd)).decode("utf-8")
            #     httpHead = "GET /" + NtripPoint + " HTTP/1.0\r\nUser-Agent: NTRIP GNSSInternetRadio/1.4.10\r\nAccept: */*\r\nConnection: close\r\nAuthorization: Basic " + user_pwd + "\r\n\r\n"
            #     ntrip.send(httpHead.encode())
            #     data = ntrip.recv(1024)
            #     print(data)
            #     # 发送 GGA（用于 VRS）
            #     ntrip.send(strGNGGA.encode())
            #     print("[INFO] 已发送 GGA 到 NTRIP 服务器")
            #
            # except Exception as e:
            #     print("[ERROR] NTRIP 连接失败: {}".format(e))
            #     return

            time.sleep(5)


    # ========== 双向转发：使用两个线程 ==========
    def forward_serial_to_stdout():
        """从 RTK 读取数据（如 NMEA 日志）并打印"""
        nmea_buffer = NMEABuffer()
        sync_threshold = 0.5
        buffer = {}  # 缓存最近的 GGA 和 HPR 数据
        while not stop_event.is_set():
            try:
                current_time = time.time()
                if RTK.in_waiting:
                    raw = RTK.read(RTK.in_waiting)
                    complete_lines = nmea_buffer.feed(raw)
                    for line in complete_lines:
                        try:
                            text = line.decode('ascii', errors='ignore')
                            timestamp = time.time()  # 记录当前时间戳
                            # 现在 text 一定是完整的一行（以 \r\n 结尾）
                            # logger.info(text)
                            if text.startswith("$GNGGA"):
                                gps = util.parse_gngga(text.decode('utf-8', errors='ignore').strip())
                                if gps:
                                    buffer['gga'] = {
                                        'timestamp': timestamp,
                                        'lat': gps[0],
                                        'lon': gps[1]
                                    }
                            elif text.startswith("$GNHPR"):
                                heading, pitch = util.parse_GNHPR(text.decode('utf-8', errors='ignore').strip())
                                buffer['hpr'] = {
                                    'timestamp': timestamp,
                                    'heading': heading,
                                    'pitch': pitch
                                }
                            # 同步 GGA 和 HPR 数据
                            if 'gga' in buffer and 'hpr' in buffer:
                                gga_ts = buffer['gga']['timestamp']
                                hpr_ts = buffer['hpr']['timestamp']

                                if abs(gga_ts - hpr_ts) < sync_threshold:  # 时间差阈值为 0.5 秒
                                    lat = buffer['gga']['lat']
                                    lon = buffer['gga']['lon']
                                    heading = buffer['hpr']['heading']
                                    pitch = buffer['hpr']['pitch']

                                    # 清空缓冲区，准备接收下一次数据
                                    del buffer['gga']
                                    del buffer['hpr']
                                    logger.info("lat={},lon={},heading={},pitch={}".format(lat,lon,heading,pitch))
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
    t3 = threading.Thread(target=sendGGA)
    t1.start()
    t2.start()
    t3.start()

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

if __name__ == "__main__":
    main()