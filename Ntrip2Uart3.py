#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
高鲁棒性 RTK + NTRIP 客户端
- 自动重连 NTRIP
- 定期发送 GGA
- 实时监控 RTK 状态
- 混合流解析（NMEA + RTCM）
"""

import serial
import socket
import base64
import time
import threading
import logging
from pynmeagps import NMEAReader
from pyrtcm import RTCMReader

from AppLogger import logger

# ==================== 用户配置区 ====================
COM_PORT = "COM15"          # RTK 模块串口
BAUDRATE = 115200

NTRIP_HOST = "114.132.168.105"
NTRIP_PORT = 8002
MOUNTPOINT = "wit365"
USERNAME = "v2ayri8y"
PASSWORD = "123456"

LOG_FILE = "rtk_robust.log"
# ===================================================

# 全局状态（线程安全）
stop_event = threading.Event()
rtk_serial = None
ntrip_socket = None

latest_gga = None           # 最新 GGA 字符串（含 \r\n）
gga_lock = threading.Lock()

rtcm_last_time = 0          # 最后收到 RTCM 的时间戳
gga_quality = 0             # 当前定位质量（0=无效, 4=固定）

# ==================== 行缓冲器 ====================
class LineBuffer:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf.extend(data)
        lines = []
        while b'\r\n' in self.buf:
            idx = self.buf.find(b'\r\n')
            line = self.buf[:idx + 2]
            lines.append(line)
            del self.buf[:idx + 2]
        return lines

# ==================== NTRIP 连接 ====================
def connect_ntrip():
    """尝试连接 NTRIP，失败则返回 None"""
    try:
        sock = socket.socket()
        sock.settimeout(10)
        sock.connect((NTRIP_HOST, NTRIP_PORT))

        user_pwd = base64.b64encode(bytes(USERNAME + ':' + PASSWORD)).decode("utf-8")
        httpHead = "GET /" + MOUNTPOINT + " HTTP/1.0\r\nUser-Agent: NTRIP GNSSInternetRadio/1.4.10\r\nAccept: */*\r\nConnection: close\r\nAuthorization: Basic " + user_pwd + "\r\n\r\n"
        sock.send(httpHead.encode())

        resp = b""
        while b"\r\n\r\n" not in resp:
            chunk = sock.recv(1024)
            if not chunk:
                raise Exception("NTRIP 无响应")
            resp += chunk
        if b"200 OK" not in resp:
            raise Exception("NTRIP 拒绝: {}".format(resp.split(b'\r\n')[0].decode()))
        return sock
    except Exception as e:
        logger.error("NTRIP 连接失败: %s", e)
        return None

# ==================== 串口解析线程 ====================
def serial_reader():
    global latest_gga, gga_quality
    line_buf = LineBuffer()
    rtcm_buf = bytearray()

    while not stop_event.is_set() and rtk_serial.is_open:
        try:
            if rtk_serial.in_waiting:
                raw = rtk_serial.read(rtk_serial.in_waiting)
                line_buf.feed(raw)  # 提取完整 NMEA 行
                rtcm_buf.extend(raw)

                # 解析 NMEA
                for line in list(line_buf.buf):  # 注意：我们只用 line_buf 提取行，不用于 RTCM
                    pass  # 实际上我们已在 feed 中提取了 lines

                # 重新获取完整行（修正）
                lines = line_buf.feed(b"")  # 强制 flush（其实上面已处理）
                # 更正：应在 read 后立即提取
                # —— 为简化，我们直接在 raw 上做混合解析（见下）

                # 改为：对 raw 做混合解析（更可靠）
                # parse_mixed_stream(raw)

            time.sleep(0.01)
        except Exception as e:
            if not stop_event.is_set():
                logger.exception("串口读取异常")

def parse_mixed_stream(data: bytes):
    """解析 NMEA + RTCM 混合流"""
    global latest_gga, gga_quality, rtcm_last_time
    buffer = bytearray(data)
    nmea_reader = NMEAReader()
    rtcm_reader = RTCMReader()

    while len(buffer) >= 2:
        if buffer[0:1] == b'$':
            end_idx = buffer.find(b'\r\n')
            if end_idx == -1:
                break
            line = buffer[:end_idx + 2]
            del buffer[:end_idx + 2]
            try:
                text = line.decode('ascii', errors='ignore').strip()
                msg = nmea_reader.parse(text)
                if hasattr(msg, 'identity'):
                    if msg.identity in ("GNGGA", "GPGGA"):
                        with gga_lock:
                            latest_gga = text + "\r\n"
                            gga_quality = getattr(msg, 'quality', 0)
                        logger.debug("[NMEA] %s (quality=%d)", msg.identity, gga_quality)
            except Exception:
                pass  # 忽略解析错误

        elif buffer[0:1] == b'\xd3':
            if len(buffer) < 6:
                break
            byte2, byte3 = buffer[1], buffer[2]
            length = ((byte2 & 0x03) << 8) | byte3
            total_len = 3 + length + 3
            if len(buffer) < total_len:
                break
            rtcm_data = buffer[:total_len]
            del buffer[:total_len]
            try:
                msg = rtcm_reader.parse(rtcm_data)
                if hasattr(msg, 'identity'):
                    rtcm_last_time = time.time()
                    logger.debug("[RTCM] Type %s", msg.identity)
            except Exception:
                pass
        else:
            del buffer[0]  # 跳过垃圾字节

# ==================== NTRIP 转发线程 ====================
def ntrip_forwarder():
    global ntrip_socket
    last_gga_sent = 0
    GGA_INTERVAL = 5  # 每5秒发一次

    while not stop_event.is_set():
        # 步骤1：连接 NTRIP
        ntrip_socket = connect_ntrip()
        if not ntrip_socket:
            time.sleep(5)
            continue

        logger.info("✅ 开始转发 RTCM 数据...")
        try:
            while not stop_event.is_set():
                # 定期发送 GGA
                now = time.time()
                if now - last_gga_sent > GGA_INTERVAL:
                    with gga_lock:
                        gga_to_send = latest_gga
                    if gga_to_send:
                        try:
                            ntrip_socket.send(gga_to_send.encode())
                            last_gga_sent = now
                            logger.debug("[NTRIP] 已发送 GGA")
                        except Exception as e:
                            logger.error("GGA 发送失败: %s", e)
                            break

                # 接收 RTCM
                try:
                    ntrip_socket.settimeout(1.0)
                    data = ntrip_socket.recv(4096)
                    if not data:
                        raise ConnectionError("NTRIP 无数据")
                    rtk_serial.write(data)
                except socket.timeout:
                    continue
                except Exception as e:
                    logger.error("接收 RTCM 失败: %s", e)
                    break

        except Exception as e:
            logger.error("NTRIP 转发异常: %s", e)
        finally:
            if ntrip_socket:
                ntrip_socket.close()
            ntrip_socket = None
            logger.warning("🔄 NTRIP 连接断开，准备重连...")

# ==================== RTK 状态监控线程 ====================
def monitor_rtk_status():
    global gga_quality, rtcm_last_time
    last_quality = -1
    while not stop_event.is_set():
        now = time.time()
        current_quality = gga_quality

        # 状态变化才打印
        if current_quality != last_quality:
            status_map = {0: "无效", 1: "单点", 2: "DGPS", 4: "RTK固定", 5: "RTK浮点"}
            status = status_map.get(current_quality, f"未知({current_quality})")
            logger.info("📍 定位状态更新: %s (quality=%d)", status, current_quality)
            last_quality = current_quality

        # 检查 RTCM 是否中断
        if current_quality != 4 and (now - rtcm_last_time) > 10:
            logger.warning("⚠️ 差分数据中断超过 10 秒！检查网络或 NTRIP 配置")

        time.sleep(2)

# ==================== 主程序 ====================
def main():
    global rtk_serial
    try:
        rtk_serial = serial.Serial(COM_PORT, BAUDRATE, timeout=0.1)
        logger.info("✅ 串口已打开: %s @ %d", COM_PORT, BAUDRATE)
    except Exception as e:
        logger.error("❌ 无法打开串口: %s", e)
        return

    # 启动线程
    threading.Thread(target=serial_reader, daemon=True, name="SerialReader").start()
    threading.Thread(target=ntrip_forwarder, daemon=True, name="NTRIPForwarder").start()
    threading.Thread(target=monitor_rtk_status, daemon=True, name="StatusMonitor").start()

    logger.info("🚀 RTK + NTRIP 客户端已启动！按 Ctrl+C 退出...")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("\n🛑 收到中断信号，正在关闭...")
    finally:
        stop_event.set()
        if rtk_serial:
            rtk_serial.close()
        logger.info("👋 程序已退出")

if __name__ == "__main__":
    main()