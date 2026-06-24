# coding=utf-8
import binascii
import copy
import json
import threading
import time
from collections import defaultdict

import cv2
import serial
import serial.tools.list_ports

from geographiclib.geodesic import Geodesic

import util
from AppLogger import logger
from ntrip_runtime import extract_gga_quality, get_shared_runtime

ser_rtk = None
rtk_output_configured = False
RTK_OUTPUT_CONFIG_COMMANDS = (
    'unlog\r\n',
    'gngga 0.1\r\n',
    'gphpr 0.1\r\n',
    'saveconfig\r\n',
)
RTK_NMEA_MAX_AGE_SECONDS = 2.0
RTK_DRAIN_MAX_LINES = 300
RTK_NTRIP_STEP_INTERVAL_SECONDS = 0.02


def _flush_serial_input(ser, reason='', port=''):
    try:
        if hasattr(ser, 'reset_input_buffer'):
            ser.reset_input_buffer()
        else:
            ser.flushInput()
        if reason:
            logger.warning("RTK_DIAG serial_input_flushed port={} reason={}".format(port, reason))
        return True
    except Exception as e:
        logger.warning("RTK_DIAG serial_input_flush_failed port={} reason={} error={}".format(port, reason, e))
        return False


def _parse_nmea_utc_seconds(utc_text):
    if not utc_text:
        return None
    try:
        value = float(utc_text)
        hour = int(value / 10000)
        minute = int((value - hour * 10000) / 100)
        second = value - hour * 10000 - minute * 100
        if hour < 0 or hour > 23 or minute < 0 or minute > 59:
            return None
        if second < 0 or second >= 61:
            return None
        return hour * 3600 + minute * 60 + second
    except Exception:
        return None


def _nmea_utc_age_seconds(utc_text, now_time=None):
    nmea_seconds = _parse_nmea_utc_seconds(utc_text)
    if nmea_seconds is None:
        return None
    if now_time is None:
        now_time = time.time()
    now_utc = time.gmtime(now_time)
    now_seconds = now_utc.tm_hour * 3600 + now_utc.tm_min * 60 + now_utc.tm_sec
    age = now_seconds - nmea_seconds
    if age > 43200:
        age -= 86400
    elif age < -43200:
        age += 86400
    return age


def _is_stale_nmea_utc(utc_text, now_time=None, max_age_seconds=RTK_NMEA_MAX_AGE_SECONDS):
    age = _nmea_utc_age_seconds(utc_text, now_time)
    return age is not None and age > max_age_seconds


def _rtk_line_to_text(line):
    if line is None:
        return ''
    if isinstance(line, bytes):
        try:
            return line.decode('utf-8', errors='ignore').strip()
        except TypeError:
            return line.decode('utf-8', 'ignore').strip()
    return str(line).strip()


def _rtk_nmea_utc(line_str):
    parts = line_str.split(',')
    if len(parts) > 1:
        return parts[1]
    return ''


def _serial_in_waiting(ser):
    try:
        return int(getattr(ser, 'in_waiting'))
    except Exception:
        pass
    try:
        return int(ser.inWaiting())
    except Exception:
        return 0


def _serial_is_open(ser):
    if ser is None:
        return False
    try:
        return bool(getattr(ser, 'is_open'))
    except Exception:
        pass
    try:
        return bool(ser.isOpen())
    except Exception:
        return False


class _RtkLatestState(object):
    """Keep only the latest usable GGA/HPR pair; old backlog is intentionally skipped."""

    def __init__(self, sync_threshold=0.5):
        self.sync_threshold = sync_threshold
        self.buffer = {}
        self.last_heading = None
        self.last_quality = None


def _consume_rtk_nmea_lines(state, lines, now_time=None, port='', correction_runtime=None,
                            max_age_seconds=RTK_NMEA_MAX_AGE_SECONDS, log_events=False):
    stats = {
        'lines': 0,
        'stale': 0,
        'samples': 0,
        'first_stale_utc': None,
        'first_stale_age': None,
    }
    latest_sample = None

    for raw_line in lines:
        line_str = _rtk_line_to_text(raw_line)
        if not line_str:
            continue
        stats['lines'] += 1
        timestamp = time.time() if now_time is None else now_time

        if line_str.startswith(("$GNGGA", "$GPGGA", "$GNHPR", "$GPHPR")):
            nmea_utc = _rtk_nmea_utc(line_str)
            nmea_age = _nmea_utc_age_seconds(nmea_utc, timestamp)
            if nmea_age is not None and nmea_age > max_age_seconds:
                stats['stale'] += 1
                if stats['first_stale_utc'] is None:
                    stats['first_stale_utc'] = nmea_utc
                    stats['first_stale_age'] = nmea_age
                state.buffer.clear()
                continue

        if line_str.startswith(("$GNGGA", "$GPGGA")):
            if correction_runtime is not None:
                correction_runtime.observe_gga(line_str)
            quality = extract_gga_quality(line_str)
            if quality != state.last_quality:
                if log_events:
                    logger.warning("RTK GGA瀹氫綅鐘舵€佹洿鏂? fix={}".format(quality))
                state.last_quality = quality

            gps = parse_gngga(line_str)
            if not gps:
                continue

            state.buffer['gga'] = {
                'timestamp': timestamp,
                'lat': gps[0],
                'lon': gps[1]
            }
            if 'ths' in state.buffer:
                gga_ts = state.buffer['gga']['timestamp']
                ths_ts = state.buffer['ths']['timestamp']
                if abs(gga_ts - ths_ts) < state.sync_threshold:
                    heading_deg = state.buffer['ths']['heading']
                    state.last_heading = heading_deg
                    latest_sample = (state.buffer['gga']['lat'], state.buffer['gga']['lon'], heading_deg)
                    del state.buffer['gga']
                    del state.buffer['ths']
                else:
                    heading_deg = state.last_heading if state.last_heading is not None else state.buffer['ths']['heading']
                    latest_sample = (state.buffer['gga']['lat'], state.buffer['gga']['lon'], heading_deg)
                    del state.buffer['gga']
            else:
                latest_sample = (state.buffer['gga']['lat'], state.buffer['gga']['lon'], state.last_heading)
                del state.buffer['gga']

            stats['samples'] += 1

        elif line_str.startswith(("$GNTHS", "$GPTHS")):
            heading = parse_GPTHS(line_str)
            if heading is not None:
                state.last_heading = heading
                state.buffer['ths'] = {
                    'timestamp': timestamp,
                    'heading': heading
                }

        elif line_str.startswith(("$GNHPR", "$GPHPR")):
            heading_info = parse_GNHPR(line_str)
            if heading_info is not None:
                state.last_heading = heading_info[0]
                state.buffer['ths'] = {
                    'timestamp': timestamp,
                    'heading': heading_info[0]
                }

    return latest_sample, stats


def _read_serial_lines_now(ser, max_lines=RTK_DRAIN_MAX_LINES):
    lines = []
    first_line = ser.readline()
    if first_line:
        lines.append(first_line)
    while len(lines) < max_lines and _serial_in_waiting(ser) > 0:
        line = ser.readline()
        if not line:
            break
        lines.append(line)
    return lines


def _start_ntrip_serial_worker(correction_runtime, serial_port,
                               interval_seconds=RTK_NTRIP_STEP_INTERVAL_SECONDS):
    stop_event = threading.Event()

    def _worker():
        while not stop_event.is_set() and _serial_is_open(serial_port):
            try:
                correction_runtime.step(serial_port)
            except Exception as exc:
                logger.error("NTRIP correction worker error: {}".format(exc))
            stop_event.wait(interval_seconds)

    thread = threading.Thread(target=_worker, name="NtripRTCMWriter")
    thread.daemon = True
    thread.start()
    return stop_event, thread


def configure_rtk_output(ser):
    global rtk_output_configured
    if rtk_output_configured:
        return True
    if ser is None:
        return False

    try:
        for command_text in RTK_OUTPUT_CONFIG_COMMANDS:
            ser.write(command_text.encode('ascii'))
            try:
                ser.flush()
            except Exception:
                pass
            logger.warning("RTK config command sent: {}".format(command_text.strip()))
            time.sleep(0.15)

        _flush_serial_input(ser, 'after_config')

        rtk_output_configured = True
        logger.warning("RTK output configured: GNGGA/GPHPR interval=0.1s")
        return True
    except Exception as e:
        logger.error("RTK output config failed: {}".format(e))
        return False

def readConfig(fileName):
    # 打开并读取 JSON 文件
    with open(fileName, 'r') as f:
        data = json.load(f)  # 将 JSON 文件内容解析为 Python 字典或列表
        return data

# 通过遍历列表，以对象属性为键，将对象分组到字典中
def group_by_attr(objects, attr_name):
    grouped = defaultdict(list)
    for obj in objects:
        key = getattr(obj, attr_name)
        grouped[key].append(obj)
    return dict(grouped)

# direction 从光伏板的左侧还是右侧启动，默认左侧
# panels    一个区域光伏板的排列信息
# goBackLen 向上直行，然后后退距离
# goLeftOrRightBackLen 左右直行，然后后退距离
# turnBackLen 转弯后，后退距离
# panelWidth 光伏板宽度
# panelHeight 光伏板高度
# upOrDownBridgeLen 上下过桥板长度
# leftOrRightBridgeLen 左右过桥板长度
# gap 光伏板之前间隙
# lineCount 每个区域的行数
# columnCount 每个区域的列数
def createTask(direction='left', panels=None, areaNumber=1,goBackLen=5, goLeftOrRightBackLen=15, turnBackLen=10,
               panelWidth=400, panelHeight=100,
               upOrDownBridgeLen=50, leftOrRightBridgeLen=150,
               gap=3,lineCount=0,columnCount=0,angle_radians=0.06):
    if panels is None:
        panels = []
    crossCount = panels.count(0)

    if areaNumber == 2:
        height = lineCount * panelHeight + (lineCount - 1) * gap + crossCount * upOrDownBridgeLen - panelHeight * 0.5
    else:
        height = lineCount * panelHeight + (lineCount - 1) * gap + crossCount * upOrDownBridgeLen - panelHeight * 0.4
    width = columnCount * panelWidth + (columnCount-1)*gap - panelWidth
    H = panelHeight*0.5
    height = int(height * math.cos(angle_radians))
    H = int(H*math.cos(angle_radians))
    list = [
        {"angle": 90, "mode": 1, "length": width, "turn_back_len": turnBackLen, "back_len": goLeftOrRightBackLen},
        {"angle": 180, "mode": 2, "length": H, "turn_back_len": turnBackLen, "back_len": 0},
        {"angle": 270, "mode": 1, "length": width, "turn_back_len": turnBackLen, "back_len": goLeftOrRightBackLen},
    ]
    # 如果从右侧出发，那就先向270方向转
    if direction == 'right':
        list = list[::-1]
    # 第一步直行任务
    goTask = [{"angle":0,"mode":1,"length":height,"turn_back_len":turnBackLen,"back_len":goBackLen}]
    # 向下跨桥任务
    crossTheBridge = {"angle": 180, "mode": 2, "length": upOrDownBridgeLen+H, "turn_back_len": turnBackLen, "back_len": 0}
    # 非向下跨桥任务
    notCrossTheBridge = {"angle": 180, "mode": 2, "length": H, "turn_back_len": turnBackLen, "back_len": 0}
    # 走固定距离，两个区域跨桥任务
    endTheBridge = {"angle": 90, "mode": 2, "length": width+leftOrRightBridgeLen, "turn_back_len": turnBackLen, "back_len": 0}

    # panels = [1,0,2,3,0,4,5,6]
    reversed_panels = panels[::-1]

    for index,item in enumerate(reversed_panels):
        if item == 0:
            continue
        if index == len(reversed_panels)-1:
            # 表示执行到最后一行了
            tmpList = list + [endTheBridge]
        else:
            next_item = reversed_panels[index+1]
            # 0表示有过桥板
            if next_item == 0:
                tmpList = list + [crossTheBridge]
            else:
                tmpList = list + [notCrossTheBridge]
        goTask.extend(tmpList)

    resultTaskList = deep_copy_list(goTask)
    del goTask
    startX = 0
    startY = 0
    # 给所有的任务加个区域，和任务号
    for index, item in enumerate(resultTaskList):
        item["id"] = index + 1
        item["areaNumber"] = areaNumber
        item['startX'] = startX
        item['startY'] = startY
        item['heading'] = (item['angle'] - 10)%360
        if item['angle'] == 0:
            item['endX'] = 0
            item['endY'] = height
        elif item['angle'] == 90:
            # 获取上一个任务
            pre_item = resultTaskList[index - 1]
            if pre_item['angle'] == 0:
                item['endX'] = width
                item['endY'] = height
            elif pre_item['angle'] == 180:
                item['endX'] = width
                item['endY'] = pre_item['endY']
            elif pre_item['angle'] == 270:
                item['endX'] = item['length']
                item['endY'] = pre_item['endY']
        elif item['angle'] == 180:
            # 获取上一个任务
            pre_item = resultTaskList[index - 1]
            if pre_item['angle'] == 90:
                item['endX'] = width
                item['endY'] = pre_item['endY'] - item['length']
            else:
                item['endX'] = 0
                item['endY'] = pre_item['endY'] - item['length'] - gap
        elif item['angle'] == 270:
            # 获取上一个任务
            pre_item = resultTaskList[index - 1]
            item['endX'] = 0
            item['endY'] = pre_item['endY']

        startX = item['endX']
        startY = item['endY']

    return resultTaskList
# 根据光伏板信息来生成任务列表
def createTaskByPanelInfo(panelInfoList,areaNumber=1,startX=0,startY=0,direction='left', isLastArea=True,lineCount=0,goBackLen=5, goLeftOrRightBackLen=15,
                      turnBackLen=10,panelWidth=113, panelHeight=226,leftOrRightBridgeLen=150,
                      gap=3,angle_radians=0.06,angle_to='y',gapX=None,gapY=None,
                      angle_radians_x=None,angle_radians_y=None):
    # logger.warn(panelInfoList)
    # 计算y轴上特定间隙总长
    if gapX is None:
        gapX = gap
    if gapY is None:
        gapY = gap
    gapX = int(gapX)
    gapY = int(gapY)
    if angle_radians_x is None:
        angle_radians_x = angle_radians if angle_to != 'y' else 0
    if angle_radians_y is None:
        angle_radians_y = angle_radians if angle_to == 'y' else 0
    x_projection = math.cos(angle_radians_x)
    y_projection = math.cos(angle_radians_y)
    projected_panel_width = int(panelWidth * x_projection)
    projected_left_or_right_bridge_len = int(leftOrRightBridgeLen * x_projection)
    projected_gap_y = int(gapY * y_projection)
    totalGapLen = 0
    for rowPanelInfo in panelInfoList:
        # 有间隙则累加
        if rowPanelInfo["isGap"]:
            totalGapLen = totalGapLen + int(rowPanelInfo['gapLen'])
    height = int((len(panelInfoList) * panelHeight + (lineCount - 1) * gapY + totalGapLen - panelHeight * 0.5) * y_projection)
    H = int(panelHeight * 0.5 * y_projection)
    # 如果y轴上有角度，height就需要cos,width就不需要处理
    # 第一步直行任务
    goTask = [{"angle": 0, "mode": 1, "length": height, "turn_back_len": turnBackLen, "back_len": goBackLen}]
    # 向下任务
    downTask = {"angle": 180, "mode": 2, "length": H, "turn_back_len": turnBackLen, "back_len": 0}

    # 计算每一行x轴长度
    for index,rowPanelInfo in enumerate(panelInfoList):
        width = rowPanelInfo['column'] * panelWidth + (rowPanelInfo['column'] - 1) * gapX - panelWidth
        # 如果是x轴上有角度，不是y轴上有角度，width就需要cos
        width = int(width * x_projection)
        list = [
            {"angle": 90, "mode": 1, "length": width, "turn_back_len": turnBackLen, "back_len": goLeftOrRightBackLen},
            {"angle": 180, "mode": 2, "length": H, "turn_back_len": turnBackLen, "back_len": 0},
            {"angle": 270, "mode": 1, "length": width, "turn_back_len": turnBackLen, "back_len": goLeftOrRightBackLen},
        ]
        # 如果从右侧出发，那就先向270方向转
        if direction == 'right':
            list = list[::-1]
        goTask = goTask + list
        # 判断是否是最后一行，如果不是最后一行，则需要加向下任务
        if index == len(panelInfoList)-1:
            if direction == 'right':
                angle = 270
            else:
                angle = 90
            # 是否是最后一个区域，如果不是则要加左右垮桥板子的长度
            if not isLastArea:
                # 去掉最后一个任务，将下面的任务替换为最后一个任务
                goTask.pop()
                # 区域中最后一个任务
                areaEndTask = {"angle": angle, "mode": 2, "length": projected_panel_width + projected_left_or_right_bridge_len,
                                "turn_back_len": turnBackLen,
                                "back_len": 0}
                goTask = goTask + [areaEndTask]
            else:
                # 去掉最后一个任务，将下面的任务替换为最后一个任务
                goTask.pop()
                areaEndTask = {"angle": 270, "mode": 1, "length": startX + width,
                                "turn_back_len": turnBackLen,
                                "back_len": 0}
                goTask = goTask + [areaEndTask]

        else:
            nextRowPanelInfo = panelInfoList[index + 1]
            downTask = {"angle": 180, "mode": 2, "length": H, "turn_back_len": turnBackLen, "back_len": 0}
            if nextRowPanelInfo["isGap"]:
                gapLen = int(int(nextRowPanelInfo["gapLen"]) * y_projection)
                downTask = {"angle": 180, "mode": 2, "length": H + gapLen, "turn_back_len": turnBackLen, "back_len": 0}
            goTask = goTask + [downTask]
    # logger.warn(goTask)
    resultTaskList = deep_copy_list(goTask)
    del goTask
    # 给所有的任务加个区域，和任务号
    if direction == 'left':
        for index, item in enumerate(resultTaskList):
            item["id"] = index + 1
            item["areaNumber"] = areaNumber
            item['startX'] = startX
            item['startY'] = startY
            # item['heading'] = (item['angle'] - 10) % 360
            if item['angle'] == 0:
                item['endX'] = startX
                item['endY'] = startY + height
            elif item['angle'] == 90:
                # 获取上一个任务
                pre_item = resultTaskList[index - 1]
                if pre_item['angle'] == 0:
                    item['endX'] = startX + item['length']
                    item['endY'] = pre_item['endY']
                elif pre_item['angle'] == 180:
                    # 该区域不是最后一个区域，并且该任务是最后一个任务
                    if not isLastArea and index == len(resultTaskList)-1:
                        item['endX'] = item['startX'] + item['length']
                        item['endY'] = pre_item['endY']
                    else:
                        item['endX'] = startX + item['length']
                        item['endY'] = pre_item['endY']
                elif pre_item['angle'] == 270:
                    item['endX'] = item['length']
                    item['endY'] = pre_item['endY']
            elif item['angle'] == 180:
                # 获取上一个任务
                pre_item = resultTaskList[index - 1]
                if pre_item['angle'] == 90:
                    item['endX'] = pre_item['endX']
                    item['endY'] = pre_item['endY'] - item['length']
                else:
                    item['endX'] = item['startX']
                    item['endY'] = pre_item['endY'] - item['length'] - projected_gap_y
            elif item['angle'] == 270:
                # 获取上一个任务
                pre_item = resultTaskList[index - 1]
                item['endX'] = item['startX'] - item['length']
                item['endY'] = pre_item['endY']
            startX = item['endX']
            startY = item['endY']
    elif direction == 'right':
        for index, item in enumerate(resultTaskList):
            item["id"] = index + 1
            item["areaNumber"] = areaNumber
            item['startX'] = startX
            item['startY'] = startY
            if item['angle'] == 0:
                item['endX'] = startX
                item['endY'] = startY + height
            elif item['angle'] == 90:
                # 获取上一个任务
                pre_item = resultTaskList[index - 1]
                item['endX'] = item['startX'] + item['length']
                item['endY'] = pre_item['endY']
            elif item['angle'] == 180:
                # 获取上一个任务
                pre_item = resultTaskList[index - 1]
                if pre_item['angle'] == 90:
                    item['endX'] = pre_item['endX']
                    item['endY'] = pre_item['endY'] - item['length']
                else:
                    item['endX'] = item['startX']
                    item['endY'] = pre_item['endY'] - item['length'] - projected_gap_y
            elif item['angle'] == 270:
                # 获取上一个任务
                pre_item = resultTaskList[index - 1]
                if pre_item['angle'] == 0:
                    item['endX'] = startX - item['length']
                    item['endY'] = pre_item['endY']
                elif pre_item['angle'] == 180:
                    # 该区域不是最后一个区域，并且该任务是最后一个任务
                    if not isLastArea and index == len(resultTaskList)-1:
                        item['endX'] = item['startX'] - item['length']
                        item['endY'] = pre_item['endY']
                    else:
                        item['endX'] = startX - item['length']
                        item['endY'] = pre_item['endY']
            startX = item['endX']
            startY = item['endY']
    # logger.warn(resultTaskList)
    return resultTaskList

# 将task添加x,y轴坐标
def convertXY(taskList,areaNumber=1,direction='left'):
    startX = 0
    startY = 0
    # 给所有的任务加个区域，和任务号
    for index, item in enumerate(taskList):
        item["id"] = index + 1
        item["areaNumber"] = areaNumber
        item['startX'] = startX
        item['startY'] = startY
        length = item['length']
        # item['heading'] = (item['angle'] - 10) % 360
        if item['angle'] == 0:
            item['endX'] = 0
            item['endY'] = length
        elif item['angle'] == 90:
            # 获取上一个任务
            pre_item = taskList[index - 1]
            pre_item_endX = pre_item['endX']
            pre_item_endY = pre_item['endY']
            if pre_item['angle'] == 0:
                item['endX'] = length
                item['endY'] = pre_item_endY
            elif pre_item['angle'] == 180:
                item['endX'] = pre_item_endX
                item['endY'] = pre_item['endY']-length
            elif pre_item['angle'] == 270:
                item['endX'] = pre_item_endX - item['length']
                item['endY'] = pre_item_endY
        elif item['angle'] == 180:
            # 获取上一个任务
            pre_item = taskList[index - 1]
            if pre_item['angle'] == 90:
                item['endX'] = 1
                item['endY'] = pre_item['endY'] - item['length']
            else:
                item['endX'] = 0
                item['endY'] = pre_item['endY'] - item['length'] - gap
        elif item['angle'] == 270:
            # 获取上一个任务
            pre_item = taskList[index - 1]
            item['endX'] = 0
            item['endY'] = pre_item['endY']

        startX = item['endX']
        startY = item['endY']

# 监听电池电量
def listenerVoltage(ser):
    if ser.is_open:
        # 发送16进制数据
        hex_data = "01 04 00 00 00 01 31 CA"  # 16进制字符串（支持空格分隔）
        command = hex_data.replace(' ', '').decode('hex')
        ser.write(command)  # 发送数据

        # 等待并接收响应
        time.sleep(0.1)
        data = ser.read(7)
        # 返回的数据：01 04 02 00 3D 78 E1
        # 第5位是电池电量
        return int(binascii.b2a_hex(data[4]), 16)
    else:
        logger.error('连接电池模块的串口打开失败')

# 深度拷贝
def deep_copy_list(lst):
    return [copy.deepcopy(item) for item in lst]

#分析gngga数据
def parse_gga_info(line):
    if not line.startswith(("$GNGGA", "$GPGGA")):
        return None
    parts = line.strip().split(',')
    if len(parts) < 7:
        return None
    if not parts[2] or not parts[3] or not parts[4] or not parts[5]:
        return None

    try:
        lat_raw = float(parts[2])
        lon_raw = float(parts[4])
        lat = int(lat_raw / 100) + (lat_raw % 100) / 60
        lon = int(lon_raw / 100) + (lon_raw % 100) / 60
        if parts[3] == 'S':
            lat = -lat
        if parts[5] == 'W':
            lon = -lon
        return {
            'lat': round(lat, 8),
            'lon': round(lon, 8),
            'quality': parts[6].strip(),
            'sentence': line.strip()
        }
    except:
        return None


def parse_gngga(line):
    info = parse_gga_info(line)
    if not info:
        return None
    if info.get('quality') != '4':
        return None
    return (info['lat'], info['lon'])

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
    if not line.startswith(("$GNTHS", "$GPTHS")):
        return None
    parts = line.strip().split(',')
    if len(parts) < 3:
        return None
    try:
        if parts[2] and parts[2] != 'A':
            return None
        heading = float(parts[1])  # 第1个字段是角度
        return heading
    except:
        return None

# 分析GNHPR数据,返回航向角和俯仰角
def parse_GNHPR(line):
    if not line.startswith(("$GNHPR", "$GPHPR")):
        return None
    try:
        parts = line.strip().split(',')
        heading = float(parts[2])
        pitch = float(parts[3])
        return heading, pitch
    except:
        return None
# 监听RTK,获取实时位置
# lon经度 lat纬度
def listenerRTK(ser,redis_cli):
    if ser.is_open:
        line = ser.readline().decode(errors='ignore').strip()
        logger.warn(line)
        if not line:
            return None
        if line.startswith("$GNGGA"):
            gps = parse_gngga(line)
            if gps:
                logger.warn("[GNGGA] Lat: {}°, Lon: {}°".format(gps[0],gps[1]))
                redis_cli.hset('currentLocation', 'lat', gps[0])
                redis_cli.hset('currentLocation', 'lon', gps[1])

        elif line.startswith("#UNIHEADINGA"):
            heading = parse_uniheadinga(line)
            if heading is not None:
                logger.warn("[UNIHEADINGA] Heading: {}°".format(heading))

        elif line.startswith("$GNTHS"):
            heading = parse_GPTHS(line)
            if heading is not None:
                # logger.warn("[GPTHS] Heading: {}°".format(heading))
                redis_cli.hset('currentLocation', 'heading', heading)

    else:
        logger.error("RTK串口打开失败！")

def readOpencv(cap):
    while True:
        ret, image = cap.read()
        if not ret:
            logger.error('无法读取视频流或文件结束')
            return
        else:
            height, width = image.shape[:2]
            logger.info('图片高度: %d, 宽度: %d', height, width)
            image = image[100:380, 125:515]
            height, width = image.shape[:2]
            center_x = width // 2
            center_y = height // 2
            logger.info('一帧x坐标: %d', center_x)
            logger.info('一帧y坐标: %d', center_y)
            break
    cap.release()
    time.sleep(1)

    while True:
        try:
            image = image[100:380, 125:515]
            image = cv2.blur(image, (5, 5))
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            lsd = cv2.createLineSegmentDetector(0)
            dlines = lsd.detect(gray)

            line_angle = 0  # 夹角
            if not (dlines[0] is None):
                for dline in dlines[0]:
                    x0 = int(round(dline[0][0]))
                    y0 = int(round(dline[0][1]))
                    x1 = int(round(dline[0][2]))
                    y1 = int(round(dline[0][3]))

                    # 计算角度
                    if y1 - y0 > 90:
                        # 计算线段与垂直轴的夹角（弧度）
                        dx = x1 - x0
                        dy = y1 - y0  # 图像坐标系中y轴向下为正
                        # 计算线段与垂直轴的夹角（90度减去与水平轴的夹角）
                        angle_rad = math.atan2(dx, dy)
                        line_angle = math.degrees(angle_rad)
            logger.info('=====================>位置偏移为0!, 角度: %d', line_angle)
            if line_angle == 0:
                return 1
        except Exception as e:
            logger.error('获取角度错误: {}'.format(e))
            pass
    cap.release()
    time.sleep(1)
    return 0

def readNtrip2Uart():
    print(1)

def readRTK(ser_rtk_params, sync_threshold = 0.5,timeout = 1):
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
def _readRTK_v2_legacy(ser_rtk_params, sync_threshold = 0.5, timeout = 1):
    global ser_rtk
    buffer = {}
    last_heading = None
    last_quality = None
    correction_runtime = get_shared_runtime(logger)
    last_diag_gga_at = [0.0]
    last_diag_hpr_at = [0.0]
    last_diag_yield_at = [0.0]
    last_yield_at = [0.0]

    def _nmea_utc(line):
        parts = line.split(',')
        if len(parts) > 1:
            return parts[1]
        return ''

    def _age_text(ts, now):
        if ts is None:
            return 'none'
        try:
            return '%.3f' % (now - float(ts))
        except Exception:
            return 'bad'

    def _diag_raw_gga(port, line, quality, gps):
        # RTK_DIAG raw_gga log disabled.
        return
        now = time.time()
        if now - last_diag_gga_at[0] < 1.0:
            return
        last_diag_gga_at[0] = now
        logger.warning(
            "RTK_DIAG raw_gga port={} utc={} quality={} lat={} lon={} last_heading={}".format(
                port, _nmea_utc(line), quality,
                gps[0] if gps else None,
                gps[1] if gps else None,
                last_heading
            )
        )

    def _diag_raw_hpr(port, line, heading_info):
        # RTK_DIAG raw_hpr log disabled.
        return
        now = time.time()
        if now - last_diag_hpr_at[0] < 1.0:
            return
        last_diag_hpr_at[0] = now
        logger.warning(
            "RTK_DIAG raw_hpr port={} utc={} heading={} pitch={}".format(
                port, _nmea_utc(line), heading_info[0], heading_info[1]
            )
        )

    def _diag_yield(reason, lat, lon, heading_deg, gga_ts, ths_ts):
        # RTK_DIAG yield log disabled.
        return
        now = time.time()
        gap = 0.0
        if last_yield_at[0] > 0:
            gap = now - last_yield_at[0]
        last_yield_at[0] = now
        if now - last_diag_yield_at[0] < 1.0 and gap <= 0.3:
            return
        last_diag_yield_at[0] = now
        logger.warning(
            "RTK_DIAG yield reason={} lat={} lon={} heading={} gap={:.3f}s gga_age={}s hpr_age={}s".format(
                reason, lat, lon, heading_deg, gap,
                _age_text(gga_ts, now),
                _age_text(ths_ts, now)
            )
        )

    while True:
        try:
            rtk_port = util.findPort('$GN')
            if not rtk_port:
                logger.warning("鏈壘鍒癛TK涓插彛锛岀瓑寰呴噸璇?...")
                time.sleep(1)
                continue

            with serial.Serial(rtk_port, ser_rtk_params['baudRate'], timeout=0.1) as ser_rtk:
                logger.warning("RTK涓插彛宸叉墦寮€: {}".format(rtk_port))
                configure_rtk_output(ser_rtk)
                _flush_serial_input(ser_rtk, 'open', rtk_port)
                while ser_rtk.is_open:
                    try:
                        correction_runtime.step(ser_rtk)
                        line = ser_rtk.readline()
                        if not line:
                            time.sleep(0.01)
                            continue

                        timestamp = time.time()
                        line_str = line.decode('utf-8', errors='ignore').strip()
                        if not line_str:
                            continue

                        if line_str.startswith(("$GNGGA", "$GPGGA", "$GNHPR", "$GPHPR")):
                            nmea_utc = _nmea_utc(line_str)
                            nmea_age = _nmea_utc_age_seconds(nmea_utc, timestamp)
                            if nmea_age is not None and nmea_age > RTK_NMEA_MAX_AGE_SECONDS:
                                logger.warning(
                                    "RTK_DIAG stale_nmea port={} utc={} age={:.3f}s max_age={:.3f}s; flushing input".format(
                                        rtk_port, nmea_utc, nmea_age, RTK_NMEA_MAX_AGE_SECONDS
                                    )
                                )
                                _flush_serial_input(ser_rtk, 'stale_nmea', rtk_port)
                                buffer.clear()
                                last_heading = None
                                time.sleep(0.02)
                                continue

                        if line_str.startswith(("$GNGGA", "$GPGGA")):
                            correction_runtime.observe_gga(line_str)
                            quality = extract_gga_quality(line_str)
                            if quality != last_quality:
                                logger.warning("RTK GGA瀹氫綅鐘舵€佹洿鏂? fix={}".format(quality))
                                last_quality = quality

                            gps = parse_gngga(line_str)
                            _diag_raw_gga(rtk_port, line_str, quality, gps)
                            if gps:
                                buffer['gga'] = {
                                    'timestamp': timestamp,
                                    'lat': gps[0],
                                    'lon': gps[1]
                                }
                                if 'ths' in buffer:
                                    gga_ts = buffer['gga']['timestamp']
                                    ths_ts = buffer['ths']['timestamp']
                                    if abs(gga_ts - ths_ts) < sync_threshold:
                                        lat = buffer['gga']['lat']
                                        lon = buffer['gga']['lon']
                                        heading_deg = buffer['ths']['heading']
                                        del buffer['gga']
                                        del buffer['ths']
                                        last_heading = heading_deg
                                        _diag_yield('matched_gga_hpr', lat, lon, heading_deg, gga_ts, ths_ts)
                                        yield lat, lon, heading_deg
                                    else:
                                        heading_deg = last_heading if last_heading is not None else buffer['ths']['heading']
                                        lat = buffer['gga']['lat']
                                        lon = buffer['gga']['lon']
                                        del buffer['gga']
                                        _diag_yield('stale_heading', lat, lon, heading_deg, gga_ts, ths_ts)
                                        yield lat, lon, heading_deg
                                else:
                                    lat = buffer['gga']['lat']
                                    lon = buffer['gga']['lon']
                                    gga_ts = buffer['gga']['timestamp']
                                    del buffer['gga']
                                    _diag_yield('last_heading', lat, lon, last_heading, gga_ts, None)
                                    yield lat, lon, last_heading

                        elif line_str.startswith(("$GNTHS", "$GPTHS")):
                            heading = parse_GPTHS(line_str)
                            if heading is not None:
                                last_heading = heading
                                buffer['ths'] = {
                                    'timestamp': timestamp,
                                    'heading': heading
                                }

                        elif line_str.startswith(("$GNHPR", "$GPHPR")):
                            heading_info = parse_GNHPR(line_str)
                            if heading_info is not None:
                                _diag_raw_hpr(rtk_port, line_str, heading_info)
                                last_heading = heading_info[0]
                                buffer['ths'] = {
                                    'timestamp': timestamp,
                                    'heading': heading_info[0]
                                }
                    except Exception as e:
                        logger.error("璇诲彇RTK鏁版嵁寮傚父: {}".format(e))
                        break
        except serial.SerialException as e:
            logger.error("鎵撳紑RTK涓插彛澶辫触: {}".format(e))
        except Exception as e:
            logger.error("RTK涓插彛鍏朵粬閿欒: {}".format(e))
        finally:
            correction_runtime.close()

        time.sleep(1)


def readRTK_v2(ser_rtk_params, sync_threshold = 0.5, timeout = 1):
    global ser_rtk
    correction_runtime = get_shared_runtime(logger)
    last_stale_log_at = 0.0

    while True:
        ntrip_stop_event = None
        ntrip_thread = None
        try:
            rtk_port = util.findPort('$GN')
            if not rtk_port:
                logger.warning("鏈壘鍒癛TK涓插彛锛岀瓑寰呴噸璇?...")
                time.sleep(1)
                continue

            with serial.Serial(rtk_port, ser_rtk_params['baudRate'], timeout=0.1) as ser_rtk:
                logger.warning("RTK涓插彛宸叉墦寮€: {}".format(rtk_port))
                state = _RtkLatestState(sync_threshold=sync_threshold)
                configure_rtk_output(ser_rtk)
                _flush_serial_input(ser_rtk, 'open', rtk_port)
                ntrip_stop_event, ntrip_thread = _start_ntrip_serial_worker(correction_runtime, ser_rtk)

                while _serial_is_open(ser_rtk):
                    try:
                        lines = _read_serial_lines_now(ser_rtk, RTK_DRAIN_MAX_LINES)
                        if not lines:
                            time.sleep(0.01)
                            continue

                        latest_sample, stats = _consume_rtk_nmea_lines(
                            state,
                            lines,
                            port=rtk_port,
                            correction_runtime=correction_runtime,
                            log_events=True
                        )

                        if stats.get('stale'):
                            now = time.time()
                            if now - last_stale_log_at >= 1.0:
                                first_age = stats.get('first_stale_age')
                                logger.warning(
                                    "RTK_DIAG stale_nmea skipped port={} count={} first_utc={} first_age={} max_age={:.3f}".format(
                                        rtk_port,
                                        stats.get('stale'),
                                        stats.get('first_stale_utc'),
                                        "%.3f" % first_age if first_age is not None else "none",
                                        RTK_NMEA_MAX_AGE_SECONDS
                                    )
                                )
                                last_stale_log_at = now

                        if len(lines) >= RTK_DRAIN_MAX_LINES and _serial_in_waiting(ser_rtk) > 0:
                            _flush_serial_input(ser_rtk, 'backlog_overflow', rtk_port)

                        if latest_sample is not None:
                            yield latest_sample[0], latest_sample[1], latest_sample[2]
                    except Exception as e:
                        logger.error("璇诲彇RTK鏁版嵁寮傚父: {}".format(e))
                        break
        except serial.SerialException as e:
            logger.error("鎵撳紑RTK涓插彛澶辫触: {}".format(e))
        except Exception as e:
            logger.error("RTK涓插彛鍏朵粬閿欒: {}".format(e))
        finally:
            if ntrip_stop_event is not None:
                ntrip_stop_event.set()
            if ntrip_thread is not None:
                try:
                    ntrip_thread.join(1.0)
                except Exception:
                    pass
            correction_runtime.close()

        time.sleep(1)


# def readRTK(ser_rtk_params, sync_threshold = 0.5,timeout = 5):
#     buffer = {}  # 缓存最近的 GGA 和 THS 数据
#     global ser_rtk
#     last_data_time = time.time()  # 记录最后一次收到任何数据的时间
#     if ser_rtk is None:
#         ser_rtk = serial.Serial(ser_rtk_params['port'], ser_rtk_params['baudRate'], rtscts=True,timeout=0.01)
#     raw_buffer = b''
#     while True:
#         try:
#             current_time = time.time()
#             # 超时判断
#             # if current_time - last_data_time > timeout:
#             #     raise RTKTimeoutError("RTK设备超时：{}秒内未收到任何数据".format(timeout))
#             #     logger.warn("RTK设备超时：{}秒内未收到任何数据".format(timeout))
#             #     logger.warn("重新连接RTK")
#             # 确保串口连接有效
#             if ser_rtk is None or not ser_rtk.is_open:
#                 rtk_port = util.findPort('"$GN"')
#                 if rtk_port is None:
#                     rtk_port = ser_rtk_params['port']
#                 ser_rtk = serial.Serial(rtk_port, ser_rtk_params['baudRate'], rtscts=True, timeout=0.01)
#                 # ser_rtk = serial.Serial(ser_rtk_params['port'], ser_rtk_params['baudRate'], rtscts=True, timeout=0.01)
#
#             raw_buffer += ser_rtk.read(ser_rtk.in_waiting)
#             # 按帧分割处理
#             while b'\n' in raw_buffer:
#                 line, raw_buffer = raw_buffer.split(b'\n', 1)
#                 if line:
#                     timestamp = time.time()  # 记录当前时间戳
#                     # logger.info(line)
#                     if line.startswith(b"$GNGGA"):
#                         gps = parse_gngga(line.decode('utf-8', errors='ignore').strip())
#                         # logger.warning(gps)
#                         if gps:
#                             buffer['gga'] = {
#                                 'timestamp': timestamp,
#                                 'lat': gps[0],
#                                 'lon': gps[1]
#                             }
#                             # yield gps[0], gps[1]
#                     elif line.startswith(b"$GNTHS"):
#                         heading = parse_GPTHS(line.decode('utf-8', errors='ignore').strip())
#                         buffer['ths'] = {
#                             'timestamp': timestamp,
#                             'heading': heading
#                         }
#
#                     # 同步 GGA 和 VTG 数据
#                     if 'gga' in buffer and 'ths' in buffer:
#                         gga_ts = buffer['gga']['timestamp']
#                         ths_ts = buffer['ths']['timestamp']
#
#                         if abs(gga_ts - ths_ts) < sync_threshold:  # 时间差阈值为 0.5 秒
#                             lat = buffer['gga']['lat']
#                             lon = buffer['gga']['lon']
#                             heading_deg = buffer['ths']['heading']
#
#                             # 清空缓冲区，准备接收下一次数据
#                             del buffer['gga']
#                             del buffer['ths']
#
#                             last_data_time = current_time
#
#                             yield lat, lon, heading_deg
#         except serial.SerialException as e:
#             logger.error("RTK串口异常：{}".format(e))
#             if ser_rtk:
#                 ser_rtk.close()
#             ser_rtk = None
#             logger.info('waiting for RTK')
#             time.sleep(6)  # 等待后尝试重连
#             # raise e
#         except Exception as e:
#             logger.error("RTK处理过程未知错误：{}".format(e))
#             if ser_rtk:
#                 ser_rtk.close()
#             ser_rtk = None
#             logger.info('waiting for RTK')
#             time.sleep(6)  # 等待后尝试重连
#             # raise e
def closeSerRtk():
    global ser_rtk
    if ser_rtk is not None and ser_rtk.is_open:
        logger.warn("关闭RTK")
        ser_rtk.close()
        ser_rtk = None

def get_distance_angle(lat1, lon1, lat2, lon2):
    geod = Geodesic.WGS84
    result = geod.Inverse(lat1, lon1, lat2, lon2)

    distance = result['s12']  # 米
    # angle = result['azi1'] % 360    # 确保角度永远是正数
    angle = result['azi1'] % 360    # 确保角度永远是正数
    # logger.warn('angle={}'.format(angle))
    angle = (angle+90)%360
    # logger.warn('angle={}'.format(angle))
    return distance, angle

# 已知起点，角度，距离，计算终点GPS
#使用示例：
# lat3, lon3 = get_B_GPS(lat1, lon1, distance_m, azimuth_deg)
# print(f"终点GPS: lat3={lat3:.8f}, lon3={lon3:.8f}")
def get_B_GPS(lat1, lon1, distance_m, azimuth_deg):
    geod = Geodesic.WGS84
    result = geod.Direct(lat1, lon1, azimuth_deg, distance_m)
    return round(result['lat2'],8), round(result['lon2'],8)

def haversine_distance(lat1, lon1, lat2, lon2):
    """
    计算两个经纬度点之间的大圆距离（米）
    """
    R = 6371000  # 地球半径，单位：米
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = (math.sin(delta_phi / 2) * math.sin(delta_phi / 2) +
         math.cos(phi1) * math.cos(phi2) *
         math.sin(delta_lambda / 2) * math.sin(delta_lambda / 2))
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c

def calculate_bearing(lat1, lon1, lat2, lon2):
    """
    计算从点1到点2的初始大地方位角（角度）
    """
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_lambda = math.radians(lon2 - lon1)

    y = math.sin(delta_lambda) * math.cos(phi2)
    x = (math.cos(phi1) * math.sin(phi2) -
         math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda))
    bearing = math.atan2(y, x)
    return (math.degrees(bearing) + 360) % 360

def cross_track_error(start_lat, start_lon, end_lat, end_lon, current_lat, current_lon):
    """
    计算当前位置相对于起点到终点路径的横向偏差（米）

    参数:
        start_lat, start_lon: 起点经纬度
        end_lat, end_lon: 终点经纬度
        current_lat, current_lon: 当前位置经纬度

    返回:
        横向偏差（米），正值表示在航线右侧，负值表示在航线左侧
    """
    # 1. 计算起点到终点的方位角
    bearing_start_to_end = calculate_bearing(start_lat, start_lon, end_lat, end_lon)

    # 2. 计算起点到当前位置的方位角
    bearing_start_to_current = calculate_bearing(start_lat, start_lon, current_lat, current_lon)

    # 3. 计算起点到当前位置的大圆距离
    distance_start_to_current = haversine_distance(start_lat, start_lon, current_lat, current_lon)

    # 4. 计算方位角差
    delta_bearing = math.radians(bearing_start_to_current - bearing_start_to_end)

    # 5. 使用球面三角公式计算横向偏差
    # d_xt = asin(sin(d_13/R) * sin(θ_13 - θ_12)) * R
    R = 6371000  # 地球半径（米）
    d_xt = math.asin(math.sin(distance_start_to_current / R) * math.sin(delta_bearing)) * R

    return d_xt


def signed_along_track_distance(start_lat, start_lon, end_lat, end_lon, current_lat, current_lon):
    """Return signed remaining distance along the start-to-end path in meters."""
    geod = Geodesic.WGS84
    path = geod.Inverse(start_lat, start_lon, end_lat, end_lon)
    current = geod.Inverse(start_lat, start_lon, current_lat, current_lon)
    delta = math.radians(current['azi1'] - path['azi1'])
    along_distance = current['s12'] * math.cos(delta)
    return path['s12'] - along_distance


def should_finish_point_to_point(distance_to_target, signed_remaining, cte,
                                 target_tolerance_m=0.03, cte_tolerance_m=0.30):
    if distance_to_target is not None and float(distance_to_target) <= target_tolerance_m:
        return True
    if signed_remaining is None or cte is None:
        return False
    return float(signed_remaining) <= 0 and abs(float(cte)) <= cte_tolerance_m


def calculate_perpendicular_point(lat1, lon1, lat2, lon2, distance_meters, side='left'):
    """
    计算从线段 (P1-P2) 垂直偏移 distance_meters 的点
    :param lat1, lon1: 第一个点的纬度、经度（度）
    :param lat2, lon2: 第二个点的纬度、经度（度）
    :param distance_meters: 垂直距离（米），正数表示偏移方向
    :param side: 'left' 或 'right'，相对于从 P1 到 P2 的方向
    :return: (lat, lon) 偏移点的经纬度
    """
    geod = Geodesic.WGS84

    # 1. 计算 P1 -> P2 的正向方位角 (degrees)
    g = geod.Inverse(lat1, lon1, lat2, lon2)
    azimuth = g['azi1']  # 从 P1 到 P2 的方位角

    # 2. 计算 P1 -> P2 的中点
    # 方法：从 P1 沿 azimuth 方向走一半距离
    half_distance = g['s12'] / 2.0
    g_mid = geod.Direct(lat1, lon1, azimuth, half_distance)
    mid_lat = g_mid['lat2']
    mid_lon = g_mid['lon2']

    # 3. 计算垂直方向的方位角
    if side == 'left':
        perpendicular_azimuth = (azimuth + 90) % 360
    else:  # 'right'
        perpendicular_azimuth = (azimuth - 90) % 360

    # 4. 从中点出发，沿垂直方向走 distance_meters
    g_perp = geod.Direct(mid_lat, mid_lon, perpendicular_azimuth, distance_meters)
    return g_perp['lat2'], g_perp['lon2']
# 将角度归一化到-180到180范围
def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def linear_decay(distance, max_distance, max_value=1.0):
    """
    线性衰减函数。
    :param distance: 当前终点的距离。
    :param max_distance: 被认为是“远”的最大距离，超过此距离返回值为0。
    :param max_value: 距离为0时的返回值。
    :return: 衰减后的值。
    """
    if distance >= max_distance:
        return 0.0
    # 确保返回值不会小于0
    temp = max_value * (1.0 - float(distance) / max_distance)
    return max(0.0, temp)
# 示例
# print(linear_decay(0, 10))    # 输出: 1.0
# print(linear_decay(5, 10))    # 输出: 0.5
# print(linear_decay(10, 10))   # 输出: 0.0
# print(linear_decay(15, 10))   # 输出: 0.0

def square_with_original_sign(num):
    """
    返回一个数的平方，保持原数的正负符号。
    """
    # 计算绝对值的平方
    abs_squared = abs(num) ** 2
    # 保持原数的正负符号
    if num < 0:
        return -abs_squared
    else:
        return abs_squared

import math

def destination_point(lat1_deg, lon1_deg, bearing_deg, distance_m):
    R = 6371000  # 地球半径，单位：米

    # 转换为弧度
    lat1 = math.radians(lat1_deg)
    lon1 = math.radians(lon1_deg)
    bearing = math.radians(bearing_deg)
    delta = distance_m / R  # 中心角

    # 计算目标点纬度
    lat2 = math.asin(
        math.sin(lat1) * math.cos(delta) +
        math.cos(lat1) * math.sin(delta) * math.cos(bearing)
    )

    # 计算目标点经度
    lon2 = lon1 + math.atan2(
        math.sin(bearing) * math.sin(delta) * math.cos(lat1),
        math.cos(delta) - math.sin(lat1) * math.sin(lat2)
    )

    # 规范化经度
    lon2 = (lon2 + math.pi) % (2 * math.pi) - math.pi

    # 转回十进制度
    return math.degrees(lat2), math.degrees(lon2)

def local_rotated_xy_to_latlon(lat0, lon0, x, y, y_axis_bearing):
    """
    反向：从自定义坐标系 (x,y) 计算目标经纬度
    """
    R = 6371000
    # 将度数转化为弧数
    bearing_rad = math.radians(y_axis_bearing)

    # 逆旋转：把 x,y 转回北-东坐标系
    cos_b = math.cos(bearing_rad)
    sin_b = math.sin(bearing_rad)

    dy_north = y * cos_b + x * sin_b
    dx_east  = y * (-sin_b) + x * cos_b

    # 转为经纬度
    dlat = dy_north / R * (180 / math.pi)
    dlon = dx_east / (R * math.cos(math.radians(lat0))) * (180 / math.pi)

    return round(lat0 + dlat,8),round(lon0 + dlon,8)

def local_rotated_xy_to_latlon_precise(lat0, lon0, x, y, y_axis_bearing):
    """
    高精度版本：根据自定义旋转坐标系 (x, y) 计算目标经纬度
    使用大地测量学方法，适用于任意距离

    参数：
        lon0, lat0: 原点经纬度
        x: 在自定义坐标系中的东向分量（X轴 = Y轴顺时针90°）
        y: 在自定义坐标系中的北向分量（Y轴方向为 y_axis_bearing）
        y_axis_bearing: Y轴方向（从正北顺时针的角度，单位：度）
    返回：
        lon, lat: 目标点经纬度
    """
    geod = Geodesic.WGS84

    # 第一步：从原点沿 Y 轴方向走 y 米
    result1 = geod.Direct(lat0, lon0, y_axis_bearing, y)
    lat_temp, lon_temp = result1['lat2'], result1['lon2']

    # 第二步：从中间点沿 X 轴方向走 x 米
    # X轴 = Y轴顺时针旋转90° → 方位角 = y_axis_bearing + 90°
    x_axis_bearing = (y_axis_bearing + 90) % 360
    result2 = geod.Direct(lat_temp, lon_temp, x_axis_bearing, x)

    return round(result2['lat2'],8),round(result2['lon2'],8)


def latlon_to_local_rotated_xy_precise(lat0, lon0, lat, lon, y_axis_bearing):
    """
    local_rotated_xy_to_latlon_precise 的逆运算
    给定原点 (lat0, lon0)、目标点 (lat, lon) 与 Y 轴方位角（从正北顺时针）
    返回自定义旋转坐标系下的 (x, y)，单位：米
    约定：Y 轴 = y_axis_bearing；X 轴 = Y 轴顺时针 90°
    """
    geod = Geodesic.WGS84
    result = geod.Inverse(lat0, lon0, lat, lon)
    distance = result['s12']
    azimuth = result['azi1']
    delta = math.radians((azimuth - y_axis_bearing) % 360)
    y = distance * math.cos(delta)
    x = distance * math.sin(delta)
    return x, y

def findPort(target_magic,timeout=3):
    ports = [p.device for p in serial.tools.list_ports.comports()]
    logger.warning("发现串口：%s", ports)

    for port in ports:
        try:
            logger.warning("正在测试 %s", port)
            with serial.Serial(
                    port, baudrate=115200, timeout=0.5,  # 小 timeout 更可控
                    exclusive=True  # 防止被其他进程抢占（Linux 支持）
            ) as ser:
                # 关键：等待设备稳定 + 清空所有残余
                time.sleep(1.0)
                ser.flushInput()
                time.sleep(0.1)
                while ser.in_waiting:
                    ser.read(ser.in_waiting)
                    time.sleep(0.01)

                start = time.time()
                buffer = ""
                while time.time() - start < timeout:
                    if ser.in_waiting:
                        raw = ser.read(ser.in_waiting)
                        try:
                            buffer += raw.decode('utf-8', errors='ignore')
                            # 检查是否包含目标暗号（不要求整行）
                            if target_magic in buffer:
                                logger.warning("找到目标串口：%s", port)
                                return port
                        except:
                            pass
                    time.sleep(0.1)
        except Exception as e:
            logger.error("端口 %s 异常: %s", port, e)
    return None

# 根据主天线位置计算小车中心点位置，小车的主天线在中心位置的左前方
# lat_m,lon_m,heading_deg:小车当前经纬度和航向角
# offset_x_front:主天线距离中心的左距离
# offset_y_left:主天线距离中心的前距离
def compute_center_from_master(lat_m, lon_m, heading_deg,offset_x_front, offset_y_left):
    lat, lon = util.get_B_GPS(lat_m, lon_m, offset_y_left, heading_deg)
    c_lat,c_lon = util.get_B_GPS(lat,lon,offset_x_front,(heading_deg+90)%360)
    return c_lat,c_lon

def compute_center_from_antenna(lat_ant, lon_ant, heading_deg, dx_back, dy_right):
    """
    已知主天线位置和航向，计算小车中心点位置（中心在主天线后方 dx_back、右侧 dy_right）

    参数:
        lat_ant, lon_ant: 主天线经纬度（度）
        heading_deg: 航向角（度），0=正北，顺时针增加
        dx_back: 中心点在主天线后方的距离（米），>0
        dy_right: 中心点在主天线右侧的距离（米），>0

    返回:
        (lat_center, lon_center) 单位：度
    """
    # 航向转弧度
    theta = math.radians(heading_deg)

    # 计算 ENU 偏移（从主天线到中心点）
    delta_e = -dx_back * math.sin(theta) - dy_right * math.cos(theta)
    delta_n = -dx_back * math.cos(theta) + dy_right * math.sin(theta)

    # 使用 geodesic 正解：从 (lat_ant, lon_ant) 出发，沿 (delta_e, delta_n) 移动
    geod = Geodesic.WGS84

    # 先计算方位角和距离
    distance = math.hypot(delta_e, delta_n)
    if distance < 1e-6:
        return lat_ant, lon_ant

    azimuth = math.degrees(math.atan2(delta_e, delta_n))  # 注意：atan2(E, N)

    # 正解计算新位置
    result = geod.Direct(lat_ant, lon_ant, azimuth, distance)
    lat_center = result['lat2']
    lon_center = result['lon2']

    return round(lat_center,8), round(lon_center,8)


def calculate_center_offset(lat_main, lon_main, heading_deg, dist_right, dist_back):
    """
    计算小车中心点经纬度
    :param lat_main: 主天线纬度 (度)
    :param lon_main: 主天线经度 (度)
    :param heading_deg: 车辆航向角 (度), 北0, 东90, 南180, 西270
    :param dist_right: 中心点在主天线右侧的距离 (米) (横向偏移)
    :param dist_back: 中心点在主天线后方的距离 (米) (纵向偏移，即您说的"下")
                      如果中心点在主天线前方，此值设为负数
    :return: (center_lat, center_lon)
    """
    R = 6371000.0  # 地球半径 (米)

    # 1. 角度转弧度
    lat_rad = math.radians(lat_main)
    heading_rad = math.radians(heading_deg)

    # 2. 计算局部东北坐标系下的增量 (ENU)
    # 公式推导：
    # 向右向量 (Heading + 90): (cos(h), -sin(h)) * dist_right
    # 向后向量 (Heading + 180): (-sin(h), -cos(h)) * dist_back

    delta_east = (dist_right * math.cos(heading_rad)) - (dist_back * math.sin(heading_rad))
    delta_north = (-dist_right * math.sin(heading_rad)) - (dist_back * math.cos(heading_rad))

    # 3. 将米转换为经纬度增量
    delta_lat_rad = delta_north / R
    # 经度方向需要除以纬度的余弦值，因为经线在两极收敛
    delta_lon_rad = delta_east / (R * math.cos(lat_rad))

    # 4. 计算最终坐标
    center_lat_rad = lat_rad + delta_lat_rad
    center_lon_rad = math.radians(lon_main) + delta_lon_rad

    # 5. 转回度数
    lat_center = math.degrees(center_lat_rad)
    lon_center = math.degrees(center_lon_rad)

    return round(lat_center,8), round(lon_center,8)

def calculate_center_gps(lat_main, lon_main, heading, offset_x, offset_y):
    """
    计算小车中心点的经纬度

    参数:
    lat_main (float): 主天线纬度 (十进制度数)
    lon_main (float): 主天线经度 (十进制度数)
    heading (float): 航向角 (度, 0度为正北, 顺时针增加)
    offset_x (float): 中心点相对于主天线的横向偏移 (米, 向右为正)
    offset_y (float): 中心点相对于主天线的纵向偏移 (米, 向前为正)

    返回:
    tuple: (中心点纬度, 中心点经度)
    """

    # 将航向角转换为弧度
    heading_rad = math.radians(heading)

    # 将偏移量转换为东向和北向的偏移量
    delta_east = offset_x * math.cos(heading_rad) + offset_y * math.sin(heading_rad)
    delta_north = -offset_x * math.sin(heading_rad) + offset_y * math.cos(heading_rad)

    # 将北向偏移量转换为纬度修正值
    delta_lat = delta_north / 111139.0

    # 将东向偏移量转换为经度修正值
    # 注意：经度每度的米数随纬度变化，需要乘以cos(纬度)
    delta_lon = delta_east / (111139.0 * math.cos(math.radians(lat_main)))

    # 计算中心点的经纬度
    lat_center = lat_main + delta_lat
    lon_center = lon_main + delta_lon

    return lat_center, lon_center


def test01():
    # 主天线位置
    lat_ant = 31.01234567
    lon_ant = 121.09876543

    # 航向：车头朝东北（45°）
    heading = 45.0  # 度

    # 中心点在主天线后方 0.3m，右侧 0.2m
    dx = 0.3  # 后
    dy = 0.2  # 右

    lat_c, lon_c = compute_center_from_antenna(lat_ant, lon_ant, heading, dx, dy)

    print("主天线: {:.8f}, {:.8f}".format(lat_ant, lon_ant))
    print("中心点: {:.8f}, {:.8f}").format(lat_c, lon_c)
    print("偏移: 后 {}m, 右 {}m, 航向 {}°".format(dx,dy,heading))
def test02():
    lat1,lon1,heading1 = 32.03659781,118.92461508,186.76
    lat2,lon2,heading2 = 32.03659481,118.92461393,276.83
    lat3,lon3,heading3 = 32.03659587,118.92461011,6.34
    lat4,lon4,heading4 = 32.03659864,118.92461165,96.24

    c1 = util.compute_center_from_master(lat1,lon1,heading1,0.2,0.13)
    c2 = util.compute_center_from_master(lat2,lon2,heading2,0.2,0.13)
    c3 = util.compute_center_from_master(lat3,lon3,heading3,0.2,0.13)
    c4 = util.compute_center_from_master(lat4,lon4,heading4,0.2,0.13)
    print(c1)
    print(c2)
    print(c3)
    print(c4)
    dis, h = util.get_distance_angle(c1[0], c1[1], c2[0], c2[1])
    print(dis)

    dis, h = util.get_distance_angle(c1[0], c1[1], c3[0], c3[1])
    print(dis)

    dis, h = util.get_distance_angle(c1[0], c1[1], c4[0], c4[1])
    print(dis)
    # ------------------------------------
    dis, h = util.get_distance_angle(c2[0], c2[1], c3[0], c3[1])
    print(dis)

    dis, h = util.get_distance_angle(c2[0], c2[1], c4[0], c4[1])
    print(dis)
    # ------------------------------------
    dis, h = util.get_distance_angle(c3[0], c3[1], c4[0], c4[1])
    print(dis)
if __name__ == '__main__':
    # test01()
    test02()


    # command = bytearray(17)
    # command[0] = 123
    # command[3] = 0
    # command[6] = 0
    # command[7] = 0
    # command[11] = 0
    # command[12] = 0
    # command[13] = 0
    # command[14] = 0
    # command[15] = 0
    # command[16] = 125
    # hex_string = hex(360)[2:]
    # print(hex_string)
    # hex_1 = "0" + hex_string[0]
    # hex_2 = hex_string[1] + hex_string[2]
    # command[11] = int(hex_1, 16)
    # command[12] = int(hex_2,16)
    #
    # print(' '.join(format(x, '02x') for x in command))
