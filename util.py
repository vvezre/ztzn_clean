# coding=utf-8
import binascii
import copy
import json
import time
from collections import defaultdict

import cv2
import serial
import serial.tools.list_ports

from geographiclib.geodesic import Geodesic

import util
from AppLogger import logger

ser_rtk = None

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
                      gap=3,angle_radians=0.06,angle_to='y'):
    # logger.warn(panelInfoList)
    # 计算y轴上特定间隙总长
    totalGapLen = 0
    for rowPanelInfo in panelInfoList:
        # 有间隙则累加
        if rowPanelInfo["isGap"]:
            totalGapLen = totalGapLen + int(rowPanelInfo['gapLen'])
    height = len(panelInfoList) * panelHeight + (lineCount - 1) * gap + totalGapLen - panelHeight * 0.5
    H = panelHeight * 0.5
    # 如果y轴上有角度，height就需要cos,width就不需要处理
    if angle_to == 'y':
        height = int(height * math.cos(angle_radians))
        H = int(H * math.cos(angle_radians))
    # 第一步直行任务
    goTask = [{"angle": 0, "mode": 1, "length": height, "turn_back_len": turnBackLen, "back_len": goBackLen}]
    # 向下任务
    downTask = {"angle": 180, "mode": 2, "length": H, "turn_back_len": turnBackLen, "back_len": 0}

    # 计算每一行x轴长度
    for index,rowPanelInfo in enumerate(panelInfoList):
        width = rowPanelInfo['column'] * panelWidth + (rowPanelInfo['column'] - 1) * gap - panelWidth
        # 如果是x轴上有角度，不是y轴上有角度，width就需要cos
        if angle_to != 'y':
            width = int(width * math.cos(angle_radians))
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
                areaEndTask = {"angle": angle, "mode": 2, "length": panelWidth + leftOrRightBridgeLen,
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
            if nextRowPanelInfo["isGap"]:
                gapLen = int(nextRowPanelInfo["gapLen"])
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
                    item['endY'] = pre_item['endY'] - item['length'] - gap
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
                    item['endY'] = pre_item['endY'] - item['length'] - gap
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

# 分析GNHPR数据,返回航向角和俯仰角
def parse_GNHPR(line):
    if not line.startswith("$GNHPR"):
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

def readRTK(ser_rtk_params, sync_threshold = 0.5,timeout = 5):
    buffer = {}  # 缓存最近的 GGA 和 THS 数据
    global ser_rtk
    last_data_time = time.time()  # 记录最后一次收到任何数据的时间
    if ser_rtk is None:
        ser_rtk = serial.Serial(ser_rtk_params['port'], ser_rtk_params['baudRate'], rtscts=True,timeout=0.01)
    raw_buffer = b''
    while True:
        try:
            current_time = time.time()
            # 超时判断
            # if current_time - last_data_time > timeout:
            #     raise RTKTimeoutError("RTK设备超时：{}秒内未收到任何数据".format(timeout))
            #     logger.warn("RTK设备超时：{}秒内未收到任何数据".format(timeout))
            #     logger.warn("重新连接RTK")
            # 确保串口连接有效
            if ser_rtk is None or not ser_rtk.is_open:
                ser_rtk = serial.Serial(ser_rtk_params['port'], ser_rtk_params['baudRate'], rtscts=True, timeout=0.01)

            raw_buffer += ser_rtk.read(ser_rtk.in_waiting)
            # 按帧分割处理
            while b'\n' in raw_buffer:
                line, raw_buffer = raw_buffer.split(b'\n', 1)
                if line:
                    timestamp = time.time()  # 记录当前时间戳
                    # logger.info(line)
                    if line.startswith(b"$GNGGA"):
                        gps = parse_gngga(line.decode('utf-8', errors='ignore').strip())
                        # logger.warning(gps)
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

                            last_data_time = current_time

                            yield lat, lon, heading_deg
        except serial.SerialException as e:
            logger.error("RTK串口异常：{}".format(e))
            if ser_rtk:
                ser_rtk.close()
            ser_rtk = None
            time.sleep(1)  # 等待后尝试重连
            raise e
        except Exception as e:
            logger.error("RTK处理过程未知错误：{}".format(e))
            if ser_rtk:
                ser_rtk.close()
            ser_rtk = None
            time.sleep(1)  # 等待后尝试重连
            raise e
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
    # ports = [p.device for p in serial.tools.list_ports.comports()]
    # logger.warning("发现串口：%s", ports)
    #
    # for port in ports:
    #     try:
    #         logger.warning("正在测试 %s 串口", port)
    #         with serial.Serial(port, baudrate=115200, timeout=1) as ser:
    #             logger.warning("正在测试 %s 串口", port)
    #             # 给设备一点时间启动（尤其嵌入式设备上电后需要初始化）
    #             time.sleep(1.0)
    #             # 清空缓冲区：多次 flush + 读空
    #             ser.flushInput()
    #             time.sleep(0.1)
    #             while ser.in_waiting > 0:
    #                 ser.read(ser.in_waiting)  # 丢弃所有残余数据
    #                 time.sleep(0.01)
    #             # 现在开始监听
    #             start = time.time()
    #             buffer = b''
    #             while time.time() - start < 3:
    #                 if ser.in_waiting:
    #                     chunk = ser.read(ser.in_waiting)
    #                     buffer += chunk
    #                     try:
    #                         text = buffer.decode('utf-8', errors='ignore')
    #                         if target_magic in text:
    #                             logger.warning("找到目标串口：%s", port)
    #                             return port+''
    #                     except:
    #                         pass
    #                 time.sleep(0.1)
    #     except Exception as e:
    #         logger.error("串口 %s 打开或通信失败：%s", port, e)
    #         continue
    # return None
    # 列出当前所有串口
    # ports = [p.device for p in serial.tools.list_ports.comports()]
    # logger.warn("发现串口：{}".format(ports))
    # # 逐个尝试
    # for port in ports:
    #     try:
    #         # 打开串口
    #         with serial.Serial(port, baudrate=115200, timeout=3) as ser:
    #             logger.warn("正在测试{}串口".format(port))
    #             # 清空旧数据
    #             ser.flushInput()
    #             # 读一行
    #             line = ser.readline().decode().strip()
    #             # 判断是否为暗号
    #             if line.startswith(target_magic):
    #                 logger.warn("找到目标串口：{}".format(port))
    #                 return port+""
    #     except Exception as e:
    #         logger.error("{}串口打开失败：{}".format(port,e))
    #         continue

# 根据主天线位置计算小车中心点位置，小车的主天线在中心位置的左前方
# lat_m,lon_m,heading_deg:小车当前经纬度和航向角
# offset_x_front:主天线距离中心的左距离
# offset_y_left:主天线距离中心的前距离
def compute_center_from_master(lat_m, lon_m, heading_deg,offset_x_front, offset_y_left):
    lat, lon = util.get_B_GPS(lat_m, lon_m, offset_y_left, heading_deg)
    c_lat,c_lon = util.get_B_GPS(lat,lon,offset_x_front,(heading_deg+90)%360)
    return c_lat,c_lon

if __name__ == '__main__':
    lat1,lon1,heading1 = 32.03659588,118.92461104,2.07
    lat2,lon2,heading2 = 32.03659839,118.92461165,93.95
    lat3,lon3,heading3 = 32.03659779,118.92461472,184.94
    lat4,lon4,heading4 = 32.03659538,118.92461397,271.75
    # c1 = util.compute_center_from_master(lat1,lon1,heading1,0.12,0.08)
    # c2 = util.compute_center_from_master(lat2,lon2,heading2,0.12,0.08)
    # print(c1)
    # print(c2)
    # dis,h = util.get_distance_angle(c1[0],c1[1],c2[0],c2[1])
    # print(dis)

    c1 = util.compute_center_from_master(lat1,lon1,heading1,0.17,0.11)
    c2 = util.compute_center_from_master(lat2,lon2,heading2,0.17,0.11)
    c3 = util.compute_center_from_master(lat3,lon3,heading3,0.17,0.11)
    c4 = util.compute_center_from_master(lat4,lon4,heading4,0.17,0.11)
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
