# coding=utf-8
import binascii
import copy
import json
import time
from collections import defaultdict

from AppLogger import logger


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
def createTask(direction='left', panels=None, areaNumber=1,goBackLen=5, goLeftOrRightBackLen=15, turnBackLen=10,
               panelWidth=400, panelHeight=100,
               upOrDownBridgeLen=50, leftOrRightBridgeLen=150):
    if panels is None:
        panels = []
    list = [
        {"angle": 90, "mode": 1, "length": 0, "turn_back_len": turnBackLen, "back_len": goLeftOrRightBackLen},
        {"angle": 180, "mode": 2, "length": panelHeight, "turn_back_len": turnBackLen, "back_len": 0},
        {"angle": 270, "mode": 1, "length": 0, "turn_back_len": turnBackLen, "back_len": goLeftOrRightBackLen},
    ]
    # 如果从右侧出发，那就先向270方向转
    if direction == 'right':
        list = list[::-1]
    # 第一步直行任务
    goTask = [{"angle":0,"mode":1,"length":0,"turn_back_len":turnBackLen,"back_len":goBackLen}]
    # 向下跨桥任务
    crossTheBridge = {"angle": 180, "mode": 2, "length": panelHeight+upOrDownBridgeLen, "turn_back_len": turnBackLen, "back_len": 0}
    # 非向下跨桥任务
    notCrossTheBridge = {"angle": 180, "mode": 2, "length": panelHeight, "turn_back_len": turnBackLen, "back_len": 0}
    # 走固定距离，两个区域跨桥任务
    endTheBridge = {"angle": 90, "mode": 2, "length": panelWidth+leftOrRightBridgeLen, "turn_back_len": turnBackLen, "back_len": 0}

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
            if next_item == 0:
                tmpList = list + [crossTheBridge]
            else:
                tmpList = list + [notCrossTheBridge]
        goTask.extend(tmpList)

    resultTaskList = deep_copy_list(goTask)
    del goTask
    # 给所有的任务加个区域，和任务号
    for index, item in enumerate(resultTaskList):
        item["id"] = index + 1
        item["areaNumber"] = areaNumber
    return resultTaskList

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

if __name__ == '__main__':
    createTask()
