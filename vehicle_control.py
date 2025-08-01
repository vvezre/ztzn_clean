# coding=utf-8
import binascii
import time

import redis
import serial

from AppLogger import logger

command = bytearray(17)
CMD_LEN = 23
redis_cli = redis.Redis(host='localhost', port=6379, db=0)

def writeCmd(ser, command):
    if ser.is_open:
        pass
    else:
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.5)
    ser.write(command)
def duplicateWriteCmd(ser, command):
    for i in range(5):
        writeCmd(ser, command)
    ser.flushOutput()
def tem_listener(data, data_length):  # 数组 数组长度
    output = 0
    for num in range(0, data_length + 1):
        output = output ^ data[num]
    return output
# 设置主功能
# 0x00：刹车
# 0x01：仅设置速度模式
# 0x02：设置速度及目标距离
# 0x03：设置旋转速度及旋转方向
def setStatus(status):
    command[1] = status
# 使能位
# 0x00：失能
# 0x01：使能
def setPowerOn(status):
    command[2] = status
def reSetStatus(ser):
    setStatus(0)
    # 15位是校验位
    command[15] = tem_listener(command, 15)
    duplicateWriteCmd(ser, command)
    time.sleep(0.5)
# 硬件功能控制
def setHWstatus(use, sth, air, gate, duo):
    hex_number = ""
    if use == 1:
        hex_number = "f"
    elif use == 0:
        hex_number == "0"
    otherControl = str(sth) + str(air) + str(gate) + str(duo)
    hex_otherControl = hex(int(otherControl, 2))[2:]
    hex_number = hex_number + hex_otherControl
    command[3] = int(hex_number, 16)
# 设置速度
def setXSpeed(status):  # 速度待定
    if status > 32767:
        status = 32767
    if status < -32768:
        status = -32768
    if int(status) < 0:
        binary_num = bin(int(status) & 0xffff)  # 将负数转换为二进制
        hex_num = hex(int(binary_num, 2))[2:]  # 将二进制转换为十六进制
        hex_1 = hex_num[0] + hex_num[1]
        command[4] = int(hex_1, 16)
        hex_2 = hex_num[2] + hex_num[3]
        command[5] = int(hex_2, 16)
    elif int(status) <= 255:
        hex_string = hex(int(status))[2:]
        hex_1 = "00"
        command[4] = int(hex_1, 16)
        hex_2 = hex_string
        command[5] = int(hex_2, 16)
    elif int(status) <= 4095:
        hex_string = hex(int(status))[2:]
        hex_1 = "0" + hex_string[0]
        command[4] = int(hex_1, 16)
        hex_2 = hex_string[1] + hex_string[2]
        command[5] = int(hex_2, 16)
    elif int(status) <= 65535:
        hex_string = hex(int(status))[2:]
        hex_1 = hex_string[0] + hex_string[1]
        command[4] = int(hex_1, 16)
        hex_2 = hex_string[2] + hex_string[3]
        command[5] = int(hex_2, 16)

def setDistance(status):
    if status > 50000:  # 设置范围为0-50000cm
        hex_string = "FF"
        command[8] = int(hex_string, 16)
        command[9] = int(hex_string, 16)
    elif int(status) <= 255:
        hex_string = hex(int(status))[2:]
        hex_1 = "00"
        command[8] = int(hex_1, 16)
        hex_2 = hex_string
        command[9] = int(hex_2, 16)
    elif int(status) <= 4095:
        hex_string = hex(int(status))[2:]
        hex_1 = "0" + hex_string[0]
        command[8] = int(hex_1, 16)
        hex_2 = hex_string[1] + hex_string[2]
        command[9] = int(hex_2, 16)
    elif int(status) <= 32767:
        hex_string = hex(int(status))[2:]
        hex_1 = hex_string[0] + hex_string[1]
        command[8] = int(hex_1, 16)
        hex_2 = hex_string[2] + hex_string[3]
        command[9] = int(hex_2, 16)

# 解析下位机传输过来的数据
def globalDataSet(data):
    i = 0
    global global_get_status
    global global_get_powerOn
    global global_get_HWstatus
    global global_get_XSpeed
    global global_get_ZSpeed
    global global_get_brushSpeed
    global global_get_edge
    global global_get_voltage
    global global_get_air
    global global_get_moveFinish

    for ch in data:
        if i == 1:
            global_get_status = int(binascii.b2a_hex(data[i]), 16)
        elif i == 2:
            global_get_powerOn = int(binascii.b2a_hex(data[i]), 16)
        elif i == 3:
            global_get_HWstatus = int(binascii.b2a_hex(data[i]), 16)
        elif i == 4:
            global_get_XSpeed = int(binascii.b2a_hex(data[i] + data[i + 1]), 16)
        elif i == 6:
            global_get_ZSpeed = int(binascii.b2a_hex(data[i] + data[i + 1]), 16)
        elif i == 8:
            global_get_brushSpeed = int(binascii.b2a_hex(data[i]), 16)
        elif i == 9:
            if binascii.b2a_hex(data[i]) == '00':
                global_get_edge = 1
            elif binascii.b2a_hex(data[i]) == 'ff':
                logger.info("返回到边指令")
                redis_cli.set("ultraSonic", "true")
                global_get_edge = 0
            else:
                global_get_edge = 0
        elif i == 10:
            global_get_voltage = int(binascii.b2a_hex(data[i]), 16)
        elif i == 11:
            global_get_air = int(binascii.b2a_hex(data[i]), 16)
        elif i == 12:
            if binascii.b2a_hex(data[i]) == '00':
                global_get_moveFinish = 0
            elif binascii.b2a_hex(data[i]) == 'bb':
                global_get_moveFinish = 1
            else:
                global_get_moveFinish = 0

        elif i == 13:
            global global_get_rotateFinish
            print(binascii.b2a_hex(data[i]))
            if binascii.b2a_hex(data[i]) == '00':
                global_get_rotateFinish = 0
            elif binascii.b2a_hex(data[i]) == 'bb':
                global_get_rotateFinish = 1
            else:
                global_get_rotateFinish = 0
        i = i + 1
    info = binascii.b2a_hex(data[11])
    logger.info(int(info,16))
    str1 = binascii.b2a_hex(data[14])
    str2 = binascii.b2a_hex(data[15])
    voltage = int(str1 + str2, 16)
    if voltage > 0:
        logger.info('获取到电压值: %d', voltage)
        roundVoltage = round(voltage * 0.01, 1)
        roundVoltage = round((roundVoltage-23)/(28-23))*100
        # redis_cli.set('voltage', roundVoltage)

    str1 = binascii.b2a_hex(data[16])
    str2 = binascii.b2a_hex(data[17])
    logger.warn("======================================={},{}".format(str1,str2))
    angle = int(str1 + str2, 16)
    redis_cli.set("angle", angle)

    str1 = binascii.b2a_hex(data[18])
    str2 = binascii.b2a_hex(data[19])
    odometer = int(str1 + str2, 16)

    redis_cli.set("odometer", odometer)

# 获取是否到边，1：表示在板子上，0：表示不在板子上
def getEdge():
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.5)
    if ser.is_open:
        redis_cli.set('moveJudge', 'true')
        while redis_cli.get("mission") == "working":
            try:
                data = ser.read(CMD_LEN * 2)
                hex_data = binascii.b2a_hex(data).decode('utf-8')
                logger.warn(hex_data)
                # 数据长度不够不要
                if len(data) < CMD_LEN:
                    print("data not full")
                    hex_data = binascii.b2a_hex(data).decode('utf-8')
                    print(hex_data)
                    continue
                i = 0
                for q in data:
                    if binascii.b2a_hex(q) == '7b':
                        break
                    else:
                        i = i + 1
                        continue
                data = data[i:]
                globalDataSet(data)
                break
            except serial.serialutil.SerialException:
                try:
                    ser.close()
                    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.5)
                except serial.serialutil.SerialException:
                    print("fail open COM")
                    global_status = "fail open COM"
                    time.sleep(0.5)
        redis_cli.set('moveJudge', 'false')
    else:
        global_status = "fail open COM"
        print("fail open COM")
    return str(global_get_edge)

# 获取是否到达距离，0：未到达；1：到达
def getDistanceArrive():
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.5)
    if ser.is_open:
        redis_cli.set('moveJudge', 'true')
        while redis_cli.get("mission") == "working":
            try:
                data = ser.read(CMD_LEN * 2)
                if len(data) < CMD_LEN:
                    print("data not full")
                    continue
                i = 0
                for q in data:
                    if binascii.b2a_hex(q) == '7b':
                        break
                    else:
                        i = i + 1
                        continue
                data = data[i:]
                globalDataSet(data)
                break
            except serial.serialutil.SerialException:
                try:
                    ser.close()
                    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.5)
                except serial.serialutil.SerialException:
                    print("fail open COM")
                    time.sleep(0.5)
        redis_cli.set('moveJudge', 'false')

    else:
        print("fail open COM")
    return str(global_get_moveFinish)

# 小车走固定距离
def goByLength(ser,length,speed=100):
    reSetStatus(ser)
    setStatus(2)
    setPowerOn(1)
    setHWstatus(0, 0, 0, 0, 0)
    setXSpeed(speed)
    setDistance(length)
    command[15] = tem_listener(command, 15)
    duplicateWriteCmd(ser, command)
    # 这里开始判断路径和到边,1能走，0不行
    while getEdge() == "1" and getDistanceArrive() == "0":
        if redis_cli.get("mission") == "complete":
            return 0
        print("keep walking")
        time.sleep(0.25)
    time.sleep(0.5)

    redis_cli.set("mission", "complete")
    if getDistanceArrive() == "1":
        print("到达")
    else:
        print("未到达，结束")