# coding=utf-8
import binascii
import time

import serial
import serial.tools.list_ports


if __name__ == '__main__':
    print([port.device for port in serial.tools.list_ports.comports()])
    # 打开端口
    ser = serial.Serial('COM7', 9600, timeout=1)

    if ser.is_open:
        print '已经打开端口'
    else:
        print '端口打开失败'

    # 发送16进制数据
    hex_data = "01 04 00 00 00 01 31 CA"  # 16进制字符串（支持空格分隔）
    command = hex_data.replace(' ', '').decode('hex')
    ser.write(command)  # 发送数据

    # 等待并接收响应
    time.sleep(0.1)
    data = ser.read(7)
    print int(binascii.b2a_hex(data[4]), 16)
    
    # 关闭端口
    ser.close()