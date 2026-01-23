# coding=utf-8

import socket
import base64
import serial

from AppLogger import logger

COM = 'COM7'                   # RTK模块的串口号
BPS = 115200                    # RTK模块的串口波特率
NtripIP = '120.253.239.161'     # Ntrip的IP地址
NtripPort = 8001                # Ntrip的端口号
NtripPoint = 'RTCM33_GRCE'             # Ntrip的挂载点
NtripUser =  'csha42914'          # Ntrip的用户名
NtripPwd  = 'ac956382'              # Ntrip的密码


# ------------------------------------------------------------------
RTK = serial.Serial(COM, BPS, timeout=0.01)
print("等待RTK模块定位...")
while True:
    data = RTK.readline()
    if len(data):
        strNMEA = data.decode("ascii")
        seg = strNMEA.split(',')
        if seg[0] == "$GNGGA":
            if len(seg[6]) and seg[6]!='0':
                strGNGGA = strNMEA + "\r\n\r\n"
                print(strGNGGA)
                break

ntrip = socket.socket()
ntrip.connect((NtripIP,NtripPort))

user_pwd = base64.b64encode(bytes(NtripUser+':'+NtripPwd)).decode("utf-8")
httpHead = "GET /"+NtripPoint+" HTTP/1.0\r\nUser-Agent: NTRIP GNSSInternetRadio/1.4.10\r\nAccept: */*\r\nConnection: close\r\nAuthorization: Basic "+user_pwd+"\r\n\r\n"
ntrip.send(httpHead.encode())
data = ntrip.recv(1024)
print(data)
ntrip.send(strGNGGA.encode())

while True:
    data = RTK.read(102400)
    if len(data):
        print(data.decode("ascii"))
    data = ntrip.recv(102400)
    RTK.write(data)
exit()
