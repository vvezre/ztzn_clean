# coding=utf-8
import json
import math

import util
import redis
from AppLogger import logger

redis_cli = redis.Redis(host='localhost', port=6379, db=0)
# startLat = 32.03647704
# startLon = 118.92448868
# startHeading = 4

# 生成任务列表，direction：left表示从左侧开始运行，right表示从右侧开始运行
# info:每一个区域光伏板排列信息
def createTask(taskName,areaList):
    global startLat, startLon, startHeading
    try:
        # 从redis中获取任务参数，用于构建任务列表
        taskParams = redis_cli.hgetall("taskParams")
        goBackLen = int(taskParams.get('goBackLen'))
        goLeftOrRightBackLen = int(taskParams.get('goLeftOrRightBackLen'))
        turnBackLen = int(taskParams.get('turnBackLen'))
        panelWidth = int(taskParams.get('panelWidth'))
        panelHeight = int(taskParams.get('panelHeight'))
        leftOrRightBridgeLen = int(taskParams.get('leftOrRightBridgeLen'))
        gap = int(taskParams.get('gap'))
        panelAngle = int(taskParams.get('panelAngle'))
        panelAngleX = int(taskParams.get('panelAngleX'))



        # 将度数转换为弧度
        angle_radians = math.radians(panelAngle)
        # 默认表示y轴上有角度
        angle_to = 'y'
        if panelAngle == 0 and panelAngleX != 0:
            # 表示x轴上有角度
            angle_to = 'x'
            angle_radians = math.radians(panelAngleX)

        # 初始航向角
        startHeading = float(taskParams.get('heading'))
        startToChargingPilePointLength = int(taskParams.get('startToChargingPilePointLength'))
        originLat = float(taskParams.get('startLat'))
        originLon = float(taskParams.get('startLon'))
        # 原点航向角
        originHeading = int(taskParams.get('originHeading'))
        # 充电桩经纬度
        chargingPileLat = float(taskParams.get('chargingPileLat'))
        chargingPileLon = float(taskParams.get('chargingPileLon'))
        startLat = originLat
        startLon = originLon

        totalTaskList = []
        redis_cli.delete('areaList')
        # 获取当前有多少个区域
        areaLength = len(areaList)
        startX = 0
        startY = 0
        for index,areaInfo in enumerate(areaList):
            areaInfoStr = json.dumps(areaInfo)
            redis_cli.lpush('areaList', areaInfoStr)

            direction = areaInfo['direction']
            panelInfoList = areaInfo['panelInfo']
            areaNumber = int(areaInfo['areaNumber'])

            lineCount = areaInfo['lineCount']

            # 获取到任务列表
            # taskList = util.createTask(direction, info, areaNumber,goBackLen, goLeftOrRightBackLen,
            #                            turnBackLen, panelWidth, panelHeight, upOrDownBridgeLen, leftOrRightBridgeLen,
            #                            gap,lineCount,columnCount,angle_radians)
            # 判断当前区域是否是最后一个区域
            isLastArea = index == len(areaList)-1
            taskList = util.createTaskByPanelInfo(panelInfoList,areaNumber,startX,startY,direction,isLastArea,lineCount,goBackLen,goLeftOrRightBackLen,turnBackLen,
                                       panelWidth,panelHeight,leftOrRightBridgeLen,gap,angle_radians,angle_to)
            # 获取这个区域中最后一个任务，留给下个区域作为起始点
            task = taskList[-1]

            startX = task['endX']
            startY = task['endY']

            totalTaskList = totalTaskList + taskList
        # 最后一个任务，endX和endY必须是0
        totalTaskList[-1]['endX'] = 0
        totalTaskList[-1]['endY'] = 0
        # 添加每个任务起始点结束点经纬度
        for index,item in enumerate(totalTaskList):
            item['heading'] = (startHeading + item['angle'])%360
            endLat,endLon = util.local_rotated_xy_to_latlon_precise(originLat,originLon,item['endX']/100.0,item['endY']/100.0,(startHeading-90)%360)
            # endLat,endLon = util.local_rotated_xy_to_latlon_precise(originLat,originLon,item['endX']/100.0,item['endY']/100.0,startHeading)
            item['startLat'] = startLat
            item['startLon'] = startLon
            item['endLat'] = endLat
            item['endLon'] = endLon
            startLat = endLat
            startLon = endLon
        taskObj = {
            'taskName': taskName,
            'startLat': originLat,         # 起始点
            'startLon': originLon,
            'goBackLen':int(taskParams.get('goBackLen')),
            'goLeftOrRightBackLen': int(taskParams.get('goLeftOrRightBackLen')),
            'turnBackLen':int(taskParams.get('turnBackLen')),
            'panelWidth':int(taskParams.get('panelWidth')),
            'panelHeight':int(taskParams.get('panelHeight')),
            'leftOrRightBridgeLen':int(taskParams.get('leftOrRightBridgeLen')),
            'voltageWarn':int(taskParams.get('voltageWarn')),
            'originHeading':originHeading,  # 起始点航向角，用于原点转正
            'heading': startHeading,        # 初始航向角，用于rtk纠偏
            'chargingPileLat':chargingPileLat,            # 充电桩经纬度
            'chargingPileLon':chargingPileLon,
            'startToChargingPilePointLength': startToChargingPilePointLength,
            'panelAngleX':panelAngleX,
            'panelAngle':panelAngle,
            'gap':int(taskParams.get('gap')),
            'taskList': totalTaskList
        }
        # 将任务列表写入配置文件
        json_str = json.dumps(taskObj, indent=2)
        fileName = taskName + ".json"
        with open(fileName, 'w') as f:
            f.write(json_str)
        return {"success":True,"msg":"生成任务成功"}
    except Exception as e:
        logger.error(e.message)
        return {"success":False,"msg":e.message}



