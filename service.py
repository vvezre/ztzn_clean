# coding=utf-8
import json

import util
import redis
from AppLogger import logger

redis_cli = redis.Redis(host='localhost', port=6379, db=0)

# 生成任务列表，direction：left表示从左侧开始运行，right表示从右侧开始运行
# info:每一个区域光伏板排列信息
def createTask(taskName,areaList):

    try:
        # 从redis中获取任务参数，用于构建任务列表
        taskParams = redis_cli.hgetall("taskParams")
        goBackLen = int(taskParams.get('goBackLen'))
        goLeftOrRightBackLen = int(taskParams.get('goLeftOrRightBackLen'))
        turnBackLen = int(taskParams.get('turnBackLen'))
        panelWidth = int(taskParams.get('panelWidth'))
        panelHeight = int(taskParams.get('panelHeight'))
        upOrDownBridgeLen = int(taskParams.get('upOrDownBridgeLen'))
        leftOrRightBridgeLen = int(taskParams.get('leftOrRightBridgeLen'))

        totalTaskList = []
        redis_cli.delete('areaList')

        for areaInfo in areaList:
            areaInfoStr = json.dumps(areaInfo)
            redis_cli.lpush('areaList', areaInfoStr)

            direction = areaInfo['direction']
            panelInfoList = areaInfo['panelInfo']
            areaNumber = areaInfo['areaNumber']
            reversed_list = panelInfoList[::-1]
            info = []
            for index, panel in enumerate(reversed_list):
                info.append(index + 1)
                if panel['value'] == '1':
                    info.append(0)
            # 获取到任务列表
            taskList = util.createTask(direction, info, areaNumber,goBackLen, goLeftOrRightBackLen,
                                       turnBackLen, panelWidth, panelHeight, upOrDownBridgeLen, leftOrRightBridgeLen)

            totalTaskList = totalTaskList + taskList
        # 去掉最后一个任务
        totalTaskList.pop()
        # 将任务列表写入配置文件
        json_str = json.dumps(totalTaskList, indent=2)
        fileName = taskName + ".json"
        with open(fileName, 'w') as f:
            f.write(json_str)

        return {"success":True,"msg":"生成任务成功"}
    except Exception as e:
        logger.error(e.message)
        return {"success":False,"msg":e.message}



