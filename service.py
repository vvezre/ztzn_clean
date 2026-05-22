# coding=utf-8
import json
import math

from layout_planner import create_return_to_origin_tasks, create_task_by_panel_layout, expand_panel_cells, panel_point_xy
import util
import redis
from AppLogger import logger

redis_cli = redis.Redis(host='localhost', port=6379, db=0, decode_responses=True)
# startLat = 32.03647704
# startLon = 118.92448868
# startHeading = 4


def _task_int(task, key, default=0):
    try:
        return int(round(float(task.get(key, default))))
    except (TypeError, ValueError):
        return default


def _renumber_tasks(tasks):
    for index, task in enumerate(tasks):
        task['id'] = index + 1


def _default_rtk_origin_from_tasks(tasks, default_y=0):
    clean_points = []
    for task in tasks or []:
        if int(task.get('mode', 0)) == 1:
            clean_points.append((_task_int(task, 'startX'), _task_int(task, 'startY')))
            clean_points.append((_task_int(task, 'endX'), _task_int(task, 'endY')))
    if clean_points:
        return min(clean_points, key=lambda point: (point[1], point[0]))
    return 0, _task_int({'value': default_y}, 'value')


def _prepare_tasks_for_rtk_origin(tasks, return_to_origin, turn_back_len, area_number, origin=(0, 0)):
    normalized = [dict(task) for task in tasks]
    origin = (_task_int({'value': origin[0] if origin else 0}, 'value'),
              _task_int({'value': origin[1] if origin else 0}, 'value'))
    _renumber_tasks(normalized)
    if return_to_origin and normalized:
        last_task = normalized[-1]
        last_point = (_task_int(last_task, 'endX'), _task_int(last_task, 'endY'))
        if last_point != origin:
            local_last_point = (last_point[0] - origin[0], last_point[1] - origin[1])
            return_tasks = create_return_to_origin_tasks(
                local_last_point,
                turnBackLen=turn_back_len,
                areaNumber=last_task.get('areaNumber', area_number),
                start_id=len(normalized) + 1
            )
            for task in return_tasks:
                task['startX'] = _task_int(task, 'startX') + origin[0]
                task['startY'] = _task_int(task, 'startY') + origin[1]
                task['endX'] = _task_int(task, 'endX') + origin[0]
                task['endY'] = _task_int(task, 'endY') + origin[1]
            normalized = normalized + return_tasks
    if origin != (0, 0):
        for item in normalized:
            item['startX'] = _task_int(item, 'startX') - origin[0]
            item['startY'] = _task_int(item, 'startY') - origin[1]
            item['endX'] = _task_int(item, 'endX') - origin[0]
            item['endY'] = _task_int(item, 'endY') - origin[1]
    _renumber_tasks(normalized)
    return normalized, origin


def _round_like_js(value):
    return int(math.floor(value + 0.5))


def _layout_anchor_cell(layout):
    layout = layout if isinstance(layout, dict) else {}
    anchor = layout.get('rtkAnchor') if isinstance(layout.get('rtkAnchor'), dict) else {}
    row_value = anchor.get('row', layout.get('rtkAnchorRow'))
    col_value = anchor.get('col', layout.get('rtkAnchorCol'))
    if row_value not in (None, '') and col_value not in (None, ''):
        return _task_int({'value': row_value}, 'value'), _task_int({'value': col_value}, 'value')

    base_cells = expand_panel_cells({
        'areas': layout.get('areas', []) or [],
        'holes': layout.get('holes', []) or []
    })
    if base_cells:
        return min(base_cells, key=lambda cell: (cell[0], cell[1]))

    cells = expand_panel_cells(layout)
    if not cells:
        return None
    return min(cells, key=lambda cell: (cell[0], cell[1]))


def _layout_rtk_origin(layout, panel_width=None, panel_height=None, gap_x=0, gap_y=0,
                       angle_radians_x=0, angle_radians_y=0):
    layout = layout if isinstance(layout, dict) else {}
    origin = layout.get('rtkOrigin') if isinstance(layout, dict) else None
    if isinstance(origin, dict):
        return _task_int(origin, 'x'), _task_int(origin, 'y')
    if 'rtkOriginX' not in layout and 'rtkOriginY' not in layout:
        configured = None
    else:
        configured = _task_int(layout, 'rtkOriginX'), _task_int(layout, 'rtkOriginY')
        if configured != (0, 0):
            return configured

    if panel_width is None or panel_height is None:
        return configured

    anchor_cell = _layout_anchor_cell(layout)
    if anchor_cell is None:
        return configured

    panel_width = _task_int({'value': panel_width}, 'value')
    panel_height = _task_int({'value': panel_height}, 'value')
    gap_x = _task_int({'value': gap_x}, 'value')
    gap_y = _task_int({'value': gap_y}, 'value')
    x_projection = math.cos(angle_radians_x or 0)
    y_projection = math.cos(angle_radians_y or 0)
    projected_panel_width = int(round(panel_width * x_projection))
    projected_panel_height = int(round(panel_height * y_projection))
    step_x = int(round((panel_width + gap_x) * x_projection))
    step_y = int(round((panel_height + gap_y) * y_projection))
    anchor_x, anchor_y = panel_point_xy(
        anchor_cell[0],
        anchor_cell[1],
        step_x,
        step_y,
        layout.get('connectors', []) or []
    )
    lane_ratio = float(layout.get('rtkAnchorLaneRatio', 0.25) or 0.25)
    return (
        _round_like_js(anchor_x + projected_panel_width * 0.5),
        _round_like_js(anchor_y + projected_panel_height * lane_ratio)
    )


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
        gapXValue = taskParams.get('gapX')
        gapYValue = taskParams.get('gapY')
        gapX = gap if gapXValue in (None, '', b'') else int(gapXValue)
        gapY = gap if gapYValue in (None, '', b'') else int(gapYValue)
        panelAngle = int(taskParams.get('panelAngle'))
        panelAngleX = int(taskParams.get('panelAngleX'))



        # 将度数转换为弧度
        angle_radians = math.radians(panelAngle)
        angle_radians_y = math.radians(panelAngle)
        angle_radians_x = math.radians(panelAngleX)
        # 默认表示y轴上有角度
        angle_to = 'y'
        if panelAngle == 0 and panelAngleX != 0:
            # 表示x轴上有角度
            angle_to = 'x'
            angle_radians = math.radians(panelAngleX)

        # 初始航向角
        startHeading = float(taskParams.get('heading'))
        startToChargingPilePointLength = _task_int(taskParams, 'startToChargingPilePointLength', 0)
        originLat = float(taskParams.get('startLat'))
        originLon = float(taskParams.get('startLon'))
        garageEntryLat = float(taskParams.get('garageEntryLat') or originLat)
        garageEntryLon = float(taskParams.get('garageEntryLon') or originLon)
        # 原点航向角
        originHeading = float(taskParams.get('originHeading'))
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
        rtkOriginOffset = (0, 0)
        for index,areaInfo in enumerate(areaList):
            areaInfoStr = json.dumps(areaInfo)
            redis_cli.lpush('areaList', areaInfoStr)

            direction = areaInfo.get('direction', 'left')
            areaNumber = int(areaInfo['areaNumber'])

            # 获取到任务列表
            # taskList = util.createTask(direction, info, areaNumber,goBackLen, goLeftOrRightBackLen,
            #                            turnBackLen, panelWidth, panelHeight, upOrDownBridgeLen, leftOrRightBridgeLen,
            #                            gap,lineCount,columnCount,angle_radians)
            # 判断当前区域是否是最后一个区域
            isLastArea = index == len(areaList)-1
            if int(areaInfo.get('layoutVersion', 1)) == 2:
                layout = areaInfo.get('layout', areaInfo)
                planStartX = startX
                planStartY = startY
                configuredRtkOrigin = None
                if not totalTaskList:
                    configuredRtkOrigin = _layout_rtk_origin(
                        layout,
                        panel_width=panelWidth,
                        panel_height=panelHeight,
                        gap_x=gapX,
                        gap_y=gapY,
                        angle_radians_x=angle_radians_x,
                        angle_radians_y=angle_radians_y,
                    )
                    if configuredRtkOrigin is not None:
                        rtkOriginOffset = configuredRtkOrigin
                        planStartX, planStartY = rtkOriginOffset
                taskList = create_task_by_panel_layout(layout,areaNumber,planStartX,planStartY,direction,isLastArea,0,goBackLen,goLeftOrRightBackLen,turnBackLen,
                                           panelWidth,panelHeight,leftOrRightBridgeLen,gap,angle_radians,angle_to,gapX,gapY,angle_radians_x,angle_radians_y)
                if not totalTaskList and configuredRtkOrigin is None:
                    rtkOriginOffset = _default_rtk_origin_from_tasks(taskList, planStartY)
                    if rtkOriginOffset != (planStartX, planStartY):
                        planStartX, planStartY = rtkOriginOffset
                        taskList = create_task_by_panel_layout(layout,areaNumber,planStartX,planStartY,direction,isLastArea,0,goBackLen,goLeftOrRightBackLen,turnBackLen,
                                           panelWidth,panelHeight,leftOrRightBridgeLen,gap,angle_radians,angle_to,gapX,gapY,angle_radians_x,angle_radians_y)
            else:
                panelInfoList = areaInfo['panelInfo']
                lineCount = areaInfo['lineCount']
                taskList = util.createTaskByPanelInfo(panelInfoList,areaNumber,startX,startY,direction,isLastArea,lineCount,goBackLen,goLeftOrRightBackLen,turnBackLen,
                                           panelWidth,panelHeight,leftOrRightBridgeLen,gap,angle_radians,angle_to,gapX,gapY,angle_radians_x,angle_radians_y)
            if not taskList:
                raise ValueError("任务区域没有可生成的路径")
            # 获取这个区域中最后一个任务，留给下个区域作为起始点
            task = taskList[-1]

            startX = task['endX']
            startY = task['endY']

            totalTaskList = totalTaskList + taskList
        returnToOrigin = True
        if len(areaList) > 0:
            lastAreaInfo = areaList[-1]
            if isinstance(lastAreaInfo, dict):
                lastLayout = lastAreaInfo.get('layout', {})
                returnToOrigin = lastAreaInfo.get('returnToOrigin', lastLayout.get('returnToOrigin', True))
        if isinstance(returnToOrigin, str):
            returnToOrigin = returnToOrigin.lower() not in ('false', '0', 'no')
        totalTaskList, rtkOriginOffset = _prepare_tasks_for_rtk_origin(
            totalTaskList,
            returnToOrigin,
            turnBackLen,
            areaLength,
            origin=rtkOriginOffset
        )
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
            'garageEntryLat': garageEntryLat, # 入舱点经纬度，不等同于充电桩
            'garageEntryLon': garageEntryLon,
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
            'lastTaskBackLength': _task_int(taskParams, 'lastTaskBackLength', 0),
            'panelAngleX':panelAngleX,
            'panelAngle':panelAngle,
            'gap':int(taskParams.get('gap')),
            'gapX':gapX,
            'gapY':gapY,
            'rtkOriginOffsetX':rtkOriginOffset[0],
            'rtkOriginOffsetY':rtkOriginOffset[1],
            'taskList': totalTaskList
        }
        # 将任务列表写入配置文件
        json_str = json.dumps(taskObj, indent=2)
        fileName = taskName + ".json"
        with open(fileName, 'w') as f:
            f.write(json_str)
        return {"success":True,"msg":"生成任务成功"}
    except Exception as e:
        logger.error(str(e))
        return {"success":False,"msg":str(e)}



