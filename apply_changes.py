import codecs

with codecs.open('main.py', 'r', encoding='utf-8') as f:
    content = f.read()

# 1. Inject `await_rtk_fix` and `execute_vehicle_exit`, and replace `exit_garage_task`
old_exit = '''def exit_garage_task():
    set_current_action('exit_garage')
    set_garage_state(GARAGE_STATE_EXITING, 'manual_exit_garage_started')
    redis_cli.set("reverse", "false")

    redis_cli.set("correct", "true")

    redis_cli.set('action', 'true')

    redis_cli.set('mission', 'working')

    exit_uav(ser)

    justMove(ser)

    reSetStatus(ser)

    moveBack(ser)

    turn(ser, 180 * 10)

    moveDiatance(ser, 110)

    moveBack(ser)

    turn(ser, 0)

    path = redis_cli.get(PATH_PLANNING_KEY)

    if LEFT_PATH_PLANNING == path:

        redis_cli.set(PATH_PLANNING_KEY, RIGHT_PATH_PLANNING)

    else:

        redis_cli.set(PATH_PLANNING_KEY, LEFT_PATH_PLANNING)

    redis_cli.set('action', 'false')

    redis_cli.set("correct", "false")

    redis_cli.set('mission', 'complete')
    set_garage_state(GARAGE_STATE_OUTSIDE, 'manual_exit_garage_completed')
    set_current_action('idle')'''

new_exit = '''def await_rtk_fix(timeout=10.0):
    import time
    start_time = time.time()
    logger.warn("等待 RTK 信号恢复...")
    time.sleep(3.0)
    while time.time() - start_time < timeout:
        if global_cur_rtk_lat is not None and global_cur_rtk_lon is not None:
            logger.warn("RTK 信号已满足要求")
            return True
        time.sleep(0.5)
    logger.warn("RTK 搜星等待超时")
    return False

def execute_vehicle_exit(back_length, is_manual=False):
    global global_status
    logger.warn("执行统一出舱流程: back_length={}, is_manual={}".format(back_length, is_manual))
    
    set_garage_state(GARAGE_STATE_EXITING, 'manual_exit_garage_started' if is_manual else 'auto_drive_exit_garage_started')
    
    redis_cli.set("reverse", "false")
    redis_cli.set("correct", "true")
    redis_cli.set('action', 'true')
    redis_cli.set("mission", "working")

    exit_uav(ser)
    reset_odometer(ser)
    
    dist = back_length if (back_length and float(back_length) > 0) else 33
    global_status = 'move back'
    moveBack(ser, float(dist))
    if str(getDistanceArrive()) == "0":
        set_garage_state(GARAGE_STATE_UNKNOWN, 'exit_garage_not_arrived')
        return False
        
    await_rtk_fix(timeout=15.0)
    set_garage_state(GARAGE_STATE_OUTSIDE, 'manual_exit_garage_completed' if is_manual else 'auto_drive_exit_garage_completed')
    set_current_action('idle')
    return True

def exit_garage_task():
    set_current_action('exit_garage')
    taskParams = redis_cli.hgetall("taskParams")
    backLength = int(float(taskParams.get('startToChargingPilePointLength') or 0))
    execute_vehicle_exit(backLength, is_manual=True)'''

content = content.replace(old_exit, new_exit)

# 2. Replace `goOutGarage`
old_goout = '''def goOutGarage(backLength):
    global global_status
    set_garage_state(GARAGE_STATE_EXITING, 'auto_drive_exit_garage_started')
    global_status = 'move back'
    reset_odometer(ser)
    redis_cli.set("mission", "working")
    moveBack(ser, backLength)
    # 如果后退没有到达，则不往下执行
    if str(getDistanceArrive()) == "0":
        set_garage_state(GARAGE_STATE_UNKNOWN, 'auto_drive_exit_garage_not_arrived')
        return
    set_garage_state(GARAGE_STATE_OUTSIDE, 'auto_drive_exit_garage_completed')'''

new_goout = '''def goOutGarage(backLength):
    execute_vehicle_exit(backLength, is_manual=False)'''

content = content.replace(old_goout, new_goout)

# 3. Add timeout to `moveBack`
old_moveback = '''    while getDistanceArrive() == "0" and redis_cli.get("mission") == "working":
        print("wait Distance finish")
        # global_status = "wait Distance finish"'''

new_moveback = '''    import time
    start_wait = time.time()
    timeout_sec = max(10, (distance / 10.0) + 10)
    while getDistanceArrive() == "0" and redis_cli.get("mission") == "working":
        if time.time() - start_wait > timeout_sec:
            logger.warn("moveBack 超时脱困! 设定距离: {}cm, 已等待: {:.1f}s".format(distance, time.time() - start_wait))
            break
        print("wait Distance finish")
        time.sleep(0.3)'''

content = content.replace(old_moveback, new_moveback)

with codecs.open('main.py', 'w', encoding='utf-8') as f:
    f.write(content)

print("Applied Phase 1 and Phase 2 changes successfully.")
