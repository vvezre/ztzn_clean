import codecs

with codecs.open('main.py', 'r', encoding='utf-8') as f:
    content = f.read()

# 1. State Inference Injection
old_status_logic = '''    status = byte_at(1)
    if status is not None:
        global_get_status = status'''

new_status_logic = '''    status = byte_at(1)
    if status is not None:
        global_get_status = status
        
        # 终极自愈推断逻辑: 如果当前状态是 unknown
        current_g_state = get_garage_state()
        if current_g_state.get('state') == GARAGE_STATE_UNKNOWN:
            # 优先级 1: 硬件明确在充电
            if status == 5:
                logger.warn("嗅探到下位机硬件处于充电状态(5)，自动恢复 garageState 为 docked_by_command")
                set_garage_state(GARAGE_STATE_DOCKED_BY_COMMAND, 'auto_recovered_by_hardware_status')
            else:
                # 获取环境推断参数
                try:
                    vol = int(redis_cli.get('voltage') or 0)
                    # 优先级 2: 高电量且无RTK信号
                    if vol > 80 and global_cur_rtk_lat is None:
                        logger.warn("嗅探到高电量({}%)且无RTK信号，环境推断为光伏板下，自动恢复为 docked_by_command".format(vol))
                        set_garage_state(GARAGE_STATE_DOCKED_BY_COMMAND, 'auto_recovered_by_battery_and_signal')
                    # 优先级 3: 有RTK且距离入舱点较远(>2米)
                    elif global_cur_rtk_lat is not None:
                        tp = redis_cli.hgetall("taskParams")
                        if tp and tp.get('garageEntryLat') and tp.get('garageEntryLon'):
                            e_lat = float(tp.get('garageEntryLat'))
                            e_lon = float(tp.get('garageEntryLon'))
                            dist, _ = util.get_distance_angle(global_cur_rtk_lat, global_cur_rtk_lon, e_lat, e_lon)
                            if dist > 2.0:
                                logger.warn("嗅探到RTK有固定解，且距离入舱点({:.2f}米)大于2米，推断为舱外 outside".format(dist))
                                set_garage_state(GARAGE_STATE_OUTSIDE, 'auto_recovered_by_rtk_distance')
                except Exception as e:
                    logger.error("自动状态推断时发生异常: {}".format(e))'''

content = content.replace(old_status_logic, new_status_logic)

# 2. Vision PD Control Injection
old_vision_logic = '''    lower1 = np.array([9, 250, 250])
    upper1 = np.array([10, 255, 255])
    lower2 = np.array([170, 140, 140])
    upper2 = np.array([180, 255, 255])
    while True:'''

new_vision_logic = '''    lower1 = np.array([9, 250, 250])
    upper1 = np.array([10, 255, 255])
    lower2 = np.array([170, 140, 140])
    upper2 = np.array([180, 255, 255])
    
    last_valid_offset = 0
    last_valid_angle = 0
    missing_line_frames = 0
    
    while True:'''

content = content.replace(old_vision_logic, new_vision_logic)

old_vision_steering = '''            if line is None:
                logger.info("没有找到线")
                line = 0
            if line == 0:
                logger.info('=====================>位置偏移为0!, 角度: %d', line_angle)
            else:
                logger.info('位置偏移: %d, 角度: %d', line, line_angle)
                # 存储角度
                redis_cli.set("angle", int(line_angle))
            # 小车直行时，角度大于10度，就操作下面逻辑
            if line_angle > 10:
                line = -line_angle * 2
            setZSpeed(line)
            duplicateWriteCmd(ser, command)'''

new_vision_steering = '''            if line is None:
                missing_line_frames += 1
                logger.info("没有找到线, 连续丢失帧数: {}".format(missing_line_frames))
                # 惯性衰减逻辑 (Anti-Loss)
                line = last_valid_offset * (0.8 ** missing_line_frames)
                line_angle = last_valid_angle * (0.8 ** missing_line_frames)
                if missing_line_frames > 15: # 丢失太久则彻底归零
                    line = 0
                    line_angle = 0
            else:
                missing_line_frames = 0
                last_valid_offset = line
                last_valid_angle = line_angle
                logger.info('位置偏移: %d, 角度: %d', line, line_angle)
                redis_cli.set("angle", int(line_angle))

            # 平滑控制转向 (PD Control Approximation)
            z_speed = line * 0.5
            if abs(line_angle) > 5:
                z_speed = -line_angle * 1.5

            # 最大极限转向保护 (Clamp)
            z_speed = max(-80, min(80, int(z_speed)))
            setZSpeed(z_speed)
            duplicateWriteCmd(ser, command)'''

content = content.replace(old_vision_steering, new_vision_steering)

with codecs.open('main.py', 'w', encoding='utf-8') as f:
    f.write(content)

print("Applied Phase 1 and 2 Logic!")
