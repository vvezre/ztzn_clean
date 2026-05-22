import codecs

with codecs.open('main.py', 'r', encoding='utf-8') as f:
    content = f.read()

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

print("Applied Vision Steering Logic!")
