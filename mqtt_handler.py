# coding=utf-8
from AppLogger import logger


class MQTTCommandHandler:
    def __init__(self,vehicle_controller):
        """
        初始化命令处理器

        Args:
            vehicle_controller: 车辆控制器对象（需要提供车辆控制方法）
        """
        self.vehicle_controller = vehicle_controller
        self.command_map = self._init_command_map()
        self.command_map.update({
            'turnLeft': self._handle_turn_left,
            'turnRight': self._handle_turn_right,
            'joystickMove': self._handle_joystick_move,
            'autoDrive': self._handle_auto_drive,
            'goOn': self._handle_go_on,
            'returnToPoint': self._handle_return_to_point,
            'enterGarage': self._handle_enter_garage,
            'exitGarage': self._handle_exit_garage,
            'adjustSpeed': self._handle_adjust_speed,
            'adjustBrushSpeed': self._handle_adjust_brush_speed,
            'toggleTracking': self._handle_toggle_tracking,
            'togglePathPlanning': self._handle_toggle_path_planning,
            'createTask': self._handle_create_task,
            'selectTask': self._handle_select_task,
            'saveTask': self._handle_save_task,
            'saveParams': self._handle_save_params,
            'getStatus': self._handle_get_status,
        })
        logger.info("MQTT命令处理器初始化完成")

    def _init_command_map(self):
        """
        初始化命令映射表

        Returns:
            dict: 命令名称到处理函数的映射
        """
        return {
            # 基础运动控制
            'drive': self._handle_drive,
            'back': self._handle_back,
            'turn_left': self._handle_turn_left,
            'turn_right': self._handle_turn_right,
            'stop': self._handle_stop,
            'parking': self._handle_parking,

            # 摇杆控制
            'joystick_move': self._handle_joystick_move,

            # 高级功能
            'auto_drive': self._handle_auto_drive,
            'go_on': self._handle_go_on,
            'return_to_point': self._handle_return_to_point,
            'enter_garage': self._handle_enter_garage,
            'exit_garage': self._handle_exit_garage,

            # 参数调整
            'adjust_speed': self._handle_adjust_speed,
            'adjust_brush_speed': self._handle_adjust_brush_speed,
            'toggle_tracking': self._handle_toggle_tracking,
            'toggle_path_planning': self._handle_toggle_path_planning,

            # 任务管理
            'create_task': self._handle_create_task,
            'select_task': self._handle_select_task,
            'save_task': self._handle_save_task,

            # 参数配置
            'save_params': self._handle_save_params,
            'get_status': self._handle_get_status,
        }
    def handle(self,message_data):
        """
                处理接收到的MQTT消息
                Args:
                    message_data: 消息数据字典，格式：
                        {
                            'command': '命令名称',
                            'params': {...}  # 可选参数
                        }

                Returns:
                    dict: 处理结果
                """
        try:
            # 提取命令和参数
            command = message_data.get('command')
            params = message_data.get('params')
            if params is None:
                params = message_data.get('parameters', {})

            if not command:
                logger.warning("消息缺少command字段: {}".format(message_data))
                return {'success': False, 'message': '消息格式错误：缺少command字段'}

            logger.info("处理MQTT命令: {}, 参数: {}".format(command, params))

            # 查找并执行命令处理函数
            handler = self.command_map.get(command)
            if handler:
                result = handler(params)
                logger.info("命令执行结果: {}".format(result))
                return result
            else:
                logger.warning("未知命令: {}".format(command))
                return {'success': False, 'message': '未知命令: {}'.format(command)}

        except Exception as e:
            logger.error("处理MQTT命令异常: {}".format(e), exc_info=True)
            return {'success': False, 'message': '命令处理异常: {}'.format(e)}

    def _handle_drive(self, params):
        """处理前进命令"""
        try:
            distance = params.get('distance', 0)
            speed = params.get('speed', None)

            if hasattr(self.vehicle_controller, 'drive'):
                self.vehicle_controller.drive(distance, speed)
                return {'success': True, 'message': '前进命令已执行'}
            else:
                return {'success': False, 'message': '车辆控制器不支持drive方法'}
        except Exception as e:
            return {'success': False, 'message': str(e)}

    def _handle_back(self, params):
        """处理后退命令"""
        try:
            distance = params.get('distance', 0)
            speed = params.get('speed', None)

            if hasattr(self.vehicle_controller, 'back'):
                self.vehicle_controller.back(distance, speed)
                return {'success': True, 'message': '后退命令已执行'}
            else:
                return {'success': False, 'message': '车辆控制器不支持back方法'}
        except Exception as e:
            return {'success': False, 'message': str(e)}

    def _handle_turn_left(self, params):
        """处理左转命令"""
        try:
            angle = params.get('angle', 90)

            if hasattr(self.vehicle_controller, 'turn_left'):
                self.vehicle_controller.turn_left(angle)
                return {'success': True, 'message': '左转命令已执行'}
            else:
                return {'success': False, 'message': '车辆控制器不支持turn_left方法'}
        except Exception as e:
            return {'success': False, 'message': str(e)}

    def _handle_turn_right(self, params):
        """处理右转命令"""
        try:
            angle = params.get('angle', 90)

            if hasattr(self.vehicle_controller, 'turn_right'):
                self.vehicle_controller.turn_right(angle)
                return {'success': True, 'message': '右转命令已执行'}
            else:
                return {'success': False, 'message': '车辆控制器不支持turn_right方法'}
        except Exception as e:
            return {'success': False, 'message': str(e)}

    def _handle_stop(self, params):
        """处理停止命令"""
        try:
            if hasattr(self.vehicle_controller, 'stop'):
                self.vehicle_controller.stop()
                return {'success': True, 'message': '停止命令已执行'}
            else:
                return {'success': False, 'message': '车辆控制器不支持stop方法'}
        except Exception as e:
            return {'success': False, 'message': str(e)}

    def _handle_parking(self, params):
        """处理急停命令"""
        try:
            if hasattr(self.vehicle_controller, 'parking'):
                self.vehicle_controller.parking()
                return {'success': True, 'message': '急停命令已执行'}
            else:
                return {'success': False, 'message': '车辆控制器不支持parking方法'}
        except Exception as e:
            return {'success': False, 'message': str(e)}

    # ==================== 摇杆控制命令处理 ====================

    def _handle_joystick_move(self, params):
        """处理摇杆移动命令"""
        try:
            distance = params.get('distance', 50)
            dir_x = params.get('dirX', 0)
            dir_y = params.get('dirY', 0)

            if hasattr(self.vehicle_controller, 'joystick_move'):
                self.vehicle_controller.joystick_move(distance, dir_x, dir_y)
                return {'success': True, 'message': '摇杆控制命令已执行'}
            else:
                return {'success': False, 'message': '车辆控制器不支持joystick_move方法'}
        except Exception as e:
            return {'success': False, 'message': str(e)}

    # ==================== 高级功能命令处理 ====================

    def _handle_auto_drive(self, params):
        """处理自动清扫命令"""
        try:
            if hasattr(self.vehicle_controller, 'auto_drive'):
                self.vehicle_controller.auto_drive()
                return {'success': True, 'message': '自动清扫已启动'}
            else:
                return {'success': False, 'message': '车辆控制器不支持auto_drive方法'}
        except Exception as e:
            return {'success': False, 'message': str(e)}

    def _handle_go_on(self, params):
        """处理继续清扫命令"""
        try:
            if hasattr(self.vehicle_controller, 'go_on'):
                self.vehicle_controller.go_on()
                return {'success': True, 'message': '继续清扫命令已执行'}
            else:
                return {'success': False, 'message': '车辆控制器不支持go_on方法'}
        except Exception as e:
            return {'success': False, 'message': str(e)}

    def _handle_return_to_point(self, params):
        """处理返回原点命令"""
        try:
            if hasattr(self.vehicle_controller, 'return_to_point'):
                self.vehicle_controller.return_to_point()
                return {'success': True, 'message': '返回原点命令已执行'}
            else:
                return {'success': False, 'message': '车辆控制器不支持return_to_point方法'}
        except Exception as e:
            return {'success': False, 'message': str(e)}

    def _handle_enter_garage(self, params):
        """处理入库命令"""
        try:
            if hasattr(self.vehicle_controller, 'enter_garage'):
                self.vehicle_controller.enter_garage()
                return {'success': True, 'message': '入库命令已执行'}
            else:
                return {'success': False, 'message': '车辆控制器不支持enter_garage方法'}
        except Exception as e:
            return {'success': False, 'message': str(e)}

    def _handle_exit_garage(self, params):
        """处理出库命令"""
        try:
            if hasattr(self.vehicle_controller, 'exit_garage'):
                self.vehicle_controller.exit_garage()
                return {'success': True, 'message': '出库命令已执行'}
            else:
                return {'success': False, 'message': '车辆控制器不支持exit_garage方法'}
        except Exception as e:
            return {'success': False, 'message': str(e)}

    # ==================== 参数调整命令处理 ====================

    def _handle_adjust_speed(self, params):
        """处理调整速度命令"""
        try:
            speed = params.get('speed', 50)

            if hasattr(self.vehicle_controller, 'adjust_speed'):
                self.vehicle_controller.adjust_speed(speed)
                return {'success': True, 'message': '速度已调整为: {}'.format(speed)}
            else:
                return {'success': False, 'message': '车辆控制器不支持adjust_speed方法'}
        except Exception as e:
            return {'success': False, 'message': str(e)}

    def _handle_adjust_brush_speed(self, params):
        """处理调整滚刷速度命令"""
        try:
            speed = params.get('speed', 50)

            if hasattr(self.vehicle_controller, 'adjust_brush_speed'):
                self.vehicle_controller.adjust_brush_speed(speed)
                return {'success': True, 'message': '滚刷速度已调整为: {}'.format(speed)}
            else:
                return {'success': False, 'message': '车辆控制器不支持adjust_brush_speed方法'}
        except Exception as e:
            return {'success': False, 'message': str(e)}

    def _handle_toggle_tracking(self, params):
        """处理切换纠偏功能命令"""
        try:
            tracking = params.get('tracking', True)

            if hasattr(self.vehicle_controller, 'toggle_tracking'):
                self.vehicle_controller.toggle_tracking(tracking)
                status = "开启" if tracking else "关闭"
                return {'success': True, 'message': '纠偏功能已{}'.format(status)}
            else:
                return {'success': False, 'message': '车辆控制器不支持toggle_tracking方法'}
        except Exception as e:
            return {'success': False, 'message': str(e)}

    def _handle_toggle_path_planning(self, params):
        """处理切换路径规划模式命令"""
        try:
            path_mode = params.get('path', 'left')

            if hasattr(self.vehicle_controller, 'toggle_path_planning'):
                self.vehicle_controller.toggle_path_planning(path_mode)
                return {'success': True, 'message': '路径规划模式已切换为: {}'.format(path_mode)}
            else:
                return {'success': False, 'message': '车辆控制器不支持toggle_path_planning方法'}
        except Exception as e:
            return {'success': False, 'message': str(e)}

    # ==================== 任务管理命令处理 ====================

    def _handle_create_task(self, params):
        """处理创建任务命令"""
        try:
            if hasattr(self.vehicle_controller, 'create_task'):
                result = self.vehicle_controller.create_task(params)
                return {'success': True, 'message': '任务创建成功', 'data': result}
            else:
                return {'success': False, 'message': '车辆控制器不支持create_task方法'}
        except Exception as e:
            return {'success': False, 'message': str(e)}

    def _handle_select_task(self, params):
        """处理选择任务命令"""
        try:
            task_name = params.get('taskName', '')

            if hasattr(self.vehicle_controller, 'select_task'):
                result = self.vehicle_controller.select_task(task_name)
                return {'success': True, 'message': '任务已选择', 'data': result}
            else:
                return {'success': False, 'message': '车辆控制器不支持select_task方法'}
        except Exception as e:
            return {'success': False, 'message': str(e)}

    def _handle_save_task(self, params):
        """处理保存任务命令"""
        try:
            task_name = params.get('taskName', '')

            if hasattr(self.vehicle_controller, 'save_task'):
                self.vehicle_controller.save_task(task_name)
                return {'success': True, 'message': '任务已保存'}
            else:
                return {'success': False, 'message': '车辆控制器不支持save_task方法'}
        except Exception as e:
            return {'success': False, 'message': str(e)}

    # ==================== 参数配置命令处理 ====================

    def _handle_save_params(self, params):
        """处理保存参数命令"""
        try:
            if hasattr(self.vehicle_controller, 'save_params'):
                self.vehicle_controller.save_params(params)
                return {'success': True, 'message': '参数已保存'}
            else:
                return {'success': False, 'message': '车辆控制器不支持save_params方法'}
        except Exception as e:
            return {'success': False, 'message': str(e)}

    def _handle_get_status(self, params):
        """处理获取状态命令"""
        try:
            if hasattr(self.vehicle_controller, 'get_status'):
                status = self.vehicle_controller.get_status()
                return {'success': True, 'message': '状态获取成功', 'data': status}
            else:
                return {'success': False, 'message': '车辆控制器不支持get_status方法'}
        except Exception as e:
            return {'success': False, 'message': str(e)}
