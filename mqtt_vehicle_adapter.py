# coding=utf-8
"""
车辆控制器适配器
将MQTT命令适配到实际的车辆控制函数
"""
import requests

from AppLogger import logger


class VehicleControllerAdapter:
    """
    车辆控制器适配器类
    将MQTT命令转换为实际的车辆控制调用
    """
    def __init__(self):
        logger.info('小车控制适配器初始化完成')

    def drive(self, params):
        '''
        前进命令
        '''
        # 接口地址
        url = "http://192.168.0.175/vehicle/drive"

        # 查询参数
        params = {
            "id": 123,
            "lang": "zh"
        }

        try:
            response = requests.get(url, params=params, timeout=10)
            response.raise_for_status()  # 如果状态码不是 2xx，抛出异常

            data = response.json()  # 假设返回 JSON
            print("用户姓名:", data["name"])

        except requests.exceptions.RequestException as e:
            print("请求失败:", e)
        except KeyError as e:
            print("返回数据格式错误:", e)
