# coding=utf-8
class RTKTimeoutError(Exception):
    """
    自定义异常：表示 RTK 设备数据读取超时。
    """
    def __init__(self, message="RTK device timeout: no valid data received"):
        super(RTKTimeoutError, self).__init__(message)
        self.message = message

    def __str__(self):
        return self.message