# coding=utf-8

from collections import deque

from AppLogger import logger


class FixedPositiveChecker:
    def __init__(self, window_size=10):
        # maxlen 参数是关键，它会自动保持 deque 的最大长度
        # 当长度超过 maxlen 时，最左边的（最老的）元素会被自动挤出
        self.buffer = deque(maxlen=window_size)
        self.window_size = window_size

    def add_number(self, number):
        """
        向缓冲区添加一个新数字。
        如果缓冲区已满，最老的数字会自动被挤出。
        """
        self.buffer.append(number)  # 新数据从右边进入
        # print("添加 {} -> 缓冲区: {}".format(number,list(self.buffer)))

    def are_all_positive(self):
        """
        检查缓冲区中的所有数字是否都是正数 (> 0)。
        :return: True 如果所有数都是正数，False 否则。
                 如果缓冲区未满，也返回 False (因为题目要求是"10个数")
        """
        # 只有当缓冲区满了（即已经有10个数）时才进行判断
        if len(self.buffer) < self.window_size:
            print("缓冲区尚未满，不进行判断。")
            return False

        # 使用 all() 函数检查所有元素是否 > 0
        result = all(num > 0 for num in self.buffer)

        # if result:
        #     logger.warn("缓冲区中的 {} 个数都是正数: {}".format(self.window_size,list(self.buffer)))
        # else:
        #     logger.warn("缓冲区中的 {} 个数不全是正数: {}".format(self.window_size,list(self.buffer)))

        return result


    def getAverage(self):
        # 只有当缓冲区满了（即已经有10个数）时才进行算平均数
        if len(self.buffer) < self.window_size:
            # print("缓冲区尚未满，不进行判断。")
            return 1000
        # 计算平均数
        average = sum(self.buffer) / len(self.buffer)
        return average

if __name__ == '__main__':
    checker = FixedPositiveChecker(window_size=10)
    data_stream = [
        1, 2, 3, 4, 5, 6, 7, 8, 9,  # 前9个正数，缓冲区未满
        10,  # 第10个，缓冲区满了！检查 -> 全是正数 ✅
        -1,  # 新数-1进来，最老的1被挤出。检查 -> 包含-1 ❌
        15, 20, 25,  # 继续添加正数，但缓冲区里还有-1 ❌
        30  # 直到-1被完全挤出...
    ]
    for num in data_stream:
        checker.add_number(num)
        checker.are_all_positive()  # 每次添加后都检查（也可以按需检查）