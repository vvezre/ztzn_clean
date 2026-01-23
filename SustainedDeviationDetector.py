# coding=utf-8
import random
import time
from collections import deque


class SustainedDeviationDetector:
    def __init__(self, threshold=0.1, duration=1.0, sample_rate=10):
        self.threshold = threshold  # 40 cm
        self.duration = duration    # 1 s
        self.sample_rate = sample_rate
        self.window_size = int(duration * sample_rate)
        self.buffer = deque(maxlen=self.window_size)
        self.timer = 0
        self.last_time = time.time()

    def update(self, deviation):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        self.buffer.append(deviation)

        if len(self.buffer) == self.window_size:
            if all(d > self.threshold for d in self.buffer):
                return True
        return False

# 使用示例
# detector = SustainedDeviationDetector()
# while True:
#     deviation = get_lateral_deviation()  # 获取横向偏差
#     if detector.update(deviation):
#         print("横向偏差持续大于40 cm")
#         break
#     time.sleep(0.1)

if __name__ == '__main__':
    detector = SustainedDeviationDetector()
    while True:
        cte = random.uniform(0, 0.3)
        if detector.update(cte):
            print("横向偏差持续大于10 cm")
            break
        else:
            print("横向偏差不是持续大于10 cm")
        time.sleep(0.01)