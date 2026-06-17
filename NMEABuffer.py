# coding=utf-8

class NMEABuffer:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        """喂入新数据，返回所有完整行（含 \r\n）"""
        self.buffer.extend(data)
        lines = []
        while b'\r\n' in self.buffer:
            idx = self.buffer.find(b'\r\n')
            line = self.buffer[:idx + 2]  # 包含 \r\n
            lines.append(line)
            del self.buffer[:idx + 2]
        return lines

    def remaining(self):
        """返回未完成的残余数据（用于调试）"""
        return bytes(self.buffer)