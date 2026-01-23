# coding=utf-8
import logging

logger = logging.getLogger()
logger.setLevel(logging.INFO)

fileHandler = logging.FileHandler('app.log')
fileHandler.setLevel(logging.WARN)

streamHandler = logging.StreamHandler()
streamHandler.setLevel(logging.INFO)
# 定义日志格式
formatter = logging.Formatter("%(asctime)s - %(filename)s[line:%(lineno)d] - %(levelname)s: %(message)s")
streamHandler.setFormatter(formatter)
fileHandler.setFormatter(formatter)

logger.addHandler(streamHandler)
logger.addHandler(fileHandler)