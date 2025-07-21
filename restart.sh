
#!/bin/bash
# 查找所有Python进程PID
pids=$(ps -aux | grep python | grep -v grep | awk '{print $2}')

if [ -z "$pids" ]; then
    echo "未找到运行的Python进程"
    exit 0
fi

echo "找到以下Python进程："
ps -aux | grep python | grep -v grep

sudo kill -9 $pids
echo "已终止PID: $pids"
echo "重新启动main程序"
nohup python main.py > output.log 2>&1 &
