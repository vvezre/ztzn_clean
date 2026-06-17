import os

with open('main.py', 'rb') as f:
    text = f.read().decode('utf-8', errors='ignore')

with open('restore.txt', 'rb') as f:
    content = f.read()
    if content.startswith(b'\xff\xfe'):
        restore = content.decode('utf-16', errors='ignore')
    else:
        restore = content.decode('utf-8', errors='ignore')

target = "logger.warn('执行任务{}；angle={},mode={},length={},back_len={}'.format(id, angle, mode, length, back_len))"
idx = text.find(target)

if idx != -1:
    idx += len(target)
    n_idx = text.find('\n', idx)
    if n_idx != -1:
        text = text[:n_idx+1] + "\n" + restore + text[n_idx+1:]
        with open('main.py', 'w', encoding='utf-8') as f:
            f.write(text)
        print("Fixed successfully")
else:
    print("Target not found")
