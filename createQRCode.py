# coding=utf-8

import qrcode

# 要编码的字符串
data = "http://192.168.0.175"

# 创建二维码对象
qr = qrcode.QRCode(
    version=1,                # 二维码大小（1~40，1 最小）
    error_correction=qrcode.constants.ERROR_CORRECT_L,  # 容错率 L(7%) M(15%) Q(25%) H(30%)
    box_size=10,              # 每个方块的像素大小
    border=4,                 # 边框宽度（最小为4）
)

# 添加数据
qr.add_data(data)
qr.make(fit=True)  # 自动调整版本以适应数据

# 生成图像
img = qr.make_image(fill_color="black", back_color="white")

# 保存为文件
img.save("qrcode.png")
print("二维码已保存为 qrcode.png")