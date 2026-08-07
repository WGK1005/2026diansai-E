import time
import struct
from machine import FPIOA, UART

# =====================================================================
# 1. 硬件引脚与串口初始化
# =====================================================================
fpioa = FPIOA()
fpioa.set_function(3, FPIOA.UART1_TXD)
fpioa.set_function(4, FPIOA.UART1_RXD)
uart = UART(UART.UART1, baudrate=115200, bits=UART.EIGHTBITS, parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)

# =====================================================================
# 2. 34字节协议打包下发函数
# =====================================================================
def send_34byte_packet(uart, x_pulses, z_pulses, y_angle_deg, theta_angle_deg):
    """
    发送严格的 34 字节数据包:
    [包头 0xA5] + [12字节有效数据 + 20字节0x00占位] + [1字节校验和]
    """
    header = b"\xA5"

    # 角度放大 10 倍转为 16 位整型
    y_int16 = int(y_angle_deg * 10)
    theta_int16 = int(theta_angle_deg * 10)

    # 打包前 12 字节有效数据 (2个int32 + 2个int16)
    useful_data = struct.pack("<2i2h", x_pulses, z_pulses, y_int16, theta_int16)

    # 填充 20 字节的 0x00，使 Payload 凑满 32 字节
    padding = bytes([0x00] * 20)
    payload = useful_data + padding

    # 计算 32 字节 Payload 的异或校验和
    checksum = 0
    for b in payload:
        checksum ^= b

    # 组装完整的 34 字节数据包并发送
    packet = header + payload + bytes([checksum])
    uart.write(packet)

    print(">>> 发送指令 -> 磁铁旋转角度: %5.1f° | 数据包总长: %d 字节" % (theta_angle_deg, len(packet)))

# =====================================================================
# 3. 5秒步进 15 度测试主循环
# =====================================================================
print(">>> 测试启动: 旋转磁铁舵机每隔 5 秒步进 15 度 <<<")

current_theta = 0.0  # 初始角度归零
direction = 1        # 1代表正向旋转，-1代表反向旋转

try:
    while True:
        # 下发指令 (屏蔽X、Z和Y轴的动作，只控制电磁铁旋转轴 Theta)
        send_34byte_packet(uart, x_pulses=0, z_pulses=0, y_angle_deg=0.0, theta_angle_deg=current_theta)

        # 严格延时 5 秒
        time.sleep(5.0)

        # 累加 15 度
        current_theta += (15.0 * direction)


except KeyboardInterrupt:
    print(">>> 测试已手动停止 <<<")
