import time
import struct
from machine import FPIOA, UART

# =====================================================================
# 1. 硬件引脚与串口初始化
# =====================================================================
fpioa = FPIOA()
fpioa.set_function(3, FPIOA.UART1_TXD)
fpioa.set_function(4, FPIOA.UART1_RXD)
# 初始化串口 1，波特率 115200
uart = UART(UART.UART1, baudrate=115200, bits=UART.EIGHTBITS, parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)

# =====================================================================
# 2. 34 字节协议打包下发函数
# =====================================================================
def send_34byte_packet(uart, x_pulses, z_pulses, y_angle_deg, theta_angle_deg):
    """
    发送严格的 34 字节数据包:
    [包头 0xA5] + [12字节有效数据 + 20字节0x00占位] + [1字节校验和]
    """
    header = b"\xA5"

    y_int16 = int(y_angle_deg * 10)
    theta_int16 = int(theta_angle_deg * 10)

    useful_data = struct.pack("<2i2h", x_pulses, z_pulses, y_int16, theta_int16)

    padding = bytes([0x00] * 20)
    payload = useful_data + padding

    checksum = 0
    for b in payload:
        checksum ^= b

    packet = header + payload + bytes([checksum])
    uart.write(packet)

    # 打印发送状态，如果是负数会清楚地带上负号
    print(f">>> 发送指令 -> X轴脉冲: {x_pulses} | 校验和: {hex(checksum)}")

# =====================================================================
# 3. 步进电机测试主循环 (正转 3 次，反转 3 次)
# =====================================================================
def main():
    print(">>> 步进电机 34 字节协议通讯测试启动 <<<")
    print(">>> 预期行为: 每隔 5 秒发一次指令。正转 3 次(+32000)后，反转 3 次(-32000) <<<")

    direction = 1       # 1 表示正转， -1 表示反转
    action_count = 0    # 记录当前方向已经执行了多少次

    try:
        while True:
            # 每次下发固定脉冲数 (32000 乘以正负号)，绝不累加
            current_x_pulses = 32000 * direction

            # 下发指令
            send_34byte_packet(
                uart=uart,
                x_pulses=current_x_pulses,
                z_pulses=0,
                y_angle_deg=0.0,
                theta_angle_deg=0.0
            )

            # 计数器加 1
            action_count += 1

            # 如果当前方向已经发送了 3 次，就翻转方向并清零计数器
            if action_count >= 3:
                direction *= -1   # 1 变成 -1，或者 -1 变成 1
                action_count = 0  # 重新开始计次
                print("--- 准备改变方向 ---")

            # 严格延时 5 秒
            time.sleep(10.0)

    except KeyboardInterrupt:
        print(">>> 测试已手动停止 <<<")
if __name__ == "__main__":
    main()
