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
    发送严格的 34 字节数据包
    """
    header = b"\xA5"

    # 角度放大 10 倍转为 16 位整型
    y_int16 = int(y_angle_deg * 10)
    theta_int16 = int(theta_angle_deg * 10)

    # 打包前 12 字节有效数据
    useful_data = struct.pack("<2i2h", x_pulses, z_pulses, y_int16, theta_int16)

    # 填充 20 字节的 0x00
    padding = bytes([0x00] * 20)
    payload = useful_data + padding

    # 计算校验和
    checksum = 0
    for b in payload:
        checksum ^= b

    packet = header + payload + bytes([checksum])
    uart.write(packet)

    print(f">>> 发送指令 -> 步进脉冲: {x_pulses:<6} | 舵机角度: {theta_angle_deg:>5.1f}° | 校验和: {hex(checksum)}")

# =====================================================================
# 3. 联合测试主循环
# =====================================================================
def main():
    print(">>> 步进电机 + 舵机 联合通讯测试启动 <<<")
    print(">>> 行为: 每隔5秒触发，步进正反3次轮换(每次32000)，舵机±90度往返(步进15度) <<<")

    # --- 步进电机状态变量 ---
    stepper_dir = 1       # 1 为正转， -1 为反转
    stepper_count = 0     # 记录当前方向执行了多少次

    # --- 舵机状态变量 ---
    theta_angle = 0.0     # 初始角度归零
    theta_dir = 1         # 1 为正向旋转，-1 为反向旋转

    try:
        while True:
            # 1. 准备当前周期的目标数据
            current_x_pulses = 9600 * stepper_dir

            # 2. 将数据合并打包，一并下发
            send_34byte_packet(
                uart=uart,
                x_pulses=current_x_pulses,
                z_pulses=0,
                y_angle_deg=0.0,
                theta_angle_deg=theta_angle
            )

            # ==========================================
            # 3. 更新下一次循环的 步进电机 状态
            # ==========================================
            stepper_count += 1
            if stepper_count >= 3:
                stepper_dir *= -1   # 翻转步进电机方向
                stepper_count = 0   # 计次清零
                print("--- [步进电机准备翻转方向] ---")

            # ==========================================
            # 4. 更新下一次循环的 旋转舵机 状态
            # ==========================================
            theta_angle += (15.0 * theta_dir)



            # 5. 严格延时 5 秒
            time.sleep(5.0)

    except KeyboardInterrupt:
        print(">>> 测试已手动停止 <<<")

if __name__ == "__main__":
    main()
