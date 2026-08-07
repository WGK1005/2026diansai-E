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
# 2. 机械参数与物理换算配置
# =====================================================================
# X 轴步进: 3200脉冲/圈，丝距2mm -> 1600 脉冲/mm
PULSES_PER_MM = 1600.0

# Y 轴舵机: 360度对应11.3cm(113mm) -> 3.1858 度/mm
Y_DEG_PER_MM = 360.0 / 113.0

# 核心动作参数
ZERO_X_PULSES = 0       # X 轴机械零点
ZERO_Y_DEG = 0.0        # Y 轴机械零点
ZERO_THETA_DEG = 0.0    # Theta 旋转机械零点

Z_UP_PULSES = 16000     # Z 轴抬起高度绝对位置 (Z↑16000)
Z_DOWN_PULSES = 0       # Z 轴下降高度绝对位置 (Z↓0，降至纸面)

STEP_DELAY = 7.0        # 步骤等待间隔（秒）
MAGNET_DELAY = 0.5      # 电磁铁响应延时

# =====================================================================
# 3. 舵机虚拟当前位置记录 (步进发绝对值，舵机发差值增量)
# =====================================================================
current_y_deg = 0.0
current_theta_deg = 0.0

# =====================================================================
# 4. 坐标解算函数 (毫米 -> 控制量)
# =====================================================================
def mm_to_motion(x_mm, y_mm, theta):
    """
    输入真实的毫米(mm)坐标，直接输出物理目标量 (X绝对脉冲, Y绝对角度)
    """
    x_pulses = int(x_mm * PULSES_PER_MM)
    y_deg = y_mm * Y_DEG_PER_MM

    return x_pulses, y_deg, theta

# =====================================================================
# 5. 34 字节协议打包下发函数
# =====================================================================
def send_34byte_packet(uart, x_pulses, z_pulses, y_angle_deg, theta_angle_deg, magnet_on=False):
    global current_y_deg, current_theta_deg

    # 舵机: 计算差值 (目标 - 当前位置)
    delta_y     = y_angle_deg     - current_y_deg
    delta_theta = theta_angle_deg - current_theta_deg

    # 更新当前位置记录
    current_y_deg     = y_angle_deg
    current_theta_deg = theta_angle_deg

    header = b"\xA5"

    y_int16  = int(delta_y     * 10)
    theta_int16 = int(delta_theta * 10)

    # 12 字节有效数据 (X/Z发绝对脉冲, Y/Theta发差值增量)
    useful_data = struct.pack("<2i2h", x_pulses, z_pulses, y_int16, theta_int16)

    # Padding: 第 1 字节为电磁铁状态
    mag_byte = 0x01 if magnet_on else 0x00
    padding = bytes([mag_byte] + [0x00] * 19)

    payload = useful_data + padding

    checksum = 0
    for b in payload:
        checksum ^= b

    packet = header + payload + bytes([checksum])
    uart.write(packet)

    mag_str = "ON " if magnet_on else "OFF"
    print(f"指令发包 -> X脉冲:{x_pulses:<7} | Z脉冲:{z_pulses:<5} | Y增量角度:{delta_y:>+6.1f}° | 磁铁:{mag_str}")

# =====================================================================
# 6. 单块碎片拼图流程 (精细化 8 步)
# =====================================================================
def place_first_piece(src_mm_x, src_mm_y, src_theta, dst_mm_x, dst_mm_y, dst_theta):
    global current_y_deg, current_theta_deg

    # 起点同步: 假定机械与舵机均从零点出发
    current_y_deg     = ZERO_Y_DEG
    current_theta_deg = ZERO_THETA_DEG

    # 将输入的真实物理毫米直接转换为控制量
    src_x_p, src_y_d, _ = mm_to_motion(src_mm_x, src_mm_y, src_theta)
    dst_x_p, dst_y_d, _ = mm_to_motion(dst_mm_x, dst_mm_y, dst_theta)

    print("\n========== 坐标解算结果 (毫米直入) ==========")
    print(f"[抓取点] X: {src_mm_x} mm -> 对应绝对脉冲: {src_x_p}")
    print(f"[抓取点] Y: {src_mm_y} mm -> 对应绝对角度: {src_y_d:.1f}°")
    print(f"[放置点] X: {dst_mm_x} mm -> 对应绝对脉冲: {dst_x_p}")
    print(f"[放置点] Y: {dst_mm_y} mm -> 对应绝对角度: {dst_y_d:.1f}°")
    print("=============================================\n")

    print("========== 开始搬运流程 ==========")

    # 步1: Z↑16000 磁铁OFF
    print("\n[步1] Z↑16000, 磁铁OFF (抬起Z轴)")
    send_34byte_packet(uart, ZERO_X_PULSES, Z_UP_PULSES, ZERO_Y_DEG, ZERO_THETA_DEG, magnet_on=False)
    time.sleep(STEP_DELAY)

    # 步2: XY→碎片上方绝对位置
    print("[步2] XY→碎片, 磁铁OFF (移动至抓取点上方)")
    send_34byte_packet(uart, src_x_p, Z_UP_PULSES, src_y_d, src_theta, magnet_on=False)
    time.sleep(STEP_DELAY)

    # 步3a: Z↓降至0 磁铁OFF
    print("[步3a] Z↓0, 磁铁OFF (下降接触碎片)")
    send_34byte_packet(uart, src_x_p, Z_DOWN_PULSES, src_y_d, src_theta, magnet_on=False)
    time.sleep(STEP_DELAY)

    # 步3b: 吸合电磁铁
    print("[步3b] 无运动, 磁铁ON (开启电磁铁吸取)")
    send_34byte_packet(uart, src_x_p, Z_DOWN_PULSES, src_y_d, src_theta, magnet_on=True)
    time.sleep(MAGNET_DELAY)

    # 步4: Z↑16000 带料抬起
    print("[步4] Z↑16000, 磁铁ON (带料抬起)")
    send_34byte_packet(uart, src_x_p, Z_UP_PULSES, src_y_d, src_theta, magnet_on=True)
    time.sleep(STEP_DELAY)

    # 步5: XY→目标上方绝对位置
    print("[步5] XY→目标+θ, 磁铁ON (移动至放置点并旋转)")
    send_34byte_packet(uart, dst_x_p, Z_UP_PULSES, dst_y_d, dst_theta, magnet_on=True)
    time.sleep(STEP_DELAY)

    # 步6a: Z↓降至0 放置
    print("[步6a] Z↓0, 磁铁ON (下降至放置平面)")
    send_34byte_packet(uart, dst_x_p, Z_DOWN_PULSES, dst_y_d, dst_theta, magnet_on=True)
    time.sleep(STEP_DELAY)

    # 步6b: 释放电磁铁
    print("[步6b] 无运动, 磁铁OFF (释放碎片)")
    send_34byte_packet(uart, dst_x_p, Z_DOWN_PULSES, dst_y_d, dst_theta, magnet_on=False)
    time.sleep(MAGNET_DELAY)

    # 步7: Z↑16000 抬起
    print("[步7] Z↑16000, 磁铁OFF (空载抬起脱离)")
    send_34byte_packet(uart, dst_x_p, Z_UP_PULSES, dst_y_d, dst_theta, magnet_on=False)
    time.sleep(STEP_DELAY)

    # 步8: 回零
    print("[步8] XY→零点, 磁铁OFF (复位机械臂)")
    send_34byte_packet(uart, ZERO_X_PULSES, Z_UP_PULSES, ZERO_Y_DEG, ZERO_THETA_DEG, magnet_on=False)
    time.sleep(STEP_DELAY)

    print("========== 搬运完成 ==========\n")

# =====================================================================
# 7. 主程序执行入口
# =====================================================================
def main():
    print(">>> 物理坐标联动系统启动 <<<\n")

    try:
        # 直接填入真实物理毫米 (mm) 坐标
        place_first_piece(
            src_mm_x=47.0,     # 抓取点 X: 47mm
            src_mm_y=40.0,     # 抓取点 Y: 40mm
            src_theta=0.0,

            dst_mm_x=178.0,    # 放置点 X: 178mm
            dst_mm_y=60.0,     # 放置点 Y: 60mm
            dst_theta=30.0
        )
    except KeyboardInterrupt:
        print(">>> 运行中止 <<<")

if __name__ == "__main__":
    main()
