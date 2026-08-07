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
# 2. 绝对坐标与硬件参数配置
# =====================================================================
# 视觉像素 -> 物理毫米 转换系数
MM_PER_PX_X = 0.4243
MM_PER_PX_Y = 0.5330

# X 轴步进: 3200脉冲/2mm = 1600 脉冲/mm
PULSES_PER_MM = 1600.0

# Y 轴舵机: 360度/113mm ≈ 3.1858 度/mm
Y_DEG_PER_MM = 360.0 / 113.0

# --- Z 轴绝对坐标位置定义 ---
Z_UP_PULSES = 16000   # 绝对位置: 安全抬起平面 (16000 脉冲)
Z_DOWN_PULSES = 0     # 绝对位置: 下降贴合纸面 (0 脉冲)

# --- 机械零点 (绝对坐标 0) ---
ZERO_X_PULSES = 0
ZERO_Y_DEG = 0.0
ZERO_THETA_DEG = 0.0

STEP_DELAY = 10.0        # 各动作步骤下发后的等待间隔（秒）
MAGNET_DELAY = 0.5      # 电磁铁吸合/释放响应延时（秒）

# =====================================================================
# 3. 34 字节协议打包下发函数
# =====================================================================
def send_34byte_packet(uart, x_pulses, z_pulses, y_angle_deg, theta_angle_deg, magnet_on=False):
    """
    发送绝对坐标 34 字节数据包:
    - [0]: 0xA5 帧头
    - [1~4]: X 轴绝对目标脉冲 (int32)
    - [5~8]: Z 轴绝对目标脉冲 (int32)
    - [9~10]: Y 轴绝对目标角度 * 10 (int16)
    - [11~12]: Theta 轴绝对目标角度 * 10 (int16)
    - [13]: 磁铁状态 (0x01=ON, 0x00=OFF)
    - [14~32]: 0x00 填充
    - [33]: XOR 校验和
    """
    header = b"\xA5"

    # 角度放大 10 倍转为 16 位整型
    y_int16 = int(y_angle_deg * 10)
    theta_int16 = int(theta_angle_deg * 10)

    # 打包 12 字节核心绝对坐标数据
    useful_data = struct.pack("<2i2h", int(x_pulses), int(z_pulses), y_int16, theta_int16)

    # 填充区：第 13 字节为磁铁状态，后 19 字节补零
    mag_byte = 0x01 if magnet_on else 0x00
    padding = bytes([mag_byte] + [0x00] * 19)

    payload = useful_data + padding

    # 计算校验和
    checksum = 0
    for b in payload:
        checksum ^= b

    packet = header + payload + bytes([checksum & 0xFF])
    uart.write(packet)

    mag_str = "ON " if magnet_on else "OFF"
    print(f"绝对坐标指令 -> X: {int(x_pulses):<6} | Z: {int(z_pulses):<5} | Y: {y_angle_deg:>5.1f}° | Theta: {theta_angle_deg:>5.1f}° | 磁铁: {mag_str}")

# =====================================================================
# 4. 像素坐标转绝对物理坐标
# =====================================================================
def px_to_absolute_motion(x_px, y_px, theta_deg):
    """
    将像素转换成相对于零点的绝对物理位置 (绝对脉冲, 绝对角度)
    """
    x_mm = x_px * MM_PER_PX_X
    y_mm = y_px * MM_PER_PX_Y

    abs_x_pulses = int(x_mm * PULSES_PER_MM)
    abs_y_deg = y_mm * Y_DEG_PER_MM

    return abs_x_pulses, abs_y_deg, theta_deg

# =====================================================================
# 5. 全绝对坐标下的 8 步搬运流程
# =====================================================================
def run_8step_process_abs(src_px_x, src_px_y, src_theta, dst_px_x, dst_px_y, dst_theta):
    # 计算起点与终点在空间中的【绝对位置坐标】
    src_x_p, src_y_d, _ = px_to_absolute_motion(src_px_x, src_px_y, src_theta)
    dst_x_p, dst_y_d, _ = px_to_absolute_motion(dst_px_x, dst_px_y, dst_theta)

    print("\n==================== 启动绝对坐标 8 步拼图流程 ====================")

    # 步 1: 原地抬起 Z 到安全高度 (16000 脉冲)，XY 保持零点
    print("\n[步 1] 原地抬起 Z 轴 (Z=16000)...")
    send_34byte_packet(uart, ZERO_X_PULSES, Z_UP_PULSES, ZERO_Y_DEG, ZERO_THETA_DEG, magnet_on=False)
    time.sleep(STEP_DELAY)

    # 步 2: 在空中移动 XY 到碎片上方绝对坐标，Z 维持安全高度
    print("[步 2] 平移至碎片上方绝对坐标...")
    send_34byte_packet(uart, src_x_p, Z_UP_PULSES, src_y_d, src_theta, magnet_on=False)
    time.sleep(STEP_DELAY)

    # 步 3a: Z 轴绝对坐标下降到纸面 (Z=0)，XY 保持碎片上方
    print("[步 3a] Z 轴下降至纸面 (Z=0)...")
    send_34byte_packet(uart, src_x_p, Z_DOWN_PULSES, src_y_d, src_theta, magnet_on=False)
    time.sleep(STEP_DELAY)

    # 步 3b: 原地吸合电磁铁 (纯状态包)
    print("[步 3b] 电磁铁吸合...")
    send_34byte_packet(uart, src_x_p, Z_DOWN_PULSES, src_y_d, src_theta, magnet_on=True)
    time.sleep(MAGNET_DELAY)

    # 步 4: Z 轴绝对坐标抬起至安全高度 (Z=16000)，带料
    print("[步 4] Z 轴带料抬起至安全高度 (Z=16000)...")
    send_34byte_packet(uart, src_x_p, Z_UP_PULSES, src_y_d, src_theta, magnet_on=True)
    time.sleep(STEP_DELAY)

    # 步 5: 在空中平移至目标点绝对坐标，并旋转对应姿态，Z 维持 16000
    print("[步 5] 移动至目标绝对点 (X,Y,Theta)...")
    send_34byte_packet(uart, dst_x_p, Z_UP_PULSES, dst_y_d, dst_theta, magnet_on=True)
    time.sleep(STEP_DELAY)

    # 步 6a: Z 轴绝对坐标下降至目标位置纸面 (Z=0)
    print("[步 6a] Z 轴下降放置 (Z=0)...")
    send_34byte_packet(uart, dst_x_p, Z_DOWN_PULSES, dst_y_d, dst_theta, magnet_on=True)
    time.sleep(STEP_DELAY)

    # 步 6b: 原地释放电磁铁 (纯状态包)
    print("[步 6b] 电磁铁释放...")
    send_34byte_packet(uart, dst_x_p, Z_DOWN_PULSES, dst_y_d, dst_theta, magnet_on=False)
    time.sleep(MAGNET_DELAY)

    # 步 7: Z 轴绝对坐标空载抬起至安全高度 (Z=16000)
    print("[步 7] Z 轴空载抬起 (Z=16000)...")
    send_34byte_packet(uart, dst_x_p, Z_UP_PULSES, dst_y_d, dst_theta, magnet_on=False)
    time.sleep(STEP_DELAY)

    # 步 8: 机械臂空载返回机械原点 (0,0,0)
    print("[步 8] 机械臂全轴复位回原点 (0,0,0)...")
    send_34byte_packet(uart, ZERO_X_PULSES, Z_UP_PULSES, ZERO_Y_DEG, ZERO_THETA_DEG, magnet_on=False)
    time.sleep(STEP_DELAY)

    print("==================== 搬运任务完全结束 ====================\n")

# =====================================================================
# 6. 主程序
# =====================================================================
def main():
    print(">>> 全绝对坐标模式 - 拼图联调程序启动 <<<")

    try:
        # 测试数据: 起点与终点像素坐标
        run_8step_process_abs(
            src_px_x=110,
            src_px_y=75,
            src_theta=0.0,

            dst_px_x=419,
            dst_px_y=112,
            dst_theta=30.0
        )
    except KeyboardInterrupt:
        print(">>> 手动停止 <<<")

if __name__ == "__main__":
    main()
