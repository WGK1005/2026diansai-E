# self_puzzle_vision.py — 电赛E题 选手自备碎片(黑底白碎片) 纯视觉识别与拼图解算
import time, os, gc, math
import image
import ulab.numpy as np

from media.sensor import Sensor, CAM_CHN_ID_0
from media.display import Display
from media.media import MediaManager

# ==================== 1. 分辨率与物理尺寸参数 ====================
WIDTH, HEIGHT = 800, 480
A4_WIDTH_MM = 210.0
A4_HEIGHT_MM = 297.0
SCALE_PX_MM = WIDTH / A4_WIDTH_MM  # 约 3.81 px/mm

# 图2 自备拼图目标矩形尺寸 (mm)
RECT_W, RECT_H = 100.0, 60.0
TARGET_CENTER_MM = (105.0, 222.75) # A4纸下半区中心 (105mm, 222.75mm)

# 黑色 A4 纸上的白色碎片灰度阈值 (极度稳定，可根据现场光照微调)
WHITE_GRAY_THRESHOLD = [(180, 255)]

# ==================== 2. 理论模板与数学解算函数 ====================

def get_self_templates():
    """图2 4个碎片的标准理论多边形顶点 (单位: mm)"""
    p0 = [0.0, 0.0]
    p1 = [20.0, 0.0]
    p2 = [100.0, 0.0]
    p3 = [100.0, 60.0]
    p4 = [0.0, 60.0]
    left_a = [0.0, 20.0]
    left_b = [0.0, 30.0]
    diag_a = [36.0, 12.0]
    diag_b = [76.0, 42.0]

    return [
        [p0, p1, diag_a, left_a],
        [left_a, diag_a, diag_b, left_b],
        [left_b, diag_b, p3, p4],
        [p1, p2, p3, diag_b, diag_a]
    ]

def get_centroid(pts):
    """计算多边形顶点质心 (mm)"""
    if not pts:
        return 0.0, 0.0
    sx = sum(p[0] for p in pts)
    sy = sum(p[1] for p in pts)
    return sx / len(pts), sy / len(pts)

def rotate_point(x, y, angle_rad):
    """2D 向量旋转"""
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    return x * cos_a - y * sin_a, x * sin_a + y * cos_a

def solve_rigid_transform(src_pts, tgt_pts):
    """求解源多边形到目标模板的最佳旋转角度与对齐残差"""
    N = len(src_pts)
    if N != len(tgt_pts):
        return 1e9, 0.0

    src_cx, src_cy = get_centroid(src_pts)
    tgt_cx, tgt_cy = get_centroid(tgt_pts)

    src_shifted = [[p[0] - src_cx, p[1] - src_cy] for p in src_pts]
    tgt_shifted = [[p[0] - tgt_cx, p[1] - tgt_cy] for p in tgt_pts]

    best_err = 1e9
    best_angle = 0.0

    # 遍历顶点的排列对应关系
    for shift in range(N):
        shifted_tgt = tgt_shifted[shift:] + tgt_shifted[:shift]

        sum_sin = 0.0
        sum_cos = 0.0
        for i in range(N):
            a1 = math.atan2(src_shifted[i][1], src_shifted[i][0])
            a2 = math.atan2(shifted_tgt[i][1], shifted_tgt[i][0])
            da = a2 - a1
            sum_sin += math.sin(da)
            sum_cos += math.cos(da)

        angle = math.atan2(sum_sin, sum_cos)

        err = 0.0
        for i in range(N):
            rx, ry = rotate_point(src_shifted[i][0], src_shifted[i][1], angle)
            err += math.sqrt((rx - shifted_tgt[i][0])**2 + (ry - shifted_tgt[i][1])**2)
        err /= N

        if err < best_err:
            best_err = err
            best_angle = angle

    return best_err, best_angle

# ==================== 3. 核心视觉提取与拼图求解入口 ====================

def process_self_puzzle_vision(img):
    """
    黑底白碎片视觉识别与几何解算核心函数
    :param img: 摄像头抓取的传感器图像对象
    :return: (is_success, motion_results)
    """
    split_y_px = int(148.5 * SCALE_PX_MM) # A4 纸上下分界线

    # 转灰度图并提取白色碎片色块
    gray_img = img.to_grayscale()
    blobs = gray_img.find_blobs(WHITE_GRAY_THRESHOLD, pixels_threshold=200, area_threshold=200, merge=True)

    # 仅保留上半区的 4 个碎片 (y < 分界线)
    valid_blobs = [b for b in blobs if b.cy() < split_y_px]

    detected_pieces = []
    for idx, blob in enumerate(valid_blobs[:4]):
        # 在原图上画框和质心标记
        img.draw_rectangle(blob[0:4], color=(0, 255, 0), thickness=2)
        img.draw_cross(blob.cx(), blob.cy(), color=(0, 255, 0), thickness=2)

        cx_mm = blob.cx() / SCALE_PX_MM
        cy_mm = blob.cy() / SCALE_PX_MM

        detected_pieces.append({
            'id': idx + 1,
            'cx_mm': cx_mm,
            'cy_mm': cy_mm,
            'rect_pts_mm': [
                (blob.x() / SCALE_PX_MM, blob.y() / SCALE_PX_MM),
                ((blob.x() + blob.w()) / SCALE_PX_MM, blob.y() / SCALE_PX_MM),
                ((blob.x() + blob.w()) / SCALE_PX_MM, (blob.y() + blob.h()) / SCALE_PX_MM),
                (blob.x() / SCALE_PX_MM, (blob.y() + blob.h()) / SCALE_PX_MM)
            ]
        })
        img.draw_string_advanced(blob.cx() + 10, blob.cy() - 10, 20, f"P{idx+1}", color=(255, 255, 0))

    # 判断是否凑齐 4 块碎片
    if len(detected_pieces) != 4:
        return False, f"捕捉中: {len(valid_blobs)}/4 块"

    # 执行 SVD 刚体变换匹配
    try:
        templates = get_self_templates()
        target_origin_x = TARGET_CENTER_MM[0] - RECT_W / 2.0
        target_origin_y = TARGET_CENTER_MM[1] - RECT_H / 2.0

        motion_results = []
        for idx, p in enumerate(detected_pieces):
            tmpl = templates[idx]
            tmpl_cx, tmpl_cy = get_centroid(tmpl)

            # 计算在下半区目标矩形中的物理位置
            tgt_x_mm = target_origin_x + tmpl_cx
            tgt_y_mm = target_origin_y + tmpl_cy

            _, angle_rad = solve_rigid_transform(p['rect_pts_mm'], tmpl)
            angle_deg = math.degrees(angle_rad)

            motion_results.append({
                'id': p['id'],
                'src_x_mm': p['cx_mm'],
                'src_y_mm': p['cy_mm'],
                'tgt_x_mm': tgt_x_mm,
                'tgt_y_mm': tgt_y_mm,
                'rotation_deg': angle_deg
            })

        return True, motion_results

    except Exception as e:
        return False, f"解算失败: {e}"

# ==================== 4. 传感器驱动与主循环 ====================

def main():
    # 初始化硬件与摄像头
    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(chn=CAM_CHN_ID_0, width=WIDTH, height=HEIGHT)
    sensor.set_pixformat(Sensor.RGB888, chn=CAM_CHN_ID_0)
    Display.init(Display.ST7701, width=WIDTH, height=HEIGHT, to_ide=True)
    MediaManager.init()
    sensor.run()

    print("K230 黑底白碎片纯视觉识别程序启动...")

    clock = time.clock()

    while True:
        clock.tick()
        os.exitpoint()

        img = sensor.snapshot(chn=CAM_CHN_ID_0)

        # 绘制 A4 纸上下分界线 (Y = 148.5mm 位置)
        split_y_px = int(148.5 * SCALE_PX_MM)
        img.draw_line(0, split_y_px, WIDTH - 1, split_y_px, color=(255, 255, 0), thickness=2)

        # 调用纯视觉识别与解算逻辑
        success, results = process_self_puzzle_vision(img)
if success:
            # 识别并拼图求解成功，在控制台实时打印解算出的坐标与旋转角
            img.draw_string_advanced(
                10, 10, 25, "SOLVED SUCCESS!", color=(0, 255, 0)
            )
            for res in results:
                src_str = f"源:({res['src_x_mm']:.1f}, {res['src_y_mm']:.1f})mm"
                tgt_str = f"目标:({res['tgt_x_mm']:.1f}, {res['tgt_y_mm']:.1f})mm"
                rot_str = f"旋转:{res['rotation_deg']:.1f}°"
                print(f"碎片 P{res['id']} -> {src_str} | {tgt_str} | {rot_str}")
        else:
            # 未抓齐 4 块碎片或解算中
            img.draw_string_advanced(
                10, 10, 25, str(results), color=(255, 255, 255)
            )

        # 实时刷新屏幕显示
        Display.show_image(img)
        gc.collect()
        time.sleep_ms(10)

if __name__ == "__main__":
    main()
