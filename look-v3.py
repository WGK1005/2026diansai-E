import time
import math
from media.sensor import *
from media.display import *
from media.media import *

# =====================================================================
#                          1. 全局配置与参数
# =====================================================================

LCD_WIDTH = 800
LCD_HEIGHT = 480

# --- 阈值配置 ---
# 稍微放宽 A 和 B 的容差，确保稳定识别棕色底上的黑纸
BLACK_A4_TH = (0, 65, -30, 30, -30, 30)
# 高亮冷白光碎片阈值
WHITE_TH = (75, 100, -30, 15, -30, 15)

# --- 物理尺寸 ---
A4_SHORT_MM = 210.0
A4_LONG_MM = 297.0
TARGET_RECT_W_MM = 100.0
TARGET_RECT_H_MM = 60.0

# 预设 4 个碎片在下半区拼合时的相对质心 offset (mm) 与目标角度
PRESET_SLOTS = [
    {"id": 1, "dx":  20.0, "dy": -15.0, "target_angle": 0.0},
    {"id": 2, "dx": -30.0, "dy": -20.0, "target_angle": 12.0},
    {"id": 3, "dx": -25.0, "dy":  15.0, "target_angle": -5.0},
    {"id": 4, "dx":  30.0, "dy":  20.0, "target_angle": -135.0}
]

def find_exact_4_corners(blob):
    pts = blob.corners()
    if len(pts) == 4:
        return pts
    cx, cy = blob.cx(), blob.cy()
    tl = min(pts, key=lambda p: (p[0] - cx) + (p[1] - cy))
    tr = max(pts, key=lambda p: (p[0] - cx) - (p[1] - cy))
    br = max(pts, key=lambda p: (p[0] - cx) + (p[1] - cy))
    bl = min(pts, key=lambda p: (p[0] - cx) - (p[1] - cy))
    return [tl, tr, br, bl]

def get_sort_key(p):
    return p["area"]

# =====================================================================
#                          2. 主程序
# =====================================================================

sensor = None
display_inited = False

try:
    sensor = Sensor(width=LCD_WIDTH, height=LCD_HEIGHT)
    sensor.reset()
    sensor.set_framesize(width=LCD_WIDTH, height=LCD_HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)

    Display.init(Display.ST7701, width=LCD_WIDTH, height=LCD_HEIGHT, to_ide=True)
    display_inited = True

    MediaManager.init()
    sensor.run()

    clock = time.clock()
    print(">>> 4角点精准识别 + A4动态原点 (修正版) <<<")

    while True:
        clock.tick()
        img = sensor.snapshot()

        origin_x, origin_y = None, None
        a4_roi = None

        # 默认自适应比例
        mm_per_pix_x = 0.5330
        mm_per_pix_y = 0.4243
        mid_y_px = LCD_HEIGHT // 2

        # -------------------------------------------------------------
        # 1. 动态寻找 A4 纸并锁定左上角 (0,0) 原点
        # -------------------------------------------------------------
        # 降低像素面积门槛，确保黑纸能被稳定框住
        a4_blobs = img.find_blobs([BLACK_A4_TH], pixels_threshold=10000, area_threshold=10000, merge=True)

        if a4_blobs:
            a4_b = max(a4_blobs, key=lambda b: b.area())
            a4_roi = a4_b.rect()

            # 判断方向并强行注入你测出来的对调比例
            if a4_b.w() > a4_b.h():
                mm_per_pix_x = 0.5330  # X轴反转修正
                mm_per_pix_y = 0.4243  # Y轴反转修正
            else:
                mm_per_pix_x = 0.4243
                mm_per_pix_y = 0.5330

            # 寻找真实左上角
            pts = a4_b.corners()
            if pts and len(pts) >= 4:
                tl = min(pts, key=lambda p: p[0] + p[1])
                origin_x, origin_y = tl[0], tl[1]
            else:
                origin_x, origin_y = a4_b.x(), a4_b.y()

            # 可视化标注绝对原点
            img.draw_circle(origin_x, origin_y, 10, color=(255, 0, 0), fill=True)
            img.draw_string_advanced(max(0, origin_x - 10), max(0, origin_y - 30), 24, "(0,0) Origin", color=(255, 0, 0))

            # 画出 A4 纸框
            img.draw_rectangle(a4_roi, color=(100, 100, 100), thickness=2)

            # 计算中线位置用于限制碎片区域
            mid_y_px = a4_b.y() + a4_b.h() // 2
            img.draw_line(a4_b.x(), mid_y_px, a4_b.x() + a4_b.w(), mid_y_px, color=(255, 140, 0), thickness=2)

        # -------------------------------------------------------------
        # 2. 基于动态原点提取白色碎片并进行相对坐标换算
        # -------------------------------------------------------------
        valid_pieces = []

        if origin_x is not None:
            roi_top = (0, 0, LCD_WIDTH, mid_y_px)

            # 开启 merge=True 和 margin=15 防止大碎片断裂
            blobs = img.find_blobs(
                [WHITE_TH],
                roi=roi_top,
                pixels_threshold=200,
                area_threshold=200,
                merge=True,
                margin=15
            )

            if blobs:
                for blob in blobs:
                    cx, cy = blob.cx(), blob.cy()
                    corners4 = find_exact_4_corners(blob)

                    for p in corners4:
                        img.draw_circle(p[0], p[1], 4, color=(0, 255, 0), fill=True)

                    far_point = max(corners4, key=lambda p: (p[0]-cx)**2 + (p[1]-cy)**2)
                    vx, vy = far_point[0] - cx, far_point[1] - cy
                    angle_deg = math.degrees(math.atan2(vx, -vy))
                    if angle_deg < 0: angle_deg += 360.0

                    # 【核心修正】：计算基于 (0,0) 原点的真实毫米坐标
                    diff_px_x = cx - origin_x
                    diff_px_y = cy - origin_y

                    valid_pieces.append({
                        "cx_px": cx,
                        "cy_px": cy,
                        "cx_mm": diff_px_x * mm_per_pix_x,
                        "cy_mm": diff_px_y * mm_per_pix_y,
                        "angle": angle_deg,
                        "area": blob.area() * mm_per_pix_x * mm_per_pix_y,
                        "blob": blob,
                        "corners4": corners4
                    })

        # -------------------------------------------------------------
        # 3. 下半区精准渲染与轨迹规划 (物理坐标映射)
        # -------------------------------------------------------------
        if len(valid_pieces) >= 4:
            valid_pieces.sort(key=get_sort_key, reverse=True)
            valid_pieces = valid_pieces[:4] # 强制只处理前4个

            # 目标位置也全部相对于 origin_x 和 origin_y 建立
            target_center_x_mm = A4_LONG_MM / 2.0 if (mm_per_pix_x > mm_per_pix_y) else A4_SHORT_MM / 2.0
            target_center_y_mm = (A4_SHORT_MM * 0.75) if (mm_per_pix_x > mm_per_pix_y) else (A4_LONG_MM * 0.75)

            # 转换为像素用于屏幕绘制
            center_x_px = origin_x + int(target_center_x_mm / mm_per_pix_x)
            center_y_px = origin_y + int(target_center_y_mm / mm_per_pix_y)
            rect_w_px = int(TARGET_RECT_W_MM / mm_per_pix_x)
            rect_h_px = int(TARGET_RECT_H_MM / mm_per_pix_y)

            top_left_x = center_x_px - rect_w_px // 2
            top_left_y = center_y_px - rect_h_px // 2
            bot_right_x = top_left_x + rect_w_px
            bot_right_y = top_left_y + rect_h_px

            # 画标准的 100mm x 60mm 绿色目标外框
            img.draw_rectangle(top_left_x, top_left_y, rect_w_px, rect_h_px, color=(0, 255, 0), thickness=2)

            for idx in range(4):
                piece = valid_pieces[idx]
                slot = PRESET_SLOTS[idx]

                # 起点像素
                src_x_px = piece["cx_px"]
                src_y_px = piece["cy_px"]

                # 终点像素
                dst_x_mm = target_center_x_mm + slot["dx"]
                dst_y_mm = target_center_y_mm + slot["dy"]
                dst_x_px = origin_x + int(dst_x_mm / mm_per_pix_x)
                dst_y_px = origin_y + int(dst_y_mm / mm_per_pix_y)

                # 计算移动向量与角度差
                dist_px = math.sqrt((dst_x_px - src_x_px)**2 + (dst_y_px - src_y_px)**2)
                delta_angle = slot["target_angle"] - piece["angle"]
                if delta_angle > 180: delta_angle -= 360
                elif delta_angle < -180: delta_angle += 360

                # 绘制起点红点、终点黄点与蓝色轨迹线
                img.draw_circle(src_x_px, src_y_px, 5, color=(255, 0, 0), fill=True)
                img.draw_circle(dst_x_px, dst_y_px, 5, color=(255, 255, 0), fill=True)
                img.draw_line(src_x_px, src_y_px, dst_x_px, dst_y_px, color=(0, 0, 255), thickness=2)

                # 标注抓取顺序与数据
                info_text = "%d:P%d d=%.1fpx a=%.1fdeg" % (idx + 1, idx, dist_px, delta_angle)
                mid_x = (src_x_px + dst_x_px) // 2
                mid_y = (src_y_px + dst_y_px) // 2
                img.draw_string(mid_x - 50, mid_y, info_text, color=(255, 255, 255), scale=1)

        else:
            if origin_x is None:
                img.draw_string_advanced(20, 20, 24, "Searching Black A4...", color=(255, 0, 0))
            else:
                img.draw_string_advanced(20, 20, 24, f"Found Pieces: {len(valid_pieces)}/4", color=(255, 255, 0))

        Display.show_image(img)

except Exception as err:
    print("Runtime Exception:", err)

finally:
    if sensor:
        try: sensor.stop()
        except: pass
    if display_inited:
        try: Display.deinit()
        except: pass
    MediaManager.deinit()
    print("System Stopped Safely")
