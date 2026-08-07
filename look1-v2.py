import time, math
from media.sensor import *
from media.display import *
from media.media import *

# =====================================================================
#                          1. 全局配置与参数
# =====================================================================
W, H = 800, 480  # 800x480 分辨率

# A4 纸真实物理尺寸 (mm)
A4_WIDTH_MM = 297.0
A4_HEIGHT_MM = 210.0

# ---------------------------------------------------------------------
# 【超宽红色 A4 纸阈值】 (兼容暗光、鲜红与深红)
# L (5~100): 宽容极暗与极亮
# A (10~127): 强行覆盖鲜红(高A) 与 深红(中高A)
# B (-30~60): 容忍偏褐色/暗红
# ---------------------------------------------------------------------
RED_A4_WIDE_TH = (5, 100, 10, 127, -30, 60)

# 【白色碎片阈值】
WHITE_TH = (55, 100, -20, 20, -20, 20)

# =====================================================================
#                          2. 核心解算函数
# =====================================================================

def get_perspective_matrix(src_pts, dst_w_mm, dst_h_mm):
    """计算像素到毫米的映射系数"""
    tl, tr, br, bl = src_pts[0], src_pts[1], src_pts[2], src_pts[3]
    w_px = (math.sqrt((tr[0]-tl[0])**2 + (tr[1]-tl[1])**2) + math.sqrt((br[0]-bl[0])**2 + (br[1]-bl[1])**2)) / 2.0
    h_px = (math.sqrt((bl[0]-tl[0])**2 + (bl[1]-tl[1])**2) + math.sqrt((br[0]-tr[0])**2 + (br[1]-tr[1])**2)) / 2.0

    k_x = dst_w_mm / float(w_px) if w_px > 0 else 1.0
    k_y = dst_h_mm / float(h_px) if h_px > 0 else 1.0
    return tl, k_x, k_y

def pixel_to_real_mm(cx, cy, origin_pt, k_x, k_y):
    """像素坐标转 A4 纸绝对毫米坐标 (以 A4 纸左上角 TL 为零点)"""
    real_x = (cx - origin_pt[0]) * k_x
    real_y = (cy - origin_pt[1]) * k_y
    return real_x, real_y

# =====================================================================
#                          3. 主程序
# =====================================================================

def init_hw():
    s = Sensor(width=W, height=H)
    s.reset()
    s.set_framesize(width=W, height=H)
    s.set_pixformat(Sensor.RGB565)

    Display.init(Display.ST7701, width=W, height=H, to_ide=True)
    MediaManager.init()
    s.run()
    time.sleep(1.0)
    return s

def main():
    print(">>> 暗光多色调红底 A4 纸 + 碎片识别程序启动 <<<")
    cam = init_hw()

    try:
        while True:
            img = cam.snapshot()

            # -------------------------------------------------------------
            # 1. 寻找红色 A4 纸建立基准坐标系 (防误判机制)
            # -------------------------------------------------------------
            # 设置较高的像素门槛 (pixels_threshold=20000)，确保碎片绝不可能被当作 A4 纸
            a4_blobs = img.find_blobs([RED_A4_WIDE_TH], pixels_threshold=20000, area_threshold=20000, merge=True)

            origin_pt = None
            valid_a4 = False

            if a4_blobs:
                a4_b = max(a4_blobs, key=lambda b: b.area())

                # 双重校验：A4 纸的像素宽度必须占整个画面一定比例
                if a4_b.w() > W * 0.35:
                    pts = a4_b.corners()

                    if len(pts) >= 4:
                        cx_a4, cy_a4 = a4_b.cx(), a4_b.cy()
                        tl = min(pts, key=lambda p: (p[0] - cx_a4) + (p[1] - cy_a4))
                        tr = max(pts, key=lambda p: (p[0] - cx_a4) - (p[1] - cy_a4))
                        br = max(pts, key=lambda p: (p[0] - cx_a4) + (p[1] - cy_a4))
                        bl = min(pts, key=lambda p: (p[0] - cx_a4) - (p[1] - cy_a4))
                        A4_CORNERS = [tl, tr, br, bl]

                        origin_pt, k_x, k_y = get_perspective_matrix(A4_CORNERS, A4_WIDTH_MM, A4_HEIGHT_MM)
                        valid_a4 = True

                        # 绘制真实的红色 A4 纸边界轮廓 (黄色线)
                        for i in range(4):
                            p_a = A4_CORNERS[i]
                            p_b = A4_CORNERS[(i + 1) % 4]
                            img.draw_line(p_a[0], p_a[1], p_b[0], p_b[1], color=(255, 255, 0), thickness=3)

                        # 锁定并绘制 A4 纸左上角为坐标原点 (0,0)
                        img.draw_circle(tl[0], tl[1], 8, color=(255, 0, 0), fill=True)
                        img.draw_string_advanced(max(0, tl[0] - 20), max(0, tl[1] - 30), 24, "(0,0) [mm]", color=(255, 0, 0))

            # -------------------------------------------------------------
            # 2. 在锁定的 A4 纸基准下提取白色碎片
            # -------------------------------------------------------------
            if valid_a4:
                # 限制只在 A4 纸外框 ROI 内部寻找碎片
                a4_rect = a4_b.rect()
                blobs = img.find_blobs([WHITE_TH], roi=a4_rect, pixels_threshold=100, area_threshold=100, merge=False)

                valid_count = 0

                if blobs:
                    # 按像素从大到小排序
                    blobs.sort(key=lambda blob: blob.pixels(), reverse=True)

                    # 过滤掉背景杂讯，只取最突出的碎片
                    valid_blobs = [b for b in blobs if b.area() < a4_b.area() * 0.5]

                    for idx, blob in enumerate(valid_blobs[:4]):
                        valid_count += 1
                        cx, cy = blob.cx(), blob.cy()

                        # 计算顶点与姿态角
                        pts_p = blob.corners()
                        max_dist_sq = -1
                        far_point = (cx, cy)
                        for p in pts_p:
                            dist_sq = (p[0] - cx)**2 + (p[1] - cy)**2
                            if dist_sq > max_dist_sq:
                                max_dist_sq = dist_sq
                                far_point = p

                        angle_rad = math.atan2(far_point[0] - cx, -(far_point[1] - cy))
                        angle_deg = math.degrees(angle_rad)
                        if angle_deg < 0:
                            angle_deg += 360.0

                        # 换算相对于 A4 纸零点 (0,0) 的毫米坐标
                        real_cx_mm, real_cy_mm = pixel_to_real_mm(cx, cy, origin_pt, k_x, k_y)

                        # 可视化标注
                        img.draw_rectangle(blob.rect(), color=(0, 255, 0), thickness=2)
                        img.draw_cross(cx, cy, color=(0, 255, 0), size=8, thickness=2)
                        img.draw_circle(far_point[0], far_point[1], 6, color=(0, 0, 255), fill=True)

                        # 标注碎片编号及毫米物理坐标
                        text_str = "No." + str(valid_count) + " (" + str(round(real_cx_mm, 1)) + "," + str(round(real_cy_mm, 1)) + ")"

                        text_x = max(0, blob.x())
                        text_y = max(0, blob.y() - 25)
                        img.draw_string_advanced(text_x, text_y, 20, text_str, color=(255, 255, 255))

            else:
                prompt_msg = "SEARCHING RED A4 PAPER..."
                img.draw_string_advanced(20, 20, 24, prompt_msg, color=(255, 0, 0))

            Display.show_image(img)
            time.sleep_ms(5)

    except KeyboardInterrupt:
        pass
    finally:
        if 'cam' in locals():
            cam.stop()
        Display.deinit()
        MediaManager.deinit()
        print("程序停止")

if __name__ == "__main__":
    main()
