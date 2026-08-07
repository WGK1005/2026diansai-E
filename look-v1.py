import time, math
from media.sensor import *
from media.display import *
from media.media import *

# ========== 基本参数 ==========
W, H = 800, 480

# =====================================================================
#                          【核心终极优化】：LAB 纯色提取
# =====================================================================
# 根据 RGB 直方图推导：放宽亮度 L (15~75) 适应反光和阴影；
# 死死卡住 A 和 B 的色偏。B_max 设为 5，彻底过滤 B值很高的黄棕色底板！
BLACK_A4_TH = (15, 75, -10, 10, -15, 5)

# 白色碎片阈值 (保持不变)
WHITE_TH = (75, 100, -30, 15, -30, 15)

# =====================================================================
#                          系统标定参数
# =====================================================================
MM_PER_PX_X = 0.4243
MM_PER_PX_Y = 0.5330

A4_LONG_MM = 297.0
A4_SHORT_MM = 210.0

HALF_LENGTH_PX_OFFSET = int((A4_LONG_MM / 2.0) / MM_PER_PX_X)
A4_BOTTOM_PX_OFFSET = int(A4_SHORT_MM / MM_PER_PX_Y)

def init_hw():
    s = Sensor(width=W, height=H)
    s.reset()
    s.set_framesize(width=W, height=H)
    s.set_pixformat(Sensor.RGB565)

    Display.init(Display.ST7701, width=W, height=H, to_ide=True)
    MediaManager.init()
    s.run()
    time.sleep(0.5)
    return s

def main():
    print("识别模式：防连片合并 + 色相排除法锁定 A4 纸")
    cam = init_hw()
    clock = time.clock()

    try:
        while True:
            clock.tick()
            img = cam.snapshot()

            origin_x, origin_y = None, None
            a4_roi = None

            # =============================================================
            # 第一步：锁定黑色 A4 纸并提取 4 个实际顶角
            # =============================================================
            black_blobs = img.find_blobs([BLACK_A4_TH], pixels_threshold=15000, area_threshold=15000, merge=True, margin=10)

            if black_blobs:
                a4_blob = max(black_blobs, key=lambda b: b.area())
                a4_roi = a4_blob.rect()

                pts = a4_blob.corners()
                if pts and len(pts) >= 4:
                    # 几何极值计算真正的 4 个纸张角点
                    tl = min(pts, key=lambda p: p[0] + p[1]) # 左上
                    br = max(pts, key=lambda p: p[0] + p[1]) # 右下
                    tr = max(pts, key=lambda p: p[0] - p[1]) # 右上
                    bl = min(pts, key=lambda p: p[0] - p[1]) # 左下

                    origin_x, origin_y = tl[0], tl[1]

                    # 绘制角点标记与四边形外框
                    img.draw_circle(tl[0], tl[1], 8, color=(255, 0, 255), fill=True) # 洋红: TL (0,0)
                    img.draw_circle(tr[0], tr[1], 8, color=(0, 255, 0), fill=True)   # 绿色: TR
                    img.draw_circle(br[0], br[1], 8, color=(0, 0, 255), fill=True)   # 蓝色: BR
                    img.draw_circle(bl[0], bl[1], 8, color=(255, 255, 0), fill=True) # 黄色: BL

                    img.draw_line(tl[0], tl[1], tr[0], tr[1], color=(0, 255, 255), thickness=2)
                    img.draw_line(tr[0], tr[1], br[0], br[1], color=(0, 255, 255), thickness=2)
                    img.draw_line(br[0], br[1], bl[0], bl[1], color=(0, 255, 255), thickness=2)
                    img.draw_line(bl[0], bl[1], tl[0], tl[1], color=(0, 255, 255), thickness=2)

                    img.draw_string_advanced(max(0, origin_x - 10), max(0, origin_y - 30), 22, "(0,0)", color=(255, 0, 255))

                    # 绘制中线参考
                    line_x = origin_x + HALF_LENGTH_PX_OFFSET
                    line_y_end = origin_y + A4_BOTTOM_PX_OFFSET
                    img.draw_line(line_x, origin_y, line_x, line_y_end, color=(255, 255, 0), thickness=2)
                else:
                    origin_x, origin_y = a4_blob.x(), a4_blob.y()
                    img.draw_rectangle(a4_roi, color=(100, 100, 100), thickness=2)

            # =============================================================
            # 第二步：碎片识别（merge=False 彻底防止多米诺合并）
            # =============================================================
            if origin_x is not None and a4_roi is not None:
                # 关键：merge=False！每个独立碎片单独形成 Blob，不触发包围盒合并
                white_blobs = img.find_blobs([WHITE_TH], roi=a4_roi, pixels_threshold=300, area_threshold=300, merge=False)

                if white_blobs:
                    # 按面积从大到小排序，取前 4 个有效碎片
                    white_blobs.sort(key=lambda blob: blob.pixels(), reverse=True)
                    valid_pieces = white_blobs[:4]

                    for idx, blob in enumerate(valid_pieces):
                        cx, cy = blob.cx(), blob.cy()

                        pts = blob.corners()
                        max_dist_sq = 0
                        far_point = (cx, cy)
                        for p in pts:
                            dx, dy = p[0] - cx, p[1] - cy
                            dist_sq = dx * dx + dy * dy
                            if dist_sq > max_dist_sq:
                                max_dist_sq = dist_sq
                                far_point = p

                        angle_rad = math.atan2(far_point[0] - cx, -(far_point[1] - cy))
                        angle_deg = math.degrees(angle_rad)
                        if angle_deg < 0:
                            angle_deg += 360.0

                        # 绘制单个碎片
                        img.draw_rectangle(blob.rect(), color=(0, 255, 0), thickness=2)
                        img.draw_cross(cx, cy, color=(0, 255, 0), size=8, thickness=2)
                        img.draw_line(cx, cy, far_point[0], far_point[1], color=(255, 255, 0), thickness=2)
                        img.draw_circle(far_point[0], far_point[1], 5, color=(0, 0, 255), fill=True)

                        # 计算绝对物理坐标（毫米）
                        real_x_mm = (cx - origin_x) * MM_PER_PX_X
                        real_y_mm = (cy - origin_y) * MM_PER_PX_Y

                        text = f"No.{idx+1} ({real_x_mm:.1f}mm,{real_y_mm:.1f}mm) A:{angle_deg:.0f}"
                        text_x = max(0, blob.x())
                        text_y = max(0, blob.y() - 22)
                        img.draw_string_advanced(text_x, text_y, 18, text, color=(255, 255, 0))

            else:
                img.draw_string_advanced(20, 20, 24, "Searching Black A4...", color=(255, 0, 0))

            Display.show_image(img)
            time.sleep_ms(10)

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
