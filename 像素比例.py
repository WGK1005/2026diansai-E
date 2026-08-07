import time
import math
from media.sensor import *
from media.display import *
from media.media import *

# =====================================================================
#                          1. 参数配置
# =====================================================================
W, H = 800, 480

# 黑色 A4 纸/基准板的真实物理尺寸 (单位：毫米)
A4_LONG_MM = 297.0
A4_SHORT_MM = 210.0

# 【修改阈值】：收紧 A 和 B 的范围，防止把棕色背景识别进去
# 黑色的 A 和 B 通常在 0 附近，而棕色的 A 和 B 偏向正数 (15~40左右)
BLACK_A4_TH = (0, 45, -15, 15, -15, 15)

# =====================================================================
#                          2. 主测试程序
# =====================================================================
def main():
    print(">>> (真实角点法) 比例标定程序启动 <<<")

    sensor = Sensor(width=W, height=H)
    sensor.reset()
    sensor.set_framesize(width=W, height=H)
    sensor.set_pixformat(Sensor.RGB565)

    Display.init(Display.ST7701, width=W, height=H, to_ide=True)
    MediaManager.init()
    sensor.run()
    time.sleep(1.0)

    try:
        while True:
            img = sensor.snapshot()

            # 寻找黑色块
            blobs = img.find_blobs([BLACK_A4_TH], pixels_threshold=15000, area_threshold=15000, merge=True)

            if blobs:
                # 找到最大的黑色色块
                a4_b = max(blobs, key=lambda b: b.area())

                # 【核心修改】：提取真正的四个角点 (随着纸张倾斜而倾斜)
                corners = a4_b.corners()

                if corners and len(corners) == 4:
                    # 对四个角点进行排序：左上(tl), 右上(tr), 右下(br), 左下(bl)
                    # 依据：x+y 最小为左上，最大为右下；x-y 最大为右上，最小为左下
                    tl = min(corners, key=lambda p: p[0] + p[1])
                    br = max(corners, key=lambda p: p[0] + p[1])
                    tr = max(corners, key=lambda p: p[0] - p[1])
                    bl = min(corners, key=lambda p: p[0] - p[1])

                    # 利用欧氏距离计算真实的像素宽和高
                    w_px_top = math.sqrt((tr[0] - tl[0])**2 + (tr[1] - tl[1])**2)
                    w_px_bottom = math.sqrt((br[0] - bl[0])**2 + (br[1] - bl[1])**2)
                    w_px = (w_px_top + w_px_bottom) / 2.0  # 取平均值更精确

                    h_px_left = math.sqrt((bl[0] - tl[0])**2 + (bl[1] - tl[1])**2)
                    h_px_right = math.sqrt((br[0] - tr[0])**2 + (br[1] - tr[1])**2)
                    h_px = (h_px_left + h_px_right) / 2.0  # 取平均值更精确

                    if w_px > 0 and h_px > 0:
                        # 判断是横放还是竖放
                        if w_px > h_px:
                            real_w, real_h = A4_LONG_MM, A4_SHORT_MM
                            orientation = "横放"
                        else:
                            real_w, real_h = A4_SHORT_MM, A4_LONG_MM
                            orientation = "竖放"

                        # 核心算式：1 像素代表多少毫米 (mm/px)
                        mm_per_px_x = real_w / w_px
                        mm_per_px_y = real_h / h_px

                        # --- 可视化绘制 ---
                        # 连接四个角点画出真实边框
                        img.draw_line(tl[0], tl[1], tr[0], tr[1], color=(255, 255, 0), thickness=3)
                        img.draw_line(tr[0], tr[1], br[0], br[1], color=(255, 255, 0), thickness=3)
                        img.draw_line(br[0], br[1], bl[0], bl[1], color=(255, 255, 0), thickness=3)
                        img.draw_line(bl[0], bl[1], tl[0], tl[1], color=(255, 255, 0), thickness=3)

                        # 标出四个角点 (左上标红点作为原点，其他标蓝点)
                        img.draw_circle(tl[0], tl[1], 8, color=(255, 0, 0), fill=True)
                        img.draw_circle(tr[0], tr[1], 6, color=(0, 0, 255), fill=True)
                        img.draw_circle(br[0], br[1], 6, color=(0, 0, 255), fill=True)
                        img.draw_circle(bl[0], bl[1], 6, color=(0, 0, 255), fill=True)

                        # 屏幕打印数据 (使用黄色字体)
                        str_w = "Px W : " + str(int(w_px)) + " px"
                        str_h = "Px H : " + str(int(h_px)) + " px"
                        str_k_x = "K_X : 1px = " + str(round(mm_per_px_x, 4)) + " mm"
                        str_k_y = "K_Y : 1px = " + str(round(mm_per_px_y, 4)) + " mm"

                        img.draw_string_advanced(20, 20, 22, str_w, color=(255, 255, 0))
                        img.draw_string_advanced(20, 50, 22, str_h, color=(255, 255, 0))
                        img.draw_string_advanced(20, 80, 26, str_k_x, color=(0, 255, 0))
                        img.draw_string_advanced(20, 115, 26, str_k_y, color=(0, 255, 0))

                        # 串口控制台打印
                        print(f"[{orientation}] X轴比例: {mm_per_px_x:.4f} mm/px | Y轴比例: {mm_per_px_y:.4f} mm/px")

            else:
                img.draw_string_advanced(20, 20, 24, "Searching Black Board...", color=(255, 255, 0))

            Display.show_image(img)
            time.sleep_ms(30)

    except KeyboardInterrupt:
        pass
    finally:
        sensor.stop()
        Display.deinit()
        MediaManager.deinit()
        print("测试结束")

if __name__ == "__main__":
    main()
