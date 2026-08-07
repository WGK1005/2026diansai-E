import time
import math
import struct
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART, FPIOA

# ===================== 1. FPIOA 引脚复用与串口初始化 =====================
# 配置引脚 3 (TX) 和 引脚 4 (RX)
fp = FPIOA()
try:
    fp.set_function(3, FPIOA.UART1_TXD)
    fp.set_function(4, FPIOA.UART1_RXD)
except Exception as e:
    print("FPIOA 引脚设置警报:", e)

# 初始化 UART1
uart = UART(1, baudrate=115200)

RED_THRESHOLD = (0, 100, 19, 127, 5, 127)

A4_W_MM = 210.0
A4_H_MM = 297.0
MM_PER_PIX_X = A4_W_MM / 320.0
MM_PER_PIX_Y = A4_H_MM / 240.0

TARGET_CENTER_X = 105.0
TARGET_CENTER_Y = 220.0

PRESET_SLOTS = [
    {"id": 1, "dx": 20.0, "dy": -15.0, "target_angle": 0.0},
    {"id": 2, "dx": -30.0, "dy": -20.0, "target_angle": 12.0},
    {"id": 3, "dx": -25.0, "dy": 15.0, "target_angle": -5.0},
    {"id": 4, "dx": 30.0, "dy": 20.0, "target_angle": -135.0}
]

def send_cmd(piece_id, src_x, src_y, src_ang, dst_x, dst_y, dst_ang):
    header = b"\x5A\xA5"
    tail = b"\xED"
    payload = struct.pack(
        "<Bhhhhhh",
        int(piece_id),
        int(src_x),
        int(src_y),
        int(src_ang),
        int(dst_x),
        int(dst_y),
        int(dst_ang)
    )
    checksum = 0
    for b in payload:
        checksum ^= b
    uart.write(header + payload + bytes([checksum]) + tail)

def get_sort_key(p):
    return (p["num_corners"], p["area"])

sensor = None
display_inited = False

try:
    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(Sensor.QVGA)
    sensor.set_pixformat(Sensor.RGB565)

    Display.init(Display.VIRT, width=320, height=240, to_ide=True)
    display_inited = True

    MediaManager.init()
    sensor.run()

    clock = time.clock()

    while True:
        clock.tick()
        img = sensor.snapshot()

        roi_top = (0, 0, 320, 120)

        blobs = img.find_blobs(
            [RED_THRESHOLD],
            roi=roi_top,
            pixels_threshold=150,
            area_threshold=150,
            merge=False
        )

        valid_pieces = []

        if blobs:
            for blob in blobs:
                cx = blob.cx()
                cy = blob.cy()

                poly_points = blob.corners()
                max_dist_sq = -1
                far_point = (cx, cy)

                for p in poly_points:
                    dx = p[0] - cx
                    dy = p[1] - cy
                    dist_sq = dx * dx + dy * dy
                    if dist_sq > max_dist_sq:
                        max_dist_sq = dist_sq
                        far_point = p

                vx = far_point[0] - cx
                vy = far_point[1] - cy

                angle_rad = math.atan2(vx, -vy)
                angle_deg = math.degrees(angle_rad)
                if angle_deg < 0:
                    angle_deg += 360.0

                phys_x = cx * MM_PER_PIX_X
                phys_y = cy * MM_PER_PIX_Y
                phys_area = blob.area() * MM_PER_PIX_X * MM_PER_PIX_Y

                piece_data = {
                    "cx": phys_x,
                    "cy": phys_y,
                    "angle": angle_deg,
                    "area": phys_area,
                    "blob": blob,
                    "far_point": far_point,
                    "num_corners": len(poly_points)
                }
                valid_pieces.append(piece_data)

        if len(valid_pieces) == 4:[cite: 1]
            valid_pieces.sort(key=get_sort_key, reverse=True)

            for idx in range(4):
                piece = valid_pieces[idx]
                slot = PRESET_SLOTS[idx]

                dst_x = TARGET_CENTER_X + slot["dx"]
                dst_y = TARGET_CENTER_Y + slot["dy"]
                dst_angle = slot["target_angle"]

                b = piece["blob"]
                cx_pix = int(piece["cx"] / MM_PER_PIX_X)
                cy_pix = int(piece["cy"] / MM_PER_PIX_Y)
                far_px = piece["far_point"]

                img.draw_rectangle(b.rect(), color=(0, 255, 0))
                img.draw_cross(cx_pix, cy_pix, color=(0, 255, 0), size=5)
                img.draw_line(cx_pix, cy_pix, far_px[0], far_px[1], color=(255, 255, 0), thickness=2)
                img.draw_circle(far_px[0], far_px[1], 4, color=(0, 0, 255), fill=True)

                text = "ID:" + str(slot['id']) + " A:" + str(int(piece['angle']))
                img.draw_string(b.x(), b.y() - 12, text, color=(255, 255, 255), scale=1)

                send_cmd(slot["id"], piece["cx"], piece["cy"], piece["angle"], dst_x, dst_y, dst_angle)

        Display.show_image(img)
        print("FPS: %.2f" % clock.fps())

except Exception as err:
    print("Error:", err)

finally:
    if sensor:
        try:
            sensor.stop()
        except:
            pass
    if display_inited:
        try:
            Display.deinit()
        except:
            pass
    MediaManager.deinit()
    print("Stopped")
