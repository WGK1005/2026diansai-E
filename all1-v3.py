# all1-v2.py — 相对位置版: 吸到碎片直接去目标, 放置后回零 (整合v1的详细识别矩形流程)
import time, math, struct, gc, json
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART, FPIOA

# ============================================================
# 1. 配置常量
# ============================================================
W, H = 800, 480

try:
    with open("lab_config.json", "r") as f:
        _cfg = json.load(f)
        BLACK_A4_TH   = tuple(_cfg[0])
        WHITE_PIECE_TH = tuple(_cfg[1])
        print("已加载阈值配置: 黑纸%s 白碎片%s" % (BLACK_A4_TH, WHITE_PIECE_TH))
except Exception:
    BLACK_A4_TH   = (15, 75, -10, 10, -15, 5)
    WHITE_PIECE_TH = (75, 100, -30, 15, -30, 15)
    print("使用默认阈值")

MM_PER_PX_X = 0.4243
MM_PER_PX_Y = 0.5330
A4_LONG_MM  = 297.0
A4_SHORT_MM = 210.0
HALF_LENGTH_PX = int((A4_LONG_MM / 2.0) / MM_PER_PX_X)
A4_BOTTOM_PX   = int(A4_SHORT_MM / MM_PER_PX_Y)

PULSES_PER_MM  = 800.0
Y_DEG_PER_MM   = 360.0 / 113.0
DIST_SCALE_X   = 0.55
DIST_SCALE_Y   = 3.8
MECH_ZERO_X_MM = 270.0 # X=0时磁铁在A4 X=270, 正=右移纸
MECH_ZERO_Y_MM = 210.0 # Y=0时磁铁在A4 Y=210(右下角), 正=伸向纸上
Z_SAFE_PULSES  = 16000
Z_DOWN_PULSES  = -16000
MOTOR_RPM      = 250
MOTOR_PPR      = 3200
SERVO_DEG_SEC  = 320.0
MOVE_MARGIN_MS = 1500
MAGNET_DELAY_MS = 500
PIECE_DELAY_MS  = 1000

ZERO_X_PULSES = 0
ZERO_Y_DEG    = 0.0
ZERO_Z_PULSES = 0

STABLE_SEC       = 3.5
STABLE_DIST_MM   = 10.0
STABLE_ANGLE_DEG = 12.0

UART_TX = 3
UART_RX = 4

FIXED_TARGETS = [
    {"id": 1, "x_mm": 200.0, "y_mm": 86.0,  "angle_deg": 206.0},
    {"id": 2, "x_mm": 235.0, "y_mm": 112.0, "angle_deg": 16.0},
    {"id": 3, "x_mm": 210.0, "y_mm": 118.0, "angle_deg": 22.0},
    {"id": 4, "x_mm": 193.0, "y_mm": 136.0, "angle_deg": 20.0},
]

# ============================================================
# 2. 视觉识别
# ============================================================
_smooth_corners = None
_smooth_alpha   = 0.2

def rectified_corners_from_blob(blob):
    """
    从blob的聚合属性(中心+尺寸+旋转角)重建完美矩形四角。
    不依赖单个像素角点, 对光线变化极其稳定。
    返回 (tl, tr, br, bl) 或 None
    """
    try:
        cx = blob.cx()
        cy = blob.cy()
        w = blob.w()
        h = blob.h()
        angle_deg = blob.rotation_deg()
        angle_rad = math.radians(angle_deg)

        hw, hh = w / 2.0, h / 2.0
        offsets = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]

        c = math.cos(angle_rad)
        s = math.sin(angle_rad)

        corners = []
        for ox_off, oy_off in offsets:
            rx = cx + ox_off * c - oy_off * s
            ry = cy + ox_off * s + oy_off * c
            corners.append((rx, ry))

        tl = min(corners, key=lambda p: p[0] + p[1])
        br = max(corners, key=lambda p: p[0] + p[1])
        tr = max(corners, key=lambda p: p[0] - p[1])
        bl = min(corners, key=lambda p: p[0] - p[1])

        return tl, tr, br, bl
    except Exception:
        return None

def smooth_corners(new_corners):
    """EMA平滑: 融合新检测角点与历史角点, 消除帧间闪烁"""
    global _smooth_corners
    if _smooth_corners is None or len(new_corners) != 4:
        _smooth_corners = tuple(new_corners)
        return _smooth_corners

    smoothed = []
    for i in range(4):
        nx, ny = new_corners[i]
        ox, oy = _smooth_corners[i]
        sx = ox + _smooth_alpha * (nx - ox)
        sy = oy + _smooth_alpha * (ny - oy)
        smoothed.append((sx, sy))

    _smooth_corners = tuple(smoothed)
    return _smooth_corners

def find_a4_paper(img):
    """
    抗畸变 + 矩形拟合 + 角点平滑。
    1. find_blobs 检测黑纸
    2. 用blob中心/尺寸/旋转角重建理想矩形角点 (抗光线)
    3. 与原始角点融合做EMA平滑 (抗闪烁)
    4. 极端情况回退到原始角点
    返回 (ox, oy, a4_roi, corners_4) 或 None
    """
    global _smooth_corners

    blobs = img.find_blobs(
        [BLACK_A4_TH],
        pixels_threshold=15000,
        area_threshold=15000,
        merge=True,
        margin=10,
    )
    if not blobs:
        _smooth_corners = None
        return None

    a4_blob = max(blobs, key=lambda b: b.area())
    a4_roi = a4_blob.rect()

    # --- 方案A: 矩形拟合 (基于blob聚合属性, 最稳定) ---
    rectified = rectified_corners_from_blob(a4_blob)

    # --- 方案B: 原始角点 (回退) ---
    raw_pts = a4_blob.corners()
    raw_corners = None
    if raw_pts and len(raw_pts) >= 4:
        tl_r = min(raw_pts, key=lambda p: p[0] + p[1])
        br_r = max(raw_pts, key=lambda p: p[0] + p[1])
        tr_r = max(raw_pts, key=lambda p: p[0] - p[1])
        bl_r = min(raw_pts, key=lambda p: p[0] - p[1])
        raw_corners = (tl_r, tr_r, br_r, bl_r)

    # --- 融合: 矩形拟合仅作辅助, 以原始角点为主 ---
    if rectified is not None and raw_corners is not None:
        blended = []
        for i in range(4):
            rx, ry = rectified[i]
            nx, ny = raw_corners[i]
            dist = math.sqrt((rx - nx) ** 2 + (ry - ny) ** 2)
            if dist > 25:
                # 偏离>25px: 角点丢失, 高度信任拟合矩形
                bx = nx * 0.15 + rx * 0.85
                by = ny * 0.15 + ry * 0.85
            else:
                # 正常: 拟合矩形占40%拉稳, 原始占60%保留细节
                bx = nx * 0.6 + rx * 0.4
                by = ny * 0.6 + ry * 0.4
            blended.append((bx, by))
        final_corners = tuple(blended)
    elif raw_corners is not None:
        final_corners = raw_corners
    elif rectified is not None:
        final_corners = rectified
    else:
        _smooth_corners = None
        return None

    # --- EMA平滑: 跨帧消除闪烁 ---
    stable = smooth_corners(final_corners)
    tl, tr, br, bl = stable
    ox, oy = int(tl[0]), int(tl[1])

    # --- 绘制 ---
    img.draw_circle(
        int(tl[0]), int(tl[1]), 8, color=(255, 0, 255), fill=True
    )  # 洋红 TL
    img.draw_circle(int(tr[0]), int(tr[1]), 8, color=(0, 255, 0), fill=True)  # 绿色 TR
    img.draw_circle(int(br[0]), int(br[1]), 8, color=(0, 0, 255), fill=True)  # 蓝色 BR
    img.draw_circle(
        int(bl[0]), int(bl[1]), 8, color=(255, 255, 0), fill=True
    )  # 黄色 BL
    img.draw_line(
        int(tl[0]), int(tl[1]), int(tr[0]), int(tr[1]), color=(0, 255, 255), thickness=2
    )
    img.draw_line(
        int(tr[0]), int(tr[1]), int(br[0]), int(br[1]), color=(0, 255, 255), thickness=2
    )
    img.draw_line(
        int(br[0]), int(br[1]), int(bl[0]), int(bl[1]), color=(0, 255, 255), thickness=2
    )
    img.draw_line(
        int(bl[0]), int(bl[1]), int(tl[0]), int(tl[1]), color=(0, 255, 255), thickness=2
    )
    img.draw_string_advanced(
        max(0, ox - 10), max(0, oy - 30), 22, "(0,0)", color=(255, 0, 255)
    )
    # 中线
    line_x = ox + HALF_LENGTH_PX
    line_y_end = oy + A4_BOTTOM_PX
    img.draw_line(line_x, oy, line_x, line_y_end, color=(255, 255, 0), thickness=2)
    img.draw_string_advanced(
        line_x + 4, oy + 4, 18, "%.0fmm" % (A4_LONG_MM / 2), color=(255, 255, 0)
    )

    return ox, oy, a4_roi, stable

def detect_pieces(img, a4_info):
    ox,oy,a4_roi,corners_4=a4_info
    blobs=img.find_blobs([WHITE_PIECE_TH],roi=a4_roi,pixels_threshold=300,area_threshold=300,merge=False)
    if not blobs: return []
    blobs.sort(key=lambda b:b.pixels(),reverse=True)
    blobs=blobs[:4]
    pieces=[]
    for idx,blob in enumerate(blobs):
        cx_px,cy_px=blob.cx(),blob.cy()
        cx_mm=(cx_px-ox)*MM_PER_PX_X; cy_mm=(cy_px-oy)*MM_PER_PX_Y
        pts=blob.corners(); max_d2=0; far_pt=(cx_px,cy_px)
        for p in pts:
            dx=p[0]-cx_px; dy=p[1]-cy_px; d2=dx*dx+dy*dy
            if d2>max_d2: max_d2=d2; far_pt=(p[0],p[1])
        ad=math.degrees(math.atan2(far_pt[0]-cx_px,-(far_pt[1]-cy_px)))
        if ad<0: ad+=360.0
        img.draw_rectangle(blob.rect(),color=(0,255,0),thickness=2)
        img.draw_cross(int(cx_px),int(cy_px),color=(0,255,0),size=8,thickness=2)
        img.draw_line(int(cx_px),int(cy_px),int(far_pt[0]),int(far_pt[1]),color=(255,255,0),thickness=2)
        img.draw_circle(int(far_pt[0]),int(far_pt[1]),5,color=(0,0,255),fill=True)
        t="No.%d (%.1f,%.1f)mm A:%.0f"%(idx+1,cx_mm,cy_mm,ad)
        img.draw_string_advanced(max(0,blob.x()),max(0,blob.y()-22),18,t,color=(255,255,0))
        pieces.append({"id":idx+1,"cx_mm":cx_mm,"cy_mm":cy_mm,"angle_deg":ad,"cx_px":cx_px,"cy_px":cy_px,"blob":blob})
    return pieces

def pieces_stable(c,p):
    if p is None or len(c)!=len(p): return False
    for cp,pp in zip(c,p):
        if cp["id"]!=pp["id"]: return False
        d=math.sqrt((cp["cx_mm"]-pp["cx_mm"])**2+(cp["cy_mm"]-pp["cy_mm"])**2)
        ad=abs(cp["angle_deg"]-pp["angle_deg"])
        if ad>180: ad=360-ad
        if d>STABLE_DIST_MM or ad>STABLE_ANGLE_DEG: return False
    return True

def pieces_snapshot(pieces):
    return[{"id":p["id"],"cx_mm":p["cx_mm"],"cy_mm":p["cy_mm"],"angle_deg":p["angle_deg"]}for p in pieces]

# ============================================================
# 3. 运动规划 — 相对位置版: src→dst直连
# ============================================================
def plan_motion_units(pieces):
    """
    8步流程:
      1. Z↑ (仅第一片)  2. XY→抓取  3a.Z↓ 3b.吸合  4.Z↑
      5. XY→目标 (直连!) 6a.Z↓ 6b.释放  7.Z↑  8.XY→回零
    """
    units=[]
    cur_x_p=ZERO_X_PULSES
    cur_y_d=ZERO_Y_DEG
    is_first=True

    for piece in pieces:
        pid=piece["id"]
        src_x_p=int((MECH_ZERO_X_MM-piece["cx_mm"])*PULSES_PER_MM*DIST_SCALE_X)
        src_y_d=(MECH_ZERO_Y_MM-piece["cy_mm"])*Y_DEG_PER_MM*DIST_SCALE_Y
        tgt=FIXED_TARGETS[pid-1]
        tgt_x_p=int((MECH_ZERO_X_MM-tgt["x_mm"])*PULSES_PER_MM*DIST_SCALE_X)
        tgt_y_d=(MECH_ZERO_Y_MM-tgt["y_mm"])*Y_DEG_PER_MM*DIST_SCALE_Y
        rot_deg=tgt["angle_deg"]-piece["angle_deg"]

        dx_to_src=src_x_p-cur_x_p
        dy_to_src=src_y_d-cur_y_d
        dx_src_to_dst=tgt_x_p-src_x_p      # 直连!
        dy_src_to_dst=tgt_y_d-src_y_d
        dx_dst_to_zero=ZERO_X_PULSES-tgt_x_p
        dy_dst_to_zero=ZERO_Y_DEG-tgt_y_d

        steps=[]
        if is_first:
            steps.append({"x_pulses":0,"z_pulses":Z_SAFE_PULSES,"y_deg":0,"theta_deg":0.0,"magnet":False,"desc":"P%d-1:Z↑"%pid})
            is_first=False

        steps+=[
            {"x_pulses":dx_to_src,     "z_pulses":0,"y_deg":dy_to_src,     "theta_deg":0.0,    "magnet":False,"desc":"P%d-2:→碎片(%.0f,%.0f)mm"%(pid,piece["cx_mm"],piece["cy_mm"])},
            {"x_pulses":0,             "z_pulses":Z_DOWN_PULSES,"y_deg":0, "theta_deg":0.0,    "magnet":False,"desc":"P%d-3a:Z↓"%pid},
            {"x_pulses":0,             "z_pulses":0,"y_deg":0,             "theta_deg":0.0,    "magnet":True, "desc":"P%d-3b:吸合"%pid},
            {"x_pulses":0,             "z_pulses":Z_SAFE_PULSES,"y_deg":0,"theta_deg":0.0,    "magnet":True, "desc":"P%d-4:抬起"%pid},
            {"x_pulses":dx_src_to_dst, "z_pulses":0,"y_deg":dy_src_to_dst,"theta_deg":rot_deg,"magnet":True, "desc":"P%d-5:→目标(%.0f,%.0f)mm θ%.1f°"%(pid,tgt["x_mm"],tgt["y_mm"],rot_deg)},
            {"x_pulses":0,             "z_pulses":Z_DOWN_PULSES,"y_deg":0,"theta_deg":0.0,    "magnet":True, "desc":"P%d-6a:Z↓放置"%pid},
            {"x_pulses":0,             "z_pulses":0,"y_deg":0,             "theta_deg":0.0,    "magnet":False,"desc":"P%d-6b:释放"%pid},
            {"x_pulses":0,             "z_pulses":Z_SAFE_PULSES,"y_deg":0,"theta_deg":0.0,    "magnet":False,"desc":"P%d-7:空载抬起"%pid},
            {"x_pulses":dx_dst_to_zero,"z_pulses":0,"y_deg":dy_dst_to_zero,"theta_deg":0.0,   "magnet":False,"desc":"P%d-8:回零"%pid},
        ]
        cur_x_p=ZERO_X_PULSES
        cur_y_d=ZERO_Y_DEG
        units.append({"piece_id":pid,"steps":steps})
    return units

# ============================================================
# 4. 通讯
# ============================================================
def init_uart():
    fpioa=FPIOA()
    fpioa.set_function(UART_TX,FPIOA.UART1_TXD)
    fpioa.set_function(UART_RX,FPIOA.UART1_RXD)
    return UART(UART.UART1,baudrate=115200,bits=UART.EIGHTBITS,parity=UART.PARITY_NONE,stop=UART.STOPBITS_ONE)

def send_packet(uart,x_p,z_p,y_deg,t_deg,magnet_on=False):
    y_i=int(y_deg*10); t_i=int(t_deg*10)
    useful=struct.pack("<2i2h",int(x_p),int(z_p),y_i,t_i)
    payload=useful+bytes([0x01 if magnet_on else 0x00])+bytes([0x00]*19)
    cksum=0
    for b in payload: cksum^=b
    uart.write(b"\xA5"+payload+bytes([cksum&0xFF]))

def send_all_units(uart,units):
    print("\n"+"="*50)
    print("[SEND] 相对位置运动指令发送")
    total=sum(len(u["steps"])for u in units)
    sent=0
    for u in units:
        print("\n--- 碎片 P%d ---"%u["piece_id"])
        for s in u["steps"]:
            send_packet(uart,s["x_pulses"],s["z_pulses"],s["y_deg"],s["theta_deg"],magnet_on=s["magnet"])
            sent+=1
            ms="吸住" if s["magnet"] else "松开"
            print("  [%2d/%2d] %s"%(sent,total,s["desc"]))
            print("         增量 → X:%+7d | Z:%+6d | Y:%+7.1f° | θ:%+6.1f° | 磁铁:%s"%(
                int(s["x_pulses"]),int(s["z_pulses"]),s["y_deg"],s["theta_deg"],ms))
            is_mo=(s["x_pulses"]==0 and s["z_pulses"]==0 and s["y_deg"]==0 and s["theta_deg"]==0)
            if is_mo: delay=MAGNET_DELAY_MS
            else:
                rps=MOTOR_RPM*MOTOR_PPR/60.0
                tx=abs(s["x_pulses"])/rps if rps>0 else 0
                tz=abs(s["z_pulses"])/rps if rps>0 else 0
                ty=abs(s["y_deg"])/SERVO_DEG_SEC if SERVO_DEG_SEC>0 else 0
                tt=abs(s["theta_deg"])/SERVO_DEG_SEC if SERVO_DEG_SEC>0 else 0
                delay=int(max(tx,tz,ty,tt)*1000)+MOVE_MARGIN_MS
            time.sleep_ms(delay)
        if PIECE_DELAY_MS>0: time.sleep_ms(PIECE_DELAY_MS)
    print("\n[DONE] 全部发送完毕, 共%d包"%sent)
    print("="*50+"\n")

# ============================================================
# 5. 主程序
# ============================================================
def main():
    print("="*40)
    print("K230 拼图装置 v2 (相对位置: src→dst直连)")
    print("="*40)
    print("[INIT] 摄像头...")
    sensor=Sensor(width=W,height=H)
    sensor.reset(); sensor.set_framesize(width=W,height=H); sensor.set_pixformat(Sensor.RGB565)
    print("[INIT] 显示器...")
    Display.init(Display.ST7701,width=W,height=H,to_ide=True)
    MediaManager.init(); sensor.run(); time.sleep(0.5)
    print("[INIT] UART...")
    uart=init_uart()
    state="DETECT"; prev_snap=None; stable_ms=0; locked=None
    print("[READY]")
    try:
        while True:
            img=sensor.snapshot(); now_ms=time.ticks_ms()
            a4_info=find_a4_paper(img)
            if a4_info is None:
                img.draw_string_advanced(20,20,24,"Searching Black A4...",color=(255,0,0))
                Display.show_image(img); time.sleep_ms(30); continue
            pieces=detect_pieces(img,a4_info)
            if state=="DONE":
                if locked:
                    for p in locked:
                        tgt=FIXED_TARGETS[p["id"]-1]; rot=tgt["angle_deg"]-p["angle_deg"]
                        txt="P%d: (%.0f,%.0f)mm %.0f° → (%.0f,%.0f)mm θ%.0f°"%(
                            p["id"],p["cx_mm"],p["cy_mm"],p["angle_deg"],tgt["x_mm"],tgt["y_mm"],rot)
                        img.draw_string_advanced(10,10+p["id"]*24,20,txt,color=(0,255,0))
                img.draw_string_advanced(10,H-25,22,"DONE",color=(0,255,0))
                Display.show_image(img); time.sleep_ms(100); continue
            if len(pieces)!=4:
                img.draw_string_advanced(10,10,22,"Pieces: %d/4"%len(pieces),color=(255,255,0))
                state="DETECT"; prev_snap=None; stable_ms=0
                Display.show_image(img); time.sleep_ms(30); continue
            snap=pieces_snapshot(pieces)
            if not pieces_stable(snap,prev_snap):
                prev_snap=snap; stable_ms=now_ms
                if state=="STABILIZING": state="DETECT"
            else:
                elapsed=time.ticks_diff(now_ms,stable_ms)/1000.0
                if elapsed>=STABLE_SEC:
                    locked=[dict(p)for p in pieces]
                    print("\n=== 碎片已锁存 (%.1fs) ==="%elapsed)
                    for p in locked:
                        tgt=FIXED_TARGETS[p["id"]-1]; rot=tgt["angle_deg"]-p["angle_deg"]
                        print("  P%d: 源(%.1f,%.1f)mm %.1f° → 目标(%.1f,%.1f)mm 旋转%.1f°"%(
                            p["id"],p["cx_mm"],p["cy_mm"],p["angle_deg"],tgt["x_mm"],tgt["y_mm"],rot))
                    send_all_units(uart,plan_motion_units(locked))
                    state="DONE"; continue
                state="STABILIZING"
            if state=="STABILIZING":
                elapsed=time.ticks_diff(now_ms,stable_ms)/1000.0
                pct=min(100,int(elapsed/STABLE_SEC*100)); bar_w=int(200*pct/100)
                img.draw_rectangle(10,H-30,200,16,color=(80,80,80),fill=True)
                img.draw_rectangle(10,H-30,bar_w,16,color=(0,255,0),fill=True)
                img.draw_string_advanced(220,H-32,18,"%.1fs/%ds"%(elapsed,STABLE_SEC),color=(0,255,0))
            elif state=="DETECT":
                img.draw_string_advanced(10,10,22,"4 pieces - stabilizing...",color=(255,255,0))
            Display.show_image(img); time.sleep_ms(30); gc.collect()
    except KeyboardInterrupt: print("中断")
    except Exception as e: print("异常:",e); import sys; sys.print_exception(e)
    finally: sensor.stop(); Display.deinit(); MediaManager.deinit(); print("结束")

if __name__=="__main__": main()
