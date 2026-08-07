# all2-v1.py — K230 发挥题一 (控制逻辑移植自 all1-v2.py)
import time, math, struct, gc, json
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART, FPIOA

# ============================================================
# 1. 配置常量 (与 all1-v2.py 一致)
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

MM_PER_PX_X = 0.4500
MM_PER_PX_Y = 0.5650
A4_LONG_MM  = 297.0
A4_SHORT_MM = 210.0
HALF_LENGTH_PX = int((A4_LONG_MM / 2.0) / MM_PER_PX_X)
A4_BOTTOM_PX   = int(A4_SHORT_MM / MM_PER_PX_Y)

PULSES_PER_MM  = 800.0
Y_DEG_PER_MM   = 360.0 / 113.0
DIST_SCALE_X   = 0.55
DIST_SCALE_Y   = 3.8
THETA_SCALE    = 1.0
MECH_ZERO_X_MM = 270.0
MECH_ZERO_Y_MM = 210.0
Z_SAFE_PULSES  = 18000
Z_DOWN_PULSES  = -18000
MOTOR_RPM      = 280
MOTOR_PPR      = 3200
SERVO_DEG_SEC  = 320.0
MOVE_MARGIN_MS = 1000
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

RECT_W_MIN, RECT_W_MAX = 82.0, 128.0
RECT_H_MIN, RECT_H_MAX = 42.0,  98.0
EDGE_LEN_TOL_ABS = 2.5
EDGE_LEN_TOL_PCT = 0.055

# ============================================================
# 2. 视觉识别 (移植自 all1-v2.py)
# ============================================================
_smooth_corners = None
_smooth_alpha   = 0.2

def rectified_corners_from_blob(blob):
    try:
        cx,cy=blob.cx(),blob.cy(); w,h=blob.w(),blob.h()
        rad=math.radians(blob.rotation_deg()); hw,hh=w/2.0,h/2.0
        c,s=math.cos(rad),math.sin(rad)
        corners=[(cx+ox*c-oy*s,cy+ox*s+oy*c)for ox,oy in[(-hw,-hh),(hw,-hh),(hw,hh),(-hw,hh)]]
        tl=min(corners,key=lambda p:p[0]+p[1]); br=max(corners,key=lambda p:p[0]+p[1])
        tr=max(corners,key=lambda p:p[0]-p[1]); bl=min(corners,key=lambda p:p[0]-p[1])
        return tl,tr,br,bl
    except Exception: return None

def smooth_corners(new_corners):
    global _smooth_corners
    if _smooth_corners is None or len(new_corners)!=4:
        _smooth_corners=tuple(new_corners); return _smooth_corners
    smoothed=[(ox+_smooth_alpha*(nx-ox),oy+_smooth_alpha*(ny-oy))for(nx,ny),(ox,oy)in zip(new_corners,_smooth_corners)]
    _smooth_corners=tuple(smoothed); return _smooth_corners

def find_a4_paper(img):
    global _smooth_corners
    blobs=img.find_blobs([BLACK_A4_TH],pixels_threshold=15000,area_threshold=15000,merge=True,margin=10)
    if not blobs: _smooth_corners=None; return None
    a4_blob=max(blobs,key=lambda b:b.area()); a4_roi=a4_blob.rect()
    rectified=rectified_corners_from_blob(a4_blob)
    raw_pts=a4_blob.corners(); raw_corners=None
    if raw_pts and len(raw_pts)>=4:
        tl_r=min(raw_pts,key=lambda p:p[0]+p[1]); br_r=max(raw_pts,key=lambda p:p[0]+p[1])
        tr_r=max(raw_pts,key=lambda p:p[0]-p[1]); bl_r=min(raw_pts,key=lambda p:p[0]-p[1])
        raw_corners=(tl_r,tr_r,br_r,bl_r)
    if rectified and raw_corners:
        blended=[]
        for i in range(4):
            rx,ry=rectified[i]; nx,ny=raw_corners[i]; d=math.sqrt((rx-nx)**2+(ry-ny)**2)
            if d>25: blended.append((nx*0.15+rx*0.85,ny*0.15+ry*0.85))
            else: blended.append((nx*0.6+rx*0.4,ny*0.6+ry*0.4))
        final_corners=tuple(blended)
    elif raw_corners: final_corners=raw_corners
    elif rectified: final_corners=rectified
    else: _smooth_corners=None; return None
    stable=smooth_corners(final_corners); tl,tr,br,bl=stable; ox,oy=int(tl[0]),int(tl[1])
    img.draw_circle(int(tl[0]),int(tl[1]),8,color=(255,0,255),fill=True)
    img.draw_circle(int(tr[0]),int(tr[1]),8,color=(0,255,0),fill=True)
    img.draw_circle(int(br[0]),int(br[1]),8,color=(0,0,255),fill=True)
    img.draw_circle(int(bl[0]),int(bl[1]),8,color=(255,255,0),fill=True)
    img.draw_line(int(tl[0]),int(tl[1]),int(tr[0]),int(tr[1]),color=(0,255,255),thickness=2)
    img.draw_line(int(tr[0]),int(tr[1]),int(br[0]),int(br[1]),color=(0,255,255),thickness=2)
    img.draw_line(int(br[0]),int(br[1]),int(bl[0]),int(bl[1]),color=(0,255,255),thickness=2)
    img.draw_line(int(bl[0]),int(bl[1]),int(tl[0]),int(tl[1]),color=(0,255,255),thickness=2)
    img.draw_string_advanced(max(0,ox-10),max(0,oy-30),22,"(0,0)",color=(255,0,255))
    lx=ox+HALF_LENGTH_PX; ly=oy+A4_BOTTOM_PX
    img.draw_line(lx,oy,lx,ly,color=(255,255,0),thickness=2)
    img.draw_string_advanced(lx+4,oy+4,18,"%.0fmm"%(A4_LONG_MM/2),color=(255,255,0))
    return ox,oy,a4_roi,stable

def detect_pieces(img,a4_info):
    ox,oy,a4_roi,corners_4=a4_info
    blobs=img.find_blobs([WHITE_PIECE_TH],roi=a4_roi,pixels_threshold=300,area_threshold=300,merge=False)
    if not blobs: return []
    blobs.sort(key=lambda b:b.pixels(),reverse=True); blobs=blobs[:4]
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
# 3. 多边形重建 (发挥题特有)
# ============================================================
def reconstruct_polygon(img,blob,ox,oy):
    roi=blob.rect(); x,y,w_roi,h_roi=roi
    margin=6; rx=max(0,x-margin); ry=max(0,y-margin)
    rw=min(img.width()-rx,w_roi+2*margin); rh=min(img.height()-ry,h_roi+2*margin)
    bcp=blob.corners(); anchors=[[(c[0]-ox)*MM_PER_PX_X,(c[1]-oy)*MM_PER_PX_Y]for c in bcp]
    try:
        lines=img.find_lines(threshold=1200,theta_margin=12,rho_margin=12,roi=(rx,ry,rw,rh))
    except Exception: lines=None
    if lines and len(lines)>=3:
        segs=[]
        for L in lines:
            x1,y1,x2,y2=L.x1(),L.y1(),L.x2(),L.y2()
            theta=math.atan2(y2-y1,x2-x1); length=math.sqrt((x2-x1)**2+(y2-y1)**2)
            segs.append((x1,y1,x2,y2,theta,length))
        merged=_merge_collinear(segs)
        if len(merged)>=3:
            merged.sort(key=lambda s:s[4]); verts_px=[]
            for i in range(len(merged)):
                s1=merged[i]; s2=merged[(i+1)%len(merged)]
                inter=_line_intersection((s1[0],s1[1]),(s1[2],s1[3]),(s2[0],s2[1]),(s2[2],s2[3]))
                if inter:
                    for ax,ay in anchors:
                        apx=ox+ax/MM_PER_PX_X; apy=oy+ay/MM_PER_PX_Y
                        if math.sqrt((inter[0]-apx)**2+(inter[1]-apy)**2)<25:
                            verts_px.append(inter); break
            if len(verts_px)>=3:
                poly=[[(v[0]-ox)*MM_PER_PX_X,(v[1]-oy)*MM_PER_PX_Y]for v in verts_px]
                cx=sum(p[0]for p in poly)/len(poly); cy=sum(p[1]for p in poly)/len(poly)
                poly.sort(key=lambda v:math.atan2(v[1]-cy,v[0]-cx))
                if 3<=len(poly)<=5: return poly[:5]
    cx_a=sum(p[0]for p in anchors)/len(anchors); cy_a=sum(p[1]for p in anchors)/len(anchors)
    anchors.sort(key=lambda p:math.atan2(p[1]-cy_a,p[0]-cx_a))
    return anchors

def _merge_collinear(segs):
    if len(segs)<=1: return segs
    clusters=[]; used=[False]*len(segs)
    for i in range(len(segs)):
        if used[i]: continue
        cl=[segs[i]]; used[i]=True
        for j in range(i+1,len(segs)):
            if used[j]: continue
            diff=abs(segs[i][4]-segs[j][4])
            if diff>math.pi: diff=2*math.pi-diff
            if diff<0.087: cl.append(segs[j]); used[j]=True
        clusters.append(cl)
    return[max(cl,key=lambda s:s[5])for cl in clusters]

def _line_intersection(a1,a2,b1,b2):
    x1,y1=a1; x2,y2=a2; x3,y3=b1; x4,y4=b2
    denom=(x1-x2)*(y3-y4)-(y1-y2)*(x3-x4)
    if abs(denom)<1e-9: return None
    px=((x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4))/denom
    py=((x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4))/denom
    return(px,py)

# ============================================================
# 4. 求解器 (发挥题特有)
# ============================================================
def edge_match_ok(la,lb): return abs(la-lb)<=max(EDGE_LEN_TOL_ABS,EDGE_LEN_TOL_PCT*min(la,lb))

def rigid_edge_align(sa,sb,ta,tb):
    svx,svy=sb[0]-sa[0],sb[1]-sa[1]; tvx,tvy=ta[0]-tb[0],ta[1]-tb[1]
    ang=math.atan2(tvy,tvx)-math.atan2(svy,svx); c,s=math.cos(ang),math.sin(ang)
    rx,ry=c*sa[0]-s*sa[1],s*sa[0]+c*sa[1]; return c,s,tb[0]-rx,tb[1]-ry

def transform_polygon(poly,c,s,tx,ty): return[[c*v[0]-s*v[1]+tx,s*v[0]+c*v[1]+ty]for v in poly]

def polygon_bbox(poly): xs=[v[0]for v in poly]; ys=[v[1]for v in poly]; return min(xs),min(ys),max(xs),max(ys)

def bboxes_overlap(ba,bb,margin=0.5): return ba[0]-margin<bb[2]and ba[2]+margin>bb[0]and ba[1]-margin<bb[3]and ba[3]+margin>bb[1]

def point_in_poly(px,py,poly):
    inside=False; n=len(poly); j=n-1
    for i in range(n):
        xi,yi=poly[i]; xj,yj=poly[j]
        if((yi>py)!=(yj>py))and(px<(xj-xi)*(py-yi)/(yj-yi)+xi): inside=not inside
        j=i
    return inside

def poly_overlap_area(pa,pb,grid=2.0):
    ba,bb=polygon_bbox(pa),polygon_bbox(pb)
    if not bboxes_overlap(ba,bb): return 0.0
    ox1,oy1=max(ba[0],bb[0]),max(ba[1],bb[1]); ox2,oy2=min(ba[2],bb[2]),min(ba[3],bb[3])
    cnt,tot=0,0; y=oy1+grid/2
    while y<oy2:
        x=ox1+grid/2
        while x<ox2:
            tot+=1
            if point_in_poly(x,y,pa)and point_in_poly(x,y,pb): cnt+=1
            x+=grid
        y+=grid
    return cnt*grid*grid if tot>0 else 0.0

def poly_area(poly):
    n=len(poly); s=0.0
    for i in range(n): s+=poly[i][0]*poly[(i+1)%n][1]-poly[(i+1)%n][0]*poly[i][1]
    return abs(s)*0.5

def solve_field_layout(pieces,max_st=30000):
    gc.collect(); n=len(pieces)
    if n==1:
        p=pieces[0]["polygon_mm"]; xs=[v[0]for v in p]; ys=[v[1]for v in p]
        return{"placements":{0:(1,0,0,0)},"width_mm":max(xs)-min(xs),"height_mm":max(ys)-min(ys)}
    edge_data=[]
    for pi,p in enumerate(pieces):
        poly=p["polygon_mm"]; m=len(poly)
        for ei in range(m):
            v0,v1=poly[ei],poly[(ei+1)%m]
            edge_data.append((pi,ei,math.sqrt((v1[0]-v0[0])**2+(v1[1]-v0[1])**2),v0,v1))
    matches={}
    for i in range(len(edge_data)):
        pi,ei,li,_,_=edge_data[i]
        for j in range(len(edge_data)):
            if i==j: continue
            pj,ej,lj,_,_=edge_data[j]
            if pi==pj: continue
            if edge_match_ok(li,lj): matches.setdefault((pi,ei),[]).append((pj,ej))
    candidates=[]; sc=[0]
    def recurse(plc,used,pset):
        sc[0]+=1
        if sc[0]>max_st: return
        if sc[0]%500==0: gc.collect()
        if len(plc)==n:
            aps=[transform_polygon(pieces[pi]["polygon_mm"],*plc[pi])for pi in range(n)]
            score,w,h=evaluate_layout(aps)
            if score<60: candidates.append((score,w,h,dict(plc)))
            return
        for ppi in list(pset):
            ppoly=transform_polygon(pieces[ppi]["polygon_mm"],*plc[ppi]); mp=len(ppoly)
            for ei in range(mp):
                if(ppi,ei)in used: continue
                for pj,ej in matches.get((ppi,ei),[]):
                    if pj in pset or(pj,ej)in used: continue
                    vsa=pieces[pj]["polygon_mm"][ej]
                    vsb=pieces[pj]["polygon_mm"][(ej+1)%len(pieces[pj]["polygon_mm"])]
                    vta=ppoly[ei]; vtb=ppoly[(ei+1)%mp]
                    c,s,tx,ty=rigid_edge_align(vsa,vsb,vta,vtb)
                    npoly=transform_polygon(pieces[pj]["polygon_mm"],c,s,tx,ty)
                    bad=False
                    for opi in pset:
                        opoly=transform_polygon(pieces[opi]["polygon_mm"],*plc[opi])
                        if poly_overlap_area(npoly,opoly)>80.0: bad=True; break
                    if bad: continue
                    at=[transform_polygon(pieces[opi]["polygon_mm"],*plc[opi])for opi in pset]+[npoly]
                    ax=[v[0]for ap in at for v in ap]; ay=[v[1]for ap in at for v in ap]
                    wt,ht=max(ax)-min(ax),max(ay)-min(ay)
                    if wt>RECT_W_MAX+10 or ht>RECT_H_MAX+10: continue
                    plc[pj]=(c,s,tx,ty); used.add((ppi,ei)); used.add((pj,ej)); pset.add(pj)
                    recurse(plc,used,pset)
                    del plc[pj]; used.discard((ppi,ei)); used.discard((pj,ej)); pset.discard(pj)
    recurse({0:(1,0,0,0)},set(),{0})
    if not candidates: return None
    candidates.sort(key=lambda x:x[0]); best=candidates[0]
    return{"placements":best[3],"width_mm":best[1],"height_mm":best[2]}

def evaluate_layout(polys):
    ax=[v[0]for p in polys for v in p]; ay=[v[1]for p in polys for v in p]
    w,h=max(ax)-min(ax),max(ay)-min(ay)
    if w<1 or h<1: return 1e9,w,h
    ta=sum(poly_area(p)for p in polys)
    fe=abs(w*h-ta)/(w*h)
    ov=sum(poly_overlap_area(polys[i],polys[j])for i in range(len(polys))for j in range(i+1,len(polys)))
    ovr=ov/max(ta,1e-6)
    we=max(0,RECT_W_MIN-w,w-RECT_W_MAX)/30.0; he=max(0,RECT_H_MIN-h,h-RECT_H_MAX)/40.0
    return 100.0*fe+300.0*ovr+20.0*(we+he),w,h

# ============================================================
# 5. 求解结果显示
# ============================================================
_solved_cache = None

def draw_solved_layout(img,pieces,solution,ox,oy):
    if solution is None: return
    placements=solution["placements"]
    w_mm=solution["width_mm"]; h_mm=solution["height_mm"]
    tox=A4_LONG_MM/2.0-w_mm/2.0; toy=A4_SHORT_MM*0.75-h_mm/2.0
    rx=int(ox+tox/MM_PER_PX_X); ry=int(oy+toy/MM_PER_PX_Y)
    rw=int(w_mm/MM_PER_PX_X); rh=int(h_mm/MM_PER_PX_Y)
    img.draw_rectangle(rx,ry,rw,rh,color=(0,255,0),thickness=3)
    img.draw_string_advanced(rx,ry-22,20,"TARGET %.0fx%.0fmm"%(w_mm,h_mm),color=(0,255,0))
    colors=[(255,255,0),(255,0,255),(0,255,255),(255,128,0)]
    for pi in range(len(pieces)):
        c,s,tx,ty=placements[pi]
        pw=transform_polygon(pieces[pi]["polygon_mm"],c,s,tx,ty)
        all_x=[v[0]for v in pw]; all_y=[v[1]for v in pw]
        for v in pw:
            sx=int(ox+(v[0]-min(all_x)+tox)/MM_PER_PX_X)
            sy=int(oy+(v[1]-min(all_y)+toy)/MM_PER_PX_Y)
            img.draw_circle(sx,sy,3,color=colors[pi%4],fill=True)

# ============================================================
# 6. 运动规划 (移植自 all1-v2.py, 目标来自求解器)
# ============================================================
def plan_motion_units(pieces,solution):
    placements=solution["placements"]
    w_mm=solution["width_mm"]; h_mm=solution["height_mm"]
    all_world=[]
    for pi in range(len(pieces)):
        c,s,tx,ty=placements[pi]
        all_world.extend(transform_polygon(pieces[pi]["polygon_mm"],c,s,tx,ty))
    min_x=min(p[0]for p in all_world); min_y=min(p[1]for p in all_world)
    tox=A4_LONG_MM/2.0-w_mm/2.0; toy=A4_SHORT_MM*0.75-h_mm/2.0

    units=[]; cur_x_p=ZERO_X_PULSES; cur_y_d=ZERO_Y_DEG; is_first=True
    for piece in pieces:
        pid=piece["id"]; pi=pid-1
        src_x_p=int((MECH_ZERO_X_MM-piece["cx_mm"])*PULSES_PER_MM*DIST_SCALE_X)
        src_y_d=(MECH_ZERO_Y_MM-piece["cy_mm"])*Y_DEG_PER_MM*DIST_SCALE_Y
        c,s,tx,ty=placements[pi]
        pw=transform_polygon(piece["polygon_mm"],c,s,tx,ty)
        tcx=sum(v[0]for v in pw)/len(pw)-min_x+tox
        tcy=sum(v[1]for v in pw)/len(pw)-min_y+toy
        tgt_x_p=int((MECH_ZERO_X_MM-tcx)*PULSES_PER_MM*DIST_SCALE_X)
        tgt_y_d=(MECH_ZERO_Y_MM-tcy)*Y_DEG_PER_MM*DIST_SCALE_Y
        raw_rot=-piece["angle_deg"]
        if raw_rot>180: raw_rot-=360
        elif raw_rot<-180: raw_rot+=360
        rot_deg=raw_rot*THETA_SCALE

        dx_to_src=src_x_p-cur_x_p; dy_to_src=src_y_d-cur_y_d
        dx_src_to_dst=tgt_x_p-src_x_p; dy_src_to_dst=tgt_y_d-src_y_d
        dx_dst_to_zero=ZERO_X_PULSES-tgt_x_p; dy_dst_to_zero=ZERO_Y_DEG-tgt_y_d
        steps=[]
        if is_first:
            steps.append({"x_pulses":0,"z_pulses":Z_SAFE_PULSES,"y_deg":0,"theta_deg":0.0,"magnet":False,"desc":"P%d-1:Z↑"%pid})
            is_first=False
        steps+=[
            {"x_pulses":dx_to_src,     "z_pulses":0,"y_deg":dy_to_src,     "theta_deg":0.0,    "magnet":False,"desc":"P%d-2:→碎片(%.0f,%.0f)mm"%(pid,piece["cx_mm"],piece["cy_mm"])},
            {"x_pulses":0,             "z_pulses":Z_DOWN_PULSES,"y_deg":0, "theta_deg":0.0,    "magnet":False,"desc":"P%d-3a:Z↓"%pid},
            {"x_pulses":0,             "z_pulses":0,"y_deg":0,             "theta_deg":0.0,    "magnet":True, "desc":"P%d-3b:吸合"%pid},
            {"x_pulses":0,             "z_pulses":Z_SAFE_PULSES,"y_deg":0,"theta_deg":0.0,    "magnet":True, "desc":"P%d-4:抬起"%pid},
            {"x_pulses":dx_src_to_dst, "z_pulses":0,"y_deg":dy_src_to_dst,"theta_deg":0.0,    "magnet":True, "desc":"P%d-5:→目标(%.0f,%.0f)mm"%(pid,tcx,tcy)},
            {"x_pulses":0,             "z_pulses":0,"y_deg":0,             "theta_deg":rot_deg,"magnet":True, "desc":"P%d-5b:旋转%.1f°"%(pid,rot_deg)},
            {"x_pulses":0,             "z_pulses":Z_DOWN_PULSES,"y_deg":0,"theta_deg":0.0,    "magnet":True, "desc":"P%d-6a:Z↓放置"%pid},
            {"x_pulses":0,             "z_pulses":0,"y_deg":0,             "theta_deg":0.0,    "magnet":False,"desc":"P%d-6b:释放"%pid},
            {"x_pulses":0,             "z_pulses":Z_SAFE_PULSES,"y_deg":0,"theta_deg":0.0,    "magnet":False,"desc":"P%d-7:空载抬起"%pid},
            {"x_pulses":dx_dst_to_zero,"z_pulses":0,"y_deg":dy_dst_to_zero,"theta_deg":0.0,   "magnet":False,"desc":"P%d-8:回零"%pid},
        ]
        cur_x_p=ZERO_X_PULSES; cur_y_d=ZERO_Y_DEG
        units.append({"piece_id":pid,"steps":steps})
    return units

# ============================================================
# 7. 通讯 (与 all1-v2.py 一致)
# ============================================================
def init_uart():
    fpioa=FPIOA(); fpioa.set_function(UART_TX,FPIOA.UART1_TXD); fpioa.set_function(UART_RX,FPIOA.UART1_RXD)
    return UART(UART.UART1,baudrate=115200,bits=UART.EIGHTBITS,parity=UART.PARITY_NONE,stop=UART.STOPBITS_ONE)

def send_packet(uart,x_p,z_p,y_deg,t_deg,magnet_on=False):
    y_i=int(y_deg*10); t_i=int(t_deg*10)
    useful=struct.pack("<2i2h",int(x_p),int(z_p),y_i,t_i)
    payload=useful+bytes([0x01 if magnet_on else 0x00])+bytes([0x00]*19)
    cksum=0
    for b in payload: cksum^=b
    uart.write(b"\xA5"+payload+bytes([cksum&0xFF]))

def send_all_units(uart,units):
    print("\n"+"="*50); print("[SEND] 运动指令发送")
    total=sum(len(u["steps"])for u in units); sent=0
    for u in units:
        print("\n--- 碎片 P%d ---"%u["piece_id"])
        for s in u["steps"]:
            send_packet(uart,s["x_pulses"],s["z_pulses"],s["y_deg"],s["theta_deg"],magnet_on=s["magnet"])
            sent+=1; ms="吸住" if s["magnet"] else "松开"
            print("  [%2d/%2d] %s"%(sent,total,s["desc"]))
            print("         增量 → X:%+7d | Z:%+6d | Y:%+7.1f° | θ:%+6.1f° | 磁铁:%s"%(
                int(s["x_pulses"]),int(s["z_pulses"]),s["y_deg"],s["theta_deg"],ms))
            is_mo=(s["x_pulses"]==0 and s["z_pulses"]==0 and s["y_deg"]==0 and s["theta_deg"]==0)
            if is_mo: delay=MAGNET_DELAY_MS
            else:
                rps=MOTOR_RPM*MOTOR_PPR/60.0
                tx=abs(s["x_pulses"])/rps if rps>0 else 0; tz=abs(s["z_pulses"])/rps if rps>0 else 0
                ty=abs(s["y_deg"])/SERVO_DEG_SEC if SERVO_DEG_SEC>0 else 0
                tt=abs(s["theta_deg"])/SERVO_DEG_SEC if SERVO_DEG_SEC>0 else 0
                delay=int(max(tx,tz,ty,tt)*1000)+MOVE_MARGIN_MS
            time.sleep_ms(delay)
        if PIECE_DELAY_MS>0: time.sleep_ms(PIECE_DELAY_MS)
    print("\n[DONE] 全部发送完毕, 共%d包"%sent); print("="*50+"\n")

# ============================================================
# 8. 主程序 (移植自 all1-v2.py, 增加求解器)
# ============================================================
def main():
    print("="*40); print("K230 发挥题一 (控制逻辑移植自 all1-v2)"); print("="*40)
    print("[INIT] 摄像头...")
    sensor=Sensor(width=W,height=H); sensor.reset(); sensor.set_framesize(width=W,height=H)
    sensor.set_pixformat(Sensor.RGB565)
    print("[INIT] 显示器...")
    Display.init(Display.ST7701,width=W,height=H,to_ide=True); MediaManager.init(); sensor.run()
    time.sleep(0.5)
    print("[INIT] UART..."); uart=init_uart()
    state="DETECT"; prev_snap=None; stable_ms=0; locked=None
    global _solved_cache
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
                ox,oy=a4_info[0],a4_info[1]
                if _solved_cache: draw_solved_layout(img,locked,_solved_cache,ox,oy)
                if locked:
                    for p in locked:
                        txt="P%d:V%d (%.0f,%.0f)mm %.0f°"%(p["id"],len(p.get("polygon_mm",[])),p["cx_mm"],p["cy_mm"],p["angle_deg"])
                        img.draw_string_advanced(10,10+p["id"]*22,18,txt,color=(0,255,0))
                if _solved_cache:
                    img.draw_string_advanced(10,H-25,20,"SOLVED %.0fx%.0fmm"%(_solved_cache["width_mm"],_solved_cache["height_mm"]),color=(0,255,0))
                Display.show_image(img); time.sleep_ms(100); continue
            if len(pieces)<2:
                img.draw_string_advanced(10,10,22,"Pieces: %d (need >=2)"%len(pieces),color=(255,255,0))
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
                    ox,oy,a4_roi,corners_4=a4_info
                    for p in locked:
                        blob=p.get("blob")
                        if blob: p["polygon_mm"]=reconstruct_polygon(img,blob,ox,oy)
                        else: p["polygon_mm"]=[[p["cx_mm"]-10,p["cy_mm"]-10],[p["cx_mm"]+10,p["cy_mm"]-10],[p["cx_mm"]+10,p["cy_mm"]+10],[p["cx_mm"]-10,p["cy_mm"]+10]]
                    print("\n=== 碎片已锁存 (%.1fs) ==="%elapsed)
                    for p in locked: print("  P%d: V%d (%.1f,%.1f)mm %.1f°"%(p["id"],len(p["polygon_mm"]),p["cx_mm"],p["cy_mm"],p["angle_deg"]))
                    print("[SOLVER] 求解中..."); t0=time.ticks_ms()
                    solution=solve_field_layout(locked)
                    dt=time.ticks_diff(time.ticks_ms(),t0)
                    if solution is None: print("[SOLVER] 失败!"); state="DETECT"; prev_snap=None; stable_ms=0; continue
                    _solved_cache=solution
                    print("[SOLVER] 成功! %.1f×%.1fmm (%.1fs)"%(solution["width_mm"],solution["height_mm"],dt/1000.0))
                    send_all_units(uart,plan_motion_units(locked,solution))
                    state="DONE"; continue
                state="STABILIZING"
            if state=="STABILIZING":
                elapsed=time.ticks_diff(now_ms,stable_ms)/1000.0
                pct=min(100,int(elapsed/STABLE_SEC*100)); bar_w=int(200*pct/100)
                img.draw_rectangle(10,H-30,200,16,color=(80,80,80),fill=True)
                img.draw_rectangle(10,H-30,bar_w,16,color=(0,255,0),fill=True)
                img.draw_string_advanced(220,H-32,18,"%.1fs/%ds"%(elapsed,STABLE_SEC),color=(0,255,0))
            elif state=="DETECT": img.draw_string_advanced(10,10,22,"%d pieces - stabilizing..."%len(pieces),color=(255,255,0))
            Display.show_image(img); time.sleep_ms(30); gc.collect()
    except KeyboardInterrupt: print("中断")
    except Exception as e: print("异常:",e); import sys; sys.print_exception(e)
    finally: sensor.stop(); Display.deinit(); MediaManager.deinit(); print("结束")

if __name__=="__main__": main()
