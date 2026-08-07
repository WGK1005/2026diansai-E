# 旋转.py — K230 拼图碎片旋转角度检测完整方案
# 从 look-v1.py / look-v3.py / look1-v3.py / all1-v1.py 中提取整合
import math

# ============================================================
# 方案A: 最远角点法 (所有look系列代码统一使用)
# 原理: 取blob旋转外接矩形的4个角点, 找离质心最远的角点,
#       质心→最远角点的向量方向即为碎片"指向"方向
# ============================================================

def angle_by_farthest_corner(blob):
    """
    最远角点法: 计算碎片指向角度
    输入: CanMV find_blobs() 返回的blob对象
    返回: angle_deg (0~360), far_point (px, py), corners
    """
    cx = blob.cx()
    cy = blob.cy()
    corners = blob.corners()  # 4个旋转外接矩形角点

    max_dist_sq = -1.0
    far_point = (cx, cy)

    for p in corners:
        dx = p[0] - cx
        dy = p[1] - cy
        dist_sq = dx * dx + dy * dy
        if dist_sq > max_dist_sq:
            max_dist_sq = dist_sq
            far_point = (p[0], p[1])

    vx = far_point[0] - cx
    vy = far_point[1] - cy

    # atan2(vx, -vy): 以图像"向上"(-Y)为0°, 顺时针为正
    angle_rad = math.atan2(vx, -vy)
    angle_deg = math.degrees(angle_rad)
    if angle_deg < 0:
        angle_deg += 360.0

    return angle_deg, far_point, corners


def angle_by_farthest_corner_from_points(cx, cy, corners):
    """
    最远角点法(纯坐标版本, 不依赖blob对象)
    cx, cy: 质心坐标 (px)
    corners: [(x1,y1), (x2,y2), (x3,y3), (x4,y4)] 角点列表
    返回: angle_deg (0~360), far_point, far_idx
    """
    max_dist_sq = -1.0
    far_point = (cx, cy)
    far_idx = 0

    for i, p in enumerate(corners):
        dx = p[0] - cx
        dy = p[1] - cy
        dist_sq = dx * dx + dy * dy
        if dist_sq > max_dist_sq:
            max_dist_sq = dist_sq
            far_point = (p[0], p[1])
            far_idx = i

    vx = far_point[0] - cx
    vy = far_point[1] - cy
    angle_rad = math.atan2(vx, -vy)
    angle_deg = math.degrees(angle_rad)
    if angle_deg < 0:
        angle_deg += 360.0

    return angle_deg, far_point, far_idx


# ============================================================
# 方案B: 多边形内角法 (用于找"最尖锐角")
# 原理: 从多边形顶点计算每个内角, 最小内角 = 最尖锐角
# ============================================================

def polygon_interior_angles(pts):
    """
    计算多边形所有内角
    pts: [[x,y], ...] 多边形顶点 (CCW顺序)
    返回: [(angle_deg, vertex_index), ...] 按角度升序
    """
    n = len(pts)
    angles = []
    for i in range(n):
        prev = pts[(i - 1) % n]
        curr = pts[i]
        nxt  = pts[(i + 1) % n]
        # 两条边向量
        v1 = (prev[0] - curr[0], prev[1] - curr[1])
        v2 = (nxt[0]  - curr[0], nxt[1]  - curr[1])
        dot   = v1[0] * v2[0] + v1[1] * v2[1]
        cross = v1[0] * v2[1] - v1[1] * v2[0]
        angle = math.degrees(math.atan2(abs(cross), dot))
        angles.append((angle, i, curr))
    angles.sort(key=lambda x: x[0])
    return angles


def sharpest_angle_info(pts):
    """
    获取多边形最尖锐角的信息
    返回: (angle_deg, vertex_index, vertex_coords)
    """
    angles = polygon_interior_angles(pts)
    return angles[0] if angles else (0, 0, pts[0])


# ============================================================
# 方案C: 最长边角度法 (原版 puzzle_core/geometry.py 使用)
# 原理: 多边形的"朝向"由最长边的方向决定
# ============================================================

def longest_edge_angle(pts):
    """
    最长边角度法: 碎片朝向 = 最长边的方向角
    pts: [[x,y], ...] 多边形顶点
    返回: angle_deg (0~180)
    """
    n = len(pts)
    max_len = -1.0
    best_edge = None
    for i in range(n):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % n]
        length = math.hypot(x1 - x0, y1 - y0)
        if length > max_len:
            max_len = length
            best_edge = (x0, y0, x1, y1)

    if best_edge is None:
        return 0.0

    x0, y0, x1, y1 = best_edge
    angle = math.degrees(math.atan2(y1 - y0, x1 - x0))
    # 归一化到 [0, 180) (线段无方向性)
    return angle % 180.0


# ============================================================
# 方案D: 综合角度 (组合最远角点 + 最长边)
# 在blob检测碎片后, 先重建多边形顶点, 再综合判断角度
# ============================================================

def combined_angle_from_blob(blob, corner_points_mm=None):
    """
    综合角度检测:
    1. 用最远角点法得主方向 angle_a
    2. 如果有mm坐标的多边形顶点, 用最长边法得 angle_b
    3. 用内角法得最尖锐角 sharpest

    返回: {
        'angle_far_corner': deg,      # 最远角点方向 (0~360)
        'angle_longest_edge': deg,    # 最长边方向 (0~180)
        'sharpest_angle': deg,         # 最尖锐内角
        'sharpest_vertex_idx': int,    # 最尖锐角顶点索引
        'polygon_vertex_count': int,   # 多边形顶点数
    }
    """
    result = {}

    # 方案A: 最远角点
    angle_a, far_pt, corners = angle_by_farthest_corner(blob)
    result['angle_far_corner'] = angle_a
    result['far_point'] = far_pt
    result['corners'] = corners

    # 方案C + B: 如果有mm多边形顶点
    if corner_points_mm is not None and len(corner_points_mm) >= 3:
        result['angle_longest_edge'] = longest_edge_angle(corner_points_mm)

        angles = polygon_interior_angles(corner_points_mm)
        if angles:
            result['sharpest_angle'] = angles[0][0]
            result['sharpest_vertex_idx'] = angles[0][1]
            result['sharpest_vertex_coords'] = angles[0][2]
        result['polygon_vertex_count'] = len(corner_points_mm)
    else:
        result['angle_longest_edge'] = 0.0
        result['sharpest_angle'] = 0.0
        result['sharpest_vertex_idx'] = 0
        result['polygon_vertex_count'] = 4  # blob.corners() 固定4个

    return result


# ============================================================
# 工具函数: blob角点 → mm坐标多边形
# ============================================================

def blob_corners_to_mm(corners_px, origin_x, origin_y, kx, ky):
    """
    将blob像素角点转为mm坐标多边形
    corners_px: [(x,y), ...] 像素角点
    origin_x, origin_y: A4纸原点像素坐标
    kx, ky: mm/px 比例
    返回: [[x_mm, y_mm], ...]
    """
    poly_mm = []
    for cp in corners_px:
        mx = (cp[0] - origin_x) * kx
        my = (cp[1] - origin_y) * ky
        poly_mm.append([mx, my])
    # 确保CCW
    if polygon_signed_area(poly_mm) < 0:
        poly_mm = poly_mm[::-1]
    return poly_mm


def polygon_signed_area(pts):
    """鞋带公式有向面积 (CCW为正)"""
    n = len(pts)
    s = 0.0
    for i in range(n):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % n]
        s += x0 * y1 - x1 * y0
    return s * 0.5


# ============================================================
# 演示: 如果直接运行此文件(在PC上测试数学逻辑)
# ============================================================

if __name__ == "__main__":
    print("=" * 50)
    print("旋转角度检测方案测试 (纯数学验证)")
    print("=" * 50)

    # 模拟一个碎片多边形 (mm坐标, 模拟自备模板第4片)
    test_poly = [
        [20.0, 0.0],
        [100.0, 0.0],
        [100.0, 60.0],
        [76.0, 42.0],
        [36.0, 12.0],
    ]

    print("\n测试多边形顶点:", test_poly)
    print("多边形顶点数:", len(test_poly))

    # 方案B: 内角
    angles = polygon_interior_angles(test_poly)
    print("\n--- 方案B: 内角分析 ---")
    for ang, idx, vtx in angles:
        print("  顶点%d (%.0f,%.0f): 内角 %.1f°" % (idx, vtx[0], vtx[1], ang))
    sharpest = angles[0]
    print("  ★ 最尖锐角: 顶点%d, %.1f°" % (sharpest[1], sharpest[0]))

    # 方案C: 最长边
    le_angle = longest_edge_angle(test_poly)
    print("\n--- 方案C: 最长边角度 ---")
    print("  最长边方向: %.1f°" % le_angle)

    # 模拟方案A (需要blob对象, 用质心+角点模拟)
    cx = sum(p[0] for p in test_poly) / len(test_poly)
    cy = sum(p[1] for p in test_poly) / len(test_poly)
    print("\n--- 质心 ---")
    print("  质心: (%.1f, %.1f)" % (cx, cy))

    # 找最远顶点模拟"最远角点法"
    max_d = 0
    far = test_poly[0]
    for p in test_poly:
        d = (p[0]-cx)**2 + (p[1]-cy)**2
        if d > max_d:
            max_d = d
            far = p
    vx = far[0] - cx
    vy = far[1] - cy
    ang = math.degrees(math.atan2(vx, -vy))
    if ang < 0:
        ang += 360.0
    print("\n--- 方案A模拟: 最远顶点法 ---")
    print("  最远顶点: (%.1f, %.1f)" % (far[0], far[1]))
    print("  方向角: %.1f°" % ang)

    print("\n" + "=" * 50)
    print("测试完毕")
