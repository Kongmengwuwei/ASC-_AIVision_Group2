# 双色小车追踪，追求更高速度
# 固定网格打点取色（12x16采样 -> 顺时针旋转90°输出16x12）
# 主控发送：
#   START\n   -> 识别一次整图并发送 $MAP ... $END
#   CARPOS\n  -> 发送一次连续小车坐标 Cx100,y100
#   CARINIT\n -> 连续发送小车坐标
#   CARSTOP\n -> 停止连续发送
#
# 连续坐标定义：
#   原点：左下角障碍 '#' 的中心
#   x正方向：向右
#   y正方向：向上
#   例如：(3.3, 5.2) -> 串口发送 C330,520\r\n

import sensor
import time
from machine import UART
import math

# ===================== 运行模式开关 =====================
IDLE_SNAPSHOT = True      # True：空闲时也抓图，IDE持续刷新；False：更省资源
DEBUG_OVERLAY = True      # True：画采样点（慢）
OVERLAY_EVERY = 2
PRINT_UART = False
PRINT_MAP = False
PRINT_TX = True
PRINT_CAR = True

# ===================== 动态小车框选开关 =====================
DRAW_CAR_BOX = False        # True：动态定位时，把小车色块外接矩形画出来
DRAW_CAR_CENTER = False     # True：动态定位时，画小车色块中心点
DRAW_TRACK_ROI = False     # True：调试时画搜索ROI
CAR_BOX_COLOR = (0, 255, 0)
CAR_CENTER_COLOR = (255, 0, 0)
TRACK_ROI_COLOR = (255, 255, 0)

# ===================== 相机初始化 =====================
sensor.reset()
sensor.set_framerate(60)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)   # 320x240
sensor.set_brightness(120)#亮度设置

sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

clock = time.clock()
#plus用的是12
uart = UART(12, baudrate=115200)

IMG_W = 320
IMG_H = 240

# ===================== 最终发送地图尺寸（16行12列） =====================
ROWS = 16
COLS = 12

# ===================== 发送裁边开关 =====================
SEND_TRIM_OUTER_WALL = True   # True：发送时去掉最外一圈固定障碍

# ===================== 相机画面采样网格尺寸（12行16列） =====================
S_ROWS = 12
S_COLS = 16
N_SAMPLES = S_ROWS * S_COLS

# ===================== 网格几何参数（你已调好） =====================
GRID_X0 = 61.0
GRID_Y0 = 61.0
GRID_DX = 13.9
GRID_DY = 13.3
GRID_ROT_DEG = 0.0
DIST_K1 = 0.0

# ===================== 倾斜/透视矫正（推荐） =====================
# 说明：不做整帧重采样，而是把采样网格/格中心投影到屏幕四边形上。
# 对于当前这种“固定网格打点取色 + 动态按格定位”的方案，这种几何矫正
# 比整帧图像扭正更省算力，而且对后续小车定位同样有效。
USE_PERSPECTIVE_RECT = True
DRAW_RECTIFY_QUAD = True
RECT_QUAD_COLOR = (0, 255, 255)

# 下面四个角点，对应“未旋转采样网格”的外接四角：
# TL: 左上, TR: 右上, BR: 右下, BL: 左下
# 默认值由原始 GRID_X0/Y0/DX/DY 推出；若屏幕有倾斜，请实测后微调。
RECT_TL_X = 62.0
RECT_TL_Y = 62.0
RECT_TR_X = 261.0
RECT_TR_Y = 62.0
RECT_BR_X = 261.0
RECT_BR_Y = 202.0
RECT_BL_X = 62.0
RECT_BL_Y = 202.0

# ===================== 方向修正 =====================
OUTPUT_ROTATE_CW_90 = True
SEND_FLIP_ROWS = False
SEND_FLIP_COLS = False

# ===================== 投票参数 =====================
VOTE_FRAMES = 8
VOTE_MIN_RATIO = 0.50

# ===================== 邻域取色参数 =====================
PATCH_RAD = 0             # 0=单点；1=3x3；2=5x5
PATCH_USE_MEDIAN = False

# ===================== RGB参考色（你已调好） =====================
# 类别顺序固定：0 '.' 1 '#' 2 'B' 3 'T' 4 'C' 5 'D'
CLASSES = ['.', '#', 'B', 'T', 'C', 'D']
REF_R = [33, 110, 148, 231,  40, 195]
REF_G = [12,  80, 135,   0, 140,  10]
REF_B = [255,120,  10, 255, 120,  54]
REF_T = [80,120, 100, 100, 140, 100]
N_CLS = 6



# ===================== 基准点显示开关 =====================
DRAW_ORIGIN_POINT = False
ORIGIN_POINT_COLOR = (255, 0, 255)   # 紫红色
ORIGIN_TEXT_COLOR  = (255, 255, 0)   # 黄色



# ===================== 墙体判别参数 =====================
WALL_DEV_T = 45
WALL_MEAN_MIN = 12
WALL_MEAN_MAX = 220


# ===================== 连续小车定位参数（find_blobs版：蓝半块 + 绿半块） =====================
# 注意：下面两个阈值需要你现场微调。
#CAR_BLUE_TH  = (48, 72, -30, 5, -60, -10)   # 蓝半块阈值（先用这组试）
#CAR_GREEN_TH = (31, 83, -79, -19, -11, 85)    # 绿半块阈值（先用这组试）
CAR_BLUE_TH  = (65, 85, -53, 4, -67, -14) #换屏幕后的蓝绿阈值亮度为120
CAR_GREEN_TH = (51, 81, -90, -49, -6, 80)



CAR_BLOB_X_STRIDE = 2        # 越大越快，越容易漏
CAR_BLOB_Y_STRIDE = 2
CAR_BLOB_PIXELS_MIN = 15     # 最小像素数
CAR_BLOB_AREA_MIN = 15       # 最小面积
CAR_BLOB_MERGE_MARGIN = 4    # 蓝绿两块靠近时允许合并
CAR_BLOB_DENSITY_MIN = 0.18  # 色块填充率下限

CAR_UNION_ASPECT_MAX = 1.70      # 联合外接框最大宽高比
CAR_UNION_FILL_MIN = 0.18        # 联合区域最小填充率
CAR_UNION_WALL_RATIO_MAX = 0.10  # 联合区域最大墙体占比

CAR_TRACK_HALF = 14        # 以上一次中心为中心的搜索半径
CAR_SMOOTH_ALPHA = 0.80    # 低通平滑系数（新值权重）
CAR_SMOOTH_DEADBAND = 0.80 # 小于该像素抖动时保持上一帧中心
CAR_SCALE = 100            # 坐标放大100倍发送
CAR_SEND_EVERY = 1         # CARINIT连续发送时每几帧发一次
BG_TRACK_EVERY = 3         # 建图完成后，空闲静默跟踪每几帧更新一次
BOOST_TRACK_EVERY = 3      # 收到CARPOS后，短时提速静默跟踪每几帧更新一次
CARPOS_BOOST_CYCLES = 8    # CARPOS后保留多少次提速静默跟踪
CAR_ROI_MARGIN = 4         # 以上一帧外接框扩展ROI时的额外边距
CAR_WALL_SCAN_STEP = 2     # 联合区域排墙时的采样步长
CAR_LOST_FULLSCAN_AFTER = 2  # 连续丢失多少次后再做整图地图区域回搜


# ===================== 原点选择 =====================
ORIGIN_MODE = 1
# 0 = 旧方式：从grid_send里找左上角'#'
# 1 = 新方式：最终输出地图左上角格中心 (0,0)
# 2 = 新方式：去掉外圈后，内部左上角格中心 (1,1)


# ===================== 采样点缓存 =====================
sample_x = [0] * N_SAMPLES
sample_y = [0] * N_SAMPLES

# 输出网格每个格中心对应的像素坐标（16x12）
out_cx = []
out_cy = []

# 地图原点：左下角障碍的格坐标
origin_rc = None  # (r0, c0)

# 跟踪状态
last_car_px = None        # 上一次小车中心像素坐标 (x, y)
last_car_rc = None        # 上一次小车粗格坐标 (r, c)
last_car_bbox = None      # 上一次小车外接框 (x, y, w, h)
carinit_mode = False
car_frame_cnt = 0
bg_track_frame_cnt = 0    # 静默跟踪计数器
track_boost_left = 0      # 剩余提速静默跟踪次数（CARPOS触发）

# ===================== 朝向估计状态（仅服务朝向，不影响建图） =====================
last_blue_center = None
last_green_center = None

# 圆周角滤波状态
last_heading_deg = None
last_heading_vx = None   # 数学坐标系单位向量 x
last_heading_vy = None   # 数学坐标系单位向量 y

# 朝向滤波参数
HEADING_SMOOTH_ALPHA = 0.25   # 新值权重；越小越稳，越大越跟手
HEADING_DEADBAND_DEG = 2.0    # 小于该角度变化时，保持上一帧结果

# 新增：反向保护阈值
# 当新方向与上一帧方向点积小于该阈值时，认为很可能发生了“误翻转”
HEADING_FLIP_DOT_T = -0.15
#===========================================================================

# 追踪临时缓冲，避免高频分配导致GC抖动
_roi_blue_mask = bytearray(0)
_roi_green_mask = bytearray(0)
_roi_blue_w = bytearray(0)
_roi_green_w = bytearray(0)
_roi_vis = bytearray(0)
last_track_miss = 0       # 连续跟踪失败次数

# 地图区域大致包围盒，用于丢失时回搜
map_px_min = 0
map_px_max = IMG_W - 1
map_py_min = 0
map_py_max = IMG_H - 1

# ===================== 串口接收缓冲 =====================
rx_buf = b""

# ===================== 像素读取加速：启动时探测一次 =====================
def _pix_to_rgb_888(pix):
    return pix[0], pix[1], pix[2]

def _pix_to_rgb_565tuple(pix):
    r = (pix[0] * 255 + 15) // 31
    g = (pix[1] * 255 + 31) // 63
    b = (pix[2] * 255 + 15) // 31
    return r, g, b

def _pix_to_rgb_565int(pix):
    r5 = (pix >> 11) & 0x1F
    g6 = (pix >> 5) & 0x3F
    b5 = pix & 0x1F
    r = (r5 * 255 + 15) // 31
    g = (g6 * 255 + 31) // 63
    b = (b5 * 255 + 15) // 31
    return r, g, b

PIX_TO_RGB = _pix_to_rgb_888

def _nearest_ref_dist(r, g, b):
    best = 10**9
    for i in range(N_CLS):
        d = abs(r - REF_R[i]) + abs(g - REF_G[i]) + abs(b - REF_B[i])
        if d < best:
            best = d
    return best

def detect_pixel_mode():
    global PIX_TO_RGB
    img = sensor.snapshot()

    test_ids = (0, N_SAMPLES // 2, N_SAMPLES - 1)
    for tid in test_ids:
        x = sample_x[tid]
        y = sample_y[tid]
        p = img.get_pixel(x, y)

        if isinstance(p, int):
            PIX_TO_RGB = _pix_to_rgb_565int
            return

        r, g, b = p[0], p[1], p[2]
        if (r > 31) or (g > 63) or (b > 31):
            PIX_TO_RGB = _pix_to_rgb_888
            return

    tid = N_SAMPLES // 2
    p = img.get_pixel(sample_x[tid], sample_y[tid])
    r0, g0, b0 = p[0], p[1], p[2]

    d888 = _nearest_ref_dist(r0, g0, b0)
    r1, g1, b1 = _pix_to_rgb_565tuple(p)
    d565 = _nearest_ref_dist(r1, g1, b1)

    PIX_TO_RGB = _pix_to_rgb_888 if d888 <= d565 else _pix_to_rgb_565tuple

# ===================== 邻域取色 =====================
PATCH_OFFS = None
PATCH_N = 1

def rebuild_patch_offsets():
    global PATCH_OFFS, PATCH_N
    r = PATCH_RAD
    if r <= 0:
        PATCH_OFFS = ((0, 0),)
        PATCH_N = 1
        return
    offs = []
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            offs.append((dx, dy))
    PATCH_OFFS = tuple(offs)
    PATCH_N = len(PATCH_OFFS)

def sample_rgb_mean(getp, pix_to_rgb, x, y):
    sr = sg = sb = 0
    wmax = IMG_W - 1
    hmax = IMG_H - 1
    for (dx, dy) in PATCH_OFFS:
        xx = x + dx
        yy = y + dy
        if xx < 0: xx = 0
        elif xx > wmax: xx = wmax
        if yy < 0: yy = 0
        elif yy > hmax: yy = hmax
        r, g, b = pix_to_rgb(getp(xx, yy))
        sr += r; sg += g; sb += b
    n = PATCH_N
    return sr // n, sg // n, sb // n

def sample_rgb_median3x3(getp, pix_to_rgb, x, y):
    wmax = IMG_W - 1
    hmax = IMG_H - 1
    rs = []
    gs = []
    bs = []
    for (dx, dy) in PATCH_OFFS:
        xx = x + dx
        yy = y + dy
        if xx < 0: xx = 0
        elif xx > wmax: xx = wmax
        if yy < 0: yy = 0
        elif yy > hmax: yy = hmax
        r, g, b = pix_to_rgb(getp(xx, yy))
        rs.append(r); gs.append(g); bs.append(b)
    rs.sort(); gs.sort(); bs.sort()
    return rs[4], gs[4], bs[4]

# ===================== 分类 =====================
def is_wall_like(r, g, b):
    m = (r + g + b) // 3
    if m < WALL_MEAN_MIN or m > WALL_MEAN_MAX:
        return False
    dev = abs(r - m) + abs(g - m) + abs(b - m)
    return dev <= WALL_DEV_T

def classify_idx(r, g, b):
    if is_wall_like(r, g, b):
        return 1

    abs_ = abs
    dr = r - REF_R[0]; dg = g - REF_G[0]; db = b - REF_B[0]
    best_d = abs_(dr) + abs_(dg) + abs_(db)
    best_i = 0

    for i in (1, 2, 3, 4, 5):
        dr = r - REF_R[i]; dg = g - REF_G[i]; db = b - REF_B[i]
        d = abs_(dr) + abs_(dg) + abs_(db)
        if d < best_d:
            best_d = d
            best_i = i

    return best_i if best_d <= REF_T[best_i] else 0

# ===================== 网格几何 =====================
def apply_global_fine_warp(x0, y0, cx, cy, inv_rn2):
    if abs(GRID_ROT_DEG) >= 1e-6:
        rad = GRID_ROT_DEG * math.pi / 180.0
        cs = math.cos(rad)
        sn = math.sin(rad)
        dx = x0 - cx
        dy = y0 - cy
        x0 = cx + dx * cs - dy * sn
        y0 = cy + dx * sn + dy * cs

    if abs(DIST_K1) > 1e-9:
        ddx = x0 - cx
        ddy = y0 - cy
        rn2 = (ddx * ddx + ddy * ddy) * inv_rn2
        scale = 1.0 + DIST_K1 * rn2
        x0 = cx + ddx * scale
        y0 = cy + ddy * scale

    return x0, y0

def bilerp_quad(u, v):
    omu = 1.0 - u
    omv = 1.0 - v
    x = (omu * omv * RECT_TL_X +
         u   * omv * RECT_TR_X +
         u   * v   * RECT_BR_X +
         omu * v   * RECT_BL_X)
    y = (omu * omv * RECT_TL_Y +
         u   * omv * RECT_TR_Y +
         u   * v   * RECT_BR_Y +
         omu * v   * RECT_BL_Y)
    return x, y

def rebuild_sample_points():
    grid_w = (S_COLS - 1) * GRID_DX
    grid_h = (S_ROWS - 1) * GRID_DY

    if USE_PERSPECTIVE_RECT:
        cx = (RECT_TL_X + RECT_TR_X + RECT_BR_X + RECT_BL_X) * 0.25
        cy = (RECT_TL_Y + RECT_TR_Y + RECT_BR_Y + RECT_BL_Y) * 0.25
        quad_w = 0.5 * (math.sqrt((RECT_TR_X - RECT_TL_X) ** 2 + (RECT_TR_Y - RECT_TL_Y) ** 2) +
                        math.sqrt((RECT_BR_X - RECT_BL_X) ** 2 + (RECT_BR_Y - RECT_BL_Y) ** 2))
        quad_h = 0.5 * (math.sqrt((RECT_BL_X - RECT_TL_X) ** 2 + (RECT_BL_Y - RECT_TL_Y) ** 2) +
                        math.sqrt((RECT_BR_X - RECT_TR_X) ** 2 + (RECT_BR_Y - RECT_TR_Y) ** 2))
        r_norm = max(quad_w, quad_h) * 0.5
        if r_norm < 1.0:
            r_norm = 1.0
        inv_rn2 = 1.0 / (r_norm * r_norm)
    else:
        cx = GRID_X0 + grid_w * 0.5
        cy = GRID_Y0 + grid_h * 0.5
        r_norm = max(grid_w, grid_h) * 0.5
        if r_norm < 1.0:
            r_norm = 1.0
        inv_rn2 = 1.0 / (r_norm * r_norm)

    idx = 0
    for i in range(S_ROWS):
        if S_ROWS <= 1:
            v = 0.0
        else:
            v = i / float(S_ROWS - 1)
        for j in range(S_COLS):
            if USE_PERSPECTIVE_RECT:
                if S_COLS <= 1:
                    u = 0.0
                else:
                    u = j / float(S_COLS - 1)
                x0, y0 = bilerp_quad(u, v)
            else:
                y0 = GRID_Y0 + i * GRID_DY
                x0 = GRID_X0 + j * GRID_DX

            xr, yr = apply_global_fine_warp(x0, y0, cx, cy, inv_rn2)

            xi = int(xr + 0.5)
            yi = int(yr + 0.5)
            if xi < 0: xi = 0
            elif xi >= IMG_W: xi = IMG_W - 1
            if yi < 0: yi = 0
            elif yi >= IMG_H: yi = IMG_H - 1

            sample_x[idx] = xi
            sample_y[idx] = yi
            idx += 1

def draw_sampling_overlay(img):
    for k in range(N_SAMPLES):
        img.draw_circle(sample_x[k], sample_y[k], 3, color=(255, 255, 255))

    if DRAW_RECTIFY_QUAD and USE_PERSPECTIVE_RECT:
        tlx = int(RECT_TL_X + 0.5); tly = int(RECT_TL_Y + 0.5)
        trx = int(RECT_TR_X + 0.5); try_ = int(RECT_TR_Y + 0.5)
        brx = int(RECT_BR_X + 0.5); bry = int(RECT_BR_Y + 0.5)
        blx = int(RECT_BL_X + 0.5); bly = int(RECT_BL_Y + 0.5)
        img.draw_line(tlx, tly, trx, try_, color=RECT_QUAD_COLOR)
        img.draw_line(trx, try_, brx, bry, color=RECT_QUAD_COLOR)
        img.draw_line(brx, bry, blx, bly, color=RECT_QUAD_COLOR)
        img.draw_line(blx, bly, tlx, tly, color=RECT_QUAD_COLOR)
        img.draw_cross(tlx, tly, color=RECT_QUAD_COLOR)
        img.draw_cross(trx, try_, color=RECT_QUAD_COLOR)
        img.draw_cross(brx, bry, color=RECT_QUAD_COLOR)
        img.draw_cross(blx, bly, color=RECT_QUAD_COLOR)

# ===================== 输出格中心像素坐标 =====================
def build_output_centers():
    global out_cx, out_cy, map_px_min, map_px_max, map_py_min, map_py_max

    xs = []
    ys = []
    for _ in range(ROWS):
        xs.append([0] * COLS)
        ys.append([0] * COLS)

    if OUTPUT_ROTATE_CW_90:
        for r in range(ROWS):
            for c in range(COLS):
                src_i = (S_ROWS - 1) - c
                src_j = r
                idx = src_i * S_COLS + src_j
                xs[r][c] = sample_x[idx]
                ys[r][c] = sample_y[idx]
    else:
        for r in range(min(ROWS, S_ROWS)):
            for c in range(min(COLS, S_COLS)):
                idx = r * S_COLS + c
                xs[r][c] = sample_x[idx]
                ys[r][c] = sample_y[idx]

    if SEND_FLIP_ROWS:
        xs.reverse()
        ys.reverse()
    if SEND_FLIP_COLS:
        for r in range(ROWS):
            xs[r].reverse()
            ys[r].reverse()

    out_cx = xs
    out_cy = ys

    xmin = 10**9
    xmax = -1
    ymin = 10**9
    ymax = -1
    for r in range(ROWS):
        for c in range(COLS):
            x = out_cx[r][c]
            y = out_cy[r][c]
            if x < xmin: xmin = x
            if x > xmax: xmax = x
            if y < ymin: ymin = y
            if y > ymax: ymax = y

    margin = 10
    map_px_min = max(0, xmin - margin)
    map_px_max = min(IMG_W - 1, xmax + margin)
    map_py_min = max(0, ymin - margin)
    map_py_max = min(IMG_H - 1, ymax + margin)

def outcell_center_px(r, c):
    return out_cx[r][c], out_cy[r][c]

# ===================== 投票结果 -> 地图 =====================
def votes_to_grid_send(counts, need):
    grid_send = []
    for _ in range(ROWS):
        grid_send.append(['.'] * COLS)

    if OUTPUT_ROTATE_CW_90:
        for r in range(ROWS):
            row = grid_send[r]
            src_j = r
            for c in range(COLS):
                src_i = (S_ROWS - 1) - c
                idx = src_i * S_COLS + src_j
                base = idx * N_CLS

                best_ci = 0
                best_cnt = counts[base]
                v = counts[base+1]
                if v > best_cnt: best_cnt = v; best_ci = 1
                v = counts[base+2]
                if v > best_cnt: best_cnt = v; best_ci = 2
                v = counts[base+3]
                if v > best_cnt: best_cnt = v; best_ci = 3
                v = counts[base+4]
                if v > best_cnt: best_cnt = v; best_ci = 4
                v = counts[base+5]
                if v > best_cnt: best_cnt = v; best_ci = 5

                row[c] = CLASSES[best_ci] if best_cnt >= need else '.'
    else:
        for r in range(min(ROWS, S_ROWS)):
            row = grid_send[r]
            for c in range(min(COLS, S_COLS)):
                idx = r * S_COLS + c
                base = idx * N_CLS
                best_ci = 0
                best_cnt = counts[base]
                for ci in (1,2,3,4,5):
                    v = counts[base+ci]
                    if v > best_cnt:
                        best_cnt = v
                        best_ci = ci
                row[c] = CLASSES[best_ci] if best_cnt >= need else '.'

    if SEND_FLIP_ROWS:
        grid_send.reverse()
    if SEND_FLIP_COLS:
        for r in range(ROWS):
            grid_send[r].reverse()

    return grid_send

# ===================== 单帧转地图（动态模式粗定位）  =====================
def image_to_grid_once(img):
    grid_send = []
    for _ in range(ROWS):
        grid_send.append(['.'] * COLS)

    getp = img.get_pixel
    pix_to_rgb = PIX_TO_RGB
    cls = classify_idx

    sample_cls = bytearray(N_SAMPLES)

    if PATCH_RAD <= 0:
        for idx in range(N_SAMPLES):
            r, g, b = pix_to_rgb(getp(sample_x[idx], sample_y[idx]))
            sample_cls[idx] = cls(r, g, b)
    else:
        sampler = sample_rgb_median3x3 if (PATCH_USE_MEDIAN and PATCH_RAD == 1) else sample_rgb_mean
        for idx in range(N_SAMPLES):
            r, g, b = sampler(getp, pix_to_rgb, sample_x[idx], sample_y[idx])
            sample_cls[idx] = cls(r, g, b)

    if OUTPUT_ROTATE_CW_90:
        for r in range(ROWS):
            row = grid_send[r]
            src_j = r
            for c in range(COLS):
                src_i = (S_ROWS - 1) - c
                idx = src_i * S_COLS + src_j
                row[c] = CLASSES[sample_cls[idx]]
    else:
        for r in range(min(ROWS, S_ROWS)):
            row = grid_send[r]
            for c in range(min(COLS, S_COLS)):
                idx = r * S_COLS + c
                row[c] = CLASSES[sample_cls[idx]]

    if SEND_FLIP_ROWS:
        grid_send.reverse()
    if SEND_FLIP_COLS:
        for r in range(ROWS):
            grid_send[r].reverse()

    return grid_send

# ===================== 地图/位置工具 =====================
def find_first_char(grid_send, ch):
    for r in range(ROWS):
        row = grid_send[r]
        for c in range(COLS):
            if row[c] == ch:
                return (r, c)
    return None

def find_left_top_wall(grid_send):
    best = None
    for r in range(ROWS):
        for c in range(COLS):
            if grid_send[r][c] == '#':
                # r越小越靠上；同r时c越小越靠左
                if (best is None) or (r < best[0]) or (r == best[0] and c < best[1]):
                    best = (r, c)
    if best is None:
        best = (0, 0)
    return best


def get_origin_rc(grid_send):
    if ORIGIN_MODE == 0:
        return find_left_top_wall(grid_send)
    elif ORIGIN_MODE == 2:
        return (1, 1)
    else:
        return (0, 0)


def nearest_output_cell(px, py):
    best_r = 0
    best_c = 0
    best_d2 = 10**18
    for r in range(ROWS):
        for c in range(COLS):
            dx = px - out_cx[r][c]
            dy = py - out_cy[r][c]
            d2 = dx*dx + dy*dy
            if d2 < best_d2:
                best_d2 = d2
                best_r = r
                best_c = c
    return best_r, best_c

def get_local_halfsize(r, c):
    hs = []
    if c > 0:
        dx = out_cx[r][c] - out_cx[r][c-1]
        dy = out_cy[r][c] - out_cy[r][c-1]
        hs.append(math.sqrt(dx * dx + dy * dy))
    if c + 1 < COLS:
        dx = out_cx[r][c+1] - out_cx[r][c]
        dy = out_cy[r][c+1] - out_cy[r][c]
        hs.append(math.sqrt(dx * dx + dy * dy))
    if r > 0:
        dx = out_cx[r][c] - out_cx[r-1][c]
        dy = out_cy[r][c] - out_cy[r-1][c]
        hs.append(math.sqrt(dx * dx + dy * dy))
    if r + 1 < ROWS:
        dx = out_cx[r+1][c] - out_cx[r][c]
        dy = out_cy[r+1][c] - out_cy[r][c]
        hs.append(math.sqrt(dx * dx + dy * dy))

    if not hs:
        half = 12
    else:
        half = int(min(hs) * 0.35)
        if half < 8: half = 8
        if half > 18: half = 18
    return half

# ===================== 图像绘制辅助 =====================
def draw_car_overlay(img, bbox, center=None, roi=None):
    if DRAW_TRACK_ROI and roi is not None:
        x0, y0, x1, y1 = roi
        img.draw_rectangle(x0, y0, x1 - x0 + 1, y1 - y0 + 1, color=TRACK_ROI_COLOR)

    if DRAW_CAR_BOX and bbox is not None:
        bx, by, bw, bh = bbox
        img.draw_rectangle(bx, by, bw, bh, color=CAR_BOX_COLOR)

    if DRAW_CAR_CENTER and center is not None:
        bx, by, bw, bh = bbox
        icx = bx + bw // 2
        icy = by + bh // 2
        #cx, cy = center
        #icx = int(cx + 0.5)
        #icy = int(cy + 0.5)
        img.draw_cross(icx, icy, color=CAR_CENTER_COLOR)




def draw_origin_overlay(img):
    # 基准点：左下角障碍 '#' 的中心 = 连续坐标原点(0,0)
    if (not DRAW_ORIGIN_POINT) or (origin_rc is None):
        return

    r0, c0 = origin_rc
    px = out_cx[r0][c0]
    py = out_cy[r0][c0]

    # 画十字 + 小圆
    img.draw_cross(px, py, color=ORIGIN_POINT_COLOR, size=8)
    img.draw_circle(px, py, 5, color=ORIGIN_POINT_COLOR)

    # 标注文字
    tx = px + 6
    ty = py - 10
    if tx < 0: tx = 0
    if ty < 0: ty = 0
    img.draw_string(tx, ty, "(0,0)", color=ORIGIN_TEXT_COLOR)



# ===================== 动态排墙 + 排他判色 + 抖动平滑 =====================
# ===================== 蓝半块 + 绿半块联合定位 =====================内含车头朝向判断


def smooth_car_center(new_px, new_py):
    if last_car_px is None:
        return new_px, new_py

    old_px, old_py = last_car_px
    dx = new_px - old_px
    dy = new_py - old_py

    if dx * dx + dy * dy <= CAR_SMOOTH_DEADBAND * CAR_SMOOTH_DEADBAND:
        return old_px, old_py

    a = CAR_SMOOTH_ALPHA
    return old_px + a * dx, old_py + a * dy

#==================================角度检测优化
def _wrap_deg(a):
    """
    把角度包到 (-180, 180] 范围。
    这样在 +180/-180 附近比较两帧角度差时，不会因为表示跳变而出错。
    """
    while a <= -180.0:
        a += 360.0
    while a > 180.0:
        a -= 360.0
    return a
#================================

def smooth_heading_vec(new_hx, new_hy):
    """
    圆周角滤波（带反向保护）：
    输入:
        new_hx, new_hy  -> 图像坐标系下的车头方向单位向量
                           (x 右为正, y 下为正)

    输出:
        (hx, hy, heading_deg)
        其中:
            hx, hy      -> 滤波后的图像坐标系单位向量
            heading_deg -> 滤波后的角度（右=0, 上=90, 左=±180, 下=-90）

    关键增强：
    1) 新方向若与上一帧方向明显相反（点积明显为负），则自动翻回去
    2) 再做圆周角低通
    """
    global last_heading_deg, last_heading_vx, last_heading_vy

    # 图像坐标 -> 数学坐标
    vx = float(new_hx)
    vy = float(-new_hy)

    n2 = vx * vx + vy * vy
    if n2 < 1e-9:
        if last_heading_deg is None:
            return None, None, None
        return last_heading_vx, -last_heading_vy, last_heading_deg

    n = math.sqrt(n2)
    vx /= n
    vy /= n

    # ------------------------------------------------
    # 反向保护：
    # 如果新方向和上一帧方向明显相反，则直接把新方向翻转回来
    # 这样可以抑制“旋转过程中偶发指向相反方向”的问题
    # ------------------------------------------------
    if last_heading_vx is not None:
        dot = vx * last_heading_vx + vy * last_heading_vy
        if dot < HEADING_FLIP_DOT_T:
            vx = -vx
            vy = -vy

    new_deg = math.degrees(math.atan2(vy, vx))

    # 第一次直接初始化
    if last_heading_vx is None:
        last_heading_vx = vx
        last_heading_vy = vy
        last_heading_deg = new_deg
        return vx, -vy, new_deg

    # 死区：变化很小就直接保持上一帧
    delta = _wrap_deg(new_deg - last_heading_deg)
    if abs(delta) <= HEADING_DEADBAND_DEG:
        return last_heading_vx, -last_heading_vy, last_heading_deg

    # 圆周低通
    a = HEADING_SMOOTH_ALPHA
    fx = (1.0 - a) * last_heading_vx + a * vx
    fy = (1.0 - a) * last_heading_vy + a * vy

    fn2 = fx * fx + fy * fy
    if fn2 < 1e-9:
        fx = vx
        fy = vy
    else:
        fn = math.sqrt(fn2)
        fx /= fn
        fy /= fn

    last_heading_vx = fx
    last_heading_vy = fy
    last_heading_deg = math.degrees(math.atan2(fy, fx))

    return fx, -fy, last_heading_deg


def _heading_rgb_to_lab(r, g, b):
    """
    仅服务朝向函数内部打分的 RGB -> Lab 近似转换。
    不影响现有 find_blobs() 阈值逻辑，只用于给单色候选 blob 增加一个
    '中心局部颜色是否更像目标颜色' 的附加评分。
    """
    def _srgb_to_linear(v255):
        v = v255 / 255.0
        if v <= 0.04045:
            return v / 12.92
        return ((v + 0.055) / 1.055) ** 2.4

    rl = _srgb_to_linear(r)
    gl = _srgb_to_linear(g)
    bl = _srgb_to_linear(b)

    X = rl * 0.4124564 + gl * 0.3575761 + bl * 0.1804375
    Y = rl * 0.2126729 + gl * 0.7151522 + bl * 0.0721750
    Z = rl * 0.0193339 + gl * 0.1191920 + bl * 0.9503041

    X /= 0.95047
    Y /= 1.00000
    Z /= 1.08883

    def f(t):
        if t > 0.008856:
            return t ** (1.0 / 3.0)
        return 7.787 * t + 16.0 / 116.0

    fx = f(X)
    fy = f(Y)
    fz = f(Z)

    L = 116.0 * fy - 16.0
    a = 500.0 * (fx - fy)
    bb = 200.0 * (fy - fz)
    return L, a, bb


def _heading_patch_lab_dist2(img, cx, cy, th):
    """
    仅服务朝向：
    在 blob 中心做一个固定 3x3 中值取色，再看它与目标阈值盒中心的距离。
    距离越小，说明这个候选在局部颜色上越像目标色。
    """
    getp = img.get_pixel
    pix_to_rgb = PIX_TO_RGB

    # 强制用 3x3 中值取色；不依赖全局 PATCH_RAD
    rs = []
    gs = []
    bs = []
    for dy in (-1, 0, 1):
        yy = cy + dy
        if yy < 0:
            yy = 0
        elif yy >= IMG_H:
            yy = IMG_H - 1

        for dx in (-1, 0, 1):
            xx = cx + dx
            if xx < 0:
                xx = 0
            elif xx >= IMG_W:
                xx = IMG_W - 1

            r, g, b = pix_to_rgb(getp(xx, yy))
            rs.append(r)
            gs.append(g)
            bs.append(b)

    rs.sort()
    gs.sort()
    bs.sort()
    r = rs[4]
    g = gs[4]
    b = bs[4]

    L, a, bb = _heading_rgb_to_lab(r, g, b)

    cL = 0.5 * (th[0] + th[1])
    ca = 0.5 * (th[2] + th[3])
    cb = 0.5 * (th[4] + th[5])

    dL = L - cL
    da = a - ca
    db = bb - cb
    return dL * dL + da * da + db * db



def union_wall_ratio(img, x0, y0, x1, y1, step=CAR_WALL_SCAN_STEP):
    getp = img.get_pixel
    pix_to_rgb = PIX_TO_RGB

    total = 0
    wall_cnt = 0
    for y in range(y0, y1 + 1, step):
        for x in range(x0, x1 + 1, step):
            r, g, b = pix_to_rgb(getp(x, y))
            total += 1
            if is_wall_like(r, g, b):
                wall_cnt += 1

    if total <= 0:
        return 1.0
    return wall_cnt / float(total)

# ===================车头朝向判断=============
def Angle_find_blue_green_blobs_in_roi(img, x0, y0, x1, y1):
    """
    朝向优化版（最终防反向修正）：
    继续采用“蓝/绿 blob 的面对面边构造分界线”的思路，
    并保留以下三项仅服务朝向的增强：

    1) 蓝/绿单色 blob 检测专用 stride=1
    2) 蓝/绿 blob 选择加入上一帧同色约束
    3) 最终 heading 向量做圆周角滤波 + 死区

    本版关键修正：
    - 把最终方向修正提前到滤波之前
    - smooth_heading_vec 内部加入点积防翻转保护
    - 解决“小车旋转时方向指针偶尔指向相反方向”的问题

    返回：
        None
        或 dict:
        {
            "center": (ucx, ucy),
            "bbox": (ubx, uby, ubw, ubh),
            "blue_center": (bcx, bcy),
            "green_center": (gcx, gcy),
            "blue_bbox": (bbx, bby, bbw, bbh),
            "green_bbox": (gbx, gby, gbw, gbh),
            "divider_line": (x0, y0, x1, y1),
            "heading_line": (x0, y0, x1, y1),
            "heading_deg": heading_deg,
        }
    """
    global last_blue_center, last_green_center

    # ------------------------------------------------
    # 0) ROI 裁剪
    # ------------------------------------------------
    if x0 < 0:
        x0 = 0
    if y0 < 0:
        y0 = 0
    if x1 >= IMG_W:
        x1 = IMG_W - 1
    if y1 >= IMG_H:
        y1 = IMG_H - 1
    if x1 < x0 or y1 < y0:
        return None

    roi = (x0, y0, x1 - x0 + 1, y1 - y0 + 1)

    # ------------------------------------------------
    # 1) 联合车体 blob（stride=1）
    # ------------------------------------------------
    union_blobs = img.find_blobs(
        [CAR_BLUE_TH, CAR_GREEN_TH],
        roi=roi,
        x_stride=1,
        y_stride=1,
        pixels_threshold=CAR_BLOB_PIXELS_MIN,
        area_threshold=CAR_BLOB_AREA_MIN,
        merge=True,
        margin=CAR_BLOB_MERGE_MARGIN
    )
    if not union_blobs:
        return None

    union_res = choose_best_car_blob(img, union_blobs)
    if union_res is None:
        return None

    ucx, ucy, union_bbox = union_res
    ubx, uby, ubw, ubh = union_bbox

    # ------------------------------------------------
    # 2) 在联合框附近分别找蓝块和绿块（stride=1）
    # ------------------------------------------------
    sub_margin = 3
    sx0 = ubx - sub_margin
    sy0 = uby - sub_margin
    sx1 = ubx + ubw - 1 + sub_margin
    sy1 = uby + ubh - 1 + sub_margin

    if sx0 < 0:
        sx0 = 0
    if sy0 < 0:
        sy0 = 0
    if sx1 >= IMG_W:
        sx1 = IMG_W - 1
    if sy1 >= IMG_H:
        sy1 = IMG_H - 1

    sub_roi = (sx0, sy0, sx1 - sx0 + 1, sy1 - sy0 + 1)

    blue_blobs = img.find_blobs(
        [CAR_BLUE_TH],
        roi=sub_roi,
        x_stride=1,
        y_stride=1,
        pixels_threshold=CAR_BLOB_PIXELS_MIN,
        area_threshold=CAR_BLOB_AREA_MIN,
        merge=False
    )

    green_blobs = img.find_blobs(
        [CAR_GREEN_TH],
        roi=sub_roi,
        x_stride=1,
        y_stride=1,
        pixels_threshold=CAR_BLOB_PIXELS_MIN,
        area_threshold=CAR_BLOB_AREA_MIN,
        merge=False
    )

    if (not blue_blobs) or (not green_blobs):
        return None

    # ------------------------------------------------
    # 3) 评分选 best_blue / best_green
    # ------------------------------------------------
    def _rect_inter_area(ax, ay, aw, ah, bx, by, bw, bh):
        ax1 = ax + aw - 1
        ay1 = ay + ah - 1
        bx1 = bx + bw - 1
        by1 = by + bh - 1

        ix0 = ax if ax > bx else bx
        iy0 = ay if ay > by else by
        ix1 = ax1 if ax1 < bx1 else bx1
        iy1 = ay1 if ay1 < by1 else by1

        if ix1 < ix0 or iy1 < iy0:
            return 0
        return (ix1 - ix0 + 1) * (iy1 - iy0 + 1)

    def _pick_best_single_color_blob(blobs, last_same_center, th):
        best = None
        best_score = -10**9

        for b in blobs:
            bx, by, bw, bh = b.rect()
            if bw <= 0 or bh <= 0:
                continue

            fill = b.density()
            if fill < CAR_BLOB_DENSITY_MIN:
                continue

            inter = _rect_inter_area(bx, by, bw, bh, ubx, uby, ubw, ubh)
            if inter <= 0:
                continue

            cx = b.cx()
            cy = b.cy()

            score = b.pixels()
            score += 25.0 * fill
            score += 0.05 * inter

            dx0 = cx - ucx
            dy0 = cy - ucy
            score -= 1.2 * (dx0 * dx0 + dy0 * dy0)

            if last_same_center is not None:
                dx1 = cx - last_same_center[0]
                dy1 = cy - last_same_center[1]
                score -= 2.5 * (dx1 * dx1 + dy1 * dy1)

            patch_d2 = _heading_patch_lab_dist2(img, cx, cy, th)
            score -= 0.03 * patch_d2

            if score > best_score:
                best_score = score
                best = b

        return best

    best_blue = _pick_best_single_color_blob(blue_blobs, last_blue_center, CAR_BLUE_TH)
    best_green = _pick_best_single_color_blob(green_blobs, last_green_center, CAR_GREEN_TH)

    if (best_blue is None) or (best_green is None):
        return None

    bbx, bby, bbw, bbh = best_blue.rect()
    gbx, gby, gbw, gbh = best_green.rect()

    bcx = best_blue.cx()
    bcy = best_blue.cy()
    gcx = best_green.cx()
    gcy = best_green.cy()

    # 更新同色历史中心（仅服务朝向）
    last_blue_center = (bcx, bcy)
    last_green_center = (gcx, gcy)

    # ------------------------------------------------
    # 4) 用绿中心 -> 蓝中心构造连续方向
    # ------------------------------------------------
    vx = bcx - gcx
    vy = bcy - gcy
    norm2 = vx * vx + vy * vy
    if norm2 < 4:
        return None

    norm = math.sqrt(norm2)
    ux = vx / norm
    uy = vy / norm

    # ------------------------------------------------
    # 5) 从矩形中心沿方向发射射线，求与边界的交点
    # ------------------------------------------------
    def _ray_hit_rect_from_center(cx, cy, rx, ry, rw, rh, dirx, diry):
        left = rx
        right = rx + rw - 1
        top = ry
        bottom = ry + rh - 1

        ts = []

        if abs(dirx) > 1e-9:
            t = (left - cx) / dirx
            if t > 0:
                y = cy + t * diry
                if top <= y <= bottom:
                    ts.append(t)

            t = (right - cx) / dirx
            if t > 0:
                y = cy + t * diry
                if top <= y <= bottom:
                    ts.append(t)

        if abs(diry) > 1e-9:
            t = (top - cy) / diry
            if t > 0:
                x = cx + t * dirx
                if left <= x <= right:
                    ts.append(t)

            t = (bottom - cy) / diry
            if t > 0:
                x = cx + t * dirx
                if left <= x <= right:
                    ts.append(t)

        if not ts:
            return cx, cy

        tmin = min(ts)
        return cx + tmin * dirx, cy + tmin * diry

    # 蓝框朝向绿框的面对面边界点：蓝中心沿“蓝 -> 绿”方向
    blue_face_x, blue_face_y = _ray_hit_rect_from_center(
        bcx, bcy,
        bbx, bby, bbw, bbh,
        -ux, -uy
    )

    # 绿框朝向蓝框的面对面边界点：绿中心沿“绿 -> 蓝”方向
    green_face_x, green_face_y = _ray_hit_rect_from_center(
        gcx, gcy,
        gbx, gby, gbw, gbh,
        ux, uy
    )

    # ------------------------------------------------
    # 6) 由“面对面边界点”构造原始 heading 方向
    # ------------------------------------------------
    hvx = blue_face_x - green_face_x
    hvy = blue_face_y - green_face_y
    hnorm2 = hvx * hvx + hvy * hvy

    if hnorm2 < 1e-6:
        hvx = vx
        hvy = vy
        hnorm2 = norm2

    hnorm = math.sqrt(hnorm2)
    hx = hvx / hnorm
    hy = hvy / hnorm

    # ------------------------------------------------
    # 7) 如果需要反转角度就把下面两行注释解除
    # ------------------------------------------------
    hx = -hx
    hy = -hy

    # ------------------------------------------------
    # 8) 对“最终方向”做圆周角滤波 + 死区 + 防翻转保护
    # ------------------------------------------------
    fhx, fhy, heading_deg = smooth_heading_vec(hx, hy)
    if heading_deg is None:
        return None

    hx = fhx
    hy = fhy

    # ------------------------------------------------
    # 9) divider 方向始终是 heading 的垂线方向
    #    注意：这里一定要用“滤波后的最终 heading”来重建 divider
    # ------------------------------------------------
    dx = -hy
    dy = hx

    # divider 中心仍然放在两个面对面边界点中点附近
    mx = 0.5 * (blue_face_x + green_face_x)
    my = 0.5 * (blue_face_y + green_face_y)

    # ------------------------------------------------
    # 10) 构造 divider_line / heading_line
    # ------------------------------------------------
    divider_half = int(min(ubw, ubh) * 0.70)
    if divider_half < 8:
        divider_half = 8
    if divider_half > 24:
        divider_half = 24

    d0x = int(mx - dx * divider_half + 0.5)
    d0y = int(my - dy * divider_half + 0.5)
    d1x = int(mx + dx * divider_half + 0.5)
    d1y = int(my + dy * divider_half + 0.5)

    head_len = int(max(ubw, ubh) * 0.90)
    if head_len < 10:
        head_len = 10
    if head_len > 28:
        head_len = 28

    h0x = int(ucx + 0.5)
    h0y = int(ucy + 0.5)
    h1x = int(ucx + hx * head_len + 0.5)
    h1y = int(ucy + hy * head_len + 0.5)

    return {
        "center": (ucx, ucy),
        "bbox": union_bbox,
        "blue_center": (bcx, bcy),
        "green_center": (gcx, gcy),
        "blue_bbox": (bbx, bby, bbw, bbh),
        "green_bbox": (gbx, gby, gbw, gbh),
        "divider_line": (d0x, d0y, d1x, d1y),
        "heading_line": (h0x, h0y, h1x, h1y),
        "heading_deg": heading_deg,
    }
#==============车头方向获取调试代码===========
def draw_car_heading_debug(img, pose):
    """
    调试函数：只做两件事
    1) 在 IDE 终端输出角度
    2) 在 IDE 图像里画出方向向量和分界线

    依赖 pose 至少包含：
        pose["heading_deg"]   -> 最终角度
        pose["heading_line"]  -> (x0, y0, x1, y1) 方向向量
        pose["divider_line"]  -> (x0, y0, x1, y1) 分界线
    """

    # 没检测到结果，直接返回
    if pose is None:
        return

    heading_deg = pose.get("heading_deg", None)
    heading_line = pose.get("heading_line", None)
    divider_line = pose.get("divider_line", None)

    # 只要缺任何一个关键字段，这一帧就不画也不打印
    if (heading_deg is None) or (heading_line is None) or (divider_line is None):
        return

    # ------------------------------------------------
    # 1) IDE 终端输出角度
    # ------------------------------------------------
    print("[HEAD] %.1f deg" % heading_deg)

    # ------------------------------------------------
    # 2) IDE 图像里画方向向量
    #    这里画的是“最终车头方向线”
    # ------------------------------------------------
    hx0, hy0, hx1, hy1 = heading_line
    img.draw_line(
        int(hx0), int(hy0),
        int(hx1), int(hy1),
        color=(255, 0, 255),   # 紫色：方向向量
        thickness=2
    )

    # ------------------------------------------------
    # 3) IDE 图像里画分界线
    # ------------------------------------------------
    dx0, dy0, dx1, dy1 = divider_line
    img.draw_line(
        int(dx0), int(dy0),
        int(dx1), int(dy1),
        color=(255, 255, 0),   # 黄色：分界线
        thickness=1
    )


# ================================================

def choose_best_car_blob(img, blobs):
    best = None
    best_score = -10**9

    for b in blobs:
        # merge=True 且传 [蓝,绿] 两个阈值时：
        # 蓝=1，绿=2，同时命中=3
        if b.code() != 3:
            continue

        bx, by, bw, bh = b.rect()
        if bw <= 0 or bh <= 0:
            continue

        aspect = bw / float(bh) if bw >= bh else bh / float(bw)
        if aspect > CAR_UNION_ASPECT_MAX:
            continue

        fill = b.density()
        if fill < CAR_UNION_FILL_MIN or fill < CAR_BLOB_DENSITY_MIN:
            continue

        wall_ratio = union_wall_ratio(img, bx, by, bx + bw - 1, by + bh - 1)
        if wall_ratio > CAR_UNION_WALL_RATIO_MAX:
            continue

        cx = bx + bw * 0.5
        cy = by + bh * 0.5
        cx = b.cx()
        cy = b.cy()

        # 分数：优先像素多、形状更方、离上次位置更近
        score = b.pixels()
        score += 40.0 * fill
        score -= 50.0 * abs(aspect - 1.0)

        if last_car_px is not None:
            dx = cx - last_car_px[0]
            dy = cy - last_car_px[1]
            score -= 2.5 * (dx * dx + dy * dy)

        if score > best_score:
            best_score = score
            best = (cx, cy, (bx, by, bw, bh))

    return best

def car_centroid_in_roi(img, x0, y0, x1, y1):
    if x0 < 0: x0 = 0
    if y0 < 0: y0 = 0
    if x1 >= IMG_W: x1 = IMG_W - 1
    if y1 >= IMG_H: y1 = IMG_H - 1
    if x1 < x0 or y1 < y0:
        return None

    roi = (x0, y0, x1 - x0 + 1, y1 - y0 + 1)

    blobs = img.find_blobs(
        [CAR_BLUE_TH, CAR_GREEN_TH],
        roi=roi,
        x_stride=CAR_BLOB_X_STRIDE,
        y_stride=CAR_BLOB_Y_STRIDE,
        pixels_threshold=CAR_BLOB_PIXELS_MIN,
        area_threshold=CAR_BLOB_AREA_MIN,
        merge=True,
        margin=CAR_BLOB_MERGE_MARGIN
    )

    if not blobs:
        return None

    return choose_best_car_blob(img, blobs)


def finalize_car_detection(img, px, py, bbox, roi=None, rough_rc=None):
    global last_car_px, last_car_rc, last_car_bbox

    px, py = smooth_car_center(px, py)

    if rough_rc is None:
        rr, cc = nearest_output_cell(px, py)
    else:
        rr, cc = rough_rc

    row_f, col_f = pixel_to_continuous_rc(px, py, rr, cc)
    xy = rc_to_user_xy(row_f, col_f)
    if xy is None:
        return None

    last_car_px = (px, py)
    last_car_rc = (rr, cc)
    last_car_bbox = bbox

    draw_car_overlay(img, bbox, (px, py), roi)
    draw_origin_overlay(img)

    return xy

def find_car_centroid_precise(img, coarse_rc):
    r, c = coarse_rc
    cx, cy = outcell_center_px(r, c)
    half = get_local_halfsize(r, c)
    x0 = cx - half
    y0 = cy - half
    x1 = cx + half
    y1 = cy + half
    res = car_centroid_in_roi(img, x0, y0, x1, y1)
    return res, (x0, y0, x1, y1)



def clear_char_in_grid(grid_send, ch):
    for r in range(ROWS):
        row = grid_send[r]
        for c in range(COLS):
            if row[c] == ch:
                row[c] = '.'

def start_find_car_and_write_back(grid_send):
    global last_car_px, last_car_rc, last_car_bbox

    img = sensor.snapshot()

    # 直接按“动态找车”的方式，在整张地图ROI里找车
    roi = (map_px_min, map_py_min, map_px_max, map_py_max)
    res = car_centroid_in_roi(img, roi[0], roi[1], roi[2], roi[3])
    if res is None:
        return False

    px, py, bbox = res

    # 先找最近格作为粗格
    rr0, cc0 = nearest_output_cell(px, py)

    # 再换算成连续格坐标，并四舍五入为整数格
    row_f, col_f = pixel_to_continuous_rc(px, py, rr0, cc0)
    rr = int(row_f + 0.5)
    cc = int(col_f + 0.5)

    # 边界裁剪
    if rr < 1:
        rr = 1
    elif rr >= ROWS:
        rr = ROWS - 2

    if cc < 1:
        cc = 1
    elif cc >= COLS:
        cc = COLS - 2

    # 不能把车写到墙里
    if grid_send[rr][cc] == '#':
        return False

    # 把地图里原来的C清掉，再写入新的整数格位置
    clear_char_in_grid(grid_send, 'C')
    grid_send[rr][cc] = 'C'

    # 内部跟踪状态仍保存像素精确值，便于后续动态跟踪
    last_car_px = (px, py)
    last_car_rc = (rr, cc)
    last_car_bbox = bbox

    if DRAW_CAR_BOX or DRAW_CAR_CENTER or DRAW_TRACK_ROI:
        draw_car_overlay(img, bbox, (px, py), roi)

    return True


# ===================== 像素坐标 -> 连续格坐标 =====================
def pixel_to_continuous_rc(px, py, rough_r, rough_c):
    br = rough_r
    bc = rough_c
    if br >= ROWS - 1:
        br = ROWS - 2
    if bc >= COLS - 1:
        bc = COLS - 2
    if br < 0: br = 0
    if bc < 0: bc = 0

    x00 = out_cx[br][bc]
    y00 = out_cy[br][bc]
    x10 = out_cx[br][bc + 1]
    y10 = out_cy[br][bc + 1]
    x01 = out_cx[br + 1][bc]
    y01 = out_cy[br + 1][bc]

    exx = x10 - x00
    exy = y10 - y00
    eyx = x01 - x00
    eyy = y01 - y00

    dx = px - x00
    dy = py - y00

    det = exx * eyy - exy * eyx
    if abs(det) < 1e-6:
        return float(rough_r), float(rough_c)

    a = (dx * eyy - dy * eyx) / det   # 列方向
    b = (exx * dy - exy * dx) / det   # 行方向（向下）

    col_f = bc + a
    row_f = br + b
    return row_f, col_f

def rc_to_user_xy(row_f, col_f):
    global origin_rc
    if origin_rc is None:
        return None
    r0, c0 = origin_rc
    y = col_f - c0
    x = row_f - r0
    return x, y

# ===================== 一次获取连续小车坐标（含图像框选） =====================
def get_car_xy_continuous_once(img=None):
    global last_car_px, last_car_rc, last_car_bbox, last_track_miss

    if origin_rc is None:
        return None

    if img is None:
        img = sensor.snapshot()

    # ===== 下面保留你原来函数的其余代码，不要删 =====
    # 1) 先用上次位置附近快速跟踪
    # 2) 回退：单帧粗定位 C 所在格
    # 3) 最后回退：整张地图区域回搜
    # ... 原函数剩余代码全部照旧 ...

    # 1) 先用上次位置附近快速跟踪，优先用上一帧外接框自适应扩展ROI
    if last_car_px is not None:
        if last_car_bbox is not None:
            bx, by, bw, bh = last_car_bbox
            x0 = bx - CAR_ROI_MARGIN
            y0 = by - CAR_ROI_MARGIN
            x1 = bx + bw - 1 + CAR_ROI_MARGIN
            y1 = by + bh - 1 + CAR_ROI_MARGIN
            half = CAR_TRACK_HALF
            cx0 = int(last_car_px[0])
            cy0 = int(last_car_px[1])
            if x1 - x0 + 1 < half * 2 + 1:
                x0 = cx0 - half
                x1 = cx0 + half
            if y1 - y0 + 1 < half * 2 + 1:
                y0 = cy0 - half
                y1 = cy0 + half
            roi = (x0, y0, x1, y1)
        else:
            cx0 = int(last_car_px[0])
            cy0 = int(last_car_px[1])
            roi = (cx0 - CAR_TRACK_HALF, cy0 - CAR_TRACK_HALF,
                   cx0 + CAR_TRACK_HALF, cy0 + CAR_TRACK_HALF)

        res = car_centroid_in_roi(img, roi[0], roi[1], roi[2], roi[3])
        if res is not None:
            px, py, bbox = res
            xy = finalize_car_detection(img, px, py, bbox, roi=roi, rough_rc=None)
            if xy is not None:
                last_track_miss = 0
                return xy

    # 2) 回退：单帧粗定位 C 所在格
    grid1 = image_to_grid_once(img)
    coarse = find_first_char(grid1, 'C')
    if coarse is not None:
        res, roi = find_car_centroid_precise(img, coarse)
        if res is not None:
            px, py, bbox = res
            xy = finalize_car_detection(img, px, py, bbox, roi=roi, rough_rc=coarse)
            if xy is not None:
                last_track_miss = 0
                return xy

    # 3) 最后回退：连续丢失几次后再做整张地图区域回搜，避免每次都扫大ROI
    last_track_miss += 1
    if last_track_miss >= CAR_LOST_FULLSCAN_AFTER:
        roi = (map_px_min, map_py_min, map_px_max, map_py_max)
        res = car_centroid_in_roi(img, roi[0], roi[1], roi[2], roi[3])
        if res is not None:
            px, py, bbox = res
            xy = finalize_car_detection(img, px, py, bbox, roi=roi, rough_rc=None)
            if xy is not None:
                last_track_miss = 0
                return xy

    # 丢失时，如果还想看到上一次框，可以保留；这里不画
    last_car_bbox = None
    return None

# ===================== 地图/坐标发送 =====================
def send_map_only(grid_send):
    if SEND_TRIM_OUTER_WALL:                #条件成立时裁掉外层墙
        send_rows = []
        for r in range(1, ROWS - 1):
            send_rows.append("".join(grid_send[r][1:COLS - 1]))
    else:
        send_rows = []
        for r in range(ROWS):
            send_rows.append("".join(grid_send[r]))

    payload = "$MAP\r\n" + "\r\n".join(send_rows) + "\r\n$END\r\n"  #核心打包

    if PRINT_TX:             #是否打印到调试终端
        print("----- TX BEGIN -----")
        print(payload, end="")
        print("----- TX END -----")

    uart.write(payload)     #发送串口

def send_car_xy(x, y):
    x100 = int(x * CAR_SCALE + 0.5) if x >= 0 else -int((-x) * CAR_SCALE + 0.5)
    y100 = int(y * CAR_SCALE + 0.5) if y >= 0 else -int((-y) * CAR_SCALE + 0.5)
    uart.write("C%d,%d\r\n" % (x100, y100))
    if PRINT_CAR:
        print("[CAR] x=%.3f y=%.3f -> C%d,%d" % (x, y, x100, y100))


#发送角度
def send_car_angle_value(angle):
    a100 = int(angle * 100 + 0.5) if angle >= 0 else -int((-angle) * 100 + 0.5)
    uart.write("A%d\r\n" % a100)


def get_car_angle_from_img(img):
    pose = Angle_find_blue_green_blobs_in_roi(
        img,
        map_px_min, map_py_min,
        map_px_max, map_py_max
    )

    if pose is None:
        return None

    return pose.get("heading_deg")

#一次性发送角度和坐标


def send_car_xy_and_angle_once():
    img = sensor.snapshot()      #.lens_corr(strength=1.2, zoom=1.0)这里先不纠正畸变

    xy = get_car_xy_continuous_once(img)
    angle = get_car_angle_from_img(img)

    if xy is None:
        uart.write("C-1,-1\r\n")
        if PRINT_CAR:
            print("[CAR] lost")
    else:
        send_car_xy(xy[0], xy[1])

    if angle is None:
        uart.write("AE\r\n")
        print("[ANG] lost")
    else:
        send_car_angle_value(angle)
        print("[ANG] %.2f" % angle)

# ===================== 静默跟踪调度 =====================
def do_silent_track_once():
    # 只更新内部跟踪状态，不发串口
    return get_car_xy_continuous_once()

# ===================== START：全局地图识别 =====================
def do_start_fixedgrid_mapping():

    global origin_rc, last_car_px, last_car_rc, last_car_bbox, last_track_miss
    global bg_track_frame_cnt, track_boost_left
    global last_blue_center, last_green_center
    global last_heading_deg, last_heading_vx, last_heading_vy


    bg_track_frame_cnt = 0
    track_boost_left = 0
    last_track_miss = 0

    counts = bytearray(N_SAMPLES * N_CLS)
    need = int(VOTE_FRAMES * VOTE_MIN_RATIO + 0.5)

    sx = sample_x
    sy = sample_y
    pix_to_rgb = PIX_TO_RGB
    cls = classify_idx

    use_patch = (PATCH_RAD > 0)

    for t in range(VOTE_FRAMES):
        img = sensor.snapshot()
        getp = img.get_pixel

        if not use_patch:
            for idx in range(N_SAMPLES):
                r, g, b = pix_to_rgb(getp(sx[idx], sy[idx]))
                ci = cls(r, g, b)
                counts[idx * N_CLS + ci] += 1
        else:
            sampler = sample_rgb_median3x3 if (PATCH_USE_MEDIAN and PATCH_RAD == 1) else sample_rgb_mean
            for idx in range(N_SAMPLES):
                r, g, b = sampler(getp, pix_to_rgb, sx[idx], sy[idx])
                ci = cls(r, g, b)
                counts[idx * N_CLS + ci] += 1

        if DEBUG_OVERLAY and ((t % OVERLAY_EVERY) == 0):
            draw_sampling_overlay(img)
            draw_origin_overlay(img)

    grid_send = votes_to_grid_send(counts, need)

    # 建立原点：左下角障碍 '#'
    origin_rc = get_origin_rc(grid_send)

    # 初始小车位置
    coarse_car = find_first_char(grid_send, 'C')
    last_car_px = None
    last_car_rc = None
    last_car_bbox = None
    #初始朝向获取
    last_blue_center = None
    last_green_center = None
    last_heading_deg = None
    last_heading_vx = None
    last_heading_vy = None

    if coarse_car is not None:
        # 地图里已经打出C，按原来的方式精定位
        img2 = sensor.snapshot()
        res, roi = find_car_centroid_precise(img2, coarse_car)
        if res is not None:
            px, py, bbox = res
            last_car_px = (px, py)
            last_car_rc = coarse_car
            last_car_bbox = bbox

            row_f, col_f = pixel_to_continuous_rc(px, py, coarse_car[0], coarse_car[1])
            xy = rc_to_user_xy(row_f, col_f)
            if xy is not None and PRINT_CAR:
                print("[START CAR] x=%.3f y=%.3f" % (xy[0], xy[1]))

            if DRAW_CAR_BOX or DRAW_CAR_CENTER or DRAW_TRACK_ROI:
                draw_car_overlay(img2, bbox, (px, py), roi)
    else:
        # 地图里没打出C，就按动态找车方式找一次，
        # 然后把结果四舍五入成整数格，写回全局地图
        ok = start_find_car_and_write_back(grid_send)
        if ok and PRINT_CAR:
            print("[START CAR FALLBACK] rc=(%d,%d)" % (last_car_rc[0], last_car_rc[1]))
        elif PRINT_CAR:
            print("[START CAR FALLBACK] not found")

    if PRINT_MAP:
        if SEND_TRIM_OUTER_WALL:
            print("===== MAP(14x10) =====")
            for r in range(1, ROWS - 1):
                print("".join(grid_send[r][1:COLS - 1]))
        else:
            print("===== MAP(16x12) =====")
            for r in range(ROWS):
                print("".join(grid_send[r]))
        print("======================")
        if origin_rc is not None:
            print("[ORIGIN] left-bottom wall rc =", origin_rc)

    send_map_only(grid_send)

# ===================== 初始化 =====================
rebuild_sample_points()
detect_pixel_mode()
rebuild_patch_offsets()
build_output_centers()

idle_cnt = 0

# ===================== 主循环 =====================
while True:
    clock.tick()

    if IDLE_SNAPSHOT:
        img = sensor.snapshot().lens_corr(strength=1.2, zoom=1.0)
        #img.lens_corr(1.6)
        #地图获取调试
        do_start_fixedgrid_mapping()
        #if DEBUG_OVERLAY and ((idle_cnt % OVERLAY_EVERY) == 0):
        #    draw_sampling_overlay(img)
        #    draw_origin_overlay(img)
        idle_cnt += 1
    #else:
        #time.sleep_ms(2)

    uart_num = uart.any()
    if uart_num:
        data = uart.read(uart_num)
        if data:
            rx_buf += data

            if len(rx_buf) > 128:
                rx_buf = rx_buf[-128:]

            while b'\n' in rx_buf:
                line, rx_buf = rx_buf.split(b'\n', 1)
                line = line.strip()
                if not line:
                    continue

                if PRINT_UART:
                    print(line)

                if line == b'START':
                    carinit_mode = False
                    car_frame_cnt = 0
                    bg_track_frame_cnt = 0
                    track_boost_left = 0
                    last_track_miss = 0
                    do_start_fixedgrid_mapping()

                elif line == b'CARPOS':
                    send_car_xy_and_angle_once()
                    bg_track_frame_cnt = 0
                    track_boost_left = CARPOS_BOOST_CYCLES


                elif line == b'CARINIT':
                    print("gotcommd")   #调试是否获取到主控命令
                    carinit_mode = True
                    car_frame_cnt = 0
                    bg_track_frame_cnt = 0
                    track_boost_left = 0

                elif line == b'CARSTOP':
                    carinit_mode = False
                    bg_track_frame_cnt = 0
                    track_boost_left = 0

    if carinit_mode:
        car_frame_cnt += 1
        if car_frame_cnt >= CAR_SEND_EVERY:
                car_frame_cnt = 0
                send_car_xy_and_angle_once()
    elif origin_rc is not None:
        bg_track_frame_cnt += 1
        track_every = BOOST_TRACK_EVERY if track_boost_left > 0 else BG_TRACK_EVERY
        if bg_track_frame_cnt >= track_every:
            bg_track_frame_cnt = 0
            do_silent_track_once()
            if track_boost_left > 0:
                track_boost_left -= 1
