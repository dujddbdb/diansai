# -*- coding: utf-8 -*-
"""
K230 target detection + LCD preview + UART3 raw tracking output.

The sensor captures 1920x1080 frames. Detection runs on a 320x240
crop centered on the calibrated full-frame laser/origin point.
"""

import gc
import math
import os
import time
from machine import FPIOA, UART
from media.display import *
from media.media import *
from media.sensor import *


# 透视坐标映射: 将四边形内归一化坐标(u,v)映射到实际图像像素坐标
def perspective_map_point(corners, u, v):
    # 解包四个角点坐标并转浮点数
    p0, p1, p2, p3 = corners
    x0, y0 = float(p0[0]), float(p0[1])
    x1, y1 = float(p1[0]), float(p1[1])
    x2, y2 = float(p2[0]), float(p2[1])
    x3, y3 = float(p3[0]), float(p3[1])
    # 计算透视变换中间参数
    dx1, dy1 = x1 - x2, y1 - y2
    dx2, dy2 = x3 - x2, y3 - y2
    sx = x0 - x1 + x2 - x3
    sy = y0 - y1 + y2 - y3
    determinant = dx1 * dy2 - dx2 * dy1

    # 行列式接近零说明是平行四边形，g和h为0
    if abs(determinant) < 1.0e-9:
        g = h = 0.0
    else:
        # 计算透视变换系数g和h
        g = (sx * dy2 - dx2 * sy) / determinant
        h = (dx1 * sy - sx * dy1) / determinant

    # 计算线性变换系数a,b,d,e
    a = x1 - x0 + g * x1
    b = x3 - x0 + h * x3
    d = y1 - y0 + g * y1
    e = y3 - y0 + h * y3
    denominator = g * u + h * v + 1.0
    # 分母接近零时返回四边形中心作为兜底
    if abs(denominator) < 1.0e-9:
        return ((x0 + x1 + x2 + x3) * 0.25,
                (y0 + y1 + y2 + y3) * 0.25)
    # 执行透视变换并返回结果
    return ((a * u + b * v + x0) / denominator,
            (d * u + e * v + y0) / denominator)


ERR_INVALID = 0x7FFF


# 打包错误数据包: 将err_y和err_z打包为4字节大端格式串口数据
def pack_error_packet(err_y, err_z):
    # 截取低16位
    err_y &= 0xFFFF
    err_z &= 0xFFFF
    # 按大端序打包成4字节
    return bytes((err_y >> 8, err_y & 0xFF,
                  err_z >> 8, err_z & 0xFF))


# 尝试导入cv_lite加速库
try:
    import cv_lite
    HAS_CV_LITE = True
except ImportError:
    HAS_CV_LITE = False


# ============================================================
# 用户可调参数区
# ============================================================
# 图像与通信
IMG_W = 320
IMG_H = 240
CAM_W = 1920
CAM_H = 1080
CAM_FPS = 60
UART_BAUD = 115200

# Manual calibration: adjust this full-frame crop center until the real
# laser point appears at the center of the 320x240 detection image.
CROP_CENTER_X = CAM_W // 2
CROP_CENTER_Y = CAM_H // 2 - 77
CROP_X = max(0, min(CAM_W - IMG_W, CROP_CENTER_X - IMG_W // 2))
CROP_Y = max(0, min(CAM_H - IMG_H, CROP_CENTER_Y - IMG_H // 2))
CROP_ROI = (CROP_X, CROP_Y, IMG_W, IMG_H)

# 显示模式
HEADLESS = False
SHOW_TO_IDE = False
SENSOR_PIXFORMAT = "GRAYSCALE"
DISPLAY_EVERY_N = 3

# cv_lite 矩形检测器参数
CV_LOW = 20
CV_HIGH = 55
CV_EPSILON = 0.030
CV_MIN_AREA_RATIO = 0.002
CV_MAX_COS = 0.50
CV_MIN_EDGE = 6

# find_rects 回退方案
FIND_RECTS_THRESHOLD = 4500

# A4黑框靶标尺寸约束 (320x240检测裁剪)
OUTER_MIN_AREA = 900
OUTER_MAX_AREA = 70000
RECT_MIN_ANGLE = 48
ASPECT_MIN = 0.55
ASPECT_MAX = 1.75
BLACK_GRAY_TH = 95
WHITE_GRAY_TH = 135

# ROI追踪 (320x240检测裁剪)
TRACK_MARGIN = 56
TRACK_MARGIN_MAX = 160
TRACK_MARGIN_VEL_GAIN = 2
TRACK_MARGIN_X = 72
TRACK_MARGIN_X_MAX = 240
TRACK_MARGIN_X_VEL_GAIN = 3
TRACK_LOST_MARGIN_STEP = 20
TRACK_FAST_SPEED_PIXELS = 36
TRACK_FAST_FULL_RESCAN_PERIOD = 8
TRACK_PREDICT_ACCEPT_PIXELS = 70
TRACK_PREDICT_ACCEPT_GAIN = 1.2
TRACK_VELOCITY_SMOOTH_NUM = 2
TRACK_VELOCITY_SMOOTH_DEN = 3
FULL_RESCAN_PERIOD = 18
LOCK_HOLD_FRAMES = 14
FRAME_SCORE_MIN_ACQUIRE = 24
FRAME_SCORE_MIN_TRACK = 19
SWITCH_CONFIRM_FRAMES = 2
JUMP_REJECT_PIXELS = 120
AREA_CHANGE_REJECT = 0.45
CORNER_SMOOTH_NUM = 3
CORNER_SMOOTH_DEN = 5
OUTPUT_SMOOTH_NUM = 3
OUTPUT_SMOOTH_DEN = 5
OUTPUT_HOLD_FRAMES = 2

# 原点校准 (裁剪后激光点在画面正中心)
# The calibrated laser point is the crop image center. Do not run laser
# detection here; moving CROP_CENTER_X/Y is the calibration.
ORIGIN_X = IMG_W // 2
ORIGIN_Y = IMG_H // 2
TARGET_PLANE_U = 0.500
TARGET_PLANE_V = 0.500


# 计算控制误差: 目标点相对于原点的像素偏移量
def control_error(cx, cy):
    # err_y为水平方向差，err_z为竖直方向差(原点y减去目标y)
    return int(cx) - ORIGIN_X, ORIGIN_Y - int(cy)


# ============================================================
# 几何工具函数
# ============================================================

# 数值限幅: 将x限制在[lo, hi]范围内
def clamp(x, lo, hi):
    # 小于下限返回下限
    if x < lo:
        return lo
    # 大于上限返回上限
    if x > hi:
        return hi
    return x


# ROI裁剪: 将矩形ROI限制在图像范围内
def roi_clip(x, y, w, h, img_w=IMG_W, img_h=IMG_H):
    # 分别对x,y,w,h做限幅
    x = int(clamp(x, 0, img_w - 1))
    y = int(clamp(y, 0, img_h - 1))
    w = int(clamp(w, 1, img_w - x))
    h = int(clamp(h, 1, img_h - y))
    return (x, y, w, h)


# 激光中心ROI: 计算以激光点为中心的裁剪区域
def laser_center_roi():
    # 计算激光点在大图中的坐标
    laser_x = CAM_W // 2 + LASER_OFFSET_X
    laser_y = CAM_H // 2 + LASER_OFFSET_Y
    # 以激光点为中心裁剪IMG_W x IMG_H的区域
    return roi_clip(laser_x - IMG_W // 2,
                    laser_y - IMG_H // 2,
                    IMG_W,
                    IMG_H,
                    CAM_W,
                    CAM_H)


# 角点转外接矩形: 由四个角点计算带外边距的外接矩形
def corners_to_box(corners, margin=0):
    # 提取所有x和y坐标
    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    # 计算带边距的外接矩形
    x0 = min(xs) - margin
    y0 = min(ys) - margin
    x1 = max(xs) + margin
    y1 = max(ys) + margin
    # 裁剪到图像范围内
    return roi_clip(x0, y0, x1 - x0 + 1, y1 - y0 + 1)


# 角点转非对称外接矩形: x和y方向使用不同边距
def corners_to_asymmetric_box(corners, margin_x=0, margin_y=0):
    # 提取所有x和y坐标
    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    # 计算非对称边距的外接矩形
    x0 = min(xs) - margin_x
    y0 = min(ys) - margin_y
    x1 = max(xs) + margin_x
    y1 = max(ys) + margin_y
    # 裁剪到图像范围内
    return roi_clip(x0, y0, x1 - x0 + 1, y1 - y0 + 1)


# 角点排序: 将四个角点按左上、右上、右下、左下顺序排列
def order_corners(corners):
    # 转整数坐标
    pts = [(int(p[0]), int(p[1])) for p in corners]
    # 用x+y和y-x判断四个角
    s = [p[0] + p[1] for p in pts]
    d = [p[1] - p[0] for p in pts]
    tl = pts[s.index(min(s))]    # 左上: x+y最小
    br = pts[s.index(max(s))]    # 右下: x+y最大
    tr = pts[d.index(min(d))]    # 右上: y-x最小
    bl = pts[d.index(max(d))]    # 左下: y-x最大
    return [tl, tr, br, bl]


# 多边形面积: 用鞋带公式计算四边形面积
def polygon_area(corners):
    # 先排序角点
    pts = order_corners(corners)
    s = 0
    # 鞋带公式累加
    for i in range(4):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) & 3]
        s += x0 * y1 - x1 * y0
    # 取绝对值的一半
    if s < 0:
        s = -s
    return s // 2


# 验证矩形角度: 检查四边形四个角是否都大于最小角度
def validate_rect_angles(corners, min_angle=RECT_MIN_ANGLE):
    # 先排序角点
    pts = order_corners(corners)
    # 遍历四个顶点
    for i in range(4):
        p0 = pts[(i - 1) & 3]
        p1 = pts[i]
        p2 = pts[(i + 1) & 3]
        # 计算两条边向量
        v1 = (p0[0] - p1[0], p0[1] - p1[1])
        v2 = (p2[0] - p1[0], p2[1] - p1[1])
        # 计算向量模长
        m1 = math.sqrt(v1[0] * v1[0] + v1[1] * v1[1])
        m2 = math.sqrt(v2[0] * v2[0] + v2[1] * v2[1])
        # 边长太小跳过
        if m1 < 1.0 or m2 < 1.0:
            return False
        # 用点积计算夹角余弦
        c = (v1[0] * v2[0] + v1[1] * v2[1]) / (m1 * m2)
        c = max(-1.0, min(1.0, c))
        # 角度小于最小值则不通过
        if math.degrees(math.acos(c)) < min_angle:
            return False
    return True


# 获取灰度值: 安全获取图像某点的灰度值
def get_gray(img, x, y):
    # 尝试读取像素，异常时返回0
    try:
        p = img.get_pixel(int(x), int(y))
    except Exception:
        return 0
    # 元组格式(RGB)转灰度
    if isinstance(p, tuple):
        if len(p) >= 3:
            return (int(p[0]) * 76 + int(p[1]) * 150 + int(p[2]) * 29) >> 8
        if len(p) == 1:
            return int(p[0])
        return 0
    # RGB565格式转灰度
    if p > 255:
        r = (p >> 11) & 0x1F
        g = (p >> 5) & 0x3F
        b = p & 0x1F
        return (r * 76 + g * 150 + b * 29) >> 8
    return int(p)


# 线段暗像素采样: 计算线段上暗像素占比
def sample_line_dark(gray, p0, p1, n=9):
    dark = 0
    total = 0
    # 在两点间均匀采样n个点
    for i in range(n):
        t = i / (n - 1)
        x = int(p0[0] + (p1[0] - p0[0]) * t)
        y = int(p0[1] + (p1[1] - p0[1]) * t)
        # 只统计图像范围内的点
        if 0 <= x < IMG_W and 0 <= y < IMG_H:
            total += 1
            # 灰度低于阈值算暗像素
            if get_gray(gray, x, y) <= BLACK_GRAY_TH:
                dark += 1
    return dark / total if total else 0.0


# 黑框评分: 评估四边形是否为黑框靶标的得分
def black_frame_score(gray, corners):
    # 先排序角点
    pts = order_corners(corners)
    dark_edges = 0
    edge_ratio_sum = 0.0
    # 四条边分别采样暗像素比例
    for i in range(4):
        r = sample_line_dark(gray, pts[i], pts[(i + 1) & 3])
        edge_ratio_sum += r
        # 暗边计数
        if r >= 0.50:
            dark_edges += 1

    # 计算中心和尺寸
    cx = (pts[0][0] + pts[2][0]) // 2
    cy = (pts[0][1] + pts[2][1]) // 2
    box_w = max(1, max(p[0] for p in pts) - min(p[0] for p in pts))
    box_h = max(1, max(p[1] for p in pts) - min(p[1] for p in pts))
    # 内部采样步长
    sx = max(6, box_w // 6)
    sy = max(6, box_h // 6)
    inner_white = 0
    # 中心及上下左右五个点采样内部亮度
    for dx, dy in ((0, 0), (-sx, 0), (sx, 0), (0, -sy), (0, sy)):
        if get_gray(gray, clamp(cx + dx, 0, IMG_W - 1),
                    clamp(cy + dy, 0, IMG_H - 1)) >= WHITE_GRAY_TH:
            inner_white += 1
    # 综合得分: 暗边数*10 + 暗边比例和 + 内部白点*2
    return dark_edges * 10 + edge_ratio_sum + inner_white * 2.0


# 简单中心计算: 四个角点的平均坐标
def simple_center(corners):
    pts = order_corners(corners)
    # 四个角点坐标取平均
    return ((pts[0][0] + pts[1][0] + pts[2][0] + pts[3][0]) // 4,
            (pts[0][1] + pts[1][1] + pts[2][1] + pts[3][1]) // 4)


# 平移角点: 将所有角点平移(dx, dy)并限幅
def shift_corners(corners, dx, dy):
    shifted = []
    for p in order_corners(corners):
        # 平移并限制在图像范围内
        x = int(clamp(p[0] + dx, 0, IMG_W - 1))
        y = int(clamp(p[1] + dy, 0, IMG_H - 1))
        shifted.append((x, y))
    return shifted


# 角点平滑: 上一帧和当前帧角点做加权平滑
def smooth_corners(prev, curr):
    # 上一帧为空直接返回当前帧
    if prev is None:
        return order_corners(curr)
    out = []
    curr = order_corners(curr)
    prev = order_corners(prev)
    keep = CORNER_SMOOTH_DEN - CORNER_SMOOTH_NUM
    # 四个角点分别做加权平均
    for p, c in zip(prev, curr):
        x = (p[0] * keep + c[0] * CORNER_SMOOTH_NUM) // CORNER_SMOOTH_DEN
        y = (p[1] * keep + c[1] * CORNER_SMOOTH_NUM) // CORNER_SMOOTH_DEN
        out.append((int(x), int(y)))
    return out


# 中心距离: 两个四边形中心点的欧氏距离
def center_distance(a, b):
    ax, ay = simple_center(a)
    bx, by = simple_center(b)
    dx = ax - bx
    dy = ay - by
    return math.sqrt(dx * dx + dy * dy)


# 面积变化率: 两个四边形面积的相对变化比例
def area_change_ratio(a, b):
    aa = polygon_area(a)
    bb = polygon_area(b)
    # 面积无效返回1.0
    if aa <= 0 or bb <= 0:
        return 1.0
    d = aa - bb
    if d < 0:
        d = -d
    # 相对变化量 = 差值 / 较大面积
    return d / max(aa, bb)


# ============================================================
# 矩形检测器类
# ============================================================
class FastRectangleTracker:
    # 初始化追踪器状态
    def __init__(self):
        self.last_corners = None          # 上一帧角点
        self.pending_corners = None       # 待确认角点
        self.pending_hits = 0             # 待确认命中次数
        self.lost_frames = 999            # 丢帧计数
        self.frame_id = 0                 # 帧序号
        self.use_cv_lite = HAS_CV_LITE    # 是否使用cv_lite加速
        self.output_x = -1                # 输出x坐标
        self.output_y = -1                # 输出y坐标
        self.output_hold = 0              # 输出保持帧数
        self.vel_x = 0.0                  # x方向速度
        self.vel_y = 0.0                  # y方向速度
        self.last_center = None           # 上一帧中心

    # 计算追踪速度: 速度向量的模长
    def _track_speed(self):
        return math.sqrt(self.vel_x * self.vel_x + self.vel_y * self.vel_y)

    # 预测角点: 根据速度预测未来几帧的角点位置
    def _predicted_corners(self, frames_ahead=1):
        # 无上一帧数据无法预测
        if self.last_corners is None:
            return None
        if frames_ahead < 1:
            frames_ahead = 1
        # 按速度平移角点
        return shift_corners(self.last_corners,
                             self.vel_x * frames_ahead,
                             self.vel_y * frames_ahead)

    # 计算追踪ROI: 根据预测位置和速度动态调整搜索区域
    def _tracking_roi(self):
        # 预测丢帧后的位置
        predicted = self._predicted_corners(self.lost_frames + 1)
        # 无预测时返回全图
        if predicted is None:
            return (0, 0, IMG_W, IMG_H)
        # 根据速度动态调整边距
        margin_x = TRACK_MARGIN_X + int(abs(self.vel_x) * TRACK_MARGIN_X_VEL_GAIN)
        margin_y = TRACK_MARGIN + int(abs(self.vel_y) * TRACK_MARGIN_VEL_GAIN)
        # 丢帧越多边距越大
        margin_x += self.lost_frames * TRACK_LOST_MARGIN_STEP
        margin_y += self.lost_frames * TRACK_LOST_MARGIN_STEP
        # 边距限幅
        margin_x = int(clamp(margin_x, TRACK_MARGIN_X, TRACK_MARGIN_X_MAX))
        margin_y = int(clamp(margin_y, TRACK_MARGIN, TRACK_MARGIN_MAX))
        # 生成非对称外接矩形
        return corners_to_asymmetric_box(predicted, margin_x, margin_y)

    # 更新运动状态: 根据新角点更新速度估计
    def _update_motion(self, corners):
        cx, cy = simple_center(corners)
        # 有上一帧中心则更新速度
        if self.last_center is not None:
            dx = cx - self.last_center[0]
            dy = cy - self.last_center[1]
            keep = TRACK_VELOCITY_SMOOTH_DEN - TRACK_VELOCITY_SMOOTH_NUM
            # 速度加权平滑
            self.vel_x = (self.vel_x * keep + dx * TRACK_VELOCITY_SMOOTH_NUM) / TRACK_VELOCITY_SMOOTH_DEN
            self.vel_y = (self.vel_y * keep + dy * TRACK_VELOCITY_SMOOTH_NUM) / TRACK_VELOCITY_SMOOTH_DEN
        # 更新上一帧中心
        self.last_center = (cx, cy)

    # 重置运动状态: 速度和中心清零
    def _reset_motion(self):
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.last_center = None

    # cv_lite矩形检测: 使用硬件加速检测矩形
    def _cv_lite_rects(self, gray, roi):
        # 不使用cv_lite直接返回空
        if not self.use_cv_lite:
            return []
        x0, y0, rw, rh = roi
        try:
            # ROI是全图直接用原图，否则裁剪
            work = gray if (x0 == 0 and y0 == 0 and rw == IMG_W and rh == IMG_H) \
                else gray.copy(roi=roi)
            np_img = work.to_numpy_ref()
            # 调用cv_lite检测矩形角点
            rects = cv_lite.grayscale_find_rectangles_with_corners(
                [rh, rw], np_img,
                CV_LOW, CV_HIGH, CV_EPSILON,
                CV_MIN_AREA_RATIO, CV_MAX_COS, CV_MIN_EDGE
            )
        except Exception:
            return []
        out = []
        if not rects:
            return out
        # 过滤检测结果
        for r in rects:
            bw = int(r[2])
            bh = int(r[3])
            box_area = bw * bh
            # 面积过滤
            if box_area < OUTER_MIN_AREA or box_area > OUTER_MAX_AREA:
                continue
            if bw <= 0 or bh <= 0:
                continue
            # 宽高比过滤
            aspect = bw / bh
            if aspect < ASPECT_MIN or aspect > ASPECT_MAX:
                continue
            # 转换为原图坐标
            corners = [
                (int(r[4]) + x0, int(r[5]) + y0),
                (int(r[6]) + x0, int(r[7]) + y0),
                (int(r[8]) + x0, int(r[9]) + y0),
                (int(r[10]) + x0, int(r[11]) + y0),
            ]
            # 角度验证
            if not validate_rect_angles(corners):
                continue
            # 多边形面积过滤
            area = polygon_area(corners)
            if area < OUTER_MIN_AREA or area > OUTER_MAX_AREA:
                continue
            out.append((order_corners(corners), area))
        return out

    # find_rects检测: 使用内置find_rects函数检测矩形
    def _find_rects(self, gray, roi):
        x0, y0, rw, rh = roi
        try:
            # 全图检测
            if x0 == 0 and y0 == 0 and rw == IMG_W and rh == IMG_H:
                rects = gray.find_rects(threshold=FIND_RECTS_THRESHOLD)
                offset_x = 0
                offset_y = 0
            else:
                # 尝试带roi参数检测
                try:
                    rects = gray.find_rects(threshold=FIND_RECTS_THRESHOLD, roi=roi)
                    offset_x = 0
                    offset_y = 0
                except TypeError:
                    # 不支持roi参数则先裁剪再检测
                    work = gray.copy(roi=roi)
                    rects = work.find_rects(threshold=FIND_RECTS_THRESHOLD)
                    offset_x = x0
                    offset_y = y0
        except Exception:
            return []
        out = []
        if not rects:
            return out
        # 过滤检测结果
        for r in rects:
            try:
                corners_roi = r.corners()
                bx, by, bw, bh = r.rect()
            except Exception:
                continue
            # 面积过滤
            box_area = int(bw) * int(bh)
            if box_area < OUTER_MIN_AREA or box_area > OUTER_MAX_AREA:
                continue
            # 宽高比过滤
            aspect = bw / max(1, bh)
            if aspect < ASPECT_MIN or aspect > ASPECT_MAX:
                continue
            # 转换为原图坐标
            corners = [(int(p[0]) + offset_x, int(p[1]) + offset_y)
                       for p in corners_roi]
            # 角度验证
            if not validate_rect_angles(corners):
                continue
            out.append((order_corners(corners), polygon_area(corners)))
        return out

    # 快速blob检测: 兼容不同版本的find_blobs调用
    def _find_blobs_fast(self, img, thresholds, **kwargs):
        try:
            return img.find_blobs(thresholds, x_stride=2, y_stride=2, **kwargs)
        except TypeError:
            return img.find_blobs(thresholds, **kwargs)

    # blob矩形检测: 通过色块检测得到矩形候选
    def _blob_rects(self, gray, roi):
        x0, y0, rw, rh = roi
        try:
            # 全图检测
            if x0 == 0 and y0 == 0 and rw == IMG_W and rh == IMG_H:
                blobs = self._find_blobs_fast(
                    gray,
                    [(0, BLACK_GRAY_TH)],
                    pixels_threshold=240,
                    area_threshold=240,
                    merge=True
                )
                offset_x = 0
                offset_y = 0
            else:
                # 尝试带roi参数检测
                try:
                    blobs = self._find_blobs_fast(
                        gray,
                        [(0, BLACK_GRAY_TH)],
                        roi=roi,
                        pixels_threshold=240, area_threshold=240, merge=True
                    )
                    offset_x = 0
                    offset_y = 0
                except TypeError:
                    # 不支持roi参数则先裁剪再检测
                    work = gray.copy(roi=roi)
                    blobs = self._find_blobs_fast(
                        work,
                        [(0, BLACK_GRAY_TH)], pixels_threshold=240,
                        area_threshold=240, merge=True
                    )
                    offset_x = x0
                    offset_y = y0
        except Exception:
            return []
        out = []
        if not blobs:
            return out
        # 过滤blob结果
        for b in blobs:
            bx, by, bw, bh = b.rect()
            area = int(bw) * int(bh)
            # 面积过滤
            if area < OUTER_MIN_AREA or area > OUTER_MAX_AREA:
                continue
            # 宽高比过滤
            aspect = bw / max(1, bh)
            if aspect < ASPECT_MIN or aspect > ASPECT_MAX:
                continue
            # 密度过滤
            try:
                density = b.density()
                if density < 0.08 or density > 0.55:
                    continue
            except Exception:
                pass
            # 用外接矩形四个角作为候选
            corners = order_corners([
                (bx + offset_x, by + offset_y),
                (bx + bw + offset_x, by + offset_y),
                (bx + bw + offset_x, by + bh + offset_y),
                (bx + offset_x, by + bh + offset_y),
            ])
            out.append((corners, area))
        return out

    # ROI扫描: 按优先级依次使用多种检测方法
    def _scan_roi(self, gray, roi):
        # 判断是否在追踪模式
        tracking_roi = (
            self.last_corners is not None and
            self.lost_frames < LOCK_HOLD_FRAMES and
            roi != (0, 0, IMG_W, IMG_H)
        )
        # 追踪模式下先用blob快速检测
        if tracking_roi:
            cands = self._blob_rects(gray, roi)
            if cands:
                return cands
        # 优先用cv_lite检测
        cands = self._cv_lite_rects(gray, roi)
        # 回退到blob检测
        if not cands:
            cands = self._blob_rects(gray, roi)
        # 再回退到find_rects检测
        if not cands:
            cands = self._find_rects(gray, roi)
        return cands

    # 选择最佳候选: 从候选矩形中选出得分最高的
    def _choose_best(self, gray, cands):
        if not cands:
            return None
        best = None
        best_score = -1
        # 遍历所有候选计算得分
        for corners, area in cands:
            # 黑框评分
            frame_score = black_frame_score(gray, corners)
            # 根据是否追踪中使用不同最低分阈值
            min_score = FRAME_SCORE_MIN_TRACK if self.last_corners is not None \
                else FRAME_SCORE_MIN_ACQUIRE
            if frame_score < min_score:
                continue
            cx, cy = simple_center(corners)
            # 基础得分 = 面积加权 + 黑框评分
            score = area * 0.001 + frame_score
            # 追踪中额外加位置和面积变化惩罚
            if self.last_corners is not None:
                # 用预测位置或上一帧位置做参考
                ref = self._predicted_corners(1)
                if ref is None:
                    ref = self.last_corners
                lx, ly = simple_center(ref)
                dist = abs(cx - lx) + abs(cy - ly)
                ar = area_change_ratio(self.last_corners, corners)
                # 距离近加分，面积变化减分
                score += max(0.0, 18.0 - dist * 0.18)
                score -= ar * 18.0
            else:
                # 初次检测时偏向图像中心
                center_penalty = (abs(cx - IMG_W // 2) +
                                  abs(cy - IMG_H // 2)) * 0.01
                score -= center_penalty
            # 更新最高分
            if score > best_score:
                best_score = score
                best = (corners, area)
        return best

    # 接受候选验证: 判断新候选是否可以替换当前追踪目标
    def _accept_candidate(self, corners):
        corners = order_corners(corners)
        # 已有追踪目标时做跳跃和面积变化检测
        if self.last_corners is not None:
            jump = center_distance(self.last_corners, corners)
            ar = area_change_ratio(self.last_corners, corners)
            # 跳跃过大或面积变化过大
            if jump > JUMP_REJECT_PIXELS or ar > AREA_CHANGE_REJECT:
                # 用预测位置验证是否可接受
                predicted = self._predicted_corners(self.lost_frames + 1)
                if predicted is not None:
                    predict_jump = center_distance(predicted, corners)
                    predict_limit = TRACK_PREDICT_ACCEPT_PIXELS + self._track_speed() * TRACK_PREDICT_ACCEPT_GAIN
                    # 预测位置近且面积变化小可接受
                    if predict_jump <= predict_limit and ar <= AREA_CHANGE_REJECT:
                        self.pending_corners = None
                        self.pending_hits = 0
                        return smooth_corners(self.last_corners, corners), True
                # 待确认机制：连续几帧都检测到同一目标才切换
                if self.pending_corners is not None and \
                        center_distance(self.pending_corners, corners) < 36:
                    self.pending_hits += 1
                else:
                    self.pending_corners = corners
                    self.pending_hits = 1
                # 确认帧数不足，继续用旧目标
                if self.pending_hits < SWITCH_CONFIRM_FRAMES:
                    return self.last_corners, False
            # 通过验证，平滑后接受
            self.pending_corners = None
            self.pending_hits = 0
            return smooth_corners(self.last_corners, corners), True
        # 首次检测也需要确认几帧
        if self.pending_corners is not None and \
                center_distance(self.pending_corners, corners) < 36:
            self.pending_hits += 1
        else:
            self.pending_corners = corners
            self.pending_hits = 1
        # 确认帧数不足
        if self.pending_hits < SWITCH_CONFIRM_FRAMES:
            if self.last_corners is None:
                return corners, False
            return None, False
        # 确认通过
        self.pending_corners = None
        self.pending_hits = 0
        return corners, True

    # 主检测函数: 执行一帧检测并返回目标坐标
    def detect(self, gray):
        self.frame_id += 1
        full_roi = (0, 0, IMG_W, IMG_H)
        rois = []
        # 追踪中先在预测ROI内检测
        if self.last_corners is not None and self.lost_frames < LOCK_HOLD_FRAMES:
            rois.append(self._tracking_roi())
        # 是否需要全图扫描
        need_full = self.last_corners is None or self.lost_frames >= LOCK_HOLD_FRAMES
        # 周期性全图重扫
        if FULL_RESCAN_PERIOD and self.last_corners is not None:
            rescan_period = FULL_RESCAN_PERIOD
            # 高速运动时缩短重扫周期
            if self._track_speed() >= TRACK_FAST_SPEED_PIXELS:
                rescan_period = TRACK_FAST_FULL_RESCAN_PERIOD
            need_full = need_full or (self.frame_id % rescan_period) == 0
        if need_full:
            rois.append(full_roi)
        # 按ROI顺序检测
        for roi in rois:
            # 扫描ROI并选最佳
            best = self._choose_best(gray, self._scan_roi(gray, roi))
            if best is not None:
                corners, _ = best
                # 验证候选是否可接受
                accepted, fresh = self._accept_candidate(corners)
                if accepted is None:
                    break
                # 新目标更新运动状态
                if fresh:
                    self._update_motion(corners)
                # 更新追踪状态
                self.last_corners = accepted
                self.lost_frames = 0
                # 透视映射得到目标点
                cx, cy = perspective_map_point(self.last_corners,
                                                TARGET_PLANE_U, TARGET_PLANE_V)
                cx, cy = int(cx + 0.5), int(cy + 0.5)
                return cx, cy, self.last_corners, fresh
        # 未检测到但在保持期内，继续用旧结果
        if self.last_corners is not None and self.lost_frames < LOCK_HOLD_FRAMES:
            self.lost_frames += 1
            cx, cy = perspective_map_point(self.last_corners,
                                            TARGET_PLANE_U, TARGET_PLANE_V)
            cx, cy = int(cx + 0.5), int(cy + 0.5)
            return cx, cy, self.last_corners, False
        # 完全丢失，重置状态
        self.last_corners = None
        self.pending_corners = None
        self.pending_hits = 0
        self.lost_frames = 999
        self._reset_motion()
        return -1, -1, None, False

    # 稳定输出: 对输出坐标做平滑和保持
    def _stable_output(self, x, y, valid):
        # 有效输入时更新平滑输出
        if valid and x >= 0 and y >= 0:
            # 首次有效直接赋值
            if self.output_x < 0 or self.output_y < 0:
                self.output_x = int(x)
                self.output_y = int(y)
            else:
                # 加权平滑
                keep = OUTPUT_SMOOTH_DEN - OUTPUT_SMOOTH_NUM
                self.output_x = (self.output_x * keep + int(x) * OUTPUT_SMOOTH_NUM) // OUTPUT_SMOOTH_DEN
                self.output_y = (self.output_y * keep + int(y) * OUTPUT_SMOOTH_NUM) // OUTPUT_SMOOTH_DEN
            # 重置保持计数
            self.output_hold = OUTPUT_HOLD_FRAMES
            return int(self.output_x), int(self.output_y)
        # 无效但在保持期内，继续输出旧值
        if self.output_hold > 0 and self.output_x >= 0 and self.output_y >= 0:
            self.output_hold -= 1
            return int(self.output_x), int(self.output_y)
        # 保持期过了，输出无效
        self.output_x = -1
        self.output_y = -1
        return -1, -1


# ============================================================
# LCD叠加绘制
# ============================================================

# 绘制坐标轴箭头: 在指定位置绘制带箭头的线段
def draw_axis_arrow(img, x0, y0, x1, y1, color):
    # 画轴线
    img.draw_line(x0, y0, x1, y1, color=color, thickness=2)
    # 根据方向画箭头（向右）
    if x1 > x0:
        img.draw_line(x1, y1, x1 - 8, y1 - 4, color=color, thickness=2)
        img.draw_line(x1, y1, x1 - 8, y1 + 4, color=color, thickness=2)
    # 向左
    elif x1 < x0:
        img.draw_line(x1, y1, x1 + 8, y1 - 4, color=color, thickness=2)
        img.draw_line(x1, y1, x1 + 8, y1 + 4, color=color, thickness=2)
    # 向上
    elif y1 < y0:
        img.draw_line(x1, y1, x1 - 4, y1 + 8, color=color, thickness=2)
        img.draw_line(x1, y1, x1 + 4, y1 + 8, color=color, thickness=2)
    # 向下
    else:
        img.draw_line(x1, y1, x1 - 4, y1 - 8, color=color, thickness=2)
        img.draw_line(x1, y1, x1 + 4, y1 - 8, color=color, thickness=2)


# 绘制叠加层: 在图像上绘制坐标轴、靶框、误差线和文字信息
def draw_overlay(img, corners, target_x, target_y, fps, fresh, err_y, err_z):
    # 定义颜色常量
    GREEN  = (0, 255, 0)
    RED    = (255, 0, 0)
    YELLOW = (255, 255, 0)
    WHITE  = (255, 255, 255)
    ORANGE = (255, 180, 0)
    CYAN   = (0, 255, 255)
    GRAY   = (128, 128, 128)
    AXIS_LEN = 42

    # 原点坐标
    origin_x = ORIGIN_X
    origin_y = ORIGIN_Y
    # 框颜色：新检测到用绿色，保持用橙色
    frame_color = GREEN if fresh else ORANGE

    # 画十字坐标轴（灰色）
    img.draw_line(origin_x - AXIS_LEN, origin_y, origin_x + AXIS_LEN, origin_y,
                  color=GRAY, thickness=1)
    img.draw_line(origin_x, origin_y - AXIS_LEN, origin_x, origin_y + AXIS_LEN,
                  color=GRAY, thickness=1)
    # 画带箭头的正方向轴（+Y青色，+Z黄色）
    draw_axis_arrow(img, origin_x, origin_y, origin_x + AXIS_LEN, origin_y, CYAN)
    draw_axis_arrow(img, origin_x, origin_y, origin_x, origin_y - AXIS_LEN, YELLOW)

    # 画原点标记（绿色圆圈+十字）
    img.draw_circle(origin_x, origin_y, 5, color=GREEN, thickness=2)
    img.draw_cross(origin_x, origin_y, color=GREEN, size=7, thickness=1)

    # 检测到目标时绘制靶框和误差线
    if corners is not None and target_x >= 0:
        # 画四边形靶框
        for i in range(4):
            x0, y0 = corners[i]
            x1, y1 = corners[(i + 1) & 3]
            img.draw_line(int(x0), int(y0), int(x1), int(y1),
                          color=frame_color, thickness=2)
        # 画目标点（红色圆圈）
        img.draw_circle(int(target_x), int(target_y), 4,
                        color=RED, thickness=2)
        # 画水平误差线（青色）
        img.draw_line(origin_x, origin_y, int(target_x), origin_y,
                      color=CYAN, thickness=1)
        # 画竖直误差线（黄色）
        img.draw_line(int(target_x), origin_y, int(target_x), int(target_y),
                      color=YELLOW, thickness=1)
        # 画原点到目标的连线（白色）
        img.draw_line(origin_x, origin_y, int(target_x), int(target_y),
                      color=WHITE, thickness=1)

    # 画文字信息
    try:
        img.draw_string_advanced(4, 4, 14, "FPS:%d" % fps, color=WHITE)
        img.draw_string_advanced(4, 22, 14, "ERR_Y:%d ERR_Z:%d" % (err_y, err_z), color=WHITE)
        img.draw_string_advanced(4, 40, 14, "O=(%d,%d)" % (origin_x, origin_y), color=WHITE)
        img.draw_string_advanced(origin_x + AXIS_LEN + 4, origin_y - 8, 14, "+Y", color=CYAN)
        img.draw_string_advanced(origin_x - 10, origin_y - AXIS_LEN - 18, 14, "+Z", color=YELLOW)
        img.draw_string_advanced(origin_x + 6, origin_y + 6, 14, "O", color=GREEN)
    except Exception:
        pass


# ============================================================
# UART3通信协议
# ============================================================

# 发送追踪数据: 通过串口发送误差数据包
def send_tracking(uart, err_y, err_z, fps=0):
    # 打包并发送4字节误差数据
    uart.write(pack_error_packet(int(err_y), int(err_z)))


# 初始化串口: 配置UART3引脚和波特率
def init_uart():
    # 配置FPIOA引脚复用
    fpioa = FPIOA()
    fpioa.set_function(50, FPIOA.UART3_TXD)
    fpioa.set_function(51, FPIOA.UART3_RXD)
    # 初始化UART3
    return UART(UART.UART3, baudrate=UART_BAUD,
                bits=UART.EIGHTBITS,
                parity=UART.PARITY_NONE,
                stop=UART.STOPBITS_ONE)


# 初始化相机: 配置传感器分辨率、帧率和像素格式
def init_camera():
    # 创建传感器对象，兼容不同API
    try:
        sensor = Sensor(width=CAM_W, height=CAM_H, fps=CAM_FPS)
    except TypeError:
        sensor = Sensor(width=CAM_W, height=CAM_H)
    # 复位并设置分辨率
    sensor.reset()
    sensor.set_framesize(width=CAM_W, height=CAM_H)
    # 根据配置设置像素格式
    if not HEADLESS:
        # 非无头模式用RGB565
        sensor.set_pixformat(Sensor.RGB565)
        pixfmt = "RGB565"
    elif SENSOR_PIXFORMAT == "RGB565":
        sensor.set_pixformat(Sensor.RGB565)
        pixfmt = "RGB565"
    else:
        # 尝试灰度格式，失败回退到RGB565
        try:
            sensor.set_pixformat(Sensor.GRAYSCALE)
            pixfmt = "GRAYSCALE"
        except Exception:
            sensor.set_pixformat(Sensor.RGB565)
            pixfmt = "RGB565"
    return sensor, pixfmt


# ============================================================
# 主入口
# ============================================================
sensor = None
uart = None
has_display = False

try:
    # 初始化串口
    uart = init_uart()
    # 初始化相机
    sensor, pixfmt = init_camera()

    # 初始化LCD显示
    has_display = False
    if not HEADLESS:
        try:
            Display.init(Display.ST7701, width=800, height=480,
                         to_ide=SHOW_TO_IDE)
            has_display = True
        except Exception:
            pass

    # 初始化媒体管理和启动传感器
    MediaManager.init()
    sensor.run()

    # 创建追踪器和时钟
    tracker = FastRectangleTracker()
    clock = time.clock()
    gc_count = 0
    display_count = 0

    # 初始化目标坐标和角点
    target_x, target_y = -1, -1
    corners = None

    # 主循环
    while True:
        # 计时开始
        clock.tick()
        # 检查退出点
        os.exitpoint()
        # 采集一帧图像
        img = sensor.snapshot()

        # 以激光点为中心裁剪检测画面
        crop_roi = laser_center_roi()
        view = img.copy(roi=crop_roi)

        # 图像预处理：灰度化
        if pixfmt == "GRAYSCALE":
            gray = view
        else:
            gray = view.to_grayscale(copy=True)

        # 目标检测追踪
        target_x, target_y, corners, fresh = tracker.detect(gray)
        # 输出平滑处理
        stable_x, stable_y = tracker._stable_output(target_x, target_y, fresh)
        target_valid = stable_x >= 0 and stable_y >= 0

        # 误差计算与串口发送
        if target_valid:
            # 计算控制误差
            cx = stable_x
            cy = stable_y
            err_y, err_z = control_error(cx, cy)
            send_y = err_y
            send_z = err_z
        else:
            # 目标丢失时发送无效值
            send_y = ERR_INVALID
            send_z = ERR_INVALID
        # 计算帧率
        fps = int(clock.fps() + 0.5)
        # 通过串口发送误差数据
        send_tracking(uart, send_y, send_z, fps if target_valid else 0)

        # LCD显示更新
        if has_display:
            display_count += 1
            # 每隔N帧更新一次显示
            if display_count >= DISPLAY_EVERY_N:
                display_count = 0
                # 绘制叠加层
                draw_overlay(view, corners, target_x, target_y, fps, fresh, send_y, send_z)
                # 居中显示图像
                Display.show_image(view, x=(800 - IMG_W) // 2, y=(480 - IMG_H) // 2)

        # 垃圾回收
        gc_count += 1
        if gc_count >= 60:
            gc_count = 0
            gc.collect()

except KeyboardInterrupt as e:
    pass
except BaseException as e:
    print("err:", e)
finally:
    # 停止传感器
    if isinstance(sensor, Sensor):
        sensor.stop()
    # 关闭显示
    try:
        Display.deinit()
    except Exception:
        pass
    # 退出点设置
    try:
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    except Exception:
        pass
    time.sleep_ms(100)
    # 关闭媒体管理
    try:
        MediaManager.deinit()
    except Exception:
        pass
    # 关闭串口
    if uart:
        uart.deinit()
