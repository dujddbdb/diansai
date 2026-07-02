# -*- coding: utf-8 -*-
# K230目标检测 + LCD预览 + UART3原始跟踪输出
# 传感器采集1920x1080帧，在以校准后的全帧激光点/原点为中心的320x240裁剪区域上进行检测

import gc
import math
import os
import time
from machine import FPIOA, UART
from media.display import *
from media.media import *
from media.sensor import *

# 透视映射函数：将四边形内归一化坐标(u,v)映射到实际图像坐标
# 参数:
#   corners: 四边形四个角点坐标列表 [p0, p1, p2, p3]
#   u: 水平方向归一化坐标 (0.0~1.0)
#   v: 垂直方向归一化坐标 (0.0~1.0)
# 返回值: (x, y) 映射后的实际像素坐标
def perspective_map_point(corners, u, v):
    # 提取四个角点坐标并转为浮点数
    p0, p1, p2, p3 = corners
    x0, y0 = float(p0[0]), float(p0[1])
    x1, y1 = float(p1[0]), float(p1[1])
    x2, y2 = float(p2[0]), float(p2[1])
    x3, y3 = float(p3[0]), float(p3[1])

    # 计算透视变换矩阵参数
    dx1, dy1 = x1 - x2, y1 - y2
    dx2, dy2 = x3 - x2, y3 - y2
    sx = x0 - x1 + x2 - x3
    sy = y0 - y1 + y2 - y3
    determinant = dx1 * dy2 - dx2 * dy1  # 行列式，判断是否退化为平行四边形

    # 行列式接近0时退化为仿射变换
    if abs(determinant) < 1.0e-9:
        g = h = 0.0
    else:
        g = (sx * dy2 - dx2 * sy) / determinant
        h = (dx1 * sy - sx * dy1) / determinant

    # 计算变换系数
    a = x1 - x0 + g * x1
    b = x3 - x0 + h * x3
    d = y1 - y0 + g * y1
    e = y3 - y0 + h * y3
    denominator = g * u + h * v + 1.0  # 透视分母

    # 分母接近0时返回四边形中心作为保护
    if abs(denominator) < 1.0e-9:
        return ((x0 + x1 + x2 + x3) * 0.25,
                (y0 + y1 + y2 + y3) * 0.25)

    # 执行透视变换并返回结果
    return ((a * u + b * v + x0) / denominator,
            (d * u + e * v + y0) / denominator)

ERR_INVALID = 0x7FFF  # 无效误差值标记，表示目标丢失

# 打包误差数据包：将err_y和err_z打包为4字节大端格式
# 参数:
#   err_y: Y轴方向误差值
#   err_z: Z轴方向误差值
# 返回值: 4字节的bytes对象，大端格式
def pack_error_packet(err_y, err_z):
    # 确保数值在16位有符号整数范围内
    err_y &= 0xFFFF
    err_z &= 0xFFFF
    # 按大端字节序打包
    return bytes((err_y >> 8, err_y & 0xFF,
                  err_z >> 8, err_z & 0xFF))

try:
    import cv_lite
    HAS_CV_LITE = True
except ImportError:
    HAS_CV_LITE = False


# ============================================================
# 用户可调参数区
# ============================================================
# 图像与通信
IMG_W = 320  # 检测图像宽度(像素)
IMG_H = 240  # 检测图像高度(像素)
CAM_W = 1920  # 摄像头全帧宽度(像素)
CAM_H = 1080  # 摄像头全帧高度(像素)
CAM_FPS = 60  # 摄像头帧率，调大则帧率高但CPU占用高，调小则反之
UART_BAUD = 115200  # 串口波特率
FULL_CHN = CAM_CHN_ID_0  # 摄像头完整输入通道，保持1920x1080
DETECT_CHN = CAM_CHN_ID_1  # 硬件裁剪识别通道，输出320x240

# 手动校准：调整全帧裁剪中心，直到真实激光点出现在320x240检测图像的中心
CROP_CENTER_X = CAM_W // 2 + 2 # 裁剪区域中心X坐标(全帧坐标系)
CROP_CENTER_Y = CAM_H // 2 - 88  # 裁剪区域中心Y坐标(全帧坐标系)，调大则裁剪区域下移，调小则上移
CROP_X = max(0, min(CAM_W - IMG_W, CROP_CENTER_X - IMG_W // 2))  # 裁剪区域左上角X
CROP_Y = max(0, min(CAM_H - IMG_H, CROP_CENTER_Y - IMG_H // 2))  # 裁剪区域左上角Y
CROP_ROI = (CROP_X, CROP_Y, IMG_W, IMG_H)  # 裁剪区域元组

# 显示模式
HEADLESS = False  # 无头模式，True则不初始化LCD显示
SHOW_TO_IDE = True  # 是否输出图像到IDE预览
SENSOR_PIXFORMAT = "GRAYSCALE"  # 传感器像素格式
DISPLAY_EVERY_N = 2  # 每N帧刷新一次显示，调大则显示帧率低但节省CPU，调小则反之

# cv_lite矩形检测器参数
CV_LOW = 30  # Canny边缘检测低阈值，调小则检测到更多边缘但噪声增多，调大则反之
CV_HIGH = 65  # Canny边缘检测高阈值，调小则边缘连接更宽松，调大则更严格
CV_EPSILON = 0.030  # 多边形逼近精度系数，调大则轮廓更简化，调小则更精细
CV_MIN_AREA_RATIO = 0.002  # 最小面积占比，调大则过滤更多小矩形
CV_MAX_COS = 0.50  # 最大角余弦值，用于判断直角，调大则角度要求更宽松
CV_MIN_EDGE = 90  # 最小边长(像素)

# find_rects回退方案
FIND_RECTS_THRESHOLD = 1000  # 矩形检测阈值，调大则检测更少但更精准，调小则反之
CV_LITE_FALLBACK_PERIOD = 12  # cv_lite较重，仅在Blob失败时降频回退
FIND_RECTS_FALLBACK_PERIOD = 30  # find_rects最慢，仅低频兜底
BLOB_X_STRIDE = 3  # Blob水平步进，调大速度更快但细线更容易漏检
BLOB_Y_STRIDE = 3  # Blob垂直步进，调大速度更快但细线更容易漏检

# A4黑框靶标尺寸约束 (320x240检测裁剪)
OUTER_MIN_AREA = 900  # 外框最小面积(像素²)
OUTER_MAX_AREA = 90000  # 外框最大面积(像素²)
RECT_MIN_ANGLE = 48  # 矩形最小角度(度)，调大则对矩形角度要求更严格
ASPECT_MIN = 0.55  # 最小宽高比
ASPECT_MAX = 1.75  # 最大宽高比
BLACK_GRAY_TH = 86  # 黑色灰度阈值，调大则更多像素被判定为黑色，调小则反之
WHITE_GRAY_TH = 130  # 白色灰度阈值，调大则判定白色更严格，调小则更宽松

# ROI追踪参数 (320x240检测裁剪)
TRACK_MARGIN = 56  # Y方向追踪边距基础值，调大则追踪范围大但速度慢
TRACK_MARGIN_MAX = 160  # Y方向追踪边距最大值
TRACK_MARGIN_VEL_GAIN = 2  # Y方向速度增益系数，速度越快边距越大
TRACK_MARGIN_X = 72  # X方向追踪边距基础值
TRACK_MARGIN_X_MAX = 240  # X方向追踪边距最大值
TRACK_MARGIN_X_VEL_GAIN = 3  # X方向速度增益系数
TRACK_LOST_MARGIN_STEP = 20  # 丢失帧数每增加1帧，边距增加的像素数
TRACK_FAST_SPEED_PIXELS = 36  # 快速移动判定阈值(像素/帧)
TRACK_FAST_FULL_RESCAN_PERIOD = 18  # 快速移动时全帧重扫周期(帧)
TRACK_PREDICT_ACCEPT_PIXELS = 70  # 预测位置接受半径基础值(像素)
TRACK_PREDICT_ACCEPT_GAIN = 1.2  # 预测位置接受半径速度增益
TRACK_VELOCITY_SMOOTH_NUM = 2  # 速度平滑分子，调大则新速度权重高、响应快但抖动大
TRACK_VELOCITY_SMOOTH_DEN = 3  # 速度平滑分母
FULL_RESCAN_PERIOD = 45  # 全帧重扫周期(帧)，调小则更频繁全扫但耗时多
LOCK_HOLD_FRAMES = 10  # 锁定保持帧数，目标丢失后保持追踪的帧数
FRAME_SCORE_MIN_ACQUIRE = 24  # 捕获目标的最低分数阈值
FRAME_SCORE_MIN_TRACK = 19  # 追踪目标的最低分数阈值
SWITCH_CONFIRM_FRAMES = 2  # 切换目标确认帧数，需要连续N帧才确认切换
JUMP_REJECT_PIXELS = 120  # 跳跃拒绝距离(像素)，超过则认为是误检
AREA_CHANGE_REJECT = 0.45  # 面积变化拒绝比例，变化超过则拒绝
CORNER_SMOOTH_NUM = 3  # 角点平滑分子，调大则新角点权重高、响应快但抖动大
CORNER_SMOOTH_DEN = 5  # 角点平滑分母
OUTPUT_SMOOTH_NUM = 3  # 输出平滑分子，调大则输出响应快但抖动大
OUTPUT_SMOOTH_DEN = 5  # 输出平滑分母
OUTPUT_HOLD_FRAMES = 2  # 输出保持帧数，目标丢失后保持输出的帧数

# 原点校准 (裁剪后激光点在画面正中心)
# 校准后的激光点就是裁剪图像中心，通过移动CROP_CENTER_X/Y来校准
ORIGIN_X = IMG_W // 2  # 原点X坐标(裁剪图像坐标系)
ORIGIN_Y = IMG_H // 2  # 原点Y坐标(裁剪图像坐标系)
TARGET_PLANE_U = 0.500  # 目标平面归一化U坐标
TARGET_PLANE_V = 0.500  # 目标平面归一化V坐标

# 计算控制误差：目标中心相对于原点的误差
# 参数:
#   cx: 目标中心X坐标
#   cy: 目标中心Y坐标
# 返回值: (err_y, err_z) Y轴和Z轴方向的误差
def control_error(cx, cy):
    return int(cx) - ORIGIN_X, ORIGIN_Y - int(cy)


# ============================================================
# 几何工具函数
# ============================================================
# 数值钳位函数：将x限制在[lo, hi]范围内
# 参数:
#   x: 输入值
#   lo: 下限
#   hi: 上限
# 返回值: 钳位后的值
def clamp(x, lo, hi):
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x

# ROI裁剪函数：将矩形区域限制在图像范围内
# 参数:
#   x, y: 矩形左上角坐标
#   w, h: 矩形宽高
#   img_w, img_h: 图像宽高
# 返回值: 裁剪后的(x, y, w, h)元组
def roi_clip(x, y, w, h, img_w=IMG_W, img_h=IMG_H):
    x = int(clamp(x, 0, img_w - 1))
    y = int(clamp(y, 0, img_h - 1))
    w = int(clamp(w, 1, img_w - x))
    h = int(clamp(h, 1, img_h - y))
    return (x, y, w, h)

# 获取检测中心ROI：返回全局裁剪区域
def detection_center_roi():
    return CROP_ROI

# 角点转外接框：根据四个角点计算外接矩形，可加边距
# 参数:
#   corners: 四个角点列表
#   margin: 扩展边距(像素)
# 返回值: 裁剪后的(x, y, w, h)元组
def corners_to_box(corners, margin=0):
    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    x0 = min(xs) - margin
    y0 = min(ys) - margin
    x1 = max(xs) + margin
    y1 = max(ys) + margin
    return roi_clip(x0, y0, x1 - x0 + 1, y1 - y0 + 1)

# 角点转非对称外接框：X和Y方向使用不同边距
# 参数:
#   corners: 四个角点列表
#   margin_x: X方向扩展边距
#   margin_y: Y方向扩展边距
# 返回值: 裁剪后的(x, y, w, h)元组
def corners_to_asymmetric_box(corners, margin_x=0, margin_y=0):
    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    x0 = min(xs) - margin_x
    y0 = min(ys) - margin_y
    x1 = max(xs) + margin_x
    y1 = max(ys) + margin_y
    return roi_clip(x0, y0, x1 - x0 + 1, y1 - y0 + 1)

# 角点排序：将四个角点按顺时针排列 [左上, 右上, 右下, 左下]
# 参数:
#   corners: 四个角点列表(任意顺序)
# 返回值: 排序后的四个角点列表
def order_corners(corners):
    pts = [(int(p[0]), int(p[1])) for p in corners]
    # 用x+y和y-x的极值来确定四个角
    s = [p[0] + p[1] for p in pts]
    d = [p[1] - p[0] for p in pts]
    tl = pts[s.index(min(s))]  # 左上: x+y最小
    br = pts[s.index(max(s))]  # 右下: x+y最大
    tr = pts[d.index(min(d))]  # 右上: y-x最小
    bl = pts[d.index(max(d))]  # 左下: y-x最大
    return [tl, tr, br, bl]

# 多边形面积：计算四边形面积(鞋带公式)
# 参数:
#   corners: 四个角点列表
# 返回值: 四边形面积(整数)
def polygon_area(corners):
    pts = order_corners(corners)
    s = 0
    for i in range(4):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) & 3]
        s += x0 * y1 - x1 * y0
    if s < 0:
        s = -s
    return s // 2

# 验证矩形角度：检查四边形四个角是否都大于最小角度
# 参数:
#   corners: 四个角点列表
#   min_angle: 最小角度阈值(度)
# 返回值: True表示角度合格，False表示不合格
def validate_rect_angles(corners, min_angle=RECT_MIN_ANGLE):
    pts = order_corners(corners)
    # 遍历四个顶点，检查每个角的角度
    for i in range(4):
        p0 = pts[(i - 1) & 3]  # 前一个点
        p1 = pts[i]            # 当前顶点
        p2 = pts[(i + 1) & 3]  # 后一个点
        # 构造两条边的向量
        v1 = (p0[0] - p1[0], p0[1] - p1[1])
        v2 = (p2[0] - p1[0], p2[1] - p1[1])
        # 计算向量模长
        m1 = math.sqrt(v1[0] * v1[0] + v1[1] * v1[1])
        m2 = math.sqrt(v2[0] * v2[0] + v2[1] * v2[1])
        if m1 < 1.0 or m2 < 1.0:
            return False
        # 用点积计算夹角余弦
        c = (v1[0] * v2[0] + v1[1] * v2[1]) / (m1 * m2)
        c = max(-1.0, min(1.0, c))  # 钳位到[-1, 1]
        # 角度小于阈值则不合格
        if math.degrees(math.acos(c)) < min_angle:
            return False
    return True

# 获取灰度值：获取图像指定位置的灰度值，兼容多种像素格式
# 参数:
#   img: 图像对象
#   x, y: 像素坐标
# 返回值: 灰度值(0~255)
def get_gray(img, x, y):
    try:
        p = img.get_pixel(int(x), int(y))
    except Exception:
        return 0
    # 处理元组格式(RGB等)
    if isinstance(p, tuple):
        if len(p) >= 3:
            # 灰度转换公式: 0.299R + 0.587G + 0.114B
            return (int(p[0]) * 76 + int(p[1]) * 150 + int(p[2]) * 29) >> 8
        if len(p) == 1:
            return int(p[0])
        return 0
    # 处理RGB565格式(16位整数)
    if p > 255:
        r = (p >> 11) & 0x1F
        g = (p >> 5) & 0x3F
        b = p & 0x1F
        return (r * 76 + g * 150 + b * 29) >> 8
    return int(p)

# 线段暗度采样：沿线段采样n个点，统计暗像素比例
# 参数:
#   gray: 灰度图像
#   p0: 线段起点
#   p1: 线段终点
#   n: 采样点数
# 返回值: 暗像素比例(0.0~1.0)
def sample_line_dark(gray, p0, p1, n=9):
    dark = 0
    total = 0
    for i in range(n):
        t = i / (n - 1)
        # 线性插值计算采样点坐标
        x = int(p0[0] + (p1[0] - p0[0]) * t)
        y = int(p0[1] + (p1[1] - p0[1]) * t)
        if 0 <= x < IMG_W and 0 <= y < IMG_H:
            total += 1
            if get_gray(gray, x, y) <= BLACK_GRAY_TH:
                dark += 1
    return dark / total if total else 0.0

# 黑框评分：评估四边形是否为黑色边框靶标
# 评分规则: 黑边数量*10 + 边暗度和 + 内部白点*2
# 参数:
#   gray: 灰度图像
#   corners: 四个角点
# 返回值: 评分值，越高越可能是黑框
def black_frame_score(gray, corners):
    pts = order_corners(corners)
    dark_edges = 0          # 暗边计数
    edge_ratio_sum = 0.0    # 边暗度累加
    # 检测四条边的暗度
    for i in range(4):
        r = sample_line_dark(gray, pts[i], pts[(i + 1) & 3])
        edge_ratio_sum += r
        if r >= 0.50:  # 暗像素超过50%认为是黑边
            dark_edges += 1

    # 计算中心点和尺寸
    cx = (pts[0][0] + pts[2][0]) // 2
    cy = (pts[0][1] + pts[2][1]) // 2
    box_w = max(1, max(p[0] for p in pts) - min(p[0] for p in pts))
    box_h = max(1, max(p[1] for p in pts) - min(p[1] for p in pts))
    # 采样步长
    sx = max(6, box_w // 6)
    sy = max(6, box_h // 6)
    # 检测内部白色(中心及上下左右五个点)
    inner_white = 0
    for dx, dy in ((0, 0), (-sx, 0), (sx, 0), (0, -sy), (0, sy)):
        if get_gray(gray, clamp(cx + dx, 0, IMG_W - 1),
                    clamp(cy + dy, 0, IMG_H - 1)) >= WHITE_GRAY_TH:
            inner_white += 1
    return dark_edges * 10 + edge_ratio_sum + inner_white * 2.0

# 简单中心计算：四个角点坐标的平均值
# 参数:
#   corners: 四个角点列表
# 返回值: (cx, cy) 中心坐标
def simple_center(corners):
    pts = order_corners(corners)
    return ((pts[0][0] + pts[1][0] + pts[2][0] + pts[3][0]) // 4,
            (pts[0][1] + pts[1][1] + pts[2][1] + pts[3][1]) // 4)

# 平移角点：将所有角点平移指定偏移量
# 参数:
#   corners: 四个角点列表
#   dx, dy: 平移偏移量
# 返回值: 平移后的角点列表
def shift_corners(corners, dx, dy):
    shifted = []
    for p in order_corners(corners):
        x = int(clamp(p[0] + dx, 0, IMG_W - 1))
        y = int(clamp(p[1] + dy, 0, IMG_H - 1))
        shifted.append((x, y))
    return shifted

# 角点平滑：用一阶低通滤波平滑角点位置
# 参数:
#   prev: 上一帧角点
#   curr: 当前帧角点
# 返回值: 平滑后的角点
def smooth_corners(prev, curr):
    if prev is None:
        return order_corners(curr)
    out = []
    curr = order_corners(curr)
    prev = order_corners(prev)
    keep = CORNER_SMOOTH_DEN - CORNER_SMOOTH_NUM  # 旧值权重
    # 逐点加权平均
    for p, c in zip(prev, curr):
        x = (p[0] * keep + c[0] * CORNER_SMOOTH_NUM) // CORNER_SMOOTH_DEN
        y = (p[1] * keep + c[1] * CORNER_SMOOTH_NUM) // CORNER_SMOOTH_DEN
        out.append((int(x), int(y)))
    return out

# 中心距离：计算两个四边形中心的欧氏距离
# 参数:
#   a: 第一个四边形角点
#   b: 第二个四边形角点
# 返回值: 中心距离(像素)
def center_distance(a, b):
    ax, ay = simple_center(a)
    bx, by = simple_center(b)
    dx = ax - bx
    dy = ay - by
    return math.sqrt(dx * dx + dy * dy)

# 面积变化率：计算两个四边形面积的相对变化
# 参数:
#   a: 第一个四边形角点
#   b: 第二个四边形角点
# 返回值: 面积变化比例(0.0~1.0)
def area_change_ratio(a, b):
    aa = polygon_area(a)
    bb = polygon_area(b)
    if aa <= 0 or bb <= 0:
        return 1.0
    d = aa - bb
    if d < 0:
        d = -d
    return d / max(aa, bb)


# ============================================================
# 矩形检测器类
# ============================================================
# 快速矩形跟踪器：实现基于ROI的目标跟踪、多策略检测、运动预测和抗干扰
# 检测策略优先级: cv_lite > blob检测 > find_rects回退
# 跟踪策略: 基于运动预测的ROI裁剪 + 全帧周期性重扫
class FastRectangleTracker:
    # 初始化跟踪器状态
    def __init__(self):
        self.last_corners = None    # 上一帧确认的角点
        self.pending_corners = None # 待确认的候选角点
        self.pending_hits = 0       # 候选角点连续命中计数
        self.lost_frames = 999      # 目标丢失帧数
        self.frame_id = 0           # 帧计数器
        self.use_cv_lite = HAS_CV_LITE  # 是否使用cv_lite加速
        self.output_x = -1          # 平滑输出X坐标
        self.output_y = -1          # 平滑输出Y坐标
        self.output_hold = 0        # 输出保持帧数
        self.vel_x = 0.0            # X方向速度(像素/帧)
        self.vel_y = 0.0            # Y方向速度(像素/帧)
        self.last_center = None     # 上一帧中心坐标

    # 计算跟踪速度标量
    def _track_speed(self):
        return math.sqrt(self.vel_x * self.vel_x + self.vel_y * self.vel_y)

    # 预测未来帧的角点位置(基于当前速度)
    # 参数:
    #   frames_ahead: 预测未来第几帧
    # 返回值: 预测的角点列表
    def _predicted_corners(self, frames_ahead=1):
        if self.last_corners is None:
            return None
        if frames_ahead < 1:
            frames_ahead = 1
        return shift_corners(self.last_corners,
                             self.vel_x * frames_ahead,
                             self.vel_y * frames_ahead)

    # 计算当前跟踪ROI区域
    # 根据目标速度和丢失帧数动态调整边距
    # 返回值: (x, y, w, h) ROI矩形
    def _tracking_roi(self):
        predicted = self._predicted_corners(self.lost_frames + 1)
        if predicted is None:
            return (0, 0, IMG_W, IMG_H)
        # 速度越快，边距越大
        margin_x = TRACK_MARGIN_X + int(abs(self.vel_x) * TRACK_MARGIN_X_VEL_GAIN)
        margin_y = TRACK_MARGIN + int(abs(self.vel_y) * TRACK_MARGIN_VEL_GAIN)
        # 丢失帧数越多，边距越大(扩大搜索范围)
        margin_x += self.lost_frames * TRACK_LOST_MARGIN_STEP
        margin_y += self.lost_frames * TRACK_LOST_MARGIN_STEP
        # 限制边距在合理范围内
        margin_x = int(clamp(margin_x, TRACK_MARGIN_X, TRACK_MARGIN_X_MAX))
        margin_y = int(clamp(margin_y, TRACK_MARGIN, TRACK_MARGIN_MAX))
        return corners_to_asymmetric_box(predicted, margin_x, margin_y)

    # 更新运动估计(速度)
    # 参数:
    #   corners: 当前帧角点
    def _update_motion(self, corners):
        cx, cy = simple_center(corners)
        if self.last_center is not None:
            dx = cx - self.last_center[0]
            dy = cy - self.last_center[1]
            # 一阶低通滤波平滑速度
            keep = TRACK_VELOCITY_SMOOTH_DEN - TRACK_VELOCITY_SMOOTH_NUM
            self.vel_x = (self.vel_x * keep + dx * TRACK_VELOCITY_SMOOTH_NUM) / TRACK_VELOCITY_SMOOTH_DEN
            self.vel_y = (self.vel_y * keep + dy * TRACK_VELOCITY_SMOOTH_NUM) / TRACK_VELOCITY_SMOOTH_DEN
        self.last_center = (cx, cy)

    # 重置运动状态
    def _reset_motion(self):
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.last_center = None

    # 使用cv_lite进行矩形检测(加速路径)
    # 参数:
    #   gray: 灰度图像
    #   roi: 检测区域
    # 返回值: [(corners, area), ...] 候选列表
    def _cv_lite_rects(self, gray, roi):
        if not self.use_cv_lite:
            return []
        x0, y0, rw, rh = roi
        try:
            # 裁剪ROI图像(全图则直接使用)
            work = gray if (x0 == 0 and y0 == 0 and rw == IMG_W and rh == IMG_H) \
                else gray.copy(roi=roi)
            np_img = work.to_numpy_ref()
            # 调用cv_lite的灰度矩形检测
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
        # 过滤和转换检测结果
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
            # 转换角点坐标到全图坐标系
            corners = [
                (int(r[4]) + x0, int(r[5]) + y0),
                (int(r[6]) + x0, int(r[7]) + y0),
                (int(r[8]) + x0, int(r[9]) + y0),
                (int(r[10]) + x0, int(r[11]) + y0),
            ]
            # 角度验证
            if not validate_rect_angles(corners):
                continue
            area = polygon_area(corners)
            if area < OUTER_MIN_AREA or area > OUTER_MAX_AREA:
                continue
            out.append((order_corners(corners), area))
        return out

    # 使用find_rects进行矩形检测(回退路径1)
    # 参数:
    #   gray: 灰度图像
    #   roi: 检测区域
    # 返回值: [(corners, area), ...] 候选列表
    def _find_rects(self, gray, roi):
        x0, y0, rw, rh = roi
        try:
            # 全图检测
            if x0 == 0 and y0 == 0 and rw == IMG_W and rh == IMG_H:
                rects = gray.find_rects(threshold=FIND_RECTS_THRESHOLD)
                offset_x = 0
                offset_y = 0
            else:
                # ROI检测(兼容带roi参数和不带的API)
                try:
                    rects = gray.find_rects(threshold=FIND_RECTS_THRESHOLD, roi=roi)
                    offset_x = 0
                    offset_y = 0
                except TypeError:
                    work = gray.copy(roi=roi)
                    rects = work.find_rects(threshold=FIND_RECTS_THRESHOLD)
                    offset_x = x0
                    offset_y = y0
        except Exception:
            return []
        out = []
        if not rects:
            return out
        # 过滤和转换检测结果
        for r in rects:
            try:
                corners_roi = r.corners()
                bx, by, bw, bh = r.rect()
            except Exception:
                continue
            box_area = int(bw) * int(bh)
            # 面积过滤
            if box_area < OUTER_MIN_AREA or box_area > OUTER_MAX_AREA:
                continue
            # 宽高比过滤
            aspect = bw / max(1, bh)
            if aspect < ASPECT_MIN or aspect > ASPECT_MAX:
                continue
            # 转换坐标
            corners = [(int(p[0]) + offset_x, int(p[1]) + offset_y)
                       for p in corners_roi]
            # 角度验证
            if not validate_rect_angles(corners):
                continue
            out.append((order_corners(corners), polygon_area(corners)))
        return out

    # 快速Blob检测(兼容不同API签名)
    def _find_blobs_fast(self, img, thresholds, **kwargs):
        try:
            return img.find_blobs(thresholds, x_stride=BLOB_X_STRIDE,
                                  y_stride=BLOB_Y_STRIDE, **kwargs)
        except TypeError:
            return img.find_blobs(thresholds, **kwargs)

    # 使用Blob检测进行矩形检测(回退路径2)
    # 参数:
    #   gray: 灰度图像
    #   roi: 检测区域
    # 返回值: [(corners, area), ...] 候选列表
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
                # ROI检测(兼容不同API)
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
        # 过滤和转换Blob结果
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
            # 密度过滤(排除太实心或太空洞的)
            try:
                density = b.density()
                if density < 0.08 or density > 0.55:
                    continue
            except Exception:
                pass
            # 用Blob外接矩形作为角点
            corners = order_corners([
                (bx + offset_x, by + offset_y),
                (bx + bw + offset_x, by + offset_y),
                (bx + bw + offset_x, by + bh + offset_y),
                (bx + offset_x, by + bh + offset_y),
            ])
            out.append((corners, area))
        return out

    # ROI扫描：按优先级尝试多种检测方法
    # 跟踪状态下优先用Blob快速检测，否则按cv_lite->blob->find_rects顺序
    # 参数:
    #   gray: 灰度图像
    #   roi: 检测区域
    # 返回值: 候选列表
    def _scan_roi(self, gray, roi):
        # 判断是否处于跟踪ROI模式(不是全图)
        tracking_roi = (
            self.last_corners is not None and
            self.lost_frames < LOCK_HOLD_FRAMES and
            roi != (0, 0, IMG_W, IMG_H)
        )
        # 跟踪模式是50FPS热路径：只跑小ROI Blob，重算法低频兜底
        if tracking_roi:
            cands = self._blob_rects(gray, roi)
            if cands:
                return cands
            if (CV_LITE_FALLBACK_PERIOD <= 1 or
                    (self.frame_id % CV_LITE_FALLBACK_PERIOD) == 0):
                return self._cv_lite_rects(gray, roi)
            return []

        # 捕获/全帧重扫也优先Blob；cv_lite和find_rects都降频回退
        cands = self._blob_rects(gray, roi)
        if (not cands and
                (CV_LITE_FALLBACK_PERIOD <= 1 or
                 (self.frame_id % CV_LITE_FALLBACK_PERIOD) == 0)):
            cands = self._cv_lite_rects(gray, roi)
        if (not cands and
                (FIND_RECTS_FALLBACK_PERIOD <= 1 or
                 (self.frame_id % FIND_RECTS_FALLBACK_PERIOD) == 0)):
            cands = self._find_rects(gray, roi)
        return cands

    # 选择最佳候选：综合评分选出最优矩形
    # 评分因素: 黑框评分 + 面积 + 与预测位置的接近度 + 面积变化
    # 参数:
    #   gray: 灰度图像
    #   cands: 候选列表
    # 返回值: (corners, area) 最佳候选，None表示无合格候选
    def _choose_best(self, gray, cands):
        if not cands:
            return None
        best = None
        best_score = -1
        for corners, area in cands:
            # 黑框质量评分
            frame_score = black_frame_score(gray, corners)
            # 根据捕获/跟踪状态使用不同的最低分阈值
            min_score = FRAME_SCORE_MIN_TRACK if self.last_corners is not None \
                else FRAME_SCORE_MIN_ACQUIRE
            if frame_score < min_score:
                continue
            cx, cy = simple_center(corners)
            # 基础评分: 面积加权 + 黑框分
            score = area * 0.001 + frame_score
            # 跟踪状态下加入位置和面积变化的评分
            if self.last_corners is not None:
                # 与预测位置的距离加分
                ref = self._predicted_corners(1)
                if ref is None:
                    ref = self.last_corners
                lx, ly = simple_center(ref)
                dist = abs(cx - lx) + abs(cy - ly)
                ar = area_change_ratio(self.last_corners, corners)
                score += max(0.0, 18.0 - dist * 0.18)  # 距离越近分越高
                score -= ar * 18.0  # 面积变化越大扣分越多
            else:
                # 捕获状态下，偏好靠近画面中心的目标
                center_penalty = (abs(cx - ORIGIN_X) +
                                  abs(cy - ORIGIN_Y)) * 0.01
                score -= center_penalty
            # 更新最佳
            if score > best_score:
                best_score = score
                best = (corners, area)
        return best

    # 接受候选判定：判断候选是否可以作为新的跟踪目标
    # 包含跳跃检测、面积变化检测、预测位置验证、切换确认机制
    # 参数:
    #   corners: 候选角点
    # 返回值: (accepted_corners, is_fresh) 接受的角点和是否为新检测
    def _accept_candidate(self, corners):
        corners = order_corners(corners)
        # 已有跟踪目标时的验证逻辑
        if self.last_corners is not None:
            # 计算跳跃距离和面积变化
            jump = center_distance(self.last_corners, corners)
            ar = area_change_ratio(self.last_corners, corners)
            # 跳跃过大或面积变化过大
            if jump > JUMP_REJECT_PIXELS or ar > AREA_CHANGE_REJECT:
                # 尝试用预测位置验证(快速运动目标可能看起来跳变大)
                predicted = self._predicted_corners(self.lost_frames + 1)
                if predicted is not None:
                    predict_jump = center_distance(predicted, corners)
                    predict_limit = TRACK_PREDICT_ACCEPT_PIXELS + self._track_speed() * TRACK_PREDICT_ACCEPT_GAIN
                    # 预测位置附近且面积变化合格则接受
                    if predict_jump <= predict_limit and ar <= AREA_CHANGE_REJECT:
                        self.pending_corners = None
                        self.pending_hits = 0
                        return smooth_corners(self.last_corners, corners), True
                # 切换确认机制: 连续N帧检测到同一新目标才切换
                if self.pending_corners is not None and \
                        center_distance(self.pending_corners, corners) < 36:
                    self.pending_hits += 1
                else:
                    self.pending_corners = corners
                    self.pending_hits = 1
                if self.pending_hits < SWITCH_CONFIRM_FRAMES:
                    return self.last_corners, False
            # 正常接受，平滑角点
            self.pending_corners = None
            self.pending_hits = 0
            return smooth_corners(self.last_corners, corners), True
        # 无跟踪目标时的捕获确认
        if self.pending_corners is not None and \
                center_distance(self.pending_corners, corners) < 36:
            self.pending_hits += 1
        else:
            self.pending_corners = corners
            self.pending_hits = 1
        if self.pending_hits < SWITCH_CONFIRM_FRAMES:
            # 首次捕获必须连续确认满SWITCH_CONFIRM_FRAMES帧才可采信，
            # 单帧候选(阴影/反光等)不得直接建立track
            return None, False
        self.pending_corners = None
        self.pending_hits = 0
        return corners, True

    # 主检测函数：每帧调用一次，执行检测和跟踪
    # 参数:
    #   gray: 灰度图像
    # 返回值: (target_x, target_y, corners, is_fresh)
    #   target_x, target_y: 透视映射后的目标中心坐标
    #   corners: 检测到的角点
    #   is_fresh: 是否为新检测到的帧
    def detect(self, gray):
        self.frame_id += 1
        full_roi = (0, 0, IMG_W, IMG_H)
        rois = []

        # 确定检测ROI列表
        if self.last_corners is not None and self.lost_frames < LOCK_HOLD_FRAMES:
            rois.append(self._tracking_roi())  # 跟踪ROI
        need_full = self.last_corners is None or self.lost_frames >= LOCK_HOLD_FRAMES

        # 周期性全帧重扫(防止跟丢)
        if FULL_RESCAN_PERIOD and self.last_corners is not None:
            rescan_period = FULL_RESCAN_PERIOD
            # 快速运动时更频繁地全扫
            if self._track_speed() >= TRACK_FAST_SPEED_PIXELS:
                rescan_period = TRACK_FAST_FULL_RESCAN_PERIOD
            need_full = need_full or (self.frame_id % rescan_period) == 0
        if need_full:
            rois.append(full_roi)

        # 逐个ROI检测，找到第一个合格目标
        for roi in rois:
            best = self._choose_best(gray, self._scan_roi(gray, roi))
            if best is not None:
                corners, _ = best
                accepted, fresh = self._accept_candidate(corners)
                if accepted is None:
                    break
                # 更新运动估计和跟踪状态
                if fresh:
                    self._update_motion(corners)
                self.last_corners = accepted
                self.lost_frames = 0
                # 透视映射得到目标中心
                cx, cy = perspective_map_point(self.last_corners,
                                                TARGET_PLANE_U, TARGET_PLANE_V)
                cx, cy = int(cx + 0.5), int(cy + 0.5)
                return cx, cy, self.last_corners, fresh

        # 未检测到目标，但在锁定保持期内，继续输出预测位置
        if self.last_corners is not None and self.lost_frames < LOCK_HOLD_FRAMES:
            self.lost_frames += 1
            cx, cy = perspective_map_point(self.last_corners,
                                            TARGET_PLANE_U, TARGET_PLANE_V)
            cx, cy = int(cx + 0.5), int(cy + 0.5)
            return cx, cy, self.last_corners, False

        # 完全丢失目标，重置状态
        # 仅在放弃一个已建立的track时才清空候选确认进度；
        # 首次捕获尚未确认时不能清空，否则SWITCH_CONFIRM_FRAMES永远无法达成
        had_track = self.last_corners is not None
        self.last_corners = None
        if had_track:
            self.pending_corners = None
            self.pending_hits = 0
        self.lost_frames = 999
        self._reset_motion()
        return -1, -1, None, False

    # 稳定输出：对输出坐标进行平滑和保持
    # 参数:
    #   x, y: 当前检测坐标
    #   valid: 是否有效
    # 返回值: (sx, sy) 平滑后的坐标，无效返回(-1, -1)
    def _stable_output(self, x, y, valid):
        # 有效输入时进行平滑
        if valid and x >= 0 and y >= 0:
            if self.output_x < 0 or self.output_y < 0:
                # 首次有效直接赋值
                self.output_x = int(x)
                self.output_y = int(y)
            else:
                # 一阶低通滤波平滑
                keep = OUTPUT_SMOOTH_DEN - OUTPUT_SMOOTH_NUM
                self.output_x = (self.output_x * keep + int(x) * OUTPUT_SMOOTH_NUM) // OUTPUT_SMOOTH_DEN
                self.output_y = (self.output_y * keep + int(y) * OUTPUT_SMOOTH_NUM) // OUTPUT_SMOOTH_DEN
            self.output_hold = OUTPUT_HOLD_FRAMES
            return int(self.output_x), int(self.output_y)
        # 无效输入时，保持输出若干帧
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
# 绘制坐标轴箭头：从(x0,y0)到(x1,y1)画带箭头的线段
# 参数:
#   img: 图像对象
#   x0, y0: 起点坐标
#   x1, y1: 终点坐标
#   color: 线条颜色
def draw_axis_arrow(img, x0, y0, x1, y1, color):
    # 绘制主轴线段
    img.draw_line(x0, y0, x1, y1, color=color, thickness=2)
    # 根据方向绘制箭头
    if x1 > x0:
        img.draw_line(x1, y1, x1 - 8, y1 - 4, color=color, thickness=2)
        img.draw_line(x1, y1, x1 - 8, y1 + 4, color=color, thickness=2)
    elif x1 < x0:
        img.draw_line(x1, y1, x1 + 8, y1 - 4, color=color, thickness=2)
        img.draw_line(x1, y1, x1 + 8, y1 + 4, color=color, thickness=2)
    elif y1 < y0:
        img.draw_line(x1, y1, x1 - 4, y1 + 8, color=color, thickness=2)
        img.draw_line(x1, y1, x1 + 4, y1 + 8, color=color, thickness=2)
    else:
        img.draw_line(x1, y1, x1 - 4, y1 - 8, color=color, thickness=2)
        img.draw_line(x1, y1, x1 + 4, y1 - 8, color=color, thickness=2)


# 绘制叠加层：坐标轴、目标框、误差信息、FPS等
# 参数:
#   img: 图像对象
#   corners: 目标角点
#   target_x, target_y: 目标中心坐标
#   fps: 帧率
#   fresh: 是否为新检测帧
#   err_y, err_z: Y/Z轴误差
def draw_overlay(img, corners, target_x, target_y, fps, fresh, err_y, err_z):
    # 颜色定义
    GREEN  = (0, 255, 0)
    RED    = (255, 0, 0)
    YELLOW = (255, 255, 0)
    WHITE  = (255, 255, 255)
    ORANGE = (255, 180, 0)
    CYAN   = (0, 255, 255)
    GRAY   = (128, 128, 128)
    AXIS_LEN = 42  # 坐标轴长度

    origin_x = ORIGIN_X
    origin_y = ORIGIN_Y
    frame_color = GREEN if fresh else ORANGE  # 新检测绿色，预测橙色

    # 绘制坐标轴参考线(灰色)
    img.draw_line(origin_x - AXIS_LEN, origin_y, origin_x + AXIS_LEN, origin_y,
                  color=GRAY, thickness=1)
    img.draw_line(origin_x, origin_y - AXIS_LEN, origin_x, origin_y + AXIS_LEN,
                  color=GRAY, thickness=1)
    # 绘制带箭头的坐标轴
    draw_axis_arrow(img, origin_x, origin_y, origin_x + AXIS_LEN, origin_y, CYAN)
    draw_axis_arrow(img, origin_x, origin_y, origin_x, origin_y - AXIS_LEN, YELLOW)

    # 绘制原点标记
    img.draw_circle(origin_x, origin_y, 5, color=GREEN, thickness=2)
    img.draw_cross(origin_x, origin_y, color=GREEN, size=7, thickness=1)

    # 绘制目标框和目标点
    if corners is not None and target_x >= 0:
        # 绘制四边形边框
        for i in range(4):
            x0, y0 = corners[i]
            x1, y1 = corners[(i + 1) & 3]
            img.draw_line(int(x0), int(y0), int(x1), int(y1),
                          color=frame_color, thickness=2)
        # 绘制目标中心
        img.draw_circle(int(target_x), int(target_y), 4,
                        color=RED, thickness=2)
        # 绘制误差分解线
        img.draw_line(origin_x, origin_y, int(target_x), origin_y,
                      color=CYAN, thickness=1)
        img.draw_line(int(target_x), origin_y, int(target_x), int(target_y),
                      color=YELLOW, thickness=1)
        # 绘制对角线
        img.draw_line(origin_x, origin_y, int(target_x), int(target_y),
                      color=WHITE, thickness=1)

    # 绘制文字信息
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
# 发送跟踪数据：通过UART发送误差数据
# 参数:
#   uart: UART对象
#   err_y: Y轴误差
#   err_z: Z轴误差
#   fps: 帧率(可选)
def send_tracking(uart, err_y, err_z, fps=0):
    uart.write(pack_error_packet(int(err_y), int(err_z)))


# 初始化UART：配置UART3引脚和波特率
# 返回值: UART对象
def init_uart():
    fpioa = FPIOA()
    fpioa.set_function(50, FPIOA.UART3_TXD)
    fpioa.set_function(51, FPIOA.UART3_RXD)
    return UART(UART.UART3, baudrate=UART_BAUD,
                bits=UART.EIGHTBITS,
                parity=UART.PARITY_NONE,
                stop=UART.STOPBITS_ONE)


# 初始化摄像头：配置分辨率、像素格式等
# 返回值: (sensor, pixfmt) 传感器对象和像素格式字符串
def init_camera():
    try:
        sensor = Sensor(width=CAM_W, height=CAM_H, fps=CAM_FPS)
    except TypeError:
        sensor = Sensor(width=CAM_W, height=CAM_H)
    sensor.reset()
    sensor.set_framesize(width=CAM_W, height=CAM_H, chn=FULL_CHN)
    # 根据显示模式配置像素格式
    if not HEADLESS:
        sensor.set_pixformat(Sensor.RGB565, chn=FULL_CHN)
        pixfmt = "RGB565"
    elif SENSOR_PIXFORMAT == "RGB565":
        sensor.set_pixformat(Sensor.RGB565, chn=FULL_CHN)
        pixfmt = "RGB565"
    else:
        # 尝试灰度格式，失败则回退到RGB565
        try:
            sensor.set_pixformat(Sensor.GRAYSCALE, chn=FULL_CHN)
            pixfmt = "GRAYSCALE"
        except Exception:
            sensor.set_pixformat(Sensor.RGB565, chn=FULL_CHN)
            pixfmt = "RGB565"

    try:
        sensor.set_framesize(width=IMG_W, height=IMG_H,
                             chn=DETECT_CHN, crop=CROP_ROI)
        sensor.set_pixformat(Sensor.GRAYSCALE, chn=DETECT_CHN)
        return sensor, "GRAYSCALE", DETECT_CHN, True
    except Exception:
        return sensor, pixfmt, FULL_CHN, False


# ============================================================
# 主入口
# ============================================================
# 全局硬件对象
sensor = None
uart = None
has_display = False

try:
    # ===== 硬件初始化 =====
    uart = init_uart()
    sensor, pixfmt, detect_chn, sensor_crop_enabled = init_camera()

    # 初始化LCD显示
    has_display = False
    if not HEADLESS:
        try:
            Display.init(Display.ST7701, width=800, height=480,
                         to_ide=SHOW_TO_IDE)
            has_display = True
        except Exception:
            pass

    # 启动媒体系统和传感器
    MediaManager.init()
    sensor.run()

    # ===== 创建跟踪器和辅助对象 =====
    tracker = FastRectangleTracker()
    clock = time.clock()
    gc_count = 0
    display_count = 0

    target_x, target_y = -1, -1
    corners = None

    # ===== 主循环 =====
    # 单帧异常不得让整个脚本退出：脚本一旦退出，UART不再有任何数据，
    # eye端会永久停留在“最后一次収到的状态”——这是“视觉状态机卡死”最常见的真实成因。
    # 因此单帧内的任何异常都在本帧内吞掉、发一次无效心跳，主循环本身永不退出。
    while True:
        clock.tick()
        os.exitpoint()
        try:
            img = sensor.snapshot(chn=detect_chn)

            # 优先使用K230传感器输出通道硬件裁剪；旧固件失败时保留软件裁剪兜底
            if sensor_crop_enabled:
                view = img
            else:
                view = img.copy(roi=detection_center_roi())

            # 转换为灰度图
            if pixfmt == "GRAYSCALE":
                gray = view
            else:
                gray = view.to_grayscale(copy=True)

            # 执行目标检测
            target_x, target_y, corners, fresh = tracker.detect(gray)

            # 平滑输出并计算误差
            # 注意: 用坐标是否有效(而非fresh)作为_stable_output的输入有效性判据。
            # detect()在LOCK_HOLD_FRAMES预测保持期内fresh=False但仍返回真实预测坐标，
            # 若继续用fresh判定会让输出比跟踪器本身提前判丢，造成靶子明明还在预测保持
            # 期内、输出却已经开始闪烁跳变到无效值的问题。
            target_has_position = target_x >= 0 and target_y >= 0
            stable_x, stable_y = tracker._stable_output(target_x, target_y, target_has_position)
            target_valid = stable_x >= 0 and stable_y >= 0
            if target_valid:
                cx = stable_x
                cy = stable_y
                err_y, err_z = control_error(cx, cy)
                send_y = err_y
                send_z = err_z
            else:
                # 目标丢失，发送无效标记
                send_y = ERR_INVALID
                send_z = ERR_INVALID

            # 计算帧率并发送数据
            fps = int(clock.fps() + 0.5)
            send_tracking(uart, send_y, send_z, fps if target_valid else 0)

            # LCD显示(降频刷新)
            if has_display:
                display_count += 1
                if display_count >= DISPLAY_EVERY_N:
                    display_count = 0
                    draw_overlay(view, corners, target_x, target_y, fps, fresh, send_y, send_z)
                    Display.show_image(view, x=(800 - IMG_W) // 2, y=(480 - IMG_H) // 2)

            # 定期垃圾回收
            gc_count += 1
            if gc_count >= 60:
                gc_count = 0
                gc.collect()
        except Exception as frame_err:
            # 单帧失败：发送一次无效标记心跳，让下游依赖“数据是否新鲜”的
            # 判断能正确感知丢失，而不是停在最后一帧误判为“仍然有效”
            print("frame err:", frame_err)
            try:
                send_tracking(uart, ERR_INVALID, ERR_INVALID, 0)
            except Exception:
                pass
            gc.collect()

except KeyboardInterrupt as e:
    pass
except BaseException as e:
    print("err:", e)
finally:
    # ===== 资源清理 =====
    if isinstance(sensor, Sensor):
        sensor.stop()
    try:
        Display.deinit()
    except Exception:
        pass
    try:
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    except Exception:
        pass
    time.sleep_ms(100)
    try:
        MediaManager.deinit()
    except Exception:
        pass
    if uart:
        uart.deinit()
