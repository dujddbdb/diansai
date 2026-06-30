# -*- coding: utf-8 -*-
"""
K230 靶心识别 + LCD显示 + UART3发送 (放大画面版)
==============================================
修改点:
1. 检测分辨率从 320x240 放大到 640x480，LCD显示画面大一倍
2. 裁剪中心对准原ORIGIN_Y=55对应的激光物理位置，激光点落在裁剪画面正中心
3. 同步放大面积阈值、ROI追踪边距等像素参数，保证检测逻辑正常
"""

import gc
import math
import os
import time
from machine import FPIOA, UART
from media.display import *
from media.media import *
from media.sensor import *

# 透视映射函数
def perspective_map_point(corners, u, v):
    p0, p1, p2, p3 = corners
    x0, y0 = float(p0[0]), float(p0[1])
    x1, y1 = float(p1[0]), float(p1[1])
    x2, y2 = float(p2[0]), float(p2[1])
    x3, y3 = float(p3[0]), float(p3[1])
    dx1, dy1 = x1 - x2, y1 - y2
    dx2, dy2 = x3 - x2, y3 - y2
    sx = x0 - x1 + x2 - x3
    sy = y0 - y1 + y2 - y3
    determinant = dx1 * dy2 - dx2 * dy1

    if abs(determinant) < 1.0e-9:
        g = h = 0.0
    else:
        g = (sx * dy2 - dx2 * sy) / determinant
        h = (dx1 * sy - sx * dy1) / determinant

    a = x1 - x0 + g * x1
    b = x3 - x0 + h * x3
    d = y1 - y0 + g * y1
    e = y3 - y0 + h * y3
    denominator = g * u + h * v + 1.0
    if abs(denominator) < 1.0e-9:
        return ((x0 + x1 + x2 + x3) * 0.25,
                (y0 + y1 + y2 + y3) * 0.25)
    return ((a * u + b * v + x0) / denominator,
            (d * u + e * v + y0) / denominator)

def pack_error_packet(err_y, err_z, fps=0):
    err_y &= 0xFFFF
    err_z &= 0xFFFF
    payload = bytes((err_y >> 8, err_y & 0xFF,
                     err_z >> 8, err_z & 0xFF,
                     int(fps) & 0xFF))
    checksum = (0x04 + len(payload) + sum(payload)) & 0xFF
    return bytes((0xAA, 0x04, len(payload))) + payload + bytes((checksum,))

try:
    import cv_lite
    HAS_CV_LITE = True
except ImportError:
    HAS_CV_LITE = False


# ============================================================
# 用户可调参数区 (已放大适配640x480)
# ============================================================
# 图像与通信
IMG_W = 300
IMG_H = 300
CAM_W = 1920
CAM_H = 1080
CAM_FPS = 60
UART_BAUD = 115200

# 激光点在1080P大图中的物理坐标 (由原320x240中心裁剪时ORIGIN_Y=55换算得到)
LASER_OFFSET_X = 0
LASER_OFFSET_Y = -77

# 显示模式
HEADLESS = False
SHOW_TO_IDE = True
SENSOR_PIXFORMAT = "GRAYSCALE"
DISPLAY_EVERY_N = 2

# cv_lite 矩形检测器参数
CV_LOW = 20
CV_HIGH = 55
CV_EPSILON = 0.030
CV_MIN_AREA_RATIO = 0.002
CV_MAX_COS = 0.50
CV_MIN_EDGE = 6

# find_rects 回退方案
FIND_RECTS_THRESHOLD = 4500

# A4黑框靶标尺寸约束 (640x480分辨率，面积×4)
OUTER_MIN_AREA = 900
OUTER_MAX_AREA = 70000
RECT_MIN_ANGLE = 48
ASPECT_MIN = 0.55
ASPECT_MAX = 1.75
BLACK_GRAY_TH = 95
WHITE_GRAY_TH = 135

# ROI追踪 (分辨率翻倍，边距×2)
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
ORIGIN_X = IMG_W // 2
ORIGIN_Y = 160
TARGET_PLANE_U = 0.500
TARGET_PLANE_V = 0.500


# ============================================================
# 几何工具函数
# ============================================================
def clamp(x, lo, hi):
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x

def roi_clip(x, y, w, h, img_w=IMG_W, img_h=IMG_H):
    x = int(clamp(x, 0, img_w - 1))
    y = int(clamp(y, 0, img_h - 1))
    w = int(clamp(w, 1, img_w - x))
    h = int(clamp(h, 1, img_h - y))
    return (x, y, w, h)

def laser_center_roi():
    laser_x = CAM_W // 2 + LASER_OFFSET_X
    laser_y = CAM_H // 2 + LASER_OFFSET_Y
    return roi_clip(laser_x - IMG_W // 2,
                    laser_y - IMG_H // 2,
                    IMG_W,
                    IMG_H,
                    CAM_W,
                    CAM_H)

def corners_to_box(corners, margin=0):
    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    x0 = min(xs) - margin
    y0 = min(ys) - margin
    x1 = max(xs) + margin
    y1 = max(ys) + margin
    return roi_clip(x0, y0, x1 - x0 + 1, y1 - y0 + 1)

def corners_to_asymmetric_box(corners, margin_x=0, margin_y=0):
    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    x0 = min(xs) - margin_x
    y0 = min(ys) - margin_y
    x1 = max(xs) + margin_x
    y1 = max(ys) + margin_y
    return roi_clip(x0, y0, x1 - x0 + 1, y1 - y0 + 1)

def order_corners(corners):
    pts = [(int(p[0]), int(p[1])) for p in corners]
    s = [p[0] + p[1] for p in pts]
    d = [p[1] - p[0] for p in pts]
    tl = pts[s.index(min(s))]
    br = pts[s.index(max(s))]
    tr = pts[d.index(min(d))]
    bl = pts[d.index(max(d))]
    return [tl, tr, br, bl]

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

def validate_rect_angles(corners, min_angle=RECT_MIN_ANGLE):
    pts = order_corners(corners)
    for i in range(4):
        p0 = pts[(i - 1) & 3]
        p1 = pts[i]
        p2 = pts[(i + 1) & 3]
        v1 = (p0[0] - p1[0], p0[1] - p1[1])
        v2 = (p2[0] - p1[0], p2[1] - p1[1])
        m1 = math.sqrt(v1[0] * v1[0] + v1[1] * v1[1])
        m2 = math.sqrt(v2[0] * v2[0] + v2[1] * v2[1])
        if m1 < 1.0 or m2 < 1.0:
            return False
        c = (v1[0] * v2[0] + v1[1] * v2[1]) / (m1 * m2)
        c = max(-1.0, min(1.0, c))
        if math.degrees(math.acos(c)) < min_angle:
            return False
    return True

def get_gray(img, x, y):
    try:
        p = img.get_pixel(int(x), int(y))
    except Exception:
        return 0
    if isinstance(p, tuple):
        if len(p) >= 3:
            return (int(p[0]) * 76 + int(p[1]) * 150 + int(p[2]) * 29) >> 8
        if len(p) == 1:
            return int(p[0])
        return 0
    if p > 255:
        r = (p >> 11) & 0x1F
        g = (p >> 5) & 0x3F
        b = p & 0x1F
        return (r * 76 + g * 150 + b * 29) >> 8
    return int(p)

def sample_line_dark(gray, p0, p1, n=9):
    dark = 0
    total = 0
    for i in range(n):
        t = i / (n - 1)
        x = int(p0[0] + (p1[0] - p0[0]) * t)
        y = int(p0[1] + (p1[1] - p0[1]) * t)
        if 0 <= x < IMG_W and 0 <= y < IMG_H:
            total += 1
            if get_gray(gray, x, y) <= BLACK_GRAY_TH:
                dark += 1
    return dark / total if total else 0.0

def black_frame_score(gray, corners):
    pts = order_corners(corners)
    dark_edges = 0
    edge_ratio_sum = 0.0
    for i in range(4):
        r = sample_line_dark(gray, pts[i], pts[(i + 1) & 3])
        edge_ratio_sum += r
        if r >= 0.50:
            dark_edges += 1

    cx = (pts[0][0] + pts[2][0]) // 2
    cy = (pts[0][1] + pts[2][1]) // 2
    box_w = max(1, max(p[0] for p in pts) - min(p[0] for p in pts))
    box_h = max(1, max(p[1] for p in pts) - min(p[1] for p in pts))
    sx = max(6, box_w // 6)
    sy = max(6, box_h // 6)
    inner_white = 0
    for dx, dy in ((0, 0), (-sx, 0), (sx, 0), (0, -sy), (0, sy)):
        if get_gray(gray, clamp(cx + dx, 0, IMG_W - 1),
                    clamp(cy + dy, 0, IMG_H - 1)) >= WHITE_GRAY_TH:
            inner_white += 1
    return dark_edges * 10 + edge_ratio_sum + inner_white * 2.0

def simple_center(corners):
    pts = order_corners(corners)
    return ((pts[0][0] + pts[1][0] + pts[2][0] + pts[3][0]) // 4,
            (pts[0][1] + pts[1][1] + pts[2][1] + pts[3][1]) // 4)

def shift_corners(corners, dx, dy):
    shifted = []
    for p in order_corners(corners):
        x = int(clamp(p[0] + dx, 0, IMG_W - 1))
        y = int(clamp(p[1] + dy, 0, IMG_H - 1))
        shifted.append((x, y))
    return shifted

def smooth_corners(prev, curr):
    if prev is None:
        return order_corners(curr)
    out = []
    curr = order_corners(curr)
    prev = order_corners(prev)
    keep = CORNER_SMOOTH_DEN - CORNER_SMOOTH_NUM
    for p, c in zip(prev, curr):
        x = (p[0] * keep + c[0] * CORNER_SMOOTH_NUM) // CORNER_SMOOTH_DEN
        y = (p[1] * keep + c[1] * CORNER_SMOOTH_NUM) // CORNER_SMOOTH_DEN
        out.append((int(x), int(y)))
    return out

def center_distance(a, b):
    ax, ay = simple_center(a)
    bx, by = simple_center(b)
    dx = ax - bx
    dy = ay - by
    return math.sqrt(dx * dx + dy * dy)

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
class FastRectangleTracker:
    def __init__(self):
        self.last_corners = None
        self.pending_corners = None
        self.pending_hits = 0
        self.lost_frames = 999
        self.frame_id = 0
        self.use_cv_lite = HAS_CV_LITE
        self.output_x = -1
        self.output_y = -1
        self.output_hold = 0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.last_center = None

    def _track_speed(self):
        return math.sqrt(self.vel_x * self.vel_x + self.vel_y * self.vel_y)

    def _predicted_corners(self, frames_ahead=1):
        if self.last_corners is None:
            return None
        if frames_ahead < 1:
            frames_ahead = 1
        return shift_corners(self.last_corners,
                             self.vel_x * frames_ahead,
                             self.vel_y * frames_ahead)

    def _tracking_roi(self):
        predicted = self._predicted_corners(self.lost_frames + 1)
        if predicted is None:
            return (0, 0, IMG_W, IMG_H)
        margin_x = TRACK_MARGIN_X + int(abs(self.vel_x) * TRACK_MARGIN_X_VEL_GAIN)
        margin_y = TRACK_MARGIN + int(abs(self.vel_y) * TRACK_MARGIN_VEL_GAIN)
        margin_x += self.lost_frames * TRACK_LOST_MARGIN_STEP
        margin_y += self.lost_frames * TRACK_LOST_MARGIN_STEP
        margin_x = int(clamp(margin_x, TRACK_MARGIN_X, TRACK_MARGIN_X_MAX))
        margin_y = int(clamp(margin_y, TRACK_MARGIN, TRACK_MARGIN_MAX))
        return corners_to_asymmetric_box(predicted, margin_x, margin_y)

    def _update_motion(self, corners):
        cx, cy = simple_center(corners)
        if self.last_center is not None:
            dx = cx - self.last_center[0]
            dy = cy - self.last_center[1]
            keep = TRACK_VELOCITY_SMOOTH_DEN - TRACK_VELOCITY_SMOOTH_NUM
            self.vel_x = (self.vel_x * keep + dx * TRACK_VELOCITY_SMOOTH_NUM) / TRACK_VELOCITY_SMOOTH_DEN
            self.vel_y = (self.vel_y * keep + dy * TRACK_VELOCITY_SMOOTH_NUM) / TRACK_VELOCITY_SMOOTH_DEN
        self.last_center = (cx, cy)

    def _reset_motion(self):
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.last_center = None

    def _cv_lite_rects(self, gray, roi):
        if not self.use_cv_lite:
            return []
        x0, y0, rw, rh = roi
        try:
            work = gray if (x0 == 0 and y0 == 0 and rw == IMG_W and rh == IMG_H) \
                else gray.copy(roi=roi)
            np_img = work.to_numpy_ref()
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
        for r in rects:
            bw = int(r[2])
            bh = int(r[3])
            box_area = bw * bh
            if box_area < OUTER_MIN_AREA or box_area > OUTER_MAX_AREA:
                continue
            if bw <= 0 or bh <= 0:
                continue
            aspect = bw / bh
            if aspect < ASPECT_MIN or aspect > ASPECT_MAX:
                continue
            corners = [
                (int(r[4]) + x0, int(r[5]) + y0),
                (int(r[6]) + x0, int(r[7]) + y0),
                (int(r[8]) + x0, int(r[9]) + y0),
                (int(r[10]) + x0, int(r[11]) + y0),
            ]
            if not validate_rect_angles(corners):
                continue
            area = polygon_area(corners)
            if area < OUTER_MIN_AREA or area > OUTER_MAX_AREA:
                continue
            out.append((order_corners(corners), area))
        return out

    def _find_rects(self, gray, roi):
        x0, y0, rw, rh = roi
        try:
            if x0 == 0 and y0 == 0 and rw == IMG_W and rh == IMG_H:
                rects = gray.find_rects(threshold=FIND_RECTS_THRESHOLD)
                offset_x = 0
                offset_y = 0
            else:
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
        for r in rects:
            try:
                corners_roi = r.corners()
                bx, by, bw, bh = r.rect()
            except Exception:
                continue
            box_area = int(bw) * int(bh)
            if box_area < OUTER_MIN_AREA or box_area > OUTER_MAX_AREA:
                continue
            aspect = bw / max(1, bh)
            if aspect < ASPECT_MIN or aspect > ASPECT_MAX:
                continue
            corners = [(int(p[0]) + offset_x, int(p[1]) + offset_y)
                       for p in corners_roi]
            if not validate_rect_angles(corners):
                continue
            out.append((order_corners(corners), polygon_area(corners)))
        return out

    def _find_blobs_fast(self, img, thresholds, **kwargs):
        try:
            return img.find_blobs(thresholds, x_stride=2, y_stride=2, **kwargs)
        except TypeError:
            return img.find_blobs(thresholds, **kwargs)

    def _blob_rects(self, gray, roi):
        x0, y0, rw, rh = roi
        try:
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
        for b in blobs:
            bx, by, bw, bh = b.rect()
            area = int(bw) * int(bh)
            if area < OUTER_MIN_AREA or area > OUTER_MAX_AREA:
                continue
            aspect = bw / max(1, bh)
            if aspect < ASPECT_MIN or aspect > ASPECT_MAX:
                continue
            try:
                density = b.density()
                if density < 0.08 or density > 0.55:
                    continue
            except Exception:
                pass
            corners = order_corners([
                (bx + offset_x, by + offset_y),
                (bx + bw + offset_x, by + offset_y),
                (bx + bw + offset_x, by + bh + offset_y),
                (bx + offset_x, by + bh + offset_y),
            ])
            out.append((corners, area))
        return out

    def _scan_roi(self, gray, roi):
        tracking_roi = (
            self.last_corners is not None and
            self.lost_frames < LOCK_HOLD_FRAMES and
            roi != (0, 0, IMG_W, IMG_H)
        )
        if tracking_roi:
            cands = self._blob_rects(gray, roi)
            if cands:
                return cands
        cands = self._cv_lite_rects(gray, roi)
        if not cands:
            cands = self._blob_rects(gray, roi)
        if not cands:
            cands = self._find_rects(gray, roi)
        return cands

    def _choose_best(self, gray, cands):
        if not cands:
            return None
        best = None
        best_score = -1
        for corners, area in cands:
            frame_score = black_frame_score(gray, corners)
            min_score = FRAME_SCORE_MIN_TRACK if self.last_corners is not None \
                else FRAME_SCORE_MIN_ACQUIRE
            if frame_score < min_score:
                continue
            cx, cy = simple_center(corners)
            score = area * 0.001 + frame_score
            if self.last_corners is not None:
                ref = self._predicted_corners(1)
                if ref is None:
                    ref = self.last_corners
                lx, ly = simple_center(ref)
                dist = abs(cx - lx) + abs(cy - ly)
                ar = area_change_ratio(self.last_corners, corners)
                score += max(0.0, 18.0 - dist * 0.18)
                score -= ar * 18.0
            else:
                center_penalty = (abs(cx - IMG_W // 2) +
                                  abs(cy - IMG_H // 2)) * 0.01
                score -= center_penalty
            if score > best_score:
                best_score = score
                best = (corners, area)
        return best

    def _accept_candidate(self, corners):
        corners = order_corners(corners)
        if self.last_corners is not None:
            jump = center_distance(self.last_corners, corners)
            ar = area_change_ratio(self.last_corners, corners)
            if jump > JUMP_REJECT_PIXELS or ar > AREA_CHANGE_REJECT:
                predicted = self._predicted_corners(self.lost_frames + 1)
                if predicted is not None:
                    predict_jump = center_distance(predicted, corners)
                    predict_limit = TRACK_PREDICT_ACCEPT_PIXELS + self._track_speed() * TRACK_PREDICT_ACCEPT_GAIN
                    if predict_jump <= predict_limit and ar <= AREA_CHANGE_REJECT:
                        self.pending_corners = None
                        self.pending_hits = 0
                        return smooth_corners(self.last_corners, corners), True
                if self.pending_corners is not None and \
                        center_distance(self.pending_corners, corners) < 36:
                    self.pending_hits += 1
                else:
                    self.pending_corners = corners
                    self.pending_hits = 1
                if self.pending_hits < SWITCH_CONFIRM_FRAMES:
                    return self.last_corners, False
            self.pending_corners = None
            self.pending_hits = 0
            return smooth_corners(self.last_corners, corners), True
        if self.pending_corners is not None and \
                center_distance(self.pending_corners, corners) < 36:
            self.pending_hits += 1
        else:
            self.pending_corners = corners
            self.pending_hits = 1
        if self.pending_hits < SWITCH_CONFIRM_FRAMES:
            if self.last_corners is None:
                return corners, False
            return None, False
        self.pending_corners = None
        self.pending_hits = 0
        return corners, True

    def detect(self, gray):
        self.frame_id += 1
        full_roi = (0, 0, IMG_W, IMG_H)
        rois = []
        if self.last_corners is not None and self.lost_frames < LOCK_HOLD_FRAMES:
            rois.append(self._tracking_roi())
        need_full = self.last_corners is None or self.lost_frames >= LOCK_HOLD_FRAMES
        if FULL_RESCAN_PERIOD and self.last_corners is not None:
            rescan_period = FULL_RESCAN_PERIOD
            if self._track_speed() >= TRACK_FAST_SPEED_PIXELS:
                rescan_period = TRACK_FAST_FULL_RESCAN_PERIOD
            need_full = need_full or (self.frame_id % rescan_period) == 0
        if need_full:
            rois.append(full_roi)
        for roi in rois:
            best = self._choose_best(gray, self._scan_roi(gray, roi))
            if best is not None:
                corners, _ = best
                accepted, fresh = self._accept_candidate(corners)
                if accepted is None:
                    break
                if fresh:
                    self._update_motion(corners)
                self.last_corners = accepted
                self.lost_frames = 0
                cx, cy = perspective_map_point(self.last_corners,
                                                TARGET_PLANE_U, TARGET_PLANE_V)
                cx, cy = int(cx + 0.5), int(cy + 0.5)
                return cx, cy, self.last_corners, fresh
        if self.last_corners is not None and self.lost_frames < LOCK_HOLD_FRAMES:
            self.lost_frames += 1
            cx, cy = perspective_map_point(self.last_corners,
                                            TARGET_PLANE_U, TARGET_PLANE_V)
            cx, cy = int(cx + 0.5), int(cy + 0.5)
            return cx, cy, self.last_corners, False
        self.last_corners = None
        self.pending_corners = None
        self.pending_hits = 0
        self.lost_frames = 999
        self._reset_motion()
        return -1, -1, None, False

    def _stable_output(self, x, y, valid):
        if valid and x >= 0 and y >= 0:
            if self.output_x < 0 or self.output_y < 0:
                self.output_x = int(x)
                self.output_y = int(y)
            else:
                keep = OUTPUT_SMOOTH_DEN - OUTPUT_SMOOTH_NUM
                self.output_x = (self.output_x * keep + int(x) * OUTPUT_SMOOTH_NUM) // OUTPUT_SMOOTH_DEN
                self.output_y = (self.output_y * keep + int(y) * OUTPUT_SMOOTH_NUM) // OUTPUT_SMOOTH_DEN
            self.output_hold = OUTPUT_HOLD_FRAMES
            return int(self.output_x), int(self.output_y)
        if self.output_hold > 0 and self.output_x >= 0 and self.output_y >= 0:
            self.output_hold -= 1
            return int(self.output_x), int(self.output_y)
        self.output_x = -1
        self.output_y = -1
        return -1, -1


# ============================================================
# LCD叠加绘制
# ============================================================
def draw_axis_arrow(img, x0, y0, x1, y1, color):
    img.draw_line(x0, y0, x1, y1, color=color, thickness=2)
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


def draw_overlay(img, corners, target_x, target_y, fps, fresh, err_y, err_z):
    GREEN  = (0, 255, 0)
    RED    = (255, 0, 0)
    YELLOW = (255, 255, 0)
    WHITE  = (255, 255, 255)
    ORANGE = (255, 180, 0)
    CYAN   = (0, 255, 255)
    GRAY   = (128, 128, 128)
    AXIS_LEN = 42

    origin_x = ORIGIN_X
    origin_y = ORIGIN_Y
    frame_color = GREEN if fresh else ORANGE

    img.draw_line(origin_x - AXIS_LEN, origin_y, origin_x + AXIS_LEN, origin_y,
                  color=GRAY, thickness=1)
    img.draw_line(origin_x, origin_y - AXIS_LEN, origin_x, origin_y + AXIS_LEN,
                  color=GRAY, thickness=1)
    draw_axis_arrow(img, origin_x, origin_y, origin_x + AXIS_LEN, origin_y, CYAN)
    draw_axis_arrow(img, origin_x, origin_y, origin_x, origin_y - AXIS_LEN, YELLOW)

    img.draw_circle(origin_x, origin_y, 5, color=GREEN, thickness=2)
    img.draw_cross(origin_x, origin_y, color=GREEN, size=7, thickness=1)

    if corners is not None and target_x >= 0:
        for i in range(4):
            x0, y0 = corners[i]
            x1, y1 = corners[(i + 1) & 3]
            img.draw_line(int(x0), int(y0), int(x1), int(y1),
                          color=frame_color, thickness=2)
        img.draw_circle(int(target_x), int(target_y), 4,
                        color=RED, thickness=2)
        img.draw_line(origin_x, origin_y, int(target_x), origin_y,
                      color=CYAN, thickness=1)
        img.draw_line(int(target_x), origin_y, int(target_x), int(target_y),
                      color=YELLOW, thickness=1)
        img.draw_line(origin_x, origin_y, int(target_x), int(target_y),
                      color=WHITE, thickness=1)

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
def send_tracking(uart, err_y, err_z, fps=0):
    uart.write(pack_error_packet(int(err_y), int(err_z), fps))


def init_uart():
    fpioa = FPIOA()
    fpioa.set_function(50, FPIOA.UART3_TXD)
    fpioa.set_function(51, FPIOA.UART3_RXD)
    return UART(UART.UART3, baudrate=UART_BAUD,
                bits=UART.EIGHTBITS,
                parity=UART.PARITY_NONE,
                stop=UART.STOPBITS_ONE)


def init_camera():
    try:
        sensor = Sensor(width=CAM_W, height=CAM_H, fps=CAM_FPS)
    except TypeError:
        sensor = Sensor(width=CAM_W, height=CAM_H)
    sensor.reset()
    sensor.set_framesize(width=CAM_W, height=CAM_H)
    if not HEADLESS:
        sensor.set_pixformat(Sensor.RGB565)
        pixfmt = "RGB565"
    elif SENSOR_PIXFORMAT == "RGB565":
        sensor.set_pixformat(Sensor.RGB565)
        pixfmt = "RGB565"
    else:
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
    uart = init_uart()
    sensor, pixfmt = init_camera()

    has_display = False
    if not HEADLESS:
        try:
            Display.init(Display.ST7701, width=800, height=480,
                         to_ide=SHOW_TO_IDE)
            has_display = True
        except Exception:
            pass

    MediaManager.init()
    sensor.run()

    tracker = FastRectangleTracker()
    clock = time.clock()
    gc_count = 0
    display_count = 0

    target_x, target_y = -1, -1
    corners = None

    while True:
        clock.tick()
        os.exitpoint()
        img = sensor.snapshot()

        # ===== 以激光点为中心裁剪640x480检测画面 =====
        crop_roi = laser_center_roi()
        if crop_roi == (0, 0, CAM_W, CAM_H):
            crop_img = img
        else:
            crop_img = img.copy(roi=crop_roi)

        if pixfmt == "GRAYSCALE":
            gray = crop_img
        else:
            gray = crop_img.to_grayscale(copy=True)

        target_x, target_y, corners, fresh = tracker.detect(gray)
        stable_x, stable_y = tracker._stable_output(target_x, target_y, fresh)
        target_valid = fresh and stable_x >= 0 and stable_y >= 0
        if target_valid:
            send_y = stable_x - ORIGIN_X
            send_z = ORIGIN_Y - stable_y
        else:
            send_y = 0
            send_z = 0
        fps = int(clock.fps() + 0.5)
        send_tracking(uart, send_y, send_z, fps if target_valid else 0)

        if has_display:
            display_count += 1
            if display_count >= DISPLAY_EVERY_N:
                display_count = 0
                draw_overlay(crop_img, corners, target_x, target_y, fps, fresh, send_y, send_z)
                Display.show_image(crop_img, x=(800 - IMG_W) // 2, y=(480 - IMG_H) // 2)

        gc_count += 1
        if gc_count >= 60:
            gc_count = 0
            gc.collect()

except KeyboardInterrupt as e:
    pass
except BaseException as e:
    print("err:", e)
finally:
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
