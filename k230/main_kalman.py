# -*- coding: utf-8 -*-
"""
K230 simplified Kalman target tracker.

Same IO contract as main.py:
- camera input: 1920x1080
- detection crop: 320x240 around CROP_CENTER_X/Y
- UART3 output: raw big-endian int16 err_y, err_z
"""

import gc
import os
import time
from machine import FPIOA, UART
from media.display import *
from media.media import *
from media.sensor import *

ERR_INVALID = 0x7FFF

IMG_W = 320
IMG_H = 240
CAM_W = 1920
CAM_H = 1080
CAM_FPS = 60
UART_BAUD = 115200

CROP_CENTER_X = CAM_W // 2
CROP_CENTER_Y = CAM_H // 2 - 77
CROP_X = max(0, min(CAM_W - IMG_W, CROP_CENTER_X - IMG_W // 2))
CROP_Y = max(0, min(CAM_H - IMG_H, CROP_CENTER_Y - IMG_H // 2))
CROP_ROI = (CROP_X, CROP_Y, IMG_W, IMG_H)
ORIGIN_X = CROP_CENTER_X - CROP_X
ORIGIN_Y = CROP_CENTER_Y - CROP_Y

HEADLESS = False
SHOW_TO_IDE = False
SENSOR_PIXFORMAT = "GRAYSCALE"
DISPLAY_EVERY_N = 3

FIND_RECTS_THRESHOLD = 4500
BLACK_GRAY_TH = 95
OUTER_MIN_AREA = 700
OUTER_MAX_AREA = 62000
ASPECT_MIN = 0.55
ASPECT_MAX = 1.75
MAX_ASSOC_DIST = 80


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


def detection_center_roi():
    return CROP_ROI


def pack_error_packet(err_y, err_z):
    err_y &= 0xFFFF
    err_z &= 0xFFFF
    return bytes((err_y >> 8, err_y & 0xFF,
                  err_z >> 8, err_z & 0xFF))


def control_error(cx, cy):
    return int(cx) - ORIGIN_X, ORIGIN_Y - int(cy)


def rect_center(rect):
    x, y, w, h = rect
    return x + w // 2, y + h // 2


class Kalman1D:
    def __init__(self):
        self.x = 0.0
        self.v = 0.0
        self.p00 = 1.0
        self.p01 = 0.0
        self.p10 = 0.0
        self.p11 = 1.0
        self.valid = False

    def reset(self, value):
        self.x = float(value)
        self.v = 0.0
        self.p00 = 1.0
        self.p01 = 0.0
        self.p10 = 0.0
        self.p11 = 1.0
        self.valid = True

    def step(self, measurement, has_measurement, dt):
        if not self.valid:
            if has_measurement:
                self.reset(measurement)
            return self.x

        self.x += self.v * dt
        p00 = self.p00 + dt * (self.p10 + self.p01) + dt * dt * self.p11 + 0.05
        p01 = self.p01 + dt * self.p11
        p10 = self.p10 + dt * self.p11
        p11 = self.p11 + 0.20
        self.p00, self.p01, self.p10, self.p11 = p00, p01, p10, p11

        if has_measurement:
            s = self.p00 + 4.0
            k0 = self.p00 / s
            k1 = self.p10 / s
            innovation = float(measurement) - self.x
            old_p00 = self.p00
            old_p01 = self.p01
            self.x += k0 * innovation
            self.v += k1 * innovation
            self.p00 = (1.0 - k0) * old_p00
            self.p01 = (1.0 - k0) * old_p01
            self.p10 = self.p10 - k1 * old_p00
            self.p11 = self.p11 - k1 * old_p01
        return self.x


class TargetKalman:
    def __init__(self):
        self.x = Kalman1D()
        self.y = Kalman1D()
        self.lost = 999
        self.last_ms = time.ticks_ms()

    def update(self, mx, my, valid):
        now = time.ticks_ms()
        dt_ms = time.ticks_diff(now, self.last_ms)
        self.last_ms = now
        if dt_ms < 1:
            dt_ms = 1
        if dt_ms > 40:
            dt_ms = 40
        dt = dt_ms * 0.001

        if valid:
            if self.x.valid:
                px = self.x.step(0, False, dt)
                py = self.y.step(0, False, dt)
                if abs(mx - px) + abs(my - py) > MAX_ASSOC_DIST:
                    self.x.reset(mx)
                    self.y.reset(my)
                else:
                    self.x.step(mx, True, 0.0)
                    self.y.step(my, True, 0.0)
            else:
                self.x.reset(mx)
                self.y.reset(my)
            self.lost = 0
        else:
            self.x.step(0, False, dt)
            self.y.step(0, False, dt)
            if self.lost < 999:
                self.lost += 1

        if self.x.valid and self.y.valid and self.lost <= 3:
            return int(self.x.x + 0.5), int(self.y.x + 0.5), True
        return -1, -1, False


def valid_rect(rect):
    x, y, w, h = rect
    area = int(w) * int(h)
    if area < OUTER_MIN_AREA or area > OUTER_MAX_AREA:
        return False
    aspect = w / max(1, h)
    return ASPECT_MIN <= aspect <= ASPECT_MAX


def detect_candidate(gray):
    best = None
    best_score = -1

    try:
        rects = gray.find_rects(threshold=FIND_RECTS_THRESHOLD)
        for r in rects:
            rect = r.rect()
            if not valid_rect(rect):
                continue
            x, y, w, h = rect
            score = w * h
            if score > best_score:
                best = rect
                best_score = score
    except Exception:
        pass

    if best is None:
        try:
            blobs = gray.find_blobs([(0, BLACK_GRAY_TH)],
                                    x_stride=2, y_stride=2,
                                    pixels_threshold=180,
                                    area_threshold=180,
                                    merge=True)
            for b in blobs:
                rect = b.rect()
                if not valid_rect(rect):
                    continue
                x, y, w, h = rect
                score = w * h
                if score > best_score:
                    best = rect
                    best_score = score
        except Exception:
            pass

    if best is None:
        return -1, -1, None, False

    cx, cy = rect_center(best)
    return cx, cy, best, True


def send_tracking(uart, err_y, err_z):
    uart.write(pack_error_packet(int(err_y), int(err_z)))


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


def draw_overlay(img, rect, tx, ty, fps, err_y, err_z):
    green = (0, 255, 0)
    red = (255, 0, 0)
    white = (255, 255, 255)
    img.draw_cross(ORIGIN_X, ORIGIN_Y, color=green, size=9, thickness=1)
    img.draw_circle(ORIGIN_X, ORIGIN_Y, 5, color=green, thickness=1)
    if rect is not None and tx >= 0:
        x, y, w, h = rect
        img.draw_rectangle(x, y, w, h, color=green, thickness=2)
        img.draw_circle(tx, ty, 4, color=red, thickness=2)
        img.draw_line(ORIGIN_X, ORIGIN_Y, tx, ty, color=white, thickness=1)
    try:
        img.draw_string_advanced(4, 4, 14, "FPS:%d" % fps, color=white)
        img.draw_string_advanced(4, 22, 14,
                                 "ERR_Y:%d ERR_Z:%d" % (err_y, err_z),
                                 color=white)
    except Exception:
        pass


sensor = None
uart = None
has_display = False

try:
    uart = init_uart()
    sensor, pixfmt = init_camera()
    if not HEADLESS:
        try:
            Display.init(Display.ST7701, width=800, height=480,
                         to_ide=SHOW_TO_IDE)
            has_display = True
        except Exception:
            pass

    MediaManager.init()
    sensor.run()
    tracker = TargetKalman()
    clock = time.clock()
    display_count = 0
    gc_count = 0

    while True:
        clock.tick()
        os.exitpoint()
        frame = sensor.snapshot()
        view = frame.copy(roi=detection_center_roi())
        gray = view if pixfmt == "GRAYSCALE" else view.to_grayscale(copy=True)

        mx, my, rect, measured = detect_candidate(gray)
        tx, ty, valid = tracker.update(mx, my, measured)
        if valid:
            err_y, err_z = control_error(tx, ty)
        else:
            err_y = ERR_INVALID
            err_z = ERR_INVALID
        send_tracking(uart, err_y, err_z)

        fps = int(clock.fps() + 0.5)
        if has_display:
            display_count += 1
            if display_count >= DISPLAY_EVERY_N:
                display_count = 0
                draw_overlay(view, rect, tx, ty, fps, err_y, err_z)
                Display.show_image(view, x=(800 - IMG_W) // 2,
                                   y=(480 - IMG_H) // 2)

        gc_count += 1
        if gc_count >= 60:
            gc_count = 0
            gc.collect()

except KeyboardInterrupt:
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
