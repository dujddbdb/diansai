# -*- coding: utf-8 -*-
# K230简化版卡尔曼目标跟踪器
# 与main.py相同的IO协议:
# - 摄像头输入: 1920x1080
# - 检测裁剪: CROP_CENTER_X/Y周围320x240区域
# - UART3输出: 大端格式int16类型的err_y, err_z

import gc
import os
import time
from machine import FPIOA, UART
from media.display import *
from media.media import *
from media.sensor import *

ERR_INVALID = 0x7FFF  # 无效误差值标记，表示目标丢失

# 图像与通信参数
IMG_W = 320     # 检测图像宽度(像素)
IMG_H = 240     # 检测图像高度(像素)
CAM_W = 1920    # 摄像头全帧宽度(像素)
CAM_H = 1080    # 摄像头全帧高度(像素)
CAM_FPS = 60    # 摄像头帧率，调大则帧率高但CPU占用高
UART_BAUD = 115200  # 串口波特率
FULL_CHN = CAM_CHN_ID_0  # 摄像头完整输入通道，保持1920x1080
DETECT_CHN = CAM_CHN_ID_1  # 硬件裁剪识别通道，输出320x240

# 手动校准：调整全帧裁剪中心，直到真实激光点出现在320x240检测图像的中心
CROP_CENTER_X = CAM_W // 2  # 裁剪区域中心X坐标(全帧坐标系)
CROP_CENTER_Y = CAM_H // 2 - 77  # 裁剪区域中心Y坐标(全帧坐标系)，调大则下移，调小则上移
CROP_X = max(0, min(CAM_W - IMG_W, CROP_CENTER_X - IMG_W // 2))  # 裁剪区域左上角X
CROP_Y = max(0, min(CAM_H - IMG_H, CROP_CENTER_Y - IMG_H // 2))  # 裁剪区域左上角Y
CROP_ROI = (CROP_X, CROP_Y, IMG_W, IMG_H)  # 裁剪区域元组
ORIGIN_X = IMG_W // 2  # 原点X坐标(裁剪图像坐标系)
ORIGIN_Y = IMG_H // 2  # 原点Y坐标(裁剪图像坐标系)

# 显示模式
HEADLESS = True           # 无头模式，True则不初始化LCD显示
SHOW_TO_IDE = False       # 是否输出图像到IDE预览
SENSOR_PIXFORMAT = "GRAYSCALE"  # 传感器像素格式
DISPLAY_EVERY_N = 12      # 每N帧刷新一次显示，调大则显示帧率低但节省CPU

# 检测参数
FIND_RECTS_THRESHOLD = 9000  # 矩形检测阈值，调大则检测更少但更精准
FIND_RECTS_FALLBACK_PERIOD = 30  # find_rects较慢，仅低频兜底
BLOB_X_STRIDE = 3                # Blob水平步进，调大速度更快但细线更容易漏检
BLOB_Y_STRIDE = 3                # Blob垂直步进，调大速度更快但细线更容易漏检
BLACK_GRAY_TH = 95           # 黑色灰度阈值，调大则更多像素被判定为黑色
OUTER_MIN_AREA = 700         # 外框最小面积(像素²)
OUTER_MAX_AREA = 90000       # 外框最大面积(像素²)
ASPECT_MIN = 0.55            # 最小宽高比
ASPECT_MAX = 1.75            # 最大宽高比
MAX_ASSOC_DIST = 80          # 最大关联距离(像素)，超过则认为是新目标


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


# 打包误差数据包：将err_y和err_z打包为4字节大端格式
# 参数:
#   err_y: Y轴误差值
#   err_z: Z轴误差值
# 返回值: 4字节的bytes对象
def pack_error_packet(err_y, err_z):
    # 确保数值在16位有符号整数范围内
    err_y &= 0xFFFF
    err_z &= 0xFFFF
    # 按大端字节序打包
    return bytes((err_y >> 8, err_y & 0xFF,
                  err_z >> 8, err_z & 0xFF))


# 计算控制误差：目标中心相对于原点的误差
# 参数:
#   cx: 目标中心X坐标
#   cy: 目标中心Y坐标
# 返回值: (err_y, err_z) Y轴和Z轴方向的误差
def control_error(cx, cy):
    return int(cx) - ORIGIN_X, ORIGIN_Y - int(cy)


# 矩形中心计算：计算矩形的中心点坐标
# 参数:
#   rect: 矩形元组 (x, y, w, h)
# 返回值: (cx, cy) 中心点坐标
def rect_center(rect):
    x, y, w, h = rect
    return x + w // 2, y + h // 2


# 一维卡尔曼滤波器：对单个坐标进行卡尔曼滤波
# 状态: [位置, 速度]
# 观测: 位置
class Kalman1D:
    # 初始化卡尔曼滤波器
    def __init__(self):
        self.x = 0.0      # 位置估计值
        self.v = 0.0      # 速度估计值
        self.p00 = 1.0    # 协方差矩阵元素(位置-位置)
        self.p01 = 0.0    # 协方差矩阵元素(位置-速度)
        self.p10 = 0.0    # 协方差矩阵元素(速度-位置)
        self.p11 = 1.0    # 协方差矩阵元素(速度-速度)
        self.valid = False  # 滤波器是否有效(是否已初始化)

    # 重置滤波器，用测量值初始化
    # 参数:
    #   value: 初始测量值
    def reset(self, value):
        self.x = float(value)
        self.v = 0.0
        self.p00 = 1.0
        self.p01 = 0.0
        self.p10 = 0.0
        self.p11 = 1.0
        self.valid = True

    # 卡尔曼滤波步进：执行预测和更新
    # 参数:
    #   measurement: 测量值(位置)
    #   has_measurement: 是否有有效测量值
    #   dt: 时间步长(秒)
    # 返回值: 滤波后的位置估计值
    def step(self, measurement, has_measurement, dt):
        # 未初始化时，有测量值则初始化
        if not self.valid:
            if has_measurement:
                self.reset(measurement)
            return self.x

        # ===== 预测步 =====
        # 状态预测: x = x + v*dt
        self.x += self.v * dt
        # 协方差预测(过程噪声Q: 位置0.05, 速度0.20)
        # 调大Q值: 滤波器响应更快，但噪声更多
        # 调小Q值: 滤波器更平滑，但响应更慢
        p00 = self.p00 + dt * (self.p10 + self.p01) + dt * dt * self.p11 + 0.05
        p01 = self.p01 + dt * self.p11
        p10 = self.p10 + dt * self.p11
        p11 = self.p11 + 0.20
        self.p00, self.p01, self.p10, self.p11 = p00, p01, p10, p11

        # ===== 更新步(有测量值时) =====
        if has_measurement:
            # 观测噪声R=4.0，调大则更信任预测(更平滑)，调小则更信任测量(更灵敏)
            s = self.p00 + 4.0
            # 计算卡尔曼增益
            k0 = self.p00 / s  # 位置增益
            k1 = self.p10 / s  # 速度增益
            # 计算创新(残差)
            innovation = float(measurement) - self.x
            # 更新状态
            old_p00 = self.p00
            old_p01 = self.p01
            self.x += k0 * innovation
            self.v += k1 * innovation
            # 更新协方差
            self.p00 = (1.0 - k0) * old_p00
            self.p01 = (1.0 - k0) * old_p01
            self.p10 = self.p10 - k1 * old_p00
            self.p11 = self.p11 - k1 * old_p01
        return self.x


# 目标卡尔曼跟踪器：二维卡尔曼滤波 + 目标关联 + 丢失保持
class TargetKalman:
    # 初始化跟踪器
    def __init__(self):
        self.x = Kalman1D()   # X方向卡尔曼滤波器
        self.y = Kalman1D()   # Y方向卡尔曼滤波器
        self.lost = 999       # 目标丢失帧数
        self.last_ms = time.ticks_ms()  # 上一帧时间戳(毫秒)

    # 更新跟踪器状态
    # 参数:
    #   mx, my: 测量坐标(检测到的目标中心)
    #   valid: 测量是否有效
    # 返回值: (tx, ty, valid) 滤波后的坐标和有效性
    def update(self, mx, my, valid):
        # 计算时间步长
        now = time.ticks_ms()
        dt_ms = time.ticks_diff(now, self.last_ms)
        self.last_ms = now
        if dt_ms < 1:
            dt_ms = 1
        if dt_ms > 40:
            dt_ms = 40
        dt = dt_ms * 0.001  # 转换为秒

        if valid:
            # 有有效测量值
            if self.x.valid:
                # 已有跟踪目标，先做一步预测
                px = self.x.step(0, False, dt)
                py = self.y.step(0, False, dt)
                # 预测位置与测量位置差距过大，认为是新目标，重置滤波器
                if abs(mx - px) + abs(my - py) > MAX_ASSOC_DIST:
                    self.x.reset(mx)
                    self.y.reset(my)
                else:
                    # 正常更新
                    self.x.step(mx, True, 0.0)
                    self.y.step(my, True, 0.0)
            else:
                # 首次捕获，直接初始化
                self.x.reset(mx)
                self.y.reset(my)
            self.lost = 0
        else:
            # 无有效测量值，只做预测
            self.x.step(0, False, dt)
            self.y.step(0, False, dt)
            if self.lost < 999:
                self.lost += 1

        # 丢失3帧内仍输出预测值，超过则输出无效
        if self.x.valid and self.y.valid and self.lost <= 3:
            return int(self.x.x + 0.5), int(self.y.x + 0.5), True
        return -1, -1, False


# 验证矩形是否符合尺寸和宽高比要求
# 参数:
#   rect: 矩形元组 (x, y, w, h)
# 返回值: True表示合格，False表示不合格
def valid_rect(rect):
    x, y, w, h = rect
    area = int(w) * int(h)
    # 面积过滤
    if area < OUTER_MIN_AREA or area > OUTER_MAX_AREA:
        return False
    # 宽高比过滤
    aspect = w / max(1, h)
    return ASPECT_MIN <= aspect <= ASPECT_MAX


# 检测候选目标：Blob快速热路径，find_rects低频兜底
# 参数:
#   gray: 灰度图像
# 返回值: (cx, cy, rect, valid) 中心坐标、矩形、是否有效
def detect_candidate(gray, frame_id=0):
    best = None
    best_score = -1

    # 方法1: Blob快速检测，作为50FPS热路径
    try:
        blobs = gray.find_blobs([(0, BLACK_GRAY_TH)],
                                x_stride=BLOB_X_STRIDE,
                                y_stride=BLOB_Y_STRIDE,
                                pixels_threshold=180,
                                area_threshold=180,
                                merge=True)
        for b in blobs:
            rect = b.rect()
            if not valid_rect(rect):
                continue
            x, y, w, h = rect
            score = w * h  # 面积作为评分
            if score > best_score:
                best = rect
                best_score = score
    except Exception:
        pass

    # 方法2: find_rects低频兜底
    if (best is None and
            (FIND_RECTS_FALLBACK_PERIOD <= 1 or
             (frame_id % FIND_RECTS_FALLBACK_PERIOD) == 0)):
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

    # 无合格目标
    if best is None:
        return -1, -1, None, False

    # 返回最大面积的目标
    cx, cy = rect_center(best)
    return cx, cy, best, True


# 发送跟踪数据：通过UART发送误差数据
# 参数:
#   uart: UART对象
#   err_y: Y轴误差
#   err_z: Z轴误差
def send_tracking(uart, err_y, err_z):
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


# 绘制叠加层：原点标记、目标框、误差信息、FPS等
# 参数:
#   img: 图像对象
#   rect: 目标矩形
#   tx, ty: 目标中心坐标(滤波后)
#   fps: 帧率
#   err_y, err_z: Y/Z轴误差
def draw_overlay(img, rect, tx, ty, fps, err_y, err_z):
    green = (0, 255, 0)
    red = (255, 0, 0)
    white = (255, 255, 255)
    # 绘制原点标记
    img.draw_cross(ORIGIN_X, ORIGIN_Y, color=green, size=9, thickness=1)
    img.draw_circle(ORIGIN_X, ORIGIN_Y, 5, color=green, thickness=1)
    # 绘制目标框和目标点
    if rect is not None and tx >= 0:
        x, y, w, h = rect
        img.draw_rectangle(x, y, w, h, color=green, thickness=2)
        img.draw_circle(tx, ty, 4, color=red, thickness=2)
        # 绘制误差线
        img.draw_line(ORIGIN_X, ORIGIN_Y, tx, ty, color=white, thickness=1)
    # 绘制文字信息
    try:
        img.draw_string_advanced(4, 4, 14, "FPS:%d" % fps, color=white)
        img.draw_string_advanced(4, 22, 14,
                                 "ERR_Y:%d ERR_Z:%d" % (err_y, err_z),
                                 color=white)
    except Exception:
        pass


# 全局硬件对象
sensor = None
uart = None
has_display = False

try:
    # ===== 硬件初始化 =====
    uart = init_uart()
    sensor, pixfmt, detect_chn, sensor_crop_enabled = init_camera()

    # 初始化LCD显示
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
    tracker = TargetKalman()
    clock = time.clock()
    display_count = 0
    gc_count = 0
    frame_id = 0

    # ===== 主循环 =====
    while True:
        clock.tick()
        os.exitpoint()
        frame_id += 1
        frame = sensor.snapshot(chn=detect_chn)
        # 优先使用K230传感器输出通道硬件裁剪；旧固件失败时保留软件裁剪兜底
        if sensor_crop_enabled:
            view = frame
        else:
            view = frame.copy(roi=detection_center_roi())
        # 转换为灰度图
        gray = view if pixfmt == "GRAYSCALE" else view.to_grayscale(copy=True)

        # 检测候选目标
        mx, my, rect, measured = detect_candidate(gray, frame_id)
        # 卡尔曼滤波更新
        tx, ty, valid = tracker.update(mx, my, measured)
        # 计算控制误差
        if valid:
            err_y, err_z = control_error(tx, ty)
        else:
            err_y = ERR_INVALID
            err_z = ERR_INVALID
        # 发送跟踪数据
        send_tracking(uart, err_y, err_z)

        # LCD显示(降频刷新)
        fps = int(clock.fps() + 0.5)
        if has_display:
            display_count += 1
            if display_count >= DISPLAY_EVERY_N:
                display_count = 0
                draw_overlay(view, rect, tx, ty, fps, err_y, err_z)
                Display.show_image(view, x=(800 - IMG_W) // 2,
                                   y=(480 - IMG_H) // 2)

        # 定期垃圾回收
        gc_count += 1
        if gc_count >= 60:
            gc_count = 0
            gc.collect()

except KeyboardInterrupt:
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
