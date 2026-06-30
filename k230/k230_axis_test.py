# -*- coding: utf-8 -*-

import gc
import os
import time
from machine import FPIOA, UART
from media.display import *
from media.media import *
from media.sensor import *


IMG_W = 320  # 图像宽度
IMG_H = 240  # 图像高度
UART_BAUD = 115200  # 串口波特率

ERROR_ORIGIN_X = IMG_W // 2  # 误差原点X(画面中心)
ERROR_ORIGIN_Y = IMG_H // 2  # 误差原点Y(画面中心)

TEST_ERR_Y = 50  # 测试Y轴误差值
TEST_ERR_Z = 50  # 测试Z轴误差值
AXIS_LEN = 70  # 坐标轴显示长度(像素)

HEADLESS = False  # 0-显示画面 1-无头模式(无显示)
SHOW_TO_IDE = True  # 0-不输出到IDE 1-输出到IDE预览


# 打包坐标数据: err_y和err_z打包为4字节大端格式
def pack_coordinate_packet(err_y, err_z):
    err_y &= 0xFFFF
    err_z &= 0xFFFF
    return bytes((err_y >> 8, err_y & 0xFF,
                  err_z >> 8, err_z & 0xFF))


# 发送坐标数据: 通过串口发送err_y和err_z
def send_tracking(uart, err_y, err_z):
    uart.write(pack_coordinate_packet(int(err_y), int(err_z)))


# 初始化串口: 配置UART3引脚和波特率
def init_uart():
    fpioa = FPIOA()
    fpioa.set_function(50, FPIOA.UART3_TXD)
    fpioa.set_function(51, FPIOA.UART3_RXD)
    return UART(UART.UART3, baudrate=UART_BAUD,
                bits=UART.EIGHTBITS,
                parity=UART.PARITY_NONE,
                stop=UART.STOPBITS_ONE)


# 初始化摄像头: 配置分辨率/像素格式
def init_camera():
    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(width=IMG_W, height=IMG_H)
    sensor.set_pixformat(Sensor.RGB565)
    return sensor


# 绘制文字: 兼容API绘制字符串
def draw_label(img, x, y, text, color):
    try:
        img.draw_string_advanced(int(x), int(y), 14, text, color=color)
    except Exception:
        pass


# 绘制坐标轴箭头: 从(x0,y0)到(x1,y1)画箭头线
def draw_axis_arrow(img, x0, y0, x1, y1, color):
    img.draw_line(x0, y0, x1, y1, color=color, thickness=2)
    dx = x1 - x0
    dy = y1 - y0
    if abs(dx) >= abs(dy):
        sx = 1 if dx >= 0 else -1
        img.draw_line(x1, y1, x1 - 8 * sx, y1 - 5, color=color, thickness=2)
        img.draw_line(x1, y1, x1 - 8 * sx, y1 + 5, color=color, thickness=2)
    else:
        sy = 1 if dy >= 0 else -1
        img.draw_line(x1, y1, x1 - 5, y1 - 8 * sy, color=color, thickness=2)
        img.draw_line(x1, y1, x1 + 5, y1 - 8 * sy, color=color, thickness=2)


# 绘制叠加层: 坐标轴/目标点/误差信息/FPS
def draw_overlay(img, fps):
    green = (0, 255, 0)
    red = (255, 0, 0)
    cyan = (0, 255, 255)
    yellow = (255, 255, 0)
    white = (255, 255, 255)
    gray = (110, 110, 110)

    target_x = ERROR_ORIGIN_X + TEST_ERR_Y
    target_y = ERROR_ORIGIN_Y - TEST_ERR_Z

    img.draw_circle(ERROR_ORIGIN_X, ERROR_ORIGIN_Y, 5, color=green, thickness=2)
    img.draw_cross(ERROR_ORIGIN_X, ERROR_ORIGIN_Y, color=green, size=9, thickness=1)

    img.draw_line(ERROR_ORIGIN_X - AXIS_LEN, ERROR_ORIGIN_Y,
                  ERROR_ORIGIN_X + AXIS_LEN, ERROR_ORIGIN_Y,
                  color=gray, thickness=1)
    img.draw_line(ERROR_ORIGIN_X, ERROR_ORIGIN_Y - AXIS_LEN,
                  ERROR_ORIGIN_X, ERROR_ORIGIN_Y + AXIS_LEN,
                  color=gray, thickness=1)

    draw_axis_arrow(img, ERROR_ORIGIN_X, ERROR_ORIGIN_Y,
                    ERROR_ORIGIN_X + AXIS_LEN, ERROR_ORIGIN_Y, cyan)
    draw_axis_arrow(img, ERROR_ORIGIN_X, ERROR_ORIGIN_Y,
                    ERROR_ORIGIN_X, ERROR_ORIGIN_Y - AXIS_LEN, yellow)

    img.draw_line(ERROR_ORIGIN_X, ERROR_ORIGIN_Y,
                  target_x, ERROR_ORIGIN_Y, color=cyan, thickness=1)
    img.draw_line(target_x, ERROR_ORIGIN_Y,
                  target_x, target_y, color=yellow, thickness=1)
    img.draw_line(ERROR_ORIGIN_X, ERROR_ORIGIN_Y,
                  target_x, target_y, color=white, thickness=1)

    img.draw_circle(target_x, target_y, 5, color=red, thickness=2)
    img.draw_cross(target_x, target_y, color=red, size=7, thickness=1)

    draw_label(img, ERROR_ORIGIN_X + AXIS_LEN + 4,
               ERROR_ORIGIN_Y - 8, "+Y", cyan)
    draw_label(img, ERROR_ORIGIN_X - 10,
               ERROR_ORIGIN_Y - AXIS_LEN - 18, "+Z", yellow)
    draw_label(img, ERROR_ORIGIN_X + 6, ERROR_ORIGIN_Y + 6, "O", green)
    draw_label(img, target_x + 6, target_y - 18, "P(Y=+50,Z=+50)", red)

    draw_label(img, 4, 4, "YOZ AXIS TEST (+Z UP)", white)
    draw_label(img, 4, 24,
               "send err_y=%d err_z=%d" % (TEST_ERR_Y, TEST_ERR_Z),
               white)
    draw_label(img, 4, 44, "screen: right=+Y, up=+Z", white)
    draw_label(img, 4, 64,
               "origin=(%d,%d)" % (ERROR_ORIGIN_X, ERROR_ORIGIN_Y),
               white)
    draw_label(img, 4, 84, "target=(%d,%d)" % (target_x, target_y), white)
    draw_label(img, 4, 104, "fps=%d" % fps, white)


sensor = None
uart = None
has_display = False  # 0-无显示 1-有显示

try:
    # 初始化: 串口+摄像头+显示+媒体管理器
    uart = init_uart()
    sensor = init_camera()

    if not HEADLESS:
        try:
            Display.init(Display.ST7701, width=800, height=480,
                         to_ide=SHOW_TO_IDE)
            has_display = True
        except Exception:
            has_display = False

    MediaManager.init()
    sensor.run()
    clock = time.clock()
    gc_count = 0

    # 主循环: 抓图->发送固定测试误差->显示
    while True:
        clock.tick()
        os.exitpoint()
        img = sensor.snapshot()
        fps = int(clock.fps() + 0.5)

        send_tracking(uart, TEST_ERR_Y, TEST_ERR_Z)

        if has_display:
            draw_overlay(img, fps)
            Display.show_image(img, x=(800 - IMG_W) // 2,
                               y=(480 - IMG_H) // 2)

        gc_count += 1
        if gc_count >= 60:  # 每60帧执行一次GC
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
        MediaManager.deinit()
    except Exception:
        pass
    if uart:
        uart.deinit()
