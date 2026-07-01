# -*- coding: utf-8 -*-
# K230坐标轴测试程序：用于验证UART通信和坐标系定义
# 发送固定的测试误差值，在LCD上显示坐标系和目标点

import gc
import os
import time
from machine import FPIOA, UART
from media.display import *
from media.media import *
from media.sensor import *


# ===== 全局配置参数 =====
IMG_W = 320  # 图像宽度(像素)
IMG_H = 240  # 图像高度(像素)
UART_BAUD = 115200  # 串口波特率

ERROR_ORIGIN_X = IMG_W // 2  # 误差原点X坐标(画面中心)
ERROR_ORIGIN_Y = IMG_H // 2  # 误差原点Y坐标(画面中心)

TEST_ERR_Y = 50  # 测试Y轴误差值(像素)，调大则目标点向右偏移更多
TEST_ERR_Z = 50  # 测试Z轴误差值(像素)，调大则目标点向上偏移更多
AXIS_LEN = 70  # 坐标轴显示长度(像素)

HEADLESS = False  # 无头模式，True则不初始化LCD显示
SHOW_TO_IDE = True  # 是否输出到IDE预览，True则在IDE中显示图像


# 打包坐标数据：err_y和err_z打包为4字节大端格式
# 参数:
#   err_y: Y轴误差值
#   err_z: Z轴误差值
# 返回值: 4字节bytes对象
def pack_coordinate_packet(err_y, err_z):
    # 确保数值在16位有符号整数范围内
    err_y &= 0xFFFF
    err_z &= 0xFFFF
    # 按大端字节序打包
    return bytes((err_y >> 8, err_y & 0xFF,
                  err_z >> 8, err_z & 0xFF))


# 发送坐标数据：通过串口发送err_y和err_z
# 参数:
#   uart: UART对象
#   err_y: Y轴误差
#   err_z: Z轴误差
def send_tracking(uart, err_y, err_z):
    uart.write(pack_coordinate_packet(int(err_y), int(err_z)))


# 初始化串口：配置UART3引脚和波特率
# 返回值: UART对象
def init_uart():
    fpioa = FPIOA()
    fpioa.set_function(50, FPIOA.UART3_TXD)  # 配置TX引脚
    fpioa.set_function(51, FPIOA.UART3_RXD)  # 配置RX引脚
    return UART(UART.UART3, baudrate=UART_BAUD,
                bits=UART.EIGHTBITS,
                parity=UART.PARITY_NONE,
                stop=UART.STOPBITS_ONE)


# 初始化摄像头：配置分辨率/像素格式
# 返回值: 传感器对象
def init_camera():
    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(width=IMG_W, height=IMG_H)
    sensor.set_pixformat(Sensor.RGB565)
    return sensor


# 绘制文字：兼容API绘制字符串
# 参数:
#   img: 图像对象
#   x, y: 文字位置
#   text: 文字内容
#   color: 文字颜色
def draw_label(img, x, y, text, color):
    try:
        img.draw_string_advanced(int(x), int(y), 14, text, color=color)
    except Exception:
        pass


# 绘制坐标轴箭头：从(x0,y0)到(x1,y1)画箭头线
# 参数:
#   img: 图像对象
#   x0, y0: 起点坐标
#   x1, y1: 终点坐标
#   color: 线条颜色
def draw_axis_arrow(img, x0, y0, x1, y1, color):
    # 绘制主轴线段
    img.draw_line(x0, y0, x1, y1, color=color, thickness=2)
    dx = x1 - x0
    dy = y1 - y0
    # 根据主轴方向绘制箭头
    if abs(dx) >= abs(dy):
        sx = 1 if dx >= 0 else -1
        img.draw_line(x1, y1, x1 - 8 * sx, y1 - 5, color=color, thickness=2)
        img.draw_line(x1, y1, x1 - 8 * sx, y1 + 5, color=color, thickness=2)
    else:
        sy = 1 if dy >= 0 else -1
        img.draw_line(x1, y1, x1 - 5, y1 - 8 * sy, color=color, thickness=2)
        img.draw_line(x1, y1, x1 + 5, y1 - 8 * sy, color=color, thickness=2)


# 绘制叠加层：坐标轴、目标点、误差信息、FPS等
# 参数:
#   img: 图像对象
#   fps: 帧率
def draw_overlay(img, fps):
    # 颜色定义
    green = (0, 255, 0)
    red = (255, 0, 0)
    cyan = (0, 255, 255)
    yellow = (255, 255, 0)
    white = (255, 255, 255)
    gray = (110, 110, 110)

    # 计算目标点坐标(原点 + 误差)
    target_x = ERROR_ORIGIN_X + TEST_ERR_Y
    target_y = ERROR_ORIGIN_Y - TEST_ERR_Z

    # 绘制原点标记
    img.draw_circle(ERROR_ORIGIN_X, ERROR_ORIGIN_Y, 5, color=green, thickness=2)
    img.draw_cross(ERROR_ORIGIN_X, ERROR_ORIGIN_Y, color=green, size=9, thickness=1)

    # 绘制坐标轴参考线(灰色)
    img.draw_line(ERROR_ORIGIN_X - AXIS_LEN, ERROR_ORIGIN_Y,
                  ERROR_ORIGIN_X + AXIS_LEN, ERROR_ORIGIN_Y,
                  color=gray, thickness=1)
    img.draw_line(ERROR_ORIGIN_X, ERROR_ORIGIN_Y - AXIS_LEN,
                  ERROR_ORIGIN_X, ERROR_ORIGIN_Y + AXIS_LEN,
                  color=gray, thickness=1)

    # 绘制带箭头的坐标轴
    draw_axis_arrow(img, ERROR_ORIGIN_X, ERROR_ORIGIN_Y,
                    ERROR_ORIGIN_X + AXIS_LEN, ERROR_ORIGIN_Y, cyan)
    draw_axis_arrow(img, ERROR_ORIGIN_X, ERROR_ORIGIN_Y,
                    ERROR_ORIGIN_X, ERROR_ORIGIN_Y - AXIS_LEN, yellow)

    # 绘制误差分解线(横纵两个分量 + 对角线)
    img.draw_line(ERROR_ORIGIN_X, ERROR_ORIGIN_Y,
                  target_x, ERROR_ORIGIN_Y, color=cyan, thickness=1)
    img.draw_line(target_x, ERROR_ORIGIN_Y,
                  target_x, target_y, color=yellow, thickness=1)
    img.draw_line(ERROR_ORIGIN_X, ERROR_ORIGIN_Y,
                  target_x, target_y, color=white, thickness=1)

    # 绘制目标点标记
    img.draw_circle(target_x, target_y, 5, color=red, thickness=2)
    img.draw_cross(target_x, target_y, color=red, size=7, thickness=1)

    # 绘制坐标标签
    draw_label(img, ERROR_ORIGIN_X + AXIS_LEN + 4,
               ERROR_ORIGIN_Y - 8, "+Y", cyan)
    draw_label(img, ERROR_ORIGIN_X - 10,
               ERROR_ORIGIN_Y - AXIS_LEN - 18, "+Z", yellow)
    draw_label(img, ERROR_ORIGIN_X + 6, ERROR_ORIGIN_Y + 6, "O", green)
    draw_label(img, target_x + 6, target_y - 18, "P(Y=+50,Z=+50)", red)

    # 绘制状态信息文字
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


# 全局硬件对象
sensor = None
uart = None
has_display = False  # 是否有显示设备

try:
    # ===== 硬件初始化 =====
    uart = init_uart()
    sensor = init_camera()

    # 初始化LCD显示
    if not HEADLESS:
        try:
            Display.init(Display.ST7701, width=800, height=480,
                         to_ide=SHOW_TO_IDE)
            has_display = True
        except Exception:
            has_display = False

    # 启动媒体系统和传感器
    MediaManager.init()
    sensor.run()
    clock = time.clock()
    gc_count = 0

    # ===== 主循环 =====
    while True:
        clock.tick()
        os.exitpoint()
        img = sensor.snapshot()
        fps = int(clock.fps() + 0.5)

        # 发送固定测试误差值
        send_tracking(uart, TEST_ERR_Y, TEST_ERR_Z)

        # LCD显示
        if has_display:
            draw_overlay(img, fps)
            Display.show_image(img, x=(800 - IMG_W) // 2,
                               y=(480 - IMG_H) // 2)

        # 定期垃圾回收
        gc_count += 1
        if gc_count >= 60:  # 每60帧执行一次GC
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
        MediaManager.deinit()
    except Exception:
        pass
    if uart:
        uart.deinit()
