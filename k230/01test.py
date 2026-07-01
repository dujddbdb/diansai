# -*- coding: utf-8 -*-
# K230矩形检测测试：摄像头采集→Blob检测→矩形验证→中心点计算→LCD显示

import time
import os
import sys

from media.sensor import *
from media.display import *
from media.media import *


# 全局变量
sensor = None
COS_THRESHOLD = 0.5  # 余弦阈值: |cos(θ)|<=0.5视为直角(允许60°~120°)，调大则角度要求更宽松


# 计算四边形中心：四个角点坐标的平均值
# 参数:
#   corner: 四个角点列表
# 返回值: (center_x, center_y) 中心点坐标
def get_center(corner):
    center_x = (corner[0][0] + corner[1][0] + corner[2][0] + corner[3][0]) / 4
    center_y = (corner[0][1] + corner[1][1] + corner[2][1] + corner[3][1]) / 4
    return center_x, center_y


# 矩形验证：用余弦定理判断角0和角2是否为直角，都通过则返回1
# 参数:
#   corner: 四个角点列表
#   threshold: 余弦阈值
# 返回值: 1表示是矩形，0表示不是
def calculate_angle(corner, threshold=COS_THRESHOLD):
    # 提取四个角点坐标
    x0, y0 = corner[0][0], corner[0][1]
    x1, y1 = corner[1][0], corner[1][1]
    x2, y2 = corner[2][0], corner[2][1]
    x3, y3 = corner[3][0], corner[3][1]

    t_sq = threshold * threshold  # 阈值平方，避免开方

    # 检测以(px,py)为顶点的角是否为直角
    # 用向量点积判断: |cosθ| <= threshold 则认为接近直角
    def check(px, py, xa, ya, xb, yb):
        v1x, v1y = xa - px, ya - py  # 边1向量
        v2x, v2y = xb - px, yb - py  # 边2向量
        l1 = v1x * v1x + v1y * v1y   # 边1长度平方
        l2 = v2x * v2x + v2y * v2y   # 边2长度平方
        if l1 == 0 or l2 == 0:  # 顶点重合，退化为直线
            return True
        d = v1x * v2x + v1y * v2y  # 向量点积
        return d * d > t_sq * l1 * l2  # |cos(θ)| > threshold 则不是直角

    # 检查角0: 以corner[0]为顶点
    if check(x0, y0, x1, y1, x3, y3):
        return 0
    # 检查角2: 以corner[2]为顶点
    if check(x2, y2, x1, y1, x3, y3):
        return 0
    return 1


try:
    # ===== 硬件初始化 =====
    sensor = Sensor(width=320, height=240)
    sensor.reset()
    sensor.set_framesize(width=320, height=240)
    sensor.set_pixformat(Sensor.RGB565)

    # LCD显示初始化
    Display.init(Display.ST7701, width=800, height=480, to_ide=False)

    # 媒体系统启动
    MediaManager.init()
    sensor.run()
    clock = time.clock()

    # 初始搜索ROI
    roi_x, roi_y, roi_w, roi_h = 20, 20, 300, 300

    # ===== 主循环 =====
    while True:
        clock.tick()
        os.exitpoint()
        img = sensor.snapshot()
        # 转换为灰度图
        gray = img.to_grayscale(copy=True)

        # Blob检测：查找灰度0~20的黑色区域
        blobs = gray.find_blobs(
            [(0, 20)],
            pixels_threshold=100,
            area_threshold=100,
            merge=True,
            margin=10,
            roi=(20, 20, 310, 230),
            x_stride=5,
            y_stride=5
        )

        # 遍历所有Blob，绘制外接矩形和四个角点
        for blob in blobs:
            x, y, w, h = blob.rect()
            img.draw_rectangle(x, y, w, h, color=(255, 255, 255))

            # 四个角点标记(不同颜色)
            img.draw_circle(x, y, 3, color=(255, 0, 0))        # 左上红
            img.draw_circle(x + w, y, 3, color=(0, 255, 0))    # 右上绿
            img.draw_circle(x + w, y + h, 3, color=(0, 0, 255))# 右下蓝
            img.draw_circle(x, y + h, 3, color=(255, 255, 0))  # 左下黄

            # 自适应ROI：扩展10像素边距，下一帧在更小区域内检测
            roi_x = x - 10
            roi_y = y - 10
            roi_w = w + 20
            roi_h = h + 20

        # 矩形检测：在ROI内查找矩形(更精确的检测)
        rects = gray.find_rects(roi=(roi_x, roi_y, roi_w, roi_h))

        # 遍历检测到的矩形，验证并绘制
        for rect in rects:
            corner = rect.corners()
            is_rect = calculate_angle(corner)

            if is_rect == 1:
                # 绘制矩形四条边(红色)
                img.draw_line(corner[0][0], corner[0][1],
                              corner[1][0], corner[1][1], color=(255, 0, 0))
                img.draw_line(corner[1][0], corner[1][1],
                              corner[2][0], corner[2][1], color=(255, 0, 0))
                img.draw_line(corner[2][0], corner[2][1],
                              corner[3][0], corner[3][1], color=(255, 0, 0))
                img.draw_line(corner[3][0], corner[3][1],
                              corner[0][0], corner[0][1], color=(255, 0, 0))

                # 绘制中心点(白色)
                center_x, center_y = get_center(corner)
                img.draw_circle(int(center_x), int(center_y), 5,
                                color=(255, 255, 255))

        # 显示FPS
        img.draw_string_advanced(0, 0, 20,
                                 "FPS:{}".format(clock.fps()),
                                 color=(128, 0, 0))

        # LCD居中显示
        Display.show_image(img, x=(800 - 320) // 2, y=(480 - 240) // 2)


except KeyboardInterrupt as e:
    pass

except BaseException as e:
    print(f"err: {e}")

finally:
    # ===== 资源清理 =====
    if isinstance(sensor, Sensor):
        sensor.stop()
    try:
        Display.deinit()
    except Exception:
        pass
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()