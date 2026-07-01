# -*- coding: utf-8 -*-
# 视觉数学工具库：透视映射、坐标转换、数据打包等


# 透视映射：将四边形内归一化坐标(u,v)映射到实际图像坐标
# 参数:
#   corners: 四边形四个角点列表 [p0, p1, p2, p3]
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
    denominator = g * u + h * v + 1.0  # 透视分母，避免除零

    # 分母接近0时返回四边形中心作为保护
    if abs(denominator) < 1.0e-9:
        return ((x0 + x1 + x2 + x3) * 0.25,
                (y0 + y1 + y2 + y3) * 0.25)

    # 执行透视变换并返回结果
    return ((a * u + b * v + x0) / denominator,
            (d * u + e * v + y0) / denominator)


# 四边形中心：计算四边形中心点(归一化坐标0.5,0.5)
# 参数:
#   corners: 四个角点列表
# 返回值: (cx, cy) 中心点坐标
def quadrilateral_center(corners):
    return perspective_map_point(corners, 0.5, 0.5)


# 相对控制误差：计算指定归一化坐标相对于原点的误差
# 参数:
#   corners: 四个角点列表
#   u, v: 目标归一化坐标
#   origin_x, origin_y: 原点坐标
# 返回值: (err_x, err_y) 相对误差
def relative_control_error(corners, u, v, origin_x, origin_y):
    x, y = perspective_map_point(corners, u, v)
    return x - float(origin_x), y - float(origin_y)


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
