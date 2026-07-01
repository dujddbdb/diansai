# -*- coding: utf-8 -*-


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
    determinant = dx1 * dy2 - dx2 * dy1  # 行列式，判断是否退化为平行四边形

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
    denominator = g * u + h * v + 1.0  # 分母，避免除零

    # 分母接近零时返回四边形中心作为兜底
    if abs(denominator) < 1.0e-9:
        return ((x0 + x1 + x2 + x3) * 0.25,
                (y0 + y1 + y2 + y3) * 0.25)

    # 执行透视变换并返回结果
    return ((a * u + b * v + x0) / denominator,
            (d * u + e * v + y0) / denominator)


# 计算四边形中心点: 返回归一化坐标(0.5,0.5)对应的像素坐标
def quadrilateral_center(corners):
    return perspective_map_point(corners, 0.5, 0.5)


# 计算相对控制误差: 目标点相对于原点的像素偏移量
def relative_control_error(corners, u, v, origin_x, origin_y):
    # 先通过透视映射得到目标点像素坐标
    x, y = perspective_map_point(corners, u, v)
    # 返回相对于原点的误差(x方向和y方向)
    return x - float(origin_x), y - float(origin_y)


# 打包坐标数据包: 将err_y和err_z打包为4字节大端格式串口数据
def pack_coordinate_packet(err_y, err_z):
    # 截取低16位
    err_y &= 0xFFFF
    err_z &= 0xFFFF
    # 按大端序打包成4字节
    return bytes((err_y >> 8, err_y & 0xFF,
                  err_z >> 8, err_z & 0xFF))
