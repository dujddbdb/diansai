# -*- coding: utf-8 -*-


# 透视映射: 将四边形内归一化坐标(u,v)映射到实际图像坐标
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
    determinant = dx1 * dy2 - dx2 * dy1  # 行列式，判断是否退化为平行四边形

    if abs(determinant) < 1.0e-9:
        g = h = 0.0
    else:
        g = (sx * dy2 - dx2 * sy) / determinant
        h = (dx1 * sy - sx * dy1) / determinant

    a = x1 - x0 + g * x1
    b = x3 - x0 + h * x3
    d = y1 - y0 + g * y1
    e = y3 - y0 + h * y3
    denominator = g * u + h * v + 1.0  # 分母，避免除零

    if abs(denominator) < 1.0e-9:
        return ((x0 + x1 + x2 + x3) * 0.25,
                (y0 + y1 + y2 + y3) * 0.25)

    return ((a * u + b * v + x0) / denominator,
            (d * u + e * v + y0) / denominator)


# 四边形中心: 计算四边形中心点(归一化坐标0.5,0.5)
def quadrilateral_center(corners):
    return perspective_map_point(corners, 0.5, 0.5)


# 相对控制误差: 计算指定归一化坐标相对于原点的误差
def relative_control_error(corners, u, v, origin_x, origin_y):
    x, y = perspective_map_point(corners, u, v)
    return x - float(origin_x), y - float(origin_y)


# 打包坐标数据: err_y和err_z打包为4字节大端格式
def pack_coordinate_packet(err_y, err_z):
    err_y &= 0xFFFF
    err_z &= 0xFFFF
    return bytes((err_y >> 8, err_y & 0xFF,
                  err_z >> 8, err_z & 0xFF))
