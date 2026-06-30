# 翻转陀螺仪 yaw 方向修正直角转向反向

**日期**: 2026-06-19

## 涉及文件
- `bsp/track_config.h`

## 修改前状态
- 实车现象：检测到左直角后，小车向右转。
- 代码中 `is_right_angle=2` 表示左直角，并在陀螺仪模式下设置 `gyro_corner_target_yaw = start + CORNER_YAW_TARGET`。
- 如果该目标在实车上产生右转，说明 BNO080 yaw 正方向与代码假设相反。

## 修改内容和原因
- 将 `GYRO_YAW_DIRECTION` 从 `1` 改为 `-1`。
- 该参数会在 `Track_Gyro_Update()` 中统一翻转 `gyro_yaw_deg` 和 `gyro_yaw_rate`，从根上修正直角 yaw 目标方向。
- 未交换左右直角检测类型，避免把灰度检测语义改乱。

## 测试建议
- 串口看 `t/is_right_angle`：左直角应仍显示 `t2`，右直角应显示 `t1`。
- 实车左直角应向左转，右直角应向右转。
- 手动轻转车体时，若左转后串口 `yaw` 增大，则当前 `GYRO_YAW_DIRECTION=-1` 与代码假设一致。
