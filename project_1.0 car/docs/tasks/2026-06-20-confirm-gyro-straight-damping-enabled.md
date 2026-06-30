# 2026-06-20 - 确认开启直线陀螺仪阻尼

## 日期
- 2026-06-20

## 涉及文件
- `bsp/track_config.h`
- `bsp/track.c`

## 修改前状态
- 用户要求开启直线阻尼。
- 检查当前配置发现直线阻尼已经处于开启状态。

## 修改内容和原因
- 未改动控制参数，确认 `GYRO_STRAIGHT_DAMPING_ENABLE = 1`。
- 当前直线阻尼参数为 `KD_GYRO_STRAIGHT = 0.5f`、`GYRO_STRAIGHT_LIMIT = 1.0f`。
- `track.c` 正常循迹分支中已经把 `gyro_correction` 叠加到灰度 PID 输出。

## 测试建议
- 下载后观察直线是否减少左右摆动。
- 如果仍快速摆动，可继续增大 `KD_GYRO_STRAIGHT`。
- 如果车变钝、压线慢，可减小 `KD_GYRO_STRAIGHT` 或增大/调整 `GYRO_STRAIGHT_LIMIT`。

## 编译验证
- Keil MDK 全量 Rebuild：`0 Error(s), 1 Warning(s)`。
- 剩余 1 个 warning 为 `track.c` 末尾换行提示；文件字节已确认以 CRLF 结尾，功能不受影响。
