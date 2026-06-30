# 最小恢复直角状态机

**日期**: 2026-06-19

## 涉及文件
- `bsp/track_config.h`

## 修改前状态
- 纯灰度循迹稳定。
- BNO080 读取已改为主循环低优先级轮询，`GYRO_TIM10_ENABLE=0`，避免 I2C 在高优先级中断中卡住。
- 直线陀螺仪阻尼已开启且实车反馈“不摇了”。
- `RIGHT_ANGLE_ENABLE=0`，直角状态机仍处于隔离关闭状态。

## 修改内容和原因
- 将 `RIGHT_ANGLE_ENABLE` 从 `0` 改为 `1`，只恢复直角状态机。
- 保持 `GYRO_CONTROL_ENABLE=1`、`GYRO_TIM10_ENABLE=0`、`GYRO_MAIN_POLL_ENABLE=1`、`KD_GYRO_STRAIGHT=0.2f`、`GYRO_STRAIGHT_LIMIT=5.0f` 不变。
- 未修改灰度映射、灰度阈值、直角方向判定和直角 PID 参数。
- 当前逻辑中 `00001111` 会进入 `type=2`，`type=2` 的直角目标 yaw 为起始 yaw 加 `CORNER_YAW_TARGET`，符合“左转航向角增大”的实车方向结论。

## 编译验证
- Keil MDK 重新构建通过：`0 Error(s), 2 Warning(s)`。
- 剩余警告为既有非阻塞项：`track.c` 注释触发 trigraph warning、`bno080.c` 文件末尾无换行。

## 测试建议
- 先低速只测一个左直角和一个右直角，观察串口 `d/ph/t/yaw/rt`。
- 直角入口应看到 `ph: 0 -> 2`，全白确认后 `ph: 2 -> 1`，转够角度后退出到 `ph: 3 -> 0`。
- 若 `00001111` 左侧黑线触发后仍向右转，优先记录当时 `t`、`yaw` 增减和左右轮转速，再调整直角 type 到 yaw 目标的映射，不改灰度映射。
