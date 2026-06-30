# 开启直线陀螺仪阻尼

**日期**: 2026-06-19

## 涉及文件
- `bsp/track_config.h`
- `bsp/track.c`

## 修改前状态
- `GYRO_CONTROL_ENABLE=1` 时直线阻尼代码已有，但没有独立开关，直线阻尼与整体陀螺仪控制绑定。
- 用户要求开启直线阻尼，用陀螺仪角速度抑制正常循迹左右摇摆。

## 修改内容和原因
- 新增 `GYRO_STRAIGHT_DAMPING_ENABLE=1`，用于单独控制正常循迹阶段的陀螺仪角速度阻尼。
- 正常循迹分支的 `gyro_correction` 只在 `GYRO_STRAIGHT_DAMPING_ENABLE=1` 时叠加到灰度 PID 输出。
- 保持直角阶段逻辑不受该开关影响，直角仍由 `GYRO_CONTROL_ENABLE` 和 yaw 目标控制。

## 当前参数
- `KD_GYRO_STRAIGHT=0.2f`
- `GYRO_STRAIGHT_LIMIT=3.0f`
- `KP_YAW_ACCEL_FF=0.0f`

## 测试建议
- 下载后先跑直线，观察是否比纯灰度更少左右摆。
- 若仍摆动明显，优先小幅增大 `KD_GYRO_STRAIGHT`；若变迟钝或反向摆，减小该值或检查 `GYRO_YAW_DIRECTION`。
- 若输出过猛，降低 `GYRO_STRAIGHT_LIMIT`；若阻尼不够，再提高限幅。

## 编译验证
- Keil MDK 构建通过：`0 Error(s), 0 Warning(s)`。
