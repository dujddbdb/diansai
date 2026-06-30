# 连续直角、视觉状态联动与双 IMU 稳定

- 日期：2026-06-22
- 涉及工程：`project_1.0 car`、`project_1.0 eye`、`project_1.0 eye/k230`

## 修改前

- car 进入陀螺仪直角阶段时差速可直接跳到约 50 RPM，phase 3 同一周期立即清零。
- BNO08X I2C 单次等待上限 100 ms，car 读取期间还会屏蔽 TIM11/TIM7。
- eye 同时叠加 car 累计 yaw 像素补偿和云台 IMU；单次 `yaw_delta` 会被重复消费。
- UART5 依赖 IDLE 间隔和单完整帧，噪声或连续帧会丢包。
- K230 使用四角平均中心，保持坐标与真实检测在协议中无法区分。

## 修改内容与原因

- 新增 `bsp/corner_profile.h`，提供五次 smoothstep 和限斜率函数。
- car phase 1 预减速；phase 2 对基础速度、陀螺仪差速和左右轮目标限斜率；末段平滑引入灰度 PID；phase 3 保持 120 ms 平滑退出并通过 USART1 持续上报。
- car 和 eye 的 BNO08X I2C 运行时超时降至 5 ms，并识别 executable reset-complete 包；car 不再为 I2C 屏蔽车轮控制定时器。模块没有独立 NRST，因此恢复链为 report 重启、I2C 解锁、软件重初始化。
- eye UART5 改为逐字节流式解析，可从噪声、截断和连续帧重新同步。
- eye 仅使用云台本地 IMU 做单次增量前馈；car IMU 只负责车体转角和阶段信息，避免双重补偿。
- K230 使用四边形对角线交点，发送 `CMD=0x05` 的坐标、置信度和 fresh/predicted 状态；预测帧不能刷新真实目标时限或触发激光。

## 主要文件

- `bsp/track.c`
- `bsp/track_config.h`
- `bsp/corner_profile.h`
- `bsp/bno080.c`
- `../project_1.0 eye/app/main.c`
- `../project_1.0 eye/bsp/gimbal_pid.c`
- `../project_1.0 eye/bsp/uart/uart5.c`
- `../project_1.0 eye/bsp/uart/car_corner_protocol.h`
- `../project_1.0 eye/k230/main.py`
- `../project_1.0 eye/k230/vision_math.py`

## 验证与实车建议

- Keil 全量重建：car、eye 均 0 error / 0 warning。
- 主机测试覆盖：差速限斜率、IMU 增量单次消费、UART5 噪声重同步、透视中心、视觉质量协议、BNO 运行策略。
- 首次实车先架空驱动轮，确认 type=1 右转、type=2 左转方向没有反向。
- 依次调 `CORNER_DIFF_SLEW_UP_RPM_PER_MS`、`CORNER_ENTRY_BLEND_MS`、`CORNER_LINE_REACQUIRE_START`，不要先提高 yaw Kp。
- 示波器/逻辑分析仪检查 INT、SCL、SDA；若仍断流，检查 2.2k~4.7k 上拉、模块近端去耦及电机电源干扰。
