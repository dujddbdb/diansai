# 紧凑直角通信、第二直角退出与云台防漂移

- 日期：2026-06-22

## 涉及文件

- `app/main.c`
- `bsp/track.c`
- `bsp/track_config.h`
- `bsp/uart_k230.c`
- `bsp/uart_k230.h`
- `../project_1.0 eye/app/main.c`
- `../project_1.0 eye/bsp/gimbal_pid.c`
- `../project_1.0 eye/bsp/gimbal_pid.h`
- `../project_1.0 eye/bsp/uart/uart5.c`
- `../project_1.0 eye/bsp/uart/uart5.h`
- `../project_1.0 eye/bsp/uart/car_corner_protocol.h`
- `../project_1.0 eye/bsp/uart_k230.c`
- `../project_1.0 eye/k230/main.py`
- `../project_1.0 eye/k230/vision_math.py`

## 修改前状态

- car 的陀螺仪直角执行分支没有硬超时，第二个直角若航向差未达到 90 度会永久停在 phase 2，LED 因此持续闪烁。
- car 到 eye 每帧 12 字节并携带 yaw、stale、seq；K230 到 eye 还携带置信度和状态，超出实际控制边界。
- eye 每 10 ms 对旧坐标重复发送相对定位命令，位置积分和残余输出可能持续累加为步进漂移。
- K230 默认开启 LCD/IDE 图传并逐帧 RGB 转灰度，浪费帧时间。

## 修改内容和原因

- car phase 2 增加 `CORNER_TURN_TIMEOUT_MS=1200` 硬退出；LED 改为 phase 2 常亮、其余熄灭。
- car 到 eye 改为固定三字节帧 `[A5][STATE][A5 XOR STATE]`。STATE 只包含 phase、左右方向和 car IMU 有效位；状态变化立即发送，转弯中每 50 ms 保活。
- K230 到 eye 使用 `AA 04 04 XH XL YH YL CS`，只传坐标；无可靠新测量发送 `-1,-1`。
- eye 仅在新坐标到达时生成一次步进相对位移；加入 2 像素死区、单帧角度上限、积分钳位并关闭位置积分。
- 云台本地 IMU 只在 car 报告直角且视觉暂时没有新帧时作温和增量稳定；视觉新帧优先，避免 IMU 抵消步进电机的主动运动。
- K230 默认无头灰度模式，关闭 LCD/IDE 图传，ROI 锁定时每 30 帧全图复扫；靶心继续使用四边形对角线交点。

## 测试建议

- 架空车轮连续触发两个直角，确认每次 phase 2 最迟 1.2 s 进入 phase 3，LED 不再无限闪烁。
- 逻辑分析仪检查 USART1→UART5：每帧严格 3 字节，115200 波特率下活动流量约 60 B/s。
- 云台静止对靶 30 秒，确认 2 像素内不再产生脉冲；快速转动车身时确认仅直角阶段启用 IMU 增量稳定。
- 上车后优先校准 `PIXEL_TO_ANGLE_FACTOR`，再调 `PID_KP_SMALL`；不要先恢复积分。
- 主机回归、Python 语法检查以及 car/eye Keil 全量编译均应为零失败、零警告。
