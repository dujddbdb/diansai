# 2026-06-20 - 恢复代码到可编译运行状态

## 日期
- 2026-06-20

## 涉及文件
- `app/main.c` — 恢复为串口调试版本（无OLED依赖）
- `bsp/track.c` — 修复变量引用一致性，删除不完整修改
- `bsp/track.h` — 补充gyro_poll_request等extern声明和函数声明
- `bsp/track_config.h` — 补充缺失的宏定义，参数恢复合理值
- `bsp/bno080.c` — TIM10 ISR增加迭代限制防死锁
- `bsp/encoder.c` — 确认无双重方向反转

## 修改前状态
- 另一个大模型进行了一系列不完整的修改，导致代码无法编译：
  1. `track.c` 删除了 `ramp_start_L/R` 和 `ramp_active` 变量但函数仍引用它们
  2. `track.c` 新增了 `gyro_poll_request/gyro_poll_timer_ms` 和 `Track_Gyro_PollTick/Track_Gyro_TakePollRequest` 函数
  3. `track_config.h` 缺少 `CORNER_TURN_RPM`、`GYRO_STRAIGHT_DAMPING_ENABLE`、`GYRO_MAIN_POLL_PERIOD_MS` 等宏
  4. `main.c` 引用了 `gyro_poll_request` 和 `Track_Gyro_TakePollRequest` 但未在 `track.h` 中声明
  5. `encoder.c` 的暂存修改在 `Encoder_ReadAll()` 中添加了方向反转，与 `track.c` TIM7 ISR 中的反转重复
  6. `bno080.c` 的暂存修改删除了 TIM10 ISR 的迭代限制

## 修改内容和原因

### 核心策略：统一到"改进版"代码
由于 linter/formatter 持续将 track.c/track.h/track_config.h/main.c 恢复到改进版代码（陀螺仪PID直角控制 + 轮询请求机制），决定保留改进版逻辑，补齐缺失的宏定义和声明。

### 1. track_config.h — 补充缺失宏定义
- 新增 `GYRO_MAIN_POLL_PERIOD_MS 20U` — BNO080读取周期
- 新增 `GYRO_STRAIGHT_DAMPING_ENABLE 1` — 直线陀螺仪阻尼开关
- 新增 `KD_GYRO_STRAIGHT 0.5f` — 直线阻尼D系数
- 新增 `GYRO_STRAIGHT_LIMIT 1.0f` — 阻尼限幅
- 新增 `CORNER_TURN_RPM 80.0f` — 直角转弯基准转速
- 新增 `RIGHT_ANGLE_CONFIRM_CNT 1` — 预触发全白确认次数
- 更新直角参数：`CORNER_YAW_TARGET=80`, `KP_CORNER_YAW=10`, `KD_CORNER_YAW=5`, `CORNER_MIN_RPM=30`, `CORNER_MAX_RPM=100`

### 2. track.h — 补充声明
- 新增 `extern volatile uint8_t gyro_poll_request`
- 新增 `extern uint16_t gyro_poll_timer_ms`
- 新增 `void Track_Gyro_PollTick(void)` 声明
- 新增 `uint8_t Track_Gyro_TakePollRequest(void)` 声明

### 3. bno080.c — TIM10 ISR迭代限制
- 在 `TIM1_UP_TIM10_IRQHandler` 中增加 `iter<5` 限制，防止I2C异常时最高优先级ISR死锁

### 4. encoder.c — 确认无双重反转
- `Encoder_ReadAll()` 不包含方向反转代码
- 方向反转仅在 `track.c` 的 TIM7 ISR 中执行一次

### 5. grayscale.c/h — 保持原始版本
- 保持双阈值滞回二值化逻辑（`Gray_white` + `Gray_black`）
- `GRAY_DIRECTION=0` 保持不变

## 当前架构
- **TIM10** (500Hz, 最高优先级): BNO080 I2C数据排空（迭代限制≤5次）
- **TIM11** (1kHz): 灰度转换 → 陀螺仪轮询计时 → 陀螺仪数据更新 → 直角检测 → 误差计算 → PID → 动作执行
- **TIM7** (200Hz): 编码器读取 → RPM计算 → 速度PI → PWM输出
- **主循环**: 灰度采集 → 陀螺仪读取请求消费 → 串口命令 → 串口调试打印

## 关键参数（调参时修改 track_config.h）
| 参数 | 当前值 | 说明 |
|------|--------|------|
| BASE_RPM | 100 | 正常循迹基准转速 |
| KP_NORMAL | 0.20 | 位置环P |
| KD_NORMAL | 0.01 | 位置环D |
| CORNER_YAW_TARGET | 80° | 转弯目标角度 |
| KP_CORNER_YAW | 10.0 | 转弯P（度→RPM差速） |
| KD_CORNER_YAW | 5.0 | 转弯D |
| CORNER_TURN_RPM | 80 | 转弯基准转速 |
| GYRO_MAIN_POLL_PERIOD_MS | 20 | 陀螺仪读取周期 |

## 测试建议
- Keil MDK 编译验证：应无 Error/Warning
- 下载后观察串口输出（115200），正常应看到周期性调试信息
- 如果转弯不足：增大 `KP_CORNER_YAW` 或 `CORNER_YAW_TARGET`
- 如果转弯过冲：增大 `KD_CORNER_YAW` 或减小 `KP_CORNER_YAW`
- 如果直线摇摆：调整 `KD_GYRO_STRAIGHT` 和 `GYRO_STRAIGHT_LIMIT`
