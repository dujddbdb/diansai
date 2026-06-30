# 2026-06-20 - 读取所有文档，采用最新逻辑

## 日期
- 2026-06-20

## 涉及文件
- `bsp/track_config.h` — 新增 RIGHT_ANGLE_PHASE2_TIMEOUT
- `bsp/track.c` — Phase2超时、陀螺仪首帧修复、除零保护
- `bsp/track.h` — (由formatter维护, gyro_poll_request声明等)
- `app/main.c` — (由formatter维护, 串口调试版本)
- `bsp/bno080.c` — I2C 400kHz、EXTI重入保护、Timer中断屏蔽

## 修改前状态
- 之前的恢复使代码编译一致，但缺少多个任务文档描述的逻辑改进
- 缺少来自 HEAD commit (2604351) 的安全修复
- 缺少 I2C 读包保护机制

## 综合所有任务文档的最终逻辑

### 1. 直角检测 Phase 2 超时 (来自 2604351 / code-bug-review)
- 新增 `right_angle_phase2_timer` 变量和 `RIGHT_ANGLE_PHASE2_TIMEOUT=500`
- Phase 2 进入时清零计时器，每1ms递增
- 超时后回退到 Phase 0，防止卡死在等待全白确认

### 2. 陀螺仪首帧修复 (来自 2604351)
- 首帧不再计算 rate/accel（避免用初始值0算出垃圾角速度）
- 首帧直接初始化 yaw_deg、target_yaw、last_yaw，yaw_rate和yaw_accel置0

### 3. Track_Calc_Err 除零保护 (来自 2604351)
- 当 `Calibrated_white[i] == 0` 时跳过该通道（`continue`），避免除零硬件异常

### 4. BNO080 I2C 时钟 400kHz (来自 2026-06-20-bno080-i2c-400khz)
- I2C1_Init 和 bsp_i2c_device_detect 中 `I2C_ClockSpeed` 从 350000 改为 400000
- 缩短单次 I2C 读包时间，减少对 5ms 速度环的阻塞

### 5. I2C 读包 EXTI 重入保护 (来自 2026-06-19-bno080-read-guard-exti-mask)
- 新增 `g_bno080_i2c_busy` 忙标志
- 新增 `BNO080_EXTI_SetEnabled()` 控制 EXTI5 中断使能
- `bno080_update()` 读包前：设 busy + 关 EXTI5 + 屏蔽 TIM11/TIM7
- `bno080_update()` 读包后：恢复 TIM11/TIM7 + 开 EXTI5 + 清 busy
- `EXTI9_5_IRQHandler()` 忙时仅清 pending，不修改数据就绪标志

### 6. I2C 期间屏蔽 TIM11+TIM7 更新中断 (来自 2026-06-19-bno080-delay-tim7-during-i2c / bno080-mask-tim11-only)
- 新增 `BNO080_TimerIrq_Lock()` / `BNO080_TimerIrq_Unlock()` 嵌套安全锁
- Lock: 关闭 TIM11 和 TIM7 的 `TIM_IT_Update` 中断使能（不停止计数器）
- Unlock: 重新使能，不清除 pending 位，保证 I2C 期间到点的中断补跑
- 只关闭中断使能位，不关闭定时器计数器，不影响速度闭环周期

### 7. 直角类型映射 (确认已正确，来自 fix-right-angle-type / final-right-angle-direction-mapping)
- 当前代码已正确：`11XXXX00` → type=1(右转), `00XXXX11` → type=2(左转)
- type=1 右转: yaw 目标 = start - 80° (yaw 减小)
- type=2 左转: yaw 目标 = start + 80° (yaw 增大, 符合实车 yaw 增大现象)
- GRAY_DIRECTION=0 保持（用户确认映射没问题）
- GYRO_YAW_DIRECTION=1 保持

## 当前架构总结

```
主循环 (低优先级):
  Grayscale_Task() → 灰度采集
  Track_Gyro_TakePollRequest() → bno080_update() → [I2C读包, 有Timer/EXTI保护]
  串口命令处理 + 调试打印

TIM11 ISR (1kHz, 最高优先级):
  Track_Gyro_PollTick() → gyro 读取请求计时(20ms)
  Track_Gray_Convert() → ir_raw[] 更新
  Track_Gyro_Update() → 消费 BNO080 数据, 更新 yaw/rate
  Track_Check_Right_Angle() → 直角状态机(Phase0→2→1→3)
  Track_Calc_Err() → 误差计算(带除零保护)
  Track_PID_Calc() → 位置PID
  Track_Action_Execute() → 直角:陀螺仪PID差速 / 直道:PID+阻尼

TIM7 ISR (200Hz):
  Encoder_ReadAll() → RPM计算 → 速度PI → PWM输出

TIM10: 禁用 (GYRO_TIM10_ENABLE=0 — 高优先级I2C导致疯转的根因)
```

## 关键参数 (track_config.h)

| 参数 | 值 | 来源 |
|------|-----|------|
| CORNER_YAW_TARGET | 80.0f | right-angle-gyro-pid-rewrite |
| KP_CORNER_YAW | 10.0f | right-angle-gyro-pid-rewrite |
| KD_CORNER_YAW | 5.0f | right-angle-gyro-pid-rewrite |
| CORNER_TURN_RPM | 80.0f | right-angle-gyro-pid-rewrite |
| GYRO_MAIN_POLL_PERIOD_MS | 20U | reduce-bno080-poll |
| KD_GYRO_STRAIGHT | 0.5f | enable-gyro-straight-damping |
| GYRO_STRAIGHT_LIMIT | 1.0f | enable-gyro-straight-damping |
| RIGHT_ANGLE_PHASE2_TIMEOUT | 500 | code-bug-review |
| I2C ClockSpeed | 400000 | bno080-i2c-400khz |

## 测试建议
- Keil MDK 编译验证
- 上电后串口应观察到周期性调试打印 (200ms)
- 直角测试: 观察 Phase 2 是否在500ms无全白时自动回退
- 观察 I2C 读包期间 TIM7/TIM11 中断是否正常补跑 (看 enc_acc_L/R 和 system_time_ms 连续性)
- 如果 BNO080 偶发停流，轻量恢复机制（I2C busy + EXTI guard）应能自动恢复
- 如果转弯不足/过冲: 调整 KP_CORNER_YAW / KD_CORNER_YAW / CORNER_YAW_TARGET
- 如果直线摇摆: 调整 KD_GYRO_STRAIGHT / GYRO_STRAIGHT_LIMIT
