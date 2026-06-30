# 任务记录: 融合循迹 + 陀螺仪直角转弯 + 航向锁定

**日期**: 2025-06-18 (同一天第二次任务)  
**前置任务**: [2025-06-18 修复航向锁定方向+冷启动](2025-06-18-fix-yaw-direction-and-cold-boot-hang.md)  
**涉及文件**:
- `app/main.c` — 完全重写，融合所有模块
- `bsp/track.c` — system_time_ms 迁移到 TIM11，速度环PI变量去static
- `bsp/track.h` — 新增 extern 声明

---

## 改动概览

之前 `main.c` 只是一个纯航向锁定测试程序。现在融合了三个模块:
1. **灰度循迹** — 位置PID + 陀螺仪航向补偿
2. **直角检测** — 灰度传感器检测入口 → 陀螺仪偏航角PID执行转弯
3. **航向锁定** — 串口命令切换，原地旋转锁定朝向

## 详细修改

### 1. `app/main.c` — 重写 (~200行 → ~270行)

**初始化新增**:
- `NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2)` — TIM11 ISR 依赖
- `Grayscale_Init()` + `Grayscale_InitCalibrate()` + ir_weight 计算
- 所有循迹状态变量清零 (直角检测、陀螺仪补偿、全局锁定等)
- 速度环PI内部状态清零 (Left_Integral 等)
- 启动 TIM11 (1kHz 主控制环)

**主循环新增**:
```c
Grayscale_Task(&gray_sensor);  // 更新灰度ADC (TIM11 ISR消费)
```

**串口命令扩展**:
| 命令 | 功能 |
|------|------|
| `hold\r\n` | 进入航向锁定模式 |
| `track\r\n` | 退出航向锁定，恢复循迹 |
| `stop\r\n` | 停转 (同时停止循迹和航向锁定) |
| `go\r\n` | 恢复运行 |
| `+90\r\n` | 角度增量 (自动进入航向锁定) |

**模式逻辑**:
- 默认 = 循迹模式 (TIM11 ISR 自动运行灰度→直角检测→位置PID→动作执行)
- 发送 `hold` 或 `+90` → 切换到航向锁定 (yaw_hold_mode=1)
- 航向锁定时 TIM11 ISR 中的 `Track_Check_Right_Angle()`、`Track_PID_Calc()`、`Track_Action_Execute()` 检查 `yaw_hold_mode` 自动跳过
- 发送 `track` → 退出航向锁定 (yaw_hold_mode=0)

**保留之前的修复**:
- BNO080 冷启动重试 (5次×500ms)
- 陀螺仪等待3秒超时
- 主循环每2秒自动重试 BNO080

### 2. `bsp/track.c` — 两处修改

**TIM7 ISR** (`track.c:818`):
```c
// 之前: system_time_ms += 5;   /* TIM7=5ms/次, 替代TIM11的1ms累加 */
// 现在: /* system_time_ms 由 TIM11 ISR 每1ms递增, TIM7不再累加 */
```
TIM11 启动后由它提供1ms精度的 system_time_ms，TIM7 不再累加（否则重复计数时间会飞）。

**速度环PI变量去static** (`track.c:99-104`):
```c
// 之前: static float Left_Integral ...
// 现在: float Left_Integral ...  (全局可见)
```
main.c 初始化时需要清零这些变量，去static后可以 extern 引用。

### 3. `bsp/track.h` — 新增 extern

```c
extern float Left_Integral, Right_Integral;
extern float Left_FilteredEnc, Right_FilteredEnc;
extern float Left_LastBias, Right_LastBias;
extern int32_t err_buffer[3];
```

---

## 数据流 (运行时)

```
主循环 Grayscale_Task()  ~500Hz+
    ↓ 更新 gray_sensor.Analog_value / .Digtal
TIM11 ISR (1kHz):
  1. Track_Gray_Convert()      → gray_sensor.Digtal → ir_raw[8]
  2. Track_Gyro_Update()       → BNO080数据 → gyro_yaw_deg
  3. Track_Check_Right_Angle() → ir_raw → 直角状态机
  4. Track_Calc_Err()          → gray_sensor.Analog → 位置误差
  5. Track_PID_Calc(err)       → 位置PID → PID_Output
  6. Track_Action_Execute()    → Left_Base_RPM / Right_Base_RPM
    ↓
TIM7 ISR (200Hz):
  编码器 → 速度PI → PWM → 电机
```

当 `yaw_hold_mode=1` 时，步骤3/5/6 自动跳过，电机转速由主循环的 `Track_YawHold_Update()` 控制。

---

## 测试建议

1. **编译**: CubeIDE 导入编译，确认无链接错误 (特别注意 extern 符号)
2. **循迹**: 放赛道上，默认应正常循迹，串口打印 `[TRACK] ...`
3. **直角**: 接近直角 → 应自动检测 → 陀螺仪转弯 → 恢复循迹
4. **航向锁定**: 发 `hold\r\n` → 原地锁定；`+90\r\n` → 转90°；`track\r\n` → 恢复循迹
5. **冷启动**: 断电重上电 → 串口应有 BNO080 初始化过程 → 自动进入循迹

## 注意事项

- 航向锁定方向上次已修复 (Left=-correction, Right=+correction)
- 如果直角转弯方向反了 → 检查 `GYRO_YAW_DIRECTION` (track_config.h:35)
- 如果直角角度不准 → 调整 `CORNER_YAW_TARGET` (默认85°) 和 `KP_CORNER_YAW` / `KD_CORNER_YAW`
- system_time_ms 现在由 TIM11 提供 1ms 精度，所有超时判断更精确
