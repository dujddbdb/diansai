# 任务记录: 修复直线循迹方向纠偏失效

**日期**: 2025-06-19  
**涉及文件**:
- `app/main.c` — 灰度校准初始化 (之前遗漏核心字段)
- `bsp/track.c` — `Track_Calc_Err()` 改用 Normal_value

---

## 问题现象

直线循迹时，小车无法进行有效的方向纠偏 — 无论实际偏离黑线多远，两轮始终保持相同转速直行，不会自动修正方向。

## 根因分析

### Bug 1 (主要): `Calibrated_white[]` 未初始化

`main.c:26-33` 的灰度初始化代码手动设置了 `Gray_white[]` 和 `Gray_black[]`（用于二值化阈值），但遗漏了 `Calibrated_white[]` 和 `Calibrated_black[]` — 这是 `Track_Calc_Err()` 计算位置误差的核心依赖。

```c
// main.c 原代码 — 只设置了二值化阈值，漏了err计算用的基准值
for (j = 0; j < 8; j++) {
    gray_sensor.Gray_white[j] = cons_W[j];      // ✓ 二值化用
    gray_sensor.Gray_black[j] = cons_B[j];      // ✓ 二值化用
    // ✗ Calibrated_white[j] = ?  未设置！BSS段默认为 0
    // ✗ Calibrated_black[j] = ?  未设置！
    // ✗ Normal_factor[j] = ?     未设置！
    // ✗ ok = 1                   未设置！
}
```

由于 `gray_sensor` 是全局变量（BSS段），`Calibrated_white[8]` 全部为 0。

在 `Track_Calc_Err()` 中（修复前）:
```c
int32_t ref = (int32_t)gray_sensor.Calibrated_white[i];  // = 0
if (ref == 0) continue;  // ← 全部8路被跳过！
```

**全部8个通道因 `ref==0` 被 `continue` 跳过 → `sum_b` 恒为 0 → 函数永远返回 `last_valid_err` (0) → `PID_Err` 恒为 0 → `PID_Output` 恒为 0 → 方向纠偏完全失效。**

### Bug 2 (次要): `Track_Calc_Err()` 使用错误的黑度计算公式

修复前的公式: `black = 10000 - (ADC * 10000 / Calibrated_white)` — 只用白基准，忽略黑基准。灵敏度低的通道（黑 ADC 值大）的黑度被严重低估。

## 修复内容

### 修复 1: main.c — 使用 `Grayscale_InitCalibrate()` 完整初始化

```c
// 修复后: 一行调用替代所有手动赋值
Grayscale_InitCalibrate(&gray_sensor, cons_W, cons_B);
// 一次性完成: Calibrated_white/black, Gray_white/black, Normal_factor, ok=1
```

### 修复 2: track.c — `Track_Calc_Err()` 改用 `Normal_value`

```c
// 修复前: 用 Analog_value + Calibrated_white 自算（只考虑白基准）
int32_t pct = (adc * 10000) / ref;
int32_t black = 10000 - pct;

// 修复后: 直接用 Gray_Normalize() 正确归一化的 Normal_value
//   Normal_value = (ADC - black_ref) * 4096 / (white_ref - black_ref)
//   各通道灵敏度已均一化，纯黑=0，纯白=4096
int32_t black = 4096 - (int32_t)gray_sensor.Normal_value[i];
```

优势:
- 使用了黑白双基准的完整归一化公式
- 各通道灵敏度差异被 Normal_factor 自动补偿
- `ir_weight` 使用固定对称值即可（无需灵敏度调整）
- 代码更简洁，无除零风险（不再依赖 `ref==0` 检查）

## 数据流 (修复后)

```
main循环: Grayscale_Task()
  → Gray_ReadAllCh()    → Analog_value[8] (原始ADC)
  → Gray_AnalogToDigital() → Digtal (二值化, ir_raw用)
  → Gray_Normalize()    → Normal_value[8] (0~4096, 已归一化)

TIM11 ISR (1ms):
  Track_Gray_Convert()  → ir_raw (数字量, 直角检测用)
  Track_Gyro_Update()   → gyro_yaw_deg
  Track_Check_Right_Angle()
  Track_Calc_Err()      → Normal_value → blackness → 加权质心 err
  Track_PID_Calc(err)   → PID_Output (RPM*10)
  Track_Action_Execute() → Left/Right_Base_RPM

TIM7 ISR (5ms):
  编码器 → 速度PI → PWM → 电机
```

## 测试建议

1. **编译下载**: Keil MDK 中重新编译整个工程，下载到小车
2. **上电循迹**: 将小车放在赛道黑线上，观察是否能自动沿黑线行驶
   - 车偏离黑线时应能看到明显的差速纠正（一轮加速、一轮减速）
3. **串口调参**: 如果纠正力度不够 → 增大 `KP_NORMAL` (track_config.h:15)
   - 如果纠正过猛（振荡） → 减小 `KP_NORMAL` 或增大 `PID_FILTER_ALPHA`
4. **校准优化** (后续): 当前 `cons_W/cons_B` 为旧校准值，建议根据实测更新
   - 参考 `.trae/specs/gray-calibration/spec.md` 中的实测数据

## 注意事项

- `Grayscale_InitCalibrate()` 内部调用 `Grayscale_InitFirst()` 会 `memset` 清零所有校准数组，然后再写入 — 这是正常行为
- `Normal_value` 由主循环的 `Grayscale_Task()` 异步更新，TIM11 ISR 只读取 — 存在微小的竞态窗口（1ms级别），不影响功能
- 如果更换传感器或赛道，需重新标定 `cons_W/cons_B` 值
