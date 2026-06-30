# 任务记录: BNO080陀螺仪融合 — 纯角速率阻尼直线 + 减速区直角转弯

**日期**: 2025-06-19 (当天两次迭代)
**前置任务**: [2025-06-19 修复Keil编译错误](2025-06-19-fix-keil-build-errors.md)

## 涉及文件
- `bsp/track_config.h` — 新增 GYRO_ENABLE=1, KD_GYRO_STRAIGHT, KP_YAW_ACCEL_FF, 直角减速区参数; **移除** KP_GYRO_STRAIGHT(航向P), GYRO_TARGET_FILTER(目标角跟踪)
- `bsp/track.c` — 全文件 #if GYRO_ENABLE 守卫, 直线改为纯角速率阻尼, 直角保留减速区
- `bsp/track.h` — 移除 gyro_target_yaw extern, 新增 gyro_yaw_accel/rate/first_valid 声明
- `app/main.c` — 新增 BNO080 初始化、TIM10启动、陀螺仪恢复和诊断输出

## 设计哲学 (经过修正)

### 初始设计 vs 最终设计

| | 初始 (已被修正) | 最终 |
|---|---|---|
| 直线循迹 | PDG: P(航向角偏差) + D(角速率) = **锁航向** | 纯阻尼: D(角速率) + FF(角加速度) = **不定航向** |
| 问题 | P项锁航向角会**抵抗过弯**, 小车无法自然转弯 | 灰度PID负责位置/过弯, 陀螺仪只管消除振荡 |
| 直角转弯 | P(角度) + D(角速率) + 减速区 | 不变 — BNO080全权控制 (直角需要目标角) |

### 最终设计核心思想
```
灰度传感器管"线在哪"      → PID位置纠偏
陀螺仪只管"转多快"        → 角速率D阻尼 + 角加速度前馈
不定航向角                 → 小车自然跟随灰度PID过弯, 不被陀螺仪拉回
```

## 修改内容

### 1. track_config.h — 移除航向锁, 保留纯阻尼

```c
// 移除 (航向角锁定 — 会抵抗过弯):
// #define KP_GYRO_STRAIGHT   0.08f    // yaw角度P (度→RPM)
// #define GYRO_TARGET_FILTER 0.005f   // 目标yaw低通跟踪

// 保留 (纯速率阻尼):
#define KD_GYRO_STRAIGHT   0.20f   // 角速率D: yaw_rate→RPM阻尼 (deg/ms)
#define KP_YAW_ACCEL_FF    0.08f   // 角加速度前馈: yaw_accel→RPM (deg/ms²)
```

### 2. track.c Track_Action_Execute() — 直线纯阻尼

```c
// 修正后:
gyro_correction = -KD_GYRO_STRAIGHT * gyro_yaw_rate    // 角速率越大, 反向阻尼越强
                + KP_YAW_ACCEL_FF  * gyro_yaw_accel;   // 预判抽动趋势, 提前抵消

// 修正前:
// gyro_target_yaw = GYRO_TARGET_FILTER * yaw + (1-filter) * target;  // 慢速跟踪目标角
// gyro_correction = KP * (yaw - target) - KD * rate;                 // P锁航向 + D阻尼
```

### 3. track.c Track_Gyro_Update() — 不再初始化目标角

首帧到达时只设 first_valid 标志, 不再设定 gyro_target_yaw.

### 4. track.h — 移除 gyro_target_yaw

新增 gyro_first_valid / gyro_yaw_accel / gyro_yaw_rate 对外声明.

### 5. main.c — BNO080 初始化 (不变)

冷启动重试(5次) → TIM10启动(500Hz数据排空) → 等首帧数据(3s超时) → 主循环自动恢复.

## GYRO_ENABLE=0 兼容性
- 编译层面移除所有陀螺仪代码 (变量、函数、ISR调用)
- 直角转弯回退到灰度固定差速转弯
- 直线循迹仅用灰度PID输出

## 测试建议
1. 直线循迹: 放赛道直线段, 观察车体是否比纯灰度更平稳 (无左右振荡)
2. 弯道: 小车应自然过弯, 不被陀螺仪"拉回"
3. 直角转弯: 接近直角 → 灰度检测入口 → BNO080控制转弯(含减速区) → yaw达目标退出
4. 串口观察: yaw 值自然变化 (不定目标角), rt(角速率) 在振荡时应有明显数值
5. 参数调试: 如果振荡未消除 → 增大 KD_GYRO_STRAIGHT (0.2→0.3); 如果阻尼过度迟钝 → 减小 (0.2→0.1)

## 参考来源
- 林威仁. 以陀螺儀與光感測器訊號控制高速循跡方法設計[D]. 义守大学, 2019. (PDG法D项—纯角速率阻尼)
- 2024电赛H题: 多路灰度循迹与陀螺仪"交替盲走"融合算法. CSDN.
- CN107511824A: 一种小车自动行驶控制系统及方法. 专利. (直角减速区)
- Cornell ECE 5725: Self-driving robot with IMU fusion for lane centering. 2024.
