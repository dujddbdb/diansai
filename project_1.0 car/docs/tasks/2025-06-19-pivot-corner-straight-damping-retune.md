# 任务记录: 枢轴直角转弯 + 直线纯阻尼重新调参 (方波赛道)

**日期**: 2025-06-19
**目标**: 2025电赛E题方波赛道平滑运行 — 直角枢轴转 + BNO08x直线精细补偿(纯阻尼)
**前置**: [2025-06-19 修三个bug(方向/时序/溢出)](2025-06-19-gyro-fusion-pdg-corner-decel.md)

## 涉及文件
- `bsp/track_config.h` — 直线KD放大、关角加速度前馈、直角枢轴参数、检测去抖
- `bsp/track.c` — 直角主体基速 BASE_RPM → CORNER_TURN_RPM (枢轴)
- `bsp/bno080.c` — 旋转向量报告 50Hz → 100Hz

## 修改前问题诊断 (读全代码后)

| 问题 | 位置 | 后果 |
|------|------|------|
| KD系数量纲太小 | KD_GYRO_STRAIGHT=0.2, KD_CORNER_YAW=0.25 | gyro_yaw_rate典型值0.2~1.0, 修正量≈0.1RPM, 陀螺仪**几乎不起作用** |
| 转弯只转80° | CORNER_YAW_TARGET=80 | 方波真90°, 系统性欠转10°, 出弯车头偏 |
| 减速区形同虚设 | CORNER_DECEL_ANGLE=1.0 | 只在最后1°降速, 全程猛冲→过冲 |
| 单帧触发无去抖 | DETECT/CONFIRM/EXIT_CNT=1 | 任何灰度抖动假触发直角 |
| 角加速度前馈是噪声 | KP_YAW_ACCEL_FF=0.08 | 50Hz下二阶差分=噪声, 注入抖动 |

## 用户决策
1. **直线**: 保留纯角速率阻尼(不锁航向), 仅重新调参
2. **直角**: 枢轴转 / 内轮近停

## 修改内容

### 1. track_config.h
```c
/* 直线纯阻尼重调 */
KD_GYRO_STRAIGHT  0.2 → 4.0    // 放大20倍, 给阻尼真正的权限 (实车2~10间调)
KP_YAW_ACCEL_FF   0.08 → 0.0   // 关闭噪声前馈

/* 直角枢轴转 */
CORNER_YAW_TARGET 80 → 88      // 接近真90°
KD_CORNER_YAW     0.25 → 1.5   // 放大, 防枢轴过冲
CORNER_TURN_RPM   (新增) 55    // 转弯主体基速, 低→枢轴
CORNER_DECEL_ANGLE 1 → 20      // 真减速区
CORNER_DECEL_RPM  85 → 35      // 末段轻柔收尾
CORNER_MIN_RPM    10 → 0       // 内轮可停 = 枢轴转
INNER_CORNER_RPM  20 → 0       // 灰度回退也枢轴

/* 检测去抖 */
DETECT_CNT 1→4, CONFIRM_CNT 1→2, EXIT_CNT 1→2
```

### 2. track.c — 直角主体基速
`Track_Action_Execute()` 直角分支: `base = BASE_RPM` → `base = CORNER_TURN_RPM`
枢轴原理: 外轮=base+|diff|→钳120, 内轮=base−|diff|→钳0。
- 右转(type1): target=start−88 → 左轮120/右轮0, 绕右轮转 ✓
- 左转(type2): target=start+88 → 左轮0/右轮120, 绕左轮转 ✓

### 3. bno080.c — 报告率
`BNO080_REPORT_INTERVAL_MS 20 → 10` (50→100Hz)
枢轴转停角更精准, 直线阻尼延迟减半。TIM10@500Hz排空足够。

## 仍存在(未改, 视情况)
- **灰度竞态**: Grayscale_Task(主循环写) vs TIM11 ISR(读Normal_value) 无锁, 有撕裂噪声。已用3样本均值+限幅+检测去抖兜底, 暂可接受。彻底解决需快照缓冲。
- **角速率来源**: 当前用yaw差分得rate, 噪声大且滞后。理想用BNO080的校准角速度报告(rad/s)直接读。属较大驱动改动, 本次未做。

## 测试建议 (电机拔线先看串口)
1. **直线**: 手持平移, 看 rt(角速率) 是否有数值; 上车看是否还画龙
   - 仍画龙 → KD_GYRO_STRAIGHT 4→6→8
   - 发抖 → 4→2
2. **直角**: 看 ph0→2→1→3 流转, dY 累积到88°退出
   - 内轮应基本停(TL或TR≈0), 外轮转
   - 过冲 → CORNER_YAW_TARGET 88→85 或 KD_CORNER_YAW 1.5→2.5
   - 欠转 → 88→90
   - 切角太狠 → CORNER_TURN_RPM 55→70 (更扫弯)
3. **假触发**: 直线段不应突然进入 ph2/ph1; 若有 → DETECT_CNT 4→8
