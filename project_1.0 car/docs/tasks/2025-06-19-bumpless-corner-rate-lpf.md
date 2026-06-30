# 任务记录: 消除"过直角抽一下" — 无扰切换 + 角速率低通滤波

**日期**: 2025-06-19
**问题**: 闭环过直角时电机"抽一下"(突跳)
**前置**: [2025-06-19 枢轴转+直线纯阻尼重调](2025-06-19-pivot-corner-straight-damping-retune.md)

## 涉及文件
- `bsp/track_config.h` — 新增 GYRO_RATE_LPF_ALPHA, BASE_RPM_SLEW
- `bsp/track.c` — 角速率LPF + 位置级目标转速斜坡限制 + 状态globals

## 联网调研结论 (别人的高级处理)

| 技术 | 解决 | 来源 |
|------|------|------|
| 无扰切换 bumpless transfer | 模式切换输出突跳 | MathWorks / Industrial Monitor |
| 角速度低通滤波再喂D项 | D放大差分噪声→抖 | joehutch line-follower / 互补滤波 |
| D-on-measurement | derivative kick | indmall PID FAQ |
| 周期匹配陀螺率≥100Hz↔≤10ms | 已满足 | 智能车CSDN |
| 直读BNO校准角速度(rad/s) | 差分yaw的阶梯+滞后 | Adafruit BNO085 report types |

## 根因分析
1. **模式切换突跳**: 直线→直角时 Left/Right_Base_RPM 从~(100,100) 瞬间跳到枢轴(120,0); 出弯反向跳。速度环FF(VELOCITY_FF_GAIN=2.0)把RPM突变直接转成PWM突变 → 抽一下。
2. **放大KD放大噪声**: 上一任务把 KD_GYRO_STRAIGHT 0.2→4.0、KD_CORNER 0.25→1.5, 而 gyro_yaw_rate 是 yaw 差分(阶梯+噪声), 放大后噪声→抖动。

## 修改内容

### 1. 角速率一阶低通 (track.c Track_Gyro_Update)
```c
yaw_rate_raw = wrap(new_yaw - gyro_yaw_deg);          // 差分原始角速率
rate_filt = α*yaw_rate_raw + (1-α)*gyro_yaw_rate;     // 一阶LPF
gyro_yaw_rate = rate_filt;                            // 滤波后才喂D项
```
`GYRO_RATE_LPF_ALPHA=0.40` (100Hz帧率上运行, ~20ms时间常数)。

### 2. 无扰切换 — 位置级目标转速斜坡 (track.c Track_Action_Execute 末尾)
```c
/* 无论直线/直角分支算出什么目标, 每1ms限制最大变化量 */
dL = Left_Base_RPM - Left_Base_RPM_prev;
if (dL > BASE_RPM_SLEW)  Left_Base_RPM = prev + BASE_RPM_SLEW;
... (R同理)
Left_Base_RPM_prev = Left_Base_RPM;  // 记忆本次输出
```
`BASE_RPM_SLEW=6.0` RPM/ms。效果: 入弯内轮 100→0 用~17ms平滑降, 外轮 100→120 用~4ms; 出弯反向同样平滑。切换连续 → 不再抽。

### 3. 新增 globals (track.c)
`Left_Base_RPM_prev / Right_Base_RPM_prev`, Track_Init 中清零。

## 与速度环已有斜坡的关系
速度环已有 VELOCITY_SLEW_LIMIT(120 PWM/5ms)是PWM级; 本次新增的是**位置级**(目标RPM/1ms), 更上游更平滑, 两者叠加。

## 测试建议 (电机拔线先看串口)
1. 过直角看 TL/TR 是否平滑过渡(不再瞬跳); 上车看是否还抽
2. 仍抽 → BASE_RPM_SLEW 6→3 (更平滑); 入弯起转太慢 → 6→10
3. 直线仍抖 → GYRO_RATE_LPF_ALPHA 0.4→0.3 (更平滑); 阻尼迟钝 → 0.4→0.5 同时可降KD
4. 串口 rt(角速率) 现在应更平滑无毛刺

## 下一步可选升级 (更彻底, 未做)
**直读 BNO080 校准角速度报告 (0x02, rad/s)** 替代差分yaw:
- 优点: 无差分噪声、无阶梯、低延迟, 是D项阻尼的理想输入
- 代价: 驱动需额外使能 SENSOR_REPORTID_GYROSCOPE_CALIBRATED, 在 bno080_parse_input_report 解析角速度三轴, 取Z轴
- 若LPF+无扰切换后仍不够顺, 再上这个
