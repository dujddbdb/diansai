# 任务记录: 移除航向锁定，简化为纯循迹+陀螺仪直角

**日期**: 2025-06-18  
**前置任务**: [融合循迹+陀螺仪直角+航向锁定](2025-06-18-fusion-track-gyro-corner-yawhold.md)  

---

## 原因

航向锁定 (yaw hold) 是测试陀螺仪用的临时功能，现在融合完成不再需要。

## 删除的内容

### track.c
- `yaw_hold_mode` / `yaw_hold_target` 变量
- `Track_YawHold_Update()` 函数 (~50行)
- `Track_YawHold_Start()` 函数
- `Track_YawHold_Stop()` 函数
- `Track_Check_Right_Angle()` 中的 `if (yaw_hold_mode) return;`
- `Track_PID_Calc()` 中的 `if (yaw_hold_mode) return;`
- `Track_Action_Execute()` 中相关注释
- `Track_Init()` 中 yaw_hold 变量清零

### track.h
- yaw_hold_mode / yaw_hold_target extern 声明
- `Track_YawHold_Update/Start/Stop` 函数声明

### track_config.h
- `KP_YAW_HOLD`, `KD_YAW_HOLD`, `YAW_HOLD_MAX_RPM`, `YAW_HOLD_MIN_RPM`, `YAW_HOLD_DEADBAND` 宏

### main.c
- 简化为纯循迹程序，只保留 `stop` / `go` 两个串口命令
- 不再调用 `Track_YawHold_Update()`
- 去掉航向锁定相关的 `hold`/`track`/`+90` 命令
- 状态打印只显示循迹模式

## 当前架构

```
主循环:
  Grayscale_Task() — ADC采集
  BNO080 重试 — 冷启动恢复
  串口: stop / go
  状态打印

TIM11 ISR (1ms):
  灰度→陀螺仪→直角检测→位置PID→动作执行

TIM7 ISR (5ms):
  编码器→速度PI→PWM
```

## 串口命令

| 命令 | 功能 |
|------|------|
| `stop\r\n` | 停转 |
| `go\r\n` | 恢复循迹 |
