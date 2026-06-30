# 2025-06-19 删除不需要的硬件补偿宏

## 涉及文件
- `bsp/track_config.h`
- `bsp/track.c`
- `CLAUDE.md`

## 修改前状态
`track_config.h` 中定义了 5 个编译期开关宏，用于在软件层补偿硬件接线问题：

```c
#define ENCODER_LEFT_INVERT  0
#define ENCODER_RIGHT_INVERT 1   // 只有右编码器需要反转，不对称
#define LR_SWAP              0
#define LEFT_MOTOR_REVERSE   0
#define RIGHT_MOTOR_REVERSE  0
```

`track.c` 中对应有编码器方向校正、电机 PWM 方向反转、L/R 通道交换三处条件判断。

## 修改内容
1. **track_config.h**：删除 5 个宏定义
2. **track.c**：删除对应的三处 `if` 条件判断，保留直通逻辑
3. **CLAUDE.md**：移除对已删除宏的文档引用

## 原因
- 这些宏是之前硬件接线不正确时的软件补丁
- `ENCODER_RIGHT_INVERT=1` 说明右编码器物理线序是反的，应在硬件层修复
- 硬件重新正确接线后，这些开关全是 dead code
- 清理后代码逻辑更清晰：编码器直读、PWM 直出、不交换通道

## 测试建议
- 确认编码器读数方向正确（电机正转时读数为正）
- 确认左右电机 PWM 方向与预期一致
- 实地跑一圈循迹确认不跑偏
