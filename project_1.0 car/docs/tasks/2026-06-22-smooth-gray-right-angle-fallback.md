# 2026-06-22 - 灰度直角回退平滑过渡

## 日期
- 2026-06-22

## 背景
- 需求: 灰度版本直角转弯也要平滑过渡, 避免进入回退时左右轮差速从 0 瞬间跳到固定值。
- 当前灰度回退已经能在无陀螺仪时执行直角, 但 `gyro_diff` 一帧给到固定差速, 速度环会感到突变。

## 涉及文件
- `bsp/track.c`
- `bsp/track_config.h`
- `tools/test_track_logic.py`

## 修改内容
- 新增 `gray_corner_diff_smooth`: 灰度直角回退专用差速平滑量。
- 新增参数:
  - `RIGHT_ANGLE_FALLBACK_DIFF_RPM`: 灰度回退目标差速。
  - `RIGHT_ANGLE_FALLBACK_DIFF_RAMP_UP`: 进入灰度回退时差速爬升速度。
  - `RIGHT_ANGLE_FALLBACK_DIFF_RAMP_DOWN`: 退出灰度回退后差速回零速度。
- 无陀螺仪直角执行时, 目标差速按方向设置为正/负, 再逐步逼近目标, 不再一帧突变。
- 退出直角后, `gray_corner_diff_smooth` 继续按下降斜坡回到 0, 再完全交还给普通循迹 PID。
- 串口调试增加 `gDiff`, 用于观察灰度回退差速平滑过程。

## 调参建议
- 转弯还是太冲: 降低 `RIGHT_ANGLE_FALLBACK_DIFF_RPM` 或 `RIGHT_ANGLE_FALLBACK_DIFF_RAMP_UP`。
- 转弯反应太慢: 增大 `RIGHT_ANGLE_FALLBACK_DIFF_RAMP_UP`, 或略增 `RIGHT_ANGLE_FALLBACK_DIFF_RPM`。
- 出弯还甩一下: 降低 `RIGHT_ANGLE_FALLBACK_DIFF_RAMP_DOWN`。

## 验证
- 已增加逻辑测试覆盖灰度回退差速爬升和退出回零。
