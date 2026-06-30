# 修复 Trae/Claude 文档中未完全落地的循迹 Bug

**日期**: 2026-06-19

## 涉及文件
- `app/main.c`
- `bsp/track.c`
- `bsp/track.h`
- `bsp/track_config.h`

## 修改前状态
读取 `CLAUDE.md`、`docs/tasks/` 和 `.trae/specs/` 后发现，部分任务文档标记为已完成，但当前代码仍存在不一致：
- `Track_Calc_Err()` 仍用 `Analog_value / Calibrated_white` 自算黑度，没有使用已按黑白基准归一化的 `Normal_value`
- PID 差速方向与 `.trae/specs/fix-pid-corner-direction` 中的定义不一致
- 直角预触发 Phase 2 设置了 `is_right_angle` 后，动作函数会提前按直角执行
- Phase 2 等不到 8 路全白时缺少超时回退
- yaw 跨 `+180/-180` 时角速率可能出现假大跳变
- `GYRO_ENABLE=0` 纯灰度分支存在隐藏编译/回退逻辑问题

## 修改内容和原因
1. `Track_Calc_Err()` 改用 `gray_sensor.Normal_value`，黑度计算为 `4096 - Normal_value[i]`，并加入 `ERR_NOISE_THRESHOLD` 白地噪声过滤。
2. 统一误差定义：bit0/1 为右侧，右侧黑线得到负误差；正常循迹差速改为 `Left = base + correction`、`Right = base - correction`，满足负误差时右轮更快。
3. 直角动作只在 `right_angle_phase == 1` 时执行，Phase 2 只等待确认，不再提前打方向。
4. 新增 `right_angle_pretrigger_timer` 和 `RIGHT_ANGLE_PRETRIGGER_TIMEOUT`，预触发等待全白超时后自动回到正常循迹。
5. 陀螺仪 yaw 差值统一经 `angle_wrap_deg()` 环绕处理，避免跨 ±180 度时产生异常角速率。
6. 无陀螺仪或陀螺仪不可用时，直角转弯回退为灰度固定差速：右直角左轮外侧高速，左直角右轮外侧高速。
7. 速度环加入 `VELOCITY_FF_GAIN` 前馈和 `VELOCITY_SLEW_LIMIT` PWM 变化率限制，减少目标突变时抽搐。

## 编译验证
- `GYRO_ENABLE=1` 默认配置：Keil MDK 命令行构建通过，`0 Error(s), 0 Warning(s)`
- `GYRO_ENABLE=0` 纯灰度配置：临时切换后构建通过，`0 Error(s), 0 Warning(s)`，随后已恢复默认 `GYRO_ENABLE=1`

## 测试建议
- 实车直线循迹：观察串口 `err`，右偏应为负，且 `TR > TL`；左偏应为正，且 `TL > TR`
- 直角测试：右直角应左轮外侧高速、右轮内侧低速；左直角相反
- 若直角误触发仍多，优先把 `RIGHT_ANGLE_DETECT_CNT` 从当前测试值 `1` 调回更稳的连续确认值
