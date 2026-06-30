# 2026-06-20 - BNO080 I2C 期间屏蔽并补进 TIM7/TIM11 中断

## 日期
- 2026-06-20

## 涉及文件
- `bsp/bno080.c`
- `bsp/track_config.h`
- `app/main.c`

## 修改前状态
- BNO080 I2C 读写期间会关闭 TIM11 和 TIM7 的更新中断，避免 I2C 时序被控制中断打断。
- 上一版恢复 TIM11 时清除了 TIM11 挂起位，相当于 I2C 期间到点的 TIM11 控制环不补算。
- 当前调试需求改为：I2C 读取期间两个定时器都不要进中断，但读完后 pending 的定时器中断要补进。

## 修改内容和原因
- 删除 `BNO080_TimerIrq_Unlock()` 中对 `TIM11` 的 `TIM_ClearITPendingBit()`。
- I2C 读写开始时同时关闭 `TIM11` 和 `TIM7` 的 `TIM_IT_Update` 中断使能。
- I2C 读写结束时只重新使能 `TIM11` 和 `TIM7` 更新中断，不清除 pending 标志。
- 这样如果 I2C 期间 TIM7/TIM11 到点，硬件更新标志会保留；读完 I2C 后中断重新使能，控制环/速度环会补执行一次。

## 陀螺仪读取周期
- 当前运行时 BNO080 由主循环轮询读取。
- `GYRO_MAIN_POLL_PERIOD_MS = 10U`，即每 10ms 触发一次 `bno080_update()`，约 100Hz。
- BNO080 报告周期也是 10ms，和主循环读取周期匹配。

## 测试建议
- 下载后观察直角阶段 `up/pk/rot/gf` 是否持续增长，不再冻结。
- 观察速度环是否稳定：TIM7 不会在 I2C 中途打断，但 pending 会补进，理论上只会短暂延后而不丢周期。
- 若仍卡 BNO080，可继续降低 I2C 频率或把 BNO080 读取改成 DMA/状态机。

## 编译验证
- Keil MDK 构建通过：`0 Error(s), 0 Warning(s)`。
