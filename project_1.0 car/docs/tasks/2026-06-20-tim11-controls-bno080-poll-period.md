# 2026-06-20 - TIM11 控制 BNO08x 读取周期

## 日期
- 2026-06-20

## 涉及文件
- `bsp/track.c`
- `bsp/track.h`
- `app/main.c`
- `bsp/track_config.h`

## 修改前状态
- BNO08x 运行时由主循环使用 `HAL_GetTick()` 判断是否到 10ms，再调用 `bno080_update()` 读取 I2C。
- I2C 读取期间会屏蔽 TIM7/TIM11 更新中断并保留 pending，避免 I2C 时序被中断打断。
- 需求调整为：BNO08x 的读取周期计数由 TIM11 负责，但 I2C 读取仍不能放在 TIM11 中断里。

## 修改内容和原因
- 新增 `gyro_poll_request` 和 `gyro_poll_timer_ms`。
- 新增 `Track_Gyro_PollTick()`：在 TIM11 1ms 中断中计数，到 `GYRO_MAIN_POLL_PERIOD_MS` 后置位读取请求。
- 新增 `Track_Gyro_TakePollRequest()`：主循环消费读取请求，消费后清零请求标志。
- `app/main.c` 主循环不再用 `HAL_GetTick()` 计时，而是看到 TIM11 置位的请求后才调用 `bno080_update()`。
- 修复 `track.c` 中被乱码注释吞掉的 `gyro_corner_target_yaw`、`gyro_last_yaw` 定义，并补回 `Track_Init()` 末尾真实 `}`。
- 将 `track_config.h`、`track.h` 保存为 UTF-8 no BOM + CRLF，消除 Keil 末尾无换行警告。

## 当前时序
- TIM11：1ms 中断，只做周期计数并置位 `gyro_poll_request`，不做 I2C。
- 主循环：看到 `gyro_poll_request=1` 后调用 `bno080_update()` 读取 BNO08x。
- BNO08x 读取周期：`GYRO_MAIN_POLL_PERIOD_MS = 10U`，即 10ms/100Hz。
- I2C 读取期间：临时关闭 TIM7/TIM11 更新中断使能，读完恢复；pending 保留，因此两个定时器中断会补进。

## 测试建议
- 下载后观察 `up` 基本按 10ms 周期增加，`pk/rot/gf` 不应在直角阶段冻结。
- 如果 `up` 增加但 `pk/rot/gf` 不增加，说明请求周期正常但 BNO08x 数据流仍停，需要继续查 I2C/BNO 恢复。
- 如果速度环抖动，观察 TIM7 是否因 I2C 读取补中断造成瞬时聚集，可再限制每次主循环最多读一个 SHTP 包。

## 编译验证
- Keil MDK 全量 Rebuild 通过：`0 Error(s), 0 Warning(s)`。
