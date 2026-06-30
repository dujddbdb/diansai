# 2026-06-19 - 修复 BNO080 直角停流与编译错误

## 日期
- 2026-06-19

## 涉及文件
- `bsp/bno080.c`
- `bsp/track_config.h`

## 修改前状态
- 直角过程中偶发 `yaw/rt` 卡死，串口里 `up` 继续增加，但 `ex/pk/rot/gf` 不再增加，说明主循环还在调用 `bno080_update()`，BNO080 的 SHTP/旋转向量数据流停止。
- `bsp/bno080.c` 存在多处乱码行尾 `//` 注释吞掉 `{`、`if`、`return` 等代码的问题，导致编译报错。
- `bsp/track_config.h` 中 `GYRO_CORNER_RECOVER_STALE_MS` 被上一行注释吞掉，直角阶段 120ms 快速恢复宏可能未真正参与编译。

## 修改内容和原因
- 将 `bsp/bno080.c` 重写为结构清晰的 UTF-8 C 文件，保留原有 BNO080/SHTP 初始化、收包、解析四元数、欧拉角转换、TIM10 可选排空接口。
- BNO080 I2C 读写期间只临时屏蔽 TIM11 更新中断，不关闭 TIM7 速度环，避免 I2C 事务被 1ms 控制中断打断，同时不破坏速度闭环。
- EXTI5 中断只置位数据就绪标志；I2C 读包仍在主循环/TIM10路径执行，避免在中断内访问 BNO080。
- `bno080_restart_reports()` 改为轻量重新下发 `SET_FEATURE`，供直角卡流时快速恢复数据流。
- 修复 `GYRO_CORNER_RECOVER_STALE_MS` 独立成行，确保直角执行阶段使用 120ms 停流恢复阈值。

## 测试建议
- 下载后重点复测直角阶段：观察 `ph1` 时若 `ex/pk/rot/gf` 停止，约 120ms 后应打印 `[Track] Gyro stale, restart BNO080 reports...` 和 `[BNO080] Restart reports...`。
- 若恢复成功，`pk/rot/gf` 应重新增加，`yaw` 应继续变化并退出直角。
- 如果仍卡死且没有恢复打印，检查 `gyro_stale_ms` 是否增长到 120 以上，以及 `Track_Gyro_Recover()` 是否在主循环持续调用。
- 如果有恢复打印但仍不恢复，下一步应考虑在轻量 `SET_FEATURE` 失败后缩短进入完整 `bno080_init()` 的路径，或降低 I2C 时钟进一步排查硬件时序。

## 编译验证
- Keil MDK 构建通过：`0 Error(s), 0 Warning(s)`。
