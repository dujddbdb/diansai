# BNO08x直角停流快速恢复与I2C读写保护

**日期**: 2026-06-19

## 涉及文件
- `bsp/track_config.h`
- `bsp/track.c`
- `bsp/bno080.c`

## 修改前状态
- 新日志显示 BNO08x 在直角阶段约 `st=10~30` 后开始停流：`ex/pk/rot/gf` 不再增加，`yaw` 固定。
- 原恢复阈值为 2000ms，直角执行中等待太久，车辆会长时间卡在 `ph1`。
- BNO08x 的 `SET_FEATURE` 发送和底层 I2C 读写未统一纳入 TIM11 屏蔽保护，仍可能被主控制环插入影响时序。

## 修改内容和原因
- 新增 `GYRO_RECOVER_STALE_MS=2000U`，作为普通循迹停流恢复阈值。
- 新增 `GYRO_CORNER_RECOVER_STALE_MS=120U`，直角执行 `ph1` 中更快触发 BNO08x 恢复。
- `Track_Gyro_Recover()` 根据是否处于直角执行阶段选择不同阈值。
- 将 `I2C1_Write_NBytes()` 和 `I2C1_Read_NBytes()` 都纳入可嵌套 TIM11 更新中断屏蔽，避免 BNO08x I2C 读/写事务被 TIM11 控制环打断。
- 保持 TIM7 速度环不关闭，避免影响速度闭环。
- 保持直角退出仍由陀螺仪 yaw 控制，没有加入灰度退出或直角超时退出。

## 测试建议
- 复测直角卡死场景：如果 BNO08x 再停流，应在约 120ms 后打印 `Gyro stale, restart BNO080 reports...`。
- 观察恢复后 `ex/pk/rot/gf` 是否重新增长，`yaw` 是否恢复变化。
- 如果仍频繁停流，下一步应重点查电源/电机干扰/I2C线，或降低 BNO08x 报告频率减轻总线负载。

## 编译验证
- Keil MDK 构建通过：`0 Error(s), 0 Warning(s)`。
