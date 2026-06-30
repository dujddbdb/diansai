# BNO080读包期间屏蔽EXTI防重入

**日期**: 2026-06-19

## 涉及文件
- `bsp/bno080.c`

## 修改前状态
- 已确认关闭 TIM10 高优先级读取后，小车可正常循迹。
- 用户判断：BNO08x 时序不对时，读数据过程中进入中断可能导致 BNO08x/I2C 卡死。
- 原驱动中 EXTI9_5 中断只置位 `g_i2c_data_ready`，但在主循环读 SHTP 包期间，INTN 仍可能再次触发，造成数据就绪标志和 I2C 读包流程交叠。

## 修改内容和原因
- 新增 `g_bno080_i2c_busy` 忙标志，`bno080_update()` 正在读包时直接拒绝重入。
- 新增 `BNO080_EXTI_SetEnabled()`，可临时关闭/恢复 EXTI_Line5。
- `bno080_update()` 读包前设置 busy 并关闭 EXTI_Line5，读包成功或失败后清 busy 并恢复 EXTI_Line5。
- `EXTI9_5_IRQHandler()` 中如果发现 `g_bno080_i2c_busy=1`，只清 pending 并返回，不再修改数据就绪状态。
- 当前仍保持 `GYRO_TIM10_ENABLE=0` 与 `GYRO_MAIN_POLL_ENABLE=1`，即主循环低优先级读取 BNO080，不恢复高优先级 TIM10 I2C。

## 测试建议
- 下载后验证正常循迹不受影响。
- 串口观察 `yaw/rt` 是否能刷新；若能刷新且车不抖，说明主循环低频轮询 + 防重入保护有效。
- 如果 `yaw/rt` 偶发停住，可先把 `GYRO_MAIN_POLL_PERIOD_MS` 调大到 `20U`，不要把 I2C 读包放回高优先级中断。
- 后续若要恢复陀螺仪控车，先只开 `GYRO_CONTROL_ENABLE=1` 的直线阻尼，直角另行测试。

## 编译验证
- Keil MDK 构建通过：`0 Error(s), 0 Warning(s)`。

## 实车验证补充
- 用户实车反馈：当前版本已恢复正常循迹。
- 结论：疯转根因基本确认是 BNO08x 运行期读包被中断/高优先级 I2C 时序打断导致，而非灰度循迹、直角状态机或陀螺仪 PID 输出本身。
- 后续不要把 BNO08x I2C 读包放回 TIM10 高优先级中断；继续使用主循环低优先级轮询，并保留读包期间 EXTI 屏蔽与 busy 防重入保护。
