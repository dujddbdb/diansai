# BNO080改为主循环低频轮询

**日期**: 2026-06-19

## 涉及文件
- `bsp/track_config.h`
- `app/main.c`
- `bsp/track.c`

## 修改前状态
- 关闭 `GYRO_TIM10_ENABLE` 后，用户反馈小车恢复正常循迹。
- 该结果说明疯转不是灰度映射、直角状态机或陀螺仪 PID 输出造成，而是 TIM10 高优先级运行期读取 BNO080/I2C 影响了控制时序。

## 修改内容和原因
- 保持 `GYRO_TIM10_ENABLE=0`，不再启动高优先级 TIM10 读取 BNO080。
- 新增 `GYRO_MAIN_POLL_ENABLE=1` 与 `GYRO_MAIN_POLL_PERIOD_MS=10U`，在主循环每 10ms 低优先级调用一次 `bno080_update()`。
- 主循环每次最多处理 1 个 BNO080 包，不再使用 `while (bno080_update()) {}` 排空，避免长时间占用 CPU。
- BNO080 有新数据时调用 `Track_Gyro_Update()` 更新 `yaw/rt`，但 `GYRO_CONTROL_ENABLE=0`，陀螺仪仍只诊断不控车。
- 新增 `GYRO_RECOVER_ENABLE=0`，跑车时不自动重初始化 BNO080，避免恢复流程带来的长延时卡顿。

## 测试建议
- 下载后应保持正常循迹，同时串口 `yaw/rt` 应能低频刷新。
- 若低频轮询仍稳定，可逐步把 `GYRO_MAIN_POLL_PERIOD_MS` 从 `10U` 改为 `20U/5U` 对比数据刷新和稳定性。
- 若出现偶发卡顿，优先把 `GYRO_MAIN_POLL_PERIOD_MS` 调大到 `20U`，不要恢复 TIM10 高优先级 I2C。
- 确认稳定后，再考虑把 `GYRO_CONTROL_ENABLE` 改为 `1` 小幅开启直线阻尼；直角功能仍建议单独恢复测试。

## 编译验证
- Keil MDK 构建通过：`0 Error(s), 0 Warning(s)`。
