# 2026-06-22 - BNO08x停流后优先轻量重启报告流

## 日期
- 2026-06-22

## 背景
- `GYRO_MAIN_POLL_PERIOD_MS` 控制的是 I2C 读包尝试周期。
- `Track_Gyro_Update()` 每 1ms 消费 `bno080_data_available()`, 没有新旋转向量时 `gyro_stale_ms++`。
- 当 `stale` 持续增长到阈值时, 说明 BNO08x 数据链路已经停流, 不能只等待下一次 10ms 读包。

## 涉及文件
- `bsp/bno080.c`
- `bsp/bno080.h`
- `bsp/track.c`
- `tools/test_track_logic.py`

## 修改内容
- 新增 `bno080_restart_reports()`: 不做软复位, 只检测 I2C ACK 并重新下发旋转向量 `SET_FEATURE`。
- `Track_Gyro_Recover()` 在 `gyro_stale_ms` 超限后先调用 `bno080_restart_reports()`。
- 只有轻量恢复失败时才执行完整 `bno080_init()`, 避免运行中频繁软复位导致长时间卡车。
- 恢复后统一清零 `gyro_first_valid` / `gyro_yaw_available` / `gyro_stale_ms`, 让下一帧 yaw 重新作为首帧进入, 避免差分跳变。
- 增加逻辑测试, 防止恢复路径退化成直接完整初始化。

## 现场观察
- 正常: `frm` 持续增长, `stale` 在两帧之间短暂上升后归零。
- 停流: `stale` 持续增长到阈值, 串口打印 `Gyro stale, restarting BNO080 reports...`。
- 轻量恢复成功: `rec` 增加一次后, `frm` 继续增长。
- 若打印 `report restart failed, full init`, 说明 I2C ACK 或 SET_FEATURE 发送失败, 应继续查电源/I2C线/电机干扰。

## 验证
- 运行 `python tools\test_track_logic.py`。
- 运行 Keil MDK 命令行构建。
