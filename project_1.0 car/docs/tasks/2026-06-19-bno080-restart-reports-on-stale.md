# BNO08x停流后轻量恢复旋转向量报告

**日期**: 2026-06-19

## 涉及文件
- `bsp/bno080.h`
- `bsp/bno080.c`
- `bsp/track.c`

## 修改前状态
- 用户提供的卡死日志中，进入直角 `ph1` 后 `yaw` 固定在 `-72.0`，`rt` 固定在 `-0.13`。
- `up` 持续增加，说明主循环还在调用 `bno080_update()`。
- `ex/pk/rot/gf` 不再增加，说明 BNO08x 不再触发 INTN，也没有新的 SHTP 包/旋转向量输出。
- 原恢复策略在 `gyro_stale_ms>=2000` 后执行完整 `bno080_init()`，软复位和产品ID握手耗时较长，会明显卡车。

## 修改内容和原因
- 新增 `bno080_restart_reports()`，停流后先检测 I2C 设备 ACK，再重新下发旋转向量 `SET_FEATURE`。
- `Track_Gyro_Recover()` 改为优先执行轻量报告重启；只有轻量恢复失败时才完整 `bno080_init()`。
- 保持直角退出仍由陀螺仪 yaw 控制，没有加入灰度退出或直角超时退出。

## 测试建议
- 复测直角卡死场景，观察停流时串口是否打印 `[Track] Gyro stale, restart BNO080 reports...`。
- 若轻量恢复有效，随后 `ex/pk/rot/gf` 应重新增加，`yaw` 应恢复变化。
- 若仍进入完整重初始化，说明 I2C ACK 或 `SET_FEATURE` 发送失败，需要继续排查电源/电机干扰/I2C线。

## 编译验证
- Keil MDK 构建通过：`0 Error(s), 0 Warning(s)`。
