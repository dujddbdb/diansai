# 2026-06-22 - 修复陀螺仪恢复与直角角度环绕

## 日期
- 2026-06-22

## 涉及文件
- `bsp/track.c`
- `bsp/track_config.h`
- `tools/test_track_logic.py`

## 修改前状态
- `Track_Gyro_Recover()` 已存在但主循环没有调用，BNO080 停流后不会自动恢复。
- `Track_Gyro_TakePollRequest()` 调用 `bno080_update()` 后没有统计成功次数，串口 `ok` 计数长期不变，容易误判数据流状态。
- `Track_Init()` 对 BNO080 使用无限重试，陀螺仪没接好或冷启动失败时整车会卡在初始化。
- 直角转弯 PD 直接使用 `gyro_corner_target_yaw - gyro_yaw_deg`，当 yaw 跨过 `-180/180` 边界时会把小误差算成接近 360 度的大误差。
- 直角 Phase1 确认后即使没有有效陀螺仪，也可能进入 Phase2，后续无法靠 yaw 退出。

## 修改内容和原因
- 新增 `Track_Angle_Delta_Deg()`，统一把 yaw 差值归一到 `[-180, 180]`，用于角速度、直角进度、直角 yaw 误差计算。
- `Track_Gyro_TakePollRequest()` 在 `bno080_update()` 成功时递增 `bno080_ok_count`，让串口诊断里的 `ok` 有真实意义。
- `Track_Main_Gyro()` 每轮同时调用 `Track_Gyro_Recover()`，让停流恢复逻辑真正生效。
- 新增 `BNO080_INIT_RETRY_MAX=3U`，上电初始化最多重试 3 次；失败后继续灰度循迹并打印提示，避免整车卡死。
- 新增 `GYRO_RECOVER_STALE_MS=2000U` 和 `GYRO_CORNER_RECOVER_STALE_MS=120U`，直角执行中更快触发陀螺仪恢复。
- Phase1 确认直角时，如果 `gyro_yaw_available==0`，立即走退出复位，不进入依赖 yaw 的直角执行。
- 新增 `tools/test_track_logic.py`，用 Python 标准库回归检查上述关键逻辑，避免后续改动回退。

## 外部参考
- SparkFun/Ceva BNO080 Datasheet: BNO080 rotation vector 姿态输出基于四元数。
- SparkFun BNO080 Arduino Library: 常见实现也是使能 rotation vector 后按数据就绪更新。
- 机器人 yaw 控制资料普遍使用角度 wrap，将 heading error 归一到 `[-180, 180]`，避免跨 `-180/180` 边界时转错方向。
- Pololu 论坛关于 90 度循迹转弯的讨论建议检测到直角后跳出普通循迹，执行单独转弯逻辑。

## 测试建议
- 串口观察 `frm/isr/ok/stale/rec`：正常时 `ok` 应随 BNO080 包读取增长；停流后 `stale` 增长并触发 `rec` 增加。
- 连续跑跨 `-180/180` yaw 边界的左右直角，观察是否仍出现突然反向或大幅过冲。
- 拔掉或延迟上电 BNO080 测试，程序不应卡在 `Track_Init()`；应打印 `BNO080 init failed` 并继续灰度循迹。
- 如果 BNO080 未接入时每 2 秒恢复尝试造成明显卡顿，可再加恢复退避或仅在直角阶段快速恢复。

## 验证
- `python tools\test_track_logic.py` 通过：5 tests OK。
- Keil MDK 命令行构建通过：`0 Error(s), 0 Warning(s)`。
