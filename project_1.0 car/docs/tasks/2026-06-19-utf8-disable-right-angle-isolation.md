# UTF-8编码与关闭直角状态机隔离摇摆

**日期**: 2026-06-19

## 涉及文件
- `bsp/track_config.h`
- `bsp/track.c`
- `app/main.c`

## 修改前状态
- 纯灰度循迹稳定。
- `GYRO_ENABLE=1` 且 `GYRO_CONTROL_ENABLE=0` 时，陀螺仪只采集不参与直线/直角控制，但实车仍出现左右摇摆。
- 源码文件存在非 UTF-8 编码，导致补丁工具无法直接修改。
- `app/main.c` 仍残留已删除的 `right_angle_pretrigger_timer` 清零语句，重新构建会报未定义。

## 修改内容和原因
- 将 `bsp/track.c` 与 `bsp/track_config.h` 转为 UTF-8 no BOM，方便后续补丁和查看。
- 新增 `RIGHT_ANGLE_ENABLE`，当前设为 `0`，临时关闭直角状态机，只保留灰度循迹与 BNO080 数据采集。
- 在 TIM11 控制环中，当 `RIGHT_ANGLE_ENABLE=0` 时强制清空直角相关状态，避免误进入 `ph=1` 后执行固定差速转弯。
- 删除 `app/main.c` 中已不存在的 `right_angle_pretrigger_timer` 清零语句，修复编译错误。

## 测试建议
- 下载当前固件后测试：如果不再左右摇摆，说明摇摆来自直角检测/执行误触发，不是陀螺仪 PID 输出。
- 串口观察 `ph`/`t` 应保持 `0`，同时 `yaw`/`rt` 仍应有输出。
- 若仍左右摇摆，下一步应排查 BNO080 初始化/TIM10 I2C排空/串口打印负载对控制周期的影响。
- 后续恢复直角功能时，将 `RIGHT_ANGLE_ENABLE` 改回 `1`，再单独收紧直角入口判断或确认逻辑。

## 编译验证
- Keil MDK 构建通过：`0 Error(s), 0 Warning(s)`。
