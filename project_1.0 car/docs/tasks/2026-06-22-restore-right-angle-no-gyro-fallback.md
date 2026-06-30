# 2026-06-22 - 恢复无陀螺仪时直角执行回退

## 日期
- 2026-06-22

## 背景
- 现象: 直角昨天可正常检测, 修改代码后小车会漏掉直角。
- 根因: 直角入口和近全白确认已经可能触发, 但确认瞬间如果 `gyro_yaw_available == 0`, 旧逻辑会直接进入退出相位并清掉 `is_right_angle`, 表现成“检测不到直角”。
- 外部实现经验: 线循迹 90 度转弯通常在检测到入口后跳出普通 PID, 进入专门转弯状态; 陀螺仪不可用时也应有灰度/定时回退, 不能直接丢弃本次直角。

## 涉及文件
- `bsp/track.c`
- `bsp/track.h`
- `bsp/track_config.h`
- `app/main.c`
- `tools/test_track_logic.py`

## 修改内容
- `Track_Check_Right_Angle()` Phase1 近全白确认后始终进入 Phase2 执行, 不再因为当帧无陀螺仪而清掉直角。
- 新增 `right_angle_fallback_timer`, `RIGHT_ANGLE_FALLBACK_MIN_MS`, `RIGHT_ANGLE_FALLBACK_TIMEOUT_MS`: 无陀螺仪直角执行时, 先固定差速转弯, 最短时间后看到黑线退出, 或到超时强制退出。
- 新增 `right_angle_gyro_armed`: 只有进入 Phase2 当刻已锁住起始 yaw, 本次直角才使用陀螺仪闭环; 防止中途恢复陀螺仪后用错误起点切换控制方式。
- `Track_Action_Execute()` 增加无陀螺仪固定差速回退:
  - 右直角 `type=1`: `gyro_diff=-50`, 左轮快、右轮慢。
  - 左直角 `type=2`: `gyro_diff=+50`, 左轮慢、右轮快。
- 主循环恢复 `Track_Main_Debug(200)`, 便于现场观察 `IR`, `RA`, `pre`, `Yaw`, `stale`。
- 修正 `track.h` 和 `track.c` 中直角相位/方向注释, 避免串口排查时把 `phase=1/2` 看反。

## 验证
- `python tools\test_track_logic.py`: 6 tests passed。
- Keil MDK 命令行构建: `0 Error(s), 0 Warning(s)`。

## 现场复测
- 串口重点看 `RA:is/phase tX pre:Y`:
  - 看到入口后应先出现 `RA:0/1 t1|t2 pre:n`。
  - 近全白确认后应进入 `RA:1/2` 或 `RA:2/2`。
  - 如果 `Yaw`/`stale` 异常, 仍应靠固定差速回退转弯, 不会直接丢弃直角。
- 若仍漏检, 保存直角前后 1 秒串口日志, 优先看 `IR` 是否出现过 `00XXXX11` 或 `11XXXX00`。
