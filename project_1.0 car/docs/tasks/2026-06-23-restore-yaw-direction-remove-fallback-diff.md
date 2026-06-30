# 2026-06-23 - 恢复航向角方向并去除灰度回退差速逻辑

## 日期
- 2026-06-23

## 涉及文件
- `bsp/track.c`
- `bsp/track_config.h`

## 修改前状态
- 直角转弯目标航向角设置方向被改反: 右直角(`is_right_angle==1`)使用 `start + 90`, 左直角使用 `start - 90`。
- 正常循迹模式中残留灰度回退差速衰减逻辑: `gray_corner_diff_smooth` 在退出直角后仍按 `RIGHT_ANGLE_FALLBACK_DIFF_RAMP_DOWN` 斜坡回零并叠加到 `gyro_diff`, 但灰度回退路径已不再使用, 该衰减块无实际作用且增加复杂度。
- `right_angle_fallback_timer` 变量及其3处赋值仍存在, 但已无任何读取使用。
- `track_config.h` 中残留5个 `RIGHT_ANGLE_FALLBACK_*` 宏定义, 已无代码引用。

## 修改内容和原因

### Task 1: 恢复航向角目标设置 (修复直角方向)
- `track.c` Phase1 全白确认后设置 `gyro_corner_target_yaw` 处, 交换 `+` 和 `-`:
  - 右直角(`is_right_angle==1`): `target = start - CORNER_YAW_TARGET` (yaw减小=右转)
  - 左直角(else): `target = start + CORNER_YAW_TARGET`
- 注释同步更新为 "右直角右转: target = start - 90 (yaw减小=右转)"。
- 原因: 恢复与 `GYRO_YAW_DIRECTION=1` (左转yaw增大) 一致的方向映射。

### Task 2: 去除灰度回退差速逻辑
- 移除 `right_angle_fallback_timer` 变量定义 (原第89行)。
- 移除该变量的全部3处赋值:
  - Phase1 进入Phase2时 (原第419行)
  - Phase3 退出重置时 (原第467行)
  - `Track_Init` 初始化时 (原第981行)
- 移除正常循迹模式中的 `gray_corner_diff_smooth` 衰减块 (原第597-604行):
  ```c
  // 已删除: if (gray_corner_diff_smooth > 0.0f) { ... } gyro_diff += gray_corner_diff_smooth;
  ```
- **保留** `gray_corner_diff_smooth` 变量定义 (第80行) 和初始化 (第973行)。
- **保留** Phase2/Phase3 中的平滑过渡功能 (第562-565行和第575-578行), 使用 `CornerProfile_Slew` 限斜率。
- **保留** 串口调试输出中的 `gDiff` 字段 (第821行)。
- 原因: 灰度回退路径已不再使用, 正常循迹不应有残留差速叠加; Phase2/3 的限斜率平滑过渡仍需保留以避免差速突变。

### Task 3: 去除无用的宏定义
- `track_config.h` 移除以下5个宏:
  - `RIGHT_ANGLE_FALLBACK_MIN_MS`
  - `RIGHT_ANGLE_FALLBACK_TIMEOUT_MS`
  - `RIGHT_ANGLE_FALLBACK_DIFF_RPM`
  - `RIGHT_ANGLE_FALLBACK_DIFF_RAMP_UP`
  - `RIGHT_ANGLE_FALLBACK_DIFF_RAMP_DOWN`
- 原因: 全项目无任何代码引用这些宏 (已用全文搜索确认)。

## 验证
- Keil MDK 命令行构建: `0 Error(s), 1 Warning(s)`。
  - 唯一警告 `right_angle_gyro_armed was set but never used` 为预先存在, 与本次修改无关。
- 全项目搜索确认: `right_angle_fallback_timer` 和5个 `RIGHT_ANGLE_FALLBACK_*` 宏无任何残留引用。

## 测试建议
- 实车验证右直角(type=1)应右转90°, 左直角(type=2)应左转90°, 方向不可反向。
- 串口观察 `D2` 页 `gDiff` 字段: 直角执行期间应有平滑爬升/回零, 正常循迹时应为0 (不再有残留衰减)。
- 若直角方向仍反, 检查 `GYRO_YAW_DIRECTION` (track_config.h:52) 和电机接线。
