# 修复直角检测状态残留导致持续进入直角

**日期**: 2026-06-19

## 涉及文件
- `bsp/track.c`

## 修改前状态
- `Track_Check_Right_Angle()` 中用于判断当前帧直角方向的 `current_type` 被定义为文件级全局变量。
- 函数内原本每次进入时清零 `current_type` 的局部变量声明被注释掉。
- 当某一帧检测到左/右直角后，后续即使灰度特征不再满足直角条件，`current_type` 仍可能保留上一次的非零值，导致 Phase 0 连续计数继续累加，表现为小车“一直进入直角”。

## 修改内容和原因
- 将 `current_type` 改回 `Track_Check_Right_Angle()` 内部局部变量，并在每次调用时初始化为 `0`。
- 删除误导性的旧注释行，避免后续维护误以为该变量仍由全局状态驱动。
- 预触发超时回退时同步清零 `right_angle_confirm_cnt`，防止全白确认计数残留到下一次预触发。

## 测试建议
- 串口观察 `ph/is_right_angle/detect_cnt/confirm_cnt`：直线段应保持 `ph=0`、`is_right_angle=0`，不应自行进入 `ph=2` 或 `ph=1`。
- 手推经过真实直角入口：应先连续满足特征后进入 `ph=2`，看到全白确认后再进入 `ph=1`。
- 如果仍误触发，优先继续调大 `RIGHT_ANGLE_DETECT_CNT`，或收紧直角入口灰度特征。
