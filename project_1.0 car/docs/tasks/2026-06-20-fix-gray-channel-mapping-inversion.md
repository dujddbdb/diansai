# 2026-06-20 - 修复灰度通道映射反转导致循迹摇头

## 日期
- 2026-06-20

## 涉及文件
- `bsp/grayscale.h` — GRAY_DIRECTION 0→1
- `app/main.c` — white_ref/black_ref 数组反转 (ir_weight 保持原值)
- `bsp/track.c` — 直角检测 Phase 0/2 type 交换 (ir_raw 模式保持原样)

## 修改前状态
- `GRAY_DIRECTION=0`，代码假设 `ir_raw[0]`=物理最左
- 实车串口输出: `IR:11111100` 但黑线在物理左边 → 通道映射反转
- 物理 CH8/CH7 在左边（传感器安装被翻转180°），但代码以为 CH0/CH1 在左边
- 导致误差计算方向相反 → 车反方向打 → 来回振荡摇头

## 根因
灰度传感器物理安装方向被翻转 180°（物理 CH8/CH7 在左边，CH1/CH0 在右边）。
GRAY_DIRECTION=0 假设 CH0 在左边，需要 GRAY_DIRECTION=1 来纠正物理→逻辑的映射。

## 修改内容

### 1. grayscale.h
- `GRAY_DIRECTION` 从 0 改为 1
- 效果: `Gray_ReadAllCh()` 中 `result[7-ch]=data`
  - Analog_value[0] = 物理 CH8 = 左边 (因物理翻转)
  - Analog_value[7] = 物理 CH1 = 右边

### 2. main.c
- **ir_weight 保持原值不变**: `{-7,-5,-3,-1,1,3,5,7}`
  - GRAY_DIRECTION 已处理物理→逻辑映射，ir_weight 只跟逻辑索引走
  - ir_weight[0]=-7=左, ir_weight[7]=+7=右 ✓
- **white_ref 反转**: `{2400,...,1820}` → `{1820,...,2400}`
  - Calibrated_white[0] 需要物理 CH8 的值(1820), Calibrated_white[7] 需要物理 CH1 的值(2400)
- **black_ref 反转**: 同上理由

### 3. track.c — 直角检测
- ir_raw 检测模式保持原样，仅交换 type 赋值:
  - 左直角: `ir_raw[0]==0 && ir_raw[1]==0` → type=2 (左转, yaw+80°)
  - 右直角: `ir_raw[6]==0 && ir_raw[7]==0` → type=1 (右转, yaw-80°)
- Phase 0 和 Phase 2 同步修改

## 修改后映射 (关键!)
```
物理位置:    左 ←────────────────────────────────→ 右
传感器:      CH8   CH7   CH6   CH5   CH4   CH3   CH2   CH1
Analog idx:  [0]   [1]   [2]   [3]   [4]   [5]   [6]   [7]
ir_raw:      [0]   [1]   [2]   [3]   [4]   [5]   [6]   [7]
ir_weight:   -7    -5    -3    -1    +1    +3    +5    +7
串口IR首位:   a     b     c     d     e     f     g     h
```

GRAY_DIRECTION=1 在 Gray_ReadAllCh 做 result[7-ch]=data:
- ch=0(物理CH1) → result[7] → Analog_value[7]=物理CH1=右边
- ch=7(物理CH8) → result[0] → Analog_value[0]=物理CH8=左边

## 测试建议
- 遮挡物理最左侧 CH8，串口应显示第一位=0: `IR:0xxxxxxx`
- 遮挡物理最右侧 CH1，串口应显示最后一位=0: `IR:xxxxxxx0`
- 直道循迹: 黑线偏左→ERR负数→L<R(左转纠偏); 偏右→ERR正数→L>R(右转纠偏)
- 左直角入口: 串口 `00111111` → type=2 左转
- 右直角入口: 串口 `11111100` → type=1 右转
- 如果检测类型正确但转向方向反，检查电机左右输出线序
