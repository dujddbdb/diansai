# 2026-06-20 - 直角转弯逻辑重写：陀螺仪PID差速+状态机优化

## 日期
- 2026-06-20

## 涉及文件
- `bsp/track_config.h` — 直角转弯参数宏定义
- `bsp/track.h` — 删除ramp斜坡相关extern声明
- `bsp/track.c` — Track_Check_Right_Angle重写, Track_Action_Execute修改, 删除ramp变量
- `app/main.c` — 删除ramp_active串口打印引用

## 修改前状态
- 直角转弯参数: CORNER_YAW_TARGET=90°, KP_CORNER_YAW=1.0, KD_CORNER_YAW=0.2, CORNER_MIN_RPM=10, CORNER_MAX_RPM=120
- Track_Action_Execute: 直角执行用BASE_RPM作基础转速, 无CORNER_TURN_RPM
- 直角执行条件: `is_right_angle != 0` (不区分phase)
- 无陀螺仪时无固定差速回退, 无超时机制
- 存在ramp斜坡变量(ramp_start_L/R, ramp_active)但未使用
- main.c串口打印引用了ramp_active

## 修改内容和原因

### 1. track_config.h — 新增陀螺仪直角参数
- 替换原直角参数区域, 新增:
  - `CORNER_YAW_TARGET=80.0f` (从90°降到80°, 更早退出)
  - `KP_CORNER_YAW=10.0f` (从1.0大幅提高, 增强转向响应)
  - `KD_CORNER_YAW=5.0f` (从0.2提高, 抑制超调)
  - `CORNER_TURN_RPM=80.0f` (转弯基准转速, 替代BASE_RPM)
  - `CORNER_DECEL_ANGLE=5.0f` (减速区角度, 预留)
  - `CORNER_DECEL_RPM=50.0f` (减速区转速, 预留)
  - `CORNER_MIN_RPM=30.0f` (从10提高, 防堵转)
  - `CORNER_MAX_RPM=100.0f` (从120降低, 限速)

### 2. Track_Check_Right_Angle — 状态机重写
- Phase 0: 直角特征检测顺序改为先右(type=1: 11XXXX00)再左(type=2: 00XXXX11)
- Phase 2: 每次重新检测直角特征更新方向; 未检测到全白重置确认计数; 无超时
- Phase 1: 陀螺仪模式用`#if GYRO_ENABLE`条件编译; 无陀螺仪超时从80ms改为4000ms
- Phase 3: 重置所有状态, 清零PID_SumErr, 设置global_detect_locked

### 3. Track_Action_Execute — 陀螺仪PID差速
- 直角执行条件改为 `is_right_angle != 0 && right_angle_phase == 1` (只在执行阶段)
- 陀螺仪模式: 用CORNER_TURN_RPM作基准, KP_CORNER_YAW/KD_CORNER_YAW计算差速
- 无陀螺仪回退: 固定差速(OUTER/INNER_CORNER_RPM)
- 保留正常循迹的软启动ramp_timer/ramp_scale不变

### 4. 删除ramp斜坡逻辑
- 删除track.c中ramp_start_L/R, ramp_active变量定义
- 删除track.h中ramp_start_L/R, ramp_active的extern声明
- 修改main.c串口打印格式, 删除ra%d和ramp_active参数

### 5. 无积分清零
- 确认Track_Check_Right_Angle的Phase 2→Phase 1转换中没有清零Left_Integral/Right_Integral

## 测试建议
- 先用RIGHT_ANGLE_DETECT_CNT=1快速测试直角触发是否正常
- 观察陀螺仪直角转弯时左右轮RPM差速是否合理(KP=10较大, 注意过冲)
- 如果转弯不足: 增大CORNER_YAW_TARGET或KP_CORNER_YAW
- 如果转弯过冲: 增大KD_CORNER_YAW或减小KP_CORNER_YAW
- 如果转弯速度太慢: 增大CORNER_TURN_RPM
- 正式赛道前将RIGHT_ANGLE_DETECT_CNT改回40
- 注意CORNER_MIN_RPM从10改为30, 低速时电机可能不会完全停止
