#ifndef __TRACK_CONFIG_H__
#define __TRACK_CONFIG_H__

// 基础巡线参数
#define BASE_RPM             80.0f   // 基础巡线转速(RPM)
#define STRAIGHT_RPM_LIMIT   150.0f  // 直道最高转速限制(RPM)
#define RAMP_STARTUP         0.1f    // 启动加速度斜率(RPM/ms)

// 巡线位置PID参数
#define KP_NORMAL            0.055f  // 巡线PID比例系数
#define KI_NORMAL            0.0f    // 巡线PID积分系数
#define KD_NORMAL            0.20f   // 巡线PID微分系数
#define ERR_FILTER_ALPHA     0.9f    // 巡线误差低通滤波系数(0-1, 1=无滤波)
#define PID_FILTER_ALPHA     0.80f   // 巡线PID输出低通滤波系数

// 灰度传感器配置
#define GRAY_DIRECTION       1       // 0-通道0对应传感器1(左到右) 1-反向(通道0对应传感器8)
#define GRAY_THRESHOLD_SHIFT 0.6f    // 灰度二值化阈值偏移量(0-1)
#define GRAY_ANALOG_EMA_ENABLE        1     // 0-关闭模拟量EMA滤波 1-开启
#define GRAY_ANALOG_EMA_PREV_WEIGHT   7     // EMA旧值权重(越大越平滑)
#define GRAY_ANALOG_EMA_NEW_WEIGHT    3     // EMA新值权重(越大响应越快)
#define GRAY_ANALOG_EMA_TOTAL_WEIGHT  10    // EMA权重总和(PREV+NEW)

// 直角检测参数
#define RIGHT_ANGLE_BLACK_CONFIRM_SAMPLES   8U    // 黑电平确认采样数(去抖)
#define RIGHT_ANGLE_FEATURE_CONFIRM_SAMPLES 1U    // 直角特征确认采样数
#define RIGHT_ANGLE_WHITE_CONFIRM_SAMPLES   2U    // 全白确认采样数
#define RIGHT_ANGLE_WHITE_MIN               8U    // 判定全白所需最少白通道数

// 直角转弯控制参数
#define CORNER_YAW_TARGET             90.0f   // 转弯目标偏航角度(度)
#define CORNER_GRAY_BLEND_START_DEG   80.0f   // 80度开始灰度渐变接管
#define CORNER_IMU_EXIT_DEG           85.0f   // 85度退出直角状态
#define KP_CORNER_YAW                 1.85f   // 转弯偏航PID比例系数
#define KD_CORNER_YAW                 22.0f   // 转弯偏航PID微分系数
#define CORNER_TURN_RPM               40.0f   // 转弯基础转速(RPM)
#define CORNER_MIN_RPM                20.0f   // 转弯最低转速(RPM)
#define CORNER_MAX_RPM                92.0f   // 转弯最高转速(RPM)

// 转弯过渡曲线参数
#define CORNER_PID_SMOOTH_ALPHA           0.30f   // 转弯PID平滑滤波系数
#define CORNER_PREVIEW_DECEL_RPM          16.0f   // 预览减速转速量(RPM)
#define CORNER_ENTRY_BLEND_PROGRESS       0.30f   // IMU差速在前20%角度进度内平滑拉起
#define CORNER_SPEED_EXIT_START_PROGRESS  0.55f   // 出弯加速起始进度(0-1)
#define CORNER_SPEED_EXIT_BLEND_WIDTH     0.45f   // 出弯加速过渡宽度
#define CORNER_GYRO_DIFF_LIMIT_RPM        65.0f   // 转弯陀螺仪差速限幅(RPM)
#define CORNER_DIFF_SLEW_UP_RPM_PER_MS    1.15f   // 转弯差速上升斜率(RPM/ms)
#define CORNER_BASE_SLEW_DOWN_RPM_PER_MS  0.90f   // 转弯基础转速下降斜率(RPM/ms)
#define CORNER_BASE_SLEW_UP_RPM_PER_MS    0.25f   // 转弯基础转速上升斜率(RPM/ms)
#define WHEEL_COMMAND_SLEW_RPM_PER_MS     2.20f   // 轮指令斜率限制(RPM/ms)

// 陀螺仪参数
#define GYRO_YAW_DIRECTION    1           // 陀螺仪偏航方向系数(±1)
#define GYRO_STRAIGHT_DAMPING_ENABLE  1   // 0-关闭直道陀螺仪阻尼 1-开启
#define KD_GYRO_STRAIGHT              5.5f   // 直道陀螺仪阻尼微分系数
#define GYRO_STRAIGHT_LIMIT           15.0f  // 直道陀螺仪阻尼输出限幅(RPM)
#define GYRO_MAIN_POLL_PERIOD_MS      10U    // 陀螺仪主轮询周期(ms)

// 速度环PID参数
#define KP_VELOCITY           20.233530f  // 速度环PID比例系数
#define KI_VELOCITY           8.702225f   // 速度环PID积分系数
#define KD_VELOCITY           0.17609f    // 速度环PID微分系数
#define VELOCITY_PWM_LIMIT    500         // 速度环PWM输出限幅
#define INTEGRAL_LIMIT        600         // 速度环积分限幅
#define LEFT_RPM_CORRECTION   1.0f        // 左轮转速修正系数
#define RIGHT_RPM_CORRECTION  1.0f        // 右轮转速修正系数

// 编码器参数
#define ENCODER_LINE        11U         // 编码器线数
#define REDUCTION_RATIO     30U         // 电机减速比
#define ENCODER_MULTIPLE    4U          // 编码器倍频
#define PULSE_PER_ROUND     (ENCODER_LINE * REDUCTION_RATIO * ENCODER_MULTIPLE) // 每圈脉冲数
#define SAMPLE_TIME         0.005f      // 速度采样周期(s)
#define RPM_COEFFICIENT     (60.0f / ((float)PULSE_PER_ROUND * SAMPLE_TIME)) // RPM转换系数

// 电机/编码器方向配置
#define ENCODER_LEFT_INVERT   0 // 0-不反转左轮编码器 1-反转
#define ENCODER_RIGHT_INVERT  1 // 0-不反转右轮编码器 1-反转
#define LR_SWAP              0 // 0-不交换左右轮 1-交换
#define LEFT_MOTOR_REVERSE   0 // 0-不反转左轮电机 1-反转
#define RIGHT_MOTOR_REVERSE  0 // 0-不反转右轮电机 1-反转

// 圈数计数参数
#define TRACK_CORNERS_PER_LAP         4U  // 每圈直角弯数量
#define TRACK_TARGET_LAPS_MAX         9U  // 最大目标圈数

#endif
