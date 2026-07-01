#ifndef __VISION_CONFIG_H__
#define __VISION_CONFIG_H__

#include "stm32f4xx.h"

/* ==================== PID参数 ==================== */
#define PID_ERROR_BLEND_START          5.0f    // PID参数混合起始误差(像素)，小于此值用小增益
#define PID_ERROR_BLEND_END            60.0f   // PID参数混合结束误差(像素)，大于此值用大增益
#define PID_GAIN_SLEW_FACTOR           0.22f   // PID增益渐变系数，调大→参数切换更快，调小→更平滑

#define PID_KP_LARGE                   10.0f   // 大误差比例增益，调大→响应更快但易超调
#define PID_KI_LARGE                   0.0f    // 大误差积分增益，调大→消静差快但易振荡
#define PID_KD_LARGE                   100.1f  // 大误差微分增益，调大→抑制超调但易放大噪声

#define PID_KP_SMALL                   0.1f    // 小误差比例增益，调大→精调更快但易抖动
#define PID_KI_SMALL                   0.0f    // 小误差积分增益，调大→消静差快但易振荡
#define PID_KD_SMALL                   50.1f   // 小误差微分增益，调大→抑制抖动但易放大噪声

#define PID_INT_SEPARATION_THRESHOLD   500     // 积分分离阈值(像素)，误差超此值关闭积分防饱和
#define GIMBAL_PIXEL_DEADBAND          1.01f   // 像素死区，误差小于此值视为无误差，调大→更稳定但精度低
#define GIMBAL_INTEGRAL_LIMIT_DEG      0.01f   // 积分限幅(度)，防止积分项过大超调，调大→积分作用强
#define PID_OUTPUT_MAX_X               1.0f    // X轴PID输出最大值(度)，限制单次最大调整量
#define PID_OUTPUT_MAX_Y               1.0f    // Y轴PID输出最大值(度)，限制单次最大调整量
#define PIXEL_TO_ANGLE_FACTOR          0.001f  // 像素转角度系数，调大→相同像素误差对应更大角度
#define GIMBAL_DERIVATIVE_FILTER_ALPHA 0.35f   // 微分项滤波系数，调大→微分响应快但噪声大
#define GIMBAL_OUTPUT_SLEW_DEG         0.65f   // 输出平滑限速(度)，相邻两次输出最大变化量，调大→响应快

/* ==================== 步进电机配置 ==================== */
#define GIMBAL_ADDR_X                  0x01    // X轴电机地址
#define GIMBAL_ADDR_Y                  0x01    // Y轴电机地址
#define GIMBAL_RPM_DEFAULT             1.0     // 默认转速(RPM)
#define GIMBAL_ACC_DEFAULT             1       // 默认加速度
#define GIMBAL_PULSES_PER_REV          3200.0f // 每转脉冲数
#define GIMBAL_DEGREES_PER_REV         360.0f  // 每转角度数
#define GIMBAL_PULSES_PER_DEG          (GIMBAL_PULSES_PER_REV / GIMBAL_DEGREES_PER_REV)  // 每度脉冲数

/* ==================== IMU前馈补偿 ==================== */
#define GIMBAL_COMPENSATION_FACTOR     0.55f   // IMU补偿系数，调大→补偿更强但易过补偿
#define GIMBAL_YAW_DELTA_THRESHOLD     0.0f    // 偏航角变化阈值，小于此值不补偿
#define GIMBAL_PITCH_DELTA_THRESHOLD   0.0f    // 俯仰角变化阈值，小于此值不补偿
#define GIMBAL_RATE_FEEDFORWARD_FACTOR 0.045f  // 角速度前馈系数，调大→动态补偿更强
#define GIMBAL_FEEDFORWARD_MAX_DEG     0.80f   // 前馈补偿最大值(度)，限制单次补偿量

#define VISION_IMU_KALMAN_Q            0.02f   // 卡尔曼滤波过程噪声Q，调大→更相信预测值
#define VISION_IMU_KALMAN_RATE_Q       0.12f   // 卡尔曼滤波角速度过程噪声Q
#define VISION_IMU_KALMAN_R            0.90f   // 卡尔曼滤波测量噪声R，调大→更平滑但滞后
#define VISION_IMU_DELTA_LIMIT_DEG     3.0f    // IMU角度变化限制(度)，防止异常值跳变
#define VISION_IMU_MIN_CORNER_BLEND    0.08f   // 最小转弯混合系数，低于此值不启用IMU补偿
#define VISION_IMU_DT_MIN_MS           1U      // 卡尔曼滤波最小时间间隔(ms)
#define VISION_IMU_DT_MAX_MS           40U     // 卡尔曼滤波最大时间间隔(ms)

/* ==================== 激光触发 ==================== */
#define VISION_LASER_THRESHOLD_PX      30      // 激光触发误差阈值(像素)，误差小于此值才允许开火
#define VISION_LASER_HOLD_FRAMES       3       // 激光触发防抖帧数，连续命中N帧才触发，调大→更稳但延迟高

/* ==================== 云台动态调速 ==================== */
#define VISION_GIMBAL_RPM_MIN          20.0f   // 云台最低转速(RPM)，调小→低速更稳但响应慢
#define VISION_GIMBAL_RPM_MAX          120.0f  // 云台最高转速(RPM)，调大→高速响应快但易过冲
#define VISION_GIMBAL_RPM_SLEW         0.25f   // 转速渐变系数，调大→转速切换更快
#define VISION_CORNER_RPM_REDUCE       0.35f   // 转弯时转速降低比例，调大→转弯减速更多

/* ==================== 圆形扫描 ==================== */
#define VISION_CIRCLE_RADIUS_PX        26.0f   // 圆形扫描半径(像素)，调大→扫描范围大
#define VISION_CIRCLE_STEP_RAD         0.12f   // 圆形扫描角步长(弧度)，调大→扫描快但粗糙

/* ==================== ROI扫描策略 ==================== */
#define VISION_SCAN_X_MIN              (-30.0f) // 扫描X轴最小值(度)
#define VISION_SCAN_X_MAX              ( 30.0f) // 扫描X轴最大值(度)
#define VISION_SCAN_Y_MIN              (-15.0f) // 扫描Y轴最小值(度)
#define VISION_SCAN_Y_MAX              ( 15.0f) // 扫描Y轴最大值(度)
#define VISION_SCAN_SPEED              (5.0f)   // 扫描速度(度/步)，调大→扫得快但易漏目标

#define VISION_LOCK_FRAMES             5       // 锁定帧数，连续命中N帧切换到小ROI
#define VISION_LOSE_FRAMES             10      // 丢失帧数，丢失N帧切换到扩展ROI
#define VISION_FULL_FRAMES             30      // 全图帧数，丢失N帧切换到全图扫描
#define VISION_SCAN_INTERVAL_MS        200     // 扫描步进间隔(ms)，调小→扫描快

#endif
