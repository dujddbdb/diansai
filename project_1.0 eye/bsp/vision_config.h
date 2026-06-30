#ifndef __VISION_CONFIG_H__
#define __VISION_CONFIG_H__

#include "stm32f4xx.h"

/* ==================== PID 参数 ==================== */

#define PID_ERROR_BLEND_START         5.0f      // 分段PID起始误差(像素)
#define PID_ERROR_BLEND_END           60.0f     // 分段PID结束误差(像素)
#define PID_GAIN_SLEW_FACTOR          2.1f      // PID参数渐变过渡系数

#define PID_KP_LARGE                  10.0f     // 大误差区比例系数
#define PID_KI_LARGE                  0.0f      // 大误差区积分系数
#define PID_KD_LARGE                  100.1f    // 大误差区微分系数

#define PID_KP_SMALL                  0.1f      // 小误差区比例系数
#define PID_KI_SMALL                  0.0f      // 小误差区积分系数
#define PID_KD_SMALL                  50.1f     // 小误差区微分系数

#define PID_INT_SEPARATION_THRESHOLD  500       // 积分分离阈值(像素)
#define GIMBAL_PIXEL_DEADBAND         1.01f     // 像素死区，小于此值停止输出
#define GIMBAL_INTEGRAL_LIMIT_DEG     0.01f     // 积分项限幅(度)
#define PID_OUTPUT_MAX_X              1.0f      // X轴单帧最大输出(度)
#define PID_OUTPUT_MAX_Y              1.0f      // Y轴单帧最大输出(度)
#define PIXEL_TO_ANGLE_FACTOR         0.001f    // 像素到角度转换系数(度/像素)

/* ==================== 云台电机配置 ==================== */

#define GIMBAL_ADDR_X                 0x01      // X轴电机驱动器地址
#define GIMBAL_ADDR_Y                 0x01      // Y轴电机驱动器地址
#define GIMBAL_RPM_DEFAULT            1.0       // 默认转速(RPM)
#define GIMBAL_ACC_DEFAULT            1         // 默认加速度
#define GIMBAL_PULSES_PER_REV         3200.0f   // 电机每圈脉冲数(微步)
#define GIMBAL_DEGREES_PER_REV        360.0f    // 每圈度数
#define GIMBAL_PULSES_PER_DEG         (GIMBAL_PULSES_PER_REV / GIMBAL_DEGREES_PER_REV) // 每度脉冲数

/* ==================== IMU前馈补偿 ==================== */

#define GIMBAL_COMPENSATION_FACTOR    0.55f     // IMU前馈补偿系数
#define GIMBAL_YAW_DELTA_THRESHOLD    0.20f     // Yaw补偿触发阈值(度)
#define GIMBAL_PITCH_DELTA_THRESHOLD  0.20f     // Pitch补偿触发阈值(度)

#define VISION_IMU_KALMAN_Q           0.02f     // IMU卡尔曼滤波过程噪声协方差
#define VISION_IMU_KALMAN_R           0.90f     // IMU卡尔曼滤波测量噪声协方差
#define VISION_IMU_DELTA_LIMIT_DEG    3.0f      // IMU单帧增量限幅(度)
#define VISION_IMU_MIN_CORNER_BLEND   0.08f     // 转弯补偿最小触发阈值

/* ==================== 视觉激光触发 ==================== */

#define VISION_LASER_THRESHOLD_PX     30        // 激光触发距离阈值(像素)
#define VISION_LASER_HOLD_FRAMES      3         // 激光连续命中帧数防抖

/* ==================== 视觉电机转速控制 ==================== */

#define VISION_GIMBAL_RPM_MIN         20.0f     // 视觉追踪电机最小转速
#define VISION_GIMBAL_RPM_MAX         120.0f    // 视觉追踪电机最大转速
#define VISION_GIMBAL_RPM_SLEW        0.25f     // 转速渐变过渡系数
#define VISION_CORNER_RPM_REDUCE      0.35f     // 车体转弯时转速衰减系数

/* ==================== 圆形扫描 ==================== */

#define VISION_CIRCLE_RADIUS_PX       26.0f     // 圆形扫描半径(像素)
#define VISION_CIRCLE_STEP_RAD        0.12f     // 圆形扫描角步长(弧度)

/* ==================== ROI扫描策略 ==================== */

#define VISION_SCAN_X_MIN             (-30.0f)  // X轴扫描最小角度(度)
#define VISION_SCAN_X_MAX             ( 30.0f)  // X轴扫描最大角度(度)
#define VISION_SCAN_Y_MIN             (-15.0f)  // Y轴扫描最小角度(度)
#define VISION_SCAN_Y_MAX             ( 15.0f)  // Y轴扫描最大角度(度)
#define VISION_SCAN_SPEED             (5.0f)    // 蛇形扫描步进角度(度)

#define VISION_LOCK_FRAMES            5         // 连续锁定帧数阈值，切换到小ROI
#define VISION_LOSE_FRAMES            10        // 连续丢失帧数阈值，切换到扩展ROI
#define VISION_FULL_FRAMES            30        // 连续丢失帧数阈值，切换全图+扫描
#define VISION_SCAN_INTERVAL_MS       200       // 扫描动作间隔(帧)

#endif
