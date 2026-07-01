#ifndef __GIMBAL_PID_H__
#define __GIMBAL_PID_H__

#include "vision_config.h"

// 单轴PID控制器结构体
typedef struct {
    float kp_large;               // 大误差比例系数
    float ki_large;               // 大误差积分系数
    float kd_large;               // 大误差微分系数
    float kp_small;               // 小误差比例系数
    float ki_small;               // 小误差积分系数
    float kd_small;               // 小误差微分系数
    float kp_runtime;             // 当前运行比例系数
    float ki_runtime;             // 当前运行积分系数
    float kd_runtime;             // 当前运行微分系数
    float error;                  // 当前误差
    float last_error;             // 上一次误差
    float integral;               // 积分项
    float derivative_filtered;    // 滤波后的微分项
    float output;                 // PID输出值
    float last_output;            // 上一次输出值
    float output_max;             // 输出最大值限制
    float int_separation_threshold; // 积分分离阈值
    float pixel_to_angle;         // 像素转角度系数
    uint8_t motor_addr;           // 电机地址
    uint16_t rpm;                 // 电机转速 (RPM)
    uint8_t acc;                  // 电机加速度
} GimbalPID_t;

// 双轴PID控制器结构体 (云台X+Y轴)
typedef struct {
    GimbalPID_t x_axis;           // X轴PID控制器
    GimbalPID_t y_axis;           // Y轴PID控制器
    float yaw_delta;              // IMU偏航角增量 (度)
    float pitch_delta;            // IMU俯仰角增量 (度)
    float yaw_rate;               // IMU偏航角速度 (度/秒)
    float pitch_rate;             // IMU俯仰角速度 (度/秒)
    float imu_dt_s;               // IMU采样周期 (秒)
    float compensation_factor;    // IMU补偿系数
    float rate_feedforward_factor; // 角速度前馈系数
    uint8_t compensation_enabled; // IMU补偿使能标志: 1-使能, 0-禁用
} GimbalDualPID_t;

// 云台PID调试状态结构体
typedef struct {
    float output_x_deg;           // X轴输出角度 (度)
    float output_y_deg;           // Y轴输出角度 (度)
    float yaw_feedforward_deg;    // 偏航前馈角度 (度)
    float pitch_feedforward_deg;  // 俯仰前馈角度 (度)
    uint8_t dir_x;                // X轴方向
    uint8_t dir_y;                // Y轴方向
    uint32_t pulses_x;            // X轴脉冲数
    uint32_t pulses_y;            // Y轴脉冲数
} GimbalDebugState_t;

// 初始化单轴PID控制器
// 参数: pid - PID控制器指针
//       output_max - 输出最大值限制
//       motor_addr - 电机地址
// 返回值: 无
void GimbalPID_Init(GimbalPID_t *pid, float output_max, uint8_t motor_addr);

// 初始化双轴PID控制器
// 参数: dual_pid - 双轴PID控制器指针
// 返回值: 无
void GimbalDualPID_Init(GimbalDualPID_t *dual_pid);

// 计算单轴PID输出
// 参数: pid - PID控制器指针
//       error_px - 像素误差
// 返回值: float - PID输出值
float GimbalPID_Calculate(GimbalPID_t *pid, float error_px);

// 更新双轴PID输出
// 参数: dual_pid - 双轴PID控制器指针
//       error_x_px - X轴像素误差
//       error_y_px - Y轴像素误差
// 返回值: 无
void GimbalDualPID_Update(GimbalDualPID_t *dual_pid, float error_x_px, float error_y_px);

// 更新双轴PID前馈补偿
// 参数: dual_pid - 双轴PID控制器指针
// 返回值: 无
void GimbalDualPID_UpdateFeedforward(GimbalDualPID_t *dual_pid);

// 清空单轴PID积分项
// 参数: pid - PID控制器指针
// 返回值: 无
void GimbalPID_ClearIntegral(GimbalPID_t *pid);

// 清空双轴PID积分项
// 参数: dual_pid - 双轴PID控制器指针
// 返回值: 无
void GimbalDualPID_ClearIntegral(GimbalDualPID_t *dual_pid);

// 设置PID参数 (大误差和小误差两组)
// 参数: pid - PID控制器指针
//       kp_large - 大误差比例系数
//       ki_large - 大误差积分系数
//       kd_large - 大误差微分系数
//       kp_small - 小误差比例系数
//       ki_small - 小误差积分系数
//       kd_small - 小误差微分系数
// 返回值: 无
void GimbalPID_SetParams(GimbalPID_t *pid,
                         float kp_large, float ki_large, float kd_large,
                         float kp_small, float ki_small, float kd_small);

// 设置像素转角度系数
// 参数: pid - PID控制器指针
//       factor - 像素转角度系数
// 返回值: 无
void GimbalPID_SetPixelToAngle(GimbalPID_t *pid, float factor);

// 设置双轴IMU补偿参数
// 参数: dual_pid - 双轴PID控制器指针
//       factor - 补偿系数
//       enabled - 使能标志: 1-使能, 0-禁用
// 返回值: 无
void GimbalDualPID_SetCompensation(GimbalDualPID_t *dual_pid, float factor, uint8_t enabled);

// 设置IMU角度增量
// 参数: dual_pid - 双轴PID控制器指针
//       yaw_delta - 偏航角增量 (度)
//       pitch_delta - 俯仰角增量 (度)
// 返回值: 无
void GimbalDualPID_SetIMUDelta(GimbalDualPID_t *dual_pid, float yaw_delta, float pitch_delta);

// 设置IMU前馈数据 (角度增量+角速度+采样周期)
// 参数: dual_pid - 双轴PID控制器指针
//       yaw_delta - 偏航角增量 (度)
//       pitch_delta - 俯仰角增量 (度)
//       yaw_rate - 偏航角速度 (度/秒)
//       pitch_rate - 俯仰角速度 (度/秒)
//       dt_s - 采样周期 (秒)
// 返回值: 无
void GimbalDualPID_SetIMUFeedforward(GimbalDualPID_t *dual_pid,
                                     float yaw_delta,
                                     float pitch_delta,
                                     float yaw_rate,
                                     float pitch_rate,
                                     float dt_s);

// 获取云台调试状态结构体指针
// 参数: 无
// 返回值: const GimbalDebugState_t* - 调试状态结构体只读指针
const GimbalDebugState_t* GimbalDualPID_GetDebugState(void);

#endif
