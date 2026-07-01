#ifndef __GIMBAL_PID_H__
#define __GIMBAL_PID_H__

#include "vision_config.h"

typedef struct {
    float kp_large;               // 大误差比例增益
    float ki_large;               // 大误差积分增益
    float kd_large;               // 大误差微分增益
    float kp_small;               // 小误差比例增益
    float ki_small;               // 小误差积分增益
    float kd_small;               // 小误差微分增益
    float kp_runtime;             // 当前运行时比例增益(渐变混合)
    float ki_runtime;             // 当前运行时积分增益(渐变混合)
    float kd_runtime;             // 当前运行时微分增益(渐变混合)
    float error;                  // 当前误差(像素)
    float last_error;             // 上一帧误差(像素)
    float integral;               // 积分项(角度)
    float derivative_filtered;    // 滤波后的微分项
    float output;                 // PID输出(度)
    float last_output;            // 上一帧输出(度)
    float output_max;             // 输出最大值限制(度)
    float int_separation_threshold; // 积分分离阈值(像素)
    float pixel_to_angle;         // 像素转角度系数
    uint8_t motor_addr;           // 电机地址
    uint16_t rpm;                 // 当前转速(RPM)
    uint8_t acc;                  // 加速度
} GimbalPID_t;

typedef struct {
    GimbalPID_t x_axis;           // X轴PID控制器
    GimbalPID_t y_axis;           // Y轴PID控制器
    float yaw_delta;              // IMU偏航角变化量(度)
    float pitch_delta;            // IMU俯仰角变化量(度)
    float yaw_rate;               // IMU偏航角速度(度/秒)
    float pitch_rate;             // IMU俯仰角速度(度/秒)
    float imu_dt_s;               // IMU时间间隔(秒)
    float compensation_factor;    // IMU补偿系数
    float rate_feedforward_factor; // 角速度前馈系数
    uint8_t compensation_enabled; // IMU补偿是否启用
} GimbalDualPID_t;

typedef struct {
    float output_x_deg;           // X轴输出角度
    float output_y_deg;           // Y轴输出角度
    float yaw_feedforward_deg;    // 偏航前馈补偿量
    float pitch_feedforward_deg;  // 俯仰前馈补偿量
    uint8_t dir_x;                // X轴方向
    uint8_t dir_y;                // Y轴方向
    uint32_t pulses_x;            // X轴脉冲数
    uint32_t pulses_y;            // Y轴脉冲数
} GimbalDebugState_t;

void GimbalPID_Init(GimbalPID_t *pid, float output_max, uint8_t motor_addr);
void GimbalDualPID_Init(GimbalDualPID_t *dual_pid);
float GimbalPID_Calculate(GimbalPID_t *pid, float error_px);
void GimbalDualPID_Update(GimbalDualPID_t *dual_pid, float error_x_px, float error_y_px);
void GimbalDualPID_UpdateFeedforward(GimbalDualPID_t *dual_pid);
void GimbalPID_ClearIntegral(GimbalPID_t *pid);
void GimbalDualPID_ClearIntegral(GimbalDualPID_t *dual_pid);
void GimbalPID_SetParams(GimbalPID_t *pid,
                         float kp_large, float ki_large, float kd_large,
                         float kp_small, float ki_small, float kd_small);
void GimbalPID_SetPixelToAngle(GimbalPID_t *pid, float factor);
void GimbalDualPID_SetCompensation(GimbalDualPID_t *dual_pid, float factor, uint8_t enabled);
void GimbalDualPID_SetIMUDelta(GimbalDualPID_t *dual_pid, float yaw_delta, float pitch_delta);
void GimbalDualPID_SetIMUFeedforward(GimbalDualPID_t *dual_pid,
                                     float yaw_delta,
                                     float pitch_delta,
                                     float yaw_rate,
                                     float pitch_rate,
                                     float dt_s);
const GimbalDebugState_t* GimbalDualPID_GetDebugState(void);

#endif
