#ifndef __GIMBAL_PID_H__
#define __GIMBAL_PID_H__

#include "vision_config.h"

typedef struct {
    float kp_large;
    float ki_large;
    float kd_large;
    float kp_small;
    float ki_small;
    float kd_small;
    float kp_runtime;
    float ki_runtime;
    float kd_runtime;
    float error;
    float last_error;
    float integral;
    float output;
    float last_output;
    float output_max;
    float int_separation_threshold;
    float pixel_to_angle;
    uint8_t motor_addr;
    uint16_t rpm;
    uint8_t acc;
} GimbalPID_t;

typedef struct {
    GimbalPID_t x_axis;
    GimbalPID_t y_axis;
    float yaw_delta;
    float pitch_delta;
    float compensation_factor;
    uint8_t compensation_enabled;
} GimbalDualPID_t;

typedef struct {
    float output_x_deg;
    float output_y_deg;
    uint8_t dir_x;
    uint8_t dir_y;
    uint32_t pulses_x;
    uint32_t pulses_y;
} GimbalDebugState_t;

// 初始化单轴PID控制器，pid: PID结构体指针，output_max: 单帧最大输出(度)，motor_addr: 电机地址
void GimbalPID_Init(GimbalPID_t *pid, float output_max, uint8_t motor_addr);
// 初始化双轴云台PID控制器，dual_pid: 双轴PID结构体指针
void GimbalDualPID_Init(GimbalDualPID_t *dual_pid);
// 单轴PID计算，pid: PID结构体指针，error_px: 像素误差，返回角度输出(度)
float GimbalPID_Calculate(GimbalPID_t *pid, float error_px);
// 双轴PID更新并控制电机，dual_pid: 双轴PID结构体，error_x_px: X轴像素误差，error_y_px: Y轴像素误差
void GimbalDualPID_Update(GimbalDualPID_t *dual_pid, float error_x_px, float error_y_px);
// 清零单轴PID积分项，pid: PID结构体指针
void GimbalPID_ClearIntegral(GimbalPID_t *pid);
// 清零双轴PID积分项，dual_pid: 双轴PID结构体指针
void GimbalDualPID_ClearIntegral(GimbalDualPID_t *dual_pid);
// 动态设置PID分段参数，pid: PID结构体指针，kp/ki/kd_large: 大误差参数，kp/ki/kd_small: 小误差参数
void GimbalPID_SetParams(GimbalPID_t *pid,
                         float kp_large, float ki_large, float kd_large,
                         float kp_small, float ki_small, float kd_small);
// 设置像素到角度转换系数，pid: PID结构体指针，factor: 转换系数(度/像素)
void GimbalPID_SetPixelToAngle(GimbalPID_t *pid, float factor);
// 设置IMU前馈补偿参数，dual_pid: 双轴PID结构体，factor: 补偿系数，enabled: 0-禁用 1-启用
void GimbalDualPID_SetCompensation(GimbalDualPID_t *dual_pid, float factor, uint8_t enabled);
// 同时设置Yaw和Pitch轴IMU增量，dual_pid: 双轴PID结构体，yaw_delta: yaw变化量(度)，pitch_delta: pitch变化量(度)
void GimbalDualPID_SetIMUDelta(GimbalDualPID_t *dual_pid, float yaw_delta, float pitch_delta);
// 获取调试状态结构体，返回只读调试状态指针
const GimbalDebugState_t* GimbalDualPID_GetDebugState(void);

#endif
