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
    float derivative_filtered;
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
    float yaw_rate;
    float pitch_rate;
    float imu_dt_s;
    float compensation_factor;
    float rate_feedforward_factor;
    uint8_t compensation_enabled;
} GimbalDualPID_t;

typedef struct {
    float output_x_deg;
    float output_y_deg;
    float yaw_feedforward_deg;
    float pitch_feedforward_deg;
    uint8_t dir_x;
    uint8_t dir_y;
    uint32_t pulses_x;
    uint32_t pulses_y;
} GimbalDebugState_t;

void GimbalPID_Init(GimbalPID_t *pid, float output_max, uint8_t motor_addr);
void GimbalDualPID_Init(GimbalDualPID_t *dual_pid);
float GimbalPID_Calculate(GimbalPID_t *pid, float error_px);
void GimbalDualPID_Update(GimbalDualPID_t *dual_pid, float error_x_px, float error_y_px);
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
