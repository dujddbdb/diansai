#ifndef __VISION_H__
#define __VISION_H__

#include "stm32f4xx.h"
#include "stdbool.h"
#include "gimbal_pid.h"
#include "vision_strategy.h"

typedef struct {
    float    error_x;
    float    error_y;
    float    error_distance;
    uint8_t  target_detected;
    uint8_t  laser_on;
    uint8_t  hit_streak;
    int16_t  gimbal_x_angle;
    int16_t  gimbal_y_angle;
    ROI_State_t roi_state;
} Vision_Context_t;

// 初始化视觉系统PID控制器
void Vision_GimbalPID_Init(void);
// 更新视觉PID输出，error_x_px: X轴像素误差，error_y_px: Y轴像素误差
void Vision_GimbalPID_Update(float error_x_px, float error_y_px);
// 清空视觉PID积分项
void Vision_GimbalPID_ClearIntegral(void);
// IMU补偿定时调用，丢包时维持补偿
void Vision_IMUFeedforward1kTick(void);
void Vision_GimbalIMUCompensationTick(void);
// 获取云台PID控制器指针，返回双轴PID结构体指针
GimbalDualPID_t* Vision_GimbalPID_GetController(void);

// 处理目标检测数据，err_y: X轴像素误差，err_z: Y轴像素误差，valid: 目标是否有效
void Vision_TargetProcess(int16_t err_y, int16_t err_z, bool valid);
// 激光触发判断，target_valid: 当前是否有有效目标，返回1-激光开启 0-激光关闭
uint8_t Vision_LaserTrigger(bool target_valid);
// 获取当前误差值，err_x: 输出X轴误差，err_y: 输出Y轴误差，dist: 输出误差距离
void Vision_GetError(float *err_x, float *err_y, float *dist);

// 初始化追踪策略模块
void Tracking_Init(void);
// 更新追踪状态，target_detected: 0-未检测到目标 1-检测到目标
void Tracking_Update(uint8_t target_detected);
// 获取追踪策略状态结构体指针
VisionStrategy_t* Tracking_GetState(void);

// 初始化整个视觉系统
void Vision_Init(void);
// 按键控制定时处理，切换视觉模式
void Vision_KeyControlTick(void);
// 视觉主处理循环，每帧调用
void Vision_Process(void);
// 获取视觉上下文状态结构体指针
Vision_Context_t* Vision_GetContext(void);

#endif
