#ifndef __VISION_H__
#define __VISION_H__

#include "stm32f4xx.h"
#include "stdbool.h"
#include "gimbal_pid.h"
#include "vision_strategy.h"

typedef struct {
    float    error_x;           // X轴误差(像素)
    float    error_y;           // Y轴误差(像素)
    float    error_distance;    // 误差距离(欧氏距离, 像素)
    uint8_t  target_detected;   // 目标是否检测到(0-无, 1-有)
    uint8_t  laser_on;          // 激光是否开启(0-关, 1-开)
    uint8_t  hit_streak;        // 连续命中帧数(激光触发防抖)
    int16_t  gimbal_x_angle;    // 云台X轴输出角度
    int16_t  gimbal_y_angle;    // 云台Y轴输出角度
    ROI_State_t roi_state;      // 当前ROI状态
} Vision_Context_t;

void Vision_GimbalPID_Init(void);
void Vision_GimbalPID_Update(float error_x_px, float error_y_px);
void Vision_GimbalPID_ClearIntegral(void);
void Vision_GimbalIMUCompensationTick(void);
GimbalDualPID_t* Vision_GimbalPID_GetController(void);

void Vision_TargetProcess(int16_t err_y, int16_t err_z, bool valid);
uint8_t Vision_LaserTrigger(bool target_valid);
void Vision_GetError(float *err_x, float *err_y, float *dist);

void Tracking_Init(void);
void Tracking_Update(uint8_t target_detected);
VisionStrategy_t* Tracking_GetState(void);

void Vision_Init(void);
void Vision_KeyControlTick(void);
void Vision_Process(void);
Vision_Context_t* Vision_GetContext(void);

#endif
