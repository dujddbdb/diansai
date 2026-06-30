#ifndef __VISION_STRATEGY_H__
#define __VISION_STRATEGY_H__

#include "vision_config.h"
#include "stdbool.h"

typedef enum {
    ROI_FULL   = 0,  // 0-全图搜索
    ROI_SMALL  = 1,  // 1-小ROI精确锁定
    ROI_EXPAND = 2   // 2-扩展ROI搜索
} ROI_State_t;

typedef struct {
    ROI_State_t roi_state;
    uint16_t    lock_count;
    uint16_t    lose_count;
    float       gimbal_x_angle;
    float       gimbal_y_angle;
    uint8_t     scanning;
} VisionStrategy_t;

// 初始化视觉追踪策略状态机
void VisionStrategy_Init(void);
// 每帧更新ROI状态机，target_detected: 0-未检测到目标 1-检测到目标
void VisionStrategy_Update(uint8_t target_detected);
// 获取当前ROI状态，返回ROI_State_t枚举值
ROI_State_t VisionStrategy_GetROIState(void);
// 获取完整策略状态结构体指针
VisionStrategy_t* VisionStrategy_GetState(void);
// 设置云台当前角度反馈，x_angle: X轴角度(度)，y_angle: Y轴角度(度)
void VisionStrategy_SetGimbalAngle(float x_angle, float y_angle);
// 执行云台蛇形扫描搜索
void VisionStrategy_GimbalScan(void);
// 重置策略状态机为初始状态
void VisionStrategy_Reset(void);

#endif
