#ifndef __VISION_STRATEGY_H__
#define __VISION_STRATEGY_H__

#include "vision_config.h"
#include "stdbool.h"

typedef enum {
    ROI_FULL   = 0,
    ROI_SMALL  = 1,
    ROI_EXPAND = 2
} ROI_State_t;

typedef struct {
    ROI_State_t roi_state;       // 当前ROI状态
    uint16_t    lock_count;      // 连续锁定帧数
    uint16_t    lose_count;      // 连续丢失帧数
    float       gimbal_x_angle;  // 云台X轴角度
    float       gimbal_y_angle;  // 云台Y轴角度
    uint8_t     scanning;        // 是否在扫描中
} VisionStrategy_t;

void VisionStrategy_Init(void);
void VisionStrategy_Update(uint8_t target_detected);
ROI_State_t VisionStrategy_GetROIState(void);
VisionStrategy_t* VisionStrategy_GetState(void);
void VisionStrategy_SetGimbalAngle(float x_angle, float y_angle);
void VisionStrategy_GimbalScan(void);
void VisionStrategy_Reset(void);

#endif
