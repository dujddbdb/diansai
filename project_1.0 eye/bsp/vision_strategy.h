#ifndef __VISION_STRATEGY_H__
#define __VISION_STRATEGY_H__

#include "vision_config.h"
#include "stdbool.h"

// ROI搜索区域状态枚举
typedef enum {
    ROI_FULL   = 0,  // 全图搜索模式
    ROI_SMALL  = 1,  // 小ROI精确锁定模式
    ROI_EXPAND = 2   // 扩展ROI搜索模式
} ROI_State_t;

// 视觉追踪策略状态结构体
typedef struct {
    ROI_State_t roi_state;      // 当前ROI搜索状态
    uint16_t    lock_count;     // 锁定计数 (连续检测到目标帧数)
    uint16_t    lose_count;     // 丢失计数 (连续未检测到目标帧数)
    float       gimbal_x_angle; // 云台X轴角度 (度)
    float       gimbal_y_angle; // 云台Y轴角度 (度)
    uint8_t     scanning;       // 扫描标志: 1-正在蛇形扫描, 0-未扫描
} VisionStrategy_t;

// 初始化视觉追踪策略状态机
// 参数: 无
// 返回值: 无
void VisionStrategy_Init(void);

// 每帧更新ROI状态机
// 参数: target_detected - 0-未检测到目标, 1-检测到目标
// 返回值: 无
void VisionStrategy_Update(uint8_t target_detected);

// 获取当前ROI状态
// 参数: 无
// 返回值: ROI_State_t - 当前ROI状态枚举值
ROI_State_t VisionStrategy_GetROIState(void);

// 获取完整策略状态结构体指针
// 参数: 无
// 返回值: VisionStrategy_t* - 策略状态结构体指针
VisionStrategy_t* VisionStrategy_GetState(void);

// 设置云台当前角度反馈
// 参数: x_angle - X轴角度 (度)
//       y_angle - Y轴角度 (度)
// 返回值: 无
void VisionStrategy_SetGimbalAngle(float x_angle, float y_angle);

// 执行云台蛇形扫描搜索
// 参数: 无
// 返回值: 无
void VisionStrategy_GimbalScan(void);

// 重置策略状态机为初始状态
// 参数: 无
// 返回值: 无
void VisionStrategy_Reset(void);

#endif
