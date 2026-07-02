#ifndef __VISION_H__
#define __VISION_H__

#include "stm32f4xx.h"
#include "stdbool.h"
#include "gimbal_pid.h"

// 视觉系统上下文状态结构体
typedef struct {
    float    error_x;            // X轴像素误差
    float    error_y;            // Y轴像素误差
    float    error_distance;     // 误差距离 (像素)
    uint8_t  target_detected;    // 目标检测标志: 1-检测到目标, 0-未检测到
    uint8_t  laser_on;           // 激光状态: 1-激光开启, 0-激光关闭
    uint8_t  hit_streak;         // 连续命中计数
    int16_t  gimbal_x_angle;     // 云台X轴角度 (度)
    int16_t  gimbal_y_angle;     // 云台Y轴角度 (度)
} Vision_Context_t;

// 初始化视觉系统PID控制器
// 参数: 无
// 返回值: 无
void Vision_GimbalPID_Init(void);

// 更新视觉PID输出
// 参数: error_x_px - X轴像素误差
//       error_y_px - Y轴像素误差
// 返回值: 无
void Vision_GimbalPID_Update(float error_x_px, float error_y_px);

// 清空视觉PID积分项
// 参数: 无
// 返回值: 无
void Vision_GimbalPID_ClearIntegral(void);

// IMU前馈1kHz定时调用 (高频更新IMU数据)
// 参数: 无
// 返回值: 无
void Vision_IMUFeedforward1kTick(void);

// IMU补偿定时调用 (丢包时维持补偿)
// 参数: 无
// 返回值: 无
void Vision_GimbalIMUCompensationTick(void);

// 获取云台PID控制器指针
// 参数: 无
// 返回值: GimbalDualPID_t* - 双轴PID结构体指针
GimbalDualPID_t* Vision_GimbalPID_GetController(void);

// 处理目标检测数据
// 参数: err_y - X轴像素误差
//       err_z - Y轴像素误差
//       valid - 目标是否有效 (true-有效, false-无效)
// 返回值: 无
void Vision_TargetProcess(int16_t err_y, int16_t err_z, bool valid);

// 激光触发判断
// 参数: target_valid - 当前是否有有效目标
// 返回值: uint8_t - 1-激光开启, 0-激光关闭
uint8_t Vision_LaserTrigger(bool target_valid);

// 获取当前误差值
// 参数: err_x - 输出X轴误差指针
//       err_y - 输出Y轴误差指针
//       dist  - 输出误差距离指针
// 返回值: 无
void Vision_GetError(float *err_x, float *err_y, float *dist);

// 初始化整个视觉系统
// 参数: 无
// 返回值: 无
void Vision_Init(void);

// 按键控制定时处理 (切换视觉模式)
// 参数: 无
// 返回值: 无
void Vision_KeyControlTick(void);

// 视觉主处理循环 (每帧调用)
// 参数: 无
// 返回值: 无
void Vision_Process(void);

// 获取视觉上下文状态结构体指针
// 参数: 无
// 返回值: Vision_Context_t* - 视觉上下文结构体指针
Vision_Context_t* Vision_GetContext(void);

#endif
