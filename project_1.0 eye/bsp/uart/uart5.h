/**
 * @file    uart5.h
 * @brief   UART5 car-to-eye corner-state receiver.
 */

#ifndef __UART5_H__
#define __UART5_H__

#include "stm32f4xx.h"

#define UART5_CAR_STATE_TIMEOUT_MS  120U  // 弯道状态超时时间(毫秒)

typedef struct {
    uint8_t phase;     // 弯道相位: 0=直道, 1=入弯, 2=弯中, 3=出弯
    uint8_t type;      // 弯道类型: 0=直道, 1=右弯, 2=左弯
    uint8_t flags;     // 标志位: bit0=IMU有效
    uint8_t fresh;     // 数据新鲜标志: 1=有效, 0=超时
    uint32_t last_ms;  // 上次收到数据的时间戳(毫秒)
    uint16_t frames;   // 累计接收帧数
} CarCornerState_t;

extern volatile CarCornerState_t car_corner_state; // 弯道状态全局变量

// 初始化UART5串口，baud: 波特率
void UART5_Init(uint32_t baud);
// 检查弯道状态数据是否新鲜，now_ms: 当前时间戳，timeout_ms: 超时阈值，返回1=新鲜 0=超时
uint8_t UART5_CarCornerFresh(uint32_t now_ms, uint32_t timeout_ms);
// 判断是否处于弯道中，now_ms: 当前时间戳，返回1=弯道中 0=直道
uint8_t UART5_CarCornerActive(uint32_t now_ms);
// 获取弯道补偿混合系数，now_ms: 当前时间戳，返回0.0~1.0的混合系数
float UART5_CarCornerBlend(uint32_t now_ms);

#endif