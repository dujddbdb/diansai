// @file    uart5.h
// @brief   UART5 车端弯道状态接收接口
// @note    接收来自车身主控的弯道状态信息, 用于视觉追踪的弯道补偿

#ifndef __UART5_H__
#define __UART5_H__

#include "stm32f4xx.h"

// 弯道状态超时时间(毫秒), 超过此时间未收到数据则认为数据失效
#define UART5_CAR_STATE_TIMEOUT_MS  120U

// 弯道状态结构体
// 存储从车身主控接收到的弯道状态信息
typedef struct {
    uint8_t phase;     // 弯道相位: 0=直道, 1=入弯, 2=弯中, 3=出弯
    uint8_t type;      // 弯道类型: 0=直道, 1=右弯, 2=左弯
    uint8_t flags;     // 标志位: bit0=IMU数据有效
    uint8_t fresh;     // 数据新鲜标志: 1=有效(新鲜), 0=超时失效
    uint32_t last_ms;  // 上次收到数据的时间戳(毫秒), 用于超时检测
    uint16_t frames;   // 累计接收帧数, 用于统计通信稳定性
} CarCornerState_t;

// 弯道状态全局变量, 存储最新的车身弯道状态数据
extern volatile CarCornerState_t car_corner_state;

// UART5初始化函数
// 功能: 初始化UART5串口, 配置GPIO引脚和串口参数, 开启接收中断
// 参数: baud - 波特率
// 返回值: 无
void UART5_Init(uint32_t baud);

// 检查弯道状态数据是否新鲜
// 功能: 检查弯道状态数据是否在超时时间内更新
// 参数: now_ms - 当前系统时间戳(毫秒)
//       timeout_ms - 超时阈值(毫秒)
// 返回值: 1=数据新鲜有效, 0=数据已超时失效
uint8_t UART5_CarCornerFresh(uint32_t now_ms, uint32_t timeout_ms);

// 判断是否处于弯道中
// 功能: 根据当前时间和弯道相位判断车辆是否处于弯道行驶状态
// 参数: now_ms - 当前系统时间戳(毫秒)
// 返回值: 1=处于弯道中, 0=直道
uint8_t UART5_CarCornerActive(uint32_t now_ms);

// 获取弯道补偿混合系数
// 功能: 根据弯道相位计算弯道补偿的混合系数, 用于平滑过渡
// 参数: now_ms - 当前系统时间戳(毫秒)
// 返回值: 0.0~1.0的混合系数, 0=无弯道补偿, 1=全量弯道补偿
float UART5_CarCornerBlend(uint32_t now_ms);

#endif
