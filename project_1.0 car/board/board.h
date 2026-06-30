/**
 * @file    board.h
 * @brief   STM32F407 板级支持包 - 系统时钟、SysTick、延时函数
 * @note    系统时钟: HSI 16MHz + PLL -> 168MHz (不依赖外部晶振)
 *          SysTick:  HCLK/8 = 21MHz, 1ms中断一次
 */

#ifndef __BOARD_H__
#define __BOARD_H__

#include "stm32f4xx.h"

extern __IO uint32_t g_system_tick;  // 全局系统滴答计数器（SysTick中断每1ms递增）

void board_init(void);               // 板级初始化：中断向量表 + SysTick配置
void delay_us(uint32_t _us);         // 微秒级延时（轮询SysTick递减计数器）
void delay_ms(uint32_t _ms);         // 毫秒级延时（内部调用delay_us）
uint32_t HAL_GetTick(void);          // 获取系统运行毫秒数（返回g_system_tick）

#endif
