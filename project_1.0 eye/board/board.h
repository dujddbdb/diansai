/**
 * @file    board.h
 * @brief   STM32F407 板级支持包 - 系统时钟、SysTick、延时函数
 * @note    系统时钟: HSI 16MHz + PLL -> 168MHz (不依赖外部晶振)
 *           SysTick:  HCLK/8 = 21MHz, 1ms中断一次
 */

#ifndef __BOARD_H__
#define __BOARD_H__

#include "stm32f4xx.h"

extern __IO uint32_t g_system_tick;

/**
 * @brief  板级初始化 (向量表+SysTick)
 */
void board_init(void);

/**
 * @brief  微秒级延时
 * @param  _us  延时微秒数
 */
void delay_us(uint32_t _us);

/**
 * @brief  毫秒级延时
 * @param  _ms  延时毫秒数
 */
void delay_ms(uint32_t _ms);

/**
 * @brief  获取系统运行时间 (ms)
 * @return 系统上电后的毫秒数
 */
uint32_t HAL_GetTick(void);

#endif
