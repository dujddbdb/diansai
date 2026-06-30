/**
 * @file    board.h
 * @brief   STM32F407 板级支持包 - 系统时钟、SysTick、延时函数
 * @note    系统时钟: HSI 16MHz + PLL -> 168MHz (不依赖外部晶振)
 *           SysTick:  HCLK/8 = 21MHz, 1ms中断一次
 */

#ifndef __BOARD_H__                                          // 头文件保护: 防止重复包含
#define __BOARD_H__                                          // 定义 __BOARD_H__ 宏标识本头文件已被包含

#include "stm32f4xx.h"                                       // 包含STM32F4外设标准库 (HSE_VALUE=8MHz, SystemCoreClock=168MHz, 寄存器定义等)

/* 全局系统滴答计数器, 每1ms递增(由SysTick中断更新) */
extern __IO uint32_t g_system_tick;                          // 全局系统滴答计数器声明 (volatile类型, SysTick中断每1ms递增)

/**
 * @brief  板级初始化 (向量表+SysTick)
 * @note   设置中断向量表基址为FLASH(0x08000000)
 *         SysTick: HCLK/8=21MHz, 1ms周期, 中断使能
 */
void board_init(void);                                       // 板级初始化函数: 中断向量表 + SysTick配置

/**
 * @brief  微秒级延时 (轮询SysTick计数器)
 * @param  _us  延时微秒数
 */
void delay_us(uint32_t _us);                                 // 微秒级延时函数声明 (轮询SysTick递减计数器)

/**
 * @brief  毫秒级延时
 * @param  _ms  延时毫秒数
 */
void delay_ms(uint32_t _ms);                                 // 毫秒级延时函数声明 (内部调用delay_us)

/**
 * @brief  获取系统运行时间 (ms)
 * @return 系统上电后的毫秒数 (由SysTick中断递增)
 */
uint32_t HAL_GetTick(void);                                  // 获取系统运行毫秒数 (返回g_system_tick)

#endif                                                       // __BOARD_H__ 头文件保护结束
