// @file    board.h
// @brief   STM32F407 板级支持包 - 系统时钟、SysTick、延时函数
// @note    系统时钟: HSI 16MHz + PLL -> 168MHz (不依赖外部晶振)
//           SysTick:  HCLK/8 = 21MHz, 1ms中断一次

#ifndef __BOARD_H__
#define __BOARD_H__

#include "stm32f4xx.h"

// 全局系统滴答计数器, 每1ms递增(由SysTick中断更新)
// 类型为 volatile, 防止编译器优化导致读取不一致
extern __IO uint32_t g_system_tick;

// 板级初始化函数
// 功能: 配置中断向量表基址为FLASH(0x08000000), 配置SysTick定时器
//       SysTick时钟源为HCLK/8=21MHz, 1ms周期中断, 使能中断
// 参数: 无
// 返回值: 无
void board_init(void);

// 微秒级延时函数
// 功能: 通过轮询SysTick计数器实现微秒级延时
// 参数: _us - 延时微秒数
// 返回值: 无
void delay_us(uint32_t _us);

// 毫秒级延时函数
// 功能: 毫秒级延时, 内部调用delay_us实现
// 参数: _ms - 延时毫秒数
// 返回值: 无
void delay_ms(uint32_t _ms);

// 获取系统运行时间函数
// 功能: 获取系统上电后的运行时间(毫秒), 返回g_system_tick的值
// 参数: 无
// 返回值: 系统运行毫秒数
uint32_t HAL_GetTick(void);

#endif
