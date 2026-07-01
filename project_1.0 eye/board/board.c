// @file    board.c
// @brief   STM32F407 板级初始化: 向量表、SysTick、延时函数
// @note    使用轮询SysTick计数器实现us/ms延时, 不依赖中断
//          HAL_GetTick()通过累计SysTick计数器计算毫秒数

#include "board.h"

// 全局滴答计数器, SysTick中断每1ms递增
__IO uint32_t g_system_tick = 0;

// 板级初始化: 设置中断向量表基址(FLASH), 配置SysTick为1ms定时
// SysTick时钟 = HCLK/8 = 168/8 = 21MHz, 重载 = 21000
void board_init(void)
{
    // 设置中断向量表基址为FLASH起始地址 0x08000000
    SCB->VTOR = 0x08000000;

    // 配置SysTick时钟源为HCLK/8 = 168MHz/8 = 21MHz
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
    // 重载值 = 21MHz/1000 - 1 = 20999, 每1ms产生一次中断
    SysTick->LOAD = (SystemCoreClock / 8 / 1000) - 1;
    // 清零当前计数器值
    SysTick->VAL  = 0;
    // 使能SysTick: 计数器使能 + 中断使能
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;
}

// 微秒级延时: 轮询SysTick递减计数器实现精确延时
// 参数: _us - 延时微秒数
void delay_us(uint32_t _us)
{
    // 将微秒转换为SysTick滴答数: 每微秒 = SystemCoreClock/8000000
    uint32_t ticks = _us * (SystemCoreClock / 8000000);
    // 快照当前SysTick计数器值, 作为基准
    uint32_t told  = SysTick->VAL;
    uint32_t tnow, tcnt = 0;

    // 循环检测SysTick变化, 累积差值直到达到目标滴答数
    while (1) {
        tnow = SysTick->VAL;
        if (tnow != told) {
            // SysTick是递减计数器: tnow < told 表示未溢出, 差值 = told - tnow
            if (tnow < told) {
                tcnt += told - tnow;
            // tnow >= told 表示计数器溢出回绕: 差值 = (LOAD-tnow) + told
            } else {
                tcnt += SysTick->LOAD - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks) break;
        }
    }
}

// 毫秒级延时: 转换为微秒后调用delay_us
void delay_ms(uint32_t _ms)
{
    delay_us(_ms * 1000);
}

// 获取系统运行时间 (由SysTick中断维护)
// 返回: 系统运行毫秒数
uint32_t HAL_GetTick(void)
{
    return g_system_tick;
}

// SysTick中断服务函数: 每1ms触发一次, 递增全局计数器
void SysTick_Handler(void)
{
    g_system_tick++;
}
