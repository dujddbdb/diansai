/**
 * @file    board.c
 * @brief   STM32F407 板级初始化: 向量表、SysTick、延时函数
 * @note    使用轮询SysTick计数器实现us/ms延时, 不依赖中断
 *          HAL_GetTick()通过累计SysTick计数器计算毫秒数
 */

#include "board.h"                                           // 包含板级支持头文件(函数声明和外部变量声明)

/* 全局滴答计数器, SysTick中断每1ms递增 */
__IO uint32_t g_system_tick = 0;                             // 系统运行毫秒计数(volatile类型, 由SysTick中断每1ms递增, 初值为0)

/**
 * @brief  板级初始化
 * @note   设置中断向量表基址(FLASH), 配置SysTick为1ms定时
 *         SysTick时钟 = HCLK/8 = 168/8 = 21MHz
 *         SysTick重载 = 21000, 即每1ms溢出一次
 */
void board_init(void)                                        // 板级初始化: 向量表定位 + SysTick配置
{                                                            // board_init函数体开始
    /* 设置中断向量表基址为FLASH起始地址 0x08000000 */
    SCB->VTOR = 0x08000000;                                  // 中断向量表重定位到FLASH基址 0x08000000 (SCB_VTOR寄存器)

    /* 配置SysTick时钟源为HCLK/8 = 168MHz/8 = 21MHz */
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);    // 选择SysTick时钟源: HCLK/8 = 168MHz/8 = 21MHz
    /* 重载值 = 21MHz/1000 - 1 = 20999, 每1ms产生一次中断 */
    SysTick->LOAD = (SystemCoreClock / 8 / 1000) - 1;        // 重载值=(168MHz/8/1000)-1=20999, 每1ms计数器减到0触发中断
    /* 清零当前计数器值 */
    SysTick->VAL  = 0;                                       // 清零当前计数器值, 确保首次计时准确
    /* 使能SysTick: 计数器使能 + 中断使能 */
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk; // 使能SysTick: 位0(ENABLE)计数器使能 | 位1(TICKINT)中断使能, 位2(CLKSOURCE)=0即HCLK/8
}                                                            // board_init函数体结束

/**
 * @brief  微秒级延时 (轮询SysTick计数器, 不依赖中断)
 * @param  _us  延时微秒数
 * @note   SysTick时钟 = HCLK/8, 每微秒计数 = SystemCoreClock/8000000
 */
void delay_us(uint32_t _us)                                  // 微秒级延时: 轮询SysTick递减计数器实现精确延时
{                                                            // delay_us函数体开始
    /* 将微秒转换为SysTick滴答数: 每微秒 = SystemCoreClock/8000000 */
    uint32_t ticks = _us * (SystemCoreClock / 8000000);      // 目标滴答数=us×(168MHz/8000000)=us×21, 即每微秒需计21个SysTick滴答
    /* 快照当前SysTick计数器值, 作为基准 */
    uint32_t told  = SysTick->VAL;                           // 快照当前SysTick计数器值作为时间基准
    uint32_t tnow, tcnt = 0;                                 // tnow(每次读取的当前计数快照), tcnt(累计滴答数, 初始为0)

    /* 循环检测SysTick变化, 累积差值直到达到目标滴答数 */
    while (1) {                                              // 主循环: 持续轮询检测SysTick计数器变化直到达到目标延时
        tnow = SysTick->VAL;                                 // 读取当前SysTick计数器值(递减计数器)
        if (tnow != told) {                                  // 计数器值发生变化时进入差分累积处理
            /* SysTick是递减计数器: tnow < told 表示未溢出, 差值 = told - tnow */
            if (tnow < told) {                               // 递减未溢出: 当前值小于旧值 (正常递减过程)
                tcnt += told - tnow;                         // 累加差值: 旧值减去当前值, 即这段时间经过的滴答数
            /* tnow >= told 表示计数器溢出回绕: 差值 = (LOAD-tnow) + told */
            } else {                                         // 递减溢出一回绕: 当前值大于等于旧值 (计数器从0回绕到LOAD)
                tcnt += SysTick->LOAD - tnow + told;         // 溢出差值=(重载值-当前值)+旧值, 即从旧值到0再从LOAD到当前值经过的总滴答数
            }                                                // 溢出回绕处理结束
            told = tnow;                                     // 更新基准值为当前计数器值, 供下次比较使用
            if (tcnt >= ticks) break;                        // 累计滴答数达到目标值, 退出循环完成延时
        }                                                    // 计数变化处理结束
    }                                                        // while(1)主循环结束
}                                                            // delay_us函数体结束

/**
 * @brief  毫秒级延时
 * @param  _ms  延时毫秒数
 */
void delay_ms(uint32_t _ms)                                  // 毫秒级延时: 转换为微秒后调用delay_us
{                                                            // delay_ms函数体开始
    delay_us(_ms * 1000);                                    // 调用微秒延时: ms×1000转换为微秒 (1ms=1000μs)
}                                                            // delay_ms函数体结束

/**
 * @brief  获取系统运行时间 (由SysTick中断维护)
 * @return 系统运行毫秒数
 * @note   如果SysTick中断被阻塞或关闭, 此函数将停止计数
 */
uint32_t HAL_GetTick(void)                                   // 获取系统运行毫秒数 (HAL库标准接口, 替代HAL原版实现)
{                                                            // HAL_GetTick函数体开始
    return g_system_tick;                                    // 返回全局系统滴答计数器值 (每1ms由SysTick中断递增)
}                                                            // HAL_GetTick函数体结束

/**
 * @brief SysTick中断服务函数
 * @note  每1ms触发一次, 递增全局计数器
 */
void SysTick_Handler(void)                                   // SysTick中断服务函数 (每1ms由硬件自动触发)
{                                                            // SysTick_Handler函数体开始
    g_system_tick++;                                         // 每毫秒递增全局系统滴答计数器 (HAL_GetTick的数据来源)
}                                                            // SysTick_Handler函数体结束
