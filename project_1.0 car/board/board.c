// board.c - STM32F407 板级初始化: 向量表、SysTick、延时函数
// 使用轮询SysTick计数器实现us/ms延时, 不依赖中断
// HAL_GetTick()通过累计SysTick计数器计算毫秒数

// 包含板级支持头文件(函数声明和外部变量声明)
#include "board.h"

// 系统运行毫秒计数（volatile类型，由SysTick中断每1ms递增，初值为0）
// 约49.7天后溢出回0（2^32毫秒）
__IO uint32_t g_system_tick = 0;

// 板级初始化函数：中断向量表定位 + SysTick配置
// 功能：
//   1. 设置中断向量表基址为FLASH起始地址(0x08000000)
//   2. 配置SysTick为1ms定时中断
// SysTick配置说明：
//   - 时钟源：HCLK/8 = 168MHz/8 = 21MHz
//   - 重载值：21000-1 = 20999
//   - 21MHz / 21000 ≈ 1000，即每秒触发1000次 = 1ms/次
void board_init(void)
{
    // 中断向量表重定位到FLASH基址 0x08000000 (SCB_VTOR寄存器)
    SCB->VTOR = 0x08000000;

    // 选择SysTick时钟源: HCLK/8 = 168MHz/8 = 21MHz
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);

    // 重载值=(168MHz/8/1000)-1=20999，每1ms计数器减到0触发中断
    SysTick->LOAD = (SystemCoreClock / 8 / 1000) - 1;

    // 清零当前计数器值，确保首次计时准确
    SysTick->VAL  = 0;

    // 使能SysTick: ENABLE(位0)计数器使能 | TICKINT(位1)中断使能
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;
}

// 微秒级延时函数：轮询SysTick递减计数器实现精确延时
// 优点：不依赖中断，在中断被屏蔽时仍可使用
// 原理：
//   - SysTick是递减计数器，从LOAD值向下数到0
//   - 读取VAL的当前值，计算已消耗的滴答数
//   - 累积达到目标滴答数时退出
void delay_us(uint32_t _us)
{
    // 目标滴答数=us×(168MHz/8000000)=us×21，即每微秒需计21个SysTick滴答
    uint32_t ticks = _us * (SystemCoreClock / 8000000);

    // 快照当前SysTick计数器值作为时间基准
    uint32_t told  = SysTick->VAL;

    // tnow=每次读取的当前计数值，tcnt=累计滴答数初值为0
    uint32_t tnow, tcnt = 0;

    // 主循环：持续轮询检测SysTick计数器变化直到达到目标延时
    while (1) {
        // 读取当前SysTick计数器值（递减计数器）
        tnow = SysTick->VAL;

        // 计数器值发生变化时进入差分累积处理
        if (tnow != told) {
            // 递减未溢出：当前值小于旧值（正常递减过程）
            if (tnow < told) {
                // 累加差值：旧值减去当前值，即这段时间经过的滴答数
                tcnt += told - tnow;
            }
            // 递减溢出一回绕：当前值大于等于旧值（计数器从0回绕到LOAD）
            else {
                // 溢出差值=(重载值-当前值)+旧值，即从旧值到0再从LOAD到当前值经过的总滴答数
                tcnt += SysTick->LOAD - tnow + told;
            }

            // 更新基准值为当前计数器值，供下次比较使用
            told = tnow;

            // 累计滴答数达到目标值，退出循环完成延时
            if (tcnt >= ticks) break;
        }
    }
}

// 毫秒级延时函数：转换为微秒后调用delay_us
void delay_ms(uint32_t _ms)
{
    // 调用微秒延时：ms×1000转换为微秒（1ms=1000μs）
    delay_us(_ms * 1000);
}

// 获取系统运行毫秒数（HAL库标准接口）
// 返回值：系统上电以来经过的毫秒数
uint32_t HAL_GetTick(void)
{
    // 返回全局系统滴答计数器值（每1ms由SysTick中断递增）
    return g_system_tick;
}

// SysTick中断服务函数（每1ms由硬件自动触发）
// 触发条件：SysTick计数器从1减到0时触发
// 功能：递增全局系统滴答计数器
void SysTick_Handler(void)
{
    // 每毫秒递增全局系统滴答计数器（HAL_GetTick的数据来源）
    g_system_tick++;
}
