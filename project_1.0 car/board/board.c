/**
 * @file    board.c
 * @brief   STM32F407 板级初始化: 向量表、SysTick、延时函数
 * @note    使用轮询SysTick计数器实现us/ms延时, 不依赖中断
 *          HAL_GetTick()通过累计SysTick计数器计算毫秒数
 */

/**
 * 包含板级支持头文件
 * 获取board_init、delay_us、delay_ms、HAL_GetTick函数声明
 * 获取g_system_tick全局变量声明
 */
#include "board.h"                                                           // 包含板级支持头文件(函数声明和外部变量声明)

/**
 * 全局系统滴答计数器
 * __IO是volatile的宏定义别名
 * 用途：记录系统上电以来经过的毫秒数
 * 更新方式：由SysTick中断每1ms递增一次
 * 溢出：约49.7天后溢出回0（2^32毫秒）
 */
__IO uint32_t g_system_tick = 0;                                             // 系统运行毫秒计数（volatile类型，由SysTick中断每1ms递增，初值为0）

/**
 * 板级初始化函数
 * 功能：
 *   1. 设置中断向量表基址为FLASH起始地址(0x08000000)
 *   2. 配置SysTick为1ms定时中断
 * SysTick配置说明：
 *   - 时钟源：HCLK/8 = 168MHz/8 = 21MHz
 *   - 重载值：21000-1 = 20999
 *   - 21MHz / 21000 ≈ 1000，即每秒触发1000次 = 1ms/次
 */
void board_init(void)                                                        // 板级初始化函数：中断向量表定位 + SysTick配置
{                                                                            // board_init函数开始
    /**
     * 设置中断向量表基址为FLASH起始地址
     * SCB->VTOR：系统控制块的向量表偏移寄存器
     * 0x08000000是STM32F4系列FLASH起始地址
     * 作用：将中断向量表重定位到FLASH开始位置
     */
    SCB->VTOR = 0x08000000;                                                 // 中断向量表重定位到FLASH基址 0x08000000 (SCB_VTOR寄存器)

    /**
     * 配置SysTick时钟源为HCLK/8
     * SysTick_CLKSourceConfig：SysTick时钟源配置函数
     * SysTick_CLKSource_HCLK_Div8：选择HCLK的8分频作为SysTick时钟
     * HCLK = 168MHz（系统时钟）
     * SysTick时钟 = 168MHz / 8 = 21MHz
     */
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);                    // 选择SysTick时钟源: HCLK/8 = 168MHz/8 = 21MHz

    /**
     * 配置SysTick重载值
     * SysTick->LOAD：SysTick重载寄存器
     * 计算公式：重载值 = (SysTick时钟频率 / 期望中断频率) - 1
     * 期望中断频率：1000Hz（每1ms一次）
     * SysTick时钟：21MHz
     * 重载值 = 21MHz / 1000Hz - 1 = 21000 - 1 = 20999
     * SystemCoreClock / 8 / 1000 = 168MHz / 8 / 1000 = 21000
     */
    SysTick->LOAD = (SystemCoreClock / 8 / 1000) - 1;                     // 重载值=(168MHz/8/1000)-1=20999，每1ms计数器减到0触发中断

    /**
     * 清零SysTick当前计数器值
     * SysTick->VAL：SysTick当前值寄存器
     * 写入0将计数器清零
     * 作用：确保首次计时从0开始
     */
    SysTick->VAL  = 0;                                                      // 清零当前计数器值，确保首次计时准确

    /**
     * 使能SysTick
     * SysTick->CTRL：SysTick控制和状态寄存器
     * SysTick_CTRL_ENABLE_Msk：使能计数器
     * SysTick_CTRL_TICKINT_Msk：使能SysTick异常（中断）
     * 注意：位2(CLKSOURCE)在前面设置为HCLK/8后自动为0
     */
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;    // 使能SysTick: ENABLE(位0)计数器使能 | TICKINT(位1)中断使能
}                                                                            // board_init函数结束

/**
 * 微秒级延时函数
 * 参数：_us - 延时微秒数
 * 实现方式：轮询SysTick递减计数器
 * 优点：不依赖中断，在中断被屏蔽时仍可使用
 * 原理：
 *   - SysTick是递减计数器，从LOAD值向下数到0
 *   - 读取VAL的当前值，计算已消耗的滴答数
 *   - 累积达到目标滴答数时退出
 */
void delay_us(uint32_t _us)                                                  // 微秒级延时函数：轮询SysTick递减计数器实现精确延时
{                                                                            // delay_us函数开始
    /**
     * 将微秒转换为SysTick滴答数
     * SysTick时钟 = SystemCoreClock / 8 = 21MHz
     * 每微秒计数次数 = 21MHz / 1000000 = 21
     * 因此：ticks = _us * 21
     */
    uint32_t ticks = _us * (SystemCoreClock / 8000000);                    // 目标滴答数=us×(168MHz/8000000)=us×21，即每微秒需计21个SysTick滴答

    /**
     * 快照当前SysTick计数器值
     * told：记录上一时刻的计数器值，作为基准
     */
    uint32_t told  = SysTick->VAL;                                          // 快照当前SysTick计数器值作为时间基准

    /**
     * tnow：当前读取的计数器值
     * tcnt：累积已消耗的滴答数
     */
    uint32_t tnow, tcnt = 0;                                               // tnow=每次读取的当前计数值，tcnt=累计滴答数初值为0

    /**
     * 主循环：持续轮询直到达到目标延时
     * SysTick是递减计数器，从LOAD向下数到0然后重载
     */
    while (1) {                                                             // 主循环：持续轮询检测SysTick计数器变化直到达到目标延时
        tnow = SysTick->VAL;                                                // 读取当前SysTick计数器值（递减计数器）

        /**
         * 检测计数器值是否发生变化
         * 如果计数器值不变（说明还没开始新一轮计数），跳过本次处理
         */
        if (tnow != told) {                                                // 计数器值发生变化时进入差分累积处理
            /**
             * 处理未溢出情况
             * 条件：tnow < told（正常递减过程中，计数器从大值往小值走）
             * 差值计算：told - tnow
             * 例如：told=10000, tnow=9990，差值=10
             */
            if (tnow < told) {                                             // 递减未溢出：当前值小于旧值（正常递减过程）
                tcnt += told - tnow;                                         // 累加差值：旧值减去当前值，即这段时间经过的滴答数
            }
            /**
             * 处理溢出回绕情况
             * 条件：tnow >= told（计数器从0回绕到LOAD，然后继续递减）
             * 例如：told=100, tnow=20000（假设LOAD=21000）
             * 差值 = (LOAD - tnow) + told = (21000 - 20000) + 100 = 1100
             */
            else {                                                           // 递减溢出一回绕：当前值大于等于旧值（计数器从0回绕到LOAD）
                tcnt += SysTick->LOAD - tnow + told;                        // 溢出差值=(重载值-当前值)+旧值，即从旧值到0再从LOAD到当前值经过的总滴答数
            }

            told = tnow;                                                     // 更新基准值为当前计数器值，供下次比较使用

            /**
             * 判断是否已达到目标延时
             * 如果累积滴答数达到目标值，退出循环
             */
            if (tcnt >= ticks) break;                                       // 累计滴答数达到目标值，退出循环完成延时
        }
    }
}                                                                            // delay_us函数结束

/**
 * 毫秒级延时函数
 * 参数：_ms - 延时毫秒数
 * 实现方式：调用delay_us实现（将毫秒转换为微秒）
 */
void delay_ms(uint32_t _ms)                                                  // 毫秒级延时函数：转换为微秒后调用delay_us
{                                                                            // delay_ms函数开始
    delay_us(_ms * 1000);                                                    // 调用微秒延时：ms×1000转换为微秒（1ms=1000μs）
}                                                                            // delay_ms函数结束

/**
 * 获取系统运行时间
 * 返回值：系统上电以来经过的毫秒数
 * 实现方式：返回g_system_tick全局变量
 * 用途：
 *   - 计算代码执行时间差
 *   - 实现超时检测
 *   - 定时任务调度
 */
uint32_t HAL_GetTick(void)                                                    // 获取系统运行毫秒数（HAL库标准接口）
{                                                                            // HAL_GetTick函数开始
    return g_system_tick;                                                    // 返回全局系统滴答计数器值（每1ms由SysTick中断递增）
}                                                                            // HAL_GetTick函数结束

/**
 * SysTick中断服务函数
 * 触发条件：SysTick计数器从1减到0时触发
 * 触发频率：每1ms触发一次（由board_init中LOAD值决定）
 * 功能：递增全局系统滴答计数器
 */
void SysTick_Handler(void)                                                     // SysTick中断服务函数（每1ms由硬件自动触发）
{                                                                            // SysTick_Handler函数开始
    g_system_tick++;                                                         // 每毫秒递增全局系统滴答计数器（HAL_GetTick的数据来源）
}                                                                            // SysTick_Handler函数结束
