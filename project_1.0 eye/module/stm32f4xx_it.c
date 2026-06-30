#include "stm32f4xx_it.h"                   /* 包含中断向量表头文件, 提供所有 Cortex-M4 系统异常处理函数的声明 */

// @file    stm32f4xx_it.c                  中断向量表处理函数集合
// @note    Fault类异常(HardFault/MemManage/BusFault/UsageFault)均进入死循环halt,
//          阻止程序继续运行, 便于调试器捕获异常现场与调用栈.
//          SysTick_Handler定义于board.c, USART1/6及EXTI9_5定义于其他模块.

/**
 * @brief  Non-Maskable Interrupt (NMI) 处理函数
 * @note   由硬件故障（如时钟失效、振荡器故障）或独立看门狗触发。
 *         此中断不可被屏蔽，优先级仅次于 HardFault。
 *         当前未实现具体处理逻辑，仅作为占位。
 */
void NMI_Handler(void) {}                   /* 不可屏蔽中断处理函数: 空实现, 触发后默认返回继续执行, 不进入死循环 */

/**
 * @brief  Hard Fault 处理函数
 * @note   硬件错误中断，当其他 Fault 异常（BusFault/MemManage/UsageFault）
 *         未被使能或优先级不足时，均会升级为此异常。
 *         进入死循环以阻止程序继续运行，便于调试器捕获现场。
 */
void HardFault_Handler(void)                /* 硬件错误中断处理函数: 程序崩溃时触发, 是所有 Fault 异常的最终升级入口 */
{                                           /* 函数体开始 */
    while (1) {}                            /* 死循环, 触发此中断说明系统异常, 需调试排查 */
}                                           /* 函数体结束 */

/**
 * @brief  Memory Management Fault 处理函数
 * @note   内存管理错误，由 MPU 违规访问或非法内存操作（如访问未映射区域、
 *         从不可执行区取指执行）触发。进入死循环以阻止程序继续运行。
 */
void MemManage_Handler(void)                /* 内存管理错误中断处理函数: 非法内存访问 (如访问未映射区域) 时触发 */
{                                           /* 函数体开始 */
    while (1) {}                            /* 死循环, 触发此中断说明系统异常, 需调试排查 */
}                                           /* 函数体结束 */

/**
 * @brief  Bus Fault 处理函数
 * @note   总线错误，由预取中止、数据中止或非对齐访问等总线异常触发。
 *         进入死循环以阻止程序继续运行。
 */
void BusFault_Handler(void)                 /* 总线错误中断处理函数: 总线访问异常 (预取中止/数据中止/非对齐访问) 时触发 */
{                                           /* 函数体开始 */
    while (1) {}                            /* 死循环, 触发此中断说明系统异常, 需调试排查 */
}                                           /* 函数体结束 */

/**
 * @brief  Usage Fault 处理函数
 * @note   用法错误，由未定义指令、非法状态（如加载到 PC 的地址 LSB=0）、
 *         除数为零等异常触发。进入死循环以阻止程序继续运行。
 */
void UsageFault_Handler(void)               /* 用法错误中断处理函数: 未定义指令 / 非法状态 / 除零等异常时触发 */
{                                           /* 函数体开始 */
    while (1) {}                            /* 死循环, 触发此中断说明系统异常, 需调试排查 */
}                                           /* 函数体结束 */

/**
 * @brief  Supervisor Call (SVC) 处理函数
 * @note   由 SVC 汇编指令触发，通常用于 RTOS 的系统调用接口。
 *         当前项目未使用 RTOS，故未实现。
 */
void SVC_Handler(void) {}                   /* 系统服务调用 (SVC) 中断处理函数: 空实现, 由 SVC 指令触发, 供 RTOS 系统调用接口使用 */

/**
 * @brief  Debug Monitor 异常处理函数
 * @note   用于调试监控，配合调试器使用。
 *         当前项目未启用此功能，故未实现。
 */
void DebugMon_Handler(void) {}              /* 调试监控 (DebugMon) 异常处理函数: 空实现, 供调试器使用, 当前项目未启用 */

/**
 * @brief  Pendable Service Call (PendSV) 处理函数
 * @note   可挂起系统调用，由软件触发，通常用于 RTOS 上下文切换。
 *         当前项目未使用 RTOS，故未实现。
 */
void PendSV_Handler(void) {}                /* 可挂起系统调用 (PendSV) 中断处理函数: 空实现, 由软件触发, 供 RTOS 上下文切换使用 */

/* SysTick_Handler is defined in board.c                           // SysTick 系统滴答定时器中断处理函数定义于 board.c
   USART1_IRQHandler is defined in uart_k230.c                     // USART1 串口中断处理函数(K230通信)定义于 uart_k230.c
   USART6_IRQHandler is defined in motor.c                         // USART6 串口中断处理函数定义于 motor.c
   EXTI9_5_IRQHandler is defined in bsp_i2c.c                      // EXTI9_5 外部中断处理函数定义于 bsp_i2c.c */
