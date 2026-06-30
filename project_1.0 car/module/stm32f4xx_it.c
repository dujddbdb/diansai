#include "stm32f4xx_it.h"

/* 中断向量表处理函数集合
 * Fault类异常均进入死循环halt，便于调试捕获现场
 * SysTick_Handler定义于board.c，USART1/6及EXTI9_5定义于其他模块
 */

// 不可屏蔽中断: 空实现，触发后返回继续执行
void NMI_Handler(void) {}

// 硬件错误中断: 死循环halt
void HardFault_Handler(void)
{
    while (1) {}
}

// 内存管理错误中断: 死循环halt
void MemManage_Handler(void)
{
    while (1) {}
}

// 总线错误中断: 死循环halt
void BusFault_Handler(void)
{
    while (1) {}
}

// 用法错误中断: 死循环halt
void UsageFault_Handler(void)
{
    while (1) {}
}

// 系统服务调用(SVC): 空实现，供RTOS使用
void SVC_Handler(void) {}

// 调试监控异常: 空实现，供调试器使用
void DebugMon_Handler(void) {}

// 可挂起系统调用(PendSV): 空实现，供RTOS上下文切换使用
void PendSV_Handler(void) {}

/* SysTick_Handler is defined in board.c
 * USART1 is TX-only for car-to-eye corner state frames
 * USART6_IRQHandler is defined in motor.c
 * EXTI9_5_IRQHandler is defined in bsp_i2c.c */
