// @file    stm32f4xx_it.c
// @brief   中断向量表处理函数集合
// @note    Fault类异常(HardFault/MemManage/BusFault/UsageFault)均进入死循环halt,
//          阻止程序继续运行, 便于调试器捕获异常现场与调用栈.
//          SysTick_Handler定义于board.c, USART1/6及EXTI9_5定义于其他模块.

#include "stm32f4xx_it.h"

// NMI不可屏蔽中断处理函数
// 由硬件故障(如时钟失效、振荡器故障)或独立看门狗触发
// 此中断不可被屏蔽, 优先级仅次于HardFault
void NMI_Handler(void) {}

// HardFault硬件错误中断处理函数
// 当其他Fault异常(BusFault/MemManage/UsageFault)未被使能或优先级不足时, 均会升级为此异常
// 进入死循环以阻止程序继续运行, 便于调试器捕获现场
void HardFault_Handler(void)
{
    while (1) {}
}

// MemManage内存管理错误中断处理函数
// 由MPU违规访问或非法内存操作(如访问未映射区域、从不可执行区取指执行)触发
void MemManage_Handler(void)
{
    while (1) {}
}

// BusFault总线错误中断处理函数
// 由预取中止、数据中止或非对齐访问等总线异常触发
void BusFault_Handler(void)
{
    while (1) {}
}

// UsageFault用法错误中断处理函数
// 由未定义指令、非法状态(如加载到PC的地址LSB=0)、除数为零等异常触发
void UsageFault_Handler(void)
{
    while (1) {}
}

// SVC系统服务调用中断处理函数
// 由SVC汇编指令触发, 通常用于RTOS的系统调用接口, 当前项目未使用RTOS
void SVC_Handler(void) {}

// DebugMon调试监控异常处理函数
// 用于调试监控, 配合调试器使用, 当前项目未启用此功能
void DebugMon_Handler(void) {}

// PendSV可挂起系统调用中断处理函数
// 由软件触发, 通常用于RTOS上下文切换, 当前项目未使用RTOS
void PendSV_Handler(void) {}

// SysTick_Handler is defined in board.c
// USART1_IRQHandler is defined in uart_k230.c
// USART6_IRQHandler is defined in motor.c
// EXTI9_5_IRQHandler is defined in bsp_i2c.c
