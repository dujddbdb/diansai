// 包含中断向量表头文件, 提供所有 Cortex-M4 系统异常处理函数的声明
#include "stm32f4xx_it.h"

// stm32f4xx_it.c - 中断向量表处理函数集合
// Fault类异常(HardFault/MemManage/BusFault/UsageFault)均进入死循环halt,
//   阻止程序继续运行, 便于调试器捕获异常现场与调用栈.
// SysTick_Handler定义于board.c, USART1/6及EXTI9_5定义于其他模块.

// NMI_Handler - 不可屏蔽中断处理函数
// 由硬件故障（如时钟失效、振荡器故障）或独立看门狗触发。
// 此中断不可被屏蔽，优先级仅次于 HardFault。
// 当前未实现具体处理逻辑，仅作为占位，触发后默认返回继续执行。
void NMI_Handler(void) {}

// HardFault_Handler - 硬件错误中断处理函数
// 当其他 Fault 异常（BusFault/MemManage/UsageFault）
//   未被使能或优先级不足时，均会升级为此异常。
// 进入死循环以阻止程序继续运行，便于调试器捕获现场。
void HardFault_Handler(void)
{
    // 死循环, 触发此中断说明系统异常, 需调试排查
    while (1) {}
}

// MemManage_Handler - 内存管理错误中断处理函数
// 由 MPU 违规访问或非法内存操作（如访问未映射区域、
//   从不可执行区取指执行）触发。
// 进入死循环以阻止程序继续运行。
void MemManage_Handler(void)
{
    // 死循环, 触发此中断说明系统异常, 需调试排查
    while (1) {}
}

// BusFault_Handler - 总线错误中断处理函数
// 由预取中止、数据中止或非对齐访问等总线异常触发。
// 进入死循环以阻止程序继续运行。
void BusFault_Handler(void)
{
    // 死循环, 触发此中断说明系统异常, 需调试排查
    while (1) {}
}

// UsageFault_Handler - 用法错误中断处理函数
// 由未定义指令、非法状态（如加载到 PC 的地址 LSB=0）、
//   除数为零等异常触发。
// 进入死循环以阻止程序继续运行。
void UsageFault_Handler(void)
{
    // 死循环, 触发此中断说明系统异常, 需调试排查
    while (1) {}
}

// SVC_Handler - 系统服务调用 (SVC) 中断处理函数
// 由 SVC 汇编指令触发，通常用于 RTOS 的系统调用接口。
// 当前项目未使用 RTOS，故未实现。
void SVC_Handler(void) {}

// DebugMon_Handler - 调试监控 (DebugMon) 异常处理函数
// 用于调试监控，配合调试器使用。
// 当前项目未启用此功能，故未实现。
void DebugMon_Handler(void) {}

// PendSV_Handler - 可挂起系统调用 (PendSV) 中断处理函数
// 由软件触发，通常用于 RTOS 上下文切换。
// 当前项目未使用 RTOS，故未实现。
void PendSV_Handler(void) {}

// SysTick_Handler is defined in board.c
//   SysTick 系统滴答定时器中断处理函数定义于 board.c
// USART1 is TX-only for car-to-eye corner state frames
//   USART1 仅用于发送车眼弯道状态帧
// USART6_IRQHandler is defined in motor.c
//   USART6 串口中断处理函数定义于 motor.c
// EXTI9_5_IRQHandler is defined in bsp_i2c.c
//   EXTI9_5 外部中断处理函数定义于 bsp_i2c.c
