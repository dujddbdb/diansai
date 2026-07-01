#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"

// 不可屏蔽中断处理函数
// 功能: 不可屏蔽中断(Non-Maskable Interrupt)处理
//       由硬件故障(时钟/振荡器失效)或独立看门狗触发
//       不可被屏蔽, 优先级仅次于HardFault
// 参数: 无
// 返回值: 无
void NMI_Handler(void);

// 硬件错误处理函数
// 功能: 硬件错误(Hard Fault)异常处理
//       其他Fault异常(BusFault/MemManage/UsageFault)未被使能或优先级不足时均升级为此异常
//       进入后执行死循环halt, 等待调试器连接
// 参数: 无
// 返回值: 无
void HardFault_Handler(void);

// 内存管理错误处理函数
// 功能: 内存管理错误(Memory Management Fault)异常处理
//       由MPU违规或非法内存访问触发
//       进入后执行死循环
// 参数: 无
// 返回值: 无
void MemManage_Handler(void);

// 总线错误处理函数
// 功能: 总线错误(Bus Fault)异常处理
//       由预取中止、数据中止或非对齐访问触发
//       进入后执行死循环
// 参数: 无
// 返回值: 无
void BusFault_Handler(void);

// 用法错误处理函数
// 功能: 用法错误(Usage Fault)异常处理
//       由未定义指令、非法状态(PC的LSB=0)、除零等异常触发
//       进入后执行死循环
// 参数: 无
// 返回值: 无
void UsageFault_Handler(void);

// 系统服务调用处理函数
// 功能: 系统服务调用(Supervisor Call)异常处理
//       由SVC汇编指令触发, 供RTOS系统调用接口使用
// 参数: 无
// 返回值: 无
void SVC_Handler(void);

// 调试监控异常处理函数
// 功能: 调试监控(Debug Monitor)异常处理
//       配合调试器使用, 用于调试监控功能
// 参数: 无
// 返回值: 无
void DebugMon_Handler(void);

// 可挂起系统调用处理函数
// 功能: 可挂起系统调用(Pendable Service Call)异常处理
//       由软件触发, 供RTOS上下文切换使用
//       优先级可配置, 可被其他高优先级中断抢占
// 参数: 无
// 返回值: 无
void PendSV_Handler(void);

// 系统滴答定时器中断处理函数
// 功能: 系统滴答定时器(SysTick)中断处理
//       实际定义位于board.c中, 每1ms触发一次
//       用于更新g_system_tick全局计数器
// 参数: 无
// 返回值: 无
void SysTick_Handler(void);

#ifdef __cplusplus
}
#endif

#endif
