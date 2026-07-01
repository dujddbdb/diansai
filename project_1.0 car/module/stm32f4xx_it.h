// 头文件保护：若本文件未被包含过，则执行后续代码
#ifndef __STM32F4xx_IT_H
// 定义保护宏，标记本文件已被包含，防止重复编译
#define __STM32F4xx_IT_H

// 判断当前是否为 C++ 编译器环境
#ifdef __cplusplus
// C++ 兼容声明：指示 C++ 编译器按 C 语言方式处理符号名，避免名称修饰导致链接失败
extern "C" {
#endif

// 包含 STM32F4 系列 MCU 标准外设库头文件
// 提供寄存器定义、中断/异常类型定义等
#include "stm32f4xx.h"

// 不可屏蔽中断处理函数
// 功能：处理不可屏蔽中断（Non-Maskable Interrupt）
// 触发原因：由硬件故障（时钟/振荡器失效）或独立看门狗触发
// 特点：不可被屏蔽，优先级仅次于 HardFault
// 参数：无
// 返回值：无
void NMI_Handler(void);

// 硬件错误处理函数
// 功能：处理硬件错误异常（Hard Fault）
// 触发原因：其他 Fault 异常（BusFault/MemManage/UsageFault）未被使能或优先级不足时均升级为此异常
// 处理方式：进入死循环 halt
// 参数：无
// 返回值：无
void HardFault_Handler(void);

// 内存管理错误处理函数
// 功能：处理内存管理错误异常（Memory Management Fault）
// 触发原因：MPU 违规或非法内存访问触发
// 处理方式：进入死循环
// 参数：无
// 返回值：无
void MemManage_Handler(void);

// 总线错误处理函数
// 功能：处理总线错误异常（Bus Fault）
// 触发原因：预取中止 / 数据中止 / 非对齐访问触发
// 处理方式：进入死循环
// 参数：无
// 返回值：无
void BusFault_Handler(void);

// 用法错误处理函数
// 功能：处理用法错误异常（Usage Fault）
// 触发原因：未定义指令 / 非法状态（PC的LSB=0） / 除零等异常触发
// 处理方式：进入死循环
// 参数：无
// 返回值：无
void UsageFault_Handler(void);

// 系统服务调用处理函数
// 功能：处理系统服务调用（Supervisor Call）
// 触发原因：由 SVC 汇编指令触发，供 RTOS 系统调用接口使用
// 参数：无
// 返回值：无
void SVC_Handler(void);

// 调试监控异常处理函数
// 功能：处理调试监控异常（Debug Monitor）
// 用途：配合调试器使用
// 参数：无
// 返回值：无
void DebugMon_Handler(void);

// 可挂起系统调用处理函数
// 功能：处理可挂起系统调用（Pendable Service Call）
// 触发原因：由软件触发，供 RTOS 上下文切换使用
// 参数：无
// 返回值：无
void PendSV_Handler(void);

// 系统滴答定时器中断处理函数
// 功能：处理系统滴答定时器（SysTick）中断
// 注意：实际定义位于 board.c
// 参数：无
// 返回值：无
void SysTick_Handler(void);

// 判断当前是否为 C++ 编译器环境
#ifdef __cplusplus
// 结束 extern "C" 块，恢复 C++ 名称修饰规则
}
#endif

// 结束头文件保护：与 #ifndef 配对，__STM32F4xx_IT_H 已定义
#endif
