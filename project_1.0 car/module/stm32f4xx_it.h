#ifndef __STM32F4xx_IT_H                    /* 头文件保护: 若本文件未被包含过, 则执行后续代码 */
#define __STM32F4xx_IT_H                    /* 定义保护宏, 标记本文件已被包含, 防止重复编译 */

#ifdef __cplusplus                          /* 判断当前是否为 C++ 编译器环境 */
extern "C" {                                /* C++ 兼容声明: 指示 C++ 编译器按 C 语言方式处理符号名, 避免名称修饰导致链接失败 */
#endif

#include "stm32f4xx.h"                      /* 包含 STM32F4 系列 MCU 标准外设库头文件, 提供寄存器定义和中断/异常类型 */

void NMI_Handler(void);                     /**< 不可屏蔽中断 (Non-Maskable Interrupt) 处理函数声明, 由硬件故障(时钟/振荡器失效)或独立看门狗触发, 不可被屏蔽, 优先级仅次于 HardFault */
void HardFault_Handler(void);               /**< 硬件错误 (Hard Fault) 处理函数声明, 其他 Fault 异常(BusFault/MemManage/UsageFault)未被使能或优先级不足时均升级为此异常, 进入死循环 halt */
void MemManage_Handler(void);               /**< 内存管理错误 (Memory Management Fault) 处理函数声明, MPU 违规或非法内存访问触发, 进入死循环 */
void BusFault_Handler(void);                /**< 总线错误 (Bus Fault) 处理函数声明, 预取中止 / 数据中止 / 非对齐访问触发, 进入死循环 */
void UsageFault_Handler(void);              /**< 用法错误 (Usage Fault) 处理函数声明, 未定义指令 / 非法状态(PC的LSB=0) / 除零等异常触发, 进入死循环 */
void SVC_Handler(void);                     /**< 系统服务调用 (Supervisor Call) 处理函数声明, 由 SVC 汇编指令触发, 供 RTOS 系统调用接口使用 */
void DebugMon_Handler(void);                /**< 调试监控 (Debug Monitor) 异常处理函数声明, 配合调试器使用 */
void PendSV_Handler(void);                  /**< 可挂起系统调用 (Pendable Service Call) 处理函数声明, 由软件触发, 供 RTOS 上下文切换使用 */
void SysTick_Handler(void);                 /**< 系统滴答定时器 (SysTick) 中断处理函数声明, 实际定义位于 board.c */

#ifdef __cplusplus                          /* 判断当前是否为 C++ 编译器环境 */
}                                           /* 结束 extern "C" 块, 恢复 C++ 名称修饰规则 */
#endif

#endif                                      /* 结束头文件保护: 与 #ifndef 配对, __STM32F4xx_IT_H 已定义 */
