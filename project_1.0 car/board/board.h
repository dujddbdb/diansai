/**
 * @file    board.h
 * @brief   STM32F407 板级支持包 - 系统时钟、SysTick、延时函数
 * @note    系统时钟: HSI 16MHz + PLL -> 168MHz (不依赖外部晶振)
 *          SysTick:  HCLK/8 = 21MHz, 1ms中断一次
 */

/**
 * 头文件条件编译保护开始
 * 如果__BOARD_H__宏未定义，则编译以下代码块；已定义则跳过整个头文件
 * 作用：防止头文件被重复包含导致编译错误
 */
#ifndef __BOARD_H__                                                           // 条件编译：检查__BOARD_H__是否已定义

/**
 * 定义__BOARD_H__宏，标记本头文件已被包含
 */
#define __BOARD_H__                                                           // 定义__BOARD_H__宏

/**
 * 包含STM32F4标准外设库头文件
 * 提供：
 *   - SCB、NVIC、SysTick等内核外设寄存器定义
 *   - SystemCoreClock等系统全局变量
 *   - uint32_t等标准数据类型
 * 注意：HSE_VALUE定义为8MHz（外部晶振频率）
 *       SystemCoreClock定义在system_stm32f4xx.c中为168MHz
 */
#include "stm32f4xx.h"                                                       // 包含STM32F4外设库头文件（HSE_VALUE=8MHz, SystemCoreClock=168MHz）

/**
 * 全局系统滴答计数器声明
 * volatile防止编译器优化，确保每次访问都从内存读取
 * __IO是volatile的宏定义别名
 * 用途：记录系统上电后的毫秒数
 * 更新方式：由SysTick中断每1ms递增一次
 */
extern __IO uint32_t g_system_tick;                                          // 全局系统滴答计数器声明（volatile类型，SysTick中断每1ms递增）

/**
 * 板级初始化函数声明
 * 功能：
 *   1. 设置中断向量表基址为FLASH起始地址(0x08000000)
 *   2. 配置SysTick为1ms定时中断
 * 调用时机：系统最开始启动时调用（在main函数之前）
 * 注意事项：此函数会启用SysTick中断和计数器
 */
void board_init(void);                                                        // 板级初始化函数声明：中断向量表 + SysTick配置

/**
 * 微秒级延时函数声明
 * 参数：_us - 延时微秒数
 * 实现方式：轮询SysTick递减计数器实现精确延时
 * 精度：取决于SysTick时钟频率和延时时间
 * 注意事项：
 *   - 不依赖中断，在中断被屏蔽时仍可使用
 *   - 最大延时受限于SysTick计数器位宽
 */
void delay_us(uint32_t _us);                                                  // 微秒级延时函数声明（轮询SysTick递减计数器）

/**
 * 毫秒级延时函数声明
 * 参数：_ms - 延时毫秒数
 * 实现方式：内部调用delay_us实现（ms * 1000转换为微秒）
 * 注意事项：
 *   - 长时间延时会阻塞CPU
 *   - 如果SysTick中断被关闭，延时会不准确
 */
void delay_ms(uint32_t _ms);                                                  // 毫秒级延时函数声明（内部调用delay_us）

/**
 * 获取系统运行时间函数声明
 * 返回值：系统上电后的毫秒数
 * 实现方式：返回全局变量g_system_tick的值
 * 用途：
 *   - 计算代码执行时间
 *   - 超时检测
 *   - 定时任务调度
 * 注意事项：
 *   - 如果SysTick中断被长时间阻塞，此值会停止增长
 *   - 约49.7天后会溢出回零
 */
uint32_t HAL_GetTick(void);                                                   // 获取系统运行毫秒数声明（返回g_system_tick）

/**
 * 头文件保护结束
 * 与文件开头的 #ifndef __BOARD_H__ 配对
 */
#endif  // __BOARD_H__                                                         // 头文件保护结束
