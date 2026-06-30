/**
 * @file    stm32f4xx_conf.h
 * @brief   STM32F4xx 标准外设库配置文件
 * @note    按需引入外设头文件, 支持条件编译(STM32F40_41xxx)
 *          定义assert_param宏用于参数检查
 */

#ifndef __STM32F4XX_CONF_H           /* 防止头文件重复包含 */
#define __STM32F4XX_CONF_H           /* 定义头文件标识宏 */

#include "stm32f4xx_adc.h"           /* 模数转换器(ADC) */
#include "stm32f4xx_crc.h"           /* CRC校验计算 */
#include "stm32f4xx_dbgmcu.h"        /* 调试MCU(Debug MCU) */
#include "stm32f4xx_dma.h"           /* 直接存储器访问(DMA) */
#include "stm32f4xx_exti.h"          /* 外部中断/事件控制器(EXTI) */
#include "stm32f4xx_flash.h"         /* 内部Flash编程 */
#include "stm32f4xx_gpio.h"          /* 通用输入输出(GPIO) */
#include "stm32f4xx_i2c.h"           /* I2C通信接口 */
#include "stm32f4xx_iwdg.h"          /* 独立看门狗(IWDG) */
#include "stm32f4xx_pwr.h"           /* 电源控制(PWR) */
#include "stm32f4xx_rcc.h"           /* 复位与时钟控制(RCC) */
#include "stm32f4xx_rtc.h"           /* 实时时钟(RTC) */
#include "stm32f4xx_sdio.h"          /* SDIO接口(SD卡通信) */
#include "stm32f4xx_spi.h"           /* SPI通信接口 */
#include "stm32f4xx_syscfg.h"        /* 系统配置控制器(SYSCFG) */
#include "stm32f4xx_tim.h"           /* 定时器(TIM) */
#include "stm32f4xx_usart.h"         /* 通用同步异步收发器(USART/串口) */
#include "stm32f4xx_wwdg.h"          /* 窗口看门狗(WWDG) */
#include "misc.h"                    /* 杂项驱动(NVIC中断优先级/SysTick等) */

#if defined(STM32F40_41xxx)          /* 条件编译: 仅STM32F40/41系列 */
#include "stm32f4xx_cryp.h"          /* 加密处理器(CRYP) */
#include "stm32f4xx_hash.h"          /* HASH哈希处理器 */
#include "stm32f4xx_rng.h"           /* 随机数发生器(RNG) */
#include "stm32f4xx_can.h"           /* CAN总线通信 */
#include "stm32f4xx_dac.h"           /* 数模转换器(DAC) */
#include "stm32f4xx_dcmi.h"          /* 数字摄像头接口(DCMI) */
#include "stm32f4xx_fsmc.h"          /* 灵活静态存储器控制器(FSMC) */
#endif

#ifdef USE_FULL_ASSERT                /* 使能断言时: 参数表达式为假则调用assert_failed */
#define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__)) /* 断言宏: 检查参数有效性 */
void assert_failed(uint8_t* file, uint32_t line);  /* 断言失败回调函数 */
#else
#define assert_param(expr) ((void)0) /* 未使能断言时: 空操作 */
#endif

#endif /* __STM32F4XX_CONF_H */
