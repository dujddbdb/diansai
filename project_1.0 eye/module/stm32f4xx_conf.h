/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_conf.h  
  * @author  MCD Application Team
  * @version V1.8.1
  * @date    27-January-2022
  * @brief   Library configuration file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_CONF_H              /* 防止头文件重复包含 */
#define __STM32F4xx_CONF_H              /* 定义头文件标识宏 */

/* Includes ------------------------------------------------------------------*/
/* Uncomment the line below to enable peripheral header file inclusion */
#include "stm32f4xx_adc.h"              /* 模数转换器(ADC) */
#include "stm32f4xx_crc.h"              /* CRC校验计算 */
#include "stm32f4xx_dbgmcu.h"           /* 调试MCU(Debug MCU) */
#include "stm32f4xx_dma.h"              /* 直接存储器访问(DMA) */
#include "stm32f4xx_exti.h"             /* 外部中断/事件控制器(EXTI) */
#include "stm32f4xx_flash.h"            /* 内部Flash编程 */
#include "stm32f4xx_gpio.h"             /* 通用输入输出(GPIO) */
#include "stm32f4xx_i2c.h"              /* I2C通信接口 */
#include "stm32f4xx_iwdg.h"             /* 独立看门狗(IWDG) */
#include "stm32f4xx_pwr.h"              /* 电源控制(PWR) */
#include "stm32f4xx_rcc.h"              /* 复位与时钟控制(RCC) */
#include "stm32f4xx_rtc.h"              /* 实时时钟(RTC) */
#include "stm32f4xx_sdio.h"             /* SDIO接口(SD卡通信) */
#include "stm32f4xx_spi.h"              /* SPI通信接口 */
#include "stm32f4xx_syscfg.h"           /* 系统配置控制器(SYSCFG) */
#include "stm32f4xx_tim.h"              /* 定时器(TIM) */
#include "stm32f4xx_usart.h"            /* 通用同步异步收发器(USART/串口) */
#include "stm32f4xx_wwdg.h"             /* 窗口看门狗(WWDG) */
#include "misc.h"                       /* 杂项驱动(NVIC中断优先级/SysTick等) */

#if defined(STM32F429_439xx) || defined(STM32F446xx) || defined(STM32F469_479xx) /* F429/F439/F446/F469/F479系列 */
#include "stm32f4xx_cryp.h"               /* 加密处理器(CRYP) */
#include "stm32f4xx_hash.h"               /* HASH哈希处理器 */
#include "stm32f4xx_rng.h"                /* 随机数发生器(RNG) */
#include "stm32f4xx_can.h"                /* CAN总线通信 */
#include "stm32f4xx_dac.h"                /* 数模转换器(DAC) */
#include "stm32f4xx_dcmi.h"               /* 数字摄像头接口(DCMI) */
#include "stm32f4xx_dma2d.h"              /* DMA2D图形加速器 */
#include "stm32f4xx_fmc.h"                /* 灵活存储器控制器(FMC) */
#include "stm32f4xx_ltdc.h"               /* LCD-TFT显示控制器(LTDC) */
#include "stm32f4xx_sai.h"                /* 串行音频接口(SAI) */
#endif /* STM32F429_439xx || STM32F446xx || STM32F469_479xx */

#if defined(STM32F427_437xx)              /* F427/F437系列 */
#include "stm32f4xx_cryp.h"               /* 加密处理器(CRYP) */
#include "stm32f4xx_hash.h"               /* HASH哈希处理器 */
#include "stm32f4xx_rng.h"                /* 随机数发生器(RNG) */
#include "stm32f4xx_can.h"                /* CAN总线通信 */
#include "stm32f4xx_dac.h"                /* 数模转换器(DAC) */
#include "stm32f4xx_dcmi.h"               /* 数字摄像头接口(DCMI) */
#include "stm32f4xx_dma2d.h"              /* DMA2D图形加速器 */
#include "stm32f4xx_fmc.h"                /* 灵活存储器控制器(FMC) */
#include "stm32f4xx_sai.h"                /* 串行音频接口(SAI) */
#endif /* STM32F427_437xx */

#if defined(STM32F40_41xxx)               /* F40/F41系列 */
#include "stm32f4xx_cryp.h"               /* 加密处理器(CRYP) */
#include "stm32f4xx_hash.h"               /* HASH哈希处理器 */
#include "stm32f4xx_rng.h"                /* 随机数发生器(RNG) */
#include "stm32f4xx_can.h"                /* CAN总线通信 */
#include "stm32f4xx_dac.h"                /* 数模转换器(DAC) */
#include "stm32f4xx_dcmi.h"               /* 数字摄像头接口(DCMI) */
#include "stm32f4xx_fsmc.h"               /* 灵活静态存储器控制器(FSMC) */
#endif /* STM32F40_41xxx */

#if defined(STM32F410xx)                  /* F410系列 */
#include "stm32f4xx_rng.h"                /* 随机数发生器(RNG) */
#include "stm32f4xx_dac.h"                /* 数模转换器(DAC) */
#endif /* STM32F410xx */

#if defined(STM32F411xE)                  /* F411xE系列 */
#include "stm32f4xx_flash_ramfunc.h"       /* Flash中RAM函数重映射 */
#endif /* STM32F411xE */

#if defined(STM32F446xx) || defined(STM32F469_479xx) /* F446/F469/F479系列 */
#include "stm32f4xx_qspi.h"               /* 四线SPI接口(QSPI) */
#endif /* STM32F446xx || STM32F469_479xx */

#if defined(STM32F410xx) || defined(STM32F446xx) /* F410/F446系列 */
#include "stm32f4xx_fmpi2c.h"             /* 快速模式增强型I2C(FMPI2C) */
#endif /* STM32F410xx || STM32F446xx */

#if defined(STM32F446xx)                  /* F446系列 */
#include "stm32f4xx_spdifrx.h"            /* SPDIF-RX数字音频接收 */
#include "stm32f4xx_cec.h"                /* CEC消费电子控制 */
#endif /* STM32F446xx */

#if defined(STM32F469_479xx)              /* F469/F479系列 */
#include "stm32f4xx_dsi.h"                /* DSI显示串行接口 */
#endif /* STM32F469_479xx */

#if defined(STM32F410xx)                  /* F410系列 */
#include "stm32f4xx_lptim.h"              /* 低功耗定时器(LPTIM) */
#endif /* STM32F410xx */

#if defined(STM32F412xG)                  /* F412xG系列 */
#include "stm32f4xx_rng.h"                /* 随机数发生器(RNG) */
#include "stm32f4xx_can.h"                /* CAN总线通信 */
#include "stm32f4xx_qspi.h"               /* 四线SPI接口(QSPI) */
#include "stm32f4xx_rng.h"                /* 随机数发生器(RNG) */
#include "stm32f4xx_fsmc.h"               /* 灵活静态存储器控制器(FSMC) */
#include "stm32f4xx_dfsdm.h"              /* 数字滤波器Sigma-Delta调制器(DFSDM) */
#endif /* STM32F412xG */

#if defined(STM32F413_423xx)              /* F413/F423系列 */
#include "stm32f4xx_cryp.h"               /* 加密处理器(CRYP) */
#include "stm32f4xx_fmpi2c.h"             /* 快速模式增强型I2C(FMPI2C) */
#include "stm32f4xx_rng.h"                /* 随机数发生器(RNG) */
#include "stm32f4xx_can.h"                /* CAN总线通信 */
#include "stm32f4xx_qspi.h"               /* 四线SPI接口(QSPI) */
#include "stm32f4xx_rng.h"                /* 随机数发生器(RNG) */
#include "stm32f4xx_fsmc.h"               /* 灵活静态存储器控制器(FSMC) */
#include "stm32f4xx_dfsdm.h"              /* 数字滤波器Sigma-Delta调制器(DFSDM) */
#endif /* STM32F413_423xx */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* If an external clock source is used, then the value of the following define 
   should be set to the value of the external clock source, else, if no external 
   clock is used, keep this define commented */
/*#define I2S_EXTERNAL_CLOCK_VAL   12288000 */ /* Value of the external clock in Hz */


/* Uncomment the line below to expanse the "assert_param" macro in the 
   Standard Peripheral Library drivers code */
/* #define USE_FULL_ASSERT    1 */

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT              /* 使能断言时 */

/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function
  *   which reports the name of the source file and the source
  *   line number of the call that failed. 
  *   If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__)) /* 断言宏: 检查参数有效性 */
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);  /* 断言失败回调函数 */
#else
  #define assert_param(expr) ((void)0) /* 未使能断言时: 空操作 */
#endif /* USE_FULL_ASSERT */

#endif /* __STM32F4xx_CONF_H */     /* 头文件包含保护结束 */

