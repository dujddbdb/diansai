#ifndef __PERIPHERAL_H__
#define __PERIPHERAL_H__

#include "stm32f4xx.h"

// 蜂鸣器GPIO端口（GPIOD）
#define BUZZER_PORT     GPIOD
// 蜂鸣器引脚（PD10，低电平触发发声）
#define BUZZER_PIN      GPIO_Pin_10
// 外部LED GPIO端口（GPIOD）
#define EXTLED_PORT     GPIOD
// 外部LED引脚（PD11，低电平点亮）
#define EXTLED_PIN      GPIO_Pin_11
// 继电器GPIO端口（GPIOE）
#define RELAY_PORT      GPIOE
// 继电器引脚（PE7，高电平吸合）
#define RELAY_PIN       GPIO_Pin_7

// 蜂鸣器初始化
// 功能：配置PD10为推挽输出模式，初始状态为高电平（静音）
// 参数：无
// 返回值：无
void Buzzer_Init(void);

// 蜂鸣器开启
// 功能：将PD10置为低电平，蜂鸣器开始发声
// 参数：无
// 返回值：无
void Buzzer_On(void);

// 蜂鸣器关闭
// 功能：将PD10置为高电平，蜂鸣器停止发声
// 参数：无
// 返回值：无
void Buzzer_Off(void);

// 蜂鸣器电平翻转
// 功能：翻转PD10的输出电平，实现蜂鸣器状态切换
// 参数：无
// 返回值：无
void Buzzer_Toggle(void);

// 外部LED初始化
// 功能：配置PD11为推挽输出模式，初始状态为高电平（熄灭）
// 参数：无
// 返回值：无
void ExtLED_Init(void);

// LED点亮
// 功能：将PD11置为低电平，外部LED点亮
// 参数：无
// 返回值：无
void ExtLED_On(void);

// LED熄灭
// 功能：将PD11置为高电平，外部LED熄灭
// 参数：无
// 返回值：无
void ExtLED_Off(void);

// LED状态翻转
// 功能：翻转PD11的输出电平，实现LED亮灭切换
// 参数：无
// 返回值：无
void ExtLED_Toggle(void);

// 继电器初始化
// 功能：配置PE7为推挽输出模式，初始状态为低电平（断开）
// 参数：无
// 返回值：无
void Relay_Init(void);

// 继电器吸合
// 功能：将PE7置为高电平，继电器吸合导通
// 参数：无
// 返回值：无
void Relay_On(void);

// 继电器断开
// 功能：将PE7置为低电平，继电器断开
// 参数：无
// 返回值：无
void Relay_Off(void);

// 继电器状态翻转
// 功能：翻转PE7的输出电平，实现继电器通断切换
// 参数：无
// 返回值：无
void Relay_Toggle(void);

#endif
