/**
 * @file    peripheral.h
 * @brief   蜂鸣器 + 外部LED + 继电器 统一驱动接口
 * @note    蜂鸣器(PD10): 无源蜂鸣器, 低电平发声, 无需PWM
 *          外部LED(PD11): 低电平点亮 (active-LOW)
 *          继电器(PE7): 高电平吸合/通电, 低电平断开
 *          全部为GPIO推挽输出, 按键控制开关
 */

#ifndef __PERIPHERAL_H__
#define __PERIPHERAL_H__

#include "stm32f4xx.h"

/**************************** 硬件引脚定义 ****************************/

/* 蜂鸣器GPIO端口定义: 蜂鸣器连接在GPIOD端口 */
#define BUZZER_PORT     GPIOD
/* 蜂鸣器引脚定义: 蜂鸣器连接在PD10引脚, 无源蜂鸣器低电平触发发声 */
#define BUZZER_PIN      GPIO_Pin_10

/* 外部LED GPIO端口定义: 外部LED连接在GPIOD端口 */
#define EXTLED_PORT     GPIOD
/* 外部LED引脚定义: LED连接在PD11引脚, 低电平点亮特性(active-LOW) */
#define EXTLED_PIN      GPIO_Pin_11

/* 继电器GPIO端口定义: 继电器连接在GPIOE端口 */
#define RELAY_PORT      GPIOE
/* 继电器引脚定义: 继电器控制引脚为PE7 */
#define RELAY_PIN       GPIO_Pin_7

/* ==================== 蜂鸣器驱动函数声明 ==================== */

/* 蜂鸣器初始化函数声明: 配置PD10为推挽输出, 初始设置为高电平静音状态 */
void Buzzer_Init(void);
/* 蜂鸣器开启函数声明: PD10输出低电平(调用GPIO_ResetBits)使蜂鸣器发声 */
void Buzzer_On(void);
/* 蜂鸣器关闭函数声明: PD10输出高电平(调用GPIO_SetBits)使蜂鸣器静音 */
void Buzzer_Off(void);
/* 蜂鸣器状态翻转函数声明: PD10引脚电平取反(调用GPIO_ToggleBits) */
void Buzzer_Toggle(void);

/* ==================== 外部LED驱动函数声明 ==================== */

/* 外部LED初始化函数声明: 配置PD11为推挽输出, 初始设置为高电平熄灭状态(低电平点亮特性) */
void ExtLED_Init(void);
/* 外部LED开启函数声明: PD11输出低电平(调用GPIO_ResetBits)点亮LED(active-LOW) */
void ExtLED_On(void);
/* 外部LED关闭函数声明: PD11输出高电平(调用GPIO_SetBits)熄灭LED */
void ExtLED_Off(void);
/* 外部LED状态翻转函数声明: PD11引脚电平取反(调用GPIO_ToggleBits) */
void ExtLED_Toggle(void);

/* ==================== 继电器驱动函数声明 (普通推挽输出, 硬件已修复) ==================== */

/* 继电器初始化函数声明: PE7配置为推挽输出, 初始设置为低电平使继电器断开 */
void Relay_Init(void);
/* 继电器吸合函数声明: PE7输出高电平使继电器吸合(通电) */
void Relay_On(void);
/* 继电器断开函数声明: PE7输出低电平使继电器断开(断电) */
void Relay_Off(void);
/* 继电器状态翻转函数声明: PE7引脚电平取反(调用GPIO_ToggleBits) */
void Relay_Toggle(void);

#endif
