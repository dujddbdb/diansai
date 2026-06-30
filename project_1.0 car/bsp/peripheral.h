#ifndef __PERIPHERAL_H__
#define __PERIPHERAL_H__

#include "stm32f4xx.h"

#define BUZZER_PORT     GPIOD       // 蜂鸣器GPIO端口
#define BUZZER_PIN      GPIO_Pin_10 // 蜂鸣器引脚(PD10,低电平发声)
#define EXTLED_PORT     GPIOD       // 外部LED GPIO端口
#define EXTLED_PIN      GPIO_Pin_11 // 外部LED引脚(PD11,低电平点亮)
#define RELAY_PORT      GPIOE       // 继电器GPIO端口
#define RELAY_PIN       GPIO_Pin_7  // 继电器引脚(PE7,高电平吸合)

// 蜂鸣器初始化(PD10推挽输出,初始高电平静音)
void Buzzer_Init(void);
// 蜂鸣器开启(低电平)
void Buzzer_On(void);
// 蜂鸣器关闭(高电平)
void Buzzer_Off(void);
// 蜂鸣器电平翻转
void Buzzer_Toggle(void);

// 外部LED初始化(PD11推挽输出,初始高电平熄灭)
void ExtLED_Init(void);
// LED点亮(低电平)
void ExtLED_On(void);
// LED熄灭(高电平)
void ExtLED_Off(void);
// LED状态翻转
void ExtLED_Toggle(void);

// 继电器初始化(PE7推挽输出,初始低电平断开)
void Relay_Init(void);
// 继电器吸合(高电平)
void Relay_On(void);
// 继电器断开(低电平)
void Relay_Off(void);
// 继电器状态翻转
void Relay_Toggle(void);

#endif
