// @file    peripheral.c
// @brief   蜂鸣器 + 外部LED + 继电器 驱动实现
// @note    使用peripheral.h中定义的引脚宏, 修改引脚只需改头文件

#include "peripheral.h"

// ==================== 蜂鸣器驱动 (无源, 低电平发声) ====================

// 初始化蜂鸣器GPIO: PD10推挽输出, 初始高电平静音
void Buzzer_Init(void) {
    GPIO_InitTypeDef g;
    // 使能GPIOD时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    // 配置引脚: 推挽输出, 50MHz, 无上拉下拉
    g.GPIO_Pin = BUZZER_PIN; g.GPIO_Mode = GPIO_Mode_OUT; g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz; g.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(BUZZER_PORT, &g);
    // 初始关闭蜂鸣器
    Buzzer_Off();
}

// 蜂鸣器开启 (低电平有效)
void Buzzer_On(void)     { GPIO_ResetBits(BUZZER_PORT, BUZZER_PIN); }
// 蜂鸣器关闭 (高电平无效)
void Buzzer_Off(void)    { GPIO_SetBits(BUZZER_PORT, BUZZER_PIN); }
// 蜂鸣器翻转
void Buzzer_Toggle(void) { GPIO_ToggleBits(BUZZER_PORT, BUZZER_PIN); }

// ==================== 外部LED驱动 (低电平点亮, active-LOW) ====================

// 初始化外部LED: PD11推挽输出, 初始高电平熄灭
void ExtLED_Init(void) {
    GPIO_InitTypeDef g;
    // 使能GPIOD时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    // 配置引脚: 推挽输出, 50MHz, 无上拉下拉
    g.GPIO_Pin = EXTLED_PIN; g.GPIO_Mode = GPIO_Mode_OUT; g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz; g.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(EXTLED_PORT, &g);
    // 初始关闭LED
    ExtLED_Off();
}

// 外部LED开启 (低电平有效)
void ExtLED_On(void)     { GPIO_ResetBits(EXTLED_PORT, EXTLED_PIN); }
// 外部LED关闭 (高电平无效)
void ExtLED_Off(void)    { GPIO_SetBits(EXTLED_PORT, EXTLED_PIN); }
// 外部LED翻转
void ExtLED_Toggle(void) { GPIO_ToggleBits(EXTLED_PORT, EXTLED_PIN); }

// ==================== 继电器驱动 (PE7, 普通推挽输出) ====================
// 吸合(ON): PE7=高电平, 断开(OFF): PE7=低电平

// 初始化继电器: PE7推挽输出, 初始低电平断开
void Relay_Init(void) {
    GPIO_InitTypeDef g;
    // 使能GPIOE时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    // 配置引脚: 推挽输出, 50MHz, 无上拉下拉
    g.GPIO_Pin = RELAY_PIN;
    g.GPIO_Mode = GPIO_Mode_OUT;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(RELAY_PORT, &g);
    // 初始断开继电器
    GPIO_ResetBits(RELAY_PORT, RELAY_PIN);
}

// 继电器吸合 (高电平有效)
void Relay_On(void)     { GPIO_SetBits(RELAY_PORT, RELAY_PIN); }
// 继电器断开 (低电平无效)
void Relay_Off(void)    { GPIO_ResetBits(RELAY_PORT, RELAY_PIN); }
// 继电器翻转
void Relay_Toggle(void) { GPIO_ToggleBits(RELAY_PORT, RELAY_PIN); }
