/**
 * @file    peripheral.c
 * @brief   蜂鸣器 + 外部LED + 继电器 驱动实现
 * @note    使用peripheral.h中定义的引脚宏, 修改引脚只需改头文件
 */

#include "peripheral.h"

/* ==================== 蜂鸣器驱动 (无源, 低电平发声) ==================== */

// 初始化PD10为推挽输出, 初始高电平静音
void Buzzer_Init(void) {
    GPIO_InitTypeDef g;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    g.GPIO_Pin = BUZZER_PIN; g.GPIO_Mode = GPIO_Mode_OUT; g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz; g.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(BUZZER_PORT, &g); Buzzer_Off();
}

void Buzzer_On(void)     { GPIO_ResetBits(BUZZER_PORT, BUZZER_PIN); }
void Buzzer_Off(void)    { GPIO_SetBits(BUZZER_PORT, BUZZER_PIN); }
void Buzzer_Toggle(void) { GPIO_ToggleBits(BUZZER_PORT, BUZZER_PIN); }

/* ==================== 外部LED驱动 (低电平点亮, active-LOW) ==================== */

// 初始化PD11为推挽输出, 初始高电平熄灭
void ExtLED_Init(void) {
    GPIO_InitTypeDef g;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    g.GPIO_Pin = EXTLED_PIN; g.GPIO_Mode = GPIO_Mode_OUT; g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz; g.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(EXTLED_PORT, &g); ExtLED_Off();
}

void ExtLED_On(void)     { GPIO_ResetBits(EXTLED_PORT, EXTLED_PIN); }
void ExtLED_Off(void)    { GPIO_SetBits(EXTLED_PORT, EXTLED_PIN); }
void ExtLED_Toggle(void) { GPIO_ToggleBits(EXTLED_PORT, EXTLED_PIN); }

/* ==================== 继电器驱动 (PE7, 普通推挽输出) ====================
 * 吸合(ON): PE7=高电平, 断开(OFF): PE7=低电平
 */

// 初始化PE7为推挽输出, 初始低电平使继电器断开
void Relay_Init(void) {
    GPIO_InitTypeDef g;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    g.GPIO_Pin = RELAY_PIN;
    g.GPIO_Mode = GPIO_Mode_OUT;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(RELAY_PORT, &g);
    GPIO_ResetBits(RELAY_PORT, RELAY_PIN);
}

void Relay_On(void)     { GPIO_SetBits(RELAY_PORT, RELAY_PIN); }
void Relay_Off(void)    { GPIO_ResetBits(RELAY_PORT, RELAY_PIN); }
void Relay_Toggle(void) { GPIO_ToggleBits(RELAY_PORT, RELAY_PIN); }

