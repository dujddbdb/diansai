// 蜂鸣器(PD10) + 外部LED(PD11) + 继电器(PE7) 驱动

#include "peripheral.h"

// 蜂鸣器初始化: PD10推挽输出, 初始高电平静音
void Buzzer_Init(void) {
    GPIO_InitTypeDef g;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    g.GPIO_Pin = BUZZER_PIN;
    g.GPIO_Mode = GPIO_Mode_OUT;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(BUZZER_PORT, &g);

    Buzzer_Off();
}

void Buzzer_On(void) {
    GPIO_ResetBits(BUZZER_PORT, BUZZER_PIN);
}

void Buzzer_Off(void) {
    GPIO_SetBits(BUZZER_PORT, BUZZER_PIN);
}

void Buzzer_Toggle(void) {
    GPIO_ToggleBits(BUZZER_PORT, BUZZER_PIN);
}

// 外部LED初始化: PD11推挽输出, 低电平点亮, 初始高电平熄灭
void ExtLED_Init(void) {
    GPIO_InitTypeDef g;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    g.GPIO_Pin = EXTLED_PIN;
    g.GPIO_Mode = GPIO_Mode_OUT;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(EXTLED_PORT, &g);

    ExtLED_Off();
}

void ExtLED_On(void) {
    GPIO_ResetBits(EXTLED_PORT, EXTLED_PIN);
}

void ExtLED_Off(void) {
    GPIO_SetBits(EXTLED_PORT, EXTLED_PIN);
}

void ExtLED_Toggle(void) {
    GPIO_ToggleBits(EXTLED_PORT, EXTLED_PIN);
}

// 继电器初始化: PE7推挽输出, 高电平吸合, 初始低电平断开
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

void Relay_On(void) {
    GPIO_SetBits(RELAY_PORT, RELAY_PIN);
}

void Relay_Off(void) {
    GPIO_ResetBits(RELAY_PORT, RELAY_PIN);
}

void Relay_Toggle(void) {
    GPIO_ToggleBits(RELAY_PORT, RELAY_PIN);
}