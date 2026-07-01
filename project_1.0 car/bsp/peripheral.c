// 蜂鸣器(PD10) + 外部LED(PD11) + 继电器(PE7) 驱动

#include "peripheral.h"

// 蜂鸣器初始化: PD10推挽输出, 初始高电平静音
void Buzzer_Init(void) {
    GPIO_InitTypeDef g;

    // 使能GPIOD端口时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    // 配置蜂鸣器引脚
    g.GPIO_Pin = BUZZER_PIN;          // 选择蜂鸣器对应引脚
    g.GPIO_Mode = GPIO_Mode_OUT;      // 设置为输出模式
    g.GPIO_OType = GPIO_OType_PP;     // 推挽输出
    g.GPIO_Speed = GPIO_Speed_50MHz;  // 输出速度50MHz
    g.GPIO_PuPd = GPIO_PuPd_NOPULL;   // 无上拉下拉
    GPIO_Init(BUZZER_PORT, &g);       // 初始化GPIO

    // 初始状态：蜂鸣器关闭（高电平静音）
    Buzzer_Off();
}

// 蜂鸣器开启（低电平有效）
void Buzzer_On(void) {
    GPIO_ResetBits(BUZZER_PORT, BUZZER_PIN);
}

// 蜂鸣器关闭（高电平静音）
void Buzzer_Off(void) {
    GPIO_SetBits(BUZZER_PORT, BUZZER_PIN);
}

// 蜂鸣器状态翻转
void Buzzer_Toggle(void) {
    GPIO_ToggleBits(BUZZER_PORT, BUZZER_PIN);
}

// 外部LED初始化: PD11推挽输出, 低电平点亮, 初始高电平熄灭
void ExtLED_Init(void) {
    GPIO_InitTypeDef g;

    // 使能GPIOD端口时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    // 配置外部LED引脚
    g.GPIO_Pin = EXTLED_PIN;          // 选择外部LED对应引脚
    g.GPIO_Mode = GPIO_Mode_OUT;      // 设置为输出模式
    g.GPIO_OType = GPIO_OType_PP;     // 推挽输出
    g.GPIO_Speed = GPIO_Speed_50MHz;  // 输出速度50MHz
    g.GPIO_PuPd = GPIO_PuPd_NOPULL;   // 无上拉下拉
    GPIO_Init(EXTLED_PORT, &g);       // 初始化GPIO

    // 初始状态：LED熄灭（高电平）
    ExtLED_Off();
}

// 外部LED点亮（低电平有效）
void ExtLED_On(void) {
    GPIO_ResetBits(EXTLED_PORT, EXTLED_PIN);
}

// 外部LED熄灭（高电平）
void ExtLED_Off(void) {
    GPIO_SetBits(EXTLED_PORT, EXTLED_PIN);
}

// 外部LED状态翻转
void ExtLED_Toggle(void) {
    GPIO_ToggleBits(EXTLED_PORT, EXTLED_PIN);
}

// 继电器初始化: PE7推挽输出, 高电平吸合, 初始低电平断开
void Relay_Init(void) {
    GPIO_InitTypeDef g;

    // 使能GPIOE端口时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    // 配置继电器引脚
    g.GPIO_Pin = RELAY_PIN;           // 选择继电器对应引脚
    g.GPIO_Mode = GPIO_Mode_OUT;      // 设置为输出模式
    g.GPIO_OType = GPIO_OType_PP;     // 推挽输出
    g.GPIO_Speed = GPIO_Speed_50MHz;  // 输出速度50MHz
    g.GPIO_PuPd = GPIO_PuPd_NOPULL;   // 无上拉下拉
    GPIO_Init(RELAY_PORT, &g);        // 初始化GPIO

    // 初始状态：继电器断开（低电平）
    GPIO_ResetBits(RELAY_PORT, RELAY_PIN);
}

// 继电器吸合（高电平有效）
void Relay_On(void) {
    GPIO_SetBits(RELAY_PORT, RELAY_PIN);
}

// 继电器断开（低电平）
void Relay_Off(void) {
    GPIO_ResetBits(RELAY_PORT, RELAY_PIN);
}

// 继电器状态翻转
void Relay_Toggle(void) {
    GPIO_ToggleBits(RELAY_PORT, RELAY_PIN);
}
