// @file    key.c
// @brief   8个独立按键扫描驱动
// @note    PC4/PC5/PB0/PA3/PA4/PA5/PA6/PA7, 内部上拉, 按下=低

#include "key.h"

// 按键GPIO定义表: BTN0=PA3, BTN1=PA4, BTN2=PA5, BTN3=PA6,
//                 BTN4=PA7, BTN5=PB0, BTN6=PC4, BTN7=PC5
static const KeyDef_t g_key[KEY_COUNT] = {
    {GPIOA, GPIO_Pin_3},
    {GPIOA, GPIO_Pin_4},
    {GPIOA, GPIO_Pin_5},
    {GPIOA, GPIO_Pin_6},
    {GPIOA, GPIO_Pin_7},
    {GPIOB, GPIO_Pin_0},
    {GPIOC, GPIO_Pin_4},
    {GPIOC, GPIO_Pin_5},
};

// 按键状态数组: g_key_state[i]=当前状态, g_key_tick[i]=消抖计数
static uint8_t g_key_state[KEY_COUNT], g_key_tick[KEY_COUNT];

// 初始化所有按键GPIO (输入模式+内部上拉) 并清零状态
void Key_Init(void)
{
    GPIO_InitTypeDef g;
    uint8_t i;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
    g.GPIO_Mode = GPIO_Mode_IN; g.GPIO_OType = GPIO_OType_PP; g.GPIO_Speed = GPIO_Speed_50MHz; g.GPIO_PuPd = GPIO_PuPd_UP;
    g.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7; GPIO_Init(GPIOA, &g);
    g.GPIO_Pin = GPIO_Pin_0; GPIO_Init(GPIOB, &g);
    g.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5; GPIO_Init(GPIOC, &g);
    for (i = 0; i < KEY_COUNT; i++) { g_key_state[i] = 0; g_key_tick[i] = 0; }
}

// 按键扫描状态机: 计数消抖, 状态值: 0=空闲, 1=按下, 2=长按, 3=释放
// 按下时计数递增至阈值10确认有效, 释放时计数清零
void Key_Scan(void)
{
    uint8_t i;
    for (i = 0; i < KEY_COUNT; i++) {
        uint8_t raw = (GPIO_ReadInputDataBit(g_key[i].port, g_key[i].pin) == Bit_RESET);
        if (raw) {
            if (g_key_tick[i] < KEY_DEBOUNCE) { g_key_tick[i]++; if (g_key_tick[i] == KEY_DEBOUNCE) g_key_state[i] = KEY_PRESS; }
            else g_key_state[i] = KEY_HOLD;
        } else {
            if (g_key_tick[i] >= KEY_DEBOUNCE) g_key_state[i] = KEY_RELEASE;
            else g_key_state[i] = 0;
            g_key_tick[i] = 0;
        }
    }
}

// 获取按键状态, idx: 按键索引(0~7), 返回状态值: 0=空闲/1=按下/2=长按/3=释放
uint8_t Key_GetState(uint8_t idx) { return (idx < KEY_COUNT) ? g_key_state[idx] : 0; }

// 扫描获取第一个刚按下按键的索引并消费(转为KEY_HOLD), 无触发返回0xFF
uint8_t Key_GetTriggered(void)
{
    uint8_t i;
    for (i = 0; i < KEY_COUNT; i++) if (g_key_state[i] == KEY_PRESS) { g_key_state[i] = KEY_HOLD; return i; }
    return 0xFF;
}

// 检测是否有任意按键刚被按下, 返回1=有 0=无
uint8_t Key_AnyPressed(void) { uint8_t i; for (i = 0; i < KEY_COUNT; i++) if (g_key_state[i] == KEY_PRESS) return 1; return 0; }