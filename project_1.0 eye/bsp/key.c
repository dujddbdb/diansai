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
    // 使能GPIOA/GPIOB/GPIOC时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
    // GPIO配置: 输入模式, 推挽, 50MHz, 上拉
    g.GPIO_Mode = GPIO_Mode_IN; g.GPIO_OType = GPIO_OType_PP; g.GPIO_Speed = GPIO_Speed_50MHz; g.GPIO_PuPd = GPIO_PuPd_UP;
    // PA3~PA7
    g.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7; GPIO_Init(GPIOA, &g);
    // PB0
    g.GPIO_Pin = GPIO_Pin_0; GPIO_Init(GPIOB, &g);
    // PC4/PC5
    g.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5; GPIO_Init(GPIOC, &g);
    // 清零状态数组
    for (i = 0; i < KEY_COUNT; i++) { g_key_state[i] = 0; g_key_tick[i] = 0; }
}

// 按键扫描状态机: 计数消抖
// 状态值: 0=空闲, 1=按下, 2=长按, 3=释放
// 按下时计数递增至阈值KEY_DEBOUNCE确认有效, 释放时计数清零
void Key_Scan(void)
{
    uint8_t i;
    for (i = 0; i < KEY_COUNT; i++) {
        // 读取按键原始状态 (低电平=按下)
        uint8_t raw = (GPIO_ReadInputDataBit(g_key[i].port, g_key[i].pin) == Bit_RESET);
        if (raw) {
            // 按键按下: 消抖计数递增
            if (g_key_tick[i] < KEY_DEBOUNCE) {
                g_key_tick[i]++;
                // 达到阈值, 确认为按下状态
                if (g_key_tick[i] == KEY_DEBOUNCE) g_key_state[i] = KEY_PRESS;
            } else {
                // 持续按下, 进入长按状态
                g_key_state[i] = KEY_HOLD;
            }
        } else {
            // 按键释放
            if (g_key_tick[i] >= KEY_DEBOUNCE) {
                // 之前是按下状态, 现在进入释放状态
                g_key_state[i] = KEY_RELEASE;
            } else {
                // 未达到阈值, 回到空闲状态
                g_key_state[i] = 0;
            }
            // 清零消抖计数
            g_key_tick[i] = 0;
        }
    }
}

// 获取按键状态
// idx: 按键索引(0~7), 返回状态值: 0=空闲/1=按下/2=长按/3=释放
uint8_t Key_GetState(uint8_t idx) {
    return (idx < KEY_COUNT) ? g_key_state[idx] : 0;
}

// 扫描获取第一个刚按下按键的索引并消费(转为KEY_HOLD)
// 无触发返回0xFF
uint8_t Key_GetTriggered(void)
{
    uint8_t i;
    for (i = 0; i < KEY_COUNT; i++) {
        if (g_key_state[i] == KEY_PRESS) {
            // 消费按下事件, 转为长按状态
            g_key_state[i] = KEY_HOLD;
            return i;
        }
    }
    return 0xFF;
}

// 检测是否有任意按键刚被按下, 返回1=有 0=无
uint8_t Key_AnyPressed(void) {
    uint8_t i;
    for (i = 0; i < KEY_COUNT; i++) {
        if (g_key_state[i] == KEY_PRESS) return 1;
    }
    return 0;
}
