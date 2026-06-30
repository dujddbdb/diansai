// 8按键扫描驱动: PA3/PA4/PA5/PA6/PA7/PB0/PC4/PC5, 内部上拉, 按下=低

#include "key.h"

// 按键GPIO定义表（端口, 引脚）
static const KeyDef_t g_key[KEY_COUNT] = {
    {GPIOA, GPIO_Pin_7},
    {GPIOC, GPIO_Pin_4},
    {GPIOC, GPIO_Pin_5},
    {GPIOA, GPIO_Pin_6},
    {GPIOA, GPIO_Pin_3},
    {GPIOA, GPIO_Pin_4},
    {GPIOA, GPIO_Pin_5},
    {GPIOB, GPIO_Pin_0},
};

// 按键状态缓冲区和消抖计数器
static uint8_t g_key_state[KEY_COUNT], g_key_tick[KEY_COUNT];

// 初始化按键GPIO: 输入模式+内部上拉
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

// 按键扫描状态机（需周期性调用）: 消抖→状态转换
// 状态: 0=idle, KEY_PRESS=刚按下, KEY_HOLD=持续按住, KEY_RELEASE=刚释放
void Key_Scan(void)
{
    uint8_t i;

    for (i = 0; i < KEY_COUNT; i++) {
        uint8_t raw = (GPIO_ReadInputDataBit(g_key[i].port, g_key[i].pin) == Bit_RESET);

        if (raw) {
            if (g_key_tick[i] < KEY_DEBOUNCE) {
                g_key_tick[i]++;
                if (g_key_tick[i] == KEY_DEBOUNCE) {
                    g_key_state[i] = KEY_PRESS;
                }
            }
            else {
                g_key_state[i] = KEY_HOLD;
            }
        }
        else {
            if (g_key_tick[i] >= KEY_DEBOUNCE) {
                g_key_state[i] = KEY_RELEASE;
            }
            else {
                g_key_state[i] = 0;
            }
            g_key_tick[i] = 0;
        }
    }
}

// 获取按键状态值, idx=按键索引(0~7), 返回状态值
uint8_t Key_GetState(uint8_t idx) {
    return (idx < KEY_COUNT) ? g_key_state[idx] : 0;
}

// 获取第一个刚按下的按键索引（消费型），无触发返回0xFF
uint8_t Key_GetTriggered(void)
{
    uint8_t i;

    for (i = 0; i < KEY_COUNT; i++) {
        if (g_key_state[i] == KEY_PRESS) {
            g_key_state[i] = KEY_HOLD;
            return i;
        }
    }
    return 0xFF;
}

// 检测是否有任意按键刚被按下，有返回1，无返回0
uint8_t Key_AnyPressed(void) {
    uint8_t i;

    for (i = 0; i < KEY_COUNT; i++) {
        if (g_key_state[i] == KEY_PRESS) {
            return 1;
        }
    }
    return 0;
}