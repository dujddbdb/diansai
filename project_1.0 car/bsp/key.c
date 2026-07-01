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

    // 使能GPIOA/B/C端口时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);

    // 配置GPIO参数：输入模式、推挽、50MHz、上拉
    g.GPIO_Mode = GPIO_Mode_IN; g.GPIO_OType = GPIO_OType_PP; g.GPIO_Speed = GPIO_Speed_50MHz; g.GPIO_PuPd = GPIO_PuPd_UP;

    // 初始化GPIOA的PA3/4/5/6/7引脚
    g.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7; GPIO_Init(GPIOA, &g);
    // 初始化GPIOB的PB0引脚
    g.GPIO_Pin = GPIO_Pin_0; GPIO_Init(GPIOB, &g);
    // 初始化GPIOC的PC4/5引脚
    g.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5; GPIO_Init(GPIOC, &g);

    // 初始化按键状态和消抖计数器为0
    for (i = 0; i < KEY_COUNT; i++) { g_key_state[i] = 0; g_key_tick[i] = 0; }
}

// 按键扫描状态机（需周期性调用）: 消抖→状态转换
// 状态: 0=idle, KEY_PRESS=刚按下, KEY_HOLD=持续按住, KEY_RELEASE=刚释放
void Key_Scan(void)
{
    uint8_t i;

    // 遍历所有按键
    for (i = 0; i < KEY_COUNT; i++) {
        // 读取按键原始电平，按下为低电平（Bit_RESET）
        uint8_t raw = (GPIO_ReadInputDataBit(g_key[i].port, g_key[i].pin) == Bit_RESET);

        // 按键按下处理
        if (raw) {
            // 消抖计数未满，继续累加
            if (g_key_tick[i] < KEY_DEBOUNCE) {
                g_key_tick[i]++;
                // 消抖计数达到阈值，标记为刚按下
                if (g_key_tick[i] == KEY_DEBOUNCE) {
                    g_key_state[i] = KEY_PRESS;
                }
            }
            // 消抖完成，标记为持续按住
            else {
                g_key_state[i] = KEY_HOLD;
            }
        }
        // 按键释放处理
        else {
            // 之前已按下，标记为刚释放
            if (g_key_tick[i] >= KEY_DEBOUNCE) {
                g_key_state[i] = KEY_RELEASE;
            }
            // 之前未按下，保持空闲状态
            else {
                g_key_state[i] = 0;
            }
            // 清零消抖计数器
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

    // 遍历查找第一个刚按下的按键
    for (i = 0; i < KEY_COUNT; i++) {
        if (g_key_state[i] == KEY_PRESS) {
            // 消费掉该事件，状态转为持续按住
            g_key_state[i] = KEY_HOLD;
            return i;
        }
    }
    // 无按键触发
    return 0xFF;
}

// 检测是否有任意按键刚被按下，有返回1，无返回0
uint8_t Key_AnyPressed(void) {
    uint8_t i;

    // 遍历检查是否有按键处于刚按下状态
    for (i = 0; i < KEY_COUNT; i++) {
        if (g_key_state[i] == KEY_PRESS) {
            return 1;
        }
    }
    return 0;
}
