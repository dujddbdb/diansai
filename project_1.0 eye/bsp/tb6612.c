// @file    tb6612.c
// @brief   TB6612双路直流电机驱动实现
// @note    TIM1_CH1(PE9)=PWMA, TIM1_CH2(PE11)=PWMB
//          A路: AIN1=PE10, AIN2=PE8
//          B路: BIN1=PE12, BIN2=PE15
//          PWM频率=168MHz/(pre*per), 占空比=speed/per

#include "tb6612.h"

// 功能: 初始化AIN/BIN方向控制GPIO
// PE8/PE10/PE12/PE15配置为推挽输出, 初始低电平刹车
static void AIN_BIN_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio;

    // 使能GPIOE时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    // GPIO配置: 推挽输出, 50MHz, 无上拉下拉
    gpio.GPIO_Mode  = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    // 分别初始化4个方向控制引脚
    gpio.GPIO_Pin = GPIO_Pin_10; GPIO_Init(GPIOE, &gpio);  // PE10=AIN1
    gpio.GPIO_Pin = GPIO_Pin_8;  GPIO_Init(GPIOE, &gpio);  // PE8=AIN2
    gpio.GPIO_Pin = GPIO_Pin_12; GPIO_Init(GPIOE, &gpio);  // PE12=BIN1
    gpio.GPIO_Pin = GPIO_Pin_15; GPIO_Init(GPIOE, &gpio);  // PE15=BIN2

    // 初始输出低电平, 刹车状态
    GPIO_ResetBits(GPIOE, GPIO_Pin_10 | GPIO_Pin_8 | GPIO_Pin_12 | GPIO_Pin_15);
}

// TB6612初始化: 方向GPIO + TIM1高级定时器PWM输出
// PWM频率 = TIM1_CLK / pre / per, 示例: 168MHz / 84 / 1000 = 2kHz
void TB6612_Init(uint16_t pre, uint16_t per)
{
    GPIO_InitTypeDef        gpio;
    TIM_TimeBaseInitTypeDef tim_base;
    TIM_OCInitTypeDef       tim_oc;

    // 初始化方向控制GPIO
    AIN_BIN_GPIO_Init();

    // 使能TIM1时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    // GPIO配置: 复用功能, 推挽输出, 50MHz, 上拉
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;

    // PE9配置为TIM1_CH1 (PWMA)
    gpio.GPIO_Pin = GPIO_Pin_9; GPIO_Init(GPIOE, &gpio);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);

    // PE11配置为TIM1_CH2 (PWMB)
    gpio.GPIO_Pin = GPIO_Pin_11; GPIO_Init(GPIOE, &gpio);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);

    // TIM1时基配置: 向上计数, 不分频
    TIM_TimeBaseStructInit(&tim_base);
    tim_base.TIM_Prescaler         = pre - 1;
    tim_base.TIM_Period            = per - 1;
    tim_base.TIM_ClockDivision     = TIM_CKD_DIV1;
    tim_base.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &tim_base);

    // PWM模式1: CNT<CCR时高电平, 初始占空比0%
    TIM_OCStructInit(&tim_oc);
    tim_oc.TIM_OCMode      = TIM_OCMode_PWM1;
    tim_oc.TIM_OutputState = TIM_OutputState_Enable;
    tim_oc.TIM_Pulse       = 0;
    tim_oc.TIM_OCPolarity  = TIM_OCPolarity_High;

    // 通道1 PWM输出配置 + 预装载使能
    TIM_OC1Init(TIM1, &tim_oc);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    // 通道2 PWM输出配置 + 预装载使能
    TIM_OC2Init(TIM1, &tim_oc);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

    // 使能TIM1计数器
    TIM_Cmd(TIM1, ENABLE);
    // 高级定时器MOE必须使能, 否则PWM不输出
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

// H桥真值表:
// ┌──────┬──────┬─────────────────────────────────────┐
// │ AIN1 │ AIN2 │          电机状态                    │
// ├──────┼──────┼─────────────────────────────────────┤
// │  1   │  0   │ 正转 — 电流路径: AOUT1 → AOUT2      │
// │  0   │  1   │ 反转 — 电流路径: AOUT2 → AOUT1      │
// │  0   │  0   │ 刹车 — 下桥臂导通, 电机线圈短接接地   │
// │  1   │  1   │ 刹车 — 上桥臂导通, 电机线圈短接VCC    │
// └──────┴──────┴─────────────────────────────────────┘

// A路电机控制(左轮), dir: 0=正转 1=反转, speed: PWM比较值 0~per
void TB6612_ACtrl(uint8_t dir, uint32_t speed)
{
    // 根据方向设置AIN1/AIN2电平
    if (dir) {
        GPIO_ResetBits(GPIOE, GPIO_Pin_10);  // AIN1=0
        GPIO_SetBits(GPIOE, GPIO_Pin_8);     // AIN2=1
    } else {
        GPIO_SetBits(GPIOE, GPIO_Pin_10);    // AIN1=1
        GPIO_ResetBits(GPIOE, GPIO_Pin_8);   // AIN2=0
    }
    // 设置PWM占空比
    TIM_SetCompare1(TIM1, speed);
}

// B路电机控制(右轮), dir: 0=正转 1=反转, speed: PWM比较值 0~per
void TB6612_BCtrl(uint8_t dir, uint32_t speed)
{
    // 根据方向设置BIN1/BIN2电平
    if (dir) {
        GPIO_ResetBits(GPIOE, GPIO_Pin_12);  // BIN1=0
        GPIO_SetBits(GPIOE, GPIO_Pin_15);    // BIN2=1
    } else {
        GPIO_SetBits(GPIOE, GPIO_Pin_12);    // BIN1=1
        GPIO_ResetBits(GPIOE, GPIO_Pin_15);  // BIN2=0
    }
    // 设置PWM占空比
    TIM_SetCompare2(TIM1, speed);
}

// 双路同时控制: A路(左轮)+B路(右轮)
void TB6612_CarCtrl(uint8_t a_dir, uint32_t a_speed,
                    uint8_t b_dir, uint32_t b_speed)
{
    TB6612_ACtrl(a_dir, a_speed);
    TB6612_BCtrl(b_dir, b_speed);
}

// 双路有符号速度控制(差分驱动标准接口)
// left≥0→A路正转, left<0→A路反转; right≥0→B路正转, right<0→B路反转
void TB6612_SetSpeed(int left, int right)
{
    TB6612_ACtrl((left  < 0) ? 1 : 0, (uint32_t)(left  < 0 ? -left  : left));
    TB6612_BCtrl((right < 0) ? 1 : 0, (uint32_t)(right < 0 ? -right : right));
}
