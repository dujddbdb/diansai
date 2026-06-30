// TB6612双路直流电机驱动: TIM1_CH1(PE9)=PWMA, TIM1_CH2(PE11)=PWMB
// AIN1=PE10, AIN2=PE8, BIN1=PE12, BIN2=PE15, PWM频率=168MHz/pre/per

#include "tb6612.h"

// 方向控制GPIO初始化: PE8/10/12/15推挽输出, 初始全部拉低刹车
static void AIN_BIN_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    gpio.GPIO_Mode  = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    gpio.GPIO_Pin = GPIO_Pin_10; GPIO_Init(GPIOE, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_8;  GPIO_Init(GPIOE, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_12; GPIO_Init(GPIOE, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_15; GPIO_Init(GPIOE, &gpio);

    GPIO_ResetBits(GPIOE, GPIO_Pin_10 | GPIO_Pin_8 | GPIO_Pin_12 | GPIO_Pin_15);
}

// 初始化TB6612: PWM(TIM1)+方向GPIO, pre=预分频, per=周期
void TB6612_Init(uint16_t pre, uint16_t per)
{
    GPIO_InitTypeDef        gpio;
    TIM_TimeBaseInitTypeDef tim_base;
    TIM_OCInitTypeDef       tim_oc;

    AIN_BIN_GPIO_Init();

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;

    gpio.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOE, &gpio);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);

    gpio.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOE, &gpio);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);

    TIM_TimeBaseStructInit(&tim_base);
    tim_base.TIM_Prescaler         = pre - 1;
    tim_base.TIM_Period            = per - 1;
    tim_base.TIM_ClockDivision     = TIM_CKD_DIV1;
    tim_base.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &tim_base);

    TIM_OCStructInit(&tim_oc);
    tim_oc.TIM_OCMode      = TIM_OCMode_PWM1;
    tim_oc.TIM_OutputState = TIM_OutputState_Enable;
    tim_oc.TIM_Pulse       = 0;
    tim_oc.TIM_OCPolarity  = TIM_OCPolarity_High;

    TIM_OC1Init(TIM1, &tim_oc);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2Init(TIM1, &tim_oc);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

// A路电机控制: dir=0正转/1反转, speed=PWM比较值
void TB6612_ACtrl(uint8_t dir, uint32_t speed)
{
    if (dir) {
        GPIO_ResetBits(GPIOE, GPIO_Pin_10);
        GPIO_SetBits(GPIOE, GPIO_Pin_8);
    } else {
        GPIO_SetBits(GPIOE, GPIO_Pin_10);
        GPIO_ResetBits(GPIOE, GPIO_Pin_8);
    }
    TIM_SetCompare1(TIM1, speed);
}

// B路电机控制: dir=0正转/1反转, speed=PWM比较值
void TB6612_BCtrl(uint8_t dir, uint32_t speed)
{
    if (dir) {
        GPIO_ResetBits(GPIOE, GPIO_Pin_12);
        GPIO_SetBits(GPIOE, GPIO_Pin_15);
    } else {
        GPIO_SetBits(GPIOE, GPIO_Pin_12);
        GPIO_ResetBits(GPIOE, GPIO_Pin_15);
    }
    TIM_SetCompare2(TIM1, speed);
}

// 双路电机同时控制
void TB6612_CarCtrl(uint8_t a_dir, uint32_t a_speed,
                    uint8_t b_dir, uint32_t b_speed)
{
    TB6612_ACtrl(a_dir, a_speed);
    TB6612_BCtrl(b_dir, b_speed);
}

// 双路有符号速度控制: left≥0前进/<0后退, right≥0前进/<0后退
// B路硬件接线取反修正
void TB6612_SetSpeed(int left, int right)
{
    TB6612_ACtrl((left  < 0) ? 1 : 0, (uint32_t)(left  < 0 ? -left  : left));
    TB6612_BCtrl((right > 0) ? 1 : 0, (uint32_t)(right > 0 ? right : -right));
}