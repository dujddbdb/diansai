// TB6612双路直流电机驱动: TIM1_CH1(PE9)=PWMA, TIM1_CH2(PE11)=PWMB
// AIN1=PE10, AIN2=PE8, BIN1=PE12, BIN2=PE15, PWM频率=168MHz/pre/per

#include "tb6612.h"

// 方向控制GPIO初始化: PE8/10/12/15推挽输出, 初始全部拉低刹车
static void AIN_BIN_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio;

    // 使能GPIOE端口时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    // GPIO配置: 推挽输出, 50MHz, 无上拉下拉
    gpio.GPIO_Mode  = GPIO_Mode_OUT;       // 输出模式
    gpio.GPIO_OType = GPIO_OType_PP;        // 推挽输出
    gpio.GPIO_Speed = GPIO_Speed_50MHz;     // 速度50MHz
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;     // 无上拉下拉

    gpio.GPIO_Pin = GPIO_Pin_10; GPIO_Init(GPIOE, &gpio);  // AIN1=PE10
    gpio.GPIO_Pin = GPIO_Pin_8;  GPIO_Init(GPIOE, &gpio);  // AIN2=PE8
    gpio.GPIO_Pin = GPIO_Pin_12; GPIO_Init(GPIOE, &gpio);  // BIN1=PE12
    gpio.GPIO_Pin = GPIO_Pin_15; GPIO_Init(GPIOE, &gpio);  // BIN2=PE15

    // 所有方向引脚拉低, 电机刹车
    GPIO_ResetBits(GPIOE, GPIO_Pin_10 | GPIO_Pin_8 | GPIO_Pin_12 | GPIO_Pin_15);
}

// 初始化TB6612: PWM(TIM1)+方向GPIO, pre=预分频, per=周期
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
    gpio.GPIO_Mode  = GPIO_Mode_AF;        // 复用功能模式
    gpio.GPIO_OType = GPIO_OType_PP;         // 推挽输出
    gpio.GPIO_Speed = GPIO_Speed_50MHz;      // 速度50MHz
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;          // 上拉

    // 配置PE9为TIM1_CH1 (PWMA)
    gpio.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOE, &gpio);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);

    // 配置PE11为TIM1_CH2 (PWMB)
    gpio.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOE, &gpio);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);

    // 配置TIM1时基
    TIM_TimeBaseStructInit(&tim_base);
    tim_base.TIM_Prescaler         = pre - 1;    // 预分频器
    tim_base.TIM_Period            = per - 1;    // 自动重装载值
    tim_base.TIM_ClockDivision     = TIM_CKD_DIV1;  // 时钟不分频
    tim_base.TIM_CounterMode       = TIM_CounterMode_Up;  // 向上计数
    TIM_TimeBaseInit(TIM1, &tim_base);               // 初始化TIM1时基

    // 配置TIM1输出比较(PWM模式1)
    TIM_OCStructInit(&tim_oc);
    tim_oc.TIM_OCMode      = TIM_OCMode_PWM1;       // PWM模式1
    tim_oc.TIM_OutputState = TIM_OutputState_Enable; // 输出使能
    tim_oc.TIM_Pulse       = 0;                       // 初始占空比0
    tim_oc.TIM_OCPolarity  = TIM_OCPolarity_High;     // 高电平有效

    // 配置通道1 (PWMA)
    TIM_OC1Init(TIM1, &tim_oc);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    // 配置通道2 (PWMB)
    TIM_OC2Init(TIM1, &tim_oc);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

    // 使能TIM1计数器
    TIM_Cmd(TIM1, ENABLE);
    // 使能TIM1 PWM主输出
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

// A路电机控制: dir=0正转/1反转, speed=PWM比较值
void TB6612_ACtrl(uint8_t dir, uint32_t speed)
{
    // 根据方向设置AIN1/AIN2电平
    if (dir) {
        // 反转: AIN1=0, AIN2=1
        GPIO_ResetBits(GPIOE, GPIO_Pin_10);
        GPIO_SetBits(GPIOE, GPIO_Pin_8);
    } else {
        // 正转: AIN1=1, AIN2=0
        GPIO_SetBits(GPIOE, GPIO_Pin_10);
        GPIO_ResetBits(GPIOE, GPIO_Pin_8);
    }
    // 设置PWM占空比
    TIM_SetCompare1(TIM1, speed);
}

// B路电机控制: dir=0正转/1反转, speed=PWM比较值
void TB6612_BCtrl(uint8_t dir, uint32_t speed)
{
    // 根据方向设置BIN1/BIN2电平
    if (dir) {
        // 反转: BIN1=0, BIN2=1
        GPIO_ResetBits(GPIOE, GPIO_Pin_12);
        GPIO_SetBits(GPIOE, GPIO_Pin_15);
    } else {
        // 正转: BIN1=1, BIN2=0
        GPIO_SetBits(GPIOE, GPIO_Pin_12);
        GPIO_ResetBits(GPIOE, GPIO_Pin_15);
    }
    // 设置PWM占空比
    TIM_SetCompare2(TIM1, speed);
}

// 双路电机同时控制
void TB6612_CarCtrl(uint8_t a_dir, uint32_t a_speed,
                    uint8_t b_dir, uint32_t b_speed)
{
    // 控制A路电机
    TB6612_ACtrl(a_dir, a_speed);
    // 控制B路电机
    TB6612_BCtrl(b_dir, b_speed);
}

// 双路有符号速度控制: left≥0前进/<0后退, right≥0前进/<0后退
// B路硬件接线取反修正
void TB6612_SetSpeed(int left, int right)
{
    // 左电机: 负数反转, 正数正转, 速度取绝对值
    TB6612_ACtrl((left  < 0) ? 1 : 0, (uint32_t)(left  < 0 ? -left  : left));
    // 右电机: 正数反转(硬件取反), 负数正转, 速度取绝对值
    TB6612_BCtrl((right > 0) ? 1 : 0, (uint32_t)(right > 0 ? right : -right));
}
