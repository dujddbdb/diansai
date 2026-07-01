// 正交编码器驱动: TIM2_CH1(PA0)+CH2(PA1)=左, TIM4_CH1(PD12)+CH2(PD13)=右, 4倍频

#include "encoder.h"

// 左编码器脉冲计数
volatile int16_t Encoder_Left  = 0;
// 右编码器脉冲计数
volatile int16_t Encoder_Right = 0;

// 初始化双路编码器: TIM2(左)+TIM4(右), 编码器模式TI12, 4倍频
void Encoder_Init(void)
{
    GPIO_InitTypeDef        gpio;
    TIM_TimeBaseInitTypeDef tim_base;
    TIM_ICInitTypeDef       tim_ic;

    // 左编码器: TIM2, PA0(CH1)+PA1(CH2)
    // 使能GPIOA端口时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    // 使能TIM2时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // 配置PA0和PA1为复用功能引脚
    gpio.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;  // 选择引脚0和1
    gpio.GPIO_Mode  = GPIO_Mode_AF;              // 复用功能模式
    gpio.GPIO_Speed = GPIO_Speed_50MHz;          // 速度50MHz
    gpio.GPIO_OType = GPIO_OType_PP;             // 推挽输出
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;              // 上拉
    GPIO_Init(GPIOA, &gpio);                      // 初始化GPIOA
    // PA0复用为TIM2通道1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
    // PA1复用为TIM2通道2
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);

    // 配置TIM2时基
    TIM_TimeBaseStructInit(&tim_base);
    tim_base.TIM_Prescaler     = 0;                  // 预分频器0
    tim_base.TIM_Period        = ENCODER_PERIOD;    // 自动重装载值
    tim_base.TIM_ClockDivision = TIM_CKD_DIV1;       // 时钟不分频
    tim_base.TIM_CounterMode   = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseInit(TIM2, &tim_base);               // 初始化TIM2时基

    // 配置TIM2输入捕获通道
    TIM_ICStructInit(&tim_ic);
    tim_ic.TIM_Channel  = TIM_Channel_1;             // 通道1
    tim_ic.TIM_ICFilter = ENCODER_IC_FILTER;        // 输入滤波
    TIM_ICInit(TIM2, &tim_ic);                        // 初始化通道1
    tim_ic.TIM_Channel  = TIM_Channel_2;             // 通道2
    TIM_ICInit(TIM2, &tim_ic);                        // 初始化通道2

    // 配置编码器接口模式: TI1TI2模式, 上升沿触发, 4倍频
    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising,
                               TIM_ICPolarity_Rising);
    TIM_SetCounter(TIM2, 0);                          // 计数器清零
    TIM_Cmd(TIM2, ENABLE);                            // 使能TIM2

    // 右编码器: TIM4, PD12(CH1)+PD13(CH2)
    // 使能GPIOD端口时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    // 使能TIM4时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    // 配置PD12和PD13为复用功能引脚
    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;  // 选择引脚12和13
    GPIO_Init(GPIOD, &gpio);                      // 初始化GPIOD
    // PD12复用为TIM4通道1
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
    // PD13复用为TIM4通道2
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);

    // 配置TIM4时基
    TIM_TimeBaseStructInit(&tim_base);
    tim_base.TIM_Period = ENCODER_PERIOD;     // 自动重装载值
    TIM_TimeBaseInit(TIM4, &tim_base);          // 初始化TIM4时基

    // 配置TIM4输入捕获通道
    TIM_ICStructInit(&tim_ic);
    tim_ic.TIM_Channel  = TIM_Channel_1;         // 通道1
    tim_ic.TIM_ICFilter = ENCODER_IC_FILTER;    // 输入滤波
    TIM_ICInit(TIM4, &tim_ic);                    // 初始化通道1
    tim_ic.TIM_Channel  = TIM_Channel_2;         // 通道2
    TIM_ICInit(TIM4, &tim_ic);                    // 初始化通道2

    // 配置编码器接口模式: TI1TI2模式, 上升沿触发, 4倍频
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising,
                               TIM_ICPolarity_Rising);
    TIM_SetCounter(TIM4, 0);                      // 计数器清零
    TIM_Cmd(TIM4, ENABLE);                        // 使能TIM4
}

// 读取编码器增量脉冲并清零计数器
void Encoder_ReadAll(void)
{
    // 读取左编码器当前计数值
    Encoder_Left  = (int16_t)TIM_GetCounter(TIM2);
    // 读取右编码器当前计数值
    Encoder_Right = (int16_t)TIM_GetCounter(TIM4);

    // 清零左编码器计数器, 准备下一次计数
    TIM_SetCounter(TIM2, 0);
    // 清零右编码器计数器, 准备下一次计数
    TIM_SetCounter(TIM4, 0);
}
