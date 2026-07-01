// @file    encoder.c
// @brief   正交编码器驱动(TIM2/TIM4编码器模式)
// @note    左编码器(电机A): TIM2_CH1(PA0) + TIM2_CH2(PA1)
//          右编码器(电机B): TIM4_CH1(PD12) + TIM4_CH2(PD13)
//          自动读取编码器计数值, 用于速度闭环控制

#include "encoder.h"

// 左编码器计数变量
volatile int16_t Encoder_Left  = 0;
// 右编码器计数变量
volatile int16_t Encoder_Right = 0;

// 初始化TIM2和TIM4为正交编码器模式 (TI12双通道4倍频计数)
void Encoder_Init(void)
{
    GPIO_InitTypeDef        gpio;
    TIM_TimeBaseInitTypeDef tim_base;
    TIM_ICInitTypeDef       tim_ic;

    // ===== 左编码器: TIM2, PA0(CH1) + PA1(CH2) =====
    // 使能GPIOA和TIM2时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // 配置PA0/PA1为复用功能, 上拉输入
    gpio.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &gpio);

    // 配置PA0/PA1复用为TIM2
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);

    // TIM2时基配置: 不分频, 周期=ENCODER_PERIOD, 向上计数
    TIM_TimeBaseStructInit(&tim_base);
    tim_base.TIM_Prescaler     = 0;
    tim_base.TIM_Period        = ENCODER_PERIOD;
    tim_base.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_base.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &tim_base);

    // 输入捕获配置: 通道1, 带滤波
    TIM_ICStructInit(&tim_ic);
    tim_ic.TIM_Channel  = TIM_Channel_1;
    tim_ic.TIM_ICFilter = ENCODER_IC_FILTER;
    TIM_ICInit(TIM2, &tim_ic);

    // 输入捕获配置: 通道2, 带滤波
    tim_ic.TIM_Channel  = TIM_Channel_2;
    TIM_ICInit(TIM2, &tim_ic);

    // 配置编码器接口模式: TI12双通道, 上升沿计数(4倍频)
    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising,
                               TIM_ICPolarity_Rising);
    // 计数器清零
    TIM_SetCounter(TIM2, 0);
    // 使能TIM2
    TIM_Cmd(TIM2, ENABLE);

    // ===== 右编码器: TIM4, PD12(CH1) + PD13(CH2) =====
    // 使能GPIOD和TIM4时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    // 配置PD12/PD13为复用功能, 上拉输入
    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_Init(GPIOD, &gpio);

    // 配置PD12/PD13复用为TIM4
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);

    // TIM4时基配置: 周期=ENCODER_PERIOD
    TIM_TimeBaseStructInit(&tim_base);
    tim_base.TIM_Period = ENCODER_PERIOD;
    TIM_TimeBaseInit(TIM4, &tim_base);

    // 输入捕获配置: 通道1, 带滤波
    TIM_ICStructInit(&tim_ic);
    tim_ic.TIM_Channel  = TIM_Channel_1;
    tim_ic.TIM_ICFilter = ENCODER_IC_FILTER;
    TIM_ICInit(TIM4, &tim_ic);

    // 输入捕获配置: 通道2, 带滤波
    tim_ic.TIM_Channel  = TIM_Channel_2;
    TIM_ICInit(TIM4, &tim_ic);

    // 配置编码器接口模式: TI12双通道, 上升沿计数(4倍频)
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising,
                               TIM_ICPolarity_Rising);

    // 计数器清零
    TIM_SetCounter(TIM4, 0);
    // 使能TIM4
    TIM_Cmd(TIM4, ENABLE);
}

// 读取双路编码器增量脉冲并清零计数器
void Encoder_ReadAll(void)
{
    // 读取左编码器计数值
    Encoder_Left  = (int16_t)TIM_GetCounter(TIM2);
    // 读取右编码器计数值
    Encoder_Right = (int16_t)TIM_GetCounter(TIM4);
    // 清零左编码器计数器
    TIM_SetCounter(TIM2, 0);
    // 清零右编码器计数器
    TIM_SetCounter(TIM4, 0);
}
