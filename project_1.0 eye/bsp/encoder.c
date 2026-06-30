// @file    encoder.c
// @brief   正交编码器驱动(TIM2/TIM4编码器模式)
// @note    左编码器(电机A): TIM2_CH1(PA0) + TIM2_CH2(PA1)
//          右编码器(电机B): TIM4_CH1(PD12) + TIM4_CH2(PD13)
//          自动读取编码器计数值, 用于速度闭环控制

#include "encoder.h"

volatile int16_t Encoder_Left  = 0;
volatile int16_t Encoder_Right = 0;

// 初始化TIM2和TIM4为正交编码器模式 (TI12双通道4倍频计数)
void Encoder_Init(void)
{
    GPIO_InitTypeDef        gpio;
    TIM_TimeBaseInitTypeDef tim_base;
    TIM_ICInitTypeDef       tim_ic;

    // ===== 左编码器: TIM2, PA0(CH1) + PA1(CH2) =====
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    gpio.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &gpio);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);

    TIM_TimeBaseStructInit(&tim_base);
    tim_base.TIM_Prescaler     = 0;
    tim_base.TIM_Period        = ENCODER_PERIOD;
    tim_base.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_base.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &tim_base);

    TIM_ICStructInit(&tim_ic);
    tim_ic.TIM_Channel  = TIM_Channel_1;
    tim_ic.TIM_ICFilter = ENCODER_IC_FILTER;
    TIM_ICInit(TIM2, &tim_ic);

    tim_ic.TIM_Channel  = TIM_Channel_2;
    TIM_ICInit(TIM2, &tim_ic);

    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising,
                               TIM_ICPolarity_Rising);
    TIM_SetCounter(TIM2, 0);
    TIM_Cmd(TIM2, ENABLE);

    // ===== 右编码器: TIM4, PD12(CH1) + PD13(CH2) =====
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_Init(GPIOD, &gpio);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);

    TIM_TimeBaseStructInit(&tim_base);
    tim_base.TIM_Period = ENCODER_PERIOD;
    TIM_TimeBaseInit(TIM4, &tim_base);

    TIM_ICStructInit(&tim_ic);
    tim_ic.TIM_Channel  = TIM_Channel_1;
    tim_ic.TIM_ICFilter = ENCODER_IC_FILTER;
    TIM_ICInit(TIM4, &tim_ic);

    tim_ic.TIM_Channel  = TIM_Channel_2;
    TIM_ICInit(TIM4, &tim_ic);

    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising,
                               TIM_ICPolarity_Rising);

    TIM_SetCounter(TIM4, 0);
    TIM_Cmd(TIM4, ENABLE);
}

// 读取双路编码器增量脉冲并清零计数器
void Encoder_ReadAll(void)
{
    Encoder_Left  = (int16_t)TIM_GetCounter(TIM2);
    Encoder_Right = (int16_t)TIM_GetCounter(TIM4);
    TIM_SetCounter(TIM2, 0);
    TIM_SetCounter(TIM4, 0);
}