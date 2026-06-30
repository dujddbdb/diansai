
// @file    encoder.h
// @brief   双路正交编码器驱动 (TIM2+TIM4编码器模式)
// @note    左编码器: TIM2_CH1(PA0)+CH2(PA1) — 对应A路电机/TB6612左轮
//          右编码器: TIM4_CH1(PD12)+CH2(PD13) — 对应B路电机/TB6612右轮
//          编码器模式TI12, 4倍频计数, 16位计数器(0~65535)
//          每次读取后清零, 得到增量脉冲(带方向符号)

#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "stm32f4xx.h"

/**************************** 用户可配置区域 ***************************/
// 编码器输入捕获滤波值: 0xF为最强15级滤波, 抗干扰能力强
#define ENCODER_IC_FILTER   0xF

// 定时器16位自动重载值(最大值)
#define ENCODER_PERIOD      65535

// 左轮编码器累计脉冲(有符号, 正=正转, 负=反转)
extern volatile int16_t Encoder_Left;
// 右轮编码器累计脉冲(有符号, 正=正转, 负=反转)
extern volatile int16_t Encoder_Right;

// 初始化TIM2和TIM4为编码器模式, 配置GPIO、时钟、输入捕获
void Encoder_Init(void);
// 读取双路编码器增量脉冲并清零计数器, 结果存入Encoder_Left/Right
void Encoder_ReadAll(void);

#endif
