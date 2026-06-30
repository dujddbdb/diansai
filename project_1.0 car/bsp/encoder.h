#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "stm32f4xx.h"

#define ENCODER_IC_FILTER   0xF   // 编码器输入滤波等级(0x0无滤波~0xF最强)
#define ENCODER_PERIOD      65535 // 定时器自动重载值

extern volatile int16_t Encoder_Left;  // 左轮编码器增量脉冲(有符号)
extern volatile int16_t Encoder_Right; // 右轮编码器增量脉冲(有符号)

// 编码器初始化(TIM2+TIM4编码器模式,4倍频)
void Encoder_Init(void);
// 读取编码器值存入全局变量后清零计数器
void Encoder_ReadAll(void);

#endif
