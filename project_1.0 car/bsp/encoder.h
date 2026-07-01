#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "stm32f4xx.h"

// 编码器输入捕获滤波器等级
// 取值范围 0x0 ~ 0xF，数值越大滤波越强，抗干扰能力越好但响应延迟增加
#define ENCODER_IC_FILTER   0xF   // 编码器输入滤波等级（0x0无滤波 ~ 0xF最强滤波）

// 定时器自动重载值
// 配置编码器计数周期，决定最大计数值，溢出后自动回绕
#define ENCODER_PERIOD      65535 // 定时器自动重载值（16位计数器最大值，0~65535）

// 左轮编码器增量脉冲计数值（有符号）
// 每次调用 Encoder_ReadAll() 后更新，单位：脉冲数
// 正值表示正转，负值表示反转
extern volatile int16_t Encoder_Left;  // 左轮编码器增量脉冲数（有符号，正=正转，负=反转）

// 右轮编码器增量脉冲计数值（有符号）
// 每次调用 Encoder_ReadAll() 后更新，单位：脉冲数
// 正值表示正转，负值表示反转
extern volatile int16_t Encoder_Right; // 右轮编码器增量脉冲数（有符号，正=正转，负=反转）

// 编码器硬件初始化
// 功能：配置TIM2和TIM4为编码器接口模式（4倍频），初始化GPIO引脚
//       配置完成后定时器开始自动计数，无需手动启动
// 参数：无
// 返回值：无
void Encoder_Init(void);

// 读取并清零左右轮编码器计数值
// 功能：读取当前定时器计数器值，计算增量脉冲数存入全局变量Encoder_Left/Encoder_Right，
//       然后清零计数器，为下一次采样做准备
//       建议以固定周期调用（如10ms/20ms）以计算转速
// 参数：无
// 返回值：无
void Encoder_ReadAll(void);

#endif
