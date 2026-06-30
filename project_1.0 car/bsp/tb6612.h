#ifndef __TB6612_H__
#define __TB6612_H__

#include "stm32f4xx.h"

#define TB6612_PRE   84       // TIM1预分频器值(PWM=2kHz时)
#define TB6612_PER   1000     // TIM1自动重装载值
#define TB6612_MAX_SPEED  TB6612_PER  // PWM最大值(100%占空比)

// TB6612初始化: pre-预分频 per-周期
void TB6612_Init(uint16_t pre, uint16_t per);
// A路电机控制: dir-0正转/1反转 speed-PWM比较值(0~per)
void TB6612_ACtrl(uint8_t dir, uint32_t speed);
// B路电机控制: dir-0正转/1反转 speed-PWM比较值(0~per)
void TB6612_BCtrl(uint8_t dir, uint32_t speed);
// 双路电机同时控制
void TB6612_CarCtrl(uint8_t a_dir, uint32_t a_speed,
                     uint8_t b_dir, uint32_t b_speed);
// 双路有符号速度控制: left/right为有符号PWM, >=0正转 <0反转
void TB6612_SetSpeed(int left, int right);

#endif
