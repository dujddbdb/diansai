/**
 * @file    tb6612.h
 * @brief   TB6612FNG双路直流有刷电机驱动接口
 * @note    硬件: TIM1高级定时器PWM + GPIO方向控制
 *          A路(左轮): PE10=AIN1, PE8=AIN2, PE9=PWMA(TIM1_CH1)
 *          B路(右轮): PE12=BIN1, PE15=BIN2, PE11=PWMB(TIM1_CH2)
 *          TB6612内部自带死区保护(<1us), 无需软件插入死区
 *
 *          PWM频率 = 168MHz / PRE / PER
 *          示例: PRE=84, PER=1000 → f_PWM = 168MHz/84/1000 = 2kHz
 *          高级定时器必须使能MOE(主输出), 否则PWM引脚无输出
 */

#ifndef __TB6612_H__
#define __TB6612_H__

#include "stm32f4xx.h"

/**************************** 用户可配置区域 ***************************/
// PWM频率参数: f_PWM = 168MHz / TB6612_PRE / TB6612_PER
// 2kHz(84,1000)默认, 10kHz(84,200)安静, 20kHz(84,100)静音但扭矩降低
// TIM1预分频器值
#define TB6612_PRE   84
// TIM1自动重装载值/PWM周期
#define TB6612_PER   1000
// 速度参数上限(等于TB6612_PER, 100%占空比)
#define TB6612_MAX_SPEED  TB6612_PER

/**************************** 函数声明 ********************************/
// 初始化TB6612: 配置GPIO方向引脚和TIM1 PWM输出, pre: 预分频器, per: PWM周期
void TB6612_Init(uint16_t pre, uint16_t per);

// A路电机(左轮)控制, dir: 0=正转 1=反转, speed: PWM比较值 0~per
void TB6612_ACtrl(uint8_t dir, uint32_t speed);

// B路电机(右轮)控制, dir: 0=正转 1=反转, speed: PWM比较值 0~per
void TB6612_BCtrl(uint8_t dir, uint32_t speed);

// 双路电机同时控制, a_dir/a_speed: A路方向/速度, b_dir/b_speed: B路方向/速度
void TB6612_CarCtrl(uint8_t a_dir, uint32_t a_speed,
                     uint8_t b_dir, uint32_t b_speed);

// 双路有符号速度控制(差分驱动), left: 左轮速度(≥0前进/<0后退), right: 右轮速度
void TB6612_SetSpeed(int left, int right);

#endif
