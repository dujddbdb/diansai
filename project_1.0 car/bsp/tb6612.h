#ifndef __TB6612_H__
#define __TB6612_H__

#include "stm32f4xx.h"

// TIM1定时器预分频器值
// PWM频率计算公式：PWM频率 = 系统时钟 / (预分频+1) / (周期+1)
// 默认配置：84MHz / (83+1) / (999+1) = 1kHz（注：此处按pre=84, per=1000计算约为1kHz）
#define TB6612_PRE   84       // TIM1预分频器值（PWM频率配置参数）

// TIM1定时器自动重装载值（PWM周期）
// 决定PWM的分辨率和周期，同时也是最大占空比值
#define TB6612_PER   1000     // TIM1自动重装载值（PWM周期，同时决定PWM分辨率）

// PWM最大值（100%占空比对应的比较值）
// 与TB6612_PER相等，用于速度上限限制，避免写入超范围值
#define TB6612_MAX_SPEED  TB6612_PER  // PWM最大值（对应100%占空比，速度上限）

// TB6612电机驱动芯片初始化
// 功能：初始化GPIO引脚（方向控制引脚、PWM输出引脚），配置TIM1为PWM输出模式
//       初始化完成后电机处于停止状态
// 参数：
//   pre - 定时器预分频器值（影响PWM频率）
//   per - 定时器自动重装载值（PWM周期，同时也是PWM最大值）
// 返回值：无
void TB6612_Init(uint16_t pre, uint16_t per);

// A路电机控制（单路独立控制）
// 功能：控制A路电机的转动方向和转速
// 参数：
//   dir   - 转动方向：0-正转，1-反转
//   speed - PWM比较值（占空比），范围 0 ~ TB6612_PER，值越大转速越快
// 返回值：无
void TB6612_ACtrl(uint8_t dir, uint32_t speed);

// B路电机控制（单路独立控制）
// 功能：控制B路电机的转动方向和转速
// 参数：
//   dir   - 转动方向：0-正转，1-反转
//   speed - PWM比较值（占空比），范围 0 ~ TB6612_PER，值越大转速越快
// 返回值：无
void TB6612_BCtrl(uint8_t dir, uint32_t speed);

// 双路电机同时控制
// 功能：同时控制A路和B路电机的转动方向和转速
//       相比分别调用ACtrl和BCtrl，两路更新更接近同步
// 参数：
//   a_dir   - A路电机转动方向：0-正转，1-反转
//   a_speed - A路电机PWM比较值，范围 0 ~ TB6612_PER
//   b_dir   - B路电机转动方向：0-正转，1-反转
//   b_speed - B路电机PWM比较值，范围 0 ~ TB6612_PER
// 返回值：无
void TB6612_CarCtrl(uint8_t a_dir, uint32_t a_speed,
                     uint8_t b_dir, uint32_t b_speed);

// 双路有符号速度控制
// 功能：使用有符号整数同时控制左右电机速度，正值正转，负值反转，绝对值为PWM值
//       这是更便捷的上层接口，自动处理方向和PWM值
// 参数：
//   left  - 左轮速度（有符号PWM值），范围 -TB6612_MAX_SPEED ~ TB6612_MAX_SPEED
//           >=0 正转，<0 反转，绝对值为PWM占空比
//   right - 右轮速度（有符号PWM值），范围 -TB6612_MAX_SPEED ~ TB6612_MAX_SPEED
//           >=0 正转，<0 反转，绝对值为PWM占空比
// 返回值：无
void TB6612_SetSpeed(int left, int right);

#endif
