#ifndef __STEPPER_H__
#define __STEPPER_H__

#include "stm32f4xx.h"

#define STEPPER_BAUD  115200  // 默认波特率，需与ZDT-652A驱动器拨码一致

#define ZDT_CMD_ENABLE   0xF3   // 使能/失能命令
#define ZDT_CMD_SPEED    0xF6   // 速度模式命令
#define ZDT_CMD_POSITION 0xFD   // 位置模式命令
#define ZDT_CMD_STOP     0xFE   // 停止命令
#define ZDT_CMD_SYNC     0xFF   // 同步启动命令
#define ZDT_CHECKSUM     0x6B   // 固定校验字节

#define STEPPER_XOY_IRQ_PRIO  1   // USART6(XOY)中断抢占优先级
#define STEPPER_YOZ_IRQ_PRIO  1   // USART2(YOZ)中断抢占优先级

// XOY面(USART6: PC6/PC7)初始化，baud为波特率
void StepperXOY_Init(uint32_t baud);
// XOY面使能/失能: addr-驱动器地址 state-0失能/1使能 sync-0立即/1等待同步
void StepperXOY_Enable(uint8_t addr, uint8_t state, uint8_t sync);
// XOY面速度模式: dir-0CW/1CCW rpm-转速 acc-加速度
void StepperXOY_Speed(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc, uint8_t sync);
// XOY面位置模式: pulses-脉冲数 rel-0绝对/1相对定位
void StepperXOY_Position(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc,
                         uint32_t pulses, uint8_t rel, uint8_t sync);
// XOY面停止电机
void StepperXOY_Stop(uint8_t addr, uint8_t sync);
// XOY面同步启动所有等待中的电机
void StepperXOY_SyncStart(uint8_t addr);

// YOZ面(USART2: PD5/PD6)初始化，baud为波特率
void StepperYOZ_Init(uint32_t baud);
// YOZ面使能/失能
void StepperYOZ_Enable(uint8_t addr, uint8_t state, uint8_t sync);
// YOZ面速度模式
void StepperYOZ_Speed(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc, uint8_t sync);
// YOZ面位置模式
void StepperYOZ_Position(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc,
                         uint32_t pulses, uint8_t rel, uint8_t sync);
// YOZ面停止电机
void StepperYOZ_Stop(uint8_t addr, uint8_t sync);
// YOZ面同步启动
void StepperYOZ_SyncStart(uint8_t addr);

#endif
