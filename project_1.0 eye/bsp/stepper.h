#ifndef __STEPPER_H__
#define __STEPPER_H__

#include "stm32f4xx.h"

#define STEPPER_BAUD  115200  // 步进电机串口默认波特率

#define ZDT_CMD_ENABLE   0xF3  // 使能电机命令字
#define ZDT_CMD_SPEED    0xF6  // 速度模式命令字
#define ZDT_CMD_POSITION 0xFD  // 位置模式命令字
#define ZDT_CMD_STOP     0xFE  // 停止命令字
#define ZDT_CMD_SYNC     0xFF  // 同步启动命令字
#define ZDT_CHECKSUM     0x6B  // 协议校验字节固定值

#define STEPPER_XOY_IRQ_PRIO  1  // XOY轴串口中断优先级
#define STEPPER_YOZ_IRQ_PRIO  1  // YOZ轴串口中断优先级

// 初始化XOY面步进电机串口，baud: 波特率
void StepperXOY_Init(uint32_t baud);
// XOY轴电机使能控制，addr: 驱动器地址，state: 0-失能 1-使能，sync: 0-立即执行 1-等待同步
void StepperXOY_Enable(uint8_t addr, uint8_t state, uint8_t sync);
// XOY轴速度模式设置，addr: 驱动器地址，dir: 0-CW顺时针 1-CCW逆时针，rpm: 转速，acc: 加速度，sync: 同步标志
void StepperXOY_Speed(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc, uint8_t sync);
// XOY轴位置命令发送，addr: 驱动器地址，dir: 方向，rpm: 转速，acc: 加速度，pulses: 脉冲数，rel: 0-绝对定位 1-相对定位，sync: 同步标志
void StepperXOY_Position(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc,
                         uint32_t pulses, uint8_t rel, uint8_t sync);
// XOY轴电机停止，addr: 驱动器地址，sync: 同步标志
void StepperXOY_Stop(uint8_t addr, uint8_t sync);
// XOY轴同步启动所有等待命令，addr: 驱动器地址
void StepperXOY_SyncStart(uint8_t addr);

// 初始化YOZ面步进电机串口，baud: 波特率
void StepperYOZ_Init(uint32_t baud);
// YOZ轴电机使能控制，addr: 驱动器地址，state: 0-失能 1-使能，sync: 0-立即执行 1-等待同步
void StepperYOZ_Enable(uint8_t addr, uint8_t state, uint8_t sync);
// YOZ轴速度模式设置，addr: 驱动器地址，dir: 0-CW顺时针 1-CCW逆时针，rpm: 转速，acc: 加速度，sync: 同步标志
void StepperYOZ_Speed(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc, uint8_t sync);
// YOZ轴位置命令发送，addr: 驱动器地址，dir: 方向，rpm: 转速，acc: 加速度，pulses: 脉冲数，rel: 0-绝对定位 1-相对定位，sync: 同步标志
void StepperYOZ_Position(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc,
                         uint32_t pulses, uint8_t rel, uint8_t sync);
// YOZ轴电机停止，addr: 驱动器地址，sync: 同步标志
void StepperYOZ_Stop(uint8_t addr, uint8_t sync);
// YOZ轴同步启动所有等待命令，addr: 驱动器地址
void StepperYOZ_SyncStart(uint8_t addr);

#endif

