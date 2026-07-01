#ifndef __STEPPER_H__
#define __STEPPER_H__

#include "stm32f4xx.h"

// 步进电机驱动器默认波特率
// 需与ZDT-652A驱动器的拨码开关设置保持一致
#define STEPPER_BAUD  115200

// ZDT驱动器命令码：使能/失能命令
#define ZDT_CMD_ENABLE   0xF3
// ZDT驱动器命令码：速度模式命令
#define ZDT_CMD_SPEED    0xF6
// ZDT驱动器命令码：位置模式命令
#define ZDT_CMD_POSITION 0xFD
// ZDT驱动器命令码：停止命令
#define ZDT_CMD_STOP     0xFE
// ZDT驱动器命令码：同步启动命令
#define ZDT_CMD_SYNC     0xFF
// ZDT驱动器通信协议固定校验字节
#define ZDT_CHECKSUM     0x6B

// XOY面对应的USART6中断抢占优先级
#define STEPPER_XOY_IRQ_PRIO  1
// YOZ面对应的USART2中断抢占优先级
#define STEPPER_YOZ_IRQ_PRIO  1

// XOY面步进电机初始化
// 功能：配置USART6（PC6=TX，PC7=RX）用于与XOY面步进电机驱动器通信
// 参数：baud - 串口通信波特率（如115200）
// 返回值：无
void StepperXOY_Init(uint32_t baud);

// XOY面步进电机使能/失能
// 功能：向指定地址的驱动器发送使能或失能命令
// 参数：addr - 驱动器地址（0~255）
//       state - 使能状态（0=失能，1=使能）
//       sync - 同步模式（0=立即执行，1=等待同步启动命令）
// 返回值：无
void StepperXOY_Enable(uint8_t addr, uint8_t state, uint8_t sync);

// XOY面步进电机速度模式
// 功能：配置电机以速度模式运行，设置方向、转速和加速度
// 参数：addr - 驱动器地址（0~255）
//       dir - 旋转方向（0=顺时针CW，1=逆时针CCW）
//       rpm - 转速（单位：转/分钟）
//       acc - 加速度（步进电机加速曲线参数）
//       sync - 同步模式（0=立即执行，1=等待同步启动命令）
// 返回值：无
void StepperXOY_Speed(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc, uint8_t sync);

// XOY面步进电机位置模式
// 功能：配置电机以位置模式运行，按指定参数运动到目标位置
// 参数：addr - 驱动器地址（0~255）
//       dir - 旋转方向（0=顺时针CW，1=逆时针CCW）
//       rpm - 转速（单位：转/分钟）
//       acc - 加速度（步进电机加速曲线参数）
//       pulses - 脉冲数（目标位置对应的步进脉冲数）
//       rel - 定位模式（0=绝对定位，1=相对定位）
//       sync - 同步模式（0=立即执行，1=等待同步启动命令）
// 返回值：无
void StepperXOY_Position(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc,
                         uint32_t pulses, uint8_t rel, uint8_t sync);

// XOY面步进电机停止
// 功能：向指定地址的驱动器发送停止命令，电机立即停止运行
// 参数：addr - 驱动器地址（0~255）
//       sync - 同步模式（0=立即执行，1=等待同步启动命令）
// 返回值：无
void StepperXOY_Stop(uint8_t addr, uint8_t sync);

// XOY面同步启动
// 功能：向指定地址的驱动器发送同步启动命令，所有等待中的电机同时启动
// 参数：addr - 驱动器地址（0~255）
// 返回值：无
void StepperXOY_SyncStart(uint8_t addr);

// YOZ面步进电机初始化
// 功能：配置USART2（PD5=TX，PD6=RX）用于与YOZ面步进电机驱动器通信
// 参数：baud - 串口通信波特率（如115200）
// 返回值：无
void StepperYOZ_Init(uint32_t baud);

// YOZ面步进电机使能/失能
// 功能：向指定地址的驱动器发送使能或失能命令
// 参数：addr - 驱动器地址（0~255）
//       state - 使能状态（0=失能，1=使能）
//       sync - 同步模式（0=立即执行，1=等待同步启动命令）
// 返回值：无
void StepperYOZ_Enable(uint8_t addr, uint8_t state, uint8_t sync);

// YOZ面步进电机速度模式
// 功能：配置电机以速度模式运行，设置方向、转速和加速度
// 参数：addr - 驱动器地址（0~255）
//       dir - 旋转方向（0=顺时针CW，1=逆时针CCW）
//       rpm - 转速（单位：转/分钟）
//       acc - 加速度（步进电机加速曲线参数）
//       sync - 同步模式（0=立即执行，1=等待同步启动命令）
// 返回值：无
void StepperYOZ_Speed(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc, uint8_t sync);

// YOZ面步进电机位置模式
// 功能：配置电机以位置模式运行，按指定参数运动到目标位置
// 参数：addr - 驱动器地址（0~255）
//       dir - 旋转方向（0=顺时针CW，1=逆时针CCW）
//       rpm - 转速（单位：转/分钟）
//       acc - 加速度（步进电机加速曲线参数）
//       pulses - 脉冲数（目标位置对应的步进脉冲数）
//       rel - 定位模式（0=绝对定位，1=相对定位）
//       sync - 同步模式（0=立即执行，1=等待同步启动命令）
// 返回值：无
void StepperYOZ_Position(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc,
                         uint32_t pulses, uint8_t rel, uint8_t sync);

// YOZ面步进电机停止
// 功能：向指定地址的驱动器发送停止命令，电机立即停止运行
// 参数：addr - 驱动器地址（0~255）
//       sync - 同步模式（0=立即执行，1=等待同步启动命令）
// 返回值：无
void StepperYOZ_Stop(uint8_t addr, uint8_t sync);

// YOZ面同步启动
// 功能：向指定地址的驱动器发送同步启动命令，所有等待中的电机同时启动
// 参数：addr - 驱动器地址（0~255）
// 返回值：无
void StepperYOZ_SyncStart(uint8_t addr);

#endif
