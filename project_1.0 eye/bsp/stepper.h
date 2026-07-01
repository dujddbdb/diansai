
#ifndef __STEPPER_H__
#define __STEPPER_H__

#include "stm32f4xx.h"

// 步进电机驱动器串口通信默认波特率
// 用于与步进电机驱动器进行UART通信的波特率配置
#define STEPPER_BAUD  115200

// 步进电机驱动器协议命令字定义
// 这些命令字用于串口通信协议，控制步进电机的各种运行模式

#define ZDT_CMD_ENABLE   0xF3  // 使能电机命令字，用于使能或失能步进电机驱动器输出
#define ZDT_CMD_SPEED    0xF6  // 速度模式命令字，用于设置电机以恒定速度运行
#define ZDT_CMD_POSITION 0xFD  // 位置模式命令字，用于设置电机运行到指定位置
#define ZDT_CMD_STOP     0xFE  // 停止命令字，用于立即停止电机运行
#define ZDT_CMD_SYNC     0xFF  // 同步启动命令字，用于同时启动多个等待中的电机
#define ZDT_CHECKSUM     0x6B  // 协议校验字节固定值，用于串口通信协议的校验

// XOY轴串口中断优先级
// 数值越小优先级越高，配置为1表示较高优先级
#define STEPPER_XOY_IRQ_PRIO  1

// YOZ轴串口中断优先级
// 数值越小优先级越高，配置为1表示较高优先级
#define STEPPER_YOZ_IRQ_PRIO  1

// ==================== XOY面步进电机控制函数 ====================

// XOY面步进电机串口初始化
// 功能：初始化XOY平面步进电机驱动器对应的串口外设
//       配置波特率、数据位、停止位、校验位等参数
//       配置对应GPIO引脚和NVIC中断
// 参数：
//   baud - 串口波特率，一般使用STEPPER_BAUD默认值115200
// 返回值：无
void StepperXOY_Init(uint32_t baud);

// XOY轴电机使能控制
// 功能：通过串口发送使能命令，控制步进电机驱动器的输出使能状态
//       使能后电机保持力矩，失能后电机自由转动
// 参数：
//   addr  - 驱动器地址，用于多台驱动器总线组网时寻址（0~255）
//   state - 使能状态，0 = 失能（电机断电），1 = 使能（电机通电）
//   sync  - 同步标志，0 = 立即执行命令，1 = 等待同步信号（SYNC命令）后执行
// 返回值：无
void StepperXOY_Enable(uint8_t addr, uint8_t state, uint8_t sync);

// XOY轴速度模式设置
// 功能：设置步进电机以恒定速度模式运行
//       电机将以指定的转速和加速度持续运行，直到收到停止命令
// 参数：
//   addr - 驱动器地址
//   dir  - 旋转方向，0 = CW顺时针，1 = CCW逆时针
//   rpm  - 转速（转/分钟）
//   acc  - 加速度档位，数值越大加速度越快
//   sync - 同步标志，0 = 立即执行，1 = 等待同步
// 返回值：无
void StepperXOY_Speed(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc, uint8_t sync);

// XOY轴位置模式命令发送
// 功能：设置步进电机以位置模式运行，电机将运行到指定的脉冲数位置
//       支持绝对定位和相对定位两种模式
// 参数：
//   addr   - 驱动器地址
//   dir    - 旋转方向，0 = CW顺时针，1 = CCW逆时针
//   rpm    - 运行转速（转/分钟）
//   acc    - 加速度档位
//   pulses - 目标脉冲数，即电机需要运行的步数
//   rel    - 定位模式，0 = 绝对定位（相对于零点），1 = 相对定位（相对于当前位置）
//   sync   - 同步标志，0 = 立即执行，1 = 等待同步
// 返回值：无
void StepperXOY_Position(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc,
                         uint32_t pulses, uint8_t rel, uint8_t sync);

// XOY轴电机停止
// 功能：发送停止命令，立即停止正在运行的步进电机
//       电机会以设定的减速度减速停止
// 参数：
//   addr - 驱动器地址
//   sync - 同步标志，0 = 立即执行，1 = 等待同步
// 返回值：无
void StepperXOY_Stop(uint8_t addr, uint8_t sync);

// XOY轴同步启动所有等待命令
// 功能：发送同步启动命令（SYNC命令），触发所有设置了sync=1的等待命令同时执行
//       用于实现多轴同步运动
// 参数：
//   addr - 驱动器地址（可使用广播地址同时触发所有驱动器）
// 返回值：无
void StepperXOY_SyncStart(uint8_t addr);

// ==================== YOZ面步进电机控制函数 ====================

// YOZ面步进电机串口初始化
// 功能：初始化YOZ平面步进电机驱动器对应的串口外设
//       配置波特率、数据位、停止位、校验位等参数
//       配置对应GPIO引脚和NVIC中断
// 参数：
//   baud - 串口波特率，一般使用STEPPER_BAUD默认值115200
// 返回值：无
void StepperYOZ_Init(uint32_t baud);

// YOZ轴电机使能控制
// 功能：通过串口发送使能命令，控制步进电机驱动器的输出使能状态
//       使能后电机保持力矩，失能后电机自由转动
// 参数：
//   addr  - 驱动器地址，用于多台驱动器总线组网时寻址（0~255）
//   state - 使能状态，0 = 失能（电机断电），1 = 使能（电机通电）
//   sync  - 同步标志，0 = 立即执行命令，1 = 等待同步信号（SYNC命令）后执行
// 返回值：无
void StepperYOZ_Enable(uint8_t addr, uint8_t state, uint8_t sync);

// YOZ轴速度模式设置
// 功能：设置步进电机以恒定速度模式运行
//       电机将以指定的转速和加速度持续运行，直到收到停止命令
// 参数：
//   addr - 驱动器地址
//   dir  - 旋转方向，0 = CW顺时针，1 = CCW逆时针
//   rpm  - 转速（转/分钟）
//   acc  - 加速度档位，数值越大加速度越快
//   sync - 同步标志，0 = 立即执行，1 = 等待同步
// 返回值：无
void StepperYOZ_Speed(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc, uint8_t sync);

// YOZ轴位置模式命令发送
// 功能：设置步进电机以位置模式运行，电机将运行到指定的脉冲数位置
//       支持绝对定位和相对定位两种模式
// 参数：
//   addr   - 驱动器地址
//   dir    - 旋转方向，0 = CW顺时针，1 = CCW逆时针
//   rpm    - 运行转速（转/分钟）
//   acc    - 加速度档位
//   pulses - 目标脉冲数，即电机需要运行的步数
//   rel    - 定位模式，0 = 绝对定位（相对于零点），1 = 相对定位（相对于当前位置）
//   sync   - 同步标志，0 = 立即执行，1 = 等待同步
// 返回值：无
void StepperYOZ_Position(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc,
                         uint32_t pulses, uint8_t rel, uint8_t sync);

// YOZ轴电机停止
// 功能：发送停止命令，立即停止正在运行的步进电机
//       电机会以设定的减速度减速停止
// 参数：
//   addr - 驱动器地址
//   sync - 同步标志，0 = 立即执行，1 = 等待同步
// 返回值：无
void StepperYOZ_Stop(uint8_t addr, uint8_t sync);

// YOZ轴同步启动所有等待命令
// 功能：发送同步启动命令（SYNC命令），触发所有设置了sync=1的等待命令同时执行
//       用于实现多轴同步运动
// 参数：
//   addr - 驱动器地址（可使用广播地址同时触发所有驱动器）
// 返回值：无
void StepperYOZ_SyncStart(uint8_t addr);

#endif
