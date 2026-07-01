#ifndef __UART_K230_H__
#define __UART_K230_H__

#include "stm32f4xx.h"

// K230通信UART默认波特率
#define K230_BAUD           115200U
// 弯道状态数据帧头（用于标识弯道状态数据帧的起始字节）
#define K230_CORNER_HEADER  0xA5U

// K230通信UART初始化
// 功能：配置与K230芯片通信的UART串口，设置波特率、数据格式等参数
// 参数：baud - 串口通信波特率（如115200）
// 返回值：无
void K230_UART_Init(uint32_t baud);

// 发送弯道状态（单字节）
// 功能：直接通过UART向K230发送单字节的弯道状态数据
// 参数：state - 弯道状态值（具体含义由通信协议定义）
// 返回值：无
void K230_SendCornerState(uint8_t state);

// 将弯道状态加入发送队列
// 功能：将弯道状态数据放入发送缓冲区队列，由轮询函数异步发送
// 参数：state - 弯道状态值（具体含义由通信协议定义）
// 返回值：无
void K230_QueueCornerState(uint8_t state);

// 轮询发送队列中的弯道状态数据
// 功能：检查发送队列，若有待发送数据则通过UART发送出去
// 参数：无
// 返回值：无
// 注意：需在主循环中周期调用该函数以保证数据及时发送
void K230_PollCornerStateTx(void);

#endif
