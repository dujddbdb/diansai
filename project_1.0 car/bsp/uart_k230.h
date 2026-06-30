#ifndef __UART_K230_H__
#define __UART_K230_H__

#include "stm32f4xx.h"

// UART通信波特率
#define K230_BAUD           115200U
// 弯道状态数据帧头
#define K230_CORNER_HEADER  0xA5U

// 初始化K230通信UART，参数baud为波特率
void K230_UART_Init(uint32_t baud);
// 发送弯道状态（单字节），参数state为状态值
void K230_SendCornerState(uint8_t state);
// 将弯道状态加入发送队列，参数state为状态值
void K230_QueueCornerState(uint8_t state);
// 轮询发送队列中的弯道状态数据
void K230_PollCornerStateTx(void);

#endif