#ifndef __UART_K230_H__
#define __UART_K230_H__

#include "stm32f4xx.h"

#define K230_BAUD           115200U  // UART通信波特率
#define K230_CORNER_HEADER  0xA5U    // 弯道状态数据帧头

void K230_UART_Init(uint32_t baud);              // 初始化K230通信UART，baud: 波特率
void K230_SendCornerState(uint8_t state);        // 立即发送弯道状态，state: 状态值
void K230_QueueCornerState(uint8_t state);       // 弯道状态加入发送队列，state: 状态值
void K230_PollCornerStateTx(void);               // 轮询发送队列中的弯道状态数据

#endif
