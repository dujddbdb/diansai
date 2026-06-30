#ifndef __USART3_H__
#define __USART3_H__

#include "stm32f4xx.h"

// USART3初始化(PD8/TX, PD9/RX), baud为波特率(如115200)
void USART3_Init(uint32_t baud);
// USART3发送单字节(轮询方式)
void USART3_SendByte(uint8_t data);
// USART3发送字符串('\0'结尾,不发送结束符)
void USART3_SendString(const char *str);

#endif
