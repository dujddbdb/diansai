#ifndef __USART3_H__
#define __USART3_H__

#include "stm32f4xx.h"

// USART3初始化
// 功能：配置USART3串口（PD8=TX，PD9=RX），设置波特率、数据位、停止位等参数
// 参数：baud - 串口通信波特率（如115200）
// 返回值：无
void USART3_Init(uint32_t baud);

// USART3发送单字节（轮询方式）
// 功能：通过USART3发送一个字节的数据，采用轮询方式等待发送完成
// 参数：data - 待发送的字节数据（0x00~0xFF）
// 返回值：无
void USART3_SendByte(uint8_t data);

// USART3发送字符串
// 功能：通过USART3发送以'\0'结尾的字符串，不包含结束符本身
// 参数：str - 指向待发送字符串的指针（必须以'\0'结尾）
// 返回值：无
void USART3_SendString(const char *str);

#endif
