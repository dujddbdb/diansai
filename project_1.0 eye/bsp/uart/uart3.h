/**
 * @file    usart3.h
 * @brief   USART3 接口声明 (辅助调试串口, 全功能同步异步串口)
 * @note    硬件: PD8=TX, PD9=RX, APB1总线, 115200-8N1, 无流控, 无中断
 *          USART3与UART的区别: USART支持同步时钟, UART仅异步; STM32F407 USART3是USART不是UART
 *          用途: 调试信息输出, 传感器数据打印
 */

#ifndef __USART3_H__
#define __USART3_H__

#include "stm32f4xx.h"

// 初始化USART3，baud: 波特率(典型115200), 8N1格式, PD8=TX/PD9=RX
void USART3_Init(uint32_t baud);
// 发送单字节，轮询TXE标志，超时8000次
void USART3_SendByte(uint8_t data);
// 发送字符串，逐字节发送直到'\0'
void USART3_SendString(const char *str);

#endif