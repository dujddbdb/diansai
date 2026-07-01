// @file    usart3.h
// @brief   USART3 接口声明 (辅助调试串口, 全功能同步异步串口)
// @note    硬件: PD8=TX, PD9=RX, APB1总线, 115200-8N1, 无流控, 无中断
//          USART3与UART的区别: USART支持同步时钟, UART仅异步; STM32F407 USART3是USART不是UART
//          用途: 调试信息输出, 传感器数据打印

#ifndef __USART3_H__
#define __USART3_H__

#include "stm32f4xx.h"

// USART3初始化函数
// 功能: 初始化USART3串口, 配置GPIO引脚(PD8=TX/PD9=RX)和串口参数(8N1格式)
// 参数: baud - 波特率, 典型值为115200
// 返回值: 无
void USART3_Init(uint32_t baud);

// USART3发送单字节函数
// 功能: 轮询方式发送一个字节数据, 等待TXE标志, 超时8000次
// 参数: data - 要发送的字节数据
// 返回值: 无
void USART3_SendByte(uint8_t data);

// USART3发送字符串函数
// 功能: 逐字节发送字符串, 直到遇到字符串结束符'\0'
// 参数: str - 指向要发送的字符串的指针
// 返回值: 无
void USART3_SendString(const char *str);

#endif
