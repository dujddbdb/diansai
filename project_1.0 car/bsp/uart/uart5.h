#ifndef __UART5_H__
#define __UART5_H__

#include "stm32f4xx.h"

// UART5初始化
// 功能：配置UART5串口（PC12=TX，PD2=RX），设置波特率、数据位、停止位等参数
// 参数：baud - 串口通信波特率（如115200）
// 返回值：无
void UART5_Init(uint32_t baud);

#endif
