#ifndef __UART5_H__
#define __UART5_H__

#include "stm32f4xx.h"

// UART5初始化(PC12/TX, PD2/RX), baud为波特率(如115200)
void UART5_Init(uint32_t baud);

#endif
