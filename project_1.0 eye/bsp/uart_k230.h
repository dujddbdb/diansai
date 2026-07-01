/**
 * @file    uart_k230.h
 * @brief   K230 视觉数据接收 - 原始误差帧解析
 *
 * 帧格式: 4字节, err_y(int16大端) + err_z(int16大端)
 * err_y: +Y向右为正, err_z: +Z向上为正
 */
#ifndef __UART_K230_H__
#define __UART_K230_H__

#include "stm32f4xx.h"
#include <stdbool.h>

#define K230_BAUD           115200U  // 通信波特率
#define K230_RX_BUF_SIZE    8U       // 接收缓冲区大小
#define K230_IRQ_PRIO       2U       // 串口中断优先级

#define K230_TRACK_RAW_LEN 4U        // 单帧原始数据长度(字节)
#define K230_TRACK_INVALID 0x7FFF    // 无效追踪标记值(0x7FFF表示无目标)

typedef struct {
    int16_t err_y;          // Y轴误差(像素), 向右为正
    int16_t err_z;          // Z轴误差(像素), 向上为正
    bool    track_valid;    // 目标追踪是否有效
    uint8_t error_code;     // 错误码: 0=正常, 2=长度错误
} K230_ParsedData;

extern uint8_t k230_rx_buf[K230_RX_BUF_SIZE];
extern uint8_t k230_rx_count;
extern bool    k230_rx_flag;
extern K230_ParsedData k230_parsed;

void K230_UART_Init(uint32_t baud);
void K230_ParsePacket(void);

#endif
