/**
 * @file    uart_k230.h
 * @brief   K230 raw error receiver for the EYE controller.
 *
 * K230 sends exactly four bytes on every frame:
 *   err_y int16 big-endian, err_z int16 big-endian
 *
 * err_y: +Y error, image right is positive.
 * err_z: +Z error, image up is positive.
 */
#ifndef __UART_K230_H__
#define __UART_K230_H__

#include "stm32f4xx.h"
#include <stdbool.h>

#define K230_BAUD           115200U  // K230通信波特率
#define K230_RX_BUF_SIZE    8U       // 接收缓冲区大小
#define K230_IRQ_PRIO       2U       // 串口中断优先级

#define K230_TRACK_RAW_LEN 4U        // 单帧原始数据长度(字节)
#define K230_TRACK_INVALID 0x7FFF    // 无效追踪标记值

typedef struct {
    int16_t err_y;          // Y轴误差(像素)
    int16_t err_z;          // Z轴误差(像素)
    bool    track_valid;    // 目标追踪是否有效
    uint8_t error_code;     // 错误码: 0=正常, 2=长度错误
} K230_ParsedData;

extern uint8_t k230_rx_buf[K230_RX_BUF_SIZE]; // 接收缓冲区
extern uint8_t k230_rx_count;                  // 已接收字节计数
extern bool    k230_rx_flag;                   // 接收完成标志
extern K230_ParsedData k230_parsed;            // 解析后的数据

// 初始化K230通信串口，baud: 波特率
void K230_UART_Init(uint32_t baud);
// 解析接收缓冲区中的数据包，填充k230_parsed
void K230_ParsePacket(void);

#endif /* __UART_K230_H__ */
