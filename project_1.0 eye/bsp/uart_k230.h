// @file    uart_k230.h
// @brief   K230视觉模块原始误差数据接收接口
// @note    K230每帧发送4个字节: err_y(int16大端) + err_z(int16大端)
//          err_y: Y轴误差, 图像右侧为正方向
//          err_z: Z轴误差, 图像上方为正方向

#ifndef __UART_K230_H__
#define __UART_K230_H__

#include "stm32f4xx.h"
#include <stdbool.h>

// K230通信波特率, 默认115200
#define K230_BAUD           115200U
// 接收缓冲区大小(字节), 设置为8字节留有余量
#define K230_RX_BUF_SIZE    8U
// 串口中断优先级
#define K230_IRQ_PRIO       2U

// 单帧原始数据长度(字节), K230每帧固定发送4字节
#define K230_TRACK_RAW_LEN 4U
// 无效追踪标记值, 表示当前未检测到目标
#define K230_TRACK_INVALID 0x7FFF

// K230解析后的数据结构体
// 存储从K230接收到的目标追踪误差信息
typedef struct {
    int16_t err_y;          // Y轴误差(像素), 图像右侧为正方向
    int16_t err_z;          // Z轴误差(像素), 图像上方为正方向
    bool    track_valid;    // 目标追踪是否有效, true=有效 false=无效
    uint8_t error_code;     // 错误码: 0=正常, 2=数据长度错误
} K230_ParsedData;

// K230接收缓冲区, 存储接收到的原始字节数据
extern uint8_t k230_rx_buf[K230_RX_BUF_SIZE];
// K230已接收字节计数, 记录当前缓冲区中已接收的字节数
extern uint8_t k230_rx_count;
// K230接收完成标志, true表示一帧数据接收完成
extern bool    k230_rx_flag;
// K230解析后的数据, 存放最新一帧解析后的有效数据
extern K230_ParsedData k230_parsed;

// K230串口初始化函数
// 功能: 初始化K230通信的串口外设, 配置GPIO、波特率、中断等
// 参数: baud - 波特率, 典型值为115200
// 返回值: 无
void K230_UART_Init(uint32_t baud);

// K230数据包解析函数
// 功能: 解析接收缓冲区中的数据包, 将结果填充到k230_parsed结构体
// 参数: 无
// 返回值: 无
void K230_ParsePacket(void);

#endif
