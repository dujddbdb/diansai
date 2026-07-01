// @file    car_corner_protocol.h
// @brief   赛道弯道协议解析器: 车端到眼端弯道状态解析
// @note    数据帧格式: [0xA5][STATE][0xA5 XOR STATE]
//          STATE位定义: bit1:0=相位, bit2=方向(1=左转), bit3=车身IMU有效

#ifndef CAR_CORNER_PROTOCOL_H
#define CAR_CORNER_PROTOCOL_H

#include <stdint.h>

// 弯道协议帧头标识字节, 每帧数据的第一个字节为0xA5
#define CAR_CORNER_HEADER 0xA5U

// 弯道协议解析器状态结构体
// 维护字节流解析的状态机
typedef struct {
    uint8_t state;   // 解析状态缓存, 暂存收到的STATE字节
    uint8_t count;   // 当前接收位置计数: 0=等待帧头, 1=已收帧头, 2=已收STATE
} CarCornerParser_t;

// 弯道协议有效载荷结构体
// 解析成功后的弯道状态数据
typedef struct {
    uint8_t phase;   // 弯道相位: 0=直道, 1=入弯, 2=弯中, 3=出弯
    uint8_t type;    // 弯道类型: 0=直道, 1=右弯, 2=左弯
    uint8_t flags;   // 标志位: bit0=IMU数据有效
} CarCornerPayload_t;

// 弯道协议解析器初始化函数
// 功能: 初始化解析器状态, 清零状态和计数
// 参数: parser - 指向解析器结构体的指针
// 返回值: 无
static inline void CarCornerParser_Init(CarCornerParser_t *parser)
{
    parser->state = 0U;
    parser->count = 0U;
}

// 弯道协议解析器喂入字节函数
// 功能: 向解析器喂入一个字节, 进行状态机解析
// 参数: parser - 指向解析器结构体的指针
//       byte - 接收到的字节数据
//       payload - 指向载荷结构体的指针, 解析成功时填充
// 返回值: 1=解析成功(得到一帧有效数据), 0=未完成一帧或校验失败
static inline uint8_t CarCornerParser_Feed(CarCornerParser_t *parser,
                                           uint8_t byte,
                                           CarCornerPayload_t *payload)
{
    if (parser->count == 0U) {
        if (byte == CAR_CORNER_HEADER) parser->count = 1U;
        return 0U;
    }
    if (parser->count == 1U) {
        parser->state = byte;
        parser->count = 2U;
        return 0U;
    }

    if (byte != (uint8_t)(CAR_CORNER_HEADER ^ parser->state)) {
        parser->count = (byte == CAR_CORNER_HEADER) ? 1U : 0U;
        return 0U;
    }
    parser->count = 0U;

    payload->phase = parser->state & 0x03U;
    payload->type = (payload->phase == 0U) ? 0U :
                    ((parser->state & 0x04U) ? 2U : 1U);
    payload->flags = (parser->state & 0x08U) ? 0x01U : 0U;
    return 1U;
}

#endif
