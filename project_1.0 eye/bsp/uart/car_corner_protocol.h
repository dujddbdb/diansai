/**
 * @file    car_corner_protocol.h
 * @brief   赛道弯道协议解析器 - 车到眼弯道状态解析
 *
 * 数据帧格式: [0xA5][STATE][0xA5 XOR STATE]
 * STATE位定义: bit1:0 相位, bit2 方向(1=左转), bit3 车身IMU有效
 */
#ifndef CAR_CORNER_PROTOCOL_H
#define CAR_CORNER_PROTOCOL_H

#include <stdint.h>

#define CAR_CORNER_HEADER 0xA5U  // 弯道协议帧头标识

typedef struct {
    uint8_t state;  // 状态机当前状态/已接收的状态字节
    uint8_t count;  // 已接收字节计数(0-等待帧头, 1-等待状态, 2-等待校验)
} CarCornerParser_t;

typedef struct {
    uint8_t phase;  // 弯道相位(0-直道, 1-入弯, 2-弯中, 3-出弯)
    uint8_t type;   // 弯道类型(0-直道, 1-右转, 2-左转)
    uint8_t flags;  // 标志位(bit0: IMU数据有效)
} CarCornerPayload_t;

static inline void CarCornerParser_Init(CarCornerParser_t *parser)
{
    parser->state = 0U;
    parser->count = 0U;
}

static inline uint8_t CarCornerParser_Feed(CarCornerParser_t *parser,
                                           uint8_t byte,
                                           CarCornerPayload_t *payload)
{
    /* 状态0: 等待帧头0xA5 */
    if (parser->count == 0U) {
        if (byte == CAR_CORNER_HEADER) parser->count = 1U;
        return 0U;
    }
    /* 状态1: 接收状态字节STATE */
    if (parser->count == 1U) {
        parser->state = byte;
        parser->count = 2U;
        return 0U;
    }

    /* 状态2: 校验和验证 (0xA5 XOR STATE) */
    if (byte != (uint8_t)(CAR_CORNER_HEADER ^ parser->state)) {
        /* 校验失败: 如果当前字节是帧头则重新开始, 否则回到等待状态 */
        parser->count = (byte == CAR_CORNER_HEADER) ? 1U : 0U;
        return 0U;
    }
    parser->count = 0U;

    /* 位域提取: phase = bit1:0 */
    payload->phase = parser->state & 0x03U;
    /* 位域提取: type = bit2 (0=右转, 1=左转), phase=0时type=0(直道) */
    payload->type = (payload->phase == 0U) ? 0U :
                    ((parser->state & 0x04U) ? 2U : 1U);
    /* 位域提取: flags = bit3 (IMU数据有效标志) */
    payload->flags = (parser->state & 0x08U) ? 0x01U : 0U;
    return 1U;
}

#endif
