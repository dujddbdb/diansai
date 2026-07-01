/** @file car_corner_protocol.h @brief 赛道弯道协议解析器: 车到眼弯道状态解析 */
#ifndef CAR_CORNER_PROTOCOL_H
#define CAR_CORNER_PROTOCOL_H

#include <stdint.h>

/* 数据帧格式: [0xA5][STATE][0xA5 XOR STATE]
 * STATE bit1:0 相位, bit2 方向(1=左转), bit3 车身IMU有效
 */
#define CAR_CORNER_HEADER 0xA5U  // 弯道协议帧头标识

typedef struct {
    uint8_t state;
    uint8_t count;
} CarCornerParser_t;

typedef struct {
    uint8_t phase;
    uint8_t type;
    uint8_t flags;
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

