#include <assert.h>
#include <stdint.h>

#include "../project_1.0 eye/bsp/uart/car_corner_protocol.h"

static void feed_frame(CarCornerParser_t *parser, CarCornerPayload_t *payload,
                       uint8_t state, uint8_t *frames)
{
    *frames += CarCornerParser_Feed(parser, 0xA5U, payload);
    *frames += CarCornerParser_Feed(parser, state, payload);
    *frames += CarCornerParser_Feed(parser, (uint8_t)(0xA5U ^ state), payload);
}

int main(void)
{
    CarCornerParser_t parser;
    CarCornerPayload_t payload;
    uint8_t frames = 0U;

    CarCornerParser_Init(&parser);
    CarCornerParser_Feed(&parser, 0x55U, &payload);
    feed_frame(&parser, &payload, 0x0EU, &frames); /* phase2, left, IMU valid */
    assert(frames == 1U);
    assert(payload.phase == 2U);
    assert(payload.type == 2U);
    assert((payload.flags & 0x01U) != 0U);

    CarCornerParser_Feed(&parser, 0xA5U, &payload);
    CarCornerParser_Feed(&parser, 0x02U, &payload);
    CarCornerParser_Feed(&parser, 0x00U, &payload); /* bad check byte */
    feed_frame(&parser, &payload, 0x03U, &frames);
    assert(frames == 2U);
    assert(payload.phase == 3U && payload.type == 1U);
    return 0;
}
