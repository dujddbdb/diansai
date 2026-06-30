#include <assert.h>
#include <stdint.h>

#if defined(__has_include) && __has_include("../project_1.0 car/bsp/right_angle_detector.h")
#include "../project_1.0 car/bsp/right_angle_detector.h"
#else
#define RIGHT_ANGLE_BLACK_CONFIRM_SAMPLES 5U
typedef struct { uint8_t filtered; } RightAngleDetector_t;
static void RightAngleDetector_Init(RightAngleDetector_t *d) { d->filtered = 0xFFU; }
static uint8_t RightAngleDetector_Update(RightAngleDetector_t *d, uint8_t raw)
{ d->filtered = raw; return raw; }
static uint8_t RightAngleDetector_Feature(uint8_t bits)
{ return ((bits & 0xC3U) == 0xC0U) ? 1U : 0U; }
static uint8_t RightAngleDetector_AllWhite(uint8_t bits) { return bits == 0xFFU; }
#endif

int main(void)
{
    RightAngleDetector_t detector;
    uint8_t filtered;
    uint8_t i;

    RightAngleDetector_Init(&detector);

    /* One grout-line sample must not become a black corner feature. */
    filtered = RightAngleDetector_Update(&detector, 0xFCU);
    assert(filtered == 0xFFU);
    assert(RightAngleDetector_Feature(filtered) == 0U);
    assert(RightAngleDetector_Feature(0x3FU) == 1U);
    assert(RightAngleDetector_Feature(0xFCU) == 2U);

    /* A persistent right-edge pair is a real initial corner feature. */
    for (i = 0U; i < RIGHT_ANGLE_BLACK_CONFIRM_SAMPLES; ++i) {
        filtered = RightAngleDetector_Update(&detector, 0xFCU);
    }
    assert(RightAngleDetector_Feature(filtered) == 2U);

    /* Execution confirmation remains strict eight-channel white. */
    filtered = RightAngleDetector_Update(&detector, 0xFFU);
    assert(RightAngleDetector_AllWhite(filtered) == 1U);
    assert(RightAngleDetector_AllWhite(0x7FU) == 0U);
    return 0;
}
