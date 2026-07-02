#include <assert.h>
#include <stdint.h>

#if defined(__has_include) && __has_include("../project_1.0 car/bsp/right_angle_detector.h")
#include "../project_1.0 car/bsp/right_angle_detector.h"
#else
#define RIGHT_ANGLE_BLACK_CONFIRM_SAMPLES 5U
#define RIGHT_ANGLE_WHITE_CONFIRM_SAMPLES 2U
#define RIGHT_ANGLE_ABANDON_CONFIRM_SAMPLES 3U
typedef struct {
    uint8_t filtered;
    uint8_t feature_type;
    uint8_t feature_hits;
    uint8_t white_hits;
    uint8_t abandon_hits;
} RightAngleDetector_t;
static void RightAngleDetector_Init(RightAngleDetector_t *d)
{ d->filtered = 0xFFU; d->feature_type = 0U; d->feature_hits = 0U; d->white_hits = 0U; d->abandon_hits = 0U; }
static uint8_t RightAngleDetector_Feature(uint8_t bits)
{ if ((bits & 0xC0U) == 0x00U) return 1U; if ((bits & 0x03U) == 0x00U) return 2U; return 0U; }
static uint8_t RightAngleDetector_AllWhite(uint8_t bits) { return bits == 0xFFU; }
static uint8_t RightAngleDetector_MiddleBlack(uint8_t bits)
{ return ((bits & 0x3CU) != 0x3CU) ? 1U : 0U; }
static uint8_t RightAngleDetector_EdgesWhite(uint8_t bits)
{ return ((bits & 0xC3U) == 0xC3U) ? 1U : 0U; }
static uint8_t RightAngleDetector_Update(RightAngleDetector_t *d, uint8_t raw)
{
    uint8_t type;
    d->filtered = raw; /* simplified stub: no per-channel debounce */
    type = RightAngleDetector_Feature(d->filtered);
    if (type == 0U) { d->feature_type = 0U; d->feature_hits = 0U; }
    else if (type == d->feature_type) { d->feature_hits++; }
    else { d->feature_type = type; d->feature_hits = 1U; }
    if (RightAngleDetector_AllWhite(d->filtered)) {
        if (d->white_hits < RIGHT_ANGLE_WHITE_CONFIRM_SAMPLES) d->white_hits++;
    } else { d->white_hits = 0U; }
    if (type == 0U && RightAngleDetector_MiddleBlack(d->filtered) &&
        RightAngleDetector_EdgesWhite(d->filtered)) {
        if (d->abandon_hits < RIGHT_ANGLE_ABANDON_CONFIRM_SAMPLES) d->abandon_hits++;
    } else { d->abandon_hits = 0U; }
    return d->filtered;
}
static uint8_t RightAngleDetector_ConfirmedFeature(const RightAngleDetector_t *d)
{ return (d->feature_hits >= 1U) ? d->feature_type : 0U; }
static uint8_t RightAngleDetector_WhiteConfirmed(const RightAngleDetector_t *d)
{ return (d->white_hits >= RIGHT_ANGLE_WHITE_CONFIRM_SAMPLES) ? 1U : 0U; }
static uint8_t RightAngleDetector_AbandonConfirmed(const RightAngleDetector_t *d)
{ return (d->abandon_hits >= RIGHT_ANGLE_ABANDON_CONFIRM_SAMPLES) ? 1U : 0U; }
static void RightAngleDetector_ResetAbandon(RightAngleDetector_t *d) { d->abandon_hits = 0U; }
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

    /* --- Genuine corner: edge-black holds and clears straight to
     * all-white. It must never look like the abandon pattern
     * (middle black, edges white) along the way, so a real corner is
     * never mistakenly cancelled. --- */
    {
        RightAngleDetector_t corner;
        RightAngleDetector_Init(&corner);

        for (i = 0U; i < RIGHT_ANGLE_BLACK_CONFIRM_SAMPLES; ++i) {
            filtered = RightAngleDetector_Update(&corner, 0xFCU);
        }
        assert(RightAngleDetector_ConfirmedFeature(&corner) == 2U);

        for (i = 0U; i < (uint8_t)(RIGHT_ANGLE_WHITE_CONFIRM_SAMPLES + 2U); ++i) {
            filtered = RightAngleDetector_Update(&corner, 0xFFU);
            assert(RightAngleDetector_AbandonConfirmed(&corner) == 0U);
        }
        assert(RightAngleDetector_AllWhite(filtered) == 1U);
        assert(RightAngleDetector_WhiteConfirmed(&corner) == 1U);
    }

    /* --- False trigger: a full-width crossbar (finish line / cross
     * track) makes every channel read black at once, which spuriously
     * matches the edge-black feature. Once the car clears the bar and
     * settles back onto the ordinary centre line (middle channels
     * black, edges white), that must be confirmed as an abandon
     * condition -- not mistaken for a real corner in progress. --- */
    {
        RightAngleDetector_t bar;
        RightAngleDetector_Init(&bar);

        for (i = 0U; i < RIGHT_ANGLE_BLACK_CONFIRM_SAMPLES; ++i) {
            filtered = RightAngleDetector_Update(&bar, 0x00U);
        }
        assert(RightAngleDetector_ConfirmedFeature(&bar) != 0U);

        for (i = 0U; i < (uint8_t)(RIGHT_ANGLE_ABANDON_CONFIRM_SAMPLES + 2U); ++i) {
            filtered = RightAngleDetector_Update(&bar, 0xE7U);
        }
        assert(RightAngleDetector_AllWhite(filtered) == 0U);
        assert(RightAngleDetector_AbandonConfirmed(&bar) == 1U);

        /* Re-arming for the next pretrigger must clear the stale tally. */
        RightAngleDetector_ResetAbandon(&bar);
        assert(RightAngleDetector_AbandonConfirmed(&bar) == 0U);
    }

    return 0;
}
