#include <assert.h>
#include <math.h>

#if defined(__has_include) && __has_include("../project_1.0 car/bsp/corner_profile.h")
#include "../project_1.0 car/bsp/corner_profile.h"
#else
/* Baseline model of the current phase switch: command jumps to target. */
static float CornerProfile_Slew(float current, float target, float max_step)
{
    (void)current;
    (void)max_step;
    return target;
}

static float CornerProfile_Smoothstep5(float progress)
{
    return progress;
}
#endif

int main(void)
{
    float first = CornerProfile_Slew(0.0f, 50.0f, 1.25f);
    float mid = CornerProfile_Smoothstep5(0.5f);

    assert(first <= 1.2501f);
    assert(first >= 1.2499f);
    assert(fabsf(mid - 0.5f) < 0.0001f);
    assert(CornerProfile_Smoothstep5(-1.0f) == 0.0f);
    assert(CornerProfile_Smoothstep5(2.0f) == 1.0f);
    return 0;
}
