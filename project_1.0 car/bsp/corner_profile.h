#ifndef __CORNER_PROFILE_H__
#define __CORNER_PROFILE_H__

// 将值限制在[0.0, 1.0]范围内
// 参数value: 输入值
// 返回值: 限制后的值
static inline float CornerProfile_Clamp01(float value)
{
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

// 五次平滑阶梯函数(两端斜率/加速度均为零)
// 参数progress: 输入进度值∈[0,1]
// 返回值: 平滑输出值∈[0,1]
static inline float CornerProfile_Smoothstep5(float progress)
{
    float u = CornerProfile_Clamp01(progress);
    return u * u * u * (u * (u * 6.0f - 15.0f) + 10.0f);
}

// 斜率限幅函数(速度斜坡限制)
// 参数current: 当前值
// 参数target: 目标值
// 参数max_step: 每ms最大变化量
// 返回值: 向目标靠近但不超过max_step速率的新值
static inline float CornerProfile_Slew(float current, float target, float max_step)
{
    float delta = target - current;
    if (max_step < 0.0f) max_step = -max_step;
    if (delta > max_step) return current + max_step;
    if (delta < -max_step) return current - max_step;
    return target;
}

#endif
