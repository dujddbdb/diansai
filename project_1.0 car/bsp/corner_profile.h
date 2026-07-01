#ifndef __CORNER_PROFILE_H__
#define __CORNER_PROFILE_H__

// 将值限制在[0.0, 1.0]范围内
// 功能：将输入值钳位到0到1的闭区间内，用于归一化进度值
// 参数：value - 输入值（任意浮点数）
// 返回值：限制后的值，范围[0.0, 1.0]
static inline float CornerProfile_Clamp01(float value)
{
    // 小于下限则返回下限0.0
    if (value < 0.0f) return 0.0f;
    // 大于上限则返回上限1.0
    if (value > 1.0f) return 1.0f;
    // 在范围内直接返回原值
    return value;
}

// 五次平滑阶梯函数（两端斜率/加速度均为零）
// 功能：使用五次多项式实现平滑过渡，在起点和终点处一阶、二阶导数均为零
//       常用于弯道加减速、舵机平滑过渡等需要平滑启停的场景
// 参数：progress - 输入进度值，范围[0, 1]
// 返回值：平滑输出值，范围[0, 1]（S型曲线，中间快两端慢）
static inline float CornerProfile_Smoothstep5(float progress)
{
    // 先钳位到[0,1]范围
    float u = CornerProfile_Clamp01(progress);
    // 五次多项式：6u^5 - 15u^4 + 10u^3
    return u * u * u * (u * (u * 6.0f - 15.0f) + 10.0f);
}

// 斜率限幅函数（速度斜坡限制）
// 功能：限制值的变化速率，实现平滑斜坡过渡，防止突变
//       常用于电机加减速、目标值平滑逼近等场景
// 参数：current - 当前值
// 参数：target - 目标值
// 参数：max_step - 每ms最大变化量（取绝对值，正负均限制）
// 返回值：向目标靠近但不超过max_step速率的新值
static inline float CornerProfile_Slew(float current, float target, float max_step)
{
    // 计算目标与当前的差值
    float delta = target - current;
    // 确保max_step为正数（取绝对值）
    if (max_step < 0.0f) max_step = -max_step;
    // 正向超过最大步长，按最大步长增加
    if (delta > max_step) return current + max_step;
    // 负向超过最大步长，按最大步长减小
    if (delta < -max_step) return current - max_step;
    // 在步长范围内，直接到达目标
    return target;
}

#endif
