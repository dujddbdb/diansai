#ifndef RIGHT_ANGLE_DETECTOR_H
#define RIGHT_ANGLE_DETECTOR_H

#include <stdint.h>
#include <string.h>
#include "track_config.h"

// 直角检测器状态结构体
// 用于存储8通道灰度传感器的去抖滤波状态和直角特征检测结果
typedef struct {
    uint8_t black_hits[8];     // 各通道黑电平命中计数（去抖用，连续命中次数）
    uint8_t filtered;          // 滤波后的数字量输出（bit=1表示白，bit=0表示黑）
    uint8_t feature_type;      // 当前检测到的特征类型（0-无特征，1-右直角，2-左直角）
    uint8_t feature_hits;      // 特征连续命中计数（用于去抖确认）
    uint8_t white_hits;        // 全白连续命中计数（用于出弯判断去抖）
    uint8_t abandon_hits;      // 放弃预触发连续命中计数（中间黑+边缘白，去抖用）
} RightAngleDetector_t;

// 检测直角特征（无滤波，即时判断）
// 功能：根据8位数字量直接判断是否存在左/右直角特征
// 参数：bits - 8位数字量（bit=1表示白，bit=0表示黑）
// 返回值：0-无特征，1-右直角（左侧两通道黑），2-左直角（右侧两通道黑）
static inline uint8_t RightAngleDetector_Feature(uint8_t bits)
{
    // 判断左高两位是否全黑 → 右直角
    if ((bits & 0xC0U) == 0x00U) return 1U;
    // 判断右低两位是否全黑 → 左直角
    if ((bits & 0x03U) == 0x00U) return 2U;
    return 0U;
}

// 判断是否全白
// 功能：判断8个通道是否全部为白色
// 参数：bits - 8位数字量
// 返回值：1-全白（0xFF），0-非全白
static inline uint8_t RightAngleDetector_AllWhite(uint8_t bits)
{
    return (bits == 0xFFU) ? 1U : 0U;
}

// 统计白通道数量
// 功能：统计8个通道中白色通道的个数
// 参数：bits - 8位数字量（bit=1表示白，bit=0表示黑）
// 返回值：白通道个数（范围0~8）
static inline uint8_t RightAngleDetector_WhiteCount(uint8_t bits)
{
    uint8_t i;
    uint8_t count = 0U;
    // 遍历8个通道，统计白色（bit=1）的数量
    for (i = 0U; i < 8U; ++i) {
        if ((bits & (uint8_t)(1U << i)) != 0U) count++;
    }
    return count;
}

// 判断白通道数量是否达标
// 功能：判断白色通道数量是否达到配置的最小阈值
// 参数：bits - 8位数字量
// 返回值：1-白通道数>=阈值，0-不达标
static inline uint8_t RightAngleDetector_WhiteEnough(uint8_t bits)
{
    return (RightAngleDetector_WhiteCount(bits) >= RIGHT_ANGLE_WHITE_MIN) ? 1U : 0U;
}

// 判断中间通道（2/3/4/5）是否存在黑线
// 功能：用于识别"已回到正常巡线中心黑线"的图案，区分真直角与横杆/交叉线误触发
// 参数：bits - 8位数字量
// 返回值：1-中间通道存在黑线，0-中间通道全白
static inline uint8_t RightAngleDetector_MiddleBlack(uint8_t bits)
{
    return ((bits & 0x3CU) != 0x3CU) ? 1U : 0U;
}

// 判断边缘通道（0/1/6/7）是否全白
// 功能：真直角特征侧通道在预触发期间应保持黑电平直至全白；若边缘先于中间转白，
//       说明黑色区域曾覆盖全宽（横杆/交叉线），并非从单侧收窄的真直角
// 参数：bits - 8位数字量
// 返回值：1-四个边缘通道全白，0-至少一个边缘通道仍为黑
static inline uint8_t RightAngleDetector_EdgesWhite(uint8_t bits)
{
    return ((bits & 0xC3U) == 0xC3U) ? 1U : 0U;
}

// 初始化直角检测器
// 功能：清零检测器所有状态，设置filtered初始值为全白(0xFF)
// 参数：detector - 检测器实例指针
// 返回值：无
static inline void RightAngleDetector_Init(RightAngleDetector_t *detector)
{
    // 清零整个结构体
    memset(detector, 0, sizeof(*detector));
    // 初始状态设为全白（未检测到黑线）
    detector->filtered = 0xFFU;
}

// 更新检测器状态（每1ms调用一次）
// 功能：执行逐通道去抖滤波，检测直角特征，统计全白状态
// 参数：detector - 检测器实例指针
// 参数：raw - 当前帧8位数字量（bit=1表示白，bit=0表示黑）
// 返回值：去抖滤波后的8位数字量
static inline uint8_t RightAngleDetector_Update(RightAngleDetector_t *detector,
                                                uint8_t raw)
{
    uint8_t i;
    uint8_t type;

    // 黑电平去抖滤波：逐通道处理
    for (i = 0U; i < 8U; ++i) {
        uint8_t mask = (uint8_t)(1U << i);
        if ((raw & mask) != 0U) {
            // 当前通道为白：清零黑命中计数，滤波输出设为白
            detector->black_hits[i] = 0U;
            detector->filtered |= mask;
        } else {
            // 当前通道为黑：累加命中计数，达到阈值后确认黑电平
            if (detector->black_hits[i] < RIGHT_ANGLE_BLACK_CONFIRM_SAMPLES) {
                detector->black_hits[i]++;
            }
            // 命中次数达到确认阈值，滤波输出设为黑
            if (detector->black_hits[i] >= RIGHT_ANGLE_BLACK_CONFIRM_SAMPLES) {
                detector->filtered &= (uint8_t)~mask;
            }
        }
    }

    // 直角特征确认（去抖）
    type = RightAngleDetector_Feature(detector->filtered);
    if (type == 0U) {
        // 无特征：清零特征类型和命中计数
        detector->feature_type = 0U;
        detector->feature_hits = 0U;
    } else if (type == detector->feature_type) {
        // 特征类型一致：累加命中计数
        if (detector->feature_hits < RIGHT_ANGLE_FEATURE_CONFIRM_SAMPLES) {
            detector->feature_hits++;
        }
    } else {
        // 特征类型变化：重置为新类型，命中计数从1开始
        detector->feature_type = type;
        detector->feature_hits = 1U;
    }

    // 全白确认（出弯判断用，去抖）
    if (RightAngleDetector_AllWhite(detector->filtered)) {
        // 全白：累加命中计数
        if (detector->white_hits < RIGHT_ANGLE_WHITE_CONFIRM_SAMPLES) {
            detector->white_hits++;
        }
    } else {
        // 非全白：清零计数
        detector->white_hits = 0U;
    }

    // 放弃预触发确认（去抖）：无特征 + 中间黑 + 边缘白 = 已回到正常巡线图案，
    // 说明预触发是横杆/交叉线等全宽黑特征造成的误判，而非真直角收窄
    if (type == 0U &&
        RightAngleDetector_MiddleBlack(detector->filtered) &&
        RightAngleDetector_EdgesWhite(detector->filtered)) {
        if (detector->abandon_hits < RIGHT_ANGLE_ABANDON_CONFIRM_SAMPLES) {
            detector->abandon_hits++;
        }
    } else {
        detector->abandon_hits = 0U;
    }

    return detector->filtered;
}

// 获取已确认的直角特征类型（经过去抖）
// 功能：返回经过去抖确认后的直角特征类型
// 参数：detector - 检测器实例指针
// 返回值：0-无，1-右直角，2-左直角
static inline uint8_t RightAngleDetector_ConfirmedFeature(
    const RightAngleDetector_t *detector)
{
    // 命中次数达到确认阈值才认为特征有效
    return (detector->feature_hits >= RIGHT_ANGLE_FEATURE_CONFIRM_SAMPLES)
         ? detector->feature_type : 0U;
}

// 检查全白状态是否已确认（经过去抖）
// 功能：判断全白状态是否经过去抖确认
// 参数：detector - 检测器实例指针
// 返回值：1-全白已确认，0-未确认
static inline uint8_t RightAngleDetector_WhiteConfirmed(
    const RightAngleDetector_t *detector)
{
    // 全白命中次数达到确认阈值才认为有效
    return (detector->white_hits >= RIGHT_ANGLE_WHITE_CONFIRM_SAMPLES) ? 1U : 0U;
}

// 检查放弃预触发条件是否已确认（经过去抖）
// 功能：判断"中间黑+边缘白"图案是否经过去抖确认，用于预触发期间放弃误触发
// 参数：detector - 检测器实例指针
// 返回值：1-放弃条件已确认，0-未确认
static inline uint8_t RightAngleDetector_AbandonConfirmed(
    const RightAngleDetector_t *detector)
{
    return (detector->abandon_hits >= RIGHT_ANGLE_ABANDON_CONFIRM_SAMPLES) ? 1U : 0U;
}

// 重置放弃预触发的去抖计数
// 功能：进入预触发阶段时清零，避免沿用进入前直道巡线积累的命中数
// 参数：detector - 检测器实例指针
// 返回值：无
static inline void RightAngleDetector_ResetAbandon(RightAngleDetector_t *detector)
{
    detector->abandon_hits = 0U;
}

#endif
