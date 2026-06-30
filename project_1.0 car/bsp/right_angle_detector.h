#ifndef RIGHT_ANGLE_DETECTOR_H
#define RIGHT_ANGLE_DETECTOR_H

#include <stdint.h>
#include <string.h>
#include "track_config.h"

// 直角检测器状态结构体
typedef struct {
    uint8_t black_hits[8];     // 各通道黑电平命中计数(去抖)
    uint8_t filtered;          // 滤波后的数字量输出(bit=1白/0黑)
    uint8_t feature_type;      // 0-无特征 1-右直角 2-左直角
    uint8_t feature_hits;      // 特征连续命中计数
    uint8_t white_hits;        // 全白连续命中计数
} RightAngleDetector_t;

// 检测直角特征(无滤波，即时判断)
// 参数bits: 8位数字量(bit=1白/0黑)
// 返回值: 0-无特征 1-右直角 2-左直角
static inline uint8_t RightAngleDetector_Feature(uint8_t bits)
{
    if ((bits & 0xC0U) == 0x00U) return 1U;
    if ((bits & 0x03U) == 0x00U) return 2U;
    return 0U;
}

// 判断是否全白
// 参数bits: 8位数字量
// 返回值: 1-全白(0xFF) 0-非全白
static inline uint8_t RightAngleDetector_AllWhite(uint8_t bits)
{
    return (bits == 0xFFU) ? 1U : 0U;
}

// 统计白通道数量
// 参数bits: 8位数字量(bit=1白/0黑)
// 返回值: 白通道个数(0~8)
static inline uint8_t RightAngleDetector_WhiteCount(uint8_t bits)
{
    uint8_t i;
    uint8_t count = 0U;
    for (i = 0U; i < 8U; ++i) {
        if ((bits & (uint8_t)(1U << i)) != 0U) count++;
    }
    return count;
}

// 判断白通道数量是否达标
// 参数bits: 8位数字量
// 返回值: 1-白通道数>=阈值 0-不达标
static inline uint8_t RightAngleDetector_WhiteEnough(uint8_t bits)
{
    return (RightAngleDetector_WhiteCount(bits) >= RIGHT_ANGLE_WHITE_MIN) ? 1U : 0U;
}

// 初始化直角检测器(清零状态, filtered=0xFF全白)
// 参数detector: 检测器实例指针
static inline void RightAngleDetector_Init(RightAngleDetector_t *detector)
{
    memset(detector, 0, sizeof(*detector));
    detector->filtered = 0xFFU;
}

// 更新检测器状态(每1ms调用一次)
// 参数detector: 检测器实例指针
// 参数raw: 当前帧8位数字量(bit=1白/0黑)
// 返回值: 去抖滤波后的8位数字量
static inline uint8_t RightAngleDetector_Update(RightAngleDetector_t *detector,
                                                uint8_t raw)
{
    uint8_t i;
    uint8_t type;

    // 黑电平去抖滤波
    for (i = 0U; i < 8U; ++i) {
        uint8_t mask = (uint8_t)(1U << i);
        if ((raw & mask) != 0U) {
            detector->black_hits[i] = 0U;
            detector->filtered |= mask;
        } else {
            if (detector->black_hits[i] < RIGHT_ANGLE_BLACK_CONFIRM_SAMPLES) {
                detector->black_hits[i]++;
            }
            if (detector->black_hits[i] >= RIGHT_ANGLE_BLACK_CONFIRM_SAMPLES) {
                detector->filtered &= (uint8_t)~mask;
            }
        }
    }

    // 直角特征确认
    type = RightAngleDetector_Feature(detector->filtered);
    if (type == 0U) {
        detector->feature_type = 0U;
        detector->feature_hits = 0U;
    } else if (type == detector->feature_type) {
        if (detector->feature_hits < RIGHT_ANGLE_FEATURE_CONFIRM_SAMPLES) {
            detector->feature_hits++;
        }
    } else {
        detector->feature_type = type;
        detector->feature_hits = 1U;
    }

    // 全白确认(出弯判断)
    if (RightAngleDetector_AllWhite(detector->filtered)) {
        if (detector->white_hits < RIGHT_ANGLE_WHITE_CONFIRM_SAMPLES) {
            detector->white_hits++;
        }
    } else {
        detector->white_hits = 0U;
    }

    return detector->filtered;
}

// 获取已确认的直角特征类型(经过去抖)
// 参数detector: 检测器实例指针
// 返回值: 0-无 1-右直角 2-左直角
static inline uint8_t RightAngleDetector_ConfirmedFeature(
    const RightAngleDetector_t *detector)
{
    return (detector->feature_hits >= RIGHT_ANGLE_FEATURE_CONFIRM_SAMPLES)
         ? detector->feature_type : 0U;
}

// 检查全白状态是否已确认(经过去抖)
// 参数detector: 检测器实例指针
// 返回值: 1-全白已确认 0-未确认
static inline uint8_t RightAngleDetector_WhiteConfirmed(
    const RightAngleDetector_t *detector)
{
    return (detector->white_hits >= RIGHT_ANGLE_WHITE_CONFIRM_SAMPLES) ? 1U : 0U;
}

#endif
