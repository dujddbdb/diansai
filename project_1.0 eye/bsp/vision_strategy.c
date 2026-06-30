#include "vision_strategy.h"

static VisionStrategy_t s_state;
static uint16_t         s_scan_timer = 0;

static inline float clampf(float val, float min_v, float max_v)
{
    if (val < min_v) return min_v;
    if (val > max_v) return max_v;
    return val;
}

void VisionStrategy_Init(void)
{
    s_state.roi_state      = ROI_FULL;
    s_state.lock_count     = 0;
    s_state.lose_count     = 0;
    s_state.gimbal_x_angle = 0.0f;
    s_state.gimbal_y_angle = 0.0f;
    s_state.scanning       = 0;
    s_scan_timer           = 0;
}

void VisionStrategy_Update(uint8_t target_detected)
{
    /* 目标检测到: 清零丢失计数，累加锁定计数 */
    if (target_detected) {
        s_state.lose_count = 0;

        if (s_state.lock_count < 65535) {
            s_state.lock_count++;
        }

        /* 连续锁定足够帧数 → 切换到小ROI精确追踪 */
        if (s_state.lock_count >= VISION_LOCK_FRAMES
            && s_state.roi_state != ROI_SMALL) {
            s_state.roi_state = ROI_SMALL;
            s_state.scanning  = 0;
        }
    } else {
        /* 目标丢失: 清零锁定计数，累加丢失计数 */
        s_state.lock_count = 0;

        if (s_state.lose_count < 65535) {
            s_state.lose_count++;
        }

        /* 丢失一定帧数 → 扩大到扩展ROI */
        if (s_state.lose_count >= VISION_LOSE_FRAMES
            && s_state.roi_state != ROI_EXPAND
            && s_state.roi_state != ROI_FULL) {
            s_state.roi_state = ROI_EXPAND;
        }

        /* 丢失更多帧数 → 切换到全图扫描 */
        if (s_state.lose_count >= VISION_FULL_FRAMES
            && s_state.roi_state != ROI_FULL) {
            s_state.roi_state = ROI_FULL;
            s_state.scanning  = 1;
        }
    }
}

ROI_State_t VisionStrategy_GetROIState(void)
{
    return s_state.roi_state;
}

VisionStrategy_t* VisionStrategy_GetState(void)
{
    return &s_state;
}

void VisionStrategy_SetGimbalAngle(float x_angle, float y_angle)
{
    s_state.gimbal_x_angle = x_angle;
    s_state.gimbal_y_angle = y_angle;
}

void VisionStrategy_GimbalScan(void)
{
    if (!s_state.scanning) return;

    /* 定时器控制扫描步进间隔 */
    if (s_scan_timer < VISION_SCAN_INTERVAL_MS) {
        s_scan_timer++;
        return;
    }
    s_scan_timer = 0;

    /* X轴向右步进 */
    s_state.gimbal_x_angle += VISION_SCAN_SPEED;

    /* X轴到达右边界: 回左边界，Y轴步进一行 */
    if (s_state.gimbal_x_angle > VISION_SCAN_X_MAX) {
        s_state.gimbal_x_angle = VISION_SCAN_X_MIN;
        s_state.gimbal_y_angle += VISION_SCAN_SPEED;

        /* Y轴到达下边界: 回上边界，重新开始 */
        if (s_state.gimbal_y_angle > VISION_SCAN_Y_MAX) {
            s_state.gimbal_y_angle = VISION_SCAN_Y_MIN;
        }
    }

    /* 角度限位保护 */
    s_state.gimbal_x_angle = clampf(s_state.gimbal_x_angle,
                                    VISION_SCAN_X_MIN,
                                    VISION_SCAN_X_MAX);
    s_state.gimbal_y_angle = clampf(s_state.gimbal_y_angle,
                                    VISION_SCAN_Y_MIN,
                                    VISION_SCAN_Y_MAX);
}

void VisionStrategy_Reset(void)
{
    VisionStrategy_Init();
}
