#include "vision_strategy.h"

// 视觉策略状态实例
static VisionStrategy_t s_state;
// 蛇形扫描定时器
static uint16_t         s_scan_timer = 0;

// 浮点数钳位函数: 将val限制在[min_v, max_v]范围内
static inline float clampf(float val, float min_v, float max_v)
{
    if (val < min_v) return min_v;
    if (val > max_v) return max_v;
    return val;
}

// 视觉策略初始化
void VisionStrategy_Init(void)
{
    // 初始化ROI状态为全图模式
    s_state.roi_state      = ROI_FULL;
    // 清零锁定计数
    s_state.lock_count     = 0;
    // 清零丢失计数
    s_state.lose_count     = 0;
    // 清零云台X轴角度
    s_state.gimbal_x_angle = 0.0f;
    // 清零云台Y轴角度
    s_state.gimbal_y_angle = 0.0f;
    // 关闭扫描状态
    s_state.scanning       = 0;
    // 清零扫描定时器
    s_scan_timer           = 0;
}

// ROI状态机: 三阶段自动切换
// 阶段1: ROI_SMALL — 小ROI精确追踪
// 阶段2: ROI_EXPAND — 丢失后扩展ROI重新搜索
// 阶段3: ROI_FULL   — 全图扫描搜索目标
void VisionStrategy_Update(uint8_t target_detected)
{
    // 目标检测到的情况
    if (target_detected) {
        // 目标出现，清零丢失计数
        s_state.lose_count = 0;

        // 锁定计数累加(防止溢出)
        if (s_state.lock_count < 65535) {
            s_state.lock_count++;
        }

        // 小ROI锁定切换: 连续锁定足够帧数 → 切换到小ROI精确追踪
        if (s_state.lock_count >= VISION_LOCK_FRAMES
            && s_state.roi_state != ROI_SMALL) {
            s_state.roi_state = ROI_SMALL;
            s_state.scanning  = 0;
        }
    } else {
        // 目标丢失的情况: 清零锁定计数
        s_state.lock_count = 0;

        // 丢失计数累加(防止溢出)
        if (s_state.lose_count < 65535) {
            s_state.lose_count++;
        }

        // 扩展ROI切换: 丢失一定帧数 → 扩大ROI区域重新搜索
        if (s_state.lose_count >= VISION_LOSE_FRAMES
            && s_state.roi_state != ROI_EXPAND
            && s_state.roi_state != ROI_FULL) {
            s_state.roi_state = ROI_EXPAND;
        }

        // 全图扫描切换: 丢失更多帧数 → 切换到全图搜索并启动蛇形扫描
        if (s_state.lose_count >= VISION_FULL_FRAMES
            && s_state.roi_state != ROI_FULL) {
            s_state.roi_state = ROI_FULL;
            s_state.scanning  = 1;
        }
    }
}

// 获取当前ROI状态
ROI_State_t VisionStrategy_GetROIState(void)
{
    return s_state.roi_state;
}

// 获取视觉策略状态指针
VisionStrategy_t* VisionStrategy_GetState(void)
{
    return &s_state;
}

// 设置云台角度(由vision.c调用，记录当前云台位置)
void VisionStrategy_SetGimbalAngle(float x_angle, float y_angle)
{
    s_state.gimbal_x_angle = x_angle;
    s_state.gimbal_y_angle = y_angle;
}

// 云台蛇形扫描搜索: X轴来回扫描，到边界后Y轴步进一行
void VisionStrategy_GimbalScan(void)
{
    // 扫描未启动，直接返回
    if (!s_state.scanning) return;

    // 定时器控制扫描步进间隔
    if (s_scan_timer < VISION_SCAN_INTERVAL_MS) {
        s_scan_timer++;
        return;
    }
    // 定时器归零，准备执行一次步进
    s_scan_timer = 0;

    // 蛇形路径: X轴每次步进扫描速度
    s_state.gimbal_x_angle += VISION_SCAN_SPEED;

    // X轴到达右边界: 回左边界，Y轴步进一行
    if (s_state.gimbal_x_angle > VISION_SCAN_X_MAX) {
        s_state.gimbal_x_angle = VISION_SCAN_X_MIN;
        s_state.gimbal_y_angle += VISION_SCAN_SPEED;

        // Y轴到达下边界: 回上边界，开始新一轮扫描
        if (s_state.gimbal_y_angle > VISION_SCAN_Y_MAX) {
            s_state.gimbal_y_angle = VISION_SCAN_Y_MIN;
        }
    }

    // 角度限位保护: 钳位到允许范围，防止越界
    s_state.gimbal_x_angle = clampf(s_state.gimbal_x_angle,
                                    VISION_SCAN_X_MIN,
                                    VISION_SCAN_X_MAX);
    s_state.gimbal_y_angle = clampf(s_state.gimbal_y_angle,
                                    VISION_SCAN_Y_MIN,
                                    VISION_SCAN_Y_MAX);
}

// 视觉策略重置
void VisionStrategy_Reset(void)
{
    VisionStrategy_Init();
}
