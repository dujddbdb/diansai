#include "vision_strategy.h"

// ==================== 全局静态变量 ====================
static VisionStrategy_t s_state;      // 策略状态机
static uint16_t         s_scan_timer = 0;  // 扫描定时器(计数)

// ==================== 工具函数 ====================

// 浮点数区间钳位: val ∈ [min_v, max_v]
static inline float clampf(float val, float min_v, float max_v)
{
    // 小于最小值返回最小值
    if (val < min_v) return min_v;
    // 大于最大值返回最大值
    if (val > max_v) return max_v;
    // 在区间内返回原值
    return val;
}

// ==================== 初始化 ====================

// 初始化视觉追踪策略状态机
void VisionStrategy_Init(void)
{
    // 初始状态: 全图搜索
    s_state.roi_state      = ROI_FULL;
    // 清零锁定计数
    s_state.lock_count     = 0;
    // 清零丢失计数
    s_state.lose_count     = 0;
    // 清零云台角度
    s_state.gimbal_x_angle = 0.0f;
    s_state.gimbal_y_angle = 0.0f;
    // 关闭扫描标志
    s_state.scanning       = 0;
    // 清零扫描定时器
    s_scan_timer           = 0;
}

// ==================== ROI三阶段状态机 ====================
// 阶段转换:
//   检测到目标 → lock_count++ → 达到LOCK_FRAMES → ROI_SMALL(精确锁定)
//   丢失目标   → lose_count++ → 达到LOSE_FRAMES → ROI_EXPAND(扩展搜索)
//               → 继续丢失达到FULL_FRAMES → ROI_FULL(全图扫描)
// 状态回退: 只有重新检测到目标才会从大ROI回到小ROI

// ROI状态机更新: 根据目标有无自动切换ROI大小
void VisionStrategy_Update(uint8_t target_detected)
{
    // ===== 目标检测到 =====
    if (target_detected) {
        // 清零丢失计数
        s_state.lose_count = 0;

        // 锁定计数加1(防溢出)
        if (s_state.lock_count < 65535) {
            s_state.lock_count++;
        }

        // 连续锁定足够帧数 → 切换到小ROI精确追踪
        if (s_state.lock_count >= VISION_LOCK_FRAMES
            && s_state.roi_state != ROI_SMALL) {
            // 切换到小ROI
            s_state.roi_state = ROI_SMALL;
            // 停止扫描
            s_state.scanning  = 0;
        }
    } else {
        // ===== 目标丢失 =====
        // 清零锁定计数
        s_state.lock_count = 0;

        // 丢失计数加1(防溢出)
        if (s_state.lose_count < 65535) {
            s_state.lose_count++;
        }

        // 丢失一定帧数 → 扩大ROI区域重新搜索(从小ROI到扩展ROI)
        if (s_state.lose_count >= VISION_LOSE_FRAMES
            && s_state.roi_state != ROI_EXPAND
            && s_state.roi_state != ROI_FULL) {
            // 切换到扩展ROI
            s_state.roi_state = ROI_EXPAND;
        }

        // 丢失更多帧数 → 切换到全图搜索并启动蛇形扫描
        if (s_state.lose_count >= VISION_FULL_FRAMES
            && s_state.roi_state != ROI_FULL) {
            // 切换到全图ROI
            s_state.roi_state = ROI_FULL;
            // 启动蛇形扫描
            s_state.scanning  = 1;
        }
    }
}

// ==================== 状态查询 ====================

// 获取当前ROI状态
ROI_State_t VisionStrategy_GetROIState(void)
{
    return s_state.roi_state;
}

// 获取完整策略状态结构体指针
VisionStrategy_t* VisionStrategy_GetState(void)
{
    return &s_state;
}

// ==================== 云台角度反馈 ====================

// 设置云台当前角度反馈，供扫描使用
void VisionStrategy_SetGimbalAngle(float x_angle, float y_angle)
{
    // 保存X轴角度
    s_state.gimbal_x_angle = x_angle;
    // 保存Y轴角度
    s_state.gimbal_y_angle = y_angle;
}

// ==================== 蛇形扫描 ====================
// 扫描路径: X轴从左到右来回扫描，到右边界后Y轴步进一行，X回到左边界
//           Y轴从上到下扫描，到下边界后回到上边界，重新开始

// 执行云台蛇形扫描搜索
void VisionStrategy_GimbalScan(void)
{
    // 未开启扫描，直接返回
    if (!s_state.scanning) return;

    // 定时器控制扫描步进间隔(计数未到则等待)
    if (s_scan_timer < VISION_SCAN_INTERVAL_MS) {
        s_scan_timer++;
        return;
    }
    // 计时到，清零定时器
    s_scan_timer = 0;

    // X轴步进: 每次向右移动一个扫描速度
    s_state.gimbal_x_angle += VISION_SCAN_SPEED;

    // X轴到达右边界: 回左边界，Y轴步进一行
    if (s_state.gimbal_x_angle > VISION_SCAN_X_MAX) {
        // X回到左边界
        s_state.gimbal_x_angle = VISION_SCAN_X_MIN;
        // Y轴向下步进一行
        s_state.gimbal_y_angle += VISION_SCAN_SPEED;

        // Y轴到达下边界: 回上边界，开始新一轮扫描
        if (s_state.gimbal_y_angle > VISION_SCAN_Y_MAX) {
            // Y回到上边界
            s_state.gimbal_y_angle = VISION_SCAN_Y_MIN;
        }
    }

    // ===== 角度限位保护 =====
    // 钳位X轴到允许范围，防止越界
    s_state.gimbal_x_angle = clampf(s_state.gimbal_x_angle,
                                    VISION_SCAN_X_MIN,
                                    VISION_SCAN_X_MAX);
    // 钳位Y轴到允许范围，防止越界
    s_state.gimbal_y_angle = clampf(s_state.gimbal_y_angle,
                                    VISION_SCAN_Y_MIN,
                                    VISION_SCAN_Y_MAX);
}

// ==================== 重置 ====================

// 重置策略状态机为初始状态
void VisionStrategy_Reset(void)
{
    VisionStrategy_Init();
}
