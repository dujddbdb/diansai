#include "vision.h"
#include "uart_k230.h"
#include "peripheral.h"
#include "gimbal_pid.h"
#include "vision_strategy.h"
#include "uart5.h"
#include "key.h"
#include "bno080.h"
#include "board.h"
#include "uart3.h"
#include <math.h>
#include <stddef.h>
#include <stdio.h>

static GimbalDualPID_t  s_gimbal_pid;
static Vision_Context_t  s_vision_ctx;
static uint8_t          s_initialized = 0U;
static uint32_t         __attribute__((unused)) s_debug_last_ms = 0U;
static uint8_t          s_last_fresh_packet = 0U;

#define VISION_TWO_PI               6.2831853f  // 2π常量, 用于圆形扫描相位计算

typedef enum {
    VISION_MODE_IDLE = 0,       // 0-空闲
    VISION_MODE_CIRCLE,         // 1-圆形扫描
    VISION_MODE_SHOOT,          // 2-追踪射击
    VISION_MODE_SHOOT_CIRCLE    // 3-追踪+圆形扫描
} Vision_Mode_t;

// IMU角度一阶卡尔曼滤波器状态
typedef struct {
    float angle;
    float p;
    uint8_t valid;
} Vision_AngleKalman_t;

static Vision_AngleKalman_t s_yaw_kf = {0};
static Vision_AngleKalman_t s_pitch_kf = {0};
static Vision_Mode_t s_eye_mode = VISION_MODE_IDLE;
static float s_circle_phase = 0.0f;

// 浮点数区间钳位: value ∈ [min_v, max_v]
static float Vision_ClampFloat(float value, float min_v, float max_v)
{
    if (value < min_v) return min_v;
    if (value > max_v) return max_v;
    return value;
}

// 角度差环绕处理: 将角度差规约到[-180°, +180°]区间
static float Vision_WrapAngleDeltaDeg(float now_deg, float last_deg)
{
    float delta = now_deg - last_deg;

    while (delta > 180.0f) delta -= 360.0f;
    while (delta < -180.0f) delta += 360.0f;

    return delta;
}

// IMU角度卡尔曼滤波
static float Vision_AngleKalmanUpdate(Vision_AngleKalman_t *kf, float measurement_deg)
{
    float k;

    // 卡尔曼滤波: 首次测量直接初始化
    if (!kf->valid) {
        kf->angle = measurement_deg;
        kf->p = 1.0f;
        kf->valid = 1U;
        return measurement_deg;
    }

    // 预测: 测量值绕回处理后，协方差P += 过程噪声Q
    measurement_deg = kf->angle + Vision_WrapAngleDeltaDeg(measurement_deg, kf->angle);
    kf->p += VISION_IMU_KALMAN_Q;
    // 更新: 卡尔曼增益 K = P / (P + R)
    k = kf->p / (kf->p + VISION_IMU_KALMAN_R);
    // 更新: 估计 = 预测 + K * (测量 - 预测)
    kf->angle += k * Vision_WrapAngleDeltaDeg(measurement_deg, kf->angle);
    // 更新: 协方差 P = (1 - K) * P
    kf->p *= (1.0f - k);

    return kf->angle;
}

// 误差混合因子: 平滑过渡小误差→大误差区间的PID参数
static float Vision_ErrorBlend(float abs_error_px)
{
    float t;

    if (abs_error_px <= PID_ERROR_BLEND_START) return 0.0f;
    if (abs_error_px >= PID_ERROR_BLEND_END) return 1.0f;

    t = (abs_error_px - PID_ERROR_BLEND_START) /
        (PID_ERROR_BLEND_END - PID_ERROR_BLEND_START);
    return t * t * (3.0f - 2.0f * t);
}

// 根据误差大小动态调整电机转速
static uint16_t Vision_UpdateAxisRpm(GimbalPID_t *axis, float abs_error_px, float corner_blend)
{
    // 根据误差混合因子计算目标转速: 误差大→转速高，误差小→转速低
    float blend = Vision_ErrorBlend(abs_error_px);
    float target = VISION_GIMBAL_RPM_MIN +
                   (VISION_GIMBAL_RPM_MAX - VISION_GIMBAL_RPM_MIN) * blend;
    float current = (float)axis->rpm;

    // 转弯时按比例降低转速，避免过冲
    target *= (1.0f - VISION_CORNER_RPM_REDUCE * corner_blend);
    target = Vision_ClampFloat(target, VISION_GIMBAL_RPM_MIN, VISION_GIMBAL_RPM_MAX);

    // 转速平滑过渡: 指数趋近目标值，避免突变
    if (current < 1.0f) current = VISION_GIMBAL_RPM_MIN;
    current += (target - current) * VISION_GIMBAL_RPM_SLEW;
    axis->rpm = (uint16_t)(current + 0.5f);

    return axis->rpm;
}

// 更新双轴云台运行时速度 (根据误差大小自适应调速)
static void Vision_UpdateGimbalRuntimeSpeed(float error_x_px, float error_y_px)
{
    float corner_blend = UART5_CarCornerBlend(HAL_GetTick());

    Vision_UpdateAxisRpm(&s_gimbal_pid.x_axis, fabsf(error_x_px), corner_blend);
    Vision_UpdateAxisRpm(&s_gimbal_pid.y_axis, fabsf(error_y_px), corner_blend);
}

// IMU前馈补偿更新
static uint8_t Vision_UpdateGimbalIMUCompensation(void)
{
    static uint8_t imu_valid = 0U;
    static float last_yaw_deg = 0.0f;
    static float last_pitch_deg = 0.0f;
    uint32_t now_ms = HAL_GetTick();
    float corner_blend = UART5_CarCornerBlend(now_ms);
    uint8_t corner_active = (corner_blend >= VISION_IMU_MIN_CORNER_BLEND) ? 1U : 0U;
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;
    float filtered_yaw;
    float filtered_pitch;

    GimbalDualPID_SetCompensation(&s_gimbal_pid,
                                  GIMBAL_COMPENSATION_FACTOR * corner_blend,
                                  corner_active);

    if (!bno080_update()) {
        return 0U;
    }

    bno080_get_euler(&roll_deg, &pitch_deg, &yaw_deg);
    (void)roll_deg;

    filtered_yaw = Vision_AngleKalmanUpdate(&s_yaw_kf, yaw_deg);
    filtered_pitch = Vision_AngleKalmanUpdate(&s_pitch_kf, pitch_deg);

    if (imu_valid && corner_active) {
        float yaw_delta = Vision_WrapAngleDeltaDeg(filtered_yaw, last_yaw_deg);
        float pitch_delta = Vision_WrapAngleDeltaDeg(filtered_pitch, last_pitch_deg);

        yaw_delta = Vision_ClampFloat(yaw_delta,
                                      -VISION_IMU_DELTA_LIMIT_DEG,
                                      VISION_IMU_DELTA_LIMIT_DEG);
        pitch_delta = Vision_ClampFloat(pitch_delta,
                                        -VISION_IMU_DELTA_LIMIT_DEG,
                                        VISION_IMU_DELTA_LIMIT_DEG);

        GimbalDualPID_SetIMUDelta(&s_gimbal_pid, yaw_delta, pitch_delta);

        last_yaw_deg = filtered_yaw;
        last_pitch_deg = filtered_pitch;
        return 1U;
    }

    last_yaw_deg = filtered_yaw;
    last_pitch_deg = filtered_pitch;
    imu_valid = 1U;
    return 0U;
}

// 判断当前模式是否为射击模式 (SHOOT 或 SHOOT_CIRCLE)
static uint8_t Vision_ModeShoots(void)
{
    return (s_eye_mode == VISION_MODE_SHOOT ||
            s_eye_mode == VISION_MODE_SHOOT_CIRCLE) ? 1U : 0U;
}

// 判断当前模式是否为圆形扫描模式 (CIRCLE 或 SHOOT_CIRCLE)
static uint8_t Vision_ModeCircles(void)
{
    return (s_eye_mode == VISION_MODE_CIRCLE ||
            s_eye_mode == VISION_MODE_SHOOT_CIRCLE) ? 1U : 0U;
}

// 计算圆形扫描偏移量: 相位累加→cos/sin生成圆轨迹→相位归零循环
static void Vision_CircleOffset(float *circle_x, float *circle_y)
{
    // 相位步进累加
    s_circle_phase += VISION_CIRCLE_STEP_RAD;
    if (s_circle_phase >= VISION_TWO_PI) {
        s_circle_phase -= VISION_TWO_PI;
    }

    // 极坐标→直角坐标: 相位控制扫描方向，半径控制扫描幅度
    *circle_x = VISION_CIRCLE_RADIUS_PX * cosf(s_circle_phase);
    *circle_y = VISION_CIRCLE_RADIUS_PX * sinf(s_circle_phase);
}

// 执行云台控制: 目标处理→PID更新→激光触发→角度记录
static void Vision_ApplyControl(float error_x, float error_y,
                                uint8_t target_seen,
                                uint8_t laser_allowed)
{
    Vision_TargetProcess((int16_t)error_x, (int16_t)error_y, true);
    if (!target_seen) {
        s_vision_ctx.target_detected = 0U;
    }
    Vision_GimbalPID_Update(s_vision_ctx.error_x, s_vision_ctx.error_y);
    Vision_LaserTrigger(laser_allowed ? true : false);
    VisionStrategy_SetGimbalAngle(s_vision_ctx.gimbal_x_angle,
                                  s_vision_ctx.gimbal_y_angle);
}

// 纯圆形扫描执行 (无目标时根据圆形扫描偏移量控制云台)
static void Vision_RunCircleOnly(void)
{
    float circle_x;
    float circle_y;

    Vision_CircleOffset(&circle_x, &circle_y);
    Vision_ApplyControl(circle_x, circle_y, 0U, 0U);
}

// 按键模式切换处理: KEY0→空闲, KEY1→圆形扫描, KEY2→追踪射击, KEY3→追踪+圆形扫描
void Vision_KeyControlTick(void)
{
    // KEY0: 切换到空闲模式，停止所有动作
    if (Key_GetState(0U) == KEY_PRESS) {
        s_eye_mode = VISION_MODE_IDLE;
        s_circle_phase = 0.0f;
        Vision_TargetProcess(0, 0, false);
        Vision_LaserTrigger(false);
        Vision_GimbalPID_ClearIntegral();
    }

    // KEY1: 切换到纯圆形扫描模式
    if (Key_GetState(1U) == KEY_PRESS) {
        s_eye_mode = VISION_MODE_CIRCLE;
        s_circle_phase = 0.0f;
        Vision_LaserTrigger(false);
        Vision_GimbalPID_ClearIntegral();
    }

    // KEY2: 切换到追踪射击模式
    if (Key_GetState(2U) == KEY_PRESS) {
        s_eye_mode = VISION_MODE_SHOOT;
        s_circle_phase = 0.0f;
        Vision_LaserTrigger(false);
        Vision_GimbalPID_ClearIntegral();
    }

    // KEY3: 切换到追踪+圆形扫描复合模式
    if (Key_GetState(3U) == KEY_PRESS) {
        s_eye_mode = VISION_MODE_SHOOT_CIRCLE;
        s_circle_phase = 0.0f;
        Vision_LaserTrigger(false);
        Vision_GimbalPID_ClearIntegral();
    }
}

static void __attribute__((unused)) Vision_DebugPrint(void)
{
    char line[192];
    uint32_t now_ms = HAL_GetTick();
    const GimbalDebugState_t *dbg = GimbalDualPID_GetDebugState();

    if ((uint32_t)(now_ms - s_debug_last_ms) < 100U) {
        return;
    }
    s_debug_last_ms = now_ms;

    snprintf(line, sizeof(line),
             "[EYE DBG] fresh=%u valid=%u raw_err=(%d,%d) err=(%.1f,%.1f) pid=(%.2f,%.2f) dir=(%u,%u) pulse=(%lu,%lu) roi=%u errc=%u\r\n",
             (unsigned)s_last_fresh_packet,
             (unsigned)k230_parsed.track_valid,
             (int)k230_parsed.err_y,
             (int)k230_parsed.err_z,
             (double)s_vision_ctx.error_x,
             (double)s_vision_ctx.error_y,
             (double)dbg->output_x_deg,
             (double)dbg->output_y_deg,
             (unsigned)dbg->dir_x,
             (unsigned)dbg->dir_y,
             (unsigned long)dbg->pulses_x,
             (unsigned long)dbg->pulses_y,
             (unsigned)s_vision_ctx.roi_state,
             (unsigned)k230_parsed.error_code);

    USART3_SendString(line);
}

void Vision_GimbalPID_Init(void)
{
    GimbalDualPID_Init(&s_gimbal_pid);
    GimbalDualPID_SetCompensation(&s_gimbal_pid,
                                  GIMBAL_COMPENSATION_FACTOR,
                                  0U);
}

void Vision_GimbalPID_Update(float error_x_px, float error_y_px)
{
    Vision_UpdateGimbalRuntimeSpeed(error_x_px, error_y_px);
    GimbalDualPID_Update(&s_gimbal_pid, error_x_px, error_y_px);
    s_vision_ctx.gimbal_x_angle = (int16_t)s_gimbal_pid.x_axis.output;
    s_vision_ctx.gimbal_y_angle = (int16_t)s_gimbal_pid.y_axis.output;
}

void Vision_GimbalPID_ClearIntegral(void)
{
    GimbalDualPID_ClearIntegral(&s_gimbal_pid);
}

void Vision_GimbalIMUCompensationTick(void)
{
    if (!s_initialized) return;
    if (s_last_fresh_packet) return;

    Vision_UpdateGimbalRuntimeSpeed(0.0f, 0.0f);
    GimbalDualPID_Update(&s_gimbal_pid, 0.0f, 0.0f);
    s_vision_ctx.gimbal_x_angle = (int16_t)s_gimbal_pid.x_axis.output;
    s_vision_ctx.gimbal_y_angle = (int16_t)s_gimbal_pid.y_axis.output;
}

GimbalDualPID_t* Vision_GimbalPID_GetController(void)
{
    return &s_gimbal_pid;
}

void Vision_TargetProcess(int16_t err_y, int16_t err_z, bool valid)
{
    if (valid) {
        s_vision_ctx.error_x = (float)err_y;
        s_vision_ctx.error_y = (float)err_z;
        s_vision_ctx.error_distance = sqrtf(s_vision_ctx.error_x * s_vision_ctx.error_x +
                                            s_vision_ctx.error_y * s_vision_ctx.error_y);
        s_vision_ctx.target_detected = 1U;
    } else {
        s_vision_ctx.error_x = 0.0f;
        s_vision_ctx.error_y = 0.0f;
        s_vision_ctx.error_distance = 1.0e6f;
        s_vision_ctx.target_detected = 0U;
    }
}

// 激光触发防抖判断: 连续命中N帧后才触发，避免误触发
uint8_t Vision_LaserTrigger(bool target_valid)
{
    // 目标无效: 立即关闭激光，清零命中计数
    if (!target_valid) {
        GPIO_ResetBits(GPIOD, GPIO_Pin_12);
        s_vision_ctx.laser_on = 0U;
        s_vision_ctx.hit_streak = 0U;
        return 0U;
    }

    // 误差在阈值内: 累加命中计数
    if (s_vision_ctx.error_distance < (float)VISION_LASER_THRESHOLD_PX) {
        if (s_vision_ctx.hit_streak < 255U) {
            s_vision_ctx.hit_streak++;
        }

        // 连续命中达到防抖帧数: 触发激光
        if (s_vision_ctx.hit_streak >= VISION_LASER_HOLD_FRAMES) {
            GPIO_SetBits(GPIOD, GPIO_Pin_12);
            s_vision_ctx.laser_on = 1U;
            return 1U;
        }
    } else {
        // 误差超出阈值: 关闭激光，清零命中计数
        GPIO_ResetBits(GPIOD, GPIO_Pin_12);
        s_vision_ctx.laser_on = 0U;
        s_vision_ctx.hit_streak = 0U;
    }

    return 0U;
}

void Vision_GetError(float *err_x, float *err_y, float *dist)
{
    if (err_x != NULL) *err_x = s_vision_ctx.error_x;
    if (err_y != NULL) *err_y = s_vision_ctx.error_y;
    if (dist  != NULL) *dist  = s_vision_ctx.error_distance;
}

void Tracking_Init(void)
{
    VisionStrategy_Init();
}

void Tracking_Update(uint8_t target_detected)
{
    VisionStrategy_Update(target_detected);
    s_vision_ctx.roi_state = VisionStrategy_GetROIState();
}

VisionStrategy_t* Tracking_GetState(void)
{
    return VisionStrategy_GetState();
}

void Vision_Init(void)
{
    K230_UART_Init(K230_BAUD);
    {
        GPIO_InitTypeDef g;
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
        g.GPIO_Pin = GPIO_Pin_12;
        g.GPIO_Mode = GPIO_Mode_OUT;
        g.GPIO_OType = GPIO_OType_PP;
        g.GPIO_Speed = GPIO_Speed_50MHz;
        g.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOD, &g);
        GPIO_ResetBits(GPIOD, GPIO_Pin_12);
    }
    Vision_GimbalPID_Init();
    Tracking_Init();

    s_vision_ctx.error_x = 0.0f;
    s_vision_ctx.error_y = 0.0f;
    s_vision_ctx.error_distance = 0.0f;
    s_vision_ctx.target_detected = 0U;
    s_vision_ctx.laser_on = 0U;
    s_vision_ctx.hit_streak = 0U;
    s_vision_ctx.gimbal_x_angle = 0;
    s_vision_ctx.gimbal_y_angle = 0;
    s_vision_ctx.roi_state = ROI_FULL;

    s_initialized = 1U;
}

// 视觉模式状态机主循环
// 四种模式: 空闲/圆形扫描/追踪射击/追踪+圆形扫描
void Vision_Process(void)
{
    uint8_t fresh_packet = 0U;
    uint8_t imu_comp_pending = 0U;

    if (!s_initialized) return;

    // 阶段1: IMU前馈补偿更新，获取车身姿态变化
    imu_comp_pending = Vision_UpdateGimbalIMUCompensation();

    // 阶段2: K230数据解析 — 接收串口数据→解析协议包→提取目标误差
    if (k230_rx_flag) {
        K230_ParsePacket();
        fresh_packet = 1U;
    }
    s_last_fresh_packet = fresh_packet;

    // 阶段3: 空闲模式 — 停止所有输出，仅保持IMU补偿
    if (s_eye_mode == VISION_MODE_IDLE) {
        s_circle_phase = 0.0f;
        Vision_TargetProcess(0, 0, false);
        Vision_LaserTrigger(false);
        Tracking_Update(0U);
    }

    // 阶段4: 收到新数据包且非空闲模式，执行核心控制逻辑
    if (fresh_packet && s_eye_mode != VISION_MODE_IDLE) {
        uint8_t target_valid = k230_parsed.track_valid ? 1U : 0U;
        float control_x = 0.0f;
        float control_y = 0.0f;
        float circle_x = 0.0f;
        float circle_y = 0.0f;

        // ROI状态机更新: 根据目标有无自动切换ROI大小
        Tracking_Update(target_valid);

        // 圆形扫描模式额外叠加圆形偏移
        if (Vision_ModeCircles()) {
            Vision_CircleOffset(&circle_x, &circle_y);
        }

        // 目标有效且射击模式: K230误差 + 圆形偏移 → PID控制 → 激光触发
        if (target_valid && Vision_ModeShoots()) {
            control_x = (float)k230_parsed.err_y;
            control_y = (float)k230_parsed.err_z;
            control_x += circle_x;
            control_y += circle_y;
            Vision_ApplyControl(control_x, control_y, 1U, 1U);
        } else if (Vision_ModeCircles()) {
            // 纯圆形扫描: 无目标，按圆形轨迹运动
            Vision_ApplyControl(circle_x, circle_y, 0U, 0U);
        } else {
            // 有数据包但无目标且非圆形模式: 停止追踪
            Vision_TargetProcess(0, 0, false);
            Vision_LaserTrigger(false);

            // 全图模式下启动蛇形扫描搜索
            if (VisionStrategy_GetROIState() == ROI_FULL) {
                VisionStrategy_GimbalScan();
            }
        }
    } else if (!fresh_packet && s_eye_mode != VISION_MODE_IDLE && Vision_ModeCircles()) {
        // 无新数据包但圆形扫描模式: 继续执行圆形扫描
        Vision_RunCircleOnly();
    }

    // 阶段5: IMU补偿数据就绪时，执行补偿控
    if (imu_comp_pending && s_eye_mode != VISION_MODE_IDLE) {
        Vision_GimbalIMUCompensationTick();
    }

//    Vision_DebugPrint();
}

Vision_Context_t* Vision_GetContext(void)
{
    return &s_vision_ctx;
}
