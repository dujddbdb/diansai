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

#define VISION_TWO_PI               6.2831853f

typedef enum {
    VISION_MODE_IDLE = 0,
    VISION_MODE_CIRCLE,
    VISION_MODE_SHOOT,
    VISION_MODE_SHOOT_CIRCLE
} Vision_Mode_t;

typedef struct {
    float angle;
    float rate;
    float p00;
    float p01;
    float p10;
    float p11;
    float p;
    uint8_t valid;
} Vision_AngleKalman_t;

static Vision_AngleKalman_t s_yaw_kf = {0};
static Vision_AngleKalman_t s_pitch_kf = {0};
static Vision_Mode_t s_eye_mode = VISION_MODE_IDLE;
static float s_circle_phase = 0.0f;

static float Vision_ClampFloat(float value, float min_v, float max_v)
{
    if (value < min_v) return min_v;
    if (value > max_v) return max_v;
    return value;
}

static float Vision_WrapAngleDeltaDeg(float now_deg, float last_deg)
{
    float delta = now_deg - last_deg;

    while (delta > 180.0f) delta -= 360.0f;
    while (delta < -180.0f) delta += 360.0f;

    return delta;
}

static float Vision_AngleKalmanUpdate(Vision_AngleKalman_t *kf, float measurement_deg)
{
    float k;

    /* 首次测量: 直接初始化 */
    if (!kf->valid) {
        kf->angle = measurement_deg;
        kf->p = 1.0f;
        kf->valid = 1U;
        return measurement_deg;
    }

    /* 预测阶段: 角度环绕处理 + 过程噪声累加 */
    measurement_deg = kf->angle + Vision_WrapAngleDeltaDeg(measurement_deg, kf->angle);
    kf->p += VISION_IMU_KALMAN_Q;

    /* 更新阶段: 计算卡尔曼增益 + 更新状态 */
    k = kf->p / (kf->p + VISION_IMU_KALMAN_R);
    kf->angle += k * Vision_WrapAngleDeltaDeg(measurement_deg, kf->angle);
    kf->p *= (1.0f - k);

    return kf->angle;
}

static void Vision_AngleKalmanStep(Vision_AngleKalman_t *kf,
                                   float measurement_deg,
                                   uint8_t has_measurement,
                                   float dt_s)
{
    float p00, p01, p10, p11;
    float innovation, s, k0, k1;

    /* 未初始化: 有测量值则初始化 */
    if (!kf->valid) {
        if (!has_measurement) return;
        kf->angle = measurement_deg;
        kf->rate = 0.0f;
        kf->p00 = 1.0f;
        kf->p01 = 0.0f;
        kf->p10 = 0.0f;
        kf->p11 = 1.0f;
        kf->p = 1.0f;
        kf->valid = 1U;
        return;
    }

    /* 预测阶段: 角度积分 + 协方差矩阵更新 */
    kf->angle += kf->rate * dt_s;

    p00 = kf->p00 + dt_s * (kf->p10 + kf->p01) +
          dt_s * dt_s * kf->p11 + VISION_IMU_KALMAN_Q;
    p01 = kf->p01 + dt_s * kf->p11;
    p10 = kf->p10 + dt_s * kf->p11;
    p11 = kf->p11 + VISION_IMU_KALMAN_RATE_Q;

    kf->p00 = p00;
    kf->p01 = p01;
    kf->p10 = p10;
    kf->p11 = p11;

    /* 无测量值: 仅预测，跳过更新 */
    if (!has_measurement) return;

    /* 更新阶段: 新息计算 + 卡尔曼增益 + 状态更新 */
    measurement_deg = kf->angle + Vision_WrapAngleDeltaDeg(measurement_deg, kf->angle);
    innovation = Vision_WrapAngleDeltaDeg(measurement_deg, kf->angle);
    s = kf->p00 + VISION_IMU_KALMAN_R;
    if (s <= 1.0e-6f) return;

    k0 = kf->p00 / s;
    k1 = kf->p10 / s;
    p00 = kf->p00;
    p01 = kf->p01;

    kf->angle += k0 * innovation;
    kf->rate += k1 * innovation;

    kf->p00 = (1.0f - k0) * p00;
    kf->p01 = (1.0f - k0) * p01;
    kf->p10 = kf->p10 - k1 * p00;
    kf->p11 = kf->p11 - k1 * p01;
    kf->p = kf->p00;
}

static float Vision_ErrorBlend(float abs_error_px)
{
    float t;

    /* 误差小于起始值: 全用小增益 */
    if (abs_error_px <= PID_ERROR_BLEND_START) return 0.0f;
    /* 误差大于结束值: 全用大增益 */
    if (abs_error_px >= PID_ERROR_BLEND_END) return 1.0f;

    /* 中间区间: smoothstep平滑过渡 */
    t = (abs_error_px - PID_ERROR_BLEND_START) /
        (PID_ERROR_BLEND_END - PID_ERROR_BLEND_START);
    return t * t * (3.0f - 2.0f * t);
}

static uint16_t Vision_UpdateAxisRpm(GimbalPID_t *axis, float abs_error_px, float corner_blend)
{
    float blend = Vision_ErrorBlend(abs_error_px);
    float target = VISION_GIMBAL_RPM_MIN +
                   (VISION_GIMBAL_RPM_MAX - VISION_GIMBAL_RPM_MIN) * blend;
    float current = (float)axis->rpm;

    /* 转弯时按比例降速 */
    target *= (1.0f - VISION_CORNER_RPM_REDUCE * corner_blend);
    target = Vision_ClampFloat(target, VISION_GIMBAL_RPM_MIN, VISION_GIMBAL_RPM_MAX);

    /* 转速指数平滑过渡 */
    if (current < 1.0f) current = VISION_GIMBAL_RPM_MIN;
    current += (target - current) * VISION_GIMBAL_RPM_SLEW;
    axis->rpm = (uint16_t)(current + 0.5f);

    return axis->rpm;
}

static void Vision_UpdateGimbalRuntimeSpeed(float error_x_px, float error_y_px)
{
    float corner_blend = UART5_CarCornerBlend(HAL_GetTick());

    Vision_UpdateAxisRpm(&s_gimbal_pid.x_axis, fabsf(error_x_px), corner_blend);
    Vision_UpdateAxisRpm(&s_gimbal_pid.y_axis, fabsf(error_y_px), corner_blend);
}

static uint8_t Vision_UpdateGimbalIMUCompensation(void)
{
    static uint8_t imu_valid = 0U;
    static float last_yaw_deg = 0.0f;
    static float last_pitch_deg = 0.0f;
    static uint32_t last_kalman_ms = 0U;
    uint32_t now_ms = HAL_GetTick();
    uint32_t dt_ms;
    float dt_s;
    float corner_blend = UART5_CarCornerBlend(now_ms);
    uint8_t corner_active = (corner_blend >= VISION_IMU_MIN_CORNER_BLEND) ? 1U : 0U;
    uint8_t has_measurement;
    float roll_deg = 0.0f, pitch_deg = 0.0f, yaw_deg = 0.0f;
    float filtered_yaw, filtered_pitch;

    /* 设置补偿系数，转弯时启用 */
    GimbalDualPID_SetCompensation(&s_gimbal_pid,
                                  GIMBAL_COMPENSATION_FACTOR * corner_blend,
                                  corner_active);

    /* 读取BNO080 IMU数据 */
    has_measurement = bno080_update();
    if (!has_measurement && !s_yaw_kf.valid) {
        return 0U;
    }

    /* 计算时间间隔dt并限幅 */
    if (last_kalman_ms == 0U) {
        dt_ms = 10U;
    } else {
        dt_ms = (uint32_t)(now_ms - last_kalman_ms);
    }
    last_kalman_ms = now_ms;
    if (dt_ms < VISION_IMU_DT_MIN_MS) dt_ms = VISION_IMU_DT_MIN_MS;
    if (dt_ms > VISION_IMU_DT_MAX_MS) dt_ms = VISION_IMU_DT_MAX_MS;
    dt_s = (float)dt_ms * 0.001f;

    /* 有测量值则读取欧拉角 */
    if (has_measurement) {
        bno080_get_euler(&roll_deg, &pitch_deg, &yaw_deg);
        (void)roll_deg;
    }

    /* 二阶卡尔曼滤波更新偏航角和俯仰角 */
    Vision_AngleKalmanStep(&s_yaw_kf, yaw_deg, has_measurement, dt_s);
    Vision_AngleKalmanStep(&s_pitch_kf, pitch_deg, has_measurement, dt_s);
    if (!s_yaw_kf.valid || !s_pitch_kf.valid) {
        return 0U;
    }

    filtered_yaw = s_yaw_kf.angle;
    filtered_pitch = s_pitch_kf.angle;

    /* IMU有效且转弯时，计算补偿量 */
    if (imu_valid && corner_active) {
        float yaw_delta = Vision_WrapAngleDeltaDeg(filtered_yaw, last_yaw_deg);
        float pitch_delta = Vision_WrapAngleDeltaDeg(filtered_pitch, last_pitch_deg);

        yaw_delta = Vision_ClampFloat(yaw_delta,
                                      -VISION_IMU_DELTA_LIMIT_DEG,
                                      VISION_IMU_DELTA_LIMIT_DEG);
        pitch_delta = Vision_ClampFloat(pitch_delta,
                                        -VISION_IMU_DELTA_LIMIT_DEG,
                                        VISION_IMU_DELTA_LIMIT_DEG);

        /* 角速度接近0: 静态角度差补偿; 否则: 角度差+角速度前馈 */
        if ((fabsf(s_yaw_kf.rate) < 0.001f) &&
            (fabsf(s_pitch_kf.rate) < 0.001f)) {
            GimbalDualPID_SetIMUDelta(&s_gimbal_pid, yaw_delta, pitch_delta);
        } else {
            GimbalDualPID_SetIMUFeedforward(&s_gimbal_pid,
                                           yaw_delta,
                                           pitch_delta,
                                           s_yaw_kf.rate,
                                           s_pitch_kf.rate,
                                           dt_s);
        }

        last_yaw_deg = filtered_yaw;
        last_pitch_deg = filtered_pitch;
        return 1U;
    }

    /* 首次有效或非转弯: 仅更新历史值 */
    last_yaw_deg = filtered_yaw;
    last_pitch_deg = filtered_pitch;
    imu_valid = 1U;
    return 0U;
}

static uint8_t Vision_ModeShoots(void)
{
    return (s_eye_mode == VISION_MODE_SHOOT ||
            s_eye_mode == VISION_MODE_SHOOT_CIRCLE) ? 1U : 0U;
}

static uint8_t Vision_ModeCircles(void)
{
    return (s_eye_mode == VISION_MODE_CIRCLE ||
            s_eye_mode == VISION_MODE_SHOOT_CIRCLE) ? 1U : 0U;
}

static void Vision_CircleOffset(float *circle_x, float *circle_y)
{
    /* 相位累加，超过2π则归零 */
    s_circle_phase += VISION_CIRCLE_STEP_RAD;
    if (s_circle_phase >= VISION_TWO_PI) {
        s_circle_phase -= VISION_TWO_PI;
    }

    /* 极坐标转直角坐标 */
    *circle_x = VISION_CIRCLE_RADIUS_PX * cosf(s_circle_phase);
    *circle_y = VISION_CIRCLE_RADIUS_PX * sinf(s_circle_phase);
}

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

static void Vision_RunCircleOnly(void)
{
    float circle_x, circle_y;

    Vision_CircleOffset(&circle_x, &circle_y);
    Vision_ApplyControl(circle_x, circle_y, 0U, 0U);
}

void Vision_KeyControlTick(void)
{
    /* KEY0: 空闲模式 */
    if (Key_GetState(0U) == KEY_PRESS) {
        s_eye_mode = VISION_MODE_IDLE;
        s_circle_phase = 0.0f;
        Vision_TargetProcess(0, 0, false);
        Vision_LaserTrigger(false);
        Vision_GimbalPID_ClearIntegral();
    }

    /* KEY1: 纯圆形扫描 */
    if (Key_GetState(1U) == KEY_PRESS) {
        s_eye_mode = VISION_MODE_CIRCLE;
        s_circle_phase = 0.0f;
        Vision_LaserTrigger(false);
        Vision_GimbalPID_ClearIntegral();
    }

    /* KEY2: 追踪射击 */
    if (Key_GetState(2U) == KEY_PRESS) {
        s_eye_mode = VISION_MODE_SHOOT;
        s_circle_phase = 0.0f;
        Vision_LaserTrigger(false);
        Vision_GimbalPID_ClearIntegral();
    }

    /* KEY3: 追踪+圆形扫描 */
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

    /* 限流: 每100ms打印一次 */
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

uint8_t Vision_LaserTrigger(bool target_valid)
{
    /* 目标无效: 立即关激光，清计数 */
    if (!target_valid) {
        Relay_Off();
        s_vision_ctx.laser_on = 0U;
        s_vision_ctx.hit_streak = 0U;
        return 0U;
    }

    /* 误差在阈值内: 累加命中计数 */
    if (s_vision_ctx.error_distance < (float)VISION_LASER_THRESHOLD_PX) {
        if (s_vision_ctx.hit_streak < 255U) {
            s_vision_ctx.hit_streak++;
        }

        /* 连续命中达到防抖帧数: 触发激光 */
        if (s_vision_ctx.hit_streak >= VISION_LASER_HOLD_FRAMES) {
            Relay_On();
            s_vision_ctx.laser_on = 1U;
            return 1U;
        }
    } else {
        /* 误差超阈值: 关激光，清计数 */
        Relay_Off();
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
    Relay_Init();
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

void Vision_Process(void)
{
    uint8_t fresh_packet = 0U;
    uint8_t imu_comp_pending = 0U;

    if (!s_initialized) return;

    /* IMU前馈补偿更新 */
    imu_comp_pending = Vision_UpdateGimbalIMUCompensation();

    /* K230数据解析 */
    if (k230_rx_flag) {
        K230_ParsePacket();
        fresh_packet = 1U;
    }
    s_last_fresh_packet = fresh_packet;

    /* 空闲模式: 停止输出，保持IMU补偿 */
    if (s_eye_mode == VISION_MODE_IDLE) {
        s_circle_phase = 0.0f;
        Vision_TargetProcess(0, 0, false);
        Vision_LaserTrigger(false);
        Tracking_Update(0U);
    }

    /* 收到新数据包且非空闲模式，执行核心控制 */
    if (fresh_packet && s_eye_mode != VISION_MODE_IDLE) {
        uint8_t target_valid = k230_parsed.track_valid ? 1U : 0U;
        float control_x = 0.0f, control_y = 0.0f;
        float circle_x = 0.0f, circle_y = 0.0f;

        /* ROI状态机更新 */
        Tracking_Update(target_valid);

        /* 圆形扫描模式叠加圆形偏移 */
        if (Vision_ModeCircles()) {
            Vision_CircleOffset(&circle_x, &circle_y);
        }

        if (target_valid && Vision_ModeShoots()) {
            /* 目标有效且射击模式: K230误差 + 圆形偏移 → PID控制 + 激光 */
            control_x = (float)k230_parsed.err_y;
            control_y = (float)k230_parsed.err_z;
            control_x += circle_x;
            control_y += circle_y;
            Vision_ApplyControl(control_x, control_y, 1U, 1U);
        } else if (Vision_ModeCircles()) {
            /* 纯圆形扫描: 无目标，按圆轨迹运动 */
            Vision_ApplyControl(circle_x, circle_y, 0U, 0U);
        } else {
            /* 有数据包但无目标且非圆形模式: 停止追踪 */
            Vision_TargetProcess(0, 0, false);
            Vision_LaserTrigger(false);

            /* 全图模式下启动蛇形扫描 */
            if (VisionStrategy_GetROIState() == ROI_FULL) {
                VisionStrategy_GimbalScan();
            }
        }
    } else if (!fresh_packet && s_eye_mode != VISION_MODE_IDLE && Vision_ModeCircles()) {
        /* 无新数据包但圆形扫描模式: 继续圆形扫描 */
        Vision_RunCircleOnly();
    }

    /* IMU补偿就绪时执行补偿控制(丢包维持) */
    if (imu_comp_pending && s_eye_mode != VISION_MODE_IDLE) {
        Vision_GimbalIMUCompensationTick();
    }

//    Vision_DebugPrint();
}

Vision_Context_t* Vision_GetContext(void)
{
    return &s_vision_ctx;
}
