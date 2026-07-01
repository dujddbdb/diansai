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

// 云台双轴PID控制器实例
static GimbalDualPID_t  s_gimbal_pid;
// 视觉上下文状态结构体
static Vision_Context_t  s_vision_ctx;
// 系统初始化标志位
static uint8_t          s_initialized = 0U;
// 调试打印上次时间戳
static uint32_t         __attribute__((unused)) s_debug_last_ms = 0U;
// 上次是否接收到新数据包标志
static uint8_t          s_last_fresh_packet = 0U;

// 2π常量, 用于圆形扫描相位计算
#define VISION_TWO_PI               6.2831853f

// 视觉工作模式枚举
typedef enum {
    VISION_MODE_IDLE = 0,       // 0-空闲模式
    VISION_MODE_CIRCLE,         // 1-圆形扫描模式
    VISION_MODE_SHOOT,          // 2-追踪射击模式
    VISION_MODE_SHOOT_CIRCLE    // 3-追踪+圆形扫描复合模式
} Vision_Mode_t;

// IMU角度一阶卡尔曼滤波器状态结构体
typedef struct {
    float angle;    // 估计角度
    float rate;     // 估计角速度
    float bias;     // 估计偏差
    float p00;      // 协方差矩阵元素 [0,0]
    float p01;      // 协方差矩阵元素 [0,1]
    float p02;      // 协方差矩阵元素 [0,2]
    float p10;      // 协方差矩阵元素 [1,0]
    float p11;      // 协方差矩阵元素 [1,1]
    float p12;      // 协方差矩阵元素 [1,2]
    float p20;      // 协方差矩阵元素 [2,0]
    float p21;      // 协方差矩阵元素 [2,1]
    float p22;      // 协方差矩阵元素 [2,2]
    float p;        // 简化协方差值
    uint8_t valid;  // 滤波器是否有效
} Vision_AngleKalman_t;

// Yaw轴卡尔曼滤波器实例
static Vision_AngleKalman_t s_yaw_kf = {0};
// Pitch轴卡尔曼滤波器实例
static Vision_AngleKalman_t s_pitch_kf = {0};
// 当前视觉工作模式
static Vision_Mode_t s_eye_mode = VISION_MODE_IDLE;
// 圆形扫描当前相位
static float s_circle_phase = 0.0f;

// 浮点数区间钳位函数: 将value限制在[min_v, max_v]范围内
static float Vision_ClampFloat(float value, float min_v, float max_v)
{
    if (value < min_v) return min_v;
    if (value > max_v) return max_v;
    return value;
}

// 角度差环绕处理函数: 将角度差规约到[-180°, +180°]区间
static float Vision_WrapAngleDeltaDeg(float now_deg, float last_deg)
{
    float delta = now_deg - last_deg;

    // 角度大于180度，减去360度
    while (delta > 180.0f) delta -= 360.0f;
    // 角度小于-180度，加上360度
    while (delta < -180.0f) delta += 360.0f;

    return delta;
}

// IMU角度卡尔曼滤波更新函数(简化版)
static float Vision_AngleKalmanUpdate(Vision_AngleKalman_t *kf, float measurement_deg)
{
    float k;

    // 首次测量直接初始化滤波器状态
    if (!kf->valid) {
        kf->angle = measurement_deg;
        kf->p = 1.0f;
        kf->valid = 1U;
        return measurement_deg;
    }

    // 预测阶段: 测量值绕回处理后，协方差P += 过程噪声Q
    measurement_deg = kf->angle + Vision_WrapAngleDeltaDeg(measurement_deg, kf->angle);
    kf->p += VISION_IMU_KALMAN_Q;
    // 更新阶段: 计算卡尔曼增益 K = P / (P + R)
    k = kf->p / (kf->p + VISION_IMU_KALMAN_R);
    // 更新阶段: 估计值 = 预测值 + K * (测量值 - 预测值)
    kf->angle += k * Vision_WrapAngleDeltaDeg(measurement_deg, kf->angle);
    // 更新阶段: 协方差更新 P = (1 - K) * P
    kf->p *= (1.0f - k);

    return kf->angle;
}

// IMU角度卡尔曼滤波步进函数(完整版，三阶状态)
static void Vision_AngleKalmanStep(Vision_AngleKalman_t *kf,
                                   float measurement_deg,
                                   uint8_t has_measurement,
                                   float dt_s)
{
    float p00;
    float p01;
    float p02;
    float p10;
    float p11;
    float p12;
    float p20;
    float p21;
    float p22;
    float innovation;
    float s;
    float k0;
    float k1;
    float k2;
    float predicted_measurement;
    float rate_decay = VISION_IMU_KALMAN_RATE_DECAY;

    // 滤波器未初始化且有测量值时，初始化状态
    if (!kf->valid) {
        if (!has_measurement) return;
        kf->angle = measurement_deg;
        kf->rate = 0.0f;
        kf->bias = 0.0f;
        // 初始化协方差矩阵为单位矩阵
        kf->p00 = 1.0f;
        kf->p01 = 0.0f;
        kf->p02 = 0.0f;
        kf->p10 = 0.0f;
        kf->p11 = 1.0f;
        kf->p12 = 0.0f;
        kf->p20 = 0.0f;
        kf->p21 = 0.0f;
        kf->p22 = 1.0f;
        kf->p = 1.0f;
        kf->valid = 1U;
        return;
    }

    // 角速度衰减系数钳位到[0, 1]范围
    if (rate_decay < 0.0f) rate_decay = 0.0f;
    if (rate_decay > 1.0f) rate_decay = 1.0f;

    // 预测阶段: 角度积分更新
    kf->angle += kf->rate * dt_s;
    // 预测阶段: 角速度衰减
    kf->rate *= rate_decay;

    // 预测阶段: 协方差矩阵时间更新
    p00 = kf->p00 + dt_s * (kf->p10 + kf->p01) +
          dt_s * dt_s * kf->p11 + VISION_IMU_KALMAN_ANGLE_Q;
    p01 = rate_decay * (kf->p01 + dt_s * kf->p11);
    p02 = kf->p02 + dt_s * kf->p12;
    p10 = rate_decay * (kf->p10 + dt_s * kf->p11);
    p11 = rate_decay * rate_decay * kf->p11 + VISION_IMU_KALMAN_RATE_Q;
    p12 = rate_decay * kf->p12;
    p20 = kf->p20 + dt_s * kf->p21;
    p21 = rate_decay * kf->p21;
    p22 = kf->p22 + VISION_IMU_KALMAN_BIAS_Q;

    // 保存更新后的协方差矩阵
    kf->p00 = p00;
    kf->p01 = p01;
    kf->p02 = p02;
    kf->p10 = p10;
    kf->p11 = p11;
    kf->p12 = p12;
    kf->p20 = p20;
    kf->p21 = p21;
    kf->p22 = p22;

    // 无测量值时，仅完成预测阶段，跳过更新阶段
    if (!has_measurement) return;

    // 更新阶段: 计算预测测量值
    predicted_measurement = kf->angle + kf->bias;
    // 更新阶段: 测量值角度环绕处理
    measurement_deg = predicted_measurement +
                      Vision_WrapAngleDeltaDeg(measurement_deg, predicted_measurement);
    // 更新阶段: 计算创新值(测量值与预测值的差)
    innovation = Vision_WrapAngleDeltaDeg(measurement_deg, predicted_measurement);
    // 更新阶段: 计算创新协方差 S
    s = kf->p00 + kf->p02 + kf->p20 + kf->p22 +
        VISION_IMU_KALMAN_MEAS_R;
    // 创新协方差过小，跳过更新避免除零
    if (s <= 1.0e-6f) return;

    // 更新阶段: 计算卡尔曼增益 K = P * H^T / S
    k0 = (kf->p00 + kf->p02) / s;
    k1 = (kf->p10 + kf->p12) / s;
    k2 = (kf->p20 + kf->p22) / s;
    // 保存当前协方差矩阵，用于后续更新
    p00 = kf->p00;
    p01 = kf->p01;
    p02 = kf->p02;
    p10 = kf->p10;
    p11 = kf->p11;
    p12 = kf->p12;
    p20 = kf->p20;
    p21 = kf->p21;
    p22 = kf->p22;

    // 更新阶段: 状态后验估计 = 先验估计 + K * 创新值
    kf->angle += k0 * innovation;
    kf->rate += k1 * innovation;
    kf->bias += k2 * innovation;

    // 更新阶段: 角速度和偏差钳位，防止溢出
    kf->rate = Vision_ClampFloat(kf->rate,
                                 -VISION_IMU_RATE_LIMIT_DPS,
                                 VISION_IMU_RATE_LIMIT_DPS);
    kf->bias = Vision_ClampFloat(kf->bias,
                                 -VISION_IMU_BIAS_LIMIT_DEG,
                                 VISION_IMU_BIAS_LIMIT_DEG);

    // 更新阶段: 协方差矩阵后验更新 P = (I - K*H) * P
    kf->p00 = p00 - k0 * (p00 + p20);
    kf->p01 = p01 - k0 * (p01 + p21);
    kf->p02 = (1.0f - k0) * p02 - k0 * p22;
    kf->p10 = p10 - k1 * (p00 + p20);
    kf->p11 = p11 - k1 * (p01 + p21);
    kf->p12 = p12 - k1 * (p02 + p22);
    kf->p20 = p20 - k2 * (p00 + p20);
    kf->p21 = p21 - k2 * (p01 + p21);
    kf->p22 = p22 - k2 * (p02 + p22);
    // 保存主协方差值
    kf->p = kf->p00;
}

// 误差混合因子计算函数: 平滑过渡小误差→大误差区间的PID参数
static float Vision_ErrorBlend(float abs_error_px)
{
    float t;

    // 误差小于起始阈值，混合因子为0
    if (abs_error_px <= PID_ERROR_BLEND_START) return 0.0f;
    // 误差大于结束阈值，混合因子为1
    if (abs_error_px >= PID_ERROR_BLEND_END) return 1.0f;

    // 误差在中间区间，使用smoothstep函数平滑过渡
    t = (abs_error_px - PID_ERROR_BLEND_START) /
        (PID_ERROR_BLEND_END - PID_ERROR_BLEND_START);
    return t * t * (3.0f - 2.0f * t);
}

// 根据误差大小动态调整单轴电机转速
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

// 更新双轴云台运行时速度(根据误差大小自适应调速)
static void Vision_UpdateGimbalRuntimeSpeed(float error_x_px, float error_y_px)
{
    // 获取车辆转弯混合因子
    float corner_blend = UART5_CarCornerBlend(HAL_GetTick());

    // 分别更新X轴和Y轴的转速
    Vision_UpdateAxisRpm(&s_gimbal_pid.x_axis, fabsf(error_x_px), corner_blend);
    Vision_UpdateAxisRpm(&s_gimbal_pid.y_axis, fabsf(error_y_px), corner_blend);
}

// 读取最新的IMU数据帧，获取欧拉角
static uint8_t Vision_ReadLatestIMUFrame(float *roll_deg,
                                         float *pitch_deg,
                                         float *yaw_deg)
{
    uint8_t i;
    uint8_t has_measurement = 0U;

    // 循环读取最多VISION_IMU_DRAIN_MAX_PACKETS个IMU数据包
    for (i = 0U; i < VISION_IMU_DRAIN_MAX_PACKETS; i++) {
        if (!bno080_update()) break;
        // 有有效数据时读取欧拉角
        if (bno080_data_available()) {
            bno080_get_euler(roll_deg, pitch_deg, yaw_deg);
            has_measurement = 1U;
        }
    }

    // 如果循环中没有读到数据，但有数据可用，就读一次
    if (!has_measurement && bno080_data_available()) {
        bno080_get_euler(roll_deg, pitch_deg, yaw_deg);
        has_measurement = 1U;
    }

    return has_measurement;
}

// 更新云台IMU补偿
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
    uint8_t compensation_enabled;
    uint8_t has_measurement;
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;
    float filtered_yaw;
    float filtered_pitch;

    // 距离上次补偿时间不足，跳过本次更新
    if (last_kalman_ms != 0U &&
        (uint32_t)(now_ms - last_kalman_ms) < VISION_IMU_FEEDFORWARD_PERIOD_MS) {
        return 0U;
    }

    // 补偿使能判断: 始终开启模式或转弯时开启
    if (VISION_IMU_FEEDFORWARD_ALWAYS_ON) {
        compensation_enabled = 1U;
        GimbalDualPID_SetCompensation(&s_gimbal_pid,
                                      GIMBAL_COMPENSATION_FACTOR,
                                      compensation_enabled);
    } else {
        // 根据转弯程度决定是否开启补偿
        compensation_enabled =
            (corner_blend >= VISION_IMU_MIN_CORNER_BLEND) ? 1U : 0U;
        GimbalDualPID_SetCompensation(&s_gimbal_pid,
                                      GIMBAL_COMPENSATION_FACTOR * corner_blend,
                                      compensation_enabled);
    }

    // 读取最新IMU数据
    has_measurement = Vision_ReadLatestIMUFrame(&roll_deg, &pitch_deg, &yaw_deg);
    // 无测量值且滤波器未初始化，直接返回
    if (!has_measurement && !s_yaw_kf.valid) {
        return 0U;
    }

    // 计算时间差dt
    if (last_kalman_ms == 0U) {
        dt_ms = VISION_IMU_FEEDFORWARD_PERIOD_MS;
    } else {
        dt_ms = (uint32_t)(now_ms - last_kalman_ms);
    }
    last_kalman_ms = now_ms;
    // 时间差钳位到合理范围
    if (dt_ms < VISION_IMU_DT_MIN_MS) dt_ms = VISION_IMU_DT_MIN_MS;
    if (dt_ms > VISION_IMU_DT_MAX_MS) dt_ms = VISION_IMU_DT_MAX_MS;
    dt_s = (float)dt_ms * 0.001f;

    // roll轴暂不使用
    (void)roll_deg;

    // Yaw轴和Pitch轴卡尔曼滤波更新
    Vision_AngleKalmanStep(&s_yaw_kf, yaw_deg, has_measurement, dt_s);
    Vision_AngleKalmanStep(&s_pitch_kf, pitch_deg, has_measurement, dt_s);
    // 滤波器无效，直接返回
    if (!s_yaw_kf.valid || !s_pitch_kf.valid) {
        return 0U;
    }

    // 获取滤波后的角度
    filtered_yaw = s_yaw_kf.angle;
    filtered_pitch = s_pitch_kf.angle;

    // IMU有效时计算角度变化并发送补偿
    if (imu_valid) {
        // 计算角度变化量(环绕处理)
        float yaw_delta = Vision_WrapAngleDeltaDeg(filtered_yaw, last_yaw_deg);
        float pitch_delta = Vision_WrapAngleDeltaDeg(filtered_pitch, last_pitch_deg);

        // 角度变化量钳位，防止异常跳变
        yaw_delta = Vision_ClampFloat(yaw_delta,
                                      -VISION_IMU_DELTA_LIMIT_DEG,
                                      VISION_IMU_DELTA_LIMIT_DEG);
        pitch_delta = Vision_ClampFloat(pitch_delta,
                                        -VISION_IMU_DELTA_LIMIT_DEG,
                                        VISION_IMU_DELTA_LIMIT_DEG);

        // 保存当前角度供下次计算
        last_yaw_deg = filtered_yaw;
        last_pitch_deg = filtered_pitch;
        // 空闲模式或补偿未使能，不发送补偿
        if (s_eye_mode == VISION_MODE_IDLE || !compensation_enabled) {
            return 0U;
        }

        // 角速度很小时，使用位置补偿模式
        if ((fabsf(s_yaw_kf.rate) < VISION_IMU_RATE_EPS_DPS) &&
            (fabsf(s_pitch_kf.rate) < VISION_IMU_RATE_EPS_DPS)) {
            GimbalDualPID_SetIMUDelta(&s_gimbal_pid, yaw_delta, pitch_delta);
        } else {
            // 角速度较大时，使用前馈补偿模式
            GimbalDualPID_SetIMUFeedforward(&s_gimbal_pid,
                                           yaw_delta,
                                           pitch_delta,
                                           s_yaw_kf.rate,
                                           s_pitch_kf.rate,
                                           dt_s);
        }

        return 1U;
    }

    // 首次有效数据，仅保存基线角度
    last_yaw_deg = filtered_yaw;
    last_pitch_deg = filtered_pitch;
    imu_valid = 1U;
    return 0U;
}

// 判断当前模式是否为射击模式(SHOOT 或 SHOOT_CIRCLE)
static uint8_t Vision_ModeShoots(void)
{
    return (s_eye_mode == VISION_MODE_SHOOT ||
            s_eye_mode == VISION_MODE_SHOOT_CIRCLE) ? 1U : 0U;
}

// 判断当前模式是否为圆形扫描模式(CIRCLE 或 SHOOT_CIRCLE)
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
    // 相位超过2π，减去2π实现循环
    if (s_circle_phase >= VISION_TWO_PI) {
        s_circle_phase -= VISION_TWO_PI;
    }

    // 极坐标转直角坐标: 相位控制扫描方向，半径控制扫描幅度
    *circle_x = VISION_CIRCLE_RADIUS_PX * cosf(s_circle_phase);
    *circle_y = VISION_CIRCLE_RADIUS_PX * sinf(s_circle_phase);
}

// 执行云台控制: 目标处理→PID更新→激光触发→角度记录
static void Vision_ApplyControl(float error_x, float error_y,
                                uint8_t target_seen,
                                uint8_t laser_allowed)
{
    // 目标处理，更新视觉上下文误差
    Vision_TargetProcess((int16_t)error_x, (int16_t)error_y, true);
    // 目标不可见时清除检测标志
    if (!target_seen) {
        s_vision_ctx.target_detected = 0U;
    }
    // 云台PID计算更新
    Vision_GimbalPID_Update(s_vision_ctx.error_x, s_vision_ctx.error_y);
    // 激光触发判断
    Vision_LaserTrigger(laser_allowed ? true : false);
    // 记录云台角度到策略模块
    VisionStrategy_SetGimbalAngle(s_vision_ctx.gimbal_x_angle,
                                  s_vision_ctx.gimbal_y_angle);
}

// 纯圆形扫描执行(无目标时根据圆形扫描偏移量控制云台)
static void Vision_RunCircleOnly(void)
{
    float circle_x;
    float circle_y;

    // 计算圆形扫描偏移量
    Vision_CircleOffset(&circle_x, &circle_y);
    // 应用圆形偏移控制云台，无目标，不触发激光
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

// 调试打印函数
static void __attribute__((unused)) Vision_DebugPrint(void)
{
    char line[192];
    uint32_t now_ms = HAL_GetTick();
    const GimbalDebugState_t *dbg = GimbalDualPID_GetDebugState();

    // 限制打印频率，最多每100ms打印一次
    if ((uint32_t)(now_ms - s_debug_last_ms) < 100U) {
        return;
    }
    s_debug_last_ms = now_ms;

    // 格式化调试信息
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

    // 通过串口3发送调试信息
    USART3_SendString(line);
}

// 云台PID初始化
void Vision_GimbalPID_Init(void)
{
    GimbalDualPID_Init(&s_gimbal_pid);
    GimbalDualPID_SetCompensation(&s_gimbal_pid,
                                  GIMBAL_COMPENSATION_FACTOR,
                                  0U);
}

// 云台PID更新
void Vision_GimbalPID_Update(float error_x_px, float error_y_px)
{
    // 先根据误差大小更新电机转速
    Vision_UpdateGimbalRuntimeSpeed(error_x_px, error_y_px);
    // 执行双轴PID计算
    GimbalDualPID_Update(&s_gimbal_pid, error_x_px, error_y_px);
    // 保存PID输出到上下文
    s_vision_ctx.gimbal_x_angle = (int16_t)s_gimbal_pid.x_axis.output;
    s_vision_ctx.gimbal_y_angle = (int16_t)s_gimbal_pid.y_axis.output;
}

// 清除云台PID积分项
void Vision_GimbalPID_ClearIntegral(void)
{
    GimbalDualPID_ClearIntegral(&s_gimbal_pid);
}

// IMU前馈补偿1kHz定时调用
void Vision_IMUFeedforward1kTick(void)
{
    uint8_t imu_comp_pending;

    // 未初始化直接返回
    if (!s_initialized) return;

    // 更新IMU补偿数据
    imu_comp_pending = Vision_UpdateGimbalIMUCompensation();
    // 补偿数据就绪且非空闲模式，执行补偿输出
    if (imu_comp_pending && s_eye_mode != VISION_MODE_IDLE) {
        GimbalDualPID_UpdateFeedforward(&s_gimbal_pid);
    }
}

// 云台IMU补偿定时调用入口
void Vision_GimbalIMUCompensationTick(void)
{
    Vision_IMUFeedforward1kTick();
}

// 获取云台PID控制器指针
GimbalDualPID_t* Vision_GimbalPID_GetController(void)
{
    return &s_gimbal_pid;
}

// 目标处理函数: 更新视觉上下文中的误差和检测状态
void Vision_TargetProcess(int16_t err_y, int16_t err_z, bool valid)
{
    if (valid) {
        // 目标有效: 保存误差值，计算欧氏距离
        s_vision_ctx.error_x = (float)err_y;
        s_vision_ctx.error_y = (float)err_z;
        s_vision_ctx.error_distance = sqrtf(s_vision_ctx.error_x * s_vision_ctx.error_x +
                                            s_vision_ctx.error_y * s_vision_ctx.error_y);
        s_vision_ctx.target_detected = 1U;
    } else {
        // 目标无效: 清除误差，距离设为很大的值
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

// 获取当前误差值
void Vision_GetError(float *err_x, float *err_y, float *dist)
{
    if (err_x != NULL) *err_x = s_vision_ctx.error_x;
    if (err_y != NULL) *err_y = s_vision_ctx.error_y;
    if (dist  != NULL) *dist  = s_vision_ctx.error_distance;
}

// 追踪系统初始化
void Tracking_Init(void)
{
    VisionStrategy_Init();
}

// 追踪系统更新
void Tracking_Update(uint8_t target_detected)
{
    VisionStrategy_Update(target_detected);
    s_vision_ctx.roi_state = VisionStrategy_GetROIState();
}

// 获取追踪状态指针
VisionStrategy_t* Tracking_GetState(void)
{
    return VisionStrategy_GetState();
}

// 视觉系统初始化
void Vision_Init(void)
{
    // 初始化K230视觉模块串口通信
    K230_UART_Init(K230_BAUD);
    {
        GPIO_InitTypeDef g;
        // 使能GPIOD时钟
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
        // 配置PD12为激光控制引脚
        g.GPIO_Pin = GPIO_Pin_12;
        g.GPIO_Mode = GPIO_Mode_OUT;
        g.GPIO_OType = GPIO_OType_PP;
        g.GPIO_Speed = GPIO_Speed_50MHz;
        g.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOD, &g);
        // 初始关闭激光
        GPIO_ResetBits(GPIOD, GPIO_Pin_12);
    }
    // 初始化云台PID控制器
    Vision_GimbalPID_Init();
    // 初始化视觉策略(ROI状态机)
    Tracking_Init();

    // 初始化视觉上下文状态
    s_vision_ctx.error_x = 0.0f;
    s_vision_ctx.error_y = 0.0f;
    s_vision_ctx.error_distance = 0.0f;
    s_vision_ctx.target_detected = 0U;
    s_vision_ctx.laser_on = 0U;
    s_vision_ctx.hit_streak = 0U;
    s_vision_ctx.gimbal_x_angle = 0;
    s_vision_ctx.gimbal_y_angle = 0;
    s_vision_ctx.roi_state = ROI_FULL;

    // 标记系统已初始化
    s_initialized = 1U;
}

// 视觉模式状态机主循环
// 四种模式: 空闲/圆形扫描/追踪射击/追踪+圆形扫描
void Vision_Process(void)
{
    uint8_t fresh_packet = 0U;

    // 系统未初始化直接返回
    if (!s_initialized) return;

    // 阶段1: IMU前馈补偿更新(在Vision_IMUFeedforward1kTick中已处理)

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

    // 阶段5: 调试打印输出(默认注释)
//    Vision_DebugPrint();
}

// 获取视觉上下文指针
Vision_Context_t* Vision_GetContext(void)
{
    return &s_vision_ctx;
}
