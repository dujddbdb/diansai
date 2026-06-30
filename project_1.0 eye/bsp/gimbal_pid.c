#include "gimbal_pid.h"
#include "board.h"
#include "stepper.h"
#include <math.h>

static GimbalDebugState_t g_gimbal_debug = {0};

#define GIMBAL_INTERLEAVE_CHUNK_PULSES          8U   // 单次交错发送脉冲块大小
#define GIMBAL_INTERLEAVE_COMMANDS_PER_UPDATE   6U   // 每次更新周期最多发送命令数
#define GIMBAL_PENDING_PULSE_LIMIT              96U  // 待发送脉冲累积上限

// 单轴步进电机命令队列: 累积待发送脉冲, 支持交错发送保证运动平滑
typedef struct {
    uint32_t pending_pulses;
    uint8_t dir;
    uint16_t rpm;
    uint8_t acc;
} GimbalAxisCommandQueue_t;

static GimbalAxisCommandQueue_t s_x_cmd_queue = {0};
static GimbalAxisCommandQueue_t s_y_cmd_queue = {0};
static uint8_t s_next_axis_to_send = 0U;

static float Gimbal_ClampFloat(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static void Gimbal_QueueAxisCommand(GimbalAxisCommandQueue_t *queue,
                                    uint8_t dir,
                                    uint16_t rpm,
                                    uint8_t acc,
                                    uint32_t pulses)
{
    if (pulses == 0U) {
        return;
    }

    if (queue->pending_pulses > 0U && queue->dir != dir) {
        queue->pending_pulses = 0U;
    }

    queue->dir = dir;
    queue->rpm = rpm;
    queue->acc = acc;

    if (pulses > GIMBAL_PENDING_PULSE_LIMIT) {
        pulses = GIMBAL_PENDING_PULSE_LIMIT;
    }

    if (queue->pending_pulses > (GIMBAL_PENDING_PULSE_LIMIT - pulses)) {
        queue->pending_pulses = GIMBAL_PENDING_PULSE_LIMIT;
    } else {
        queue->pending_pulses += pulses;
    }
}

static uint32_t Gimbal_TakeAxisChunk(GimbalAxisCommandQueue_t *queue)
{
    uint32_t chunk;

    if (queue->pending_pulses == 0U) {
        return 0U;
    }

    chunk = queue->pending_pulses;
    if (chunk > GIMBAL_INTERLEAVE_CHUNK_PULSES) {
        chunk = GIMBAL_INTERLEAVE_CHUNK_PULSES;
    }

    queue->pending_pulses -= chunk;
    return chunk;
}

static uint8_t Gimbal_SendAxisChunk(GimbalDualPID_t *dual_pid, uint8_t axis)
{
    uint32_t chunk;

    if (axis == 0U) {
        chunk = Gimbal_TakeAxisChunk(&s_x_cmd_queue);
        if (chunk == 0U) {
            return 0U;
        }
        // X轴步进电机位置命令发送
        StepperXOY_Position(dual_pid->x_axis.motor_addr,
                            s_x_cmd_queue.dir,
                            s_x_cmd_queue.rpm,
                            s_x_cmd_queue.acc,
                            chunk,
                            0,
                            0);
        return 1U;
    }

    chunk = Gimbal_TakeAxisChunk(&s_y_cmd_queue);
    if (chunk == 0U) {
        return 0U;
    }
    // Y轴步进电机位置命令发送
    StepperYOZ_Position(dual_pid->y_axis.motor_addr,
                        s_y_cmd_queue.dir,
                        s_y_cmd_queue.rpm,
                        s_y_cmd_queue.acc,
                        chunk,
                        0,
                        0);
    return 1U;
}

static void Gimbal_FlushFineInterleaved(GimbalDualPID_t *dual_pid)
{
    uint8_t sent;

    // 双轴交错发送小脉冲块，保证运动平滑
    for (sent = 0U; sent < GIMBAL_INTERLEAVE_COMMANDS_PER_UPDATE; sent++) {
        uint8_t axis = s_next_axis_to_send;

        if (!Gimbal_SendAxisChunk(dual_pid, axis)) {
            axis ^= 1U;
            if (!Gimbal_SendAxisChunk(dual_pid, axis)) {
                break;
            }
        }

        s_next_axis_to_send = axis ^ 1U;
    }
}

static uint32_t Gimbal_AngleToPulses(float angle_deg)
{
    float pulses = fabsf(angle_deg) * GIMBAL_PULSES_PER_DEG;

    if (pulses < 0.5f) {
        return 0U;
    }

    return (uint32_t)(pulses + 0.5f);
}

static float GimbalPID_BlendFactor(float abs_error_px)
{
    float t;

    if (abs_error_px <= PID_ERROR_BLEND_START) {
        return 0.0f;
    }

    if (abs_error_px >= PID_ERROR_BLEND_END) {
        return 1.0f;
    }

    t = (abs_error_px - PID_ERROR_BLEND_START) /
        (PID_ERROR_BLEND_END - PID_ERROR_BLEND_START);
    return t * t * (3.0f - 2.0f * t);
}

static void GimbalPID_UpdateRuntimeGains(GimbalPID_t *pid, float abs_error_px)
{
    float blend;
    float target_kp;
    float target_ki;
    float target_kd;

    // 误差混合因子: 小误差→小增益(精细)，大误差→大增益(快速)
    blend = GimbalPID_BlendFactor(abs_error_px);

    // 目标参数 = 小误差参数 + (大误差参数 - 小误差参数) * blend
    target_kp = pid->kp_small + (pid->kp_large - pid->kp_small) * blend;
    target_ki = pid->ki_small + (pid->ki_large - pid->ki_small) * blend;
    target_kd = pid->kd_small + (pid->kd_large - pid->kd_small) * blend;

    // PID参数平滑渐变: 指数趋近目标值，避免参数突变导致振荡
    pid->kp_runtime += (target_kp - pid->kp_runtime) * PID_GAIN_SLEW_FACTOR;
    pid->ki_runtime += (target_ki - pid->ki_runtime) * PID_GAIN_SLEW_FACTOR;
    pid->kd_runtime += (target_kd - pid->kd_runtime) * PID_GAIN_SLEW_FACTOR;
}

void GimbalPID_Init(GimbalPID_t *pid, float output_max, uint8_t motor_addr) {
    pid->kp_large = PID_KP_LARGE;
    pid->ki_large = PID_KI_LARGE;
    pid->kd_large = PID_KD_LARGE;

    pid->kp_small = PID_KP_SMALL;
    pid->ki_small = PID_KI_SMALL;
    pid->kd_small = PID_KD_SMALL;

    pid->kp_runtime = PID_KP_SMALL;
    pid->ki_runtime = PID_KI_SMALL;
    pid->kd_runtime = PID_KD_SMALL;

    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative_filtered = 0.0f;
    pid->output = 0.0f;
    pid->last_output = 0.0f;

    pid->output_max = output_max;
    pid->int_separation_threshold = PID_INT_SEPARATION_THRESHOLD;

    pid->pixel_to_angle = PIXEL_TO_ANGLE_FACTOR;

    pid->motor_addr = motor_addr;
    pid->rpm = GIMBAL_RPM_DEFAULT;
    pid->acc = GIMBAL_ACC_DEFAULT;
}

void GimbalDualPID_Init(GimbalDualPID_t *dual_pid) {
    GimbalPID_Init(&dual_pid->x_axis, PID_OUTPUT_MAX_X, GIMBAL_ADDR_X);
    GimbalPID_Init(&dual_pid->y_axis, PID_OUTPUT_MAX_Y, GIMBAL_ADDR_Y);

    dual_pid->yaw_delta = 0.0f;
    dual_pid->pitch_delta = 0.0f;
    dual_pid->yaw_rate = 0.0f;
    dual_pid->pitch_rate = 0.0f;
    dual_pid->imu_dt_s = 0.0f;
    dual_pid->compensation_factor = GIMBAL_COMPENSATION_FACTOR;
    dual_pid->rate_feedforward_factor = GIMBAL_RATE_FEEDFORWARD_FACTOR;
    dual_pid->compensation_enabled = 0;
}

float GimbalPID_Calculate(GimbalPID_t *pid, float error_px) {
    float kp, ki, kd;
    float error_angle;
    float derivative_raw;
    float p_term, i_term, d_term;
    float output_raw;
    float output_delta;
    float output_smoothed;

    // 像素死区: 误差小于阈值时视为无误差，输出归零，避免微小抖动
    if (fabsf(error_px) <= GIMBAL_PIXEL_DEADBAND) {
        GimbalPID_UpdateRuntimeGains(pid, 0.0f);

        pid->error = error_px;
        pid->last_error = 0.0f;
        pid->integral = 0.0f;
        pid->derivative_filtered = 0.0f;

        pid->output = 0.0f;
        pid->last_output = 0.0f;

        return 0.0f;
    }

    // 像素误差转为角度误差
    error_angle = error_px * pid->pixel_to_angle;
    pid->error = error_px;

    // 分段PID参数更新: 根据误差大小动态调整Kp/Ki/Kd
    GimbalPID_UpdateRuntimeGains(pid, fabsf(error_px));

    kp = pid->kp_runtime;
    ki = pid->ki_runtime;
    kd = pid->kd_runtime;

    // 积分分离: 误差大时关闭积分（防饱和），误差小时开启积分（消静差）
    if (fabsf(error_px) > pid->int_separation_threshold) {
        pid->integral = 0.0f;
    } else {
        pid->integral += error_angle;

        // 积分限幅: 防止积分项过大导致超调
        if (pid->integral > GIMBAL_INTEGRAL_LIMIT_DEG) {
            pid->integral = GIMBAL_INTEGRAL_LIMIT_DEG;
        } else if (pid->integral < -GIMBAL_INTEGRAL_LIMIT_DEG) {
            pid->integral = -GIMBAL_INTEGRAL_LIMIT_DEG;
        }
    }

    // 单轴PID计算: P(比例) + I(积分) + D(微分)
    p_term = kp * error_angle;
    i_term = ki * pid->integral;
    derivative_raw = error_angle - pid->last_error * pid->pixel_to_angle;
    pid->derivative_filtered +=
        (derivative_raw - pid->derivative_filtered) * GIMBAL_DERIVATIVE_FILTER_ALPHA;
    d_term = kd * pid->derivative_filtered;

    output_raw = p_term + i_term + d_term;

    // 输出限幅: 钳位到[-output_max, +output_max]
    if (output_raw > pid->output_max) {
        output_raw = pid->output_max;
    } else if (output_raw < -pid->output_max) {
        output_raw = -pid->output_max;
    }

    // 输出平滑限速: 相邻两次输出的变化量不超过output_max，防止突变
    output_delta = output_raw - pid->last_output;

    if (fabsf(output_delta) > GIMBAL_OUTPUT_SLEW_DEG) {
        if (output_delta > 0) {
            output_smoothed = pid->last_output + GIMBAL_OUTPUT_SLEW_DEG;
        } else {
            output_smoothed = pid->last_output - GIMBAL_OUTPUT_SLEW_DEG;
        }
    } else {
        output_smoothed = output_raw;
    }
    output_smoothed = Gimbal_ClampFloat(output_smoothed, pid->output_max);

    pid->last_error = error_px;
    pid->last_output = output_smoothed;
    pid->output = output_smoothed;

    return output_smoothed;
}

void GimbalDualPID_Update(GimbalDualPID_t *dual_pid, float error_x_px, float error_y_px) {
    float output_x, output_y;
    float yaw_compensation_angle = 0.0f;
    float pitch_compensation_angle = 0.0f;

    uint8_t dir_x, dir_y;
    uint32_t pulses_x, pulses_y;

    // 双轴PID计算: X轴和Y轴独立计算
    output_x = GimbalPID_Calculate(&dual_pid->x_axis, error_x_px);
    output_y = GimbalPID_Calculate(&dual_pid->y_axis, error_y_px);

    // IMU前馈补偿: 车身姿态变化反向补偿到云台输出
    // 补偿量 = -IMU角度变化 × 补偿系数，抵消车身晃动对瞄准的影响
    if (dual_pid->compensation_enabled) {
        if (fabsf(dual_pid->yaw_delta) >= GIMBAL_YAW_DELTA_THRESHOLD) {
            yaw_compensation_angle =
                -(dual_pid->yaw_delta +
                  dual_pid->yaw_rate * dual_pid->rate_feedforward_factor) *
                dual_pid->compensation_factor;
            yaw_compensation_angle =
                Gimbal_ClampFloat(yaw_compensation_angle, GIMBAL_FEEDFORWARD_MAX_DEG);
            output_x += yaw_compensation_angle;
        }

        if (fabsf(dual_pid->pitch_delta) >= GIMBAL_PITCH_DELTA_THRESHOLD) {
            pitch_compensation_angle =
                -(dual_pid->pitch_delta +
                  dual_pid->pitch_rate * dual_pid->rate_feedforward_factor) *
                dual_pid->compensation_factor;
            pitch_compensation_angle =
                Gimbal_ClampFloat(pitch_compensation_angle, GIMBAL_FEEDFORWARD_MAX_DEG);
            output_y += pitch_compensation_angle;
        }
    }

    output_x = Gimbal_ClampFloat(output_x, dual_pid->x_axis.output_max);
    output_y = Gimbal_ClampFloat(output_y, dual_pid->y_axis.output_max);
    dual_pid->x_axis.output = output_x;
    dual_pid->y_axis.output = output_y;

    // 补偿数据单次使用，用后清零
    dual_pid->yaw_delta = 0.0f;
    dual_pid->pitch_delta = 0.0f;
    dual_pid->yaw_rate = 0.0f;
    dual_pid->pitch_rate = 0.0f;
    dual_pid->imu_dt_s = 0.0f;

    // 角度→脉冲→方向: 负数反转方向
    if (output_x <= 0) {
        dir_x = 0;
        pulses_x = Gimbal_AngleToPulses(output_x);
    } else {
        dir_x = 1;
        pulses_x = Gimbal_AngleToPulses(output_x);
    }

    if (output_y >= 0) {
        dir_y = 0;
        pulses_y = Gimbal_AngleToPulses(output_y);
    } else {
        dir_y = 1;
        pulses_y = Gimbal_AngleToPulses(output_y);
    }

    // 双轴交叉输出: 两轴脉冲交错发送，保证运动平滑
    Gimbal_QueueAxisCommand(&s_x_cmd_queue,
                            dir_x,
                            dual_pid->x_axis.rpm,
                            dual_pid->x_axis.acc,
                            pulses_x);
    Gimbal_QueueAxisCommand(&s_y_cmd_queue,
                            dir_y,
                            dual_pid->y_axis.rpm,
                            dual_pid->y_axis.acc,
                            pulses_y);
    Gimbal_FlushFineInterleaved(dual_pid);

    g_gimbal_debug.output_x_deg = output_x;
    g_gimbal_debug.output_y_deg = output_y;
    g_gimbal_debug.yaw_feedforward_deg = yaw_compensation_angle;
    g_gimbal_debug.pitch_feedforward_deg = pitch_compensation_angle;
    g_gimbal_debug.dir_x = dir_x;
    g_gimbal_debug.dir_y = dir_y;
    g_gimbal_debug.pulses_x = pulses_x;
    g_gimbal_debug.pulses_y = pulses_y;
}

void GimbalPID_ClearIntegral(GimbalPID_t *pid) {
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->derivative_filtered = 0.0f;
}

void GimbalDualPID_ClearIntegral(GimbalDualPID_t *dual_pid) {
    GimbalPID_ClearIntegral(&dual_pid->x_axis);
    GimbalPID_ClearIntegral(&dual_pid->y_axis);
}

void GimbalPID_SetParams(GimbalPID_t *pid,
                         float kp_large, float ki_large, float kd_large,
                         float kp_small, float ki_small, float kd_small) {
    pid->kp_large = kp_large;
    pid->ki_large = ki_large;
    pid->kd_large = kd_large;

    pid->kp_small = kp_small;
    pid->ki_small = ki_small;
    pid->kd_small = kd_small;
}

void GimbalPID_SetPixelToAngle(GimbalPID_t *pid, float factor) {
    pid->pixel_to_angle = factor;
}

void GimbalDualPID_SetCompensation(GimbalDualPID_t *dual_pid, float factor, uint8_t enabled) {
    dual_pid->compensation_factor = factor;
    dual_pid->compensation_enabled = enabled;
}

void GimbalDualPID_SetIMUDelta(GimbalDualPID_t *dual_pid, float yaw_delta, float pitch_delta) {
    GimbalDualPID_SetIMUFeedforward(dual_pid,
                                    yaw_delta,
                                    pitch_delta,
                                    0.0f,
                                    0.0f,
                                    0.0f);
}

void GimbalDualPID_SetIMUFeedforward(GimbalDualPID_t *dual_pid,
                                     float yaw_delta,
                                     float pitch_delta,
                                     float yaw_rate,
                                     float pitch_rate,
                                     float dt_s) {
    dual_pid->yaw_delta = yaw_delta;
    dual_pid->pitch_delta = pitch_delta;
    dual_pid->yaw_rate = yaw_rate;
    dual_pid->pitch_rate = pitch_rate;
    dual_pid->imu_dt_s = dt_s;
}

const GimbalDebugState_t* GimbalDualPID_GetDebugState(void)
{
    return &g_gimbal_debug;
}
