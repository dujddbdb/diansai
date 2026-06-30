#include "gimbal_pid.h"
#include "board.h"
#include "stepper.h"
#include <math.h>

// ==================== 全局静态变量 ====================
static GimbalDebugState_t g_gimbal_debug = {0};  // PID调试信息

// ==================== 交错发送配置 ====================
#define GIMBAL_INTERLEAVE_CHUNK_PULSES          8U   // 单次交错发送脉冲块大小
#define GIMBAL_INTERLEAVE_COMMANDS_PER_UPDATE   6U   // 每次更新周期最多发送命令数
#define GIMBAL_PENDING_PULSE_LIMIT              96U  // 待发送脉冲累积上限

// ==================== 单轴命令队列 ====================
// 单轴步进电机命令队列: 累积待发送脉冲, 支持交错发送保证运动平滑
typedef struct {
    uint32_t pending_pulses;   // 待发送脉冲数
    uint8_t  dir;              // 运动方向
    uint16_t rpm;              // 转速
    uint8_t  acc;              // 加速度
} GimbalAxisCommandQueue_t;

static GimbalAxisCommandQueue_t s_x_cmd_queue = {0};  // X轴命令队列
static GimbalAxisCommandQueue_t s_y_cmd_queue = {0};  // Y轴命令队列
static uint8_t s_next_axis_to_send = 0U;              // 下一个发送的轴(0-X, 1-Y)

// ==================== 工具函数 ====================

// 浮点数对称钳位: value ∈ [-limit, +limit]
static float Gimbal_ClampFloat(float value, float limit)
{
    // 正向超上限，返回上限
    if (value > limit) return limit;
    // 负向超下限，返回下限
    if (value < -limit) return -limit;
    // 在范围内返回原值
    return value;
}

// ==================== 命令队列操作 ====================

// 将脉冲命令加入队列，dir: 方向，rpm: 转速，acc: 加速度，pulses: 脉冲数
static void Gimbal_QueueAxisCommand(GimbalAxisCommandQueue_t *queue,
                                    uint8_t dir,
                                    uint16_t rpm,
                                    uint8_t acc,
                                    uint32_t pulses)
{
    // 脉冲数为0，无需入队
    if (pulses == 0U) {
        return;
    }

    // 方向改变: 清空之前的待发送脉冲(反向运动)
    if (queue->pending_pulses > 0U && queue->dir != dir) {
        queue->pending_pulses = 0U;
    }

    // 更新当前运动参数
    queue->dir = dir;
    queue->rpm = rpm;
    queue->acc = acc;

    // 脉冲数超限，截断到上限
    if (pulses > GIMBAL_PENDING_PULSE_LIMIT) {
        pulses = GIMBAL_PENDING_PULSE_LIMIT;
    }

    // 累加脉冲数，防止溢出上限
    if (queue->pending_pulses > (GIMBAL_PENDING_PULSE_LIMIT - pulses)) {
        queue->pending_pulses = GIMBAL_PENDING_PULSE_LIMIT;
    } else {
        queue->pending_pulses += pulses;
    }
}

// 从队列取出一块脉冲(最多CHUNK_PULSES个)
static uint32_t Gimbal_TakeAxisChunk(GimbalAxisCommandQueue_t *queue)
{
    uint32_t chunk;

    // 队列空，返回0
    if (queue->pending_pulses == 0U) {
        return 0U;
    }

    // 取出所有剩余脉冲或一块
    chunk = queue->pending_pulses;
    if (chunk > GIMBAL_INTERLEAVE_CHUNK_PULSES) {
        chunk = GIMBAL_INTERLEAVE_CHUNK_PULSES;
    }

    // 队列中减去已取出的脉冲数
    queue->pending_pulses -= chunk;
    return chunk;
}

// 发送单轴一块脉冲命令，axis: 0-X轴 1-Y轴，返回是否发送成功
static uint8_t Gimbal_SendAxisChunk(GimbalDualPID_t *dual_pid, uint8_t axis)
{
    uint32_t chunk;

    // X轴: 从X队列取脉冲并发送
    if (axis == 0U) {
        chunk = Gimbal_TakeAxisChunk(&s_x_cmd_queue);
        if (chunk == 0U) {
            return 0U;
        }
        // 调用X轴步进电机位置命令
        StepperXOY_Position(dual_pid->x_axis.motor_addr,
                            s_x_cmd_queue.dir,
                            s_x_cmd_queue.rpm,
                            s_x_cmd_queue.acc,
                            chunk,
                            0,
                            0);
        return 1U;
    }

    // Y轴: 从Y队列取脉冲并发送
    chunk = Gimbal_TakeAxisChunk(&s_y_cmd_queue);
    if (chunk == 0U) {
        return 0U;
    }
    // 调用Y轴步进电机位置命令
    StepperYOZ_Position(dual_pid->y_axis.motor_addr,
                        s_y_cmd_queue.dir,
                        s_y_cmd_queue.rpm,
                        s_y_cmd_queue.acc,
                        chunk,
                        0,
                        0);
    return 1U;
}

// 双轴交错发送小脉冲块，保证运动平滑
static void Gimbal_FlushFineInterleaved(GimbalDualPID_t *dual_pid)
{
    uint8_t sent;

    // 循环发送，最多COMMANDS_PER_UPDATE次
    for (sent = 0U; sent < GIMBAL_INTERLEAVE_COMMANDS_PER_UPDATE; sent++) {
        uint8_t axis = s_next_axis_to_send;

        // 尝试发送当前轴
        if (!Gimbal_SendAxisChunk(dual_pid, axis)) {
            // 当前轴无数据，换另一轴试试
            axis ^= 1U;
            if (!Gimbal_SendAxisChunk(dual_pid, axis)) {
                // 两轴都没数据，退出循环
                break;
            }
        }

        // 切换下一个发送的轴(交错)
        s_next_axis_to_send = axis ^ 1U;
    }
}

// ==================== 角度脉冲转换 ====================

// 角度转脉冲数，angle_deg: 角度(度)，返回脉冲数
static uint32_t Gimbal_AngleToPulses(float angle_deg)
{
    // 角度绝对值 × 每度脉冲数
    float pulses = fabsf(angle_deg) * GIMBAL_PULSES_PER_DEG;

    // 不足0.5脉冲，返回0
    if (pulses < 0.5f) {
        return 0U;
    }

    // 四舍五入取整
    return (uint32_t)(pulses + 0.5f);
}

// ==================== PID参数渐变混合 ====================

// 误差混合因子计算: 使用smoothstep曲线平滑过渡
static float GimbalPID_BlendFactor(float abs_error_px)
{
    float t;

    // 误差小于起始值: 混合因子=0，全用小增益
    if (abs_error_px <= PID_ERROR_BLEND_START) {
        return 0.0f;
    }

    // 误差大于结束值: 混合因子=1，全用大增益
    if (abs_error_px >= PID_ERROR_BLEND_END) {
        return 1.0f;
    }

    // 中间区间: 归一化到[0,1]
    t = (abs_error_px - PID_ERROR_BLEND_START) /
        (PID_ERROR_BLEND_END - PID_ERROR_BLEND_START);
    // smoothstep平滑: t²*(3-2t)，两端导数为0，过渡平滑
    return t * t * (3.0f - 2.0f * t);
}

// 更新运行时PID增益(渐变混合): 根据误差大小平滑切换大小增益参数
static void GimbalPID_UpdateRuntimeGains(GimbalPID_t *pid, float abs_error_px)
{
    float blend;
    float target_kp;
    float target_ki;
    float target_kd;

    // 步骤1: 计算误差混合因子
    blend = GimbalPID_BlendFactor(abs_error_px);

    // 步骤2: 线性插值计算目标增益
    // 目标Kp = 小Kp + (大Kp - 小Kp) * blend
    target_kp = pid->kp_small + (pid->kp_large - pid->kp_small) * blend;
    // 目标Ki = 小Ki + (大Ki - 小Ki) * blend
    target_ki = pid->ki_small + (pid->ki_large - pid->ki_small) * blend;
    // 目标Kd = 小Kd + (大Kd - 小Kd) * blend
    target_kd = pid->kd_small + (pid->kd_large - pid->kd_small) * blend;

    // 步骤3: PID参数指数渐变，避免参数突变导致振荡
    // Kp渐变趋近目标值
    pid->kp_runtime += (target_kp - pid->kp_runtime) * PID_GAIN_SLEW_FACTOR;
    // Ki渐变趋近目标值
    pid->ki_runtime += (target_ki - pid->ki_runtime) * PID_GAIN_SLEW_FACTOR;
    // Kd渐变趋近目标值
    pid->kd_runtime += (target_kd - pid->kd_runtime) * PID_GAIN_SLEW_FACTOR;
}

// ==================== PID初始化 ====================

// 初始化单轴PID控制器
void GimbalPID_Init(GimbalPID_t *pid, float output_max, uint8_t motor_addr) {
    // 设置大误差PID参数
    pid->kp_large = PID_KP_LARGE;
    pid->ki_large = PID_KI_LARGE;
    pid->kd_large = PID_KD_LARGE;

    // 设置小误差PID参数
    pid->kp_small = PID_KP_SMALL;
    pid->ki_small = PID_KI_SMALL;
    pid->kd_small = PID_KD_SMALL;

    // 运行时参数初始化为小增益
    pid->kp_runtime = PID_KP_SMALL;
    pid->ki_runtime = PID_KI_SMALL;
    pid->kd_runtime = PID_KD_SMALL;

    // 清零误差和积分项
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative_filtered = 0.0f;
    pid->output = 0.0f;
    pid->last_output = 0.0f;

    // 设置输出限制和积分分离阈值
    pid->output_max = output_max;
    pid->int_separation_threshold = PID_INT_SEPARATION_THRESHOLD;

    // 设置像素转角度系数
    pid->pixel_to_angle = PIXEL_TO_ANGLE_FACTOR;

    // 设置电机地址和默认转速/加速度
    pid->motor_addr = motor_addr;
    pid->rpm = GIMBAL_RPM_DEFAULT;
    pid->acc = GIMBAL_ACC_DEFAULT;
}

// 初始化双轴PID控制器
void GimbalDualPID_Init(GimbalDualPID_t *dual_pid) {
    // 初始化X轴PID
    GimbalPID_Init(&dual_pid->x_axis, PID_OUTPUT_MAX_X, GIMBAL_ADDR_X);
    // 初始化Y轴PID
    GimbalPID_Init(&dual_pid->y_axis, PID_OUTPUT_MAX_Y, GIMBAL_ADDR_Y);

    // 清零IMU补偿相关变量
    dual_pid->yaw_delta = 0.0f;
    dual_pid->pitch_delta = 0.0f;
    dual_pid->yaw_rate = 0.0f;
    dual_pid->pitch_rate = 0.0f;
    dual_pid->imu_dt_s = 0.0f;
    // 设置默认补偿系数和前馈系数
    dual_pid->compensation_factor = GIMBAL_COMPENSATION_FACTOR;
    dual_pid->rate_feedforward_factor = GIMBAL_RATE_FEEDFORWARD_FACTOR;
    // 默认关闭补偿
    dual_pid->compensation_enabled = 0;
}

// ==================== 单轴PID计算 ====================

// 单轴PID计算，error_px: 像素误差，返回输出角度(度)
float GimbalPID_Calculate(GimbalPID_t *pid, float error_px) {
    float kp, ki, kd;
    float error_angle;
    float derivative_raw;
    float p_term, i_term, d_term;
    float output_raw;
    float output_delta;
    float output_smoothed;

    // ===== 步骤1: 像素死区处理 =====
    // 误差小于死区阈值，视为无误差，输出归零，避免微小抖动
    if (fabsf(error_px) <= GIMBAL_PIXEL_DEADBAND) {
        // 更新运行时增益(用0误差，即小增益)
        GimbalPID_UpdateRuntimeGains(pid, 0.0f);

        // 清零所有状态
        pid->error = error_px;
        pid->last_error = 0.0f;
        pid->integral = 0.0f;
        pid->derivative_filtered = 0.0f;

        // 输出归零
        pid->output = 0.0f;
        pid->last_output = 0.0f;

        return 0.0f;
    }

    // ===== 步骤2: 像素误差转角度误差 =====
    error_angle = error_px * pid->pixel_to_angle;
    pid->error = error_px;

    // ===== 步骤3: 分段PID参数更新(渐变混合) =====
    GimbalPID_UpdateRuntimeGains(pid, fabsf(error_px));

    // 读取当前运行时PID参数
    kp = pid->kp_runtime;
    ki = pid->ki_runtime;
    kd = pid->kd_runtime;

    // ===== 步骤4: 积分分离 + 积分限幅 =====
    // 误差大于分离阈值: 关闭积分，防止饱和
    if (fabsf(error_px) > pid->int_separation_threshold) {
        pid->integral = 0.0f;
    } else {
        // 误差在阈值内: 累加积分项
        pid->integral += error_angle;

        // 积分限幅: 钳位到[-LIMIT, +LIMIT]，防止积分超调
        if (pid->integral > GIMBAL_INTEGRAL_LIMIT_DEG) {
            pid->integral = GIMBAL_INTEGRAL_LIMIT_DEG;
        } else if (pid->integral < -GIMBAL_INTEGRAL_LIMIT_DEG) {
            pid->integral = -GIMBAL_INTEGRAL_LIMIT_DEG;
        }
    }

    // ===== 步骤5: 单轴PID计算 — P + I + D =====
    // 比例项: P = Kp × 误差
    p_term = kp * error_angle;
    // 积分项: I = Ki × 积分累加值
    i_term = ki * pid->integral;
    // 微分项: 原始微分 = 当前误差 - 上次误差
    derivative_raw = error_angle - pid->last_error * pid->pixel_to_angle;
    // 微分项一阶低通滤波，滤除噪声
    pid->derivative_filtered +=
        (derivative_raw - pid->derivative_filtered) * GIMBAL_DERIVATIVE_FILTER_ALPHA;
    // 微分项: D = Kd × 滤波后微分
    d_term = kd * pid->derivative_filtered;

    // PID总输出
    output_raw = p_term + i_term + d_term;

    // ===== 步骤6: 输出限幅 =====
    // 钳位到[-output_max, +output_max]
    if (output_raw > pid->output_max) {
        output_raw = pid->output_max;
    } else if (output_raw < -pid->output_max) {
        output_raw = -pid->output_max;
    }

    // ===== 步骤7: 输出平滑限速 =====
    // 计算本次输出与上次输出的差值
    output_delta = output_raw - pid->last_output;

    // 变化量超过限速值: 按最大限速步进
    if (fabsf(output_delta) > GIMBAL_OUTPUT_SLEW_DEG) {
        if (output_delta > 0) {
            // 正向增加，加最大步长
            output_smoothed = pid->last_output + GIMBAL_OUTPUT_SLEW_DEG;
        } else {
            // 负向减小，减最大步长
            output_smoothed = pid->last_output - GIMBAL_OUTPUT_SLEW_DEG;
        }
    } else {
        // 变化量在限速内，直接使用
        output_smoothed = output_raw;
    }
    // 再次限幅确保不超范围
    output_smoothed = Gimbal_ClampFloat(output_smoothed, pid->output_max);

    // ===== 步骤8: 保存状态供下次使用 =====
    pid->last_error = error_px;
    pid->last_output = output_smoothed;
    pid->output = output_smoothed;

    // 返回平滑后的输出
    return output_smoothed;
}

// ==================== 双轴PID更新 + IMU前馈补偿 ====================

// 双轴PID更新，error_x_px: X轴像素误差，error_y_px: Y轴像素误差
void GimbalDualPID_Update(GimbalDualPID_t *dual_pid, float error_x_px, float error_y_px) {
    float output_x, output_y;
    float yaw_compensation_angle = 0.0f;    // 偏航补偿角度
    float pitch_compensation_angle = 0.0f;  // 俯仰补偿角度

    uint8_t dir_x, dir_y;
    uint32_t pulses_x, pulses_y;

    // 步骤1: 双轴独立PID计算
    output_x = GimbalPID_Calculate(&dual_pid->x_axis, error_x_px);
    output_y = GimbalPID_Calculate(&dual_pid->y_axis, error_y_px);

    // ===== 步骤2: IMU前馈补偿 =====
    // 补偿量 = -(IMU角度变化 + 角速度×前馈系数) × 补偿系数
    // 负号表示反向补偿，抵消车身晃动
    if (dual_pid->compensation_enabled) {
        // X轴(偏航)补偿
        if (fabsf(dual_pid->yaw_delta) >= GIMBAL_YAW_DELTA_THRESHOLD) {
            // 补偿角度 = -(角度变化 + 角速度×前馈系数) × 补偿系数
            yaw_compensation_angle =
                -(dual_pid->yaw_delta +
                  dual_pid->yaw_rate * dual_pid->rate_feedforward_factor) *
                dual_pid->compensation_factor;
            // 补偿量限幅，防止过补偿
            yaw_compensation_angle =
                Gimbal_ClampFloat(yaw_compensation_angle, GIMBAL_FEEDFORWARD_MAX_DEG);
            // 叠加到X轴输出
            output_x += yaw_compensation_angle;
        }

        // Y轴(俯仰)补偿
        if (fabsf(dual_pid->pitch_delta) >= GIMBAL_PITCH_DELTA_THRESHOLD) {
            // 补偿角度 = -(角度变化 + 角速度×前馈系数) × 补偿系数
            pitch_compensation_angle =
                -(dual_pid->pitch_delta +
                  dual_pid->pitch_rate * dual_pid->rate_feedforward_factor) *
                dual_pid->compensation_factor;
            // 补偿量限幅，防止过补偿
            pitch_compensation_angle =
                Gimbal_ClampFloat(pitch_compensation_angle, GIMBAL_FEEDFORWARD_MAX_DEG);
            // 叠加到Y轴输出
            output_y += pitch_compensation_angle;
        }
    }

    // 步骤3: 补偿后再次限幅，保存最终输出
    output_x = Gimbal_ClampFloat(output_x, dual_pid->x_axis.output_max);
    output_y = Gimbal_ClampFloat(output_y, dual_pid->y_axis.output_max);
    dual_pid->x_axis.output = output_x;
    dual_pid->y_axis.output = output_y;

    // 步骤4: 补偿数据单次使用，用后清零
    dual_pid->yaw_delta = 0.0f;
    dual_pid->pitch_delta = 0.0f;
    dual_pid->yaw_rate = 0.0f;
    dual_pid->pitch_rate = 0.0f;
    dual_pid->imu_dt_s = 0.0f;

    // ===== 步骤5: 角度→脉冲→方向转换 =====
    // X轴: 负方向(≤0)dir=0，正方向(>0)dir=1
    if (output_x <= 0) {
        dir_x = 0;
        pulses_x = Gimbal_AngleToPulses(output_x);
    } else {
        dir_x = 1;
        pulses_x = Gimbal_AngleToPulses(output_x);
    }

    // Y轴: 正方向(≥0)dir=0，负方向(<0)dir=1 (与X轴方向定义相反)
    if (output_y >= 0) {
        dir_y = 0;
        pulses_y = Gimbal_AngleToPulses(output_y);
    } else {
        dir_y = 1;
        pulses_y = Gimbal_AngleToPulses(output_y);
    }

    // ===== 步骤6: 双轴交错输出 =====
    // 脉冲加入命令队列
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
    // 交错发送小脉冲块，保证运动平滑
    Gimbal_FlushFineInterleaved(dual_pid);

    // 步骤7: 保存调试信息
    g_gimbal_debug.output_x_deg = output_x;
    g_gimbal_debug.output_y_deg = output_y;
    g_gimbal_debug.yaw_feedforward_deg = yaw_compensation_angle;
    g_gimbal_debug.pitch_feedforward_deg = pitch_compensation_angle;
    g_gimbal_debug.dir_x = dir_x;
    g_gimbal_debug.dir_y = dir_y;
    g_gimbal_debug.pulses_x = pulses_x;
    g_gimbal_debug.pulses_y = pulses_y;
}

// ==================== 积分清零 ====================

// 清空单轴PID积分项
void GimbalPID_ClearIntegral(GimbalPID_t *pid) {
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->derivative_filtered = 0.0f;
}

// 清空双轴PID积分项
void GimbalDualPID_ClearIntegral(GimbalDualPID_t *dual_pid) {
    GimbalPID_ClearIntegral(&dual_pid->x_axis);
    GimbalPID_ClearIntegral(&dual_pid->y_axis);
}

// ==================== 参数设置 ====================

// 设置单轴PID大小误差参数
void GimbalPID_SetParams(GimbalPID_t *pid,
                         float kp_large, float ki_large, float kd_large,
                         float kp_small, float ki_small, float kd_small) {
    // 设置大误差参数
    pid->kp_large = kp_large;
    pid->ki_large = ki_large;
    pid->kd_large = kd_large;

    // 设置小误差参数
    pid->kp_small = kp_small;
    pid->ki_small = ki_small;
    pid->kd_small = kd_small;
}

// 设置像素转角度系数
void GimbalPID_SetPixelToAngle(GimbalPID_t *pid, float factor) {
    pid->pixel_to_angle = factor;
}

// 设置IMU补偿系数和启用状态
void GimbalDualPID_SetCompensation(GimbalDualPID_t *dual_pid, float factor, uint8_t enabled) {
    dual_pid->compensation_factor = factor;
    dual_pid->compensation_enabled = enabled;
}

// 设置IMU角度变化量(静态补偿，角速度为0)
void GimbalDualPID_SetIMUDelta(GimbalDualPID_t *dual_pid, float yaw_delta, float pitch_delta) {
    GimbalDualPID_SetIMUFeedforward(dual_pid,
                                    yaw_delta,
                                    pitch_delta,
                                    0.0f,
                                    0.0f,
                                    0.0f);
}

// 设置IMU前馈补偿(动态补偿，含角速度)
void GimbalDualPID_SetIMUFeedforward(GimbalDualPID_t *dual_pid,
                                     float yaw_delta,
                                     float pitch_delta,
                                     float yaw_rate,
                                     float pitch_rate,
                                     float dt_s) {
    // 保存角度变化量
    dual_pid->yaw_delta = yaw_delta;
    dual_pid->pitch_delta = pitch_delta;
    // 保存角速度
    dual_pid->yaw_rate = yaw_rate;
    dual_pid->pitch_rate = pitch_rate;
    // 保存时间间隔
    dual_pid->imu_dt_s = dt_s;
}

// ==================== 调试接口 ====================

// 获取PID调试状态结构体指针
const GimbalDebugState_t* GimbalDualPID_GetDebugState(void)
{
    return &g_gimbal_debug;
}
