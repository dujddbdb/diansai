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

// ==================== 全局静态变量 ====================
static GimbalDualPID_t  s_gimbal_pid;          // 双轴PID控制器
static Vision_Context_t  s_vision_ctx;          // 视觉上下文状态
static uint8_t          s_initialized = 0U;     // 初始化标志
static uint32_t         __attribute__((unused)) s_debug_last_ms = 0U;  // 调试打印计时
static uint8_t          s_last_fresh_packet = 0U;  // 上一帧是否有新数据包

#define VISION_TWO_PI               6.2831853f  // 2π常量, 用于圆形扫描相位计算

// ==================== 视觉模式枚举 ====================
typedef enum {
    VISION_MODE_IDLE = 0,       // 0-空闲模式，停止所有动作
    VISION_MODE_CIRCLE,         // 1-纯圆形扫描模式
    VISION_MODE_SHOOT,          // 2-追踪射击模式
    VISION_MODE_SHOOT_CIRCLE    // 3-追踪+圆形扫描复合模式
} Vision_Mode_t;

// ==================== IMU角度卡尔曼滤波器状态 ====================
typedef struct {
    float angle;        // 滤波后角度
    float rate;         // 滤波后角速度
    float p00;          // 协方差矩阵P[0][0]
    float p01;          // 协方差矩阵P[0][1]
    float p10;          // 协方差矩阵P[1][0]
    float p11;          // 协方差矩阵P[1][1]
    float p;            // 简化协方差(单阶用)
    uint8_t valid;      // 滤波器是否已初始化有效
} Vision_AngleKalman_t;

static Vision_AngleKalman_t s_yaw_kf = {0};     // 偏航角卡尔曼滤波器
static Vision_AngleKalman_t s_pitch_kf = {0};   // 俯仰角卡尔曼滤波器
static Vision_Mode_t s_eye_mode = VISION_MODE_IDLE;  // 当前视觉模式
static float s_circle_phase = 0.0f;             // 圆形扫描当前相位

// ==================== 工具函数 ====================

// 浮点数区间钳位: value ∈ [min_v, max_v]
static float Vision_ClampFloat(float value, float min_v, float max_v)
{
    // 小于最小值返回最小值
    if (value < min_v) return min_v;
    // 大于最大值返回最大值
    if (value > max_v) return max_v;
    // 在区间内返回原值
    return value;
}

// 角度差环绕处理: 将角度差规约到[-180°, +180°]区间
static float Vision_WrapAngleDeltaDeg(float now_deg, float last_deg)
{
    // 计算原始角度差
    float delta = now_deg - last_deg;

    // 正向超过180°，减360°绕回
    while (delta > 180.0f) delta -= 360.0f;
    // 负向超过-180°，加360°绕回
    while (delta < -180.0f) delta += 360.0f;

    // 返回归一化后的角度差
    return delta;
}

// ==================== 卡尔曼滤波(单阶简化版) ====================

// IMU角度卡尔曼滤波(单阶简化版，仅角度状态)
static float Vision_AngleKalmanUpdate(Vision_AngleKalman_t *kf, float measurement_deg)
{
    float k;

    // 首次测量: 直接初始化角度和协方差，返回原始测量值
    if (!kf->valid) {
        // 初始化角度为测量值
        kf->angle = measurement_deg;
        // 初始化协方差P=1
        kf->p = 1.0f;
        // 标记滤波器有效
        kf->valid = 1U;
        // 首次直接返回测量值
        return measurement_deg;
    }

    // 预测阶段: 处理角度环绕，累加过程噪声Q
    measurement_deg = kf->angle + Vision_WrapAngleDeltaDeg(measurement_deg, kf->angle);
    // 预测协方差: P = P + Q
    kf->p += VISION_IMU_KALMAN_Q;

    // 更新阶段: 计算卡尔曼增益 K = P / (P + R)
    k = kf->p / (kf->p + VISION_IMU_KALMAN_R);
    // 更新角度估计: 估计值 = 预测值 + K * (测量值 - 预测值)
    kf->angle += k * Vision_WrapAngleDeltaDeg(measurement_deg, kf->angle);
    // 更新协方差: P = (1 - K) * P
    kf->p *= (1.0f - k);

    // 返回滤波后的角度
    return kf->angle;
}

// ==================== 卡尔曼滤波(二阶完整版，角度+角速度) ====================

// IMU角度卡尔曼滤波步进(二阶，角度+角速度双状态)
static void Vision_AngleKalmanStep(Vision_AngleKalman_t *kf,
                                   float measurement_deg,
                                   uint8_t has_measurement,
                                   float dt_s)
{
    float p00;
    float p01;
    float p10;
    float p11;
    float innovation;
    float s;
    float k0;
    float k1;

    // 滤波器未初始化: 有测量值则初始化，无测量值直接返回
    if (!kf->valid) {
        // 无测量值，无法初始化，直接返回
        if (!has_measurement) return;
        // 初始化角度为测量值
        kf->angle = measurement_deg;
        // 初始化角速度为0
        kf->rate = 0.0f;
        // 初始化协方差矩阵P为单位矩阵
        kf->p00 = 1.0f;
        kf->p01 = 0.0f;
        kf->p10 = 0.0f;
        kf->p11 = 1.0f;
        kf->p = 1.0f;
        // 标记滤波器有效
        kf->valid = 1U;
        return;
    }

    // 预测阶段: 角度积分更新
    kf->angle += kf->rate * dt_s;

    // 预测阶段: 协方差矩阵P更新
    // P[0][0] += dt*(P[1][0]+P[0][1]) + dt²*P[1][1] + Q_angle
    p00 = kf->p00 + dt_s * (kf->p10 + kf->p01) +
          dt_s * dt_s * kf->p11 + VISION_IMU_KALMAN_Q;
    // P[0][1] += dt * P[1][1]
    p01 = kf->p01 + dt_s * kf->p11;
    // P[1][0] += dt * P[1][1]
    p10 = kf->p10 + dt_s * kf->p11;
    // P[1][1] += Q_rate
    p11 = kf->p11 + VISION_IMU_KALMAN_RATE_Q;

    // 保存更新后的协方差矩阵
    kf->p00 = p00;
    kf->p01 = p01;
    kf->p10 = p10;
    kf->p11 = p11;

    // 无测量值: 仅做预测，跳过更新阶段
    if (!has_measurement) return;

    // 更新阶段: 处理角度环绕，计算新息(测量残差)
    measurement_deg = kf->angle + Vision_WrapAngleDeltaDeg(measurement_deg, kf->angle);
    // 新息 = 测量值 - 预测值
    innovation = Vision_WrapAngleDeltaDeg(measurement_deg, kf->angle);
    // 新息协方差 S = P[0][0] + R
    s = kf->p00 + VISION_IMU_KALMAN_R;
    // 分母太小，防止除零
    if (s <= 1.0e-6f) return;

    // 计算卡尔曼增益
    // K[0] = P[0][0] / S
    k0 = kf->p00 / s;
    // K[1] = P[1][0] / S
    k1 = kf->p10 / s;
    // 暂存更新前的协方差值
    p00 = kf->p00;
    p01 = kf->p01;

    // 更新状态估计
    // 角度更新: angle += K[0] * 新息
    kf->angle += k0 * innovation;
    // 角速度更新: rate += K[1] * 新息
    kf->rate += k1 * innovation;

    // 更新协方差矩阵
    // P[0][0] = (1 - K[0]) * P[0][0]
    kf->p00 = (1.0f - k0) * p00;
    // P[0][1] = (1 - K[0]) * P[0][1]
    kf->p01 = (1.0f - k0) * p01;
    // P[1][0] = P[1][0] - K[1] * P[0][0]
    kf->p10 = kf->p10 - k1 * p00;
    // P[1][1] = P[1][1] - K[1] * P[0][1]
    kf->p11 = kf->p11 - k1 * p01;
    // 保存简化协方差值
    kf->p = kf->p00;
}

// ==================== 误差混合与动态调速 ====================

// 误差混合因子: 平滑过渡小误差→大误差区间的PID参数
// 使用smoothstep曲线，避免参数突变
static float Vision_ErrorBlend(float abs_error_px)
{
    float t;

    // 误差小于起始值: 混合因子=0，全用小增益参数
    if (abs_error_px <= PID_ERROR_BLEND_START) return 0.0f;
    // 误差大于结束值: 混合因子=1，全用大增益参数
    if (abs_error_px >= PID_ERROR_BLEND_END) return 1.0f;

    // 中间区间: 归一化到[0,1]
    t = (abs_error_px - PID_ERROR_BLEND_START) /
        (PID_ERROR_BLEND_END - PID_ERROR_BLEND_START);
    // smoothstep平滑: t²*(3-2t)，两端导数为0
    return t * t * (3.0f - 2.0f * t);
}

// 根据误差大小动态调整单轴电机转速
static uint16_t Vision_UpdateAxisRpm(GimbalPID_t *axis, float abs_error_px, float corner_blend)
{
    // 步骤1: 根据误差计算混合因子
    float blend = Vision_ErrorBlend(abs_error_px);
    // 步骤2: 线性插值计算目标转速: 误差大→转速高，误差小→转速低
    float target = VISION_GIMBAL_RPM_MIN +
                   (VISION_GIMBAL_RPM_MAX - VISION_GIMBAL_RPM_MIN) * blend;
    // 获取当前转速
    float current = (float)axis->rpm;

    // 步骤3: 转弯时按比例降低转速，避免过冲
    target *= (1.0f - VISION_CORNER_RPM_REDUCE * corner_blend);
    // 转速钳位到允许范围
    target = Vision_ClampFloat(target, VISION_GIMBAL_RPM_MIN, VISION_GIMBAL_RPM_MAX);

    // 步骤4: 转速平滑过渡(指数趋近)，避免突变
    if (current < 1.0f) current = VISION_GIMBAL_RPM_MIN;
    // 指数平滑: current += (target - current) * slew
    current += (target - current) * VISION_GIMBAL_RPM_SLEW;
    // 四舍五入保存到PID结构体
    axis->rpm = (uint16_t)(current + 0.5f);

    // 返回更新后的转速
    return axis->rpm;
}

// 更新双轴云台运行时速度 (根据误差大小自适应调速)
static void Vision_UpdateGimbalRuntimeSpeed(float error_x_px, float error_y_px)
{
    // 获取小车转弯混合系数(来自底盘通信)
    float corner_blend = UART5_CarCornerBlend(HAL_GetTick());

    // X轴根据X方向误差调整转速
    Vision_UpdateAxisRpm(&s_gimbal_pid.x_axis, fabsf(error_x_px), corner_blend);
    // Y轴根据Y方向误差调整转速
    Vision_UpdateAxisRpm(&s_gimbal_pid.y_axis, fabsf(error_y_px), corner_blend);
}

// ==================== IMU前馈补偿 ====================

// IMU前馈补偿更新: 读取IMU数据→卡尔曼滤波→计算补偿量
static uint8_t Vision_UpdateGimbalIMUCompensation(void)
{
    static uint8_t imu_valid = 0U;              // IMU数据是否有效
    static float last_yaw_deg = 0.0f;           // 上一帧偏航角
    static float last_pitch_deg = 0.0f;         // 上一帧俯仰角
    static uint32_t last_kalman_ms = 0U;        // 上一次卡尔曼更新时间
    uint32_t now_ms = HAL_GetTick();            // 当前时间
    uint32_t dt_ms;                             // 时间间隔(ms)
    float dt_s;                                 // 时间间隔(s)
    float corner_blend = UART5_CarCornerBlend(now_ms);  // 转弯混合系数
    uint8_t corner_active = (corner_blend >= VISION_IMU_MIN_CORNER_BLEND) ? 1U : 0U;  // 是否在转弯
    uint8_t has_measurement;                    // 是否有新IMU测量值
    float roll_deg = 0.0f;                      // 横滚角(未使用)
    float pitch_deg = 0.0f;                     // 俯仰角原始值
    float yaw_deg = 0.0f;                       // 偏航角原始值
    float filtered_yaw;                         // 滤波后偏航角
    float filtered_pitch;                       // 滤波后俯仰角

    // 步骤1: 设置IMU补偿系数，转弯时启用补偿
    GimbalDualPID_SetCompensation(&s_gimbal_pid,
                                  GIMBAL_COMPENSATION_FACTOR * corner_blend,
                                  corner_active);

    // 步骤2: 读取BNO080 IMU数据
    has_measurement = bno080_update();
    // 无测量值且滤波器未初始化，直接返回
    if (!has_measurement && !s_yaw_kf.valid) {
        return 0U;
    }

    // 步骤3: 计算时间间隔dt并限幅
    if (last_kalman_ms == 0U) {
        // 首次调用默认10ms
        dt_ms = 10U;
    } else {
        // 计算时间差
        dt_ms = (uint32_t)(now_ms - last_kalman_ms);
    }
    // 更新时间戳
    last_kalman_ms = now_ms;
    // dt下限保护
    if (dt_ms < VISION_IMU_DT_MIN_MS) dt_ms = VISION_IMU_DT_MIN_MS;
    // dt上限保护
    if (dt_ms > VISION_IMU_DT_MAX_MS) dt_ms = VISION_IMU_DT_MAX_MS;
    // 转换为秒
    dt_s = (float)dt_ms * 0.001f;

    // 步骤4: 有测量值则读取欧拉角
    if (has_measurement) {
        bno080_get_euler(&roll_deg, &pitch_deg, &yaw_deg);
        // 横滚角暂不使用
        (void)roll_deg;
    }

    // 步骤5: 二阶卡尔曼滤波更新偏航角和俯仰角
    Vision_AngleKalmanStep(&s_yaw_kf, yaw_deg, has_measurement, dt_s);
    Vision_AngleKalmanStep(&s_pitch_kf, pitch_deg, has_measurement, dt_s);
    // 滤波器无效则返回
    if (!s_yaw_kf.valid || !s_pitch_kf.valid) {
        return 0U;
    }

    // 读取滤波后的角度
    filtered_yaw = s_yaw_kf.angle;
    filtered_pitch = s_pitch_kf.angle;

    // 步骤6: IMU有效且转弯时，计算并设置补偿量
    if (imu_valid && corner_active) {
        // 计算偏航角变化量(环绕处理)
        float yaw_delta = Vision_WrapAngleDeltaDeg(filtered_yaw, last_yaw_deg);
        // 计算俯仰角变化量(环绕处理)
        float pitch_delta = Vision_WrapAngleDeltaDeg(filtered_pitch, last_pitch_deg);

        // 偏航角变化量限幅，防止异常跳变
        yaw_delta = Vision_ClampFloat(yaw_delta,
                                      -VISION_IMU_DELTA_LIMIT_DEG,
                                      VISION_IMU_DELTA_LIMIT_DEG);
        // 俯仰角变化量限幅，防止异常跳变
        pitch_delta = Vision_ClampFloat(pitch_delta,
                                        -VISION_IMU_DELTA_LIMIT_DEG,
                                        VISION_IMU_DELTA_LIMIT_DEG);

        // 角速度接近0: 仅用角度差补偿(静态补偿)
        if ((fabsf(s_yaw_kf.rate) < 0.001f) &&
            (fabsf(s_pitch_kf.rate) < 0.001f)) {
            GimbalDualPID_SetIMUDelta(&s_gimbal_pid, yaw_delta, pitch_delta);
        } else {
            // 有角速度: 角度差+角速度前馈补偿(动态补偿)
            GimbalDualPID_SetIMUFeedforward(&s_gimbal_pid,
                                           yaw_delta,
                                           pitch_delta,
                                           s_yaw_kf.rate,
                                           s_pitch_kf.rate,
                                           dt_s);
        }

        // 保存上一帧角度
        last_yaw_deg = filtered_yaw;
        last_pitch_deg = filtered_pitch;
        // 返回有补偿数据
        return 1U;
    }

    // 步骤7: 首次有效数据或非转弯状态，仅更新历史值
    last_yaw_deg = filtered_yaw;
    last_pitch_deg = filtered_pitch;
    imu_valid = 1U;
    // 返回无补偿数据
    return 0U;
}

// ==================== 模式判断辅助函数 ====================

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

// ==================== 圆形扫描 ====================

// 计算圆形扫描偏移量: 相位累加→cos/sin生成圆轨迹→相位归零循环
static void Vision_CircleOffset(float *circle_x, float *circle_y)
{
    // 相位步进累加
    s_circle_phase += VISION_CIRCLE_STEP_RAD;
    // 相位超过2π则减去2π，循环往复
    if (s_circle_phase >= VISION_TWO_PI) {
        s_circle_phase -= VISION_TWO_PI;
    }

    // 极坐标→直角坐标: X = R * cos(θ)
    *circle_x = VISION_CIRCLE_RADIUS_PX * cosf(s_circle_phase);
    // 极坐标→直角坐标: Y = R * sin(θ)
    *circle_y = VISION_CIRCLE_RADIUS_PX * sinf(s_circle_phase);
}

// ==================== 云台控制执行 ====================

// 执行云台控制: 目标处理→PID更新→激光触发→角度记录
static void Vision_ApplyControl(float error_x, float error_y,
                                uint8_t target_seen,
                                uint8_t laser_allowed)
{
    // 步骤1: 目标处理，更新误差和距离
    Vision_TargetProcess((int16_t)error_x, (int16_t)error_y, true);
    // 目标未检测到: 清除检测标志
    if (!target_seen) {
        s_vision_ctx.target_detected = 0U;
    }
    // 步骤2: PID计算并更新云台输出
    Vision_GimbalPID_Update(s_vision_ctx.error_x, s_vision_ctx.error_y);
    // 步骤3: 激光触发判断
    Vision_LaserTrigger(laser_allowed ? true : false);
    // 步骤4: 记录当前云台角度到策略模块
    VisionStrategy_SetGimbalAngle(s_vision_ctx.gimbal_x_angle,
                                  s_vision_ctx.gimbal_y_angle);
}

// 纯圆形扫描执行 (无目标时根据圆形扫描偏移量控制云台)
static void Vision_RunCircleOnly(void)
{
    float circle_x;
    float circle_y;

    // 计算圆形扫描偏移量
    Vision_CircleOffset(&circle_x, &circle_y);
    // 用圆形偏移量作为控制量，无目标，不开激光
    Vision_ApplyControl(circle_x, circle_y, 0U, 0U);
}

// ==================== 按键模式切换 ====================

// 按键模式切换处理: KEY0→空闲, KEY1→圆形扫描, KEY2→追踪射击, KEY3→追踪+圆形扫描
void Vision_KeyControlTick(void)
{
    // KEY0按下: 切换到空闲模式，停止所有动作
    if (Key_GetState(0U) == KEY_PRESS) {
        // 设置模式为空闲
        s_eye_mode = VISION_MODE_IDLE;
        // 重置圆形扫描相位
        s_circle_phase = 0.0f;
        // 清除目标处理
        Vision_TargetProcess(0, 0, false);
        // 关闭激光
        Vision_LaserTrigger(false);
        // 清除PID积分
        Vision_GimbalPID_ClearIntegral();
    }

    // KEY1按下: 切换到纯圆形扫描模式
    if (Key_GetState(1U) == KEY_PRESS) {
        // 设置模式为圆形扫描
        s_eye_mode = VISION_MODE_CIRCLE;
        // 重置圆形扫描相位，从0开始
        s_circle_phase = 0.0f;
        // 关闭激光
        Vision_LaserTrigger(false);
        // 清除PID积分
        Vision_GimbalPID_ClearIntegral();
    }

    // KEY2按下: 切换到追踪射击模式
    if (Key_GetState(2U) == KEY_PRESS) {
        // 设置模式为追踪射击
        s_eye_mode = VISION_MODE_SHOOT;
        // 重置圆形扫描相位
        s_circle_phase = 0.0f;
        // 关闭激光(待目标稳定后自动开启)
        Vision_LaserTrigger(false);
        // 清除PID积分
        Vision_GimbalPID_ClearIntegral();
    }

    // KEY3按下: 切换到追踪+圆形扫描复合模式
    if (Key_GetState(3U) == KEY_PRESS) {
        // 设置模式为追踪+圆形扫描
        s_eye_mode = VISION_MODE_SHOOT_CIRCLE;
        // 重置圆形扫描相位，从0开始
        s_circle_phase = 0.0f;
        // 关闭激光(待目标稳定后自动开启)
        Vision_LaserTrigger(false);
        // 清除PID积分
        Vision_GimbalPID_ClearIntegral();
    }
}

// ==================== 调试打印 ====================

static void __attribute__((unused)) Vision_DebugPrint(void)
{
    char line[192];
    uint32_t now_ms = HAL_GetTick();
    const GimbalDebugState_t *dbg = GimbalDualPID_GetDebugState();

    // 限流: 每100ms打印一次
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

    // 通过USART3发送调试信息
    USART3_SendString(line);
}

// ==================== PID相关接口函数 ====================

// 初始化视觉系统PID控制器
void Vision_GimbalPID_Init(void)
{
    // 初始化双轴PID
    GimbalDualPID_Init(&s_gimbal_pid);
    // 设置默认补偿系数
    GimbalDualPID_SetCompensation(&s_gimbal_pid,
                                  GIMBAL_COMPENSATION_FACTOR,
                                  0U);
}

// 更新视觉PID输出，error_x_px: X轴像素误差，error_y_px: Y轴像素误差
void Vision_GimbalPID_Update(float error_x_px, float error_y_px)
{
    // 根据误差动态调整云台转速
    Vision_UpdateGimbalRuntimeSpeed(error_x_px, error_y_px);
    // 双轴PID计算并输出
    GimbalDualPID_Update(&s_gimbal_pid, error_x_px, error_y_px);
    // 保存X轴输出角度到上下文
    s_vision_ctx.gimbal_x_angle = (int16_t)s_gimbal_pid.x_axis.output;
    // 保存Y轴输出角度到上下文
    s_vision_ctx.gimbal_y_angle = (int16_t)s_gimbal_pid.y_axis.output;
}

// 清空视觉PID积分项
void Vision_GimbalPID_ClearIntegral(void)
{
    GimbalDualPID_ClearIntegral(&s_gimbal_pid);
}

// IMU补偿定时调用，丢包时维持补偿
void Vision_GimbalIMUCompensationTick(void)
{
    // 未初始化直接返回
    if (!s_initialized) return;
    // 有新视觉数据包时不调用(主循环已处理)
    if (s_last_fresh_packet) return;

    // 无新数据包时，用0误差更新转速(保持低速)
    Vision_UpdateGimbalRuntimeSpeed(0.0f, 0.0f);
    // 用0误差执行PID(仅IMU补偿生效)
    GimbalDualPID_Update(&s_gimbal_pid, 0.0f, 0.0f);
    // 保存输出角度到上下文
    s_vision_ctx.gimbal_x_angle = (int16_t)s_gimbal_pid.x_axis.output;
    s_vision_ctx.gimbal_y_angle = (int16_t)s_gimbal_pid.y_axis.output;
}

// 获取云台PID控制器指针，返回双轴PID结构体指针
GimbalDualPID_t* Vision_GimbalPID_GetController(void)
{
    return &s_gimbal_pid;
}

// ==================== 目标处理与激光触发 ====================

// 处理目标检测数据，err_y: X轴像素误差，err_z: Y轴像素误差，valid: 目标是否有效
void Vision_TargetProcess(int16_t err_y, int16_t err_z, bool valid)
{
    // 目标有效: 更新误差和距离，标记检测到目标
    if (valid) {
        // 保存X轴误差
        s_vision_ctx.error_x = (float)err_y;
        // 保存Y轴误差
        s_vision_ctx.error_y = (float)err_z;
        // 计算欧氏距离(误差大小)
        s_vision_ctx.error_distance = sqrtf(s_vision_ctx.error_x * s_vision_ctx.error_x +
                                            s_vision_ctx.error_y * s_vision_ctx.error_y);
        // 标记目标已检测到
        s_vision_ctx.target_detected = 1U;
    } else {
        // 目标无效: 清零误差，距离设为很大值，标记未检测到
        s_vision_ctx.error_x = 0.0f;
        s_vision_ctx.error_y = 0.0f;
        s_vision_ctx.error_distance = 1.0e6f;
        s_vision_ctx.target_detected = 0U;
    }
}

// 激光触发防抖判断: 连续命中N帧后才触发，避免误触发
// target_valid: 当前是否有有效目标，返回1-激光开启 0-激光关闭
uint8_t Vision_LaserTrigger(bool target_valid)
{
    // 目标无效: 立即关闭激光，清零命中计数
    if (!target_valid) {
        // 关闭继电器(激光)
        Relay_Off();
        // 标记激光关闭
        s_vision_ctx.laser_on = 0U;
        // 清零连续命中计数
        s_vision_ctx.hit_streak = 0U;
        return 0U;
    }

    // 误差在阈值内: 累加命中计数
    if (s_vision_ctx.error_distance < (float)VISION_LASER_THRESHOLD_PX) {
        // 计数加1(防溢出)
        if (s_vision_ctx.hit_streak < 255U) {
            s_vision_ctx.hit_streak++;
        }

        // 连续命中达到防抖帧数: 触发激光
        if (s_vision_ctx.hit_streak >= VISION_LASER_HOLD_FRAMES) {
            // 开启继电器(激光)
            Relay_On();
            // 标记激光开启
            s_vision_ctx.laser_on = 1U;
            return 1U;
        }
    } else {
        // 误差超出阈值: 关闭激光，清零命中计数
        Relay_Off();
        s_vision_ctx.laser_on = 0U;
        s_vision_ctx.hit_streak = 0U;
    }

    // 未达到触发条件，返回0
    return 0U;
}

// 获取当前误差值，err_x: 输出X轴误差，err_y: 输出Y轴误差，dist: 输出误差距离
void Vision_GetError(float *err_x, float *err_y, float *dist)
{
    if (err_x != NULL) *err_x = s_vision_ctx.error_x;
    if (err_y != NULL) *err_y = s_vision_ctx.error_y;
    if (dist  != NULL) *dist  = s_vision_ctx.error_distance;
}

// ==================== 追踪策略接口 ====================

// 初始化追踪策略模块
void Tracking_Init(void)
{
    VisionStrategy_Init();
}

// 更新追踪状态，target_detected: 0-未检测到目标 1-检测到目标
void Tracking_Update(uint8_t target_detected)
{
    // 更新ROI策略状态机
    VisionStrategy_Update(target_detected);
    // 保存ROI状态到上下文
    s_vision_ctx.roi_state = VisionStrategy_GetROIState();
}

// 获取追踪策略状态结构体指针
VisionStrategy_t* Tracking_GetState(void)
{
    return VisionStrategy_GetState();
}

// ==================== 系统初始化 ====================

// 初始化整个视觉系统
void Vision_Init(void)
{
    // 初始化K230视觉模块串口
    K230_UART_Init(K230_BAUD);
    // 初始化继电器(激光由继电器控制, PE7)
    Relay_Init();
    // 初始化云台PID
    Vision_GimbalPID_Init();
    // 初始化追踪策略
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

// ==================== 视觉主循环(状态机) ====================

// 视觉模式状态机主循环
// 四种模式: 空闲/圆形扫描/追踪射击/追踪+圆形扫描
void Vision_Process(void)
{
    uint8_t fresh_packet = 0U;          // 是否有新数据包
    uint8_t imu_comp_pending = 0U;      // IMU补偿数据是否就绪

    // 未初始化直接返回
    if (!s_initialized) return;

    // 阶段1: IMU前馈补偿更新，获取车身姿态变化
    imu_comp_pending = Vision_UpdateGimbalIMUCompensation();

    // 阶段2: K230数据解析 — 接收串口数据→解析协议包→提取目标误差
    if (k230_rx_flag) {
        // 解析K230数据包
        K230_ParsePacket();
        // 标记有新数据包
        fresh_packet = 1U;
    }
    // 保存上一帧新包标志
    s_last_fresh_packet = fresh_packet;

    // 阶段3: 空闲模式 — 停止所有输出，仅保持IMU补偿
    if (s_eye_mode == VISION_MODE_IDLE) {
        // 重置圆形扫描相位
        s_circle_phase = 0.0f;
        // 清除目标处理
        Vision_TargetProcess(0, 0, false);
        // 关闭激光
        Vision_LaserTrigger(false);
        // 更新追踪状态(未检测到目标)
        Tracking_Update(0U);
    }

    // 阶段4: 收到新数据包且非空闲模式，执行核心控制逻辑
    if (fresh_packet && s_eye_mode != VISION_MODE_IDLE) {
        uint8_t target_valid = k230_parsed.track_valid ? 1U : 0U;  // 目标是否有效
        float control_x = 0.0f;     // 最终X控制量
        float control_y = 0.0f;     // 最终Y控制量
        float circle_x = 0.0f;      // 圆形扫描X偏移
        float circle_y = 0.0f;      // 圆形扫描Y偏移

        // 子阶段4.1: ROI状态机更新 — 根据目标有无自动切换ROI大小
        Tracking_Update(target_valid);

        // 子阶段4.2: 圆形扫描模式额外叠加圆形偏移
        if (Vision_ModeCircles()) {
            Vision_CircleOffset(&circle_x, &circle_y);
        }

        // 子阶段4.3: 目标有效且射击模式 — K230误差 + 圆形偏移 → PID控制 → 激光触发
        if (target_valid && Vision_ModeShoots()) {
            // 读取K230输出的X方向误差
            control_x = (float)k230_parsed.err_y;
            // 读取K230输出的Y方向误差
            control_y = (float)k230_parsed.err_z;
            // 叠加圆形扫描X偏移
            control_x += circle_x;
            // 叠加圆形扫描Y偏移
            control_y += circle_y;
            // 执行云台控制，允许激光
            Vision_ApplyControl(control_x, control_y, 1U, 1U);
        } else if (Vision_ModeCircles()) {
            // 子阶段4.4: 纯圆形扫描 — 无目标，按圆形轨迹运动
            Vision_ApplyControl(circle_x, circle_y, 0U, 0U);
        } else {
            // 子阶段4.5: 有数据包但无目标且非圆形模式 — 停止追踪
            Vision_TargetProcess(0, 0, false);
            Vision_LaserTrigger(false);

            // 全图模式下启动蛇形扫描搜索目标
            if (VisionStrategy_GetROIState() == ROI_FULL) {
                VisionStrategy_GimbalScan();
            }
        }
    } else if (!fresh_packet && s_eye_mode != VISION_MODE_IDLE && Vision_ModeCircles()) {
        // 阶段5: 无新数据包但圆形扫描模式 — 继续执行圆形扫描
        Vision_RunCircleOnly();
    }

    // 阶段6: IMU补偿数据就绪时，执行补偿控制(丢包维持)
    if (imu_comp_pending && s_eye_mode != VISION_MODE_IDLE) {
        Vision_GimbalIMUCompensationTick();
    }

//    Vision_DebugPrint();
}

// 获取视觉上下文状态结构体指针
Vision_Context_t* Vision_GetContext(void)
{
    return &s_vision_ctx;
}
