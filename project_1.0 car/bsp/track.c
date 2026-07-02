#include "track.h"
#include "encoder.h"
#include "tb6612.h"
#include "board.h"
#include "bno080.h"
#include "uart3.h"
#include "uart_k230.h"
#include "corner_profile.h"
#include "right_angle_detector.h"
#include <math.h>
#include "peripheral.h"
#include "stdio.h"

// 灰度传感器实例结构体，存储8通道灰度传感器的校准值和模拟量数据
GrayscaleSensor_t gray_sensor;

// 8路灰度传感器的权重数组，用于加权计算巡线偏差中心
int32_t  ir_weight[8];
// 8路灰度传感器的数字量原始数据（0=黑线，1=白线）
uint8_t  ir_raw[8];
// 是否检测到黑线标志（0=全白无线，1=检测到黑线）
uint8_t  has_black;

// PID控制器当前偏差值
float    PID_Err       = 0.0f;
// PID控制器上一拍偏差值（用于微分项计算）
float    PID_LastErr   = 0.0f;
// PID控制器积分累积值
float    PID_SumErr    = 0.0f;
// PID控制器输出值（PWM单位）
int16_t  PID_Output    = 0;
// PID输出低通滤波后的值
float    pid_filtered_output = 0.0f;
// 上一次有效的偏差值（全白时保持使用）
int32_t  last_valid_err = 0;
// 偏差值低通滤波后的值
float    err_filtered   = 0.0f;


// 左轮基础目标转速（RPM），由上层控制算法给出
volatile float  Left_Base_RPM    = 0.0f;
// 右轮基础目标转速（RPM），由上层控制算法给出
volatile float  Right_Base_RPM   = 0.0f;
// 左轮最终PWM输出值（速度环输出）
volatile int    Left_Final_PWM   = 0;
// 右轮最终PWM输出值（速度环输出）
volatile int    Right_Final_PWM  = 0;
// 左轮当前实际转速（RPM），由编码器换算得到
volatile float  Left_Speed_RPM   = 0.0f;
// 右轮当前实际转速（RPM），由编码器换算得到
volatile float  Right_Speed_RPM  = 0.0f;
// 速度环使能标志（0=禁用，1=使能）
volatile uint8_t Velocity_Loop_Enable = 0;
// 巡线运行使能标志（0=停止，1=运行）
volatile uint8_t Track_Run_Enable = 0U;
// 目标圈数设置
volatile uint16_t Track_Target_Laps = 1U;
// 已完成圈数计数
volatile uint16_t Track_Completed_Laps = 0U;
// 当前圈已过弯道计数
volatile uint16_t Track_Corner_Count = 0U;

// 是否处于弯道中的标志（0=直道，1=弯道）
static uint8_t  is_in_corner    = 0;
// 转弯角度进度（0~1，0=未开始，1=完成）
static float    angle_progress  = 0.0f;

// 上一次目标基础转速（调试用）
static float g_last_target_base = 0.0f;
// 上一次陀螺仪差速修正值（调试用）
static float g_last_gyro_diff = 0.0f;
// 上一次PID修正值（调试用）
static float g_last_pid_correction = 0.0f;
// 上一次转弯角度进度（调试用）
static float g_last_angle_progress = 0.0f;

// 直角转弯检测结果（0=无，1=左转，2=右转）
uint32_t is_right_angle              = 0;
// 直角检测状态机当前阶段（0=空闲，1=预触发，2=IMU主体，3=灰度渐变接管）
uint32_t right_angle_phase           = 0;
// 直角初始触发标志（特征首次确认时记录）
uint32_t right_angle_initial_flag    = 0;
// 直角检测类型（左转/右转）
uint32_t right_angle_detect_type     = 0;
// 直角检测器实例结构体
static RightAngleDetector_t right_angle_detector;
// 直角检测器滤波后的8位灰度数据
static uint8_t right_angle_filtered_bits = 0xFFU;

// 左轮编码器累计脉冲数（累加统计用）
volatile int32_t enc_acc_L = 0;
// 右轮编码器累计脉冲数（累加统计用）
volatile int32_t enc_acc_R = 0;
// 定时器7心跳计数（速度环运行标志）
volatile uint32_t tim7_heartbeat = 0;
// 巡线调试快照结构体（存储调试信息）
volatile TrackDebugSnapshot_t g_track_debug;

// 系统运行时间（毫秒），由1ms定时器递增
volatile uint32_t system_time_ms = 0;

// 1ms任务执行标志（由1ms中断置位，主循环清零）
volatile uint32_t flag_1ms = 0;
// 5ms任务执行标志（由1ms中断分频置位，主循环清零）
volatile uint32_t flag_5ms = 0;

// 左轮速度环积分项
float Left_Integral     = 0.0f;
// 右轮速度环积分项
float Right_Integral    = 0.0f;
// 左轮编码器滤波值（低通滤波后）
float Left_FilteredEnc  = 0.0f;
// 右轮编码器滤波值（低通滤波后）
float Right_FilteredEnc = 0.0f;
// 左轮速度环上一拍偏差（用于微分项计算）
float Left_LastBias     = 0.0f;
// 右轮速度环上一拍偏差（用于微分项计算）
float Right_LastBias    = 0.0f;

// 陀螺仪偏航角（度），经方向处理后的当前车头朝向
volatile float   gyro_yaw_deg       = 0.0f;
// 陀螺仪yaw数据可用标志（0=无数据，1=有新数据）
volatile uint32_t gyro_yaw_available = 0;

// 陀螺仪轮询请求标志（由定时器置位，主循环读取后清零）
volatile uint32_t gyro_poll_request = 0;
// 陀螺仪轮询定时器计数（毫秒）
uint16_t gyro_poll_timer_ms = 0;

// 弯道起始时的陀螺仪偏航角（进入转弯时记录）
float gyro_corner_start_yaw  = 0.0f;
// 弯道目标偏航角（计算出的转弯目标角度）
float gyro_corner_target_yaw = 0.0f;
// 上一拍陀螺仪偏航角
float gyro_last_yaw          = 0.0f;
// 陀螺仪偏航角加速度（度/秒^2）
volatile float gyro_yaw_accel = 0.0f;
// 陀螺仪偏航角速度（度/秒）
volatile float gyro_yaw_rate  = 0.0f;
// 陀螺仪首帧有效数据标志（0=未初始化，1=已获取首帧）
uint32_t gyro_first_valid     = 0;

// 陀螺仪数据帧计数（成功读取的帧数）
volatile uint16_t gyro_frame_count = 0;
// 陀螺仪中断触发计数
volatile uint16_t gyro_isr_count   = 0;
// BNO080成功读取计数
volatile uint16_t bno080_ok_count  = 0;
// 陀螺仪原始偏航角（未经过方向处理）
volatile float gyro_raw_yaw   = 0.0f;
// 陀螺仪原始横滚角
volatile float gyro_raw_roll  = 0.0f;
// 陀螺仪原始俯仰角
volatile float gyro_raw_pitch = 0.0f;

// BNO080初始化成功标志
volatile uint16_t bno080_ok = 0;

// 浮点型限幅函数：将val限制在[-limit, limit]范围内
static inline float f_clamp(float val, float limit) {
    // 超过上限则返回上限
    if (val >  limit) return  limit;
    // 低于下限则返回下限
    if (val < -limit) return -limit;
    // 在范围内直接返回原值
    return val;
}

// 单位区间限幅函数：将val限制在[0, 1]范围内
static inline float f_clamp01(float val) {
    if (val > 1.0f) return 1.0f;
    if (val < 0.0f) return 0.0f;
    return val;
}

// 整型限幅函数：将val限制在[-limit, limit]范围内
static inline int i_clamp(int val, int limit) {
    // 超过上限则返回上限
    if (val >  limit) return  limit;
    // 低于下限则返回下限
    if (val < -limit) return -limit;
    // 在范围内直接返回原值
    return val;
}

// 计算两个角度的差值（处理-180~180度环绕问题，保证差值在±180度以内）
// 参数：target-目标角度，current-当前角度（单位：度）
// 返回值：角度差值（-180~180度）
static float Track_Angle_Delta_Deg(float target, float current)
{
    // 计算原始角度差
    float delta = target - current;
    // 差值大于180度，减去360度等效到负方向
    while (delta > 180.0f) delta -= 360.0f;
    // 差值小于-180度，加上360度等效到正方向
    while (delta < -180.0f) delta += 360.0f;
    // 返回归一化后的角度差
    return delta;
}

// IMU直角转弯力道包络：已转0/90度附近弱，45度附近最强
static float Track_Corner_Imu_ForceScale(float turned_deg)
{
    float center = CORNER_IMU_MID_DEG;
    float min_scale = f_clamp01(CORNER_IMU_FORCE_MIN_SCALE);
    float x;
    float envelope;

    if (center <= 0.0f) {
        return 1.0f;
    }

    if (turned_deg < 0.0f) {
        turned_deg = -turned_deg;
    }

    x = (turned_deg - center) / center;
    envelope = 1.0f - (x * x);
    envelope = f_clamp01(envelope);

    return min_scale + (1.0f - min_scale) * envelope;
}

// 最后10度灰度渐变接管：开始为0，退出角时为1
static float Track_Corner_GrayBlend(float turned_deg)
{
    float width = CORNER_IMU_EXIT_DEG - CORNER_GRAY_BLEND_START_DEG;

    if (width <= 0.0f) {
        return 1.0f;
    }

    return CornerProfile_Smoothstep5(
        (turned_deg - CORNER_GRAY_BLEND_START_DEG) / width);
}

// 向K230视觉模块发送当前转弯状态（状态变化时立即发送，转弯中周期发送）
static void Track_Send_Corner_State_To_Vision(void)
{
    // 上一次发送的状态（用于检测状态变化）
    static uint8_t last_state = 0xFFU;
    // 空闲状态是否已发送过标志（保证空闲时只发送一次）
    static uint8_t sent_idle = 0;
    // 上一次发送的时间戳（用于周期发送计时）
    static uint32_t last_send_ms = 0;
    // 当前状态机阶段
    uint8_t phase = right_angle_phase;
    // 转弯类型（取当前转弯状态或初始触发标志）
    uint8_t type = is_right_angle ? is_right_angle : right_angle_initial_flag;
    // 状态编码：低2位=phase，bit2=type(左右转)，bit3=陀螺仪就绪
    uint8_t state = phase & 0x03U;
    // 是否需要发送标志
    uint8_t should_send = 0;

    // 轮询K230串口发送状态，推进发送缓冲区
    K230_PollCornerStateTx();

    // 右转类型设置bit2标志位
    if (type == 2U) state |= 0x04U;
    // bit3恒置1（陀螺仪就绪标志）
    state |= 0x08U;

    // 非空闲状态时重置空闲发送标志
    if (right_angle_phase != 0U || type != 0U) {
        sent_idle = 0;
    }

    // 发送时机判断：状态变化时立即发送，或转弯中每50ms周期性发送，或空闲时发送一次
    // 状态变化：立即发送
    if (state != last_state) {
        should_send = 1;
    }
    // 转弯进行中：每50ms周期性发送一次
    else if (phase != 0U && (uint32_t)(system_time_ms - last_send_ms) >= 50U) {
        should_send = 1;
    }
    // 刚进入空闲状态：发送一次空闲通知
    else if (phase == 0U && type == 0U && !sent_idle) {
        should_send = 1;
        sent_idle = 1;
    }

    // 不需要发送则直接返回
    if (!should_send) return;

    // 将状态写入发送队列
    K230_QueueCornerState(state);
    // 立即触发发送
    K230_PollCornerStateTx();
    // 记录本次发送的状态和时间
    last_state = state;
    last_send_ms = system_time_ms;
}

// 复位直角转弯状态机：将所有直角检测相关状态变量清零
static void Track_Reset_Right_Angle_State(void)
{
    // 清除直角转弯标志
    is_right_angle          = 0;
    // 状态机回到空闲阶段
    right_angle_phase       = 0;
    // 清除初始触发标志
    right_angle_initial_flag = 0;
    // 清除检测类型
    right_angle_detect_type = 0;
}

// 复位巡线控制状态：将所有PID、转速、滤波等控制变量清零
static void Track_Reset_Control_State(void)
{
    // 复位PID偏差相关变量
    PID_Err = 0.0f;
    PID_LastErr = 0.0f;
    PID_SumErr = 0.0f;
    PID_Output = 0;
    pid_filtered_output = 0.0f;
    err_filtered = 0.0f;
    // 复位调试用历史变量
    g_last_target_base = 0.0f;
    g_last_gyro_diff = 0.0f;
    g_last_pid_correction = 0.0f;
    g_last_angle_progress = 0.0f;
    // 复位左右轮目标转速
    Left_Base_RPM = 0.0f;
    Right_Base_RPM = 0.0f;
    // 复位最终PWM输出
    Left_Final_PWM = 0;
    Right_Final_PWM = 0;
    // 复位速度环积分项
    Left_Integral = 0.0f;
    Right_Integral = 0.0f;
    // 复位编码器滤波值
    Left_FilteredEnc = 0.0f;
    Right_FilteredEnc = 0.0f;
    // 复位速度环上一拍偏差
    Left_LastBias = 0.0f;
    Right_LastBias = 0.0f;
    // 清除弯道中标志
    is_in_corner = 0U;
}

// 启动巡线控制：使能运行标志，复位计数和状态，启动速度环
void Track_ControlStart(void)
{
    // 置位巡线运行使能标志
    Track_Run_Enable = 1U;
    // 已完成圈数清零
    Track_Completed_Laps = 0U;
    // 弯道计数清零
    Track_Corner_Count = 0U;
    // 复位直角检测状态机
    Track_Reset_Right_Angle_State();
    // 复位所有控制状态变量
    Track_Reset_Control_State();
    // 使能速度环
    Velocity_Loop_Enable = 1U;
}

// 停止巡线控制：禁用运行标志，复位所有状态，电机停车
void Track_ControlStop(void)
{
    // 清除巡线运行使能标志
    Track_Run_Enable = 0U;
    // 禁用速度环
    Velocity_Loop_Enable = 0U;
    // 复位直角检测状态机
    Track_Reset_Right_Angle_State();
    // 复位所有控制状态变量
    Track_Reset_Control_State();
    // 电机驱动输出0，停车
    TB6612_SetSpeed(0, 0);
}

// 目标圈数加1：增加设定的目标圈数（有上限保护）
void Track_TargetLapAdd(void)
{
    // 未达到最大圈数时，目标圈数加1
    if (Track_Target_Laps < TRACK_TARGET_LAPS_MAX) {
        Track_Target_Laps++;
    }
}

// 目标圈数减1：减少设定的目标圈数（最小为1圈）
void Track_TargetLapSub(void)
{
    // 大于1圈时，目标圈数减1
    if (Track_Target_Laps > 1U) {
        Track_Target_Laps--;
    }
}

// 注册弯道完成：弯道计数+1，满一圈弯道数则圈数+1，达到目标圈数则停车
// 功能：每完成一个直角弯道调用一次，累计圈数并判断是否结束
static void Track_RegisterCornerComplete(void)
{
    // 未使能运行时直接返回
    if (!Track_Run_Enable) return;

    // 弯道计数自增，达到每圈弯道数时圈数自增
    if (++Track_Corner_Count >= TRACK_CORNERS_PER_LAP) {
        // 弯道计数清零，开始下一圈
        Track_Corner_Count = 0U;
        // 已完成圈数加1（防止溢出）
        if (Track_Completed_Laps < 0xFFFFU) {
            Track_Completed_Laps++;
        }
        // 达到目标圈数时，停止巡线
        if (Track_Completed_Laps >= Track_Target_Laps) {
            Track_ControlStop();
        }
    }
}

// 灰度数据转换：读取灰度传感器数字量，拆解为8通道数组，更新直角检测器
void Track_Gray_Convert(void)
{
    // 读取灰度传感器8位数字量（1位=1个传感器通道）
    uint8_t digital = Grayscale_GetDigital(&gray_sensor);
    uint8_t i;

    // 将8位数字量拆解为8个单通道的二进制数组
    for (i = 0; i < 8; i++) {
        ir_raw[i] = (digital >> i) & 1;
    }

    // 判断是否检测到黑线（8位不全为1表示有黑线）
    has_black = (digital != 0xFF) ? 1 : 0;
    // 更新直角检测器的滤波状态，输入最新的8位灰度数据
    right_angle_filtered_bits = RightAngleDetector_Update(&right_angle_detector,
                                                           digital);
}

// 响应陀螺仪轮询请求：检查并清除轮询标志，触发一次BNO080数据读取
// 返回值：1=执行了读取，0=无请求
uint8_t Track_Gyro_TakePollRequest(void)
{
    // 有轮询请求时
    if (gyro_poll_request) {
        // 清除请求标志
        gyro_poll_request = 0;
        // 执行BNO080数据更新读取
        if (bno080_update()) {
            // 读取成功计数加1
            bno080_ok_count++;
        }
        // 返回有请求
        return 1;
    }
    // 返回无请求
    return 0;
}

// 陀螺仪数据更新：从BNO080读取欧拉角，计算偏航角速度和角加速度
// 功能：首帧初始化基准，后续帧计算速率和加速度，提供给控制系统使用
void Track_Gyro_Update(void)
{
    // 中断调用计数（调试用）
    gyro_isr_count++;

    // 检查BNO080是否有新数据可用
    if (bno080_data_available()) {
        // 定义临时变量存储欧拉角
        float roll, pitch, yaw;
        // 读取BNO080的欧拉角数据
        bno080_get_euler(&roll, &pitch, &yaw);

        // 有效数据帧计数递增
        gyro_frame_count++;

        // --- 保存原始欧拉角（供调试查看）
        gyro_raw_roll  = roll;
        gyro_raw_pitch = pitch;
        gyro_raw_yaw   = yaw;

        // --- 首帧数据处理：初始化基准
        if (!gyro_first_valid) {
            // 根据方向配置处理yaw角（正向或反向）
            gyro_yaw_deg   = (GYRO_YAW_DIRECTION >= 0) ? yaw : -yaw;
            // 角速度初始化为0
            gyro_yaw_rate  = 0.0f;
            // 角加速度初始化为0
            gyro_yaw_accel = 0.0f;
            // 保存上一拍yaw（首帧时等于当前值）
            gyro_last_yaw    = gyro_yaw_deg;
            // 标记yaw数据可用
            gyro_yaw_available = 1;
            // 标记首帧已处理
            gyro_first_valid = 1;
        }
        // --- 后续帧处理：计算角速度和角加速度
        else {
            // 根据方向配置处理新的yaw角
            float new_yaw = (GYRO_YAW_DIRECTION >= 0) ? yaw : -yaw;
            // 计算当前角速度（新yaw - 旧yaw，处理角度环绕）
            float yaw_rate_now = Track_Angle_Delta_Deg(new_yaw, gyro_yaw_deg);
            // 计算角加速度（当前角速度 - 上一拍角速度）
            gyro_yaw_accel = yaw_rate_now - gyro_yaw_rate;
            // 更新角速度为当前值
            gyro_yaw_rate  = yaw_rate_now;

            // 保存上一拍yaw角
            gyro_last_yaw = gyro_yaw_deg;

            // 更新当前yaw角为新值
            gyro_yaw_deg = new_yaw;
            // 标记有新数据可用
            gyro_yaw_available = 1;
        }
    }
}

// 计算巡线偏差：基于8路灰度传感器加权平均，得到黑线中心相对于传感器中心的偏移
// 返回值：偏差值（负值表示偏左，正值表示偏右）
int32_t Track_Calc_Err(void)
{
    // 加权和（分子）：各通道黑度 × 权重
    int32_t sum_w = 0;
    // 黑度和（分母）：所有通道黑度总和
    int32_t sum_b = 0;
    uint8_t i;

    // --- 第一步：遍历8路灰度传感器，计算加权和与黑度和
    for (i = 0; i < 8; i++) {
        // 当前通道ADC采样值
        int32_t adc = (int32_t)gray_sensor.Analog_value[i];
        // 当前通道白参考值（校准时的白值）
        int32_t ref = (int32_t)gray_sensor.Calibrated_white[i];
        // 白参考为0时跳过（防止除零）
        if (ref == 0) continue;
        // 计算当前值相对于白值的百分比（放大10000倍，避免浮点运算）
        int32_t pct = (adc * 10000) / ref;
        // 限制最大值（超过白值时按白值算）
        if (pct > 10000) pct = 10000;
        // 计算黑度（10000 - 白值百分比，越大表示越黑）
        int32_t black = 10000 - pct;
        // 黑度 × 权重，累加到加权和
        sum_w += black * ir_weight[i];
        // 黑度累加
        sum_b += black;
    }

    // 原始偏差值
    int32_t raw_err;

    // --- 第二步：计算偏差中心（加权平均）
    // 全白（没有黑线）时，保持上一次有效偏差
    if (sum_b == 0) {
        raw_err = last_valid_err;
    }
    // 有黑线时，计算加权平均偏差
    else {
        // 偏差 = (加权和 × 1000) / 黑度和 （放大1000倍提高精度）
        raw_err = (int32_t)(((long long)sum_w * 1000) / sum_b);
        // 保存为最后一次有效偏差
        last_valid_err = raw_err;
    }

    // --- 第三步：直道时低通滤波，抑制高频抖动
    // 非转弯状态下对偏差做一阶低通滤波
    if (is_right_angle == 0) {
        // 一阶RC低通滤波公式：新输出 = α*新值 + (1-α)*旧输出
        err_filtered = ERR_FILTER_ALPHA * (float)raw_err + (1.0f - ERR_FILTER_ALPHA) * err_filtered;
        // 四舍五入转换为整型
        raw_err = (int32_t)(err_filtered + 0.5f);
    }

    // 返回最终偏差值
    return raw_err;
}

// 直角检测状态机：检测直角入口并按IMU角度推进转弯过程
// 流程：特征锁定 → 全白入口 → IMU主体 → 最后10度灰度渐变 → 85度提前退出
void Track_Check_Right_Angle(void)
{
    // ==============================================
    // phase 0: 空闲检测，锁定直角方向
    // ==============================================
    if (is_right_angle == 0 && right_angle_phase == 0U)
    {
        uint8_t feature = RightAngleDetector_ConfirmedFeature(&right_angle_detector);

        if (feature != 0U) {
            right_angle_initial_flag = feature;
            right_angle_detect_type = feature;
            right_angle_phase = 1U;
        }
    }

    // ==============================================
    // phase 1: 等待全白入口，随后锁定IMU起始角
    // ==============================================
    else if (is_right_angle == 0 && right_angle_phase == 1U)
    {
        uint8_t filtered_feature = RightAngleDetector_Feature(right_angle_filtered_bits);

        if (right_angle_initial_flag != 0U &&
            RightAngleDetector_AllWhite(right_angle_filtered_bits) &&
            RightAngleDetector_WhiteConfirmed(&right_angle_detector)) {
            is_right_angle = right_angle_initial_flag;
            right_angle_detect_type = right_angle_initial_flag;
            right_angle_phase = 2U;
            gyro_corner_start_yaw = gyro_yaw_deg;
            if (is_right_angle == 1U) {
                gyro_corner_target_yaw = gyro_corner_start_yaw - CORNER_YAW_TARGET;
            } else {
                gyro_corner_target_yaw = gyro_corner_start_yaw + CORNER_YAW_TARGET;
            }
            is_in_corner = 1U;
        } else if (right_angle_initial_flag != 0U &&
                   filtered_feature == 0U &&
                   !RightAngleDetector_AllWhite(right_angle_filtered_bits)) {
            Track_Reset_Right_Angle_State();
        }
    }

    // ==============================================
    // phase 2: IMU二次PID主体，转到75度进入灰度渐变区
    // ==============================================
    else if (is_right_angle != 0 && right_angle_phase == 2U)
    {
        float yaw_diff = fabsf(Track_Angle_Delta_Deg(gyro_yaw_deg,
                                                     gyro_corner_start_yaw));

        if (yaw_diff >= CORNER_GRAY_BLEND_START_DEG) {
            right_angle_phase = 3U;
        }
    }

    // ==============================================
    // phase 3: 最后10度灰度渐变接管，85度提前退出
    // ==============================================
    else if (is_right_angle != 0 && right_angle_phase == 3U)
    {
        float yaw_diff = fabsf(Track_Angle_Delta_Deg(gyro_yaw_deg,
                                                     gyro_corner_start_yaw));

        if (yaw_diff < CORNER_IMU_EXIT_DEG) {
            return;
        }

        Track_Reset_Right_Angle_State();
        RightAngleDetector_Init(&right_angle_detector);
        right_angle_filtered_bits = 0xFFU;

        Left_Integral     = 0.0f;
        Right_Integral    = 0.0f;
        Left_FilteredEnc  = 0.0f;
        Right_FilteredEnc = 0.0f;

        is_in_corner = 0U;
        Track_RegisterCornerComplete();
    }
}

// 巡线PID控制器计算：根据偏差计算PID输出，用于方向控制
// 参数：err-巡线偏差值（灰度加权中心偏差）
void Track_PID_Calc(int32_t err)
{
    // --- 偏差更新与积分累积
    // 保存当前偏差到全局偏差变量
    PID_Err = (float)err;
    // 积分项累积：将当前偏差累加到积分总和
    PID_SumErr += PID_Err;

    // --- PID三项计算：标准位置式PID
    // 比例项(P)：当前偏差 × 比例系数KP
    // 积分项(I)：累积积分总和 × 积分系数KI
    // 微分项(D)：偏差变化量(当前-上次) × 微分系数KD
    float output_raw = KP_NORMAL * PID_Err
                     + KI_NORMAL * PID_SumErr
                     + KD_NORMAL * (PID_Err - PID_LastErr);

    // --- 输出限幅：限制PID输出最大值，防止PWM超调
    output_raw = f_clamp(output_raw, STRAIGHT_RPM_LIMIT * 10.0f);

    // --- 输出低通滤波：一阶RC低通，平滑输出减少高频抖动
    // 新值权重=滤波系数*当前输出 + (1-滤波系数)*上次滤波输出
    pid_filtered_output = PID_FILTER_ALPHA * output_raw
                        + (1.0f - PID_FILTER_ALPHA) * pid_filtered_output;

    // --- 输出更新与状态保存
    // 滤波后输出转换为整型，作为最终PID输出
    PID_Output  = (int16_t)pid_filtered_output;
    // 保存当前偏差为上一拍偏差，供下次微分项计算用
    PID_LastErr = PID_Err;
}

// 动作执行器：根据当前状态（直道/预触发/转弯中）合成左右轮目标转速
// 直角主体只用二次函数IMU PID，最后10度灰度渐变接管
void Track_Action_Execute(void)
{
    // --- 局部变量定义与初始化
    // 目标基础转速（默认直道基础转速）
    float target_base = BASE_RPM;
    // 陀螺仪差速修正量（左轮减、右轮加，用于转向差速）
    float gyro_diff   = 0.0f;
    // PID修正量（灰度巡线的PID输出修正）
    float pid_correction = 0.0f;
    // PID输出转换为RPM单位（原始输出×0.1系数缩放）
    float raw_pid = (float)PID_Output * 0.1f;

    // ==============================================
    // 状态分支：预触发，继续正常巡线，不额外减速
    // ==============================================
    if (right_angle_phase == 1U) {
        is_in_corner = 0U;
        target_base = BASE_RPM;
        gyro_diff = 0.0f;
        pid_correction = raw_pid;
    }

    // ==============================================
    // 状态分支：转弯中（IMU PID二次包络 + 末段灰度渐变）
    // ==============================================
    else if (is_right_angle != 0 &&
             (right_angle_phase == 2U || right_angle_phase == 3U)) {
        float yaw_diff = 0.0f;
        float gray_blend = 0.0f;
        float imu_diff_limit = 0.0f;
        is_in_corner = 1U;

        yaw_diff = fabsf(Track_Angle_Delta_Deg(gyro_yaw_deg, gyro_corner_start_yaw));
        if (right_angle_phase == 3U) {
            gray_blend = Track_Corner_GrayBlend(yaw_diff);
        }

        // 基础转速保持不变，避免明显加速/减速
        target_base = CORNER_TURN_RPM;
        imu_diff_limit = CORNER_MAX_RPM - target_base;
        if (imu_diff_limit > (target_base - CORNER_MIN_RPM)) {
            imu_diff_limit = target_base - CORNER_MIN_RPM;
        }
        if (imu_diff_limit < 0.0f) {
            imu_diff_limit = 0.0f;
        }

        {
            float yaw_error = Track_Angle_Delta_Deg(gyro_corner_target_yaw,
                                                    gyro_yaw_deg);
            float force_scale = Track_Corner_Imu_ForceScale(yaw_diff);
            gyro_diff = KP_CORNER_YAW * yaw_error
                      - KD_CORNER_YAW * gyro_yaw_rate;
            gyro_diff *= force_scale * (1.0f - gray_blend);
        }

        gyro_diff = f_clamp(gyro_diff, imu_diff_limit);
        pid_correction = gray_blend * raw_pid;
    }

    // ==============================================
    // 状态分支：正常直道巡线
    // 功能：使用基础转速 + 灰度PID修正
    // ==============================================
    else {
        // 清除弯道中标志
        is_in_corner = 0;
        // 使用直道基础转速
        target_base  = BASE_RPM;
        // 直道无陀螺仪差速修正
        gyro_diff    = 0.0f;
        // 完全使用灰度PID修正
        pid_correction = raw_pid;
    }

    if (is_in_corner) {
        g_last_angle_progress = f_clamp01(
            fabsf(Track_Angle_Delta_Deg(gyro_yaw_deg, gyro_corner_start_yaw)) /
            CORNER_YAW_TARGET);
    } else {
        g_last_angle_progress = 0.0f;
    }
    g_last_gyro_diff = gyro_diff;
    g_last_target_base = target_base;
    g_last_pid_correction = pid_correction;

    // ==============================================
    // 直道陀螺仪阻尼：利用yaw角速度抑制直道车身晃动
    // ==============================================
    if (!is_in_corner) {
        // 直道阻尼功能使能时
        if (GYRO_STRAIGHT_DAMPING_ENABLE) {
            // 阻尼量 = -角速度 × 阻尼系数（负号表示与运动方向相反，起阻尼作用）
            float straight_damping = -KD_GYRO_STRAIGHT * gyro_yaw_rate;
            // 阻尼量限幅
            straight_damping = f_clamp(straight_damping, GYRO_STRAIGHT_LIMIT);
            // 叠加到陀螺仪差速修正量
            gyro_diff += straight_damping;
        }
    }

    // ==============================================
    // 合成左右轮目标转速 + 限幅
    // ==============================================
    // --- 合成左右轮目标转速
    // 左轮 = 基础转速 - 陀螺仪差速(左减) + PID修正(左加)
    // 右轮 = 基础转速 + 陀螺仪差速(右加) - PID修正(右减)
    Left_Base_RPM  = target_base - gyro_diff + pid_correction;
    Right_Base_RPM = target_base + gyro_diff - pid_correction;

    // --- 直道转速限幅：限制在0~最大直道转速范围内
    if (Left_Base_RPM  < 0.0f) Left_Base_RPM  = 0.0f;
    if (Left_Base_RPM  > STRAIGHT_RPM_LIMIT) Left_Base_RPM  = STRAIGHT_RPM_LIMIT;
    if (Right_Base_RPM < 0.0f) Right_Base_RPM = 0.0f;
    if (Right_Base_RPM > STRAIGHT_RPM_LIMIT) Right_Base_RPM = STRAIGHT_RPM_LIMIT;

    // --- 弯道转速限幅：转弯时限制在弯道最小/最大转速范围内
    if (is_in_corner) {
        if (Left_Base_RPM  < CORNER_MIN_RPM) Left_Base_RPM  = CORNER_MIN_RPM;
        if (Left_Base_RPM  > CORNER_MAX_RPM) Left_Base_RPM  = CORNER_MAX_RPM;
        if (Right_Base_RPM < CORNER_MIN_RPM) Right_Base_RPM = CORNER_MIN_RPM;
        if (Right_Base_RPM > CORNER_MAX_RPM) Right_Base_RPM = CORNER_MAX_RPM;
    }
}

// 调试阶段函数（预留接口，当前未实现）
// 参数：time-调试时间参数
void Track_Main_Debug_Stage(uint32_t time)
{
    // 抑制未使用参数警告
    (void)time;
}

// 单电机速度环PID计算：位置式PID，包含编码器滤波、死区、积分限幅
// 参数：
//   encoder_count - 当前编码器脉冲数
//   target_rpm    - 目标转速(RPM)
//   integral      - 积分项指针（输入输出）
//   filtered_enc  - 编码器滤波值指针（输入输出）
//   correction    - 转速修正系数
//   last_bias     - 上一拍偏差指针（输入输出，用于微分项）
// 返回值：PID计算后的PWM输出值
int Track_Velocity_PI(int encoder_count, float target_rpm,
                      float *integral, float *filtered_enc,
                      float correction, float *last_bias)
{
    float target_pulse, bias, pwm_out;

    // --- 目标转速转换：RPM → 每采样周期目标脉冲数
    // 目标转速乘以修正系数
    target_rpm  = target_rpm * correction;
    // 目标脉冲数 = 目标转速 × 每转脉冲数 × 采样时间 / 60秒
    target_pulse = (target_rpm * (float)PULSE_PER_ROUND * SAMPLE_TIME) / 60.0f;

    // --- 编码器值低通滤波：一阶滤波，α=0.5
    *filtered_enc = 0.5f * (float)encoder_count + 0.5f * (*filtered_enc);

    // --- 计算偏差：目标脉冲数 - 滤波后编码器值
    bias = target_pulse - (*filtered_enc);

    // --- 积分项更新（带死区和限幅）
    // 死区：偏差绝对值大于1时才积分，防止小偏差导致积分饱和
    if (fabsf(bias) > 1.0f) {
        // 积分累加
        *integral += KI_VELOCITY * bias;
        // 积分限幅：防止积分项过大
        *integral = f_clamp(*integral, (float)INTEGRAL_LIMIT);
    }

    // --- PID输出计算：比例项 + 积分项 + 微分项
    pwm_out = KP_VELOCITY * bias + *integral + KD_VELOCITY * (bias - *last_bias);
    // 保存当前偏差供下次微分项使用
    *last_bias = bias;

    // --- 输出限幅后返回
    return i_clamp((int)pwm_out, VELOCITY_PWM_LIMIT);
}

// TIM11定时器初始化：配置为1ms周期中断，提供系统时基
void Track_TIM11_Init(void)
{
    // 定时器时基初始化结构体
    TIM_TimeBaseInitTypeDef  tim_base;
    // 嵌套向量中断控制器初始化结构体
    NVIC_InitTypeDef         nvic;

    // 使能TIM11外设时钟（APB2总线）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);

    // 预分频器：168MHz / 168 = 1MHz计数频率（1us计数一次）
    tim_base.TIM_Prescaler     = 168 - 1;
    // 自动重装载值：计数1000次产生一次更新中断，即1ms周期
    tim_base.TIM_Period        = 1000 - 1;
    // 时钟分频：不分频，使用定时器时钟源直接计数
    tim_base.TIM_ClockDivision = TIM_CKD_DIV1;
    // 计数模式：向上计数（从0计数到重装载值）
    tim_base.TIM_CounterMode   = TIM_CounterMode_Up;
    // 将配置写入TIM11定时器时基寄存器
    TIM_TimeBaseInit(TIM11, &tim_base);

    // 配置中断通道：TIM1触发和通信中断、TIM11全局中断
    nvic.NVIC_IRQChannel                   = TIM1_TRG_COM_TIM11_IRQn;
    // 抢占优先级：0级（最高优先级）
    nvic.NVIC_IRQChannelPreemptionPriority  = 0;
    // 子优先级：0级
    nvic.NVIC_IRQChannelSubPriority         = 0;
    // 使能该中断通道
    nvic.NVIC_IRQChannelCmd                 = ENABLE;
    // 将NVIC配置写入中断控制器
    NVIC_Init(&nvic);

    // 使能TIM11更新中断（计数溢出时触发）
    TIM_ITConfig(TIM11, TIM_IT_Update, ENABLE);
    // 启动TIM11定时器开始计数
    TIM_Cmd(TIM11, ENABLE);
}

// TIM11更新中断服务函数：1ms定时中断，提供系统时基和任务调度标志
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
    // 判断是否为TIM11更新中断触发
    if (TIM_GetITStatus(TIM11, TIM_IT_Update) != RESET) {
        // 清除TIM11更新中断标志位，防止重复进入
        TIM_ClearITPendingBit(TIM11, TIM_IT_Update);
        // 系统时间计数器自增（毫秒级时基）
        system_time_ms++;
        // 置位1ms任务执行标志，主循环检测后执行1ms任务
        flag_1ms = 1;

        // 陀螺仪轮询定时：每GYRO_MAIN_POLL_PERIOD_MS毫秒触发一次读取请求
        gyro_poll_timer_ms++;
        // 达到轮询周期时，清零计数器并触发读取请求
        if (gyro_poll_timer_ms >= GYRO_MAIN_POLL_PERIOD_MS) {
            gyro_poll_timer_ms = 0;
            gyro_poll_request = 1;
        }

        // 5ms任务分频：每5次1ms中断产生一次5ms任务标志
        {
            // 静态分频计数器，记录1ms中断次数
            static uint8_t ms5_cnt = 0;
            // 计数器自增，达到5次时清零并置位5ms标志
            if (++ms5_cnt >= 5) {
                ms5_cnt = 0;
                flag_5ms = 1;
            }
        }
    }
}

// 1ms主任务：传感器采集 → 状态机 → PID计算 → 动作输出
// 功能：巡线控制主循环，每1ms执行一次方向环控制
void Track_Main_1ms(void)
{
    // 1ms标志未置位时直接返回
    if (!flag_1ms) return;
    // 清除1ms标志
    flag_1ms = 0;

    // --- 传感器数据采集
    // 灰度传感器数据转换：读取数字量、拆解通道、更新直角检测器
    Track_Gray_Convert();

    // 陀螺仪数据更新：读取新数据、计算角速度和角加速度
    Track_Gyro_Update();

    // --- 运行状态判断：未使能时复位状态并发送状态
    if (!Track_Run_Enable) {
        // 复位直角检测状态机
        Track_Reset_Right_Angle_State();
        // 复位控制状态
        Track_Reset_Control_State();
        // 发送转弯状态给视觉模块
        Track_Send_Corner_State_To_Vision();
        return;
    }

    // --- 直角检测状态机处理
    Track_Check_Right_Angle();

    // 发送转弯状态给视觉模块
    Track_Send_Corner_State_To_Vision();

    // 再次检查运行使能（状态机中可能触发停车）
    if (!Track_Run_Enable) {
        return;
    }

    // --- 巡线控制：偏差计算 → PID计算
    {
        // 计算巡线偏差
        int32_t err = Track_Calc_Err();
        // 保存偏差到全局变量
        PID_Err = (float)err;
        // PID控制器计算
        Track_PID_Calc(err);
    }

    // --- 动作执行：合成左右轮目标转速
    Track_Action_Execute();
}

// 5ms主任务：速度环控制
// 功能：每5ms执行一次速度环PID计算和电机输出
void Track_Main_5ms(void)
{
    // 5ms标志未置位时直接返回
    if (!flag_5ms) return;
    // 清除5ms标志
    flag_5ms = 0;
    // 执行速度环控制
    Track_SpeedLoop();
}

// 陀螺仪主任务：轮询读取陀螺仪数据
// 功能：调用轮询请求函数，触发BNO080数据读取
void Track_Main_Gyro(void)
{
    // 调用陀螺仪轮询请求处理（忽略返回值）
    (void)Track_Gyro_TakePollRequest();
}

// 调试输出函数：定时输出陀螺仪欧拉角和四元数数据到串口
// 参数：time-输出间隔时间（毫秒）
void Track_Main_Debug(uint32_t time)
{
    // 上次输出时间记录
    static uint32_t print_timer = 0;
    // 未到输出时间直接返回
    if ((int32_t)(system_time_ms - print_timer) < time) return;
    // 更新上次输出时间
    print_timer = system_time_ms;

    // 定义欧拉角和四元数变量
    float roll_deg, pitch_deg, yaw_deg;
    const BNO080_RotationVector_t *qv;
    // 输出缓冲区
    char buf[160];
    int len;
    // 读取欧拉角
    bno080_get_euler(&roll_deg, &pitch_deg, &yaw_deg);
    // 读取旋转矢量（四元数）
    qv = bno080_get_rotation_vector();
    // 格式化输出字符串
    len = sprintf(buf,
        "R:%+7.2f P:%+7.2f Y:%+7.2f  Q(wxyz):%+.4f %+.4f %+.4f %+.4f\r\n",
        roll_deg, pitch_deg, yaw_deg,
        qv->real, qv->i, qv->j, qv->k);
    // 通过串口3逐字节发送
    { int i; for (i = 0; i < len; i++) USART3_SendByte((uint8_t)buf[i]); }
}

// 速度环控制：编码器读取 → 速度换算 → PI控制 → PWM输出
// 功能：左右轮独立速度闭环控制，跟踪上层给定的目标转速
void Track_SpeedLoop(void)
{
    // --- 心跳计数：标记速度环运行状态
    tim7_heartbeat++;

    // --- 第一步：读取编码器原始脉冲数
    Encoder_ReadAll();

    // 累加左右轮编码器脉冲（累计统计用）
    enc_acc_L += Encoder_Left;
    enc_acc_R += Encoder_Right;

    // --- 第二步：编码器方向修正（根据硬件安装配置）
    int16_t enc_l = Encoder_Left;
    int16_t enc_r = Encoder_Right;
    // 左轮编码器方向翻转配置
    if (ENCODER_LEFT_INVERT)  enc_l = -enc_l;
    // 右轮编码器方向翻转配置
    if (ENCODER_RIGHT_INVERT) enc_r = -enc_r;

    // --- 第三步：脉冲数换算为转速(RPM)，并限幅防止异常值
    Left_Speed_RPM  = (float)enc_l * RPM_COEFFICIENT;
    Right_Speed_RPM = (float)enc_r * RPM_COEFFICIENT;
    // 转速限幅0~400RPM，防止异常值
    Left_Speed_RPM  = f_clamp(Left_Speed_RPM,  400.0f);
    Right_Speed_RPM = f_clamp(Right_Speed_RPM, 400.0f);

    // --- 第四步：速度环PI控制计算
    if (Velocity_Loop_Enable) {
        // 左轮速度PI控制：输入编码器脉冲、目标转速、积分项、滤波值、修正系数、上一拍偏差
        Left_Final_PWM  = Track_Velocity_PI(enc_l,  Left_Base_RPM,
                                            &Left_Integral,  &Left_FilteredEnc,
                                            LEFT_RPM_CORRECTION,  &Left_LastBias);
        // 右轮速度PI控制
        Right_Final_PWM = Track_Velocity_PI(enc_r, Right_Base_RPM,
                                            &Right_Integral, &Right_FilteredEnc,
                                            RIGHT_RPM_CORRECTION, &Right_LastBias);
    } else {
        // 速度环禁用时，PWM输出为0
        Left_Final_PWM  = 0;
        Right_Final_PWM = 0;
    }

    // --- 第五步：PWM输出限幅，防止超出电机驱动范围
    Left_Final_PWM  = i_clamp(Left_Final_PWM,  VELOCITY_PWM_LIMIT);
    Right_Final_PWM = i_clamp(Right_Final_PWM, VELOCITY_PWM_LIMIT);

    // --- 第六步：电机方向修正（根据硬件接线配置）
    if (LEFT_MOTOR_REVERSE)  Left_Final_PWM  = -Left_Final_PWM;
    if (RIGHT_MOTOR_REVERSE) Right_Final_PWM = -Right_Final_PWM;

    // --- 第七步：左右轮交换（支持硬件左右电机接线配置），最终输出到TB6612电机驱动
    if (LR_SWAP) {
        // 左右交换：输出时右轮→左输出，左轮→右输出
        TB6612_SetSpeed(Right_Final_PWM, Left_Final_PWM);
    } else {
        // 正常输出：左→左，右→右
        TB6612_SetSpeed(Left_Final_PWM, Right_Final_PWM);
    }
}

// 巡线系统总初始化：初始化所有硬件外设、传感器、控制变量，启动系统
void Track_Init(void)
{
    // --- 第一部分：系统级配置
    // 配置NVIC中断优先级分组（2位抢占优先级，2位子优先级）
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    // --- 第二部分：灰度传感器初始化与校准
    // 初始化灰度传感器硬件
    Grayscale_Init();
    {
        // 8通道白参考值（校准时白色背景的ADC值）
        unsigned short white_ref[8] = {2646, 3088, 3105, 3162, 2807, 2484, 3101, 2786};
        // 8通道黑参考值（校准时黑色线条的ADC值）
        unsigned short black_ref[8] = { 708, 1787, 1351, 1127,  887,  632, 1258, 1123};
        // 将白/黑参考值写入传感器校准参数
        Grayscale_InitCalibrate(&gray_sensor, white_ref, black_ref);

        {
            // --- 计算各通道权重：原始位置权重 × 白值均值 ÷ 各通道白值
            // 目的：补偿不同通道灵敏度差异，使加权平均更准确
            int32_t raw_w[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
            int32_t avg = 0;
            uint8_t j;
            // 计算8通道白值的平均值
            for (j = 0; j < 8; j++)
                avg += gray_sensor.Calibrated_white[j];
            avg /= 8;
            // 逐个通道计算最终权重（灵敏度低的通道权重更大）
            for (j = 0; j < 8; j++)
                ir_weight[j] = raw_w[j] * avg
                             / (int32_t)gray_sensor.Calibrated_white[j];
        }
    }

    // --- 第三部分：外设初始化
    // 初始化编码器接口（用于读取左右轮转速）
    Encoder_Init();

    // 初始化TB6612电机驱动（PWM频率和周期配置）
    TB6612_Init(TB6612_PRE, TB6612_PER);

    // --- 第四部分：全局变量初始化 - 灰度与PID相关
    {
        uint8_t i;
        // 8路灰度原始数据清零
        for (i = 0; i < 8; i++) ir_raw[i] = 0;
    }
    // 黑线检测标志清零
    has_black           = 0;
    // PID滤波输出清零
    pid_filtered_output = 0.0f;
    // PID积分项清零
    PID_SumErr          = 0.0f;
    // PID上一拍偏差清零
    PID_LastErr         = 0.0f;
    // PID输出清零
    PID_Output          = 0;
    // 最后有效偏差清零
    last_valid_err      = 0;

    // --- 第五部分：全局变量初始化 - 电机转速相关
    Left_Base_RPM   = 0.0f;
    Right_Base_RPM  = 0.0f;
    Left_Final_PWM  = 0;
    Right_Final_PWM = 0;

    // --- 第六部分：全局变量初始化 - 直角检测状态机相关
    is_right_angle              = 0;
    right_angle_phase           = 0;
    right_angle_initial_flag = 0;
    right_angle_detect_type  = 0;
    // 初始化直角检测器
    RightAngleDetector_Init(&right_angle_detector);
    right_angle_filtered_bits = 0xFFU;

    // --- 第七部分：全局变量初始化 - 陀螺仪相关
    gyro_yaw_deg          = 0.0f;
    gyro_yaw_available    = 0;
    gyro_corner_start_yaw = 0.0f;
    gyro_corner_target_yaw = 0.0f;
    gyro_last_yaw         = 0.0f;
    gyro_yaw_accel        = 0.0f;
    gyro_yaw_rate         = 0.0f;
    gyro_first_valid      = 0;
    gyro_frame_count      = 0;
    gyro_isr_count        = 0;
    bno080_ok_count       = 0;
    gyro_raw_yaw          = 0.0f;
    gyro_raw_roll         = 0.0f;
    gyro_raw_pitch        = 0.0f;

    // --- 第八部分：全局变量初始化 - 速度环相关
    Left_Integral      = 0.0f;
    Right_Integral     = 0.0f;
    Left_FilteredEnc   = 0.0f;
    Right_FilteredEnc  = 0.0f;
    Left_LastBias      = 0.0f;
    Right_LastBias     = 0.0f;

    // --- 第九部分：启动1ms系统定时器
    Track_TIM11_Init();

    // --- 第十部分：BNO080陀螺仪初始化（循环直到成功输出首帧数据）
    // 循环重试：如果初始化失败或没有数据，则重新初始化I2C和BNO080
    while (!gyro_yaw_available) {
        // 初始化I2C硬件接口
        BNO080_I2C_Init();
        // 初始化BNO080传感器，返回是否成功
        bno080_ok = bno080_init();
        // 初始化成功后，轮询读取直到获得第一帧有效yaw数据
        if (bno080_ok) {
            while (!gyro_yaw_available) {
                // 触发陀螺仪读取请求
                gyro_poll_request = 1U;
                // 执行陀螺仪读取
                Track_Main_Gyro();
                // 处理陀螺仪数据，更新yaw角
                Track_Gyro_Update();
            }
        }
    }

    // --- 第十一部分：运行参数初始化，最后停车
    // 目标圈数设置为1圈
    Track_Target_Laps = 1U;
    // 已完成圈数清零
    Track_Completed_Laps = 0U;
    // 弯道计数清零
    Track_Corner_Count = 0U;
    // 调用停车函数，确保初始状态为停止
    Track_ControlStop();
}
