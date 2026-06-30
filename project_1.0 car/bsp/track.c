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

GrayscaleSensor_t gray_sensor;

int32_t  ir_weight[8];
uint8_t  ir_raw[8];
uint8_t  has_black;

float    PID_Err       = 0.0f;
float    PID_LastErr   = 0.0f;
float    PID_SumErr    = 0.0f;
int16_t  PID_Output    = 0;
float    pid_filtered_output = 0.0f;
int32_t  last_valid_err = 0;
float    err_filtered   = 0.0f;


volatile float  Left_Base_RPM    = 0.0f;
volatile float  Right_Base_RPM   = 0.0f;
float           base_rpm_current = 0.0f;
float           corner_min_current = 0.0f;
volatile int    Left_Final_PWM   = 0;
volatile int    Right_Final_PWM  = 0;
volatile float  Left_Speed_RPM   = 0.0f;
volatile float  Right_Speed_RPM  = 0.0f;
volatile uint8_t Velocity_Loop_Enable = 0;
volatile uint8_t Track_Run_Enable = 0U;
volatile uint16_t Track_Target_Laps = 1U;
volatile uint16_t Track_Completed_Laps = 0U;
volatile uint16_t Track_Corner_Count = 0U;

static uint8_t  is_in_corner    = 0;
static float    angle_progress  = 0.0f;

static float pid_smooth = 0.0f;
static float corner_gyro_diff_smooth = 0.0f;
static float g_last_target_base = 0.0f;
static float g_last_gyro_diff = 0.0f;
static float g_last_pid_correction = 0.0f;
static float g_last_angle_progress = 0.0f;

uint32_t is_right_angle              = 0;
uint32_t right_angle_phase           = 0;
uint32_t right_angle_initial_flag    = 0;
uint32_t right_angle_detect_type     = 0;
static RightAngleDetector_t right_angle_detector;
static uint8_t right_angle_filtered_bits = 0xFFU;

volatile int32_t enc_acc_L = 0;
volatile int32_t enc_acc_R = 0;
volatile uint32_t tim7_heartbeat = 0;
volatile TrackDebugSnapshot_t g_track_debug;

volatile uint32_t system_time_ms = 0;

volatile uint32_t flag_1ms = 0;
volatile uint32_t flag_5ms = 0;

float Left_Integral     = 0.0f;
float Right_Integral    = 0.0f;
float Left_FilteredEnc  = 0.0f;
float Right_FilteredEnc = 0.0f;
float Left_LastBias     = 0.0f;
float Right_LastBias    = 0.0f;

volatile float   gyro_yaw_deg       = 0.0f;
volatile uint32_t gyro_yaw_available = 0;

volatile uint32_t gyro_poll_request = 0;
uint16_t gyro_poll_timer_ms = 0;

float gyro_corner_start_yaw  = 0.0f;
float gyro_corner_target_yaw = 0.0f;
float gyro_last_yaw          = 0.0f;
volatile float gyro_yaw_accel = 0.0f;
volatile float gyro_yaw_rate  = 0.0f;
uint32_t gyro_first_valid     = 0;

volatile uint16_t gyro_frame_count = 0;
volatile uint16_t gyro_isr_count   = 0;
volatile uint16_t bno080_ok_count  = 0;
volatile float gyro_raw_yaw   = 0.0f;
volatile float gyro_raw_roll  = 0.0f;
volatile float gyro_raw_pitch = 0.0f;

volatile uint16_t bno080_ok = 0;

static inline float f_clamp(float val, float limit) {
    if (val >  limit) return  limit;
    if (val < -limit) return -limit;
    return val;
}
static inline int i_clamp(int val, int limit) {
    if (val >  limit) return  limit;
    if (val < -limit) return -limit;
    return val;
}

// 计算角度差（处理-180~180度环绕）
static float Track_Angle_Delta_Deg(float target, float current)
{
    float delta = target - current;
    while (delta > 180.0f) delta -= 360.0f;
    while (delta < -180.0f) delta += 360.0f;
    return delta;
}

// 向视觉模块发送转弯状态
static void Track_Send_Corner_State_To_Vision(void)
{
    static uint8_t last_state = 0xFFU;
    static uint8_t sent_idle = 0;
    static uint32_t last_send_ms = 0;
    uint8_t phase = right_angle_phase;
    uint8_t type = is_right_angle ? is_right_angle : right_angle_initial_flag;
    // 状态编码：低2位=phase，bit2=type(左右转)，bit3=陀螺仪就绪
    uint8_t state = phase & 0x03U;
    uint8_t should_send = 0;

    // 尝试发送之前排队的状态
    K230_PollCornerStateTx();

    // 左转标记置位bit2
    if (type == 2U) state |= 0x04U;
    // 陀螺仪就绪标记置位bit3
    state |= 0x08U;

    // 非空闲状态清除空闲发送标记
    if (right_angle_phase != 0U || type != 0U) {
        sent_idle = 0;
    }

    // 判断是否需要发送：状态变化立即发、转弯中周期发、空闲发一次
    if (state != last_state) {
        should_send = 1;
    } else if (phase != 0U && (uint32_t)(system_time_ms - last_send_ms) >= 50U) {
        should_send = 1;
    } else if (phase == 0U && type == 0U && !sent_idle) {
        should_send = 1;
        sent_idle = 1;
    }

    // 无需发送则直接返回
    if (!should_send) return;

    // 状态入队并立即发送，更新上次状态和时间
    K230_QueueCornerState(state);
    K230_PollCornerStateTx();
    last_state = state;
    last_send_ms = system_time_ms;
}

// 复位直角转弯状态机
static void Track_Reset_Right_Angle_State(void)
{
    is_right_angle          = 0;
    right_angle_phase       = 0;
    right_angle_initial_flag = 0;
    right_angle_detect_type = 0;
}

// 复位巡线控制状态
static void Track_Reset_Control_State(void)
{
    PID_Err = 0.0f;
    PID_LastErr = 0.0f;
    PID_SumErr = 0.0f;
    PID_Output = 0;
    pid_filtered_output = 0.0f;
    pid_smooth = 0.0f;
    err_filtered = 0.0f;
    corner_gyro_diff_smooth = 0.0f;
    base_rpm_current = 0.0f;
    corner_min_current = 0.0f;
    g_last_target_base = 0.0f;
    g_last_gyro_diff = 0.0f;
    g_last_pid_correction = 0.0f;
    g_last_angle_progress = 0.0f;
    Left_Base_RPM = 0.0f;
    Right_Base_RPM = 0.0f;
    Left_Final_PWM = 0;
    Right_Final_PWM = 0;
    Left_Integral = 0.0f;
    Right_Integral = 0.0f;
    Left_FilteredEnc = 0.0f;
    Right_FilteredEnc = 0.0f;
    Left_LastBias = 0.0f;
    Right_LastBias = 0.0f;
    is_in_corner = 0U;
}

void Track_ControlStart(void)
{
    Track_Run_Enable = 1U;
    Track_Completed_Laps = 0U;
    Track_Corner_Count = 0U;
    Track_Reset_Right_Angle_State();
    Track_Reset_Control_State();
    Velocity_Loop_Enable = 1U;
}

void Track_ControlStop(void)
{
    Track_Run_Enable = 0U;
    Velocity_Loop_Enable = 0U;
    Track_Reset_Right_Angle_State();
    Track_Reset_Control_State();
    TB6612_SetSpeed(0, 0);
}

void Track_TargetLapAdd(void)
{
    if (Track_Target_Laps < TRACK_TARGET_LAPS_MAX) {
        Track_Target_Laps++;
    }
}

void Track_TargetLapSub(void)
{
    if (Track_Target_Laps > 1U) {
        Track_Target_Laps--;
    }
}

// 完成一个弯，计数并判断是否跑完全程
static void Track_RegisterCornerComplete(void)
{
    // 未启动巡线则不计数
    if (!Track_Run_Enable) return;

    // 弯道计数+1，判断是否满一圈
    if (++Track_Corner_Count >= TRACK_CORNERS_PER_LAP) {
        // 满一圈则弯道计数清零，已完成圈数+1
        Track_Corner_Count = 0U;
        if (Track_Completed_Laps < 0xFFFFU) {
            Track_Completed_Laps++;
        }
        // 达到目标圈数则停车
        if (Track_Completed_Laps >= Track_Target_Laps) {
            Track_ControlStop();
        }
    }
}

void Track_Gray_Convert(void)
{
    // 读取灰度传感器数字量
    uint8_t digital = Grayscale_GetDigital(&gray_sensor);
    uint8_t i;

    // 数字量拆解为8通道二进制数组
    for (i = 0; i < 8; i++) {
        ir_raw[i] = (digital >> i) & 1;
    }

    // 判断是否检测到黑线（非全白即有线）
    has_black = (digital != 0xFF) ? 1 : 0;
    // 更新直角检测器滤波状态
    right_angle_filtered_bits = RightAngleDetector_Update(&right_angle_detector,
                                                           digital);
}

uint8_t Track_Gyro_TakePollRequest(void)
{
    // 有轮询请求则消费并读取BNO080数据
    if (gyro_poll_request) {
        // 清除请求标志
        gyro_poll_request = 0;
        // 读取成功则计数+1
        if (bno080_update()) {
            bno080_ok_count++;
        }
        return 1;
    }
    return 0;
}

void Track_Gyro_Update(void)
{
    // 中断计数+1
    gyro_isr_count++;

    // 有新数据则处理
    if (bno080_data_available()) {
        float roll, pitch, yaw;
        // 读取欧拉角
        bno080_get_euler(&roll, &pitch, &yaw);

        // 有效帧计数+1
        gyro_frame_count++;

        // 保存原始欧拉角供调试
        gyro_raw_roll  = roll;
        gyro_raw_pitch = pitch;
        gyro_raw_yaw   = yaw;

        // 首帧数据初始化
        if (!gyro_first_valid) {
            // 应用方向系数，初始化yaw基准
            gyro_yaw_deg   = (GYRO_YAW_DIRECTION >= 0) ? yaw : -yaw;
            // 速率和加速度初始化为0
            gyro_yaw_rate  = 0.0f;
            gyro_yaw_accel = 0.0f;
            // 保存上一帧yaw
            gyro_last_yaw    = gyro_yaw_deg;
            // 标记数据可用
            gyro_yaw_available = 1;
            gyro_first_valid = 1;
        } else {
            // 后续帧计算角速率和角加速度
            float new_yaw = (GYRO_YAW_DIRECTION >= 0) ? yaw : -yaw;
            // 计算yaw变化率（度/ms）
            float yaw_rate_now = Track_Angle_Delta_Deg(new_yaw, gyro_yaw_deg);
            // 角加速度 = 当前速率 - 上次速率
            gyro_yaw_accel = yaw_rate_now - gyro_yaw_rate;
            // 更新角速率
            gyro_yaw_rate  = yaw_rate_now;

            // 保存上一帧yaw
            gyro_last_yaw = gyro_yaw_deg;

            // 更新当前yaw，标记数据可用
            gyro_yaw_deg = new_yaw;
            gyro_yaw_available = 1;
        }
    }
}

int32_t Track_Calc_Err(void)
{
    int32_t sum_w = 0, sum_b = 0;
    uint8_t i;

    // 遍历8通道，加权计算黑线中心位置
    for (i = 0; i < 8; i++) {
        // 读取当前通道ADC值和白参考值
        int32_t adc = (int32_t)gray_sensor.Analog_value[i];
        int32_t ref = (int32_t)gray_sensor.Calibrated_white[i];
        if (ref == 0) continue;
        // 计算当前值占白参考的百分比（0~10000）
        int32_t pct = (adc * 10000) / ref;
        if (pct > 10000) pct = 10000;
        // 计算黑度（越大越黑）
        int32_t black = 10000 - pct;
        // 加权累积：黑度×位置权重
        sum_w += black * ir_weight[i];
        // 黑度总和
        sum_b += black;
    }

    int32_t raw_err;

    // 全白无黑线时，保持上次有效偏差
    if (sum_b == 0) {
        raw_err = last_valid_err;
    } else {
        // 加权平均计算偏差值（-7000~7000）
        raw_err = (int32_t)(((long long)sum_w * 1000) / sum_b);
        // 保存为有效偏差
        last_valid_err = raw_err;
    }

    // 非转弯状态下对偏差做低通滤波
    if (is_right_angle == 0) {
        // 一阶低通滤波：新值*alpha + 旧值*(1-alpha)
        err_filtered = ERR_FILTER_ALPHA * (float)raw_err + (1.0f - ERR_FILTER_ALPHA) * err_filtered;
        // 四舍五入取整
        raw_err = (int32_t)(err_filtered + 0.5f);
    }

    return raw_err;
}

// 直角检测状态机
void Track_Check_Right_Angle(void)
{
    // 抑制未使用变量警告(调试预留)
    (void)g_last_target_base;
    (void)g_last_gyro_diff;
    (void)g_last_pid_correction;
    (void)g_last_angle_progress;

    // phase 0/1: 空闲或预触发，检测直角特征
    if (is_right_angle == 0 && right_angle_phase <= 1U)
    {
        // 读取已确认的直角特征
        uint8_t feature = RightAngleDetector_ConfirmedFeature(&right_angle_detector);
        // 检测到特征则记录类型并进入预触发
        if (feature != 0U) {
            right_angle_initial_flag = feature;
            right_angle_detect_type = feature;
            // phase0时切换到phase1预触发
            if (right_angle_phase == 0U) {
                right_angle_phase = 1U;
            }
        }
    }
    // phase 1: 预触发阶段，等待全白+陀螺仪就绪
    if (right_angle_phase == 1U)
    {
        uint8_t filtered_feature = RightAngleDetector_Feature(right_angle_filtered_bits);
        // 条件满足：有初始标志+全白+白确认，进入转弯
        if (right_angle_initial_flag != 0U &&
            RightAngleDetector_AllWhite(right_angle_filtered_bits) &&
            RightAngleDetector_WhiteConfirmed(&right_angle_detector)) {
            // 锁定转弯类型，标记直角状态
            right_angle_detect_type = right_angle_initial_flag;
            is_right_angle = right_angle_initial_flag;
            // 切换到phase2执行转弯
            right_angle_phase = 2;
            // 记录转弯起始yaw
            gyro_corner_start_yaw = gyro_yaw_deg;
            // 根据左右转设置目标yaw
            if (is_right_angle == 1) {
                gyro_corner_target_yaw = gyro_corner_start_yaw - CORNER_YAW_TARGET;
            } else {
                gyro_corner_target_yaw = gyro_corner_start_yaw + CORNER_YAW_TARGET;
            }
        } else if (right_angle_initial_flag != 0U &&
                   filtered_feature == 0U &&
                   !RightAngleDetector_AllWhite(right_angle_filtered_bits)) {
            // 特征消失且未全白：误触发，复位状态
            Track_Reset_Right_Angle_State();
        }
    }
    // phase 2: 转弯执行中，陀螺仪闭环控制，转到灰度接管角
    else if (is_right_angle != 0 && right_angle_phase == 2)
    {
        // 计算已转过的角度
        float yaw_diff = fabsf(Track_Angle_Delta_Deg(gyro_yaw_deg,
                                                     gyro_corner_start_yaw));
        // 达到灰度融合起始角，切换到phase3
        if (yaw_diff >= CORNER_GRAY_BLEND_START_DEG) {
            right_angle_phase = 3;
        }

    }
    // phase 3: 灰度接管段，IMU判断退出时机
    else if (is_right_angle != 0 && right_angle_phase == 3)
    {
        // 计算已转过的角度
        float yaw_diff = fabsf(Track_Angle_Delta_Deg(gyro_yaw_deg,
                                                     gyro_corner_start_yaw));
        // 未到退出角度，继续等待
        if (yaw_diff < CORNER_IMU_EXIT_DEG) {
            return;
        }

        // 达到退出角度：清除直角状态
        right_angle_phase = 0;
        is_right_angle    = 0;

        // 复位直角检测器状态
        Track_Reset_Right_Angle_State();
        // 复位转弯相关积分和滤波
        corner_gyro_diff_smooth = 0.0f;
        Left_Integral     = 0.0f;
        Right_Integral    = 0.0f;
        Left_FilteredEnc  = 0.0f;
        Right_FilteredEnc = 0.0f;

        // 复位弯道最低转速
        corner_min_current = BASE_RPM;

        // 标记不在弯道中
        is_in_corner = 0;

        // 注册弯道完成，更新圈数计数
        Track_RegisterCornerComplete();
    }
}

void Track_PID_Calc(int32_t err)
{
    // 保存当前偏差
    PID_Err = (float)err;
    // 积分项累积
    PID_SumErr += PID_Err;

    // PID计算：比例 + 积分 + 微分
    float output_raw = KP_NORMAL * PID_Err
                     + KI_NORMAL * PID_SumErr
                     + KD_NORMAL * (PID_Err - PID_LastErr);

    // 输出限幅
    output_raw = f_clamp(output_raw, STRAIGHT_RPM_LIMIT * 10.0f);

    // 输出低通滤波平滑
    pid_filtered_output = PID_FILTER_ALPHA * output_raw
                        + (1.0f - PID_FILTER_ALPHA) * pid_filtered_output;

    // 转换为整数输出，保存上次偏差
    PID_Output  = (int16_t)pid_filtered_output;
    PID_LastErr = PID_Err;
}

void Track_Action_Execute(void)
{
    float target_base = BASE_RPM;
    float gyro_diff   = 0.0f;
    float pid_correction = 0.0f;
    // PID输出转换为RPM单位
    float raw_pid = (float)PID_Output * 0.1f;

    // PID输出一阶平滑滤波
    pid_smooth += CORNER_PID_SMOOTH_ALPHA * (raw_pid - pid_smooth);
    // 保存调试快照
    g_last_target_base = BASE_RPM;
    g_last_gyro_diff = 0.0f;
    g_last_pid_correction = pid_smooth;
    g_last_angle_progress = 0.0f;

    // 状态机：预触发阶段
    if (right_angle_phase == 1U) {
        // 预触发：标记未入弯，提前减速
        is_in_corner = 0U;
        target_base = BASE_RPM - CORNER_PREVIEW_DECEL_RPM;
        pid_correction = pid_smooth;
    }
    // 状态机：转弯执行阶段(phase2/3)
    else if (is_right_angle != 0 &&
               (right_angle_phase == 2U || right_angle_phase == 3U)) {
        float entry;
        float gray_blend;
        float yaw_diff = 0.0f;
        // 标记在弯道中
        is_in_corner = 1;
        {
            // 计算已转过的角度和进度(0~1)
            yaw_diff = fabsf(Track_Angle_Delta_Deg(gyro_yaw_deg, gyro_corner_start_yaw));
            angle_progress = yaw_diff / CORNER_YAW_TARGET;
            if (angle_progress > 1.0f) angle_progress = 1.0f;
        }
        g_last_angle_progress = angle_progress;
        // 计算入弯平滑权重(0→1)和灰度接管权重(0→1)
        entry = CornerProfile_Smoothstep5(
            angle_progress / CORNER_ENTRY_BLEND_PROGRESS);
        gray_blend = CornerProfile_Smoothstep5(
            (yaw_diff - CORNER_GRAY_BLEND_START_DEG) /
            (CORNER_IMU_EXIT_DEG - CORNER_GRAY_BLEND_START_DEG));

        {
            // 计算出弯加速进度
            float exit_speed = CornerProfile_Smoothstep5(
                (angle_progress - CORNER_SPEED_EXIT_START_PROGRESS) /
                CORNER_SPEED_EXIT_BLEND_WIDTH);
            // 计算弯中深度(入弯×(1-出弯))，深度越大速度越低
            float corner_depth = entry * (1.0f - exit_speed);
            // 基础转速 = 直道速度 - (直道速度-弯道速度) × 弯中深度
            target_base = BASE_RPM - (BASE_RPM - CORNER_TURN_RPM) * corner_depth;
        }

        // 陀螺仪PD闭环控制差速
        {
            // 计算yaw角误差
            float yaw_error = Track_Angle_Delta_Deg(gyro_corner_target_yaw,
                                                    gyro_yaw_deg);
            // PD输出：比例×误差 - 微分×角速率
            gyro_diff = KP_CORNER_YAW * yaw_error
                      - KD_CORNER_YAW * gyro_yaw_rate;
        }
        // 陀螺仪差速限幅
        gyro_diff = f_clamp(gyro_diff, CORNER_GYRO_DIFF_LIMIT_RPM);
        // 权重融合：入弯权重×(1-灰度权重)，实现陀螺仪主导→灰度主导过渡
        gyro_diff *= entry * (1.0f - gray_blend);
        // 差速斜率限制（上升沿）
        corner_gyro_diff_smooth = CornerProfile_Slew(
            corner_gyro_diff_smooth, gyro_diff,
            CORNER_DIFF_SLEW_UP_RPM_PER_MS);
        gyro_diff = corner_gyro_diff_smooth;
        g_last_gyro_diff = gyro_diff;

        // 灰度PID修正随灰度融合权重逐步接管
        pid_correction = gray_blend * pid_smooth;
    }
    // 状态机：正常直道巡线
    else {
        is_in_corner = 0;
        target_base  = BASE_RPM;
        gyro_diff    = 0.0f;
        pid_correction = pid_smooth;
    }

    // 非弯道时施加陀螺仪直道阻尼
    if (!is_in_corner) {
        if (GYRO_STRAIGHT_DAMPING_ENABLE) {
            // 阻尼量 = -Kd × yaw角速率（反方向抑制晃动）
            float straight_damping = -KD_GYRO_STRAIGHT * gyro_yaw_rate;
            // 阻尼输出限幅
            straight_damping = f_clamp(straight_damping, GYRO_STRAIGHT_LIMIT);
            // 叠加到差速修正
            gyro_diff += straight_damping;
        }
    }

    // 基础转速斜坡限制（加减速速率不同）
    base_rpm_current = CornerProfile_Slew(
        base_rpm_current, target_base,
        (target_base < base_rpm_current) ? CORNER_BASE_SLEW_DOWN_RPM_PER_MS
                                        : CORNER_BASE_SLEW_UP_RPM_PER_MS);
    // 保存调试快照
    g_last_target_base = target_base;
    g_last_pid_correction = pid_correction;

    {
    // 保存上一轮左右轮目标转速
    float previous_left = Left_Base_RPM;
    float previous_right = Right_Base_RPM;
    // 合成左右轮目标转速：左轮=基础-差速+PID，右轮=基础+差速-PID
    Left_Base_RPM  = base_rpm_current - gyro_diff + pid_correction;
    Right_Base_RPM = base_rpm_current + gyro_diff - pid_correction;

    // 直道转速限幅（0~最高转速）
    if (Left_Base_RPM  < 0.0f) Left_Base_RPM  = 0.0f;
    if (Left_Base_RPM  > STRAIGHT_RPM_LIMIT) Left_Base_RPM  = STRAIGHT_RPM_LIMIT;
    if (Right_Base_RPM < 0.0f) Right_Base_RPM = 0.0f;
    if (Right_Base_RPM > STRAIGHT_RPM_LIMIT) Right_Base_RPM = STRAIGHT_RPM_LIMIT;

    // 弯道转速限幅（最低~最高弯道转速）
    if (is_in_corner) {
        if (Left_Base_RPM  < CORNER_MIN_RPM) Left_Base_RPM  = CORNER_MIN_RPM;
        if (Left_Base_RPM  > CORNER_MAX_RPM) Left_Base_RPM  = CORNER_MAX_RPM;
        if (Right_Base_RPM < CORNER_MIN_RPM) Right_Base_RPM = CORNER_MIN_RPM;
        if (Right_Base_RPM > CORNER_MAX_RPM) Right_Base_RPM = CORNER_MAX_RPM;
    }
    // 轮指令斜率限制：每周期变化不超过最大值
    Left_Base_RPM = CornerProfile_Slew(previous_left, Left_Base_RPM,
                                       WHEEL_COMMAND_SLEW_RPM_PER_MS);
    Right_Base_RPM = CornerProfile_Slew(previous_right, Right_Base_RPM,
                                        WHEEL_COMMAND_SLEW_RPM_PER_MS);
    }
}

void Track_Main_Debug_Stage(uint32_t time)
{
    (void)time;
}

// 单电机速度环PI计算
int Track_Velocity_PI(int encoder_count, float target_rpm,
                      float *integral, float *filtered_enc,
                      float correction, float *last_bias)
{
    float target_pulse, bias, pwm_out;

    // 应用转速修正系数，转换为目标脉冲数/采样周期
    target_rpm  = target_rpm * correction;
    target_pulse = (target_rpm * (float)PULSE_PER_ROUND * SAMPLE_TIME) / 60.0f;

    // 编码器值一阶低通滤波
    *filtered_enc = 0.5f * (float)encoder_count + 0.5f * (*filtered_enc);

    // 计算偏差 = 目标脉冲 - 实际滤波脉冲
    bias = target_pulse - (*filtered_enc);

    // 死区判断，偏差大于1才积分（防止小偏差积分饱和）
    if (fabsf(bias) > 1.0f) {
        // 积分项累积
        *integral += KI_VELOCITY * bias;
        // 积分限幅
        *integral = f_clamp(*integral, (float)INTEGRAL_LIMIT);
    }

    // PID计算：比例 + 积分 + 微分
    pwm_out = KP_VELOCITY * bias + *integral + KD_VELOCITY * (bias - *last_bias);
    // 保存本次偏差供下次微分用
    *last_bias = bias;

    // 输出PWM限幅后返回
    return i_clamp((int)pwm_out, VELOCITY_PWM_LIMIT);
}

void Track_TIM11_Init(void)
{
    TIM_TimeBaseInitTypeDef  tim_base;
    NVIC_InitTypeDef         nvic;

    // 开启TIM11时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);

    // 配置定时器：168预分频(1MHz)，1000周期(1ms)
    tim_base.TIM_Prescaler     = 168 - 1;
    tim_base.TIM_Period        = 1000 - 1;
    tim_base.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_base.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM11, &tim_base);

    // 配置中断优先级并使能
    nvic.NVIC_IRQChannel                   = TIM1_TRG_COM_TIM11_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority  = 0;
    nvic.NVIC_IRQChannelSubPriority         = 0;
    nvic.NVIC_IRQChannelCmd                 = ENABLE;
    NVIC_Init(&nvic);

    // 使能更新中断，启动定时器
    TIM_ITConfig(TIM11, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM11, ENABLE);
}

// 1ms定时器中断
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
    // 检查更新中断标志
    if (TIM_GetITStatus(TIM11, TIM_IT_Update) != RESET) {
        // 清除中断标志
        TIM_ClearITPendingBit(TIM11, TIM_IT_Update);
        // 系统时间+1ms，置1ms标志
        system_time_ms++;
        flag_1ms = 1;

        // 陀螺仪轮询定时计数，到点触发读取请求
        gyro_poll_timer_ms++;
        if (gyro_poll_timer_ms >= GYRO_MAIN_POLL_PERIOD_MS) {
            gyro_poll_timer_ms = 0;
            gyro_poll_request = 1;
        }

        // 5ms分频：每5次1ms中断产生一次5ms标志
        {
            static uint8_t ms5_cnt = 0;
            if (++ms5_cnt >= 5) {
                ms5_cnt = 0;
                flag_5ms = 1;
            }
        }
    }
}

// 1ms主任务
void Track_Main_1ms(void)
{
    // 无1ms标志则直接返回
    if (!flag_1ms) return;
    flag_1ms = 0;

    // 灰度数据读取与转换
    Track_Gray_Convert();

    // 陀螺仪数据更新
    Track_Gyro_Update();

    // 未启动巡线则复位状态并返回
    if (!Track_Run_Enable) {
        Track_Reset_Right_Angle_State();
        Track_Reset_Control_State();
        Track_Send_Corner_State_To_Vision();
        return;
    }

    // 直角检测状态机
    Track_Check_Right_Angle();

    // 发送转弯状态给视觉模块
    Track_Send_Corner_State_To_Vision();

    // 再次检查运行标志（可能在状态机中被停止）
    if (!Track_Run_Enable) {
        return;
    }

    {
        // 计算巡线偏差，执行PID计算
        int32_t err = Track_Calc_Err();
        PID_Err = (float)err;
        Track_PID_Calc(err);
    }

    // 合成左右轮目标转速输出
    Track_Action_Execute();
}

// 5ms主任务：速度环
void Track_Main_5ms(void)
{
    // 无5ms标志则直接返回
    if (!flag_5ms) return;
    flag_5ms = 0;
    // 执行速度环计算和电机输出
    Track_SpeedLoop();
}

void Track_Main_Gyro(void)
{
    (void)Track_Gyro_TakePollRequest();
}

void Track_Main_Debug(uint32_t time)
{
    static uint32_t print_timer = 0;
    if ((int32_t)(system_time_ms - print_timer) < time) return;
    print_timer = system_time_ms;

    float roll_deg, pitch_deg, yaw_deg;
    const BNO080_RotationVector_t *qv;
    char buf[160];
    int len;
    bno080_get_euler(&roll_deg, &pitch_deg, &yaw_deg);
    qv = bno080_get_rotation_vector();
    len = sprintf(buf,
        "R:%+7.2f P:%+7.2f Y:%+7.2f  Q(wxyz):%+.4f %+.4f %+.4f %+.4f\r\n",
        roll_deg, pitch_deg, yaw_deg,
        qv->real, qv->i, qv->j, qv->k);
    { int i; for (i = 0; i < len; i++) USART3_SendByte((uint8_t)buf[i]); }
}

// 速度环PID计算与电机输出
void Track_SpeedLoop(void)
{
    // 心跳计数+1
    tim7_heartbeat++;

    // 读取左右轮编码器脉冲
    Encoder_ReadAll();

    // 编码器累计计数
    enc_acc_L += Encoder_Left;
    enc_acc_R += Encoder_Right;

    // 编码器方向修正
    int16_t enc_l = Encoder_Left;
    int16_t enc_r = Encoder_Right;
    if (ENCODER_LEFT_INVERT)  enc_l = -enc_l;
    if (ENCODER_RIGHT_INVERT) enc_r = -enc_r;

    // 脉冲转换为RPM，限幅防止异常值
    Left_Speed_RPM  = (float)enc_l * RPM_COEFFICIENT;
    Right_Speed_RPM = (float)enc_r * RPM_COEFFICIENT;
    Left_Speed_RPM  = f_clamp(Left_Speed_RPM,  400.0f);
    Right_Speed_RPM = f_clamp(Right_Speed_RPM, 400.0f);

    // 速度环使能时执行PI计算
    if (Velocity_Loop_Enable) {
        // 左轮速度环PI计算
        Left_Final_PWM  = Track_Velocity_PI(enc_l,  Left_Base_RPM,
                                            &Left_Integral,  &Left_FilteredEnc,
                                            LEFT_RPM_CORRECTION,  &Left_LastBias);
        // 右轮速度环PI计算
        Right_Final_PWM = Track_Velocity_PI(enc_r, Right_Base_RPM,
                                            &Right_Integral, &Right_FilteredEnc,
                                            RIGHT_RPM_CORRECTION, &Right_LastBias);
    } else {
        // 禁用速度环则输出0
        Left_Final_PWM  = 0;
        Right_Final_PWM = 0;
    }

    // PWM输出限幅
    Left_Final_PWM  = i_clamp(Left_Final_PWM,  VELOCITY_PWM_LIMIT);
    Right_Final_PWM = i_clamp(Right_Final_PWM, VELOCITY_PWM_LIMIT);

    // 电机方向修正
    if (LEFT_MOTOR_REVERSE)  Left_Final_PWM  = -Left_Final_PWM;
    if (RIGHT_MOTOR_REVERSE) Right_Final_PWM = -Right_Final_PWM;

    // 左右轮交换（硬件兼容），输出到TB6612
    if (LR_SWAP) {
        TB6612_SetSpeed(Right_Final_PWM, Left_Final_PWM);
    } else {
        TB6612_SetSpeed(Left_Final_PWM, Right_Final_PWM);
    }
}

void Track_Init(void)
{
    // 设置中断优先级分组
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    // 灰度传感器初始化与校准
    Grayscale_Init();
    {
        // 白/黑参考值（出厂校准数据）
        unsigned short white_ref[8] = {2646, 3088, 3105, 3162, 2807, 2484, 3101, 2786};
        unsigned short black_ref[8] = { 708, 1787, 1351, 1127,  887,  632, 1258, 1123};
        Grayscale_InitCalibrate(&gray_sensor, white_ref, black_ref);

        {
            // 计算位置权重（白值均值归一化，补偿各通道灵敏度差异）
            int32_t raw_w[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
            int32_t avg = 0;
            uint8_t j;
            // 计算白值均值
            for (j = 0; j < 8; j++)
                avg += gray_sensor.Calibrated_white[j];
            avg /= 8;
            // 各通道权重 = 原始权重 × 均值 ÷ 通道白值
            for (j = 0; j < 8; j++)
                ir_weight[j] = raw_w[j] * avg
                             / (int32_t)gray_sensor.Calibrated_white[j];
        }
    }

    // 编码器初始化
    Encoder_Init();

    // TB6612电机驱动初始化
    TB6612_Init(TB6612_PRE, TB6612_PER);

    // 全局变量初始化
    {
        uint8_t i;
        for (i = 0; i < 8; i++) ir_raw[i] = 0;
    }
    has_black           = 0;
    pid_filtered_output = 0.0f;
    PID_SumErr          = 0.0f;
    PID_LastErr         = 0.0f;
    PID_Output          = 0;
    last_valid_err      = 0;

    // 左右轮目标转速和PWM初始化
    Left_Base_RPM   = 0.0f;
    Right_Base_RPM  = 0.0f;
    Left_Final_PWM  = 0;
    Right_Final_PWM = 0;

    // 直角检测状态初始化
    is_right_angle              = 0;
    right_angle_phase           = 0;
    right_angle_initial_flag = 0;
    right_angle_detect_type  = 0;
    corner_gyro_diff_smooth = 0.0f;
    RightAngleDetector_Init(&right_angle_detector);
    right_angle_filtered_bits = 0xFFU;

    // 陀螺仪相关变量初始化
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

    // 速度环相关变量初始化
    Left_Integral      = 0.0f;
    Right_Integral     = 0.0f;
    Left_FilteredEnc   = 0.0f;
    Right_FilteredEnc  = 0.0f;
    Left_LastBias      = 0.0f;
    Right_LastBias     = 0.0f;

    // 启动1ms定时器
    Track_TIM11_Init();

    // 初始化BNO080陀螺仪，直到获取首帧yaw数据
    while (!gyro_yaw_available) {
        // I2C和BNO080初始化
        BNO080_I2C_Init();
        bno080_ok = bno080_init();
        if (bno080_ok) {
            // 轮询读取直到首帧数据有效
            while (!gyro_yaw_available) {
                gyro_poll_request = 1U;
                Track_Main_Gyro();
                Track_Gyro_Update();
            }
        }
    }

    // 圈数和弯道计数初始化，停止巡线
    Track_Target_Laps = 1U;
    Track_Completed_Laps = 0U;
    Track_Corner_Count = 0U;
    Track_ControlStop();
}
