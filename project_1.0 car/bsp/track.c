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

    K230_PollCornerStateTx();

    if (type == 2U) state |= 0x04U;
    state |= 0x08U;

    if (right_angle_phase != 0U || type != 0U) {
        sent_idle = 0;
    }

    // 发送时机：状态变化时立即发送，或转弯中每50ms周期性发送，或空闲时发送一次
    if (state != last_state) {
        should_send = 1;
    } else if (phase != 0U && (uint32_t)(system_time_ms - last_send_ms) >= 50U) {
        should_send = 1;
    } else if (phase == 0U && type == 0U && !sent_idle) {
        should_send = 1;
        sent_idle = 1;
    }

    if (!should_send) return;

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
    if (!Track_Run_Enable) return;

    // 每完成一个弯角，弯道计数+1；满一圈弯道数则圈数+1
    if (++Track_Corner_Count >= TRACK_CORNERS_PER_LAP) {
        Track_Corner_Count = 0U;
        if (Track_Completed_Laps < 0xFFFFU) {
            Track_Completed_Laps++;
        }
        // 判断是否达到目标圈数，达到则停车
        if (Track_Completed_Laps >= Track_Target_Laps) {
            Track_ControlStop();
        }
    }
}

void Track_Gray_Convert(void)
{
    uint8_t digital = Grayscale_GetDigital(&gray_sensor);
    uint8_t i;

    // 数字量拆解为8通道二进制数组，供后续逻辑使用
    for (i = 0; i < 8; i++) {
        ir_raw[i] = (digital >> i) & 1;
    }

    // 判断是否有黑线压住（非全白即有线）
    has_black = (digital != 0xFF) ? 1 : 0;
    // 更新直角检测器滤波状态
    right_angle_filtered_bits = RightAngleDetector_Update(&right_angle_detector,
                                                           digital);
}

uint8_t Track_Gyro_TakePollRequest(void)
{
    // 响应1ms定时器轮询请求，触发一次BNO080数据读取
    if (gyro_poll_request) {
        gyro_poll_request = 0;
        if (bno080_update()) {
            bno080_ok_count++;
        }
        return 1;
    }
    return 0;
}

void Track_Gyro_Update(void)
{
    gyro_isr_count++;

    if (bno080_data_available()) {
        float roll, pitch, yaw;
        bno080_get_euler(&roll, &pitch, &yaw);

        gyro_frame_count++;

        // 保存原始欧拉角，供调试查看
        gyro_raw_roll  = roll;
        gyro_raw_pitch = pitch;
        gyro_raw_yaw   = yaw;

        if (!gyro_first_valid) {
            // 首帧数据：初始化yaw基准，速率/加速度清零
            gyro_yaw_deg   = (GYRO_YAW_DIRECTION >= 0) ? yaw : -yaw;
            gyro_yaw_rate  = 0.0f;
            gyro_yaw_accel = 0.0f;
            gyro_last_yaw    = gyro_yaw_deg;
            gyro_yaw_available = 1;
            gyro_first_valid = 1;
        } else {
            // 后续帧：计算yaw角速率和角加速度
            float new_yaw = (GYRO_YAW_DIRECTION >= 0) ? yaw : -yaw;
            float yaw_rate_now = Track_Angle_Delta_Deg(new_yaw, gyro_yaw_deg);
            gyro_yaw_accel = yaw_rate_now - gyro_yaw_rate;
            gyro_yaw_rate  = yaw_rate_now;

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

    // 加权平均：每个通道的黑度×权重，计算灰度中心位置作为巡线偏差
    for (i = 0; i < 8; i++) {
        int32_t adc = (int32_t)gray_sensor.Analog_value[i];
        int32_t ref = (int32_t)gray_sensor.Calibrated_white[i];
        if (ref == 0) continue;
        int32_t pct = (adc * 10000) / ref;
        if (pct > 10000) pct = 10000;
        int32_t black = 10000 - pct;
        sum_w += black * ir_weight[i];
        sum_b += black;
    }

    int32_t raw_err;

    // 全白无偏差时，保持上次有效偏差值
    if (sum_b == 0) {
        raw_err = last_valid_err;
    } else {
        raw_err = (int32_t)(((long long)sum_w * 1000) / sum_b);
        last_valid_err = raw_err;
    }

    // 非转弯状态对偏差做低通滤波，抑制高频抖动
    if (is_right_angle == 0) {
        err_filtered = ERR_FILTER_ALPHA * (float)raw_err + (1.0f - ERR_FILTER_ALPHA) * err_filtered;
        raw_err = (int32_t)(err_filtered + 0.5f);
    }

    return raw_err;
}

// 直角检测状态机
void Track_Check_Right_Angle(void)
{
    // 抑制未使用变量警告(这些变量为调试/未来功能预留)
    (void)g_last_target_base;
    (void)g_last_gyro_diff;
    (void)g_last_pid_correction;
    (void)g_last_angle_progress;

    // phase 0: 空闲等待，检测直角特征信号触发
    if (is_right_angle == 0 && right_angle_phase <= 1U)
    {
        uint8_t feature = RightAngleDetector_ConfirmedFeature(&right_angle_detector);
        if (feature != 0U) {
            right_angle_initial_flag = feature;
            right_angle_detect_type = feature;
            if (right_angle_phase == 0U) {
                right_angle_phase = 1U;
            }
        }
    }
    // phase 1: 预触发阶段，等待灰度全白+陀螺仪就绪后锁定目标yaw进入转弯
    if (right_angle_phase == 1U)
    {
        uint8_t filtered_feature = RightAngleDetector_Feature(right_angle_filtered_bits);
        if (right_angle_initial_flag != 0U &&
            RightAngleDetector_AllWhite(right_angle_filtered_bits) &&
            RightAngleDetector_WhiteConfirmed(&right_angle_detector)) {
            // 条件满足：记录初始yaw，计算目标yaw（左/右转方向不同），切换到phase 2
            right_angle_detect_type = right_angle_initial_flag;
            is_right_angle = right_angle_initial_flag;
            right_angle_phase = 2;
            gyro_corner_start_yaw = gyro_yaw_deg;
            if (is_right_angle == 1) {
                gyro_corner_target_yaw = gyro_corner_start_yaw - CORNER_YAW_TARGET;
            } else {
                gyro_corner_target_yaw = gyro_corner_start_yaw + CORNER_YAW_TARGET;
            }
        } else if (right_angle_initial_flag != 0U &&
                   filtered_feature == 0U &&
                   !RightAngleDetector_AllWhite(right_angle_filtered_bits)) {
            // 特征消失且未全白：误触发，复位
            Track_Reset_Right_Angle_State();
        }
    }
    // phase 2: 执行转弯，陀螺仪闭环控制，转到灰度接管起始角
    else if (is_right_angle != 0 && right_angle_phase == 2)
    {
        float yaw_diff = fabsf(Track_Angle_Delta_Deg(gyro_yaw_deg,
                                                     gyro_corner_start_yaw));
        // 转到灰度融合起始角度，切换至phase 3
        if (yaw_diff >= CORNER_GRAY_BLEND_START_DEG) {
            right_angle_phase = 3;
        }

    }
    // phase 3: 灰度接管段，由IMU角度判断退出
    else if (is_right_angle != 0 && right_angle_phase == 3)
    {
        float yaw_diff = fabsf(Track_Angle_Delta_Deg(gyro_yaw_deg,
                                                     gyro_corner_start_yaw));
        // 未达到退出角度，继续等待
        if (yaw_diff < CORNER_IMU_EXIT_DEG) {
            return;
        }

        // 达到退出角度：退出直角状态，清除所有转弯相关状态
        right_angle_phase = 0;
        is_right_angle    = 0;

        Track_Reset_Right_Angle_State();
        // 复位积分与滤波，防止转弯后残余影响直道
        corner_gyro_diff_smooth = 0.0f;
        Left_Integral     = 0.0f;
        Right_Integral    = 0.0f;
        Left_FilteredEnc  = 0.0f;
        Right_FilteredEnc = 0.0f;

        corner_min_current = BASE_RPM;

        is_in_corner = 0;

        // 注册弯道完成，触发圈数计数
        Track_RegisterCornerComplete();
    }
}

void Track_PID_Calc(int32_t err)
{
    PID_Err = (float)err;
    PID_SumErr += PID_Err;

    // PID计算：比例项=偏差*Kp，积分项=累积积分*Ki，微分项=偏差变化*Kd
    float output_raw = KP_NORMAL * PID_Err
                     + KI_NORMAL * PID_SumErr
                     + KD_NORMAL * (PID_Err - PID_LastErr);

    // 输出限幅：防止PWM超调
    output_raw = f_clamp(output_raw, STRAIGHT_RPM_LIMIT * 10.0f);

    // 低通滤波：平滑输出，减少高频抖动
    pid_filtered_output = PID_FILTER_ALPHA * output_raw
                        + (1.0f - PID_FILTER_ALPHA) * pid_filtered_output;

    PID_Output  = (int16_t)pid_filtered_output;
    PID_LastErr = PID_Err;
}

void Track_Action_Execute(void)
{
    float target_base = BASE_RPM;
    float gyro_diff   = 0.0f;
    float pid_correction = 0.0f;
    float raw_pid = (float)PID_Output * 0.1f;

    // PID输出平滑，减少突变
    pid_smooth += CORNER_PID_SMOOTH_ALPHA * (raw_pid - pid_smooth);
    g_last_target_base = BASE_RPM;
    g_last_gyro_diff = 0.0f;
    g_last_pid_correction = pid_smooth;
    g_last_angle_progress = 0.0f;

    if (right_angle_phase == 1U) {
        // 预触发减速：降低基础转速，为转弯做准备
        is_in_corner = 0U;
        target_base = BASE_RPM - CORNER_PREVIEW_DECEL_RPM;
        pid_correction = pid_smooth;
    } else if (is_right_angle != 0 &&
               (right_angle_phase == 2U || right_angle_phase == 3U)) {
        // 转弯中：计算入弯进度和灰度融合权重
        float entry;
        float gray_blend;
        float yaw_diff = 0.0f;
        is_in_corner = 1;
        {
            yaw_diff = fabsf(Track_Angle_Delta_Deg(gyro_yaw_deg, gyro_corner_start_yaw));
            angle_progress = yaw_diff / CORNER_YAW_TARGET;
            if (angle_progress > 1.0f) angle_progress = 1.0f;
        }
        g_last_angle_progress = angle_progress;
        // entry: 入弯平滑权重（0→1），gray_blend: 灰度接管权重（0→1）
        entry = CornerProfile_Smoothstep5(
            angle_progress / CORNER_ENTRY_BLEND_PROGRESS);
        gray_blend = CornerProfile_Smoothstep5(
            (yaw_diff - CORNER_GRAY_BLEND_START_DEG) /
            (CORNER_IMU_EXIT_DEG - CORNER_GRAY_BLEND_START_DEG));

        {
            // 入弯减速-出弯加速曲线：弯中深度越大，基础转速越低
            float exit_speed = CornerProfile_Smoothstep5(
                (angle_progress - CORNER_SPEED_EXIT_START_PROGRESS) /
                CORNER_SPEED_EXIT_BLEND_WIDTH);
            float corner_depth = entry * (1.0f - exit_speed);
            target_base = BASE_RPM - (BASE_RPM - CORNER_TURN_RPM) * corner_depth;
        }

        // 转弯陀螺仪PD闭环：yaw误差*Kp - yaw角速率*Kd
        {
            float yaw_error = Track_Angle_Delta_Deg(gyro_corner_target_yaw,
                                                    gyro_yaw_deg);
            gyro_diff = KP_CORNER_YAW * yaw_error
                      - KD_CORNER_YAW * gyro_yaw_rate;
        }
        gyro_diff = f_clamp(gyro_diff, CORNER_GYRO_DIFF_LIMIT_RPM);
        // entry和gray_blend做权重调配：弯中陀螺仪主导，弯末灰度逐渐接管
        gyro_diff *= entry * (1.0f - gray_blend);
        corner_gyro_diff_smooth = CornerProfile_Slew(
            corner_gyro_diff_smooth, gyro_diff,
            CORNER_DIFF_SLEW_UP_RPM_PER_MS);
        gyro_diff = corner_gyro_diff_smooth;
        g_last_gyro_diff = gyro_diff;

        // 灰度PID修正随灰度融合权重逐步接管
        pid_correction = gray_blend * pid_smooth;
    } else {
        // 正常巡线：使用基础转速+PID修正
        is_in_corner = 0;
        target_base  = BASE_RPM;
        gyro_diff    = 0.0f;
        pid_correction = pid_smooth;
    }

    if (!is_in_corner) {
        // 陀螺仪直道阻尼：利用yaw_rate做直道防抖，抑制车身晃动
        if (GYRO_STRAIGHT_DAMPING_ENABLE) {
            float straight_damping = -KD_GYRO_STRAIGHT * gyro_yaw_rate;
            straight_damping = f_clamp(straight_damping, GYRO_STRAIGHT_LIMIT);
            gyro_diff += straight_damping;
        }
    }

    // 基础转速斜坡限制：限制加减速速率，防止突变
    base_rpm_current = CornerProfile_Slew(
        base_rpm_current, target_base,
        (target_base < base_rpm_current) ? CORNER_BASE_SLEW_DOWN_RPM_PER_MS
                                        : CORNER_BASE_SLEW_UP_RPM_PER_MS);
    g_last_target_base = target_base;
    g_last_pid_correction = pid_correction;

    {
    float previous_left = Left_Base_RPM;
    float previous_right = Right_Base_RPM;
    // 合成左右轮目标转速：左轮=基础转速-差速修正+PID修正，右轮=基础转速+差速修正-PID修正
    Left_Base_RPM  = base_rpm_current - gyro_diff + pid_correction;
    Right_Base_RPM = base_rpm_current + gyro_diff - pid_correction;

    // 直道限幅：防止转速超范围
    if (Left_Base_RPM  < 0.0f) Left_Base_RPM  = 0.0f;
    if (Left_Base_RPM  > STRAIGHT_RPM_LIMIT) Left_Base_RPM  = STRAIGHT_RPM_LIMIT;
    if (Right_Base_RPM < 0.0f) Right_Base_RPM = 0.0f;
    if (Right_Base_RPM > STRAIGHT_RPM_LIMIT) Right_Base_RPM = STRAIGHT_RPM_LIMIT;

    // 弯道限幅：限制转弯时转速范围，防止过冲或失速
    if (is_in_corner) {
        if (Left_Base_RPM  < CORNER_MIN_RPM) Left_Base_RPM  = CORNER_MIN_RPM;
        if (Left_Base_RPM  > CORNER_MAX_RPM) Left_Base_RPM  = CORNER_MAX_RPM;
        if (Right_Base_RPM < CORNER_MIN_RPM) Right_Base_RPM = CORNER_MIN_RPM;
        if (Right_Base_RPM > CORNER_MAX_RPM) Right_Base_RPM = CORNER_MAX_RPM;
    }
    // 轮指令斜率限制：防止相邻周期轮速突变
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

    // 目标转速×修正系数，转换为目标脉冲数
    target_rpm  = target_rpm * correction;
    target_pulse = (target_rpm * (float)PULSE_PER_ROUND * SAMPLE_TIME) / 60.0f;

    // 编码器低通滤波：新值*0.5 + 旧值*0.5
    *filtered_enc = 0.5f * (float)encoder_count + 0.5f * (*filtered_enc);

    bias = target_pulse - (*filtered_enc);

    // 死区：偏差小于1时不积分，防止积分饱和
    if (fabsf(bias) > 1.0f) {
        *integral += KI_VELOCITY * bias;
        // 积分限幅：防止积分项过大
        *integral = f_clamp(*integral, (float)INTEGRAL_LIMIT);
    }

    // 速度环PI：比例项=偏差*Kp，积分项=累积积分，微分项=偏差变化*Kd
    pwm_out = KP_VELOCITY * bias + *integral + KD_VELOCITY * (bias - *last_bias);
    *last_bias = bias;

    // 输出限幅：防止PWM超调
    return i_clamp((int)pwm_out, VELOCITY_PWM_LIMIT);
}

void Track_TIM11_Init(void)
{
    TIM_TimeBaseInitTypeDef  tim_base;
    NVIC_InitTypeDef         nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);

    tim_base.TIM_Prescaler     = 168 - 1;
    tim_base.TIM_Period        = 1000 - 1;
    tim_base.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_base.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM11, &tim_base);

    nvic.NVIC_IRQChannel                   = TIM1_TRG_COM_TIM11_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority  = 0;
    nvic.NVIC_IRQChannelSubPriority         = 0;
    nvic.NVIC_IRQChannelCmd                 = ENABLE;
    NVIC_Init(&nvic);

    TIM_ITConfig(TIM11, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM11, ENABLE);
}

// 1ms定时器中断
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM11, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM11, TIM_IT_Update);
        // 系统时间基准：每1ms递增
        system_time_ms++;
        flag_1ms = 1;

        // 陀螺仪轮询定时：每GYRO_MAIN_POLL_PERIOD_MS ms触发一次读取请求
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
    if (!flag_1ms) return;
    flag_1ms = 0;

    // 灰度采集：数字量读取与滤波
    Track_Gray_Convert();

    // 陀螺仪数据更新：角速率/角加速度计算
    Track_Gyro_Update();

    if (!Track_Run_Enable) {
        Track_Reset_Right_Angle_State();
        Track_Reset_Control_State();
        Track_Send_Corner_State_To_Vision();
        return;
    }

    // 直角检测状态机：检测→预触发→转弯→退出
    Track_Check_Right_Angle();

    Track_Send_Corner_State_To_Vision();

    if (!Track_Run_Enable) {
        return;
    }

    {
        // 巡线偏差计算 → PID控制量计算 → 动作执行
        int32_t err = Track_Calc_Err();
        PID_Err = (float)err;
        Track_PID_Calc(err);
    }

    // 执行动作输出：合成左右轮目标转速
    Track_Action_Execute();
}

// 5ms主任务：速度环
void Track_Main_5ms(void)
{
    if (!flag_5ms) return;
    flag_5ms = 0;
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
    tim7_heartbeat++;

    // 编码器读取：获取左右轮原始脉冲
    Encoder_ReadAll();

    enc_acc_L += Encoder_Left;
    enc_acc_R += Encoder_Right;

    // 方向修正：根据配置翻转编码器符号
    int16_t enc_l = Encoder_Left;
    int16_t enc_r = Encoder_Right;
    if (ENCODER_LEFT_INVERT)  enc_l = -enc_l;
    if (ENCODER_RIGHT_INVERT) enc_r = -enc_r;

    // 脉冲转转速(RPM)，限幅400防止异常值
    Left_Speed_RPM  = (float)enc_l * RPM_COEFFICIENT;
    Right_Speed_RPM = (float)enc_r * RPM_COEFFICIENT;
    Left_Speed_RPM  = f_clamp(Left_Speed_RPM,  400.0f);
    Right_Speed_RPM = f_clamp(Right_Speed_RPM, 400.0f);

    if (Velocity_Loop_Enable) {
        // 速度环PI计算：左右轮独立PI控制，跟踪上层给定的目标转速
        Left_Final_PWM  = Track_Velocity_PI(enc_l,  Left_Base_RPM,
                                            &Left_Integral,  &Left_FilteredEnc,
                                            LEFT_RPM_CORRECTION,  &Left_LastBias);
        Right_Final_PWM = Track_Velocity_PI(enc_r, Right_Base_RPM,
                                            &Right_Integral, &Right_FilteredEnc,
                                            RIGHT_RPM_CORRECTION, &Right_LastBias);
    } else {
        Left_Final_PWM  = 0;
        Right_Final_PWM = 0;
    }

    // PWM输出限幅
    Left_Final_PWM  = i_clamp(Left_Final_PWM,  VELOCITY_PWM_LIMIT);
    Right_Final_PWM = i_clamp(Right_Final_PWM, VELOCITY_PWM_LIMIT);

    // 电机方向修正：根据配置翻转PWM符号
    if (LEFT_MOTOR_REVERSE)  Left_Final_PWM  = -Left_Final_PWM;
    if (RIGHT_MOTOR_REVERSE) Right_Final_PWM = -Right_Final_PWM;

    // 左右交换：支持LR_SWAP硬件配置，最终输出到电机驱动
    if (LR_SWAP) {
        TB6612_SetSpeed(Right_Final_PWM, Left_Final_PWM);
    } else {
        TB6612_SetSpeed(Left_Final_PWM, Right_Final_PWM);
    }
}

void Track_Init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    // 灰度传感器初始化：载入白/黑参考值，计算各通道权重
    Grayscale_Init();
    {
        unsigned short white_ref[8] = {2646, 3088, 3105, 3162, 2807, 2484, 3101, 2786};
        unsigned short black_ref[8] = { 708, 1787, 1351, 1127,  887,  632, 1258, 1123};
        Grayscale_InitCalibrate(&gray_sensor, white_ref, black_ref);

        {
            // 权重计算：原始位置权重×白值均值÷各通道白值，补偿灵敏度差异
            int32_t raw_w[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
            int32_t avg = 0;
            uint8_t j;
            for (j = 0; j < 8; j++)
                avg += gray_sensor.Calibrated_white[j];
            avg /= 8;
            for (j = 0; j < 8; j++)
                ir_weight[j] = raw_w[j] * avg
                             / (int32_t)gray_sensor.Calibrated_white[j];
        }
    }

    // 编码器初始化
    Encoder_Init();

    TB6612_Init(TB6612_PRE, TB6612_PER);

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

    Left_Base_RPM   = 0.0f;
    Right_Base_RPM  = 0.0f;
    Left_Final_PWM  = 0;
    Right_Final_PWM = 0;

    is_right_angle              = 0;
    right_angle_phase           = 0;
    right_angle_initial_flag = 0;
    right_angle_detect_type  = 0;
    corner_gyro_diff_smooth = 0.0f;
    RightAngleDetector_Init(&right_angle_detector);
    right_angle_filtered_bits = 0xFFU;

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

    Left_Integral      = 0.0f;
    Right_Integral     = 0.0f;
    Left_FilteredEnc   = 0.0f;
    Right_FilteredEnc  = 0.0f;
    Left_LastBias      = 0.0f;
    Right_LastBias     = 0.0f;

    // 1ms定时器启动
    Track_TIM11_Init();

    // 循环初始化，直到BNO080真正启动并输出首帧yaw
    while (!gyro_yaw_available) {
        BNO080_I2C_Init();
        bno080_ok = bno080_init();
        if (bno080_ok) {
            while (!gyro_yaw_available) {
                gyro_poll_request = 1U;
                Track_Main_Gyro();
                Track_Gyro_Update();
            }
        }
    }

    Track_Target_Laps = 1U;
    Track_Completed_Laps = 0U;
    Track_Corner_Count = 0U;
    Track_ControlStop();
}
