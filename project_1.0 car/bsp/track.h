#ifndef __TRACK_H__
#define __TRACK_H__

#include "stm32f4xx.h"
#include "track_config.h"
#include "grayscale.h"

// 灰度传感器实例（全局共享的灰度传感器数据结构）
extern GrayscaleSensor_t gray_sensor;

// 红外传感器权重数组（8个通道的加权系数，用于计算加权误差）
extern int32_t  ir_weight[8];
// 红外原始数据数组（8个通道的原始采集值，0-255）
extern uint8_t  ir_raw[8];
// 是否检测到黑线标志（0-未检测到黑线，1-检测到黑线）
extern uint8_t  has_black;

// 当前巡线误差（PID控制器的输入误差值）
extern float    PID_Err;
// 上一次巡线误差（用于PID微分项计算）
extern float    PID_LastErr;
// 巡线误差积分（用于PID积分项累积）
extern float    PID_SumErr;
// 巡线PID输出（PID计算后的原始输出值）
extern int16_t  PID_Output;
// 滤波后的PID输出（经过滤波处理的PID输出）
extern float    pid_filtered_output;
// 上一次有效误差值（丢失黑线时保持的最后有效误差）
extern int32_t  last_valid_err;

// 左轮基础目标转速（巡线直行时的基础转速，单位RPM）
extern volatile float  Left_Base_RPM;
// 右轮基础目标转速（巡线直行时的基础转速，单位RPM）
extern volatile float  Right_Base_RPM;
// 左轮最终PWM输出（经过速度环和限幅后的最终PWM值）
extern volatile int    Left_Final_PWM;
// 右轮最终PWM输出（经过速度环和限幅后的最终PWM值）
extern volatile int    Right_Final_PWM;
// 左轮当前转速（编码器实测转速，单位RPM）
extern volatile float  Left_Speed_RPM;
// 右轮当前转速（编码器实测转速，单位RPM）
extern volatile float  Right_Speed_RPM;
// 速度环使能标志（0-禁用速度环开环运行，1-使能速度环闭环控制）
extern volatile uint8_t Velocity_Loop_Enable;
// 巡线运行使能标志（0-停止巡线电机停转，1-运行巡线控制）
extern volatile uint8_t Track_Run_Enable;
// 目标圈数（设置的比赛总圈数）
extern volatile uint16_t Track_Target_Laps;
// 已完成圈数（当前已经完成的圈数）
extern volatile uint16_t Track_Completed_Laps;
// 已过弯计数（每圈清零，用于统计单圈过弯数量）
extern volatile uint16_t Track_Corner_Count;

// 直角检测结果（0-无直角，1-右直角，2-左直角）
extern uint32_t is_right_angle;
// 直角处理阶段（0-空闲，1-预触发，2-执行转弯，3-退出过渡）
extern uint32_t right_angle_phase;
// 直角初始触发标志（0-未触发，1-右直角触发，2-左直角触发）
extern uint32_t right_angle_initial_flag;
// 直角检测类型（与is_right_angle相同，用于内部状态判断）
extern uint32_t right_angle_detect_type;

// 当前偏航角度（陀螺仪测量的累计偏航角，单位度）
extern volatile float gyro_yaw_deg;
// 陀螺仪数据有效标志（0-数据无效，1-数据有效）
extern volatile uint32_t gyro_yaw_available;

// 陀螺仪帧计数（成功读取的陀螺仪数据帧数）
extern volatile uint16_t gyro_frame_count;
// 陀螺仪中断计数（陀螺仪产生的中断次数）
extern volatile uint16_t gyro_isr_count;
// BNO080正常读取计数（BNO080传感器正常通讯的计数）
extern volatile uint16_t bno080_ok_count;

// 原始偏航角（陀螺仪直接输出的原始偏航角数据）
extern volatile float gyro_raw_yaw;
// 原始横滚角（陀螺仪直接输出的原始横滚角数据）
extern volatile float gyro_raw_roll;
// 原始俯仰角（陀螺仪直接输出的原始俯仰角数据）
extern volatile float gyro_raw_pitch;

// 转弯起始偏航角（进入弯道时记录的起始偏航角）
extern float gyro_corner_start_yaw;
// 转弯目标偏航角（弯道目标偏航角，用于判断转弯进度）
extern float gyro_corner_target_yaw;
// 上一次偏航角（上一时刻的偏航角，用于计算角速度）
extern float gyro_last_yaw;
// 偏航角加速度（偏航角的角加速度，单位度/ms^2）
extern volatile float gyro_yaw_accel;
// 偏航角速度（偏航角的角速度，单位度/ms）
extern volatile float gyro_yaw_rate;
// 首帧有效标志（0-首帧未获取，1-首帧数据已获取有效）
extern uint32_t gyro_first_valid;

// 陀螺仪轮询请求标志（0-无请求，1-需要轮询读取数据）
extern volatile uint32_t gyro_poll_request;
// 陀螺仪轮询计时器（轮询间隔计时，单位ms）
extern uint16_t gyro_poll_timer_ms;

// 巡线调试快照结构体（保存某一时刻的巡线关键状态，用于调试分析）
typedef struct {
    volatile int32_t  err;                  // 当前巡线误差值
    volatile float    pid_output_raw;       // PID原始输出（未经滤波）
    volatile float    pid_filtered_output;  // PID滤波输出（经过滤波处理）
    volatile float    gyro_diff;            // 陀螺仪差速补偿值
    volatile float    target_base;          // 目标基础转速
    volatile float    pid_correction;       // PID修正量（叠加到基础转速上的量）
    volatile float    angle_progress;       // 转弯角度进度（0-1，0为起始，1为完成）
} TrackDebugSnapshot_t;

// 调试快照全局变量（保存最新的巡线调试数据）
extern volatile TrackDebugSnapshot_t g_track_debug;

// 左轮编码器累计值（左编码器脉冲累计计数）
extern volatile int32_t enc_acc_L;
// 右轮编码器累计值（右编码器脉冲累计计数）
extern volatile int32_t enc_acc_R;
// TIM7心跳计数（TIM7定时器中断计数，用于系统时基）
extern volatile uint32_t tim7_heartbeat;

// 系统时间（系统启动后的累计时间，单位ms）
extern volatile uint32_t system_time_ms;

// 1ms定时标志（每1ms置位一次，用于任务调度）
extern volatile uint32_t flag_1ms;
// 5ms定时标志（每5ms置位一次，用于任务调度）
extern volatile uint32_t flag_5ms;

// 巡线模块初始化
// 功能：初始化灰度传感器、电机、陀螺仪、定时器等所有巡线相关硬件和状态
// 参数：无
// 返回值：无
void Track_Init(void);

// 启动巡线控制
// 功能：复位所有巡线状态变量，使能速度环，启动巡线
// 参数：无
// 返回值：无
void Track_ControlStart(void);

// 停止巡线控制
// 功能：停止电机运转，复位巡线状态，禁用速度环
// 参数：无
// 返回值：无
void Track_ControlStop(void);

// 目标圈数加1
// 功能：增加目标圈数，不超过TRACK_TARGET_LAPS_MAX最大值
// 参数：无
// 返回值：无
void Track_TargetLapAdd(void);

// 目标圈数减1
// 功能：减少目标圈数，不低于1圈
// 参数：无
// 返回值：无
void Track_TargetLapSub(void);

// 灰度传感器数据转换处理
// 功能：读取灰度传感器模拟量并转换为数字量，更新直角检测器状态
// 参数：无
// 返回值：无
void Track_Gray_Convert(void);

// 陀螺仪数据更新处理
// 功能：在Gyro_TakePollRequest消费数据后调用，计算角速度和角加速度
// 参数：无
// 返回值：无
void Track_Gyro_Update(void);

// 取出陀螺仪轮询请求
// 功能：检查并消费陀螺仪轮询请求，返回请求状态
// 参数：无
// 返回值：0-无请求，1-有请求（已消费并更新计时器）
uint8_t Track_Gyro_TakePollRequest(void);

// 计算巡线误差
// 功能：根据灰度传感器数据计算加权误差值
// 参数：无
// 返回值：加权误差值，范围约-7000~7000（左负右正）
int32_t Track_Calc_Err(void);

// 直角检测状态机
// 功能：每1ms调用一次，处理直角检测和转弯状态机逻辑
// 参数：无
// 返回值：无
void Track_Check_Right_Angle(void);

// 巡线PID计算
// 功能：根据输入误差计算PID输出
// 参数：err - 输入误差值（巡线误差）
// 返回值：无（结果保存在PID_Output等全局变量中）
void Track_PID_Calc(int32_t err);

// 执行电机动作输出
// 功能：合成左右轮目标转速，经过斜坡限幅后输出到电机
// 参数：无
// 返回值：无
void Track_Action_Execute(void);

// 调试阶段处理（旧版兼容）
// 功能：旧版调试阶段处理函数，当前为空实现
// 参数：time - 时间参数
// 返回值：无
void Track_Main_Debug_Stage(uint32_t time);

// TIM11定时器初始化
// 功能：初始化TIM11定时器为1ms周期，产生巡线时基中断
// 参数：无
// 返回值：无
void Track_TIM11_Init(void);

// 速度环PI控制计算
// 功能：每5ms调用一次，执行左右轮速度闭环PI控制
// 参数：无
// 返回值：无
void Track_SpeedLoop(void);

// 1ms定时主任务
// 功能：1ms周期主任务，执行灰度转换→陀螺仪更新→直角检测→PID计算→动作输出
// 参数：无
// 返回值：无
void Track_Main_1ms(void);

// 5ms定时主任务
// 功能：5ms周期主任务，执行速度环计算和电机PWM输出
// 参数：无
// 返回值：无
void Track_Main_5ms(void);

// 陀螺仪轮询主任务
// 功能：消费陀螺仪轮询请求，读取BNO080传感器数据
// 参数：无
// 返回值：无
void Track_Main_Gyro(void);

// 调试输出处理
// 功能：通过USART3打印陀螺仪等调试数据
// 参数：time - 打印间隔时间（单位ms）
// 返回值：无
void Track_Main_Debug(uint32_t time);

// 单电机速度PI控制
// 功能：对单个电机执行速度闭环PI控制计算
// 参数：encoder_count - 编码器脉冲数（有符号，当前速度反馈）
// 参数：target_rpm - 目标转速（单位RPM）
// 参数：integral - 积分项指针（函数内部累积，需外部保存）
// 参数：filtered_enc - 滤波后编码器值指针（输出滤波后的速度值）
// 参数：correction - 转速修正系数（左右轮差异补偿系数）
// 参数：last_bias - 上一次偏差指针（用于微分项计算，需外部保存）
// 返回值：PWM输出值（整数，范围根据配置而定）
int Track_Velocity_PI(int encoder_count, float target_rpm,
                      float *integral, float *filtered_enc,
                      float correction, float *last_bias);

// 左轮速度环积分项（左轮PI控制器的积分累积值）
extern float Left_Integral;
// 右轮速度环积分项（右轮PI控制器的积分累积值）
extern float Right_Integral;
// 左轮滤波编码器值（左轮滤波后的速度反馈值）
extern float Left_FilteredEnc;
// 右轮滤波编码器值（右轮滤波后的速度反馈值）
extern float Right_FilteredEnc;
// 左轮上一次偏差（左轮PI控制器上一次的偏差值，用于微分项）
extern float Left_LastBias;
// 右轮上一次偏差（右轮PI控制器上一次的偏差值，用于微分项）
extern float Right_LastBias;

#endif
