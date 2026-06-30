#ifndef __TRACK_H__
#define __TRACK_H__

#include "stm32f4xx.h"
#include "track_config.h"
#include "grayscale.h"

extern GrayscaleSensor_t gray_sensor;        // 灰度传感器实例

extern int32_t  ir_weight[8];                // 红外传感器权重数组
extern uint8_t  ir_raw[8];                   // 红外原始数据数组
extern uint8_t  has_black;                   // 0-未检测到黑线 1-检测到黑线

extern float    PID_Err;                     // 当前巡线误差
extern float    PID_LastErr;                 // 上一次巡线误差
extern float    PID_SumErr;                  // 巡线误差积分
extern int16_t  PID_Output;                  // 巡线PID输出
extern float    pid_filtered_output;         // 滤波后的PID输出
extern int32_t  last_valid_err;              // 上一次有效误差值

extern volatile float  Left_Base_RPM;        // 左轮基础目标转速
extern volatile float  Right_Base_RPM;       // 右轮基础目标转速
extern volatile int    Left_Final_PWM;       // 左轮最终PWM输出
extern volatile int    Right_Final_PWM;      // 右轮最终PWM输出
extern volatile float  Left_Speed_RPM;       // 左轮当前转速
extern volatile float  Right_Speed_RPM;      // 右轮当前转速
extern volatile uint8_t Velocity_Loop_Enable;// 0-禁用速度环 1-使能
extern volatile uint8_t Track_Run_Enable;    // 0-停止巡线 1-运行
extern volatile uint16_t Track_Target_Laps;  // 目标圈数
extern volatile uint16_t Track_Completed_Laps;// 已完成圈数
extern volatile uint16_t Track_Corner_Count; // 已过弯计数(每圈清零)

extern uint32_t is_right_angle;              // 0-无直角 1-右直角 2-左直角
extern uint32_t right_angle_phase;           // 0-空闲 1-预触发 2-执行转弯 3-退出过渡
extern uint32_t right_angle_initial_flag;    // 0-未触发 1-右直角触发 2-左直角触发
extern uint32_t right_angle_detect_type;     // 直角检测类型(同is_right_angle)

extern volatile float gyro_yaw_deg;          // 当前偏航角度(度)
extern volatile uint32_t gyro_yaw_available; // 0-数据无效 1-数据有效

extern volatile uint16_t gyro_frame_count;   // 陀螺仪帧计数
extern volatile uint16_t gyro_isr_count;     // 陀螺仪中断计数
extern volatile uint16_t bno080_ok_count;    // BNO080正常读取计数

extern volatile float gyro_raw_yaw;          // 原始偏航角
extern volatile float gyro_raw_roll;         // 原始横滚角
extern volatile float gyro_raw_pitch;        // 原始俯仰角

extern float gyro_corner_start_yaw;          // 转弯起始偏航角
extern float gyro_corner_target_yaw;         // 转弯目标偏航角
extern float gyro_last_yaw;                  // 上一次偏航角
extern volatile float gyro_yaw_accel;        // 偏航角加速度
extern volatile float gyro_yaw_rate;         // 偏航角速度(度/ms)
extern uint32_t gyro_first_valid;            // 0-首帧未获取 1-首帧有效

extern volatile uint32_t gyro_poll_request;  // 0-无请求 1-需要轮询读取
extern uint16_t gyro_poll_timer_ms;          // 陀螺仪轮询计时器(ms)

// 巡线调试快照结构体
typedef struct {
    volatile int32_t  err;                  // 当前误差
    volatile float    pid_output_raw;       // PID原始输出
    volatile float    pid_filtered_output;  // PID滤波输出
    volatile float    gyro_diff;            // 陀螺仪差速
    volatile float    target_base;          // 目标基础转速
    volatile float    pid_correction;       // PID修正量
    volatile float    angle_progress;       // 转弯角度进度(0-1)
} TrackDebugSnapshot_t;

extern volatile TrackDebugSnapshot_t g_track_debug; // 调试快照全局变量

extern volatile int32_t enc_acc_L;           // 左轮编码器累计值
extern volatile int32_t enc_acc_R;           // 右轮编码器累计值
extern volatile uint32_t tim7_heartbeat;     // TIM7心跳计数

extern volatile uint32_t system_time_ms;     // 系统时间(ms)

extern volatile uint32_t flag_1ms;           // 1ms定时标志
extern volatile uint32_t flag_5ms;           // 5ms定时标志

// 巡线模块初始化（传感器、电机、陀螺仪、定时器）
void Track_Init(void);
// 启动巡线控制（复位状态，使能速度环）
void Track_ControlStart(void);
// 停止巡线控制（电机停转，复位状态）
void Track_ControlStop(void);
// 目标圈数加1（不超过TRACK_TARGET_LAPS_MAX）
void Track_TargetLapAdd(void);
// 目标圈数减1（不低于1）
void Track_TargetLapSub(void);

// 灰度传感器数据转换处理（读取数字量，更新直角检测器）
void Track_Gray_Convert(void);
// 陀螺仪数据更新处理（在 Gyro_TakePollRequest 消费数据后调用，计算角速度/角加速度）
void Track_Gyro_Update(void);
// 取出陀螺仪轮询请求，返回0-无请求 1-有请求(已消费并更新)
uint8_t Track_Gyro_TakePollRequest(void);
// 计算巡线误差，返回加权误差值(-7000~7000)
int32_t Track_Calc_Err(void);
// 直角检测状态机（每1ms调用）
void Track_Check_Right_Angle(void);
// 巡线PID计算，err为输入误差
void Track_PID_Calc(int32_t err);
// 执行电机动作输出（合成左右轮目标转速，含斜坡限幅）
void Track_Action_Execute(void);
// 调试阶段处理（旧版兼容，空实现）
void Track_Main_Debug_Stage(uint32_t time);
// TIM11定时器初始化(1ms周期，产生时基)
void Track_TIM11_Init(void);
// 速度环PI控制计算（每5ms调用）
void Track_SpeedLoop(void);
// 1ms定时主任务（灰度转换→陀螺仪更新→直角检测→PID→动作输出）
void Track_Main_1ms(void);
// 5ms定时主任务（速度环计算与电机输出）
void Track_Main_5ms(void);
// 陀螺仪轮询主任务（消费轮询请求，读取BNO080数据）
void Track_Main_Gyro(void);
// 调试输出处理（通过USART3打印陀螺仪数据，time为打印间隔ms）
void Track_Main_Debug(uint32_t time);

// 单电机速度PI控制
// encoder_count: 编码器脉冲数(有符号)
// target_rpm: 目标转速(RPM)
// integral: 积分项指针（函数内累积）
// filtered_enc: 滤波后编码器值指针
// correction: 转速修正系数(左右轮差异补偿)
// last_bias: 上一次偏差指针(用于微分项)
// 返回PWM输出值
int Track_Velocity_PI(int encoder_count, float target_rpm,
                      float *integral, float *filtered_enc,
                      float correction, float *last_bias);

extern float Left_Integral;                  // 左轮速度环积分项
extern float Right_Integral;                 // 右轮速度环积分项
extern float Left_FilteredEnc;               // 左轮滤波编码器值
extern float Right_FilteredEnc;              // 右轮滤波编码器值
extern float Left_LastBias;                  // 左轮上一次偏差
extern float Right_LastBias;                 // 右轮上一次偏差

#endif
