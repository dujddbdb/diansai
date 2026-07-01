#ifndef __BNO080_H__
#define __BNO080_H__

#include "stm32f4xx.h"

// BNO080 I2C 7位设备地址
// BNO080的I2C地址由ADDR引脚决定，ADDR接GND时为0x4B，接VDD时为0x4A
#define BNO080_ADDR             0x4B             // BNO080 I2C 7位设备地址（ADDR引脚接GND时）

// I2C写地址（8位格式，7位地址左移1位，最低位为0表示写）
#define BNO080_SLAVE_ADDR_WR    (BNO080_ADDR << 1)      // I2C写地址（8位格式，bit0=0）

// I2C读地址（8位格式，7位地址左移1位，最低位为1表示读）
#define BNO080_SLAVE_ADDR_RD    ((BNO080_ADDR << 1) | 0x01) // I2C读地址（8位格式，bit0=1）

// SHTP协议通道号定义
// SHTP（Sensor Hub Transport Protocol）是BNO080使用的传输协议，支持多个逻辑通道
#define CHANNEL_COMMAND         0    // SHTP命令通道（用于发送命令和接收响应）
#define CHANNEL_EXECUTABLE      1    // SHTP可执行通道（用于固件更新和复位操作）
#define CHANNEL_CONTROL         2    // SHTP控制通道（用于配置传感器特性，如SET_FEATURE命令）
#define CHANNEL_REPORTS         3    // SHTP传感器报告通道（用于接收传感器数据报告）
#define CHANNEL_WAKE_REPORTS    4    // SHTP唤醒报告通道（用于低功耗唤醒时的传感器报告）
#define CHANNEL_GYRO            5    // SHTP陀螺仪专用通道（用于高频率陀螺仪数据）

// SHTP数据包相关常量
#define SHTP_HEADER_LENGTH      4    // SHTP包头长度（字节），包含长度(2B)+通道(1B)+序号(1B)
#define MAX_PACKET_SIZE         128  // SHTP最大载荷数据长度（字节），单包最大128字节
#define I2C_BUFFER_LENGTH       32   // I2C分块读取缓冲区大小（字节），I2C单次读取的最大字节数

// SHTP数据包结构体
// 用于存储SHTP协议的完整数据包，包括包头、载荷及解析后的信息
typedef struct {
    uint8_t  header[SHTP_HEADER_LENGTH]; // SHTP包头原始数据（4字节：长度低字节、长度高字节、通道号、包序号）
    uint8_t  data[MAX_PACKET_SIZE];      // SHTP载荷数据缓冲区（最大128字节有效数据）
    uint16_t data_length;                // 载荷有效数据长度（字节），不包含包头
    uint8_t  channel_number;             // SHTP通道号（0~5，对应CHANNEL_xxx宏定义）
    uint8_t  sequence_number;            // 数据包序号（每个通道独立计数，用于检测丢包和重传）
} SHTP_Packet_TypeDef;

// 旋转向量（四元数）数据结构体
// 存储BNO080输出的融合姿态数据，以四元数形式表示
typedef struct {
    float   i, j, k, real;  // 四元数分量：real=w（实部），i/j/k（虚部，对应x/y/z轴）
    float   rad_accuracy;    // 姿态估计精度（半径精度，单位：弧度），值越小精度越高
    uint8_t accuracy;        // 精度等级：0-不可靠，1-低精度，2-中精度，3-高精度
} BNO080_RotationVector_t;

// EXTI5中断计数器（调试用）
// 每次BNO080的INTN引脚触发外部中断时递增，用于调试和统计中断触发频率
extern volatile uint32_t g_exti5_count; // EXTI5中断触发计数（调试用，统计BNO080中断次数）

// BNO080 I2C硬件初始化
// 功能：初始化I2C外设、GPIO引脚（SDA/SCL/INTN），配置外部中断
//       INTN引脚配置为下降沿触发的外部中断，用于通知主机有数据可读
// 参数：无
// 返回值：无
void    BNO080_I2C_Init(void);

// I2C软件复位
// 功能：通过I2C总线发送SHTP软件复位命令，复位BNO080芯片内部状态
//       复位后需要等待芯片重新启动（约100ms）
// 参数：无
// 返回值：无
void    bsp_i2c_soft_reset(void);

// I2C设备检测
// 功能：扫描I2C总线，检测BNO080设备是否存在并响应
// 参数：无
// 返回值：uint8_t - 检测结果：1-检测到设备，0-未检测到设备
uint8_t bsp_i2c_device_detect(void);

// 接收SHTP数据包
// 功能：通过I2C从BNO080读取一个完整的SHTP数据包，解析包头信息
//       读取失败或数据无效时返回0
// 参数：
//   pkt - 指向SHTP_Packet_TypeDef结构体的指针（用于存储接收到的数据包）
// 返回值：uint8_t - 接收结果：1-接收成功，0-接收失败
uint8_t bsp_i2c_receive_packet(SHTP_Packet_TypeDef *pkt);

// 发送SHTP数据包
// 功能：通过I2C向BNO080发送一个SHTP数据包，自动添加包头（长度、通道、序号）
//       发送前会等待设备就绪，发送失败时返回0
// 参数：
//   ch   - SHTP通道号（0~5，使用CHANNEL_xxx宏定义）
//   data - 指向待发送载荷数据的缓冲区指针
//   len  - 待发送载荷数据长度（字节），不超过MAX_PACKET_SIZE
// 返回值：uint8_t - 发送结果：1-发送成功，0-发送失败
uint8_t bsp_i2c_send_packet(uint8_t ch, const uint8_t *data, uint16_t len);

// 阻塞等待数据就绪
// 功能：阻塞等待BNO080的INTN引脚触发（数据就绪），或直到超时
//       通过检测数据就绪标志位实现，适用于初始化阶段的同步操作
// 参数：
//   timeout_ms - 超时时间（毫秒），0表示无限等待
// 返回值：uint8_t - 等待结果：1-数据就绪，0-等待超时
uint8_t bsp_i2c_wait_for_data(uint32_t timeout_ms);

// 非阻塞查询数据就绪标志
// 功能：查询数据就绪软标志位状态，不阻塞，立即返回
//       注意：这是查询软标志，不会清除标志
// 参数：无
// 返回值：uint8_t - 标志状态：1-有数据就绪，0-无数据
uint8_t bsp_i2c_get_data_ready_flag(void);

// 清除数据就绪软标志
// 功能：清除数据就绪标志位，为下一次数据接收做准备
//       在处理完接收到的数据后调用
// 参数：无
// 返回值：无
void    bsp_i2c_clear_data_ready_flag(void);

// BNO080完整初始化
// 功能：完成BNO080的全流程初始化，包括硬件初始化、设备检测、软复位、
//       启用旋转向量传感器报告、配置报告周期等
//       调用此函数后即可使用bno080_update()获取姿态数据
// 参数：无
// 返回值：uint8_t - 初始化结果：1-成功，0-失败（可能原因：设备未连接、通信错误等）
uint8_t bno080_init(void);

// BNO080主循环更新（非阻塞）
// 功能：查询是否有新的传感器数据，有则解析并更新内部旋转向量数据
//       应在主循环中周期性调用（建议调用频率高于传感器报告频率）
//       该函数非阻塞，无新数据时立即返回0
// 参数：无
// 返回值：uint8_t - 更新结果：1-有新数据已更新，0-无新数据
uint8_t bno080_update(void);

// 检查新数据是否可用（消费型）
// 功能：检查是否有未读取的新姿态数据，读取后自动清除"新数据"标志
//       用于上层应用判断是否需要读取最新数据
// 参数：无
// 返回值：uint8_t - 数据状态：1-有新数据可用，0-无新数据
uint8_t bno080_data_available(void);

// 获取最新旋转向量（只读指针）
// 功能：返回当前最新旋转向量数据的只读指针
//       注意：返回的是内部数据指针，请勿修改其内容
//       如需修改，请先复制到本地变量
// 参数：无
// 返回值：const BNO080_RotationVector_t* - 指向旋转向量结构体的只读指针
const BNO080_RotationVector_t *bno080_get_rotation_vector(void);

// 四元数转换为欧拉角（角度制）
// 功能：将当前旋转向量（四元数）转换为欧拉角（横滚/俯仰/偏航），单位为度
//       转换顺序：ZYX（偏航-俯仰-横滚）
// 参数：
//   roll_deg  - 指向float的指针，用于输出横滚角（Roll，绕x轴），单位：度
//   pitch_deg - 指向float的指针，用于输出俯仰角（Pitch，绕y轴），单位：度
//   yaw_deg   - 指向float的指针，用于输出偏航角（Yaw，绕z轴），单位：度
// 返回值：无
void bno080_get_euler(float *roll_deg, float *pitch_deg, float *yaw_deg);

// TIM10定时器初始化
// 功能：初始化TIM10定时器，配置为2ms周期中断
//       用于I2C通信的时间隔离，防止I2C操作过于频繁
//       定时器中断中可实现I2C通信的定时调度
// 参数：无
// 返回值：无
void BNO080_TIM10_Init(void);

#endif
