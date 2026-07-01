
// @file    bno080.h
// @brief   BNO080/Bno085 9轴IMU完整驱动接口 (含I2C底层)
// @note    I2C1: PB8(SCL) PB9(SDA), 400kHz
//          INT: PB5, 下降沿触发
//          SHTP协议, 旋转向量+欧拉角输出

#ifndef __BNO080_H__
#define __BNO080_H__

#include "stm32f4xx.h"

// ---- I2C地址 ----

// BNO080传感器7位I2C从机地址（默认出厂值为0x4B）
#define BNO080_ADDR             0x4B

// BNO080 I2C写操作地址（8位格式，最低位=0表示写操作）
// 计算公式：从机地址左移1位，最低位补0
#define BNO080_SLAVE_ADDR_WR    (BNO080_ADDR << 1)

// BNO080 I2C读操作地址（8位格式，最低位=1表示读操作）
// 计算公式：从机地址左移1位，最低位置1
#define BNO080_SLAVE_ADDR_RD    ((BNO080_ADDR << 1) | 0x01)

// ---- SHTP通道定义 ----
// SHTP（Sensor Hub Transport Protocol）是博世传感器使用的传输协议
// 共6个逻辑通道，每个通道负责不同类型的数据传输

#define CHANNEL_COMMAND         0   // SHTP通道0：命令通道，用于发送传感器Hub命令和接收响应
#define CHANNEL_EXECUTABLE      1   // SHTP通道1：可执行通道，用于固件下载和软复位命令
#define CHANNEL_CONTROL         2   // SHTP通道2：控制通道，用于SET_FEATURE配置和产品ID请求
#define CHANNEL_REPORTS         3   // SHTP通道3：传感器报告通道，旋转向量等数据从此通道输出
#define CHANNEL_WAKE_REPORTS    4   // SHTP通道4：唤醒报告通道，低功耗模式下的传感器数据输出
#define CHANNEL_GYRO            5   // SHTP通道5：陀螺仪通道，原始校准用陀螺仪数据输出

// SHTP协议包头固定长度：4字节
// 包头结构：packet_length[2字节] + channel[1字节] + sequence[1字节]
#define SHTP_HEADER_LENGTH      4

// SHTP数据包最大载荷长度（字节）
// 超出此长度的数据包将被截断，防止接收缓冲区溢出
#define MAX_PACKET_SIZE         128

// I2C分块读取缓冲区大小（字节）
// 用于大数据包的分块接收，每次读取I2C_BUFFER_LENGTH字节
#define I2C_BUFFER_LENGTH       32

// ---- SHTP包结构定义 ----

// SHTP数据包结构体
// 用于存储从I2C接收或要发送的SHTP协议完整数据包
typedef struct {
    uint8_t  header[SHTP_HEADER_LENGTH];  // SHTP包头缓冲区（4字节：长度+通道+序号）
    uint8_t  data[MAX_PACKET_SIZE];       // 载荷数据缓冲区，存储包的实际数据内容
    uint16_t data_length;                 // 载荷实际有效长度（字节数），不包含包头
    uint8_t  channel_number;              // SHTP通道号，取值范围0~5，对应不同的逻辑通道
    uint8_t  sequence_number;             // 包序号，用于数据包序号跟踪和流控
} SHTP_Packet_TypeDef;

// ---- 旋转向量数据结构 ----

// BNO080旋转向量数据结构体
// 存储传感器输出的四元数旋转向量及精度信息
typedef struct {
    float   i, j, k, real;          // 四元数分量（i + j + k + real），用于表示三维空间旋转
    float   rad_accuracy;           // 半径精度（弧度），表示旋转向量的估计误差
    uint8_t accuracy;               // 精度等级，0=不可靠，1=低，2=中，3=高精度
} BNO080_RotationVector_t;

// ---- 调试用全局变量 ----

// EXTI5中断触发次数计数器（调试用途）
// 每次BNO080的INT引脚触发外部中断时此变量加1
// 可用于检查传感器数据输出频率和中断是否正常工作
extern volatile uint32_t g_exti5_count;

// ---- I2C底层函数（内部使用，bsp_前缀） ----

// BNO080 I2C接口初始化
// 功能：初始化I2C1外设（PB8=SCL, PB9=SDA），配置为400kHz快速模式
//       配置INTN中断引脚（PB5）为下降沿触发外部中断
//       使能相关GPIO和I2C时钟
// 参数：无
// 返回值：无
void    BNO080_I2C_Init(void);

// 发送SHTP软复位命令（I2C总线重启）
// 功能：通过I2C向BNO080发送软复位命令，复位传感器Hub
//       复位后传感器需要重新初始化和配置
// 参数：无
// 返回值：无
void    bsp_i2c_soft_reset(void);

// 检测BNO080设备是否在I2C总线上（含总线解锁）
// 功能：发送I2C起始信号和设备地址，检测是否有ACK应答
//       如果总线被锁死，会先执行总线解锁序列
// 参数：无
// 返回值：uint8_t - 检测结果
//         1 = 检测到BNO080设备（收到ACK）
//         0 = 未检测到设备（无ACK应答）
uint8_t bsp_i2c_device_detect(void);

// 从I2C接收SHTP数据包
// 功能：从I2C总线读取一个完整的SHTP数据包，解析包头并存储数据
//       支持分块读取大数据包，自动拼接完整数据
// 参数：
//   pkt - 输出参数，指向存储接收到数据包的结构体指针
// 返回值：uint8_t - 接收状态
//         1 = 接收成功
//         0 = 接收失败（超时或通信错误）
uint8_t bsp_i2c_receive_packet(SHTP_Packet_TypeDef *pkt);

// 通过I2C发送SHTP数据包
// 功能：封装SHTP包头，通过I2C向BNO080发送指定通道的数据包
//       自动添加包序号，处理数据包长度
// 参数：
//   ch   - SHTP通道号，取值范围0~5
//   data - 要发送的数据缓冲区指针
//   len  - 要发送的数据长度（字节数）
// 返回值：uint8_t - 发送状态
//         1 = 发送成功
//         0 = 发送失败（超时或通信错误）
uint8_t bsp_i2c_send_packet(uint8_t ch, const uint8_t *data, uint16_t len);

// 阻塞等待数据就绪（INTN下降沿触发）
// 功能：等待BNO080的INT引脚产生下降沿，表示有新数据可读
//       阻塞式等待，直到数据就绪或超时
// 参数：
//   timeout_ms - 超时时间（毫秒）
// 返回值：uint8_t - 等待结果
//         1 = 数据就绪（有新数据可读）
//         0 = 等待超时（未收到数据就绪信号）
uint8_t bsp_i2c_wait_for_data(uint32_t timeout_ms);

// 非阻塞查询数据就绪标志
// 功能：查询是否有新的数据就绪（不阻塞，立即返回）
//       仅读取标志位，不清除标志
// 参数：无
// 返回值：uint8_t - 数据就绪状态
//         1 = 有新数据待读取
//         0 = 无新数据
uint8_t bsp_i2c_get_data_ready_flag(void);

// 清除数据就绪标志
// 功能：清除数据就绪标志位，为下一次数据接收做准备
//       通常在读取完数据包后调用
// 参数：无
// 返回值：无
void    bsp_i2c_clear_data_ready_flag(void);

// ---- BNO080应用层函数 ----

// 完整初始化BNO080传感器
// 功能：执行完整的BNO080初始化流程，包括：
//       1. 检测I2C总线上的设备
//       2. 软复位传感器
//       3. SHTP协议握手
//       4. 使能旋转向量传感器报告
//       初始化成功后传感器开始输出旋转向量数据
// 参数：无
// 返回值：uint8_t - 初始化结果
//         1 = 初始化成功
//         0 = 初始化失败（某一步骤出错）
uint8_t bno080_init(void);

// 主循环更新函数（非阻塞）
// 功能：检测是否有新的传感器数据包到达，如果有则解析并更新内部数据
//       非阻塞函数，应在主循环中定期调用
//       自动解析旋转向量报告并更新内部缓存
// 参数：无
// 返回值：uint8_t - 更新状态
//         1 = 有新数据已更新
//         0 = 无新数据
uint8_t bno080_update(void);

// 检查是否有新的传感器数据（消费型读取）
// 功能：检查数据就绪标志，读取后自动清除标志位
//       用于轮询方式获取新数据通知
//       注意：调用bno080_update()也会处理数据，此函数仅检查标志
// 参数：无
// 返回值：uint8_t - 数据就绪状态
//         1 = 有新数据
//         0 = 无新数据
uint8_t bno080_data_available(void);

// 获取最新旋转向量（四元数格式）
// 功能：返回指向内部旋转向量数据缓存的常量指针
//       数据由bno080_update()函数更新
//       注意：返回的是指针，数据会随更新而变化，如需保存请自行拷贝
// 参数：无
// 返回值：const BNO080_RotationVector_t* - 旋转向量结构体常量指针
const BNO080_RotationVector_t *bno080_get_rotation_vector(void);

// 四元数转欧拉角（角度制，单位：度）
// 功能：将当前旋转向量的四元数转换为欧拉角（滚转/俯仰/偏航）
//       输出角度范围：roll: -180~180, pitch: -90~90, yaw: -180~180
// 参数：
//   roll_deg  - 输出参数，滚转角（绕X轴旋转），单位：度
//   pitch_deg - 输出参数，俯仰角（绕Y轴旋转），单位：度
//   yaw_deg   - 输出参数，偏航角（绕Z轴旋转），单位：度
// 返回值：无
void bno080_get_euler(float *roll_deg, float *pitch_deg, float *yaw_deg);

#endif
