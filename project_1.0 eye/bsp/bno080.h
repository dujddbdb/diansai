/**
 * @file    bno080.h
 * @brief   BNO080/Bno085 9轴IMU完整驱动接口 (含I2C底层)
 * @note    I2C1: PB8(SCL) PB9(SDA), 400kHz
 *          INT: PB5, 下降沿触发
 *          SHTP协议, 旋转向量+欧拉角输出
 */

#ifndef __BNO080_H__
#define __BNO080_H__

#include "stm32f4xx.h"

/* ---- I2C地址 ---- */
#define BNO080_ADDR             0x4B                            // BNO080传感器7位I2C从机地址 (默认值为0x4B)
#define BNO080_SLAVE_ADDR_WR    (BNO080_ADDR << 1)              // BNO080 I2C写操作地址 (8位, 最低位=0表示写)
#define BNO080_SLAVE_ADDR_RD    ((BNO080_ADDR << 1) | 0x01)     // BNO080 I2C读操作地址 (8位, 最低位=1表示读)

/* ---- SHTP通道 ---- */
#define CHANNEL_COMMAND         0   // SHTP通道0: 命令通道 (用于发送传感器hub命令和接收响应)
#define CHANNEL_EXECUTABLE      1   // SHTP通道1: 可执行通道 (用于固件下载和软复位命令)
#define CHANNEL_CONTROL         2   // SHTP通道2: 控制通道 (用于SET_FEATURE配置和产品ID请求)
#define CHANNEL_REPORTS         3   // SHTP通道3: 传感器报告通道 (旋转向量等数据从此通道输出)
#define CHANNEL_WAKE_REPORTS    4   // SHTP通道4: 唤醒报告通道 (低功耗模式下的传感器数据)
#define CHANNEL_GYRO            5   // SHTP通道5: 陀螺仪通道 (原始校准用陀螺仪数据)

#define SHTP_HEADER_LENGTH      4   // SHTP协议包头固定长度: 4字节 (packet_length[2B] + channel[1B] + sequence[1B])
#define MAX_PACKET_SIZE         128 // SHTP数据包最大载荷长度 (字节), 超出部分将被截断以防止缓冲区溢出
#define I2C_BUFFER_LENGTH       32  // I2C分块读取缓冲区大小 (字节), 用于大数据包的分块接收

/* ---- SHTP包结构 ---- */
typedef struct {
    uint8_t  header[SHTP_HEADER_LENGTH];  // SHTP包头缓冲区 (4字节: 长度+通道+序号)
    uint8_t  data[MAX_PACKET_SIZE];       // 载荷数据缓冲区
    uint16_t data_length;                 // 载荷实际有效长度
    uint8_t  channel_number;              // SHTP通道号 (0~5)
    uint8_t  sequence_number;             // 包序号
} SHTP_Packet_TypeDef;

/* ---- 旋转向量 ---- */
typedef struct {
    float   i, j, k, real;          // 四元数分量
    float   rad_accuracy;           // 半径精度
    uint8_t accuracy;               // 精度等级 (0=不可靠, 3=高精度)
} BNO080_RotationVector_t;

/* ---- 调试用全局变量 ---- */
extern volatile uint32_t g_exti5_count; // EXTI5中断触发次数(调试用)

/* ---- I2C底层 (内部使用, bsp_前缀) ---- */

// 初始化I2C1 (PB8=SCL, PB9=SDA) + INTN中断 (PB5, 下降沿)
void    BNO080_I2C_Init(void);

// 发送SHTP软复位命令 (I2C总线重启)
void    bsp_i2c_soft_reset(void);

// 检测BNO080是否在I2C总线上 (含总线解锁), 返回1=检测到 0=未检测到
uint8_t bsp_i2c_device_detect(void);

// 从I2C接收SHTP数据包, pkt: 输出接收到的包, 返回1=成功 0=失败
uint8_t bsp_i2c_receive_packet(SHTP_Packet_TypeDef *pkt);

// 通过I2C发送SHTP数据包, ch: 通道号(0-5), data: 数据, len: 长度, 返回1=成功 0=失败
uint8_t bsp_i2c_send_packet(uint8_t ch, const uint8_t *data, uint16_t len);

// 阻塞等待数据就绪 (INTN下降沿), timeout_ms: 超时时间(ms), 返回1=就绪 0=超时
uint8_t bsp_i2c_wait_for_data(uint32_t timeout_ms);

// 非阻塞查询数据就绪标志, 返回1=有数据 0=无数据
uint8_t bsp_i2c_get_data_ready_flag(void);

// 清除数据就绪标志
void    bsp_i2c_clear_data_ready_flag(void);

/* ---- BNO080应用层 ---- */

// 完整初始化BNO080 (检测+复位+握手+使能旋转向量报告), 返回1=成功 0=失败
uint8_t bno080_init(void);

// 主循环更新函数 (非阻塞, 检测并解析新数据包), 返回1=有新数据 0=无
uint8_t bno080_update(void);

// 运行中重新下发SET_FEATURE恢复旋转向量报告 (数据流中断后恢复)
uint8_t bno080_restart_reports(void);

// 检查是否有新的传感器数据 (消费型: 读取后自动清除), 返回1=有新数据 0=无
uint8_t bno080_data_available(void);

// 获取最新旋转向量 (四元数), 返回指向内部旋转向量的常量指针
const BNO080_RotationVector_t *bno080_get_rotation_vector(void);

// 四元数转欧拉角 (度), roll_deg/pitch_deg/yaw_deg: 输出滚转/俯仰/偏航角
void bno080_get_euler(float *roll_deg, float *pitch_deg, float *yaw_deg);

#endif
