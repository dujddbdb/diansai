#ifndef __BNO080_H__
#define __BNO080_H__

#include "stm32f4xx.h"

#define BNO080_ADDR             0x4B             // BNO080 I2C 7位地址
#define BNO080_SLAVE_ADDR_WR    (BNO080_ADDR << 1)      // I2C写地址(8位)
#define BNO080_SLAVE_ADDR_RD    ((BNO080_ADDR << 1) | 0x01) // I2C读地址(8位)

#define CHANNEL_COMMAND         0    // SHTP命令通道
#define CHANNEL_EXECUTABLE      1    // SHTP可执行通道(固件/复位)
#define CHANNEL_CONTROL         2    // SHTP控制通道(SET_FEATURE)
#define CHANNEL_REPORTS         3    // SHTP传感器报告通道
#define CHANNEL_WAKE_REPORTS    4    // SHTP唤醒报告通道
#define CHANNEL_GYRO            5    // SHTP陀螺仪通道

#define SHTP_HEADER_LENGTH      4    // SHTP包头长度(字节)
#define MAX_PACKET_SIZE         128  // SHTP最大载荷长度(字节)
#define I2C_BUFFER_LENGTH       32   // I2C分块读取缓冲区大小(字节)

// SHTP数据包结构体
typedef struct {
    uint8_t  header[SHTP_HEADER_LENGTH]; // 包头(长度2B+通道1B+序号1B)
    uint8_t  data[MAX_PACKET_SIZE];      // 载荷数据
    uint16_t data_length;                // 载荷有效长度
    uint8_t  channel_number;             // SHTP通道号(0~5)
    uint8_t  sequence_number;            // 包序号
} SHTP_Packet_TypeDef;

// 旋转向量(四元数)
typedef struct {
    float   i, j, k, real;  // 四元数分量(real=w实部, ijk虚部)
    float   rad_accuracy;    // 半径精度(弧度)
    uint8_t accuracy;        // 精度等级 0-不可靠 1-低 2-中 3-高
} BNO080_RotationVector_t;

extern volatile uint32_t g_exti5_count; // EXTI5中断计数(调试)

// I2C+INTN初始化
void    BNO080_I2C_Init(void);
// SHTP软复位
void    bsp_i2c_soft_reset(void);
// 检测I2C设备,返回1-检测到 0-未检测到
uint8_t bsp_i2c_device_detect(void);
// 接收SHTP数据包,返回1-成功 0-失败
uint8_t bsp_i2c_receive_packet(SHTP_Packet_TypeDef *pkt);
// 发送SHTP数据包,返回1-成功 0-失败
uint8_t bsp_i2c_send_packet(uint8_t ch, const uint8_t *data, uint16_t len);
// 阻塞等待数据就绪,timeout_ms超时(ms),返回1-就绪 0-超时
uint8_t bsp_i2c_wait_for_data(uint32_t timeout_ms);
// 非阻塞查询数据就绪标志,返回1-有数据 0-无数据
uint8_t bsp_i2c_get_data_ready_flag(void);
// 清除数据就绪软标志
void    bsp_i2c_clear_data_ready_flag(void);

// BNO080完整初始化,返回1-成功 0-失败
uint8_t bno080_init(void);
// 轻量恢复旋转向量报告,返回1-成功 0-失败
// 主循环更新(非阻塞),返回1-有新数据 0-无数据
uint8_t bno080_update(void);
// 检查新数据可用(消费型),返回1-有新数据 0-无
uint8_t bno080_data_available(void);
// 获取最新旋转向量(只读指针)
const BNO080_RotationVector_t *bno080_get_rotation_vector(void);
// 四元数转欧拉角(度),输出roll/pitch/yaw
void bno080_get_euler(float *roll_deg, float *pitch_deg, float *yaw_deg);
// TIM10初始化(2ms周期,用于I2C通信隔离)
void BNO080_TIM10_Init(void);

#endif
