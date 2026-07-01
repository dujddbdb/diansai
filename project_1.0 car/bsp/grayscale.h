#ifndef __GRAYSCALE_H__
#define __GRAYSCALE_H__

#include "stm32f4xx.h"
#include <string.h>
#include "track_config.h"

// ADC分辨率选项枚举值
#define ADC_8Bits  3    // 8位ADC分辨率，对应量程0~255
#define ADC_10Bits 2    // 10位ADC分辨率，对应量程0~1023
#define ADC_12Bits 1    // 12位ADC分辨率，对应量程0~4095
#define ADC_14Bits 0    // 14位ADC分辨率，对应量程0~16383

// 当前使用的ADC分辨率配置（底层硬件配置，根据实际ADC硬件选择）
#define GRAY_ADC_BITS ADC_12Bits

// 默认白阈值（8通道，可被上层InitCalibrate函数覆盖）
// 通道编号：CH8(最左) ~ CH1(最右)
#define GRAY_W0  2646    // CH8（最左侧通道）白色校准基准阈值
#define GRAY_W1  3088    // CH7 白色校准基准阈值
#define GRAY_W2  3105    // CH6 白色校准基准阈值
#define GRAY_W3  3162    // CH5 白色校准基准阈值
#define GRAY_W4  2807    // CH4（中间通道）白色校准基准阈值
#define GRAY_W5  2484    // CH3 白色校准基准阈值
#define GRAY_W6  3101    // CH2 白色校准基准阈值
#define GRAY_W7  2786    // CH1（最右侧通道）白色校准基准阈值

// 默认黑阈值（8通道，可被上层InitCalibrate函数覆盖）
#define GRAY_B0   780    // CH8（最左侧通道）黑色校准基准阈值
#define GRAY_B1  1965    // CH7 黑色校准基准阈值
#define GRAY_B2  1486    // CH6 黑色校准基准阈值
#define GRAY_B3  1240    // CH5 黑色校准基准阈值
#define GRAY_B4   976    // CH4（中间通道）黑色校准基准阈值
#define GRAY_B5   695    // CH3 黑色校准基准阈值
#define GRAY_B6  1384    // CH2 黑色校准基准阈值
#define GRAY_B7  1235    // CH1（最右侧通道）黑色校准基准阈值

// 地址线引脚宏定义（底层硬件GPIO配置，用于多路选择器通道切换）
#define GRAY_ADDR0(n) GPIO_WriteBit(GPIOB, GPIO_Pin_14, (n) ? Bit_SET : Bit_RESET) // AD0地址线，PB14引脚
#define GRAY_ADDR1(n) GPIO_WriteBit(GPIOB, GPIO_Pin_13, (n) ? Bit_SET : Bit_RESET) // AD1地址线，PB13引脚
#define GRAY_ADDR2(n) GPIO_WriteBit(GPIOB, GPIO_Pin_12, (n) ? Bit_SET : Bit_RESET) // AD2地址线，PB12引脚

// 8通道灰度传感器数据结构体
// 存储传感器所有状态数据，包括原始值、滤波值、校准参数等
typedef struct {
    unsigned short Analog_raw[8];        // 原始ADC采样值数组（8通道，经8次均值滤波）
    unsigned short Analog_value[8];      // EMA指数移动平均滤波后的ADC值数组（8通道）
    unsigned short Normal_value[8];      // 归一化后的值数组（8通道，范围0~ADC量程）
    unsigned short Calibrated_white[8];  // 白校准基准值数组（8通道，实测白色地面的ADC值）
    unsigned short Calibrated_black[8];  // 黑校准基准值数组（8通道，实测黑色赛道的ADC值）
    unsigned short Gray_white[8];        // 二值化白阈值数组（8通道，高于此值判为白色）
    unsigned short Gray_black[8];        // 二值化黑阈值数组（8通道，低于此值判为黑色）
    double Normal_factor[8];             // 归一化系数数组（8通道，用于黑白之间的线性映射）
    double bits;                         // ADC量程值（根据GRAY_ADC_BITS计算，如12位为4095.0）
    unsigned char Digtal;                // 8位数字量输出（每位对应一个通道，bit=1表示白色，bit=0表示黑色）
    unsigned char Time_out;              // 采样超时阈值（单位：时基tick数，超时则判定传感器异常）
    unsigned char Tick;                  // 时基计数器（用于超时检测和采样节拍控制）
    unsigned char ok;                    // 校准完成标志：0-未校准，1-校准完成
    unsigned char analog_ema_init;       // EMA滤波初始化标志：0-未初始化，1-已初始化（首次采样直接赋值）
} GrayscaleSensor_t;

// 灰度传感器硬件初始化
// 功能：初始化ADC外设和GPIO引脚（地址线、ADC输入）
// 参数：无
// 返回值：无
void Grayscale_Init(void);

// 读取单次ADC转换值
// 功能：启动一次ADC转换并读取结果（选择当前地址线对应的通道）
// 参数：无
// 返回值：unsigned short - ADC转换结果（范围0~ADC量程）
unsigned short Grayscale_ADC_Read(void);

// 初始化传感器结构体
// 功能：将传感器结构体所有字段清零，恢复初始状态
// 参数：
//   s - 指向GrayscaleSensor_t结构体的指针（待初始化的传感器实例）
// 返回值：无
void Grayscale_InitFirst(GrayscaleSensor_t *s);

// 使用实测黑白基准值完成传感器校准
// 功能：根据实测的白色和黑色ADC值，计算归一化系数和二值化阈值
// 参数：
//   s     - 指向GrayscaleSensor_t结构体的指针（待校准的传感器实例）
//   white - 白色基准值数组指针（8通道实测白色地面ADC值）
//   black - 黑色基准值数组指针（8通道实测黑色赛道ADC值）
// 返回值：无
void Grayscale_InitCalibrate(GrayscaleSensor_t *s,
                             unsigned short *white, unsigned short *black);

// 传感器主任务函数
// 功能：完成8通道采集→EMA滤波→二值化→归一化的完整处理流程
//       需要周期性调用（建议在定时器中断或主循环中定时调用）
// 参数：
//   s - 指向GrayscaleSensor_t结构体的指针（传感器实例）
// 返回值：无
void Grayscale_Task(GrayscaleSensor_t *s);

// 获取二值化数字量
// 功能：返回8通道的二值化结果（每位代表一个通道）
// 参数：
//   s - 指向GrayscaleSensor_t结构体的指针（传感器实例）
// 返回值：unsigned char - 8位数字量（bit=1表示白色，bit=0表示黑色，bit7对应CH8，bit0对应CH1）
unsigned char Grayscale_GetDigital(GrayscaleSensor_t *s);

// 获取归一化模拟值
// 功能：获取8通道归一化后的ADC值数组（0~ADC量程范围）
// 参数：
//   s   - 指向GrayscaleSensor_t结构体的指针（传感器实例）
//   out - 输出数组指针（用于存储8通道归一化值，需至少8个unsigned short空间）
// 返回值：unsigned char - 状态码：1-成功，0-传感器未校准
unsigned char Grayscale_GetNormalized(GrayscaleSensor_t *s, unsigned short *out);

// 获取原始ADC值（重新采样）
// 功能：重新采集一次8通道原始ADC值并输出
// 参数：
//   s   - 指向GrayscaleSensor_t结构体的指针（传感器实例）
//   out - 输出数组指针（用于存储8通道原始ADC值，需至少8个unsigned short空间）
// 返回值：unsigned char - 状态码：1-成功，0-传感器未校准
unsigned char Grayscale_GetAnalog(GrayscaleSensor_t *s, unsigned short *out);

#endif
