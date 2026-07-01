
// @file    grayscale.h
// @brief   感为8通道灰度传感器驱动 (无MCU版, 适配STM32F407)
// @note    硬件: PC0=ADC1_IN10(模拟输出OUT), PB14=AD0, PB13=AD1, PB12=AD2(3位地址选择)
//          传感器需5V供电, 与单片机共地
//          通过3位地址线切换8个通道(000~111), 每次切换后延时50us等待信号稳定
//          每个通道采集8次ADC取均值滤波
//
//          感为官方参考: No_Mcu_Ganv_Grayscale_Sensor_Config.h
//          本文件基于官方驱动重构, 保留原API命名以兼容现有工程

#ifndef __GRAYSCALE_H__
#define __GRAYSCALE_H__

#include "stm32f4xx.h"
#include <string.h>

// ADC分辨率配置枚举值
// 用于配置GRAY_ADC_BITS宏，选择ADC的分辨率档位
#define ADC_8Bits  3    // 8位ADC分辨率，对应满量程255
#define ADC_10Bits 2    // 10位ADC分辨率，对应满量程1024
#define ADC_12Bits 1    // 12位ADC分辨率，对应满量程4096
#define ADC_14Bits 0    // 14位ADC分辨率，对应满量程16384

// 用户可配置区域
// 输出结果方向控制
// 0 = 正常顺序（通道0对应传感器物理通道1）
// 1 = 反向顺序（通道0对应传感器物理通道8）
#define GRAY_DIRECTION 1

// ADC分辨率选择，四选一（根据硬件实际ADC配置选择）
#define GRAY_ADC_BITS ADC_12Bits

// 8通道灰度传感器白阈值（每通道独立配置，需根据实际硬件实测后填入）
// 判定规则：ADC值 > 白阈值(W) → 判为白线（数字量对应位置1）
// 标定方法：将传感器对准白色表面，读取ADC值后填入对应通道
#define GRAY_W0  2840   // 通道0白阈值
#define GRAY_W1  3030   // 通道1白阈值
#define GRAY_W2  2830   // 通道2白阈值
#define GRAY_W3  3100   // 通道3白阈值
#define GRAY_W4  2695   // 通道4白阈值
#define GRAY_W5  2415   // 通道5白阈值
#define GRAY_W6  3045   // 通道6白阈值
#define GRAY_W7  2175   // 通道7白阈值

// 8通道灰度传感器黑阈值（每通道独立配置，需根据实际硬件实测后填入）
// 判定规则：ADC值 < 黑阈值(B) → 判为黑线（数字量对应位清0）
// 介于白阈值和黑阈值之间 → 灰色区域（保持原值不变）
// 标定方法：将传感器对准黑色表面，读取ADC值后填入对应通道
#define GRAY_B0  580    // 通道0黑阈值
#define GRAY_B1  1110   // 通道1黑阈值
#define GRAY_B2  695    // 通道2黑阈值
#define GRAY_B3  735    // 通道3黑阈值
#define GRAY_B4  690    // 通道4黑阈值
#define GRAY_B5  700    // 通道5黑阈值
#define GRAY_B6  780    // 通道6黑阈值
#define GRAY_B7  330    // 通道7黑阈值

// 硬件抽象层配置
// 地址选择线操作宏（AD0为最低位LSB，AD1，AD2为最高位MSB）
// 对应硬件引脚：PB14=AD0, PB13=AD1, PB12=AD2
// 注意：传感器地址线为低电平有效，宏内部已处理电平逻辑
// 参数n：0 = 选中该地址位（输出低电平），1 = 未选中（输出高电平）
#define GRAY_ADDR0(n) GPIO_WriteBit(GPIOB, GPIO_Pin_14, (n) ? Bit_SET : Bit_RESET)  // 地址线AD0控制
#define GRAY_ADDR1(n) GPIO_WriteBit(GPIOB, GPIO_Pin_13, (n) ? Bit_SET : Bit_RESET)  // 地址线AD1控制
#define GRAY_ADDR2(n) GPIO_WriteBit(GPIOB, GPIO_Pin_12, (n) ? Bit_SET : Bit_RESET)  // 地址线AD2控制

// 灰度传感器数据结构
// 存储传感器的所有状态数据、校准参数和处理结果
typedef struct {
    unsigned short Analog_value[8];     // 8通道原始ADC模拟值数组，范围0~4095（12位ADC）
    unsigned short Normal_value[8];     // 8通道归一化后的模拟值数组，范围0~4095（线性映射到满量程）
    unsigned short Calibrated_white[8]; // 8通道白色校准基准ADC值，校准时对准白色表面采集
    unsigned short Calibrated_black[8]; // 8通道黑色校准基准ADC值，校准时对准黑色表面采集
    unsigned short Gray_white[8];       // 8通道白灰度阈值，计算公式：(2*白基准 + 黑基准)/3，偏白侧阈值
    unsigned short Gray_black[8];       // 8通道黑灰度阈值，计算公式：(白基准 + 2*黑基准)/3，偏黑侧阈值
    double Normal_factor[8];            // 8通道归一化系数，计算公式：满量程/(白基准 - 黑基准)
    double bits;                        // ADC满量程值（如12位ADC对应4096.0）
    unsigned char Digtal;               // 8位数字量输出，每bit对应一个通道（bit7~bit0对应通道7~0）
    unsigned char Time_out;             // 采样超时阈值/时基阈值，用于定时器模式下的采样间隔控制
    unsigned char Tick;                 // 时基计数器，定时器模式下使用，用于控制采样频率
    unsigned char ok;                   // 校准完成标志位，1 = 已完成校准可正常使用，0 = 未校准
} GrayscaleSensor_t;

// 函数声明

// 灰度传感器硬件初始化
// 功能：配置ADC1通道10（PC0）为模拟输入，配置PB12~PB14为推挽输出（地址选择线）
//       ADC配置为12位分辨率、独立模式、PCLK2/4=21MHz时钟、软件触发转换
// 参数：无
// 返回值：无
void Grayscale_Init(void);

// 读取ADC1通道10的单次转换值
// 功能：启动一次ADC1通道10的软件触发转换，等待转换完成后读取结果
// 参数：无
// 返回值：unsigned short - ADC转换结果（12位时范围0-4095），超时则返回0
unsigned short Grayscale_ADC_Read(void);

// 首次初始化传感器结构体（不含校准数据）
// 功能：清零传感器结构体的所有数据，将校准标志ok置为0，归一化系数置0
//       调用此函数后传感器处于未校准状态，需先采集黑白基准值
//       再调用Grayscale_InitCalibrate完成校准
// 参数：
//   s - 传感器结构体指针，指向要初始化的GrayscaleSensor_t变量
// 返回值：无
void Grayscale_InitFirst(GrayscaleSensor_t *s);

// 带校准参数的完整初始化（对应官方No_MCU_Ganv_Sensor_Init）
// 功能：根据传入的黑白基准值完成传感器校准，计算灰度阈值和归一化系数
//       每个通道的white/black可以不同，实现逐通道独立校准
//       校准完成后将ok标志置1
// 计算公式：
//   Gray_white = (white*2 + black) / 3  （偏白侧阈值，2/3白+1/3黑）
//   Gray_black = (white + black*2) / 3  （偏黑侧阈值，1/3白+2/3黑）
//   Normal_factor = 满量程 / (white - black)  （归一化系数）
// 参数：
//   s     - 传感器结构体指针
//   white - 8通道白色基准ADC值数组指针（传感器对准白色表面时的读数）
//   black - 8通道黑色基准ADC值数组指针（传感器对准黑色表面时的读数）
// 返回值：无
void Grayscale_InitCalibrate(GrayscaleSensor_t *s,
                             unsigned short *white, unsigned short *black);

// 传感器任务函数（主循环中调用）
// 功能：执行一次完整的8通道传感器数据采集与处理流程
//       处理流程：8通道ADC采集（每通道8次均值滤波）→ 二值化处理 → 归一化处理
//       无时基限制模式，每次调用立即执行完整采集流程
// 参数：
//   s - 传感器结构体指针，采集结果存储在此结构体中
// 返回值：无
void Grayscale_Task(GrayscaleSensor_t *s);

// 获取二值化数字量（对应官方Get_Digtal_For_User）
// 功能：从传感器结构体中获取8位二值化数字量结果
// 参数：
//   s - 传感器结构体指针
// 返回值：unsigned char - 8位数字量
//         bit7~bit0 对应通道7~通道0
//         1 = 白线（ADC值大于白阈值）
//         0 = 黑线（ADC值小于黑阈值）
//         灰色区间保持上一次的值不变
unsigned char Grayscale_GetDigital(GrayscaleSensor_t *s);

// 获取归一化模拟值（对应官方Get_Normalize_For_User）
// 功能：获取8通道归一化后的模拟值（已线性映射到0~满量程范围）
// 参数：
//   s   - 传感器结构体指针
//   out - 输出缓冲区指针，用于存放8个unsigned short类型的归一化值（共16字节）
// 返回值：unsigned char - 状态码
//         1 = 成功（传感器已校准）
//         0 = 失败（传感器未校准，数据不可用）
unsigned char Grayscale_GetNormalized(GrayscaleSensor_t *s, unsigned short *out);

// 获取原始ADC模拟值（重新采样，对应官方Get_Anolog_Value）
// 功能：重新执行8通道ADC采集（含均值滤波），返回原始ADC模拟值
//       每次调用都会触发新的采集，不同于直接读取结构体中的缓存
// 参数：
//   s   - 传感器结构体指针
//   out - 输出缓冲区指针，用于存放8个unsigned short类型的原始ADC值（共16字节）
// 返回值：unsigned char - 状态码
//         1 = 成功（传感器已校准）
//         0 = 失败（传感器未校准，数据不可用）
unsigned char Grayscale_GetAnalog(GrayscaleSensor_t *s, unsigned short *out);

#endif
