
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
#include <string.h>                         /* memset/memcpy函数声明 */

// ADC分辨率配置
#define ADC_8Bits  3    // 8位ADC 量程255
#define ADC_10Bits 2    // 10位ADC 量程1024
#define ADC_12Bits 1    // 12位ADC 量程4096
#define ADC_14Bits 0    // 14位ADC 量程16384

// 用户可配置区域
// 输出结果方向: 0=正常顺序(通道0对应传感器1), 1=反向(通道0对应传感器8)
#define GRAY_DIRECTION 1

// ADC分辨率选择 (根据硬件配置四选一)
#define GRAY_ADC_BITS ADC_12Bits

// 8通道灰度阈值 (每通道独立, 用独立工程实测后填入)
//   ADC值 &gt; 白阈值(W) → 判为白线(数字量置1)
//   ADC值 &lt; 黑阈值(B) → 判为黑线(数字量清0)
//   介于两者之间 → 灰色(保持0)
//   标定方法: 传感器对准白线→用独立工程读ADC值→填入Wx
//            传感器对准黑线→用独立工程读ADC值→填入Bx
#define GRAY_W0  2840
#define GRAY_W1  3030
#define GRAY_W2  2830
#define GRAY_W3  3100
#define GRAY_W4  2695
#define GRAY_W5  2415
#define GRAY_W6  3045
#define GRAY_W7  2175
#define GRAY_B0  580
#define GRAY_B1  1110
#define GRAY_B2  695
#define GRAY_B3  735
#define GRAY_B4  690
#define GRAY_B5  700
#define GRAY_B6  780
#define GRAY_B7  330

// 硬件抽象层配置
// 地址选择线宏 (AD0=LSB, AD1, AD2=MSB) — 对应PB14/PB13/PB12
// 注意取反逻辑: 传感器地址线为低电平有效
#define GRAY_ADDR0(n) GPIO_WriteBit(GPIOB, GPIO_Pin_14, (n) ? Bit_SET : Bit_RESET)
#define GRAY_ADDR1(n) GPIO_WriteBit(GPIOB, GPIO_Pin_13, (n) ? Bit_SET : Bit_RESET)
#define GRAY_ADDR2(n) GPIO_WriteBit(GPIOB, GPIO_Pin_12, (n) ? Bit_SET : Bit_RESET)

// 传感器数据结构
typedef struct {
    unsigned short Analog_value[8];     // 8通道原始ADC模拟值 (0~4095)
    unsigned short Normal_value[8];     // 8通道归一化值 (0~4095)
    unsigned short Calibrated_white[8]; // 8通道白校准基准ADC值
    unsigned short Calibrated_black[8]; // 8通道黑校准基准ADC值
    unsigned short Gray_white[8];       // 8通道白灰度阈值 (2/3白+1/3黑)
    unsigned short Gray_black[8];       // 8通道黑灰度阈值 (1/3白+2/3黑)
    double Normal_factor[8];            // 8通道归一化系数 =量程/(白-黑)
    double bits;                        // ADC量程 (12bit=4096.0)
    unsigned char Digtal;               // 8位数字量输出 (每bit对应1通道)
    unsigned char Time_out;             // 采样超时/时基阈值
    unsigned char Tick;                 // 时基计数 (带定时器模式使用)
    unsigned char ok;                   // 校准完成标志 (1=已校准)
} GrayscaleSensor_t;

// 函数声明

// 硬件初始化
// 灰度传感器硬件初始化
// PC0=ADC1_IN10(模拟输入), PB12-14=地址选择(推挽输出)
// ADC: 12位分辨率, 独立模式, PCLK2/4=21MHz, 软件触发
void Grayscale_Init(void);

// 读取ADC1通道10的单次转换值 (12位)
// 返回: ADC值 (0-4095), 超时返回0
unsigned short Grayscale_ADC_Read(void);

// 传感器校准
// 首次初始化传感器结构体 (不含校准数据, ok=0)
// s: 传感器结构体指针
// 清零所有校准数据和状态, 归一化系数置0
// 调用后传感器处于未校准状态, 需先采集黑白值再调用Grayscale_InitCalibrate
void Grayscale_InitFirst(GrayscaleSensor_t *s);

// 带校准参数的完整初始化 (与官方No_MCU_Ganv_Sensor_Init对应)
// s: 传感器结构体指针
// white: 8通道白色基准ADC值 (传感器对准白色表面时的读数)
// black: 8通道黑色基准ADC值 (传感器对准黑色表面时的读数)
// 每个通道的white/black可以不同, 自动计算:
// - Gray_white = (white*2 + black)/3  (偏白阈值)
// - Gray_black = (white + black*2)/3  (偏黑阈值)
// - Normal_factor = 4096 / (white - black)
// 校准完成后 ok=1
void Grayscale_InitCalibrate(GrayscaleSensor_t *s,
                             unsigned short *white, unsigned short *black);

// 传感器任务 (主循环中调用)
// 执行一次完整的传感器数据采集与处理 (与官方No_Mcu_Ganv_Sensor_Task对应)
// s: 传感器结构体指针
// 处理流程: 8通道ADC采集(8次均值) -&gt; 二值化 -&gt; 归一化
// 无时基模式, 每次调用立即执行完整采集
void Grayscale_Task(GrayscaleSensor_t *s);

// 用户数据获取接口
// 获取二值化数字量 (与官方Get_Digtal_For_User对应)
// s: 传感器结构体指针
// 返回: 8位数字量 bit7~bit0对应通道7~0:
//       1=白线(ADC值&gt;白阈值), 0=黑线(ADC值&lt;黑阈值), 灰色区间保持原值
unsigned char Grayscale_GetDigital(GrayscaleSensor_t *s);

// 获取归一化模拟值 (0-4095, 与官方Get_Normalize_For_User对应)
// s: 传感器结构体指针
// out: 输出缓冲区 (16字节, 8个unsigned short)
// 返回: 1=成功(已校准), 0=失败(未校准)
unsigned char Grayscale_GetNormalized(GrayscaleSensor_t *s, unsigned short *out);

// 获取原始ADC模拟值 (重新采样, 与官方Get_Anolog_Value对应)
// s: 传感器结构体指针
// out: 输出缓冲区 (16字节, 8个unsigned short)
// 返回: 1=成功(已校准), 0=失败(未校准)
// 每次调用重新执行8通道ADC采集+均值滤波
unsigned char Grayscale_GetAnalog(GrayscaleSensor_t *s, unsigned short *out);

#endif


