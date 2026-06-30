#ifndef __GRAYSCALE_H__
#define __GRAYSCALE_H__

#include "stm32f4xx.h"
#include <string.h>
#include "track_config.h"

// ADC分辨率选项
#define ADC_8Bits  3    // 8位ADC分辨率
#define ADC_10Bits 2    // 10位ADC分辨率
#define ADC_12Bits 1    // 12位ADC分辨率
#define ADC_14Bits 0    // 14位ADC分辨率

#define GRAY_ADC_BITS ADC_12Bits // ADC分辨率选择(底层硬件配置)

// 默认校准阈值(可被上层InitCalibrate覆盖)
#define GRAY_W0  2646    // CH8(左)白阈值
#define GRAY_W1  3088    // CH7白阈值
#define GRAY_W2  3105    // CH6白阈值
#define GRAY_W3  3162    // CH5白阈值
#define GRAY_W4  2807    // CH4白阈值
#define GRAY_W5  2484    // CH3白阈值
#define GRAY_W6  3101    // CH2白阈值
#define GRAY_W7  2786    // CH1(右)白阈值

#define GRAY_B0   780    // CH8黑阈值
#define GRAY_B1  1965    // CH7黑阈值
#define GRAY_B2  1486    // CH6黑阈值
#define GRAY_B3  1240    // CH5黑阈值
#define GRAY_B4   976    // CH4黑阈值
#define GRAY_B5   695    // CH3黑阈值
#define GRAY_B6  1384    // CH2黑阈值
#define GRAY_B7  1235    // CH1黑阈值

// 地址线引脚定义(底层硬件)
#define GRAY_ADDR0(n) GPIO_WriteBit(GPIOB, GPIO_Pin_14, (n) ? Bit_SET : Bit_RESET) // AD0地址线(PB14)
#define GRAY_ADDR1(n) GPIO_WriteBit(GPIOB, GPIO_Pin_13, (n) ? Bit_SET : Bit_RESET) // AD1地址线(PB13)
#define GRAY_ADDR2(n) GPIO_WriteBit(GPIOB, GPIO_Pin_12, (n) ? Bit_SET : Bit_RESET) // AD2地址线(PB12)

// 8通道灰度传感器数据结构体
typedef struct {
    unsigned short Analog_raw[8];        // 原始ADC值(8次均值)
    unsigned short Analog_value[8];      // EMA滤波后ADC值
    unsigned short Normal_value[8];      // 归一化值(0~量程)
    unsigned short Calibrated_white[8];  // 白校准基准值
    unsigned short Calibrated_black[8];  // 黑校准基准值
    unsigned short Gray_white[8];        // 二值化白阈值
    unsigned short Gray_black[8];        // 二值化黑阈值
    double Normal_factor[8];             // 归一化系数
    double bits;                         // ADC量程值
    unsigned char Digtal;                // 8位数字量输出(bit=1为白,0为黑)
    unsigned char Time_out;              // 采样超时阈值
    unsigned char Tick;                  // 时基计数
    unsigned char ok;                    // 0-未校准 1-校准完成
    unsigned char analog_ema_init;       // EMA初始化标志
} GrayscaleSensor_t;

// 硬件初始化(ADC+GPIO)
void Grayscale_Init(void);
// 读取单次ADC值，返回转换结果(0~4095)
unsigned short Grayscale_ADC_Read(void);
// 初始化传感器结构体(清零状态)
void Grayscale_InitFirst(GrayscaleSensor_t *s);
// 使用实测黑白基准值完成校准
void Grayscale_InitCalibrate(GrayscaleSensor_t *s,
                             unsigned short *white, unsigned short *black);
// 传感器主任务:8通道采集→EMA滤波→二值化→归一化
void Grayscale_Task(GrayscaleSensor_t *s);
// 获取二值化数字量(8bit, bit=1白/0黑)
unsigned char Grayscale_GetDigital(GrayscaleSensor_t *s);
// 获取归一化模拟值(0~4095)，返回1-成功 0-未校准
unsigned char Grayscale_GetNormalized(GrayscaleSensor_t *s, unsigned short *out);
// 获取原始ADC值(重新采样)，返回1-成功 0-未校准
unsigned char Grayscale_GetAnalog(GrayscaleSensor_t *s, unsigned short *out);

#endif
