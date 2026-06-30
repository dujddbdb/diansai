/**
 * @file    grayscale.c
 * @brief   感为8通道灰度传感器驱动实现 (基于官方No_Mcu_Ganv_Grayscale_Sensor重构)
 * @note    硬件: PC0=ADC1_IN10(模拟输入), PB12=AD2, PB13=AD1, PB14=AD0(地址选择)
 *          ADC采样: 12位分辨率, 右对齐, 采样时间480周期, 软件触发
 *          地址编码: 3位二进制(000~111)选通8个传感器通道, 低电平有效(取反输出)
 *          滤波器: 每通道8次ADC均值
 */

#include "grayscale.h"
#include "board.h"

/* ---- 内部辅助函数声明 ---- */
static void Gray_ReadAllCh(unsigned short *result);         // 采集8通道ADC模拟值 (均值滤波)
static void Gray_AnalogToDigital(unsigned short *adc_val,   // 模拟值转数字量/二值化
                                  unsigned short *white_th,
                                  unsigned short *black_th,
                                  unsigned char  *digital);
static void Gray_Normalize(unsigned short *adc_val,         // 归一化ADC值到量程范围
                            double *factor,
                            unsigned short *black_ref,
                            unsigned short *result, double max_val);

/* ===== GPIO和ADC初始化 ===== */

void Grayscale_Init(void)
{
    GPIO_InitTypeDef      gpio;
    ADC_InitTypeDef       adc;
    ADC_CommonInitTypeDef adc_common;

    // ADC1通道10: PC0模拟输入
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    gpio.GPIO_Pin   = GPIO_Pin_0;
    gpio.GPIO_Mode  = GPIO_Mode_AN;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &gpio);

    // ADC通用配置: 独立模式, 4分频(ADCCLK=84/4=21MHz)
    ADC_DeInit();
    adc_common.ADC_DMAAccessMode   = ADC_DMAAccessMode_Disabled;
    adc_common.ADC_Mode            = ADC_Mode_Independent;
    adc_common.ADC_Prescaler       = ADC_Prescaler_Div4;
    adc_common.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&adc_common);

    // ADC1: 12位, 右对齐, 单次转换, 软件触发
    adc.ADC_ContinuousConvMode    = DISABLE;
    adc.ADC_DataAlign             = ADC_DataAlign_Right;
    adc.ADC_ExternalTrigConvEdge  = ADC_ExternalTrigConvEdge_None;
    adc.ADC_NbrOfConversion       = 1;
    adc.ADC_Resolution            = ADC_Resolution_12b;
    adc.ADC_ScanConvMode          = DISABLE;
    ADC_Init(ADC1, &adc);
    ADC_Cmd(ADC1, ENABLE);

    // 地址选择GPIO: PB12(AD2), PB13(AD1), PB14(AD0)
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    gpio.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
    gpio.GPIO_Mode  = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &gpio);
    GPIO_WriteBit(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14, Bit_RESET);
}

unsigned short Grayscale_ADC_Read(void)
{
    uint32_t timeout = 100000;
    ADC1->CR2 &= ~ADC_CR2_ALIGN;
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_480Cycles);
    ADC_SoftwareStartConv(ADC1);
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)) {
        if (--timeout == 0) return 0;
    }
    return ADC_GetConversionValue(ADC1);
}

/* ===== 8通道模拟值采集 (均值滤波) ===== */

static void Gray_ReadAllCh(unsigned short *result)
{
    unsigned char ch, sample;
    unsigned int  sum;

    for (ch = 0; ch < 8; ch++) {
        // 地址线低电平有效→输出取反, 选通对应传感器通道
        GRAY_ADDR0(!(ch & 0x01));
        GRAY_ADDR1(!(ch & 0x02));
        GRAY_ADDR2(!(ch & 0x04));
        delay_us(50);

        sum = 0;
        for (sample = 0; sample < 8; sample++) {
            sum += Grayscale_ADC_Read();
        }

#if GRAY_DIRECTION == 0
        result[ch]     = sum / 8;
#else
        result[7 - ch] = sum / 8;
#endif
    }
}

/* ===== 二值化与归一化 ===== */

static void Gray_AnalogToDigital(unsigned short *adc_val,
                                  unsigned short *white_th,
                                  unsigned short *black_th,
                                  unsigned char  *digital)
{
    int i;
    for (i = 0; i < 8; i++) {
        if (adc_val[i] > white_th[i]) {
            *digital |= (1 << i);          // 白线: bit置1
        } else if (adc_val[i] < black_th[i]) {
            *digital &= ~(1 << i);         // 黑线: bit清0
        }
        // 灰色区间保持上一次状态 (滞回防抖)
    }
}

static void Gray_Normalize(unsigned short *adc_val, double *factor,
                            unsigned short *black_ref,
                            unsigned short *result, double max_val)
{
    int i;
    unsigned short n;
    for (i = 0; i < 8; i++) {
        if (adc_val[i] < black_ref[i]) {
            n = 0;
        } else {
            n = (unsigned short)((adc_val[i] - black_ref[i]) * factor[i]);
        }
        if (n > (unsigned short)max_val) {
            n = (unsigned short)max_val;
        }
        result[i] = n;
    }
}

/* ===== 传感器校准 ===== */

void Grayscale_InitFirst(GrayscaleSensor_t *s)
{
    int i;

    memset(s->Calibrated_black, 0, sizeof(s->Calibrated_black));
    memset(s->Calibrated_white, 0, sizeof(s->Calibrated_white));
    memset(s->Normal_value,     0, sizeof(s->Normal_value));
    memset(s->Analog_value,     0, sizeof(s->Analog_value));

    for (i = 0; i < 8; i++) {
        s->Normal_factor[i] = 0.0;
    }

    s->Digtal   = 0;
    s->Time_out = 0;
    s->Tick     = 0;
    s->ok       = 0;
}

void Grayscale_InitCalibrate(GrayscaleSensor_t *s,
                              unsigned short *white, unsigned short *black)
{
    int i;

    Grayscale_InitFirst(s);

#if GRAY_ADC_BITS == ADC_8Bits
    s->bits = 255.0;
#elif GRAY_ADC_BITS == ADC_10Bits
    s->bits = 1024.0;
#elif GRAY_ADC_BITS == ADC_12Bits
    s->bits = 4096.0;
#elif GRAY_ADC_BITS == ADC_14Bits
    s->bits = 16384.0;
#else
    s->bits = 4096.0;
#endif

    s->Time_out = 1;

    for (i = 0; i < 8; i++) {
        unsigned short temp;

        if (black[i] >= white[i]) {
            temp    = white[i];
            white[i] = black[i];
            black[i] = temp;
        }

        // 偏白阈值(2:1)和偏黑阈值(1:2)分界
        s->Gray_white[i] = (white[i] * 2 + black[i]) / 3;
        s->Gray_black[i] = (white[i] + black[i] * 2) / 3;

        s->Calibrated_black[i] = black[i];
        s->Calibrated_white[i] = white[i];

        if ((white[i] == 0 && black[i] == 0) || (white[i] == black[i])) {
            s->Normal_factor[i] = 0.0;
            continue;
        }

        s->Normal_factor[i] = s->bits / (double)(white[i] - black[i]);
    }

    s->ok = 1;
}

/* ===== 传感器主任务 ===== */

void Grayscale_Task(GrayscaleSensor_t *s)
{
    Gray_ReadAllCh(s->Analog_value);
    Gray_AnalogToDigital(s->Analog_value, s->Gray_white,
                         s->Gray_black, &s->Digtal);
    Gray_Normalize(s->Analog_value, s->Normal_factor,
                   s->Calibrated_black, s->Normal_value, s->bits);
}

/* ===== 用户数据获取接口 ===== */

unsigned char Grayscale_GetDigital(GrayscaleSensor_t *s)
{
    return s->Digtal;
}

unsigned char Grayscale_GetNormalized(GrayscaleSensor_t *s, unsigned short *out)
{
    if (!s->ok) return 0;
    memcpy(out, s->Normal_value, 16);
    return 1;
}

unsigned char Grayscale_GetAnalog(GrayscaleSensor_t *s, unsigned short *out)
{
    Gray_ReadAllCh(s->Analog_value);
    memcpy(out, s->Analog_value, 16);
    if (!s->ok) return 0;
    return 1;
}