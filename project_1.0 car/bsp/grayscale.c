// 8通道灰度传感器驱动: PC0=ADC1_IN10, PB12/13/14=地址选择, ADC12位+EMA滤波

#include "grayscale.h"
#include "board.h"
#include "track.h"

static void Gray_ReadAllCh(unsigned short *result);
static void Gray_AnalogToDigital(unsigned short *adc_val,
                                  unsigned short *white_th,
                                  unsigned short *black_th,
                                  unsigned char  *digital);
static void Gray_Normalize(unsigned short *adc_val,
                            double *factor,
                            unsigned short *black_ref,
                            unsigned short *result, double max_val);

// 灰度传感器硬件初始化(ADC1 + GPIO地址线)
void Grayscale_Init(void)
{
    GPIO_InitTypeDef      gpio;
    ADC_InitTypeDef       adc;
    ADC_CommonInitTypeDef adc_common;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    // PC0: ADC1_IN10 模拟输入
    gpio.GPIO_Pin   = GPIO_Pin_0;
    gpio.GPIO_Mode  = GPIO_Mode_AN;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &gpio);

    // ADC通用配置: 独立模式, 4分频(ADCCLK=21MHz)
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

    // PB12/13/14: 3位地址选择线(推挽输出)
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    gpio.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
    gpio.GPIO_Mode  = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &gpio);
    GPIO_WriteBit(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14, Bit_RESET);
}

// 读取单次ADC值(0~4095)，超时返回0
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

// 采集8通道ADC模拟值(每通道8次均值)
static void Gray_ReadAllCh(unsigned short *result)
{
    unsigned char ch, sample;
    unsigned int  sum;

    for (ch = 0; ch < 8; ch++) {
        // 设置地址线选通通道(低电平有效)
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

// ADC模拟值→二值化数字量(白=1 黑=0)
static void Gray_AnalogToDigital(unsigned short *adc_val,
                                  unsigned short *white_th,
                                  unsigned short *black_th,
                                  unsigned char  *digital)
{
    int i;
    (void)black_th;
    for (i = 0; i < 8; i++) {
        if (adc_val[i] >= white_th[i]) {
            *digital |= (1 << i);
        } else {
            *digital &= ~(1 << i);
        }
    }
}

// ADC值归一化到量程范围(0~max_val)
static void Gray_Normalize(unsigned short *adc_val,
                            double *factor,
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
        if (n > (unsigned short)max_val) n = (unsigned short)max_val;
        result[i] = n;
    }
}

// 传感器结构体清零(无校准数据)
void Grayscale_InitFirst(GrayscaleSensor_t *s)
{
    int i;
    memset(s->Calibrated_black, 0, sizeof(s->Calibrated_black));
    memset(s->Calibrated_white, 0, sizeof(s->Calibrated_white));
    memset(s->Normal_value,     0, sizeof(s->Normal_value));
    memset(s->Analog_raw,       0, sizeof(s->Analog_raw));
    memset(s->Analog_value,     0, sizeof(s->Analog_value));
    for (i = 0; i < 8; i++) s->Normal_factor[i] = 0.0;
    s->Digtal   = 0;
    s->Time_out = 0;
    s->Tick     = 0;
    s->ok       = 0;
    s->analog_ema_init = 0;
}

// 使用实测黑白基准值校准传感器
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

        // 计算二值化阈值: 黑基准 + (白基准-黑基准) * 偏移系数
        {
            float shift = GRAY_THRESHOLD_SHIFT;
            if (shift < 0.0f) shift = 0.0f;
            if (shift > 1.0f) shift = 1.0f;
            unsigned short threshold = (unsigned short)(black[i] + (white[i] - black[i]) * shift);
            s->Gray_white[i] = threshold;
            s->Gray_black[i] = threshold;
        }

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

// 传感器主任务: 采集→EMA滤波→二值化→归一化
void Grayscale_Task(GrayscaleSensor_t *s)
{
    int i;

    Gray_ReadAllCh(s->Analog_raw);

    // EMA一阶低通滤波(模拟量)
#if GRAY_ANALOG_EMA_ENABLE
    if (!s->analog_ema_init) {
        for (i = 0; i < 8; i++) s->Analog_value[i] = s->Analog_raw[i];
        s->analog_ema_init = 1;
    } else {
        for (i = 0; i < 8; i++) {
            unsigned int filtered;
            filtered = (unsigned int)s->Analog_value[i] * GRAY_ANALOG_EMA_PREV_WEIGHT
                     + (unsigned int)s->Analog_raw[i]   * GRAY_ANALOG_EMA_NEW_WEIGHT;
            s->Analog_value[i] = (unsigned short)(filtered / GRAY_ANALOG_EMA_TOTAL_WEIGHT);
        }
    }
#else
    for (i = 0; i < 8; i++) s->Analog_value[i] = s->Analog_raw[i];
#endif

    Gray_AnalogToDigital(s->Analog_value, s->Gray_white,
                         s->Gray_black, &s->Digtal);
    Gray_Normalize(s->Analog_value, s->Normal_factor,
                   s->Calibrated_black, s->Normal_value, s->bits);
}

// 获取二值化数字量(bit=1白/0黑)
unsigned char Grayscale_GetDigital(GrayscaleSensor_t *s)
{
    return s->Digtal;
}

// 获取归一化模拟值(0~4095)，返回1-成功 0-未校准
unsigned char Grayscale_GetNormalized(GrayscaleSensor_t *s, unsigned short *out)
{
    if (!s->ok) return 0;
    memcpy(out, s->Normal_value, 16);
    return 1;
}

// 获取原始ADC值(重新采样)，返回1-成功 0-未校准
unsigned char Grayscale_GetAnalog(GrayscaleSensor_t *s, unsigned short *out)
{
    Gray_ReadAllCh(s->Analog_raw);
    memcpy(out, s->Analog_raw, 16);
    if (!s->ok) return 0;
    return 1;
}
