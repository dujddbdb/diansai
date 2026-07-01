// @file    grayscale.c
// @brief   感为8通道灰度传感器驱动实现 (基于官方No_Mcu_Ganv_Grayscale_Sensor重构)
// @note    硬件: PC0=ADC1_IN10(模拟输入), PB12=AD2, PB13=AD1, PB14=AD0(地址选择)
//          ADC采样: 12位分辨率, 右对齐, 采样时间480周期, 软件触发
//          地址编码: 3位二进制(000~111)选通8个传感器通道, 低电平有效(取反输出)
//          滤波器: 每通道8次ADC均值

#include "grayscale.h"
#include "board.h"

// ---- 内部辅助函数声明 ----
static void Gray_ReadAllCh(unsigned short *result);
static void Gray_AnalogToDigital(unsigned short *adc_val,
                                  unsigned short *white_th,
                                  unsigned short *black_th,
                                  unsigned char  *digital);
static void Gray_Normalize(unsigned short *adc_val,
                            double *factor,
                            unsigned short *black_ref,
                            unsigned short *result, double max_val);

// ===== GPIO和ADC初始化 =====
// 初始化灰度传感器: ADC1通道10(PC0) + 地址选择GPIO(PB12/PB13/PB14)
void Grayscale_Init(void)
{
    GPIO_InitTypeDef      gpio;
    ADC_InitTypeDef       adc;
    ADC_CommonInitTypeDef adc_common;

    // 使能GPIOC和ADC1时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    // 配置PC0为模拟输入模式 (ADC1_IN10)
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

    // ADC1配置: 12位分辨率, 右对齐, 单次转换, 软件触发
    adc.ADC_ContinuousConvMode    = DISABLE;
    adc.ADC_DataAlign             = ADC_DataAlign_Right;
    adc.ADC_ExternalTrigConvEdge  = ADC_ExternalTrigConvEdge_None;
    adc.ADC_NbrOfConversion       = 1;
    adc.ADC_Resolution            = ADC_Resolution_12b;
    adc.ADC_ScanConvMode          = DISABLE;
    ADC_Init(ADC1, &adc);
    ADC_Cmd(ADC1, ENABLE);

    // 地址选择GPIO初始化: PB12(AD2), PB13(AD1), PB14(AD0)
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    // 配置PB12/PB13/PB14为推挽输出
    gpio.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
    gpio.GPIO_Mode  = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &gpio);
    // 初始输出低电平, 选通通道0
    GPIO_WriteBit(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14, Bit_RESET);
}

// 读取ADC1通道10的单次转换值, 带超时保护
unsigned short Grayscale_ADC_Read(void)
{
    uint32_t timeout = 100000;
    // 清除对齐标志, 确保右对齐
    ADC1->CR2 &= ~ADC_CR2_ALIGN;
    // 配置规则通道10, 采样时间480周期
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_480Cycles);
    // 软件触发转换
    ADC_SoftwareStartConv(ADC1);
    // 等待转换完成, 超时返回0
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)) {
        if (--timeout == 0) return 0;
    }
    // 返回转换结果
    return ADC_GetConversionValue(ADC1);
}

// ===== 8通道模拟值采集 (均值滤波) =====
// 功能: 依次选通8个通道, 每通道采样8次取平均值
// 参数: result - 输出8通道ADC值数组
static void Gray_ReadAllCh(unsigned short *result)
{
    unsigned char ch, sample;
    unsigned int  sum;

    // 遍历8个通道
    for (ch = 0; ch < 8; ch++) {
        // 地址线低电平有效→输出取反, 选通对应传感器通道
        GRAY_ADDR0(!(ch & 0x01));
        GRAY_ADDR1(!(ch & 0x02));
        GRAY_ADDR2(!(ch & 0x04));
        // 等待通道切换稳定
        delay_us(50);

        // 每通道采样8次求和
        sum = 0;
        for (sample = 0; sample < 8; sample++) {
            sum += Grayscale_ADC_Read();
        }

        // 根据方向配置决定通道顺序
#if GRAY_DIRECTION == 0
        result[ch]     = sum / 8;
#else
        result[7 - ch] = sum / 8;
#endif
    }
}

// ===== 二值化与归一化 =====
// 功能: 模拟值转数字量(二值化), 带滞回防抖
// 参数: adc_val - 8通道ADC值, white_th - 偏白阈值, black_th - 偏黑阈值, digital - 输出二值化结果
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

// 功能: 归一化ADC值到量程范围 [0, max_val]
// 参数: adc_val - 原始ADC值, factor - 归一化系数, black_ref - 黑色参考值, result - 输出归一化结果, max_val - 最大值上限
static void Gray_Normalize(unsigned short *adc_val, double *factor,
                            unsigned short *black_ref,
                            unsigned short *result, double max_val)
{
    int i;
    unsigned short n;
    for (i = 0; i < 8; i++) {
        // 低于黑色参考值则为0
        if (adc_val[i] < black_ref[i]) {
            n = 0;
        } else {
            // 按比例放大到量程范围
            n = (unsigned short)((adc_val[i] - black_ref[i]) * factor[i]);
        }
        // 钳位到最大值
        if (n > (unsigned short)max_val) {
            n = (unsigned short)max_val;
        }
        result[i] = n;
    }
}

// ===== 传感器校准 =====
// 功能: 初始化传感器结构体, 清零所有数据
void Grayscale_InitFirst(GrayscaleSensor_t *s)
{
    int i;

    // 清零校准数据和结果数组
    memset(s->Calibrated_black, 0, sizeof(s->Calibrated_black));
    memset(s->Calibrated_white, 0, sizeof(s->Calibrated_white));
    memset(s->Normal_value,     0, sizeof(s->Normal_value));
    memset(s->Analog_value,     0, sizeof(s->Analog_value));

    // 清零归一化系数
    for (i = 0; i < 8; i++) {
        s->Normal_factor[i] = 0.0;
    }

    // 清零状态变量
    s->Digtal   = 0;
    s->Time_out = 0;
    s->Tick     = 0;
    s->ok       = 0;
}

// 功能: 使用黑白参考值进行校准, 计算阈值和归一化系数
// 参数: s - 传感器结构体, white - 白色参考值, black - 黑色参考值
void Grayscale_InitCalibrate(GrayscaleSensor_t *s,
                              unsigned short *white, unsigned short *black)
{
    int i;

    // 先初始化结构体
    Grayscale_InitFirst(s);

    // 根据ADC位数设置满量程值
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

    // 逐通道计算阈值和系数
    for (i = 0; i < 8; i++) {
        unsigned short temp;

        // 确保white[i] > black[i], 否则交换
        if (black[i] >= white[i]) {
            temp    = white[i];
            white[i] = black[i];
            black[i] = temp;
        }

        // 偏白阈值(2:1)和偏黑阈值(1:2)分界, 形成滞回区间
        s->Gray_white[i] = (white[i] * 2 + black[i]) / 3;
        s->Gray_black[i] = (white[i] + black[i] * 2) / 3;

        // 保存校准参考值
        s->Calibrated_black[i] = black[i];
        s->Calibrated_white[i] = white[i];

        // 黑白值相同或为0时跳过, 系数保持0
        if ((white[i] == 0 && black[i] == 0) || (white[i] == black[i])) {
            s->Normal_factor[i] = 0.0;
            continue;
        }

        // 计算归一化系数 = 满量程 / (白值 - 黑值)
        s->Normal_factor[i] = s->bits / (double)(white[i] - black[i]);
    }

    // 校准完成标记
    s->ok = 1;
}

// ===== 传感器主任务 =====
// 功能: 采集→二值化→归一化, 应周期性调用
void Grayscale_Task(GrayscaleSensor_t *s)
{
    // 采集8通道模拟值
    Gray_ReadAllCh(s->Analog_value);
    // 二值化处理
    Gray_AnalogToDigital(s->Analog_value, s->Gray_white,
                         s->Gray_black, &s->Digtal);
    // 归一化处理
    Gray_Normalize(s->Analog_value, s->Normal_factor,
                   s->Calibrated_black, s->Normal_value, s->bits);
}

// ===== 用户数据获取接口 =====

// 获取二值化结果
unsigned char Grayscale_GetDigital(GrayscaleSensor_t *s)
{
    return s->Digtal;
}

// 获取归一化值, 成功返回1, 未校准返回0
unsigned char Grayscale_GetNormalized(GrayscaleSensor_t *s, unsigned short *out)
{
    if (!s->ok) return 0;
    memcpy(out, s->Normal_value, 16);
    return 1;
}

// 获取原始模拟值, 成功返回1, 未校准返回0
unsigned char Grayscale_GetAnalog(GrayscaleSensor_t *s, unsigned short *out)
{
    Gray_ReadAllCh(s->Analog_value);
    memcpy(out, s->Analog_value, 16);
    if (!s->ok) return 0;
    return 1;
}
