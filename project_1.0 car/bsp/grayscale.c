// 8通道灰度传感器驱动: PC0=ADC1_IN10, PB12/13/14=地址选择, ADC12位+EMA滤波

#include "grayscale.h"
#include "board.h"
#include "track.h"

// 读取全部8通道ADC模拟值
static void Gray_ReadAllCh(unsigned short *result);
// ADC模拟值转换为二值化数字量
static void Gray_AnalogToDigital(unsigned short *adc_val,
                                  unsigned short *white_th,
                                  unsigned short *black_th,
                                  unsigned char  *digital);
// ADC值归一化到指定量程范围
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

    // 使能GPIOC端口时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    // 使能ADC1时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    // 配置PC0为ADC模拟输入引脚
    gpio.GPIO_Pin   = GPIO_Pin_0;          // 选择引脚0
    gpio.GPIO_Mode  = GPIO_Mode_AN;        // 模拟输入模式
    gpio.GPIO_Speed = GPIO_Speed_100MHz;   // 速度100MHz
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;    // 无上拉下拉
    GPIO_Init(GPIOC, &gpio);               // 初始化GPIOC

    // ADC通用配置: 独立模式, 4分频(ADCCLK=21MHz)
    ADC_DeInit();                          // 复位ADC配置
    adc_common.ADC_DMAAccessMode   = ADC_DMAAccessMode_Disabled;  // 禁用DMA
    adc_common.ADC_Mode            = ADC_Mode_Independent;        // 独立模式
    adc_common.ADC_Prescaler       = ADC_Prescaler_Div4;          // 4分频
    adc_common.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;  // 两次采样间隔5周期
    ADC_CommonInit(&adc_common);           // 初始化ADC通用配置

    // ADC1: 12位, 右对齐, 单次转换, 软件触发
    adc.ADC_ContinuousConvMode    = DISABLE;       // 单次转换模式
    adc.ADC_DataAlign             = ADC_DataAlign_Right;  // 数据右对齐
    adc.ADC_ExternalTrigConvEdge  = ADC_ExternalTrigConvEdge_None;  // 无外部触发
    adc.ADC_NbrOfConversion       = 1;             // 转换通道数1
    adc.ADC_Resolution            = ADC_Resolution_12b;  // 12位分辨率
    adc.ADC_ScanConvMode          = DISABLE;       // 非扫描模式
    ADC_Init(ADC1, &adc);                          // 初始化ADC1
    ADC_Cmd(ADC1, ENABLE);                         // 使能ADC1

    // PB12/13/14: 3位地址选择线(推挽输出)
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  // 使能GPIOB时钟
    gpio.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;  // 选择12/13/14引脚
    gpio.GPIO_Mode  = GPIO_Mode_OUT;                  // 输出模式
    gpio.GPIO_OType = GPIO_OType_PP;                   // 推挽输出
    gpio.GPIO_Speed = GPIO_Speed_100MHz;               // 速度100MHz
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;                // 无上拉下拉
    GPIO_Init(GPIOB, &gpio);                           // 初始化GPIOB
    GPIO_WriteBit(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14, Bit_RESET);  // 地址线初始拉低
}

// 读取单次ADC值(0~4095)，超时返回0
unsigned short Grayscale_ADC_Read(void)
{
    uint32_t timeout = 100000;
    // 清除对齐标志
    ADC1->CR2 &= ~ADC_CR2_ALIGN;
    // 配置ADC规则通道: 通道10, 采样时间480周期
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_480Cycles);
    // 软件启动转换
    ADC_SoftwareStartConv(ADC1);
    // 等待转换完成, 超时则返回0
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)) {
        if (--timeout == 0) return 0;
    }
    // 返回转换结果
    return ADC_GetConversionValue(ADC1);
}

// 采集8通道ADC模拟值(每通道8次均值)
static void Gray_ReadAllCh(unsigned short *result)
{
    unsigned char ch, sample;
    unsigned int  sum;

    // 循环扫描8个通道
    for (ch = 0; ch < 8; ch++) {
        // 设置地址线选通通道(低电平有效)
        GRAY_ADDR0(!(ch & 0x01));
        GRAY_ADDR1(!(ch & 0x02));
        GRAY_ADDR2(!(ch & 0x04));
        // 等待传感器稳定
        delay_us(50);

        // 每通道采样8次取平均
        sum = 0;
        for (sample = 0; sample < 8; sample++) {
            sum += Grayscale_ADC_Read();
        }

        // 根据方向配置存储到对应位置
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
    // 遍历8个通道进行二值化判断
    for (i = 0; i < 8; i++) {
        // ADC值大于等于白色阈值则置1(白色), 否则置0(黑色)
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
    // 遍历8个通道进行归一化
    for (i = 0; i < 8; i++) {
        // 低于黑色基准则归一化为0
        if (adc_val[i] < black_ref[i]) {
            n = 0;
        } else {
            // (ADC值 - 黑色基准) × 归一化系数
            n = (unsigned short)((adc_val[i] - black_ref[i]) * factor[i]);
        }
        // 上限截断, 不超过最大值
        if (n > (unsigned short)max_val) n = (unsigned short)max_val;
        result[i] = n;
    }
}

// 传感器结构体清零(无校准数据)
void Grayscale_InitFirst(GrayscaleSensor_t *s)
{
    int i;
    // 清零黑色校准值数组
    memset(s->Calibrated_black, 0, sizeof(s->Calibrated_black));
    // 清零白色校准值数组
    memset(s->Calibrated_white, 0, sizeof(s->Calibrated_white));
    // 清零归一化值数组
    memset(s->Normal_value,     0, sizeof(s->Normal_value));
    // 清零原始ADC值数组
    memset(s->Analog_raw,       0, sizeof(s->Analog_raw));
    // 清零滤波后模拟值数组
    memset(s->Analog_value,     0, sizeof(s->Analog_value));
    // 清零归一化系数数组
    for (i = 0; i < 8; i++) s->Normal_factor[i] = 0.0;
    // 清零二值化数字量
    s->Digtal   = 0;
    // 清零超时标志
    s->Time_out = 0;
    // 清零滴答计数
    s->Tick     = 0;
    // 清零校准完成标志
    s->ok       = 0;
    // 清零EMA初始化标志
    s->analog_ema_init = 0;
}

// 使用实测黑白基准值校准传感器
void Grayscale_InitCalibrate(GrayscaleSensor_t *s,
                              unsigned short *white, unsigned short *black)
{
    int i;
    // 先清零结构体
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

    // 设置超时标志
    s->Time_out = 1;

    // 逐个通道计算校准参数
    for (i = 0; i < 8; i++) {
        unsigned short temp;
        // 确保白色值大于黑色值, 否则交换
        if (black[i] >= white[i]) {
            temp    = white[i];
            white[i] = black[i];
            black[i] = temp;
        }

        // 计算二值化阈值: 黑基准 + (白基准-黑基准) * 偏移系数
        {
            float shift = GRAY_THRESHOLD_SHIFT;
            // 限制偏移系数在0~1之间
            if (shift < 0.0f) shift = 0.0f;
            if (shift > 1.0f) shift = 1.0f;
            // 计算阈值并保存到黑白阈值(使用同一阈值)
            unsigned short threshold = (unsigned short)(black[i] + (white[i] - black[i]) * shift);
            s->Gray_white[i] = threshold;
            s->Gray_black[i] = threshold;
        }

        // 保存校准后的黑色基准值
        s->Calibrated_black[i] = black[i];
        // 保存校准后的白色基准值
        s->Calibrated_white[i] = white[i];

        // 黑白值相同或都为0时, 归一化系数设为0
        if ((white[i] == 0 && black[i] == 0) || (white[i] == black[i])) {
            s->Normal_factor[i] = 0.0;
            continue;
        }
        // 计算归一化系数: 满量程 / (白基准 - 黑基准)
        s->Normal_factor[i] = s->bits / (double)(white[i] - black[i]);
    }

    // 标记校准完成
    s->ok = 1;
}

// 传感器主任务: 采集→EMA滤波→二值化→归一化
void Grayscale_Task(GrayscaleSensor_t *s)
{
    int i;

    // 采集8通道原始ADC值
    Gray_ReadAllCh(s->Analog_raw);

    // EMA一阶低通滤波(模拟量)
#if GRAY_ANALOG_EMA_ENABLE
    // 首次初始化: 直接用原始值作为滤波初值
    if (!s->analog_ema_init) {
        for (i = 0; i < 8; i++) s->Analog_value[i] = s->Analog_raw[i];
        s->analog_ema_init = 1;
    } else {
        // EMA滤波: 滤波值 = 历史值×旧权重 + 新值×新权重
        for (i = 0; i < 8; i++) {
            unsigned int filtered;
            filtered = (unsigned int)s->Analog_value[i] * GRAY_ANALOG_EMA_PREV_WEIGHT
                     + (unsigned int)s->Analog_raw[i]   * GRAY_ANALOG_EMA_NEW_WEIGHT;
            s->Analog_value[i] = (unsigned short)(filtered / GRAY_ANALOG_EMA_TOTAL_WEIGHT);
        }
    }
#else
    // 不启用滤波时直接使用原始值
    for (i = 0; i < 8; i++) s->Analog_value[i] = s->Analog_raw[i];
#endif

    // 模拟值转换为二值化数字量
    Gray_AnalogToDigital(s->Analog_value, s->Gray_white,
                         s->Gray_black, &s->Digtal);
    // 模拟值归一化到满量程
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
    // 未校准则返回失败
    if (!s->ok) return 0;
    // 拷贝归一化值到输出缓冲区
    memcpy(out, s->Normal_value, 16);
    return 1;
}

// 获取原始ADC值(重新采样)，返回1-成功 0-未校准
unsigned char Grayscale_GetAnalog(GrayscaleSensor_t *s, unsigned short *out)
{
    // 重新采集所有通道
    Gray_ReadAllCh(s->Analog_raw);
    // 拷贝原始值到输出缓冲区
    memcpy(out, s->Analog_raw, 16);
    // 未校准则返回失败
    if (!s->ok) return 0;
    return 1;
}
