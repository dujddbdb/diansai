// @file    uart_k230.c
// @brief   K230通信串口驱动
// @note    USART1: PB6=TX, PB7=RX, 中断接收

#include "uart_k230.h"

// K230接收缓冲区
uint8_t k230_rx_buf[K230_RX_BUF_SIZE];
// 接收字节计数（ISR写, 主循环读, 必须volatile防止编译器缓存导致状态机卡死）
volatile uint8_t k230_rx_count = 0U;
// 接收完成标志 (IDLE中断置位, 必须volatile防止编译器缓存导致状态机卡死)
volatile bool    k230_rx_flag = false;
// 解析后的数据
K230_ParsedData k230_parsed = {0};

// 初始化K230通信串口: PB6=TX, PB7=RX, 中断接收
void K230_UART_Init(uint32_t baud)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;

    // 使能GPIOB和USART1时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    // 配置PB6/PB7复用为USART1
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

    // GPIO配置: 复用功能, 推挽输出, 上拉, 100MHz
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &gpio);

    // USART配置: 波特率baud, 8位数据, 1停止位, 无校验, 收发模式
    USART_DeInit(USART1);
    USART_StructInit(&usart);
    usart.USART_BaudRate = baud;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &usart);

    // 使能接收寄存器非空中断
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    // 使能空闲线路检测中断
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
    // 使能USART1
    USART_Cmd(USART1, ENABLE);

    // NVIC配置: USART1_IRQn, 优先级K230_IRQ_PRIO
    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = K230_IRQ_PRIO;
    nvic.NVIC_IRQChannelSubPriority = 0U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

// 功能: 大端序2字节转有符号16位整数
static int16_t K230_ReadI16BE(uint8_t hi, uint8_t lo)
{
    return (int16_t)(((uint16_t)hi << 8) | (uint16_t)lo);
}

// 解析接收缓冲区: 4字节→err_y/err_z, 校验有效标记
void K230_ParsePacket(void)
{
    // 清零解析结果
    k230_parsed.error_code = 0U;
    k230_parsed.track_valid = false;

    // 接收长度等于跟踪数据长度时解析
    if (k230_rx_count == K230_TRACK_RAW_LEN) {
        // 解析err_y (大端序)
        k230_parsed.err_y = K230_ReadI16BE(k230_rx_buf[0], k230_rx_buf[1]);
        // 解析err_z (大端序)
        k230_parsed.err_z = K230_ReadI16BE(k230_rx_buf[2], k230_rx_buf[3]);
        // 判断跟踪是否有效 (两个误差不都是无效值)
        k230_parsed.track_valid =
            !((k230_parsed.err_y == (int16_t)K230_TRACK_INVALID) &&
              (k230_parsed.err_z == (int16_t)K230_TRACK_INVALID));
    } else {
        // 长度不匹配, 清零误差值
        k230_parsed.err_y = 0;
        k230_parsed.err_z = 0;
        // 设置错误码
        k230_parsed.error_code = 2U;
    }

    // 清零接收标志和计数
    k230_rx_flag = false;
    k230_rx_count = 0U;
}

// USART1中断服务: RXNE逐字节接收, IDLE帧结束标记
void USART1_IRQHandler(void)
{
    // 接收寄存器非空中断
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t data = (uint8_t)USART1->DR;

        // 缓冲区未满则存入数据
        if (k230_rx_count < K230_RX_BUF_SIZE) {
            k230_rx_buf[k230_rx_count++] = data;
        }
        // 清除中断标志
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }

    // 空闲线路中断: 一帧数据接收完成
    if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET) {
        // 读SR和DR清除空闲中断标志
        (void)USART1->SR;
        (void)USART1->DR;
        // 置位接收完成标志
        k230_rx_flag = true;
    }
}
