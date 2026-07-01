/**
 * @file    uart_k230.c
 * @brief   K230 视觉数据接收 - USART1 中断接收 + 帧解析
 */
#include "uart_k230.h"

uint8_t k230_rx_buf[K230_RX_BUF_SIZE];
uint8_t k230_rx_count = 0U;
bool    k230_rx_flag = false;
K230_ParsedData k230_parsed = {0};

void K230_UART_Init(uint32_t baud)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;

    /* 开启GPIO和USART时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* GPIO复用功能配置: PB6=USART1_TX, PB7=USART1_RX */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

    /* GPIO模式配置: 复用推挽输出, 上拉 */
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &gpio);

    /* USART参数配置: 8N1, 收发模式 */
    USART_DeInit(USART1);
    USART_StructInit(&usart);
    usart.USART_BaudRate = baud;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &usart);

    /* 使能接收中断和空闲中断, 开启USART */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
    USART_Cmd(USART1, ENABLE);

    /* NVIC中断配置: USART1中断优先级 */
    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = K230_IRQ_PRIO;
    nvic.NVIC_IRQChannelSubPriority = 0U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

static int16_t K230_ReadI16BE(uint8_t hi, uint8_t lo)
{
    /* 大端序转换: 高字节左移8位 | 低字节 */
    return (int16_t)(((uint16_t)hi << 8) | (uint16_t)lo);
}

void K230_ParsePacket(void)
{
    /* 初始化解析结果: 默认无效 */
    k230_parsed.error_code = 0U;
    k230_parsed.track_valid = false;

    if (k230_rx_count == K230_TRACK_RAW_LEN) {
        /* 解析Y轴和Z轴误差(大端序) */
        k230_parsed.err_y = K230_ReadI16BE(k230_rx_buf[0], k230_rx_buf[1]);
        k230_parsed.err_z = K230_ReadI16BE(k230_rx_buf[2], k230_rx_buf[3]);
        /* 有效性判断: 两个轴都为0x7FFF表示无目标 */
        k230_parsed.track_valid =
            !((k230_parsed.err_y == (int16_t)K230_TRACK_INVALID) &&
              (k230_parsed.err_z == (int16_t)K230_TRACK_INVALID));
    } else {
        /* 帧长度错误: 清零误差, 设置错误码 */
        k230_parsed.err_y = 0;
        k230_parsed.err_z = 0;
        k230_parsed.error_code = 2U;
    }

    /* 清空接收标志和计数, 准备下一帧 */
    k230_rx_flag = false;
    k230_rx_count = 0U;
}

void USART1_IRQHandler(void)
{
    /* RXNE中断: 逐字节接收数据 */
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t data = (uint8_t)USART1->DR;

        if (k230_rx_count < K230_RX_BUF_SIZE) {
            k230_rx_buf[k230_rx_count++] = data;
        }
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }

    /* IDLE中断: 一帧接收结束, 置位接收标志 */
    if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET) {
        (void)USART1->SR;
        (void)USART1->DR;
        k230_rx_flag = true;
    }
}
