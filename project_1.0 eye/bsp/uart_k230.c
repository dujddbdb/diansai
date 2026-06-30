/**
 * @file    uart_k230.c
 * @brief   Raw K230 error receiver.
 */
#include "uart_k230.h"

uint8_t k230_rx_buf[K230_RX_BUF_SIZE];
uint8_t k230_rx_count = 0U;
bool    k230_rx_flag = false;
K230_ParsedData k230_parsed = {0};

// 初始化K230通信串口: PB6=TX, PB7=RX, 中断接收
void K230_UART_Init(uint32_t baud)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &gpio);

    USART_DeInit(USART1);
    USART_StructInit(&usart);
    usart.USART_BaudRate = baud;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &usart);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
    USART_Cmd(USART1, ENABLE);

    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = K230_IRQ_PRIO;
    nvic.NVIC_IRQChannelSubPriority = 0U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

// 大端序2字节转有符号16位整数
static int16_t K230_ReadI16BE(uint8_t hi, uint8_t lo)
{
    return (int16_t)(((uint16_t)hi << 8) | (uint16_t)lo);
}

// 解析接收缓冲区: 4字节→err_y/err_z, 校验有效标记
void K230_ParsePacket(void)
{
    k230_parsed.error_code = 0U;
    k230_parsed.track_valid = false;

    if (k230_rx_count == K230_TRACK_RAW_LEN) {
        k230_parsed.err_y = K230_ReadI16BE(k230_rx_buf[0], k230_rx_buf[1]);
        k230_parsed.err_z = K230_ReadI16BE(k230_rx_buf[2], k230_rx_buf[3]);
        k230_parsed.track_valid =
            !((k230_parsed.err_y == (int16_t)K230_TRACK_INVALID) &&
              (k230_parsed.err_z == (int16_t)K230_TRACK_INVALID));
    } else {
        k230_parsed.err_y = 0;
        k230_parsed.err_z = 0;
        k230_parsed.error_code = 2U;
    }

    k230_rx_flag = false;
    k230_rx_count = 0U;
}

// USART1中断服务: RXNE逐字节接收, IDLE帧结束标记
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t data = (uint8_t)USART1->DR;

        if (k230_rx_count < K230_RX_BUF_SIZE) {
            k230_rx_buf[k230_rx_count++] = data;
        }
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }

    if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET) {
        (void)USART1->SR;
        (void)USART1->DR;
        k230_rx_flag = true;
    }
}
