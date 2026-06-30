// USART3驱动: PD8(TX)+PD9(RX), 115200-8N1, 无流控, 无中断

#include "uart3.h"

// 初始化USART3: PD8=TX, PD9=RX, 复用推挽上拉, 100MHz
void USART3_Init(uint32_t baud)
{
    GPIO_InitTypeDef g; USART_InitTypeDef u;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

    g.GPIO_Pin   = GPIO_Pin_8;
    g.GPIO_Mode  = GPIO_Mode_AF;
    g.GPIO_Speed = GPIO_Speed_100MHz;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &g);

    g.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOD, &g);

    USART_DeInit(USART3);
    USART_StructInit(&u);

    u.USART_BaudRate            = baud;
    u.USART_WordLength          = USART_WordLength_8b;
    u.USART_StopBits            = USART_StopBits_1;
    u.USART_Parity              = USART_Parity_No;
    u.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    u.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    USART_Init(USART3, &u);
    USART_Cmd(USART3, ENABLE);
}

// 发送单字节，轮询TXE，超时8000次循环防死锁
void USART3_SendByte(uint8_t data)
{
    uint16_t t = 0;

    USART3->DR = data;
    while (!(USART3->SR & USART_FLAG_TXE)) { if (++t > 8000) return; }
}

// 发送字符串，逐字节发送直到'\0'
void USART3_SendString(const char *str)
{
    while (*str) USART3_SendByte((uint8_t)(*str++));
}