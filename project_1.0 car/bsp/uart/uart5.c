// UART5驱动: PC12(TX, GPIOC) + PD2(RX, GPIOD), 115200-8N1, 无流控, 无中断

#include "uart5.h"

// 初始化UART5: PC12=TX, PD2=RX, 复用推挽上拉, 100MHz
void UART5_Init(uint32_t baud)
{
    GPIO_InitTypeDef g; USART_InitTypeDef u;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2,  GPIO_AF_UART5);

    g.GPIO_Pin   = GPIO_Pin_12;
    g.GPIO_Mode  = GPIO_Mode_AF;
    g.GPIO_Speed = GPIO_Speed_100MHz;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &g);

    g.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOD, &g);

    USART_DeInit(UART5);
    USART_StructInit(&u);

    u.USART_BaudRate            = baud;
    u.USART_WordLength          = USART_WordLength_8b;
    u.USART_StopBits            = USART_StopBits_1;
    u.USART_Parity              = USART_Parity_No;
    u.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    u.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    USART_Init(UART5, &u);
    USART_Cmd(UART5, ENABLE);
}