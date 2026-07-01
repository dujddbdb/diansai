// UART5驱动: PC12(TX, GPIOC) + PD2(RX, GPIOD), 115200-8N1, 无流控, 无中断

#include "uart5.h"

// 初始化UART5: PC12=TX, PD2=RX, 复用推挽上拉, 100MHz
void UART5_Init(uint32_t baud)
{
    GPIO_InitTypeDef g; USART_InitTypeDef u;

    // 使能GPIOC、GPIOD和UART5时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

    // 配置PC12/PD2复用为UART5
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2,  GPIO_AF_UART5);

    // 配置PC12(TX)：复用功能、100MHz、推挽、上拉
    g.GPIO_Pin   = GPIO_Pin_12;
    g.GPIO_Mode  = GPIO_Mode_AF;
    g.GPIO_Speed = GPIO_Speed_100MHz;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &g);

    // 配置PD2(RX)：复用功能、100MHz、推挽、上拉
    g.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOD, &g);

    // 复位UART5并使用默认配置
    USART_DeInit(UART5);
    USART_StructInit(&u);

    // 配置波特率
    u.USART_BaudRate            = baud;
    // 8位数据长度
    u.USART_WordLength          = USART_WordLength_8b;
    // 1个停止位
    u.USART_StopBits            = USART_StopBits_1;
    // 无校验
    u.USART_Parity              = USART_Parity_No;
    // 收发模式
    u.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    // 无硬件流控
    u.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    // 初始化UART5并使能
    USART_Init(UART5, &u);
    USART_Cmd(UART5, ENABLE);
}
