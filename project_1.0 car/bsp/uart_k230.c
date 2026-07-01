// K230通信UART驱动: USART1_TX(PB6), 115200bps, 仅发送模式

#include "uart_k230.h"

// 弯道状态发送帧缓冲区（3字节：帧头+状态+校验）
static uint8_t corner_tx_frame[3];
// 当前发送字节索引
static uint8_t corner_tx_index = 0U;
// 发送活跃标志（1=正在发送，0=空闲）
static uint8_t corner_tx_active = 0U;

// 初始化USART1: PB6→TX, 8N1, 仅发送
void K230_UART_Init(uint32_t baud)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;

    // 使能GPIOB和USART1时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    // 配置PB6复用为USART1
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);

    // 配置PB6(TX)：复用功能、100MHz、推挽、上拉
    gpio.GPIO_Pin   = GPIO_Pin_6;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &gpio);

    // 复位USART1并使用默认配置
    USART_DeInit(USART1);
    USART_StructInit(&usart);
    // 配置波特率
    usart.USART_BaudRate            = baud;
    // 8位数据长度
    usart.USART_WordLength          = USART_WordLength_8b;
    // 1个停止位
    usart.USART_StopBits            = USART_StopBits_1;
    // 无校验
    usart.USART_Parity              = USART_Parity_No;
    // 仅发送模式
    usart.USART_Mode                = USART_Mode_Tx;
    // 无硬件流控
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    // 初始化USART1并使能
    USART_Init(USART1, &usart);
    USART_Cmd(USART1, ENABLE);
}

// 发送弯道状态（立即发送）
void K230_SendCornerState(uint8_t state)
{
    // 将弯道状态加入发送队列
    K230_QueueCornerState(state);
    // 轮询发送
    K230_PollCornerStateTx();
}

// 将弯道状态加入发送队列：帧头+A5+状态+异或校验
void K230_QueueCornerState(uint8_t state)
{
    // 第1字节：帧头
    corner_tx_frame[0] = K230_CORNER_HEADER;
    // 第2字节：弯道状态
    corner_tx_frame[1] = state;
    // 第3字节：异或校验（帧头 ^ 状态）
    corner_tx_frame[2] = (uint8_t)(K230_CORNER_HEADER ^ state);
    // 重置发送索引
    corner_tx_index = 0U;
    // 标记发送活跃
    corner_tx_active = 1U;
}

// 轮询发送队列中的字节，TXE就绪时逐字节发送
void K230_PollCornerStateTx(void)
{
    // 无发送任务则返回
    if (!corner_tx_active) return;
    // 发送数据寄存器未就绪则返回
    if ((USART1->SR & USART_FLAG_TXE) == 0U) return;

    // 发送当前字节，索引递增
    USART1->DR = corner_tx_frame[corner_tx_index++];
    // 全部发送完成，重置状态
    if (corner_tx_index >= sizeof(corner_tx_frame)) {
        corner_tx_index = 0U;
        corner_tx_active = 0U;
    }
}
