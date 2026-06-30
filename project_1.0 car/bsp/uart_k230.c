// K230通信UART驱动: USART1_TX(PB6), 115200bps, 仅发送模式

#include "uart_k230.h"

static uint8_t corner_tx_frame[3];
static uint8_t corner_tx_index = 0U;
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

    // GPIO配置: 复用推挽输出，上拉
    gpio.GPIO_Pin   = GPIO_Pin_6;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &gpio);

    // USART配置: 8N1, 仅发送
    USART_DeInit(USART1);
    USART_StructInit(&usart);
    usart.USART_BaudRate            = baud;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_Mode                = USART_Mode_Tx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &usart);

    // 使能USART1
    USART_Cmd(USART1, ENABLE);
}

// 发送弯道状态（立即发送）
void K230_SendCornerState(uint8_t state)
{
    K230_QueueCornerState(state);
    K230_PollCornerStateTx();
}

// 将弯道状态加入发送队列：帧头+状态+异或校验
void K230_QueueCornerState(uint8_t state)
{
    // 组装3字节帧: 帧头 + 状态 + 异或校验
    corner_tx_frame[0] = K230_CORNER_HEADER;
    corner_tx_frame[1] = state;
    corner_tx_frame[2] = (uint8_t)(K230_CORNER_HEADER ^ state);

    // 重置发送索引，标记发送激活
    corner_tx_index = 0U;
    corner_tx_active = 1U;
}

// 轮询发送队列中的字节，TXE就绪时逐字节发送
void K230_PollCornerStateTx(void)
{
    // 无发送任务或发送寄存器未空则返回
    if (!corner_tx_active) return;
    if ((USART1->SR & USART_FLAG_TXE) == 0U) return;

    // 发送当前字节，索引+1
    USART1->DR = corner_tx_frame[corner_tx_index++];

    // 发完一帧：重置索引，标记发送完成
    if (corner_tx_index >= sizeof(corner_tx_frame)) {
        corner_tx_index = 0U;
        corner_tx_active = 0U;
    }
}
