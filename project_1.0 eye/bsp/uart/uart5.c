// @file    uart5.c
// @brief   UART5 driver for car-to-eye corner state frames.
//
// UART5 pins:
//   PC12 = TX
//   PD2  = RX
//
// Protocol: [0xA5][STATE][0xA5 XOR STATE]

#include "uart5.h"
#include "board.h"
#include "car_corner_protocol.h"

// 全局弯道状态变量
volatile CarCornerState_t car_corner_state = {0};

// UART5流解析器
static CarCornerParser_t uart5_stream_parser;

// 检查弯道状态是否新鲜
// now_ms: 当前时间戳, timeout_ms: 超时阈值, 返回1=新鲜 0=超时
uint8_t UART5_CarCornerFresh(uint32_t now_ms, uint32_t timeout_ms)
{
    if (!car_corner_state.fresh) return 0U;
    return ((uint32_t)(now_ms - car_corner_state.last_ms) <= timeout_ms) ? 1U : 0U;
}

// 判断是否处于弯道中
// now_ms: 当前时间戳, 返回1=弯道中 0=直道
uint8_t UART5_CarCornerActive(uint32_t now_ms)
{
    if (!UART5_CarCornerFresh(now_ms, UART5_CAR_STATE_TIMEOUT_MS)) return 0U;
    return (car_corner_state.phase != 0U) ? 1U : 0U;
}

// 获取弯道补偿混合系数
// now_ms: 当前时间戳, 返回0.0~1.0
float UART5_CarCornerBlend(uint32_t now_ms)
{
    float blend;

    // 状态超时则返回0
    if (!UART5_CarCornerFresh(now_ms, UART5_CAR_STATE_TIMEOUT_MS)) {
        return 0.0f;
    }

    // 根据弯道阶段返回混合系数
    switch (car_corner_state.phase) {
    case 1U:
        blend = 0.0f;
        break;
    case 2U:
        blend = 1.0f;
        break;
    case 3U:
        blend = 0.55f;
        break;
    default:
        blend = 0.0f;
        break;
    }

    // 钳位到[0, 1]范围
    if (blend > 1.0f) blend = 1.0f;
    if (blend < 0.0f) blend = 0.0f;
    return blend;
}

// 初始化UART5: PC12=TX, PD2=RX, 中断接收弯道协议帧
void UART5_Init(uint32_t baud)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;

    // 使能GPIOC/GPIOD和UART5时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

    // 配置PC12/PD2复用为UART5
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);

    // PC12(TX): 复用推挽输出, 上拉, 100MHz
    gpio.GPIO_Pin = GPIO_Pin_12;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &gpio);

    // PD2(RX): 复用推挽输出, 上拉, 100MHz
    gpio.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOD, &gpio);

    // UART5配置: 波特率baud, 8位数据, 1停止位, 无校验, 收发模式
    USART_DeInit(UART5);
    USART_StructInit(&usart);
    usart.USART_BaudRate = baud;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(UART5, &usart);

    // 初始化协议解析器
    CarCornerParser_Init(&uart5_stream_parser);
    car_corner_state.fresh = 0U;

    // 使能接收寄存器非空中断
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
    // 使能空闲线路检测中断
    USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);

    // NVIC配置: UART5_IRQn, 抢占优先级3, 子优先级1
    nvic.NVIC_IRQChannel = UART5_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    // 使能UART5
    USART_Cmd(UART5, ENABLE);
}

// UART5中断服务: RXNE逐字节喂入协议解析器, IDLE清空
void UART5_IRQHandler(void)
{
    // 接收寄存器非空中断
    if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET) {
        uint8_t data = (uint8_t)UART5->DR;
        CarCornerPayload_t payload;
        // 喂入协议解析器, 解析成功则更新状态
        if (CarCornerParser_Feed(&uart5_stream_parser, data, &payload)) {
            car_corner_state.phase = payload.phase;
            car_corner_state.type = payload.type;
            car_corner_state.flags = payload.flags;
            car_corner_state.last_ms = HAL_GetTick();
            car_corner_state.fresh = 1U;
            car_corner_state.frames++;
        }
        // 清除中断标志
        USART_ClearITPendingBit(UART5, USART_IT_RXNE);
    }

    // 空闲线路中断: 读SR+DR清除标志
    if (USART_GetITStatus(UART5, USART_IT_IDLE) != RESET) {
        UART5->SR;
        UART5->DR;
    }
}
