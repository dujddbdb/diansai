#include "stepper.h"
#include "board.h"

// 串口单字节发送
static void USART_SendByte(USART_TypeDef *USARTx, uint16_t data) {
    uint16_t t = 0; USARTx->DR = (data & 0x01FF);
    while (!(USARTx->SR & USART_FLAG_TXE)) { if (++t > 8000) return; }
}

// 串口多字节命令发送
static void USART_SendCmd(USART_TypeDef *USARTx, uint8_t *cmd, uint8_t len) {
    uint8_t i; for (i=0;i<len;i++) USART_SendByte(USARTx, cmd[i]);
}

// XOY面USART2初始化
void StepperXOY_Init(uint32_t baud)
{
    GPIO_InitTypeDef g; USART_InitTypeDef u; NVIC_InitTypeDef n;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
    g.GPIO_Pin=GPIO_Pin_5;g.GPIO_Mode=GPIO_Mode_AF;g.GPIO_Speed=GPIO_Speed_100MHz;
    g.GPIO_OType=GPIO_OType_PP;g.GPIO_PuPd=GPIO_PuPd_UP;GPIO_Init(GPIOD,&g);
    g.GPIO_Pin=GPIO_Pin_6;GPIO_Init(GPIOD,&g);
    USART_DeInit(USART2);USART_StructInit(&u);
    u.USART_BaudRate=baud;u.USART_WordLength=USART_WordLength_8b;
    u.USART_StopBits=USART_StopBits_1;u.USART_Parity=USART_Parity_No;
    u.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
    u.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
    USART_Init(USART2,&u);USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
    USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);USART_Cmd(USART2,ENABLE);
    n.NVIC_IRQChannel=USART2_IRQn;n.NVIC_IRQChannelPreemptionPriority=1;
    n.NVIC_IRQChannelSubPriority=0;n.NVIC_IRQChannelCmd=ENABLE;NVIC_Init(&n);
}

// XOY轴电机使能控制
void StepperXOY_Enable(uint8_t addr, uint8_t state, uint8_t sync) {
    uint8_t c[6]={addr,0xF3,0xAB,state,sync,0x6B};
    USART_SendCmd(USART2,c,6);
}

// XOY轴速度模式设置
void StepperXOY_Speed(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc, uint8_t sync) {
    uint8_t c[8]={addr,0xF6,dir,(uint8_t)(rpm>>8),(uint8_t)rpm,acc,sync,0x6B};
    USART_SendCmd(USART2,c,8);
}

// XOY轴位置命令发送
void StepperXOY_Position(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc, uint32_t pulses, uint8_t rel, uint8_t sync) {
    uint8_t c[13]={addr,0xFD,dir,(uint8_t)(rpm>>8),(uint8_t)rpm,acc,
        (uint8_t)(pulses>>24),(uint8_t)(pulses>>16),(uint8_t)(pulses>>8),(uint8_t)pulses,rel,sync,0x6B};
    USART_SendCmd(USART2,c,13);
}

// XOY轴电机停止
void StepperXOY_Stop(uint8_t addr, uint8_t sync) {
    uint8_t c[5]={addr,0xFE,0x98,sync,0x6B};
    USART_SendCmd(USART2,c,5);
}

// XOY轴同步启动
void StepperXOY_SyncStart(uint8_t addr) {
    uint8_t c[4]={addr,0xFF,0x66,0x6B};
    USART_SendCmd(USART2,c,4);
}

// YOZ面USART6初始化
void StepperYOZ_Init(uint32_t baud)
{
    GPIO_InitTypeDef g; USART_InitTypeDef u; NVIC_InitTypeDef n;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
    g.GPIO_Pin=GPIO_Pin_6;g.GPIO_Mode=GPIO_Mode_AF;g.GPIO_Speed=GPIO_Speed_100MHz;
    g.GPIO_OType=GPIO_OType_PP;g.GPIO_PuPd=GPIO_PuPd_UP;GPIO_Init(GPIOC,&g);
    g.GPIO_Pin=GPIO_Pin_7;GPIO_Init(GPIOC,&g);
    USART_DeInit(USART6);USART_StructInit(&u);
    u.USART_BaudRate=baud;u.USART_WordLength=USART_WordLength_8b;
    u.USART_StopBits=USART_StopBits_1;u.USART_Parity=USART_Parity_No;
    u.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
    u.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
    USART_Init(USART6,&u);USART_ITConfig(USART6,USART_IT_RXNE,ENABLE);
    USART_ITConfig(USART6,USART_IT_IDLE,ENABLE);USART_Cmd(USART6,ENABLE);
    n.NVIC_IRQChannel=USART6_IRQn;n.NVIC_IRQChannelPreemptionPriority=1;
    n.NVIC_IRQChannelSubPriority=0;n.NVIC_IRQChannelCmd=ENABLE;NVIC_Init(&n);
}

// YOZ轴电机使能控制
void StepperYOZ_Enable(uint8_t addr, uint8_t state, uint8_t sync) {
    uint8_t c[6]={addr,0xF3,0xAB,state,sync,0x6B};
    USART_SendCmd(USART6,c,6);
}

// YOZ轴速度模式设置
void StepperYOZ_Speed(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc, uint8_t sync) {
    uint8_t c[8]={addr,0xF6,dir,(uint8_t)(rpm>>8),(uint8_t)rpm,acc,sync,0x6B};
    USART_SendCmd(USART6,c,8);
}

// YOZ轴位置命令发送
void StepperYOZ_Position(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc, uint32_t pulses, uint8_t rel, uint8_t sync) {
    uint8_t c[13]={addr,0xFD,dir,(uint8_t)(rpm>>8),(uint8_t)rpm,acc,
        (uint8_t)(pulses>>24),(uint8_t)(pulses>>16),(uint8_t)(pulses>>8),(uint8_t)pulses,rel,sync,0x6B};
    USART_SendCmd(USART6,c,13);
}

// YOZ轴电机停止
void StepperYOZ_Stop(uint8_t addr, uint8_t sync) {
    uint8_t c[5]={addr,0xFE,0x98,sync,0x6B};
    USART_SendCmd(USART6,c,5);
}

// YOZ轴同步启动
void StepperYOZ_SyncStart(uint8_t addr) {
    uint8_t c[4]={addr,0xFF,0x66,0x6B};
    USART_SendCmd(USART6,c,4);
}

// USART6中断服务(YOZ面)
void USART6_IRQHandler(void) {
    if(USART_GetITStatus(USART6,USART_IT_RXNE)!=RESET) {
        uint8_t d=(uint8_t)USART6->DR;
        USART_ClearITPendingBit(USART6,USART_IT_RXNE);
        (void)d;
    }
    if(USART_GetITStatus(USART6,USART_IT_IDLE)!=RESET) {
        USART6->SR; USART6->DR;
    }
}

// USART2中断服务(XOY面)
void USART2_IRQHandler(void) {
    if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET) {
        uint8_t d=(uint8_t)USART2->DR;
        USART_ClearITPendingBit(USART2,USART_IT_RXNE);
        (void)d;
    }
    if(USART_GetITStatus(USART2,USART_IT_IDLE)!=RESET) {
        USART2->SR; USART2->DR;
    }
}
