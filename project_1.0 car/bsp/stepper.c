// XOY/YOZ步进电机统一驱动(ZDT协议): USART6(XOY,PC6/PC7)+USART2(YOZ,PD5/PD6), 校验0x6B

#include "stepper.h"
#include "board.h"

// 功能：USART发送单字节，带超时保护，防止死循环
static void USART_SendByte(USART_TypeDef *USARTx, uint16_t data) {
    uint16_t t = 0;
    // 将数据写入发送数据寄存器
    USARTx->DR = (data & 0x01FF);
    // 等待发送数据寄存器为空，超时则返回
    while (!(USARTx->SR & USART_FLAG_TXE)) {
        if (++t > 8000) return;
    }
}

// 功能：USART发送命令字节数组，逐字节发送
static void USART_SendCmd(USART_TypeDef *USARTx, uint8_t *cmd, uint8_t len) {
    uint8_t i;
    for (i=0;i<len;i++) USART_SendByte(USARTx, cmd[i]);
}

// XOY面初始化: USART6, PC6(TX)+PC7(RX), RXNE+IDLE中断
void StepperXOY_Init(uint32_t baud)
{
    GPIO_InitTypeDef g;
    USART_InitTypeDef u;
    NVIC_InitTypeDef n;

    // 使能GPIOC和USART6时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

    // 配置PC6/PC7复用为USART6
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

    // 配置PC6(TX)：复用功能、100MHz、推挽、上拉
    g.GPIO_Pin=GPIO_Pin_6;
    g.GPIO_Mode=GPIO_Mode_AF;
    g.GPIO_Speed=GPIO_Speed_100MHz;
    g.GPIO_OType=GPIO_OType_PP;
    g.GPIO_PuPd=GPIO_PuPd_UP;
    GPIO_Init(GPIOC,&g);

    // 配置PC7(RX)：复用功能、100MHz、推挽、上拉
    g.GPIO_Pin=GPIO_Pin_7;
    GPIO_Init(GPIOC,&g);

    // 复位USART6并使用默认配置
    USART_DeInit(USART6);
    USART_StructInit(&u);

    // 配置USART6参数：波特率、8位数据、1停止位、无校验、收发模式、无流控
    u.USART_BaudRate=baud;
    u.USART_WordLength=USART_WordLength_8b;
    u.USART_StopBits=USART_StopBits_1;
    u.USART_Parity=USART_Parity_No;
    u.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
    u.USART_HardwareFlowControl=USART_HardwareFlowControl_None;

    // 初始化USART6，使能RXNE和IDLE中断，使能USART6
    USART_Init(USART6,&u);
    USART_ITConfig(USART6,USART_IT_RXNE,ENABLE);
    USART_ITConfig(USART6,USART_IT_IDLE,ENABLE);
    USART_Cmd(USART6,ENABLE);

    // 配置USART6中断：抢占优先级1，子优先级0，使能
    n.NVIC_IRQChannel=USART6_IRQn;
    n.NVIC_IRQChannelPreemptionPriority=1;
    n.NVIC_IRQChannelSubPriority=0;
    n.NVIC_IRQChannelCmd=ENABLE;
    NVIC_Init(&n);
}

// XOY面使能: [addr, 0xF3, 0xAB, state, sync, 0x6B]
void StepperXOY_Enable(uint8_t addr, uint8_t state, uint8_t sync) {
    uint8_t c[6]={addr,0xF3,0xAB,state,sync,0x6B};
    USART_SendCmd(USART6,c,6);
}

// XOY面速度模式: [addr, 0xF6, dir, rpmH, rpmL, acc, sync, 0x6B]
void StepperXOY_Speed(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc, uint8_t sync) {
    uint8_t c[8]={addr,0xF6,dir,(uint8_t)(rpm>>8),(uint8_t)rpm,acc,sync,0x6B};
    USART_SendCmd(USART6,c,8);
}

// XOY面位置模式: [addr, 0xFD, dir, rpmH, rpmL, acc, pulses[31:0], rel, sync, 0x6B]
void StepperXOY_Position(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc, uint32_t pulses, uint8_t rel, uint8_t sync) {
    uint8_t c[13]={addr,0xFD,dir,(uint8_t)(rpm>>8),(uint8_t)rpm,acc,
        (uint8_t)(pulses>>24),(uint8_t)(pulses>>16),(uint8_t)(pulses>>8),(uint8_t)pulses,rel,sync,0x6B};
    USART_SendCmd(USART6,c,13);
}

// XOY面停止: [addr, 0xFE, 0x98, sync, 0x6B]
void StepperXOY_Stop(uint8_t addr, uint8_t sync) {
    uint8_t c[5]={addr,0xFE,0x98,sync,0x6B};
    USART_SendCmd(USART6,c,5);
}

// XOY面同步启动: [addr, 0xFF, 0x66, 0x6B]
void StepperXOY_SyncStart(uint8_t addr) {
    uint8_t c[4]={addr,0xFF,0x66,0x6B};
    USART_SendCmd(USART6,c,4);
}

// YOZ面初始化: USART2, PD5(TX)+PD6(RX), RXNE+IDLE中断
void StepperYOZ_Init(uint32_t baud)
{
    GPIO_InitTypeDef g;
    USART_InitTypeDef u;
    NVIC_InitTypeDef n;

    // 使能GPIOD和USART2时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    // 配置PD5/PD6复用为USART2
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

    // 配置PD5(TX)：复用功能、100MHz、推挽、上拉
    g.GPIO_Pin=GPIO_Pin_5;
    g.GPIO_Mode=GPIO_Mode_AF;
    g.GPIO_Speed=GPIO_Speed_100MHz;
    g.GPIO_OType=GPIO_OType_PP;
    g.GPIO_PuPd=GPIO_PuPd_UP;
    GPIO_Init(GPIOD,&g);

    // 配置PD6(RX)：复用功能、100MHz、推挽、上拉
    g.GPIO_Pin=GPIO_Pin_6;
    GPIO_Init(GPIOD,&g);

    // 复位USART2并使用默认配置
    USART_DeInit(USART2);
    USART_StructInit(&u);

    // 配置USART2参数：波特率、8位数据、1停止位、无校验、收发模式、无流控
    u.USART_BaudRate=baud;
    u.USART_WordLength=USART_WordLength_8b;
    u.USART_StopBits=USART_StopBits_1;
    u.USART_Parity=USART_Parity_No;
    u.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
    u.USART_HardwareFlowControl=USART_HardwareFlowControl_None;

    // 初始化USART2，使能RXNE和IDLE中断，使能USART2
    USART_Init(USART2,&u);
    USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
    USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);
    USART_Cmd(USART2,ENABLE);

    // 配置USART2中断：抢占优先级1，子优先级0，使能
    n.NVIC_IRQChannel=USART2_IRQn;
    n.NVIC_IRQChannelPreemptionPriority=1;
    n.NVIC_IRQChannelSubPriority=0;
    n.NVIC_IRQChannelCmd=ENABLE;
    NVIC_Init(&n);
}

// YOZ面使能: [addr, 0xF3, 0xAB, state, sync, 0x6B]
void StepperYOZ_Enable(uint8_t addr, uint8_t state, uint8_t sync) {
    uint8_t c[6]={addr,0xF3,0xAB,state,sync,0x6B};
    USART_SendCmd(USART2,c,6);
}

// YOZ面速度模式: [addr, 0xF6, dir, rpmH, rpmL, acc, sync, 0x6B]
void StepperYOZ_Speed(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc, uint8_t sync) {
    uint8_t c[8]={addr,0xF6,dir,(uint8_t)(rpm>>8),(uint8_t)rpm,acc,sync,0x6B};
    USART_SendCmd(USART2,c,8);
}

// YOZ面位置模式: [addr, 0xFD, dir, rpmH, rpmL, acc, pulses[31:0], rel, sync, 0x6B]
void StepperYOZ_Position(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc, uint32_t pulses, uint8_t rel, uint8_t sync) {
    uint8_t c[13]={addr,0xFD,dir,(uint8_t)(rpm>>8),(uint8_t)rpm,acc,
        (uint8_t)(pulses>>24),(uint8_t)(pulses>>16),(uint8_t)(pulses>>8),(uint8_t)pulses,rel,sync,0x6B};
    USART_SendCmd(USART2,c,13);
}

// YOZ面停止: [addr, 0xFE, 0x98, sync, 0x6B]
void StepperYOZ_Stop(uint8_t addr, uint8_t sync) {
    uint8_t c[5]={addr,0xFE,0x98,sync,0x6B};
    USART_SendCmd(USART2,c,5);
}

// YOZ面同步启动: [addr, 0xFF, 0x66, 0x6B]
void StepperYOZ_SyncStart(uint8_t addr) {
    uint8_t c[4]={addr,0xFF,0x66,0x6B};
    USART_SendCmd(USART2,c,4);
}

// USART6中断处理函数：清除RXNE/IDLE标志，丢弃接收数据
// 功能：处理USART6接收中断和空闲中断，读取数据后丢弃（仅发送使用）
void USART6_IRQHandler(void) {
    // 处理接收数据寄存器非空中断
    if(USART_GetITStatus(USART6,USART_IT_RXNE)!=RESET) {
        // 读取接收数据
        uint8_t d=(uint8_t)USART6->DR;
        // 清除RXNE中断标志
        USART_ClearITPendingBit(USART6,USART_IT_RXNE);
        // 丢弃数据（避免未使用变量警告）
        (void)d;
    }
    // 处理空闲线路中断
    if(USART_GetITStatus(USART6,USART_IT_IDLE)!=RESET) {
        // 读SR再读DR清除IDLE标志
        USART6->SR; USART6->DR;
    }
}

// USART2中断处理函数：清除RXNE/IDLE标志，丢弃接收数据
// 功能：处理USART2接收中断和空闲中断，读取数据后丢弃（仅发送使用）
void USART2_IRQHandler(void) {
    // 处理接收数据寄存器非空中断
    if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET) {
        // 读取接收数据
        uint8_t d=(uint8_t)USART2->DR;
        // 清除RXNE中断标志
        USART_ClearITPendingBit(USART2,USART_IT_RXNE);
        // 丢弃数据（避免未使用变量警告）
        (void)d;
    }
    // 处理空闲线路中断
    if(USART_GetITStatus(USART2,USART_IT_IDLE)!=RESET) {
        // 读SR再读DR清除IDLE标志
        USART2->SR; USART2->DR;
    }
}
