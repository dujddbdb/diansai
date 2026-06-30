// BNO080 9轴IMU驱动: I2C1(PB8=SCL,PB9=SDA,400kHz), PB5=INTN下降沿, SHTP协议, 四元数→欧拉角

#include "bno080.h"
#include "board.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

// BNO080 INTN引脚: PB5
#define BNO080_INTN_PIN     GPIO_Pin_5
#define BNO080_INTN_PORT    GPIOB

// I2C设备检测超时
#define I2C_TIMEOUT_COUNT           100000U
// I2C传输超时(ms)
#define I2C_TRANSFER_TIMEOUT_MS      5U

// I2C数据就绪标志: INTN中断已触发
static volatile uint8_t g_i2c_data_ready = 0;
// I2C忙标志: 正在读包中, 防止重入
// EXTI5中断触发次数统计(调试用)
volatile uint32_t g_exti5_count = 0;
// SHTP各通道发送序号
static uint8_t g_sequence[6] = {0, 0, 0, 0, 0, 0};
// BNO080运行时I2C地址(7位)
static uint8_t g_bno080_addr = 0x00;

// 内部函数前向声明
static void I2C1_Init(void);
static void EXTI5_Init(void);
static uint8_t I2C1_WaitWhileBusy(void);
static uint8_t I2C1_WaitEvent(uint32_t event);
static uint8_t I2C1_Write_NBytes(const uint8_t *pdata, uint16_t size);
static uint8_t I2C1_Read_NBytes(uint8_t *pdata, uint16_t size);

// 初始化BNO080 I2C通信接口和INTN外部中断
void BNO080_I2C_Init(void)
{
    g_i2c_data_ready = 0;
    I2C1_Init();
    EXTI5_Init();
}

// 初始化I2C1: PB8=SCL, PB9=SDA, 400kHz
static void I2C1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    I2C_DeInit(I2C1);
    I2C_StructInit(&I2C_InitStructure);
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 400000;
    I2C_Init(I2C1, &I2C_InitStructure);
    I2C_Cmd(I2C1, ENABLE);
}

// 初始化PB5外部中断: INTN下降沿触发
static void EXTI5_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = BNO080_INTN_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(BNO080_INTN_PORT, &GPIO_InitStructure);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource5);

    EXTI_InitStructure.EXTI_Line = EXTI_Line5;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    if (GPIO_ReadInputDataBit(BNO080_INTN_PORT, BNO080_INTN_PIN) == Bit_RESET) {
        g_i2c_data_ready = 1;
    }
}

// EXTI9_5中断: INTN下降沿→置位数据就绪标志
void EXTI9_5_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line5) != RESET) {
        EXTI_ClearITPendingBit(EXTI_Line5);
        g_exti5_count++;
        g_i2c_data_ready = 1;
    }
}

// 等待I2C总线空闲(BUSY清零), 超时返回0
static uint8_t I2C1_WaitWhileBusy(void)
{
    uint32_t timeout = HAL_GetTick();
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) {
        if ((HAL_GetTick() - timeout) > I2C_TRANSFER_TIMEOUT_MS) {
            return 0;
        }
    }
    return 1;
}

// 等待I2C事件, 超时发STOP释放总线, 返回0
static uint8_t I2C1_WaitEvent(uint32_t event)
{
    uint32_t timeout = HAL_GetTick();
    while (!I2C_CheckEvent(I2C1, event)) {
        if ((HAL_GetTick() - timeout) > I2C_TRANSFER_TIMEOUT_MS) {
            I2C_GenerateSTOP(I2C1, ENABLE);
            return 0;
        }
    }
    return 1;
}

// I2C1发送N字节数据
static uint8_t I2C1_Write_NBytes(const uint8_t *pdata, uint16_t size)
{
    uint16_t i;
    uint32_t timeout;

    if ((pdata == 0) || (size == 0)) return 0;
    if (!I2C1_WaitWhileBusy()) return 0;

    I2C_GenerateSTART(I2C1, ENABLE);
    if (!I2C1_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) return 0;

    I2C_Send7bitAddress(I2C1, (uint8_t)(g_bno080_addr << 1), I2C_Direction_Transmitter);
    if (!I2C1_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) return 0;

    for (i = 0; i < size; i++) {
        I2C_SendData(I2C1, pdata[i]);
        timeout = HAL_GetTick();
        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
            if ((HAL_GetTick() - timeout) > I2C_TRANSFER_TIMEOUT_MS) {
                I2C_GenerateSTOP(I2C1, ENABLE);
                return 0;
            }
        }
    }

    I2C_GenerateSTOP(I2C1, ENABLE);
    return 1;
}

// I2C1接收N字节数据, 最后一字节发NACK+STOP
static uint8_t I2C1_Read_NBytes(uint8_t *pdata, uint16_t size)
{
    uint16_t i;
    uint32_t timeout;

    if ((pdata == 0) || (size == 0)) return 0;
    if (!I2C1_WaitWhileBusy()) return 0;

    I2C_AcknowledgeConfig(I2C1, ENABLE);
    I2C_GenerateSTART(I2C1, ENABLE);
    if (!I2C1_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) return 0;

    I2C_Send7bitAddress(I2C1, (uint8_t)((g_bno080_addr << 1) | 0x01), I2C_Direction_Receiver);
    if (!I2C1_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) return 0;

    for (i = 0; i < size; i++) {
        if (i == (size - 1U)) {
            I2C_AcknowledgeConfig(I2C1, DISABLE);
            I2C_GenerateSTOP(I2C1, ENABLE);
        }
        timeout = HAL_GetTick();
        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
            if ((HAL_GetTick() - timeout) > I2C_TRANSFER_TIMEOUT_MS) {
                I2C_GenerateSTOP(I2C1, ENABLE);
                I2C_AcknowledgeConfig(I2C1, ENABLE);
                return 0;
            }
        }
        pdata[i] = I2C_ReceiveData(I2C1);
    }

    I2C_AcknowledgeConfig(I2C1, ENABLE);
    return 1;
}

// 发送SHTP软复位命令, 等待BNO080重启完成(最多500ms)
void bsp_i2c_soft_reset(void)
{
    uint8_t reset_cmd[5];
    uint8_t dummy[4];
    uint32_t start;

    reset_cmd[0] = 5;
    reset_cmd[1] = 0;
    reset_cmd[2] = CHANNEL_EXECUTABLE;
    reset_cmd[3] = 0;
    reset_cmd[4] = 1;

    (void)I2C1_Write_NBytes(reset_cmd, sizeof(reset_cmd));
    delay_ms(50);

    start = HAL_GetTick();
    while ((HAL_GetTick() - start) < 500U) {
        if (!I2C1_Read_NBytes(dummy, sizeof(dummy))) break;
        if ((dummy[0] == 0U) && (dummy[1] == 0U) && (dummy[2] == 0U) && (dummy[3] == 0U)) break;
        delay_ms(10);
    }
    delay_ms(50);
}

// 从I2C接收一个完整SHTP数据包, 先读4字节包头再分块读剩余数据
uint8_t bsp_i2c_receive_packet(SHTP_Packet_TypeDef *packet)
{
    uint8_t header[SHTP_HEADER_LENGTH];
    uint8_t temp_buffer[I2C_BUFFER_LENGTH];
    uint16_t packet_length;
    uint16_t remaining;
    uint16_t data_spot = 0;

    if (packet == 0) return 0;
    if (!I2C1_Read_NBytes(header, SHTP_HEADER_LENGTH)) return 0;

    packet->header[0] = header[0];
    packet->header[1] = header[1];
    packet->header[2] = header[2];
    packet->header[3] = header[3];
    packet->channel_number = header[2];
    packet->sequence_number = header[3];

    packet_length = (uint16_t)(((uint16_t)header[1] << 8) | header[0]);
    packet_length &= 0x7FFFu;

    if (packet_length < SHTP_HEADER_LENGTH) {
        packet->data_length = 0;
        return 0;
    }

    remaining = packet_length - SHTP_HEADER_LENGTH;
    packet->data_length = (remaining > MAX_PACKET_SIZE) ? MAX_PACKET_SIZE : remaining;

    while (remaining > 0U) {
        uint16_t chunk_size = remaining;
        uint16_t i;
        if (chunk_size > (I2C_BUFFER_LENGTH - SHTP_HEADER_LENGTH))
            chunk_size = I2C_BUFFER_LENGTH - SHTP_HEADER_LENGTH;
        if (!I2C1_Read_NBytes(temp_buffer, chunk_size + SHTP_HEADER_LENGTH)) return 0;
        for (i = 0; i < chunk_size; i++) {
            if (data_spot < MAX_PACKET_SIZE)
                packet->data[data_spot++] = temp_buffer[SHTP_HEADER_LENGTH + i];
        }
        remaining -= chunk_size;
    }
    return 1;
}

// 通过I2C发送SHTP数据包, channel_number=通道号, data=载荷, data_length=载荷长度
uint8_t bsp_i2c_send_packet(uint8_t channel_number, const uint8_t *data, uint16_t data_length)
{
    uint8_t packet_buffer[MAX_PACKET_SIZE + SHTP_HEADER_LENGTH];
    uint16_t packet_length;
    uint16_t i;

    if ((channel_number >= 6U) || (data == 0) || (data_length > MAX_PACKET_SIZE)) return 0;

    packet_length = data_length + SHTP_HEADER_LENGTH;
    packet_buffer[0] = (uint8_t)(packet_length & 0xFFU);
    packet_buffer[1] = (uint8_t)((packet_length >> 8) & 0x7FU);
    packet_buffer[2] = channel_number;
    packet_buffer[3] = g_sequence[channel_number]++;

    for (i = 0; i < data_length; i++)
        packet_buffer[SHTP_HEADER_LENGTH + i] = data[i];

    return I2C1_Write_NBytes(packet_buffer, packet_length);
}

// 阻塞等待数据就绪(INTN信号), timeout_ms=超时(ms)
uint8_t bsp_i2c_wait_for_data(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (!g_i2c_data_ready) {
        if ((HAL_GetTick() - start) > timeout_ms) return 0;
    }
    return 1;
}

// 非阻塞获取数据就绪标志(软标志+硬引脚)
uint8_t bsp_i2c_get_data_ready_flag(void)
{
    if (g_i2c_data_ready) return 1;
    if (GPIO_ReadInputDataBit(BNO080_INTN_PORT, BNO080_INTN_PIN) == Bit_RESET) return 1;
    return 0;
}

// 清除数据就绪软标志
void bsp_i2c_clear_data_ready_flag(void)
{
    g_i2c_data_ready = 0;
}

// I2C总线恢复: 手动产生9个SCL脉冲+STOP强制释放从机
static void I2C_Bus_Unlock(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    uint8_t i;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_SetBits(GPIOB, GPIO_Pin_8 | GPIO_Pin_9);
    delay_ms(1);

    for (i = 0; i < 9; i++) {
        GPIO_ResetBits(GPIOB, GPIO_Pin_8);
        delay_ms(1);
        GPIO_SetBits(GPIOB, GPIO_Pin_8);
        delay_ms(1);
    }

    GPIO_ResetBits(GPIOB, GPIO_Pin_9);
    delay_ms(1);
    GPIO_SetBits(GPIOB, GPIO_Pin_9);
    delay_ms(1);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// 检测BNO080设备: 总线解锁→I2C重初始化→扫描0x4B/0x4A地址
uint8_t bsp_i2c_device_detect(void)
{
    uint32_t timeout = 0;

    I2C_Bus_Unlock();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    I2C_DeInit(I2C1);
    {
        I2C_InitTypeDef I2C_InitStructure;
        I2C_StructInit(&I2C_InitStructure);
        I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
        I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
        I2C_InitStructure.I2C_OwnAddress1 = 0x00;
        I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
        I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
        I2C_InitStructure.I2C_ClockSpeed = 400000;
        I2C_Init(I2C1, &I2C_InitStructure);
        I2C_Cmd(I2C1, ENABLE);
    }

    timeout = 0;
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) {
        timeout++;
        if (timeout > I2C_TIMEOUT_COUNT) {
            return 0;
        }
    }

    {
        uint8_t addr_list[2] = {0x4B, 0x4A};
        uint8_t addr_idx;
        uint8_t found = 0;

        for (addr_idx = 0; addr_idx < 2; addr_idx++) {
            uint8_t addr_7bit = addr_list[addr_idx];

            I2C_GenerateSTART(I2C1, ENABLE);
            timeout = 0;
            while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
                timeout++;
                if (timeout > I2C_TIMEOUT_COUNT) {
                    I2C_GenerateSTOP(I2C1, ENABLE);
                    break;
                }
            }
            if (timeout > I2C_TIMEOUT_COUNT) continue;

            I2C_Send7bitAddress(I2C1, (uint8_t)(addr_7bit << 1), I2C_Direction_Transmitter);
            timeout = 0;
            while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
                timeout++;
                if (timeout > I2C_TIMEOUT_COUNT) {
                    I2C_GenerateSTOP(I2C1, ENABLE);
                    break;
                }
            }

            I2C_GenerateSTOP(I2C1, ENABLE);

            if (timeout <= I2C_TIMEOUT_COUNT) {
                g_bno080_addr = addr_7bit;
                found = 1;
                break;
            }
        }

        if (!found) {
            return 0;
        }
    }
    return 1;
}

// SHTP报告ID定义
#define SHTP_REPORT_COMMAND_RESPONSE      0xF1
#define SHTP_REPORT_PRODUCT_ID_RESPONSE   0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST    0xF9
#define SHTP_REPORT_BASE_TIMESTAMP        0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND   0xFD
#define BNO080_EXECUTABLE_RESET_COMPLETE  0x01U

// 传感器报告ID
#define SENSOR_REPORTID_ROTATION_VECTOR       0x05
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR  0x08

// 定点数Q值: 四元数Q14, 精度Q12
#define BNO080_ROTATION_VECTOR_Q1    14
#define BNO080_ROTATION_ACCURACY_Q1  12

// 报告间隔20ms(50Hz), 初始化超时300ms
#define BNO080_REPORT_INTERVAL_MS    20U
#define BNO080_INIT_TIMEOUT_MS      300U
#define BNO080_DATA_TIMEOUT_MS       10U
#define BNO080_MAX_INIT_PACKETS      12U

// BNO080驱动内部上下文
typedef struct {
    uint8_t product_id_ok;
    uint8_t feature_enabled;
    uint8_t new_data;
    uint8_t report_id;
    BNO080_RotationVector_t rotation_vector;
} BNO080_Context_t;

static BNO080_Context_t g_bno080;

// 内部函数前向声明
static float bno080_q_to_float(int16_t fixed_point_value, uint8_t q_point);
static uint8_t bno080_request_product_id(void);
static uint8_t bno080_set_feature_command(uint8_t report_id, uint16_t time_between_reports_ms);
static uint8_t bno080_wait_for_packet(uint32_t timeout_ms, SHTP_Packet_TypeDef *packet);
static uint8_t bno080_handle_packet(const SHTP_Packet_TypeDef *packet);
static uint8_t bno080_parse_input_report(const SHTP_Packet_TypeDef *packet);
static void bno080_clear_context(void);

// Q定点数→浮点数
static float bno080_q_to_float(int16_t fixed_point_value, uint8_t q_point)
{
    return ((float)fixed_point_value) / (float)(1 << q_point);
}

// 清零驱动上下文
static void bno080_clear_context(void)
{
    memset(&g_bno080, 0, sizeof(g_bno080));
}

// 等待数据就绪→接收SHTP包
static uint8_t bno080_wait_for_packet(uint32_t timeout_ms, SHTP_Packet_TypeDef *packet)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) <= timeout_ms) {
        if (bsp_i2c_get_data_ready_flag()) {
            bsp_i2c_clear_data_ready_flag();
            if (bsp_i2c_receive_packet(packet)) return 1;
        }
    }
    return 0;
}

// 请求产品ID
static uint8_t bno080_request_product_id(void)
{
    uint8_t request[2] = {SHTP_REPORT_PRODUCT_ID_REQUEST, 0};
    SHTP_Packet_TypeDef packet;
    uint8_t i;

    if (!bsp_i2c_send_packet(CHANNEL_CONTROL, request, sizeof(request))) return 0;

    for (i = 0; i < BNO080_MAX_INIT_PACKETS; i++) {
        if (!bno080_wait_for_packet(BNO080_INIT_TIMEOUT_MS, &packet)) return 0;
        if ((packet.channel_number == CHANNEL_CONTROL) &&
            (packet.data_length >= 2U) &&
            (packet.data[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)) {
            g_bno080.product_id_ok = 1;
            if (packet.data_length >= 14U) {
                uint32_t part = ((uint32_t)packet.data[7] << 24) | ((uint32_t)packet.data[6] << 16)
                              | ((uint32_t)packet.data[5] << 8)  |  (uint32_t)packet.data[4];
                uint16_t patch = (uint16_t)(((uint16_t)packet.data[13] << 8) | packet.data[12]);
                (void)part;
                (void)patch;
            }
            return 1;
        }
        (void)bno080_handle_packet(&packet);
    }
    return 0;
}

// 发送SET_FEATURE命令使能传感器报告
static uint8_t bno080_set_feature_command(uint8_t report_id, uint16_t time_between_reports_ms)
{
    uint8_t command[17];
    uint32_t interval_us = (uint32_t)time_between_reports_ms * 1000UL;
    memset(command, 0, sizeof(command));
    command[0] = SHTP_REPORT_SET_FEATURE_COMMAND;
    command[1] = report_id;
    command[5] = (uint8_t)(interval_us & 0xFFU);
    command[6] = (uint8_t)((interval_us >> 8) & 0xFFU);
    command[7] = (uint8_t)((interval_us >> 16) & 0xFFU);
    command[8] = (uint8_t)((interval_us >> 24) & 0xFFU);
    return bsp_i2c_send_packet(CHANNEL_CONTROL, command, sizeof(command));
}

// 解析传感器报告: 提取旋转向量四元数(Q14定点→浮点)
static uint8_t bno080_parse_input_report(const SHTP_Packet_TypeDef *packet)
{
    uint8_t report_id, status;
    int16_t data1, data2, data3, data4, data5 = 0;

    if (packet->data_length < 17U) return 0;
    if (packet->data[0] != SHTP_REPORT_BASE_TIMESTAMP) return 0;

    report_id = packet->data[5];
    status = packet->data[7] & 0x03U;
    data1 = (int16_t)(((uint16_t)packet->data[10] << 8) | packet->data[9]);
    data2 = (int16_t)(((uint16_t)packet->data[12] << 8) | packet->data[11]);
    data3 = (int16_t)(((uint16_t)packet->data[14] << 8) | packet->data[13]);
    data4 = (int16_t)(((uint16_t)packet->data[16] << 8) | packet->data[15]);
    if (packet->data_length >= 19U)
        data5 = (int16_t)(((uint16_t)packet->data[18] << 8) | packet->data[17]);

    if ((report_id != SENSOR_REPORTID_ROTATION_VECTOR) &&
        (report_id != SENSOR_REPORTID_GAME_ROTATION_VECTOR)) return 0;

    g_bno080.report_id = report_id;
    g_bno080.rotation_vector.i = bno080_q_to_float(data1, BNO080_ROTATION_VECTOR_Q1);
    g_bno080.rotation_vector.j = bno080_q_to_float(data2, BNO080_ROTATION_VECTOR_Q1);
    g_bno080.rotation_vector.k = bno080_q_to_float(data3, BNO080_ROTATION_VECTOR_Q1);
    g_bno080.rotation_vector.real = bno080_q_to_float(data4, BNO080_ROTATION_VECTOR_Q1);
    g_bno080.rotation_vector.rad_accuracy = bno080_q_to_float(data5, BNO080_ROTATION_ACCURACY_Q1);
    g_bno080.rotation_vector.accuracy = status;
    g_bno080.new_data = 1;
    return 1;
}

// 根据通道类型分发SHTP数据包
static uint8_t bno080_handle_packet(const SHTP_Packet_TypeDef *packet) {
    if ((packet->channel_number == CHANNEL_EXECUTABLE) &&
        (packet->data_length >= 1U) &&
        (packet->data[0] == BNO080_EXECUTABLE_RESET_COMPLETE)) {
        g_bno080.new_data = 0U;
        return 1U;
    }
    if (packet->channel_number == CHANNEL_REPORTS) {
        return bno080_parse_input_report(packet);
    }
    if (packet->channel_number == CHANNEL_CONTROL) {
        if ((packet->data_length >= 2U) && (packet->data[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)) {
            g_bno080.product_id_ok = 1;
            return 1;
        }
        if ((packet->data_length >= 1U) && (packet->data[0] == SHTP_REPORT_COMMAND_RESPONSE))
            return 1;
    }
    return 0;
}

// 初始化BNO080: 清理→设备检测→软复位→排空→产品ID→使能旋转向量
uint8_t bno080_init(void)
{
    uint8_t i;
    SHTP_Packet_TypeDef packet;

    bno080_clear_context();

    if (!bsp_i2c_device_detect()) {
        return 0;
    }

    bsp_i2c_soft_reset();
    delay_ms(300);

    for (i = 0; i < BNO080_MAX_INIT_PACKETS; i++) {
        if (!bno080_wait_for_packet(200U, &packet)) break;
        (void)bno080_handle_packet(&packet);
    }

    if (!bno080_request_product_id()) {
        return 0;
    }

    if (!bno080_set_feature_command(SENSOR_REPORTID_GAME_ROTATION_VECTOR, BNO080_REPORT_INTERVAL_MS)) {
        return 0;
    }
    g_bno080.feature_enabled = 1;

    for (i = 0; i < 10; i++) {
        if (!bno080_wait_for_packet(50U, &packet)) continue;
        (void)bno080_handle_packet(&packet);
        if (g_bno080.new_data) {
            g_bno080.new_data = 0;
            return 1;
        }
    }
    return 1;
}

// 主循环更新(非阻塞): 检测INTN→读包→分发
uint8_t bno080_update(void) {
    SHTP_Packet_TypeDef packet;
    uint8_t result = 0;

    if (!g_bno080.feature_enabled) return 0;
    if (!bsp_i2c_get_data_ready_flag()) return 0;

    bsp_i2c_clear_data_ready_flag();
    if (bsp_i2c_receive_packet(&packet)) {
        result = bno080_handle_packet(&packet);
    }

    return result;
}

// 检查是否有新数据(消费型: 读取后自动清零)
uint8_t bno080_data_available(void) {
    if (g_bno080.new_data) {
        g_bno080.new_data = 0;
        return 1;
    }
    return 0;
}

// 获取最新旋转向量(四元数)
const BNO080_RotationVector_t *bno080_get_rotation_vector(void)
{
    return &g_bno080.rotation_vector;
}

// 四元数→欧拉角(roll/pitch/yaw, 度)
void bno080_get_euler(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    float qw = g_bno080.rotation_vector.real;
    float qx = g_bno080.rotation_vector.i;
    float qy = g_bno080.rotation_vector.j;
    float qz = g_bno080.rotation_vector.k;
    float norm, roll, pitch, yaw;

    norm = sqrtf((qw * qw) + (qx * qx) + (qy * qy) + (qz * qz));
    if (norm > 0.0f) {
        qw /= norm; qx /= norm; qy /= norm; qz /= norm;
    }

    roll = atan2f(2.0f * ((qw * qx) + (qy * qz)), 1.0f - 2.0f * ((qx * qx) + (qy * qy)));

    {
        float sinp = 2.0f * ((qw * qy) - (qz * qx));
        if (sinp >= 1.0f) pitch = 1.5707963f;
        else if (sinp <= -1.0f) pitch = -1.5707963f;
        else pitch = asinf(sinp);
    }

    yaw = atan2f(2.0f * ((qw * qz) + (qx * qy)), 1.0f - 2.0f * ((qy * qy) + (qz * qz)));

    if (roll_deg != 0)  *roll_deg  = roll * 57.2957795f;
    if (pitch_deg != 0) *pitch_deg = pitch * 57.2957795f;
    if (yaw_deg != 0)   *yaw_deg   = yaw * 57.2957795f;
}

// BNO080专用定时器初始化: TIM10, 2ms/500Hz, 最高优先级
void BNO080_TIM10_Init(void)
{
    TIM_TimeBaseInitTypeDef tim;
    NVIC_InitTypeDef        nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);

    TIM_TimeBaseStructInit(&tim);
    tim.TIM_Prescaler     = 168 - 1;
    tim.TIM_Period        = 2000 - 1;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM10, &tim);

    nvic.NVIC_IRQChannel                   = TIM1_UP_TIM10_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    TIM_ClearFlag(TIM10, TIM_FLAG_Update);
    TIM_ITConfig(TIM10, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM10, ENABLE);
}

// TIM10中断: 每2ms排空SHTP数据包, 最多5次迭代
void TIM1_UP_TIM10_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM10, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM10, TIM_IT_Update);
        extern volatile uint16_t bno080_ok_count;
        uint8_t iter = 0;
        while (bno080_update() && iter < 5) {
            bno080_ok_count++;
            iter++;
        }
    }
}
