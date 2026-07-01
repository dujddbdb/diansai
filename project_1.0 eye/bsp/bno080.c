// @file    bno080.c
// @brief   BNO080/Bno085 9轴IMU驱动实现 (I2C + SHTP协议)
// @note    I2C1: PB8(SCL), PB9(SDA), 400kHz, PB5(INTN)下降沿中断

#include "bno080.h"
#include "board.h"
#include "uart3.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

// PB8=SCL, PB9=SDA, PB5=INT
#define BNO080_INTN_PIN     GPIO_Pin_5
#define BNO080_INTN_PORT    GPIOB

#define I2C_TIMEOUT_COUNT           100000U
#define I2C_TRANSFER_TIMEOUT_MS       5U

// I2C数据就绪标志 (由EXTI中断置位)
static volatile uint8_t g_i2c_data_ready = 0;
// EXTI5中断计数 (调试用)
volatile uint32_t g_exti5_count = 0;
// SHTP各通道序列号
static uint8_t g_sequence[6] = {0, 0, 0, 0, 0, 0};
// BNO080设备I2C地址 (7位)
static uint8_t g_bno080_addr = 0x00;

// 内部函数声明
static void I2C1_Init(void);
static void EXTI5_Init(void);
static uint8_t I2C1_WaitWhileBusy(void);
static uint8_t I2C1_WaitEvent(uint32_t event);
static uint8_t I2C1_Write_NBytes(const uint8_t *pdata, uint16_t size);
static uint8_t I2C1_Read_NBytes(uint8_t *pdata, uint16_t size);

// ===== I2C底层 =====

// BNO080 I2C初始化: 配置I2C1 + EXTI5中断
void BNO080_I2C_Init(void)
{
    g_i2c_data_ready = 0;
    I2C1_Init();
    EXTI5_Init();
}

// 功能: 初始化I2C1 (PB8=SCL, PB9=SDA, 400kHz)
static void I2C1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;

    // 使能GPIOB和I2C1时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    // 配置PB8/PB9复用为I2C1
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

    // GPIO配置: 复用功能, 开漏输出, 上拉, 100MHz
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // I2C配置: 标准模式, 7位地址, 400kHz, 使能应答
    I2C_DeInit(I2C1);
    I2C_StructInit(&I2C_InitStructure);
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 400000;
    I2C_Init(I2C1, &I2C_InitStructure);
    // 使能I2C1
    I2C_Cmd(I2C1, ENABLE);
}

// 功能: 初始化EXTI5 (PB5), 下降沿触发, 用于接收BNO080数据就绪中断
static void EXTI5_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 使能GPIOB和SYSCFG时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    // 配置PB5为上拉输入 (INTN)
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = BNO080_INTN_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(BNO080_INTN_PORT, &GPIO_InitStructure);

    // 配置EXTI线5映射到GPIOB
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource5);

    // EXTI配置: 线5, 中断模式, 下降沿触发
    EXTI_InitStructure.EXTI_Line = EXTI_Line5;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // NVIC配置: EXTI9_5_IRQn, 最低优先级
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 上电时若INTN已为低则立即置位数据就绪
    if (GPIO_ReadInputDataBit(BNO080_INTN_PORT, BNO080_INTN_PIN) == Bit_RESET) {
        g_i2c_data_ready = 1;
    }
}

// EXTI9_5中断服务函数: 检测BNO080 INTN下降沿, 置位数据就绪标志
void EXTI9_5_IRQHandler(void)
{
    // 检查EXTI线5中断标志
    if (EXTI_GetITStatus(EXTI_Line5) != RESET) {
        // 中断计数+1
        g_exti5_count++;
        // 置位数据就绪标志
        g_i2c_data_ready = 1;
        // 清除中断挂起位
        EXTI_ClearITPendingBit(EXTI_Line5);
    }
}

// 功能: 等待I2C总线空闲, 超时返回0
static uint8_t I2C1_WaitWhileBusy(void)
{
    uint32_t timeout = HAL_GetTick();
    // 等待BUSY标志清零
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) {
        if ((HAL_GetTick() - timeout) > I2C_TRANSFER_TIMEOUT_MS) {
            return 0;
        }
    }
    return 1;
}

// 功能: 等待指定I2C事件, 超时发送STOP并返回0
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

// 功能: I2C写N字节数据
static uint8_t I2C1_Write_NBytes(const uint8_t *pdata, uint16_t size)
{
    uint16_t i;
    uint32_t timeout;

    // 参数检查
    if ((pdata == 0) || (size == 0)) return 0;
    // 等待总线空闲
    if (!I2C1_WaitWhileBusy()) return 0;

    // 发送起始信号
    I2C_GenerateSTART(I2C1, ENABLE);
    // 等待主模式选择事件
    if (!I2C1_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) return 0;

    // 发送设备地址+写方向
    I2C_Send7bitAddress(I2C1, (uint8_t)(g_bno080_addr << 1), I2C_Direction_Transmitter);
    // 等待发送模式选择事件
    if (!I2C1_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) return 0;

    // 逐字节发送数据
    for (i = 0; i < size; i++) {
        I2C_SendData(I2C1, pdata[i]);
        timeout = HAL_GetTick();
        // 等待字节发送完成
        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
            if ((HAL_GetTick() - timeout) > I2C_TRANSFER_TIMEOUT_MS) {
                I2C_GenerateSTOP(I2C1, ENABLE);
                return 0;
            }
        }
    }

    // 发送停止信号
    I2C_GenerateSTOP(I2C1, ENABLE);
    return 1;
}

// 功能: I2C读N字节数据
static uint8_t I2C1_Read_NBytes(uint8_t *pdata, uint16_t size)
{
    uint16_t i;
    uint32_t timeout;

    // 参数检查
    if ((pdata == 0) || (size == 0)) return 0;
    // 等待总线空闲
    if (!I2C1_WaitWhileBusy()) return 0;

    // 使能应答
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    // 发送起始信号
    I2C_GenerateSTART(I2C1, ENABLE);
    // 等待主模式选择事件
    if (!I2C1_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) return 0;

    // 发送设备地址+读方向
    I2C_Send7bitAddress(I2C1, (uint8_t)((g_bno080_addr << 1) | 0x01), I2C_Direction_Receiver);
    // 等待接收模式选择事件
    if (!I2C1_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) return 0;

    // 逐字节接收数据
    for (i = 0; i < size; i++) {
        // 最后一个字节: 关闭应答+发送STOP
        if (i == (size - 1U)) {
            I2C_AcknowledgeConfig(I2C1, DISABLE);
            I2C_GenerateSTOP(I2C1, ENABLE);
        }
        timeout = HAL_GetTick();
        // 等待字节接收完成
        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
            if ((HAL_GetTick() - timeout) > I2C_TRANSFER_TIMEOUT_MS) {
                I2C_GenerateSTOP(I2C1, ENABLE);
                I2C_AcknowledgeConfig(I2C1, ENABLE);
                return 0;
            }
        }
        // 读取接收数据
        pdata[i] = I2C_ReceiveData(I2C1);
    }

    // 恢复应答使能
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    return 1;
}

// ===== SHTP协议层 =====

// 功能: 软件复位BNO080, 清空旧数据包
void bsp_i2c_soft_reset(void)
{
    uint8_t reset_cmd[5];
    uint8_t dummy[4];
    uint32_t start;

    // 构造复位命令: 长度5字节, 可执行通道, 命令码1
    reset_cmd[0] = 5;
    reset_cmd[1] = 0;
    reset_cmd[2] = CHANNEL_EXECUTABLE;
    reset_cmd[3] = 0;
    reset_cmd[4] = 1;

    // 发送复位命令
    (void)I2C1_Write_NBytes(reset_cmd, sizeof(reset_cmd));
    delay_ms(50);

    // 循环读取直到收到全0响应或超时, 排空复位前的旧数据包
    start = HAL_GetTick();
    while ((HAL_GetTick() - start) < 500U) {
        if (!I2C1_Read_NBytes(dummy, sizeof(dummy))) break;
        if ((dummy[0] == 0U) && (dummy[1] == 0U) && (dummy[2] == 0U) && (dummy[3] == 0U)) break;
        delay_ms(10);
    }
    delay_ms(50);
}

// 功能: 接收SHTP数据包 (分块读取)
uint8_t bsp_i2c_receive_packet(SHTP_Packet_TypeDef *packet)
{
    uint8_t header[SHTP_HEADER_LENGTH];
    uint8_t temp_buffer[I2C_BUFFER_LENGTH];
    uint16_t packet_length, remaining, data_spot = 0;

    // 参数检查
    if (packet == 0) return 0;
    // 读取SHTP帧头(4字节)
    if (!I2C1_Read_NBytes(header, SHTP_HEADER_LENGTH)) return 0;

    // 解析帧头
    packet->header[0] = header[0];
    packet->header[1] = header[1];
    packet->header[2] = header[2];
    packet->header[3] = header[3];
    packet->channel_number = header[2];
    packet->sequence_number = header[3];

    // 计算包长度 (低15位有效)
    packet_length = (uint16_t)(((uint16_t)header[1] << 8) | header[0]);
    packet_length &= 0x7FFFu;

    // 包长小于帧头长度, 无效
    if (packet_length < SHTP_HEADER_LENGTH) {
        packet->data_length = 0;
        return 0;
    }

    // 计算数据部分长度
    remaining = packet_length - SHTP_HEADER_LENGTH;
    packet->data_length = (remaining > MAX_PACKET_SIZE) ? MAX_PACKET_SIZE : remaining;

    // 分块循环读取 (每块最多28字节)
    while (remaining > 0U) {
        uint16_t chunk_size = remaining;
        uint16_t i;
        // 限制每块大小
        if (chunk_size > (I2C_BUFFER_LENGTH - SHTP_HEADER_LENGTH))
            chunk_size = I2C_BUFFER_LENGTH - SHTP_HEADER_LENGTH;
        // 读取一块数据 (帧头+数据)
        if (!I2C1_Read_NBytes(temp_buffer, chunk_size + SHTP_HEADER_LENGTH)) return 0;
        // 拷贝数据到packet缓冲区
        for (i = 0; i < chunk_size; i++) {
            if (data_spot < MAX_PACKET_SIZE)
                packet->data[data_spot++] = temp_buffer[SHTP_HEADER_LENGTH + i];
        }
        remaining -= chunk_size;
    }
    return 1;
}

// 功能: 发送SHTP数据包
uint8_t bsp_i2c_send_packet(uint8_t channel_number, const uint8_t *data, uint16_t data_length)
{
    uint8_t packet_buffer[MAX_PACKET_SIZE + SHTP_HEADER_LENGTH];
    uint16_t packet_length, i;

    // 参数检查: 通道号范围/数据指针/数据长度
    if ((channel_number >= 6U) || (data == 0) || (data_length > MAX_PACKET_SIZE)) return 0;

    // 构造SHTP帧头
    packet_length = data_length + SHTP_HEADER_LENGTH;
    packet_buffer[0] = (uint8_t)(packet_length & 0xFFU);
    packet_buffer[1] = (uint8_t)((packet_length >> 8) & 0x7FU);
    packet_buffer[2] = channel_number;
    packet_buffer[3] = g_sequence[channel_number]++;

    // 拷贝数据
    for (i = 0; i < data_length; i++)
        packet_buffer[SHTP_HEADER_LENGTH + i] = data[i];

    // 发送数据包
    return I2C1_Write_NBytes(packet_buffer, packet_length);
}

// 功能: 等待数据就绪标志, 超时返回0
uint8_t bsp_i2c_wait_for_data(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (!g_i2c_data_ready) {
        if ((HAL_GetTick() - start) > timeout_ms) return 0;
    }
    return 1;
}

// 功能: 获取数据就绪状态 (软件标志 + 硬件引脚检测)
uint8_t bsp_i2c_get_data_ready_flag(void)
{
    if (g_i2c_data_ready) return 1;
    if (GPIO_ReadInputDataBit(BNO080_INTN_PORT, BNO080_INTN_PIN) == Bit_RESET) return 1;
    return 0;
}

// 功能: 清除数据就绪标志
void bsp_i2c_clear_data_ready_flag(void)
{
    g_i2c_data_ready = 0;
}

// 功能: I2C总线死锁恢复 - 发9个SCL脉冲+STOP条件强制释放总线
static void I2C_Bus_Unlock(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    uint8_t i;

    // 切换SCL/SDA为GPIO开漏输出模式
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // SCL和SDA初始拉高
    GPIO_SetBits(GPIOB, GPIO_Pin_8 | GPIO_Pin_9);
    delay_us(10);

    // 发送9个SCL时钟脉冲, 强制从机释放SDA
    for (i = 0; i < 9; i++) {
        GPIO_ResetBits(GPIOB, GPIO_Pin_8);
        delay_us(10);
        GPIO_SetBits(GPIOB, GPIO_Pin_8);
        delay_us(10);
    }

    // 发送STOP条件: SCL高时SDA从低变高
    GPIO_ResetBits(GPIOB, GPIO_Pin_9);
    delay_us(10);
    GPIO_SetBits(GPIOB, GPIO_Pin_9);
    delay_us(10);

    // 恢复为I2C复用功能
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// 功能: I2C设备检测 - 解锁总线+重新初始化+扫描地址0x4B/0x4A
uint8_t bsp_i2c_device_detect(void)
{
    uint32_t timeout = 0;

    // 解锁总线 → 重新初始化I2C1
    I2C_Bus_Unlock();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    // 重新初始化I2C1
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

    // 等待BUSY清零
    timeout = 0;
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) {
        timeout++;
        if (timeout > I2C_TIMEOUT_COUNT) {
            USART3_SendString("[BNO080] I2C detect: BUSY timeout\r\n");
            return 0;
        }
    }

    // 扫描I2C地址 0x4B 和 0x4A
    {
        uint8_t addr_list[2] = {0x4B, 0x4A};
        uint8_t addr_idx;
        uint8_t found = 0;

        for (addr_idx = 0; addr_idx < 2; addr_idx++) {
            uint8_t addr_7bit = addr_list[addr_idx];

            // 发送起始信号
            I2C_GenerateSTART(I2C1, ENABLE);
            timeout = 0;
            // 等待主模式选择
            while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
                timeout++;
                if (timeout > I2C_TIMEOUT_COUNT) {
                    I2C_GenerateSTOP(I2C1, ENABLE);
                    break;
                }
            }
            if (timeout > I2C_TIMEOUT_COUNT) continue;

            // 发送设备地址+写
            I2C_Send7bitAddress(I2C1, (uint8_t)(addr_7bit << 1), I2C_Direction_Transmitter);
            timeout = 0;
            // 等待应答(发送模式选择)
            while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
                timeout++;
                if (timeout > I2C_TIMEOUT_COUNT) {
                    I2C_GenerateSTOP(I2C1, ENABLE);
                    break;
                }
            }

            // 发送停止信号
            I2C_GenerateSTOP(I2C1, ENABLE);

            // 检测到设备应答
            if (timeout <= I2C_TIMEOUT_COUNT) {
                g_bno080_addr = addr_7bit;
                found = 1;
                {
                    char tmp[64];
                    snprintf(tmp, sizeof(tmp), "[BNO080] I2C detect: ACK at addr 0x%02X!\r\n", (unsigned)addr_7bit);
                    USART3_SendString(tmp);
                }
                break;
            } else {
                {
                    char tmp[64];
                    snprintf(tmp, sizeof(tmp), "[BNO080] I2C detect: NACK at addr 0x%02X\r\n", (unsigned)addr_7bit);
                    USART3_SendString(tmp);
                }
            }
        }

        if (!found) {
            USART3_SendString("[BNO080] Step2: FAILED - no device at 0x4B or 0x4A!\r\n");
            return 0;
        }
    }
    return 1;
}

// ===== BNO080应用层 =====

#define SHTP_REPORT_COMMAND_RESPONSE      0xF1
#define SHTP_REPORT_PRODUCT_ID_RESPONSE   0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST    0xF9
#define SHTP_REPORT_BASE_TIMESTAMP        0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND   0xFD
#define BNO080_EXECUTABLE_RESET_COMPLETE  0x01U

#define SENSOR_REPORTID_ROTATION_VECTOR       0x05
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR  0x08

#define BNO080_ROTATION_VECTOR_Q1    14
#define BNO080_ROTATION_ACCURACY_Q1  12

#define BNO080_REPORT_INTERVAL_MS    1U
#define BNO080_INIT_TIMEOUT_MS       300U
#define BNO080_DATA_TIMEOUT_MS       10U
#define BNO080_MAX_INIT_PACKETS      12U

// BNO080上下文结构体
typedef struct {
    uint8_t product_id_ok;
    uint8_t feature_enabled;
    uint8_t new_data;
    uint8_t report_id;
    BNO080_RotationVector_t rotation_vector;
} BNO080_Context_t;

static BNO080_Context_t g_bno080;

// 内部函数声明
static float bno080_q_to_float(int16_t fixed_point_value, uint8_t q_point);
static uint8_t bno080_request_product_id(void);
static uint8_t bno080_set_feature_command(uint8_t report_id, uint16_t time_between_reports_ms);
static uint8_t bno080_wait_for_packet(uint32_t timeout_ms, SHTP_Packet_TypeDef *packet);
static uint8_t bno080_handle_packet(const SHTP_Packet_TypeDef *packet);
static uint8_t bno080_parse_input_report(const SHTP_Packet_TypeDef *packet);
static void bno080_clear_context(void);

// 功能: Q定点数转浮点数
// float_val = 定点值 / (2^Q值)
static float bno080_q_to_float(int16_t fixed_point_value, uint8_t q_point)
{
    return ((float)fixed_point_value) / (float)(1 << q_point);
}

// 功能: 清零BNO080上下文
static void bno080_clear_context(void)
{
    memset(&g_bno080, 0, sizeof(g_bno080));
}

// 功能: 等待INTN中断并接收SHTP数据包
static uint8_t bno080_wait_for_packet(uint32_t timeout_ms, SHTP_Packet_TypeDef *packet)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) <= timeout_ms) {
        // 检测数据就绪
        if (bsp_i2c_get_data_ready_flag()) {
            bsp_i2c_clear_data_ready_flag();
            // 接收数据包
            if (bsp_i2c_receive_packet(packet)) return 1;
        }
    }
    return 0;
}

// 功能: 请求产品ID握手
static uint8_t bno080_request_product_id(void)
{
    uint8_t request[2] = {SHTP_REPORT_PRODUCT_ID_REQUEST, 0};
    SHTP_Packet_TypeDef packet;
    uint8_t i;

    // 发送产品ID请求 (控制通道)
    if (!bsp_i2c_send_packet(CHANNEL_CONTROL, request, sizeof(request))) return 0;

    // 循环等待响应, 最多BNO080_MAX_INIT_PACKETS个包
    for (i = 0; i < BNO080_MAX_INIT_PACKETS; i++) {
        if (!bno080_wait_for_packet(BNO080_INIT_TIMEOUT_MS, &packet)) return 0;
        // 检查是否是产品ID响应
        if ((packet.channel_number == CHANNEL_CONTROL) &&
            (packet.data_length >= 2U) &&
            (packet.data[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)) {
            g_bno080.product_id_ok = 1;
            return 1;
        }
        // 处理其他数据包
        (void)bno080_handle_packet(&packet);
    }
    return 0;
}

// 功能: 发送SET_FEATURE命令使能传感器报告
static uint8_t bno080_set_feature_command(uint8_t report_id, uint16_t time_between_reports_ms)
{
    uint8_t command[17];
    uint32_t interval_us = (uint32_t)time_between_reports_ms * 1000UL;
    // 清零命令缓冲区
    memset(command, 0, sizeof(command));
    command[0] = SHTP_REPORT_SET_FEATURE_COMMAND;
    command[1] = report_id;
    // 报告间隔(微秒), 小端序
    command[5] = (uint8_t)(interval_us & 0xFFU);
    command[6] = (uint8_t)((interval_us >> 8) & 0xFFU);
    command[7] = (uint8_t)((interval_us >> 16) & 0xFFU);
    command[8] = (uint8_t)((interval_us >> 24) & 0xFFU);
    // 发送特征设置命令
    return bsp_i2c_send_packet(CHANNEL_CONTROL, command, sizeof(command));
}

// 功能: 解析旋转向量传感器报告 (Q14四元数 + Q12精度)
static uint8_t bno080_parse_input_report(const SHTP_Packet_TypeDef *packet)
{
    uint8_t report_id, status;
    int16_t data1, data2, data3, data4, data5 = 0;

    // 数据长度检查 + 时间戳报告ID检查
    if (packet->data_length < 17U) return 0;
    if (packet->data[0] != SHTP_REPORT_BASE_TIMESTAMP) return 0;

    // 解析报告ID和状态
    report_id = packet->data[5];
    status = packet->data[7] & 0x03U;
    // 解析四元数分量 (小端序, 16位有符号)
    data1 = (int16_t)(((uint16_t)packet->data[10] << 8) | packet->data[9]);
    data2 = (int16_t)(((uint16_t)packet->data[12] << 8) | packet->data[11]);
    data3 = (int16_t)(((uint16_t)packet->data[14] << 8) | packet->data[13]);
    data4 = (int16_t)(((uint16_t)packet->data[16] << 8) | packet->data[15]);
    // 精度字段(可选)
    if (packet->data_length >= 19U)
        data5 = (int16_t)(((uint16_t)packet->data[18] << 8) | packet->data[17]);

    // 只处理旋转向量和游戏旋转向量
    if ((report_id != SENSOR_REPORTID_ROTATION_VECTOR) &&
        (report_id != SENSOR_REPORTID_GAME_ROTATION_VECTOR)) return 0;

    // 保存解析结果
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

// 功能: 按通道号分发SHTP数据包
static uint8_t bno080_handle_packet(const SHTP_Packet_TypeDef *packet)
{
    // 可执行通道: 复位完成标志
    if ((packet->channel_number == CHANNEL_EXECUTABLE) &&
        (packet->data_length >= 1U) &&
        (packet->data[0] == BNO080_EXECUTABLE_RESET_COMPLETE)) {
        g_bno080.new_data = 0U;
        return 1U;
    }
    // 传感器报告通道: 解析输入报告
    if (packet->channel_number == CHANNEL_REPORTS)
        return bno080_parse_input_report(packet);
    // 控制通道: 产品ID响应 / 命令响应
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

// BNO080完整初始化: 检测→复位→排空旧包→产品ID握手→使能旋转向量报告
uint8_t bno080_init(void)
{
    uint8_t i;
    SHTP_Packet_TypeDef packet;

    USART3_SendString("[BNO080] Init start...\r\n");

    // Step1: 清零上下文
    bno080_clear_context();
    USART3_SendString("[BNO080] Step1: context cleared\r\n");

    // Step2: I2C设备检测
    USART3_SendString("[BNO080] Step2: device detect...\r\n");
    if (!bsp_i2c_device_detect()) {
        USART3_SendString("[BNO080] Step2: FAILED - device not found on I2C bus!\r\n");
        return 0;
    }
    USART3_SendString("[BNO080] Step2: OK - device detected\r\n");

    // Step3: 软件复位
    USART3_SendString("[BNO080] Step3: soft reset...\r\n");
    bsp_i2c_soft_reset();
    delay_ms(300);
    USART3_SendString("[BNO080] Step3: OK\r\n");

    // Step4: 排空旧数据包
    USART3_SendString("[BNO080] Step4: drain stale packets...\r\n");
    for (i = 0; i < BNO080_MAX_INIT_PACKETS; i++) {
        if (!bno080_wait_for_packet(200U, &packet)) break;
        (void)bno080_handle_packet(&packet);
    }

    // Step5: 请求产品ID握手
    USART3_SendString("[BNO080] Step5: request product ID...\r\n");
    if (!bno080_request_product_id()) {
        USART3_SendString("[BNO080] Step5: FAILED - no product ID response!\r\n");
        return 0;
    }
    USART3_SendString("[BNO080] Step5: OK - product ID verified\r\n");

    // Step6: 使能游戏旋转向量报告
    USART3_SendString("[BNO080] Step6: enable game rotation vector...\r\n");
    if (!bno080_set_feature_command(SENSOR_REPORTID_GAME_ROTATION_VECTOR, BNO080_REPORT_INTERVAL_MS)) {
        USART3_SendString("[BNO080] Step6: FAILED - set feature command failed!\r\n");
        return 0;
    }
    g_bno080.feature_enabled = 1;

    // Step7: 验证数据流
    USART3_SendString("[BNO080] Step7: verify data flow...\r\n");
    for (i = 0; i < 10; i++) {
        if (!bno080_wait_for_packet(50U, &packet)) continue;
        (void)bno080_handle_packet(&packet);
        if (g_bno080.new_data) {
            g_bno080.new_data = 0;
            USART3_SendString("[BNO080] Step7: data flowing OK\r\n");
            USART3_SendString("[BNO080] Init SUCCESS!\r\n");
            return 1;
        }
    }
    USART3_SendString("[BNO080] Step7: FAILED - no first data frame\r\n");
    return 0;
}

// 主循环更新 (非阻塞): 检测INTN→读包→分发处理
uint8_t bno080_update(void)
{
    SHTP_Packet_TypeDef packet;
    uint8_t result = 0;
    // 传感器未使能则直接返回
    if (!g_bno080.feature_enabled) return 0;
    // 无数据就绪则直接返回
    if (!bsp_i2c_get_data_ready_flag()) return 0;
    // 清除标志并读取数据包
    bsp_i2c_clear_data_ready_flag();
    if (bsp_i2c_receive_packet(&packet)) {
        result = bno080_handle_packet(&packet);
    }
    return result;
}

// 消费型接口: 检查新数据可用性 (读取后自动清零)
uint8_t bno080_data_available(void)
{
    if (g_bno080.new_data) {
        g_bno080.new_data = 0;
        return 1;
    }
    return 0;
}

// 获取旋转向量只读指针
const BNO080_RotationVector_t *bno080_get_rotation_vector(void)
{
    return &g_bno080.rotation_vector;
}

// 四元数→欧拉角转换 (输出角度制)
void bno080_get_euler(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    float qw = g_bno080.rotation_vector.real;
    float qx = g_bno080.rotation_vector.i;
    float qy = g_bno080.rotation_vector.j;
    float qz = g_bno080.rotation_vector.k;
    float norm, roll, pitch, yaw;

    // 四元数归一化
    norm = sqrtf((qw * qw) + (qx * qx) + (qy * qy) + (qz * qz));
    if (norm > 0.0f) {
        qw /= norm; qx /= norm; qy /= norm; qz /= norm;
    }

    // Roll (绕X轴): atan2(2(qw*qx + qy*qz), 1-2(qx²+qy²))
    roll = atan2f(2.0f * ((qw * qx) + (qy * qz)), 1.0f - 2.0f * ((qx * qx) + (qy * qy)));

    // Pitch (绕Y轴): asin(2(qw*qy - qz*qx)), 钳位到[-π/2, π/2]
    {
        float sinp = 2.0f * ((qw * qy) - (qz * qx));
        if (sinp >= 1.0f) pitch = 1.5707963f;
        else if (sinp <= -1.0f) pitch = -1.5707963f;
        else pitch = asinf(sinp);
    }

    // Yaw (绕Z轴): atan2(2(qw*qz + qx*qy), 1-2(qy²+qz²))
    yaw = atan2f(2.0f * ((qw * qz) + (qx * qy)), 1.0f - 2.0f * ((qy * qy) + (qz * qz)));

    // 弧度转角度 (×180/π ≈ 57.2957795)
    if (roll_deg != 0)  *roll_deg  = roll * 57.2957795f;
    if (pitch_deg != 0) *pitch_deg = pitch * 57.2957795f;
    if (yaw_deg != 0)   *yaw_deg   = yaw * 57.2957795f;
}
