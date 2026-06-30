/**
 * @file    main.c
 * @brief   Eye项目主程序入口 - 云台视觉追踪控制
 * @note    功能描述:
 *          1. 板级初始化 (SysTick/SystemClock)
 *          2. 串口初始化 (USART3调试/UART5与K230通信/步进电机控制)
 *          3. BNO080 IMU初始化 (I2C1, 惯性测量)
 *          4. 视觉追踪主循环 (100Hz调度)
 *          5. IMU补偿云台角度漂移
 *
 * 硬件连接:
 *          USART3 (PD8/TX, PD9/RX) - 调试串口, 115200bps
 *          UART5  (PC12/TX, PD2/RX) - 与K230通信, 接收靶心坐标
 *          USART2 (PD5/TX, PD6/RX)  - XOY轴步进电机控制
 *          USART6 (PC6/TX, PC7/RX)  - YOZ轴步进电机控制
 *          I2C1   (PB8/SCL, PB9/SDA) - BNO080 IMU
 *
 * 调度周期: 100Hz (10ms)
 */

#include "board.h"                       // 板级支持包: SysTick初始化/延时函数/HAL_GetTick
#include "stm32f4xx.h"                   // STM32F4标准外设库: GPIO/TIM/USART/I2C等外设寄存器定义
#include "uart3.h"                       // USART3驱动: 调试串口通信 (PD8=USART3_TX, PD9=USART3_RX)
#include "uart5.h"                       // UART5驱动: K230视觉数据接收 (PC12=UART5_TX, PD2=UART5_RX)
#include "peripheral.h"                  // 外设驱动: 外部LED (PD11), 用于目标检测状态指示
#include "stepper.h"                     // 步进电机驱动: XOY轴(USART2)/YOZ轴(USART6)云台控制
#include "vision.h"                      // 视觉控制: K230数据解析/靶心追踪/激光触发
#include "bno080.h"                      // BNO080 IMU驱动: 四元数转欧拉角/惯性测量
#include "key.h"                         // 按键驱动: 四模式状态机切换

/* ================================================================
 *  串口波特率定义
 * ================================================================ */
#define EYE_DEBUG_BAUD   115200U        // USART3调试串口波特率: 115200bps, 用于打印调试信息
#define EYE_UART5_BAUD   115200U        // UART5与K230通信波特率: 115200bps, 用于接收靶心坐标
#define EYE_STEPPER_BAUD 115200U        // 步进电机串口波特率: 115200bps, ZDT磁编码器协议

/* ================================================================
 *  主函数入口
 * ================================================================
 * @brief  系统初始化和主循环
 * @note   初始化顺序: 板级 → 调试串口 → 外设 → 电机 → 传感器 → 视觉
 *         主循环频率: 100Hz (由视觉模块调度)
 */
int main(void)
{
    uint8_t imu_ready;                  // IMU初始化状态标志
    uint32_t last_vision_ms = 0U;

    /* ---- 第一阶段: 板级初始化 ---- */
    board_init();                       // 初始化SysTick定时器, 配置中断向量表

    /* ---- 第二阶段: 调试串口初始化 ---- */
    USART3_Init(EYE_DEBUG_BAUD);        // 初始化USART3调试串口 (115200bps, 8N1)

    /* ---- 第三阶段: 外设初始化 ---- */
    ExtLED_Init();                      // 初始化外部LED (PD11)
    ExtLED_Off();                       // 初始状态: LED熄灭
    Key_Init();                         // 初始化模式切换按键

    /* ---- 第四阶段: 步进电机初始化 (云台XOY和YOZ双轴) ---- */
    StepperXOY_Init(EYE_STEPPER_BAUD); // 初始化XOY轴步进电机 (USART2, PD5/TX, PD6/RX)
    StepperYOZ_Init(EYE_STEPPER_BAUD); // 初始化YOZ轴步进电机 (USART6, PC6/TX, PC7/RX)

    // 使能云台电机: X轴和Y轴都使能
    StepperXOY_Enable(GIMBAL_ADDR_X, 1U, 0U);  // X轴使能: addr=1, enable=1, direction=0
    StepperYOZ_Enable(GIMBAL_ADDR_Y, 1U, 0U);  // Y轴使能: addr=1, enable=1, direction=0

    /* ---- 第五阶段: UART5与K230通信初始化 ---- */
    UART5_Init(EYE_UART5_BAUD);         // 初始化UART5 (115200bps, K230坐标数据接收)

    /* ---- 第六阶段: BNO080 IMU初始化 (I2C1) ---- */
    BNO080_I2C_Init();                  // 初始化I2C1引脚 (PB8=SCL, PB9=SDA)
    imu_ready = bno080_init();          // 初始化BNO080, 获取初始化状态

    /* ---- 第七阶段: 视觉系统初始化 ---- */
    Vision_Init();                      // 初始化视觉模块 (解析K230数据/靶心追踪/激光触发)

    /* ---- 主循环: 100Hz视觉追踪调度 ---- */
    while (1) {
        uint32_t now_ms = HAL_GetTick();
        (void)imu_ready;
        if ((uint32_t)(now_ms - last_vision_ms) < 5U) {
            continue;
        }
        last_vision_ms = now_ms;

        Key_Scan();
        Vision_KeyControlTick();
        Vision_Process();

        /* 目标检测状态指示: LED亮=检测到目标, LED灭=未检测到 */
        if (Vision_GetContext()->target_detected) {
            ExtLED_On();                // 检测到目标: LED点亮
        } else {
            ExtLED_Off();               // 未检测到目标: LED熄灭
        }
    }
}
