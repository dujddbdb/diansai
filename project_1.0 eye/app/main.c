/**
 * @file    main.c
 * @brief   Eye项目主程序入口 - 云台视觉追踪控制
 * @note    调度周期: 100Hz (10ms)
 */

/* 系统层 */
#include "board.h"
#include "stm32f4xx.h"

/* 驱动层 */
#include "uart3.h"
#include "uart5.h"
#include "peripheral.h"
#include "stepper.h"
#include "bno080.h"
#include "key.h"

/* 控制层 */
#include "vision.h"

#define EYE_DEBUG_BAUD   115200U
#define EYE_UART5_BAUD   115200U
#define EYE_STEPPER_BAUD 115200U

int main(void)
{
    uint8_t imu_ready;
    uint32_t last_vision_ms = 0U;

    /* 板级初始化 */
    board_init();

    /* 调试串口初始化 */
    USART3_Init(EYE_DEBUG_BAUD);

    /* 外设初始化 */
    ExtLED_Init();
    ExtLED_Off();
    Key_Init();

    /* 步进电机初始化 (云台XOY和YOZ双轴) */
    StepperXOY_Init(EYE_STEPPER_BAUD);
    StepperYOZ_Init(EYE_STEPPER_BAUD);

    StepperXOY_Enable(GIMBAL_ADDR_X, 1U, 0U);
    StepperYOZ_Enable(GIMBAL_ADDR_Y, 1U, 0U);

    /* UART5与K230通信初始化 */
    UART5_Init(EYE_UART5_BAUD);

    /* BNO080 IMU初始化 */
    BNO080_I2C_Init();
    imu_ready = bno080_init();

    /* 视觉系统初始化 */
    Vision_Init();

    /* 主循环: 100Hz视觉追踪调度 */
    while (1) {
        uint32_t now_ms = HAL_GetTick();
        if ((uint32_t)(now_ms - last_imu_ms) >= VISION_IMU_FEEDFORWARD_PERIOD_MS) {
            last_imu_ms = now_ms;
            Vision_GimbalIMUCompensationTick();
        }
        (void)imu_ready;
        if ((uint32_t)(now_ms - last_vision_ms) < VISION_CONTROL_PERIOD_MS) {
            continue;
        }
        last_vision_ms = now_ms;

        Key_Scan();
        Vision_KeyControlTick();
        Vision_Process();

        /* 目标检测状态指示 */
        if (Vision_GetContext()->target_detected) {
            ExtLED_On();
        } else {
            ExtLED_Off();
        }
    }
}
