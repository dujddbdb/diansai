/**
 * @file    main.c
 * @brief   主程序入口 - 智能车巡线 + 直角转弯控制
 * @note    TIM11 1kHz: 设置 flag_1ms/flag_5ms + system_time_ms
 *          主循环: 按键扫描 → 灰度采集 → 陀螺仪轮询 → 调度函数
 */

/* 系统层 */
#include "stm32f4xx.h"
#include "board.h"
#include <string.h>

/* 驱动层 */
#include "grayscale.h"
#include "encoder.h"
#include "tb6612.h"
#include "bno080.h"
#include "peripheral.h"
#include "key.h"
#include "oled/oled.h"

/* 控制层 */
#include "track.h"
#include "uart_k230.h"
#include "uart3.h"

static void Car_OLEDShowTargetLap(void)
{
    static uint16_t last_display_lap = 0xFFFFU;
    uint16_t display_lap = Track_Target_Laps;

    if (last_display_lap == Track_Target_Laps) {
        return;
    }

    last_display_lap = Track_Target_Laps;
    OLED_Clear();
    OLED_ShowNum(0, 0, (u32)display_lap, 1, 8, 1);
    OLED_Refresh();
}

static void Car_KeyControlTick(void)
{
    if (Key_GetState(0U) == KEY_PRESS) {
        Track_ControlStart();
    }
    if (Key_GetState(1U) == KEY_PRESS) {
        Track_ControlStop();
    }
    if (Key_GetState(2U) == KEY_PRESS) {
        Track_TargetLapAdd();
    }
    if (Key_GetState(3U) == KEY_PRESS) {
        Track_TargetLapSub();
    }
}

int main(void)
{
    // 板级初始化：向量表 + SysTick时基
    board_init();

    // 外设初始化：串口、LED、按键、OLED
    USART3_Init(115200);
    K230_UART_Init(115200);
    ExtLED_Init();
    Relay_Init();
    Key_Init();
    OLED_Init();
    Car_OLEDShowTargetLap();

    // 巡线系统初始化：传感器、电机、陀螺仪、定时器
    Track_Init();
    Car_OLEDShowTargetLap();

    // 主循环
    while (1) {
        Key_Scan();
        Car_KeyControlTick();
        Car_OLEDShowTargetLap();

        // 灰度传感器数据采集
        Grayscale_Task(&gray_sensor);

        // 陀螺仪数据轮询消费
        Track_Main_Gyro();

        // 1ms周期任务：巡线PID + 直角检测 + 动作输出
        Track_Main_1ms();

        // 5ms周期任务：速度环控制
        Track_Main_5ms();

        // 调试输出（已注释）
//        Track_Main_Debug(100);
    }
}
