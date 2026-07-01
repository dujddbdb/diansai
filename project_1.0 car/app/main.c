// @file    main.c
// @brief   主程序入口 - 智能车巡线 + 直角转弯控制

#include "stm32f4xx.h"
#include "board.h"
#include "track.h"
#include "bno080.h"
#include "tb6612.h"
#include "uart3.h"
#include "uart_k230.h"
#include "grayscale.h"
#include "encoder.h"
#include "peripheral.h"
#include "key.h"
#include "oled/oled.h"
#include <string.h>

// OLED显示目标圈数，仅在圈数变化时刷新显示
static void Car_OLEDShowTargetLap(void)
{
    static uint16_t last_display_lap = 0xFFFFU;
    uint16_t display_lap = Track_Target_Laps;

    // 圈数未变化则直接返回，避免频繁刷新OLED
    if (last_display_lap == Track_Target_Laps) {
        return;
    }

    // 更新记录值并刷新OLED显示
    last_display_lap = Track_Target_Laps;
    OLED_Clear();
    OLED_ShowNum(58, 20, (u32)display_lap, 1, 24, 1);
    OLED_Refresh();
}

// 按键控制处理：启动/停止/增减目标圈数
static void Car_KeyControlTick(void)
{
    // 按键0：启动控制
    if (Key_GetState(0U) == KEY_PRESS) {
        Track_ControlStart();
    }
    // 按键1：停止控制
    if (Key_GetState(1U) == KEY_PRESS) {
        Track_ControlStop();
    }
    // 按键2：增加目标圈数
    if (Key_GetState(2U) == KEY_PRESS) {
        Track_TargetLapAdd();
    }
    // 按键3：减少目标圈数
    if (Key_GetState(3U) == KEY_PRESS) {
        Track_TargetLapSub();
    }
}

// 主函数 - 系统初始化与主循环
int main(void)
{
    // ========== 板级初始化 ==========
    // 配置向量表、SysTick定时器，为系统提供时基
    board_init();

    // ========== 串口初始化 ==========
    // 初始化串口3，用于调试信息输出
    USART3_Init(115200);
    // 初始化K230视觉模块串口通信
    K230_UART_Init(115200);

    // ========== 外设初始化 ==========
    // 扩展LED、继电器、按键、OLED初始化
		ExtLED_Init();
		Relay_Init();
    Key_Init();
    OLED_Init();
    Car_OLEDShowTargetLap();

    // ========== 巡线系统初始化 ==========
    // 统一初始化灰度传感器、编码器、电机驱动、定时器中断、陀螺仪等
    Track_Init();
    Car_OLEDShowTargetLap();

    // ========== 主循环 ==========
    // 无限循环，调度所有周期性任务
		while (1) {
        // ----- 按键与显示处理 -----
        Key_Scan();
        Car_KeyControlTick();
        Car_OLEDShowTargetLap();

        // ----- 传感器数据采集 -----
        // 灰度传感器数据采集，获取赛道灰度信息
        Grayscale_Task(&gray_sensor);

        // ----- 陀螺仪数据处理 -----
        // 陀螺仪数据轮询消费，处理BNO080传感器数据
				Track_Main_Gyro();

        // ----- 1ms周期任务 -----
        // 灰度值转换、陀螺仪数据更新、直角转弯检测、PID控制计算、动作执行
        Track_Main_1ms();

        // ----- 5ms周期任务 -----
        // 电机速度环控制，更新电机输出
        Track_Main_5ms();

        // ----- 调试输出 -----
        // 串口调试输出，用于查看传感器数值
        Track_Main_Debug(100);

        // ----- 系统状态指示 -----
        // IMU有数据时点亮LED/关闭继电器，无数据时熄灭LED/吸合继电器
        if (gyro_yaw_available) 
        {
            ExtLED_On();
            Relay_Off();
        }
        else
        {
            ExtLED_Off();
            Relay_On();
        }					
    }
}
