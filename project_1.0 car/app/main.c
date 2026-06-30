/**
 * @file    main.c
 * @brief   主程序入口 - 智能车巡线 + 直角转弯控制 (主循环架构)
 * @note    TIM11 1kHz: 仅设置 flag_1ms/flag_5ms + system_time_ms
 *          主循环:   灰度采集 → 调度函数 → 串口调试
 */

/**
 * @brief  包含 STM32F4 标准外设库头文件，提供芯片外设寄存器定义和驱动接口
 * @note   stm32f4xx.h是STM32F4系列芯片的标准外设库头文件
 * @note   该文件包含:
 *         - 寄存器定义: 各种外设寄存器的地址映射
 *         - 类型定义: GPIO_InitTypeDef, USART_InitTypeDef等
 *         - 宏定义: RCC_AHB1Periph_GPIOD, GPIO_Pin_8等
 *         - 库函数: RCC_AHB1PeriphClockCmd(), GPIO_Init()等
 * @note   所有STM32外设驱动代码都必须包含此文件
 */
#include "stm32f4xx.h"

/**
 * @brief  包含板级初始化头文件，包含系统时钟、GPIO等基础配置
 * @note   board.h通常包含:
 *         - board_init(): 板级初始化函数，配置向量表和SysTick定时器
 *         - SystemInit(): 系统时钟初始化函数
 *         - delay_ms(): 毫秒级延时函数
 *         - delay_us(): 微秒级延时函数
 * @note   必须在使用任何外设之前调用board_init()
 */
#include "board.h"

/**
 * @brief  包含巡线控制头文件，定义赛道检测、直角转弯等核心算法
 * @note   track.h包含:
 *         - Track_Init(): 巡线系统初始化
 *         - Track_Main_1ms(): 1ms周期任务
 *         - Track_Main_5ms(): 5ms周期任务
 *         - Track_Main_Gyro(): 陀螺仪数据处理
 *         - Track_Main_Debug(): 调试信息输出
 *         - gray_sensor: 灰度传感器数据结构
 * @note   这是智能车核心控制逻辑所在
 */
#include "track.h"

/**
 * @brief  包含 BNO080 九轴传感器头文件，用于获取陀螺仪航向角数据
 * @note   bno080.h包含:
 *         - BNO080传感器相关函数和数据结构
 *         - gyro_yaw_available: 陀螺仪数据可用标志
 *         - 获取九轴传感器的加速度、角速度、磁场等数据
 * @note   BNO080是一款九轴惯性测量单元(IMU)
 * @note   用于测量车身姿态角和航向角
 */
#include "bno080.h"

/**
 * @brief  包含 TB6612 双H桥电机驱动芯片头文件，控制左右电机正反转和速度
 * @note   tb6612.h包含:
 *         - TB6612电机驱动相关函数
 *         - 控制两个电机的正转、反转、制动
 *         - PWM速度控制接口
 * @note   TB6612是双H桥电机驱动芯片
 * @note   可同时驱动两路直流电机
 */
#include "tb6612.h"

/**
 * @brief  包含串口3头文件，用于调试信息输出
 * @note   uart3.h包含:
 *         - USART3_Init(): USART3初始化函数
 * @note   USART3配置为115200-8N1
 * @note   用于输出调试信息和传感器数据
 */
#include "uart3.h"

/**
 * @brief  包含 K230 视觉模块串口头文件，接收视觉处理结果
 * @note   uart_k230.h包含:
 *         - K230_UART_Init(): K230视觉模块串口初始化
 *         - 接收K230视觉模块的处理结果
 *         - 视觉模块可能进行图像处理和目标检测
 * @note   K230可能是Kendryte K210或类似AI视觉模块
 * @note   用于获取视觉感知结果，辅助巡线决策
 */
#include "uart_k230.h"

/**
 * @brief  包含灰度传感器头文件，用于采集赛道灰度信息
 * @note   grayscale.h包含:
 *         - Grayscale_Task(): 灰度传感器采集任务
 *         - gray_sensor: 灰度传感器数据结构
 *         - 包含多个灰度传感器的数据
 * @note   灰度传感器用于检测赛道线的位置
 * @note   通常有多个传感器并排布置
 */
#include "grayscale.h"

/**
 * @brief  包含编码器头文件，用于获取电机转速反馈
 * @note   encoder.h包含:
 *         - 编码器相关函数和数据结构
 *         - 获取电机转速反馈
 *         - 用于闭环速度控制
 * @note   编码器通常连接到电机轴
 * @note   通过检测编码器脉冲数计算转速
 */
#include "encoder.h"

/**
 * @brief  包含外设初始化头文件，初始化LED等扩展外设
 * @note   peripheral.h包含:
 *         - ExtLED_Init(): 扩展LED初始化
 *         - ExtLED_On(): LED亮
 *         - ExtLED_Off(): LED灭
 *         - 其他扩展外设初始化函数
 * @note   LED用于系统状态指示
 * @note   方便调试和观察系统运行状态
 */
#include "peripheral.h"
#include "key.h"
#include "oled/oled.h"

/**
 * @brief  包含字符串处理库，提供 memset 等字符串操作函数
 * @note   string.h提供:
 *         - memset(): 内存设置函数
 *         - memcpy(): 内存拷贝函数
 *         - strlen(): 字符串长度函数
 *         - strcmp(): 字符串比较函数
 * @note   本文件暂未直接使用
 * @note   为扩展功能预留
 */
#include <string.h>

static void Car_OLEDShowTargetLap(void)
{
    static uint16_t last_display_lap = 0xFFFFU;
    uint16_t display_lap = Track_Target_Laps;

    if (last_display_lap == Track_Target_Laps) {
        return;
    }

    last_display_lap = Track_Target_Laps;
    OLED_Clear();
    OLED_ShowNum(58, 20, (u32)display_lap, 1, 24, 1);
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

/**
 * @brief  主函数 - 系统初始化与主循环
 * @note   初始化后进入主循环，调度所有模块任务(100Hz)
 * @retval 无返回值，程序永不返回
 * @note   main函数是ARM Cortex-M处理器的入口点
 * @note   复位后首先执行此函数
 */
int main(void)
{
    /**
     * @brief  板级初始化: 配置向量表和SysTick定时器，为系统提供时基
     * @note   board_init()完成以下工作:
     *         - 配置NVIC向量表
     *         - 初始化SysTick定时器(1kHz，1ms中断)
     *         - 配置系统时钟
     * @note   必须第一个调用，为整个系统提供时基基准
     * @note   flag_1ms和flag_5ms由定时器中断设置
     */
    board_init();

    /**
     * @brief  初始化串口3，波特率115200，用于调试信息输出
     * @note   USART3_Init(115200)完成:
     *         - 使能GPIOD时钟
     *         - 使能USART3时钟
     *         - 配置PD8=USART3_TX, PD9=USART3_RX
     *         - 配置115200-8N1-无流控-收发模式
     * @note   USART3用于输出调试信息到PC
     */
    USART3_Init(115200);

    /**
     * @brief  初始化K230视觉模块串口通信，波特率115200
     * @note   K230_UART_Init(115200)完成:
     *         - 初始化UART5用于K230通信
     *         - 配置PC12=UART5_TX, PD2=UART5_RX
     *         - 配置115200-8N1-无流控-收发模式
     * @note   K230视觉模块可能进行目标检测
     */
    K230_UART_Init(115200);

    /**
     * @brief  初始化扩展LED，用于指示系统状态
     * @note   ExtLED_Init()完成:
     *         - 配置LED相关GPIO引脚
     *         - 初始化LED为熄灭状态
     * @note   LED可用于指示:
     *         - 系统运行状态
     *         - 传感器数据状态
     *         - 故障报警
     */
		ExtLED_Init();
		Relay_Init();
    Key_Init();
    OLED_Init();
    Car_OLEDShowTargetLap();

    /**
     * @brief  统一初始化: 灰度传感器、编码器、电机驱动、定时器中断、等待陀螺仪数据准备全部在Track_Init函数中完成
     * @note   Track_Init()完成所有传感器和外设的初始化:
     *         - 灰度传感器初始化
     *         - 编码器初始化
     *         - TB6612电机驱动初始化
     *         - TIM11定时器中断初始化(1kHz)
     *         - BNO080陀螺仪初始化和数据准备
     * @note   这是智能车系统的核心初始化函数
     * @note   初始化完成后系统即可运行
     */
    Track_Init();
    Car_OLEDShowTargetLap();

    /**
     * @brief  主循环 - 无限循环，调度所有周期性任务
     * @note   while(1)是死循环，程序永不退出
     * @note   主循环频率约100Hz (10ms周期)
     * @note   每次循环依次执行各个任务模块
     */
		while (1) {
        Key_Scan();
        Car_KeyControlTick();
        Car_OLEDShowTargetLap();

        /**
         * @brief  灰度传感器数据采集任务，获取赛道灰度信息
         * @note   Grayscale_Task()完成:
         *         - 读取灰度传感器ADC值
         *         - 判断赛道线位置
         *         - 更新gray_sensor数据结构
         * @note   这是巡线控制的基础输入
         * @note   必须在其他任务之前执行
         */
        Grayscale_Task(&gray_sensor);

        /**
         * @brief  陀螺仪数据轮询消费任务，处理BNO080传感器数据
         * @note   Track_Main_Gyro()完成:
         *         - 检查陀螺仪数据就绪标志
         *         - 读取航向角(yaw)等数据
         *         - 更新系统状态
         * @note   用于获取车身当前朝向
         * @note   对直角转弯检测有重要作用
         */
				Track_Main_Gyro();

        /**
         * @brief  1毫秒周期任务：灰度值转换、陀螺仪数据更新、直角转弯检测、PID控制计算、动作执行
         * @note   Track_Main_1ms()完成:
         *         - 灰度传感器数据处理
         *         - 陀螺仪数据更新
         *         - 直角转弯检测算法
         *         - PID控制计算
         *         - 电机动作执行
         * @note   这是巡线控制的核心函数
         * @note   由定时器标志触发执行
         */
        Track_Main_1ms();

        /**
         * @brief  5毫秒周期任务：电机速度环控制，更新电机输出
         * @note   Track_Main_5ms()完成:
         *         - 速度环PID控制
         *         - 电机转速反馈
         *         - 更新TB6612电机输出
         * @note   速度环确保电机转速稳定
         * @note   提高巡线平稳性
         */
        Track_Main_5ms();

        /**
         * @brief  200毫秒串口调试输出函数调用(已注释)，用于查看传感器数值
         * @note   Track_Main_Debug(200)完成:
         *         - 通过USART3输出调试信息
         *         - 打印灰度传感器值
         *         - 打印陀螺仪数据
         *         - 打印电机转速等
         * @note   已注释掉，不输出调试信息
         * @note   需要调试时可取消注释
         */
        Track_Main_Debug(100);

        /**
         * @brief  IMU有数据时点亮LED/关闭继电器，无数据时熄灭LED/吸合继电器
         */
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
