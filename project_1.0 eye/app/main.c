// Eye项目主程序 - 云台视觉追踪控制
// 调度周期: 100Hz (10ms)

#include "board.h"
#include "stm32f4xx.h"
#include "uart3.h"
#include "uart5.h"
#include "peripheral.h"
#include "stepper.h"
#include "vision.h"
#include "bno080.h"
#include "key.h"

// 串口波特率定义
#define EYE_DEBUG_BAUD   115200U
#define EYE_UART5_BAUD   115200U
#define EYE_STEPPER_BAUD 115200U

// 等待BNO080 IMU首帧数据就绪
// 返回值: 1-初始化成功
static uint8_t Eye_WaitForIMUData(void)
{
    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    while (1) {
        BNO080_I2C_Init();
        if (bno080_init()) {
            while (1) {
                if (bno080_update() && bno080_data_available()) {
                    bno080_get_euler(&roll_deg, &pitch_deg, &yaw_deg);
                    USART3_SendString("[EYE] BNO080 first frame ready\r\n");
                    return 1U;
                }
                delay_ms(1U);
            }
        }
        USART3_SendString("[EYE] waiting BNO080 first frame...\r\n");
        delay_ms(100U);
    }
}

// 主函数入口
// 初始化顺序: 板级 → 调试串口 → 外设 → 电机 → 传感器 → 视觉
// 主循环频率: 100Hz
int main(void)
{
    uint8_t imu_ready;
    uint32_t last_imu_ms = 0U;
    uint32_t last_vision_ms = 0U;

    // 第一阶段: 板级初始化
    board_init();

    // 第二阶段: 调试串口初始化
    USART3_Init(EYE_DEBUG_BAUD);

    // 第三阶段: 外设初始化 (LED + 按键)
    ExtLED_Init();
    ExtLED_Off();
    Key_Init();

    // 第四阶段: 步进电机初始化 (云台XOY和YOZ双轴)
    StepperXOY_Init(EYE_STEPPER_BAUD);
    StepperYOZ_Init(EYE_STEPPER_BAUD);

    // 使能云台电机: X轴和Y轴都使能
    StepperXOY_Enable(GIMBAL_ADDR_X, 1U, 0U);
    StepperYOZ_Enable(GIMBAL_ADDR_Y, 1U, 0U);

    // 第五阶段: UART5与K230通信初始化
    UART5_Init(EYE_UART5_BAUD);

    // 第六阶段: BNO080 IMU初始化 (I2C1)
    imu_ready = Eye_WaitForIMUData();

    // 第七阶段: 视觉系统初始化
    Vision_Init();

    // 主循环: 100Hz视觉追踪调度
    while (1) {
        uint32_t now_ms = HAL_GetTick();

        // IMU前馈补偿任务: 按固定周期更新IMU数据补偿云台漂移
        if ((uint32_t)(now_ms - last_imu_ms) >= VISION_IMU_FEEDFORWARD_PERIOD_MS) {
            last_imu_ms = now_ms;
            Vision_GimbalIMUCompensationTick();
        }

        (void)imu_ready;

        // 视觉控制周期判断: 未到时间则跳过
        if ((uint32_t)(now_ms - last_vision_ms) < VISION_CONTROL_PERIOD_MS) {
            continue;
        }
        last_vision_ms = now_ms;

        // 按键扫描任务: 检测模式切换按键
        Key_Scan();

        // 按键控制任务: 处理视觉模式切换
        Vision_KeyControlTick();

        // 视觉主处理任务: 解析K230数据 + PID计算 + 云台控制
        Vision_Process();

        // 目标检测状态指示: LED亮=检测到目标, LED灭=未检测到
        if (Vision_GetContext()->target_detected) {
            ExtLED_On();
        } else {
            ExtLED_Off();
        }
    }
}
