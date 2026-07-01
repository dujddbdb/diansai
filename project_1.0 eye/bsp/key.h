// @file    key.h
// @brief   8个独立按键扫描接口 (消抖状态机)
// @note    硬件: PC4/PC5/PB0/PA3/PA4/PA5/PA6/PA7, 内部上拉, 按下=低电平
//          按键映射: BTN0=PA3(继电器), BTN1=PA4(蜂鸣器), BTN2=PA5(页面切换),
//                    BTN3=PA6(备用), BTN4=PA7(外部LED)
//          消抖: 连续10次扫描均检测到按下→确认有效
//          状态机: idle → KEY_PRESS(单次触发) → KEY_HOLD(持续) → KEY_RELEASE → idle

#ifndef __KEY_H__
#define __KEY_H__

#include "stm32f4xx.h"

// ============================ 用户可配置区域 ============================

// 按键总数: 8个独立按键 BTN0~BTN7
#define KEY_COUNT      8
// 消抖阈值: 连续10次扫描检测到按下则确认有效 (约10ms，取决于主循环周期)
#define KEY_DEBOUNCE   10

// ============================ 按键状态值 ============================

// 按键状态: 刚按下 (单次触发，下一扫描周期自动变为KEY_HOLD)
#define KEY_PRESS      1
// 按键状态: 持续按住 (消抖计数已达阈值且按键仍按下)
#define KEY_HOLD       2
// 按键状态: 刚释放 (按键由按下变为释放的瞬间，下一周期回到0/idle)
#define KEY_RELEASE    3

// 按键硬件定义结构体
// 描述单个按键对应的GPIO端口和引脚信息
typedef struct {
    GPIO_TypeDef *port;        // GPIO端口指针，指向按键所在的端口寄存器基地址 (GPIOA/GPIOB/GPIOC)
    uint16_t     pin;          // GPIO引脚编号，指定按键对应的具体引脚 (GPIO_Pin_0~GPIO_Pin_7)
} KeyDef_t;

// 按键初始化函数
// 功能: 初始化所有按键的GPIO引脚(输入模式+内部上拉)并清零状态缓冲区和消抖计数器
// 参数: 无
// 返回值: 无
void Key_Init(void);

// 按键扫描函数
// 功能: 每个主循环周期调用一次，执行消抖计数和状态机转换
// 参数: 无
// 返回值: 无
void Key_Scan(void);

// 获取指定按键的当前状态值
// 功能: 获取指定按键的当前状态
// 参数: idx - 按键索引(0~7)，对应BTN0~BTN7
// 返回值: 按键状态值 (KEY_PRESS/KEY_HOLD/KEY_RELEASE) 或 0(空闲状态)
uint8_t Key_GetState(uint8_t idx);

// 获取第一个触发的按键索引
// 功能: 获取第一个触发(刚按下)的按键索引并消费该事件
// 参数: 无
// 返回值: 触发的按键索引(0~7)，无触发时返回0xFF
uint8_t Key_GetTriggered(void);

// 检测是否有任意按键刚被按下
// 功能: 检测是否有任意按键刚被按下
// 参数: 无
// 返回值: 有任意按键刚按下返回1，无则返回0
uint8_t Key_AnyPressed(void);

#endif
