
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

/**************************** 用户可配置区域 ***************************/
#define KEY_COUNT      8       // 按键总数: 8个独立按键 BTN0~BTN7
#define KEY_DEBOUNCE   10      // 消抖阈值: 连续10次扫描检测到按下则确认有效 (约10ms，取决于主循环周期)

/**************************** 按键状态值 ********************************/
#define KEY_PRESS      1       // 按键状态: 刚按下 (单次触发，下一扫描周期自动变为KEY_HOLD)
#define KEY_HOLD       2       // 按键状态: 持续按住 (消抖计数已达阈值且按键仍按下)
#define KEY_RELEASE    3       // 按键状态: 刚释放 (按键由按下变为释放的瞬间，下一周期回到0/idle)

typedef struct {
    GPIO_TypeDef *port;        // GPIO端口指针，指向按键所在的端口寄存器基地址 (GPIOA/GPIOB/GPIOC)
    uint16_t     pin;          // GPIO引脚编号，指定按键对应的具体引脚 (GPIO_Pin_0~GPIO_Pin_7)
} KeyDef_t;

void    Key_Init(void);                                                       // 初始化所有按键的GPIO引脚(输入模式+内部上拉)并清零状态缓冲区和消抖计数器
void    Key_Scan(void);                                                       // 按键扫描函数，每个主循环周期调用一次，执行消抖计数和状态机转换
uint8_t Key_GetState(uint8_t idx);                                            // 获取指定按键的当前状态值，idx为按键索引(0~7)，返回状态值或0
uint8_t Key_GetTriggered(void);                                               // 获取第一个触发(刚按下)的按键索引并消费，无触发时返回0xFF
uint8_t Key_AnyPressed(void);                                                 // 检测是否有任意按键刚被按下，有则返回1，无则返回0

#endif
