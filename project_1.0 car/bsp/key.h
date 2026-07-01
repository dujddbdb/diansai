#ifndef __KEY_H__
#define __KEY_H__

#include "stm32f4xx.h"

// 按键总数（系统支持的最大按键数量）
#define KEY_COUNT      8
// 消抖阈值（连续检测到N次相同状态才确认按键按下，用于机械按键防抖）
#define KEY_DEBOUNCE   10

// 按键状态：刚按下（单次触发，读取后自动清除）
#define KEY_PRESS      1
// 按键状态：持续按住（按键保持按下状态）
#define KEY_HOLD       2
// 按键状态：刚释放（按键刚松开，读取后自动清除）
#define KEY_RELEASE    3

// 按键GPIO定义结构体
// 用于描述单个按键的硬件连接信息
typedef struct {
    GPIO_TypeDef *port;  // GPIO端口指针（如GPIOA、GPIOB等）
    uint16_t     pin;    // GPIO引脚编号（如GPIO_Pin_0、GPIO_Pin_1等）
} KeyDef_t;

// 按键初始化
// 功能：配置所有按键对应的GPIO为输入模式并启用内部上拉电阻，清零按键状态变量
// 参数：无
// 返回值：无
void    Key_Init(void);

// 按键扫描
// 功能：周期扫描所有按键的电平状态，进行消抖处理并更新按键状态
// 参数：无
// 返回值：无
// 注意：需周期调用，建议调用间隔约1ms
void    Key_Scan(void);

// 获取指定按键状态
// 功能：读取指定索引按键的当前状态
// 参数：idx - 按键索引（取值范围0~7）
// 返回值：按键状态值
//         0 - 空闲状态（按键未按下）
//         1 - 刚按下（KEY_PRESS，单次触发）
//         2 - 持续按住（KEY_HOLD）
//         3 - 刚释放（KEY_RELEASE，单次触发）
uint8_t Key_GetState(uint8_t idx);

// 获取第一个刚按下的按键索引（消费型）
// 功能：查找并返回第一个检测到刚按下事件的按键索引，读取后该事件自动清除
// 参数：无
// 返回值：按键索引（0~7），无按键按下时返回0xFF
uint8_t Key_GetTriggered(void);

// 检测是否有任意按键刚按下
// 功能：快速检测是否存在任意按键的刚按下事件
// 参数：无
// 返回值：检测结果
//         1 - 有按键刚按下
//         0 - 无按键刚按下
uint8_t Key_AnyPressed(void);

#endif
