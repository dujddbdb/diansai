#ifndef __KEY_H__
#define __KEY_H__

#include "stm32f4xx.h"

#define KEY_COUNT      8       // 按键总数
#define KEY_DEBOUNCE   10      // 消抖阈值(连续N次确认按下)

#define KEY_PRESS      1       // 按键状态: 刚按下(单次触发)
#define KEY_HOLD       2       // 按键状态: 持续按住
#define KEY_RELEASE    3       // 按键状态: 刚释放

// 按键GPIO定义结构体
typedef struct {
    GPIO_TypeDef *port;  // GPIO端口指针
    uint16_t     pin;    // GPIO引脚编号
} KeyDef_t;

// 按键初始化(输入+内部上拉,清零状态)
void    Key_Init(void);
// 按键扫描(需周期调用,约1ms/次)
void    Key_Scan(void);
// 获取指定按键状态: idx-0~7 返回0-idle/1-PRESS/2-HOLD/3-RELEASE
uint8_t Key_GetState(uint8_t idx);
// 获取第一个刚按下的按键索引(消费型),无则返回0xFF
uint8_t Key_GetTriggered(void);
// 检测是否有任意按键刚按下: 1-有 0-无
uint8_t Key_AnyPressed(void);

#endif
