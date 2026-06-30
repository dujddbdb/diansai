#ifndef __OLED_H
#define __OLED_H

#include "stm32f4xx.h"
#include "stdlib.h"
#include "Encoder.h"

// 类型别名 (兼容不同代码风格)
#ifndef u8
#define u8 uint8_t
#endif
#ifndef u16
#define u16 uint16_t
#endif
#ifndef u32
#define u32 uint32_t
#endif

/************************** OLED硬件引脚定义 **************************/
// IIC SCL引脚（PB10）置低——模拟I2C时钟线拉低
#define OLED_SCL_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_10) // GPIOB复位PB10位→SCL输出低电平
// IIC SCL引脚（PB10）置高——模拟I2C时钟线拉高
#define OLED_SCL_Set() GPIO_SetBits(GPIOB,GPIO_Pin_10)   // GPIOB置位PB10位→SCL输出高电平

// IIC SDA(DIN)引脚（PB11）置低——模拟I2C数据线拉低
#define OLED_SDA_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_11) // GPIOB复位PB11位→SDA输出低电平
// IIC SDA(DIN)引脚（PB11）置高——模拟I2C数据线拉高
#define OLED_SDA_Set() GPIO_SetBits(GPIOB,GPIO_Pin_11)   // GPIOB置位PB11位→SDA输出高电平

// 复位引脚定义（注释掉，根据硬件可选）
//#define OLED_RES_Clr() GPIO_ResetBits(GPIOD,GPIO_Pin_4) // 复位引脚置低（已注释，未使用）
//#define OLED_RES_Set() GPIO_SetBits(GPIOD,GPIO_Pin_4)   // 复位引脚置高（已注释，未使用）

/************************** 写入类型定义 **************************/
#define OLED_CMD  0   // OLED写入类型：命令/指令（mode=0→发送0x00控制字节）
#define OLED_DATA 1   // OLED写入类型：数据（mode=1→发送0x40数据字节）

/************************** 函数声明 **************************/
// 清除OLED指定坐标的像素点, x: 横坐标(0~127), y: 纵坐标(0~63)
void OLED_ClearPoint(u8 x,u8 y);

// 显示颜色反转控制, i: 0=正常 1=反色
void OLED_ColorTurn(u8 i);

// 屏幕旋转180度控制, i: 0=正常 1=旋转180度
void OLED_DisplayTurn(u8 i);

// IIC通信起始信号 (SCL高时SDA从高到低)
void OLED_I2C_Start(void);

// IIC通信停止信号 (SCL高时SDA从低到高)
void OLED_I2C_Stop(void);

// IIC等待应答信号
void OLED_I2C_WaitAck(void);

// IIC发送一个字节, dat: 要发送的字节 (高位先行)
void OLED_Send_Byte(u8 dat);

// 向OLED写入一个字节, dat: 数据, mode: 0=指令 1=数据
void OLED_WR_Byte(u8 dat,u8 mode);

// 开启OLED显示 (电荷泵+屏幕)
void OLED_DisPlay_On(void);

// 关闭OLED显示 (电荷泵+屏幕)
void OLED_DisPlay_Off(void);

// 刷新显存到OLED屏幕 (逐页写入OLED_GRAM)
void OLED_Refresh(void);

// OLED清屏 (清空显存+刷新)
void OLED_Clear(void);

// OLED画点, x: 横坐标(0~127), y: 纵坐标(0~63), t: 1=点亮 0=熄灭
void OLED_DrawPoint(u8 x,u8 y,u8 t);

// 画线 (Bresenham算法), x1,y1: 起点, x2,y2: 终点, mode: 1=点亮 0=熄灭
void OLED_DrawLine(u8 x1,u8 y1,u8 x2,u8 y2,u8 mode);

// 画圆 (中点算法), x,y: 圆心, r: 半径
void OLED_DrawCircle(u8 x,u8 y,u8 r);

// 显示单个字符, x,y: 坐标, chr: ASCII字符, size1: 字体大小(6x8/6x12/8x16/12x24), mode: 0=反色 1=正常
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size1,u8 mode);

// 显示6x8规格字符, x,y: 坐标, chr: ASCII字符, mode: 0=反色 1=正常
void OLED_ShowChar6x8(u8 x,u8 y,u8 chr,u8 mode);

// 显示ASCII字符串, x,y: 坐标, *chr: 字符串, size1: 字体大小, mode: 0=反色 1=正常
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 size1,u8 mode);

// 显示数字, x,y: 坐标, num: 数字, len: 位数, size1: 字体大小, mode: 0=反色 1=正常
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size1,u8 mode);

// 显示中文字符, x,y: 坐标, num: 字库索引, size1: 字体大小(16/24/32/64), mode: 0=反色 1=正常
void OLED_ShowChinese(u8 x,u8 y,u8 num,u8 size1,u8 mode);

// 中文滚动显示, num: 字符个数, space: 字符间隔, mode: 0=反色 1=正常
void OLED_ScrollDisplay(u8 num,u8 space,u8 mode);

// 显示位图, x,y: 坐标, sizex,sizey: 图片尺寸, BMP[]: 位图数据, mode: 0=反色 1=正常
void OLED_ShowPicture(u8 x,u8 y,u8 sizex,u8 sizey,u8 BMP[],u8 mode);

// OLED初始化 (配置GPIO+IIC+写入初始化指令序列)
void OLED_Init(void);

// IIC通信延时 (保证时序稳定, 约13us)
void OLED_IIC_delay(void);

#endif

