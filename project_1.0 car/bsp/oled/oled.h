// SSD1306 OLED显示屏驱动头文件，PB10=SCL, PB11=SDA, I2C地址0x78, 分辨率128x64
// 支持6x8/6x12/8x16/12x24英文字体, 16/24/32/64中文字体

#ifndef __OLED_H__
#define __OLED_H__

#include "stm32f4xx.h"
#include "stdlib.h"
#include "Encoder.h"

// 类型别名
#ifndef u8
#define u8 uint8_t
#endif
#ifndef u16
#define u16 uint16_t
#endif
#ifndef u32
#define u32 uint32_t
#endif

// I2C SCL引脚（PB10）置低
#define OLED_SCL_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_10)
// I2C SCL引脚（PB10）置高
#define OLED_SCL_Set() GPIO_SetBits(GPIOB,GPIO_Pin_10)
// I2C SDA引脚（PB11）置低
#define OLED_SDA_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_11)
// I2C SDA引脚（PB11）置高
#define OLED_SDA_Set() GPIO_SetBits(GPIOB,GPIO_Pin_11)

// 写入类型：命令
#define OLED_CMD  0
// 写入类型：数据
#define OLED_DATA 1

// 函数声明

// 熄灭(x,y)位置的像素
void OLED_ClearPoint(u8 x,u8 y);
// 显示颜色反转，i=0正常显示，i=1反色显示
void OLED_ColorTurn(u8 i);
// 屏幕旋转180度，i=0正常，i=1旋转
void OLED_DisplayTurn(u8 i);
// I2C起始信号
void OLED_I2C_Start(void);
// I2C停止信号
void OLED_I2C_Stop(void);
// I2C等待应答
void OLED_I2C_WaitAck(void);
// I2C发送一个字节，高位先行
void OLED_Send_Byte(u8 dat);
// 向OLED写入一个字节，dat为数据，mode为OLED_CMD/OLED_DATA
void OLED_WR_Byte(u8 dat,u8 mode);
// 开启OLED显示
void OLED_DisPlay_On(void);
// 关闭OLED显示
void OLED_DisPlay_Off(void);
// 刷新显存到OLED屏幕
void OLED_Refresh(void);
// 清屏
void OLED_Clear(void);
// 画点，x/y为坐标，t=1点亮/t=0熄灭
void OLED_DrawPoint(u8 x,u8 y,u8 t);
// 画线（Bresenham算法），x1/y1起点，x2/y2终点，mode=1点亮/mode=0熄灭
void OLED_DrawLine(u8 x1,u8 y1,u8 x2,u8 y2,u8 mode);
// 画圆（中点算法），x/y为圆心，r为半径
void OLED_DrawCircle(u8 x,u8 y,u8 r);
// 显示单个字符，x/y为坐标，chr为ASCII字符，size1为字体大小，mode=1正常/mode=0反色
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size1,u8 mode);
// 显示6x8字符，轻量化版本
void OLED_ShowChar6x8(u8 x,u8 y,u8 chr,u8 mode);
// 显示ASCII字符串，chr为字符串指针，size1为字体大小，mode=1正常/mode=0反色
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 size1,u8 mode);
// 显示数字，num为数值，len为位数，size1为字体大小，mode=1正常/mode=0反色
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size1,u8 mode);
// 显示中文，num为字库索引，size1为字体大小(16/24/32/64)，mode=1正常/mode=0反色
void OLED_ShowChinese(u8 x,u8 y,u8 num,u8 size1,u8 mode);
// 中文滚动显示，num为字符个数，space为间隔，mode=1正常/mode=0反色
void OLED_ScrollDisplay(u8 num,u8 space,u8 mode);
// 显示位图，x/y为坐标，sizex/sizey为尺寸，BMP为位图数据，mode=1正常/mode=0反色
void OLED_ShowPicture(u8 x,u8 y,u8 sizex,u8 sizey,u8 BMP[],u8 mode);
// OLED初始化，配置GPIO和SSD1306初始化序列
void OLED_Init(void);
// I2C通信延时，约13us
void OLED_IIC_delay(void);

#endif