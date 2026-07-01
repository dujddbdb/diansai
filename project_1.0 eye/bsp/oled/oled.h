#ifndef __OLED_H
#define __OLED_H

#include "stm32f4xx.h"
#include "stdlib.h"
#include "Encoder.h"

// 类型别名 (兼容不同代码风格)
// u8 类型别名, 定义为 uint8_t
#ifndef u8
#define u8 uint8_t
#endif
// u16 类型别名, 定义为 uint16_t
#ifndef u16
#define u16 uint16_t
#endif
// u32 类型别名, 定义为 uint32_t
#ifndef u32
#define u32 uint32_t
#endif

// ========================== OLED硬件引脚定义 ==========================

// IIC SCL引脚（PB10）置低——模拟I2C时钟线拉低
#define OLED_SCL_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_10)
// IIC SCL引脚（PB10）置高——模拟I2C时钟线拉高
#define OLED_SCL_Set() GPIO_SetBits(GPIOB,GPIO_Pin_10)

// IIC SDA(DIN)引脚（PB11）置低——模拟I2C数据线拉低
#define OLED_SDA_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_11)
// IIC SDA(DIN)引脚（PB11）置高——模拟I2C数据线拉高
#define OLED_SDA_Set() GPIO_SetBits(GPIOB,GPIO_Pin_11)

// 复位引脚定义（注释掉，根据硬件可选）
//#define OLED_RES_Clr() GPIO_ResetBits(GPIOD,GPIO_Pin_4) // 复位引脚置低（已注释，未使用）
//#define OLED_RES_Set() GPIO_SetBits(GPIOD,GPIO_Pin_4)   // 复位引脚置高（已注释，未使用）

// ========================== 写入类型定义 ==========================

// OLED写入类型：命令/指令（mode=0→发送0x00控制字节）
#define OLED_CMD  0
// OLED写入类型：数据（mode=1→发送0x40数据字节）
#define OLED_DATA 1

// ========================== 函数声明 ==========================

// 清除OLED指定坐标的像素点
// 功能: 将指定坐标位置的像素点清除(熄灭)
// 参数: x - 横坐标(0~127)
//       y - 纵坐标(0~63)
// 返回值: 无
void OLED_ClearPoint(u8 x,u8 y);

// 显示颜色反转控制
// 功能: 设置OLED显示是否反色
// 参数: i - 0=正常显示, 1=反色显示
// 返回值: 无
void OLED_ColorTurn(u8 i);

// 屏幕旋转180度控制
// 功能: 设置OLED屏幕显示方向
// 参数: i - 0=正常方向, 1=旋转180度
// 返回值: 无
void OLED_DisplayTurn(u8 i);

// IIC通信起始信号
// 功能: 产生I2C通信起始信号 (SCL高电平时SDA从高变低)
// 参数: 无
// 返回值: 无
void OLED_I2C_Start(void);

// IIC通信停止信号
// 功能: 产生I2C通信停止信号 (SCL高电平时SDA从低变高)
// 参数: 无
// 返回值: 无
void OLED_I2C_Stop(void);

// IIC等待应答信号
// 功能: 等待I2C从设备的应答信号
// 参数: 无
// 返回值: 无
void OLED_I2C_WaitAck(void);

// IIC发送一个字节
// 功能: 通过I2C发送一个字节数据, 高位先行
// 参数: dat - 要发送的字节数据
// 返回值: 无
void OLED_Send_Byte(u8 dat);

// 向OLED写入一个字节
// 功能: 向OLED写入命令或数据
// 参数: dat - 要写入的字节数据
//       mode - 0=写入命令, 1=写入数据
// 返回值: 无
void OLED_WR_Byte(u8 dat,u8 mode);

// 开启OLED显示
// 功能: 开启OLED显示, 包括电荷泵和屏幕显示
// 参数: 无
// 返回值: 无
void OLED_DisPlay_On(void);

// 关闭OLED显示
// 功能: 关闭OLED显示, 包括电荷泵和屏幕显示
// 参数: 无
// 返回值: 无
void OLED_DisPlay_Off(void);

// 刷新显存到OLED屏幕
// 功能: 将本地显存OLED_GRAM中的数据逐页写入OLED屏幕
// 参数: 无
// 返回值: 无
void OLED_Refresh(void);

// OLED清屏
// 功能: 清空本地显存并刷新到屏幕, 屏幕全黑
// 参数: 无
// 返回值: 无
void OLED_Clear(void);

// OLED画点
// 功能: 在指定坐标绘制一个像素点
// 参数: x - 横坐标(0~127)
//       y - 纵坐标(0~63)
//       t - 1=点亮像素, 0=熄灭像素
// 返回值: 无
void OLED_DrawPoint(u8 x,u8 y,u8 t);

// 画线 (Bresenham算法)
// 功能: 使用Bresenham算法在两点之间画一条直线
// 参数: x1,y1 - 起点坐标
//       x2,y2 - 终点坐标
//       mode - 1=点亮线条, 0=熄灭线条
// 返回值: 无
void OLED_DrawLine(u8 x1,u8 y1,u8 x2,u8 y2,u8 mode);

// 画圆 (中点算法)
// 功能: 使用中点圆算法绘制圆形
// 参数: x,y - 圆心坐标
//       r - 圆的半径(像素)
// 返回值: 无
void OLED_DrawCircle(u8 x,u8 y,u8 r);

// 显示单个字符
// 功能: 在指定位置显示一个ASCII字符
// 参数: x,y - 字符左上角坐标
//       chr - 要显示的ASCII字符
//       size1 - 字体大小(6x8/6x12/8x16/12x24)
//       mode - 0=反色显示, 1=正常显示
// 返回值: 无
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size1,u8 mode);

// 显示6x8规格字符
// 功能: 在指定位置显示一个6x8像素的ASCII字符
// 参数: x,y - 字符左上角坐标
//       chr - 要显示的ASCII字符
//       mode - 0=反色显示, 1=正常显示
// 返回值: 无
void OLED_ShowChar6x8(u8 x,u8 y,u8 chr,u8 mode);

// 显示ASCII字符串
// 功能: 在指定位置显示一个ASCII字符串
// 参数: x,y - 字符串左上角坐标
//       chr - 指向字符串的指针
//       size1 - 字体大小
//       mode - 0=反色显示, 1=正常显示
// 返回值: 无
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 size1,u8 mode);

// 显示数字
// 功能: 在指定位置显示一个数字
// 参数: x,y - 数字左上角坐标
//       num - 要显示的数字值
//       len - 数字位数
//       size1 - 字体大小
//       mode - 0=反色显示, 1=正常显示
// 返回值: 无
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size1,u8 mode);

// 显示中文字符
// 功能: 在指定位置显示一个中文字符(需字库支持)
// 参数: x,y - 字符左上角坐标
//       num - 中文字在字库中的索引编号
//       size1 - 字体大小(16/24/32/64)
//       mode - 0=反色显示, 1=正常显示
// 返回值: 无
void OLED_ShowChinese(u8 x,u8 y,u8 num,u8 size1,u8 mode);

// 中文滚动显示
// 功能: 实现中文字符的滚动显示效果
// 参数: num - 字符个数
//       space - 字符间隔(像素)
//       mode - 0=反色显示, 1=正常显示
// 返回值: 无
void OLED_ScrollDisplay(u8 num,u8 space,u8 mode);

// 显示位图
// 功能: 在指定位置显示一幅位图图片
// 参数: x,y - 图片左上角坐标
//       sizex,sizey - 图片宽度和高度(像素)
//       BMP[] - 位图数据数组
//       mode - 0=反色显示, 1=正常显示
// 返回值: 无
void OLED_ShowPicture(u8 x,u8 y,u8 sizex,u8 sizey,u8 BMP[],u8 mode);

// OLED初始化
// 功能: 初始化OLED, 配置GPIO引脚、模拟IIC、写入初始化指令序列
// 参数: 无
// 返回值: 无
void OLED_Init(void);

// IIC通信延时
// 功能: 软件延时, 保证I2C时序稳定, 约13微秒
// 参数: 无
// 返回值: 无
void OLED_IIC_delay(void);

#endif
