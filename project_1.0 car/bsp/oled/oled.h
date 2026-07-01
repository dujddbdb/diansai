// SSD1306 OLED显示屏驱动头文件
// 硬件连接：PB10=SCL，PB11=SDA
// I2C地址：0x78
// 分辨率：128x64
// 支持字体：6x8/6x12/8x16/12x24英文字体，16/24/32/64中文字体

#ifndef __OLED_H__
#define __OLED_H__

#include "stm32f4xx.h"
#include "stdlib.h"
#include "Encoder.h"

// 8位无符号整数类型别名
#ifndef u8
#define u8 uint8_t
#endif
// 16位无符号整数类型别名
#ifndef u16
#define u16 uint16_t
#endif
// 32位无符号整数类型别名
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

// OLED写入类型：命令
#define OLED_CMD  0
// OLED写入类型：数据
#define OLED_DATA 1

// 熄灭指定位置的像素点
// 功能：在显存中将指定坐标的像素点清除（熄灭）
// 参数：x - 像素点X坐标（0~127）
//       y - 像素点Y坐标（0~63）
// 返回值：无
void OLED_ClearPoint(u8 x,u8 y);

// 显示颜色反转
// 功能：设置OLED显示屏是否反色显示
// 参数：i - 显示模式（0=正常显示，1=反色显示）
// 返回值：无
void OLED_ColorTurn(u8 i);

// 屏幕旋转180度
// 功能：设置OLED显示屏是否旋转180度显示
// 参数：i - 旋转模式（0=正常方向，1=旋转180度）
// 返回值：无
void OLED_DisplayTurn(u8 i);

// I2C起始信号
// 功能：产生I2C通信的起始信号（SCL高电平时SDA由高变低）
// 参数：无
// 返回值：无
void OLED_I2C_Start(void);

// I2C停止信号
// 功能：产生I2C通信的停止信号（SCL高电平时SDA由低变高）
// 参数：无
// 返回值：无
void OLED_I2C_Stop(void);

// I2C等待应答
// 功能：等待从设备发送的应答信号
// 参数：无
// 返回值：无
void OLED_I2C_WaitAck(void);

// I2C发送一个字节
// 功能：通过I2C总线发送一个字节的数据，高位先行
// 参数：dat - 待发送的字节数据
// 返回值：无
void OLED_Send_Byte(u8 dat);

// 向OLED写入一个字节
// 功能：向OLED控制器写入命令或数据
// 参数：dat - 待写入的字节数据
//       mode - 写入类型（OLED_CMD=命令，OLED_DATA=数据）
// 返回值：无
void OLED_WR_Byte(u8 dat,u8 mode);

// 开启OLED显示
// 功能：发送命令开启OLED显示，屏幕从休眠状态唤醒
// 参数：无
// 返回值：无
void OLED_DisPlay_On(void);

// 关闭OLED显示
// 功能：发送命令关闭OLED显示，屏幕进入休眠状态（省电）
// 参数：无
// 返回值：无
void OLED_DisPlay_Off(void);

// 刷新显存到OLED屏幕
// 功能：将本地显存中的数据全部刷新到OLED屏幕上显示
// 参数：无
// 返回值：无
void OLED_Refresh(void);

// 清屏
// 功能：清空本地显存中的所有像素数据（全部熄灭）
// 参数：无
// 返回值：无
void OLED_Clear(void);

// 画点
// 功能：在显存中指定坐标位置绘制或清除一个像素点
// 参数：x - 像素点X坐标（0~127）
//       y - 像素点Y坐标（0~63）
//       t - 绘制模式（1=点亮，0=熄灭）
// 返回值：无
void OLED_DrawPoint(u8 x,u8 y,u8 t);

// 画线（Bresenham算法）
// 功能：在显存中使用Bresenham算法绘制一条直线
// 参数：x1 - 起点X坐标
//       y1 - 起点Y坐标
//       x2 - 终点X坐标
//       y2 - 终点Y坐标
//       mode - 绘制模式（1=点亮，0=熄灭）
// 返回值：无
void OLED_DrawLine(u8 x1,u8 y1,u8 x2,u8 y2,u8 mode);

// 画圆（中点算法）
// 功能：在显存中使用中点圆算法绘制一个圆形轮廓
// 参数：x - 圆心X坐标
//       y - 圆心Y坐标
//       r - 圆的半径（单位：像素）
// 返回值：无
void OLED_DrawCircle(u8 x,u8 y,u8 r);

// 显示单个字符
// 功能：在指定位置显示一个ASCII字符
// 参数：x - 字符起始X坐标
//       y - 字符起始Y坐标
//       chr - 待显示的ASCII字符
//       size1 - 字体大小（可选值：6x8, 6x12, 8x16, 12x24等）
//       mode - 显示模式（1=正常显示，0=反色显示）
// 返回值：无
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size1,u8 mode);

// 显示6x8字符（轻量化版本）
// 功能：在指定位置显示一个6x8点阵的ASCII字符（占用资源更少）
// 参数：x - 字符起始X坐标
//       y - 字符起始Y坐标
//       chr - 待显示的ASCII字符
//       mode - 显示模式（1=正常显示，0=反色显示）
// 返回值：无
void OLED_ShowChar6x8(u8 x,u8 y,u8 chr,u8 mode);

// 显示ASCII字符串
// 功能：在指定位置显示一个以'\0'结尾的ASCII字符串
// 参数：x - 字符串起始X坐标
//       y - 字符串起始Y坐标
//       chr - 指向待显示字符串的指针
//       size1 - 字体大小
//       mode - 显示模式（1=正常显示，0=反色显示）
// 返回值：无
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 size1,u8 mode);

// 显示数字
// 功能：在指定位置显示一个数值型数字，按指定位数显示
// 参数：x - 数字起始X坐标
//       y - 数字起始Y坐标
//       num - 待显示的数值（32位无符号整数）
//       len - 显示位数（不足则高位补零）
//       size1 - 字体大小
//       mode - 显示模式（1=正常显示，0=反色显示）
// 返回值：无
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size1,u8 mode);

// 显示中文
// 功能：在指定位置显示一个中文汉字（从字库中按索引查找）
// 参数：x - 中文起始X坐标
//       y - 中文起始Y坐标
//       num - 中文字库索引编号
//       size1 - 字体大小（可选值：16/24/32/64）
//       mode - 显示模式（1=正常显示，0=反色显示）
// 返回值：无
void OLED_ShowChinese(u8 x,u8 y,u8 num,u8 size1,u8 mode);

// 中文滚动显示
// 功能：实现中文内容的滚动显示效果
// 参数：num - 滚动显示的字符个数
//       space - 字符之间的间隔（单位：像素）
//       mode - 显示模式（1=正常显示，0=反色显示）
// 返回值：无
void OLED_ScrollDisplay(u8 num,u8 space,u8 mode);

// 显示位图
// 功能：在指定位置显示一幅位图图片
// 参数：x - 位图起始X坐标
//       y - 位图起始Y坐标
//       sizex - 位图宽度（单位：像素）
//       sizey - 位图高度（单位：像素）
//       BMP - 指向位图数据数组的指针
//       mode - 显示模式（1=正常显示，0=反色显示）
// 返回值：无
void OLED_ShowPicture(u8 x,u8 y,u8 sizex,u8 sizey,u8 BMP[],u8 mode);

// OLED初始化
// 功能：配置I2C通信的GPIO引脚，并发送SSD1306初始化序列
// 参数：无
// 返回值：无
void OLED_Init(void);

// I2C通信延时
// 功能：软件I2C通信时的延时函数，用于控制I2C时序（约13微秒）
// 参数：无
// 返回值：无
void OLED_IIC_delay(void);

#endif
