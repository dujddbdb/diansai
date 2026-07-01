// SSD1306 OLED显示屏驱动: SW-I2C(PB10=SCL,PB11=SDA), I2C地址0x78, 128x64
// 支持6x8/6x12/8x16/12x24英文字体, 16/24/32/64中文字体

#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"
#include "board.h"

// OLED显存: 144列×8页, 每字节8像素
u8 OLED_GRAM[144][8];

// 颜色反转: i=0正常, i=1反色
void OLED_ColorTurn(u8 i)
{
	// 正常显示
	if(i==0)
	{
		OLED_WR_Byte(0xA6,OLED_CMD);
	}
	// 反色显示
	if(i==1)
	{
		OLED_WR_Byte(0xA7,OLED_CMD);
	}
}

// 屏幕旋转180度: i=0正常, i=1旋转
void OLED_DisplayTurn(u8 i)
{
	// 正常方向
	if(i==0)
	{
		OLED_WR_Byte(0xC8,OLED_CMD);
		OLED_WR_Byte(0xA1,OLED_CMD);
	}
	// 旋转180度
	if(i==1)
	{
		OLED_WR_Byte(0xC0,OLED_CMD);
		OLED_WR_Byte(0xA0,OLED_CMD);
	}
}

// I2C延时约13us
void OLED_IIC_delay(void)
{
    delay_us(13);
}

// I2C起始信号: SCL=1时SDA从1→0
void OLED_I2C_Start(void)
{
	// SDA置高
	OLED_SDA_Set();
	// SCL置高
	OLED_SCL_Set();
	OLED_IIC_delay();
	// SDA拉低，产生起始信号
	OLED_SDA_Clr();
	OLED_IIC_delay();
	// SCL拉低，准备发送数据
	OLED_SCL_Clr();
	OLED_IIC_delay();
}

// I2C停止信号: SCL=1时SDA从0→1
void OLED_I2C_Stop(void)
{
	// SDA拉低
	OLED_SDA_Clr();
	// SCL置高
	OLED_SCL_Set();
	OLED_IIC_delay();
	// SDA置高，产生停止信号
	OLED_SDA_Set();
	OLED_IIC_delay();
}

// I2C等待应答(简化处理)
void OLED_I2C_WaitAck(void)
{
	// SDA置高，释放总线
	OLED_SDA_Set();
	OLED_IIC_delay();
	// SCL置高，读取应答位
	OLED_SCL_Set();
	OLED_IIC_delay();
	// SCL拉低
	OLED_SCL_Clr();
	OLED_IIC_delay();
}

// I2C发送一个字节, 高位先行
void OLED_Send_Byte(u8 dat)
{
	u8 i;
	// 循环8次，逐位发送
	for(i=0;i<8;i++)
	{
		// 判断最高位
		if(dat&0x80)
		{
			// 发送1
			OLED_SDA_Set();
        }
		else
		{
			// 发送0
			OLED_SDA_Clr();
        }
		OLED_IIC_delay();
		// SCL置高，数据稳定
		OLED_SCL_Set();
		OLED_IIC_delay();
		// SCL拉低，准备下一位
		OLED_SCL_Clr();
		// 数据左移一位
		dat<<=1;
    }
}

// 向OLED写入一个字节: mode=OLED_CMD写命令, mode=OLED_DATA写数据
void OLED_WR_Byte(u8 dat,u8 mode)
{
	// 发送起始信号
	OLED_I2C_Start();
	// 发送器件地址(0x78)
	OLED_Send_Byte(0x78);
	// 等待应答
	OLED_I2C_WaitAck();
	// 发送控制字节：数据/命令选择
	if(mode){OLED_Send_Byte(0x40);}
  else{OLED_Send_Byte(0x00);}
	OLED_I2C_WaitAck();
	// 发送数据/命令字节
	OLED_Send_Byte(dat);
	OLED_I2C_WaitAck();
	// 发送停止信号
	OLED_I2C_Stop();
}

// 开启OLED显示(电荷泵+显示开)
void OLED_DisPlay_On(void)
{
	// 电荷泵设置命令
	OLED_WR_Byte(0x8D,OLED_CMD);
	// 使能电荷泵
	OLED_WR_Byte(0x14,OLED_CMD);
	// 开启显示
	OLED_WR_Byte(0xAF,OLED_CMD);
}

// 关闭OLED显示(电荷泵关+显示关)
void OLED_DisPlay_Off(void)
{
	// 电荷泵设置命令
	OLED_WR_Byte(0x8D,OLED_CMD);
	// 关闭电荷泵
	OLED_WR_Byte(0x10,OLED_CMD);
	// 关闭显示
	OLED_WR_Byte(0xAE,OLED_CMD);
}

// 刷新显存到屏幕: 逐页写入128列数据
void OLED_Refresh(void)
{
	u8 i,n;
	// 遍历8页
	for(i=0;i<8;i++)
	{
		// 设置页地址
		OLED_WR_Byte(0xb0+i,OLED_CMD);
		// 设置列低地址
		OLED_WR_Byte(0x00,OLED_CMD);
		// 设置列高地址
		OLED_WR_Byte(0x10,OLED_CMD);
		// 发送起始信号
		OLED_I2C_Start();
		// 发送器件地址
		OLED_Send_Byte(0x78);
		OLED_I2C_WaitAck();
		// 发送数据控制字节
		OLED_Send_Byte(0x40);
		OLED_I2C_WaitAck();
		// 发送128列数据
		for(n=0;n<128;n++)
		{
			OLED_Send_Byte(OLED_GRAM[n][i]);
			OLED_I2C_WaitAck();
		}
		// 停止信号
		OLED_I2C_Stop();
  }
}

// 清屏: 显存清零+刷新
void OLED_Clear(void)
{
	u8 i,n;
	// 遍历8页
	for(i=0;i<8;i++)
	{
	   // 遍历128列
	   for(n=0;n<128;n++)
		{
		 // 显存清零
		 OLED_GRAM[n][i]=0;
		}
  }
  // 刷新到屏幕
	OLED_Refresh();
}

// 熄灭(x,y)像素
void OLED_ClearPoint(u8 x,u8 y)
{
    OLED_DrawPoint(x,y,0);
}

// 画点: t=1点亮, t=0熄灭
void OLED_DrawPoint(u8 x,u8 y,u8 t)
{
	u8 i,m,n;
	// 计算所在页
	i=y/8;
	// 计算页内偏移
	m=y%8;
	// 计算位掩码
	n=1<<m;
	// 点亮像素
	if(t){OLED_GRAM[x][i]|=n;}
	// 熄灭像素
	else
	{
		// 取反
		OLED_GRAM[x][i]=~OLED_GRAM[x][i];
		// 置位（取反后相当于清零对应位
		OLED_GRAM[x][i]|=n;
		// 再取反恢复
		OLED_GRAM[x][i]=~OLED_GRAM[x][i];
	}
}

// Bresenham直线算法: mode=1点亮, mode=0熄灭
void OLED_DrawLine(u8 x1,u8 y1,u8 x2,u8 y2,u8 mode)
{
	u16 t;
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;
	// X方向增量
	delta_x=x2-x1;
	// Y方向增量
	delta_y=y2-y1;
	uRow=x1;
	uCol=y1;
	// 确定X方向步进
	if(delta_x>0)incx=1;
	else if (delta_x==0)incx=0;
	else {incx=-1;delta_x=-delta_x;}
	// 确定Y方向步进
	if(delta_y>0)incy=1;
	else if (delta_y==0)incy=0;
	else {incy=-1;delta_y=-delta_y;}
	// 确定主方向距离
	if(delta_x>delta_y)distance=delta_x;
	else distance=delta_y;
	// 逐点绘制
	for(t=0;t<distance+1;t++)
	{
		// 画当前点
		OLED_DrawPoint(uRow,uCol,mode);
		xerr+=delta_x;
		yerr+=delta_y;
		// X方向误差累积超过距离，X步进
		if(xerr>distance)
		{
			xerr-=distance;
			uRow+=incx;
		}
		// Y方向误差累积超过距离，Y步进
		if(yerr>distance)
		{
			yerr-=distance;
			uCol+=incy;
		}
	}
}

// 中点画圆算法: 8分法对称绘制
void OLED_DrawCircle(u8 x,u8 y,u8 r)
{
	int a, b, num;
    a = 0;
    b = r;
    // 中点画圆主循环
    while(2 * b * b >= r * r)
    {
        // 8个对称点
        OLED_DrawPoint(x + a, y - b,1);
        OLED_DrawPoint(x - a, y - b,1);
        OLED_DrawPoint(x - a, y + b,1);
        OLED_DrawPoint(x + a, y + b,1);
        OLED_DrawPoint(x + b, y + a,1);
        OLED_DrawPoint(x + b, y - a,1);
        OLED_DrawPoint(x - b, y - a,1);
        OLED_DrawPoint(x - b, y + a,1);
        a++;
        // 计算中点误差项
        num = (a * a + b * b) - r*r;
        // 误差>0，Y方向减1
        if(num > 0)
        {
            b--;
            a--;
        }
    }
}

// 显示单个字符: size1=字体大小(6/8/12/16/24), mode=1正常/0反色
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size1,u8 mode)
{
	u8 i,m,temp,size2,chr1;
	u8 x0=x,y0=y;
	// 计算字符数据字节数
	if(size1==8||size1==6)size2=6;
	else size2=(size1/8+((size1%8)?1:0))*(size1/2);
	// 字符偏移（从空格开始）
	chr1=chr-' ';
	// 逐字节绘制
	for(i=0;i<size2;i++)
	{
		// 根据字体大小选择对应字库
		if(size1==8||size1==6)
			  {temp=asc2_0806[chr1][i];}
		else if(size1==12)
        {temp=asc2_1206[chr1][i];}
		else if(size1==16)
        {temp=asc2_1608[chr1][i];}
		else if(size1==24)
        {temp=asc2_2412[chr1][i];}
		else return;
		// 逐位绘制
		for(m=0;m<8;m++)
		{
			// 该位为1，按模式绘制
			if(temp&0x01)OLED_DrawPoint(x,y,mode);
			// 该位为0，反模式绘制
			else OLED_DrawPoint(x,y,!mode);
			temp>>=1;
			y++;
		}
		x++;
		// 换行处理（大字体）
		if((size1!=8&&size1!=6)&&((x-x0)==size1/2))
		{
			x=x0;
			y0=y0+8;
		}
		y=y0;
	}
}

// 显示6x8字符(轻量版)
void OLED_ShowChar6x8(u8 x,u8 y,u8 chr,u8 mode)
{
	OLED_ShowChar(x,y,chr,6,mode);
}

// 显示ASCII字符串: 自动根据字体大小调整间距
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 size1,u8 mode)
{
	// 遍历可打印字符范围
	while((*chr>=' ')&&(*chr<='~'))
	{
		// 显示当前字符
		OLED_ShowChar(x,y,*chr,size1,mode);
		// X坐标递增，根据字体大小调整间距
		if(size1==8||size1==6)x+=6;
		else x+=size1/2;
		chr++;
	}
}

// 幂运算: m^n
u32 OLED_Pow(u8 m,u8 n)
{
	u32 result=1;
	// 连乘n次
	while(n--)
	{
		result*=m;
	}
	return result;
}

// 显示数字: num=数值, len=位数, size1=字体大小, mode=1正常/0反色
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size1,u8 mode)
{
	u8 t,temp,m=0;
	if(size1==8||size1==6)m=2;
	// 逐位显示
	for(t=0;t<len;t++)
	{
		// 取第t位数字
		temp=(num/OLED_Pow(10,len-t-1))%10;
		// 数字为0
		if(temp==0)
		{
			OLED_ShowChar(x+(size1/2+m)*t,y,'0',size1,mode);
		}
		// 数字非0
		else
		{
			OLED_ShowChar(x+(size1/2+m)*t,y,temp+'0',size1,mode);
		}
	}
}

// 显示中文: num=字库索引, size1=字体大小(16/24/32/64), mode=1正常/0反色
void OLED_ShowChinese(u8 x,u8 y,u8 num,u8 size1,u8 mode)
{
	u8 m,temp;
	u8 x0=x,y0=y;
	u16 i,size3=(size1/8+((size1%8)?1:0))*size1;
	// 逐字节绘制
	for(i=0;i<size3;i++)
	{
		// 根据字体大小选择对应中文字库
		if(size1==16)
				{temp=Hzk1[num][i];}
		else if(size1==24)
				{temp=Hzk2[num][i];}
		else if(size1==32)
				{temp=Hzk3[num][i];}
		else if(size1==64)
				{temp=Hzk4[num][i];}
		else return;
		// 逐位绘制
		for(m=0;m<8;m++)
		{
			// 该位为1，按模式绘制
			if(temp&0x01)OLED_DrawPoint(x,y,mode);
			// 该位为0，反模式绘制
			else OLED_DrawPoint(x,y,!mode);
			temp>>=1;
			y++;
		}
		x++;
		// 换行处理
		if((x-x0)==size1)
		{x=x0;y0=y0+8;}
		y=y0;
	}
}

// 中文滚动显示: num=字符个数, space=间隔, mode=1正常/0反色
void OLED_ScrollDisplay(u8 num,u8 space,u8 mode)
{
	u8 i,n,t=0,m=0,r;
	while(1)
	{
		// 每16帧添加一个新字符
		if(m==0)
		{
		OLED_ShowChinese(128,24,t,16,mode);
			t++;
		}
		// 一轮显示完一轮
		if(t==num)
		{
			// 滚动间隔
			for(r=0;r<16*space;r++)
			{
				// 左移一列
				for(i=1;i<144;i++)
				{
					for(n=0;n<8;n++)
					{
						OLED_GRAM[i-1][n]=OLED_GRAM[i][n];
					}
				}
				OLED_Refresh();
			}
		t=0;
		}
		m++;
		if(m==16){m=0;}
		// 左移一列
		for(i=1;i<144;i++)
		{
			for(n=0;n<8;n++)
			{
				OLED_GRAM[i-1][n]=OLED_GRAM[i][n];
			}
		}
		OLED_Refresh();
	}
}

// 显示位图: BMP=位图数据, sizex/sizey=尺寸, mode=1正常/0反色
void OLED_ShowPicture(u8 x,u8 y,u8 sizex,u8 sizey,u8 BMP[],u8 mode)
{
	u16 j=0;
	u8 i,n,temp,m;
	u8 x0=x,y0=y;
	// 计算页数
	sizey=sizey/8+((sizey%8)?1:0);
	// 逐页绘制
	for(n=0;n<sizey;n++)
	{
		// 逐列绘制
		for(i=0;i<sizex;i++)
		{
			// 读取位图数据
			temp=BMP[j];
			j++;
			// 逐位绘制
			for(m=0;m<8;m++)
			{
				// 该位为1，按模式绘制
				if(temp&0x01)OLED_DrawPoint(x,y,mode);
				// 该位为0，反模式绘制
				else OLED_DrawPoint(x,y,!mode);
				temp>>=1;
				y++;
			}
			x++;
			// 换行处理
			if((x-x0)==sizex)
			{
				x=x0;
				y0=y0+8;
			}
			y=y0;
		}
	}
}

// OLED初始化: GPIO配置+SSD1306初始化序列
void OLED_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	// 使能GPIOB时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	// 配置PB10(SCL)和PB11(SDA)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	// 输出模式
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	// 推挽输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	// 速度100MHz
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	// 上拉
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	// 延时等待OLED上电稳定
	delay_ms(200);

	// 关闭显示
	OLED_WR_Byte(0xAE,OLED_CMD);
	// 设置低列地址
	OLED_WR_Byte(0x00,OLED_CMD);
	// 设置高列地址
	OLED_WR_Byte(0x10,OLED_CMD);
	// 设置起始行
	OLED_WR_Byte(0x40,OLED_CMD);
	// 对比度设置命令
	OLED_WR_Byte(0x81,OLED_CMD);
	// 对比度值
	OLED_WR_Byte(0xCF,OLED_CMD);
	// 段重映射
	OLED_WR_Byte(0xA1,OLED_CMD);
	// 输出扫描方向
	OLED_WR_Byte(0xC8,OLED_CMD);
	// 正常显示
	OLED_WR_Byte(0xA6,OLED_CMD);
	// 多路复用比命令
	OLED_WR_Byte(0xA8,OLED_CMD);
	// 多路复用比(64MUX
	OLED_WR_Byte(0x3f,OLED_CMD);
	// 显示偏移命令
	OLED_WR_Byte(0xD3,OLED_CMD);
	// 偏移为0
	OLED_WR_Byte(0x00,OLED_CMD);
	// 显示时钟分频命令
	OLED_WR_Byte(0xd5,OLED_CMD);
	// 分频比
	OLED_WR_Byte(0x80,OLED_CMD);
	// 预充电周期命令
	OLED_WR_Byte(0xD9,OLED_CMD);
	// 预充电周期
	OLED_WR_Byte(0xF1,OLED_CMD);
	// 硬件配置命令
	OLED_WR_Byte(0xDA,OLED_CMD);
	// COM引脚配置
	OLED_WR_Byte(0x12,OLED_CMD);
	// VCOMH命令
	OLED_WR_Byte(0xDB,OLED_CMD);
	// VCOMH电平
	OLED_WR_Byte(0x40,OLED_CMD);
	// 内存地址设置命令
	OLED_WR_Byte(0x20,OLED_CMD);
	// 页地址模式
	OLED_WR_Byte(0x02,OLED_CMD);
	// 电荷泵命令
	OLED_WR_Byte(0x8D,OLED_CMD);
	// 使能电荷泵
	OLED_WR_Byte(0x14,OLED_CMD);
	// 全局显示开
	OLED_WR_Byte(0xA4,OLED_CMD);
	// 正常显示
	OLED_WR_Byte(0xA6,OLED_CMD);
	// 清屏
	OLED_Clear();
	// 开启显示
	OLED_WR_Byte(0xAF,OLED_CMD);
}
