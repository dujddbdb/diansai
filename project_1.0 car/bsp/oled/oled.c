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
	if(i==0)
	{
		OLED_WR_Byte(0xA6,OLED_CMD);
	}
	if(i==1)
	{
		OLED_WR_Byte(0xA7,OLED_CMD);
	}
}

// 屏幕旋转180度: i=0正常, i=1旋转
void OLED_DisplayTurn(u8 i)
{
	if(i==0)
	{
		OLED_WR_Byte(0xC8,OLED_CMD);
		OLED_WR_Byte(0xA1,OLED_CMD);
	}
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
	OLED_SDA_Set();
	OLED_SCL_Set();
	OLED_IIC_delay();
	OLED_SDA_Clr();
	OLED_IIC_delay();
	OLED_SCL_Clr();
	OLED_IIC_delay();
}

// I2C停止信号: SCL=1时SDA从0→1
void OLED_I2C_Stop(void)
{
	OLED_SDA_Clr();
	OLED_SCL_Set();
	OLED_IIC_delay();
	OLED_SDA_Set();
	OLED_IIC_delay();
}

// I2C等待应答(简化处理)
void OLED_I2C_WaitAck(void)
{
	OLED_SDA_Set();
	OLED_IIC_delay();
	OLED_SCL_Set();
	OLED_IIC_delay();
	OLED_SCL_Clr();
	OLED_IIC_delay();
}

// I2C发送一个字节, 高位先行
void OLED_Send_Byte(u8 dat)
{
	u8 i;
	for(i=0;i<8;i++)
	{
		if(dat&0x80)
		{
			OLED_SDA_Set();
        }
		else
		{
			OLED_SDA_Clr();
        }
		OLED_IIC_delay();
		OLED_SCL_Set();
		OLED_IIC_delay();
		OLED_SCL_Clr();
		dat<<=1;
    }
}

// 向OLED写入一个字节: mode=OLED_CMD写命令, mode=OLED_DATA写数据
void OLED_WR_Byte(u8 dat,u8 mode)
{
	OLED_I2C_Start();
	OLED_Send_Byte(0x78);
	OLED_I2C_WaitAck();
	if(mode){OLED_Send_Byte(0x40);}
  else{OLED_Send_Byte(0x00);}
	OLED_I2C_WaitAck();
	OLED_Send_Byte(dat);
	OLED_I2C_WaitAck();
	OLED_I2C_Stop();
}

// 开启OLED显示(电荷泵+显示开)
void OLED_DisPlay_On(void)
{
	OLED_WR_Byte(0x8D,OLED_CMD);
	OLED_WR_Byte(0x14,OLED_CMD);
	OLED_WR_Byte(0xAF,OLED_CMD);
}

// 关闭OLED显示(电荷泵关+显示关)
void OLED_DisPlay_Off(void)
{
	OLED_WR_Byte(0x8D,OLED_CMD);
	OLED_WR_Byte(0x10,OLED_CMD);
	OLED_WR_Byte(0xAE,OLED_CMD);
}

// 刷新显存到屏幕: 逐页写入128列数据
void OLED_Refresh(void)
{
	u8 i,n;
	for(i=0;i<8;i++)
	{
		OLED_WR_Byte(0xb0+i,OLED_CMD);
		OLED_WR_Byte(0x00,OLED_CMD);
		OLED_WR_Byte(0x10,OLED_CMD);
		OLED_I2C_Start();
		OLED_Send_Byte(0x78);
		OLED_I2C_WaitAck();
		OLED_Send_Byte(0x40);
		OLED_I2C_WaitAck();
		for(n=0;n<128;n++)
		{
			OLED_Send_Byte(OLED_GRAM[n][i]);
			OLED_I2C_WaitAck();
		}
		OLED_I2C_Stop();
  }
}

// 清屏: 显存清零+刷新
void OLED_Clear(void)
{
	u8 i,n;
	for(i=0;i<8;i++)
	{
	   for(n=0;n<128;n++)
		{
		 OLED_GRAM[n][i]=0;
		}
  }
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
	i=y/8;
	m=y%8;
	n=1<<m;
	if(t){OLED_GRAM[x][i]|=n;}
	else
	{
		OLED_GRAM[x][i]=~OLED_GRAM[x][i];
		OLED_GRAM[x][i]|=n;
		OLED_GRAM[x][i]=~OLED_GRAM[x][i];
	}
}

// Bresenham直线算法: mode=1点亮, mode=0熄灭
void OLED_DrawLine(u8 x1,u8 y1,u8 x2,u8 y2,u8 mode)
{
	u16 t;
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;
	delta_x=x2-x1;
	delta_y=y2-y1;
	uRow=x1;
	uCol=y1;
	if(delta_x>0)incx=1;
	else if (delta_x==0)incx=0;
	else {incx=-1;delta_x=-delta_x;}
	if(delta_y>0)incy=1;
	else if (delta_y==0)incy=0;
	else {incy=-1;delta_y=-delta_y;}
	if(delta_x>delta_y)distance=delta_x;
	else distance=delta_y;
	for(t=0;t<distance+1;t++)
	{
		OLED_DrawPoint(uRow,uCol,mode);
		xerr+=delta_x;
		yerr+=delta_y;
		if(xerr>distance)
		{
			xerr-=distance;
			uRow+=incx;
		}
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
    while(2 * b * b >= r * r)
    {
        OLED_DrawPoint(x + a, y - b,1);
        OLED_DrawPoint(x - a, y - b,1);
        OLED_DrawPoint(x - a, y + b,1);
        OLED_DrawPoint(x + a, y + b,1);
        OLED_DrawPoint(x + b, y + a,1);
        OLED_DrawPoint(x + b, y - a,1);
        OLED_DrawPoint(x - b, y - a,1);
        OLED_DrawPoint(x - b, y + a,1);
        a++;
        num = (a * a + b * b) - r*r;
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
	if(size1==8||size1==6)size2=6;
	else size2=(size1/8+((size1%8)?1:0))*(size1/2);
	chr1=chr-' ';
	for(i=0;i<size2;i++)
	{
		if(size1==8||size1==6)
			  {temp=asc2_0806[chr1][i];}
		else if(size1==12)
        {temp=asc2_1206[chr1][i];}
		else if(size1==16)
        {temp=asc2_1608[chr1][i];}
		else if(size1==24)
        {temp=asc2_2412[chr1][i];}
		else return;
		for(m=0;m<8;m++)
		{
			if(temp&0x01)OLED_DrawPoint(x,y,mode);
			else OLED_DrawPoint(x,y,!mode);
			temp>>=1;
			y++;
		}
		x++;
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
	while((*chr>=' ')&&(*chr<='~'))
	{
		OLED_ShowChar(x,y,*chr,size1,mode);
		if(size1==8||size1==6)x+=6;
		else x+=size1/2;
		chr++;
	}
}

// 幂运算: m^n
u32 OLED_Pow(u8 m,u8 n)
{
	u32 result=1;
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
	for(t=0;t<len;t++)
	{
		temp=(num/OLED_Pow(10,len-t-1))%10;
		if(temp==0)
		{
			OLED_ShowChar(x+(size1/2+m)*t,y,'0',size1,mode);
		}
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
	for(i=0;i<size3;i++)
	{
		if(size1==16)
				{temp=Hzk1[num][i];}
		else if(size1==24)
				{temp=Hzk2[num][i];}
		else if(size1==32)
				{temp=Hzk3[num][i];}
		else if(size1==64)
				{temp=Hzk4[num][i];}
		else return;
		for(m=0;m<8;m++)
		{
			if(temp&0x01)OLED_DrawPoint(x,y,mode);
			else OLED_DrawPoint(x,y,!mode);
			temp>>=1;
			y++;
		}
		x++;
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
		if(m==0)
		{
		OLED_ShowChinese(128,24,t,16,mode);
			t++;
		}
		if(t==num)
		{
			for(r=0;r<16*space;r++)
			{
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
	sizey=sizey/8+((sizey%8)?1:0);
	for(n=0;n<sizey;n++)
	{
		for(i=0;i<sizex;i++)
		{
			temp=BMP[j];
			j++;
			for(m=0;m<8;m++)
			{
				if(temp&0x01)OLED_DrawPoint(x,y,mode);
				else OLED_DrawPoint(x,y,!mode);
				temp>>=1;
				y++;
			}
			x++;
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
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	delay_ms(200);

	OLED_WR_Byte(0xAE,OLED_CMD);
	OLED_WR_Byte(0x00,OLED_CMD);
	OLED_WR_Byte(0x10,OLED_CMD);
	OLED_WR_Byte(0x40,OLED_CMD);
	OLED_WR_Byte(0x81,OLED_CMD);
	OLED_WR_Byte(0xCF,OLED_CMD);
	OLED_WR_Byte(0xA1,OLED_CMD);
	OLED_WR_Byte(0xC8,OLED_CMD);
	OLED_WR_Byte(0xA6,OLED_CMD);
	OLED_WR_Byte(0xA8,OLED_CMD);
	OLED_WR_Byte(0x3f,OLED_CMD);
	OLED_WR_Byte(0xD3,OLED_CMD);
	OLED_WR_Byte(0x00,OLED_CMD);
	OLED_WR_Byte(0xd5,OLED_CMD);
	OLED_WR_Byte(0x80,OLED_CMD);
	OLED_WR_Byte(0xD9,OLED_CMD);
	OLED_WR_Byte(0xF1,OLED_CMD);
	OLED_WR_Byte(0xDA,OLED_CMD);
	OLED_WR_Byte(0x12,OLED_CMD);
	OLED_WR_Byte(0xDB,OLED_CMD);
	OLED_WR_Byte(0x40,OLED_CMD);
	OLED_WR_Byte(0x20,OLED_CMD);
	OLED_WR_Byte(0x02,OLED_CMD);
	OLED_WR_Byte(0x8D,OLED_CMD);
	OLED_WR_Byte(0x14,OLED_CMD);
	OLED_WR_Byte(0xA4,OLED_CMD);
	OLED_WR_Byte(0xA6,OLED_CMD);
	OLED_Clear();
	OLED_WR_Byte(0xAF,OLED_CMD);
}