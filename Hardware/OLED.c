#include "stm32f10x.h"
#include "OLED_Font.h"
#include "OLED.h"
#include <string.h>

/*引脚配置*/
#define OLED_W_SCL(x)		GPIO_WriteBit(GPIOB, GPIO_Pin_8, (BitAction)(x))
#define OLED_W_SDA(x)		GPIO_WriteBit(GPIOB, GPIO_Pin_9, (BitAction)(x))

/* 显存：8 页 x 128 列。所有绘制先写这里，再按页搬到屏幕。
 * OLED_SENT 保存"屏幕当前内容"，用来跳过没有变化的页，
 * 这样每次刷新只传真正变化的 1~2 页，且不会出现整屏闪烁。 */
static uint8_t OLED_GRAM[8][128];
static uint8_t OLED_SENT[8][128];
static uint8_t OLED_PageCursor = 0;

/*引脚初始化*/
void OLED_I2C_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	OLED_W_SCL(1);
	OLED_W_SDA(1);
}

/**
  * @brief  I2C开始
  */
void OLED_I2C_Start(void)
{
	OLED_W_SDA(1);
	OLED_W_SCL(1);
	OLED_W_SDA(0);
	OLED_W_SCL(0);
}

/**
  * @brief  I2C停止
  */
void OLED_I2C_Stop(void)
{
	OLED_W_SDA(0);
	OLED_W_SCL(1);
	OLED_W_SDA(1);
}

/**
  * @brief  I2C发送一个字节
  */
void OLED_I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		OLED_W_SDA(!!(Byte & (0x80 >> i)));
		OLED_W_SCL(1);
		OLED_W_SCL(0);
	}
	OLED_W_SCL(1);	//额外的一个时钟，不处理应答信号
	OLED_W_SCL(0);
}

/**
  * @brief  OLED写命令
  */
void OLED_WriteCommand(uint8_t Command)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78);		//从机地址
	OLED_I2C_SendByte(0x00);		//写命令
	OLED_I2C_SendByte(Command); 
	OLED_I2C_Stop();
}

/**
  * @brief  OLED写数据（单字节，保留兼容）
  */
void OLED_WriteData(uint8_t Data)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78);		//从机地址
	OLED_I2C_SendByte(0x40);		//写数据
	OLED_I2C_SendByte(Data);
	OLED_I2C_Stop();
}

static void OLED_WriteDataN(const uint8_t *Buf, uint16_t Len)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(0x78);
    OLED_I2C_SendByte(0x40);
    while (Len--) OLED_I2C_SendByte(*Buf++);
    OLED_I2C_Stop();
}

/**
  * @brief  OLED设置光标位置
  */
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
	OLED_WriteCommand(0xB0 | Y);					//设置Y位置
	OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));	//设置X位置高4位
	OLED_WriteCommand(0x00 | (X & 0x0F));			//设置X位置低4位
}

/**
  * @brief  OLED清屏（只清显存，真正刷屏由 OLED_Refresh* 完成）
  */
void OLED_Clear(void)
{
    memset(OLED_GRAM, 0x00, sizeof(OLED_GRAM));
}

/**
  * @brief  把一页显存推到屏幕（128 字节，一次 I2C 传输）
  */
static void OLED_PushPage(uint8_t Page)
{
    OLED_SetCursor(Page, 0);
    OLED_WriteDataN(OLED_GRAM[Page], 128);
    memcpy(OLED_SENT[Page], OLED_GRAM[Page], 128);
}

/**
  * @brief  推送全部 8 页（初始化时用）
  */
void OLED_RefreshAll(void)
{
    uint8_t page;
    for (page = 0; page < 8; page++)
        OLED_PushPage(page);
}

/**
  * @brief  推送下一页真正变化的内容，一次调用只传一页
  * @note   放在 10ms 控制拍里调用，刷完一屏需要 8 拍（80ms），
  *         单次耗时只有约 0.3ms，不会再阻塞控制周期。
  */
void OLED_RefreshNext(void)
{
    uint8_t page = OLED_PageCursor;

    OLED_PageCursor = (uint8_t)((page + 1) & 0x07);

    if (memcmp(OLED_GRAM[page], OLED_SENT[page], 128) != 0)
        OLED_PushPage(page);
}

void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
    uint8_t idx = (uint8_t)(Char - ' ');
    uint8_t page, col;

    if (Line < 1 || Line > 4 || Column < 1 || Column > 16) return;
    if (idx > 94) return;                       /* 只支持 ASCII 可见字符 */

    page = (uint8_t)((Line - 1) * 2);
    col  = (uint8_t)((Column - 1) * 8);

    memcpy(&OLED_GRAM[page][col],     &OLED_F8x16[idx][0], 8);
    memcpy(&OLED_GRAM[page + 1][col], &OLED_F8x16[idx][8], 8);
}

/**
  * @brief  OLED显示字符串
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  String 要显示的字符串，ASCII可见字符
  */
void OLED_ShowString(uint8_t Line, uint8_t Column, const char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i++)
	{
		OLED_ShowChar(Line, (uint8_t)(Column + i), String[i]);
	}
}

/**
  * @brief  OLED次方函数
  */
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y--)
	{
		Result *= X;
	}
	return Result;
}

/**
  * @brief  OLED显示数字（十进制，正数）
  */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, (uint8_t)(Column + i), (char)(Number / OLED_Pow(10, Length - i - 1) % 10 + '0'));
	}
}

/**
  * @brief  OLED显示数字（十进制，带符号数）
  */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
	uint8_t i;
	uint32_t Number1;
	if (Number >= 0)
	{
		OLED_ShowChar(Line, Column, '+');
		Number1 = (uint32_t)Number;
	}
	else
	{
		OLED_ShowChar(Line, Column, '-');
		Number1 = (uint32_t)(-Number);
	}
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, (uint8_t)(Column + i + 1), (char)(Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0'));
	}
}

/**
  * @brief  OLED显示数字（十六进制，正数）
  */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i, SingleNumber;
	for (i = 0; i < Length; i++)							
	{
		SingleNumber = (uint8_t)(Number / OLED_Pow(16, Length - i - 1) % 16);
		if (SingleNumber < 10)
		{
			OLED_ShowChar(Line, (uint8_t)(Column + i), (char)(SingleNumber + '0'));
		}
		else
		{
			OLED_ShowChar(Line, (uint8_t)(Column + i), (char)(SingleNumber - 10 + 'A'));
		}
	}
}

/**
  * @brief  OLED显示数字（二进制，正数）
  */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, (uint8_t)(Column + i), (char)(Number / OLED_Pow(2, Length - i - 1) % 2 + '0'));
	}
}

/**
  * @brief  OLED初始化
  */
void OLED_Init(void)
{
	uint32_t i, j;
	
	for (i = 0; i < 1000; i++)			//上电延时
	{
		for (j = 0; j < 1000; j++);
	}
	
	OLED_I2C_Init();			//端口初始化
	
	OLED_WriteCommand(0xAE);	//关闭显示
	
	OLED_WriteCommand(0xD5);	//设置显示时钟分频比/振荡器频率
	OLED_WriteCommand(0x80);
	
	OLED_WriteCommand(0xA8);	//设置多路复用率
	OLED_WriteCommand(0x3F);
	
	OLED_WriteCommand(0xD3);	//设置显示偏移
	OLED_WriteCommand(0x00);
	
	OLED_WriteCommand(0x40);	//设置显示开始行
	
	OLED_WriteCommand(0xA1);	//设置左右方向，0xA1正常 0xA0左右反置
	
	OLED_WriteCommand(0xC8);	//设置上下方向，0xC8正常 0xC0上下反置

	OLED_WriteCommand(0xDA);	//设置COM引脚硬件配置
	OLED_WriteCommand(0x12);
	
	OLED_WriteCommand(0x81);	//设置对比度控制
	OLED_WriteCommand(0xCF);

	OLED_WriteCommand(0xD9);	//设置预充电周期
	OLED_WriteCommand(0xF1);

	OLED_WriteCommand(0xDB);	//设置VCOMH取消选择级别
	OLED_WriteCommand(0x30);

	OLED_WriteCommand(0xA4);	//设置整个显示打开/关闭

	OLED_WriteCommand(0xA6);	//设置正常/倒转显示

	OLED_WriteCommand(0x8D);	//设置充电泵
	OLED_WriteCommand(0x14);

	OLED_WriteCommand(0xAF);	//开启显示
		
	OLED_Clear();				//清显存
	OLED_RefreshAll();			//上电清屏一次
}
