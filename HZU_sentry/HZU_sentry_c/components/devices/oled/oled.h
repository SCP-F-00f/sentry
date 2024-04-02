#ifndef __OLED_H__
#define __OLED_H__

#include "sys.h"
#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
/*****************************设置数据类型*****************************/
typedef char *pchar;
typedef unsigned char *puchar;
typedef int  *pint;
typedef unsigned int *puint;

typedef char int8;
typedef int int16;
typedef long int int32;

typedef unsigned char uint8;
typedef unsigned int uint16;
typedef unsigned long int uint32;
/*****************************************
【功 能 】:初始化引脚定义
【说 明 】:直接在这里改IO就可以了*/
/*----RST-----B10-----*/
/*----DC-----B9-----*/
/*----SCLK-----B3-----*/
/*----MOSI-----A7-----*/
#define LED_SCLH       PBout(12)=1
#define LED_SCLL       PBout(12)=0

#define LED_SDAH       PBout(13)=1
#define LED_SDAL       PBout(13)=0

#define LED_RSTH       PBout(15)=1
#define LED_RSTL       PBout(15)=0

#define LED_DCH        PBout(14)=1
#define LED_DCL        PBout(14)=0
//测试f429
//#define LED_SCLH       PBout(10)=1
//#define LED_SCLL       PBout(10)=0

//#define LED_SDAH       PBout(9)=1
//#define LED_SDAL       PBout(9)=0

//#define LED_RSTH       PBout(3)=1
//#define LED_RSTL       PBout(3)=0

//#define LED_DCH        PAout(7)=1
//#define LED_DCL        PAout(7)=0
/************************************************/
void LEDPIN_Init(void);   //LED控制引脚初始化
void oled_init(void);
void LED_CLS(void);
void LED_Set_Pos(uint8 x, uint8 y);//设置坐标函数
void LED_WrDat(uint8 data);   //写数据函数
void LED_P6x8Char(uint8 x,uint8 y,uint8 ch);
void LED_P6x8Str(uint8 x,uint8 y,uint8 ch[]);
void LED_P8x16Str(uint8 x,uint8 y,uint8 ch[]);
void LED_P14x16Str(uint8 x,uint8 y,uint8 ch[]);
void LED_PXx16MixStr(uint8 x, uint8 y, uint8 ch[]);
void LED_PrintBMP(uint8 x0,uint8 y0,uint8 x1,uint8 y1,uint8 bmp[]); 
void LED_Fill(uint8 dat);
void LED_PrintValueC(uint8 x, uint8 y,int8 data);
void LED_PrintValueI(uint8 x, uint8 y, int16 data);
void LED_PrintValueI1(uint8 x, uint8 y, int16 data);
void LED_PrintValueI2(uint8 x, uint8 y, int16 data);
void LED_PrintValueF(uint8 x, uint8 y, float data, uint8 num);
void LED_PrintValueFP(uint8 x, uint8 y, uint16 data, uint8 num);
void LED_PrintEdge(void);
void LED_Cursor(uint8 cursor_column, uint8 cursor_row);
void LED_PrintLine(void);
 

int32_t OLED_printf(uint8_t x,uint8_t y,const char *pFormat, ...);
void OLED_Set_Pos(unsigned char x, unsigned char y) ;//外部，图像显示用
 
void img_show(void);
void team_name(void);
void look_look(void);

#endif



