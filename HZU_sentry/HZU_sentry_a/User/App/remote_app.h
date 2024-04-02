#ifndef REMOTE_APP_H
#define REMOTE_APP_H

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"

#define RC_CHANNEL_VALUE_MAX            (1684u)      
#define RC_CHANNEL_VALUE_MIDDLE         (1024u) 
#define RC_CHANNEL_VALUE_MIN            (364u) 

#define RC_SW_UP                        ((uint16_t)1)
#define RC_SW_MID                       ((uint16_t)3)
#define RC_SW_DOWN                      ((uint16_t)2)

#define RC_CHANNEL_ALLOW_ERROR_VALUE    (700u)
#define RC_SW_MAX_VALUE                 (3u)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)	//运动小陀螺
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)	//静止小陀螺
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)

typedef  struct
{
	struct
	{
		int16_t ch0;
		int16_t ch1;
		int16_t ch2;
		int16_t ch3;
		int16_t ch4;
		uint8_t s1;
		uint8_t s2;
	} rc;
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
		uint8_t last_press_l;
		uint8_t last_press_r;
	} mouse;
	struct    //__packed struct是为了防止编译器优化，可以防止传输错误，但是也带来了麻烦
	{
		uint16_t v;
	} key;
}RC_ctrl_t;

extern RC_ctrl_t rc_ctrl_data;

int16_t RC_abs(int16_t value);
void get_remote_data(uint8_t *dbus_buff, RC_ctrl_t *rc_ctrl_data);
void RC_data_error_process(void);
RC_ctrl_t *get_rc_data_point(void);
#endif


