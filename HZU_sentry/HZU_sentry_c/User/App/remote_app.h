#ifndef REMOTE_APP_H
#define REMOTE_APP_H

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"

#define RC_CHANNEL_VALUE_MAX            (1684u)      
#define RC_CHANNEL_VALUE_MIDDLE         (1024u) 
#define RC_CHANNEL_VALUE_MIN            (364u) 

#define RC_SW_UP                        ((uint16_t)1)	//遥控器左侧拨码
#define RC_SW_MID                       ((uint16_t)3)
#define RC_SW_DOWN                      ((uint16_t)2)

#define RC_CHANNEL_ALLOW_ERROR_VALUE    (700u)
#define RC_SW_MAX_VALUE                 (3u)


#define ANALOY_KEY_THRE_VALUE    (50) 
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)

#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)

typedef struct//__packed 是为了防止编译器优化(字节对齐)，减少内存占用，但也导致cpu读取效率降低
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
	struct    
	{
		uint16_t v;
	} key;
	float yaw_sensit;
	float pitch_sensit;
}RC_ctrl_t;

typedef struct
{
	int16_t key_up;
	int16_t key_down;
	int16_t key_ok;
	int16_t key_back;
	void *rc_ctrl_data_point;
}rc_analog_key_t;

extern RC_ctrl_t rc_ctrl_data;
extern rc_analog_key_t rc_analog_key;
void get_remote_data(uint8_t *dbus_buff, RC_ctrl_t *rc_ctrl_data);
void RC_data_error_process(void);
RC_ctrl_t *get_rc_data_point(void);
void rc_analoy_key_init(void);
void get_rc_analoy_key_value(rc_analog_key_t *rc_analoy_key, RC_ctrl_t *rc_ctrl_data);

#endif


