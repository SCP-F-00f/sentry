#ifndef GUI_APP_H
#define GUI_APP_H

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
#include "oled.h"

#define ORDER_OFFSET_VALUE (1)
#define GUI_TASK_INIT_TIME       (300)

void visualscope(void);
static unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT);
void OutPut_Data1(void);
void GUI_interaction(void);
void PD_PID_debugging(void);
void differ_speed_debug(void);
void speed_key_debug(void);
void show_ad_value(void);
void show_max_tran(void);
void on_state_img(void);
void data_and_flag(void);
void vcan_visualscope(void);
void BOMA_look_img(void);
//void function_selecct(void);
void Meet_debugging(void);


void bsp_senior_msg(void);
void pid_debugging(void);
void judge_data(void);
void vision_debug(void);
#endif

