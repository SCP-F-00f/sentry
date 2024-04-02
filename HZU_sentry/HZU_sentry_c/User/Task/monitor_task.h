#ifndef MONITOR_TASK_H
#define MONITOR_TASK_H

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"

typedef struct
{
	uint32_t time;            //模块刷新时的系统时间
	uint8_t enable_flag;      //是否使能监控标志位
	uint8_t *error_msg;       //oled打印的错误信息
	uint8_t lost_flag;        //离线标志位
	uint16_t timeout;         //运行超时时间

//	uint16_t error_oled_row;  //错误显示再oled上的行
//	uint8_t display_flag;
}error_t;

typedef struct //需要监控的模块
{
	error_t remote;
	error_t yaw_motor;
	error_t pitch_motor;
	error_t trigger_motor;
	error_t fric1_motor;
	error_t fric2_motor;
	
	uint32_t system_current_time;//系统当前时间
	uint16_t error_cnt;//错误数量
	uint8_t exist_error_flag;

}monitor_t;

extern monitor_t monitor;

#endif
