#ifndef ULTRASONIC_TASK_H
#define ULTRASONIC_TASK_H

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
#include "remote_app.h"
#include "chassis_task.h"
#include "dwa.h"

#define ULTRASONIC_TASK_INIT_TIME       (10)


typedef struct car_
{
	
	float v[2];//xy速度cm/s
	float distance[2];//sy移动距离cm
	float position_a[2];//map当前位置
	int16_t position_b[2];
	int16_t init_position[2];//map初始位置
	uint16_t width;
	uint16_t init_width;
	uint16_t angle;
	uint16_t target[2];
}car_typedef;


void ULTRASONIC_task(void *argument);

#endif
