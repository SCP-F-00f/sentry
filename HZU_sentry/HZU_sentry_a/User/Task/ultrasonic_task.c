/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HongYuJia 
 * @Teammate：
 * @Version: V3.0
 * @Date:2021.4.13
 * @Description:   人机交互OLED显示
 * @Note:       
 * @Others: 
**/
#include "ULTRASONIC_task.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "oled.h"
#include "remote_app.h"
#include "pid.h"
#include "judge.h"
#include "ins_task.h"
#include "rule.h"
#include "i2c.h"

extern volatile int time;
extern volatile long long us_5;
extern volatile uint16_t T1;
extern volatile int tim;
extern chassis_control_data_t chassis_control_data;
uint8_t HCSR04_address;
extern volatile uint8_t odometer_sign;

double HCSR04_GetValue()
{
		
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
	  us_5 = 0;
	  while(us_5<3)
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
	  time = 0;
	  vTaskDelay(20);
	  tim = time;
	return ((tim * 0.000001) * 340 * 100) / 2;
}  //cm

extern uint8_t map[240][168];
extern float yaw;
extern car_typedef car;
float slip_num=18.0,car_time=0.000005,position_a[2];
float kkk,l;
float cmd_yaw;
extern connect_t connect_data;
void ULTRASONIC_task(void *argument)
{
	uint8_t HCSR04_command=0x8a;
	vTaskDelay(ULTRASONIC_TASK_INIT_TIME);
	while(1)
	{
	if(odometer_sign)
	{
		extern float yaw_yaw;
		odometer_sign=0;
		car.v[0] = ( cm1_msg.speed_rpm + cm2_msg.speed_rpm - cm3_msg.speed_rpm - cm4_msg.speed_rpm)/(60.0/(double)(2*PI))*cos(PI/4)*cos(PI/4)*7.625/19;
		car.v[1] = (cm1_msg.speed_rpm - cm2_msg.speed_rpm - cm3_msg.speed_rpm + cm4_msg.speed_rpm)/(60.0/(double)(2*PI))*cos(PI/4)*cos(PI/4)*7.625/19;//60 rpm = 2PI rad/s, r = 15.25 cm, cos45 麦轮解算
		car.distance[0] = (car.v[0]*cos(yaw_yaw*PI/180)+car.v[1]*sin(yaw_yaw*PI/180))*car_time;//2us, s = v*t, cm 0.000005	
		car.distance[1] = (car.v[0]*sin(yaw_yaw*PI/180)+car.v[1]*cos(yaw_yaw*PI/180))*car_time;
		position_a[0] =   position_a[0] + (car.distance[0]);
		position_a[1] =   position_a[1] + (car.distance[1]);
		car.position_a[0] =  position_a[0]*slip_num;
		car.position_a[1] =  position_a[1]*slip_num;
		cmd_yaw += connect_data.rotate_speed*car_time;
	}
	}
}



