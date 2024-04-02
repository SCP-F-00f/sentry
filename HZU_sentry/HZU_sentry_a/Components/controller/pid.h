#ifndef PID_H
#define PID_H
#include "main.h"

enum PID_MODE
{
	PID_POSITION = 0,
    PID_DELTA = 1
};

typedef struct pid_t/*????????,?????????*/
{
	float set;
	float fdb;

	float err[3];
	
	float kp;
	float ki;
	float kd;

	float iout;
	float ioutMax;
	
	float output;
	float outputMax;
	
	float kp_offset;
	float ki_offset;
	float kd_offset;
	
	uint8_t mode;
	
	void (*Calc)(struct pid_t *pid);//????
	void (*Reset)(struct pid_t *pid);
}pid_t;

typedef struct
{
	pid_t position_pid;
	pid_t speed_pid;
}cascade_pid_t;

typedef struct
{
	float kp;
	float ki;
	float kd;
	
	float ioutput_max;
	float output_max;
	uint8_t mode;
}cali_pid_t;

typedef struct
{
	cali_pid_t position;
	cali_pid_t speed;
}cali_cascade_pid_t;
typedef struct
{
	cali_cascade_pid_t yaw_pid;
	cali_cascade_pid_t pitch_pid;
}cali_gimbal_t;

typedef struct
{	
	cali_cascade_pid_t trigger_pid;
	cali_pid_t fric1_pid;
	cali_pid_t fric2_pid;
}cali_shoot_t;
typedef struct
{
	cali_pid_t cm_pid;
	cali_pid_t rotate_pid;
}cali_chassis_t;

extern cali_gimbal_t cali_gimbal_pid;
extern cali_shoot_t cali_shoot_pid;
extern cali_chassis_t cali_chassis_pid;

void PID_Calc(pid_t *pid);
void PID_Reset(pid_t *pid);

#endif
