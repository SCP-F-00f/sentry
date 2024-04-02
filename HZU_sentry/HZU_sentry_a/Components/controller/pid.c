#include "pid.h"
/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note           
  */
void PID_Calc(pid_t *pid)
{
	pid->err[2] = pid->err[1];
	pid->err[1] = pid->err[0];
	pid->err[0] = pid->set - pid->fdb;
	if(pid->mode==PID_DELTA)//???PID
	{
		pid->iout = pid->ki*pid->err[0];
		
		pid->output += pid->kp*(pid->err[0]-pid->err[1]) \
					 + pid->iout                         \
					 + pid->kd*(pid->err[0]-2.0f*pid->err[1]+pid->err[2]);
	}
	else if(pid->mode==PID_POSITION)//???PID
	{
		pid->iout += pid->ki*pid->err[0];
		if(pid->iout > pid->ioutMax)            pid->iout = pid->ioutMax;
	    else if(pid->iout < (-pid->ioutMax))    pid->iout = (-pid->ioutMax);
		
		pid->output  = pid->kp*(pid->err[0]) \
					 + pid->iout             \
					 + pid->kd*(pid->err[0]-pid->err[1]);
	}
	//????
	if(pid->output > pid->outputMax)            pid->output = pid->outputMax;
	else if(pid->output < (-pid->outputMax))    pid->output = (-pid->outputMax);
}
/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note           
  */
void PID_Reset(pid_t *pid)
{
	pid->set = 0.0f;
	pid->fdb = 0.0f;
	pid->err[0] = 0.0f;
	pid->err[1] = 0.0f;
	pid->err[2] = 0.0f;
	pid->iout = 0.0f;
	pid->output = 0.0f;
}





cali_chassis_t cali_chassis_pid =
{
/**
* @brief fric1 pid param         
* @note    
*/	
	.cm_pid.kp = 20.0,  //6.2  //10stable  20
	.cm_pid.ki = 0.0,  //0.3   //0
	.cm_pid.kd = 10,
	.cm_pid.ioutput_max = 1000,
	.cm_pid.output_max = 5000,
	.cm_pid.mode = PID_POSITION,			//PID_DELTA	PID_POSITION
	
	.rotate_pid.kp = 0.5,  //1.0
	.rotate_pid.ki = 0.0,
	.rotate_pid.kd = 30,	//30
	.rotate_pid.ioutput_max = 1000,
	.rotate_pid.output_max = 5000,
	.rotate_pid.mode = PID_POSITION,	//PID_POSITION	//待调试确定 目前抖动很大
};

