#include "pid.h"
/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note           
  *  参数整定找最佳， 从小到大顺序查。
                    先是比例后积分， 最后再把微分加。
                    曲线振荡很频繁， 比例度盘要放大。
                    曲线漂浮绕大弯， 比例度盘往小扳。
                    曲线偏离回复慢， 积分时间往下降。
                    曲线波动周期长， 积分时间再加长。
                    曲线振荡频率快， 先把微分降下来。
                    动差大来波动慢， 微分时间应加长。
                    理想曲线两个波， 前高后低四比一。
                    一看二调多分析， 调节质量不会低。
  */
	 float c1,c2,c3,temp;
void PID_Calc(pid_t *pid)
{
	pid->err[2] = pid->err[1];
	pid->err[1] = pid->err[0];
	pid->err[0] = pid->set - pid->fdb;
	if(pid->mode==PID_DELTA)//增量式PID
	{
		pid->iout = pid->ki*pid->err[0];
		
		pid->output += pid->kp*(pid->err[0]-pid->err[1]) \
					 + pid->iout                         \
					 + pid->kd*(pid->err[0]-2.0f*pid->err[1]+pid->err[2]);
	}
	else if(pid->mode==PID_POSITION)//位置式PID
	{
		pid->iout += pid->ki*pid->err[0];
		if(pid->iout > pid->ioutMax)            pid->iout = pid->ioutMax;
	    else if(pid->iout < (-pid->ioutMax))    pid->iout = (-pid->ioutMax);
		//微分先行
//	temp=pid->kd * pid->gama + pid->kp;		
//  c3=pid->kd/temp;
//  c2=(pid->kd+ pid->kp)/temp;
//  c1=pid->gama*c3;
//  pid-> derivative=c1* pid-> derivative+c2*pid->output+c3* pid-> lastout;
//	pid-> lastout=pid->output;
		pid->thisdev = pid->kd*(1-pid->a)*(pid->err[0]-pid->err[1])+ pid->a * pid->lastdev;
		pid->output  = pid->kp*(pid->err[0]) \
					 + pid->iout +  pid->thisdev;
		pid->lastdev = pid->thisdev;
		
	}
	//输出限幅
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


cali_gimbal_t cali_gimbal_pid = 
{
/**
* @brief yaw pid param     PID_POSITION = 0,PID_DELTA = 1
* @note    
*/
	
	.yaw_pid.position.kp = 21,//30 80 27
	.yaw_pid.position.ki = 0,
	.yaw_pid.position.kd = 30,//2000
	.yaw_pid.position.a = 0,
	.yaw_pid.position.ioutput_max = 1000,
	.yaw_pid.position.output_max = 9999,
	.yaw_pid.position.mode = PID_POSITION,	
	
	.yaw_pid.speed.kp =420,//30
	.yaw_pid.speed.ki = 0,//0.2
	.yaw_pid.speed.kd = 0,
	.yaw_pid.speed.ioutput_max = 1000,
	.yaw_pid.speed.output_max = 20000,
	.yaw_pid.speed.mode = PID_POSITION,	
/**
* @brief pitch pid param         
* @note    
*/
	.pitch_pid.position.kp =26,//26
	.pitch_pid.position.ki = 0,//1.1
	.pitch_pid.position.kd = 20,//20
	.pitch_pid.position.a = 0,
	.pitch_pid.position.ioutput_max = 700,
	.pitch_pid.position.output_max = 10000,
	.pitch_pid.position.mode = PID_POSITION,	

//	.pitch_pid.position.kp =6.591,//40,//25.0,//150
//	.pitch_pid.position.ki = 0,
//	.pitch_pid.position.kd = 61,//60,//0,//750//5000  //6-64
//	.pitch_pid.position.ioutput_max = 1000,
//	.pitch_pid.position.output_max = 9999,
//	.pitch_pid.position.mode = PID_POSITION,
	
	.pitch_pid.speed.kp = 450,//190
	.pitch_pid.speed.ki = 0.9,//0.9
	.pitch_pid.speed.kd = 	0,//209,
	.pitch_pid.speed.ioutput_max = 5500,
	.pitch_pid.speed.output_max = 20000,
	.pitch_pid.speed.mode = PID_POSITION,	


};

cali_shoot_t cali_shoot_pid = 
{
	/**
* @brief trigger pid param         
* @note    
*/
	.trigger_pid.position.kp = 30.0,//30越大，连续发射越快，后面要用时间控制发射速度
	.trigger_pid.position.ki =0,
	.trigger_pid.position.kd = 0,
	.trigger_pid.position.ioutput_max = 1000,
	.trigger_pid.position.output_max = 9999,
	.trigger_pid.position.mode = PID_POSITION,	
	
	.trigger_pid.speed.kp = 100.0,//100决定了响应速度，不能给太小
	.trigger_pid.speed.ki = 0,
	.trigger_pid.speed.kd = 0.0,
	.trigger_pid.speed.ioutput_max = 1000,
	.trigger_pid.speed.output_max = 9999,
	.trigger_pid.speed.mode = PID_POSITION,	
/**
* @brief fric1 pid param         
* @note    
*/	
	.fric1_pid.kp = 20.0,
	.fric1_pid.ki = 0.0,
	.fric1_pid.kd = 0.0,
	.fric1_pid.ioutput_max = 1000,
	.fric1_pid.output_max = 9999,
	.fric1_pid.mode = PID_POSITION,	//PID_DELTA PID_POSITION

/**
* @brief fric2 pid param        
* @note    
*/	
	.fric2_pid.kp = 20.0,
	.fric2_pid.ki = 0.0,
	.fric2_pid.kd = 0.0,
	.fric2_pid.ioutput_max = 1000,
	.fric2_pid.output_max = 9999,
	.fric2_pid.mode = PID_POSITION,	

};

