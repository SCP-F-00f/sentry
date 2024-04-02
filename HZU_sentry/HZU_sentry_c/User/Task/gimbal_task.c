/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HongYuJia  
 * @Teammate：
 * @Version: V3.0
 * @Date:2020.4.13
 * @Description: 关于云台的控制
 * @Note:       
 * @Others: 
**/
#include "gimbal_task.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "remote_app.h"
#include "can1_app.h"
#include "pid.h"
#include "INS_task.h"
#include "connect_task.h"
#include "GUI_task.h"
#include "monitor_task.h"
#include "tim.h"
#include "gpio.h"
#include "oled.h"
#include "start_task.h"
#include "gimbal_task.h"
#include "can2_app.h"
#include "flash.h"
#include "usart.h"
#include "shoot_task.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"

#include "stm32f4xx_hal_cortex.h"

#define Logistic( X, K, X0, L)	((L)/(1+pow(2.718282f,-(K)*((X)-(X0))))+0.0002)

uint8_t view_control_flag=0,last_work_mode;
float gimbal_yaw_set1,gimbal_yaw_fdb1,yaw_delta,out,yaw_sensit;
extern uint8_t gimbal_init_ok_flag;
extern float value;
extern uint8 getflag;
extern _tx2_control_data control_data;
extern TaskHandle_t INS_Task_Handler;
extern monitor_t monitor;

gimbal_pid_t gimbal_pid;
gimbal_control_data_t gimbal_control_data;
robot_work_mode_e robot_work_mode;
robot_control_mode_e robot_control_mode = 2;
gimbal_work_mode_e gimbal_work_mode;
extern shoot_control_data_t shoot_control_data;

extern ext_Judge_data_t Judge_data;
extern shoot_control_data_t shoot_control_data;
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
float gimbal_abs(float value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}			

float forwardfeed(float in)
{
out = 1025 * gimbal_abs(in) - 7.114;
if(in<0)
 out = - out;
return out;

}	

//机器人工作模式
void set_robot_control_mode(robot_control_mode_e mode)
{
	robot_control_mode = mode;
}
uint8_t get_robot_control_mode(void)
{
	return robot_control_mode;
}
void robot_control_mode_update(RC_ctrl_t *rc_s)	
{
	//control mode
	switch(rc_s->rc.s2) 
	{
		case RC_SW_UP:
			set_robot_control_mode(KEY_MOUSE_MODE);break;
		case RC_SW_MID:
			set_robot_control_mode(REMOTE_MODE);break;
		case RC_SW_DOWN:
			set_robot_control_mode(GUI_CALI_MODE);break;
	}
//	 if(monitor.exist_error_flag == 1)//发生严重错误时，不受遥控指挥强行转为调试模式
//	 {
//	 	set_robot_control_mode(GUI_CALI_MODE);
//	 }
}
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
//uint8_t start_time_flag = 0;
//int32_t start_time_us = 0;
//int32_t current_time_us = 0;
uint32_t time_tick_ms = 0;
uint8_t gimbal_position_init_finish_flag = 0;

//robot_work_mode
void set_robot_work_mode(robot_work_mode_e mode)
{
	robot_work_mode = mode;
}
uint8_t get_robot_work_mode(void)
{
	return robot_work_mode;
}
void robot_work_mode_update(RC_ctrl_t *rc_s)
{
	//work mode
	if(gimbal_position_init_finish_flag == 0 && get_robot_control_mode() != GUI_CALI_MODE)
	{
		set_robot_work_mode(ROBOT_INIT_MODE);	//初始化	
		gimbal_position_init_finish_flag = 1;
	}
	else if(get_robot_control_mode() == REMOTE_MODE)	//遥控器控制工作模式
	{
		switch(rc_s->rc.s1)
		{
			case RC_SW_UP:
				set_robot_work_mode(ROBOT_ROTATE_STOP_MODE);break;
			case RC_SW_MID:
				set_robot_work_mode(ROBOT_ROTATE_MOTION_MODE);break;
			case RC_SW_DOWN:
				set_robot_work_mode(ROBOT_COMMON_MODE);break;
		}
	}
	else if(get_robot_control_mode() == KEY_MOUSE_MODE)	//键鼠控制工作模式EQV
	{
		if(rc_s->key.v & ROBOT_ROTATE_STOP_MODE_KEY)
		{
			set_robot_work_mode(ROBOT_ROTATE_STOP_MODE);
		}
		else if(rc_s->key.v & ROBOT_ROTATE_MOTION_MODE_KEY)
		{
			set_robot_work_mode(ROBOT_ROTATE_MOTION_MODE);
		}
		else if(rc_s->key.v & ROBOT_COMMON_MODE_KEY)
		{
			set_robot_work_mode(ROBOT_COMMON_MODE);
			shoot_control_data.magazine_control_flag = 0; 
		}
	}
	else if(get_robot_control_mode() == GUI_CALI_MODE)
	{
		set_robot_work_mode(ROBOT_CALI_MODE);	//调试模式
		gimbal_position_init_finish_flag = 0;
	}	
	
	if(get_robot_work_mode() == ROBOT_INIT_MODE)
	{
		GREEDLED_ON();
	}
	else 
	{
		GREEDLED_OFF();
	}
}	
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
uint8_t stop_rotate_flag=0,last_rotate_flag,work_mode_change,rotate_flag=1;

//云台工作模式
void set_gimbal_work_mode(gimbal_work_mode_e mode)
{
	gimbal_work_mode = mode;
}
uint8_t get_gimbal_work_mode(void)
{
	return gimbal_work_mode;
}

uint8_t work_mode;
void gimbal_work_mode_update(RC_ctrl_t *rc_s, gimbal_control_data_t *gimbal_control_data)
{
	
	robot_control_mode_update(rc_s);
	robot_work_mode_update(rc_s);
	
	work_mode = get_robot_work_mode();	
	if((last_rotate_flag==1)&&((work_mode!=ROBOT_ROTATE_MOTION_MODE)||(work_mode!=ROBOT_ROTATE_STOP_MODE)))
	{
		stop_rotate_flag = 1;
		last_rotate_flag = 0;
	}
	
	if(get_robot_work_mode() == ROBOT_CALI_MODE)
	{
		set_gimbal_work_mode(GIMBAL_CALI_MODE);
	}
	else if(get_robot_work_mode() == ROBOT_INIT_MODE)
	{
		set_gimbal_work_mode(GIMBAL_RELATIVE_ANGLE_MODE);		
	}
	else if(get_robot_work_mode() == ROBOT_ROTATE_STOP_MODE)
	{
		work_mode_change=1;
		last_rotate_flag = 1;
		set_gimbal_work_mode(GIMBAL_ROTATE_MODE);		
	}
	
	else 
	{
		set_gimbal_work_mode(GIMBAL_ABSOLUTE_ANGLE_MODE);		
	}
	last_work_mode=get_robot_work_mode();
	
	switch(get_gimbal_work_mode())
	{
		case GIMBAL_RELATIVE_ANGLE_MODE:         //编码值   *hyj
		{
			gimbal_control_data->yaw_motor_fdb_mode = GIMBAL_MOTOR_ENCONDE;
			gimbal_control_data->pitch_motor_fdb_mode = GIMBAL_MOTOR_ENCONDE;
		}break;
		case GIMBAL_ABSOLUTE_ANGLE_MODE:         //陀螺仪   *hyj
		{
			gimbal_control_data->yaw_motor_fdb_mode = GIMBAL_MOTOR_GYRO;
			gimbal_control_data->pitch_motor_fdb_mode = GIMBAL_MOTOR_GYRO;
		}break;
	}
}	
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
#define MOUSE_SINGLE_TIME_ALLOW_VALUE  38   //150
void gimbal_val_limit(int16_t *val, int16_t min, int16_t max)
{
	if(*val < min)
	{
		*val = min;
	}
	else if(*val > max)
	{
		*val = max;
	}
}
void mouse_sensit_cali(RC_ctrl_t *rc_ctrl)
{
	if( rc_ctrl->mouse.x < 20)
	{
		rc_ctrl->yaw_sensit = GAMBAL_MOUSE_YAW_MIN_ANGLE_SET_FACT;
	}
	else
	{
		rc_ctrl->yaw_sensit = GAMBAL_MOUSE_YAW_MAX_ANGLE_SET_FACT;
	}
	
	if ( rc_ctrl->mouse.y < 20)
	{
		rc_ctrl->pitch_sensit = GAMBAL_MOUSE_PITCH_MIN_ANGLE_SET_FACT;
	}
	else
	{
		rc_ctrl->pitch_sensit = GAMBAL_MOUSE_PITCH_MAX_ANGLE_SET_FACT;
	}
}


float abs_fun(float a)
{
	if(a < 0) a = -a;
	return a;
}

float vision_K = 1;
int ROBOT_ROTATE_MOTION_MODE_flag;

pid_t delta_yaw= 
{
	.kp = 0.05,//0.05
	.ki = 0.0008,//0.008,
	.kd = 0,
	.ioutMax = 1000,
	.outputMax = 5000,
	.mode = PID_POSITION,
	.Calc = &PID_Calc,
	.Reset =  &PID_Reset,
};
pid_t delta_pitch= 
{
	.kp = 0.65,
	.ki = 0.006,
	.kd = 10,
	.ioutMax = 30,
	.outputMax = 500,
	.mode = PID_POSITION,
	.Calc = &PID_Calc,
	.Reset =  &PID_Reset,
};

pid_t vision_pitch_angle = 
{
	.kp = 0.01,
	.ki = 0,
	.kd = 0,
	.a = 0.95, //不完全微分系数
	.ioutMax = 2000,
	.outputMax = 10000,
	.mode = PID_POSITION,
	.Calc = &PID_Calc,
	.Reset =  &PID_Reset,
};

pid_t vision_pitch_speed = 
{
	.kp = 10,
	.ki = 0,
	.kd = 0,
	.ioutMax = 1000,
	.outputMax = 20000,
	.mode = PID_POSITION,
	.Calc = &PID_Calc,
	.Reset =  &PID_Reset,
};
pid_t vision_yaw_angle = 
{
	.kp = 12,
	.ki = 0,
	.kd = 500,
	.a = 0,  ////不完全微分系数
	.ioutMax = 2000,
	.outputMax = 10000,
	.mode = PID_POSITION,
	.Calc = &PID_Calc,
	.Reset =  &PID_Reset,
};

pid_t vision_yaw_speed = 
{
	.kp = 120,
	.ki = 0,
	.kd = 0,
	.ioutMax = 1000,
	.outputMax = 20000,
	.mode = PID_POSITION,
	.Calc = &PID_Calc,
	.Reset =  &PID_Reset,
};

uint16_t WINDOW_SIZE=20;
float slidingWindowFilter(int input) {
		int window[50] = {0};
    int sum = 0;
    int output;
    sum -= window[0];
    for (int i = 0; i < WINDOW_SIZE - 1; i++) {
        window[i] = window[i + 1];
    }
    window[WINDOW_SIZE - 1] = input;
    sum += input;

    output = sum / WINDOW_SIZE;
		return output;
}

float tt,PITCH,delta_yaw_dev,delta_pitch_dev,delta_pitch_dev_y1=-1.0,delta_pitch_dev_y2=3.0,gyroy_aver;
extern float yaw_cnt;
uint8_t vision_flag=0,rotate_break_flag;

void gimbal_set_and_fdb_update(gimbal_control_data_t *gimbal_control_data,
							   robot_control_mode_e robot_control_mode,							   
							   _tx2_control_data control_data  )
{

			gyroy_aver=slidingWindowFilter(gimbal_abs(gimbal_control_data->gimbal_INS->gyro_y));
	    if(gimbal_control_data->pitch_motor_fdb_mode == GIMBAL_MOTOR_GYRO)//mpu
	{
				if(stop_rotate_flag)
			 {
			 yaw_cnt=0;
			 stop_rotate_flag=0;
			 }
			if(robot_control_mode == GUI_CALI_MODE)
		{
			delta_yaw.iout=0;
			delta_pitch.iout=0;	
		}
			else if(robot_control_mode == KEY_MOUSE_MODE) //KEY_MOUSE_MODE
		{
			if(gimbal_work_mode == GIMBAL_ROTATE_MODE&&!rotate_break_flag)
	{
       //gimbal_control_data->gimbal_pitch_set=40 * sin( tt * PI/180);
			gimbal_control_data->gimbal_pitch_set=0;
       tt+=0.07;
       if(tt==180)
       tt=0;
       gimbal_control_data->gimbal_yaw_fdb = gimbal_control_data->gimbal_INS->yaw_angle;
       gimbal_control_data->gimbal_pitch_fdb = gimbal_control_data->gimbal_INS->pitch_angle;	
			 if(control_data.recog_flag==1 )
				 rotate_break_flag=1;
			 
	}	
			else if(control_data.recog_flag==1 )
			{
				delta_pitch_dev=(control_data.Target_distance-5000.0)/4000.0*5.0+3.0;//y=(x-x2)/(x1-x2)*(y1-y2)+y2  (1000,-2) (5000,3)
				delta_yaw.set=0;
				if(control_data.yaw_sign)
					delta_yaw.fdb=control_data.yaw_dev-delta_yaw_dev;
				else
					delta_yaw.fdb=-control_data.yaw_dev-delta_yaw_dev;
				delta_yaw.Calc(&delta_yaw);
				gimbal_control_data->gimbal_yaw_set += delta_yaw.output;
				
				delta_pitch.set=0;
				if(control_data.pitch_sign)
					delta_pitch.fdb=control_data.pitch_dev-delta_pitch_dev;
				else
					delta_pitch.fdb=-control_data.pitch_dev-delta_pitch_dev;
				delta_pitch.Calc(&delta_pitch);
				gimbal_control_data->gimbal_pitch_set = delta_pitch.output;
				control_data.recog_flag=0;
				vision_flag=1;
				if(!control_data.recog_flag)
				 rotate_break_flag=0;
			}
			else
			{
				gimbal_val_limit(&gimbal_control_data->rc_ctrl->mouse.x, \
								-MOUSE_SINGLE_TIME_ALLOW_VALUE,          \
								MOUSE_SINGLE_TIME_ALLOW_VALUE);
				gimbal_val_limit(&gimbal_control_data->rc_ctrl->mouse.y, \
								-MOUSE_SINGLE_TIME_ALLOW_VALUE,          \
								MOUSE_SINGLE_TIME_ALLOW_VALUE);
				mouse_sensit_cali(gimbal_control_data->rc_ctrl);

				gimbal_control_data->gimbal_yaw_set += (gimbal_control_data->rc_ctrl->mouse.x) *   gimbal_control_data->rc_ctrl->yaw_sensit;
				gimbal_control_data->gimbal_pitch_set += (gimbal_control_data->rc_ctrl->mouse.y) * gimbal_control_data->rc_ctrl->pitch_sensit;
			}
		}
		else if(robot_control_mode == REMOTE_MODE) //REMOTE_MODE		调试视觉用
		 //else if(robot_control_mode == REMOTE_MODE ) //REMOTE_MODE
		{
			//视觉处理     *hyj
				if(gimbal_work_mode == GIMBAL_ROTATE_MODE&&(rotate_break_flag==0))
	{
       //gimbal_control_data->gimbal_pitch_set=40 * sin( tt * PI/180);
			gimbal_control_data->gimbal_pitch_set=0;
       tt+=0.07;
       if(tt==180)
       tt=0;
       gimbal_control_data->gimbal_yaw_fdb = gimbal_control_data->gimbal_INS->yaw_angle;
       gimbal_control_data->gimbal_pitch_fdb = gimbal_control_data->gimbal_INS->pitch_angle;	
			 if(control_data.recog_flag==1 )
			 {
				 rotate_break_flag=1;
				 set_gimbal_work_mode(GIMBAL_ABSOLUTE_ANGLE_MODE);	
			 }
			 
	}	
			else if(control_data.recog_flag==1 )
			{
				gyroy_aver=slidingWindowFilter(gimbal_abs(gimbal_control_data->gimbal_INS->gyro_y));
				//delta_yaw_dev=(gyroy_aver)//y=(x-x2)/(x1-x2)*(y1-y2)+y2 
				delta_pitch_dev=-(control_data.Target_distance-5000.0)/4000.0*(delta_pitch_dev_y1-delta_pitch_dev_y2)+delta_pitch_dev_y2;//y=(x-x2)/(x1-x2)*(y1-y2)+y2  (1000,-1) (5000,3)
				delta_yaw.set=0;
				if(control_data.yaw_sign)
					delta_yaw.fdb=control_data.yaw_dev-2+delta_yaw_dev;
				else
					delta_yaw.fdb=-control_data.yaw_dev-2-delta_yaw_dev;
				delta_yaw.Calc(&delta_yaw);
				gimbal_control_data->gimbal_yaw_set = delta_yaw.output;
				
				delta_pitch.set=0;
				if(control_data.pitch_sign)
					delta_pitch.fdb=control_data.pitch_dev-delta_pitch_dev;
				else
					delta_pitch.fdb=-control_data.pitch_dev-delta_pitch_dev;
				delta_pitch.Calc(&delta_pitch);
				gimbal_control_data->gimbal_pitch_set = delta_pitch.output;
				control_data.recog_flag=0;
				vision_flag=1;
			}
			

			
			else
			{
				//hzp
				yaw_sensit = Logistic(gimbal_abs(gimbal_control_data->rc_ctrl->rc.ch3*0.0182),1,6,0.00015);
				
				gimbal_control_data->gimbal_yaw_set += (-gimbal_control_data->rc_ctrl->rc.ch0) *    \
													 yaw_sensit;				
				//gimbal_control_data->gimbal_yaw_set += 0.005*value;
				gimbal_control_data->gimbal_pitch_set += (-gimbal_control_data->rc_ctrl->rc.ch1) *  \
													 GAMBAL_PITCH_MAX_ANGLE_SET_FACT;//ch1
			}
		}	
		if(!control_data.recog_flag)
				{
				 rotate_break_flag=0;
				}
		gimbal_control_data->gimbal_yaw_fdb = gimbal_control_data->gimbal_INS->yaw_angle;
		yaw_delta = gimbal_control_data->gimbal_yaw_fdb - gimbal_yaw_fdb1;
		gimbal_yaw_fdb1 = gimbal_control_data->gimbal_yaw_fdb;
		gimbal_control_data->gimbal_pitch_fdb = gimbal_control_data->gimbal_INS->pitch_angle;	
	if(gimbal_control_data->gimbal_pitch_set>18)
	{
		gimbal_control_data->gimbal_pitch_set=18;
	}
	else if(gimbal_control_data->gimbal_pitch_set<-25)
	{
		gimbal_control_data->gimbal_pitch_set=-25;
	}
		else
		gimbal_control_data->gimbal_pitch_set=gimbal_control_data->gimbal_pitch_set;
	}
	else if(gimbal_control_data->pitch_motor_fdb_mode == GIMBAL_MOTOR_ENCONDE)//初始化时的编码模式 set 可修改
	{
		//yaw
		gimbal_control_data->gimbal_yaw_set =  gimbal_control_data->gimbal_INS->yaw_angle\
		+((float)(GAMBAL_YAW_INIT_ENCODE_VALUE-gimbal_control_data->gimbal_yaw_motor_msg->encoder.raw_value))/GAMBAL_YAW_angle_VALUE;
		gimbal_control_data->gimbal_yaw_fdb = gimbal_control_data->gimbal_INS->yaw_angle;	
		//pitch
		gimbal_control_data->gimbal_pitch_set = gimbal_control_data->gimbal_INS->pitch_angle\
		-((float)(GAMBAL_PITCH_INIT_ENCODE_VALUE-gimbal_control_data->gimbal_pitch_motor_msg->encoder.raw_value))/GAMBAL_PITCH_angle_VALUE;	
		//gimbal_control_data->gimbal_pitch_set =  0;
		gimbal_control_data->gimbal_pitch_fdb = gimbal_control_data->gimbal_INS->pitch_angle;
	if(gimbal_control_data->gimbal_pitch_set>18)
	{
		gimbal_control_data->gimbal_pitch_set=18;
	}
	else if(gimbal_control_data->gimbal_pitch_set<-25)
	{
		gimbal_control_data->gimbal_pitch_set=-25;
	}
		else
		gimbal_control_data->gimbal_pitch_set=gimbal_control_data->gimbal_pitch_set;
	}

}


/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
float rotate_speed_angle;

void gimbal_cascade_pid_calculate(gimbal_pid_t *gimbal_pid,       \
					              gimbal_control_data_t *gimbal_control_data)
{
	
	//yaw轴角度pid计算
		if(gimbal_work_mode == GIMBAL_ROTATE_MODE&&!rotate_break_flag)
	{
	gimbal_pid->yaw_pid.speed_pid.set = -150;	
	gimbal_pid->yaw_pid.speed_pid.fdb = -gimbal_control_data->gimbal_INS->gyro_y;
	}
	else
	{
	//gimbal_pid->yaw_pid.position_pid.set = gimbal_control_data->gimbal_yaw_set;
	gimbal_pid->yaw_pid.position_pid.set = gimbal_control_data->gimbal_yaw_set;
	gimbal_pid->yaw_pid.position_pid.fdb = gimbal_control_data->gimbal_yaw_fdb;
	gimbal_pid->yaw_pid.position_pid.Calc(&gimbal_pid->yaw_pid.position_pid);
	
	//yaw轴速度pid计算
	//gimbal_pid->yaw_pid.speed_pid.set = 0 ;
	if(robot_control_mode == KEY_MOUSE_MODE)
	{
		rotate_speed_angle = control_data.rotate_speed*180/PI;
		gimbal_pid->yaw_pid.speed_pid.set = rotate_speed_angle;
		gimbal_pid->yaw_pid.speed_pid.fdb = -gimbal_control_data->gimbal_INS->gyro_y;
		}
		else
		{
			gimbal_pid->yaw_pid.speed_pid.set = -gimbal_pid->yaw_pid.position_pid.output;	
			gimbal_pid->yaw_pid.speed_pid.fdb = -gimbal_control_data->gimbal_INS->gyro_y;
		}
	}
	gimbal_pid->yaw_pid.speed_pid.Calc(&gimbal_pid->yaw_pid.speed_pid);
	
	//pitch轴角度pid计算
	gimbal_pid->pitch_pid.position_pid.set = -gimbal_control_data->gimbal_pitch_set;
	gimbal_pid->pitch_pid.position_pid.fdb = -gimbal_control_data->gimbal_pitch_fdb; 
	gimbal_pid->pitch_pid.position_pid.Calc(&gimbal_pid->pitch_pid.position_pid);
	//pitch轴速度pid计算
	gimbal_pid->pitch_pid.speed_pid.set = gimbal_pid->pitch_pid.position_pid.output;
	//gimbal_pid->pitch_pid.speed_pid.set = 0;
	gimbal_pid->pitch_pid.speed_pid.fdb = -gimbal_control_data->gimbal_INS->gyro_z;
	gimbal_pid->pitch_pid.speed_pid.Calc(&gimbal_pid->pitch_pid.speed_pid);	
	
}
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void gimbal_control_loop(gimbal_pid_t *gimbal_pid,       \
					     gimbal_control_data_t *gimbal_control_data)
{
	//这里改为了正值

	gimbal_control_data->given_current.yaw_motor = -gimbal_pid->yaw_pid.speed_pid.output;//g6020反向安装需要加负号
	gimbal_control_data->given_current.pitch_motor = gimbal_pid->pitch_pid.speed_pid.output;
	//gimbal_control_data->given_current.yaw_motor = 0;
	//gimbal_control_data->given_current.pitch_motor = 0;

	if(get_robot_control_mode() == GUI_CALI_MODE )//if(get_gimbal_work_mode() == GIMBAL_CALI_MODE)
	{
		set_gimbal_stop();
	}
	else 
	{
		set_gimbal_behaviour(gimbal_control_data->given_current.yaw_motor,   \
							gimbal_control_data->given_current.pitch_motor); 
	}
	
}

/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void gimbal_pid_init(pid_t *pid, cali_pid_t *cali_pid)
{
	pid->kp = cali_pid->kp;
	pid->ki = cali_pid->ki;
	pid->kd = cali_pid->kd;
	pid->a = cali_pid->a;
	
	pid->ioutMax = cali_pid->ioutput_max;
	pid->outputMax = cali_pid->output_max;
	
	pid->mode = cali_pid->mode;
	
	pid->Calc = &PID_Calc;
	pid->Reset =  &PID_Reset;
}
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void gimbal_init(gimbal_pid_t *gimbal_pid,   \
				 cali_gimbal_t *cali_pid,    \
				 gimbal_control_data_t *gimbal_control_data)
{
	gimbal_control_data->rc_ctrl = get_rc_data_point();
	gimbal_control_data->gimbal_INS = get_INS_point();
	gimbal_control_data->gimbal_yaw_motor_msg = get_yaw_motor_msg_point();
	gimbal_control_data->gimbal_pitch_motor_msg = get_pitch_motor_msg_point();
	//yaw cascade pid
	gimbal_pid_init(&gimbal_pid->yaw_pid.position_pid, &cali_pid->yaw_pid.position);
	gimbal_pid_init(&gimbal_pid->yaw_pid.speed_pid, &cali_pid->yaw_pid.speed);
	//pitch cascade pid
	gimbal_pid_init(&gimbal_pid->pitch_pid.position_pid, &cali_pid->pitch_pid.position);
	gimbal_pid_init(&gimbal_pid->pitch_pid.speed_pid, &cali_pid->pitch_pid.speed);
	//GIMBAL_InitArgument();

	set_robot_control_mode(GUI_CALI_MODE);
	set_robot_work_mode(ROBOT_CALI_MODE);
	set_gimbal_work_mode(GIMBAL_CALI_MODE);

}
/**
  * @brief        运行时挂起GUI任务
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */


/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note  		1.gimbal_task 跑不了oled程序 md不知道什么问题  
				2.while循环的前面两个函数消耗4%(其中第一个0.3%) 整个while消耗15%
  */
void gimbal_task(void *argument)
{
	TickType_t current_time = 0;
	
	vTaskDelay(GIMBAL_TASK_INIT_TIME);
	gimbal_init(&gimbal_pid, &cali_gimbal_pid, &gimbal_control_data);
	while(1)
	{		
		current_time = xTaskGetTickCount();                         //当前系统时间       *hyj
		send_gyro_data_to_chassis();
		gimbal_work_mode_update(&rc_ctrl_data, &gimbal_control_data);//更新云台状态
		gimbal_set_and_fdb_update(&gimbal_control_data, robot_control_mode, control_data );//set fdb数据更新
		
		gimbal_cascade_pid_calculate(&gimbal_pid, &gimbal_control_data);//串级pid计算
		gimbal_control_loop(&gimbal_pid, &gimbal_control_data);//控制循环
		vTaskDelayUntil(&current_time, GIMBAL_TASK_TIME_1MS);       //1ms一次         *hyj     
	}	
}
