/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HongYuJia  
 * @Teammate：
 * @Version: V3.0
 * @Date:2020.4.13
 * @Description: 关于发射机构的控制
 * @Note:       
 * @Others: 
**/
#include "shoot_task.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "remote_app.h"
#include "can1_app.h"
#include "can2_app.h"
#include "pid.h"
#include "connect_task.h"
#include "GUI_task.h"
#include "tim.h"
#include "gpio.h"
#include "oled.h"
#include "rule.h"

fric_mode_e fric_mode;
	uint8_t flag1;
shoot_mode_e shoot_mode;
shoot_pid_t shoot_pid;

shoot_control_data_t shoot_control_data;
extern shoot_real_mag shoot_mag;
uint16_t shoot_speed = 0;

uint8_t get_shoot_mode(void)
{
	return shoot_mode;
}
void set_shoot_mode(shoot_mode_e mode)
{
	shoot_mode = mode;
}

void set_fric_mode(fric_mode_e mode)//设置摩擦轮状态
{
	fric_mode = mode;
}
uint8_t get_fric_mode(void)
{
	return fric_mode;
}
/**
  * @brief        获取鼠标上的健值信息，置位相关标志位
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
static void get_mouse_press_msg(shoot_control_data_t *shoot)
{
	//left right singel_click
	if(shoot->rc_ctrl->mouse.last_press_l == 0 && shoot->rc_ctrl->mouse.press_l == 1)
	{
		shoot->mouse_left_single_click_flag = 1;//值是瞬间变化的
	}
	else
	{
		shoot->mouse_left_single_click_flag = 0;
	}
	
	if(shoot->rc_ctrl->mouse.last_press_r == 0 && shoot->rc_ctrl->mouse.press_r == 1)
	{
		shoot->mouse_right_single_click_flag = 1;
	}
	else
	{
		shoot->mouse_right_single_click_flag = 0;
	}
	shoot->rc_ctrl->mouse.last_press_l = shoot->rc_ctrl->mouse.press_l;
	shoot->rc_ctrl->mouse.last_press_r = shoot->rc_ctrl->mouse.press_r;
	
	//left right long_press
	if(shoot->rc_ctrl->mouse.press_l)
	{
		if(shoot->press_left_time < PRESS_LONG_TIME)
		{
			shoot->press_left_time ++;
		}
		else if(shoot->press_left_time == PRESS_LONG_TIME)
		{
			shoot->mouse_left_long_press_flag = 1;
		}
	}
	else
	{
		shoot->press_left_time = 0;
		shoot->mouse_left_long_press_flag = 0;
	}
	
	if(shoot->rc_ctrl->mouse.press_r)
	{
		if(shoot->press_right_time < PRESS_LONG_TIME)
		{
			shoot->press_right_time ++;
		}
		else if(shoot->press_right_time == PRESS_LONG_TIME)
		{
			shoot->mouse_right_long_press_flag = 1;
		}
	}
	else
	{
		shoot->press_right_time = 0;
		shoot->mouse_right_long_press_flag = 0;
	}
}
/**
  * @brief        遥控模拟鼠标控制 使用相同标志位 通过不同的控制状态切换
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
static void get_rc_control_msg(shoot_control_data_t *shoot)
{
	static int16_t rc_ch4;
	static int16_t rc_ch4_last;
	
	rc_ch4 = shoot->rc_ctrl->rc.ch4;
	
	if(rc_ch4_last < RC_CH4_MAX && rc_ch4 >= RC_CH4_MAX)//下左 
	{
		shoot->mouse_left_single_click_flag = 1;
		//shoot->single_bullet_flag=1;
	}		
	else
	{
		shoot->mouse_left_single_click_flag = 0;
	}		
	if(rc_ch4_last > -RC_CH4_MAX && rc_ch4 <= -RC_CH4_MAX)//上右
	{
		shoot->mouse_right_single_click_flag = 1;
	}		
	else
	{
		shoot->mouse_right_single_click_flag = 0;
	}
	rc_ch4_last = rc_ch4;
	
	//left right long_press
	if(rc_ch4 >= RC_CH4_MAX)//下左
	{
		if(shoot->press_left_time < PRESS_LONG_TIME)
		{
			shoot->press_left_time ++;
		}
		else if(shoot->press_left_time == PRESS_LONG_TIME)
		{
			shoot->mouse_left_long_press_flag = 1;
		}
	}
	else
	{
		shoot->press_left_time = 0;
		shoot->mouse_left_long_press_flag = 0;
	}
	
	if(rc_ch4 <= -RC_CH4_MAX)//上右
	{
		if(shoot->press_right_time < PRESS_LONG_TIME)
		{
			shoot->press_right_time ++;
		}
		else if(shoot->press_right_time == PRESS_LONG_TIME)
		{
			shoot->mouse_right_long_press_flag = 1;
		}
	}
	else
	{
		shoot->press_right_time = 0;
		shoot->mouse_right_long_press_flag = 0;
	}
}
/**
  * @brief      根据robot不同的控制方式获取相关标志位，切换或者调试时清空标志位
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
extern _tx2_control_data control_data;
uint8_t last_yaw_sign,vision_shoot,flaga=0;


static void get_shoot_related_flags(shoot_control_data_t *shoot)
{
	static uint8_t control_mode;
	static uint8_t control_mode_last;
	
	control_mode = get_robot_control_mode();
	
	if(control_mode == KEY_MOUSE_MODE)
	{
		get_mouse_press_msg(shoot);
	}
	else if(control_mode == REMOTE_MODE)
	{
		get_rc_control_msg(shoot);
	}
//	if(control_data.yaw_sign==1&&last_yaw_sign==0)
//	vision_shoot=1;
//	else
//	vision_shoot=0;
	if(flaga)
	{
	if(control_data.recog_flag)
	{
	vision_shoot=1;
	//shoot->mouse_right_long_press_flag = 1;
	}
	else
	{
	//shoot->mouse_right_long_press_flag = 0;
		vision_shoot=0;
	}
	}
	last_yaw_sign=control_data.yaw_sign;
	//模式切换 clear related flags 
	if((control_mode_last == KEY_MOUSE_MODE && control_mode == REMOTE_MODE) || \
	   (control_mode_last == REMOTE_MODE && control_mode == KEY_MOUSE_MODE) || \
	   (control_mode == GUI_CALI_MODE) || shoot->shoot_control_flag || shoot->magazine_control_flag ||!control_data.recog_flag)
	{
		shoot->press_right_time = 0;
		shoot->mouse_right_long_press_flag = 0;
		shoot->mouse_right_single_click_flag = 0;
		
		shoot->press_left_time = 0;
		shoot->mouse_left_long_press_flag = 0;
		shoot->mouse_left_single_click_flag = 0;
		
		shoot->shoot_control_flag = 0;
		set_shoot_mode(SHOOT_STOP);//发射关闭
	    set_fric_mode(FRIC_WHEEL_OFF);//摩擦轮关闭
//		BLUELED_ON();
	}
	control_mode_last = control_mode;
}
/**
  * @brief         
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
float shoot_abs(float value)
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
/**
  * @brief         摩擦轮和发射模式更新 
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
static void shoot_mode_update(shoot_control_data_t *shoot, shoot_pid_t *shoot_pid)
{
	//set_fric_mode(FRIC_WHEEL_NORMAL); //测试才开启                                      
	if(shoot->mouse_left_single_click_flag == 1||vision_shoot==1)                                           
	{
		if(get_fric_mode() == FRIC_WHEEL_OFF)//关闭则开启为未稳定模式                        
			set_fric_mode(FRIC_WHEEL_INSTABLE);//测试屏蔽
	}
	
	//摩擦轮状态			
	if(get_fric_mode() == FRIC_WHEEL_NORMAL)
	{
		if(/*(shoot_abs(shoot_pid->fric_1_pid.fdb -  shoot_pid->fric_1_pid.set)  \         
			> FRIC_ALLOW_SPEED_OFFSET) ||								   \
		   (shoot_abs(shoot_pid->fric_2_pid.fdb -  shoot_pid->fric_2_pid.set)  \
		    > FRIC_ALLOW_SPEED_OFFSET)*/0)
		{
			set_fric_mode(FRIC_WHEEL_INSTABLE);//测试屏蔽
//			set_fric_mode(FRIC_WHEEL_NORMAL);//测试才开启默认为稳定
		}

	}
	else if(get_fric_mode() == FRIC_WHEEL_INSTABLE)//这里有些问题，初始化fdb和set都为0 会通过
	{
		if(/*(shoot_abs(shoot_pid->fric_1_pid.fdb - FRIC_MIDDLE_SPEED)         \             
			< FRIC_ALLOW_SPEED_OFFSET) &&									 \
		   (shoot_abs(shoot_pid->fric_2_pid.fdb - shoot_pid->fric_2_pid.set) \
		    < FRIC_ALLOW_SPEED_OFFSET)*/1)
		{
			set_fric_mode(FRIC_WHEEL_NORMAL);
		}		
	}
	
	switch(get_shoot_mode())
	{
		case SHOOT_STOP:
		{	
			if(get_fric_mode() == FRIC_WHEEL_NORMAL)
			{
				set_shoot_mode(SHOOT_READY); 
			}
		}break;
		case SHOOT_READY:
		{
			if((shoot->mouse_left_single_click_flag == 1)||   \
			   (shoot->mouse_right_long_press_flag == 1)||vision_shoot==1 )     //连发或者单反都开启发射模式
			{
				set_shoot_mode(SHOOT_BULLET);
			}

			if(get_fric_mode() == FRIC_WHEEL_INSTABLE)
			{
				set_shoot_mode(SHOOT_STOP);	
			}
		}break;
		case SHOOT_BULLET:
		{
			if(get_fric_mode() == FRIC_WHEEL_INSTABLE)
			{
				set_shoot_mode(SHOOT_STOP);
			}	
		}break;
	}
	
	if(get_shoot_mode() == SHOOT_STOP)//激光指示表示摩擦轮速度正常 可以发射
	{
		//LASER_OFF();
	}
	else
	{
		//LASER_ON();
	}
}
/**
  * @brief        更新将用pid的set和fdb
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
static void shoot_set_and_fdb_update(shoot_control_data_t *shoot)//拨轮电机需要初始化到目标角度
{
	shoot->trigger_position_angle_fdb = (shoot->trigger_motor_msg->encoder.raw_value +      \
										 shoot->trigger_motor_msg->encoder.round_cnt*8192 - \
										 shoot->trigger_motor_msg->encoder.ecd_bias	)*      \
										 TRIGGER_ECD_TO_ANGLE;
	//shoot_bullet
	if(get_shoot_mode() == SHOOT_BULLET)//发射
	{
		if(shoot->single_bullet_flag == 0)
		{
			shoot->trigger_position_angle_set += SINGLE_BULLET_ANGLE_INC;	
			shoot->single_bullet_flag = 1;//设置标志位防止连续加	
		}

		if(shoot_abs(shoot->trigger_position_angle_fdb - shoot->trigger_position_angle_set) < 15.0f)//0.2f
		{
			shoot->single_bullet_flag = 0;
			if(shoot->mouse_right_long_press_flag == 0)//判断是否需要连发
			{
				set_shoot_mode(SHOOT_READY);//不需要连发转为准备模式
				
			}
		}
		else shoot->revolution_bullet_flag=0;
		
		
		
	}
	//fric
	if(get_fric_mode() != FRIC_WHEEL_OFF)
	{
		shoot->fric1_set =  -shoot_mag.allow_fri_speed;
		shoot->fric2_set =  shoot_mag.allow_fri_speed;
		// shoot->fric1_set =  -FRIC_TEXT_SPEED;//测试
		// shoot->fric2_set =  FRIC_TEXT_SPEED;
		
	}
	else
	{
		shoot->fric1_set = 0.0;
		shoot->fric2_set = 0.0;
	}

	shoot->fric1_fdb = (float)shoot->fric_motor1_msg->encoder.filter_rate;
	shoot->fric2_fdb = (float)shoot->fric_motor2_msg->encoder.filter_rate;
}
/**
  * @brief        pid计算
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
uint16_t long_shoot=100;
static void shoot_cascade_pid_calculate(shoot_pid_t *shoot_pid, shoot_control_data_t *shoot)
{
	//trigger pid
	shoot_pid->trigger_pid.position_pid.set = shoot->trigger_position_angle_set;
	//shoot_pid->trigger_pid.position_pid.set =90;
	shoot_pid->trigger_pid.position_pid.fdb = shoot->trigger_position_angle_fdb;
	shoot_pid->trigger_pid.position_pid.Calc(&shoot_pid->trigger_pid.position_pid);
	if(vision_shoot)
	shoot_pid->trigger_pid.speed_pid.set = long_shoot;
	else
	shoot_pid->trigger_pid.speed_pid.set = shoot_pid->trigger_pid.position_pid.output;
	//shoot_pid->trigger_pid.speed_pid.set =500;
	shoot_pid->trigger_pid.speed_pid.fdb = shoot->trigger_motor_msg->encoder.filter_rate;
	shoot_pid->trigger_pid.speed_pid.Calc(&shoot_pid->trigger_pid.speed_pid);
	
	shoot->given_current.trigger_motor = shoot_pid->trigger_pid.speed_pid.output;
	//fric1 pid
//	if(flag1)
//	{
//	shoot_pid->fric_1_pid.set = 0;
//	shoot->single_bullet_flag = 0;
////	flag1=0;
//	}
//	if(shoot->single_bullet_flag)
	shoot_pid->fric_1_pid.set = shoot->fric1_set;
	shoot_pid->fric_1_pid.fdb = shoot->fric1_fdb;
	shoot_pid->fric_1_pid.Calc(&shoot_pid->fric_1_pid);
	
	shoot->given_current.fric1_motor = shoot_pid->fric_1_pid.output;
	//fric2 pid
	shoot_pid->fric_2_pid.set = shoot->fric2_set;
	shoot_pid->fric_2_pid.fdb = shoot->fric2_fdb;
	shoot_pid->fric_2_pid.Calc(&shoot_pid->fric_2_pid);
	
	shoot->given_current.fric2_motor = shoot_pid->fric_2_pid.output;
}
/**
  * @brief         can电流控制循环
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
static void shoot_control_loop(shoot_control_data_t *shoot)
{
			if(get_robot_control_mode() == GUI_CALI_MODE)
				set_shoot_behaviour(0,0,0);
			else
	 		set_shoot_behaviour(shoot->given_current.fric1_motor,    \
							shoot->given_current.fric2_motor,   \
							shoot->given_current.trigger_motor  ); 
	
	//}
}
/**
  * @brief         发射机构pid初始化子函数
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
static void shoot_pid_init(pid_t *pid, cali_pid_t *cali_pid)
{
	pid->kp = cali_pid->kp;
	pid->ki = cali_pid->ki;
	pid->kd = cali_pid->kd;
	
	pid->ioutMax = cali_pid->ioutput_max;
	pid->outputMax = cali_pid->output_max;
	
	pid->mode = cali_pid->mode;
	
	pid->Calc = &PID_Calc;
	pid->Reset =  &PID_Reset;
}
/**
  * @brief         发射机构初始化函数
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
static void shoot_init(shoot_pid_t *shoot_pid, cali_shoot_t *cali_pid, shoot_control_data_t *shoot)
{
	shoot->rc_ctrl = get_rc_data_point();
	shoot->trigger_motor_msg = get_trigger_motor_msg_point();
	shoot->fric_motor1_msg = get_fric_motor1_msg_point();
	shoot->fric_motor2_msg = get_fric_motor2_msg_point();
	
	shoot->trigger_position_angle_set = (shoot->trigger_motor_msg->encoder.ecd_bias)*  \
										 TRIGGER_ECD_TO_ANGLE;//初始化位置
	//trigger 
	shoot_pid_init(&shoot_pid->trigger_pid.position_pid, &cali_pid->trigger_pid.position);
	shoot_pid_init(&shoot_pid->trigger_pid.speed_pid, &cali_pid->trigger_pid.speed);
	//fric
	shoot_pid_init(&shoot_pid->fric_1_pid, &cali_pid->fric1_pid);
	shoot_pid_init(&shoot_pid->fric_2_pid, &cali_pid->fric2_pid);
	
	set_shoot_mode(SHOOT_STOP);
	set_fric_mode(FRIC_WHEEL_OFF);
}
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			发射机构及弹仓开关控制        
  * @note           
  */
void shoot_magazine_control(shoot_control_data_t *shoot)
{
	if(shoot->rc_ctrl->rc.s2 == RC_SW_UP )                 //按键模式   
	{
		if(shoot->rc_ctrl->key.v  & MAGAZINE_CONTROL)      
		{
			shoot->magazine_control_flag = 1; 
		}
		if (shoot->rc_ctrl->key.v & SHOOT_CONTROL)         
		{
			shoot->shoot_control_flag = 1;
		}
		if (shoot->rc_ctrl->key.v & VISION_BUFF)
		{
			shoot->shoot_vision_flag = 1;
		}
		if (shoot->rc_ctrl->key.v & VISION_BUFF_OVER)
		{
			shoot->shoot_vision_flag = 0;
		}
	}
	if(shoot->rc_ctrl->rc.s2 == RC_SW_DOWN )                 //遥控模式    
	{
		if(shoot->rc_ctrl->rc.ch4>= RC_CH4_MAX )	
		{
			shoot->magazine_control_flag = 1; 
		}
		else if(shoot->rc_ctrl->rc.ch4<= -RC_CH4_MAX ) 
		{
			shoot->magazine_control_flag = 0;
		}
	}
	
	if(shoot->magazine_control_flag )
	{
		pwm1_on(3894);
	}
	else 
	{
		pwm1_on(1850);
	}
}

/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
int a = 3000;

void shoot_task(void *argument)
{
	
	TickType_t current_time = 0;
	
	vTaskDelay(SHOOT_TASK_INIT_TIME);
	shoot_init(&shoot_pid, &cali_shoot_pid, &shoot_control_data);
	while(1)
	{	
		current_time = xTaskGetTickCount();                     // 当前系统时间 
		shoot_magazine_control(&shoot_control_data);			// 发射机构控制  
		shoot_limit();                                          // 规则限制
		
		get_shoot_related_flags(&shoot_control_data);
		shoot_mode_update(&shoot_control_data, &shoot_pid);
		shoot_set_and_fdb_update(&shoot_control_data);
		shoot_cascade_pid_calculate(&shoot_pid, &shoot_control_data);
		shoot_control_loop(&shoot_control_data);                    
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,a);//4150//2200
		vTaskDelayUntil(&current_time, SHOOT_TASK_TIME_1MS);        //1ms一次       *hyj
	}	
}

