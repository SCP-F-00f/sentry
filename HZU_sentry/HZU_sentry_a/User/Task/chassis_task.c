/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HongYuJia 
 * @Teammate：
 * @Version: V3.0
 * @Date:2021.4.13
 * @Description:   关于底盘的控制
 * @Note:       
 * @Others: 
**/
#include "chassis_task.h"
#include "pid.h"
#include "can1_app.h"
#include "can2_app.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "pid.h"
#include "math.h"
#include "rule.h"
#include "connect_task.h"
#include "main.h"
#include "oled.h"
#include "stdio.h"
#include "stdlib.h"
//#include "time.h"
#include "ULTRASONIC_task.h"

chassis_control_data_t chassis_control_data;
chassis_pid_t chassis_pid;

float last_in=0,ch2,ch3;
float T=0.001,rotate_speed=0.07;
float out,chassis_value,delta_init_value,last_delta,delta,yaw_set,yaw_fdb;
int yaw_raw=7045,yaw_rotate_set=500,yaw_raw1,k=1,delta_value;
uint8_t rotate_init_flag=1,last_mode,delta_start_flag;



volatile uint16_t T1;
volatile int tim;

car_typedef car=
{
	.position_a[0] = 0,
	.position_a[1] = 0,
	.init_width = (uint8_t)(60/map_divide),
	.target[0] = 144,
	.target[1] = 108
};

//uint32_t RNG_Get_RandomRange(int min,int max)

//{

//return HAL_RNG_GetRandomNumber(&RNG_Handler)%(max-min+1) +min;

//}

float forwardfeed(float in)
{
//	int k1=2000000000,k2=491200000;
// out=k1*(in-last_in)/T-k2*in;
//	if((in<70)&&(in>-70))
//		out=0;
//	else
//	out=0.7*(0.05*in-79.36);
	//last_in=in;
	if(in<17)
		out=0;
	else if(in>=17&&in<103)
		out=(double)0.0778*in+562.3;
	else if(in>=103&&in<157)
		out=(double)0.1319*in+559.8;
	else if(in>=157&&in<193)
		out=(double)0.0804*in+567.7;
	else if(in>=193&&in<232)
		out=(double)0.0553*in+572.5;
	else if(in>=232&&in<275)
		out=(double)0.0602*in+571.5;
	else if(in>=275)
		out=(double)0.0399*in+575.5;

	return out;
}
void odometer_delta(chassis_control_data_t *chassis)
{
	if(chassis->connect->can2_rc_ctrl.control_mode==1&&last_mode==2)
	{
		delta_init_value = chassis->connect->can2_rc_ctrl.gyro.yaw_fdb;
		delta_start_flag=1;
	}
last_mode = chassis->connect->can2_rc_ctrl.control_mode;
if(delta_start_flag)
delta_value=RC_abs(chassis->connect->can2_rc_ctrl.gyro.yaw_fdb-delta_init_value);

//	if(delta>=90)
//	{
//	if(delta_value%90==0)
//		delta=0;
		delta=delta_value-k*90;

//	}
	
	//if((delta==0&&last_delta<0))
		if(delta<0)
	{
		k--;
		}
	if(delta==90)
		k++;
//	if(delta>0&&last_delta==0)
	last_delta=delta;
	yaw_set=chassis->connect->can2_rc_ctrl.gyro.yaw_set;
	yaw_fdb=chassis->connect->can2_rc_ctrl.gyro.yaw_fdb;
}

int16_t RC_abs(int16_t value)
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
  * @brief        	底盘pid初始化
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void chassis_pid_init(pid_t *pid, cali_pid_t *cali_pid)
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
  * @brief          底盘初始化
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void chassis_init(chassis_control_data_t *chassis, chassis_pid_t *chassis_pid)
{
	chassis->connect = get_connect_data_point();
	chassis->cm1_msg = get_cm1_msg_point();
	chassis->cm2_msg = get_cm2_msg_point();
	chassis->cm3_msg = get_cm3_msg_point();
	chassis->cm4_msg = get_cm4_msg_point();
	chassis->yaw_motor_msg = get_yaw_motor_msg_point();
	
	chassis_pid_init(&chassis_pid->cm1_pid, &cali_chassis_pid.cm_pid);
	chassis_pid_init(&chassis_pid->cm2_pid, &cali_chassis_pid.cm_pid);
	chassis_pid_init(&chassis_pid->cm3_pid, &cali_chassis_pid.cm_pid);
	chassis_pid_init(&chassis_pid->cm4_pid, &cali_chassis_pid.cm_pid);
	
	chassis_pid_init(&chassis_pid->rotate_pid, &cali_chassis_pid.rotate_pid);
}
/**
  * @brief        小陀螺下的运动解算 
  * @author         
  * @param[in]      
  * @retval			
  * @note        //前面会出现负号，是因为6020是反向安装的，也可根据实际调试得到 
  */
void rotate_motion_mode_process(chassis_control_data_t *chassis)
{
	chassis->rotate_motion.yaw_current_ecd = chassis->yaw_motor_msg->encoder.raw_value;
	if(chassis->chassis_control_mode_flag)
	{
		chassis->rotate_motion.yaw_init_ecd = GAMBAL_YAW_INIT_ENCODE_VALUE_RHOMB;
	}
	else
	{
		chassis->rotate_motion.yaw_init_ecd = GAMBAL_YAW_INIT_ENCODE_VALUE_COMMON;
	}
	//得到从中值沿逆时针方向0到360度变化的角度
	if(chassis->rotate_motion.yaw_current_ecd <= chassis->rotate_motion.yaw_init_ecd)
	{
		chassis->rotate_motion.											   \
		chassis_gimbal_angle = (float)(chassis->rotate_motion.yaw_init_ecd \
							    - chassis->rotate_motion.yaw_current_ecd)  \
								* GAMBAL_ENCODE_TO_ANGLE;
	}
	else if(chassis->rotate_motion.yaw_current_ecd > chassis->rotate_motion.yaw_init_ecd)
	{
		chassis->rotate_motion.   													   \
		chassis_gimbal_angle = 360.0f - (float)(chassis->rotate_motion.yaw_current_ecd \
										 - chassis->rotate_motion.yaw_init_ecd)        \
										 * GAMBAL_ENCODE_TO_ANGLE;
	}
#if 1
	chassis->forward_back_set = (int16_t)                                                         \
								((float)chassis->forward_back *  								  \
								 cos(chassis->rotate_motion.chassis_gimbal_angle / 180.0f * PI) + \
							    (float)chassis->left_right * 									  \
								(-sin(chassis->rotate_motion.chassis_gimbal_angle / 180.0f * PI)));
	
	chassis->left_right_set =   (int16_t)                                                         \
								((float)chassis->forward_back *  								  \
								 sin(chassis->rotate_motion.chassis_gimbal_angle / 180.0f * PI) + \
							    (float)chassis->left_right * 									  \
								 cos(chassis->rotate_motion.chassis_gimbal_angle / 180.0f * PI));
#else
	//forward_and_left test
	chassis->forward_back_set = (int16_t)                                                        \
								((float)chassis->forward_back *  								 \
								 cos(chassis->rotate_motion.chassis_gimbal_angle / 180.0f * PI));
	chassis->left_right_set =   (int16_t)                                                        \
								((float)chassis->forward_back *  								 \
								 sin(chassis->rotate_motion.chassis_gimbal_angle / 180.0f * PI));
								
	//left_and_right test
	chassis->forward_back_set =  (int16_t)
								 ((float)chassis->left_right * 									 \
								 (-sin(chassis->rotate_motion.chassis_gimbal_angle / 180.0f * PI)));
	chassis->left_right_set = 	 (int16_t)
								 ((float)chassis->left_right * 									 \
								  cos(chassis->rotate_motion.chassis_gimbal_angle / 180.0f * PI));
#endif
	
}
/**
  * @brief        获取移动控制量
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */

uint16_t time1,time2;
uint16_t speed = 0,NORMAL_SPPED=700,SPORT_ROTATE_DECREASE=100;
float num1=0.7,speed1,num2=1.2;
uint8_t close_flag,last_close_flag,move_close_flag;
float speed_factor1,speed_factor2,speed_factor3;

void get_forward_back_value(chassis_control_data_t *chassis)
{
	
//	int16_t speed = 0;

	if(chassis->connect->can2_rc_ctrl.mouse.key & CHASSIS_HIGH_SPEED_KEY)
	{
		
		speed = NORMAL_SPPED * 2;

	}
	if(move_close_flag)
	{
		speed1 -= num2;
		speed = speed1;
		if(speed==0)
		move_close_flag=0;
	}
	if(chassis->connect->can2_rc_ctrl.mouse.key & ((CHASSIS_FORWARD_KEY)|(CHASSIS_BACK_KEY)|(CHASSIS_LEFT_KEY)|(CHASSIS_RIGHT_KEY)))
	{
		speed1 += num1;
		speed = speed1;
		if(chassis->rotate_buff_flag && speed>(NORMAL_SPPED-SPORT_ROTATE_DECREASE))
			speed=(NORMAL_SPPED-SPORT_ROTATE_DECREASE);
		else if(speed>NORMAL_SPPED)
		speed = NORMAL_SPPED;
		close_flag=1;
	}
	else if(close_flag)
	{
		close_flag=0;
	}
	
	if(close_flag==0&&last_close_flag==1)
		move_close_flag=1;
	last_close_flag=close_flag;

	
	if(chassis->connect->can2_rc_ctrl.control_mode == REMOTE_MODE)      
	{
		if ( RC_abs(chassis->connect->can2_rc_ctrl.rc.ch3) < 500 || RC_abs(chassis->connect->can2_rc_ctrl.rc.ch2) < 500)
		{
			if(chassis->connect->can2_rc_ctrl.rc.ch3 < 0 && chassis->connect->can2_rc_ctrl.rc.ch2 < 0)
			{	
				chassis->forward_back = -((chassis->connect->can2_rc_ctrl.rc.ch3 * chassis->connect->can2_rc_ctrl.rc.ch3)/500) *  \
										CHASSIS_RC_CTRL_SPPED_MAX_FACT;
				chassis->left_right = -((chassis->connect->can2_rc_ctrl.rc.ch2 *   chassis->connect->can2_rc_ctrl.rc.ch2)/500) *   \
									CHASSIS_RC_CTRL_SPPED_MAX_FACT;
			}
			else if (chassis->connect->can2_rc_ctrl.rc.ch3 < 0 )
			{
				chassis->forward_back = -((chassis->connect->can2_rc_ctrl.rc.ch3 * chassis->connect->can2_rc_ctrl.rc.ch3)/500) *  \
									CHASSIS_RC_CTRL_SPPED_MAX_FACT;
				chassis->left_right = (chassis->connect->can2_rc_ctrl.rc.ch2 *   chassis->connect->can2_rc_ctrl.rc.ch2)/500 *   \
								CHASSIS_RC_CTRL_SPPED_MAX_FACT;
			}
			else if ( chassis->connect->can2_rc_ctrl.rc.ch2 < 0 )
			{
				chassis->forward_back = (chassis->connect->can2_rc_ctrl.rc.ch3 * chassis->connect->can2_rc_ctrl.rc.ch3)/500 *  \
									CHASSIS_RC_CTRL_SPPED_MAX_FACT;
				chassis->left_right = -((chassis->connect->can2_rc_ctrl.rc.ch2 *   chassis->connect->can2_rc_ctrl.rc.ch2)/500) *   \
								    CHASSIS_RC_CTRL_SPPED_MAX_FACT;
			}
			else 
			{
				chassis->forward_back = (chassis->connect->can2_rc_ctrl.rc.ch3 * chassis->connect->can2_rc_ctrl.rc.ch3)/500 *  \
									CHASSIS_RC_CTRL_SPPED_MAX_FACT;
				chassis->left_right = (chassis->connect->can2_rc_ctrl.rc.ch2 *   chassis->connect->can2_rc_ctrl.rc.ch2)/500 *   \
								    CHASSIS_RC_CTRL_SPPED_MAX_FACT;
			}
		}
		else 
		{
			chassis->forward_back = chassis->connect->can2_rc_ctrl.rc.ch3 *    \
									CHASSIS_RC_CTRL_SPPED_MAX_FACT;
			chassis->left_right = chassis->connect->can2_rc_ctrl.rc.ch2 *      \
									CHASSIS_RC_CTRL_SPPED_MAX_FACT;
		}
		
	}
	else if(chassis->connect->can2_rc_ctrl.control_mode ==  KEY_MOUSE_MODE)   //鼠标键模式  *hyj
	{
//			//forward and back
//			if(chassis->connect->can2_rc_ctrl.mouse.key & CHASSIS_FORWARD_KEY)//D
//			{
//				chassis->forward_back = speed;

//			}
//			else if(chassis->connect->can2_rc_ctrl.mouse.key & CHASSIS_BACK_KEY)//A
//			{
//				chassis->forward_back = -speed;

//			}
//			else
//			{
//				chassis->forward_back = 0;
//			}
//			//left and right
//			if(chassis->connect->can2_rc_ctrl.mouse.key & CHASSIS_LEFT_KEY)//W
//			{
//				chassis->left_right = -speed;
//			}
//			else if(chassis->connect->can2_rc_ctrl.mouse.key & CHASSIS_RIGHT_KEY)//S
//			{
//				chassis->left_right = speed;
//			}
//			else
//			{
//				chassis->left_right = 0;
//			}	
//		
	}	
}
/**
  * @brief         获取底盘旋转值  云台旋转逆时针编码值变大  左5000 右3000
  * @author         
  * @param[in]      
  * @retval			
  * @note          底盘跟随pid目前不稳定 可能某些量的转换过程中方向有错误
  */
float rotate_abs(float val)
{
	if(val < 0)
	{
		val = -val;
	}
	return val;
}
void get_rotate_value(chassis_control_data_t *chassis, chassis_pid_t *chassis_pid)
{
	yaw_raw1 = chassis->yaw_motor_msg->encoder.raw_value;
	if(chassis->connect->can2_rc_ctrl.work_mode == ROBOT_COMMON_MODE)	//	底盘跟随pid
	{
		if(chassis->chassis_control_mode_flag)
		{
			chassis_pid->rotate_pid.set =  GAMBAL_YAW_INIT_ENCODE_VALUE_RHOMB;
		}
		else
		{	
			chassis_pid->rotate_pid.set = GAMBAL_YAW_INIT_ENCODE_VALUE_COMMON;
		}
		chassis_pid->rotate_pid.fdb = (float)(chassis->yaw_motor_msg->encoder.raw_value  \
									+ ((chassis->connect->can2_rc_ctrl.gyro.yaw_set \
								    -chassis->connect->can2_rc_ctrl.gyro.yaw_fdb) * GAMBAL_YAW_angle_VALUE+0.5f));//+ chassis->yaw_motor_msg->encoder.round_cnt * 8192
		//chassis_pid->rotate_pid.fdb = (float)chassis->yaw_motor_msg->encoder.raw_value;
//		chassis_value1 = (chassis->connect->can2_rc_ctrl.rc.ch2)*rotate_speed;
//		chassis_value = (chassis->connect->can2_rc_ctrl.gyro.yaw_set \
//								    -chassis->connect->can2_rc_ctrl.gyro.yaw_fdb) * GAMBAL_YAW_angle_VALUE;
		chassis_pid->rotate_pid.Calc(&chassis_pid->rotate_pid);
		
		chassis->rotate = chassis_pid->rotate_pid.output;//由负修改为正   6.25
		//chassis->rotate_buff_flag = 0;

	}
	else if(chassis->connect->can2_rc_ctrl.work_mode == ROBOT_ROTATE_MOTION_MODE)   //变速小陀螺    *hyj
	{
//		if(rotate_init_flag)
//		{
//		rotate_init_flag=0;
//		yaw_rotate_set = 500;
//		}
		if(RC_abs(chassis->yaw_motor_msg->encoder.raw_value - yaw_raw)>2048)
		{
		chassis->rotate_buff_flag=1;
		yaw_raw=chassis->yaw_motor_msg->encoder.raw_value;
		}
		//delta_raw = rand();
		if(chassis->rotate_buff_flag)      
        {
			 srand(xTaskGetTickCount());
			//czh
				//chassis->rotate = CHASSIS_ROTATE_STOP_SPEED;//500u;
				//chassis->rotate = rand() % CHASSIS_ROTATE_BUFF_SPEED + CHASSIS_ROTATE_BASE_SPEED;// 200u;
				yaw_rotate_set = rand() % CHASSIS_ROTATE_BUFF_SPEED + CHASSIS_ROTATE_BASE_SPEED;
				chassis->rotate_buff_flag = 0;

		}
//	 if(chassis->rotate_buff_flag != 1)     //空档期默认为基础速度       
//	 {
//	 	chassis->rotate = 500u;	//CHASSIS_ROTATE_STOP_SPEED（1000） CHASSIS_ROTATE_BASE_SPEED（600）
//	 }
	}
//	else if(chassis->connect->can2_rc_ctrl.work_mode == ROBOT_ROTATE_STOP_MODE)	//静止小陀螺
//	{
//		chassis->rotate = 0;//1500u;
//	}
	else 
	{
		chassis->rotate = 0;
	}
}

uint8_t action_flag,sentry_mode,sentry_work_mode,move_speed=200;
//void action_conduct(chassis_control_data_t *chassis,car_typedef *car)
//{
//switch(sentry_mode)
//{
//	case 0:
//		if(action_flag==1)
//		{
//		sentry_work_mode=ROBOT_COMMON_MODE;
//		sentry_mode=1;
//	}break;
//	case 1:
//	if(car->position_b[1]>=-1*3)
//		chassis->forward_back = move_speed;
//	else
//		sentry_mode=2;
//	break;
//	case 2:
//	if(car->position_b[0]>=-1*3)
//		chassis->left_right = move_speed;
//	else
//		sentry_mode=3;
//	break;
//	case 3:
//	if(car->position_b[1]<=0)
//		chassis->forward_back = -move_speed;
//	else
//		sentry_mode=04;
//	break;
//	default:break;
//	}
//}

/**
  * @brief        更新底盘电机设定值和反馈值
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
float speed_factor1,speed_factor2,speed_factor3;
 pid_t cmd_forward_pid =
{	
	.kp = 22.0,  //6.2  //10stable  20
	.ki = 0.5,  //0.3   //0
	.kd = 0,
	.ioutMax = 1000,
	.outputMax = 5000,
	.mode = PID_POSITION,			//PID_DELTA	PID_POSITION
};
 pid_t cmd_zuoyou_pid =
{	
	.kp = 6.0,  //6.2  //10stable  20
	.ki = 0.0,  //0.3   //0
	.kd = 0,
	.ioutMax = 1000,
	.outputMax = 5000,
	.mode = PID_POSITION,			//PID_DELTA	PID_POSITION
};
 pid_t rotate_pid =
{	
	.kp = 0.5,  //6.2  //10stable  20
	.ki = 0,  //0.3   //0
	.kd = 30,
	.ioutMax = 1000,
	.outputMax = 5000,
	.mode = PID_POSITION,			//PID_DELTA	PID_POSITION
};

extern float cmd_yaw;

void chassis_set_and_fdb_update(chassis_control_data_t *chassis, \
								chassis_pid_t *chassis_pid)
{
	switch(chassis->connect->can2_rc_ctrl.work_mode)
	//switch(sentry_work_mode)
	{
		case ROBOT_CALI_MODE:
		case ROBOT_INIT_MODE:
		case ROBOT_INIT_END_MODE:
		{
			chassis->forward_back = 0; //模式转换时清零
			chassis->left_right = 0;
			chassis->rotate = 0;

			chassis->forward_back_set = 0;
			chassis->left_right_set = 0;
			chassis->rotate_set = 0;
		}break;
		case ROBOT_COMMON_MODE: //普通底盘跟随模式
		{

			get_forward_back_value(chassis);
			get_rotate_value(chassis, chassis_pid);
			
			chassis->forward_back_set = chassis->forward_back;
			chassis->left_right_set = chassis->left_right;
			chassis->rotate_set = chassis->rotate;
		}break;
		case ROBOT_ROTATE_MOTION_MODE: //运动小陀螺模式
		{
			get_forward_back_value(chassis);//获取控制值，再使用下面函数做转换
			rotate_motion_mode_process(chassis);//运动小陀螺解算
			get_rotate_value(chassis, chassis_pid);
			
			chassis->rotate_set = yaw_rotate_set;
		}break;
		case ROBOT_ROTATE_STOP_MODE: //静止高速小陀螺模式
		{
			//get_rotate_value(chassis, chassis_pid);
			get_forward_back_value(chassis);
//			chassis->forward_back_set = 0;
//			chassis->left_right_set = 0;
//			chassis->rotate_set = chassis->rotate;
			chassis->forward_back_set = chassis->forward_back;
			chassis->left_right_set = chassis->left_right;
			chassis->rotate = 0;
			chassis->rotate_set = 0;
			// chassis->connect->can2_rc_ctrl.gyro.yaw_set;
			// chassis->connect->can2_rc_ctrl.gyro.yaw_fdb;
			// LED_P6x8Str(16,5,(uint8_t *)"yaw_set:");
			// LED_PrintValueI(75,5,chassis_control_data.connect->can2_rc_ctrl.gyro.yaw_fdb);
		}break;
		default:
		{
			chassis->forward_back = 0;
			chassis->left_right = 0;
			chassis->rotate = 0;
			
			chassis->forward_back_set = 0;
			chassis->left_right_set = 0;
			chassis->rotate_set = 0;
		}break;
	}

	switch(sentry_mode)
{
	case 0:
		if(action_flag==1)
		{
		sentry_work_mode=ROBOT_COMMON_MODE;
		sentry_mode=1;
	}break;
	case 1:
	if(car.position_b[0]>=-1*1)//前2格
		chassis->left_right = -move_speed;
	else
		sentry_mode=2;
	break;
	case 2:
	if(car.position_b[1]>=-1*3)//右4格
		chassis->forward_back = move_speed;
	else
		sentry_mode=3;
	break;
	case 3:
	if(car.position_b[0]>=-1*1)//前2格
		chassis->left_right = -move_speed;
	else
		sentry_mode=04;
	case 4:
	if(car.position_b[1]<=0)//左4格
		chassis->forward_back = -move_speed;
	else
		sentry_mode=05;
	case 5:
	sentry_work_mode=ROBOT_ROTATE_MOTION_MODE;
	sentry_mode=06;
	break;
	default:break;
	}

		switch(sentry_work_mode)
	{
		case ROBOT_COMMON_MODE: //普通底盘跟随模式
		{		
			chassis->forward_back_set = chassis->forward_back;
			chassis->left_right_set = chassis->left_right;
			chassis->rotate_set = chassis->rotate;
		}break;
		case ROBOT_ROTATE_MOTION_MODE: //运动小陀螺模式
		{
			get_rotate_value(chassis, chassis_pid);
			
			chassis->rotate_set = yaw_rotate_set;
		}break;
		default:break;
	}
	
#if 0
	chassis->rotate_set = 0; //单独调试使用 不需要旋转量
#endif	
	
	speed_factor1= (1-0.63)/(1-0.4)*(robot_status.chassis_power_limit/100.0-1)+1;// 0.616*(robot_status.chassis_power_limit/100.0)+0.383;
	speed_factor2 = 0.616*(robot_status.chassis_power_limit/100.0)+0.383;
	speed_factor3 =  0.616*(robot_status.chassis_power_limit/100.0)+0.383;
//	chassis->forward_back_set= speed_factor1*chassis->forward_back;
//	chassis->left_right_set  = speed_factor2*chassis->left_right;


			if(chassis->connect->can2_rc_ctrl.control_mode ==  KEY_MOUSE_MODE)   //鼠标键模式  *hyj
	{
		cmd_forward_pid.set = -(chassis->connect->forward_speed)*100.0;
		cmd_zuoyou_pid.set = (chassis->connect->zuoyou_speed)*100.0;
	}
			else if(chassis->connect->can2_rc_ctrl.control_mode ==  REMOTE_MODE)
	{
		cmd_forward_pid.set = -chassis->connect->can2_rc_ctrl.rc.ch2*0.5;
		cmd_zuoyou_pid.set  = -chassis->connect->can2_rc_ctrl.rc.ch3*0.5 ;
	}
			else
			{
			cmd_forward_pid.set =0;
			cmd_zuoyou_pid.set 	=0;
			}
		//cmd_forward_pid.set = chassis->connect->can2_rc_ctrl.rc.ch2*0.5;
		cmd_forward_pid.fdb = -car.v[0];
		PID_Calc(&cmd_forward_pid);
		
		//cmd_zuoyou_pid.set = chassis->connect->can2_rc_ctrl.rc.ch3*0.5 ;
		cmd_zuoyou_pid.fdb = car.v[1];
		PID_Calc(&cmd_zuoyou_pid);
		rotate_pid.set = GAMBAL_YAW_INIT_ENCODE_VALUE_COMMON;
		rotate_pid.fdb = (float)chassis->yaw_motor_msg->encoder.raw_value;
		PID_Calc(&rotate_pid);
		chassis->cm1_set = -cmd_forward_pid.output +cmd_zuoyou_pid.output + rotate_pid.output;
		chassis->cm2_set = -cmd_forward_pid.output -cmd_zuoyou_pid.output+ rotate_pid.output;
		chassis->cm3_set =  cmd_forward_pid.output -cmd_zuoyou_pid.output+  rotate_pid.output;
		chassis->cm4_set =  cmd_forward_pid.output +cmd_zuoyou_pid.output+  rotate_pid.output;
		
	
	chassis->cm1_fdb = chassis->cm1_msg->encoder.filter_rate;
	chassis->cm2_fdb = chassis->cm2_msg->encoder.filter_rate;
	chassis->cm3_fdb = chassis->cm3_msg->encoder.filter_rate;
	chassis->cm4_fdb = chassis->cm4_msg->encoder.filter_rate;
	//绿灯
	if(chassis->connect->can2_rc_ctrl.work_mode == ROBOT_INIT_MODE)
	{
		HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_SET);
	}
}

/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void chassis_pid_calculate(chassis_control_data_t *chassis,  \
						   chassis_pid_t *chassis_pid)
{
	chassis_pid->cm1_pid.set = chassis->cm1_set;
	chassis_pid->cm2_pid.set = chassis->cm2_set;
	chassis_pid->cm3_pid.set = chassis->cm3_set;
	chassis_pid->cm4_pid.set = chassis->cm4_set;
	
	chassis_pid->cm1_pid.fdb = chassis->cm1_fdb;
	chassis_pid->cm2_pid.fdb = chassis->cm2_fdb;
	chassis_pid->cm3_pid.fdb = chassis->cm3_fdb;
	chassis_pid->cm4_pid.fdb = chassis->cm4_fdb;
	
	chassis_pid->cm1_pid.Calc(&chassis_pid->cm1_pid);
	chassis_pid->cm2_pid.Calc(&chassis_pid->cm2_pid);
	chassis_pid->cm3_pid.Calc(&chassis_pid->cm3_pid);
	chassis_pid->cm4_pid.Calc(&chassis_pid->cm4_pid);
}


void chassis_forwardfeed(chassis_control_data_t *chassis)
{
 chassis->cm1_ff=forwardfeed(RC_abs(chassis->cm1_msg->encoder.filter_rate));
 chassis->cm2_ff=forwardfeed(RC_abs(chassis->cm2_msg->encoder.filter_rate));
	chassis->cm3_ff=forwardfeed(RC_abs(chassis->cm3_msg->encoder.filter_rate));
	chassis->cm4_ff=forwardfeed(RC_abs(chassis->cm4_msg->encoder.filter_rate));
	if(chassis->cm1_msg->encoder.filter_rate>0)
	{
	chassis->cm2_ff=-chassis->cm2_ff;
	chassis->cm3_ff=-chassis->cm3_ff;		
	}
	else
	{
	chassis->cm1_ff=-chassis->cm1_ff;
	chassis->cm4_ff=-chassis->cm4_ff;
	}
		
}

/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */

void chassis_control_loop(chassis_control_data_t *chassis, \
						  chassis_pid_t *chassis_pid)
{
	chassis->given_current.cm1 = chassis_pid->cm1_pid.output + chassis->cm1_ff;
	chassis->given_current.cm2 = chassis_pid->cm2_pid.output + chassis->cm2_ff;
	chassis->given_current.cm3 = chassis_pid->cm3_pid.output + chassis->cm3_ff;
	chassis->given_current.cm4 = chassis_pid->cm4_pid.output + chassis->cm4_ff;
	
	if(chassis->connect->can2_rc_ctrl.control_mode == GUI_CALI_MODE)
	{
		set_chassis_stop();
	}
	else 
	{
		set_chassis_behaviour(chassis->given_current.cm1,
							  chassis->given_current.cm2,
							  chassis->given_current.cm3,
							  chassis->given_current.cm4);
	}
}

/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           void chassis_task(void * pvParameters)
  */
 
void chassis_task(void *argument)
{
	TickType_t current_time = 0;

	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	chassis_init(&chassis_control_data, &chassis_pid);
	
	while(1)
	{
		current_time = xTaskGetTickCount();                         //当前系统时间       *hyj
		//action_conduct(&chassis_control_data,&car);
		chassis_set_and_fdb_update(&chassis_control_data, &chassis_pid);
		chassis_power_limit();
		srand(xTaskGetTickCount());
		odometer_delta(&chassis_control_data);
		chassis_pid_calculate(&chassis_control_data, &chassis_pid);
		chassis_forwardfeed(&chassis_control_data);
		chassis_control_loop(&chassis_control_data, &chassis_pid);
		vTaskDelayUntil(&current_time, 1);       //1ms一次         *hyj
	}
}

