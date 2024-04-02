#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
#include "pid.h"
#include "remote_app.h"
#include "gimbal_task.h"

#define SHOOT_TASK_TIME_1MS         (1u)

// #define FRIC_NORMAL_SPPED_VALUE     (300.0f)//摩擦轮正常速度值
#define FRIC_TEXT_SPEED           (600.0f)

#define FRIC_ALLOW_SPEED_OFFSET     (300.0f)//摩擦轮速度允许偏移
#define TRIGGER_INIT_POSITION_ECD   (2000)//拨轮电机初始位置

#define TRIGGER_GRID          	    (12) //拨盘格数
#define SINGLE_BULLET_ECD_INC 	    (24576.0f)//转过的编码值 
#define SINGLE_BULLET_ANGLE_INC     (30.0f)//单发拨轮需要转过的角度
#define TRIGGER_ECD_TO_ANGLE  	    (0.001220703125f)//连续编码值转到连续角度 考虑减速比36:1

#define SHOOT_TASK_INIT_TIME        (3010)//开始前延时一段时间 等待其他模块接收到数据

#define PRESS_LONG_TIME             (400)//长按值
#define RC_CH4_MAX                  (610)

#define MAGAZINE_CONTROL            KEY_PRESSED_OFFSET_R         //弹仓控制 
#define SHOOT_CONTROL               KEY_PRESSED_OFFSET_CTRL      //发射机构保险
#define VISION_BUFF                 KEY_PRESSED_OFFSET_F         //视觉射击
#define VISION_BUFF_OVER            KEY_PRESSED_OFFSET_G         //取消视觉射击
typedef enum
{
	SHOOT_STOP = 0,
	SHOOT_READY,
	SHOOT_BULLET,
}shoot_mode_e;

typedef enum 
{
	FRIC_WHEEL_NORMAL = 0,  //正常的
	FRIC_WHEEL_INSTABLE,    //异常的
	FRIC_WHEEL_OFF,
}fric_mode_e;

typedef struct
{
	cascade_pid_t trigger_pid;
	pid_t fric_1_pid;
	pid_t fric_2_pid;
}shoot_pid_t;

typedef struct
{
	RC_ctrl_t *rc_ctrl;
	motor_msg_t *trigger_motor_msg;
	motor_msg_t *fric_motor1_msg;
	motor_msg_t *fric_motor2_msg;	

	given_current_t given_current;
	
	uint8_t single_bullet_flag;
	uint8_t revolution_bullet_flag;
	float trigger_position_angle_set;
	float trigger_position_angle_fdb;
	float trigger_continuous_set;
	float trigger_continyous_fdb;

	
	float fric1_set;
	float fric1_fdb;
	
	float fric2_set;
	float fric2_fdb;
	
	//鼠标+遥控器ch4              
	uint16_t press_left_time;//左键按下时间统计
	uint16_t press_right_time;
	uint8_t mouse_left_long_press_flag;      //左长按   
	uint8_t mouse_right_long_press_flag;     //右长按   （连发模式）
	uint8_t mouse_left_single_click_flag;    //左单击   （点射模式）
	uint8_t mouse_right_single_click_flag;   //右单击 

	uint8_t magazine_control_flag;      //1:开弹仓      0:关弹仓
	uint8_t shoot_control_flag;         //发射机构保险
	uint8_t shoot_vision_flag;          //视觉模式
}shoot_control_data_t;

uint8_t get_shoot_mode(void);
uint8_t get_fric_mode(void);
#endif
