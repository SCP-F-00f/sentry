#ifndef CALI_TASK_H
#define CALI_TASK_H

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
#include "can1_app.h"
#include "connect_task.h"

#define CALI_CHECK               (59u)       //pid参数标志
#define CALI_PARAM_FEATURE       (30u)       //自定义参数标志

typedef enum
{
	YAW_POSITION_PID = 1,
	YAW_SPEED_PID,
	PITCH_POSITION_PID,
	PITCH_SPEED_PID,
	TRIGGER_POSITION_PID,
	TRIGGER_SPEED_PID,
	FRIC1_PID,
	FRIC2_PID,
	CM_PID,
	ROTATE_PID,
	OTHER_1_PID,
	OTHER_2_PID,
	
}pid_type_e;

#pragma pack(push, 1) //防止编译器4字节对齐 导致转换成float失败（之前的压栈）

typedef struct
{
	uint8_t cali_p_array[4];
	uint8_t cali_i_array[4];
	uint8_t cali_d_array[4];
	
	uint8_t cali_param1_array[4]; //自定义参数数组
	uint8_t cali_param2_array[4];
	uint8_t cali_param3_array[4];
	uint8_t cali_param4_array[4];
	uint8_t cali_param5_array[4];
	uint8_t cali_param6_array[4];
	
	uint8_t check_top_byte;
	uint8_t check_bottom_byte;
	uint8_t pid_type_data;
	
	float p;
	float i;
	float d;
	
	float param1; //自定义参数 可以直接赋值给其他的调试参数
	float param2;
	float param3;
	float param4;
	float param5;
	float param6;
	
	uint8_t receive_success_flag; //接收pc数据成功标志位
	uint8_t beep_flag;            //提示标志位
	
} pc_cali_t;

#pragma pack(pop) //恢复之前字节对齐数

typedef struct
{
	const connect_t *connect;
	const motor_msg_t * shoot_fric1_msg;
	const motor_msg_t * shoot_fric2_msg;
	
	uint8_t system_control_mode;
	uint8_t system_work_mode;
	uint8_t gimbal_work_mode;
	uint8_t shoot_work_mode;
	uint8_t fric_work_mode;
	uint8_t chassis_work_mode;
	
	int16_t fric1_speed;
	int16_t fric2_speed;
	int16_t shoot_cnt;

	int16_t yaw_angle;//float
	int16_t pitch_angle;
	int16_t roll;
	
} pc_cali_tx_t;
extern pc_cali_t pc_cali;

extern void set_chassis_CM_pid_data_to_chassis(void);
extern void set_chassis_rotate_pid_data_to_chassis(void)  ;

void get_pc_cali_data(pc_cali_t *pc_cali, uint8_t *usart1_data);
	
#endif


