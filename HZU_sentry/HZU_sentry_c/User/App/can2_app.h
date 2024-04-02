#ifndef CAN2_APP_H
#define CAN2_APP_H

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
#include "can1_app.h"

//#define RATE_BUF_SIZE 6  //电机速度滑动滤波窗口大小

typedef struct
{
	
	uint8_t robot_id;                           //机器人ID
	//发射机构
	uint16_t shooter_id1_17mm_cooling_rate;     //每秒冷却值0：17mm的  1：42mm的  
	uint16_t shooter_id1_17mm_cooling_limit;    //枪口热量上限
	uint16_t shooter_id1_17mm_speed_limit;      //速度上限
	uint8_t mains_power_shooter_output ;        //电源是否输出给发射机构
	float bullet_speed;                         //当前射速
	uint16_t shooter_id1_17mm_cooling_heat;     //17mm当前枪口热量
	uint8_t hurt_type : 4;                      //0x2 超射速扣血；0x3 超枪口热量扣血；

} ext_Judge_data_t;                     //实时裁判信息   *hyj

typedef struct
{
	float imu_yaw;
	float imu_wz;
	float imu_ax;
	float imu_ay;
	float zuobiao[2];
	int16_t encoder_position[2];
	
} odem_data_t;

typedef enum
{	
	CAN2_GIMBAL_STD_ID = 0x1FF,     //云台控制
	CAN2_YAW_MOTOR_STD_ID = 0x206,	// yaw电机
	CAN2_PITCH_MOTOR_STD_ID = 0x205,	//pitch电机
	//can2发
	CAN2_CONNECT_RC_CTRL_STD_ID = 0x200,
	CAN2_CONNECT_CM_GYRO_STD_ID = 0x208,
	//can2收
	CAN2_SHOOT_17mm_ID = 0x020B,         //17mm发射机构裁判信息
	CAN2_SHOOT_JUDGE_ID = 0x020C,        //发射机构裁判信息
	CAN2_ODEM_ID = 0x209,
} can2_msg_id_e;

typedef	union a__
	{
		float aaa;
		uint8_t data[4];
	}a_1;

extern motor_msg_t yaw_motor_msg;
extern motor_msg_t pitch_motor_msg;

void can2_message_progress(CAN_RxHeaderTypeDef *pHeader, uint8_t aData[]);
                             
void shoot_17mm_mag(ext_Judge_data_t *Judge_data, uint8_t aData[]);
void shoot_judge_process(ext_Judge_data_t *Judge_data, uint8_t aData[]);
void odem_data_process(odem_data_t *odem_data, uint8_t aData[]);
void accel_data_process(odem_data_t *odem_data, uint8_t aData[]);
void speed_data_process(odem_data_t *odem_data, uint8_t aData[]);
void position_data_process(odem_data_t *odem_data, uint8_t aData[]);

void set_gimbal_behaviour(int16_t yaw_iq, int16_t pitch_iq);
void set_gimbal_stop(void);

motor_msg_t *get_yaw_motor_msg_point(void);
motor_msg_t *get_pitch_motor_msg_point(void);


#endif

