#ifndef CAN2_APP_H
#define CAN2_APP_H

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
#include "can1_app.h"

#define CAN_2 hcan2

typedef enum
{	
	//yaw轴电机
	CAN2_YAW_MOTOR_STD_ID = 0x206,
	//can2收
	CAN2_CONNECT_RC_CTRL_STD_ID = 0x200,
	CAN2_CONNECT_CM_GYRO_STD_ID = 0x208,
	//can2发
	CAN2_SHOOT_17mm_ID = 0x020B,         //17mm发射机构裁判信息
	CAN2_SHOOT_JUDGE_ID = 0x020C,        //发射机构裁判信息
	CAN2_ODEM_ID = 0x209,

} can2_msg_id_e;

void send_shoot_17mm_data(void);
void send_shoot_judge_data(void);
void send_odem_data(void);
motor_msg_t *get_yaw_motor_msg_point(void);
void can2_message_progress(CAN_RxHeaderTypeDef *pHeader, uint8_t aData[]);

#endif
