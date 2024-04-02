/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HongYuJia 
 * @Teammate：
 * @Version: V3.0
 * @Date:2021.4.13
 * @Description:   关于AC板之间的信息交互
 * @Note:       
 * @Others: 
**/
#include "connect_task.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "can1_app.h"
#include "can2_app.h"
#include "main.h"
#include "chassis_task.h"
#include "ins_task.h"


extern CAN_TxHeaderTypeDef can2_tx_header;
extern uint8_t can2_tx_data[CAN_TX_BUF_SIZE];
connect_t connect_data;
extern imu_t imu;

/**
  * @brief          can2接收rc数据保存
  * @author         
  * @param[in] 
  * @retval	
  * @note        
  */
void connect_rc_ctrl_process(connect_t *connect_data, uint8_t aData[])
{
	connect_data->can2_rc_ctrl.control_mode = (aData[0]);
	connect_data->can2_rc_ctrl.work_mode = (aData[1]);
	connect_data->can2_rc_ctrl.rc.ch3 = (aData[2]<<8)|aData[3];
	connect_data->can2_rc_ctrl.rc.ch2 = (aData[4]<<8)|aData[5];
	connect_data->can2_rc_ctrl.mouse.key = (aData[6]<<8)|aData[7];
	
	connect_data->can2_rc_ctrl.rc.ch2 -= RC_CHANNEL_VALUE_MIDDLE;
	connect_data->can2_rc_ctrl.rc.ch3 -= RC_CHANNEL_VALUE_MIDDLE;
	connect_data->can2_rc_ctrl.rc.ch2 = -connect_data->can2_rc_ctrl.rc.ch2;
	connect_data->receive_rc_data_flag = 1;//表示已经接收到了can2的rc数据
}

typedef union float_char_
{
	float f;
	uint8_t c[4];
}float_char;
float yaw_yaw;
void connect_gyro_data_process(connect_t *connect_data, uint8_t aData[])
{
	float_char f_c;
	f_c.c[0] = aData[0];
	f_c.c[1] = aData[1];
	f_c.c[2] = aData[2];
	f_c.c[3] = aData[3];
	connect_data->can2_rc_ctrl.gyro.yaw_set = f_c.f/10;
	f_c.c[0] = aData[4];
	f_c.c[1] = aData[5];
	f_c.c[2] = aData[6];
	f_c.c[3] = aData[7];
	connect_data->can2_rc_ctrl.gyro.yaw_fdb = f_c.f/10;
	yaw_yaw = connect_data->can2_rc_ctrl.gyro.yaw_fdb;
	connect_data->receive_rc_data_flag = 1;//表示已经接收到了can2的rc数据

}





/**
  * @brief          连接初始化
  * @author         
  * @param[in] 
  * @retval	
  * @note           考虑未连接 挂起任务调度器 或进入临界区 while(1)
  */
void connect_init(connect_t *connect_data) 
{
	connect_data->cm1_msg = get_cm1_msg_point();
	connect_data->cm2_msg = get_cm2_msg_point();
	connect_data->cm3_msg = get_cm3_msg_point();
	connect_data->cm4_msg = get_cm4_msg_point();
	
//	check_connect(connect_data);
}

void connect_cmd_data_process(connect_t *connect_data, uint8_t aData[])
{
	if(aData[0] == 'a')
	{
		float_char f_c;
		f_c.c[0] = aData[1];
		f_c.c[1] = aData[2];
		f_c.c[2] = aData[3];
		f_c.c[3] = aData[4];
		connect_data->forward_speed = f_c.f;
	}
	else if(aData[0] == 'b')
	{
		float_char f_c;
		f_c.c[0] = aData[1];
		f_c.c[1] = aData[2];
		f_c.c[2] = aData[3];
		f_c.c[3] = aData[4];
		connect_data->zuoyou_speed = f_c.f;
	}
	else if(aData[0] == 'c')
	{
		float_char f_c;
		f_c.c[0] = aData[1];
		f_c.c[1] = aData[2];
		f_c.c[2] = aData[3];
		f_c.c[3] = aData[4];
		connect_data->rotate_speed = f_c.f;
	}

}

/**
  * @brief          连接任务
  * @author         
  * @param[in] 
  * @retval	
  * @note           
  */
 
void connect_task(void *argument)
{
	connect_init(&connect_data);
	while(1)
	{
		send_shoot_17mm_data();
		send_shoot_judge_data();
		send_odem_data();
		//vTaskDelay(10);
	}
}
/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note           
  */
connect_t *get_connect_data_point(void)
{
	return &connect_data;
}
