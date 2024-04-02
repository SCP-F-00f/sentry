/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HongYuJia  
 * @Teammate：
 * @Version: V3.0
 * @Date:2020.4.13
 * @Description: 关于AC板之间的信息交互
 * @Note:       
 * @Others: 
**/
#include "connect_task.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gpio.h"
#include "remote_app.h"
#include "can1_app.h"
#include "gimbal_task.h"
#include "can.h"
#include "can2_app.h"
#include "tim.h"
#include "GUI_task.h"

extern CAN_RxHeaderTypeDef can2_rx_header;
extern uint8_t can2_rx_data[CAN_RX_BUF_SIZE];
extern CAN_TxHeaderTypeDef can2_tx_header;
extern uint8_t can2_tx_data[CAN_TX_BUF_SIZE];
extern CAN_HandleTypeDef hcan2;
extern RC_ctrl_t rc_ctrl_data;
extern gimbal_control_data_t gimbal_control_data;
extern ext_Judge_data_t Judge_data;
extern INS_t INS;
extern odem_data_t odem_data;
connect_t connect_data;
_tx2_tx_data vision_tx_data;


/**
* @brief        can2获取rc数据 发送往底盘
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void can2_get_rc_data(connect_t *connect_data)
{
	connect_data->can2_rc_ctrl.control_mode = get_robot_control_mode();
	connect_data->can2_rc_ctrl.work_mode = get_robot_work_mode();
	connect_data->can2_rc_ctrl.rc.ch2 = connect_data->rc_ctrl->rc.ch2 + \
													  RC_CHANNEL_VALUE_MIDDLE;
	connect_data->can2_rc_ctrl.rc.ch3 = connect_data->rc_ctrl->rc.ch3 + \
													  RC_CHANNEL_VALUE_MIDDLE;
	connect_data->can2_rc_ctrl.mouse.key = connect_data->rc_ctrl->key.v;
}
void send_rc_to_chassis(can2_rc_ctrl_t *can2_rc_ctrl)  
{
	can2_tx_header.StdId = CAN2_CONNECT_RC_CTRL_STD_ID;
    can2_tx_header.IDE = CAN_ID_STD;
    can2_tx_header.RTR = CAN_RTR_DATA;
    can2_tx_header.DLC = 0x08;
    
    can2_tx_data[0] = (uint8_t)(can2_rc_ctrl->control_mode);
    can2_tx_data[1] = (uint8_t)(can2_rc_ctrl->work_mode);
    can2_tx_data[2] = (uint8_t)(can2_rc_ctrl->rc.ch2 >> 8);
    can2_tx_data[3] = (uint8_t)(can2_rc_ctrl->rc.ch2);
    can2_tx_data[4] = (uint8_t)(can2_rc_ctrl->rc.ch3 >> 8);
    can2_tx_data[5] = (uint8_t)(can2_rc_ctrl->rc.ch3);
    can2_tx_data[6] = (uint8_t)(can2_rc_ctrl->mouse.key >> 8);
    can2_tx_data[7] = (uint8_t)(can2_rc_ctrl->mouse.key);
	HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX1  );
}
/**
* @brief        can2获取GYRO陀螺仪数据，发送往底盘
  * @author         
  * @param[in]      
  * @retval			
  * @note           
*/
typedef union float_char_
{
	float f;
	uint8_t c[4];
}float_char;
extern gimbal_control_data_t gimbal_control_data;
void send_gyro_data_to_chassis(void)  
{
	can2_tx_header.StdId = CAN2_CONNECT_CM_GYRO_STD_ID;
    can2_tx_header.IDE = CAN_ID_STD;
    can2_tx_header.RTR = CAN_RTR_DATA;
    can2_tx_header.DLC = 0x08;
	float_char f_c;
	f_c.f = (gimbal_control_data.gimbal_yaw_set*10);
	can2_tx_data[0] = f_c.c[0];
	can2_tx_data[1] = f_c.c[1];
	can2_tx_data[2] = f_c.c[2];
	can2_tx_data[3] = f_c.c[3];
	f_c.f = (gimbal_control_data.gimbal_INS->yaw_angle*10);
	can2_tx_data[4] = f_c.c[0];
	can2_tx_data[5] = f_c.c[1];
	can2_tx_data[6] = f_c.c[2];
	can2_tx_data[7] = f_c.c[3];
	HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX1  );
}

uint8_t tx_data[22];
 a_1 bbb;

void vision_tx(_tx2_tx_data *vision_tx_data,INS_t *INS)
{
//		if(Judge_data.robot_id < 10) //己方红色
//		vision_tx_data->robot_color=0;
//		else
		vision_tx_data->robot_color=1;
		if(connect_data.can2_rc_ctrl.work_mode==4||connect_data.can2_rc_ctrl.work_mode==5)
		vision_tx_data->robot_mode=4;
		else
		vision_tx_data->robot_mode=0;
		vision_tx_data->robot_id=2;
		vision_tx_data->gyro_x=INS->gyro_y;
		vision_tx_data->gyro_y=INS->gyro_z ;
		vision_tx_data->yaw=INS->yaw_angle ;
		vision_tx_data->pitch=INS->pitch_angle;
		//tx_data.gyro_accle
		vision_tx_data->bullet_speed=Judge_data.bullet_speed;
		tx_data[0]='S';
		tx_data[1]=vision_tx_data->robot_color;
		tx_data[2]=vision_tx_data->robot_mode;
		tx_data[3]=vision_tx_data->robot_id;
		unsigned char* ptr = (unsigned char*)&(vision_tx_data->yaw);
		tx_data[4] = *(ptr + 0);
		tx_data[5] = *(ptr + 1);
    tx_data[6] = *(ptr + 2);  // 获取低位字节
    tx_data[7] = *(ptr + 3);
		ptr = (unsigned char*)&(vision_tx_data->pitch);
		tx_data[8] =  *(ptr +  0);
		tx_data[9] =  *(ptr +  1);
    tx_data[10] = *(ptr + 2);  // 获取低位字节
    tx_data[11] = *(ptr + 3);		
//		tx_data[12]=(vision_tx_data->gyro_accle);
//		tx_data[13]=(vision_tx_data->gyro_accle<<8);
		bbb.aaa = (float)(vision_tx_data->gyro_x);
    tx_data[12] =  bbb.data[0];  // 获取低位字节
    tx_data[13] =  bbb.data[1];
		tx_data[14] =  bbb.data[2];
		tx_data[15] =  bbb.data[3];
		bbb.aaa = (float)(vision_tx_data->gyro_y);
    tx_data[12] =  bbb.data[0];  // 获取低位字节
    tx_data[13] =  bbb.data[1];
		tx_data[14] =  bbb.data[2];
		tx_data[15] =  bbb.data[3];
		tx_data[20]=Judge_data.bullet_speed;
		tx_data[21]='E';
		HAL_UART_Transmit(&huart1, tx_data, sizeof(tx_data), 1000);
		
}


/**
  * @brief          连接初始化
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void connect_init(connect_t *connect_data)
{
	connect_data->rc_ctrl = get_rc_data_point();
	
//	vTaskSuspendAll();//挂起任务调度器
//	check_connect(connect_data);//如果未连接将无法退出循环，云台也不会发送rc数据
//	xTaskResumeAll();
}

uint8_t radar_tx_data[30];
extern float yaw_temp;
void odem_tx(odem_data_t *odem_data)
{
	typedef	union a__
	{
		float aaa;
		uint8_t data[4];
	}a_1;
	a_1 aaa;
		radar_tx_data[0]='F';
		aaa.aaa = yaw_temp;
		radar_tx_data[1] = aaa.data[0];
		radar_tx_data[2] = aaa.data[1];
    radar_tx_data[3] = aaa.data[2];  // 获取低位字节
    radar_tx_data[4] = aaa.data[3];
		unsigned char* ptr = (unsigned char*)&(odem_data->imu_wz);
		radar_tx_data[5] = *(ptr + 0);
		radar_tx_data[6] = *(ptr + 1);
    radar_tx_data[7] = *(ptr + 2);  // 获取低位字节
    radar_tx_data[8] = *(ptr + 3);
		aaa.aaa = (odem_data->imu_ax);
		radar_tx_data[ 9] = aaa.data[0];
		radar_tx_data[10] = aaa.data[1];
    radar_tx_data[11] = aaa.data[2];// 获取低位字节
    radar_tx_data[12] = aaa.data[3];
		aaa.aaa = (odem_data->imu_ay);
		radar_tx_data[13] = aaa.data[0];
		radar_tx_data[14] = aaa.data[1];
    radar_tx_data[15] = aaa.data[2];// 获取低位字节
    radar_tx_data[16] = aaa.data[3];
		ptr = (unsigned char*)&(odem_data->zuobiao[0]);
		radar_tx_data[17] = *(ptr + 0);
		radar_tx_data[18] = *(ptr + 1);
    radar_tx_data[19] = *(ptr + 2);  // 获取低位字节
    radar_tx_data[20] = *(ptr + 3);
		ptr = (unsigned char*)&(odem_data->zuobiao[1]);
		radar_tx_data[21] = *(ptr + 0);
		radar_tx_data[22] = *(ptr + 1);
    radar_tx_data[23] = *(ptr + 2);  // 获取低位字节
    radar_tx_data[24] = *(ptr + 3);
		ptr = (unsigned char*)&(odem_data->encoder_position[0]);
		radar_tx_data[25] = *(ptr + 0);
		radar_tx_data[26] = *(ptr + 1);
		ptr = (unsigned char*)&(odem_data->encoder_position[1]);
		radar_tx_data[27] = *(ptr + 0);
		radar_tx_data[28] = *(ptr + 1);
		radar_tx_data[29]='G';
		HAL_UART_Transmit(&huart6, radar_tx_data, sizeof(radar_tx_data), 1000);
}
extern CAN_TxHeaderTypeDef can2_cmd_header;
uint8_t radar_cmd_data[30];
void send_cmd_to_chassis(_tx2_control_data *control_data)
{
	can2_cmd_header.StdId = 0x111;
	can2_cmd_header.IDE = CAN_ID_STD;
	can2_cmd_header.RTR = CAN_RTR_DATA;
	can2_cmd_header.DLC = 0x08;
	float_char f_c;
	f_c.f = (control_data->forward_speed);
	radar_cmd_data[0] = 'a';
	radar_cmd_data[1] = f_c.c[0];
	radar_cmd_data[2] = f_c.c[1];
	radar_cmd_data[3] = f_c.c[2];
	radar_cmd_data[4] = f_c.c[3];
	radar_cmd_data[5] = 0;
	radar_cmd_data[6] = 0;
	radar_cmd_data[7] = 0;
	HAL_CAN_AddTxMessage(&hcan2, &can2_cmd_header, radar_cmd_data, (uint32_t *) CAN_TX_MAILBOX1  );
	vTaskDelay(10);
	f_c.f = (control_data->zuoyou_speed);
	radar_cmd_data[0] = 'b';
	radar_cmd_data[1] = f_c.c[0];
	radar_cmd_data[2] = f_c.c[1];
	radar_cmd_data[3] = f_c.c[2];
	radar_cmd_data[4] = f_c.c[3];
	radar_cmd_data[5] = 0;
	radar_cmd_data[6] = 0;
	radar_cmd_data[7] = 0;
	HAL_CAN_AddTxMessage(&hcan2, &can2_cmd_header, radar_cmd_data, (uint32_t *) CAN_TX_MAILBOX1  );
	vTaskDelay(10);
	f_c.f = (control_data->rotate_speed);
	radar_cmd_data[0] = 'c';
	radar_cmd_data[1] = f_c.c[0];
	radar_cmd_data[2] = f_c.c[1];
	radar_cmd_data[3] = f_c.c[2];
	radar_cmd_data[4] = f_c.c[3];
	radar_cmd_data[5] = 0;
	radar_cmd_data[6] = 0;
	radar_cmd_data[7] = 0;
	HAL_CAN_AddTxMessage(&hcan2, &can2_cmd_header, radar_cmd_data, (uint32_t *) CAN_TX_MAILBOX2  );
	vTaskDelay(10);
}


/**
  * @brief         连接底盘任务
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
extern _tx2_control_data control_data;
void connect_task(void *argument)
{	
	// TickType_t current_time = 0;
	odem_data.zuobiao[0] = 3.85821;
	odem_data.zuobiao[1] = -0.714966;
	connect_init(&connect_data);
	while(1)
	{
		// current_time = xTaskGetTickCount();                         //当前系统时间       *hyj
		can2_get_rc_data(&connect_data);
		send_cmd_to_chassis(&control_data);
		send_rc_to_chassis(&connect_data.can2_rc_ctrl);
		vision_tx(&vision_tx_data,&INS);
		odem_tx(&odem_data);//串口发送给小电脑巡航数据
		
		vTaskDelay(5);                        // 10ms一次
		
//		 vTaskDelayUntil(&current_time, CONNECT_TASK_TIME_1MS);       //1ms一次         *hyj     
	}
}


connect_t *get_connect_data_point(void)
{
	return &connect_data;
}
