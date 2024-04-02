/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HongYuJia 
 * @Teammate：
 * @Version: V3.0
 * @Date:2021.4.13
 * @Description:  CAN2应用（包括裁判系统信息的传输、AC板间信息的交互）
 * @Note:       
 * @Others: 
**/
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "connect_task.h"
#include "can.h"
#include "can1_app.h"
#include "can2_app.h"
#include "judge.h"
#include "ins_task.h"
#include "chassis_task.h"
#include "stm32f4xx_hal_can.h"
#include "ULTRASONIC_task.h"


extern CAN_RxHeaderTypeDef can2_rx_header;
extern uint8_t can2_rx_data[CAN_RX_BUF_SIZE];
extern CAN_TxHeaderTypeDef can2_tx_header;
extern uint8_t can2_tx_data[CAN_TX_BUF_SIZE];
extern CAN_HandleTypeDef hcan2;
extern imu_t imu;
extern car_typedef car;
extern chassis_control_data_t chassis_control_data;
motor_msg_t yaw_motor_msg = {0};
/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note    默认电机的can发送频率为1KHZ       
  */ 

static void yaw_motor_msg_process(motor_msg_t *m, uint8_t aData[])
{
	int16_t i;
	m->encoder.filter_rate_sum = 0;//进入清零
	m->encoder.last_raw_value = m->encoder.raw_value; 
	if(m->encoder.start_flag==0)//上电采集原始角度
	{
		m->encoder.ecd_bias = (aData[0]<<8)|aData[1];//初始位置
		m->encoder.last_raw_value = (aData[0]<<8)|aData[1];
		m->encoder.raw_value = m->encoder.last_raw_value;
		m->encoder.start_flag = 1;
	}
	else
	{
		m->encoder.raw_value = (aData[0]<<8)|aData[1];
	}
	
	m->encoder.diff = m->encoder.raw_value - m->encoder.last_raw_value;
	if(m->encoder.diff < -6000)//两次编码器的反馈值差别太大，表示圈数发生了改变                         
	{                          //7500根据控制周期可调，即保证一个控制周期内 转子机械角度变化小于8191-7500=691的量 
		m->encoder.round_cnt ++;
		m->encoder.ecd_raw_rate = m->encoder.diff + 8192;
	}
	else if(m->encoder.diff > 6000)
	{
		m->encoder.round_cnt --;
		m->encoder.ecd_raw_rate = m->encoder.diff - 8192;
	}
	else
	{
		m->encoder.ecd_raw_rate = m->encoder.diff;
	}
	//计算得到角度值，范围正负无穷大
	m->encoder.ecd_angle = (float)(m->encoder.raw_value - m->encoder.ecd_bias)*360/8192  \
								   + m->encoder.round_cnt * 360;
	
	m->encoder.rate_buf[m->encoder.buf_count++] = m->encoder.ecd_raw_rate;
	if(m->encoder.buf_count == RATE_BUF_SIZE)
	{
		m->encoder.buf_count = 0;
	}
	//计算速度平均值
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		m->encoder.filter_rate_sum += m->encoder.rate_buf[i];
	}
	m->encoder.filter_rate = (int32_t)(m->encoder.filter_rate_sum/RATE_BUF_SIZE);	
	/*---------------------非编码器数据------------------------*/
	m->speed_rpm = (int16_t)(aData[2] << 8 | aData[3]);     
	m->given_current = (int16_t)(aData[4] << 8 | aData[5]); 
	m->temperate = aData[6];          
}

extern chassis_pid_t chassis_pid;
void can2_message_progress(CAN_RxHeaderTypeDef *pHeader, uint8_t aData[])
{
	if(pHeader == NULL || aData == NULL )
	{
		return;
	}
	switch(pHeader->StdId)
	{
		//get rc control 
		case CAN2_CONNECT_RC_CTRL_STD_ID:
		{
			connect_rc_ctrl_process(&connect_data ,aData);
		}break;

		//get gimbal control 
		case CAN2_YAW_MOTOR_STD_ID:
		{
			yaw_motor_msg_process(&yaw_motor_msg ,aData); 
		}break;
		case CAN2_CONNECT_CM_GYRO_STD_ID:
		{
			connect_gyro_data_process(&connect_data ,aData);
		}
		break;
		case 0x111:
		{
			connect_cmd_data_process(&connect_data ,aData);
		}
		break;


		default: break;
	}
}


extern ext_game_robot_status_t robot_status;
extern ext_power_heat_data_t power_heat;
extern ext_shoot_data_t shoot_data; 
extern ext_robot_hurt_t robot_hurt;

void send_shoot_17mm_data(void)  
{
	can2_tx_header.StdId = CAN2_SHOOT_17mm_ID;
    can2_tx_header.IDE = CAN_ID_STD;
    can2_tx_header.RTR = CAN_RTR_DATA;
    can2_tx_header.DLC = 0x08;

    can2_tx_data[0] = (uint8_t)(robot_status.shooter_id1_17mm_cooling_rate>>8);
    can2_tx_data[1] = (uint8_t)(robot_status.shooter_id1_17mm_cooling_rate);
    can2_tx_data[2] = (uint8_t)(robot_status.shooter_id1_17mm_cooling_limit>>8);
    can2_tx_data[3] = (uint8_t)(robot_status.shooter_id1_17mm_cooling_limit);
	can2_tx_data[4] = (uint8_t)(robot_status.shooter_id1_17mm_speed_limit>>8);
    can2_tx_data[5] = (uint8_t)(robot_status.shooter_id1_17mm_speed_limit);
    can2_tx_data[6] = (uint8_t)(power_heat.shooter_id1_17mm_cooling_heat>>8);
    can2_tx_data[7] = (uint8_t)(power_heat.shooter_id1_17mm_cooling_heat);
	HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX0);
}
void send_shoot_judge_data(void)  
{
		can2_tx_header.StdId = CAN2_SHOOT_JUDGE_ID;
    can2_tx_header.IDE = CAN_ID_STD;
    can2_tx_header.RTR = CAN_RTR_DATA;
    can2_tx_header.DLC = 0x08;

    can2_tx_data[0] = (uint8_t)(((uint16_t)(shoot_data.bullet_speed*100))>>8);     //保留两位小数
    can2_tx_data[1] = (uint8_t)((uint16_t)(shoot_data.bullet_speed*100));
    can2_tx_data[2] = (uint8_t)(robot_hurt.hurt_type);
    can2_tx_data[3] = (uint8_t)(robot_status.mains_power_shooter_output);
		can2_tx_data[4] = (uint8_t)(robot_status.robot_id);
    can2_tx_data[5] = 0;
    can2_tx_data[6] = 0;
    can2_tx_data[7] = 0;
		HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX0);
}
typedef	union a__
	{
		float aaa;
		uint8_t data[4];
	}a_1;
unsigned int intValue;
	extern float cmd_yaw;;
void send_odem_data(void)  
{
	a_1 a;
		can2_tx_header.StdId = CAN2_ODEM_ID;
    can2_tx_header.IDE = CAN_ID_STD;
    can2_tx_header.RTR = CAN_RTR_DATA;
    can2_tx_header.DLC = 0x05;	

		intValue = *((unsigned int*)&(imu.wz));
	  can2_tx_data[0] = 0x1b;
	  can2_tx_data[1] = (intValue >> 24) & 0xFF;
	  can2_tx_data[2] = (intValue >> 16) & 0xFF;
	  can2_tx_data[3] = (intValue >> 8) & 0xFF;
		can2_tx_data[4] = intValue & 0xFF;
		can2_tx_data[5] = 0;
		can2_tx_data[6] = 0;
		can2_tx_data[7] = 0;
		HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX0);
		vTaskDelay(10);
	
		intValue = *((unsigned int*)&(cmd_yaw));
		can2_tx_data[0] = 0x1a;
	  can2_tx_data[1] = (intValue >> 24) & 0xFF;
	  can2_tx_data[2] = (intValue >> 16) & 0xFF;
	  can2_tx_data[3] = (intValue >> 8) & 0xFF;
		can2_tx_data[4] = intValue & 0xFF;
	  can2_tx_data[5] = 0;
	  can2_tx_data[6] = 0;
	  can2_tx_data[7] = 0;
		HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX0);
		vTaskDelay(10);
		
		a.aaa = (float)(car.position_a[0]);
		can2_tx_data[0] = 0x1c;
		can2_tx_data[1] = a.data[0];
		can2_tx_data[2] = a.data[1];
		can2_tx_data[3] = a.data[2];
		can2_tx_data[4] = a.data[3];
		can2_tx_data[5] = 0;
		can2_tx_data[6] = 0;
		can2_tx_data[7] = 0;
		HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX0);
		vTaskDelay(10);
		
		a.aaa = (float)(car.position_a[1]);
		can2_tx_data[0] = 0x1d;
		can2_tx_data[1] = a.data[0];
		can2_tx_data[2] = a.data[1];
		can2_tx_data[3] = a.data[2];
		can2_tx_data[4] = a.data[3];
		can2_tx_data[5] = 0;
		can2_tx_data[6] = 0;
		can2_tx_data[7] = 0;
		HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX0);
		vTaskDelay(10);
		
		intValue = *((unsigned int*)&(car.v[0]));
		can2_tx_data[0] = 0x1e;
		can2_tx_data[1] = (intValue >> 24) & 0xFF;
		can2_tx_data[2] = (intValue >> 16) & 0xFF;
		can2_tx_data[3] = (intValue >> 8) & 0xFF;
		can2_tx_data[4] = intValue & 0xFF;
		can2_tx_data[5] = 0;
		can2_tx_data[6] = 0;
		can2_tx_data[7] = 0;
		HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX0);
		vTaskDelay(10);
		
		intValue = *((unsigned int*)&(car.v[1]));
		can2_tx_data[0] = 0x1f;
		can2_tx_data[1] = (intValue >> 24) & 0xFF;
		can2_tx_data[2] = (intValue >> 16) & 0xFF;
		can2_tx_data[3] = (intValue >> 8) & 0xFF;
		can2_tx_data[4] = intValue & 0xFF;
		can2_tx_data[5] = 0;
		can2_tx_data[6] = 0;
		can2_tx_data[7] = 0;
		HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX0);
		vTaskDelay(10);
		
		can2_tx_data[0] = 0x2a;
		can2_tx_data[1] = (uint8_t)(car.position_b[0]>>8);
		can2_tx_data[2] = (uint8_t)(car.position_b[0]);
		can2_tx_data[3] = (uint8_t)(car.position_b[1]>>8);
		can2_tx_data[4] = (uint8_t)(car.position_b[1]);
		can2_tx_data[5] = 0;
		can2_tx_data[6] = 0;
		can2_tx_data[7] = 0;
		HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX0);
		vTaskDelay(10);
		
		
}
//void send_accel_data(void)  
//{
//		can2_tx_header.StdId = CAN2_ODEM_ID;
//    can2_tx_header.IDE = CAN_ID_STD;
//    can2_tx_header.RTR = CAN_RTR_DATA;
//    can2_tx_header.DLC = 0x08;
//	
//		intValue = *((unsigned int*)&(imu.ax));
//	  can2_tx_data[0] =  (intValue >> 24) & 0xFF;
//	  can2_tx_data[1] =  (intValue >> 16) & 0xFF;
//	  can2_tx_data[2] =  (intValue >> 8) & 0xFF;
//	  can2_tx_data[3] =  intValue & 0xFF;
//		intValue = *((unsigned int*)&(imu.ay));
//	  can2_tx_data[4] =  (intValue >> 24) & 0xFF;
//	  can2_tx_data[5] =  (intValue >> 16) & 0xFF;
//	  can2_tx_data[6] =  (intValue >> 8) & 0xFF;
//	  can2_tx_data[7] =  intValue & 0xFF;
//		HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX0);
//}

//void send_speed_data(void)  
//{
//		can2_tx_header.StdId = CAN2_ODEM_ID;
//    can2_tx_header.IDE = CAN_ID_STD;
//    can2_tx_header.RTR = CAN_RTR_DATA;
//    can2_tx_header.DLC = 0x08;
//	
//		intValue = *((unsigned int*)&(car.v[0]));
//	  can2_tx_data[0] =  (intValue >> 24) & 0xFF;
//	  can2_tx_data[1] =  (intValue >> 16) & 0xFF;
//	  can2_tx_data[2] =  (intValue >> 8) & 0xFF;
//	  can2_tx_data[3] =  intValue & 0xFF;
//		intValue = *((unsigned int*)&(car.v[1]));
//	  can2_tx_data[4] =  (intValue >> 24) & 0xFF;
//	  can2_tx_data[5] =  (intValue >> 16) & 0xFF;
//	  can2_tx_data[6] =  (intValue >> 8) & 0xFF;
//	  can2_tx_data[7] =  intValue & 0xFF;
//		HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX0);
//}

//void send_position_data(void)  
//{
//		can2_tx_header.StdId = CAN2_ODEM_ID;
//    can2_tx_header.IDE = CAN_ID_STD;
//    can2_tx_header.RTR = CAN_RTR_DATA;
//    can2_tx_header.DLC = 0x08;
//	
//	  can2_tx_data[0] =  (uint8_t)(car.position_b[0]>>8);
//	  can2_tx_data[1] =  (uint8_t)(car.position_b[0]);
//	  can2_tx_data[2] =  (uint8_t)(car.position_b[1]>>8);
//	  can2_tx_data[3] =  (uint8_t)(car.position_b[1]);
//	  can2_tx_data[4] =  0;
//	  can2_tx_data[5] =  0;
//	  can2_tx_data[6] =  0;
//	  can2_tx_data[7] =  0;
//		HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX0);
//}

motor_msg_t *get_yaw_motor_msg_point(void)
{
	return &yaw_motor_msg;
}

