/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HongYuJia
 * @Teammate
 * @Version: V1.0
 * @Date: 2021.4.13
 * @Description:     CAN2应用（AC板信息交互、yaw轴pitch轴电机控制）           
 * @Note:           
 * @Others: 
**/
#include "test_task.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "can2_app.h"
#include "monitor_task.h"
#include "connect_task.h"
#include "semphr.h"

extern CAN_TxHeaderTypeDef can2_tx_header;
extern uint8_t can2_tx_data[CAN_TX_BUF_SIZE];
extern CAN_HandleTypeDef hcan2;

motor_msg_t yaw_motor_msg = {0};//can2 
motor_msg_t pitch_motor_msg = {0};
ext_Judge_data_t Judge_data;
odem_data_t odem_data;
a_1 aaa;
/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note    默认电机的can发送频率为1KHZ       
  */
static void gimbal_motor_msg_process(motor_msg_t *m,  uint8_t aData[])
{
	int16_t i;
	m->encoder.filter_rate_sum = 0;//进入清零
	if(m->encoder.raw_value !=  (aData[0]<<8|aData[1]))
	{
		m->encoder.last_raw_value = m->encoder.raw_value; 
	}
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
/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note           
  */
 
void can2_message_progress(CAN_RxHeaderTypeDef *pHeader, uint8_t aData[])
{
	if(pHeader == NULL || aData == NULL )
	{
		return;
	}
	switch(pHeader->StdId)
	{
		//get gimbal control 
		case CAN2_YAW_MOTOR_STD_ID:
		{
			gimbal_motor_msg_process(&yaw_motor_msg ,aData); 
			monitor.yaw_motor.time = xTaskGetTickCount();
		}break;
		case CAN2_PITCH_MOTOR_STD_ID:
		{
			gimbal_motor_msg_process(&pitch_motor_msg ,aData); 
			monitor.pitch_motor.time = xTaskGetTickCount();			
		}break;
		//获得裁判系统的信息   *hyj 
		case CAN2_SHOOT_17mm_ID:
		{
			shoot_17mm_mag(&Judge_data ,aData); 
		}break;
		case CAN2_SHOOT_JUDGE_ID:
		{
			shoot_judge_process(&Judge_data ,aData); 
		}break;
			case CAN2_ODEM_ID :
		{
			odem_data_process(&odem_data ,aData); 
		}break;
		default: break;
	}
	}

uint8_t* aData1;
unsigned int newValue;

	
void odem_data_process(odem_data_t *odem_data, uint8_t aData[])
{
	aData1 = aData;
	switch(aData[0])
	{
		case 0x1a:
		{
		newValue = (aData[1] << 24) | (aData[2] << 16) | (aData[3] << 8) | aData[4];
   odem_data->imu_yaw = *((float*)&newValue);
		}break;
	  case 0x1b:
		{
		newValue = (aData[1] << 24) | (aData[2] << 16) | (aData[3] << 8) | aData[4];
   odem_data->imu_wz = *((float*)&newValue);
		}break;
		case 0x1c:
		{
		aaa.data[0] = aData[1];
			aaa.data[1] = aData[2];
			aaa.data[2] = aData[3];
			aaa.data[3] = aData[4];
		odem_data->imu_ax = aaa.aaa;
			}break;
		case 0x1d:
		{
		aaa.data[0] = aData[1];
			aaa.data[1] = aData[2];
			aaa.data[2] = aData[3];
			aaa.data[3] = aData[4];
		odem_data->imu_ay = aaa.aaa;
			}break;
		case 0x2a:
		{
		newValue = (aData[1] << 8) | (aData[2]);
		odem_data->encoder_position[0] = *((int16_t*)&newValue);
		newValue = (aData[3] << 8) | (aData[4] );
		odem_data->encoder_position[1]= *((int16_t*)&newValue);
		}break;
		default: break;
	}
}


void shoot_17mm_mag(ext_Judge_data_t *Judge_data, uint8_t aData[])
{
	Judge_data->shooter_id1_17mm_cooling_rate   = ((int16_t)aData[0] << 8) | (int16_t)aData[1];
	Judge_data->shooter_id1_17mm_cooling_limit  = ((int16_t)aData[2] << 8) | (int16_t)aData[3];
	Judge_data->shooter_17mm_1_barrel_heat    = ((int16_t)aData[4] << 8) | (int16_t)aData[5];
	Judge_data->shooter_17mm_2_barrel_heat   = ((int16_t)aData[6] << 8) | (int16_t)aData[7];
}
void shoot_judge_process(ext_Judge_data_t *Judge_data, uint8_t aData[])
{
	Judge_data->bullet_speed = ((float)((aData[0]<<8)|aData[1]))/100;
	Judge_data->hurt_type = aData[2];
	Judge_data->mains_power_shooter_output = aData[3];
	Judge_data->robot_id = aData[4];
}

/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note  给底盘电调板发送指令，ID号为0x200?档着谭祷豂D为0x201-0x204
		   -16384 ~ +16384 对应-20A ~ +20A 电流控制精度0.00122A
		   电机的发送频率为1KHZ，转子位置0-8191 对应 360也是就分辨率为0.04394531..(转子的)
		   减速比为1：19 也就是对应转轴转速为分辨率为0.002312911       
  */
void set_gimbal_behaviour(int16_t yaw_iq, int16_t pitch_iq)  
{	
	can2_tx_header.StdId = CAN2_GIMBAL_STD_ID;
    can2_tx_header.IDE = CAN_ID_STD;
    can2_tx_header.RTR = CAN_RTR_DATA;
    can2_tx_header.DLC = 0x08;
    
    can2_tx_data[0] = (uint8_t)(pitch_iq >> 8);
    can2_tx_data[1] = (uint8_t)pitch_iq;
    can2_tx_data[2] = (uint8_t)(yaw_iq >> 8);
    can2_tx_data[3] = (uint8_t)yaw_iq;
    can2_tx_data[4] = (uint8_t)(0 >> 8);
    can2_tx_data[5] = (uint8_t)0;
    can2_tx_data[6] = (uint8_t)(0 >> 8);
    can2_tx_data[7] = (uint8_t)0;
	HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX0  );
}
/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note    
  */
void set_gimbal_stop(void)
{
    can2_tx_header.StdId = CAN2_GIMBAL_STD_ID;
    can2_tx_header.IDE = CAN_ID_STD;
    can2_tx_header.RTR = CAN_RTR_DATA;
    can2_tx_header.DLC = 0x08;
    
    can2_tx_data[0] = (uint8_t)(0 >> 8);
    can2_tx_data[1] = (uint8_t)0;
    can2_tx_data[2] = (uint8_t)(0 >> 8);
    can2_tx_data[3] = (uint8_t)0;
    can2_tx_data[4] = (uint8_t)(0 >> 8);
    can2_tx_data[5] = (uint8_t)0;
    can2_tx_data[6] = (uint8_t)(0 >> 8);
    can2_tx_data[7] = (uint8_t)0;
	HAL_CAN_AddTxMessage(&hcan2, &can2_tx_header, can2_tx_data, (uint32_t *) CAN_TX_MAILBOX0  );
}
/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note    
  */
motor_msg_t *get_yaw_motor_msg_point(void)
{
	return &yaw_motor_msg;
}
motor_msg_t *get_pitch_motor_msg_point(void)
{
	return &pitch_motor_msg;
}
