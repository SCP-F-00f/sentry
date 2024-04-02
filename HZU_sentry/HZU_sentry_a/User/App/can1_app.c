/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HongYuJia 
 * @Teammate：
 * @Version: V3.0
 * @Date:2021.4.13
 * @Description: CAN1应用（包括底盘四个电机、底盘功率控制板的通信）
 * @Note:       
 * @Others: 
**/
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can1_app.h"
#include "can2_app.h"
#include "can.h"
#include "stm32f4xx_hal_can.h"

extern CAN_RxHeaderTypeDef can1_rx_header;
extern uint8_t can1_rx_data[CAN_RX_BUF_SIZE];
extern CAN_TxHeaderTypeDef can1_tx_header;
extern uint8_t can1_tx_data[CAN_TX_BUF_SIZE];
extern CAN_HandleTypeDef hcan1;
	
motor_msg_t cm1_msg = {0};
motor_msg_t cm2_msg = {0};
motor_msg_t cm3_msg = {0};
motor_msg_t cm4_msg = {0};
/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note    默认电机的can发送频率为1KHZ       
  */
static void chassis_motor_msg_process(motor_msg_t *m, uint8_t aData[])
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
/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note           
  */
void can1_message_progress(CAN_RxHeaderTypeDef *pHeader, uint8_t aData[])
{
	if(pHeader == NULL || aData == NULL )
	{
		return;
	}
	switch(pHeader->StdId)
	{
		//get shoot control 
		case CAN1_CHASSIS_MOTOR1_STD_ID:
		{
			chassis_motor_msg_process(&cm1_msg ,aData);            
		}break;
		case CAN1_CHASSIS_MOTOR2_STD_ID:
		{
			chassis_motor_msg_process(&cm2_msg ,aData);              
		}break;
		case CAN1_CHASSIS_MOTOR3_STD_ID:
		{
			chassis_motor_msg_process(&cm3_msg ,aData);            
		}break;
		case CAN1_CHASSIS_MOTOR4_STD_ID:
		{
			chassis_motor_msg_process(&cm4_msg ,aData);            
		}break;
	}
}
/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note  给底盘电调板发送指令，ID号为0x200?底盘返回ID为0x201-0x204
		   -16384 ~ +16384 对应-20A ~ +20A 电流控制精度0.00122A
		   电机的发送频率为1KHZ，转子位置0-8191 对应 360也是就分辨率为0.04394531..(转子的)
		   减速比为1：19 也就是对应转轴转速为分辨率为0.002312911       
  */
void set_chassis_behaviour(int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)  
{
    can1_tx_header.StdId = CAN1_CHASSIS_STD_ID;
    can1_tx_header.IDE = CAN_ID_STD;
    can1_tx_header.RTR = CAN_RTR_DATA;
    can1_tx_header.DLC = 0x08;
    
    can1_tx_data[0] = (uint8_t)(cm1_iq >> 8);
    can1_tx_data[1] = (uint8_t)cm1_iq;
    can1_tx_data[2] = (uint8_t)(cm2_iq >> 8);
    can1_tx_data[3] = (uint8_t)cm2_iq;
    can1_tx_data[4] = (uint8_t)(cm3_iq >> 8);
    can1_tx_data[5] = (uint8_t)cm3_iq;
    can1_tx_data[6] = (uint8_t)(cm4_iq >> 8);
    can1_tx_data[7] = (uint8_t)cm4_iq;
	HAL_CAN_AddTxMessage(&hcan1, &can1_tx_header, can1_tx_data, (uint32_t *) CAN_TX_MAILBOX0  );
}
/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note    
  */
void set_chassis_stop(void)
{
	
    can1_tx_header.StdId = CAN1_CHASSIS_STD_ID;
    can1_tx_header.IDE = CAN_ID_STD;
    can1_tx_header.RTR = CAN_RTR_DATA;
    can1_tx_header.DLC = 0x08;
    
    can1_tx_data[0] = (uint8_t)(0 >> 8);
    can1_tx_data[1] = (uint8_t)0;
    can1_tx_data[2] = (uint8_t)(0 >> 8);
    can1_tx_data[3] = (uint8_t)0;
    can1_tx_data[4] = (uint8_t)(0 >> 8);
    can1_tx_data[5] = (uint8_t)0;
    can1_tx_data[6] = (uint8_t)(0 >> 8);
    can1_tx_data[7] = (uint8_t)0;
	HAL_CAN_AddTxMessage(&hcan1, &can1_tx_header, can1_tx_data, (uint32_t *) CAN_TX_MAILBOX0  );
}

void set_chassis_power(uint16_t temPower)    //底盘控制功率
{          
    can1_tx_header.StdId = CHASSIS_CONTROL_ID;
    can1_tx_header.IDE = CAN_ID_STD;
    can1_tx_header.RTR = CAN_RTR_DATA;
    can1_tx_header.DLC = 0x08;
    
    can1_tx_data[0] = (uint8_t)(temPower >> 8);
    can1_tx_data[1] = (uint8_t)temPower;
    can1_tx_data[2] = (uint8_t)(0 >> 8);
    can1_tx_data[3] = (uint8_t)0;
    can1_tx_data[4] = (uint8_t)(0 >> 8);
    can1_tx_data[5] = (uint8_t)0;
    can1_tx_data[6] = (uint8_t)(0 >> 8);
    can1_tx_data[7] = (uint8_t)0;
	HAL_CAN_AddTxMessage(&hcan1, &can1_tx_header, can1_tx_data, (uint32_t *) CAN_TX_MAILBOX0  );
}

/**
  * @brief          返回电机指针
  * @author         
  * @param[in] 
  * @retval	
  * @note    
  */
motor_msg_t *get_cm1_msg_point(void)
{
	return &cm1_msg;
}
motor_msg_t *get_cm2_msg_point(void)
{
	return &cm2_msg;
}
motor_msg_t *get_cm3_msg_point(void)
{
	return &cm3_msg;
}
motor_msg_t *get_cm4_msg_point(void)
{
	return &cm4_msg;
}
