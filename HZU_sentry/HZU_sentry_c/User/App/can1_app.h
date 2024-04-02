
#ifndef CAN1_APP_H
#define CAN1_APP_H

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"

#define RATE_BUF_SIZE 6  //电机速度滑动滤波窗口大小

typedef struct 
{
	struct //匿名结构体定义结构体变量    *hyj
	{                               
		int32_t raw_value;          //编码值
		int32_t last_raw_value;     //上一编码值
		int32_t diff;               //前后编码误差
		int32_t round_cnt;          //圈数
		int32_t ecd_raw_rate;       //
		int32_t ecd_bias;           //初始位置
		int32_t filter_rate;        //角速度平均值
		uint8_t start_flag;         //初始位置标志
		int32_t buf_count;          
		int32_t filter_rate_sum;    //
		int32_t rate_buf[RATE_BUF_SIZE];
		float ecd_angle;            //角度  
	}encoder;//定义一个名为Encoder的结构体       
	
	int32_t speed_rpm;
	int32_t given_current;
	int32_t temperate;
}motor_msg_t;

typedef enum
{
	CAN1_SHOOT_STD_ID = 0x200,//万向节

	CAN1_TRIGGER_MOTOR_STD_ID = 0x203,
	CAN1_FRICTION_MOTOR1_STD_ID = 0x201,
	CAN1_FRICTION_MOTOR2_STD_ID = 0x202,
	
} can1_msg_id_e;

extern motor_msg_t trigger_motor_msg;
extern motor_msg_t fric_motor1_msg;
extern motor_msg_t fric_motor2_msg;

void can1_message_progress( CAN_RxHeaderTypeDef *pHeader, uint8_t aData[]);

void set_shoot_behaviour(int16_t trigger_iq, \
						 int16_t fric1_iq,   \
						 int16_t fric2_iq);
void set_shoot_stop(void);
void set_thumbwheel_behaviour(int16_t thumbwheel_motor) ;
motor_msg_t *get_trigger_motor_msg_point(void);
motor_msg_t *get_fric_motor1_msg_point(void);
motor_msg_t *get_fric_motor2_msg_point(void);




#endif

