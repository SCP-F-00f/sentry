/**
 * @Copyright(c),
 * @FileName:.h
 * @Author:
 * @Teammate£
 * @Version: V1.0
 * @Date:2021.2.19
 * @Description:    文件中关于飞镖的信息都没加，因为目前用不到，后期需要的话自己去协议里找哈    *hyj
 * @Note:           
 * @Others: 
**/
#ifndef _JUDGE_H
#define _JUDGE_H

#include "stm32f4xx_hal.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "struct_typedef.h"

#define    JUDGE_LEN_HEADER    5          //帧头长
#define    JUDGE_LEN_CMDID     2          //命令码长度
#define    JUDGE_LEN_TAIL      2	      //帧尾CRC16

#define SEND_MAX_LEN     200

typedef struct
{
	uint8_t SOF; 	//数据帧起始字节，固定值为 0xA5
	uint16_t data_length;
	uint8_t seq;	//包序号
	uint8_t CRC8;	//帧头 CRC8 校验
} frame_header_t;

typedef struct
{
	frame_header_t frame_header;
	uint16_t cmd_id ;    //接收pc数据成功标志位
	uint8_t data[30];
	uint16_t frame_tail;
} judge_manegement_t;

typedef enum  
{
	GAME_STATUS_DATA_ID = 0X001,		  //比赛状态数据：0x0001。发送频率：1Hz
	ROBOT_HP_ID = 0x0003,	              //比赛机器人血量数据，1Hz 周期发送, 所有机器人，ext_game_robot_HP_t;
	EVENT_DATA_ID = 0x0101,               //场地事件数据，事件改变后发送
	SUPPPLY_PROJECTILE_ACTION_ID = 0x0102,//场地补给站动作标识数据，动作改变后发送
	SUPPLEMENTARY_BILLETS_ID = 0x0103,     //请求补给站补弹数据，由参赛队发送，上限 10Hz。（RM 对抗赛尚未开放）
	REFEREE_WARNING_ID = 0x0104,          //裁判警告数据，警告发生后发送
	DATA_REMAINING_ID = 0x0105,           //飞镖发射口倒计时，1Hz 周期发送
	DART_STAUTS_ID = 0x0004,              //飞镖发射状态，飞镖发射后发送
	ROBOT_STATE_ID = 0x0201,              //机器人状态数据，10Hz 周期发送，发送范围单一，ext_game_robot_status_t;
	ROBOT_POWER_HEART_ID = 0x0202,	      //实时功率热量数据，50Hz 周期发送
	ROBOT_POSITION_ID = 0x0203,           //机器人位置数据，10Hz 发送
	BUFF_ID = 0x0204,                     //机器人增益数据，增益状态改变后发送
	AERIAL_ROBOT_ENERGY_ID = 0x0205,      //空中机器人能量状态数据，10Hz 周期发送，只有空中机器人主控发送
	ROBOT_HURT_ID = 0x0206,               //伤害状态数据，伤害发生后发送
	SHOOT_DATA_ID = 0x0207,               //实时射击数据，子弹发射后发送
	RFID_STATUS_ID = 0x0209,              //机器人 RFID 状态，1Hz 周期发送
	BULLETS_NUMBER_ID = 0X0208,           //子弹剩余发送数，空中机器人以及哨兵机器人发送，1Hz 周期发送
	DART_CLIENT_CMD_ID = 0X020A,          //飞镖机器人客户端指令书，10Hz 周期发送

	ROBOT_COMMUNICATION_ID = 0x0301,	  //机器人间交互数据，发送方触发发送，上限 10Hz
	
} POWER_MANAGEMENT_CMD_ID_e;



typedef __packed struct
{
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
} ext_game_status_t;          //比赛状态数据：0x0001。发送频率：1Hz

typedef __packed struct
{
 	uint8_t winner;
} ext_game_result_t;          //比赛结果数据：0x0002。发送频率：比赛结束后发送


typedef __packed struct
{
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP;
	uint16_t red_3_robot_HP;
	uint16_t red_4_robot_HP;
	uint16_t red_5_robot_HP;
	uint16_t red_7_robot_HP;
	uint16_t red_outpost_HP;
	uint16_t red_base_HP;
	uint16_t blue_1_robot_HP;
	uint16_t blue_2_robot_HP;
	uint16_t blue_3_robot_HP;
	uint16_t blue_4_robot_HP;
	uint16_t blue_5_robot_HP;
	uint16_t blue_7_robot_HP;
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
} ext_game_robot_HP_t;			//机器人血量数据：0x0003。发送频率：1Hz


typedef __packed struct
{
	uint32_t event_type;
} ext_event_data_t;      		//场地事件数据：0x0101。发送频率：事件改变后发送

typedef __packed struct
{
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
	uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;//补给站动作标识：0x0102。发送频率：动作改变后发送, 发送范围：己方机器人


typedef __packed struct
{
	uint8_t level;
	uint8_t foul_robot_id; 
} ext_referee_warning_t; 		//裁判警告信息：cmd_id (0x0104)。发送频率：警告发生后发送

typedef struct
{
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t remain_HP;
	uint16_t max_HP;
	uint16_t shooter_id1_17mm_cooling_rate;      //机器人1号  17mm
	uint16_t shooter_id1_17mm_cooling_limit;
	uint16_t shooter_id1_17mm_speed_limit;

	uint16_t shooter_id2_17mm_cooling_rate;      //机器人2号  17mm
	uint16_t shooter_id2_17mm_cooling_limit;
	uint16_t shooter_id2_17mm_speed_limit;
	
	uint16_t shooter_id1_42mm_cooling_rate;      //机器人    42mm
 	uint16_t shooter_id1_42mm_cooling_limit;
	uint16_t shooter_id1_42mm_speed_limit;
	uint16_t chassis_power_limit;
	uint8_t mains_power_gimbal_output ;
	uint8_t mains_power_chassis_output ;
	uint8_t mains_power_shooter_output ;

} ext_game_robot_status_t;              //比赛机器人状态：0x0201。发送频率：10Hz


typedef __packed struct
{
	uint16_t chassis_volt; 
	uint16_t chassis_current; 
	float chassis_power; 
	uint16_t chassis_power_buffer;     //底盘缓冲能量
	uint16_t shooter_id1_17mm_cooling_heat;
	uint16_t shooter_id2_17mm_cooling_heat;
	uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;				//实时功率热量数据：0x0202。发送频率：50Hz

typedef struct
{
	float x;
	float y;
	float z;
	float yaw;                //位置枪口
} ext_game_robot_pos_t;       //机器人位置：0x0203。发送频率：10Hz


typedef __packed struct
{
 	uint8_t power_rune_buff;
} ext_buff_t;            //机器人增益：0x0204。发送频率：1Hz

typedef __packed struct
{
 	uint8_t attack_time;
} ext_aerial_robot_energy_t;//空中机器人能量状态：0x0205。发送频率：10Hz


typedef __packed struct
{
	uint8_t armor_id : 4;        //受到攻击的装甲板ID
	uint8_t hurt_type : 4;       //0x2 超射速扣血；0x3 超枪口热量扣血；

} ext_robot_hurt_t;		//伤害状态：0x0206。发送频率：伤害发生后发送

typedef struct
{
	uint8_t bullet_type;
	uint8_t shooter_id;     //1：1 号 17mm 发射机构  2：2 号 17mm 发射机构   3：42mm 发射机构
	uint8_t bullet_freq;
	float bullet_speed;
} ext_shoot_data_t;     //实时射击信息：0x0207。发送频率：射击后发送


typedef __packed struct
{
	uint16_t shootnum;//统计发弹量,每触发一次则认为发射了一颗
	uint16_t bullet_remaining_num_17mm;
	uint16_t bullet_remaining_num_42mm;
	uint16_t coin_remaining_num;

} ext_bullet_remaining_t;	//子弹剩余发射数：0x0208。发送频率：10Hz 周期发送，所有机器人发送


typedef __packed struct
{
	uint32_t rfid_status;
} ext_rfid_status_t;//机器人 RFID 状态：0x0209。发送频率：1Hz，发送范围：单一机器人


void process_judge_message(uint8_t *ReadFromUsart);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);
void get_game_status(uint8_t *ReadFromUsart, ext_game_status_t *game_status);
void get_referee_warning(uint8_t *ReadFromUsart, ext_referee_warning_t *referee_warning);
void get_robot_status(uint8_t *ReadFromUsart, ext_game_robot_status_t *robot_status);
void get_power_heart(uint8_t *ReadFromUsart, ext_power_heat_data_t *power_heat);
void get_robot_buff(uint8_t *ReadFromUsart, ext_buff_t *robot_buff);
void get_robot_hurt(uint8_t *ReadFromUsart, ext_robot_hurt_t *robot_hurt);
void get_shoot_infor(uint8_t *ReadFromUsart, ext_shoot_data_t *shoot_data);
void get_bullets_remaining(uint8_t *ReadFromUsart, ext_bullet_remaining_t *bullets_remaining);

#endif
