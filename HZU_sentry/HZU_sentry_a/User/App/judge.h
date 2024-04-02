/**
 * @Copyright(c),
 * @FileName:.h
 * @Author:lzf
 * @Teammate£
 * @Version: 裁判系统V1.61
 * @Date:2024.3.8
 * @Description:    
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

typedef __packed struct
{
	uint8_t SOF; 	//数据帧起始字节，固定值为 0xA5
	uint16_t data_length;
	uint8_t seq;	//包序号
	uint8_t CRC8;	//帧头 CRC8 校验
} frame_header_struct_t;

typedef __packed struct
{
	frame_header_struct_t frame_header;
	uint16_t cmd_id ;    //接收pc数据成功标志位
	uint8_t data[30];
	uint16_t frame_tail;
} judge_manegement_t;

typedef __packed enum  
{
	GAME_STATUS_DATA_ID = 0X001,		  //比赛状态数据：0x0001。发送频率：1Hz
	ROBOT_HP_ID = 0x0003,	              //比赛机器人血量数据，1Hz 周期发送, 所有机器人，ext_game_robot_HP_t;
	EVENT_DATA_ID = 0x0101,               //场地事件数据，事件改变后发送
	SUPPPLY_PROJECTILE_ACTION_ID = 0x0102,//场地补给站动作标识数据，动作改变后发送
	SUPPLEMENTARY_BILLETS_ID = 0x0103,     //请求补给站补弹数据，由参赛队发送，上限 10Hz。（RM 对抗赛尚未开放）
	REFEREE_WARNING_ID = 0x0104,          //裁判警告数据，警告发生后发送
	DATA_REMAINING_ID = 0x0105,           //飞镖发射口倒计时，1Hz 周期发送
	DART_STAUTS_ID = 0x0004,              //飞镖发射状态，飞镖发射后发送
	ROBOT_STATE_ID = 0x0201,              //机器人状态数据，10Hz 周期发送，发送范围单一，robot_status_t;
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

/* Referee Defines -----------------------------------------------------------*/
/* 比赛类型 */
#define Game_Type_RMUC    					1 //超级对抗赛 
#define Game_Type_RMUT    					2 //单项赛 
#define Game_Type_RMUA    					3 //人工智能挑战赛 
#define Game_Type_RMUL_3V3					4 //高校联盟赛3V3 
#define Game_Type_RMUL_INFANTRY_1V1 5 //高校联盟赛步兵1V1

/* 比赛阶段 */
#define Game_Progress_Unstart   0 //未开始比赛
#define Game_Progress_Prepare   1 //准备阶段
#define Game_Progress_SelfCheck 2 //自检阶段
#define Game_Progress_5sCount   3 //5s倒计时
#define Game_Progress_Battle    4 //对战中
#define Game_Progress_Calculate 5 //比赛结算中

/* 比赛结果 */
#define Game_Result_Draw    0 //平局
#define Game_Result_RedWin  1 //红方胜利
#define Game_Result_BlueWin 2 //蓝方胜利

/* 警告信息 */
#define Warning_Yellow  1 //黄牌警告
#define Warning_Red     2 //红牌警告
#define Warning_Failure 3 //判负

/* 机器人ID */
#define Robot_ID_Red_Hero         1 //红方英雄
#define Robot_ID_Red_Engineer     2 //红方工程
#define Robot_ID_Red_Infantry3    3 //红方步兵3
#define Robot_ID_Red_Infantry4    4 //红方步兵4
#define Robot_ID_Red_Infantry5    5 //红方步兵5
#define Robot_ID_Red_Aerial       6 //红方无人机
#define Robot_ID_Red_Sentry       7 //红方哨兵
#define Robot_ID_Red_Darts        8 //红方飞镖
#define Robot_ID_Red_Radar        9 //红方雷达
#define Robot_ID_Red_Outpost     10 //红方前哨战
#define Robot_ID_Red_Base				 11	//红方基地
#define Robot_ID_Blue_Hero      101 //蓝方英雄
#define Robot_ID_Blue_Engineer  102 //蓝方工程
#define Robot_ID_Blue_Infantry3 103 //蓝方步兵3
#define Robot_ID_Blue_Infantry4 104 //蓝方步兵4
#define Robot_ID_Blue_Infantry5 105 //蓝方步兵5
#define Robot_ID_Blue_Aerial    106 //蓝方无人机
#define Robot_ID_Blue_Sentry    107 //蓝方哨兵
#define Robot_ID_Blue_Darts     108 //蓝方飞镖
#define Robot_ID_Blue_Radar     109 //蓝方雷达
#define Robot_ID_Blue_Outpost   110 //蓝方前哨战
#define Robot_ID_Blue_Base			111	//蓝方基地

/* 机器人等级 */
#define Robot_Level_1 1 //1级
#define Robot_Level_2 2 //2级
#define Robot_Level_3 3 //3级

/* 扣血类型 */
#define Hurt_Type_ArmoredPlate     0 //装甲板伤害
#define Hurt_Type_ModuleOffline    1 //模块离线
#define Hurt_Type_OverShootSpeed   2 //枪口超射速
#define Hurt_Type_OverShootHeat    3 //枪管超热量
#define Hurt_Type_OverChassisPower 4 //底盘超功率
#define Hurt_Type_Collision        5 //装甲撞击

/* 弹丸类型 */
#define Projectile_Type_17mm 1 //1号17mm发射机构
#define Projectile_Type_42mm 2 //2号17mm发射机构

/* 发射机构ID */
#define Shooter_ID1_17mm 1 			//1号17mm发射机构
#define Shooter_ID2_17mm 2 			//2号17mm发射机构
#define Shooter_ID1_42mm 3 		//1号42mm发射机构

///* 飞镖信息 */
//#define Dart_State_Open     0 //飞镖闸门开启
//#define Dart_State_Close    1 //飞镖闸门关闭
//#define Dart_State_Changing 2 //正在开启或者关闭中
//#define Dart_Target_Outpost 0 //飞镖目标为前哨站
//#define Dart_Target_Base    1 //飞镖目标为基地

/* 操作手ID */
#define Cilent_ID_Red_Hero       0x0101 //红方英雄操作手
#define Cilent_ID_Red_Engineer   0x0102 //红方工程操作手
#define Cilent_ID_Red_Infantry3  0x0103 //红方步兵3操作手
#define Cilent_ID_Red_Infantry4  0x0104 //红方步兵4操作手
#define Cilent_ID_Red_Infantry5  0x0105 //红方步兵5操作手
#define Cilent_ID_Red_Aerial     0x0106 //红方飞手
#define Cilent_ID_Blue_Hero      0x0165 //蓝方英雄操作手
#define Cilent_ID_Blue_Engineer  0x0166 //蓝方工程操作手
#define Cilent_ID_Blue_Infantry3 0x0167 //蓝方步兵3操作手
#define Cilent_ID_Blue_Infantry4 0x0168 //蓝方步兵4操作手
#define Cilent_ID_Blue_Infantry5 0x0169 //蓝方步兵5操作手
#define Cilent_ID_Blue_Aerial    0x016A //蓝方飞手

#define Referee_System    			 0x8080 //柴爿系统服务器（哨兵、雷达

/* UI绘制内容cmdID */
#define UI_DataID_Delete   0x100 //客户端删除图形
#define UI_DataID_Draw1    0x101 //客户端绘制1个图形
#define UI_DataID_Draw2    0x102 //客户端绘制2个图形
#define UI_DataID_Draw5    0x103 //客户端绘制5个图形
#define UI_DataID_Draw7    0x104 //客户端绘制7个图形
#define UI_DataID_DrawChar 0x110 //客户端绘制字符图形

/* UI删除操作 */
#define UI_Delete_Invalid 0 //空操作
#define UI_Delete_Layer   1 //删除图层
#define UI_Delete_All     2 //删除所有

/* UI绘制操作 */
#define UI_Graph_invalid 0 //空操作
#define UI_Graph_Add     1 //增加图形
#define UI_Graph_Change  2 //修改图形
#define UI_Graph_Delete  3 //删除图形

/* UI图形类型 */
#define UI_Graph_Line      0 //直线
#define UI_Graph_Rectangle 1 //矩形
#define UI_Graph_Circle    2 //整圆
#define UI_Graph_Ellipse   3 //椭圆
#define UI_Graph_Arc       4 //圆弧
#define UI_Graph_Float     5 //浮点型
#define UI_Graph_Int       6 //整形
#define UI_Graph_String    7 //字符型

/* UI图形颜色 */
#define UI_Color_Main   0 //红/蓝(己方)
#define UI_Color_Yellow 1 //黄色
#define UI_Color_Green  2 //绿色
#define UI_Color_Orange 3 //橙色
#define UI_Color_Purple 4 //紫色
#define UI_Color_Pink   5 //粉色
#define UI_Color_Cyan   6 //青色
#define UI_Color_Black  7 //黑色
#define UI_Color_White  8 //白色

/* 0x000X --------------------------------------------------------------------*/
typedef __packed struct  //0x0001 比赛状态数据
{
  uint8_t game_type : 4;
  uint8_t game_progress : 4;
  uint16_t stage_remain_time;
  uint64_t SyncTimeStamp;
}game_status_t;

typedef __packed struct  //0x0002 比赛结果数据
{
 uint8_t winner;
}game_result_t;

typedef __packed struct  //0x0003 机器人血量数据
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
}game_robot_HP_t;

/* 0x010X --------------------------------------------------------------------*/
typedef __packed struct  //0x0101 场地事件数据
{
 uint32_t event_data;
}event_data_t;

typedef __packed struct  //0x0102 补给站交互
{
	uint8_t  reserved;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
	uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

typedef __packed struct	//0x0104 裁判警告信息
{
 uint8_t level;
 uint8_t offending_robot_id;
 uint8_t count;
}referee_warning_t;

typedef __packed struct  //0x0105 己方飞镖发射倒计时
{
 uint8_t dart_remaining_time;
 uint16_t dart_info;
}dart_info_t;

/* 0x020X --------------------------------------------------------------------*/
typedef __packed struct  //0x0201 己方机器人数据
{
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t current_HP;
	uint16_t maximum_HP;
	uint16_t shooter_barrel_cooling_value;
	uint16_t shooter_barrel_heat_limit;
	uint16_t chassis_power_limit; 
	uint8_t power_management_gimbal_output : 1;
	uint8_t power_management_chassis_output : 1;
	uint8_t power_management_shooter_output : 1;
}robot_status_t;

typedef __packed struct  //0x0202 实时功率热量数据
{
 uint16_t chassis_voltage;
 uint16_t chassis_current;
 float chassis_power;
 uint16_t buffer_energy;
 uint16_t shooter_17mm_1_barrel_heat;
 uint16_t shooter_17mm_2_barrel_heat;
 uint16_t shooter_42mm_barrel_heat;
}power_heat_data_t;

typedef __packed struct  //0x0203 机器人位置
{
 float x;
 float y;
 float angle;
}robot_pos_t;

typedef __packed struct  //0x0204 机器人增益
{
 uint8_t recovery_buff;
 uint8_t cooling_buff;
 uint8_t defence_buff;
 uint8_t vulnerability_buff;	
 uint16_t attack_buff;
}buff_t;

typedef __packed struct  //0x0205 空中机器人状态
{
 uint8_t airforce_status;
 uint8_t time_remain;
}air_support_data_t;

typedef __packed struct  //0x0206 受到伤害来源
{
 uint8_t armor_id : 4;
 uint8_t HP_deduction_reason : 4;
}hurt_data_t;

typedef __packed struct  //0x0207 射击信息
{
 uint8_t bullet_type;
 uint8_t shooter_number;
 uint8_t launching_frequency;
 float initial_speed;
}shoot_data_t;

typedef __packed struct  //0x0208 子弹、金币剩余数
{
 uint16_t projectile_allowance_17mm;
 uint16_t projectile_allowance_42mm;
 uint16_t remaining_gold_coin;
}projectile_allowance_t;

typedef __packed struct	//0x0209 机器人RFID(无接触自动识别技术)状态
{
 uint32_t rfid_status;
}rfid_status_t;

typedef __packed struct	//0x020A 飞镖机器人客户端指令数据
{
 uint8_t dart_launch_opening_status;
 uint8_t reserved;
 uint16_t target_change_time;
 uint16_t latest_launch_cmd_time;
}dart_client_cmd_t;

typedef __packed struct  //0x020B 己方机器人位置信息
{
 float hero_x;
 float hero_y;
 float engineer_x;
 float engineer_y;
 float standard_3_x;
 float standard_3_y;
 float standard_4_x;
 float standard_4_y;
 float standard_5_x;
 float standard_5_y;
}ground_robot_position_t;

typedef __packed struct	//0x020C 对方机器人被标记进度(0-120)
{
 uint8_t mark_hero_progress;
 uint8_t mark_engineer_progress;
 uint8_t mark_standard_3_progress;
 uint8_t mark_standard_4_progress;
 uint8_t mark_standard_5_progress;
 uint8_t mark_sentry_progress;
}radar_mark_data_t;

typedef __packed struct	//0x020D 哨兵兑换结果
												//0-10：非远程发弹量；11-14：远程发弹量
												//15-18：远程血量
{
 uint32_t sentry_info;
} sentry_info_t;

typedef __packed struct	//0x020E 雷达易伤效果
												//0-1：雷达触发双倍易伤的机会
												//2：是否触发双倍易伤
{
 uint8_t radar_info;
} radar_info_t;


/* 0x030X --------------------------------------------------------------------*/
typedef __packed struct  //0x0301 机器人相互通讯
{
 uint16_t data_cmd_id;
 uint16_t sender_id;
 uint16_t receiver_id;
 uint8_t user_data[113];				//x max = 113
}robot_interaction_data_t;

/*  ---------------------------0x0301子内容ID-----------------------------------------*/
typedef __packed struct  //0x0100 删除图层
{
	uint8_t delete_type;
	uint8_t layer;
}interaction_layer_delete_t;

typedef __packed struct  //0x0101 绘制图形
{ 
	uint8_t figure_name[3]; 
	uint32_t operate_tpye:3; 
	uint32_t figure_tpye:3; 
	uint32_t layer:4; 
	uint32_t color:4; 
	uint32_t details_a:9;
	uint32_t details_b:9;
	uint32_t width:10; 
	uint32_t start_x:11; 
	uint32_t start_y:11; 
	uint32_t details_c:10; 
	uint32_t details_d:11; 
	uint32_t details_e:11; 
}interaction_figure_t;

typedef __packed struct  //0x0102 绘制两个图形
{
 interaction_figure_t interaction_figure[2];
}interaction_figure_2_t;

typedef __packed struct  //0x0103 绘制五个图形
{
	interaction_figure_t interaction_figure[5];
}interaction_figure_3_t;

typedef __packed struct  //0x0104 绘制七个图形
{
	interaction_figure_t interaction_figure[7];
}interaction_figure_4_t;

typedef __packed struct  //绘制UI UI图形数据
{
	uint8_t  graphic_name[3];
	uint32_t operate_tpye:3;
	uint32_t graphic_tpye:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	uint32_t radius:10;
	uint32_t end_x:11;
	uint32_t end_y:11;
} graphic_data_struct_t;

typedef __packed struct	//0x0110 绘制字符
{
interaction_figure_t  grapic_data_struct;
uint8_t data[30];
} ext_client_custom_character_t;

typedef __packed struct	//0x0120 哨兵自主决策指令
{
uint32_t sentry_cmd; 
} sentry_cmd_t;

typedef __packed struct	//0x0121 雷达自主决策指令
{
uint8_t radar_cmd;
} radar_cmd_t;
/*  ---------------------------0x0301子内容ID-----------------------------------------*/

typedef __packed struct	//0x0303 选手下发坐标
{
float target_position_x;
float target_position_y;
uint8_t cmd_keyboard;
uint8_t target_robot_id;
uint8_t cmd_source;
}map_command_t;

typedef __packed struct		//0x0304 键鼠遥控数据同步
{
int16_t mouse_x;
int16_t mouse_y;
int16_t mouse_z;
int8_t left_button_down;
int8_t right_button_down;
uint16_t keyboard_value;
uint16_t reserved;
}remote_control_t;

typedef __packed struct	//0x0305 小地图接受坐标
{
uint16_t target_robot_id;
float target_position_x;
float target_position_y;
}map_robot_data_t;

typedef __packed struct	//0x0307 哨兵移动路径点
{
uint8_t intention;
uint16_t start_position_x;
uint16_t start_position_y;
int8_t delta_x[49];
int8_t delta_y[49];
uint16_t sender_id;
}map_data_t;

typedef __packed struct	//0x0308 自定义接受数据
{ 
uint16_t sender_id;
uint16_t receiver_id;
uint8_t user_data[30];
}custom_info_t;

/* 自定义绘制UI结构体 -------------------------------------------------------*/


typedef __packed struct  //绘制UI UI字符串数据
{
	uint8_t  string_name[3];
	uint32_t operate_tpye:3;
	uint32_t graphic_tpye:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	uint32_t null;
	uint8_t stringdata[30];
} string_data_struct_t;

typedef __packed struct  //绘制UI UI删除图形数据
{
	uint8_t operate_tpye;
	uint8_t layer;
} delete_data_struct_t;

typedef __packed struct //绘制UI 绘制1个图形完整结构体
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	robot_interaction_data_t Interactive_Header;
	graphic_data_struct_t Graphic[1];
	uint16_t CRC16;
} UI_Graph1_t;

typedef __packed struct //绘制UI 绘制2个图形完整结构体
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	robot_interaction_data_t Interactive_Header;
	graphic_data_struct_t Graphic[2];
	uint16_t CRC16;
} UI_Graph2_t;

typedef __packed struct //绘制UI 绘制5个图形完整结构体
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	robot_interaction_data_t Interactive_Header;
	graphic_data_struct_t Graphic[5];
	uint16_t CRC16;
} UI_Graph5_t;

typedef __packed struct //绘制UI 绘制7个图形完整结构体
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	robot_interaction_data_t Interactive_Header;
	graphic_data_struct_t Graphic[7];
	uint16_t CRC16;
} UI_Graph7_t;

typedef __packed struct //绘制UI 绘制1字符串完整结构体
{ 
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	robot_interaction_data_t Interactive_Header;
	string_data_struct_t String;
	uint16_t CRC16;
} UI_String_t;

typedef __packed struct  //绘制UI UI删除图形完整结构体
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	robot_interaction_data_t Interactive_Header;
	delete_data_struct_t Delete;
	uint16_t CRC16;
} UI_Delete_t;


//void Referee_StructInit(void);

//void Referee_SolveFifoData(uint8_t *frame);

void UI_Draw_Line(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
	                char                   GraphName[3], //图形名 作为客户端的索引
									uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
									uint8_t                Layer,        //UI图形图层 [0,9]
									uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
									uint16_t               Width,        //线宽
									uint16_t               StartX,       //起始坐标X
									uint16_t               StartY,       //起始坐标Y
									uint16_t               EndX,         //截止坐标X
									uint16_t               EndY);        //截止坐标Y
void UI_Draw_Rectangle(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
	                     char                   GraphName[3], //图形名 作为客户端的索引
									     uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
									     uint8_t                Layer,        //UI图形图层 [0,9]
							     	 	 uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
							     	   uint16_t               Width,        //线宽
							     		 uint16_t               StartX,       //起始坐标X
							     		 uint16_t               StartY,       //起始坐标Y
							     		 uint16_t               EndX,         //截止坐标X
							     		 uint16_t               EndY);        //截止坐标Y
void UI_Draw_Circle(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
	                  char                   GraphName[3], //图形名 作为客户端的索引
									  uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
									  uint8_t                Layer,        //UI图形图层 [0,9]
							     	uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
										uint16_t               Width,        //线宽
										uint16_t               CenterX,      //圆心坐标X
							      uint16_t               CenterY,      //圆心坐标Y
										uint16_t               Radius);      //半径
void UI_Draw_Ellipse(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
	                   char                   GraphName[3], //图形名 作为客户端的索引
									   uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
									   uint8_t                Layer,        //UI图形图层 [0,9]
							     	 uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
										 uint16_t               Width,        //线宽
										 uint16_t               CenterX,      //圆心坐标X
							       uint16_t               CenterY,      //圆心坐标Y
										 uint16_t               XHalfAxis,    //X半轴长
										 uint16_t               YHalfAxis);   //Y半轴长
void UI_Draw_Arc(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
	               char                   GraphName[3], //图形名 作为客户端的索引
							   uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
								 uint8_t                Layer,        //UI图形图层 [0,9]
							   uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
								 uint16_t               StartAngle,   //起始角度 [0,360]
								 uint16_t               EndAngle,     //截止角度 [0,360]
								 uint16_t               Width,        //线宽
								 uint16_t               CenterX,      //圆心坐标X
							   uint16_t               CenterY,      //圆心坐标Y
								 uint16_t               XHalfAxis,    //X半轴长
								 uint16_t               YHalfAxis);   //Y半轴长
void UI_Draw_Float(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
	                 char                   GraphName[3], //图形名 作为客户端的索引
							     uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
								   uint8_t                Layer,        //UI图形图层 [0,9]
							     uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
									 uint16_t               NumberSize,   //字体大小
									 uint16_t               Significant,  //有效位数
									 uint16_t               Width,        //线宽
							     uint16_t               StartX,       //起始坐标X
							     uint16_t               StartY,       //起始坐标Y
									 float                  FloatData);   //数字内容
void UI_Draw_Int(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
	               char                   GraphName[3], //图形名 作为客户端的索引
							   uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
								 uint8_t                Layer,        //UI图形图层 [0,9]
							   uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
								 uint16_t               NumberSize,   //字体大小
								 uint16_t               Width,        //线宽
							   uint16_t               StartX,       //起始坐标X
							   uint16_t               StartY,       //起始坐标Y
								 int32_t                IntData);     //数字内容
void UI_Draw_String(string_data_struct_t *String,        //UI图形数据结构体指针
	                  char                  StringName[3], //图形名 作为客户端的索引
							      uint8_t               StringOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
								    uint8_t               Layer,         //UI图形图层 [0,9]
							      uint8_t               Color,         //UI图形颜色 对应UI_Color_XXX的9种颜色
										uint16_t              CharSize,      //字体大小
									  uint16_t              StringLength,  //字符串长度
									  uint16_t              Width,         //线宽
							      uint16_t              StartX,        //起始坐标X
							      uint16_t              StartY,        //起始坐标Y
										char                 *StringData);   //字符串内容

void UI_PushUp_Graphs(uint8_t Counter, void *Graphs, uint8_t RobotID);
void UI_PushUp_String(UI_String_t *String, uint8_t RobotID);
void UI_PushUp_Delete(UI_Delete_t *Delete, uint8_t RobotID);


void process_judge_message(uint8_t *ReadFromUsart);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);
void get_game_status(uint8_t *ReadFromUsart, game_status_t *game_status);
void get_referee_warning(uint8_t *ReadFromUsart, referee_warning_t *referee_warning);
void get_robot_status(uint8_t *ReadFromUsart, robot_status_t *robot_status);
void get_power_heart(uint8_t *ReadFromUsart, power_heat_data_t *power_heat);
void get_robot_buff(uint8_t *ReadFromUsart, buff_t *robot_buff);
void get_robot_hurt(uint8_t *ReadFromUsart, hurt_data_t *robot_hurt);
void get_shoot_infor(uint8_t *ReadFromUsart, shoot_data_t *shoot_data);
void get_bullets_remaining(uint8_t *ReadFromUsart, projectile_allowance_t *bullets_remaining);

#endif
