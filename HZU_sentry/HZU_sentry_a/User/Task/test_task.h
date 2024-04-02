#ifndef TEST_TASK_H
#define TEST_TASK_H

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
#include "judge.h"

#define TRANSMIT_SHOW_DATA_TIME       (35)     //上限传速30HZ

/* 
	交互数据，包括一个统一的数据段头结构，
	包含了内容 ID，发送者以及接受者的 ID 和内容数据段，
	整个交互数据的包总共长最大为 128 个字节，
	减去 frame_header,cmd_id,frame_tail 以及数据段头结构的 6 个字节，
	故而发送的内容数据段最大为 113。
	整个交互数据 0x0301 的包上行频率为 10Hz。
	
机器人 ID：
	1，英雄(红)；
	2，工程(红)；
	3/4/5，步兵(红)；
	6，空中(红)；
	7，哨兵(红)；
	
	101，英雄(蓝)；
	102，工程(蓝)；
	103/104/105，步兵(蓝)；
	106，空中(蓝)；
	107，哨兵(蓝)。 
客户端 ID： 
	0x0101 英雄操作手客户端(红) ；
	0x0102 工程操作手客户端(红 )；
	0x0103/0x0104/0x0105，步兵操作手客户端(红)；
	0x0106，空中操作手客户端(红)； 

	0x0165，英雄操作手客户端(蓝)；
	0x0166，工程操作手客户端(蓝)；
	0x0167/0x0168/0x0169，操作手客户端步兵(蓝)；
	0x016A，空中操作手客户端(蓝)。 
*/
/* 交互数据接收信息：0x0301  */
typedef  struct 
{ 
	uint16_t data_cmd_id;    
	uint16_t send_ID;    
	uint16_t receiver_ID; 
} ext_student_interactive_header_data_t; 

/* 
	客户端 客户端自定义数据：cmd_id:0x0301。内容 ID:0xD180
	发送频率：上限 10Hz

	字节偏移量 	大小 	说明 				备注 
	0 			2 		数据的内容 ID 		0xD180 
	2 			2 		发送者的 ID 		需要校验发送者机器人的 ID 正确性 
	4 			2 		客户端的 ID 		只能为发送者机器人对应的客户端 
	6 			4 		自定义浮点数据 1 	 
	10 			4 		自定义浮点数据 2 	 
	14 			4 		自定义浮点数据 3 	 
	18 			1 		自定义 8 位数据 4 	 

*/

//typedef  struct
//{
//	uint8_t graphic_name[3];     //图形名（在删除，修改等操作中，作为客户端的索引）    
//	//图像配置  （协议上有可参考）
//	uint32_t operate_tpye:3;     //图形操作
//	uint32_t graphic_tpye:3;     //图形类型
//	uint32_t layer:4;            //图层数
//	uint32_t color:4;            //颜色
//    
//	uint32_t start_angle:9;      //起始角度
//	uint32_t end_angle:9;        //终止角度

//	uint32_t width:10;           //线宽
//	uint32_t start_x:11;         //起点x坐标
//	uint32_t start_y:11;         //起点y坐标

//	uint32_t radius:10;          //字体大小或者半径
//	uint32_t end_x:11;           //终点x坐标
//	uint32_t end_y:11;           //终点y坐标

//}graphic_data_struct_t; //图形数据

typedef struct
{
   graphic_data_struct_t    grapic_data_struct;    //可变成数组，传多个数据

}ext_client_custom_graphic_single_t;


typedef struct
{
	uint8_t SOF; 	//数据帧起始字节，固定值为 0xA5
	uint16_t data_length;
	uint8_t seq;	//包序号
	uint8_t CRC8;	//帧头 CRC8 校验
} frame_header_t;
//帧头  命令码   数据段头结构  数据段   帧尾
//上传客户端

typedef  struct
{
	frame_header_t   						txFrameHeader;  //帧头
	uint16_t		 						CmdID;          //命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	ext_client_custom_graphic_single_t      clientData;     //客户端数据段
	uint16_t		 						FrameTail;      //帧尾
}ext_Send_User_Data_t;


int Operator_ID(void);
void Show_User_Data(void);


#endif



