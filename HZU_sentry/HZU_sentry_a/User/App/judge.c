/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HongYuJia 
 * @Teammate：
 * @Version: V3.0
 * @Date:2021.4.13
 * @Description:   截止目前，有用到的会先声明该结构体变量名（即不用则不声明）   
 * 					后期根据自己的需要，按照新裁判系统协议进行调整 *hyj
 * @Note:           
 * @Others: 
**/
#include "judge.h"
#include "rule.h"
#include "can2_app.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "usart.h"


judge_manegement_t judge_manegement;
extern robot_t Infantry;

ext_game_status_t game_status;              //最后几秒，爆发模式
ext_referee_warning_t referee_warning;
ext_game_robot_status_t robot_status;
ext_power_heat_data_t power_heat;
ext_buff_t robot_buff;
ext_robot_hurt_t robot_hurt;                 //辅助检查是否违规
ext_shoot_data_t shoot_data;               
ext_bullet_remaining_t bullets_remaining;  

//裁判系统官方CRC校验
//crc8 生成多项式:G(x)=x8+x5+x4+1 
const uint8_t CRC8_INIT = 0xff;
const uint8_t CRC8_TAB[256] =
{		
		0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
		0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
		0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62, 
		0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
		0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 
		0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a, 
		0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24, 
		0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
		0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd, 
		0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
		0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 
		0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73, 
		0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b, 
		0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 
		0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
		0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35, 
};
const uint16_t CRC_INIT = 0xffff;
const uint16_t wCRC_Table[256] =
{

		0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
		0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
		0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
		0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
		0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
		0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
		0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
		0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
		0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
		0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
		0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
		0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
		0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
		0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
		0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
		0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
		0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
		0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
		0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
		0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
		0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
		0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
		0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
		0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
		0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
		0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
		0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
		0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
		0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
		0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
		0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
		0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78

};

uint8_t Get_CRC8_Check_Sum( uint8_t *pchMessage, uint16_t dwLength, uint8_t ucCRC8 )
{
	uint8_t ucIndex;
	while (dwLength--)
	{
		ucIndex = ucCRC8^(*pchMessage++);
		ucCRC8 = CRC8_TAB[ucIndex];
	}
	return ucCRC8;
}
/*
 * Descriptions: CRC8 验证函数
 * Input: Data to Verify,Stream length = Data + checksum
 * Output: True or False (CRC Verify Result)
*/
uint32_t Verify_CRC8_Check_Sum( uint8_t *pchMessage, uint16_t dwLength)
{
	uint8_t ucExpected = 0;
	if (pchMessage == 0 || dwLength <= 2)
	{
		return 0;
	}
	ucExpected = Get_CRC8_Check_Sum( pchMessage, dwLength-1, CRC8_INIT);
	return ( ucExpected == pchMessage[dwLength-1] );
}
/*
 * Descriptions: append CRC8 to the end of data
 * Input: Data to CRC and append,Stream length = Data + checksum
 * Output: True or False (CRC Verify Result)
 * 用于上传自定义数据、机器人间交互、场地交互         *hyj
 * 
*/
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
	unsigned char ucCRC = 0;
	if ((pchMessage == 0) || (dwLength <= 2)) return;
	ucCRC = Get_CRC8_Check_Sum ( (unsigned char *)pchMessage, dwLength-1, CRC8_INIT);
	pchMessage[dwLength-1] = ucCRC;
}
/*
 * Descriptions: CRC16 校验函数
 * Input: Data to check,Stream length, initialized checksum     //要检查的数据、流长度、初始化校验和
 * Output: CRC checksum                                         //checksum:检查和(用以校验数据项的和)
*/ 
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
{
    uint8_t chData;
	if (pchMessage == NULL)
	{
		return 0xFFFF;
	}
	while(dwLength--)
	{
		chData = *pchMessage++;
		(wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
	}
	return wCRC;
}
/*
 * Descriptions: CRC16 验证函数
 * Input: Data to Verify,Stream length = Data + checksum
 * Output: True or False (CRC Verify Result)
*/ 
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
	uint16_t wExpected = 0;
	if ((pchMessage == NULL) || (dwLength <= 2))
	{
		return 0;
	}
	wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC_INIT);
	return ( (wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}
/*
 * Descriptions: 在数据末尾追加CRC16
 * Input: Data to CRC and append,Stream length = Data + checksum
 * Output: True or False (CRC Verify Result)
 * 用于上传自定义数据、机器人间交互、场地交互          *hyj
*/
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength)
{
	uint16_t wCRC = 0;
	if ((pchMessage == NULL) || (dwLength <= 2))
	{
		return;
	} 
	wCRC = Get_CRC16_Check_Sum ( (uint8_t *)pchMessage, dwLength-2, CRC_INIT );
	pchMessage[dwLength-2] = (uint8_t)(wCRC & 0x00ff);
	pchMessage[dwLength-1] = (uint8_t)((wCRC >> 8)& 0x00ff);
}


/**
  * @brief          
  * @author 
  * @retval	
  * @note       获取电源管理模块发送的数据，中断处理防止数据覆盖，115200
*/
void process_judge_message(uint8_t *ReadFromUsart)
{
	judge_manegement.frame_header.SOF = ReadFromUsart[0];
	if(judge_manegement.frame_header.SOF == 0XA5 )
	{
		//帧头CRC8校验
		if (Verify_CRC8_Check_Sum( ReadFromUsart, JUDGE_LEN_HEADER ) )
		{
			//统计一帧数据长度,用于CR16校验
			uint16_t judge_length = ReadFromUsart[ 1 ] + JUDGE_LEN_HEADER + JUDGE_LEN_CMDID + JUDGE_LEN_TAIL;
			//帧尾CRC16校验
			if(Verify_CRC16_Check_Sum(ReadFromUsart,judge_length) )
			{
				judge_manegement.frame_header.data_length = ReadFromUsart[2]<<8|ReadFromUsart[1];
				judge_manegement.cmd_id = ReadFromUsart[6]<<8|ReadFromUsart[5];
				switch(judge_manegement.cmd_id)
				{        //都别删，回校更新完裁判系统固件，检测后，会再整理     *hyj
					case GAME_STATUS_DATA_ID:get_game_status(ReadFromUsart,&game_status);break;
					case REFEREE_WARNING_ID:get_referee_warning(ReadFromUsart,&referee_warning);break;
					case ROBOT_STATE_ID:{
						if(robot_status.remain_HP !=  (ReadFromUsart[10]<<8|ReadFromUsart[9]))
						{
							Infantry.Pre_HP = robot_status.remain_HP;                       //获取前一血量  
						}
						get_robot_status(ReadFromUsart,&robot_status);}
						break;	
					case ROBOT_POWER_HEART_ID:get_power_heart(ReadFromUsart,&power_heat);break;
					case BUFF_ID:get_robot_buff(ReadFromUsart,&robot_buff);break;
					case ROBOT_HURT_ID:get_robot_hurt(ReadFromUsart,&robot_hurt);robot_hurt_analysis();break;
					case SHOOT_DATA_ID:get_shoot_infor(ReadFromUsart,&shoot_data);bullets_remaining.shootnum++;break;
					case BULLETS_NUMBER_ID:get_bullets_remaining(ReadFromUsart,&bullets_remaining);break;
					default :break;
				}
				if(*(ReadFromUsart + JUDGE_LEN_HEADER + JUDGE_LEN_CMDID + judge_manegement.frame_header.data_length + JUDGE_LEN_TAIL) == 0xA5)
				{
					//如果一个数据包出现了多帧数据,则再次读取
					process_judge_message((ReadFromUsart + JUDGE_LEN_HEADER + JUDGE_LEN_CMDID + judge_manegement.frame_header.data_length + JUDGE_LEN_TAIL));
				}
			}
		}
	}

}

/**********根据裁判系统通信协议格式———— 从第7位开始为有效数据位*********hyj*******/
void get_game_status(uint8_t *ReadFromUsart, ext_game_status_t *game_status)   //比赛状态数据
{
	game_status->game_type = ReadFromUsart[7] & 0x0f;
	game_status->game_progress = (ReadFromUsart[7] & 0xf0) >> 4 ;              //先屏蔽无效位，再移       *hyj
	game_status->stage_remain_time = ReadFromUsart[9]<<8|ReadFromUsart[8];
	char SourceDataH[64] ;
	char SourceDataL[32] ;
	sprintf(SourceDataH,"%d", (ReadFromUsart[17]<<24 | ReadFromUsart[16]<<16 | ReadFromUsart[15]<<8 | ReadFromUsart[14]) );
	sprintf(SourceDataL,"%d", (ReadFromUsart[13]<<24 | ReadFromUsart[12]<<16 | ReadFromUsart[11]<<8 | ReadFromUsart[10]) );   
	strcat(SourceDataH,SourceDataL);      
	game_status->SyncTimeStamp = atoi(SourceDataH);                                        //拼接64位，待测

}
void get_referee_warning(uint8_t *ReadFromUsart, ext_referee_warning_t *referee_warning)   //裁判警告数据，警告发生后发送
{
	referee_warning->level = ReadFromUsart[7];
	referee_warning->foul_robot_id = ReadFromUsart[8];

}
void get_robot_status(uint8_t *ReadFromUsart, ext_game_robot_status_t *robot_status)     //获取机器人状态，单一发送
{                                                                      
	robot_status->robot_id = ReadFromUsart[7];
	robot_status->robot_level = ReadFromUsart[8];
	robot_status->remain_HP = ReadFromUsart[10]<<8|ReadFromUsart[9];
	robot_status->max_HP = ReadFromUsart[12]<<8|ReadFromUsart[11];
	robot_status->shooter_id1_17mm_cooling_rate = ReadFromUsart[14]<<8|ReadFromUsart[13];
	robot_status->shooter_id1_17mm_cooling_limit = ReadFromUsart[16]<<8|ReadFromUsart[15];
	robot_status->shooter_id1_17mm_speed_limit = ReadFromUsart[18] <<8|ReadFromUsart[17];
	
	robot_status->shooter_id2_17mm_cooling_rate = ReadFromUsart[20]<<8|ReadFromUsart[19];
	robot_status->shooter_id2_17mm_cooling_limit = ReadFromUsart[22]<<8|ReadFromUsart[21];
	robot_status->shooter_id2_17mm_speed_limit = ReadFromUsart[24]<<8|ReadFromUsart[23];
	
	robot_status->shooter_id1_42mm_cooling_rate = ReadFromUsart[26]<<8|ReadFromUsart[25];
	robot_status->shooter_id1_42mm_cooling_limit = ReadFromUsart[28]<<8|ReadFromUsart[27];
	robot_status->shooter_id1_42mm_speed_limit = ReadFromUsart[30]<<8|ReadFromUsart[29];
	
	robot_status->chassis_power_limit = ReadFromUsart[32] <<8|ReadFromUsart[31];
	robot_status->mains_power_gimbal_output = ReadFromUsart[33] & 0x01 ;             //先屏蔽无效位，再移到想要的位数   *hyj
	robot_status->mains_power_chassis_output = (ReadFromUsart[33] & 0x02) >>1;
	robot_status->mains_power_shooter_output = (ReadFromUsart[33] & 0x04) >>2;

}
void get_power_heart(uint8_t *ReadFromUsart, ext_power_heat_data_t *power_heat)         //实时功率热量数据
{
	power_heat->chassis_volt = ReadFromUsart[8]<<8|ReadFromUsart[7];
	power_heat->chassis_current = ReadFromUsart[10]<<8|ReadFromUsart[9];
	int32_t SourceData = ReadFromUsart[14]<<24|ReadFromUsart[13]<<16|ReadFromUsart[12]<<8|ReadFromUsart[11];
	memcpy((void *)&power_heat->chassis_power, (void *)&SourceData, sizeof(power_heat->chassis_power));
	power_heat->chassis_power_buffer = ReadFromUsart[16]<<8|ReadFromUsart[15];
	power_heat->shooter_id1_17mm_cooling_heat = ReadFromUsart[18]<<8|ReadFromUsart[17];
	power_heat->shooter_id2_17mm_cooling_heat = ReadFromUsart[20]<<8|ReadFromUsart[19];
	power_heat->shooter_id1_42mm_cooling_heat = ReadFromUsart[22]<<8|ReadFromUsart[21];
}
void get_robot_buff(uint8_t *ReadFromUsart, ext_buff_t *robot_buff)   //机器人增益数据，增益状态改变后发送
{
	robot_buff->power_rune_buff = ReadFromUsart[7] & 0x0f ;
}
void get_robot_hurt(uint8_t *ReadFromUsart, ext_robot_hurt_t *robot_hurt)   //伤害状态数据，伤害发生后发
{
	robot_hurt->armor_id = ReadFromUsart[7] & 0x0f;
	robot_hurt->hurt_type = (ReadFromUsart[7] & 0xf0) >>4 ;             //只能检测到掉线扣血、装甲打击扣血   后面得重新刷固件      *hyj
	
}
void get_shoot_infor(uint8_t *ReadFromUsart, ext_shoot_data_t *shoot_data)              //实时射击信息 
{
	shoot_data->bullet_type = ReadFromUsart[7];
	shoot_data->shooter_id = ReadFromUsart[8];
	shoot_data->bullet_freq = ReadFromUsart[9];
	int32_t SourceData = ReadFromUsart[13]<<24 | ReadFromUsart[12]<<16 | ReadFromUsart[11]<<8 | ReadFromUsart[10];
	memcpy((void *)&shoot_data->bullet_speed, (void *)&SourceData, sizeof(shoot_data->bullet_speed));
}
void get_bullets_remaining(uint8_t *ReadFromUsart, ext_bullet_remaining_t *bullets_remaining)   //子弹剩余发射数目
{
	bullets_remaining->bullet_remaining_num_17mm = ReadFromUsart[8]<<8|ReadFromUsart[7];
	bullets_remaining->bullet_remaining_num_42mm = ReadFromUsart[10]<<8|ReadFromUsart[9];
	bullets_remaining->coin_remaining_num = ReadFromUsart[12]<<8|ReadFromUsart[11];
}


