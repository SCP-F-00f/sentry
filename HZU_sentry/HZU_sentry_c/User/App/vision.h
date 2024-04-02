#ifndef _VISION_H
#define _VISION_H

#include "gimbal_task.h"

//摄像头与枪口安装的差值     单位mm         *hyj
#define yaw_install_Difference                (30.30f)     //向左补偿 
#define pitch_install_Difference              (34.50f)     //向下补偿
/* 步兵ID预编译,仅适用于调试,分区赛国赛需另外对应 */
#define    DEBUG_ID_ONE     1		//旧步兵
#define    DEBUG_ID_TWO     2		//新步兵
#define    DEBUG_ID_THREE   3		//英雄
#define    DEBUG_ID_FOUR    4		//哨兵

#define YAW 0
#define PITCH 1

#define MECH 0
#define GYRO 1

#define NOW  0
#define LAST 1

/*二阶卡尔曼*/
#define KF_ANGLE	0               
#define KF_SPEED	1
#define KF_ACCEL	2

#define    INFANTRY_DEBUG_ID    DEBUG_ID_FOUR
typedef struct  //视觉目标速度测量
{
  int delay_cnt;//计算相邻两帧目标不变持续时间,用来判断速度是否为0
  int freq;
  int last_time;//上次受到目标角度的时间
  float last_position;//上个目标角度
  float speed;//速度
  float last_speed;//上次速度
  float processed_speed;//速度计算结果
  
}speed_calc_data_t;

typedef struct
{
	float debug_y_sk;// = 38;//35;//30;//yaw移动预测系数,越大预测越多
	float debug_y_sb_sk;//哨兵预测系数
	float debug_y_sb_brig_sk;//桥头哨兵
	float debug_p_sk;//pitch移动预测系数,越大预测越多

	float debug_auto_err_y;// = 10;//15;//10;//15;//yaw角度过大关闭预测
	float debug_auto_err_p;//pitch角度过大关闭预测
	float debug_kf_delay;// = 150;//100;//200;//120;//150;//预测延时开启
	float debug_kf_speed_yl;//yaw速度过低关闭预测
	float debug_kf_speed_yl_sb;//抬头打哨兵时减小最低可开预测量
	float debug_kf_speed_yh;//yaw速度过高关闭预测
	float debug_kf_speed_pl;//pitch速度过低关闭预测
	float debug_kf_y_angcon;// = 130;//125;//115;//135;//yaw预测量限幅
	float debug_kf_p_angcon;//pitch预测量限幅

	float debug_kf_speed_ph;

}vision_KM_debug_data_t;

void vision_install_compensate(void);
void vision_Gravity_compensate(void);
void GIMBAL_InitArgument(void);
void GIMBAL_AUTO_Mode_Ctrl(gimbal_control_data_t *gimbal_control_data);
void test(gimbal_control_data_t *gimbal_control_data);
#endif

