#ifndef _RULE_H
#define _RULE_H

#include "judge.h"
#include "struct_typedef.h"

typedef struct
{
    uint16_t Pre_HP;                  //上一血量
    uint16_t hurt_HP;                 //扣血量
    uint16_t allow_power;             //控制板的阈值功率
	uint8_t Armor_Huar_flag;          //装甲伤害标志
    uint8_t Off_Line_flag;            //掉线标志
	uint8_t Excess_Speed_shoot_flag;  //超射速标志
    uint8_t Excess_Heat_flag;         //超热量标志
    uint8_t Over_Power_flag;          //超功率标志
	uint8_t Armor_BUMP_flag;          //装甲撞击标志

    float over_power_P;               //超功百分比
    uint16_t over_power_part;         //超功部分

}robot_t;


extern robot_status_t robot_status;
extern power_heat_data_t power_heat;
extern hurt_data_t robot_hurt; 


void robot_hurt_analysis(void);
void chassis_power_limit(void);
void hurt_init(void);
#endif
