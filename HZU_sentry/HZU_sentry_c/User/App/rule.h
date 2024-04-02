#ifndef _RULE_H
#define _RULE_H

#include "can2_app.h"
#include "struct_typedef.h"
 
//摩擦轮映射参数     *hyj
#define shoot_K		28
#define shoot_B   130

typedef struct
{
	uint8_t Excess_Heat_flag;         //超热量标志
	uint8_t Excess_Speed_shoot_flag;  //超射速标志
    uint16_t residual_heat;           //剩余热量
	uint8_t allow_shoot_flag;         //允许发射标志
	uint8_t allow_shoot;			  //热量允许的发弹量
	uint16_t allow_shoot_speed;       //合理的射速
	uint16_t allow_fri_speed;		  //合理的摩擦轮速度

}shoot_real_mag;

void shoot_limit(void);
void shoot_heat_limit(void);
void shoot_speed_limit(void);

#endif
