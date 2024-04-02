/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HongYuJia
 * @Teammate£
 * @Version: V1.0
 * @Date:2021.4.13
 * @Description:    分析机器人扣血原因，若是因为超功率扣血，则根据规则调整功率阈值
 *                    与底盘功率控制板通信，动态调节功率阈值。
 *                     （小于阈值，一路供电；大于阈值，两路供电。）       
 * @Note:           
 * @Others: 
**/
#include "rule.h"
#include "judge.h"
#include "can1_app.h"
#include "struct_typedef.h"

robot_t Infantry;
/**
  * @brief    步兵扣血分析  
  * @author         
  * @param[in]      
  * @retval		
  * @note             *hyj     
  */
 void robot_hurt_analysis(void)
 {
   hurt_init();
   if(Infantry.Pre_HP > robot_status.current_HP )          //扣血了
   {
     switch (robot_hurt.HP_deduction_reason)                       //伤害分析
     {
        case 0x00:Infantry.Armor_Huar_flag = 1;break;
        case 0x01:Infantry.Off_Line_flag = 1;break;
        case 0x02:Infantry.Excess_Speed_shoot_flag = 1;break;
        case 0x03:Infantry.Excess_Heat_flag  = 1;break;
        case 0x04:Infantry.Over_Power_flag = 1;break;
        case 0x05:Infantry.Armor_BUMP_flag = 1;break;
        default:
          break;
     }
   }
 }
void hurt_init(void)
{
  	Infantry.Armor_Huar_flag = 0 ;     
    Infantry.Off_Line_flag = 0 ;
	  Infantry.Excess_Speed_shoot_flag = 0 ; 
    Infantry.Excess_Heat_flag  = 0 ;  
    Infantry.Over_Power_flag = 0 ;         
	  Infantry.Armor_BUMP_flag = 0 ;          
}


/**
  * @brief    底盘功率限制  
  * @author         
  * @param[in]      wwww
  * @retval		
  * @note      通过更改底盘控制板的功率阈值来实现不超功       *hyj     
  */
int Infantry_flag=0;
void chassis_power_limit(void)     
{
  Infantry.allow_power = robot_status.chassis_power_limit;         //先给默认值
  if( power_heat.chassis_power > robot_status.chassis_power_limit && Infantry.Over_Power_flag )
  {
     Infantry.hurt_HP = Infantry.Pre_HP - robot_status.current_HP;
     Infantry.over_power_P = Infantry.hurt_HP/(robot_status.maximum_HP*0.1);
    if (Infantry.over_power_P == 10 )
    {
        Infantry.over_power_part = robot_status.chassis_power_limit *0.1;
    }
    else if (Infantry.over_power_P == 20)
    {
        Infantry.over_power_part = robot_status.chassis_power_limit *0.15;
    }
    else if (Infantry.over_power_P == 40)
    {
        Infantry.over_power_part = robot_status.chassis_power_limit *0.2;
    }
    Infantry.allow_power = power_heat.chassis_power - Infantry.over_power_part;
		Infantry_flag=1;
  }
  else
  {
      Infantry.allow_power = robot_status.chassis_power_limit;
		Infantry_flag=2;
  }
  //得加最值限制
  //set_chassis_power(Infantry.allow_power*100);        //底盘控制功率   

  set_chassis_power(8000);        //80W功率   8000
  
}


