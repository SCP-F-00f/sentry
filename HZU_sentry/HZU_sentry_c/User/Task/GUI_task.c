/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HongYuJia  
 * @Teammate：
 * @Version: V3.0
 * @Date:2020.4.13
 * @Description: OLED人机交互
 * @Note:       
 * @Others: 
**/
#include "GUI_task.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "oled.h"
#include "remote_app.h"
#include "GUI_task.h"
#include "tim.h"
#include "main.h"
#include "pid.h"
#include "connect_task.h"
#include "usart.h"
#include "gimbal_task.h"
#include "gpio.h"
#include "shoot_task.h"
#include "INS_task.h"
#include "rule.h"
#include "vision.h"
/**
  * @brief         主菜单选择指 示，控制星号即指示标的 跳动  人机交互总函数   
  * @author         
  * @param[in]      
  * @retval		   返回空
  * @note
  */
 
extern _tx2_control_data control_data;
extern gimbal_control_data_t gimbal_control_data;
extern motor_msg_t yaw_motor_msg;
extern motor_msg_t pitch_motor_msg;
extern INS_t INS;
extern connect_t connect_data;
extern ext_Judge_data_t Judge_data;
extern shoot_real_mag shoot_mag;
extern vision_KM_debug_data_t vision_KM;
extern float vision_K;
extern float yaw_install_k;
extern float pitch_install_k;
extern float vision_G;
static void menu_interface()            
{
	LED_P6x8Str(7,0,(uint8_t *)"        MENU        ");    //122个x坐标  8个y坐标（页）7个x坐标刚好写一个星号
    LED_P6x8Str(7,2,(uint8_t *)"bsp sensor msg");          //115个y坐标可显示20个字母，相当于一个字母约占6个y坐标
    LED_P6x8Str(7,3,(uint8_t *)"pid debugging");
    LED_P6x8Str(7,4,(uint8_t *)"judge data");
    LED_P6x8Str(7,5,(uint8_t *)"vision debug");

}
void GUI_interaction(void)     
{
    int order_num = 0;
	int order_offset = 4 - ORDER_OFFSET_VALUE;//可选数量
    int oled_x = 1;
    int oled_y[6] = {2, 3, 4, 5, 6, 7}; 
 
    LED_Fill(0x00);
    LED_P6x8Char(oled_x, oled_y[order_num], '*');  
    LED_P6x8Char(oled_x, oled_y[order_num+order_offset], ' '); 
	while(1)
	{
		vTaskDelay(10);
		LED_P6x8Char(oled_x, oled_y[order_num], '*');  
		if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/) //检测key状态（带延时消抖）
		{
			delay_ms(100);         //防止数据跳动
			if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/)
			{    
				order_num++;
				if(order_num > order_offset)        order_num = 0;//做这个判断是为了 当选项到达最后一项时，若继续按按键则跳回第一项
				if(order_num==0)  //做这个判断是为了 如果 菜单选项从最后一项跳到第一项，则让星号从最后一项跟着跳到第一项
				{ 
				  LED_P6x8Char(oled_x, oled_y[order_num],'*');
				  LED_P6x8Char(oled_x, oled_y[order_num+order_offset],' ');
				}
				else//若按键往下，则将星号往下移动，并将星号原本所在位置变为空格，即将星号去掉
				{
				  LED_P6x8Char(oled_x, oled_y[order_num],'*');
				  LED_P6x8Char(oled_x, oled_y[order_num-1],' ');
				}

			}
		}
		else if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/) 
		{
			delay_ms(100);
			if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/)
			{   
				if(order_num==0) //做这个判断是为了 如果 菜单选项从最后一项跳到第一项，则让星号从第一项跟着跳到最后一项             
				{
					LED_P6x8Char(oled_x, oled_y[order_num],' ');
					LED_P6x8Char(oled_x, oled_y[order_num+order_offset],'*');
				}
				else  //若按键往上，则将星号往上移动，并将星号原本所在位置变为空格，即将星号去掉
				{
					LED_P6x8Char(oled_x, oled_y[order_num],' ');
					LED_P6x8Char(oled_x, oled_y[order_num-1],'*');
				}
				order_num--;
				if(order_num<0)       order_num=order_offset;
			}
		}
		else if(rc_analog_key.key_ok == 1 /*|| KEY_OK == KEY_DOWN*/) 
		{
		   delay_ms(100);
		   if(rc_analog_key.key_ok == 1 /*|| KEY_OK == KEY_DOWN*/)
		   {   
			   LED_Fill(0x00);
			   switch(order_num)
			   {                  
				  case 0:bsp_senior_msg();break;
				  case 1:pid_debugging();break;
                  case 2:judge_data();break;
                  case 3:vision_debug();break;
				  default:break;
			   }
		   }
		}
		menu_interface();//界面
	}
}
/**
  * @brief          相关模块的功能检查
  * @author         
  * @param[in]      
  * @retval			返回空
  * @note
  */
static void can_device_interface(void)
{
	LED_P6x8Str(16,0,(uint8_t *)"yaw:");    //122个x坐标  8个y坐标（页） 7个x坐标刚好写一个星号
    LED_P6x8Str(16,1,(uint8_t *)"pitch:");
    LED_P6x8Str(16,2,(uint8_t *)"trigger:");
    LED_P6x8Str(16,3,(uint8_t *)"fric1:");
	LED_P6x8Str(16,4,(uint8_t *)"fric2:");	
	LED_P6x8Str(16,5,(uint8_t *)"cm1_4:");
    // LED_P6x8Str(6,5,(uint8_t *)"Y:");
    // LED_PrintValueF(18,5, (float)gimbal_control_data.gimbal_INS->yaw_angle,2);
    // LED_P6x8Str(60,5,(uint8_t *)"P:");
	// LED_PrintValueF(78,5, (float)gimbal_control_data.gimbal_INS->pitch_angle,2);
    // LED_P6x8Str(6,6,(uint8_t *)"R:");
    // LED_PrintValueF(18,6,(float)gimbal_control_data.gimbal_INS->roll_angle,2);
    // LED_P6x8Str(6,7,(uint8_t *)"z:");
	// LED_PrintValueF(18,7, (float)gimbal_control_data.gimbal_INS->gyro_z,2);

		
	// LED_PrintValueI(75,0,yaw_motor_msg.encoder.raw_value);
    // LED_PrintValueI(75,1,pitch_motor_msg.encoder.raw_value);
	// LED_PrintValueI(75,2,trigger_motor_msg.encoder.raw_value);
    // LED_PrintValueI(75,3,fric_motor1_msg.encoder.raw_value);
	// LED_PrintValueI(75,4,fric_motor2_msg.encoder.raw_value);
	


// //	LED_PrintValueI(16,6,connect_data.cm2_encode);
// //	LED_PrintValueI(75,6,connect_data.cm1_encode);
// //	LED_PrintValueI(16,7,connect_data.cm3_encode);
// //	LED_PrintValueI(75,7,connect_data.cm4_encode);
	
	LED_PrintValueI(16,6,connect_data.cm2_rate);
	LED_PrintValueI(75,6,connect_data.cm1_rate);
	LED_PrintValueI(16,7,connect_data.cm3_rate);
	LED_PrintValueI(75,7,connect_data.cm4_rate);
}

static void can_single_device_msg(motor_msg_t *m)
{
	while(1)
	{
		vTaskDelay(10);
		LED_P6x8Str(16,0,(uint8_t *)"encoder:");    //122个x坐标  8个y坐标（页） 7个x坐标刚好写一个星号
		LED_P6x8Str(16,1,(uint8_t *)"speedrpm:");
		LED_P6x8Str(16,2,(uint8_t *)"current:");
		LED_P6x8Str(16,3,(uint8_t *)"temperate:");
		LED_PrintValueI(75,0,m->encoder.raw_value);
		LED_PrintValueI(75,1,m->speed_rpm);
		LED_PrintValueI(75,2,m->given_current);
		LED_PrintValueI(75,3,m->temperate);
        LED_P6x8Str(0,5,(uint8_t *)"Y:");
        LED_PrintValueF(18,5, (float)gimbal_control_data.gimbal_INS->yaw_angle,2);
        LED_P6x8Str(60,5,(uint8_t *)"P:");
        LED_PrintValueF(78,5, (float)gimbal_control_data.gimbal_INS->pitch_angle,2);
		if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
		{
			delay_ms(100);
			if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
			{
				LED_Fill(0x00);
				break;
			}
		}
	}	
}
/**
  * @brief         电机
  * @author         
  * @param[in]      
  * @retval		   返回空
  * @note
  */
static void can_device(void)
{
    int order_num = 0;
	int order_offset = 5 - ORDER_OFFSET_VALUE;
    int order_x = 2;
    int order_y[8] = {0, 1, 2, 3, 4, 5, 6, 7}; 
    LED_Fill(0x00);
    LED_P6x8Char(order_x, order_y[order_num], '*');  
    LED_P6x8Char(order_x,order_y[order_num+order_offset], ' ');
    while(1)
    {   
		vTaskDelay(10);
        LED_P6x8Char(order_x, order_y[order_num], '*'); 
        if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/)
        {
            delay_ms(100);
            if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/)
            {
                order_num++;
                if(order_num>order_offset)        order_num = 0;
                if(order_num==0)
                {
                    LED_P6x8Char(2, order_y[order_num],'*');
                    LED_P6x8Char(2, order_y[order_num+order_offset],' ');
                }
                else
                {
                    LED_P6x8Char(2, order_y[order_num],'*');
                    LED_P6x8Char(2, order_y[order_num-1],' ');
                }
                
            }
        }
        else if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/)
        {
            delay_ms(100);
            if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/)
            {
                if(order_num==0)
                {
                    LED_P6x8Char(2, order_y[order_num],' ');
                    LED_P6x8Char(2, order_y[order_num+order_offset],'*');
                }
                else
                {
                    LED_P6x8Char(2, order_y[order_num],' ');
                    LED_P6x8Char(2, order_y[order_num-1],'*');
                }
                order_num--;
                if(order_num<0)    order_num = order_offset;
            }
        }
        if(rc_analog_key.key_ok == 1 /*|| KEY_OK == KEY_DOWN*/)
        {
            delay_ms(100);
            if(rc_analog_key.key_ok == 1 /*|| KEY_OK == KEY_DOWN*/)
            {
                LED_Fill(0x00);
                while(1)
                {
					vTaskDelay(10);
					if(rc_analog_key.key_ok == 1 /*|| KEY_OK == KEY_DOWN*/)
					{
						delay_ms(100);
						if(rc_analog_key.key_ok == 1 /*|| KEY_OK == KEY_DOWN*/)
						{	
							LED_Fill(0x00);
							switch(order_num)
							{
								case 0:can_single_device_msg(&yaw_motor_msg);break;
								case 1:can_single_device_msg(&pitch_motor_msg);break;
								case 2:can_single_device_msg(&trigger_motor_msg);break;
								case 3:can_single_device_msg(&fric_motor1_msg);break;
								case 4:can_single_device_msg(&fric_motor2_msg);break;
								default:break;
							}
						}
					}
                    if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
                    {
                        delay_ms(100);
                        if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
                        {
                            LED_Fill(0x00);
                            break;
                        }
                    }
					LED_P6x8Char(10, order_y[order_num],'>');
                    LED_P6x8Char(2, order_y[order_num], '*'); 
                    can_device_interface();
                }
            }
        }
        if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
        {
            delay_ms(100);
            if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
            {
                LED_Fill(0x00);
                break;
            }
        }
        can_device_interface();
    }
}
/**
  * @brief         遥控器 
  * @author         
  * @param[in]      
  * @retval		   返回空
  * @note
  */
static void dbus_device(void)//DBUS
{
	static uint8_t remote_key_flag = 0;
	while(1)
	{
		vTaskDelay(10);
		if(remote_key_flag==0)//remote data显示
		{
			LED_P6x8Str(16,0,(uint8_t *)"ch0:");    //122个x坐标  8个y坐标（页） 7个x坐标刚好写一个星号
			LED_P6x8Str(16,1,(uint8_t *)"ch1:");
			LED_P6x8Str(16,2,(uint8_t *)"ch2:");
			LED_P6x8Str(16,3,(uint8_t *)"ch3:");
			LED_P6x8Str(16,4,(uint8_t *)"ch4:");
			LED_P6x8Str(16,5,(uint8_t *)"s1:");
			LED_P6x8Str(16,6,(uint8_t *)"s2:");
			LED_P6x8Str(56,7,(uint8_t *)"V");//向下标志
			
			LED_PrintValueI(75,0,rc_ctrl_data.rc.ch0);
			LED_PrintValueI(75,1,rc_ctrl_data.rc.ch1);
			LED_PrintValueI(75,2,rc_ctrl_data.rc.ch2);
			LED_PrintValueI(75,3,rc_ctrl_data.rc.ch3);
			LED_PrintValueI(75,4,rc_ctrl_data.rc.ch4);
			LED_PrintValueI2(75,5,rc_ctrl_data.rc.s1);
			LED_PrintValueI2(75,6,rc_ctrl_data.rc.s2);
		}
		else if(remote_key_flag==1)//mouse key data 显示
		{
			LED_P6x8Str(16,0,(uint8_t *)"mousex:");    //122个x坐标  8个y坐标（页） 7个x坐标刚好写一个星号
			LED_P6x8Str(16,1,(uint8_t *)"mousey:");
			LED_P6x8Str(16,2,(uint8_t *)"mousez:");
			LED_P6x8Str(16,3,(uint8_t *)"mousel:");
			LED_P6x8Str(16,4,(uint8_t *)"mouser:");
			LED_P6x8Str(16,5,(uint8_t *)"keyv:");
			
			LED_PrintValueI(75,0,rc_ctrl_data.mouse.x);
			LED_PrintValueI(75,1,rc_ctrl_data.mouse.y);
			LED_PrintValueI(75,2,rc_ctrl_data.mouse.z);
			LED_PrintValueI(75,3,rc_ctrl_data.mouse.press_l);
			LED_PrintValueI2(75,4,rc_ctrl_data.mouse.press_r);
			LED_PrintValueI2(75,5,rc_ctrl_data.key.v);

		}
		if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/)
		{
			delay_ms(100);
			if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/)
			{
				remote_key_flag = 0;
				LED_Fill(0x00);
			}
		}
		else if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/)
		{
			delay_ms(100);
			if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/)
			{
				remote_key_flag = 1;
				LED_Fill(0x00);
			}
		}
		if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)//退出
		{
			delay_ms(100);         //防止数据跳动
			if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
			{
				LED_Fill(0x00);
				break;
			}
		}
	}
	
}
/**
  * @brief         激光
  * @author         
  * @param[in]      
  * @retval		   返回空
  * @note
  */
static void laser_device(void)//LASER
{
	static uint8_t mode_select_flag = 0;
	while(1)
	{
		vTaskDelay(10);
		if(mode_select_flag == 0)
		{
			LED_P6x8Str(16,0,(uint8_t *)"LASER_OFF()");
			//LASER_OFF();
		}
		else if(mode_select_flag == 1)
		{
			LED_P6x8Str(16,0,(uint8_t *)"LASER_ON()");
			//LASER_ON();
		}
		
		if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/)
		{
			delay_ms(100);
			if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/)
			{
				mode_select_flag = 0;
				LED_Fill(0x00);
			}
		}
		else if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/)
		{
			delay_ms(100);
			if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/)
			{
				mode_select_flag = 1;
				LED_Fill(0x00);
			}
		}
		if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)//退出
		{
			delay_ms(100);         //防止数据跳动
			if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
			{
				LED_Fill(0x00);
				break;
			}
		}
	}
}
/**
  * @brief         led
  * @author         
  * @param[in]      
  * @retval		   返回空
  * @note
  */
static void led_device(void)//LED
{
	static uint8_t mode_select_flag = 0;
	while(1)
	{
		vTaskDelay(10);
		if(mode_select_flag == 0)
		{
			LED_P6x8Str(16,0,(uint8_t *)"GREEDLED_OFF()");
			LED_P6x8Str(16,1,(uint8_t *)"REDLED_OFF()");
			GREEDLED_OFF();
			REDLED_OFF();
		}
		else if(mode_select_flag == 1)
		{
			LED_P6x8Str(16,0,(uint8_t *)"GREEDLED_ON()");
			LED_P6x8Str(16,1,(uint8_t *)"REDLED_ON()");
			GREEDLED_ON();
			REDLED_ON();
		}
		
		if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/)
		{
			delay_ms(100);
			if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/)
			{
				mode_select_flag = 0;
				LED_Fill(0x00);
			}
		}
		else if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/)
		{
			delay_ms(100);
			if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/)
			{
				mode_select_flag = 1;
				LED_Fill(0x00);
			}
		}
		if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)//退出
		{
			delay_ms(100);         //防止数据跳动
			if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
			{
				LED_Fill(0x00);
				break;
			}
		}
	}
}
/**
  * @brief         蜂鸣器
  * @author         
  * @param[in]      
  * @retval		   返回空
  * @note
  */
static void buzzer_device(void)//BUZZER
{
	static uint8_t mode_select_flag = 0;
	while(1)
	{
		vTaskDelay(10);
		if(mode_select_flag == 0)
		{
			LED_P6x8Str(16,0,(uint8_t *)"BUZZER_OFF()");
			buzzer1_off();
		}
		else if(mode_select_flag == 1)
		{
			LED_P6x8Str(16,0,(uint8_t *)"BUZZER_ON()");
			buzzer_on(50);
		}
		
		if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/)
		{
			delay_ms(100);
			if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/)
			{
				mode_select_flag = 0;
				LED_Fill(0x00);
			}
		}
		else if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/)
		{
			delay_ms(100);
			if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/)
			{
				mode_select_flag = 1;
				LED_Fill(0x00);
			}
		}
		if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)//退出
		{
			delay_ms(100);         //防止数据跳动
			if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
			{
				LED_Fill(0x00);
				break;
			}
		}
	}	
}
/**
  * @brief        C板陀螺仪数据
  * @author         
  * @param[in]      
  * @retval		   返回空
  * @note
  */
static void GYRO_device(void)
{
	while(1)
	{
        LED_P6x8Str(7,0,(uint8_t *)"        GYRO        ");    //122个x坐标  8个y坐标（页）7个x坐标刚好写一个星号
        LED_P6x8Str(7,1,(uint8_t *)"yaw:");
        LED_PrintValueF(58,1, INS.yaw_angle ,2);
        LED_P6x8Str(7,2,(uint8_t *)"gyro_x:");
        LED_PrintValueF(58,2, INS.gyro_x ,2);

        LED_P6x8Str(7,3,(uint8_t *)"pitch:");
        LED_PrintValueF(58,3, INS.pitch_angle ,2);
        LED_P6x8Str(7,4,(uint8_t *)"gyro_y:");
        LED_PrintValueF(58,4, INS.gyro_y ,2);

        LED_P6x8Str(7,5,(uint8_t *)"roll:");
        LED_PrintValueF(58,5, INS.roll_angle ,2);
        LED_P6x8Str(7,6,(uint8_t *)"gyro_z:");
        LED_PrintValueF(58,6, INS.gyro_z ,2);

		vTaskDelay(10);
		if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)//退出
		{
			delay_ms(100);         //防止数据跳动
			if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
			{
				LED_Fill(0x00);
				break;
			}
		}
	}	
    
}
static void shoot_device(void)
{
    while(1)
	{
        LED_P6x8Str(7,0,(uint8_t *)"     shoot    ");    //122个x坐标  8个y坐标（页）7个x坐标刚好写一个星号
        LED_P6x8Str(7,1,(uint8_t *)"Allow_V:");
        LED_PrintValueI(58,1, shoot_mag.allow_fri_speed);
        LED_P6x8Str(7,2,(uint8_t *)"Ex_V_F:");
        LED_PrintValueC(58,2, shoot_mag.Excess_Speed_shoot_flag);
        LED_P6x8Str(7,3,(uint8_t *)"V_lim:");
        LED_PrintValueI(58,3, Judge_data.shooter_id1_17mm_speed_limit);
        LED_P6x8Str(7,4,(uint8_t *)"H_lim:");
        LED_PrintValueI(58,4, Judge_data.shooter_id1_17mm_cooling_limit);
		vTaskDelay(10);
		if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)//退出
		{
			delay_ms(100);         //防止数据跳动
			if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
			{
				LED_Fill(0x00);
				break;
			}
		}
	}	
}

/**
  * @brief         bsp
  * @author         
  * @param[in]      
  * @retval		   返回空
  * @note
  */
static void bsp_interface(void)
{
	LED_P6x8Str(7,0,(uint8_t *)"can_device_msg");      //115个y坐标可显示20个字母，相当于一个字母约占6个y坐标
    LED_P6x8Str(7,1,(uint8_t *)"dbus_device_msg");
    LED_P6x8Str(7,2,(uint8_t *)"laser_device_msg");
	LED_P6x8Str(7,3,(uint8_t *)"led_device_msg");
	LED_P6x8Str(7,4,(uint8_t *)"buzzer_device_msg");
	LED_P6x8Str(7,5,(uint8_t *)"GYRO_msg");
    LED_P6x8Str(7,6,(uint8_t *)"Shoot_msg");
}
static void bsp_senior_msg(void)
{
    int order_num = 0;
    int order_offset = 7 - ORDER_OFFSET_VALUE;//多少个可选
    int oled_x = 1;
    int oled_y[8] = {0, 1, 2, 3, 4, 5, 6, 7}; 
	
	bsp_interface();
    LED_Fill(0x00);
    LED_P6x8Char(oled_x, oled_y[order_num], '*');  
    LED_P6x8Char(oled_x, oled_y[order_num+order_offset], ' '); 
	while(1)
	{
		vTaskDelay(10);
		LED_P6x8Char(oled_x, oled_y[order_num], '*');  
		if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/) //检测key状态（带延时消抖）
		{
			delay_ms(100);         //防止数据跳动
			if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/)
			{
				order_num++;
				if(order_num > order_offset)        order_num = 0;//做这个判断是为了 当选项到达最后一项时，若继续按按键则跳回第一项
				if(order_num==0)  //做这个判断是为了 如果 菜单选项从最后一项跳到第一项，则让星号从最后一项跟着跳到第一项
				{ 
					LED_P6x8Char(oled_x, oled_y[order_num],'*');
					LED_P6x8Char(oled_x, oled_y[order_num+order_offset],' ');
				}
				else//若按键往下，则将星号往下移动，并将星号原本所在位置变为空格，即将星号去掉
				{
					LED_P6x8Char(oled_x, oled_y[order_num],'*');
					LED_P6x8Char(oled_x, oled_y[order_num-1],' ');
				}
			}
		}
		else if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/) 
		{
			delay_ms(100);
			if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/)
			{   
				if(order_num==0)  //做这个判断是为了 如果 菜单选项从最后一项跳到第一项，则让星号从第一项跟着跳到最后一项
				{
					LED_P6x8Char(oled_x, oled_y[order_num],' ');
					LED_P6x8Char(oled_x, oled_y[order_num+order_offset],'*');
				}
				else//若按键往上，则将星号往上移动，并将星号原本所在位置变为空格，即将星号去掉
				{
					LED_P6x8Char(oled_x, oled_y[order_num],' ');
					LED_P6x8Char(oled_x, oled_y[order_num-1],'*');
				}
				order_num--;
				if(order_num<0)       order_num=order_offset;
			}
		}
		else if(rc_analog_key.key_ok == 1 /*|| KEY_OK == KEY_DOWN*/) 
		{
		   delay_ms(100);
		   if(rc_analog_key.key_ok == 1 /*|| KEY_OK == KEY_DOWN*/)
		   {   
			   LED_Fill(0x00);
			   switch(order_num)
			   {                  
				  case 0:can_device();break;
				  case 1:dbus_device();break;
				  case 2:laser_device();break;
				  case 3:led_device();break; 
				  case 4:buzzer_device();break;
                  case 5:GYRO_device();break;
                  case 6:shoot_device();break;
				  default:break;
			   }
		   }
		}
		if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
		{
			delay_ms(100);         //防止数据跳动
			if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
			{
				LED_Fill(0x00);
				break;
			}
		}
		bsp_interface();
	}
}
/**
  * @brief         单级pid
  * @author         
  * @param[in]      
  * @retval		   返回空
  * @note
  */
 
static void pid_single_interface(cali_pid_t *cali_pid)
{

    LED_P6x8Str(16,0,(uint8_t *)"p:");   
    LED_P6x8Str(16,1,(uint8_t *)"i:");
    LED_P6x8Str(16,2,(uint8_t *)"d:");
    LED_P6x8Str(16,3,(uint8_t *)"mode:");

    LED_PrintValueF(75,0,cali_pid->kp,2);
    LED_PrintValueF(75,1,cali_pid->ki,3); 
    LED_PrintValueF(75,2,cali_pid->kd,2);
//  LED_PrintValueF(75,3,cali_pid->mode,1);
	
	if(cali_pid->mode == PID_POSITION)
	{
		LED_P6x8Str(75,3,(uint8_t *)"POSITION");
	}
	else if(cali_pid->mode == PID_DELTA)
	{
		LED_P6x8Str(75,3,(uint8_t *)"DELTA");
	}

}
static void pid_single_debug(cali_pid_t *cali_pid)
{

    int order_num = 0;
    int order_offset = 4 - ORDER_OFFSET_VALUE;//多少个可选
    int oled_x = 1;
    int oled_y[8]={0, 1, 2, 3, 4, 5, 6, 7};
	pid_single_interface(cali_pid);
    LED_Fill(0x00);
    LED_P6x8Char(oled_x, oled_y[order_num], '*');  
    LED_P6x8Char(oled_x, oled_y[order_num+order_offset], ' ');
    while(1)
    {   
		vTaskDelay(10);
        LED_P6x8Char(oled_x, oled_y[order_num], '*'); 
        if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/)
        {
            delay_ms(100);
            if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/)
            {
                order_num++;
                if(order_num>order_offset)        order_num = 0;
                if(order_num==0)
                {
                    LED_P6x8Char(2, oled_y[order_num],'*');
                    LED_P6x8Char(2, oled_y[order_num+order_offset],' ');
                }
                else
                {
                    LED_P6x8Char(2, oled_y[order_num],'*'); 
                    LED_P6x8Char(2, oled_y[order_num-1],' '); 
                }
                
            }
        }
        else if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/)
        {
            delay_ms(100);
            if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/)
            {
                if(order_num==0)
                {
                    LED_P6x8Char(2, oled_y[order_num],' ');
                    LED_P6x8Char(2, oled_y[order_num+order_offset],'*');
                }
                else
                {
                    LED_P6x8Char(2, oled_y[order_num],' ');
                    LED_P6x8Char(2, oled_y[order_num-1],'*');
                }
                order_num--;
                if(order_num<0)    order_num = order_offset;
            }
        }
        if(rc_analog_key.key_ok == 1 /*|| KEY_OK == KEY_DOWN*/)
        {
            delay_ms(100);
            if(rc_analog_key.key_ok == 1 /*|| KEY_OK == KEY_DOWN*/)
            {
                LED_Fill(0x00);
                while(1)
                {
					vTaskDelay(10);
                    LED_P6x8Char(10, oled_y[order_num],'>');
                    LED_P6x8Char(2, oled_y[order_num], '*');  
                    //LED_P6x8Char(2, PD_PID_y[pid_num+7], ' ');
                    if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/)
                    {
                        delay_ms(100);
                        if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/)
                        {
                            LED_Fill(0x00);
                            switch(order_num)
                            {
                                case 0:cali_pid->kp += (float)0.2;break;
                                case 1:cali_pid->ki += (float)0.1;break;
                                case 2:cali_pid->kd += (float)0.2;break;
                                case 3:((cali_pid->mode==0)?(cali_pid->mode=1):(cali_pid->mode=0));break;
                                default:break;
                            }
                        }
                    }
                    else if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/)
                    {
                        delay_ms(100);
                        if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/)
                        {
                            LED_Fill(0x00);
                            switch(order_num)
                            {
                                case 0:cali_pid->kp -= (float)0.2;break;
                                case 1:cali_pid->ki -= (float)0.1;break;
                                case 2:cali_pid->kd -= (float)0.2;break;
                                case 3:((cali_pid->mode==0)?(cali_pid->mode=1):(cali_pid->mode=0));break;
                                default:break;
                            }
                        }
                    }
                    if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
                    {
                        delay_ms(100);
                        if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
                        {
                            LED_Fill(0x00);
                            break;
                        }
                    }
                    pid_single_interface(cali_pid);
                }

            }
        }
        if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
        {
            delay_ms(100);
            if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
            {
                LED_Fill(0x00);
                break;
            }
        }
        pid_single_interface(cali_pid);
    }
}
/**
  * @brief         串级pid
  * @author         
  * @param[in]      
  * @retval		   返回空
  * @note
  */
static void pid_cascade_interface(cali_cascade_pid_t *cali_cascade_pid)
{
    LED_P6x8Str(16,0,(uint8_t *)"P_p:"); 
    LED_P6x8Str(16,1,(uint8_t *)"P_i:");
    LED_P6x8Str(16,2,(uint8_t *)"P_d:");
    // LED_P6x8Str(16,3,(uint8_t *)"P_mode:");
    LED_P6x8Str(16,3,(uint8_t *)"P_outmax:");
    LED_P6x8Str(16,4,(uint8_t *)"S_p:");   
    LED_P6x8Str(16,5,(uint8_t *)"S_i:");
    LED_P6x8Str(16,6,(uint8_t *)"S_d:");
    LED_P6x8Str(16,7,(uint8_t *)"S_mode:");
	
    LED_PrintValueF(75,0,cali_cascade_pid->position.kp,2);
    LED_PrintValueF(75,1,cali_cascade_pid->position.ki,2); 
    LED_PrintValueF(75,2,cali_cascade_pid->position.kd,2);
    LED_PrintValueF(75,3,cali_cascade_pid->position.output_max,2);

//  LED_PrintValueF(75,3,cali_cascade_pid->position.mode,1);
    LED_PrintValueF(75,4,cali_cascade_pid->speed.kp,1);
    LED_PrintValueF(75,5,cali_cascade_pid->speed.ki,1); 
    LED_PrintValueF(75,6,cali_cascade_pid->speed.kd,1);	
//  LED_PrintValueF(75,7,cali_cascade_pid->speed.mode,1);

	// if(cali_cascade_pid->position.mode == PID_POSITION)
	// {
	// 	LED_P6x8Str(75,3,(uint8_t *)"POSITION");
	// }
	// else if(cali_cascade_pid->position.mode == PID_DELTA)
	// {
	// 	LED_P6x8Str(75,3,(uint8_t *)"DELTA");
	// }
	
	if(cali_cascade_pid->speed.mode == 	PID_POSITION)
	{
		LED_P6x8Str(75,7,(uint8_t *)"POSITION");
	}
	else if(cali_cascade_pid->speed.mode == PID_DELTA)
	{
		LED_P6x8Str(75,7,(uint8_t *)"DELTA");
	}

}
static void pid_cascade_debug(cali_cascade_pid_t *cali_cascade_pid)
{
    int order_num = 0;
    int order_offset = 8 - ORDER_OFFSET_VALUE;//多少个可选
    int oled_x = 1;
    int oled_y[8]={0, 1, 2, 3, 4, 5, 6, 7};
	pid_cascade_interface(cali_cascade_pid);
    LED_Fill(0x00);
    LED_P6x8Char(oled_x, oled_y[order_num], '*');  
    LED_P6x8Char(oled_x, oled_y[order_num+order_offset], ' ');
    while(1)
    {
		vTaskDelay(10);
        LED_P6x8Char(oled_x, oled_y[order_num], '*'); 
        if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/)
        {
            delay_ms(100);
            if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/)
            {
                order_num++;
                if(order_num>order_offset)        order_num = 0;
                if(order_num==0)
                {
                    LED_P6x8Char(2, oled_y[order_num],'*');
                    LED_P6x8Char(2, oled_y[order_num+order_offset],' ');
                }
                else
                {
                    LED_P6x8Char(2, oled_y[order_num],'*'); 
                    LED_P6x8Char(2, oled_y[order_num-1],' '); 
                }
                
            }
        }
        else if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/)
        {
            delay_ms(100);
            if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/)
            {
                if(order_num==0)
                {
                    LED_P6x8Char(2, oled_y[order_num],' ');
                    LED_P6x8Char(2, oled_y[order_num+order_offset],'*');
                }
                else
                {
                    LED_P6x8Char(2, oled_y[order_num],' ');
                    LED_P6x8Char(2, oled_y[order_num-1],'*');
                }
                order_num--;
                if(order_num<0)    order_num = order_offset;
            }
        }
        if(rc_analog_key.key_ok == 1 /*|| KEY_OK == KEY_DOWN*/)
        {
            delay_ms(100);
            if(rc_analog_key.key_ok == 1 /*|| KEY_OK == KEY_DOWN*/)
            {
                LED_Fill(0x00);
                while(1)
                {
					vTaskDelay(10);
                    LED_P6x8Char(10, oled_y[order_num],'>');
                    LED_P6x8Char(2, oled_y[order_num], '*');  
                    //LED_P6x8Char(2, PD_PID_y[pid_num+7], ' ');
                    if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/)
                    {
                        delay_ms(100);
                        if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/)
                        {
                            LED_Fill(0x00);
                            switch(order_num)
                            {
                                case 0:cali_cascade_pid->position.kp += (float)1;break;
                                // case 0:shoot_K += 1;break;
                                // case 1:shoot_B += 1;break;
                                case 1:cali_cascade_pid->position.ki += (float)0.01;break;
                                case 2:cali_cascade_pid->position.kd += (float)100;break;
                                // case 3:((cali_cascade_pid->position.mode==0)? \
								// 		(cali_cascade_pid->position.mode=1):  \
								// 		(cali_cascade_pid->position.mode=0));break;
                                case 3:cali_cascade_pid->position.output_max += (float)100;break;
                                case 4:cali_cascade_pid->speed.kp += (float)1;break;
                                case 5:cali_cascade_pid->speed.ki += (float)0.2;break;
                                case 6:cali_cascade_pid->speed.kd += (float)1;break;
                                case 7:((cali_cascade_pid->speed.mode==0)? \
										(cali_cascade_pid->speed.mode=1):  \
										(cali_cascade_pid->speed.mode=0));break;
                                default:break;
                            }
                        }
                    }
                    else if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/)
                    {
                        delay_ms(100);
                        if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/)
                        {
                            LED_Fill(0x00);
                            switch(order_num)
                            {
                                case 0:cali_cascade_pid->position.kp -= (float)1;break;
                                // case 0:shoot_K -= 1;break;
                                // case 1:shoot_B -= 1;break;
                                case 1:cali_cascade_pid->position.ki -= (float)0.01;break;
                                case 2:cali_cascade_pid->position.kd -= (float)100;break;
                                // case 3:((cali_cascade_pid->position.mode==0)? \
								// 		(cali_cascade_pid->position.mode=1):  \
										(cali_cascade_pid->position.mode=0));break;
                                case 3:cali_cascade_pid->position.output_max -= (float)100;break;
                                case 4:cali_cascade_pid->speed.kp -= (float)1;break;
                                case 5:cali_cascade_pid->speed.ki -= (float)0.2;break;
                                case 6:cali_cascade_pid->speed.kd -= (float)1;break;
                                case 7:((cali_cascade_pid->speed.mode==0)? \
										(cali_cascade_pid->speed.mode=1):  \
										(cali_cascade_pid->speed.mode=0));break;
                                default:break;
                            }
                        }
                    }
                    if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
                    {
                        delay_ms(100);
                        if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
                        {
                            LED_Fill(0x00);
                            break;
                        }
                    }
                    pid_cascade_interface(cali_cascade_pid);
                }

            }
        }
        if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
        {
            delay_ms(100);
            if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
            {
                LED_Fill(0x00);
                break;
            }
        }
        pid_cascade_interface(cali_cascade_pid);
    }
}
/**
  * @brief         pid
  * @author         
  * @param[in]      
  * @retval		   返回空
  * @note
  */
static void pid_interface(void)
{
    LED_P6x8Str(16,0,(uint8_t *)"yaw_pid");   
    LED_P6x8Str(16,1,(uint8_t *)"pitch_pid");
    LED_P6x8Str(16,2,(uint8_t *)"trigger_pid");
    LED_P6x8Str(16,3,(uint8_t *)"fric1_pid");	
	LED_P6x8Str(16,4,(uint8_t *)"fric2_pid");
	LED_P6x8Str(16,5,(uint8_t *)"cm_pid");
	LED_P6x8Str(16,6,(uint8_t *)"rotate_pid");
}
static void pid_debugging(void)
{

    int order_num = 0;
	int order_offset = 7 - ORDER_OFFSET_VALUE;
    int order_x = 2;
    int order_y[8] = {0, 1, 2, 3, 4, 5, 6, 7};
	pid_interface();	
    LED_Fill(0x00);
    LED_P6x8Char(order_x, order_y[order_num], '*');  
    LED_P6x8Char(order_x,order_y[order_num+order_offset], ' ');
    while(1)
    {   
		vTaskDelay(10);
        LED_P6x8Char(order_x, order_y[order_num], '*'); 
        if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/)
        {
            delay_ms(100);
            if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/)
            {
                order_num++;
                if(order_num>order_offset)        order_num = 0;
                if(order_num==0)
                {
                    LED_P6x8Char(2, order_y[order_num],'*');
                    LED_P6x8Char(2, order_y[order_num+order_offset],' ');
                }
                else
                {
                    LED_P6x8Char(2, order_y[order_num],'*');
                    LED_P6x8Char(2, order_y[order_num-1],' ');
                }
                
            }
        }
        else if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/)
        {
            delay_ms(100);
            if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/)
            {
                if(order_num==0)
                {
                    LED_P6x8Char(2, order_y[order_num],' ');
                    LED_P6x8Char(2, order_y[order_num+order_offset],'*');
                }
                else
                {
                    LED_P6x8Char(2, order_y[order_num],' ');
                    LED_P6x8Char(2, order_y[order_num-1],'*');
                }
                order_num--;
                if(order_num<0)    order_num = order_offset;
            }
        }
        if(rc_analog_key.key_ok == 1 /*|| KEY_OK == KEY_DOWN*/)
        {
            delay_ms(100);
            if(rc_analog_key.key_ok == 1 /*|| KEY_OK == KEY_DOWN*/)
            {
                LED_Fill(0x00);
                while(1)
                {
					vTaskDelay(10);
					if(rc_analog_key.key_ok == 1 /*|| KEY_OK == KEY_DOWN*/)
					{
						delay_ms(100);
						if(rc_analog_key.key_ok == 1 /*|| KEY_OK == KEY_DOWN*/)
						{
							LED_Fill(0x00);
							switch(order_num)
							{
								case 0:pid_cascade_debug(&cali_gimbal_pid.yaw_pid);break;
								case 1:pid_cascade_debug(&cali_gimbal_pid.pitch_pid);break;
								case 2:pid_cascade_debug(&cali_shoot_pid.trigger_pid);break;
								case 3:pid_single_debug(&cali_shoot_pid.fric1_pid);break;
								case 4:pid_single_debug(&cali_shoot_pid.fric2_pid);break;
								default:break;
							}
						}
					}
                    LED_P6x8Char(10, order_y[order_num],'>');
                    LED_P6x8Char(2, order_y[order_num], '*');  
                    if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
                    {
                        delay_ms(100);
                        if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
                        {
                            LED_Fill(0x00);
                            break;
                        }
                    }
                    pid_interface();
                }
            }
        }
        if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
        {
            delay_ms(100);
            if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
            {
                LED_Fill(0x00);
                break;
            }
        }
        pid_interface();
    }
}
void judge_data(void)
{
    while(1)
	{
        LED_P6x8Str(7,0,(uint8_t *)"  ***JUDGE DATA***  ");    //122个x坐标  8个y坐标（页）7个x坐标刚好写一个星号
        LED_P6x8Str(7,1,(uint8_t *)"ID:");
        LED_PrintValueC(58,1, Judge_data.robot_id);
        LED_P6x8Str(7,2,(uint8_t *)"EX_V_F:");
        LED_PrintValueC(58,2, shoot_mag.Excess_Speed_shoot_flag);
        LED_P6x8Str(7,3,(uint8_t *)"EX_H_F:");
        LED_PrintValueC(58,3, shoot_mag.Excess_Heat_flag);

        LED_P6x8Str(7,4,(uint8_t *)"sh_V:");
        LED_PrintValueI(58,4,Judge_data.bullet_speed );
        LED_P6x8Str(7,5,(uint8_t *)"heat:");
        LED_PrintValueI(58,5,Judge_data.shooter_id1_17mm_cooling_heat );

        LED_P6x8Str(7,6,(uint8_t *)"sh_V_lim");
        LED_PrintValueI(58,6, Judge_data.shooter_id1_17mm_speed_limit);
        LED_P6x8Str(7,7,(uint8_t *)"sh_H_lim");
        LED_PrintValueI(58,7, Judge_data.shooter_id1_17mm_cooling_limit);

		vTaskDelay(10);
		if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)//退出
		{
			delay_ms(100);         //防止数据跳动
			if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
			{
				LED_Fill(0x00);
				break;
			}
		}
	}	
}
/**
  * @brief         视觉卡尔曼预测参数调试     
  * @author         
  * @param[in]      
  * @retval		   返回空
  * @note
  */
void vision_KM_interface(void)
{
    LED_P6x8Str(7,0,(uint8_t *)" *YAW*    *PITCH*");    //122个x坐标  8个y坐标（页）7个x坐标刚好写一个星号
    // LED_P6x8Str(16,1,(uint8_t *)"V:");
    // LED_PrintValueF(37,1,vision_KM.debug_y_sk,0);
    LED_P6x8Str(16,1,(uint8_t *)"V_K:");
    LED_PrintValueF(37,1,vision_K,3);
    LED_P6x8Str(74,1,(uint8_t *)"V:");
    LED_PrintValueF(99,1,vision_KM.debug_p_sk,0);

    // LED_P6x8Str(16,2,(uint8_t *)"err:");
    // LED_PrintValueF(37,2,vision_KM.debug_auto_err_y,0);
     LED_P6x8Str(16,2,(uint8_t *)"yaw:");
    LED_PrintValueF(37,2,yaw_install_k,1);
    LED_P6x8Str(74,2,(uint8_t *)"err:");
    LED_PrintValueF(99,2,vision_KM.debug_auto_err_p,0);

    // LED_P6x8Str(16,3,(uint8_t *)"L_V:");
    // LED_PrintValueF(37,3,vision_KM.debug_kf_speed_yl,1);
    LED_P6x8Str(16,3,(uint8_t *)"pit:");
    LED_PrintValueF(37,3,pitch_install_k,1);
    LED_P6x8Str(74,3,(uint8_t *)"L_V:");
    LED_PrintValueF(99,3,vision_KM.debug_kf_speed_pl,1);

    // LED_P6x8Str(16,4,(uint8_t *)"H_V:");
    // LED_PrintValueF(37,4,vision_KM.debug_kf_speed_yh,0);
    LED_P6x8Str(16,4,(uint8_t *)"G:");
    LED_PrintValueF(37,4,vision_G,2);
    LED_P6x8Str(74,4,(uint8_t *)"H_V:");
    LED_PrintValueF(99,4,vision_KM.debug_kf_speed_ph,0);

    LED_P6x8Str(16,5,(uint8_t *)"lim:");
    LED_PrintValueF(37,5,vision_KM.debug_kf_y_angcon,0);
    LED_P6x8Str(74,5,(uint8_t *)"lim:");
    LED_PrintValueF(99,5,vision_KM.debug_kf_p_angcon,0);

    LED_P6x8Str(16,6,(uint8_t *)"dy_go:");
    LED_PrintValueF(40,6,vision_KM.debug_kf_delay,0);
        
}
static void vision_yaw_KM(void)
{
    int order_num = 0;
    int order_offset = 6 - ORDER_OFFSET_VALUE;//多少个可选
    int oled_x = 1;
    int oled_y[6]={ 1, 2, 3, 4, 5, 6};
    LED_Fill(0x00);
    LED_P6x8Char(oled_x, oled_y[order_num], '*');  
    LED_P6x8Char(oled_x, oled_y[order_num+order_offset], ' ');
    while(1)
    {
		vTaskDelay(10);
        LED_P6x8Char(oled_x, oled_y[order_num], '*'); 
        if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/)
        {
            delay_ms(100);
            if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/)
            {
                order_num++;
                if(order_num>order_offset)        order_num = 0;
                if(order_num==0)
                {
                    LED_P6x8Char(2, oled_y[order_num],'*');
                    LED_P6x8Char(2, oled_y[order_num+order_offset],' ');
                }
                else
                {
                    LED_P6x8Char(2, oled_y[order_num],'*'); 
                    LED_P6x8Char(2, oled_y[order_num-1],' '); 
                }
                
            }
        }
        else if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/)
        {
            delay_ms(100);
            if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/)
            {
                if(order_num==0)
                {
                    LED_P6x8Char(2, oled_y[order_num],' ');
                    LED_P6x8Char(2, oled_y[order_num+order_offset],'*');
                }
                else
                {
                    LED_P6x8Char(2, oled_y[order_num],' ');
                    LED_P6x8Char(2, oled_y[order_num-1],'*');
                }
                order_num--;
                if(order_num<0)    order_num = order_offset;
            }
        }
        if(rc_analog_key.key_ok == 1 /*|| KEY_OK == KEY_DOWN*/)
        {
            delay_ms(100);
            if(rc_analog_key.key_ok == 1 /*|| KEY_OK == KEY_DOWN*/)
            {
                LED_Fill(0x00);
                while(1)
                {
					vTaskDelay(10);
                    LED_P6x8Char(10, oled_y[order_num],'>');
                    LED_P6x8Char(2, oled_y[order_num], '*');  
                    //LED_P6x8Char(2, PD_PID_y[pid_num+7], ' ');
                    if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/)
                    {
                        delay_ms(100);
                        if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/)
                        {
                            LED_Fill(0x00);
                            switch(order_num)
                            {
                                // case 0:vision_KM.debug_y_sk += (float)1;break;
                                case 0:vision_K += (float)0.001;break;
                                case 1:yaw_install_k += (float)0.1;break;
                                case 2:pitch_install_k += (float)0.1;break;
                                case 3:vision_G += (float)1;break;
                                // case 1:vision_KM.debug_auto_err_y += (float)10;break;
                                // case 2:vision_KM.debug_kf_speed_yl += (float)0.1;break;
                                // case 3:vision_KM.debug_kf_speed_yh += (float)1;break;
                                case 4:vision_KM.debug_kf_y_angcon += (float)5;break;
                                case 5:vision_KM.debug_kf_delay += (float)10;break;
                                default:break;
                            }
                        }
                    }
                    else if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/)
                    {
                        delay_ms(100);
                        if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/)
                        {
                            LED_Fill(0x00);
                            switch(order_num)
                            {
                                // case 0:vision_KM.debug_y_sk -= (float)1;break;
                                case 0:vision_K -= (float)0.001;break;
                                case 1:yaw_install_k -= (float)0.1;break;
                                case 2:pitch_install_k -= (float)0.1;break;
                                case 3:vision_G -= (float)1;break;
                                // case 1:vision_KM.debug_auto_err_y -= (float)10;break;
                                // case 2:vision_KM.debug_kf_speed_yl -= (float)0.1;break;
                                // case 3:vision_KM.debug_kf_speed_yh -= (float)1;break;
                                case 4:vision_KM.debug_kf_y_angcon -= (float)5;break;
                                case 5:vision_KM.debug_kf_delay -= (float)10;break;
                                default:break;
                            }
                        }
                    }
                    if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
                    {
                        delay_ms(100);
                        if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
                        {
                            LED_Fill(0x00);
                            break;
                        }
                    }
                    vision_KM_interface();
                }

            }
        }
        if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
        {
            delay_ms(100);
            if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
            {
                LED_Fill(0x00);
                break;
            }
        }
        vision_KM_interface();
    }
}
static void vision_pitch_KM(void)
{
    int order_num = 0;
    int order_offset = 5 - ORDER_OFFSET_VALUE;//多少个可选
    int oled_x = 1;
    int oled_y[8]={1, 2, 3, 4, 5};
	vision_KM_interface();
    LED_Fill(0x00);
    LED_P6x8Char(oled_x, oled_y[order_num], '*');  
    LED_P6x8Char(oled_x, oled_y[order_num+order_offset], ' ');
    while(1)
    {   
		vTaskDelay(10);
        LED_P6x8Char(oled_x, oled_y[order_num], '*'); 
        if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/)
        {
            delay_ms(100);
            if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/)
            {
                order_num++;
                if(order_num>order_offset)        order_num = 0;
                if(order_num==0)
                {
                    LED_P6x8Char(2, oled_y[order_num],'*');
                    LED_P6x8Char(2, oled_y[order_num+order_offset],' ');
                }
                else
                {
                    LED_P6x8Char(2, oled_y[order_num],'*'); 
                    LED_P6x8Char(2, oled_y[order_num-1],' '); 
                }
                
            }
        }
        else if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/)
        {
            delay_ms(100);
            if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/)
            {
                if(order_num==0)
                {
                    LED_P6x8Char(2, oled_y[order_num],' ');
                    LED_P6x8Char(2, oled_y[order_num+order_offset],'*');
                }
                else
                {
                    LED_P6x8Char(2, oled_y[order_num],' ');
                    LED_P6x8Char(2, oled_y[order_num-1],'*');
                }
                order_num--;
                if(order_num<0)    order_num = order_offset;
            }
        }
        if(rc_analog_key.key_ok == 1 /*|| KEY_OK == KEY_DOWN*/)
        {
            delay_ms(100);
            if(rc_analog_key.key_ok == 1 /*|| KEY_OK == KEY_DOWN*/)
            {
                LED_Fill(0x00);
                while(1)
                {
					vTaskDelay(10);
                    LED_P6x8Char(10, oled_y[order_num],'>');
                    LED_P6x8Char(2, oled_y[order_num], '*');  
                    //LED_P6x8Char(2, PD_PID_y[pid_num+7], ' ');
                    if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/)
                    {
                        delay_ms(100);
                        if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/)
                        {
                            LED_Fill(0x00);
                            switch(order_num)
                            {
                                case 0:vision_KM.debug_p_sk += (float)1;break;
                                case 1:vision_KM.debug_auto_err_p += (float)10;break;
                                case 2:vision_KM.debug_kf_speed_pl += (float)0.1;break;
                                case 3:vision_KM.debug_kf_speed_ph += (float)1;break;
                                case 4:vision_KM.debug_kf_p_angcon += (float)5;break;
                                default:break;
                            }
                        }
                    }
                    else if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/)
                    {
                        delay_ms(100);
                        if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/)
                        {
                            LED_Fill(0x00);
                            switch(order_num)
                            {
                                case 0:vision_KM.debug_p_sk -= (float)1;break;
                                case 1:vision_KM.debug_auto_err_p -= (float)10;break;
                                case 2:vision_KM.debug_kf_speed_pl -= (float)0.1;break;
                                case 3:vision_KM.debug_kf_speed_ph -= (float)1;break;
                                case 4:vision_KM.debug_kf_p_angcon -= (float)5;break;
                                default:break;
                            }
                        }
                    }
                    if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
                    {
                        delay_ms(100);
                        if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
                        {
                            LED_Fill(0x00);
                            break;
                        }
                    }
                    vision_KM_interface();
                }

            }
        }
        if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
        {
            delay_ms(100);
            if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
            {
                LED_Fill(0x00);
                break;
            }
        }
        vision_KM_interface();
    }
}
// extern float *yaw_kf_result;
// extern float *pitch_kf_result;
static void vision_interface(void)
{
    LED_P6x8Str(7,0,(uint8_t *)"    Vision Debug  "); 
    LED_P6x8Str(16,1,(uint8_t *)"yaw");   
    LED_P6x8Str(16,2,(uint8_t *)"pitch");
    LED_P6x8Str(0,3,(uint8_t *)"Y:");
    LED_PrintValueF(12,3,control_data.yaw_dev,2);
    LED_P6x8Str(64,3,(uint8_t *)"P:");
    LED_PrintValueF(76,3,control_data.pitch_dev,2);

    // LED_P6x8Str(0,4,(uint8_t *)"KY:");
    // LED_PrintValueF(12,4,yaw_kf_result[KF_ANGLE] ,2);
    // LED_P6x8Str(64,4,(uint8_t *)"KP:");
    // LED_PrintValueF(76,4,pitch_kf_result[KF_ANGLE],2);
    //  LED_P6x8Str(0,4,(uint8_t *)"V_K:");
    // LED_PrintValueF(12,4,vision_K,3);
    LED_P6x8Str(0,5,(uint8_t *)"D:");
    LED_PrintValueF(12,5,control_data.Target_distance,2);
    
    LED_P6x8Str(0,6,(uint8_t *)"Yfdb:");
    LED_PrintValueF(30,6, (float)gimbal_control_data.gimbal_INS->yaw_angle,1);
    LED_P6x8Str(64,6,(uint8_t *)"Pfdb:");
    LED_PrintValueF(94,6,(float)gimbal_control_data.gimbal_INS->roll_angle,1);

    LED_P6x8Str(0,7,(uint8_t *)"Yset:");
    LED_PrintValueF(30,7, (float)gimbal_control_data.gimbal_yaw_set,1);
    LED_P6x8Str(64,7,(uint8_t *)"Pset:");
    LED_PrintValueF(94,7,(float)gimbal_control_data.gimbal_pitch_set,1);
}
void vision_debug(void)
{
    int order_num = 0;
	int order_offset = 2 - ORDER_OFFSET_VALUE;//可选数量
    int oled_x = 1;
    int oled_y[6] = {1, 2, 3, 4, 5, 6}; 
 
    LED_Fill(0x00);
    LED_P6x8Char(oled_x, oled_y[order_num], '*');  
    LED_P6x8Char(oled_x, oled_y[order_num+order_offset], ' '); 
	while(1)
	{
		vTaskDelay(10);
		LED_P6x8Char(oled_x, oled_y[order_num], '*');  
		if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/) //检测key状态（带延时消抖）
		{
			delay_ms(100);         //防止数据跳动
			if(rc_analog_key.key_down == 1/* || KEY_D == KEY_DOWN*/)
			{    
				order_num++;
				if(order_num > order_offset)        order_num = 0;//做这个判断是为了 当选项到达最后一项时，若继续按按键则跳回第一项
				if(order_num==0)  //做这个判断是为了 如果 菜单选项从最后一项跳到第一项，则让星号从最后一项跟着跳到第一项
				{ 
				  LED_P6x8Char(oled_x, oled_y[order_num],'*');
				  LED_P6x8Char(oled_x, oled_y[order_num+order_offset],' ');
				}
				else//若按键往下，则将星号往下移动，并将星号原本所在位置变为空格，即将星号去掉
				{
				  LED_P6x8Char(oled_x, oled_y[order_num],'*');
				  LED_P6x8Char(oled_x, oled_y[order_num-1],' ');
				}

			}
		}
		else if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/) 
		{
			delay_ms(100);
			if(rc_analog_key.key_up == 1 /*|| KEY_U == KEY_DOWN*/)
			{   
				if(order_num==0) //做这个判断是为了 如果 菜单选项从最后一项跳到第一项，则让星号从第一项跟着跳到最后一项             
				{
					LED_P6x8Char(oled_x, oled_y[order_num],' ');
					LED_P6x8Char(oled_x, oled_y[order_num+order_offset],'*');
				}
				else  //若按键往上，则将星号往上移动，并将星号原本所在位置变为空格，即将星号去掉
				{
					LED_P6x8Char(oled_x, oled_y[order_num],' ');
					LED_P6x8Char(oled_x, oled_y[order_num-1],'*');
				}
				order_num--;
				if(order_num<0)       order_num=order_offset;
			}
		}
		else if(rc_analog_key.key_ok == 1 /*|| KEY_OK == KEY_DOWN*/) 
		{
		   delay_ms(100);
		   if(rc_analog_key.key_ok == 1 /*|| KEY_OK == KEY_DOWN*/)
		   {   
			   LED_Fill(0x00);
			   switch(order_num)
			   {                  
				  case 0:vision_yaw_KM();break;
				  case 1:vision_pitch_KM();break;
				  default:break;
			   }
		   }
		}
        else if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)//退出
		{
			delay_ms(100);         //防止数据跳动
			if(rc_analog_key.key_back == 1 /*|| KEY_BACK == KEY_DOWN*/)
			{
				LED_Fill(0x00);
				break;
			}
		}
		vision_interface();//界面
	}
}
/**
  * @brief        GUI人机交互任务
  * @author         
  * @param[in]      
  * @retval			
  * @note  		
  */
void GUI_task(void *argument)
{
	vTaskDelay(GUI_TASK_INIT_TIME);
	oled_init();
	rc_analoy_key_init();
	
	while(1)
	{
		GUI_interaction();
        // vTaskDelay(10);
	}
}

