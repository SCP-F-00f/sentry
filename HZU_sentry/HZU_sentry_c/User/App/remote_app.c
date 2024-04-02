/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HongYuJia
 * @Teammate
 * @Version: V1.0
 * @Date: 2021.4.13
 * @Description:     遥控数据解析           
 * @Note:           
 * @Others: 
**/


#include "FreeRTOS.h"
#include "task.h"
#include "remote_app.h"
#include "usart.h"

rc_analog_key_t rc_analog_key;

RC_ctrl_t rc_ctrl_data = {0};
/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note    
  */
void get_remote_data(uint8_t *dbus_buff, RC_ctrl_t *rc_ctrl_data)
{
	if(dbus_buff == NULL || rc_ctrl_data == NULL)
	{
		return;
	}
	
	rc_ctrl_data->rc.ch0 = ((int16_t)dbus_buff[0] | ((int16_t)dbus_buff[1] << 8)) & 0x07FF; 
    rc_ctrl_data->rc.ch1 = (((int16_t)dbus_buff[1] >> 3) | ((int16_t)dbus_buff[2] << 5)) & 0x07FF;
    rc_ctrl_data->rc.ch2 = (((int16_t)dbus_buff[2] >> 6) | ((int16_t)dbus_buff[3] << 2) |
                           ((int16_t)dbus_buff[4] << 10)) & 0x07FF;
    rc_ctrl_data->rc.ch3 = (((int16_t)dbus_buff[4] >> 1) | ((int16_t)dbus_buff[5]<<7)) & 0x07FF;
	rc_ctrl_data->rc.ch4 = ((int16_t)dbus_buff[16]) | ((int16_t)dbus_buff[17] << 8);//左侧拨轮
    
    rc_ctrl_data->rc.s1 = ((dbus_buff[5] >> 4) & 0x000C) >> 2;
    rc_ctrl_data->rc.s2 = ((dbus_buff[5] >> 4) & 0x0003);

    rc_ctrl_data->mouse.x = ((int16_t)dbus_buff[6]) | ((int16_t)dbus_buff[7] << 8);
    rc_ctrl_data->mouse.y = ((int16_t)dbus_buff[8]) | ((int16_t)dbus_buff[9] << 8);
    rc_ctrl_data->mouse.z = ((int16_t)dbus_buff[10]) | ((int16_t)dbus_buff[11] << 8);    

    rc_ctrl_data->mouse.press_l = dbus_buff[12];
    rc_ctrl_data->mouse.press_r = dbus_buff[13];
 
    rc_ctrl_data->key.v = ((int16_t)dbus_buff[14] | (int16_t)dbus_buff[15] << 8);// | ((int16_t)dbus_buff[15] << 8);
  
	rc_ctrl_data->rc.ch0 -= RC_CHANNEL_VALUE_MIDDLE;//转化为-660~+660
	rc_ctrl_data->rc.ch1 -= RC_CHANNEL_VALUE_MIDDLE;
	rc_ctrl_data->rc.ch2 -= RC_CHANNEL_VALUE_MIDDLE;
	rc_ctrl_data->rc.ch3 -= RC_CHANNEL_VALUE_MIDDLE;
	rc_ctrl_data->rc.ch4 -= RC_CHANNEL_VALUE_MIDDLE;//左侧拨轮
	
	RC_data_error_process();
}




/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note    
  */
static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}
/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note    
  */
static void RC_reset(void)
{
	//rc
	rc_ctrl_data.rc.ch0 = 0;
	rc_ctrl_data.rc.ch1 = 0;
	rc_ctrl_data.rc.ch2 = 0;
	rc_ctrl_data.rc.ch3 = 0;
	//mouse
	rc_ctrl_data.mouse.press_l = RC_SW_DOWN;
	rc_ctrl_data.mouse.press_r = RC_SW_DOWN;
	rc_ctrl_data.mouse.x = 0;
	rc_ctrl_data.mouse.y = 0;
	rc_ctrl_data.mouse.z = 0;
	//key
	rc_ctrl_data.key.v = 0;
	
}
/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note    
  */
void RC_data_error_process(void)//防止开机或者运行中出错  三开关修改成不等于
{
	UBaseType_t uxSavedInterruptStatus; 
	
	if(RC_abs(rc_ctrl_data.rc.ch0) > RC_CHANNEL_ALLOW_ERROR_VALUE || \
	   RC_abs(rc_ctrl_data.rc.ch1) > RC_CHANNEL_ALLOW_ERROR_VALUE || \
	   RC_abs(rc_ctrl_data.rc.ch2) > RC_CHANNEL_ALLOW_ERROR_VALUE || \
	   RC_abs(rc_ctrl_data.rc.ch3) > RC_CHANNEL_ALLOW_ERROR_VALUE || \
	   rc_ctrl_data.mouse.press_l > RC_SW_MAX_VALUE               || \
	   rc_ctrl_data.mouse.press_r > RC_SW_MAX_VALUE                  \
	  )
	{
		uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();	//在中断进入临界区
		//set_gimbal_stop();                                      //云台停止 待修改
		//set_shoot_stop();                                       //发射机构停止
		RC_reset();
		//usart3_reset();
		taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
	}
}
/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note    
  */
RC_ctrl_t *get_rc_data_point(void)
{
	return &rc_ctrl_data;
}

void get_rc_analoy_key_value(rc_analog_key_t *analoy_key, RC_ctrl_t *rc_ctrl_data)
{
	if(analoy_key == NULL || rc_ctrl_data == NULL)
	{
		return;
	}
	
	if(rc_ctrl_data->rc.ch3>ANALOY_KEY_THRE_VALUE)  analoy_key->key_up = 1;//上
	else analoy_key->key_up = 0;
	
	if(rc_ctrl_data->rc.ch3<(-ANALOY_KEY_THRE_VALUE))  analoy_key->key_down = 1;//下
	else analoy_key->key_down = 0;
	
	if(rc_ctrl_data->rc.ch2>ANALOY_KEY_THRE_VALUE)  analoy_key->key_back = 1;//右
	else analoy_key->key_back = 0;
	
	if(rc_ctrl_data->rc.ch2<(-ANALOY_KEY_THRE_VALUE))  analoy_key->key_ok = 1;//左
	else analoy_key->key_ok = 0;
}

void rc_analoy_key_init(void)
{
	rc_analog_key.key_back = 0;
	rc_analog_key.key_down = 0;
	rc_analog_key.key_ok = 0;
	rc_analog_key.key_up = 0;
	rc_analog_key.rc_ctrl_data_point = get_rc_data_point();
}
