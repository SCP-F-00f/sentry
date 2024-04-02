/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HongYuJia 
 * @Teammate：
 * @Version: V3.0
 * @Date:2021.4.13
 * @Description:   解析遥控数据
 * @Note:       
 * @Others: 
**/
//#include "test_task.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "remote_app.h"
#include "can1_app.h"
#include "can2_app.h"

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
	if(dbus_buff == NULL || rc_ctrl_data == NULL)//野指针强制退出
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
 
    rc_ctrl_data->key.v = ((int16_t)dbus_buff[14] | (int16_t)dbus_buff[14] << 8);// | ((int16_t)dbus_buff[15] << 8);
  
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
//int16_t RC_abs(int16_t value)
//{
//    if (value > 0)
//    {
//        return value;
//    }
//    else
//    {
//        return -value;
//    }
//}
/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note    
  */
static void RC_reset(void)
{
	rc_ctrl_data.rc.ch0 = 0;
	rc_ctrl_data.rc.ch1 = 0;
	rc_ctrl_data.rc.ch2 = 0;
	rc_ctrl_data.rc.ch3 = 0;
	
	rc_ctrl_data.mouse.press_l = RC_SW_DOWN;
	rc_ctrl_data.mouse.press_r = RC_SW_DOWN;
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
		uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();	//进入临界区
		set_chassis_stop();
		RC_reset();
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

