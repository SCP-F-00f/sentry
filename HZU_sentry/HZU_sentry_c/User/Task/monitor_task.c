/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HuangYe
 * @Teammate：
 * @Version: V3.0
 * @Date:2020.3.10
 * @Description:          监控任务确定各个模块是否在线
 * @Others:               目前的问题：oled刷屏函数会导致gui工作不正常。开机一瞬间蜂鸣器
						  回响，开机那会检测到离线（问题）。是否需要挂起任务，逻辑还
						  有待思考。
**/
#include "monitor_task.h"
#include "test_task.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "oled.h"
#include "remote_app.h"
#include "can.h"
#include "tim.h"
#include "gimbal_task.h"
#include "timers.h"

extern TaskHandle_t Gimbal_Task_Handler;
extern TaskHandle_t Shoot_Task_Handler;
extern TaskHandle_t Connect_Task_Handler;
extern TaskHandle_t INS_Task_Handler;
extern TaskHandle_t GUI_Task_Handler;
extern TaskHandle_t Test_Task_Handler;
extern TaskHandle_t Cali_Task_Handler;

/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
monitor_t monitor = 
{
	.remote.enable_flag = 1, //使能的模块才做处理
	.remote.error_msg = (uint8_t *)"remote error",
	.remote.timeout = 500,//超时时间设置
	
	.yaw_motor.enable_flag = 1,
	.yaw_motor.error_msg = (uint8_t *)"yaw_motor error",
	.yaw_motor.timeout = 500,
	
	.pitch_motor.enable_flag = 1,
	.pitch_motor.error_msg = (uint8_t *)"pitch_motor error",
	.pitch_motor.timeout = 500,
	
	.trigger_motor.enable_flag = 1,
	.trigger_motor.error_msg = (uint8_t *)"trigger_motor error",
	.trigger_motor.timeout = 4000,
	
	.fric1_motor.enable_flag = 1,
	.fric1_motor.error_msg = (uint8_t *)"fric1_motor error",
	.fric1_motor.timeout = 4000,
	
	.fric2_motor.enable_flag = 1,
	.fric2_motor.error_msg = (uint8_t *)"fric2_motor error",
	.fric2_motor.timeout = 4000,
};
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void is_lost(monitor_t *monitor, error_t *error)
{
	if(error->enable_flag == 1)
	{
		if(monitor->system_current_time > error->time + error->timeout)
		{
			if(monitor->error_cnt < 8 && error->lost_flag == 0)//大于8不再显示
			{
				error->lost_flag = 1;
//				error->error_oled_row = monitor->error_cnt;//保存当前错误所显示的oled行
				monitor->error_cnt ++;//错误数量加
			}
		}
		else
		{
			if(error->lost_flag == 1)//恢复了
			{
				error->lost_flag = 0;//丢失标志位复位
				monitor->error_cnt --;//总数减1
//				monitor->single_error_resume_flag = 1;
//				monitor->single_error_resume_row = error->error_oled_row;//保存之前的oled位置
//				error->error_oled_row = 0;//复位
//				LED_Fill(0x00);//开机瞬间检测到模块离线，运行到这里，执行清屏导致oledGUI开机会有异常
			}
		}
	}
}
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void error_list_display(monitor_t *monitor, error_t *error)
{
	static uint16_t error_row = 0;
	if(error->enable_flag == 1)
	{
		if(error->lost_flag == 1)
		{
	//		if(monitor->single_error_resume_flag)//存在一个错误恢复
	//		{
	//			if(error->error_oled_row > monitor->single_error_resume_row)//后面显示向前移判断
	//			{
	//				if(monitor->error_cnt == error->error_oled_row)//到达oled最后一个error位置
	//				{
	//					monitor->single_error_resume_flag = 0;//停止移动
	//				}
	//				error->error_oled_row -= 1;
	//			}
	//		}
			
	//		if(error->display_flag == 0)
	//		{
	//			error->error_oled_row = error_row;//保存错误的oled行
	//			error_row ++;
	//			error->display_flag = 1;
	//		}
			//测试pc_cali暂时关闭
			if(get_robot_control_mode() != GUI_CALI_MODE)
			{
				LED_P6x8Str(0, error_row, (uint8_t *)error->error_msg);
			}
			error_row ++;
			if(error_row == monitor->error_cnt)
			{
				error_row = 0;
			}
			
		}
	}
}
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void monitor_process(monitor_t *monitor)
{
	is_lost(monitor, &monitor->remote);
	is_lost(monitor, &monitor->yaw_motor);
	is_lost(monitor, &monitor->pitch_motor);
	is_lost(monitor, &monitor->trigger_motor);
	is_lost(monitor, &monitor->fric1_motor);
	is_lost(monitor, &monitor->fric2_motor);	
	
	error_list_display(monitor, &monitor->remote);
	error_list_display(monitor, &monitor->yaw_motor);
	error_list_display(monitor, &monitor->pitch_motor);
	error_list_display(monitor, &monitor->trigger_motor);
	error_list_display(monitor, &monitor->fric1_motor);
	error_list_display(monitor, &monitor->fric2_motor);
	
}
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
TimerHandle_t Timer_100MS_Task_Handler;
void Timer_100MS_Task(void);
uint8_t task_suspend_flag = 0;
void monitor_task(void *argument)
{
	
	vTaskDelay(200);
	//创建软件定时器
	Timer_100MS_Task_Handler = xTimerCreate((char *)"Timer_100MS_Task",
							           (TickType_t)300,//1s
							           (UBaseType_t)pdTRUE,//周期
							           (void *)1,//定时器编号
							           (TimerCallbackFunction_t)Timer_100MS_Task);//调用函数 						
	while(1)
	{
		monitor.system_current_time = xTaskGetTickCount();
		monitor_process(&monitor);
		
		if(monitor.error_cnt != 0)//有一个模块不在线 考虑挂起任务调度器 或者进入临界区 while(1)
		{
//			vTaskSuspendAll();
//			vTaskResume(NULL);
			monitor.exist_error_flag = 1;//存在离线模块错误标志置1，使系统的模式变为停止
			if(task_suspend_flag == 0)
			{			
				task_suspend_flag = 1;
				//LED_Fill(0x00);
				xTimerStart(Timer_100MS_Task_Handler, 0);
			}
			
		}
		else
		{
			task_suspend_flag = 0;
			monitor.exist_error_flag = 0;
			xTimerStop(Timer_100MS_Task_Handler, 0);
			BUZZER_OFF();//和调试cali提示有冲突
		}
		vTaskDelay(10);                     //10ms一次
	}	
}

void Timer_100MS_Task(void)
{
	static uint8 cnt = 0;
	cnt ++;
	
	if(cnt%4 == 0)
	{
		BUZZER_ON();
		cnt = 0;
	}
	else 
	{
		BUZZER_OFF();
	}
	
}


