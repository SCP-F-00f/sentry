/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HongYuJia  
 * @Teammate：
 * @Version: V3.0
 * @Date:2020.4.13
 * @Description:   测试任务，用于查看并检测系统运行状态
 * @Note:       
 * @Others: 
**/
#include "test_task.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "event_groups.h"
#include "oled.h"
#include "gpio.h"
#include "remote_app.h"
#include "bmi088driver.h"
#include "INS_task.h"
#include "gimbal_task.h"
#include "stdio.h"
#include "string.h"
#include "shoot_task.h"
#include "can.h"
#include "tim.h"
#include "monitor_task.h"
#include "connect_task.h"
#include "flash.h"
#include "gpio.h"
#include <stdlib.h>
#include <stdio.h>

extern fp32 INS_angle[3];
extern fp32 INS_gyro[3];
extern INS_t INS;
float yaw_angle_max = 0.0;
float yaw_angle_speed_max = 0.0;
extern uint8_t start_time_flag;
extern int32_t start_time_us;
extern int32_t current_time_us;
extern int16_t pitch_motor_can_set_current;

extern robot_work_mode_e robot_work_mode;
extern robot_control_mode_e robot_control_mode;
extern gimbal_control_data_t gimbal_control_data;
extern gimbal_work_mode_e gimbal_work_mode;
extern shoot_control_data_t shoot_control_data;
uint8_t can2_buff[8] = {10, 11, 10, 11, 10, 11, 10, 11,};
extern uint8_t can2_rx_data[CAN_RX_BUF_SIZE];


#define MAX_PSC             1000

#define MAX_BUZZER_PWM      20000
#define MIN_BUZZER_PWM      10000

uint16_t psc = 0;
uint16_t pwm = MIN_BUZZER_PWM;
char ch[400] = {0};
extern gimbal_pid_t gimbal_pid;
volatile uint8_t flash_flag = 0;
volatile uint32_t flash_data = 0;


const u8 TEXT_Buffer[]={"STM32 FLASH TEST"};
#define TEXT_LENTH sizeof(TEXT_Buffer)	 		  	//数组长度	
#define SIZE TEXT_LENTH/4+((TEXT_LENTH%4)?1:0)

#define FLASH_SAVE_ADDR  0x080C0000 	//设置FLASH 保存地址(必须为4的倍数，且所在扇区,要大于本代码所占用到的扇区.

u8 datatemp[SIZE];
extern TaskHandle_t Gimbal_Task_Handler;
extern TaskHandle_t GUI_Task_Handler;
extern RC_ctrl_t rc_ctrl_data;

extern QueueHandle_t test_queue_handler;

extern shoot_control_data_t shoot_control_data;
extern shoot_pid_t shoot_pid;
extern float shoot_abs(float value);

extern RC_ctrl_t rc_ctrl_data;

void test_task(void *argument)
{

    while(1)
    {	
		
//		LED_P6x8Str(0, 0, (uint8_t *)"p_set");
//		LED_PrintValueI(40, 0, shoot_control_data.trigger_position_angle_set);

/**
* @brief 各任务运行时间输出
* @note    
*/
#if 1 //0   *hyj
		memset(ch,0,400);				//信息缓冲区清零
		vTaskGetRunTimeStats(ch);		//获取任务运行时间信息
		// printf("#任务名\t\t\t运行时间\t运行所占百分比\r\n");   //发数据给串口   *hyj
		// printf("%s\r\n", ch);
		
/**
* @brief 各任务运行状态输出
* @note    
*/	
#elif 0 		
		memset(ch,0,400);				//信息缓冲区清零
		vTaskList(ch);
		printf("task_name\ttask_state\tpriority\tstack\ttasK_num\r\n");
		printf("%s\r\n", ch);
/**
* @brief vTaskGetInfo()
* @note    
*/
#elif 0
		TaskHandle_t TaskHandle;	
		TaskStatus_t TaskStatus;

		TaskHandle=xTaskGetHandle("GUI_task");		//根据任务名获取任务句柄。

		vTaskGetInfo((TaskHandle_t	)TaskHandle, 		//任务句柄
					 (TaskStatus_t*	)&TaskStatus, 		//任务信息结构体
					 (BaseType_t	)pdTRUE,			//允许统计任务堆栈历史最小剩余大小
					 (eTaskState	)eInvalid);			//函数自己获取任务运行状态

		printf("任务名:                %s\r\n",TaskStatus.pcTaskName);
		printf("任务编号:              %d\r\n",(int)TaskStatus.xTaskNumber);
		printf("任务状态:              %d\r\n",TaskStatus.eCurrentState);
		printf("任务当前优先级:        %d\r\n",(int)TaskStatus.uxCurrentPriority);
		printf("任务基优先级:          %d\r\n",(int)TaskStatus.uxBasePriority);
		printf("运行时间计数:          %d\r\n",(int)TaskStatus.ulRunTimeCounter);
		printf("任务堆栈基地址:        %#x\r\n",(int)TaskStatus.pxStackBase);
		printf("任务堆栈历史剩余最小值:%d\r\n",TaskStatus.usStackHighWaterMark);	
	    printf("/**************************结束***************************/\r\n");	
#endif		
		vTaskDelay(100);
		
    }
}
