/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HongYuJia  
 * @Teammate：
 * @Version: V3.0
 * @Date:2020.4.13
 * @Description:  开始任务，用于创建所有线程，用完就休眠！
 * @Note:       
 * @Others: 
**/
#include "start_task.h"
#include "FreeRTOS.h"
#include "task.h"		
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "event_groups.h"
#include "cmsis_os2.h"
#include "cali_task.h"
#include "connect_task.h"
#include "gimbal_task.h"
#include "INS_task.h"
#include "monitor_task.h"
#include "shoot_task.h"
#include "test_task.h"


osThreadId_t INS_Task_Handler;
const osThreadAttr_t INS_Task_attr = {
  .name = "INS_Task",
  .priority = (osPriority_t) osPriorityBelowNormal5,
  .stack_size = 512 * 4
};
void INS_task(void *argument);

osThreadId_t Gimbal_Task_Handler;
const osThreadAttr_t Gimbal_Task_attr = {
  .name = "Gimbal_Task",
  .priority = (osPriority_t) osPriorityBelowNormal5,
  .stack_size = 128 * 4
};
void gimbal_task(void *argument);

osThreadId_t Shoot_Task_Handler;
const osThreadAttr_t Shoot_Task_attr = {
  .name = "Shoot_Task",
  .priority = (osPriority_t) osPriorityBelowNormal3,
  .stack_size = 256 * 4
};
void shoot_task(void *argument);

osThreadId_t Connect_Task_Handler;
const osThreadAttr_t Connect_Task_attr = {
  .name = "Connect_Task",
  .priority = (osPriority_t) osPriorityBelowNormal3,
  .stack_size = 256 * 4
};
void connect_task(void *argument);


//osThreadId_t GUI_Task_Handler;
//const osThreadAttr_t GUI_Task_attr = {
//  .name = "GUI_Task",
//  .priority = (osPriority_t) osPriorityLow7,
//  .stack_size = 256 * 4
//};
void GUI_task(void *argument);

osThreadId_t Test_Task_Handler;
const osThreadAttr_t Test_Task_attr = {
  .name = "Test_Task",
  .priority = (osPriority_t) osPriorityLow6,
  .stack_size = 256 * 4
};
void test_task(void *argument);

osThreadId_t Monitor_Task_Handler;
const osThreadAttr_t Monitor_Task_attr = {
  .name = "Monitor_Task",
  .priority = (osPriority_t) osPriorityLow4,
  .stack_size = 256 * 4
};
void monitor_task(void *argument);

osThreadId_t Cali_Task_Handler;
const osThreadAttr_t Cali_Task_attr = {
  .name = "Cali_Task",
  .priority = (osPriority_t) osPriorityLow5,
  .stack_size = 128 * 4
};
void cali_task(void *argument);

osThreadId_t adjust_Task_Handler;
const osThreadAttr_t adjust_Task_attr = {
  .name = "adjust_task",
  .priority = (osPriority_t) osPriorityBelowNormal2,
  .stack_size = 128 * 4
};
void adjust_task(void *argument);


void start_task(void *argument)
{
  /* 锁住RTOS内核防止数据解析过程中断，造成错误 */
	osKernelLock();
	//create gimbal_task	
	Gimbal_Task_Handler = osThreadNew(gimbal_task, NULL, &Gimbal_Task_attr);
	//create connect_task
	Connect_Task_Handler = osThreadNew(connect_task, NULL, &Connect_Task_attr);
    //create GUI_task				
	//GUI_Task_Handler = osThreadNew(GUI_task, NULL, &GUI_Task_attr);
	//create INS_task
	INS_Task_Handler = osThreadNew(INS_task, NULL, &INS_Task_attr);
	//create test_task		
	Test_Task_Handler = osThreadNew(test_task, NULL, &Test_Task_attr);
	//create shoot_task		
	Shoot_Task_Handler = osThreadNew(shoot_task, NULL, &Shoot_Task_attr);
	//create monitor_task		
	Monitor_Task_Handler = osThreadNew(monitor_task, NULL, &Monitor_Task_attr);
adjust_Task_Handler = osThreadNew(adjust_task, NULL, &adjust_Task_attr);
	//create cali_task		
	//Cali_Task_Handler = osThreadNew(cali_task, NULL, &Cali_Task_attr);     //串口给视觉用了
  //解锁
  osKernelUnlock();

  osThreadTerminate(osThreadGetId()); /* 结束自身 */
}
