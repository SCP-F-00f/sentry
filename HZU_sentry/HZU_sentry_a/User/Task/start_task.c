/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HongYuJia 
 * @Teammate��
 * @Version: V3.0
 * @Date:2021.4.13
 * @Description:   ��ʼ���������߳��������������������
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
#include "chassis_task.h"
#include "connect_task.h"
#include "ULTRASONIC_task.h"

//��Ҫ��ջ��С���㷽������Ҫ��ջֵ (stack_size*4) = ��ǰ����(stack_size *4) - ջ����ʷʣ��ֵ*4 
osThreadId_t chassis_task_Handler;
const osThreadAttr_t chassis_task_attr = {
  .name = "chassis_task",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4 
};
void chassis_task(void *argument);

osThreadId_t connect_task_Handler;
const osThreadAttr_t connect_task_attr = {
  .name = "connect_task",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4 
};
void connect_task(void *argument);

osThreadId_t ULTRASONIC_task_Handler;
const osThreadAttr_t ULTRASONIC_task_attr = {
  .name = "ULTRASONIC_task",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 512 * 8
};
void ULTRASONIC_task(void *argument);

osThreadId_t test_task_Handler;
const osThreadAttr_t test_task_attr = {
  .name = "test_task",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4 
};
void test_task(void *argument);

osThreadId_t INS_task_Handler;
const osThreadAttr_t INS_task_attr = {
  .name = "INS_task",
  .priority = (osPriority_t) osPriorityHigh1,
  .stack_size = 128 * 4 
};
void INS_task(void *argument);

void start_task(void *argument)
{
  /* ��סRTOS�ں˷�ֹ���ݽ��������жϣ���ɴ��� */
  osKernelLock();
	//create gimbal_task	
  chassis_task_Handler = osThreadNew(chassis_task, NULL, &chassis_task_attr);
	//create connect_task
	connect_task_Handler = osThreadNew(connect_task, NULL, &connect_task_attr);
  //create GUI_task				
	ULTRASONIC_task_Handler = osThreadNew(ULTRASONIC_task, NULL, &ULTRASONIC_task_attr);
	//create test_task				
	test_task_Handler = osThreadNew(test_task, NULL, &test_task_attr);
	//creat ins_task
	INS_task_Handler = osThreadNew(INS_task, NULL, &INS_task_attr);
  //����
  osKernelUnlock();

  osThreadTerminate(osThreadGetId()); /* �������� */
}