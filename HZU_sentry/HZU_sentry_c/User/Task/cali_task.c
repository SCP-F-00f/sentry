///**
// * @Copyright(C),
// * @FileName:.c
// * @Author: HongYuJia  
// * @Teammate：
// * @Version: V3.0
// * @Date:2020.4.13
// * @Description: 机器人各个参数的校正，配合蓝牙串口进行数据调参
// * 				（目前C板4pin的串口被小电脑占用，未启动该任务
// * 				 后期可以将小电脑的交互放在3pin上，利用好资源）
// * @Note:       
// * @Others: 
//**/
//#include "test_task.h"
//#include "delay.h"
//#include "FreeRTOS.h"
//#include "task.h"
//#include "oled.h"
//#include "tim.h"
//#include "gpio.h"
//#include "remote_app.h"
//#include "bmi088driver.h"
//#include "INS_task.h"
//#include "gimbal_task.h"
//#include "stdio.h"
//#include "string.h"
//#include "shoot_task.h"
//#include "can.h"
//#include "monitor_task.h"
//#include "cali_task.h"
//#include "GUI_task.h"
//#include <string.h>
//#include "usart.h"
//#include "connect_task.h"
//#include "cmsis_os2.h"


//pc_cali_tx_t pc_cali_tx;

//pc_cali_t pc_cali;

//cali_pid_t cali_other_pid;
//extern cali_chassis_t cali_chassis_pid;
//extern gimbal_control_data_t gimbal_control_data;


//uint8_t buff[8] = {30,30,30,40,40,0,0,0};
//uint8_t buf[1] = {0x02};
//void send_state_data_package(pc_cali_tx_t *pc_cali_tx)
//{
//	pc_cali_tx->system_control_mode = get_robot_control_mode();
//	pc_cali_tx->system_work_mode = get_robot_work_mode();
//	pc_cali_tx->gimbal_work_mode = get_gimbal_work_mode();
//	pc_cali_tx->shoot_work_mode = get_shoot_mode();
//	pc_cali_tx->fric_work_mode = get_fric_mode();
//	pc_cali_tx->chassis_work_mode = 0;
//	
//	pc_cali_tx->fric1_speed = pc_cali_tx->shoot_fric1_msg->encoder.filter_rate;
//	pc_cali_tx->fric2_speed = pc_cali_tx->shoot_fric2_msg->encoder.filter_rate;
//	
//	usart1_tx_cali_data[0] = get_robot_control_mode();
//	usart1_tx_cali_data[1] = get_robot_work_mode();
//	usart1_tx_cali_data[2] = get_gimbal_work_mode();
//	usart1_tx_cali_data[3] = get_shoot_mode();
//	usart1_tx_cali_data[4] = get_fric_mode();
//	usart1_tx_cali_data[5] = pc_cali_tx->fric1_speed >> 8;
//	usart1_tx_cali_data[6] = pc_cali_tx->fric1_speed;
//	usart1_tx_cali_data[7] = pc_cali_tx->fric2_speed >> 8;
//	usart1_tx_cali_data[8] = pc_cali_tx->fric2_speed;
//	
//	usart1_tx_cali_data[9] = pc_cali_tx->connect->cm1_rate >> 8;
//	usart1_tx_cali_data[10] = pc_cali_tx->connect->cm1_rate;
//	usart1_tx_cali_data[11] = pc_cali_tx->connect->cm2_rate >> 8;
//	usart1_tx_cali_data[12] = pc_cali_tx->connect->cm2_rate;
//	usart1_tx_cali_data[13] = pc_cali_tx->connect->cm3_rate >> 8;
//	usart1_tx_cali_data[14] = pc_cali_tx->connect->cm3_rate;
//	usart1_tx_cali_data[15] = pc_cali_tx->connect->cm4_rate >> 8;
//	usart1_tx_cali_data[16] = pc_cali_tx->connect->cm4_rate;
//	
//	usart1_send_data(usart1_tx_cali_data);
//}
///**
//  * @brief          
//  * @author         
//  * @param[in] 
//  * @retval	
//  * @note       
//  */
//void get_usart_data_to_array(uint8_t *array, uint8_t *usart1)
//{
//	array[0] = *(usart1 + 3);
//	array[1] = *(usart1 + 2);
//	array[2] = *(usart1 + 1);
//	array[3] = *(usart1);
//}
///**
//  * @brief          
//  * @author         
//  * @param[in] 
//  * @retval	
//  * @note       获取pc端发送的调式数据，中断处理防止数据覆盖
//  */
//void get_pc_cali_data(pc_cali_t *pc_cali, uint8_t *usart1_data)
//{
//	if(pc_cali == NULL || usart1_data == NULL)
//	{
//		return;
//	}
//	pc_cali->check_top_byte = usart1_data[0]; //获取帧头尾效验位
//	pc_cali->check_bottom_byte = usart1_data[29];
//	
//	pc_cali->pid_type_data = usart1_data[1];

//	if(pc_cali->check_top_byte == CALI_CHECK && pc_cali->check_bottom_byte == CALI_CHECK)
//	{		
//		if(pc_cali->pid_type_data != CALI_PARAM_FEATURE)//pid
//		{
//			get_usart_data_to_array(&pc_cali->cali_p_array[0], &usart1_data[2]);
//			get_usart_data_to_array(&pc_cali->cali_i_array[0], &usart1_data[6]);
//			get_usart_data_to_array(&pc_cali->cali_d_array[0], &usart1_data[10]);			
//		}
//		else //自定义参数
//		{  
//			get_usart_data_to_array(&pc_cali->cali_param1_array[0], &usart1_data[2]);
//			get_usart_data_to_array(&pc_cali->cali_param2_array[0], &usart1_data[6]);
//			get_usart_data_to_array(&pc_cali->cali_param3_array[0], &usart1_data[10]);
//			get_usart_data_to_array(&pc_cali->cali_param4_array[0], &usart1_data[14]);
//			get_usart_data_to_array(&pc_cali->cali_param5_array[0], &usart1_data[18]);
//			get_usart_data_to_array(&pc_cali->cali_param6_array[0], &usart1_data[22]);						
//		}
//		pc_cali->receive_success_flag = 1;
//	}
//	else
//	{
//		//memset(usart1_data, 0, CALI_DATA_PACKAGE_SIZE * sizeof(uint8_t));//启用会发生数据错误，待
//	}
//}
///**
//  * @brief          
//  * @author         
//  * @param[in] 
//  * @retval	
//  * @note    
//  */
//void get_cali_pid(pc_cali_t *pc_cali, cali_pid_t *cali_pid)
//{
//	cali_pid->kp = pc_cali->p;
//	cali_pid->ki = pc_cali->i;
//	cali_pid->kd = pc_cali->d;
//}
///**
//  * @brief          
//  * @author         
//  * @param[in] 
//  * @retval	
//  * @note    
//  */
//void various_types_pid_update(pc_cali_t *pc_cali)
//{
//	switch(pc_cali->pid_type_data)
//	{
//		case YAW_POSITION_PID: //yaw
//			get_cali_pid(pc_cali, &cali_gimbal_pid.yaw_pid.position);
//		break;
//		case YAW_SPEED_PID:
//			get_cali_pid(pc_cali, &cali_gimbal_pid.yaw_pid.speed);
//		break;
//		case PITCH_POSITION_PID: //pitch
//			get_cali_pid(pc_cali, &cali_gimbal_pid.pitch_pid.position);
//		break;
//		case PITCH_SPEED_PID:
//			get_cali_pid(pc_cali, &cali_gimbal_pid.pitch_pid.speed);
//		break;
//		case TRIGGER_POSITION_PID: //trigger
//			get_cali_pid(pc_cali, &cali_shoot_pid.trigger_pid.position);
//		break;
//		case TRIGGER_SPEED_PID:
//			get_cali_pid(pc_cali, &cali_shoot_pid.trigger_pid.speed);
//		break;
//		case FRIC1_PID: //fric
//			get_cali_pid(pc_cali, &cali_shoot_pid.fric1_pid);
//		break;
//		case FRIC2_PID:
//			get_cali_pid(pc_cali, &cali_shoot_pid.fric2_pid);
//		break;
//		case CM_PID: //cm四个电机
//			get_cali_pid(pc_cali, &cali_chassis_pid.cm_pid);
//			set_chassis_CM_pid_data_to_chassis();
//		break;
//		case ROTATE_PID://滚动方向
//			get_cali_pid(pc_cali, &cali_chassis_pid.rotate_pid);
//			set_chassis_rotate_pid_data_to_chassis();
//		break;
//		case OTHER_1_PID: //other
//			get_cali_pid(pc_cali, &cali_other_pid);
//		break;
//		case OTHER_2_PID:
//			get_cali_pid(pc_cali, &cali_other_pid);
//		break;
//		default :
//		break;
//	}
//}
///**
//  * @brief         4字节unsigned char 转换为float
//  * @author         
//  * @param[in] 
//  * @retval	
//  * @note    
//  */
//void type_conversion(pc_cali_t *pc_cali)
//{ 
//	//pid
//	pc_cali->p = *((float *)pc_cali->cali_p_array);
//	pc_cali->i = *((float *)pc_cali->cali_i_array);
//	pc_cali->d = *((float *)pc_cali->cali_d_array);
//	//other param
//	pc_cali->param1 = *((float *)pc_cali->cali_param1_array);
//	pc_cali->param2 = *((float *)pc_cali->cali_param2_array);
//	pc_cali->param3 = *((float *)pc_cali->cali_param3_array);
//	pc_cali->param4 = *((float *)pc_cali->cali_param4_array);
//	pc_cali->param5 = *((float *)pc_cali->cali_param5_array);
//	pc_cali->param6 = *((float *)pc_cali->cali_param6_array);
//}
///**
//  * @brief          
//  * @author         
//  * @param[in] 
//  * @retval	
//  * @note    
//  */
//void cali_init(pc_cali_tx_t *pc_cali_tx)
//{
//	pc_cali_tx->shoot_fric1_msg = get_fric_motor1_msg_point();
//	pc_cali_tx->shoot_fric2_msg = get_fric_motor2_msg_point();
//	
//	pc_cali_tx->connect = get_connect_data_point();
//}
///**
//  * @brief          
//  * @author         
//  * @param[in] 
//  * @retval	
//  * @note    
//  */
//extern gimbal_pid_t gimbal_pid;
//extern shoot_pid_t shoot_pid;
//void get_cali_pid_data(pid_t *pid, cali_pid_t *cali_pid)
//{
//	pid->kp = cali_pid->kp;
//	pid->ki = cali_pid->ki;
//	pid->kd = cali_pid->kd;
//	pid->mode = cali_pid->mode;
//}
//void cali_various_motor_pid(gimbal_pid_t *gimbal_pid, 
//							cali_gimbal_t *cali_gimbal_pid, 
//							shoot_pid_t *shoot_pid, 
//							cali_shoot_t *cali_shoot_pid)
//{
//	//yaw cascade pid
//	get_cali_pid_data(&gimbal_pid->yaw_pid.position_pid, &cali_gimbal_pid->yaw_pid.position);
//	get_cali_pid_data(&gimbal_pid->yaw_pid.speed_pid, &cali_gimbal_pid->yaw_pid.speed);
//	//pitch cascade pid
//	get_cali_pid_data(&gimbal_pid->pitch_pid.position_pid, &cali_gimbal_pid->pitch_pid.position);
//	get_cali_pid_data(&gimbal_pid->pitch_pid.speed_pid, &cali_gimbal_pid->pitch_pid.speed);
//	
//	//trigger 
//	get_cali_pid_data(&shoot_pid->trigger_pid.position_pid, &cali_shoot_pid->trigger_pid.position);
//	get_cali_pid_data(&shoot_pid->trigger_pid.speed_pid, &cali_shoot_pid->trigger_pid.speed);
//	//fric
//	get_cali_pid_data(&shoot_pid->fric_1_pid, &cali_shoot_pid->fric1_pid);
//	get_cali_pid_data(&shoot_pid->fric_2_pid, &cali_shoot_pid->fric2_pid);
//}
///**
//  * @brief         蜂鸣器提示函数 
//  * @author         
//  * @param[in] 
//  * @retval	
//  * @note    
//  */
//uint8_t beep_process(void)
//{
//	static TickType_t current_time = 0;
//	static TickType_t start_time = 0;
//	static uint8_t start_flag = 0;
//	
//	if(start_flag == 0)
//	{
//		start_time = xTaskGetTickCount();
//		start_flag = 1;
//	}
//	
//	current_time = xTaskGetTickCount();
//	
//	if((current_time > start_time + 100 && current_time < start_time + 200) || \
//	   (current_time > start_time + 300 && current_time < start_time + 400))
//	{
//		buzzer_on(50);
//	}
//	else
//	{
//		buzzer_off();
//	}
//	
//	if(current_time > start_time + 400)
//	{
//		start_flag = 0;
//		buzzer_off();
//		return 1;
//	}
//	return 0;
//}
//void vcan_sendware(void *wareaddr, uint32_t waresize)
//{
//	// vcan_sendware((uint8_t *)Virtual_var, sizeof(Virtual_var));
//    #define CMD_WARE     3
//    uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};    //串口调试 使用的前命令
//    uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE};    //串口调试 使用的后命令
//	usart1_send_data(cmdf);
//	usart1_send_data(wareaddr);
//	usart1_send_data(cmdr);
//    // uart_putbuff(VCAN_PORT, cmdf, sizeof(cmdf));    //先发送前命令
//    // uart_putbuff(VCAN_PORT, (uint8_t *)wareaddr, waresize);    //发送数据
//    // uart_putbuff(VCAN_PORT, cmdr, sizeof(cmdr));    //发送后命令
//}
//uint8_t Virtual_var[6];

///**
//  * @brief          
//  * @author         
//  * @param[in] 
//  * @retval	
//  * @note    
//  */
//void cali_task(void *argument)
//{
//	cali_init(&pc_cali_tx);
//    while(1)
//    {
//		// get_pc_cali_data(&pc_cali, usart1_cali_data);
//	
//		if(pc_cali.receive_success_flag == 1)      //与GUI菜单调试配合
//		{
//			type_conversion(&pc_cali);             //类型转换转为float
//			various_types_pid_update(&pc_cali);    //pid数据更新 接收一次更新一次
//			pc_cali.receive_success_flag = 0;
//			pc_cali.beep_flag = 1;

//		}
//		if(pc_cali.beep_flag == 1)
//		{
//			if(beep_process() == 1)
//			{
//				pc_cali.beep_flag = 0;
//			}
//		}
//		//taskENTER_CRITICAL(); //进入临界区发送防止数据被打断
//		send_state_data_package(&pc_cali_tx);//发送车子状态数据包
//		//taskEXIT_CRITICAL();            	
//		cali_various_motor_pid(&gimbal_pid, &cali_gimbal_pid,//更新pid
//							   &shoot_pid, &cali_shoot_pid);
//		// Virtual_var[0] = gimbal_control_data.gimbal_INS->gyro_x;
//		// vcan_sendware((uint8_t *)Virtual_var, sizeof(Virtual_var));

//	
//		vTaskDelay(100);          //100ms一次
//		
//    }
//}
