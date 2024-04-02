/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#define CALI_DATA_PACKAGE_SIZE (40u)
extern uint8_t usart1_rx_cali_data[CALI_DATA_PACKAGE_SIZE];
extern uint8_t usart1_tx_cali_data[CALI_DATA_PACKAGE_SIZE];
#define DBUS_RX_BUF_NUM 36
extern uint8_t usart3_remote_data[DBUS_RX_BUF_NUM];
#define VISION_DATA_PACKAGE_SIZE (40u)
extern uint8_t usart6_rx_vision_data[VISION_DATA_PACKAGE_SIZE];
extern uint8_t usart6_tx_vision_data[VISION_DATA_PACKAGE_SIZE];


/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart3;

extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */

void usart_Receive_DMA(void);
void Transmit_To_vision(uint8_t enemy);
/**********USART1**********/
void usart1_send_data(uint8_t *tx_data_package);

void TX2_RX_INT(void);
/***********USART3******************/
void usart3_reset(void);


typedef __packed struct 
{ 
	uint8_t frame_seq;
	uint8_t shoot_mode;
	uint8_t shoot_speed;
	float pitch_dev;
	float yaw_dev;
	int16_t rail_speed;
	uint8_t gimbal_mode;
	
}_tx2_feedback_data;

typedef __packed struct 
{

  volatile uint8_t recog_flag;
	volatile uint8_t shoot_flag;
	uint8_t yaw_sign;
	uint8_t view_control_flag;
	uint16_t yaw_raw;
	float 	 yaw_dev;
	uint8_t  pitch_sign;
	uint16_t pitch_raw;
	float 	 pitch_dev;
	float yaw_old_dev;
	float pitch_old_dev;
	uint16_t Target_distance;
	float forward_speed;
	float zuoyou_speed;
	float rotate_speed;
	char died;
}_tx2_control_data;


typedef __packed struct 
{
	uint8_t robot_color;
	uint8_t robot_mode;
	uint8_t robot_id;
	int16_t 	gyro_x;
	int16_t   gyro_y;
	float 	yaw;
	float 	pitch;
	short   gyro_accle;
	uint8_t bullet_speed;
}_tx2_tx_data;

uint8_t TX2_MES(_tx2_control_data *control_data);

typedef struct
{
	
	uint8_t check_top_byte;
	uint8_t check_bottom_byte;
	uint8_t pid_type_data;

	
	uint8_t receive_success_flag; 
	uint8_t beep_flag;            
	
} vision_t;


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

