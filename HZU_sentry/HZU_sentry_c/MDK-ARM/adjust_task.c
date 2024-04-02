#include "adjust_task.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gimbal_task.h"
#include "usart.h"

char Usart_Receive[10]={0},Usart_Receive1[10]={0}; 
extern gimbal_pid_t gimbal_pid;
unsigned char Usart_Cnt=0;
//unsigned char  *Usart_Receive;
float result;
uint8_t j=0,i=1;
uint16_t	scale=1;

void Clear_Usart_Receive(void)
{
		unsigned char c;
		for(c=0;c<10;c++)
		{
			 Usart_Receive1[c]=Usart_Receive[c];
				Usart_Receive[c]=0;
		}
			Usart_Cnt=0;
}

void adjust_task(void *argument)
{
			vTaskDelay(1);
	while(1)
	{
		vTaskDelay(1);
					 if(Usart_Receive[Usart_Cnt-2]=='\r'&&Usart_Receive[Usart_Cnt-1]=='\n')
{
						//delay_ms(100);
						if(Usart_Receive[1]=='-')
							i++;
						while(Usart_Receive[i]>='0'&&Usart_Receive[i]<='9')
						{
						result=result*10+Usart_Receive[i]-48;
						i++;
						}
						
						if(Usart_Receive[i]=='.')
						{
							scale=1;
							i++;
						while(Usart_Receive[i]>='0'&&Usart_Receive[i]<='9')
						{
							result=result*10+Usart_Receive[i]-48;
							scale*=10;
							i++;
						}			
						}
						if(Usart_Receive[1]=='-')
						result=-result;
						result/=scale;
						scale=1;
						switch(Usart_Receive[0])
						{
							case 'a':gimbal_pid.pitch_pid.position_pid.kp=result;  break;
							case 'b':gimbal_pid.pitch_pid.position_pid.ki=result;  break;
							case 'c':gimbal_pid.pitch_pid.position_pid.kd=result;  break;
							case 'd':gimbal_pid.pitch_pid.speed_pid.kp=result;     break;
							case 'e':gimbal_pid.pitch_pid.speed_pid.ki=result;     break;
							case 'f':gimbal_pid.pitch_pid.speed_pid.kd=result;     break;
							case 'g':gimbal_pid.yaw_pid.position_pid.kp=result;  break;
							case 'h':gimbal_pid.yaw_pid.position_pid.ki=result;  break;
							case 'i':gimbal_pid.yaw_pid.position_pid.kd=result;  break;
							case 'j':gimbal_pid.yaw_pid.speed_pid.kp=result;     break;
							case 'k':gimbal_pid.yaw_pid.speed_pid.ki=result;     break;
							case 'l':gimbal_pid.yaw_pid.speed_pid.kd=result;     break;

							

							default:break;
						}

						result=0;
						i=1;					
						Clear_Usart_Receive();
					
}
	}
	
}