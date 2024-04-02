#include "dwa.h"
#include "ULTRASONIC_task.h"
#include "chassis_task.h"
#include "math.h"

uint8_t map[240][168];
extern car_typedef car;
uint16_t dist,yy,speed_a,angle_a,xxx,a10,a11;
float cos_ang;
uint8_t j_num=0;

int16_t k1;
int16_t trace[(speed_max/speed_divide) + (angle_max/angle_divide)];
int16_t heading[(speed_max/speed_divide) + (angle_max/angle_divide)];
int16_t v_a[2][(speed_max/speed_divide) + (angle_max/angle_divide)];
int16_t value[(speed_max/speed_divide) + (angle_max/angle_divide)];

int16_t CalcObstacleEval(uint16_t speed_a, uint16_t angle_a, float t)
{
	dist = (speed_a * t)/map_divide;
	cos_ang = cos(angle_a*PI/180);
	for( xxx = 0; xxx < dist * cos(angle_a*PI/180); xxx++)
	{
		
		yy = (uint16_t)(xxx*tan(angle_a*PI/180));
		for(int i = 0; i < xxx; i++)
		{
			car.width=(90-angle_a)*car.init_width/90;
				if(j_num < yy)
				j_num++;
				for(int k = 0; k < car.width/2; k++)
					{
				a10=car.position_b[0] + (xxx - (uint16_t)(car.init_width/2) + i);
				a11=car.position_b[1] + (yy - (uint16_t)(car.init_width/2) + j_num)+k;
				
				if( map[a10][a11] == 1)
				{
					return -100;
				}
				if( map[a10][a11-2*k] == 1)
				{
					return -100;
				}
					}
			
		}
	}
	delay_us(10);
	return 100;
	
}

int16_t CalcHeadingEval(uint16_t speed_a, uint16_t angle_a)
{
	uint16_t k,head;
//	if((k = atan((car.position[1]-car.target[1])/(car.position[0]-car.target[0]))*360/(2*PI)) < 0)
//	{
//		if((angle_a - k + 180) < 0)
//		{
//			return -(angle_a - k + 180)*100/180;
//		}
//		else
//		{
//			return (angle_a - k + 180)*100/180;
//		}
//	}
//	else
//	{
//		if((angle_a - k) < 0)
//		{
//			return -(angle_a - k)*100/180;
//		}
//		else
//		{
//			return (angle_a - k)*100/180;
//		}
//	}
		k = atan(RC_abs((car.position_b[1]-car.target[1])/(car.position_b[0]-car.target[0])))*180/(PI);
	 return head = (90 - RC_abs(angle_a-k))*100/90;
		
}


int best_vaule,best_speed,best_angle;
void dwa_calculate(chassis_control_data_t *chassis)
{
	uint16_t k,j;
	for(speed_a = 0; speed_a < speed_max; speed_a += speed_divide)
	{
		for(angle_a = 0; angle_a < angle_max; angle_a += angle_divide)
		{
			//trace[k] = CalcObstacleEval(speed_a, angle_a, 0.5);
			
			heading[k1] = CalcHeadingEval(speed_a, angle_a);
			//value[k] = trace[k]+heading[k1];
			value[k] = heading[k1];
			j_num=0;
			v_a[0][k] = speed_a;
			v_a[1][k] = angle_a;
			k1++;
		}
	delay_us(10);
	}
	k1 = 0;
	best_vaule = 0;
	for(k = 0; k < (speed_max/speed_divide) + (angle_max/angle_divide); k++)
	{
		
		if(value[k] > best_vaule)
		{
			best_speed = v_a[0][k];
			best_angle = v_a[1][k];
			best_vaule = value[k];
		}
	}
	chassis->forward_back = 2000 * cos(best_angle*PI/180);//3*0.1632
	chassis->left_right = 2000 * sin(best_angle*PI/180);///100*127.32/7.8
}



