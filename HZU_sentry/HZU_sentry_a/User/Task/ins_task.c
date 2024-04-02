#include "ins_task.h"
#include "FreeRTOS.h"
#include "task.h"

extern imu_t imu;
float yaw_init,yaw;
void INS_task(void *argument)
{
	mpu_device_init();
	init_quaternion();
	for(int i = 0; i < 400; i++)
	{
		mpu_get_data();
		imu_ahrs_update();
		imu_attitude_update(); 
		yaw_init = imu.yaw;
		HAL_Delay(10);
	}
	for( ; ; )
	{
		yaw = imu.yaw - yaw_init;
		mpu_get_data();
		imu_ahrs_update();
		imu_attitude_update(); 
		vTaskDelay(10);
	}
}

