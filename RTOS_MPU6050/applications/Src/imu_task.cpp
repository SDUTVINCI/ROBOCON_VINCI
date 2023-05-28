#include "imu_task.h"

// __weak弱函数
extern "C"
void imu_task(void const * argument)
{
	//查询地址，并检测设备是否正常
	I2C_Search_Addr();
	
	//初始化MPU6050
  MPU6050_Init();
	osDelay(IMU_INIT_WAITING_TIME);
	while(true)
	{
		MPU6050_Read_All();
		osDelay(100);
	}
}

