#include "master_computer_task.h"

extern IMU_t imu_data;

uint8_t buffer[20] = "OK\r\n";

// __weakÈõº¯Êý
extern "C"
void master_computer_task(void const * argument)
{
	while(true)
	{
		printf("Ax=%.2f,Ay=%.2f,Az=%.2f\r\n",imu_data.Accel.X,imu_data.Accel.Y,imu_data.Accel.Z);
	  printf("Gx=%.2f,Gy=%.2f,Gz=%.2f\r\n",imu_data.Gyro.X,imu_data.Gyro.Y,imu_data.Gyro.Z);
	  printf ("Temperature=%.2f,\r\n",imu_data.Temp);
		osDelay(500);
		
	}
}
