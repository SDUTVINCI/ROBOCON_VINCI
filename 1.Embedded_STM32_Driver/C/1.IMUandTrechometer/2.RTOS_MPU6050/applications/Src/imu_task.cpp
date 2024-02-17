#include "imu_task.h"

// __weak������
extern "C"
void imu_task(void const * argument)
{
	//��ѯ��ַ��������豸�Ƿ�����
	I2C_Search_Addr();
	
	//��ʼ��MPU6050
  MPU6050_Init();
	osDelay(IMU_INIT_WAITING_TIME);
	while(true)
	{
		MPU6050_Read_All();
		osDelay(100);
	}
}

