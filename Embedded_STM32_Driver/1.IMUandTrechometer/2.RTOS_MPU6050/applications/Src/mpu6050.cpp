#include "mpu6050.h"

uint8_t I2C_Device_Addr;   //���ҵ���д��MPU6050_ADDR����

IMU_t imu_data;

//IMU_Kalman_t imu_kalman_data;  //�������˲���д


/**
 * @brief  I2C�豸��ַ��ѯ����
 * @param  void
 * @retval void
* @attention �ҵ�һ���豸��ַ�Ϳ���ע�͵���Ȼ����MPU6050_ADDR����д��
 */
void I2C_Search_Addr(void)
{
  for(I2C_Device_Addr = 0;I2C_Device_Addr < 255;I2C_Device_Addr++)
  {
		//Ѱ���豸��ַ������ӡ��ַ��Ȼ�����豸�Ƿ����
	  if(HAL_I2C_IsDeviceReady (&hi2c1 ,I2C_Device_Addr ,1 ,1000) == HAL_OK )
	  {
//			printf("HAL_OK\r\n");
//		  printf("%d\r\n",Addr_Inspect);
		  break;
	  }
  }
}


/**
 * @brief  MPU6050��ʼ������  �Ը����Ĵ�����������
 * @param  void
 * @retval void
* @attention 
 */
void MPU6050_Init(void)
{
	uint8_t check,Data;
	
	// ����豸ID WHO_AM_I
	//Ĭ��ֵ��0x68(104)��ֻ�õ�bit6-bit1 ��x110 100x��Ҳ���� 0110 1000
	HAL_I2C_Mem_Read (&hi2c1 ,MPU6050_ADDR,WHO_AM_I_REG,1,&check ,1,1000);
	
	if(check == 0x68)  //����豸����
	{
		//�ѵ�Դ����1�Ĵ�����0���򲻻�ʧ�ܶ�Ӧ��������ʱ��Դѡ���ڲ�8MHz����
		Data = 0x00; //0000 0000
		HAL_I2C_Mem_Write (&hi2c1 ,MPU6050_ADDR ,PWR_MGMT_1_REG ,1,&Data ,1,1000);
		
		//SMPLRT_DIV�Ĵ���,�������ݲ�����Ϊ1KHz;
		// ����Ƶ��=���������Ƶ��/��1+SMPLRT_DIV��
		Data = 0x07;   //0000 0111
		HAL_I2C_Mem_Write (&hi2c1 ,MPU6050_ADDR ,SMPLRT_DIV_REG ,1 ,&Data,1,1000);
		
		// ����ACCEL_CONFIG�Ĵ���
		// ������X,Y,Z,���Լ죬���ټƵ������̷�Χѡ���2g
		Data = 0x00;   //0000 0000
		HAL_I2C_Mem_Write (&hi2c1 ,MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);
		
		// ����GYRO_CONFIG�Ĵ���
		//  ������X,Y,Z,���Լ죬�����ǵ������̷�Χѡ��� 250deg/s
		Data = 0x00;  //0000 0000
		HAL_I2C_Mem_Write (&hi2c1 ,MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}
}




/**
 * @brief  MPU6050��ȡ���ٶȺ���
 * @param  void
 * @retval void
* @attention 
 */
void MPU6050_Read_Accel(void)
{
	uint8_t Rec_Data[6];
	
	int16_t Accel_X_Buffer = 0;
	int16_t Accel_Y_Buffer = 0;
	int16_t Accel_Z_Buffer = 0;
	//��ȡ6�ֽڵ����ݴ�ACCEL_XOUT_H�Ĵ�����
	
	HAL_I2C_Mem_Read (&hi2c1 ,MPU6050_ADDR ,ACCEL_XOUT_H_REG ,1,Rec_Data ,6,1000);
	//�Ѹ�8λ�͵�8λ�ϲ�Ϊ16λ����
	Accel_X_Buffer = (int16_t)(Rec_Data [0] <<8 | Rec_Data [1]);
	Accel_Y_Buffer = (int16_t)(Rec_Data [2] <<8 | Rec_Data [3]);
	Accel_Z_Buffer = (int16_t)(Rec_Data [4] <<8 | Rec_Data [5]);
	
	//��Ϊ���ټƵ������̷�Χѡ���2g,���Լ��ٶȼƲ���ֵ����������ͷֱ���Ҫѡ16384LSB/g
	imu_data.Accel.X = Accel_X_Buffer/16384.0f;
	imu_data.Accel.Y = Accel_Y_Buffer/16384.0f;
	imu_data.Accel.Z = Accel_Z_Buffer/16384.0f;
}

/**
 * @brief  MPU6050��ȡ���ٶȺ���
 * @param  void
 * @retval void
* @attention 
 */
void MPU6050_Read_Gyro(void )
{
	uint8_t Rec_Data[6];
	
	int16_t Gyro_X_Buffer = 0;
	int16_t Gyro_Y_Buffer = 0;
	int16_t Gyro_Z_Buffer = 0;
	//��ȡ6�ֽڵ����ݴ�GYRO_XOUT_H�Ĵ�����
	
	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR ,GYRO_XOUT_H_REG ,1,Rec_Data ,6 ,1000);
	//�Ѹ�8λ�͵�8λ�ϲ�Ϊ16λ����
	Gyro_X_Buffer = (int16_t )(Rec_Data [0] << 8 | Rec_Data [1]);
	Gyro_Y_Buffer = (int16_t )(Rec_Data [2] << 8 | Rec_Data [3]);
	Gyro_Z_Buffer = (int16_t )(Rec_Data [4] << 8 | Rec_Data [5]);
	//��Ϊ�����ǵ������̷�Χѡ��� 250deg/s,���������ǲ���ֵ����������ͷֱ���Ҫѡ131LSB/��/s
	imu_data.Gyro.X = Gyro_X_Buffer/131.0f;
	imu_data.Gyro.Y = Gyro_Y_Buffer/131.0f;
	imu_data.Gyro.Z = Gyro_Z_Buffer/131.0f;
}

/**
 * @brief  MPU6050��ȡ�¶Ⱥ���
 * @param  void
 * @retval void
* @attention 
 */
void MPU6050_Read_Temp(void)
{
	uint8_t Rec_Data[2];
	int16_t Temp_Buffer = 0;
	//��ȡ2�ֽڵ����ݴ�TEMP_OUT_H_REG�Ĵ�����
	HAL_I2C_Mem_Read (&hi2c1 ,MPU6050_ADDR ,TEMP_OUT_H_REG ,1 ,Rec_Data  ,2 ,1000);
	
	//�Ѹ�8λ�͵�8λ�ϲ�Ϊ16λ����
	Temp_Buffer = (int16_t )(Rec_Data [0]<<8 | Rec_Data [1]);
	
	//˵������Ĺ�ʽ
	//Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53
	imu_data.Temp = 36.53f + (Temp_Buffer ) / 340.0f;
}

//��ȡIMU6����¶�����
void MPU6050_Read_All(void)
{
	MPU6050_Read_Accel();
	MPU6050_Read_Gyro();
	MPU6050_Read_Temp();
}



//��Ԫ������ŷ���ǣ���ʱûд�꣬����

//fp32 Kp = 100; //����������Ƽ��ٶȼ�/��ǿ�Ƶ������ٶ�
//fp32 Ki = 0.002; //���������������ƫ��������ٶ�
//fp32 Half_Time = 0.001; //�������ڵ�һ��


////�������������ڸ�����ܵ���Ԫ��(��ʼ����Ԫ����ֵ)  ��Ԫ��Quaternions
//int64_t quat[4] = {1,0,0,0};


////��Ki���ŵĻ��������(��ʼ��)
//fp32 eXInt = 0;
//fp32 eYInt = 0;
//fp32 eZInt = 0;

//Euler_temp Euler_buffer;

//void MPU6050_Calc_Euler(IMU_t * imu_data)
//{
//		 //����������
//    fp32 norm = sqrt(imu_data->Accel.X*imu_data->Accel.X+imu_data->Accel.Y*imu_data->Accel.Y+imu_data->Accel.Z*imu_data->Accel.Z);
//		
//    //��Ԫ��
//    imu_data->Accel.X = imu_data->Accel.X/norm;
//    imu_data->Accel.Y = imu_data->Accel.Y/norm;
//    imu_data->Accel.Z = imu_data->Accel.Z/norm;
//	
//	  //���Ʒ��������
//    Euler_buffer.Velocity.X = 2*(quat[1]*quat[3] - quat[0]*quat[2]);
//    Euler_buffer.Velocity.Y = 2*(quat[0]*quat[1] + quat[2]*quat[3]);
//    Euler_buffer.Velocity.Z = quat[0]*quat[0] - quat[1]*quat[1] - quat[2]*quat[2] + quat[3]*quat[3];
//	
//		//���������ͷ��򴫸��������ο�����֮��Ľ���˻����ܺ�
//    Euler_buffer.Errors.X = (imu_data->Accel.Y*Euler_buffer.Velocity.Z - imu_data->Accel.Z*Euler_buffer.Velocity.Y);
//    Euler_buffer.Errors.Y = (imu_data->Accel.Z*Euler_buffer.Velocity.X - imu_data->Accel.X*Euler_buffer.Velocity.Z);
//    Euler_buffer.Errors.Z = (imu_data->Accel.X*Euler_buffer.Velocity.Y - imu_data->Accel.Y*Euler_buffer.Velocity.X);
//	
//	  //������������������
//    eXInt += Euler_buffer.Errors.X*Ki;
//    eYInt += Euler_buffer.Errors.Y*Ki;
//    eZInt += Euler_buffer.Errors.Z*Ki;
//		
//		//������������ǲ���
//    imu_data->Gyro.X += Kp * Euler_buffer.Errors.X + eXInt;
//    imu_data->Gyro.Y += Kp * Euler_buffer.Errors.Y + eYInt;
//    imu_data->Gyro.Z += Kp * Euler_buffer.Errors.Z + eZInt;
//		
//		
//    //������Ԫ��
//    quat[0] += (-quat[1]*imu_data->Gyro.X - quat[2]*imu_data->Gyro.Y - quat[3]*imu_data->Gyro.Z)*Half_Time;
//    quat[1] += ( quat[0]*imu_data->Gyro.X + quat[2]*imu_data->Gyro.Z - quat[3]*imu_data->Gyro.Y)*Half_Time;
//    quat[2] += ( quat[0]*imu_data->Gyro.Y - quat[1]*imu_data->Gyro.Z + quat[3]*imu_data->Gyro.X)*Half_Time;
//    quat[3] += ( quat[0]*imu_data->Gyro.Z + quat[1]*imu_data->Gyro.Y - quat[2]*imu_data->Gyro.X)*Half_Time;
//		
//		
//    //��������Ԫ��
//    norm = sqrt(quat[0]*quat[0] + quat[1]*quat[1] + quat[2]*quat[2] + quat[3]*quat[3]);
//    quat[0] /= norm;
//    quat[1] /= norm;
//    quat[2] /= norm;
//    quat[3] /= norm;
//		
//		//��ȡŷ���� pitch��roll��yaw
//    imu_data->Euler.pitch = asin(-2*quat[1]*quat[3]+2*quat[0]*quat[2])*57.3;
//    imu_data->Euler.roll = atan2(2*quat[2]*quat[3]+2*quat[0]*quat[1],-2*quat[1]*quat[1]-2*quat[2]*quat[2]+1)*57.3;
//    imu_data->Euler.yaw = atan2(2*(quat[1]*quat[2] + quat[0]*quat[3]),quat[0]*quat[0]+quat[1]*quat[1]-quat[2]*quat[2]-quat[3]*quat[3])*57.3;

//}



