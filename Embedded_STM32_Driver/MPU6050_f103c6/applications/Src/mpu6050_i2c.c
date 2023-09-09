#include "mpu6050_i2c.h"
#include "i2c.h"
uint8_t I2C_Device_Addr;   //查找到后，写到MPU6050_ADDR宏里

IMU_t imu_data;

//IMU_Kalman_t imu_kalman_data;  //卡尔曼滤波待写


/**
 * @brief  I2C设备地址查询函数
 * @param  void
 * @retval void
* @attention 找到一次设备地址就可以注释掉，然后在MPU6050_ADDR宏里写上
 */
void I2C_Search_Addr(void)
{
  for(I2C_Device_Addr = 0;I2C_Device_Addr < 255;I2C_Device_Addr++)
  {
		//寻找设备地址，并打印地址，然后检查设备是否就绪
	  if(HAL_I2C_IsDeviceReady (&hi2c1 ,I2C_Device_Addr ,1 ,1) == HAL_OK )
	  {
			printf("\r\n设备正常\r\n");
		  printf("\r\nI2C设备的地址是:%d\r\n",I2C_Device_Addr);
		  break;
	  }
  }
}


/**
 * @brief  MPU6050初始化函数  对各个寄存器进行配置
 * @param  void
 * @retval void
* @attention 
 */
void MPU6050_Init(void)
{
	uint8_t check,Data;
	
	// 检查设备ID WHO_AM_I
	//默认值是0x68(104)，只用到bit6-bit1 即x110 100x，也就是 0110 1000
	HAL_I2C_Mem_Read (&hi2c1 ,MPU6050_ADDR,WHO_AM_I_REG,1,&check ,1,1000);
	
	if(check == 0x68)  //如果设备存在
	{
		//把电源管理1寄存器清0，则不会失能对应传感器，时钟源选择内部8MHz振荡器
		Data = 0x00; //0000 0000
		HAL_I2C_Mem_Write (&hi2c1 ,MPU6050_ADDR ,PWR_MGMT_1_REG ,1,&Data ,1,1000);
		
		//SMPLRT_DIV寄存器,设置数据采样率为1KHz;
		// 采样频率=陀螺仪输出频率/（1+SMPLRT_DIV）
		Data = 0x07;   //0000 0111
		HAL_I2C_Mem_Write (&hi2c1 ,MPU6050_ADDR ,SMPLRT_DIV_REG ,1 ,&Data,1,1000);
		
		// 设置ACCEL_CONFIG寄存器
		// 不开启X,Y,Z,轴自检，加速计的满量程范围选择±2g
		Data = 0x00;   //0000 0000
		HAL_I2C_Mem_Write (&hi2c1 ,MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);
		
		// 设置GYRO_CONFIG寄存器
		//  不开启X,Y,Z,轴自检，陀螺仪的满量程范围选择± 250deg/s
		Data = 0x00;  //0000 0000
		HAL_I2C_Mem_Write (&hi2c1 ,MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}
}




/**
 * @brief  MPU6050读取加速度函数
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
	//读取6字节的数据从ACCEL_XOUT_H寄存器中
	
	HAL_I2C_Mem_Read (&hi2c1 ,MPU6050_ADDR ,ACCEL_XOUT_H_REG ,1,Rec_Data ,6,1000);
	//把高8位和低8位合并为16位数据
	Accel_X_Buffer = (int16_t)(Rec_Data [0] <<8 | Rec_Data [1]);
	Accel_Y_Buffer = (int16_t)(Rec_Data [2] <<8 | Rec_Data [3]);
	Accel_Z_Buffer = (int16_t)(Rec_Data [4] <<8 | Rec_Data [5]);
	
	//因为加速计的满量程范围选择±2g,所以加速度计测量值的灵敏度最低分辨率要选16384LSB/g
	imu_data.Accel.X = Accel_X_Buffer/16384.0f;
	imu_data.Accel.Y = Accel_Y_Buffer/16384.0f;
	imu_data.Accel.Z = Accel_Z_Buffer/16384.0f;
}

/**
 * @brief  MPU6050读取角速度函数
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
	//读取6字节的数据从GYRO_XOUT_H寄存器中
	
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR ,GYRO_XOUT_H_REG ,1,Rec_Data ,6 ,1000);
	//把高8位和低8位合并为16位数据
	Gyro_X_Buffer = (int16_t )(Rec_Data [0] << 8 | Rec_Data [1]);
	Gyro_Y_Buffer = (int16_t )(Rec_Data [2] << 8 | Rec_Data [3]);
	Gyro_Z_Buffer = (int16_t )(Rec_Data [4] << 8 | Rec_Data [5]);
	//因为陀螺仪的满量程范围选择± 250deg/s,所以陀螺仪测量值的灵敏度最低分辨率要选131LSB/°/s
	imu_data.Gyro.X = Gyro_X_Buffer/131.0f;
	imu_data.Gyro.Y = Gyro_Y_Buffer/131.0f;
	imu_data.Gyro.Z = Gyro_Z_Buffer/131.0f;
}

/**
 * @brief  MPU6050读取温度函数
 * @param  void
 * @retval void
* @attention 
 */
void MPU6050_Read_Temp(void)
{
	uint8_t Rec_Data[2];
	int16_t Temp_Buffer = 0;
	//读取2字节的数据从TEMP_OUT_H_REG寄存器中
	HAL_I2C_Mem_Read(&hi2c1 ,MPU6050_ADDR ,TEMP_OUT_H_REG ,1 ,Rec_Data  ,2 ,1000);
	
	//把高8位和低8位合并为16位数据
	Temp_Buffer = (int16_t )(Rec_Data [0]<<8 | Rec_Data [1]);
	
	//说明书里的公式
	//Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53
	imu_data.Temp = 36.53f + (Temp_Buffer ) / 340.0f;
}

//读取IMU6轴和温度数据
void MPU6050_Read_All(void)
{
	MPU6050_Read_Accel();
	MPU6050_Read_Gyro();
	MPU6050_Read_Temp();
}



//四元数解析欧拉角，暂时没写完，待改

//fp32 Kp = 100; //比例增益控制加速度计/磁强计的收敛速度
//fp32 Ki = 0.002; //积分增益控制陀螺偏差的收敛速度
//fp32 Half_Time = 0.001; //采样周期的一半


////传感器框架相对于辅助框架的四元数(初始化四元数的值)  四元数Quaternions
//int64_t quat[4] = {1,0,0,0};


////由Ki缩放的积分误差项(初始化)
//fp32 eXInt = 0;
//fp32 eYInt = 0;
//fp32 eZInt = 0;

//Euler_temp Euler_buffer;

//void MPU6050_Calc_Euler(IMU_t * imu_data)
//{
//		 //测量正常化
//    fp32 norm = sqrt(imu_data->Accel.X*imu_data->Accel.X+imu_data->Accel.Y*imu_data->Accel.Y+imu_data->Accel.Z*imu_data->Accel.Z);
//		
//    //单元化
//    imu_data->Accel.X = imu_data->Accel.X/norm;
//    imu_data->Accel.Y = imu_data->Accel.Y/norm;
//    imu_data->Accel.Z = imu_data->Accel.Z/norm;
//	
//	  //估计方向的重力
//    Euler_buffer.Velocity.X = 2*(quat[1]*quat[3] - quat[0]*quat[2]);
//    Euler_buffer.Velocity.Y = 2*(quat[0]*quat[1] + quat[2]*quat[3]);
//    Euler_buffer.Velocity.Z = quat[0]*quat[0] - quat[1]*quat[1] - quat[2]*quat[2] + quat[3]*quat[3];
//	
//		//错误的领域和方向传感器测量参考方向之间的交叉乘积的总和
//    Euler_buffer.Errors.X = (imu_data->Accel.Y*Euler_buffer.Velocity.Z - imu_data->Accel.Z*Euler_buffer.Velocity.Y);
//    Euler_buffer.Errors.Y = (imu_data->Accel.Z*Euler_buffer.Velocity.X - imu_data->Accel.X*Euler_buffer.Velocity.Z);
//    Euler_buffer.Errors.Z = (imu_data->Accel.X*Euler_buffer.Velocity.Y - imu_data->Accel.Y*Euler_buffer.Velocity.X);
//	
//	  //积分误差比例积分增益
//    eXInt += Euler_buffer.Errors.X*Ki;
//    eYInt += Euler_buffer.Errors.Y*Ki;
//    eZInt += Euler_buffer.Errors.Z*Ki;
//		
//		//调整后的陀螺仪测量
//    imu_data->Gyro.X += Kp * Euler_buffer.Errors.X + eXInt;
//    imu_data->Gyro.Y += Kp * Euler_buffer.Errors.Y + eYInt;
//    imu_data->Gyro.Z += Kp * Euler_buffer.Errors.Z + eZInt;
//		
//		
//    //整合四元数
//    quat[0] += (-quat[1]*imu_data->Gyro.X - quat[2]*imu_data->Gyro.Y - quat[3]*imu_data->Gyro.Z)*Half_Time;
//    quat[1] += ( quat[0]*imu_data->Gyro.X + quat[2]*imu_data->Gyro.Z - quat[3]*imu_data->Gyro.Y)*Half_Time;
//    quat[2] += ( quat[0]*imu_data->Gyro.Y - quat[1]*imu_data->Gyro.Z + quat[3]*imu_data->Gyro.X)*Half_Time;
//    quat[3] += ( quat[0]*imu_data->Gyro.Z + quat[1]*imu_data->Gyro.Y - quat[2]*imu_data->Gyro.X)*Half_Time;
//		
//		
//    //正常化四元数
//    norm = sqrt(quat[0]*quat[0] + quat[1]*quat[1] + quat[2]*quat[2] + quat[3]*quat[3]);
//    quat[0] /= norm;
//    quat[1] /= norm;
//    quat[2] /= norm;
//    quat[3] /= norm;
//		
//		//获取欧拉角 pitch、roll、yaw
//    imu_data->Euler.pitch = asin(-2*quat[1]*quat[3]+2*quat[0]*quat[2])*57.3;
//    imu_data->Euler.roll = atan2(2*quat[2]*quat[3]+2*quat[0]*quat[1],-2*quat[1]*quat[1]-2*quat[2]*quat[2]+1)*57.3;
//    imu_data->Euler.yaw = atan2(2*(quat[1]*quat[2] + quat[0]*quat[3]),quat[0]*quat[0]+quat[1]*quat[1]-quat[2]*quat[2]-quat[3]*quat[3])*57.3;

//}



