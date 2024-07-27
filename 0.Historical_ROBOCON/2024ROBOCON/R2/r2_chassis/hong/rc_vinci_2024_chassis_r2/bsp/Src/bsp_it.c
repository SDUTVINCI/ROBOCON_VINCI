#include "include.h"
#include "L1S.h"
#include "imu605.h"
extern uint8_t imu_rx_buffer[121];
extern imu605_t imu605;

//int distance;
//uint8_t L1S_Y_rx_buffer[1];
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if (huart->Instance == USART2)
//	{
//		IMU_Read_Euler(imu_rx_buffer);
//	}
//	else if (huart->Instance == USART6)
//	{
////		L1S_Read_Distance(1, L1S_Y_rx_buffer[0], &distance);
//	}
//}


